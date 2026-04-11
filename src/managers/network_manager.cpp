#include "network_manager.h"
#include "http_manager.h"
#include "logger.h"
#include <WiFi.h>
#include <ESPmDNS.h>

static const char* TAG = "NET-MANAGER";

// ── Target server (change for testing) ────────────────────────────────────────
static const char*    HOME_SERVER_IP   = "192.168.2.39";
static const uint16_t HOME_SERVER_PORT = 8080;
// ──────────────────────────────────────────────────────────────────────────────

static const char*    MDNS_BASE       = "beacon";
static const uint16_t ADVERTISE_PORT  = 8883;

// ── WiFi event handler ────────────────────────────────────────────────────────

static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            LOGI(TAG, "Connected to WiFi");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            LOGI(TAG, "Got IP: %s", WiFi.localIP().toString().c_str());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            LOGW(TAG, "Disconnected from WiFi, reconnecting...");
            WiFi.reconnect();
            break;
        default:
            break;
    }
}

// ── NetworkManager ────────────────────────────────────────────────────────────

NetworkManager::NetworkManager(PlaceNetConfig* config, SDCardModule* sd, BLEModule* ble)
    : config_(config), sd_(sd), ble_(ble) {}

bool NetworkManager::connectWiFi() {
    if (strlen(staSSID_) == 0) {
        LOGW(TAG, "No WiFi SSID configured, skipping connection");
        return false;
    }

    WiFi.onEvent(onWiFiEvent);
    WiFi.mode(WIFI_STA);
    WiFi.begin(staSSID_, staPassword_);

    LOGI(TAG, "Connecting to WiFi SSID='%s'...", staSSID_);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (WiFi.status() != WL_CONNECTED) {
        LOGE(TAG, "Failed to connect to WiFi");
        return false;
    }

    LOGI(TAG, "WiFi connected: IP=%s", WiFi.localIP().toString().c_str());
    return true;
}

bool NetworkManager::startMDNS() {
    if (MDNS.begin(MDNS_BASE)) {
        snprintf(resolvedHostname_, sizeof(resolvedHostname_), "%s", MDNS_BASE);
        LOGI(TAG, "mDNS started: %s.local", resolvedHostname_);
        return true;
    }

    for (int i = 1; i <= 10; i++) {
        char candidate[32];
        snprintf(candidate, sizeof(candidate), "%s-%d", MDNS_BASE, i);
        if (MDNS.begin(candidate)) {
            snprintf(resolvedHostname_, sizeof(resolvedHostname_), "%s", candidate);
            LOGI(TAG, "mDNS started: %s.local", resolvedHostname_);
            return true;
        }
    }

    LOGW(TAG, "mDNS failed to start");
    return false;
}

bool NetworkManager::setup() {
    // Stop BLE before starting WiFi — they share the radio.
    if (ble_) {
        LOGI(TAG, "Stopping BLE before WiFi init");
        ble_->stop();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Extract the first enabled WiFi entry from config.
    bool hasCredentials = false;
    if (config_) {
        for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
            if (config_->wifi[i].enabled && strlen(config_->wifi[i].ssid) > 0) {
                strncpy(staSSID_, config_->wifi[i].ssid, sizeof(staSSID_) - 1);
                staSSID_[sizeof(staSSID_) - 1] = '\0';
                strncpy(staPassword_, config_->wifi[i].password, sizeof(staPassword_) - 1);
                staPassword_[sizeof(staPassword_) - 1] = '\0';
                hasCredentials = true;
                break;
            }
        }
    }

    if (!hasCredentials) {
        LOGW(TAG, "No enabled WiFi credentials in config, skipping network setup");
        return false;
    }

    if (!connectWiFi()) {
        return false;
    }

    startMDNS();

    HTTPManager http(HOME_SERVER_IP, HOME_SERVER_PORT);

    if (!http.checkHealth()) {
        LOGW(TAG, "Home server health check failed before handshake");
        // Continue anyway — server may be slow to start.
    }

    char deviceAddress[64];
    snprintf(deviceAddress, sizeof(deviceAddress), "%s:%u",
             WiFi.localIP().toString().c_str(), ADVERTISE_PORT);

    String brokerResponse;
    if (!http.performHandshake(deviceAddress, resolvedHostname_, ADVERTISE_PORT, brokerResponse)) {
        LOGE(TAG, "PlaceNet registration handshake failed");
        return false;
    }

#ifdef HAS_SDCARD
    if (sd_ && sd_->isInitialized()) {
        MQTTBrokerInfo brokerInfo;
        if (http.parseMQTTBrokerResponse(brokerResponse, &brokerInfo)) {
            if (!sd_->saveMQTTBroker(&brokerInfo)) {
                LOGW(TAG, "Failed to save MQTT broker info to SD");
            }
        } else {
            LOGW(TAG, "Could not parse MQTT broker info from handshake response");
        }
    }
#endif

    if (!http.checkHealth()) {
        LOGW(TAG, "Home server health check failed after handshake");
    }

    return true;
}
