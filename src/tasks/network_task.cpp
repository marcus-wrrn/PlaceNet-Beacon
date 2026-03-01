#include "network_task.h"
#include "config.h"
#include "logger.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

static const char* TAG = "NETWORK_TASK";

// ── Target server (change for testing) ────────────────────────────────────────
static const char* HOME_SERVER_IP   = "192.168.2.39";
static const uint16_t HOME_SERVER_PORT = 8080;
// ──────────────────────────────────────────────────────────────────────────────

static char staSSID[MAX_SSID_LENGTH]         = {};
static char staPassword[MAX_PASSWORD_LENGTH] = {};
static const char* mdnsBase = "beacon";
static char resolvedHostname[32] = "";
static const uint16_t ADVERTISE_PORT = 8883;

// ── WiFi ──────────────────────────────────────────────────────────────────────

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

static bool connectWiFi() {
    if (strlen(staSSID) == 0) {
        LOGW(TAG, "No WiFi SSID configured, skipping connection");
        return false;
    }

    WiFi.onEvent(onWiFiEvent);
    WiFi.mode(WIFI_STA);
    WiFi.begin(staSSID, staPassword);

    LOGI(TAG, "Connecting to WiFi SSID='%s'...", staSSID);

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

// ── mDNS ─────────────────────────────────────────────────────────────────────

static bool startMDNS() {
    if (MDNS.begin(mdnsBase)) {
        snprintf(resolvedHostname, sizeof(resolvedHostname), "%s", mdnsBase);
        LOGI(TAG, "mDNS started: %s.local", resolvedHostname);
        return true;
    }

    for (int i = 1; i <= 10; i++) {
        char candidate[32];
        snprintf(candidate, sizeof(candidate), "%s-%d", mdnsBase, i);
        if (MDNS.begin(candidate)) {
            snprintf(resolvedHostname, sizeof(resolvedHostname), "%s", candidate);
            LOGI(TAG, "mDNS started: %s.local", resolvedHostname);
            return true;
        }
    }

    LOGW(TAG, "mDNS failed to start");
    return false;
}

// ── Registration handshake ────────────────────────────────────────────────────

// Build the base URL for the home server.
static String homeServerBase() {
    String url = "http://";
    url += HOME_SERVER_IP;
    url += ":";
    url += HOME_SERVER_PORT;
    return url;
}

// POST / with X-PlaceNet-Init header and device registration JSON.
// Returns true if the server responds 200 "Device verified".
static bool performHandshake() {
    String localIP = WiFi.localIP().toString();

    char deviceAddress[64];
    snprintf(deviceAddress, sizeof(deviceAddress), "%s:%u",
             localIP.c_str(), ADVERTISE_PORT);

    // Build JSON payload per PlaceNet Initialization Protocol v0.0.1.
    JsonDocument doc;
    doc["address"]        = deviceAddress;
    JsonObject mdns       = doc["mdns"].to<JsonObject>();
    mdns["hostname"]      = resolvedHostname;
    mdns["port"]          = ADVERTISE_PORT;

    String payload;
    serializeJson(doc, payload);

    String url = homeServerBase() + "/";

    LOGI(TAG, "Sending PlaceNet registration to %s", url.c_str());

    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-PlaceNet-Init", "0.0.1");
    http.setTimeout(10000);

    int httpCode = http.POST(payload);

    if (httpCode < 0) {
        LOGE(TAG, "POST / failed: %s", http.errorToString(httpCode).c_str());
        http.end();
        return false;
    }

    String body = http.getString();
    LOGI(TAG, "POST / -> HTTP %d: %s", httpCode, body.c_str());
    http.end();

    if (httpCode == 200) {
        LOGI(TAG, "Device verified by home server");
        return true;
    }

    LOGE(TAG, "Handshake rejected (HTTP %d)", httpCode);
    return false;
}

// GET /health — verify the home server is reachable and responding.
static bool checkHealth() {
    String url = homeServerBase() + "/health";

    LOGI(TAG, "Checking server health at %s", url.c_str());

    HTTPClient http;
    http.begin(url);
    http.setTimeout(5000);

    int httpCode = http.GET();

    if (httpCode < 0) {
        LOGE(TAG, "GET /health failed: %s", http.errorToString(httpCode).c_str());
        http.end();
        return false;
    }

    String body = http.getString();
    LOGI(TAG, "GET /health -> HTTP %d: %s", httpCode, body.c_str());
    http.end();

    if (httpCode == 200) {
        LOGI(TAG, "Home server is healthy");
        return true;
    }

    LOGW(TAG, "Unexpected health response: HTTP %d", httpCode);
    return false;
}

// ── Public API ────────────────────────────────────────────────────────────────

bool setupNetworkTask(PlaceNetConfig* config, uint32_t stackDepth) {
    // Find the first enabled WiFi entry.
    bool hasCredentials = false;
    if (config) {
        for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
            if (config->wifi[i].enabled && strlen(config->wifi[i].ssid) > 0) {
                strncpy(staSSID, config->wifi[i].ssid, sizeof(staSSID) - 1);
                staSSID[sizeof(staSSID) - 1] = '\0';
                strncpy(staPassword, config->wifi[i].password, sizeof(staPassword) - 1);
                staPassword[sizeof(staPassword) - 1] = '\0';
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

    if (!checkHealth()) {
        LOGW(TAG, "Home server health check failed before handshake");
        // Continue anyway — server may be slow to start.
    }

    if (!performHandshake()) {
        LOGE(TAG, "PlaceNet registration handshake failed");
        return false;
    }

    // Re-verify health after handshake.
    if (!checkHealth()) {
        LOGW(TAG, "Home server health check failed after handshake");
    }

    return true;
}
