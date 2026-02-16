#include "network_task.h"
#include "config.h"
#include "logger.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <esp_mac.h>
#include <freertos/task.h>

static const char* TAG = "NETWORK_TASK";

static WebServer server(80);
static const char* apSSID = "PlaceNet-Beacon";
static const char* mdnsBase = "beacon";
static char resolvedHostname[32] = "";

static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_AP_START:
            LOGI(TAG, "WiFi AP started");
            break;
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            LOGI(TAG, "Station connected to AP");
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            LOGI(TAG, "Station disconnected from AP");
            break;
        default:
            break;
    }
}

static bool startWiFiAP() {
    WiFi.onEvent(onWiFiEvent);
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(apSSID)) {
        LOGE(TAG, "Failed to start WiFi AP");
        return false;
    }
    LOGI(TAG, "WiFi AP started: SSID='%s' IP=%s", apSSID, WiFi.softAPIP().toString().c_str());
    return true;
}

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

    LOGW(TAG, "mDNS failed, AP still reachable via IP");
    return false;
}

static void handleRoot() {
    LOGI(TAG, "Received response!");
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);

    String ip = WiFi.softAPIP().toString();
    String hostname = strlen(resolvedHostname) > 0
        ? String(resolvedHostname) + ".local"
        : "N/A";

    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    String html = "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>PlaceNet Beacon</title>"
        "<style>"
        "body{font-family:sans-serif;margin:2em;background:#1a1a2e;color:#e0e0e0;}"
        "h1{color:#00d4ff;}"
        "table{border-collapse:collapse;margin-top:1em;}"
        "td{padding:0.4em 1em;border-bottom:1px solid #333;}"
        "td:first-child{font-weight:bold;color:#aaa;}"
        "</style></head><body>"
        "<h1>PlaceNet Beacon</h1>"
        "<table>"
        "<tr><td>Device</td><td>" + String(BOARD_VARIANT_NAME) + "</td></tr>"
        "<tr><td>MAC</td><td>" + String(macStr) + "</td></tr>"
        "<tr><td>IP</td><td>" + ip + "</td></tr>"
        "<tr><td>WiFi Mode</td><td>AP</td></tr>"
        "<tr><td>Hostname</td><td>" + hostname + "</td></tr>"
        "</table></body></html>";

    server.send(200, "text/html", html);
}

bool setupNetworkTask(uint32_t stackDepth) {
    if (!startWiFiAP()) {
        return false;
    }

    startMDNS();

    server.on("/", handleRoot);
    server.begin();
    LOGI(TAG, "HTTP server started on port 80");

    BaseType_t result = xTaskCreatePinnedToCore(
        networkTask,
        "Network",
        stackDepth,
        nullptr,
        6,
        nullptr,
        1
    );

    if (result == pdPASS) {
        LOGI(TAG, "Network task created on core 1 (priority 6)");
        return true;
    }

    LOGE(TAG, "Failed to create network task");
    return false;
}

void networkTask(void* pvParameters) {
    LOGI(TAG, "Network task started");
    uint32_t lastDebugPrint = 0;
    while (1) {
        server.handleClient();

        uint32_t now = millis();
        if (now - lastDebugPrint > 30000) {
            LOGI(TAG, "Network task alive, AP clients: %d", WiFi.softAPgetStationNum());
            lastDebugPrint = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
