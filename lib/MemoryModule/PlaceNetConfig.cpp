#include "PlaceNetConfig.h"
#include "logger.h"

#ifdef HAS_SDCARD
#include "SDCardModule.h"
#endif

static const char* TAG = "CONFIG";

PlaceNetConfig::PlaceNetConfig() {
    reset();
}

void PlaceNetConfig::reset() {
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        wifi[i] = WiFiCredentials();
    }
    mqtt = MQTTConfig();
    httpServer = HTTPServerConfig();
    beacon = BeaconConfig();
}

#ifdef HAS_SDCARD
bool PlaceNetConfig::resetHard(SDCardModule* sd) {
    bool success = true;

    if (!sd || !sd->isInitialized()) {
        LOGE(TAG, "resetHard: SD card not available");
        success = false;
    } else {
        if (sd->configExists() && !sd->deleteFile(CONFIG_FILE_PATH)) {
            LOGE(TAG, "resetHard: failed to delete config file");
            success = false;
        }
        if (sd->mqttBrokerExists() && !sd->deleteFile(MQTT_BROKER_FILE_PATH)) {
            LOGE(TAG, "resetHard: failed to delete MQTT broker file");
            success = false;
        }
    }

    reset();
    LOGI(TAG, "resetHard: config reset%s", success ? " and SD files erased" : " (SD erase incomplete)");
    return success;
}
#endif

bool PlaceNetConfig::isValidSSID(const char* ssid) const {
    if (!ssid || strlen(ssid) == 0) {
        return false;
    }
    size_t len = strlen(ssid);
    return len > 0 && len <= 32;
}

bool PlaceNetConfig::isValidPassword(const char* password) const {
    if (!password) {
        return false;
    }
    size_t len = strlen(password);
    return len == 0 || (len >= 8 && len <= 63);
}

bool PlaceNetConfig::isValidMQTTBroker(const char* broker) const {
    if (!broker || strlen(broker) == 0) {
        return false;
    }
    return strlen(broker) <= MAX_MQTT_BROKER_LENGTH;
}

bool PlaceNetConfig::validate() const {
    bool hasValidWiFi = false;
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        if (wifi[i].enabled) {
            if (!isValidSSID(wifi[i].ssid)) {
                LOGE(TAG, "Invalid SSID for WiFi network %d", i);
                return false;
            }
            if (!isValidPassword(wifi[i].password)) {
                LOGE(TAG, "Invalid password for WiFi network %d", i);
                return false;
            }
            hasValidWiFi = true;
        }
    }

    if (!hasValidWiFi) {
        LOGW(TAG, "No valid WiFi networks configured");
    }

    if (mqtt.enabled) {
        if (!isValidMQTTBroker(mqtt.broker)) {
            LOGE(TAG, "Invalid MQTT broker address");
            return false;
        }
        if (mqtt.port == 0 || mqtt.port > 65535) {
            LOGE(TAG, "Invalid MQTT port: %d", mqtt.port);
            return false;
        }
    }

    if (httpServer.enabled) {
        if (strlen(httpServer.url) == 0) {
            LOGE(TAG, "Invalid HTTP server URL");
            return false;
        }
        if (httpServer.port == 0 || httpServer.port > 65535) {
            LOGE(TAG, "Invalid HTTP server port: %d", httpServer.port);
            return false;
        }
    }

    if (beacon.beaconIntervalMs < 1000) {
        LOGW(TAG, "Beacon interval is very short: %lu ms", beacon.beaconIntervalMs);
    }

    return true;
}

bool PlaceNetConfig::isSetUp() const {
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        if (wifi[i].enabled && isValidSSID(wifi[i].ssid)) {
            return true;
        }
    }
    return false;
}

void PlaceNetConfig::print() const {
    LOGI(TAG, "=== PlaceNet Configuration ===");

    LOGI(TAG, "WiFi Networks:");
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        if (wifi[i].enabled) {
            LOGI(TAG, "  [%d] SSID: %s, Password: %s",
                 i, wifi[i].ssid, wifi[i].password[0] ? "***" : "(none)");
        }
    }

    if (mqtt.enabled) {
        LOGI(TAG, "MQTT Configuration:");
        LOGI(TAG, "  Broker: %s:%d", mqtt.broker, mqtt.port);
        LOGI(TAG, "  Client ID: %s", mqtt.clientId);
        LOGI(TAG, "  Base Topic: %s", mqtt.baseTopic);
        LOGI(TAG, "  TLS: %s", mqtt.useTLS ? "enabled" : "disabled");
        LOGI(TAG, "  Auth: %s", mqtt.username[0] ? "enabled" : "disabled");
    } else {
        LOGI(TAG, "MQTT: disabled");
    }

    if (httpServer.enabled) {
        LOGI(TAG, "HTTP Server Configuration:");
        LOGI(TAG, "  URL: %s:%d", httpServer.url, httpServer.port);
        LOGI(TAG, "  TLS: %s", httpServer.useTLS ? "enabled" : "disabled");
    } else {
        LOGI(TAG, "HTTP Server: disabled");
    }

    LOGI(TAG, "Beacon Configuration:");
    LOGI(TAG, "  Interval: %lu ms", beacon.beaconIntervalMs);
    LOGI(TAG, "  LoRa: %s", beacon.loraEnabled ? "enabled" : "disabled");
    LOGI(TAG, "  GPS: %s", beacon.gpsEnabled ? "enabled" : "disabled");
    LOGI(TAG, "  BLE: %s", beacon.bleEnabled ? "enabled" : "disabled");

    LOGI(TAG, "==============================");
}
