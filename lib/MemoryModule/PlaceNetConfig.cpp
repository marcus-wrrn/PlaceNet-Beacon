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
    lora = LoRaConfig();
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

    // LoRa PHY parameters — warn (don't fail) on out-of-range values so a
    // questionable BLE write still persists and can be corrected.
    if (lora.frequency < 100.0f || lora.frequency > 1000.0f) {
        LOGW(TAG, "LoRa frequency looks out of range: %.3f MHz", lora.frequency);
    }
    if (lora.spreadingFactor < 6 || lora.spreadingFactor > 12) {
        LOGW(TAG, "LoRa spreading factor out of range: %u", lora.spreadingFactor);
    }
    if (lora.codingRate < 5 || lora.codingRate > 8) {
        LOGW(TAG, "LoRa coding rate out of range: %u", lora.codingRate);
    }

    return true;
}

bool PlaceNetConfig::isValidLoRaConfig() const {
    // Frequency: ISM/LoRa range covering both 433/868/915 MHz plans.
    if (lora.frequency < 100.0f || lora.frequency > 1000.0f) {
        LOGE(TAG, "Invalid LoRa frequency: %.3f MHz", lora.frequency);
        return false;
    }
    // Bandwidth: SX126x supports up to 500 kHz; reject zero/negative/oversize.
    if (lora.bandwidth <= 0.0f || lora.bandwidth > 500.0f) {
        LOGE(TAG, "Invalid LoRa bandwidth: %.1f kHz", lora.bandwidth);
        return false;
    }
    // Spreading factor: 6-12 (SX126x).
    if (lora.spreadingFactor < 6 || lora.spreadingFactor > 12) {
        LOGE(TAG, "Invalid LoRa spreading factor: %u", lora.spreadingFactor);
        return false;
    }
    // Coding rate: 4/5 - 4/8 -> denominator 5-8.
    if (lora.codingRate < 5 || lora.codingRate > 8) {
        LOGE(TAG, "Invalid LoRa coding rate: 4/%u", lora.codingRate);
        return false;
    }
    // Sync word 0x00 is reserved/invalid for a configured radio.
    if (lora.syncWord == 0x00) {
        LOGE(TAG, "Invalid LoRa sync word: 0x%02X", lora.syncWord);
        return false;
    }
    return true;
}

bool PlaceNetConfig::isReadyForOperation() const {
    // Must have at least one fully valid WiFi network.
    bool hasValidWiFi = false;
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        if (wifi[i].enabled) {
            if (!isValidSSID(wifi[i].ssid)) {
                LOGE(TAG, "Not ready: invalid SSID for WiFi network %d", i);
                return false;
            }
            if (!isValidPassword(wifi[i].password)) {
                LOGE(TAG, "Not ready: invalid password for WiFi network %d", i);
                return false;
            }
            hasValidWiFi = true;
        }
    }
    if (!hasValidWiFi) {
        LOGE(TAG, "Not ready: no valid WiFi network configured");
        return false;
    }

    // Radio PHY profile must be fully valid — the device cannot operate on
    // the mesh with a bad profile.
    if (!isValidLoRaConfig()) {
        LOGE(TAG, "Not ready: invalid LoRa radio configuration");
        return false;
    }

    // Beacon interval must be sane (and non-zero) to avoid a runaway TX loop.
    if (beacon.beaconIntervalMs < 1000) {
        LOGE(TAG, "Not ready: beacon interval too short: %lu ms", beacon.beaconIntervalMs);
        return false;
    }

    // Optional services: only validate when enabled.
    if (mqtt.enabled) {
        if (!isValidMQTTBroker(mqtt.broker)) {
            LOGE(TAG, "Not ready: invalid MQTT broker address");
            return false;
        }
        if (mqtt.port == 0) {
            LOGE(TAG, "Not ready: invalid MQTT port: %u", mqtt.port);
            return false;
        }
    }

    if (httpServer.enabled) {
        if (strlen(httpServer.url) == 0) {
            LOGE(TAG, "Not ready: invalid HTTP server URL");
            return false;
        }
        if (httpServer.port == 0) {
            LOGE(TAG, "Not ready: invalid HTTP server port: %u", httpServer.port);
            return false;
        }
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

    LOGI(TAG, "LoRa Radio Configuration:");
    LOGI(TAG, "  Frequency: %.3f MHz", lora.frequency);
    LOGI(TAG, "  Bandwidth: %.1f kHz", lora.bandwidth);
    LOGI(TAG, "  Spreading Factor: %u", lora.spreadingFactor);
    LOGI(TAG, "  Coding Rate: 4/%u", lora.codingRate);
    LOGI(TAG, "  Sync Word: 0x%02X", lora.syncWord);

    LOGI(TAG, "==============================");
}
