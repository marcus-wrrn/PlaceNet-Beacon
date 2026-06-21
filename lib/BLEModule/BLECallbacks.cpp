#include "BLECallbacks.h"
#include "BLEModule.h"
#include "logger.h"
#include <ArduinoJson.h>

static const char* TAG = "BLE";

void BeaconServerCallbacks::onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    LOGI(TAG, "Client connected: %s", connInfo.getAddress().toString().c_str());
    pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
}

void BeaconServerCallbacks::onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    // NimBLE 2.x: advertising restart is handled by server_->advertiseOnDisconnect(true) in BLEModule::init().
    LOGI(TAG, "Client disconnected (reason: %d)", reason);
}

void WiFiCharacteristicCallbacks::onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
    LOGD(TAG, "WiFi %s: onRead", pCharacteristic->getUUID().toString().c_str());
}

void WiFiCharacteristicCallbacks::onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
    if (!bleModule_) return;

    std::string uuid = pCharacteristic->getUUID().toString();
    std::string value = pCharacteristic->getValue();

    if (pCharacteristic == bleModule_->wifiSsidChar_) {
        if (value.length() >= sizeof(bleModule_->wifiCreds_.ssid)) {
            LOGW(TAG, "SSID too long, truncating");
            value = value.substr(0, sizeof(bleModule_->wifiCreds_.ssid) - 1);
        }
        strncpy(bleModule_->wifiCreds_.ssid, value.c_str(), sizeof(bleModule_->wifiCreds_.ssid) - 1);
        bleModule_->wifiCreds_.ssid[sizeof(bleModule_->wifiCreds_.ssid) - 1] = '\0';
        LOGI(TAG, "WiFi SSID set: %s", bleModule_->wifiCreds_.ssid);
    } else if (pCharacteristic == bleModule_->wifiPasswordChar_) {
        if (value.length() >= sizeof(bleModule_->wifiCreds_.password)) {
            LOGW(TAG, "Password too long, truncating");
            value = value.substr(0, sizeof(bleModule_->wifiCreds_.password) - 1);
        }
        strncpy(bleModule_->wifiCreds_.password, value.c_str(), sizeof(bleModule_->wifiCreds_.password) - 1);
        bleModule_->wifiCreds_.password[sizeof(bleModule_->wifiCreds_.password) - 1] = '\0';
        LOGI(TAG, "WiFi password set");

        if (strlen(bleModule_->wifiCreds_.ssid) > 0) {
            bleModule_->wifiCreds_.pending = true;
            LOGI(TAG, "WiFi credentials ready for SSID: %s", bleModule_->wifiCreds_.ssid);

            if (bleModule_->wifiCredsCallback_) {
                bleModule_->wifiCredsCallback_(&bleModule_->wifiCreds_);
            }
            if (bleModule_->credentialsCallback_) {
                bleModule_->credentialsCallback_(bleModule_->wifiCreds_.ssid, bleModule_->wifiCreds_.password);
            }
        } else {
            LOGW(TAG, "Password received but no SSID set yet");
        }
    } else if (pCharacteristic == bleModule_->loraConfigChar_) {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, value);
        if (err) {
            LOGW(TAG, "LoRa config JSON parse error: %s", err.c_str());
            return;
        }

        BLELoRaConfig& c = bleModule_->loraConfig_;
        c.frequency       = doc["frequency"]       | c.frequency;
        c.bandwidth       = doc["bandwidth"]       | c.bandwidth;
        c.spreadingFactor = doc["spreadingFactor"] | c.spreadingFactor;
        c.codingRate      = doc["codingRate"]      | c.codingRate;
        c.syncWord        = doc["syncWord"]        | c.syncWord;
        c.pending         = true;

        // Reflect the accepted value back so a subsequent read returns it.
        pCharacteristic->setValue(value);

        LOGI(TAG, "LoRa config set: %.3f MHz, BW %.1f kHz, SF %u, CR 4/%u, sync 0x%02X",
             c.frequency, c.bandwidth, c.spreadingFactor, c.codingRate, c.syncWord);

        if (bleModule_->loraConfigCallback_) {
            bleModule_->loraConfigCallback_(c);
        }
    } else if (pCharacteristic == bleModule_->serverConfigChar_) {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, value);
        if (err) {
            LOGW(TAG, "Server config JSON parse error: %s", err.c_str());
            return;
        }

        BLEServerConfig& c = bleModule_->serverConfig_;
        const char* addr = doc["address"];
        if (addr) {
            strncpy(c.address, addr, sizeof(c.address) - 1);
            c.address[sizeof(c.address) - 1] = '\0';
        }
        c.port    = doc["port"] | c.port;
        c.pending = true;

        pCharacteristic->setValue(value);

        LOGI(TAG, "Server config set: %s:%u", c.address, c.port);

        if (bleModule_->serverConfigCallback_) {
            bleModule_->serverConfigCallback_(c);
        }
    }
}
