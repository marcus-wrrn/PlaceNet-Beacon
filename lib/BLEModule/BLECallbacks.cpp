#include "BLECallbacks.h"
#include "BLEModule.h"
#include "logger.h"

static const char* TAG = "BLE";

void BeaconServerCallbacks::onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    LOGI(TAG, "Client connected: %s", connInfo.getAddress().toString().c_str());
    pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
}

void BeaconServerCallbacks::onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    LOGI(TAG, "Client disconnected (reason: %d), restarting advertising", reason);
    NimBLEDevice::startAdvertising();
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
        } else {
            LOGW(TAG, "Password received but no SSID set yet");
        }
    }
}
