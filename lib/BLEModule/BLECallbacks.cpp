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

void BeaconServerCallbacks::onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) {
    LOGI(TAG, "MTU updated: %u for connection: %u", MTU, connInfo.getConnHandle());
}

uint32_t BeaconServerCallbacks::onPassKeyDisplay() {
    uint32_t passkey = 123456;
    LOGI(TAG, "Passkey display: %lu", passkey);
    return passkey;
}

void BeaconServerCallbacks::onConfirmPassKey(NimBLEConnInfo& connInfo, uint32_t pass_key) {
    LOGI(TAG, "Passkey confirmation requested: %lu", pass_key);
    NimBLEDevice::injectConfirmPasskey(connInfo, true);
}

void BeaconServerCallbacks::onAuthenticationComplete(NimBLEConnInfo& connInfo) {
    if (!connInfo.isEncrypted()) {
        NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
        LOGW(TAG, "Encryption failed, disconnecting client");
        return;
    }
    LOGI(TAG, "Secured connection to: %s", connInfo.getAddress().toString().c_str());
}

void BeaconCharacteristicCallbacks::onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
    LOGD(TAG, "%s: onRead, value: %s",
         pCharacteristic->getUUID().toString().c_str(),
         pCharacteristic->getValue().c_str());
}

void BeaconCharacteristicCallbacks::onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
    LOGI(TAG, "%s: onWrite, value: %s",
         pCharacteristic->getUUID().toString().c_str(),
         pCharacteristic->getValue().c_str());
}

void BeaconCharacteristicCallbacks::onStatus(NimBLECharacteristic* pCharacteristic, int code) {
    LOGD(TAG, "Notification/Indication status: %d (%s)", code, NimBLEUtils::returnCodeToString(code));
}

void BeaconCharacteristicCallbacks::onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) {
    const char* subType;
    switch (subValue) {
        case 0: subType = "Unsubscribed"; break;
        case 1: subType = "Notifications"; break;
        case 2: subType = "Indications"; break;
        case 3: subType = "Notifications+Indications"; break;
        default: subType = "Unknown"; break;
    }
    LOGI(TAG, "Client %s: %s for %s",
         connInfo.getAddress().toString().c_str(),
         subType,
         pCharacteristic->getUUID().toString().c_str());
}

void BeaconDescriptorCallbacks::onWrite(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) {
    LOGD(TAG, "Descriptor %s written: %s",
         pDescriptor->getUUID().toString().c_str(),
         pDescriptor->getValue().c_str());
}

void BeaconDescriptorCallbacks::onRead(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) {
    LOGD(TAG, "Descriptor %s read", pDescriptor->getUUID().toString().c_str());
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
