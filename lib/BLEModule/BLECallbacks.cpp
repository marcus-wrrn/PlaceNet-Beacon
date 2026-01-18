#include "BLECallbacks.h"
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
