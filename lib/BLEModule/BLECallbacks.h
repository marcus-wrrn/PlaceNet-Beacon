#pragma once
#include <NimBLEDevice.h>

class BeaconServerCallbacks : public NimBLEServerCallbacks {
public:
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override;
    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override;
    uint32_t onPassKeyDisplay() override;
    void onConfirmPassKey(NimBLEConnInfo& connInfo, uint32_t pass_key) override;
    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override;
};

class BeaconCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
    void onStatus(NimBLECharacteristic* pCharacteristic, int code) override;
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override;
};

class BeaconDescriptorCallbacks : public NimBLEDescriptorCallbacks {
public:
    void onWrite(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) override;
    void onRead(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) override;
};
