#pragma once
#include <NimBLEDevice.h>

class BLEModule;

class BeaconServerCallbacks : public NimBLEServerCallbacks {
public:
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override;
};

class WiFiCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
public:
    void setBLEModule(BLEModule* module) { bleModule_ = module; }

    void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;

private:
    BLEModule* bleModule_ = nullptr;
};
