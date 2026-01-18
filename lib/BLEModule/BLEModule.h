#pragma once

#include "config.h"
#include "BLECallbacks.h"
#include <NimBLEDevice.h>

#define BLE_DEVICE_NAME "PlaceNet-Beacon"

#define SERVICE_UUID_CONFIG     "180A"  // Device Information Service
#define CHAR_UUID_DEVICE_NAME   "2A00"  // Device Name
#define CHAR_UUID_FIRMWARE_REV  "2A26"  // Firmware Revision

#define SERVICE_UUID_BEACON     "DEAD"  // Custom beacon service
#define CHAR_UUID_BEACON_DATA   "BEEF"  // Beacon data characteristic (notify)
#define CHAR_UUID_BEACON_CONFIG "F00D"  // Beacon configuration (read/write)

class BLEModule {
public:
    BLEModule();
    ~BLEModule();

    bool init();
    bool isInitialized() const { return initialized_; }

    void startAdvertising();
    void stopAdvertising();
    bool isAdvertising() const;

    uint16_t getConnectedCount() const;
    bool isConnected() const { return getConnectedCount() > 0; }

    bool notifyBeaconData(const uint8_t* data, size_t length);
    bool setBeaconConfig(const char* config);
    std::string getBeaconConfig() const;

private:
    bool initialized_;
    NimBLEServer* server_;
    NimBLEService* deviceInfoService_;
    NimBLEService* beaconService_;
    NimBLECharacteristic* beaconDataChar_;
    NimBLECharacteristic* beaconConfigChar_;

    BeaconServerCallbacks serverCallbacks_;
    BeaconCharacteristicCallbacks charCallbacks_;
    BeaconDescriptorCallbacks descCallbacks_;

    bool createDeviceInfoService();
    bool createBeaconService();
};
