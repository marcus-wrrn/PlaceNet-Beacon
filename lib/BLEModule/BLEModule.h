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

#define SERVICE_UUID_WIFI       "12345678-1234-1234-1234-123456789ABC"  // WiFi provisioning service
#define CHAR_UUID_WIFI_SSID     "12345678-1234-1234-1234-123456789AB1"  // WiFi SSID (read/write)
#define CHAR_UUID_WIFI_PASSWORD "12345678-1234-1234-1234-123456789AB2"  // WiFi password (write only)
#define CHAR_UUID_WIFI_STATUS   "12345678-1234-1234-1234-123456789AB3"  // WiFi connection status (read/notify)

#define WIFI_CRED_MAX_LEN 64

struct WiFiCredentials {
    char ssid[32];
    char password[WIFI_CRED_MAX_LEN];
    bool pending;
};

typedef void (*WiFiCredentialsCallback)(const WiFiCredentials* creds);

class BLEModule {
public:
    BLEModule();
    BLEModule(bool isenabled);
    ~BLEModule();

    bool init();
    bool isInitialized() const { return initialized_; }
    bool isEnabled() const { return enabled_; }
    void setEnabled(bool enabled) { enabled_ = enabled; }

    void startAdvertising();
    void stopAdvertising();
    bool isAdvertising() const;

    uint16_t getConnectedCount() const;
    bool isConnected() const { return getConnectedCount() > 0; }

    bool notifyBeaconData(const uint8_t* data, size_t length);
    bool setBeaconConfig(const char* config);
    std::string getBeaconConfig() const;

    void setWiFiCredentialsCallback(WiFiCredentialsCallback callback);
    bool hasPendingWiFiCredentials() const;
    WiFiCredentials getWiFiCredentials();
    void setWiFiStatus(const char* status);

private:
    bool initialized_;
    bool enabled_;
    NimBLEServer* server_;
    NimBLEService* deviceInfoService_;
    NimBLEService* beaconService_;
    NimBLEService* wifiService_;
    NimBLECharacteristic* beaconDataChar_;
    NimBLECharacteristic* beaconConfigChar_;
    NimBLECharacteristic* wifiSsidChar_;
    NimBLECharacteristic* wifiPasswordChar_;
    NimBLECharacteristic* wifiStatusChar_;

    WiFiCredentials wifiCreds_;
    WiFiCredentialsCallback wifiCredsCallback_;

    BeaconServerCallbacks serverCallbacks_;
    BeaconCharacteristicCallbacks charCallbacks_;
    BeaconDescriptorCallbacks descCallbacks_;
    WiFiCharacteristicCallbacks wifiCharCallbacks_;

    bool createDeviceInfoService();
    bool createBeaconService();
    bool createWiFiService();

    friend class WiFiCharacteristicCallbacks;
};
