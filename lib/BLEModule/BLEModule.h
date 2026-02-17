#pragma once

#include "config.h"
#include "BLECallbacks.h"
#include <NimBLEDevice.h>
#include <functional>

#define BLE_DEVICE_NAME "PlaceNet-Beacon"

#define SERVICE_UUID_WIFI       "12345678-1234-1234-1234-123456789ABC"  // WiFi provisioning service
#define CHAR_UUID_WIFI_SSID     "12345678-1234-1234-1234-123456789AB1"  // WiFi SSID (read/write)
#define CHAR_UUID_WIFI_PASSWORD "12345678-1234-1234-1234-123456789AB2"  // WiFi password (write only)
#define CHAR_UUID_WIFI_STATUS   "12345678-1234-1234-1234-123456789AB3"  // WiFi connection status (read/notify)

#define WIFI_CRED_MAX_LEN 64

struct BLEWiFiCredentials {
    char ssid[32];
    char password[WIFI_CRED_MAX_LEN];
    bool pending;
};

typedef void (*WiFiCredentialsCallback)(const BLEWiFiCredentials* creds);

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
    void stop();

    uint16_t getConnectedCount() const;
    bool isConnected() const { return getConnectedCount() > 0; }

    void setWiFiCredentialsCallback(WiFiCredentialsCallback callback);
    void setCredentialsCallback(std::function<void(const char* ssid, const char* pass)> cb);
    bool hasPendingWiFiCredentials() const;
    BLEWiFiCredentials getWiFiCredentials();
    void setWiFiStatus(const char* status);

private:
    bool initialized_;
    bool enabled_;
    NimBLEServer* server_;
    NimBLEService* wifiService_;
    NimBLECharacteristic* wifiSsidChar_;
    NimBLECharacteristic* wifiPasswordChar_;
    NimBLECharacteristic* wifiStatusChar_;

    BLEWiFiCredentials wifiCreds_;
    WiFiCredentialsCallback wifiCredsCallback_;
    std::function<void(const char* ssid, const char* pass)> credentialsCallback_;

    BeaconServerCallbacks serverCallbacks_;
    WiFiCharacteristicCallbacks wifiCharCallbacks_;

    bool createWiFiService();

    friend class WiFiCharacteristicCallbacks;
};
