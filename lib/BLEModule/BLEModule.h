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

// Radio + server provisioning service. Each characteristic carries a small JSON
// document so the whole profile is written atomically and can be read back.
#define SERVICE_UUID_CONFIG     "12345678-1234-1234-1234-123456789ABD"  // Radio/server config service
#define CHAR_UUID_LORA_CONFIG   "12345678-1234-1234-1234-123456789AB4"  // LoRa PHY profile (read/write JSON)
#define CHAR_UUID_SERVER_CONFIG "12345678-1234-1234-1234-123456789AB5"  // Home server addr+port (read/write JSON)

#define WIFI_CRED_MAX_LEN 64
#define BLE_SERVER_ADDR_MAX_LEN MAX_HTTP_SERVER_LENGTH

struct BLEWiFiCredentials {
    char ssid[32];
    char password[WIFI_CRED_MAX_LEN];
    bool pending;
};

// LoRa PHY profile written over BLE as JSON:
//   {"frequency":915.1,"bandwidth":125.0,"spreadingFactor":8,"codingRate":5,"syncWord":18}
struct BLELoRaConfig {
    float   frequency;        // MHz
    float   bandwidth;        // kHz
    uint8_t spreadingFactor;  // 6-12
    uint8_t codingRate;       // 5-8 (4/5 - 4/8)
    uint8_t syncWord;         // e.g. 0x12 = private network
    bool    pending;
};

// Home server target written over BLE as JSON:
//   {"address":"192.168.2.39","port":8080}
struct BLEServerConfig {
    char     address[BLE_SERVER_ADDR_MAX_LEN];
    uint16_t port;
    bool     pending;
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

    // ── Radio / server provisioning ───────────────────────────────────────────
    void setLoRaConfigCallback(std::function<void(const BLELoRaConfig&)> cb);
    void setServerConfigCallback(std::function<void(const BLEServerConfig&)> cb);

    // Seed the readable value of the config characteristics with the values
    // currently in effect, so the app shows the real config on connect.
    // Call after init().
    void setCurrentLoRaConfig(float frequency, float bandwidth, uint8_t spreadingFactor,
                              uint8_t codingRate, uint8_t syncWord);
    void setCurrentServerConfig(const char* address, uint16_t port);

private:
    bool initialized_;
    bool enabled_;
    NimBLEServer* server_;
    NimBLEService* wifiService_;
    NimBLECharacteristic* wifiSsidChar_;
    NimBLECharacteristic* wifiPasswordChar_;
    NimBLECharacteristic* wifiStatusChar_;

    NimBLEService* configService_;
    NimBLECharacteristic* loraConfigChar_;
    NimBLECharacteristic* serverConfigChar_;

    BLEWiFiCredentials wifiCreds_;
    BLELoRaConfig   loraConfig_;
    BLEServerConfig serverConfig_;
    WiFiCredentialsCallback wifiCredsCallback_;
    std::function<void(const char* ssid, const char* pass)> credentialsCallback_;
    std::function<void(const BLELoRaConfig&)> loraConfigCallback_;
    std::function<void(const BLEServerConfig&)> serverConfigCallback_;

    BeaconServerCallbacks serverCallbacks_;
    WiFiCharacteristicCallbacks wifiCharCallbacks_;

    bool createWiFiService();
    bool createConfigService();

    friend class WiFiCharacteristicCallbacks;
};
