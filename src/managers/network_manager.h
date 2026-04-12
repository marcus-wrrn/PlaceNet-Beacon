#pragma once

#include <freertos/FreeRTOS.h>
#include "PlaceNetConfig.h"
#include "SDCardModule.h"
#include "BLEModule.h"
#include "mqtt_manager.h"

class NetworkManager {
public:
    NetworkManager(PlaceNetConfig* config, SDCardModule* sd, BLEModule* ble = nullptr);

    // Stops BLE (if running), connects WiFi, starts mDNS, performs PlaceNet
    // handshake, and initiates the MQTT connection.
    // Returns true on success.
    bool setup();

    // Transfers ownership of the MQTTManager created during setup() to the
    // caller.  Returns nullptr if setup() did not reach the MQTT stage.
    MQTTManager* takeMqttManager();

private:
    bool connectWiFi();
    bool startMDNS();

    PlaceNetConfig* config_;
    SDCardModule*   sd_;
    BLEModule*      ble_;
    MQTTManager*    mqttManager_ = nullptr;

    char staSSID_[MAX_SSID_LENGTH]         = {};
    char staPassword_[MAX_PASSWORD_LENGTH] = {};
    char resolvedHostname_[32]             = "";
};
