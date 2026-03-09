#pragma once

#include <freertos/FreeRTOS.h>
#include "PlaceNetConfig.h"
#include "SDCardModule.h"

class NetworkManager {
public:
    NetworkManager(PlaceNetConfig* config, SDCardModule* sd);

    // Connects WiFi, starts mDNS, performs PlaceNet handshake.
    // Returns true on success.
    bool setup();

private:
    bool connectWiFi();
    bool startMDNS();

    PlaceNetConfig* config_;
    SDCardModule*   sd_;

    char staSSID_[MAX_SSID_LENGTH]         = {};
    char staPassword_[MAX_PASSWORD_LENGTH] = {};
    char resolvedHostname_[32]             = "";
};
