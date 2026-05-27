#pragma once
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstring>
#include "LoRaModule.h"
#include "BLEModule.h"
#include "PlaceNetConfig.h"
#include "SDCardModule.h"

// Forward declaration — full definition pulled in via network_manager.h in .cpp files.
class MQTTManager;

#ifdef DISPLAY_MODEL
#include "DisplayModule.h"
extern DisplayModule display;
#endif

#ifdef HAS_GPS
#include "GPSModule.h"
#endif

extern int pktCount;

enum BeaconState {
    STATE_SETUP,
    STATE_PROVISIONING,
    STATE_TRANSITIONING,
    STATE_OPERATIONAL,
};

struct SupervisorContext {
    LoRaModule*     lora;
    BLEModule*      ble;
    PlaceNetConfig* config;
    SDCardModule*   sd;
    MQTTManager*    mqtt = nullptr;
#ifdef HAS_GPS
    GPSModule*      gps;
#endif
};

struct DisplayState {
    uint16_t batteryVoltage;
    int      sentCount;
    int      receivedCount;
    int16_t  lastRssi;
    float    lastSnr;
    char     lastUrl[128];
    uint8_t  lastKid[4];
    uint8_t  lastTok[4];
    BeaconState beaconState;

    bool operator!=(const DisplayState& other) const {
        return batteryVoltage  != other.batteryVoltage  ||
               sentCount       != other.sentCount       ||
               receivedCount   != other.receivedCount   ||
               lastRssi        != other.lastRssi        ||
               lastSnr         != other.lastSnr         ||
               beaconState     != other.beaconState     ||
               memcmp(lastKid, other.lastKid, 4) != 0  ||
               memcmp(lastTok, other.lastTok, 4) != 0  ||
               strcmp(lastUrl, other.lastUrl) != 0;
    }
};

/**
 * @brief Main supervisor task — drives state machine and spawns worker tasks
 *
 * States:
 *   STATE_SETUP        → no valid config, init BLE
 *   STATE_PROVISIONING → BLE advertising, waiting for credentials
 *   STATE_TRANSITIONING→ credentials received, save to SD, stop BLE
 *   STATE_OPERATIONAL  → BLE off, spawn LoRa/GPS/Network tasks, process packets
 *
 * @param pvParameters Pointer to SupervisorContext
 */
void mainTask(void* pvParameters);

bool setupMainTask(SupervisorContext* ctx, uint32_t stackDepth);
