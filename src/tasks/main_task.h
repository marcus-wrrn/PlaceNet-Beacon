#pragma once
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstring>
#include "LoRaModule.h"
#include "BLEModule.h"
#include "PlaceNetConfig.h"
#include "SDCardModule.h"

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
#ifdef HAS_GPS
    GPSModule*      gps;
#endif
};

struct DisplayState {
    uint16_t batteryVoltage;
    double latitude;
    double longitude;
    uint8_t satelliteCount;
    float altitude;
    int packetCount;
    int16_t lastRssi;
    float lastSnr;
    bool lastPacketWasSent;
    uint8_t receivedData[LORA_MAX_PACKET_SIZE + 1];
    BeaconState beaconState;

    bool operator!=(const DisplayState& other) const {
        return batteryVoltage != other.batteryVoltage ||
               latitude != other.latitude ||
               longitude != other.longitude ||
               satelliteCount != other.satelliteCount ||
               altitude != other.altitude ||
               packetCount != other.packetCount ||
               lastRssi != other.lastRssi ||
               lastSnr != other.lastSnr ||
               lastPacketWasSent != other.lastPacketWasSent ||
               beaconState != other.beaconState ||
               strcmp((const char*)receivedData, (const char*)other.receivedData) != 0;
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
