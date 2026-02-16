#pragma once
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstring>
#include "LoRaModule.h"

#ifdef DISPLAY_MODEL
#include "DisplayModule.h"
extern DisplayModule display;
#endif

#ifdef HAS_BLE
class BLEModule;
#endif

extern int pktCount;

struct MainTaskParams {
    BLEModule* ble;
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
               strcmp((const char*)receivedData, (const char*)other.receivedData) != 0;
    }
};

/**
 * @brief Main task function - handles LoRa packet processing and display updates
 *
 * This task:
 * - Waits for LoRa packets from loraUpdateQueue
 * - Logs packet information (RSSI, SNR, length)
 * - Updates display with packet data
 *
 * @param pvParameters Unused (nullptr)
 */
void mainTask(void* pvParameters);

bool setupMainTask(BLEModule* ble, uint32_t stackDepth);
bool setupMainTask(uint32_t stackDepth);