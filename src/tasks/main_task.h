#pragma once
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#ifdef DISPLAY_MODEL
#include "DisplayModule.h"
extern DisplayModule display;
#endif

#ifdef HAS_BLE
class BLEModule;
#endif
 
extern int pktCount;

struct MainTaskParams {
#ifdef HAS_BLE
    BLEModule* ble;
#endif
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

#ifdef HAS_BLE
bool setupMainTask(BLEModule* ble, uint32_t stackDepth);
#else
bool setupMainTask(uint32_t stackDepth);
#endif
