#pragma once
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#ifdef DISPLAY_MODEL
#include "DisplayModule.h"
extern DisplayModule display;
#endif

extern QueueHandle_t loraUpdateQueue;
extern int pktCount;

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
