#pragma once
#include "config.h"
#include "GPSModule.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class LoRaModule;

#define PACKET_LEN 256

struct LoRaTaskParams {
    LoRaModule* lora;
};

extern QueueHandle_t loraUpdateQueue;

/**
 * @brief LoRa task function - handles TX/RX radio operations
 *
 * This task:
 * - Waits for packets on TX queue and transmits them
 * - Listens for incoming packets and places them on RX queue
 * - Manages radio state (TX/RX switching)
 *
 * @param pvParameters Pointer to LoRaTaskParams struct
 */
void loraTask(void* pvParameters);
bool setupLoRaTask(LoRaModule* lora, uint32_t stackDepth);
