#pragma once

#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class LoRaModule;

struct LoRaTaskParams {
    LoRaModule* lora;
#ifdef DISPLAY_MODEL
    QueueHandle_t displayEventQueue;
#endif
};

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
