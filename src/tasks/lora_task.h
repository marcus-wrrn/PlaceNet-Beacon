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

extern QueueHandle_t loraRxQueue;   // lora_task  → main_task  (received packets + TX echoes)
extern QueueHandle_t loraTxQueue;   // mqtt_manager → lora_task (packets to transmit)

/**
 * @brief LoRa task function - handles TX/RX radio operations
 *
 * This task:
 * - Always stays in continuous RX mode (listening)
 * - When a packet arrives on loraTxQueue, switches to TX, transmits,
 *   then immediately returns to RX mode
 * - Pushes received packets (and TX echoes) onto loraRxQueue for main_task
 *
 * @param pvParameters Pointer to LoRaTaskParams struct
 */
void loraTask(void* pvParameters);
bool setupLoRaTask(LoRaModule* lora, uint32_t stackDepth);
