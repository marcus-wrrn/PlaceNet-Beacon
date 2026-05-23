#include "lora_task.h"
#include "LoRaModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

static const char* TAG = "LORA_TASK";

QueueHandle_t loraRxQueue = nullptr;
QueueHandle_t loraTxQueue = nullptr;

static LoRaTaskParams loraParams;

bool setupLoRaTask(LoRaModule* lora, uint32_t stackDepth) {
    if (!lora) {
        LOGE(TAG, "LoRa module pointer is null");
        return false;
    }

    loraRxQueue = xQueueCreate(LORA_RX_QUEUE_LEN, sizeof(LoRaPacket));
    if (!loraRxQueue) {
        LOGE(TAG, "Failed to create loraRxQueue");
        return false;
    }

    loraTxQueue = xQueueCreate(LORA_TX_QUEUE_LEN, sizeof(LoRaPacket));
    if (!loraTxQueue) {
        LOGE(TAG, "Failed to create loraTxQueue");
        vQueueDelete(loraRxQueue);
        loraRxQueue = nullptr;
        return false;
    }

    loraParams.lora = lora;

    BaseType_t result = xTaskCreatePinnedToCore(
        loraTask,
        "LoRa",
        stackDepth,
        &loraParams,
        8,
        nullptr,
        1
    );

    if (result == pdPASS) {
        LOGI(TAG, "LoRa task created on core 1 (priority 8)");
        return true;
    } else {
        LOGE(TAG, "Failed to create LoRa task");
        vQueueDelete(loraRxQueue);
        vQueueDelete(loraTxQueue);
        loraRxQueue = nullptr;
        loraTxQueue = nullptr;
        return false;
    }
}

void loraTask(void* pvParameters) {
    LoRaTaskParams* params = static_cast<LoRaTaskParams*>(pvParameters);

    if (!params || !params->lora) {
        LOGE(TAG, "LoRa task parameters invalid, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LoRaModule* lora = params->lora;
    LOGI(TAG, "LoRa task starting...");

    if (!lora->init()) {
        LOGE(TAG, "LoRa initialization failed, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    if (!lora->startListening()) {
        LOGE(TAG, "Failed to start listening mode, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "LoRa task running - listening for packets, TX driven by loraTxQueue");

    LoRaPacket pkt;
    uint32_t rxCount = 0;
    uint32_t txCount = 0;

    while (true) {
        // --- TX: check for outbound packet (non-blocking) ---
        if (xQueueReceive(loraTxQueue, &pkt, 0) == pdPASS) {
            txCount++;
            LOGI(TAG, "TX #%lu: transmitting %d bytes", txCount, pkt.length);

            bool ok = lora->transmit(pkt.data, pkt.length);
            if (ok) {
                LOGI(TAG, "TX #%lu: transmitted successfully", txCount);

                // Echo to loraRxQueue so main_task can update the display.
                // Convention: rssi=0, snr=0 indicates a locally-sent packet.
                LoRaPacket echo = pkt;
                echo.rssi = 0;
                echo.snr  = 0.0f;
                if (xQueueSend(loraRxQueue, &echo, 0) != pdPASS) {
                    LOGW(TAG, "TX #%lu: loraRxQueue full, TX echo dropped", txCount);
                }
            } else {
                LOGW(TAG, "TX #%lu: transmission failed", txCount);
            }

            // Return to RX mode after every transmission
            if (!lora->startListening()) {
                LOGE(TAG, "Failed to re-enter listening mode after TX");
            }
        }

        // --- RX: poll for incoming packet (50 ms timeout keeps TX queue responsive) ---
        if (lora->receive(&pkt, 50)) {
            rxCount++;
            LOGI(TAG, "=== RX Packet #%lu ===", rxCount);
            LOGI(TAG, "Length: %d bytes", pkt.length);
            LOGI(TAG, "RSSI:   %d dBm",   pkt.rssi);
            LOGI(TAG, "SNR:    %.2f dB",  pkt.snr);
            LOGI(TAG, "Data:   %.*s",     pkt.length, (const char*)pkt.data);
            LOGI(TAG, "======================");

            if (xQueueSend(loraRxQueue, &pkt, portMAX_DELAY) != pdPASS) {
                LOGE(TAG, "RX #%lu: failed to queue packet to loraRxQueue", rxCount);
            }
        }
    }
}
