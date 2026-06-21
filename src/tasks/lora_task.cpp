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

    loraRxQueue = xQueueCreate(LORA_RX_QUEUE_LEN, sizeof(LoRaRxMsg));
    if (!loraRxQueue) {
        LOGE(TAG, "Failed to create loraRxQueue");
        return false;
    }

    loraTxQueue = xQueueCreate(LORA_TX_QUEUE_LEN, sizeof(LoRaTxMsg));
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
        9,
        nullptr,
        1
    );

    if (result == pdPASS) {
        LOGI(TAG, "LoRa task created on core 1 (priority 9)");
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

    LOGI(TAG, "LoRa task running - interrupt-driven RX, TX driven by loraTxQueue");

    LoRaTxMsg txMsg;
    LoRaPacket raw;
    LoRaRxMsg  rxMsg;
    LoRaRxMsg  echo;
    uint32_t   rxCount = 0;
    uint32_t   txCount = 0;

    while (true) {
        // ── TX: drain one outgoing message if queued ──────────────────────
        if (xQueueReceive(loraTxQueue, &txMsg, 0) == pdPASS) {
            txCount++;
            bool ok = false;

            if (txMsg.type == LoRaTxType::MESH_PACKET) {
                LOGI(TAG, "TX #%lu: MeshPacket payloadType=%d", txCount, txMsg.packet.payloadType);
                ok = lora->transmitMeshCorePacket(txMsg.packet);
            } else {
                LOGI(TAG, "TX #%lu: Advert name='%s'", txCount, txMsg.advert.name);
                ok = lora->transmitMeshCoreAdvert(txMsg.advert);
            }

            if (ok) {
                LOGI(TAG, "TX #%lu: successful", txCount);

                // Echo onto loraRxQueue so main_task can update sent counters.
                // isEcho=true / rssi=0 / snr=0 distinguishes this from a real RX.
                memset(&echo, 0, sizeof(echo));
                echo.isEcho = true;

                if (txMsg.type == LoRaTxType::MESH_PACKET) {
                    echo.packet = txMsg.packet;
                } else {
                    // Reconstruct the ADVERT MeshPacket that was put on air.
                    echo.packet.payloadType = meshcore::PAYLOAD_TYPE_ADVERT;
                    size_t advLen = txMsg.advert.serialize(
                        echo.packet.payload, sizeof(echo.packet.payload));
                    echo.packet.payloadLen = static_cast<uint8_t>(advLen);
                }

                if (xQueueSend(loraRxQueue, &echo, 0) != pdPASS) {
                    LOGW(TAG, "TX #%lu: loraRxQueue full, echo dropped", txCount);
                }
            } else {
                LOGW(TAG, "TX #%lu: failed", txCount);
            }

            if (!lora->startListening()) {
                LOGE(TAG, "Failed to re-enter listening mode after TX");
            }
        }

        // ── RX: non-blocking read driven by DIO1 interrupt flag ───────────
        if (lora->readPacket(&raw)) {
            memset(&rxMsg, 0, sizeof(rxMsg));
            rxMsg.rssi   = raw.rssi;
            rxMsg.snr    = raw.snr;
            rxMsg.isEcho = false;

            if (lora->parsePacket(raw, rxMsg.packet)) {
                rxCount++;
                LOGI(TAG, "RX #%lu: %d bytes payloadType=%d RSSI=%d dBm SNR=%.2f dB",
                     rxCount, raw.length, rxMsg.packet.payloadType, rxMsg.rssi, rxMsg.snr);

                if (xQueueSend(loraRxQueue, &rxMsg, portMAX_DELAY) != pdPASS) {
                    LOGE(TAG, "RX #%lu: failed to enqueue", rxCount);
                }
            } else {
                LOGW(TAG, "RX: %d bytes received but MeshPacket parse failed — discarding", raw.length);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
