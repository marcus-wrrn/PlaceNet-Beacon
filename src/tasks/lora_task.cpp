#include "lora_task.h"
#include "LoRaModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "LORA_TASK";

void loraTask(void* pvParameters) {
    LoRaModule* lora = static_cast<LoRaModule*>(pvParameters);

    if (!lora) {
        LOGE(TAG, "LoRa module pointer is null, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    // Initialize LoRa radio
    if (!lora->init()) {
        LOGE(TAG, "LoRa initialization failed, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "LoRa task started");

    LoRaPacket txPacket;
    QueueHandle_t txQueue = lora->getTxQueue();

    while (1) {
        // TODO: Implement LoRa TX/RX logic

        // Example TX logic:
        // if (xQueueReceive(txQueue, &txPacket, pdMS_TO_TICKS(100)) == pdTRUE) {
        //     LOGI(TAG, "Transmitting packet (%d bytes)", txPacket.length);
        //     // radio->transmit(txPacket.data, txPacket.length);
        // }

        // Example RX logic:
        // if (radio->available()) {
        //     LoRaPacket rxPacket;
        //     rxPacket.length = radio->receive(rxPacket.data, LORA_MAX_PACKET_SIZE);
        //     rxPacket.rssi = radio->getRSSI();
        //     rxPacket.snr = radio->getSNR();
        //     xQueueSend(lora->getRxQueue(), &rxPacket, 0);
        // }

        LOGW(TAG, "LoRa task running (STUB - not implemented)");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
