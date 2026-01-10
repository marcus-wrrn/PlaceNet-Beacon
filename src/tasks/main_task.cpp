#include "main_task.h"
#include "lora_task.h"
#include "LoRaModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "MAIN";

void mainTask(void* pvParameters) {
    LOGI(TAG, "Main task starting...");

    if (loraUpdateQueue == nullptr) {
        LOGE(TAG, "ERROR: loraUpdateQueue is NULL!");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "Waiting for LoRa packets...");

    LoRaPacket pkt;
    uint32_t loopCount = 0;

    while (true) {
        loopCount++;

        if (loopCount % 100 == 0) {
            LOGI(TAG, "Loop iteration %lu, queue items waiting: %d",
                 loopCount, uxQueueMessagesWaiting(loraUpdateQueue));
        }

        if (xQueueReceive(loraUpdateQueue, &pkt, portMAX_DELAY)) {
            pktCount++;
            LOGI(TAG, "#%d Received packet with RSSI: %d dBm\nSNR: %.2f dB\nLength: %d",
                 pktCount, pkt.rssi, pkt.snr, pkt.length);

#ifdef DISPLAY_MODEL
            char buffer[sizeof(pkt)];
            snprintf(buffer, sizeof(buffer),
            "#%d Received packet with RSSI: %ddBm\nSNR: %.2f dB\nLength: %d",
            pktCount, pkt.rssi, pkt.snr, pkt.length);
            display.clearBuffer();
            display.drawStrF(0, 8, "#%d, RSSI: %d, SNR: %.2f", pktCount, pkt.rssi, pkt.snr);
            display.drawStrF(0, 16, "%.*s", pkt.length, (const char*)pkt.data);
            display.sendBuffer();
#endif
        }
    }
}
