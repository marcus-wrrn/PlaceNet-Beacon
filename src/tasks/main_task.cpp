#include "main_task.h"
#include "lora_task.h"
#include "LoRaModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef HAS_PMU
#include "pmu_task.h"
#include "PMUModule.h"
#endif

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

#ifdef HAS_PMU
    PMUState pmuState;
    uint16_t batteryVoltage = 0;
    bool hasBatteryData = false;
#endif

    while (true) {
        loopCount++;

        if (loopCount % 100 == 0) {
            LOGI(TAG, "Loop iteration %lu, queue items waiting: %d",
                 loopCount, uxQueueMessagesWaiting(loraUpdateQueue));
        }

#ifdef HAS_PMU
        // Check for PMU state updates (non-blocking)
        if (pmuStateQueue && xQueueReceive(pmuStateQueue, &pmuState, 0) == pdPASS) {
            batteryVoltage = pmuState.battery_voltage;
            hasBatteryData = true;
            LOGI(TAG, "Battery voltage updated: %u mV", batteryVoltage);
        }
#endif

        if (xQueueReceive(loraUpdateQueue, &pkt, pdMS_TO_TICKS(100))) {
            pktCount++;
            // TODO: Replace with an actual broadcast/receive test
            bool isSentPacket = (pkt.rssi == 0 && pkt.snr == 0.0f);

            if (isSentPacket) {
                LOGI(TAG, "Packet #%d sent\n%.*s", pktCount, pkt.length, (const char*)pkt.data);
            } else {
                LOGI(TAG, "#%d Received packet with RSSI: %d dBm\nSNR: %.2f dB\nLength: %d",
                     pktCount, pkt.rssi, pkt.snr, pkt.length);
            }

#ifdef DISPLAY_MODEL
            display.clearBuffer();
            if (isSentPacket) {
                display.drawStrF(0, 8, "Packet #%d sent", pktCount);
                display.drawStrF(0, 16, "%.*s", pkt.length, (const char*)pkt.data);
            } else {
                display.drawStrF(0, 8, "#%d, RSSI: %d, SNR: %.2f", pktCount, pkt.rssi, pkt.snr);
                display.drawStrF(0, 16, "%.*s", pkt.length, (const char*)pkt.data);
            }
#ifdef HAS_PMU
            if (hasBatteryData) {
                display.drawStrF(0, 24, "Battery: %u mV", batteryVoltage);
            }
#endif
            display.sendBuffer();
#endif
        }
    }
}
