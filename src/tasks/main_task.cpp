#include "main_task.h"
#include "lora_task.h"
#include "LoRaModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef HAS_PMU
#include "pmu_task.h"
#include "PMUModule.h"
#include "managers/pmu_manager.h"
#endif

#ifdef HAS_GPS
#include "location_task.h"
#include "GPSModule.h"
#include "managers/gps_manager.h"
#endif
#include "BLEModule.h"

static const char* TAG = "MAIN";
static MainTaskParams mainParams = {};

bool setupMainTask(BLEModule* ble, uint32_t stackDepth) {
    mainParams.ble = ble;
    BaseType_t result = xTaskCreatePinnedToCore(
        mainTask,
        "MainTask",
        stackDepth,
        &mainParams,
        10,
        nullptr,
        0
    );

    if (result == pdPASS) {
        LOGI(TAG, "Main task created on core 0 (priority 10)");
        return true;
    } else {
        LOGE(TAG, "Failed to create main task");
        return false;
    }
}

void mainTask(void* pvParameters) {
    LOGI(TAG, "Main task starting...");

    MainTaskParams* params = static_cast<MainTaskParams*>(pvParameters);

#ifdef HAS_BLE
    BLEModule* ble = params ? params->ble : nullptr;
    if (ble) {
        LOGI(TAG, "BLE module available for requests");
    }
#endif

    if (loraUpdateQueue == nullptr) {
        LOGE(TAG, "ERROR: loraUpdateQueue is NULL!");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "Waiting for LoRa packets...");

    LoRaPacket pkt;
    uint32_t loopCount = 0;

#ifdef HAS_PMU
    PMUManager pmuManager;
#endif

#ifdef HAS_GPS
    GPSManager gpsManager(locationTaskUpdateQueue);
#endif

    while (true) {
        loopCount++;

        if (loopCount % 100 == 0) {
            LOGI(TAG, "Loop iteration %lu, queue items waiting: %d",
                 loopCount, uxQueueMessagesWaiting(loraUpdateQueue));
        }

#ifdef HAS_PMU
        pmuManager.updatePMU();
#endif

#ifdef HAS_GPS
        gpsManager.updateGPS();
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

            if (ble && ble->isConnected()) {
                ble->notifyBeaconData(pkt.data, pkt.length);
            }

#ifdef DISPLAY_MODEL
            display.clearBuffer();
            if (isSentPacket) {
                display.drawLine("Packet #%d sent", pktCount);
                display.drawLine("%.*s", pkt.length, (const char*)pkt.data);
            } else {
                display.drawLine("#%d, RSSI: %d, SNR: %.2f", pktCount, pkt.rssi, pkt.snr);
                display.drawLine("%.*s", pkt.length, (const char*)pkt.data);
            }
#ifdef HAS_PMU
            pmuManager.logPMU(&display);
#endif
#ifdef HAS_GPS
            gpsManager.logGPS(&display);
#endif
            display.sendBuffer();
#endif
        }
    }
}
