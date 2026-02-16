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

bool setupMainTask(uint32_t stackDepth) {
    return setupMainTask(nullptr, stackDepth);
}

void mainTask(void* pvParameters) {
    LOGI(TAG, "Main task starting...");

    MainTaskParams* params = static_cast<MainTaskParams*>(pvParameters);

    BLEModule* ble = params ? params->ble : nullptr;
    if (ble && ble->isEnabled()) {
        LOGI(TAG, "BLE module available for requests");
    } else {
        LOGI(TAG, "BLE module disabled or unavailable");
    }

    if (loraUpdateQueue == nullptr) {
        LOGE(TAG, "ERROR: loraUpdateQueue is NULL!");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "Waiting for LoRa packets...");

    LoRaPacket pkt;
    DisplayState currentState = {};
    DisplayState lastDisplayedState = {};
    TickType_t lastDisplayUpdate = 0;
    const TickType_t displayUpdateInterval = pdMS_TO_TICKS(5000);

#ifdef HAS_PMU
    PMUManager pmuManager;
#endif

#ifdef HAS_GPS
    GPSManager gpsManager(locationTaskUpdateQueue);
#endif

    while (true) {
#ifdef HAS_PMU
        if (pmuManager.updatePMU()) {
            currentState.batteryVoltage = pmuManager.getState().battery_voltage;
        }
#endif

#ifdef HAS_GPS
        if (gpsManager.updateGPS()) {
            const GPSData& gpsData = gpsManager.getData();
            currentState.latitude = gpsData.position.latitude;
            currentState.longitude = gpsData.position.longitude;
            currentState.satelliteCount = gpsData.metadata.satelliteCount;
            currentState.altitude = gpsData.metadata.altitude;
        }
#endif

        while (xQueueReceive(loraUpdateQueue, &pkt, 0) == pdPASS) {
            pktCount++;
            bool isSentPacket = (pkt.rssi == 0 && pkt.snr == 0.0f);

            currentState.packetCount = pktCount;
            currentState.lastRssi = pkt.rssi;
            currentState.lastSnr = pkt.snr;
            currentState.lastPacketWasSent = isSentPacket;
            memcpy(currentState.receivedData, pkt.data, pkt.length);
            currentState.receivedData[pkt.length] = '\0';

            if (isSentPacket) {
                LOGI(TAG, "Packet #%d sent\n%.*s", pktCount, pkt.length, (const char*)pkt.data);
            } else {
                LOGI(TAG, "#%d Received packet with RSSI: %d dBm\nSNR: %.2f dB\nLength: %d",
                     pktCount, pkt.rssi, pkt.snr, pkt.length);
            }

            if (ble && ble->isEnabled() && ble->isConnected()) {
                ble->notifyBeaconData(pkt.data, pkt.length);
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (now - lastDisplayUpdate >= displayUpdateInterval) {
            if (currentState != lastDisplayedState) {
#ifdef DISPLAY_MODEL
                display.clearBuffer();

                if (currentState.packetCount > 0) {
                    if (currentState.lastPacketWasSent) {
                        display.drawLine("Packet #%d sent", currentState.packetCount);
                    } else {
                        display.drawLine("#%d, RSSI: %d, SNR: %.2f",
                            currentState.packetCount, currentState.lastRssi, currentState.lastSnr);
                        display.drawLine((char*)currentState.receivedData);
                    }
                }

#ifdef HAS_PMU
                pmuManager.logPMU(&display);
#endif

#ifdef HAS_GPS
                gpsManager.logGPS(&display);
#endif

                display.sendBuffer();
#endif
                lastDisplayedState = currentState;
            }
            lastDisplayUpdate = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
