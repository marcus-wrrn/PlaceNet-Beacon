#include "location_task.h"

#ifdef HAS_GPS

#include "GPSModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "LOCATION_TASK";

QueueHandle_t locationUpdateQueue = nullptr;

#define GPS_READ_INTERVAL_MS 5000
#define GPS_READ_TIMEOUT_MS 5000

void locationTask(void* pvParameters) {
    LocationTaskParams* params = static_cast<LocationTaskParams*>(pvParameters);

    if (!params || !params->gps) {
        LOGE(TAG, "Location task parameters invalid, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    GPSModule* gps = params->gps;
    LOGI(TAG, "Location task started");

    if (!gps->isInitialized()) {
        LOGE(TAG, "GPS not initialized, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    GPSData data;
    uint32_t readCount = 0;
    uint32_t successCount = 0;

    LOGI(TAG, "Reading GPS data every %d ms...", GPS_READ_INTERVAL_MS);

    while (1) {
        readCount++;

        LOGI(TAG, "Reading GPS data (attempt #%lu)...", readCount);

        if (gps->readData(&data, GPS_READ_TIMEOUT_MS)) {
            successCount++;

            if (gps->isFixValid(&data)) {
                LOGI(TAG, "GPS fix valid (success #%lu)", successCount);
                gps->logData(&data);

                if (locationUpdateQueue != nullptr) {
                    BaseType_t result = xQueueSend(locationUpdateQueue, &data, portMAX_DELAY);
                    if (result == pdPASS) {
                        LOGI(TAG, "Location queued successfully to locationUpdateQueue");
                    } else {
                        LOGE(TAG, "Failed to queue location to locationUpdateQueue");
                    }
                } else {
                    LOGE(TAG, "locationUpdateQueue is null! Cannot send location to main task");
                }
            } else {
                LOGW(TAG, "GPS fix invalid (no satellite lock)");
            }
        } else {
            LOGW(TAG, "Failed to read GPS data (timeout or parse error)");
        }

        LOGV(TAG, "Delaying for %d ms before next read...", GPS_READ_INTERVAL_MS);
        vTaskDelay(pdMS_TO_TICKS(GPS_READ_INTERVAL_MS));
        LOGV(TAG, "Delay complete, looping back...");
    }
}

#endif // HAS_GPS
