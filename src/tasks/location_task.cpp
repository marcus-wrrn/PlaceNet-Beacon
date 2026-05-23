#include "location_task.h"

#ifdef HAS_GPS

#include "GPSModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "LOCATION_TASK";

QueueHandle_t locationTaskUpdateQueue = nullptr;
static LocationTaskParams locationParams;

bool setupLocationTask(GPSModule* gps, uint32_t stackDepth) {
    if (!gps) {
        LOGE(TAG, "GPS module pointer is null");
        return false;
    }

    locationTaskUpdateQueue = xQueueCreate(5, sizeof(GPSData));
    if (!locationTaskUpdateQueue) {
        LOGE(TAG, "Failed to create location task update queue");
        return false;
    }

    locationParams.gps = gps;

    BaseType_t result = xTaskCreatePinnedToCore(
        locationTask,
        "Location",
        stackDepth,
        &locationParams,
        8,
        nullptr,
        1
    );

    if (result == pdPASS) {
        LOGI(TAG, "Location task created on core 1 (priority 9)");
        return true;
    } else {
        LOGE(TAG, "Failed to create Location task");
        vQueueDelete(locationTaskUpdateQueue);
        locationTaskUpdateQueue = nullptr;
        return false;
    }
}

#define GPS_READ_INTERVAL_MS 5000
#define GPS_READ_TIMEOUT_MS 10000

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

                if (locationTaskUpdateQueue != nullptr) {
                    BaseType_t result = xQueueSend(locationTaskUpdateQueue, &data, portMAX_DELAY);
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
