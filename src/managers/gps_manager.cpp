#include "gps_manager.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


static const char* TAG = "GPS-MANAGER";

bool GPSManager::updateGPS() {
    if (locationUpdateQueue && xQueueReceive(locationUpdateQueue, &data_, 0) == pdPASS) {
        LOGI(TAG, "GPS location updated: %.6f, %.6f (Sats: %d)",
                data_.position.latitude, data_.position.longitude,
                data_.metadata.satelliteCount);
        gpsUpdated_ = true;
        gpsDataIsNull_ = false;
        return true;
    }
    gpsUpdated_ = false;
    return false;
}

void GPSManager::logGPS(DisplayModule* display) {
    if (display && !gpsDataIsNull_) {
        display->drawLine("%.6f, %.6f", data_.position.latitude, data_.position.longitude);
        display->drawLine("Sats: %d, Alt: %.1fm", data_.metadata.satelliteCount, data_.metadata.altitude);
    }
}

