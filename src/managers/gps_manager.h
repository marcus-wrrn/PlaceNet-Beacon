#pragma once
#include "config.h"

#ifdef HAS_GPS
#include "GPSModule.h"
#include "DisplayModule.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class GPSManager {
public:
    GPSManager(QueueHandle_t queue) : data_{}, gpsUpdated_(false), gpsDataIsNull_(true), locationQueue_(queue) {}

    QueueHandle_t getQueue() const { return locationQueue_; }

    bool updateGPS();
    void logGPS(DisplayModule* display);

    bool hasData() const { return !gpsDataIsNull_; }
    const GPSData& getData() const { return data_; }

private:
    GPSData data_;
    bool gpsUpdated_;
    bool gpsDataIsNull_;
    QueueHandle_t locationQueue_;
};

#endif