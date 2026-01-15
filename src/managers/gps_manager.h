#pragma once
#include "config.h"

#ifdef HAS_GPS
#include "GPSModule.h"
#include "DisplayModule.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>


extern QueueHandle_t locationUpdateQueue;

class GPSManager {
public:
    GPSManager() : data_{}, gpsUpdated_(false), gpsDataIsNull_(true) {}

    bool updateGPS();
    void logGPS(DisplayModule* display);

    bool hasData() const { return !gpsDataIsNull_; }
    const GPSData& getData() const { return data_; }

private:
    GPSData data_;
    bool gpsUpdated_;
    bool gpsDataIsNull_;
};

#endif