#pragma once

#include "config.h"

#ifdef HAS_GPS

#include "GPSModule.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct LocationTaskParams {
    GPSModule* gps;
};

extern QueueHandle_t locationTaskUpdateQueue;

void locationTask(void* pvParameters);

#endif // HAS_GPS
