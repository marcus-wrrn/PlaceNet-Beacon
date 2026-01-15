#pragma once

#include "config.h"

#ifdef HAS_PMU

#include "PMUModule.h"
#include "DisplayModule.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

extern QueueHandle_t pmuStateQueue;

class PMUManager {
public:
    PMUManager() : state_{}, pmuUpdated_(false), pmuDataIsNull_(true) {}

    bool updatePMU();
    void logPMU(DisplayModule* display);

    bool hasData() const { return !pmuDataIsNull_; }
    const PMUState& getState() const { return state_; }

private:
    PMUState state_;
    bool pmuUpdated_;
    bool pmuDataIsNull_;
};

#endif // HAS_PMU
