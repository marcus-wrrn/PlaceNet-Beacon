#include "pmu_manager.h"

#ifdef HAS_PMU

#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "PMU-MANAGER";

bool PMUManager::updatePMU() {
    if (pmuStateQueue && xQueueReceive(pmuStateQueue, &state_, 0) == pdPASS) {
        LOGI(TAG, "Battery voltage updated: %u mV", state_.battery_voltage);
        pmuUpdated_ = true;
        pmuDataIsNull_ = false;
        return true;
    }
    pmuUpdated_ = false;
    return false;
}

void PMUManager::logPMU(DisplayModule* display) {
    if (display && !pmuDataIsNull_) {
        display->drawLine("Bat: %u mV", state_.battery_voltage);
    }
}

#endif // HAS_PMU
