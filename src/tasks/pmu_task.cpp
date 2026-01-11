#include "pmu_task.h"

#ifdef HAS_PMU

#include "PMUModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "PMU_TASK";
QueueHandle_t pmuStateQueue = nullptr;

void pmuTask(void* pvParameters) {
    PMUModule* pmu = static_cast<PMUModule*>(pvParameters);

    if (!pmu) {
        LOGE(TAG, "PMU module pointer is null, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    if (!pmuStateQueue) {
        LOGE(TAG, "PMU state queue is null, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "PMU task started");

    while (1) {
        // Send battery state every minute (on timeout or after interrupt)
        PMUState state;
        state.battery_voltage = pmu->getBatteryVoltage();

        if (xQueueSend(pmuStateQueue, &state, 0) != pdPASS) {
            LOGW(TAG, "Failed to send PMU state to queue (queue full)");
        } else {
            LOGD(TAG, "Battery voltage: %u mV", state.battery_voltage);
        }

        // Wait for PMU interrupt or timeout after 60 seconds
        uint32_t notifyValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60000));

        if (notifyValue > 0) {
            // PMU interrupt received
            LOGD(TAG, "PMU interrupt received, processing events...");
            pmu->processEvents(nullptr);
        }

        
    }
}

#endif // HAS_PMU
