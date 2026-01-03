#include "pmu_task.h"

#ifdef HAS_PMU

#include "PMUModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "PMU_TASK";

void pmuTask(void* pvParameters) {
    PMUModule* pmu = static_cast<PMUModule*>(pvParameters);

    if (!pmu) {
        LOGE(TAG, "PMU module pointer is null, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "PMU task started");

    while (1) {
        uint32_t notifyValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (notifyValue > 0) {
            LOGD(TAG, "PMU interrupt received, processing events...");
            pmu->processEvents(nullptr);
        }
    }
}

#endif // HAS_PMU
