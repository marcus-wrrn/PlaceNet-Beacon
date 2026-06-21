#include "pmu_task.h"

#ifdef HAS_PMU

#include "PMUModule.h"
#include "main_task.h"
#include "logger.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "PMU_TASK";
QueueHandle_t pmuStateQueue = nullptr;

static const uint8_t  PEKEY_SETUP_PRESSES   = 3; // number of times needed to press key to enter setup
static const uint32_t PEKEY_SETUP_WINDOW_MS = 5000;

// Invoked from PMUModule::processEvents() on each PEKEY short-press IRQ.
static void onPmuShortPress() {
    static uint8_t  pressCount = 0;
    static uint32_t windowStart = 0;

    uint32_t now = millis();
    if (pressCount == 0 || (now - windowStart) > PEKEY_SETUP_WINDOW_MS) {
        pressCount  = 0;
        windowStart = now;
    }
    pressCount++;
    LOGI(TAG, "Power-key short press %u/%u", pressCount, PEKEY_SETUP_PRESSES);

    if (pressCount >= PEKEY_SETUP_PRESSES) {
        pressCount = 0;
        requestSetupModeReboot();  // sets RTC flag and reboots — does not return
    }
}

bool setupPMUTask(PMUModule* pmu, uint32_t stackDepth) {
    if (!pmu) {
        LOGE(TAG, "PMU module pointer is null");
        return false;
    }

    pmuStateQueue = xQueueCreate(5, sizeof(PMUState));
    if (!pmuStateQueue) {
        LOGE(TAG, "Failed to create PMU state queue");
        return false;
    }

    TaskHandle_t pmuTaskHandle = nullptr;
    BaseType_t result = xTaskCreatePinnedToCore(
        pmuTask,
        "PMU",
        stackDepth,
        pmu,
        configMAX_PRIORITIES - 1,
        &pmuTaskHandle,
        0
    );

    if (result == pdPASS && pmuTaskHandle != nullptr) {
        pmu->setTaskHandle(pmuTaskHandle);
        LOGI(TAG, "PMU task created on core 0 (priority %d)", configMAX_PRIORITIES - 1);
        return true;
    } else {
        LOGE(TAG, "Failed to create PMU task");
        vQueueDelete(pmuStateQueue);
        pmuStateQueue = nullptr;
        return false;
    }
}

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
            pmu->processEvents(onPmuShortPress);
        }

        
    }
}

#endif // HAS_PMU
