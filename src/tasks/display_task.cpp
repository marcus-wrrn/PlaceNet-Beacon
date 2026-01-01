#include "display_task.h"

#ifdef DISPLAY_MODEL

#include "DisplayModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "DISPLAY_TASK";

void displayTask(void* pvParameters) {
    DisplayModule* display = static_cast<DisplayModule*>(pvParameters);

    if (!display) {
        LOGE(TAG, "Display module pointer is null, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    if (!display->init()) {
        LOGE(TAG, "Display initialization failed, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "Display task started");

    DisplayCommand cmd;
    QueueHandle_t queue = display->getCommandQueue();

    while (1) {
        if (xQueueReceive(queue, &cmd, portMAX_DELAY) == pdTRUE) {
            display->renderCommand(cmd);
        }
    }
}

#endif
