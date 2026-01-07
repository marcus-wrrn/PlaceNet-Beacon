#pragma once

#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/**
 * @file display_task.h
 * @brief FreeRTOS task for display rendering
 */

/**
 * @brief Parameters for display task
 */
struct DisplayTaskParams {
    QueueHandle_t eventQueue;
};

#ifdef DISPLAY_MODEL

/**
 * @brief Display task function - renders display commands from queue
 *
 * This task blocks waiting for display events on the queue.
 * DisplayModule is instantiated locally within the task.
 *
 * @param pvParameters Pointer to DisplayTaskParams struct
 */
void displayTask(void* pvParameters);

#endif // DISPLAY_MODEL
