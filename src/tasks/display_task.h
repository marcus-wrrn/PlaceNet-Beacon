#pragma once

#include "config.h"

#ifdef DISPLAY_MODEL

/**
 * @file display_task.h
 * @brief FreeRTOS task for display rendering
 */

/**
 * @brief Display task function - renders display commands from queue
 *
 * This task blocks waiting for display commands on the queue.
 * When a command is received, it renders it using the DisplayModule.
 *
 * @param pvParameters Pointer to DisplayModule instance
 */
void displayTask(void* pvParameters);

#endif // DISPLAY_MODEL
