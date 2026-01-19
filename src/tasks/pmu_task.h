#pragma once
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#ifdef HAS_PMU
#include "PMUModule.h"

extern QueueHandle_t pmuStateQueue;

/**
 * @brief PMU task function - handles power management events
 *
 * This task blocks waiting for ISR notifications from the PMU interrupt.
 * When notified, it processes PMU events (battery, VBUS, power button).
 *
 * @param pvParameters Pointer to PMUModule instance
 */
void pmuTask(void* pvParameters);
bool setupPMUTask(PMUModule* pmu, uint32_t stackDepth);

#endif