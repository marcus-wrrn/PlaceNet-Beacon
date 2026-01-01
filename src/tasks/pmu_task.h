#pragma once
#include "config.h"
#ifdef HAS_PMU

/**
 * @brief PMU task function - handles power management events
 *
 * This task blocks waiting for ISR notifications from the PMU interrupt.
 * When notified, it processes PMU events (battery, VBUS, power button).
 *
 * @param pvParameters Pointer to PMUModule instance
 */
void pmuTask(void* pvParameters);

#endif