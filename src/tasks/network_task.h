#pragma once
#include <freertos/FreeRTOS.h>

void networkTask(void* pvParameters);
bool setupNetworkTask(uint32_t stackDepth);
