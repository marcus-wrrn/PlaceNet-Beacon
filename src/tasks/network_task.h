#pragma once
#include <freertos/FreeRTOS.h>
#include "PlaceNetConfig.h"

void networkTask(void* pvParameters);
bool setupNetworkTask(PlaceNetConfig* config, uint32_t stackDepth);
