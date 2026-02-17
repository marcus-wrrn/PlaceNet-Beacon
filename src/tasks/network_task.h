#pragma once
#include <freertos/FreeRTOS.h>
#include "PlaceNetConfig.h"

bool setupNetworkTask(PlaceNetConfig* config, uint32_t stackDepth);
