#pragma once
#include <freertos/FreeRTOS.h>
#include "PlaceNetConfig.h"
#include "SDCardModule.h"

bool setupNetworkTask(PlaceNetConfig* config, SDCardModule* sd, uint32_t stackDepth);
