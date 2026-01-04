#pragma once

#include "config.h"

class LoRaModule;
#ifdef DISPLAY_MODEL
class DisplayModule;
#endif

struct LoRaTaskParams {
    LoRaModule* lora;
#ifdef DISPLAY_MODEL
    DisplayModule* display;
#endif
};

/**
 * @brief LoRa task function - handles TX/RX radio operations
 *
 * This task:
 * - Waits for packets on TX queue and transmits them
 * - Listens for incoming packets and places them on RX queue
 * - Manages radio state (TX/RX switching)
 *
 * @param pvParameters Pointer to LoRaTaskParams struct
 */
void loraTask(void* pvParameters);
