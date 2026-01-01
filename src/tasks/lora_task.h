#pragma once

#include "config.h"

/**
 * @brief LoRa task function - handles TX/RX radio operations
 *
 * This task:
 * - Waits for packets on TX queue and transmits them
 * - Listens for incoming packets and places them on RX queue
 * - Manages radio state (TX/RX switching)
 *
 * @param pvParameters Pointer to LoRaModule instance
 */
void loraTask(void* pvParameters);
