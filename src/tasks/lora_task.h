#pragma once
#include "config.h"
#include "MeshPacket.h"
#include "MeshAdvert.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class LoRaModule;

// Discriminator for outbound queue items.
enum class LoRaTxType : uint8_t { MESH_PACKET, ADVERT };

// Item placed on loraTxQueue by producers (main_task, mqtt_manager, etc.).
// Set `type` then fill the corresponding field; the other field is ignored.
struct LoRaTxMsg {
    LoRaTxType           type;
    meshcore::MeshPacket packet;  // valid when type == MESH_PACKET
    meshcore::Advert     advert;  // valid when type == ADVERT
};

// Item placed on loraRxQueue by lora_task for every received (or echoed) frame.
// The packet is always decoded; inspect packet.payloadType to classify it.
// isEcho==true means we sent this frame (rssi==0, snr==0).
struct LoRaRxMsg {
    int16_t              rssi;
    float                snr;
    bool                 isEcho;
    meshcore::MeshPacket packet;
};

struct LoRaTaskParams {
    LoRaModule* lora;
};

extern QueueHandle_t loraRxQueue;  // lora_task  → main_task  (LoRaRxMsg)
extern QueueHandle_t loraTxQueue;  // producers  → lora_task  (LoRaTxMsg)

/**
 * @brief LoRa task — interrupt-driven RX, queue-driven TX.
 *
 * Always stays in continuous RX mode.  When a LoRaTxMsg arrives on
 * loraTxQueue it switches to TX, transmits, echoes a LoRaRxMsg with
 * isEcho=true onto loraRxQueue, then returns to RX.  Received frames are
 * parsed into LoRaRxMsg and forwarded on loraRxQueue.
 *
 * @param pvParameters Pointer to LoRaTaskParams.
 */
void loraTask(void* pvParameters);
bool setupLoRaTask(LoRaModule* lora, uint32_t stackDepth);
