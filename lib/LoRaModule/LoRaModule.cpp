#include "LoRaModule.h"
#include "logger.h"

static const char* TAG = "LORA_MODULE";

LoRaModule::LoRaModule()
    : txQueue_(nullptr)
    , rxQueue_(nullptr)
    , mode_(LORA_MODE_IDLE)
    , initialized_(false)
{
    // Create TX and RX queues
    txQueue_ = xQueueCreate(10, sizeof(LoRaPacket));
    rxQueue_ = xQueueCreate(10, sizeof(LoRaPacket));

    if (!txQueue_ || !rxQueue_) {
        LOGE(TAG, "Failed to create LoRa queues");
    }
}

LoRaModule::~LoRaModule() {
    if (txQueue_) {
        vQueueDelete(txQueue_);
        txQueue_ = nullptr;
    }
    if (rxQueue_) {
        vQueueDelete(rxQueue_);
        rxQueue_ = nullptr;
    }

    // TODO: Delete radio instance
}

bool LoRaModule::init() {
    if (initialized_) {
        LOGW(TAG, "LoRa already initialized");
        return true;
    }

    LOGI(TAG, "Initializing LoRa radio...");

    // TODO: Implement radio initialization
    // Example:
    // radio_ = new SX1262(new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN));
    // int state = radio_->begin(frequency, bandwidth, sf, cr, syncWord, power, preambleLength);
    // if (state != RADIOLIB_ERR_NONE) {
    //     LOGE(TAG, "Radio initialization failed, code: %d", state);
    //     return false;
    // }

    LOGW(TAG, "LoRa initialization STUB - not yet implemented");
    LOGI(TAG, "TODO: Implement RadioLib initialization");

    initialized_ = false;  // Set to true when actually implemented
    return false;  // Return true when implemented
}

bool LoRaModule::send(const uint8_t* data, uint8_t length) {
    if (!initialized_) {
        LOGW(TAG, "Cannot send - LoRa not initialized");
        return false;
    }

    if (length > LORA_MAX_PACKET_SIZE) {
        LOGE(TAG, "Packet too large: %d bytes (max %d)", length, LORA_MAX_PACKET_SIZE);
        return false;
    }

    // Create packet
    LoRaPacket packet;
    memcpy(packet.data, data, length);
    packet.length = length;

    // Queue packet for TX task
    if (xQueueSend(txQueue_, &packet, 0) != pdTRUE) {
        LOGW(TAG, "TX queue full, packet dropped");
        return false;
    }

    LOGD(TAG, "Packet queued for transmission (%d bytes)", length);
    return true;
}

bool LoRaModule::setMode(LoRaMode mode) {
    if (!initialized_) {
        LOGW(TAG, "Cannot set mode - LoRa not initialized");
        return false;
    }

    // TODO: Implement radio mode changes
    LOGD(TAG, "Setting mode to %d (STUB)", mode);

    mode_ = mode;
    return true;
}
