#include "LoRaModule.h"
#include "logger.h"

static const char* TAG = "LORA_MODULE";

// Static singleton pointer for the ISR callback.
LoRaModule* LoRaModule::instance_ = nullptr;

void IRAM_ATTR LoRaModule::onDio1Interrupt() {
    if (instance_) {
        instance_->rxFlag_ = true;
    }
}

LoRaModule::LoRaModule()
    : radio_(nullptr),
      mode_(LORA_MODE_IDLE),
      initialized_(false),
      rxFlag_(false),
      frequency_(MESHCORE_RADIO_FREQ),
      bandwidth_(MESHCORE_RADIO_BW),
      spreadingFactor_(MESHCORE_RADIO_SF),
      codingRate_(MESHCORE_RADIO_CR),
      txPower_(CONFIG_RADIO_OUTPUT_POWER),
      syncWord_(MESHCORE_SYNC_WORD),
      txRecordIndex_(0),
      txRecordCount_(0)
{
    memset(txRecords_, 0, sizeof(txRecords_));
}

LoRaModule::~LoRaModule() {
    if (radio_) {
        delete radio_;
        radio_ = nullptr;
    }
}

bool LoRaModule::init() {
    if (initialized_) {
        LOGW(TAG, "LoRa already initialized");
        return true;
    }

    LOGI(TAG, "Initializing LoRa radio...");
    LOGI(TAG, "Frequency: %.1f MHz, BW: %.1f kHz, SF: %d, CR: 4/%d, Power: %d dBm",
         frequency_, bandwidth_, spreadingFactor_, codingRate_, txPower_);

    // TODO: make radio module dynamic depending on modem
    radio_ = new SX1262(new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN));

    if (!radio_) {
        LOGE(TAG, "Failed to allocate radio instance");
        return false;
    }

    const uint16_t preambleLen = MESHCORE_PREAMBLE_LEN(spreadingFactor_);
    int state = radio_->begin(frequency_, bandwidth_, spreadingFactor_, codingRate_,
                              syncWord_, txPower_, preambleLen);

    if (state != RADIOLIB_ERR_NONE) {
        LOGE(TAG, "Radio initialization failed, code: %d", state);
        delete radio_;
        radio_ = nullptr;
        return false;
    }

    // MeshCore frames are sent with CRC enabled; receivers drop packets whose
    // CRC mode does not match, so this is required for over-the-air parity.
    state = radio_->setCRC(1);
    if (state != RADIOLIB_ERR_NONE) {
        LOGE(TAG, "Failed to enable CRC, code: %d", state);
        delete radio_;
        radio_ = nullptr;
        return false;
    }

    // Register the DIO1 interrupt so the radio signals us when a full
    // packet has been received.  The ISR simply sets rxFlag_; the task
    // drains it via readPacket().
    instance_ = this;
    rxFlag_   = false;
    radio_->setDio1Action(onDio1Interrupt);

    LOGI(TAG, "LoRa radio initialized successfully");
    initialized_ = true;
    return true;
}

bool LoRaModule::setMode(LoRaMode mode) {
    if (!initialized_) {
        LOGW(TAG, "Cannot set mode - LoRa not initialized");
        return false;
    }

    LOGD(TAG, "Setting mode to %d", mode);
    mode_ = mode;
    return true;
}

bool LoRaModule::transmit(const uint8_t* data, uint8_t length) {
    if (!initialized_) {
        LOGW(TAG, "Cannot transmit - LoRa not initialized");
        return false;
    }

    if (!radio_) {
        LOGE(TAG, "Radio instance is null");
        return false;
    }

    if (length > LORA_MAX_PACKET_SIZE) {
        LOGE(TAG, "Packet too large: %d bytes (max %d)", length, LORA_MAX_PACKET_SIZE);
        return false;
    }

    uint32_t timeOnAirUs = radio_->getTimeOnAir(length);
    uint32_t timeOnAirMs = (timeOnAirUs + 500) / 1000;
    LOGD(TAG, "Transmitting %d bytes (ToA: %lu ms)...", length, timeOnAirMs);

    int state = radio_->transmit(const_cast<uint8_t*>(data), length);

    if (state != RADIOLIB_ERR_NONE) {
        LOGE(TAG, "Transmission failed, code: %d", state);
        return false;
    } 

    recordTransmission(timeOnAirMs);
    LOGI(TAG, "Transmission successful");
    return true;
    
}

bool LoRaModule::startListening() {
    if (!initialized_) {
        LOGW(TAG, "Cannot start listening - LoRa not initialized");
        return false;
    }

    if (!radio_) {
        LOGE(TAG, "Radio instance is null");
        return false;
    }

    // Clear any stale DIO1 flag before arming the receiver.
    // DIO1 is shared between TX-done and RX-done events; a TX that just
    // finished will have set rxFlag_ — if we don't clear it here we will
    // immediately try to read a packet from an empty FIFO.
    rxFlag_ = false;

    LOGI(TAG, "Starting continuous receive mode...");
    int state = radio_->startReceive();

    if (state == RADIOLIB_ERR_NONE) {
        mode_ = LORA_MODE_RX;
        LOGI(TAG, "Receive mode started successfully");
        return true;
    } 
    
    LOGE(TAG, "Failed to start receive mode, code: %d", state);
    return false;
}

bool LoRaModule::readPacket(LoRaPacket* packet) {
    if (!initialized_ || !radio_ || !packet) {
        return false;
    }

    if (!rxFlag_) {
        return false;
    }
    rxFlag_ = false;

    size_t len = radio_->getPacketLength();

    int state = radio_->readData(packet->data, len);

    if (radio_->startReceive() != RADIOLIB_ERR_NONE) {
        LOGW(TAG, "readPacket: failed to re-arm startReceive");
    } else {
        mode_ = LORA_MODE_RX;
    }

    if (state == RADIOLIB_ERR_NONE) {
        packet->length = (uint8_t)len;
        packet->rssi   = radio_->getRSSI();
        packet->snr    = radio_->getSNR();

        LOGI(TAG, "Packet received: %d bytes, RSSI: %d dBm, SNR: %.2f dB",
             packet->length, packet->rssi, packet->snr);
        return true;
    }

    if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        LOGD(TAG, "CRC mismatch, ignoring packet");
    } else {
        LOGW(TAG, "readData error, code: %d", state);
    }
    return false;
}

void LoRaModule::recordTransmission(uint32_t timeOnAir) {
    txRecords_[txRecordIndex_].timestamp = millis();
    txRecords_[txRecordIndex_].timeOnAir = timeOnAir;

    txRecordIndex_ = (txRecordIndex_ + 1) % MAX_TX_RECORDS;

    if (txRecordCount_ < MAX_TX_RECORDS) {
        txRecordCount_++;
    }
}

float LoRaModule::getDutyCycle(uint32_t windowMs) {
    if (txRecordCount_ == 0) {
        return 0.0f;
    }

    uint32_t currentTime = millis();
    uint32_t totalToA = 0;
    uint8_t validRecords = 0;

    for (uint8_t i = 0; i < txRecordCount_; i++) {
        uint32_t age = currentTime - txRecords_[i].timestamp;

        if (age <= windowMs) {
            totalToA += txRecords_[i].timeOnAir;
            validRecords++;
        }
    }

    if (validRecords == 0) {
        return 0.0f;
    }

    float dutyCycle = (float)totalToA / (float)windowMs * 100.0f;
    return dutyCycle;
}
