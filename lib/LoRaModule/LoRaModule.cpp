#include "LoRaModule.h"
#include "logger.h"

static const char* TAG = "LORA_MODULE";

LoRaModule::LoRaModule()
    : radio_(nullptr),
      mode_(LORA_MODE_IDLE),
      initialized_(false),
      frequency_(CONFIG_RADIO_FREQ),
      bandwidth_(CONFIG_RADIO_BW),
      spreadingFactor_(10),
      codingRate_(7),
      txPower_(CONFIG_RADIO_OUTPUT_POWER),
      syncWord_(0x12),
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

    int state = radio_->begin(frequency_, bandwidth_, spreadingFactor_, codingRate_, syncWord_, txPower_, 8);

    if (state != RADIOLIB_ERR_NONE) {
        LOGE(TAG, "Radio initialization failed, code: %d", state);
        delete radio_;
        radio_ = nullptr;
        return false;
    }

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

bool LoRaModule::receive(LoRaPacket* packet, uint32_t timeoutMs) {
    if (!initialized_ || !radio_ || !packet) {
        return false;
    }

    int state = radio_->receive(packet->data, LORA_MAX_PACKET_SIZE, timeoutMs * 1000);

    if (state == RADIOLIB_ERR_NONE) {
        packet->length = radio_->getPacketLength();
        packet->rssi = radio_->getRSSI();
        packet->snr = radio_->getSNR();

        LOGI(TAG, "Packet received: %d bytes, RSSI: %d dBm, SNR: %.2f dB",
             packet->length, packet->rssi, packet->snr);

        mode_ = LORA_MODE_RX;
        return true;
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        return false;
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        LOGD(TAG, "CRC mismatch, ignoring packet");
        return false;
    } else {
        LOGW(TAG, "Receive error, code: %d", state);
        return false;
    }
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
