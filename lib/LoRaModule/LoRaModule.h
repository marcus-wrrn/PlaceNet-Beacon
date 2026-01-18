#pragma once

#include "config.h"
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstdint>

#define LORA_MAX_PACKET_SIZE 255

struct LoRaPacket {
    uint8_t data[LORA_MAX_PACKET_SIZE];
    uint8_t length;
    int16_t rssi;
    float snr;
};

enum LoRaMode {
    LORA_MODE_IDLE,
    LORA_MODE_TX,
    LORA_MODE_RX,
    LORA_MODE_SLEEP
};

class LoRaModule {
public:
    LoRaModule();
    ~LoRaModule();
    bool init();
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Transmit data immediately (blocking)
     * @param data Pointer to data buffer
     * @param length Length of data to transmit
     * @return true if transmission successful
     */
    bool transmit(const uint8_t* data, uint8_t length);

    /**
     * @brief Start continuous receive mode
     * @return true if receive mode started successfully
     */
    bool startListening();

    /**
     * @brief Check for and retrieve received packet (blocking with timeout)
     * @param packet Pointer to LoRaPacket struct to fill with received data
     * @param timeoutMs Timeout in milliseconds (default: 100ms)
     * @return true if packet was received
     */
    bool receive(LoRaPacket* packet, uint32_t timeoutMs = 100);

    /**
     * @brief Set radio mode
     * @param mode LoRa mode (IDLE, TX, RX, SLEEP)
     * @return true if mode set successfully
     */
    bool setMode(LoRaMode mode);

    LoRaMode getMode() const { return mode_; }

    /**
     * @brief Get duty cycle percentage over specified window
     * @param windowMs Time window in milliseconds (default: 3600000 = 1 hour)
     * @return Duty cycle as percentage (0.0 - 100.0)
     */
    float getDutyCycle(uint32_t windowMs = 3600000);

private:
    struct TransmissionRecord {
        uint32_t timestamp;
        uint32_t timeOnAir;
    };

    static const uint8_t MAX_TX_RECORDS = 100;

    SX1262* radio_;

    LoRaMode mode_;
    bool initialized_;

    float frequency_;
    float bandwidth_;
    uint8_t spreadingFactor_;
    uint8_t codingRate_;
    int8_t txPower_;
    uint8_t syncWord_;

    TransmissionRecord txRecords_[MAX_TX_RECORDS];
    uint8_t txRecordIndex_;
    uint8_t txRecordCount_;

    void recordTransmission(uint32_t timeOnAir);
};
