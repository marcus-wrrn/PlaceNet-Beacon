#pragma once

#include "config.h"
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/**
 * @file LoRaModule.h
 * @brief LoRa radio module for FreeRTOS-based communication
 *
 * TODO: Implementation required - this is a skeleton design
 */

// Maximum packet size
#define LORA_MAX_PACKET_SIZE 255

struct LoRaPacket {
    uint8_t data[LORA_MAX_PACKET_SIZE];
    uint8_t length;
    int16_t rssi;  // For received packets
    float snr;     // For received packets
};

enum LoRaMode {
    LORA_MODE_IDLE,
    LORA_MODE_TX,
    LORA_MODE_RX,
    LORA_MODE_SLEEP
};

class LoRaModule {
public:
    /**
     * @brief Constructor
     */
    LoRaModule();

    /**
     * @brief Destructor
     */
    ~LoRaModule();

    /**
     * @brief Initialize the LoRa radio
     * @return true if radio initialized successfully
     */
    bool init();

    /**
     * @brief Check if module is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Send a LoRa packet
     * @param data Pointer to data buffer
     * @param length Length of data to send
     * @return true if packet queued successfully
     */
    bool send(const uint8_t* data, uint8_t length);

    /**
     * @brief Set radio mode
     * @param mode LoRa mode (IDLE, TX, RX, SLEEP)
     * @return true if mode set successfully
     */
    bool setMode(LoRaMode mode);

    /**
     * @brief Get TX queue handle
     * @return QueueHandle_t for sending packets
     */
    QueueHandle_t getTxQueue() { return txQueue_; }

    /**
     * @brief Get RX queue handle
     * @return QueueHandle_t for receiving packets
     */
    QueueHandle_t getRxQueue() { return rxQueue_; }

    /**
     * @brief Get current mode
     * @return Current LoRa mode
     */
    LoRaMode getMode() const { return mode_; }

    // TODO: Add methods for:
    // - setFrequency()
    // - setBandwidth()
    // - setSpreadingFactor()
    // - setCodingRate()
    // - setPower()
    // - getLastRSSI()
    // - getLastSNR()

private:
    // TODO: Add RadioLib radio instance
    // Example: SX1262* radio_;

    QueueHandle_t txQueue_;
    QueueHandle_t rxQueue_;
    LoRaMode mode_;
    bool initialized_;

    // TODO: Add radio configuration parameters
    // - Frequency
    // - Bandwidth
    // - Spreading factor
    // - Coding rate
    // - TX power
};
