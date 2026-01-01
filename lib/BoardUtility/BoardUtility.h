#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <esp_sleep.h>

/**
 * @file BoardUtility.h
 * @brief Static utility functions for board initialization and diagnostics
 */

typedef struct {
    String          chipModel;
    float           psramSize;
    uint8_t         chipModelRev;
    uint8_t         chipFreq;
    uint8_t         flashSize;
    uint8_t         flashSpeed;
} DevInfo_t;

namespace BoardUtility {
    /**
     * @brief Print chip information (wakeup reason, PSRAM, flash size, flash speed, model, revision, frequency, SDK version, compile date/time, MAC address)
     */
    void printChipInfo();

    void printWakeupReason();

    /**
     * @brief Scan I2C bus for devices
     * @param wire Pointer to TwoWire interface (e.g., &Wire or &Wire1)
     */
    void scanI2C(TwoWire* wire);

    /**
     * @brief Get device information structure
     * @return DevInfo_t containing chip details
     */
    DevInfo_t getDeviceInfo();
}
