#pragma once

#include "config.h"

#ifdef HAS_PMU

#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>

/**
 * @file PMUManager.h
 * @brief Power Management Unit controller for AXP192/AXP2101
 */

class PMUManager {
public:
    /**
     * @brief Constructor
     * @param wire I2C bus interface (e.g., Wire1)
     * @param irqPin GPIO pin for PMU interrupts
     */
    PMUManager(TwoWire& wire, int irqPin);

    /**
     * @brief Destructor - cleans up PMU and detaches interrupt
     */
    ~PMUManager();

    // Non-copyable, non-movable
    PMUManager(const PMUManager&) = delete;
    PMUManager& operator=(const PMUManager&) = delete;
    PMUManager(PMUManager&&) = delete;
    PMUManager& operator=(PMUManager&&) = delete;

    /**
     * @brief Initialize and configure the PMU
     * @return true if PMU was detected and configured successfully
     */
    bool initialize();

    /**
     * @brief Check if PMU is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Disable all peripheral power outputs
     * Called before deep sleep or shutdown
     */
    void disablePeripherals();

    /**
     * @brief Process PMU interrupt events
     * @param buttonPressCallback Optional callback for button press events
     */
    void processEvents(void (*buttonPressCallback)(void) = nullptr);

    /**
     * @brief Check if PMU has pending interrupt
     * @return true if interrupt flag is set
     */
    bool hasInterrupt() const { return pmuInterrupt_; }

    /**
     * @brief Clear the interrupt flag
     */
    void clearInterrupt() { pmuInterrupt_ = false; }

    /**
     * @brief Get direct access to the PMU interface
     * @return Pointer to XPowersLibInterface, or nullptr if not initialized
     */
    XPowersLibInterface* getPMU() { return pmu_; }
    const XPowersLibInterface* getPMU() const { return pmu_; }

    /**
     * @brief Get PMU chip model name
     * @return Chip model string ("AXP192", "AXP2101", or "Unknown")
     */
    const char* getChipModel() const;

    /**
     * @brief Check if battery is connected
     * @return true if battery detected
     */
    bool hasBattery() const;

    /**
     * @brief Check if battery is charging
     * @return true if charging
     */
    bool isCharging() const;

    /**
     * @brief Check if VBUS (USB) is connected
     * @return true if VBUS detected
     */
    bool isVbusConnected() const;

    /**
     * @brief Get battery voltage in millivolts
     * @return Battery voltage (mV)
     */
    uint16_t getBatteryVoltage() const;

    /**
     * @brief Get VBUS voltage in millivolts
     * @return VBUS voltage (mV)
     */
    uint16_t getVbusVoltage() const;

    /**
     * @brief Get system voltage in millivolts
     * @return System voltage (mV)
     */
    uint16_t getSystemVoltage() const;

private:
    /**
     * @brief Static ISR handler for PMU interrupts
     */
    static void pmuInterruptHandler();

    /**
     * @brief Instance interrupt handler
     */
    void handleInterrupt();

    /**
     * @brief Detect and initialize the PMU chip
     * @return true if PMU detected
     */
    bool detectAndInitializePMU();

    /**
     * @brief Configure AXP192 power rails and interrupts
     */
    void configureAXP192();

    /**
     * @brief Configure AXP2101 power rails and interrupts
     */
    void configureAXP2101();

    /**
     * @brief Enable voltage and current measurements
     */
    void configureMeasurements();

    /**
     * @brief Log all power channel voltages
     */
    void logPowerChannels();

    // Member variables
    TwoWire& wire_;                      // I2C bus reference
    int irqPin_;                         // Interrupt GPIO pin
    XPowersLibInterface* pmu_;           // PMU interface
    bool initialized_;                   // Initialization status
    volatile bool pmuInterrupt_;         // Interrupt flag

    // Static instance pointer for ISR access
    static PMUManager* instance_;
};

#endif // HAS_PMU
