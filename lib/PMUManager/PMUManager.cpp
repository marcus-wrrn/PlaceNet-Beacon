/**
 * @file PMUManager.cpp
 * @brief Power Management Unit implementation for AXP192/AXP2101
 */

#include "PMUManager.h"

#ifdef HAS_PMU

#include "logger.h"
#include <esp_sleep.h>

#define TAG "PMU_MANAGER"

static PMUManager* g_pmuInstance = nullptr;

PMUManager* PMUManager::instance_ = nullptr;

PMUManager::PMUManager(TwoWire& wire, int irqPin)
    : wire_(wire)
    , irqPin_(irqPin)
    , pmu_(nullptr)
    , initialized_(false)
    , pmuInterrupt_(false)
{
    if (g_pmuInstance != nullptr) {
        LOGW(TAG, "Multiple PMUManager instances detected!");
    }
    g_pmuInstance = this;
    instance_ = this;
}

PMUManager::~PMUManager() {
    if (pmu_) {
        disablePeripherals();
        delete pmu_;
        pmu_ = nullptr;
    }

    if (g_pmuInstance == this) {
        g_pmuInstance = nullptr;
    }
    if (instance_ == this) {
        instance_ = nullptr;
    }

    detachInterrupt(digitalPinToInterrupt(irqPin_));
}

void PMUManager::pmuInterruptHandler() {
    if (g_pmuInstance) {
        g_pmuInstance->handleInterrupt();
    }
}

void PMUManager::handleInterrupt() {
    pmuInterrupt_ = true;
}

bool PMUManager::detectAndInitializePMU() {
    if (!pmu_) {
        pmu_ = new XPowersAXP2101(wire_);
        if (!pmu_->init()) {
            LOGW(TAG, "Failed to find AXP2101 power management");
            delete pmu_;
            pmu_ = nullptr;
        } else {
            LOGI(TAG, "AXP2101 PMU init succeeded, using AXP2101 PMU");
        }
    }

    // Try AXP192 if AXP2101 not found
    if (!pmu_) {
        pmu_ = new XPowersAXP192(wire_);
        if (!pmu_->init()) {
            LOGW(TAG, "Failed to find AXP192 power management");
            delete pmu_;
            pmu_ = nullptr;
        } else {
            LOGI(TAG, "AXP192 PMU init succeeded, using AXP192 PMU");
        }
    }

    return (pmu_ != nullptr);
}

// Configure AXP192
void PMUManager::configureAXP192() {
    pmu_->setProtectedChannel(XPOWERS_DCDC3);

    // lora
    pmu_->setPowerChannelVoltage(XPOWERS_LDO2, 3300);
    // gps
    pmu_->setPowerChannelVoltage(XPOWERS_LDO3, 3300);
    // oled
    pmu_->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);

    pmu_->enablePowerOutput(XPOWERS_LDO2);
    pmu_->enablePowerOutput(XPOWERS_LDO3);

    //protected oled power source
    pmu_->setProtectedChannel(XPOWERS_DCDC1);
    //protected esp32 power source
    pmu_->setProtectedChannel(XPOWERS_DCDC3);
    // enable oled power
    pmu_->enablePowerOutput(XPOWERS_DCDC1);

    //disable not use channel
    pmu_->disablePowerOutput(XPOWERS_DCDC2);

    pmu_->disableIRQ(XPOWERS_AXP192_ALL_IRQ);

    pmu_->enableIRQ(XPOWERS_AXP192_VBUS_REMOVE_IRQ |
                   XPOWERS_AXP192_VBUS_INSERT_IRQ |
                   XPOWERS_AXP192_BAT_CHG_DONE_IRQ |
                   XPOWERS_AXP192_BAT_CHG_START_IRQ |
                   XPOWERS_AXP192_BAT_REMOVE_IRQ |
                   XPOWERS_AXP192_BAT_INSERT_IRQ |
                   XPOWERS_AXP192_PKEY_SHORT_IRQ
                  );
}

// Configure AXP2101
void PMUManager::configureAXP2101() {
    // T-Beam S3 Supreme specific configuration

    // In order to avoid bus occupation, during initialization, the SD card and QMC sensor are powered off and restarted
    if (ESP_SLEEP_WAKEUP_UNDEFINED == esp_sleep_get_wakeup_cause()) {
        LOGI(TAG, "Power off and restart ALDO BLDO..");
        pmu_->disablePowerOutput(XPOWERS_ALDO1);
        pmu_->disablePowerOutput(XPOWERS_ALDO2);
        pmu_->disablePowerOutput(XPOWERS_BLDO1);
        delay(250);
    }

    //gps
    pmu_->setPowerChannelVoltage(XPOWERS_ALDO4, 3300);
    pmu_->enablePowerOutput(XPOWERS_ALDO4);

    // lora
    pmu_->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
    pmu_->enablePowerOutput(XPOWERS_ALDO3);

    // Sensor
    pmu_->setPowerChannelVoltage(XPOWERS_ALDO1, 3300);
    pmu_->enablePowerOutput(XPOWERS_ALDO1);

    pmu_->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
    pmu_->enablePowerOutput(XPOWERS_ALDO2);

    //Sdcard
    pmu_->setPowerChannelVoltage(XPOWERS_BLDO1, 3300);
    pmu_->enablePowerOutput(XPOWERS_BLDO1);

    pmu_->setPowerChannelVoltage(XPOWERS_BLDO2, 3300);
    pmu_->enablePowerOutput(XPOWERS_BLDO2);

    //face m.2
    pmu_->setPowerChannelVoltage(XPOWERS_DCDC3, 3300);
    pmu_->enablePowerOutput(XPOWERS_DCDC3);

    pmu_->setPowerChannelVoltage(XPOWERS_DCDC4, XPOWERS_AXP2101_DCDC4_VOL2_MAX);
    pmu_->enablePowerOutput(XPOWERS_DCDC4);

    pmu_->setPowerChannelVoltage(XPOWERS_DCDC5, 3300);
    pmu_->enablePowerOutput(XPOWERS_DCDC5);

    //ESP32 VDD 3300mV - protected, automatically managed
    pmu_->setProtectedChannel(XPOWERS_DCDC1);

    //not use channel
    pmu_->disablePowerOutput(XPOWERS_DCDC2);
    pmu_->disablePowerOutput(XPOWERS_DLDO1);
    pmu_->disablePowerOutput(XPOWERS_DLDO2);
    pmu_->disablePowerOutput(XPOWERS_VBACKUP);

    // Set constant current charge current limit
    pmu_->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);

    // Set charge cut-off voltage
    pmu_->setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

    // Disable all interrupts
    pmu_->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    // Clear all interrupt flags
    pmu_->clearIrqStatus();
    // Enable the required interrupt function
    pmu_->enableIRQ(
        XPOWERS_AXP2101_BAT_INSERT_IRQ    | XPOWERS_AXP2101_BAT_REMOVE_IRQ      |   //BATTERY
        XPOWERS_AXP2101_VBUS_INSERT_IRQ   | XPOWERS_AXP2101_VBUS_REMOVE_IRQ     |   //VBUS
        XPOWERS_AXP2101_PKEY_SHORT_IRQ    | XPOWERS_AXP2101_PKEY_LONG_IRQ       |   //POWER KEY
        XPOWERS_AXP2101_BAT_CHG_DONE_IRQ  | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
    );
}

// Configure measurements
void PMUManager::configureMeasurements() {
    pmu_->enableSystemVoltageMeasure();
    pmu_->enableVbusVoltageMeasure();
    pmu_->enableBattVoltageMeasure();
}

// Log power channels
void PMUManager::logPowerChannels() {
    LOGI(TAG, "=========================================");
    if (pmu_->isChannelAvailable(XPOWERS_DCDC1)) {
        LOGI(TAG, "DC1  : %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_DCDC1)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_DCDC1));
    }
    if (pmu_->isChannelAvailable(XPOWERS_DCDC2)) {
        LOGI(TAG, "DC2  : %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_DCDC2)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_DCDC2));
    }
    if (pmu_->isChannelAvailable(XPOWERS_DCDC3)) {
        LOGI(TAG, "DC3  : %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_DCDC3)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_DCDC3));
    }
    if (pmu_->isChannelAvailable(XPOWERS_DCDC4)) {
        LOGI(TAG, "DC4  : %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_DCDC4)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_DCDC4));
    }
    if (pmu_->isChannelAvailable(XPOWERS_DCDC5)) {
        LOGI(TAG, "DC5  : %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_DCDC5)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_DCDC5));
    }
    if (pmu_->isChannelAvailable(XPOWERS_LDO2)) {
        LOGI(TAG, "LDO2 : %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_LDO2)   ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_LDO2));
    }
    if (pmu_->isChannelAvailable(XPOWERS_LDO3)) {
        LOGI(TAG, "LDO3 : %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_LDO3)   ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_LDO3));
    }
    if (pmu_->isChannelAvailable(XPOWERS_ALDO1)) {
        LOGI(TAG, "ALDO1: %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_ALDO1)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_ALDO1));
    }
    if (pmu_->isChannelAvailable(XPOWERS_ALDO2)) {
        LOGI(TAG, "ALDO2: %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_ALDO2)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_ALDO2));
    }
    if (pmu_->isChannelAvailable(XPOWERS_ALDO3)) {
        LOGI(TAG, "ALDO3: %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_ALDO3)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_ALDO3));
    }
    if (pmu_->isChannelAvailable(XPOWERS_ALDO4)) {
        LOGI(TAG, "ALDO4: %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_ALDO4)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_ALDO4));
    }
    if (pmu_->isChannelAvailable(XPOWERS_BLDO1)) {
        LOGI(TAG, "BLDO1: %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_BLDO1)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_BLDO1));
    }
    if (pmu_->isChannelAvailable(XPOWERS_BLDO2)) {
        LOGI(TAG, "BLDO2: %s   Voltage: %04u mV",  pmu_->isPowerChannelEnable(XPOWERS_BLDO2)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_BLDO2));
    }
    LOGI(TAG, "=========================================");
}

// Initialize PMU
bool PMUManager::initialize() {
    if (initialized_) {
        LOGW(TAG, "PMU already initialized");
        return true;
    }

    if (!detectAndInitializePMU()) {
        return false;
    }

    pmu_->setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

    // Configure based on chip model
    if (pmu_->getChipModel() == XPOWERS_AXP192) {
        configureAXP192();
    } else if (pmu_->getChipModel() == XPOWERS_AXP2101) {
        configureAXP2101();
    }

    configureMeasurements();
    logPowerChannels();

    // Set the time of pressing the button to turn off
    pmu_->setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    uint8_t opt = pmu_->getPowerKeyPressOffTime();
    const char* timeout_str;
    switch (opt) {
    case XPOWERS_POWEROFF_4S: timeout_str = "4 Second";
        break;
    case XPOWERS_POWEROFF_6S: timeout_str = "6 Second";
        break;
    case XPOWERS_POWEROFF_8S: timeout_str = "8 Second";
        break;
    case XPOWERS_POWEROFF_10S: timeout_str = "10 Second";
        break;
    default:
        timeout_str = "Unknown";
        break;
    }
    LOGI(TAG, "PowerKeyPressOffTime: %s", timeout_str);

    // Attach interrupt
    pinMode(irqPin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(irqPin_), pmuInterruptHandler, FALLING);

    initialized_ = true;
    LOGI(TAG, "PMU initialization complete");
    return true;
}

// Disable peripherals
void PMUManager::disablePeripherals() {
    if (!pmu_) return;

    pmu_->setChargingLedMode(XPOWERS_CHG_LED_OFF);
    // Disable the PMU measurement section
    pmu_->disableSystemVoltageMeasure();
    pmu_->disableVbusVoltageMeasure();
    pmu_->disableBattVoltageMeasure();
    pmu_->disableTemperatureMeasure();
    pmu_->disableBattDetection();

    if (pmu_->getChipModel() == XPOWERS_AXP2101) {
        // Disable all PMU interrupts
        pmu_->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
        pmu_->clearIrqStatus();

        // T-Beam S3 Supreme peripheral power down
        pmu_->disablePowerOutput(XPOWERS_ALDO4);
        pmu_->disablePowerOutput(XPOWERS_ALDO3);
        pmu_->disablePowerOutput(XPOWERS_ALDO2);
        pmu_->disablePowerOutput(XPOWERS_ALDO1);
        pmu_->disablePowerOutput(XPOWERS_BLDO1);
        pmu_->disablePowerOutput(XPOWERS_BLDO2);
        pmu_->disablePowerOutput(XPOWERS_DCDC3);
        pmu_->disablePowerOutput(XPOWERS_DCDC4);
        pmu_->disablePowerOutput(XPOWERS_DCDC5);
    } else if (pmu_->getChipModel() == XPOWERS_AXP192) {
        pmu_->disableIRQ(XPOWERS_AXP192_ALL_IRQ);
        pmu_->clearIrqStatus();
        pmu_->disablePowerOutput(XPOWERS_LDO2);
        pmu_->disablePowerOutput(XPOWERS_LDO3);
    }
}

void PMUManager::processEvents(void (*buttonPressCallback)(void)) {
    if (!pmu_) {
        return;
    }
    if (!pmuInterrupt_) {
        return;
    }

    pmuInterrupt_ = false;
    uint32_t status = pmu_->getIrqStatus();
    LOGD(TAG, "STATUS => HEX: %X BIN: %s", status, String(status, BIN).c_str());

    if (pmu_->isVbusInsertIrq()) {
        LOGI(TAG, "isVbusInsert");
    }
    if (pmu_->isVbusRemoveIrq()) {
        LOGI(TAG, "isVbusRemove");
    }
    if (pmu_->isBatInsertIrq()) {
        LOGI(TAG, "isBatInsert");
    }
    if (pmu_->isBatRemoveIrq()) {
        LOGI(TAG, "isBatRemove");
    }
    if (pmu_->isPekeyShortPressIrq()) {
        LOGI(TAG, "isPekeyShortPress");
        if (buttonPressCallback) {
            buttonPressCallback();
        }
    }
    if (pmu_->isPekeyLongPressIrq()) {
        LOGI(TAG, "isPekeyLongPress");
    }
    if (pmu_->isBatChargeDoneIrq()) {
        LOGI(TAG, "isBatChargeDone");
    }
    if (pmu_->isBatChargeStartIrq()) {
        LOGI(TAG, "isBatChargeStart");
    }
    pmu_->clearIrqStatus();
}

const char* PMUManager::getChipModel() const {
    if (!pmu_) return "Unknown";

    switch (pmu_->getChipModel()) {
        case XPOWERS_AXP192:
            return "AXP192";
        case XPOWERS_AXP2101:
            return "AXP2101";
        default:
            return "Unknown";
    }
}

bool PMUManager::hasBattery() const {
    return pmu_ && pmu_->isBatteryConnect();
}

bool PMUManager::isCharging() const {
    return pmu_ && pmu_->isCharging();
}

bool PMUManager::isVbusConnected() const {
    return pmu_ && pmu_->isVbusIn();
}

uint16_t PMUManager::getBatteryVoltage() const {
    return pmu_ ? pmu_->getBattVoltage() : 0;
}

uint16_t PMUManager::getVbusVoltage() const {
    return pmu_ ? pmu_->getVbusVoltage() : 0;
}

uint16_t PMUManager::getSystemVoltage() const {
    return pmu_ ? pmu_->getSystemVoltage() : 0;
}

#endif // HAS_PMU
