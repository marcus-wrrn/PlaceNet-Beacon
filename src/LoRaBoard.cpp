/**
 * @file      LoRaBoard.cpp
 * @brief     Board initialization and peripheral management for T-Beam Supreme
 */

#include "LoRaBoard.h"

#include "soc/rtc.h"

#ifdef ENABLE_BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#endif

// Constructor
LoRaBoardManager::LoRaBoardManager()
    : deviceOnline_(0)
    , initialized_(false)
    , displayDisabled_(false)
    #ifdef HAS_PMU
    , pmu_(nullptr)
    , pmuInterrupt_(false)
    #endif
    #ifdef DISPLAY_MODEL
    , u8g2_(nullptr)
    #endif
    #ifdef HAS_GPS
    , gpsFound_(false)
    #endif
    #ifdef HAS_SDCARD
    , sdCardSpi_(HSPI)
    #endif
{}

// Destructor
LoRaBoardManager::~LoRaBoardManager() {
    #ifdef HAS_PMU
    if (pmu_) {
        disablePeripherals();
        delete pmu_;
        pmu_ = nullptr;
    }
    #endif

    #ifdef DISPLAY_MODEL
    if (u8g2_) {
        delete u8g2_;
        u8g2_ = nullptr;
    }
    #endif
}

LoRaBoardManager& LoRaBoardManager::getInstance() {
    static LoRaBoardManager instance;
    return instance;
}

#ifdef HAS_PMU
void LoRaBoardManager::pmuInterruptHandler() {
    getInstance().handlePMUInterrupt();
}

void LoRaBoardManager::handlePMUInterrupt() {
    pmuInterrupt_ = true;
}

bool LoRaBoardManager::initializePower() {
    if (!pmu_) {
        pmu_ = new XPowersAXP2101(PMU_WIRE_PORT);
        if (!pmu_->init()) {
            Serial.println("Warning: Failed to find AXP2101 power management");
            delete pmu_;
            pmu_ = NULL;
        } else {
            Serial.println("AXP2101 PMU init succeeded, using AXP2101 PMU");
        }
    }

    if (!pmu_) {
        pmu_ = new XPowersAXP192(PMU_WIRE_PORT);
        if (!pmu_->init()) {
            Serial.println("Warning: Failed to find AXP192 power management");
            delete pmu_;
            pmu_ = NULL;
        } else {
            Serial.println("AXP192 PMU init succeeded, using AXP192 PMU");
        }
    }

    if (!pmu_) {
        return false;
    }

    deviceOnline_ |= POWERMANAGE_ONLINE;

    pmu_->setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, pmuInterruptHandler, FALLING);

    if (pmu_->getChipModel() == XPOWERS_AXP192) {

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

    } else if (pmu_->getChipModel() == XPOWERS_AXP2101) {

        // T-Beam S3 Supreme specific configuration

        // In order to avoid bus occupation, during initialization, the SD card and QMC sensor are powered off and restarted
        if (ESP_SLEEP_WAKEUP_UNDEFINED == esp_sleep_get_wakeup_cause()) {
            Serial.println("Power off and restart ALDO BLDO..");
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

    pmu_->enableSystemVoltageMeasure();
    pmu_->enableVbusVoltageMeasure();
    pmu_->enableBattVoltageMeasure();

    Serial.printf("=========================================\n");
    if (pmu_->isChannelAvailable(XPOWERS_DCDC1)) {
        Serial.printf("DC1  : %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_DCDC1)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_DCDC1));
    }
    if (pmu_->isChannelAvailable(XPOWERS_DCDC2)) {
        Serial.printf("DC2  : %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_DCDC2)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_DCDC2));
    }
    if (pmu_->isChannelAvailable(XPOWERS_DCDC3)) {
        Serial.printf("DC3  : %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_DCDC3)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_DCDC3));
    }
    if (pmu_->isChannelAvailable(XPOWERS_DCDC4)) {
        Serial.printf("DC4  : %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_DCDC4)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_DCDC4));
    }
    if (pmu_->isChannelAvailable(XPOWERS_DCDC5)) {
        Serial.printf("DC5  : %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_DCDC5)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_DCDC5));
    }
    if (pmu_->isChannelAvailable(XPOWERS_LDO2)) {
        Serial.printf("LDO2 : %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_LDO2)   ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_LDO2));
    }
    if (pmu_->isChannelAvailable(XPOWERS_LDO3)) {
        Serial.printf("LDO3 : %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_LDO3)   ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_LDO3));
    }
    if (pmu_->isChannelAvailable(XPOWERS_ALDO1)) {
        Serial.printf("ALDO1: %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_ALDO1)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_ALDO1));
    }
    if (pmu_->isChannelAvailable(XPOWERS_ALDO2)) {
        Serial.printf("ALDO2: %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_ALDO2)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_ALDO2));
    }
    if (pmu_->isChannelAvailable(XPOWERS_ALDO3)) {
        Serial.printf("ALDO3: %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_ALDO3)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_ALDO3));
    }
    if (pmu_->isChannelAvailable(XPOWERS_ALDO4)) {
        Serial.printf("ALDO4: %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_ALDO4)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_ALDO4));
    }
    if (pmu_->isChannelAvailable(XPOWERS_BLDO1)) {
        Serial.printf("BLDO1: %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_BLDO1)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_BLDO1));
    }
    if (pmu_->isChannelAvailable(XPOWERS_BLDO2)) {
        Serial.printf("BLDO2: %s   Voltage: %04u mV \n",  pmu_->isPowerChannelEnable(XPOWERS_BLDO2)  ? "+" : "-",  pmu_->getPowerChannelVoltage(XPOWERS_BLDO2));
    }
    Serial.printf("=========================================\n");

    // Set the time of pressing the button to turn off
    pmu_->setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    uint8_t opt = pmu_->getPowerKeyPressOffTime();
    Serial.print("PowerKeyPressOffTime:");
    switch (opt) {
    case XPOWERS_POWEROFF_4S: Serial.println("4 Second");
        break;
    case XPOWERS_POWEROFF_6S: Serial.println("6 Second");
        break;
    case XPOWERS_POWEROFF_8S: Serial.println("8 Second");
        break;
    case XPOWERS_POWEROFF_10S: Serial.println("10 Second");
        break;
    default:
        break;
    }
    return true;
}

void LoRaBoardManager::disablePeripherals()
{
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
        // Clear the PMU interrupt status before sleeping
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
        // Disable all PMU interrupts
        pmu_->disableIRQ(XPOWERS_AXP192_ALL_IRQ);
        // Clear the PMU interrupt status
        pmu_->clearIrqStatus();
        // LoRa VDD
        pmu_->disablePowerOutput(XPOWERS_LDO2);
        // GNSS VDD
        pmu_->disablePowerOutput(XPOWERS_LDO3);
    }
}

void LoRaBoardManager::processPMUEvents(void (*pressed_cb)(void))
{
    if (!pmu_) {
        return;
    }
    if (!pmuInterrupt_) {
        return;
    }

    pmuInterrupt_ = false;
    // Get PMU Interrupt Status Register
    uint32_t status = pmu_->getIrqStatus();
    Serial.print("STATUS => HEX:");
    Serial.print(status, HEX);
    Serial.print(" BIN:");
    Serial.println(status, BIN);

    if (pmu_->isVbusInsertIrq()) {
        Serial.println("isVbusInsert");
    }
    if (pmu_->isVbusRemoveIrq()) {
        Serial.println("isVbusRemove");
    }
    if (pmu_->isBatInsertIrq()) {
        Serial.println("isBatInsert");
    }
    if (pmu_->isBatRemoveIrq()) {
        Serial.println("isBatRemove");
    }
    if (pmu_->isPekeyShortPressIrq()) {
        Serial.println("isPekeyShortPress");
        if (pressed_cb) {
            pressed_cb();
        }
    }
    if (pmu_->isPekeyLongPressIrq()) {
        Serial.println("isPekeyLongPress");
    }
    if (pmu_->isBatChargeDoneIrq()) {
        Serial.println("isBatChargeDone");
    }
    if (pmu_->isBatChargeStartIrq()) {
        Serial.println("isBatChargeStart");
    }
    // Clear PMU Interrupt Status Register
    pmu_->clearIrqStatus();
}
#endif

#ifdef DISPLAY_ADDR
bool LoRaBoardManager::initializeDisplay()
{
    Wire.beginTransmission(DISPLAY_ADDR);
    if (Wire.endTransmission() == 0) {
        Serial.printf("Find Display model at 0x%X address\n", DISPLAY_ADDR);
        u8g2_ = new DISPLAY_MODEL(U8G2_R0, U8X8_PIN_NONE);
        u8g2_->begin();
        u8g2_->clearBuffer();
        u8g2_->setFont(u8g2_font_inb19_mr);
        u8g2_->drawStr(0, 30, "LilyGo");
        u8g2_->drawHLine(2, 35, 47);
        u8g2_->drawHLine(3, 36, 47);
        u8g2_->drawVLine(45, 32, 12);
        u8g2_->drawVLine(46, 33, 12);
        u8g2_->setFont(u8g2_font_inb19_mf);
        u8g2_->drawStr(58, 60, "LoRa");
        u8g2_->sendBuffer();
        u8g2_->setFont(u8g2_font_fur11_tf);
        delay(3000);
        return true;
    }

    Serial.printf("Warning: Failed to find Display at 0x%0X address\n", DISPLAY_ADDR);
    return false;
}
#endif

#ifdef HAS_SDCARD
bool LoRaBoardManager::writeFile(const char *path, const char *buffer)
{
    bool rlst = false;
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return false;
    }
    if (file.print(buffer)) {
        Serial.println("File written");
        rlst = true;
    } else {
        Serial.println("Write failed");
        rlst = false;
    }
    file.close();
    return rlst;
}

bool LoRaBoardManager::readFile(const char *path, uint8_t *buffer, size_t size)
{
    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return false;
    }
    file.read(buffer, size);
    file.close();
    return true;
}

bool LoRaBoardManager::testSDCardReadWrite()
{
    const char *path = "/test_sd.txt";
    const char *message = "This is a string for reading and writing SD card.";
    uint8_t buffer[128] = {0};

    if (!writeFile(path, message)) {
        Serial.println("SD Text write failed");
        return false;
    }
    delay(100);

    readFile(path, buffer, 128);

    if (memcmp(buffer, message, strlen(message)) != 0) {
        Serial.println("SD verification failed");
        return false;
    }
    Serial.println("SD verification successful");
    return true;
}

bool LoRaBoardManager::initializeSDCard()
{
    bool rlst = SD.begin(SDCARD_CS, sdCardSpi_);

    if (rlst) {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.print("Sd Card init succeeded, The current available capacity is ");
        Serial.print(cardSize / 1024.0);
        Serial.println(" GB");
        deviceOnline_ |= SDCARD_ONLINE;
        return testSDCardReadWrite();
    } else {
        Serial.println("Warning: Failed to init Sd Card");
    }
    return false;
}
#endif

void LoRaBoardManager::printWakeupReason()
{
    Serial.print("Reset reason:");
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        Serial.println(" In case of deep sleep, reset was not caused by exit from deep sleep");
        break;
    case ESP_SLEEP_WAKEUP_ALL:
        break;
    case ESP_SLEEP_WAKEUP_EXT0:
        Serial.println("Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        Serial.println("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        Serial.println("Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        Serial.println("Wakeup caused by ULP program");
        break;
    default:
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }
}

void LoRaBoardManager::getChipInfo()
{
    Serial.println("-----------------------------------");

    printWakeupReason();

    if (psramFound()) {
        uint32_t psram = ESP.getPsramSize();
        deviceInfo_.psramSize = psram / 1024.0 / 1024.0;
        Serial.printf("PSRAM is enable! PSRAM: %.2fMB\n", deviceInfo_.psramSize);
        deviceOnline_ |= PSRAM_ONLINE;
    } else {
        Serial.println("PSRAM is disable!");
        deviceInfo_.psramSize = 0;
    }

    Serial.print("Flash:");
    deviceInfo_.flashSize       = ESP.getFlashChipSize() / 1024.0 / 1024.0;
    deviceInfo_.flashSpeed      = ESP.getFlashChipSpeed() / 1000 / 1000;
    deviceInfo_.chipModel       = ESP.getChipModel();
    deviceInfo_.chipModelRev    = ESP.getChipRevision();
    deviceInfo_.chipFreq        = ESP.getCpuFreqMHz();

    Serial.print(deviceInfo_.flashSize);
    Serial.println(" MB");
    Serial.print("Flash speed:");
    Serial.print(deviceInfo_.flashSpeed);
    Serial.println(" M");
    Serial.print("Model:");
    Serial.println(deviceInfo_.chipModel);
    Serial.print("Chip Revision:");
    Serial.println(deviceInfo_.chipModelRev);
    Serial.print("Freq:");
    Serial.print(deviceInfo_.chipFreq);
    Serial.println(" MHZ");
    Serial.print("SDK Ver:");
    Serial.println(ESP.getSdkVersion());
    Serial.print("DATE:");
    Serial.println(__DATE__);
    Serial.print("TIME:");
    Serial.println(__TIME__);

    uint8_t mac[6];
    char macStr[18] = {0};
    esp_efuse_mac_get_default(mac);
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("EFUSE MAC: ");
    Serial.print(macStr);
    Serial.println();

    Serial.println("-----------------------------------");
}

bool LoRaBoardManager::initialize(bool disableDisplay)
{
    // Prevent re-initialization
    if (initialized_) {
        Serial.println("Warning: LoRaBoardManager already initialized");
        return true;
    }

    displayDisabled_ = disableDisplay;

    getChipInfo();

    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);

#ifdef HAS_SDCARD
    sdCardSpi_.begin(SDCARD_SCLK, SDCARD_MISO, SDCARD_MOSI);
#endif

#ifdef I2C1_SDA
    Wire1.begin(I2C1_SDA, I2C1_SCL);
    Serial.println("Scan Wire1...");
    scanI2CDevices(&Wire1);
#endif

#ifdef HAS_GPS
#ifdef GPS_EN_PIN
    pinMode(GPS_EN_PIN, OUTPUT);
    digitalWrite(GPS_EN_PIN, HIGH);
#endif

#ifdef GPS_PPS_PIN
    pinMode(GPS_PPS_PIN, INPUT);
#endif

    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#endif

    initializePower();

    // Perform an I2C scan after power-on operation
#ifdef I2C_SDA
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("Scan Wire...");
    scanI2CDevices(&Wire);
#endif

    initializeSDCard();

#ifdef DISPLAY_ADDR
    if (!disableDisplay) {
        initializeDisplay();
    }
#endif

#ifdef HAS_GPS
    gpsFound_ = initializeGPS();
    uint32_t baudrate[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 4800};
    if (!gpsFound_) {
        // Restore factory settings
        for (int i = 0; i < sizeof(baudrate) / sizeof(baudrate[0]); ++i) {
            Serial.printf("Update baudrate : %u\n", baudrate[i]);
            SerialGPS.updateBaudRate(baudrate[i]);
            if (recoveryGPS()) {
                Serial.println("UBlox GNSS init succeeded, using UBlox GNSS Module\n");
                gpsModel_ = "UBlox";
                gpsFound_ = true;
                break;
            }
        }
    } else {
        gpsModel_ = "L76K";
    }

    if (gpsFound_) {
        deviceOnline_ |= GPS_ONLINE;
    }

    // Enable 32.768KHz crystal for RTC
    enableSlowClock();
#endif

    initialized_ = true;
    Serial.println("Board initialization complete");
    return true;
}

void LoRaBoardManager::printDeviceStatus(bool radio_online)
{
    Serial.print("Radio        : ");
    Serial.println((radio_online) ? "+" : "-");

    Serial.print("PSRAM        : ");
    Serial.println((psramFound()) ? "+" : "-");

#ifdef DISPLAY_MODEL
    Serial.print("Display      : ");
    Serial.println((u8g2_) ? "+" : "-");
#endif

#ifdef HAS_SDCARD
    Serial.print("Sd Card      : ");
    Serial.println((SD.cardSize() != 0) ? "+" : "-");
#endif

#ifdef HAS_PMU
    Serial.print("Power        : ");
    Serial.println((pmu_) ? "+" : "-");
#endif

#ifdef HAS_GPS
    Serial.print("GPS          : ");
    Serial.println((gpsFound_) ? "+" : "-");
#endif

#ifdef DISPLAY_MODEL
    if (u8g2_) {
        u8g2_->clearBuffer();
        u8g2_->setFont(u8g2_font_NokiaLargeBold_tf);
        uint16_t str_w = u8g2_->getStrWidth(BOARD_VARIANT_NAME);
        u8g2_->drawStr((u8g2_->getWidth() - str_w) / 2, 16, BOARD_VARIANT_NAME);
        u8g2_->drawHLine(5, 21, u8g2_->getWidth() - 5);

        u8g2_->drawStr(0, 38, "Disp:");     u8g2_->drawStr(45, 38, (u8g2_) ? "+" : "-");

#ifdef HAS_SDCARD
        u8g2_->drawStr(0, 54, "SD :");      u8g2_->drawStr(45, 54, (SD.cardSize() != 0) ? "+" : "-");
#endif

        u8g2_->drawStr(62, 38, "Radio:");    u8g2_->drawStr(120, 38, (radio_online) ? "+" : "-");

#ifdef HAS_PMU
        u8g2_->drawStr(62, 54, "Power:");    u8g2_->drawStr(120, 54, (pmu_) ? "+" : "-");
#endif

        u8g2_->sendBuffer();

        delay(2000);
    }
#endif
}

void LoRaBoardManager::scanI2CDevices(TwoWire *w)
{
    uint8_t err, addr;
    int nDevices = 0;
    uint32_t start = 0;

    Serial.println("I2C Devices scanning");
    for (addr = 1; addr < 127; addr++) {
        start = millis();
        w->beginTransmission(addr);
        delay(2);
        err = w->endTransmission();
        if (err == 0) {
            nDevices++;
            switch (addr) {
            case 0x77:
            case 0x76:
                Serial.println("\tFind BMX280 Sensor!");
                deviceOnline_ |= BME280_ONLINE;
                break;
            case 0x34:
                Serial.println("\tFind AXP192/AXP2101 PMU!");
                deviceOnline_ |= POWERMANAGE_ONLINE;
                break;
            case 0x3C:
                Serial.println("\tFind SSD1306/SH1106 dispaly!");
                deviceOnline_ |= DISPLAY_ONLINE;
                break;
            case 0x51:
                Serial.println("\tFind PCF8563 RTC!");
                deviceOnline_ |= PCF8563_ONLINE;
                break;
            case 0x1C:
                Serial.println("\tFind QMC6310 MAG Sensor!");
                deviceOnline_ |= QMC6310_ONLINE;
                break;
            default:
                Serial.print("\tI2C device found at address 0x");
                if (addr < 16) {
                    Serial.print("0");
                }
                Serial.print(addr, HEX);
                Serial.println(" !");
                break;
            }
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16) {
                Serial.print("0");
            }
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");

    Serial.println("Scan devices done.");
    Serial.println("\n");
}

#ifdef HAS_GPS

bool LoRaBoardManager::probeL76K()
{
    bool result = false;
    uint32_t startTimeout;
    SerialGPS.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
    delay(5);
    // Get version information
    startTimeout = millis() + 3000;
    Serial.print("Try to init L76K . Wait stop .");
    while (SerialGPS.available()) {
        int c = SerialGPS.read();
        if (millis() > startTimeout) {
            Serial.println("Wait L76K stop NMEA timeout!");
            return false;
        }
    };
    Serial.println();
    SerialGPS.flush();
    delay(200);

    SerialGPS.write("$PCAS06,0*1B\r\n");
    startTimeout = millis() + 500;
    String ver = "";
    while (!SerialGPS.available()) {
        if (millis() > startTimeout) {
            Serial.println("Get L76K timeout!");
            return false;
        }
    }
    SerialGPS.setTimeout(10);
    ver = SerialGPS.readStringUntil('\n');
    if (ver.startsWith("$GPTXT,01,01,02")) {
        Serial.println("L76K GNSS init succeeded, using L76K GNSS Module\n");
        result = true;
    }
    delay(500);

    // Initialize the L76K Chip, use GPS + GLONASS
    SerialGPS.write("$PCAS04,5*1C\r\n");
    delay(250);
    // only ask for RMC and GGA
    SerialGPS.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
    delay(250);
    // Switch to Vehicle Mode
    SerialGPS.write("$PCAS11,3*1E\r\n");
    return result;
}

bool LoRaBoardManager::initializeGPS()
{
    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    bool result = false;
    for (int i = 0; i < 3; ++i) {
        result = probeL76K();
        if (result) {
            return result;
        }
    }
    return result;
}

int LoRaBoardManager::getGPSAck(uint8_t *buffer, uint16_t size, uint8_t requestedClass, uint8_t requestedID)
{
    uint16_t ubxFrameCounter = 0;
    bool ubxFrame = 0;
    uint32_t startTime = millis();
    uint16_t needRead;

    while (millis() - startTime < 800) {
        while (SerialGPS.available()) {
            int c = SerialGPS.read();
            switch (ubxFrameCounter) {
            case 0:
                if (c == 0xB5) {
                    ubxFrameCounter++;
                }
                break;
            case 1:
                if (c == 0x62) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 2:
                if (c == requestedClass) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 3:
                if (c == requestedID) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 4:
                needRead = c;
                ubxFrameCounter++;
                break;
            case 5:
                needRead |= (c << 8);
                ubxFrameCounter++;
                break;
            case 6:
                if (needRead >= size) {
                    ubxFrameCounter = 0;
                    break;
                }
                if (SerialGPS.readBytes(buffer, needRead) != needRead) {
                    ubxFrameCounter = 0;
                } else {
                    return needRead;
                }
                break;

            default:
                break;
            }
        }
    }
    return 0;
}

bool LoRaBoardManager::recoveryGPS()
{
    uint8_t buffer[256];
    uint8_t cfg_clear1[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1C, 0xA2};
    uint8_t cfg_clear2[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA1};
    uint8_t cfg_clear3[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1D, 0xB3};
    SerialGPS.write(cfg_clear1, sizeof(cfg_clear1));

    if (getGPSAck(buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }
    SerialGPS.write(cfg_clear2, sizeof(cfg_clear2));
    if (getGPSAck(buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }
    SerialGPS.write(cfg_clear3, sizeof(cfg_clear3));
    if (getGPSAck(buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }
    // UBX-CFG-RATE, Size 8, 'Navigation/measurement rate settings'
    uint8_t cfg_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};
    SerialGPS.write(cfg_rate, sizeof(cfg_rate));
    if (getGPSAck(buffer, 256, 0x06, 0x08)) {
        Serial.println("Get ack successes!");
    } else {
        return false;
    }
    return true;
}

#endif

#ifdef ENABLE_BLE

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void LoRaBoardManager::initializeBLE()
{
    uint8_t mac[6];
    char macStr[18] = {0};
    esp_efuse_mac_get_default(mac);
    sprintf(macStr, "%02X:%02X", mac[0], mac[1]);

    String dev = BOARD_VARIANT_NAME;
    dev.concat('-');
    dev.concat(macStr);

    Serial.print("Starting BLE:");
    Serial.println(dev);

    BLEDevice::init(dev.c_str());
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);

    pCharacteristic->setValue("Hello World");
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Characteristic defined! Now you can read it in your phone!");
}
#endif

#define CALIBRATE_ONE(cali_clk) calibrateRTC(cali_clk, #cali_clk)

uint32_t LoRaBoardManager::calibrateRTC(rtc_cal_sel_t cal_clk, const char *name)
{
    const uint32_t cal_count = 1000;
    uint32_t cali_val;
    for (int i = 0; i < 5; ++i) {
        cali_val = rtc_clk_cal(cal_clk, cal_count);
    }
    return cali_val;
}

void LoRaBoardManager::enableSlowClock()
{
    rtc_clk_32k_enable(true);
    CALIBRATE_ONE(RTC_CAL_RTC_MUX);
    uint32_t cal_32k = CALIBRATE_ONE(RTC_CAL_32K_XTAL);
    if (cal_32k == 0) {
        Serial.printf("32K XTAL OSC has not started up");
    } else {
        rtc_clk_slow_freq_set(RTC_SLOW_FREQ_32K_XTAL);
        Serial.println("Switching RTC Source to 32.768Khz succeeded, using 32K XTAL");
        CALIBRATE_ONE(RTC_CAL_RTC_MUX);
        CALIBRATE_ONE(RTC_CAL_32K_XTAL);
    }
    CALIBRATE_ONE(RTC_CAL_RTC_MUX);
    CALIBRATE_ONE(RTC_CAL_32K_XTAL);
    if (rtc_clk_slow_freq_get() != RTC_SLOW_FREQ_32K_XTAL) {
        Serial.println("Warning: Failed to set rtc clk to 32.768Khz !!!");
        return;
    }
    deviceOnline_ |= OSC32768_ONLINE;
}

void LoRaBoardManager::scanWiFiNetworks()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.println("WiFi Scan start");
    int n = WiFi.scanNetworks();
    Serial.println("WiFi Scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        Serial.println("Nr | SSID                             | RSSI | CH | Encryption");
        for (int i = 0; i < n; ++i) {
            Serial.printf("%2d", i + 1);
            Serial.print(" | ");
            Serial.printf("%-32.32s", WiFi.SSID(i).c_str());
            Serial.print(" | ");
            Serial.printf("%4ld", WiFi.RSSI(i));
            Serial.print(" | ");
            Serial.printf("%2ld", WiFi.channel(i));
            Serial.print(" | ");
            switch (WiFi.encryptionType(i)) {
            case WIFI_AUTH_OPEN:            Serial.print("open"); break;
            case WIFI_AUTH_WEP:             Serial.print("WEP"); break;
            case WIFI_AUTH_WPA_PSK:         Serial.print("WPA"); break;
            case WIFI_AUTH_WPA2_PSK:        Serial.print("WPA2"); break;
            case WIFI_AUTH_WPA_WPA2_PSK:    Serial.print("WPA+WPA2"); break;
            case WIFI_AUTH_WPA2_ENTERPRISE: Serial.print("WPA2-EAP"); break;
            case WIFI_AUTH_WPA3_PSK:        Serial.print("WPA3"); break;
            case WIFI_AUTH_WPA2_WPA3_PSK:   Serial.print("WPA2+WPA3"); break;
            case WIFI_AUTH_WAPI_PSK:        Serial.print("WAPI"); break;
            default:                        Serial.print("unknown");
            }
            Serial.println();
            delay(10);
        }
    }
    Serial.println("");

    WiFi.scanDelete();
}
