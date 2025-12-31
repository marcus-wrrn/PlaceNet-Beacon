/**
 * @file      LoRaBoard.cpp
 * @brief     Board initialization and peripheral management for T-Beam Supreme
 */

#include "BoardManager.h"
#include "logger.h"
#include "soc/rtc.h"
#define TAG "BOARD_MANAGER"
#ifdef ENABLE_BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#endif

LoRaBoardManager::LoRaBoardManager()
    : deviceOnline_(0)
    , initialized_(false)
    , displayDisabled_(false)
    #ifdef HAS_PMU
    , pmuManager_(nullptr)
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

LoRaBoardManager::~LoRaBoardManager() {
    #ifdef HAS_PMU
    if (pmuManager_) {
        pmuManager_->disablePeripherals();
        delete pmuManager_;
        pmuManager_ = nullptr;
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

#ifdef DISPLAY_ADDR
bool LoRaBoardManager::initializeDisplay() {
    Wire.beginTransmission(DISPLAY_ADDR);
    if (Wire.endTransmission() == 0) {
        LOGI(TAG, "Find Display model at 0x%X address", DISPLAY_ADDR);
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

    LOGW(TAG, "Failed to find Display at 0x%0X address", DISPLAY_ADDR);
    return false;
}
#endif

#ifdef HAS_SDCARD
bool LoRaBoardManager::writeFile(const char *path, const char *buffer) {
    bool rlst = false;
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        LOGE(TAG, "Failed to open file for writing");
        return false;
    }
    if (file.print(buffer)) {
        LOGD(TAG, "File written");
        rlst = true;
    } else {
        LOGE(TAG, "Write failed");
        rlst = false;
    }
    file.close();
    return rlst;
}

bool LoRaBoardManager::readFile(const char *path, uint8_t *buffer, size_t size) {
    File file = SD.open(path, FILE_READ);
    if (!file) {
        LOGE(TAG, "Failed to open file for reading");
        return false;
    }
    file.read(buffer, size);
    file.close();
    return true;
}

bool LoRaBoardManager::testSDCardReadWrite() {
    const char *path = "/test_sd.txt";
    const char *message = "This is a string for reading and writing SD card.";
    uint8_t buffer[128] = {0};

    if (!writeFile(path, message)) {
        LOGE(TAG, "SD Text write failed");
        return false;
    }
    delay(100);

    readFile(path, buffer, 128);

    if (memcmp(buffer, message, strlen(message)) != 0) {
        LOGE(TAG, "SD verification failed");
        return false;
    }
    LOGI(TAG, "SD verification successful");
    return true;
}

bool LoRaBoardManager::initializeSDCard() {
    bool rlst = SD.begin(SDCARD_CS, sdCardSpi_);

    if (rlst) {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        LOGI(TAG, "Sd Card init succeeded, The current available capacity is %.2f GB", cardSize / 1024.0);
        deviceOnline_ |= SDCARD_ONLINE;
        return testSDCardReadWrite();
    } else {
        LOGW(TAG, "Failed to init Sd Card");
    }
    return false;
}
#endif

void LoRaBoardManager::printWakeupReason() {
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        LOGI(TAG, "Reset reason: In case of deep sleep, reset was not caused by exit from deep sleep");
        break;
    case ESP_SLEEP_WAKEUP_ALL:
        break;
    case ESP_SLEEP_WAKEUP_EXT0:
        LOGI(TAG, "Reset reason: Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        LOGI(TAG, "Reset reason: Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        LOGI(TAG, "Reset reason: Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        LOGI(TAG, "Reset reason: Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        LOGI(TAG, "Reset reason: Wakeup caused by ULP program");
        break;
    default:
        LOGI(TAG, "Reset reason: Wakeup was not caused by deep sleep: %d", wakeup_reason);
        break;
    }
}

void LoRaBoardManager::getChipInfo() {
    LOGI(TAG, "-----------------------------------");

    printWakeupReason();

    if (psramFound()) {
        uint32_t psram = ESP.getPsramSize();
        deviceInfo_.psramSize = psram / 1024.0 / 1024.0;
        LOGI(TAG, "PSRAM is enable! PSRAM: %.2fMB", deviceInfo_.psramSize);
        deviceOnline_ |= PSRAM_ONLINE;
    } else {
        LOGI(TAG, "PSRAM is disable!");
        deviceInfo_.psramSize = 0;
    }

    deviceInfo_.flashSize       = ESP.getFlashChipSize() / 1024.0 / 1024.0;
    deviceInfo_.flashSpeed      = ESP.getFlashChipSpeed() / 1000 / 1000;
    deviceInfo_.chipModel       = ESP.getChipModel();
    deviceInfo_.chipModelRev    = ESP.getChipRevision();
    deviceInfo_.chipFreq        = ESP.getCpuFreqMHz();

    LOGI(TAG, "Flash: %.2f MB", deviceInfo_.flashSize);
    LOGI(TAG, "Flash speed: %d M", deviceInfo_.flashSpeed);
    LOGI(TAG, "Model: %s", deviceInfo_.chipModel);
    LOGI(TAG, "Chip Revision: %d", deviceInfo_.chipModelRev);
    LOGI(TAG, "Freq: %d MHZ", deviceInfo_.chipFreq);
    LOGI(TAG, "SDK Ver: %s", ESP.getSdkVersion());
    LOGI(TAG, "DATE: %s", __DATE__);
    LOGI(TAG, "TIME: %s", __TIME__);

    uint8_t mac[6];
    char macStr[18] = {0};
    esp_efuse_mac_get_default(mac);
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    LOGI(TAG, "EFUSE MAC: %s", macStr);

    LOGI(TAG, "-----------------------------------");
}

bool LoRaBoardManager::initialize(bool disableDisplay) {
    // Prevent re-initialization
    if (initialized_) {
        LOGW(TAG, "LoRaBoardManager already initialized");
        return true;
    }

    displayDisabled_ = disableDisplay;

    LOGI(TAG, "Initializing Board Manager");

    getChipInfo();

    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);

#ifdef HAS_SDCARD
    sdCardSpi_.begin(SDCARD_SCLK, SDCARD_MISO, SDCARD_MOSI);
#endif

#ifdef I2C1_SDA
    Wire1.begin(I2C1_SDA, I2C1_SCL);
    LOGI(TAG, "Scan Wire1...");
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

#ifdef HAS_PMU
    pmuManager_ = new PMUManager(PMU_WIRE_PORT, PMU_IRQ);
    if (pmuManager_->initialize()) {
        deviceOnline_ |= POWERMANAGE_ONLINE;
        LOGI(TAG, "PMU initialization successful");
    } else {
        delete pmuManager_;
        pmuManager_ = nullptr;
        LOGW(TAG, "PMU initialization failed");
    }
#endif

    // Perform an I2C scan after power-on operation
#ifdef I2C_SDA
    Wire.begin(I2C_SDA, I2C_SCL);
    LOGI(TAG, "Scan Wire...");
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
            LOGI(TAG, "Update baudrate : %u", baudrate[i]);
            SerialGPS.updateBaudRate(baudrate[i]);
            if (recoveryGPS()) {
                LOGI(TAG, "UBlox GNSS init succeeded, using UBlox GNSS Module");
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
    LOGI(TAG, "Board initialization complete");
    return true;
}

void LoRaBoardManager::printDeviceStatus(bool radio_online) {
    LOGI(TAG, "Radio        : %s", (radio_online) ? "+" : "-");

    LOGI(TAG, "PSRAM        : %s", (psramFound()) ? "+" : "-");

#ifdef DISPLAY_MODEL
    LOGI(TAG, "Display      : %s", (u8g2_) ? "+" : "-");
#endif

#ifdef HAS_SDCARD
    LOGI(TAG, "Sd Card      : %s", (SD.cardSize() != 0) ? "+" : "-");
#endif

#ifdef HAS_PMU
    LOGI(TAG, "Power        : %s", (pmuManager_) ? "+" : "-");
#endif

#ifdef HAS_GPS
    LOGI(TAG, "GPS          : %s", (gpsFound_) ? "+" : "-");
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
        u8g2_->drawStr(62, 54, "Power:");    u8g2_->drawStr(120, 54, (pmuManager_) ? "+" : "-");
#endif

        u8g2_->sendBuffer();

        delay(2000);
    }
#endif
}

void LoRaBoardManager::scanI2CDevices(TwoWire *w) {
    uint8_t err, addr;
    int nDevices = 0;
    uint32_t start = 0;

    LOGI(TAG, "I2C Devices scanning");
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
                LOGI(TAG, "  Find BMX280 Sensor!");
                deviceOnline_ |= BME280_ONLINE;
                break;
            case 0x34:
                LOGI(TAG, "  Find AXP192/AXP2101 PMU!");
                deviceOnline_ |= POWERMANAGE_ONLINE;
                break;
            case 0x3C:
                LOGI(TAG, "  Find SSD1306/SH1106 dispaly!");
                deviceOnline_ |= DISPLAY_ONLINE;
                break;
            case 0x51:
                LOGI(TAG, "  Find PCF8563 RTC!");
                deviceOnline_ |= PCF8563_ONLINE;
                break;
            case 0x1C:
                LOGI(TAG, "  Find QMC6310 MAG Sensor!");
                deviceOnline_ |= QMC6310_ONLINE;
                break;
            default:
                LOGI(TAG, "  I2C device found at address 0x%02X !", addr);
                break;
            }
        } else if (err == 4) {
            LOGE(TAG, "Unknown error at address 0x%02X", addr);
        }
    }
    if (nDevices == 0)
        LOGI(TAG, "No I2C devices found");

    LOGI(TAG, "Scan devices done.");
}

#ifdef HAS_GPS

bool LoRaBoardManager::probeL76K() {
    bool result = false;
    uint32_t startTimeout;
    SerialGPS.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
    delay(5);
    // Get version information
    startTimeout = millis() + 3000;
    LOGD(TAG, "Try to init L76K . Wait stop .");
    while (SerialGPS.available()) {
        int c = SerialGPS.read();
        if (millis() > startTimeout) {
            LOGW(TAG, "Wait L76K stop NMEA timeout!");
            return false;
        }
    };
    SerialGPS.flush();
    delay(200);

    SerialGPS.write("$PCAS06,0*1B\r\n");
    startTimeout = millis() + 500;
    String ver = "";
    while (!SerialGPS.available()) {
        if (millis() > startTimeout) {
            LOGW(TAG, "Get L76K timeout!");
            return false;
        }
    }
    SerialGPS.setTimeout(10);
    ver = SerialGPS.readStringUntil('\n');
    if (ver.startsWith("$GPTXT,01,01,02")) {
        LOGI(TAG, "L76K GNSS init succeeded, using L76K GNSS Module");
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

bool LoRaBoardManager::initializeGPS() {
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

int LoRaBoardManager::getGPSAck(uint8_t *buffer, uint16_t size, uint8_t requestedClass, uint8_t requestedID) {
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

bool LoRaBoardManager::recoveryGPS() {
    uint8_t buffer[256];
    uint8_t cfg_clear1[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1C, 0xA2};
    uint8_t cfg_clear2[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA1};
    uint8_t cfg_clear3[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1D, 0xB3};
    SerialGPS.write(cfg_clear1, sizeof(cfg_clear1));

    if (getGPSAck(buffer, 256, 0x05, 0x01)) {
        LOGD(TAG, "Get ack successes!");
    }
    SerialGPS.write(cfg_clear2, sizeof(cfg_clear2));
    if (getGPSAck(buffer, 256, 0x05, 0x01)) {
        LOGD(TAG, "Get ack successes!");
    }
    SerialGPS.write(cfg_clear3, sizeof(cfg_clear3));
    if (getGPSAck(buffer, 256, 0x05, 0x01)) {
        LOGD(TAG, "Get ack successes!");
    }
    // UBX-CFG-RATE, Size 8, 'Navigation/measurement rate settings'
    uint8_t cfg_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};
    SerialGPS.write(cfg_rate, sizeof(cfg_rate));
    if (getGPSAck(buffer, 256, 0x06, 0x08)) {
        LOGD(TAG, "Get ack successes!");
    } else {
        return false;
    }
    return true;
}

#endif

#ifdef ENABLE_BLE

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void LoRaBoardManager::initializeBLE() {
    uint8_t mac[6];
    char macStr[18] = {0};
    esp_efuse_mac_get_default(mac);
    sprintf(macStr, "%02X:%02X", mac[0], mac[1]);

    String dev = BOARD_VARIANT_NAME;
    dev.concat('-');
    dev.concat(macStr);

    LOGI(TAG, "Starting BLE: %s", dev.c_str());

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
    LOGI(TAG, "Characteristic defined! Now you can read it in your phone!");
}
#endif

#define CALIBRATE_ONE(cali_clk) calibrateRTC(cali_clk, #cali_clk)

uint32_t LoRaBoardManager::calibrateRTC(rtc_cal_sel_t cal_clk, const char *name) {
    const uint32_t cal_count = 1000;
    uint32_t cali_val;
    for (int i = 0; i < 5; ++i) {
        cali_val = rtc_clk_cal(cal_clk, cal_count);
    }
    return cali_val;
}

void LoRaBoardManager::enableSlowClock() {
    rtc_clk_32k_enable(true);
    CALIBRATE_ONE(RTC_CAL_RTC_MUX);
    uint32_t cal_32k = CALIBRATE_ONE(RTC_CAL_32K_XTAL);
    if (cal_32k == 0) {
        LOGW(TAG, "32K XTAL OSC has not started up");
    } else {
        rtc_clk_slow_freq_set(RTC_SLOW_FREQ_32K_XTAL);
        LOGI(TAG, "Switching RTC Source to 32.768Khz succeeded, using 32K XTAL");
        CALIBRATE_ONE(RTC_CAL_RTC_MUX);
        CALIBRATE_ONE(RTC_CAL_32K_XTAL);
    }
    CALIBRATE_ONE(RTC_CAL_RTC_MUX);
    CALIBRATE_ONE(RTC_CAL_32K_XTAL);
    if (rtc_clk_slow_freq_get() != RTC_SLOW_FREQ_32K_XTAL) {
        LOGW(TAG, "Failed to set rtc clk to 32.768Khz !!!");
        return;
    }
    deviceOnline_ |= OSC32768_ONLINE;
}

void LoRaBoardManager::scanWiFiNetworks() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    LOGI(TAG, "WiFi Scan start");
    int n = WiFi.scanNetworks();
    LOGI(TAG, "WiFi Scan done");
    if (n == 0) {
        LOGI(TAG, "no networks found");
    } else {
        LOGI(TAG, "%d networks found", n);
        LOGI(TAG, "Nr | SSID                             | RSSI | CH | Encryption");
        for (int i = 0; i < n; ++i) {
            const char* enc_type;
            switch (WiFi.encryptionType(i)) {
            case WIFI_AUTH_OPEN:            enc_type = "open"; break;
            case WIFI_AUTH_WEP:             enc_type = "WEP"; break;
            case WIFI_AUTH_WPA_PSK:         enc_type = "WPA"; break;
            case WIFI_AUTH_WPA2_PSK:        enc_type = "WPA2"; break;
            case WIFI_AUTH_WPA_WPA2_PSK:    enc_type = "WPA+WPA2"; break;
            case WIFI_AUTH_WPA2_ENTERPRISE: enc_type = "WPA2-EAP"; break;
            case WIFI_AUTH_WPA3_PSK:        enc_type = "WPA3"; break;
            case WIFI_AUTH_WPA2_WPA3_PSK:   enc_type = "WPA2+WPA3"; break;
            case WIFI_AUTH_WAPI_PSK:        enc_type = "WAPI"; break;
            default:                        enc_type = "unknown";
            }
            LOGI(TAG, "%2d | %-32.32s | %4ld | %2ld | %s",
                 i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i), enc_type);
            delay(10);
        }
    }

    WiFi.scanDelete();
}
