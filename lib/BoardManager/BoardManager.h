#pragma once

#include "config.h"

#ifdef HAS_PMU
#include "PMUManager.h"
#endif

#ifdef HAS_SDCARD
#include <SD.h>
#endif

#if defined(ARDUINO_ARCH_ESP32)
#include <FS.h>
#include <WiFi.h>
#endif

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#ifdef DISPLAY_MODEL
#include <U8g2lib.h>
#endif

#ifdef HAS_PMU
#include <XPowersLib.h>
#endif

#include <esp_mac.h>
#include "soc/rtc.h"

#ifdef _BIT_SHIFT
#undef _BIT_SHIFT
#endif
#define _BIT_SHIFT(b)  (1ULL << (uint64_t)(b))

enum {
    POWERMANAGE_ONLINE  = _BIT_SHIFT(0),
    DISPLAY_ONLINE      = _BIT_SHIFT(1),
    RADIO_ONLINE        = _BIT_SHIFT(2),
    GPS_ONLINE          = _BIT_SHIFT(3),
    PSRAM_ONLINE        = _BIT_SHIFT(4),
    SDCARD_ONLINE       = _BIT_SHIFT(5),
    AXDL345_ONLINE      = _BIT_SHIFT(6),
    BME280_ONLINE       = _BIT_SHIFT(7),
    BMP280_ONLINE       = _BIT_SHIFT(8),
    BME680_ONLINE       = _BIT_SHIFT(9),
    QMC6310_ONLINE      = _BIT_SHIFT(10),
    QMI8658_ONLINE      = _BIT_SHIFT(11),
    PCF8563_ONLINE      = _BIT_SHIFT(12),
    OSC32768_ONLINE     = _BIT_SHIFT(13),
};

typedef struct {
    String          chipModel;
    float           psramSize;
    uint8_t         chipModelRev;
    uint8_t         chipFreq;
    uint8_t         flashSize;
    uint8_t         flashSpeed;
} DevInfo_t;

#ifdef HAS_GPS
#define SerialGPS Serial1
#endif

class LoRaBoardManager {
public:
    // Singleton pattern
    static LoRaBoardManager& getInstance();
    LoRaBoardManager(const LoRaBoardManager&) = delete;
    LoRaBoardManager& operator=(const LoRaBoardManager&) = delete;
    LoRaBoardManager(LoRaBoardManager&&) = delete;
    LoRaBoardManager& operator=(LoRaBoardManager&&) = delete;

    // Core initialization
    bool initialize(bool disableDisplay = false);
    bool isInitialized() const { return initialized_; }

    // Subsystem initialization

    #ifdef DISPLAY_ADDR
    bool initializeDisplay();
    #endif

    #ifdef HAS_SDCARD
    bool initializeSDCard();
    #endif

    #ifdef HAS_GPS
    bool initializeGPS();
    bool recoveryGPS();
    #endif

    // Runtime operations
    void printDeviceStatus(bool radioOnline);
    void scanI2CDevices(TwoWire* wire);
    void scanWiFiNetworks();

    #ifdef ENABLE_BLE
    void initializeBLE();
    #endif

    // Accessors for peripherals
    uint32_t getDeviceOnlineFlags() const { return deviceOnline_; }
    const DevInfo_t& getDeviceInfo() const { return deviceInfo_; }

    #ifdef HAS_PMU
    PMUManager* getPMUManager() { return pmuManager_; }
    const PMUManager* getPMUManager() const { return pmuManager_; }
    #endif

    #ifdef DISPLAY_MODEL
    DISPLAY_MODEL* getDisplay() { return u8g2_; }
    #endif

    #ifdef HAS_GPS
    bool isGPSFound() const { return gpsFound_; }
    const String& getGPSModel() const { return gpsModel_; }
    #endif

    #ifdef HAS_SDCARD
    SPIClass& getSDCardSPI() { return sdCardSpi_; }
    #endif

private:
    // Private constructor/destructor
    LoRaBoardManager();
    ~LoRaBoardManager();

    // Helper methods
    void getChipInfo();
    void printWakeupReason();

    #ifdef HAS_GPS
    bool probeL76K();
    static int getGPSAck(uint8_t* buffer, uint16_t size, uint8_t reqClass, uint8_t reqID);
    void enableSlowClock();
    static uint32_t calibrateRTC(rtc_cal_sel_t calClk, const char* name);
    #endif

    #ifdef HAS_SDCARD
    bool testSDCardReadWrite();
    static bool writeFile(const char* path, const char* buffer);
    static bool readFile(const char* path, uint8_t* buffer, size_t size);
    #endif

    // Member variables (organized by subsystem)
    uint32_t deviceOnline_;
    DevInfo_t deviceInfo_;
    bool initialized_;
    bool displayDisabled_;

    #ifdef HAS_PMU
    PMUManager* pmuManager_;
    #endif

    #ifdef DISPLAY_MODEL
    DISPLAY_MODEL* u8g2_;
    #endif

    #ifdef HAS_GPS
    bool gpsFound_;
    String gpsModel_;
    #endif

    #ifdef HAS_SDCARD
    SPIClass sdCardSpi_;
    #endif
};

#ifdef DISPLAY_MODEL
#define U8G2_HOR_ALIGN_CENTER(t)    ((LoRaBoardManager::getInstance().getDisplay()->getDisplayWidth() -  (LoRaBoardManager::getInstance().getDisplay()->getUTF8Width(t))) / 2)
#define U8G2_HOR_ALIGN_RIGHT(t)     (LoRaBoardManager::getInstance().getDisplay()->getDisplayWidth()  -  LoRaBoardManager::getInstance().getDisplay()->getUTF8Width(t))
#endif
