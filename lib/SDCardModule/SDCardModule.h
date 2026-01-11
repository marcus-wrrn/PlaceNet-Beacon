#pragma once

#include "config.h"

#ifdef HAS_SDCARD

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

#define SD_MOUNT_POINT "/sd"
#define SD_MAX_FILES 10

class SDCardModule {
public:
    SDCardModule();
    ~SDCardModule();

    SDCardModule(const SDCardModule&) = delete;
    SDCardModule& operator=(const SDCardModule&) = delete;

    bool init();
    bool isInitialized() const { return initialized_; }

    bool fileExists(const char* path);
    size_t getFileSize(const char* path);
    File openFile(const char* path);
    void listDirectory(const char* path);

    uint64_t getCardSize();
    uint8_t getCardType();

private:
    bool initialized_;
    uint8_t cardType_;
    uint64_t cardSize_;
};

#endif // HAS_SDCARD
