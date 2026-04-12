#pragma once

#include "config.h"

#ifdef HAS_SDCARD

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "PlaceNetConfig.h"

#define SD_MOUNT_POINT "/sd"
#define SD_MAX_FILES 10
#define CONFIG_FILE_PATH "/config.json"
#define MQTT_BROKER_FILE_PATH "/mqtt_broker.json"
#define DEVICE_CERT_FILE_PATH "/device_cert.pem"

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
    bool writeFile(const char* path, const char* content);
    bool readFile(const char* path, char* buffer, size_t bufferSize);
    bool deleteFile(const char* path);

    bool loadConfig(PlaceNetConfig* config);
    bool saveConfig(const PlaceNetConfig* config);
    bool configExists();

    bool saveMQTTBroker(const MQTTBrokerInfo* broker);
    bool loadMQTTBroker(MQTTBrokerInfo* broker);
    bool mqttBrokerExists();

    uint64_t getCardSize();
    uint8_t getCardType();

private:
    bool initialized_;
    uint8_t cardType_;
    uint64_t cardSize_;
#if defined(T_BEAM_S3_SUPREME_SX1262) || defined(T_BEAM_S3_SUPREME_LR1121)
    SPIClass sdSPI_;
#endif

    bool parseConfigJSON(const char* json, PlaceNetConfig* config);
    bool createConfigJSON(const PlaceNetConfig* config, char* buffer, size_t bufferSize);
};

#endif // HAS_SDCARD
