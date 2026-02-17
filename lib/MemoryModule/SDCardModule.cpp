#include "SDCardModule.h"

#ifdef HAS_SDCARD

#include "logger.h"
#include <ArduinoJson.h>

static const char* TAG = "SD_MODULE";

SDCardModule::SDCardModule()
    : initialized_(false)
    , cardType_(CARD_NONE)
    , cardSize_(0)
#if defined(T_BEAM_S3_SUPREME_SX1262) || defined(T_BEAM_S3_SUPREME_LR1121)
    , sdSPI_(HSPI)
#endif
{}

SDCardModule::~SDCardModule() {
    if (initialized_) {
        SD.end();
        initialized_ = false;
    }
}

bool SDCardModule::init() {
    if (initialized_) {
        LOGW(TAG, "SD card already initialized");
        return true;
    }

    LOGI(TAG, "Initializing SD card...");
    LOGI(TAG, "SD pins: MOSI=%d, MISO=%d, SCK=%d, CS=%d",
         SDCARD_MOSI, SDCARD_MISO, SDCARD_SCLK, SDCARD_CS);

#if defined(T_BEAM_S3_SUPREME_SX1262) || defined(T_BEAM_S3_SUPREME_LR1121)
    sdSPI_.begin(SDCARD_SCLK, SDCARD_MISO, SDCARD_MOSI, SDCARD_CS);

    if (!SD.begin(SDCARD_CS, sdSPI_, 4000000U, SD_MOUNT_POINT, SD_MAX_FILES)) {
        LOGE(TAG, "SD card mount failed");
        return false;
    }
#elif defined(T_DECK_SX1262)
    if (!SD.begin(SDCARD_CS, SPI, 4000000U, SD_MOUNT_POINT, SD_MAX_FILES)) {
        LOGE(TAG, "SD card mount failed");
        return false;
    }
#endif

    cardType_ = SD.cardType();
    if (cardType_ == CARD_NONE) {
        LOGE(TAG, "No SD card detected");
        return false;
    }

    cardSize_ = SD.cardSize();

    const char* typeStr = "UNKNOWN";
    switch (cardType_) {
        case CARD_MMC: typeStr = "MMC"; break;
        case CARD_SD: typeStr = "SD"; break;
        case CARD_SDHC: typeStr = "SDHC"; break;
    }

    LOGI(TAG, "SD card initialized successfully");
    LOGI(TAG, "Card Type: %s", typeStr);
    LOGI(TAG, "Card Size: %llu MB", cardSize_ / (1024 * 1024));

    initialized_ = true;
    return true;
}

bool SDCardModule::fileExists(const char* path) {
    if (!initialized_) {
        return false;
    }
    ESP_LOGI(TAG, "Checking that the file exists");
    return SD.exists(path);
}

size_t SDCardModule::getFileSize(const char* path) {
    if (!initialized_) {
        return 0;
    }

    File file = SD.open(path);
    if (!file) {
        return 0;
    }

    size_t size = file.size();
    file.close();
    return size;
}

File SDCardModule::openFile(const char* path) {
    if (!initialized_) {
        return File();
    }
    return SD.open(path);
}

void SDCardModule::listDirectory(const char* path) {
    if (!initialized_) {
        LOGW(TAG, "Cannot list directory - SD not initialized");
        return;
    }

    File root = SD.open(path);
    if (!root) {
        LOGE(TAG, "Failed to open directory: %s", path);
        return;
    }

    if (!root.isDirectory()) {
        LOGE(TAG, "Not a directory: %s", path);
        return;
    }

    LOGI(TAG, "Listing directory: %s", path);
    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            LOGI(TAG, "  DIR: %s", file.name());
        } else {
            LOGI(TAG, "  FILE: %s (%zu bytes)", file.name(), file.size());
        }
        file = root.openNextFile();
    }
}

uint64_t SDCardModule::getCardSize() {
    return cardSize_;
}

uint8_t SDCardModule::getCardType() {
    return cardType_;
}

bool SDCardModule::writeFile(const char* path, const char* content) {
    if (!initialized_) {
        LOGE(TAG, "SD card not initialized");
        return false;
    }

    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        LOGE(TAG, "Failed to open file for writing: %s", path);
        return false;
    }

    size_t written = file.print(content);
    file.close();

    if (written != strlen(content)) {
        LOGE(TAG, "Write incomplete: %zu/%zu bytes", written, strlen(content));
        return false;
    }

    LOGI(TAG, "Wrote %zu bytes to %s", written, path);
    return true;
}

bool SDCardModule::readFile(const char* path, char* buffer, size_t bufferSize) {
    if (!initialized_) {
        LOGE(TAG, "SD card not initialized");
        return false;
    }

    File file = SD.open(path);
    if (!file) {
        LOGE(TAG, "Failed to open file for reading: %s", path);
        return false;
    }

    size_t fileSize = file.size();
    if (fileSize >= bufferSize) {
        LOGE(TAG, "Buffer too small: need %zu, have %zu", fileSize + 1, bufferSize);
        file.close();
        return false;
    }

    size_t bytesRead = file.readBytes(buffer, fileSize);
    buffer[bytesRead] = '\0';
    file.close();

    LOGI(TAG, "Read %zu bytes from %s", bytesRead, path);
    return true;
}

bool SDCardModule::deleteFile(const char* path) {
    if (!initialized_) {
        LOGE(TAG, "SD card not initialized");
        return false;
    }

    if (!SD.remove(path)) {
        LOGE(TAG, "Failed to delete file: %s", path);
        return false;
    }

    LOGI(TAG, "Deleted file: %s", path);
    return true;
}

bool SDCardModule::configExists() {
    return fileExists(CONFIG_FILE_PATH);
}

bool SDCardModule::loadConfig(PlaceNetConfig* config) {
    if (!config) {
        LOGE(TAG, "Null config pointer");
        return false;
    }

    if (!initialized_) {
        LOGE(TAG, "SD card not initialized");
        return false;
    }

    if (!fileExists(CONFIG_FILE_PATH)) {
        LOGW(TAG, "Config file does not exist: %s", CONFIG_FILE_PATH);
        return false;
    }

    char buffer[4096];

    bool success = false;
    if (readFile(CONFIG_FILE_PATH, buffer, sizeof(buffer))) {
        success = parseConfigJSON(buffer, config);
        if (success) {
            LOGI(TAG, "Configuration loaded successfully");
            config->print();
        } else {
            LOGE(TAG, "Failed to parse config JSON");
        }
    }

    return success;
}

bool SDCardModule::saveConfig(const PlaceNetConfig* config) {
    if (!config) {
        LOGE(TAG, "Null config pointer");
        return false;
    }

    if (!initialized_) {
        LOGE(TAG, "SD card not initialized");
        return false;
    }

    if (!config->validate()) {
        LOGE(TAG, "Config validation failed");
        return false;
    }

    char buffer[4096];

    bool success = false;
    if (createConfigJSON(config, buffer, sizeof(buffer))) {
        success = writeFile(CONFIG_FILE_PATH, buffer);
        if (success) {
            LOGI(TAG, "Configuration saved successfully");
        }
    } else {
        LOGE(TAG, "Failed to create config JSON");
    }

    return success;
}

bool SDCardModule::parseConfigJSON(const char* json, PlaceNetConfig* config) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);

    if (error) {
        LOGE(TAG, "JSON parse error: %s", error.c_str());
        return false;
    }

    config->reset();

    JsonArray wifiArray = doc["wifi"];
    if (wifiArray) {
        int index = 0;
        for (JsonObject wifiObj : wifiArray) {
            if (index >= MAX_WIFI_NETWORKS) break;

            const char* ssid = wifiObj["ssid"];
            const char* password = wifiObj["password"];
            bool enabled = wifiObj["enabled"] | false;

            if (ssid) {
                strncpy(config->wifi[index].ssid, ssid, MAX_SSID_LENGTH - 1);
            }
            if (password) {
                strncpy(config->wifi[index].password, password, MAX_PASSWORD_LENGTH - 1);
            }
            config->wifi[index].enabled = enabled;
            index++;
        }
    }

    JsonObject mqttObj = doc["mqtt"];
    if (mqttObj) {
        const char* broker = mqttObj["broker"];
        if (broker) {
            strncpy(config->mqtt.broker, broker, MAX_MQTT_BROKER_LENGTH - 1);
        }

        config->mqtt.port = mqttObj["port"] | 1883;

        const char* username = mqttObj["username"];
        if (username) {
            strncpy(config->mqtt.username, username, MAX_PASSWORD_LENGTH - 1);
        }

        const char* password = mqttObj["password"];
        if (password) {
            strncpy(config->mqtt.password, password, MAX_PASSWORD_LENGTH - 1);
        }

        const char* clientId = mqttObj["clientId"];
        if (clientId) {
            strncpy(config->mqtt.clientId, clientId, MAX_MQTT_CLIENT_ID_LENGTH - 1);
        }

        const char* baseTopic = mqttObj["baseTopic"];
        if (baseTopic) {
            strncpy(config->mqtt.baseTopic, baseTopic, MAX_MQTT_TOPIC_LENGTH - 1);
        }

        config->mqtt.enabled = mqttObj["enabled"] | false;
        config->mqtt.useTLS = mqttObj["useTLS"] | false;
    }

    JsonObject httpObj = doc["httpServer"];
    if (httpObj) {
        const char* url = httpObj["url"];
        if (url) {
            strncpy(config->httpServer.url, url, MAX_HTTP_SERVER_LENGTH - 1);
        }

        config->httpServer.port = httpObj["port"] | 80;
        config->httpServer.enabled = httpObj["enabled"] | false;
        config->httpServer.useTLS = httpObj["useTLS"] | false;
    }

    JsonObject beaconObj = doc["beacon"];
    if (beaconObj) {
        config->beacon.beaconIntervalMs = beaconObj["intervalMs"] | 30000;
        config->beacon.loraEnabled = beaconObj["loraEnabled"] | true;
        config->beacon.gpsEnabled = beaconObj["gpsEnabled"] | true;
        config->beacon.bleEnabled = beaconObj["bleEnabled"] | false;
    }

    return true;
}

bool SDCardModule::createConfigJSON(const PlaceNetConfig* config, char* buffer, size_t bufferSize) {
    JsonDocument doc;

    JsonArray wifiArray = doc["wifi"].to<JsonArray>();
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        if (config->wifi[i].enabled) {
            JsonObject wifiObj = wifiArray.add<JsonObject>();
            wifiObj["ssid"] = config->wifi[i].ssid;
            wifiObj["password"] = config->wifi[i].password;
            wifiObj["enabled"] = config->wifi[i].enabled;
        }
    }

    JsonObject mqttObj = doc["mqtt"].to<JsonObject>();
    mqttObj["broker"] = config->mqtt.broker;
    mqttObj["port"] = config->mqtt.port;
    mqttObj["username"] = config->mqtt.username;
    mqttObj["password"] = config->mqtt.password;
    mqttObj["clientId"] = config->mqtt.clientId;
    mqttObj["baseTopic"] = config->mqtt.baseTopic;
    mqttObj["enabled"] = config->mqtt.enabled;
    mqttObj["useTLS"] = config->mqtt.useTLS;

    JsonObject httpObj = doc["httpServer"].to<JsonObject>();
    httpObj["url"] = config->httpServer.url;
    httpObj["port"] = config->httpServer.port;
    httpObj["enabled"] = config->httpServer.enabled;
    httpObj["useTLS"] = config->httpServer.useTLS;

    JsonObject beaconObj = doc["beacon"].to<JsonObject>();
    beaconObj["intervalMs"] = config->beacon.beaconIntervalMs;
    beaconObj["loraEnabled"] = config->beacon.loraEnabled;
    beaconObj["gpsEnabled"] = config->beacon.gpsEnabled;
    beaconObj["bleEnabled"] = config->beacon.bleEnabled;

    size_t written = serializeJsonPretty(doc, buffer, bufferSize);
    if (written == 0 || written >= bufferSize - 1) {
        LOGE(TAG, "JSON serialization failed or buffer too small");
        return false;
    }

    return true;
}

#endif // HAS_SDCARD
