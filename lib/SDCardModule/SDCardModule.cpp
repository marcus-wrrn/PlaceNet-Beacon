#include "SDCardModule.h"

#ifdef HAS_SDCARD

#include "logger.h"

static const char* TAG = "SD_MODULE";

SDCardModule::SDCardModule()
    : initialized_(false)
    , cardType_(CARD_NONE)
    , cardSize_(0)
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
    SPIClass sdSPI(HSPI);
    sdSPI.begin(SDCARD_SCLK, SDCARD_MISO, SDCARD_MOSI, SDCARD_CS);

    if (!SD.begin(SDCARD_CS, sdSPI, 40000000, SD_MOUNT_POINT, SD_MAX_FILES)) {
        LOGE(TAG, "SD card mount failed");
        return false;
    }
#elif defined(T_DECK_SX1262)
    if (!SD.begin(SDCARD_CS, SPI, 40000000, SD_MOUNT_POINT, SD_MAX_FILES)) {
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

#endif // HAS_SDCARD
