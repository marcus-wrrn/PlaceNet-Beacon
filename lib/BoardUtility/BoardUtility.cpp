#include "BoardUtility.h"
#include "logger.h"
#include <esp_mac.h>

static const char* TAG = "BOARD_UTIL";

namespace BoardUtility {

void printWakeupReason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

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

void printChipInfo() {
    LOGI(TAG, "-----------------------------------");

    printWakeupReason();

    // PSRAM
    if (psramFound()) {
        uint32_t psram = ESP.getPsramSize();
        float psramMB = psram / 1024.0 / 1024.0;
        LOGI(TAG, "PSRAM is enabled! PSRAM: %.2fMB", psramMB);
    } else {
        LOGI(TAG, "PSRAM is disabled!");
    }

    // Flash and chip info
    float flashMB = ESP.getFlashChipSize() / 1024.0 / 1024.0;
    uint8_t flashSpeed = ESP.getFlashChipSpeed() / 1000 / 1000;
    String chipModel = ESP.getChipModel();
    uint8_t chipRev = ESP.getChipRevision();
    uint8_t cpuFreq = ESP.getCpuFreqMHz();

    LOGI(TAG, "Flash: %.2f MB", flashMB);
    LOGI(TAG, "Flash speed: %d MHz", flashSpeed);
    LOGI(TAG, "Model: %s", chipModel.c_str());
    LOGI(TAG, "Chip Revision: %d", chipRev);
    LOGI(TAG, "Freq: %d MHz", cpuFreq);
    LOGI(TAG, "SDK Ver: %s", ESP.getSdkVersion());
    LOGI(TAG, "DATE: %s", __DATE__);
    LOGI(TAG, "TIME: %s", __TIME__);

    // MAC address
    uint8_t mac[6];
    char macStr[18] = {0};
    esp_efuse_mac_get_default(mac);
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    LOGI(TAG, "EFUSE MAC: %s", macStr);

    LOGI(TAG, "-----------------------------------");
}

void scanI2C(TwoWire* wire) {
    uint8_t err, addr;
    int nDevices = 0;

    LOGI(TAG, "I2C Devices scanning");

    for (addr = 1; addr < 127; addr++) {
        wire->beginTransmission(addr);
        delay(2);
        err = wire->endTransmission();

        if (err == 0) {
            nDevices++;
            switch (addr) {
            case 0x77:
            case 0x76:
                LOGI(TAG, "  Find BMX280 Sensor at 0x%02X!", addr);
                break;
            case 0x34:
                LOGI(TAG, "  Find AXP192/AXP2101 PMU at 0x%02X!", addr);
                break;
            case 0x3C:
                LOGI(TAG, "  Find SSD1306/SH1106 display at 0x%02X!", addr);
                break;
            case 0x51:
                LOGI(TAG, "  Find PCF8563 RTC at 0x%02X!", addr);
                break;
            case 0x1C:
                LOGI(TAG, "  Find QMC6310 MAG Sensor at 0x%02X!", addr);
                break;
            default:
                LOGI(TAG, "  I2C device found at address 0x%02X!", addr);
                break;
            }
        } else if (err == 4) {
            LOGE(TAG, "Unknown error at address 0x%02X", addr);
        }
    }

    if (nDevices == 0) {
        LOGI(TAG, "No I2C devices found");
    }

    LOGI(TAG, "Scan devices done.");
}

DevInfo_t getDeviceInfo() {
    DevInfo_t info;

    info.chipModel = ESP.getChipModel();
    info.chipModelRev = ESP.getChipRevision();
    info.chipFreq = ESP.getCpuFreqMHz();
    info.flashSize = ESP.getFlashChipSize() / 1024.0 / 1024.0;
    info.flashSpeed = ESP.getFlashChipSpeed() / 1000 / 1000;

    if (psramFound()) {
        uint32_t psram = ESP.getPsramSize();
        info.psramSize = psram / 1024.0 / 1024.0;
    } else {
        info.psramSize = 0;
    }

    return info;
}

} // namespace BoardUtility
