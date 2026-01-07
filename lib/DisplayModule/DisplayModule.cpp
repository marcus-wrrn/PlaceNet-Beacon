#include "DisplayModule.h"

#ifdef DISPLAY_MODEL

#include "logger.h"
#include <Wire.h>

static const char* TAG = "DISPLAY_MODULE";

DisplayModule::DisplayModule()
    : u8g2_(nullptr)
    , initialized_(false)
{
}

DisplayModule::~DisplayModule() {
    if (u8g2_) {
        delete u8g2_;
        u8g2_ = nullptr;
    }
}

bool DisplayModule::init() {
    if (initialized_) {
        LOGW(TAG, "Display already initialized");
        return true;
    }

    Wire.beginTransmission(DISPLAY_ADDR);
    if (Wire.endTransmission() != 0) {
        LOGW(TAG, "Failed to find Display at 0x%02X address", DISPLAY_ADDR);
        return false;
    }

    LOGI(TAG, "Found display at 0x%02X address", DISPLAY_ADDR);

    LOGI(TAG, "Creating U8G2 display object...");
    u8g2_ = new DISPLAY_MODEL(U8G2_R0, U8X8_PIN_NONE);

    if (!u8g2_) {
        LOGE(TAG, "Failed to allocate U8G2 display object");
        return false;
    }

    LOGI(TAG, "Calling u8g2->begin()...");
    u8g2_->begin();

    LOGI(TAG, "Clearing buffer...");
    u8g2_->clearBuffer();

    LOGI(TAG, "Setting font...");
    u8g2_->setFont(u8g2_font_fur11_tf);

    initialized_ = true;
    LOGI(TAG, "Display initialized successfully");

    return true;
}

#endif // DISPLAY_MODEL
