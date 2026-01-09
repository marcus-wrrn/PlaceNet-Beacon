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

bool DisplayModule::renderBootSplash() {
    u8g2_->clearBuffer();
    u8g2_->setFont(u8g2_font_4x6_tr);
    u8g2_->drawStr(0, 30, "The Beacon");
    u8g2_->drawHLine(2, 50, 47);
    u8g2_->drawHLine(3, 54, 47);
    u8g2_->drawVLine(3, 54, 10);
    u8g2_->drawStr(30, 60, "Is Lit");
    u8g2_->sendBuffer();
    return true;
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

    LOGI(TAG, "Calling u8g2->begin()...");
    bool result = u8g2_->begin();
    if (!result) {
        LOGE(TAG, "U8G2 Could not initialize");
        return result;
    }

    result = renderBootSplash();
    if (!result) {
        LOGE(TAG, "Splash Screen failed to initialize");
    }

    initialized_ = true;
    LOGI(TAG, "Display initialized successfully");
    return result;
}

#endif // DISPLAY_MODEL
