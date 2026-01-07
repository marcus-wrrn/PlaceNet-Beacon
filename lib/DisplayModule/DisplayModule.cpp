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
    u8g2_->setFont(u8g2_font_inb19_mr);
    u8g2_->drawStr(0, 30, "PlaceNet");
    u8g2_->drawHLine(2, 50, 47);
    u8g2_->drawHLine(3, 54, 47);
    u8g2_->drawVLine(3, 54, 10);
    u8g2_->drawStr(30, 60, "Beacon");
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
    u8g2_->begin();

    initialized_ = true;
    LOGI(TAG, "Display initialized successfully");
    return renderBootSplash();
}

#endif // DISPLAY_MODEL
