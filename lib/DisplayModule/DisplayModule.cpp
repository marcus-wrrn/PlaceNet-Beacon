#include "DisplayModule.h"

#ifdef DISPLAY_MODEL

#include "logger.h"
#include <Wire.h>
#include <cstdarg>

static const char* TAG = "DISPLAY_MODULE";

DisplayModule::DisplayModule()
    : u8g2_(nullptr)
    , initialized_(false)
    , lineCount_(0)
{}

DisplayModule::~DisplayModule() {
    if (u8g2_) {
        delete u8g2_;
        u8g2_ = nullptr;
    }
}

bool DisplayModule::renderBootSplash() {
    u8g2_->clearBuffer();
    u8g2_->setFont(u8g2_font_5x8_tr);
    u8g2_->drawStr(0, 8, "PlaceNet Beacon Online");
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
    lineCount_ = 0;
    LOGI(TAG, "Display initialized successfully");
    return result;
}

void DisplayModule::clearBuffer() {
    if (u8g2_) {
        u8g2_->clearBuffer();
    }
    lineCount_ = 0;
}

void DisplayModule::drawStr(int x, int y, const char* str) {
    if (u8g2_) {
        u8g2_->drawStr(x, y, str);
    }
}

void DisplayModule::drawStrF(int x, int y, const char* format, ...) {
    if (!u8g2_) {
        return;
    }

    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    u8g2_->drawStr(x, y, buffer);
}

void DisplayModule::drawLine(const char* format, ...) {
    if (!u8g2_) {
        return;
    }

    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    u8g2_->drawStr(0, (lineCount_ + 1) * 8, buffer);
    lineCount_++;
}

void DisplayModule::sendBuffer() {
    if (u8g2_) {
        u8g2_->sendBuffer();
    }
}

void DisplayModule::setFont(const uint8_t* font) {
    if (u8g2_) {
        u8g2_->setFont(font);
    }
}

#endif // DISPLAY_MODEL
