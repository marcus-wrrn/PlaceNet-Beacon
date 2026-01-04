#include "DisplayModule.h"

#ifdef DISPLAY_MODEL

#include "logger.h"
#include <Wire.h>

static const char* TAG = "DISPLAY_MODULE";

DisplayModule::DisplayModule()
    : u8g2_(nullptr)
    , commandQueue_(nullptr)
    , initialized_(false)
{
    commandQueue_ = xQueueCreate(10, sizeof(DisplayCommand));
    if (!commandQueue_) {
        LOGE(TAG, "Failed to create display command queue");
    }
}

DisplayModule::~DisplayModule() {
    if (u8g2_) {
        delete u8g2_;
        u8g2_ = nullptr;
    }
    if (commandQueue_) {
        vQueueDelete(commandQueue_);
        commandQueue_ = nullptr;
    }
}

bool DisplayModule::init() {
    if (initialized_) {
        LOGW(TAG, "Display already initialized");
        return true;
    }

    // Check if display is present on I2C
    Wire.beginTransmission(DISPLAY_ADDR);
    if (Wire.endTransmission() != 0) {
        LOGW(TAG, "Failed to find Display at 0x%02X address", DISPLAY_ADDR);
        return false;
    }

    LOGI(TAG, "Find Display model at 0x%02X address", DISPLAY_ADDR);

    u8g2_ = new DISPLAY_MODEL(U8G2_R0, U8X8_PIN_NONE);
    u8g2_->begin();
    u8g2_->clearBuffer();

    u8g2_->setFont(u8g2_font_inb19_mr);
    u8g2_->drawStr(0, 30, "PlaceNet");
    u8g2_->drawHLine(2, 50, 47);
    u8g2_->drawHLine(3, 54, 47);
    u8g2_->setFont(u8g2_font_inb19_mf);
    u8g2_->drawStr(30, 60, "Beacon");
    u8g2_->sendBuffer();
    u8g2_->setFont(u8g2_font_fur11_tf);

    initialized_ = true;
    LOGI(TAG, "Display initialized successfully");

    vTaskDelay(pdMS_TO_TICKS(3000));  // Show splash for 3 seconds

    return true;
}

void DisplayModule::renderCommand(const DisplayCommand& cmd) {
    if (!u8g2_ || !initialized_) {
        LOGW(TAG, "Display not initialized, cannot render");
        return;
    }

    switch (cmd.type) {
        case DISPLAY_CLEAR:
            u8g2_->clearBuffer();
            //LOGD(TAG, "Clear buffer");
            break;

        case DISPLAY_DRAW_TEXT:
            u8g2_->drawStr(cmd.data.drawText.x, cmd.data.drawText.y, cmd.data.drawText.text);
            // LOGD(TAG, "Draw text at (%d, %d): %s",
            //          cmd.data.drawText.x, cmd.data.drawText.y, cmd.data.drawText.text);
            break;

        case DISPLAY_DRAW_HLINE:
            u8g2_->drawHLine(cmd.data.drawHLine.x, cmd.data.drawHLine.y, cmd.data.drawHLine.width);
            // LOGD(TAG, "Draw H-line at (%d, %d) width %d",
            //          cmd.data.drawHLine.x, cmd.data.drawHLine.y, cmd.data.drawHLine.width);
            break;

        case DISPLAY_DRAW_VLINE:
            u8g2_->drawVLine(cmd.data.drawVLine.x, cmd.data.drawVLine.y, cmd.data.drawVLine.height);
            // LOGD(TAG, "Draw V-line at (%d, %d) height %d",
            //          cmd.data.drawVLine.x, cmd.data.drawVLine.y, cmd.data.drawVLine.height);
            break;

        case DISPLAY_SEND_BUFFER:
            u8g2_->sendBuffer();
            // LOGD(TAG, "Send buffer to display");
            break;

        case DISPLAY_SET_FONT:
            u8g2_->setFont(cmd.data.setFont.font);
            // LOGD(TAG, "Set font");
            break;

        default:
            // LOGW(TAG, "Unknown display command type: %d", cmd.type);
            break;
    }
}

void DisplayModule::clear() {
    if (!commandQueue_) {
        LOGW(TAG, "Command queue not available");
        return;
    }
    DisplayCommand cmd;
    cmd.type = DISPLAY_CLEAR;
    xQueueSend(commandQueue_, &cmd, 0);
}

void DisplayModule::drawText(const char* text, uint8_t x, uint8_t y) {
    if (!commandQueue_) {
        LOGW(TAG, "Command queue not available");
        return;
    }
    DisplayCommand cmd;
    cmd.type = DISPLAY_DRAW_TEXT;
    cmd.data.drawText.x = x;
    cmd.data.drawText.y = y;
    strncpy(cmd.data.drawText.text, text, sizeof(cmd.data.drawText.text) - 1);
    cmd.data.drawText.text[sizeof(cmd.data.drawText.text) - 1] = '\0';
    xQueueSend(commandQueue_, &cmd, 0);
}

void DisplayModule::sendBuffer() {
    if (!commandQueue_) {
        LOGW(TAG, "Command queue not available");
        return;
    }
    DisplayCommand cmd;
    cmd.type = DISPLAY_SEND_BUFFER;
    xQueueSend(commandQueue_, &cmd, 0);
}

void DisplayModule::drawHLine(uint8_t x, uint8_t y, uint8_t width) {
    if (!commandQueue_) {
        LOGW(TAG, "Command queue not available");
        return;
    }
    DisplayCommand cmd;
    cmd.type = DISPLAY_DRAW_HLINE;
    cmd.data.drawHLine.x = x;
    cmd.data.drawHLine.y = y;
    cmd.data.drawHLine.width = width;
    xQueueSend(commandQueue_, &cmd, 0);
}

void DisplayModule::drawVLine(uint8_t x, uint8_t y, uint8_t height) {
    if (!commandQueue_) {
        LOGW(TAG, "Command queue not available");
        return;
    }
    DisplayCommand cmd;
    cmd.type = DISPLAY_DRAW_VLINE;
    cmd.data.drawVLine.x = x;
    cmd.data.drawVLine.y = y;
    cmd.data.drawVLine.height = height;
    xQueueSend(commandQueue_, &cmd, 0);
}

void DisplayModule::setFont(const uint8_t* font) {
    if (!commandQueue_) {
        LOGW(TAG, "Command queue not available");
        return;
    }
    DisplayCommand cmd;
    cmd.type = DISPLAY_SET_FONT;
    cmd.data.setFont.font = font;
    xQueueSend(commandQueue_, &cmd, 0);
}

#endif // DISPLAY_MODEL
