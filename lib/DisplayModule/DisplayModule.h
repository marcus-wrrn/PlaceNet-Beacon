#pragma once

#include "config.h"

#ifdef DISPLAY_MODEL

#include <U8g2lib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

enum DisplayCommandType {
    DISPLAY_CLEAR,
    DISPLAY_DRAW_TEXT,
    DISPLAY_DRAW_HLINE,
    DISPLAY_DRAW_VLINE,
    DISPLAY_SEND_BUFFER,
    DISPLAY_SET_FONT
};

struct DisplayCommand {
    DisplayCommandType type;
    union {
        struct {
            uint8_t x;
            uint8_t y;
            char text[64];
        } drawText;
        struct {
            uint8_t x;
            uint8_t y;
            uint8_t width;
        } drawHLine;
        struct {
            uint8_t x;
            uint8_t y;
            uint8_t height;
        } drawVLine;
        struct {
            const uint8_t* font;
        } setFont;
    } data;
};

class DisplayModule {
public:
    /**
     * @brief Constructor - creates command queue
     */
    DisplayModule();

    /**
     * @brief Destructor - deletes display and queue
     */
    ~DisplayModule();

    /**
     * @brief Initialize the display hardware
     * @return true if display initialized successfully
     */
    bool init();

    /**
     * @brief Get the command queue handle
     * @return QueueHandle_t for sending display commands
     */
    QueueHandle_t getCommandQueue() { return commandQueue_; }

    /**
     * @brief Render a display command (called by display task)
     * @param cmd Display command to render
     */
    void renderCommand(const DisplayCommand& cmd);

    /**
     * @brief Get direct access to U8g2 display (use with caution in task)
     * @return Pointer to DISPLAY_MODEL
     */
    DISPLAY_MODEL* getDisplay() { return u8g2_; }

    /**
     * @brief Clear the display buffer (queue-based)
     */
    void clear();

    /**
     * @brief Draw text at specified position (queue-based)
     * @param text Text string to draw
     * @param x X coordinate
     * @param y Y coordinate
     */
    void drawText(const char* text, uint8_t x, uint8_t y);

    /**
     * @brief Send buffer to display (queue-based)
     */
    void sendBuffer();

    /**
     * @brief Draw horizontal line (queue-based)
     * @param x X coordinate
     * @param y Y coordinate
     * @param width Line width
     */
    void drawHLine(uint8_t x, uint8_t y, uint8_t width);

    /**
     * @brief Draw vertical line (queue-based)
     * @param x X coordinate
     * @param y Y coordinate
     * @param height Line height
     */
    void drawVLine(uint8_t x, uint8_t y, uint8_t height);

    /**
     * @brief Set font (queue-based)
     * @param font Pointer to U8g2 font
     */
    void setFont(const uint8_t* font);

private:
    DISPLAY_MODEL* u8g2_;
    QueueHandle_t commandQueue_;
    bool initialized_;
};

// Helper macros for horizontal alignment
#define DISPLAY_HOR_ALIGN_CENTER(display, t) (((display)->getDisplayWidth() - (display)->getUTF8Width(t)) / 2)
#define DISPLAY_HOR_ALIGN_RIGHT(display, t) ((display)->getDisplayWidth() - (display)->getUTF8Width(t))

#endif // DISPLAY_MODEL