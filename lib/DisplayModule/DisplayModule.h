#pragma once

#include "config.h"

#ifdef DISPLAY_MODEL

#include <U8g2lib.h>

class DisplayModule {
public:
    DisplayModule();
    ~DisplayModule();

    bool init();

    bool isInitialized() const { return initialized_; }

    // Display operations
    void clearBuffer();
    void drawStr(int x, int y, const char* str);
    void drawStrF(int x, int y, const char* format, ...);
    void sendBuffer();
    void setFont(const uint8_t* font);

private:
    bool renderBootSplash();
    DISPLAY_MODEL* u8g2_;
    bool initialized_;
};

#define DISPLAY_HOR_ALIGN_CENTER(display, t) (((display)->getDisplayWidth() - (display)->getUTF8Width(t)) / 2)
#define DISPLAY_HOR_ALIGN_RIGHT(display, t) ((display)->getDisplayWidth() - (display)->getUTF8Width(t))

#endif // DISPLAY_MODEL