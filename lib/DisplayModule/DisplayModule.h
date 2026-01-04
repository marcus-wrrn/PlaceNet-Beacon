#pragma once

#include "config.h"

#ifdef DISPLAY_MODEL

#include <U8g2lib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "DisplayEvents.h"

class DisplayModule {
public:
    DisplayModule();
    ~DisplayModule();

    bool init();

    QueueHandle_t getEventQueue() { return eventQueue_; }

    DISPLAY_MODEL* getDisplay() { return u8g2_; }

    bool isInitialized() const { return initialized_; }

private:
    DISPLAY_MODEL* u8g2_;
    QueueHandle_t eventQueue_;
    bool initialized_;
};

#define DISPLAY_HOR_ALIGN_CENTER(display, t) (((display)->getDisplayWidth() - (display)->getUTF8Width(t)) / 2)
#define DISPLAY_HOR_ALIGN_RIGHT(display, t) ((display)->getDisplayWidth() - (display)->getUTF8Width(t))

#endif // DISPLAY_MODEL