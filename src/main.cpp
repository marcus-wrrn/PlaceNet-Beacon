#include <Arduino.h>
#include "LoRaBoard.h"
#include "logger.h"

static const char* TAG = "MAIN";

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    LOGI(TAG, "Starting initialization...");

    LoRaBoardManager& board = LoRaBoardManager::getInstance();
    if (!board.initialize()) {
        LOGE(TAG, "Board initialization failed!");
    } else {
        LOGI(TAG, "Board initialized successfully");
    }
}

void loop() {
    Serial.println("Hello");

    #ifdef HAS_PMU
    LoRaBoardManager::getInstance().processPMUEvents(nullptr);
    #endif

    delay(100);
}
