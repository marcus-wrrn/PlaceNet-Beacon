#include <Arduino.h>
#include "BoardManager.h"
#include "logger.h"

static const char* TAG = "MAIN";

void setup() {
    // Serial.begin() is now called in initialize()

    LoRaBoardManager& board = LoRaBoardManager::getInstance();
    if (!board.initialize()) {
        Serial.println("Board initialization failed!");
    }
    LOGI(TAG, "Board Initialized");

    Serial.println("Success");
}

void loop() {
    Serial.println("Hello");

    #ifdef HAS_PMU
    LoRaBoardManager::getInstance().processPMUEvents(nullptr);
    #endif

    delay(100);
}
