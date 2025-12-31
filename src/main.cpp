#include <Arduino.h>
#include "BoardManager.h"
#include "logger.h"

static const char* TAG = "MAIN";

void setup() {
    Serial.begin(115200);

    LoRaBoardManager& board = LoRaBoardManager::getInstance();
    if (!board.initialize()) {
        Serial.println("Board initialization failed!");
    }
    LOGI(TAG, "Board Initialized");
    board.printDeviceStatus(false);
}

void loop() {
    LOGI(TAG, "Hello");

    #ifdef HAS_PMU
    PMUManager* pmu = LoRaBoardManager::getInstance().getPMUManager();
    if (pmu) {
        pmu->processEvents(nullptr);
    }
    #endif

    delay(1000);
}
