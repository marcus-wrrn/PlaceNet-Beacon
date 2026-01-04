#include "lora_task.h"
#include "LoRaModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "LORA_TASK";

#define BEACON_URL "https://placenet.local"
#define BEACON_INTERVAL_MS 30000
#define DUTY_CYCLE_REPORT_INTERVAL_MS 60000

void loraTask(void* pvParameters) {
    LoRaModule* lora = static_cast<LoRaModule*>(pvParameters);

    if (!lora) {
        LOGE(TAG, "LoRa module pointer is null, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "LoRa task starting...");

    if (!lora->init()) {
        LOGE(TAG, "LoRa initialization failed, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "LoRa task started - Broadcasting URL every %d ms", BEACON_INTERVAL_MS);

    const char* beaconUrl = BEACON_URL;
    uint8_t beaconLength = strlen(beaconUrl);
    uint32_t beaconCount = 0;

    uint32_t lastDutyCycleReport = millis();

    while (1) {
        beaconCount++;
        LOGI(TAG, "Beacon #%lu: Transmitting URL (%d bytes)", beaconCount, beaconLength);

        if (lora->transmit((const uint8_t*)beaconUrl, beaconLength)) {
            LOGI(TAG, "Beacon transmitted successfully");
        } else {
            LOGW(TAG, "Beacon transmission failed, will retry on next cycle");
        }

        uint32_t currentTime = millis();
        if (currentTime - lastDutyCycleReport >= DUTY_CYCLE_REPORT_INTERVAL_MS) {
            float dutyCycle1h = lora->getDutyCycle(3600000);
            float dutyCycle10m = lora->getDutyCycle(600000);
            float dutyCycle1m = lora->getDutyCycle(60000);

            Serial.println("=== LoRa Duty Cycle Report ===");
            Serial.printf("  1 minute:  %.3f%%\n", dutyCycle1m);
            Serial.printf(" 10 minutes: %.3f%%\n", dutyCycle10m);
            Serial.printf("  1 hour:    %.3f%%\n", dutyCycle1h);
            Serial.println("==============================");

            lastDutyCycleReport = currentTime;
        }

        vTaskDelay(pdMS_TO_TICKS(BEACON_INTERVAL_MS));
    }
}
