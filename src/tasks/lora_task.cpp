#include "lora_task.h"
#include "LoRaModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "LORA_TASK";

#define BEACON_URL "https://placenet.local"
#define BEACON_INTERVAL_MS 10000

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

    while (1) {
        beaconCount++;
        LOGI(TAG, "Beacon #%lu: Transmitting URL (%d bytes)", beaconCount, beaconLength);

        if (lora->transmit((const uint8_t*)beaconUrl, beaconLength)) {
            LOGI(TAG, "Beacon transmitted successfully");
        } else {
            LOGW(TAG, "Beacon transmission failed, will retry on next cycle");
        }

        vTaskDelay(pdMS_TO_TICKS(BEACON_INTERVAL_MS));
    }
}
