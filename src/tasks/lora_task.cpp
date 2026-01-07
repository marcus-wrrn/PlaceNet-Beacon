#include "lora_task.h"
#include "LoRaModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> 

static const char* TAG = "LORA_TASK";

QueueHandle_t loraUpdateQueue = nullptr;

#ifdef LORA_MODE_BEACON
#define BEACON_URL "https://placenet.local"
#define BEACON_INTERVAL_MS 60000
#define DUTY_CYCLE_REPORT_INTERVAL_MS 60000
#endif

#ifdef LORA_MODE_RECEIVER
#endif

void loraTask(void* pvParameters) {
    LoRaTaskParams* params = static_cast<LoRaTaskParams*>(pvParameters);
    
    if (!params || !params->lora) {
        LOGE(TAG, "LoRa task parameters invalid, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LoRaModule* lora = params->lora;

    LOGI(TAG, "LoRa task starting...");

    if (!lora->init()) {
        LOGE(TAG, "LoRa initialization failed, task exiting");
        vTaskDelete(nullptr);
        return;
    }

#ifdef LORA_MODE_BEACON
    LOGI(TAG, "LoRa task started - Broadcasting URL every %d ms", BEACON_INTERVAL_MS);

    const char* beaconUrl = BEACON_URL;
    uint8_t beaconLength = strlen(beaconUrl);
    uint32_t beaconCount = 0;

    uint32_t lastDutyCycleReport = millis();

    while (1) {
        beaconCount++;
        LOGI(TAG, "Beacon #%lu: Transmitting URL (%d bytes)", beaconCount, beaconLength);

        bool success = lora->transmit((const uint8_t*)beaconUrl, beaconLength);
        if (success) {
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

#ifdef DISPLAY_MODEL
            if (displayQueue) {
                DisplayEvent evt = createLoRaTxEvent(
                    success,
                    beaconCount,
                    dutyCycle1m,
                    dutyCycle10m
                );
                sendDisplayEvent(displayQueue, evt);
            }
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(BEACON_INTERVAL_MS));
    }

#elif defined(LORA_MODE_RECEIVER)
    LOGI(TAG, "LoRa task started - Receiver mode");

    if (!lora->startListening()) {
        LOGE(TAG, "Failed to start listening mode, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "Listening for LoRa packets...");

    LoRaPacket packet;
    uint32_t packetCount = 0;

    while (1) {
        if (lora->receive(&packet)) {
            packetCount++;

            LOGI(TAG, "=== Packet #%lu Received ===", packetCount);
            LOGI(TAG, "Length: %d bytes", packet.length);
            LOGI(TAG, "RSSI: %d dBm", packet.rssi);
            LOGI(TAG, "SNR: %.2f dB", packet.snr);

            Serial.print("Data (text): ");
            for (uint8_t i = 0; i < packet.length; i++) {
                if (packet.data[i] >= 32 && packet.data[i] <= 126) {
                    Serial.print((char)packet.data[i]);
                } else {
                    Serial.print('.');
                }
            }
            Serial.println();

            Serial.print("Data (hex): ");
            for (uint8_t i = 0; i < packet.length; i++) {
                Serial.printf("%02X ", packet.data[i]);
            }
            Serial.println();
            LOGI(TAG, "===========================");

            if (loraUpdateQueue != nullptr) {
                BaseType_t result = xQueueSend(loraUpdateQueue, &packet, portMAX_DELAY);
                if (result == pdPASS) {
                    LOGI(TAG, "Packet queued successfully to loraUpdateQueue");
                } else {
                    LOGE(TAG, "Failed to queue packet to loraUpdateQueue");
                }
            } else {
                LOGE(TAG, "loraUpdateQueue is null! Cannot send packet to main task");
            }
        }
    }

#else
    #error "Must define either LORA_MODE_BEACON or LORA_MODE_RECEIVER in config.h"
#endif
}
