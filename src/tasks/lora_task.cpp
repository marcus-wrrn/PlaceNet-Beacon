#include "lora_task.h"
#include "LoRaModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef DISPLAY_MODEL
#include "DisplayModule.h"
#endif

static const char* TAG = "LORA_TASK";

#ifdef LORA_MODE_BEACON
#define BEACON_URL "https://placenet.local"
#define BEACON_INTERVAL_MS 60000
#define DUTY_CYCLE_REPORT_INTERVAL_MS 60000
#endif

#ifdef LORA_MODE_RECEIVER

#ifdef DISPLAY_MODEL
void displayReceivedPacket(DisplayModule* display, LoRaPacket* packet) {
    if (!display || !packet) {
        return;
    }

    char buffer[64];
    display->clear();

    display->drawText("RX PACKET:", 0, 12);

    memset(buffer, 0, sizeof(buffer));
    uint8_t displayLen = (packet->length < 20) ? packet->length : 20;
    memcpy(buffer, packet->data, displayLen);
    buffer[displayLen] = '\0';

    for (uint8_t i = 0; i < displayLen; i++) {
        if (buffer[i] < 32 || buffer[i] > 126) {
            buffer[i] = '.';
        }
    }

    display->drawText(buffer, 0, 28);

    snprintf(buffer, sizeof(buffer), "RSSI:%ddBm", packet->rssi);
    display->drawText(buffer, 0, 44);

    snprintf(buffer, sizeof(buffer), "SNR:%.1fdB Len:%d", packet->snr, packet->length);
    display->drawText(buffer, 0, 60);

    display->sendBuffer();
}
#endif
#endif

void loraTask(void* pvParameters) {
    LoRaTaskParams* params = static_cast<LoRaTaskParams*>(pvParameters);

    if (!params || !params->lora) {
        LOGE(TAG, "LoRa task parameters invalid, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LoRaModule* lora = params->lora;

#ifdef DISPLAY_MODEL
    DisplayModule* display = params->display;
#endif

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

#elif defined(LORA_MODE_RECEIVER)
    LOGI(TAG, "LoRa task started - Receiver mode");
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

#ifdef DISPLAY_MODEL
            if (display) {
                displayReceivedPacket(display, &packet);
            }
#endif
        }
    }

#else
    #error "Must define either LORA_MODE_BEACON or LORA_MODE_RECEIVER in config.h"
#endif
}
