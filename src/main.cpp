#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "logger.h"
#include "BoardUtility.h"

#ifdef HAS_PMU
#include "PMUModule.h"
#include "tasks/pmu_task.h"
#endif

#ifdef DISPLAY_MODEL
#include "DisplayModule.h"
#include "tasks/display_task.h"
#endif

#include "LoRaModule.h"
#include "tasks/lora_task.h"

static const char *TAG = "MAIN";

// Global module pointers (managed by setup)
#ifdef HAS_PMU
static PMUModule* g_pmu = nullptr;
#endif

#ifdef DISPLAY_MODEL
static DisplayModule* g_display = nullptr;
#endif

static LoRaModule* g_lora = nullptr;

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    LOGI(TAG, "===========================================");
    LOGI(TAG, "PlaceNet Beacon - FreeRTOS Architecture");
    LOGI(TAG, "===========================================");

    BoardUtility::printChipInfo();
    BoardUtility::printWakeupReason();

#ifdef I2C1_SDA
    Wire1.begin(I2C1_SDA, I2C1_SCL);
    LOGI(TAG, "Scan Wire1 (I2C1)...");
    BoardUtility::scanI2C(&Wire1);
#endif

#ifdef HAS_PMU
    g_pmu = new PMUModule(PMU_WIRE_PORT, PMU_IRQ);
    if (!g_pmu->initialize()) {
        LOGE(TAG, "PMU initialization failed!");
        delete g_pmu;
        g_pmu = nullptr;
    } else {
        LOGI(TAG, "PMU initialized successfully");
    }
#endif

    delay(100); // Allow PMU to stabilize power rails

#ifdef I2C_SDA
    Wire.begin(I2C_SDA, I2C_SCL);
    LOGI(TAG, "Scan Wire (I2C0)...");
    BoardUtility::scanI2C(&Wire);
#endif

    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    LOGI(TAG, "SPI bus initialized");

#ifdef HAS_GPS
#ifdef GPS_EN_PIN
    pinMode(GPS_EN_PIN, OUTPUT);
    digitalWrite(GPS_EN_PIN, HIGH);
    LOGI(TAG, "GPS power enabled");
#endif
#endif

#ifdef DISPLAY_MODEL
    g_display = new DisplayModule();
    if (!g_display) {
        LOGE(TAG, "Failed to create DisplayModule");
    }
#endif

    g_lora = new LoRaModule();
    if (!g_lora) {
        LOGE(TAG, "Failed to create LoRaModule");
    }

    LOGI(TAG, "Spawning FreeRTOS tasks...");

#ifdef HAS_PMU
    if (g_pmu) {
        TaskHandle_t pmuTaskHandle = nullptr;
        BaseType_t result = xTaskCreatePinnedToCore(
            pmuTask,                        
            "PMU",                          
            4096,                           
            g_pmu,                          
            configMAX_PRIORITIES - 1,       
            &pmuTaskHandle,                 
            0                               
        );

        if (result == pdPASS && pmuTaskHandle != nullptr) {
            g_pmu->setTaskHandle(pmuTaskHandle);
            LOGI(TAG, "PMU task created on core 0 (priority %d)", configMAX_PRIORITIES - 1);
        } else {
            LOGE(TAG, "Failed to create PMU task");
        }
    }
#endif

#ifdef DISPLAY_MODEL
    if (g_display) {
        BaseType_t result = xTaskCreatePinnedToCore(
            displayTask,                    
            "Display",                     
            4096,                           
            g_display,                      
            5,                              
            nullptr,                        
            1                              
        );

        if (result == pdPASS) {
            LOGI(TAG, "Display task created on core 1 (priority 5)");
        } else {
            LOGE(TAG, "Failed to create Display task");
        }
    }
#endif

    if (g_lora) {
        BaseType_t result = xTaskCreatePinnedToCore(
            loraTask,                       
            "LoRa",                         
            8192,                           
            g_lora,                         
            8,                             
            nullptr,                        
            1                               
        );

        if (result == pdPASS) {
            LOGI(TAG, "LoRa task created on core 1 (priority 8)");
        } else {
            LOGE(TAG, "Failed to create LoRa task");
        }
    }

    LOGI(TAG, "===========================================");
    LOGI(TAG, "All tasks spawned - entering main loop");
    LOGI(TAG, "===========================================");
}

void loop() {

#ifdef DISPLAY_MODEL
    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate > 100) {
        lastUpdate = millis();

        if (g_display) {
            char buffer[64];
            g_display->clear();
            snprintf(buffer, sizeof(buffer), "Uptime: %lus", millis() / 1000);
            g_display->drawText(buffer, 0, 16);

#ifdef HAS_PMU
            if (g_pmu) {
                snprintf(buffer, sizeof(buffer), "Batt: %dmV", g_pmu->getBatteryVoltage());
                g_display->drawText(buffer, 0, 32);
            }
#endif
            g_display->sendBuffer();
        }
    }
#endif
    LOGI(TAG, "Main loop running - %lu seconds uptime", millis() / 1000);
    vTaskDelay(pdMS_TO_TICKS(100));
}
