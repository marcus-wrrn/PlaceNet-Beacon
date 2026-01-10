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
#endif

#include "LoRaModule.h"
#include "tasks/lora_task.h"

static const char *TAG = "MAIN";

// Global module pointers and queues (managed by setup)
#ifdef HAS_PMU
static PMUModule* g_pmu = nullptr;
#endif

#ifdef DISPLAY_MODEL
static DisplayModule display;
#endif

static LoRaModule* g_lora = nullptr;
static int pktCount = 0;

void mainTask(void* pvParameters) {
    const char* TASK_TAG = "MAIN_TASK";

    LOGI(TASK_TAG, "Main task starting...");

    if (loraUpdateQueue == nullptr) {
        LOGE(TASK_TAG, "ERROR: loraUpdateQueue is NULL!");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TASK_TAG, "Waiting for LoRa packets...");

    LoRaPacket pkt;
    uint32_t loopCount = 0;

    while (true) {
        loopCount++;

        if (loopCount % 100 == 0) {
            LOGI(TASK_TAG, "Loop iteration %lu, queue items waiting: %d",
                 loopCount, uxQueueMessagesWaiting(loraUpdateQueue));
        }

        // Block indefinitely waiting for packets (no timeout)
        // Task will wake immediately when packet arrives
        if (xQueueReceive(loraUpdateQueue, &pkt, portMAX_DELAY)) {
            pktCount++;
            LOGI(TASK_TAG, "#%d Received packet with RSSI: %d dBm\nSNR: %.2f dB\nLength: %d",
                 pktCount, pkt.rssi, pkt.snr, pkt.length);
            
            char buffer[sizeof(pkt)];
            snprintf(buffer, sizeof(buffer),
            "#%d Received packet with RSSI: %ddBm\nSNR: %.2f dB\nLength: %d", 
            pktCount, pkt.rssi, pkt.snr, pkt.length);
            display.clearBuffer();
            display.drawStrF(0, 8, "#%d, RSSI: %d, SNR: %.2f", pktCount, pkt.rssi, pkt.snr);
            display.drawStrF(0, 16, "%.*s", pkt.length, (const char*)pkt.data);
            display.sendBuffer();
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    LOGI(TAG, "===========================================");
    LOGI(TAG, "Beacon");
    LOGI(TAG, "===========================================");

    BoardUtility::printChipInfo();
    BoardUtility::printWakeupReason();

#ifdef BOARD_POWERON_PIN
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);
    LOGI(TAG, "Peripheral power enabled (GPIO %d)", BOARD_POWERON_PIN);
    delay(100);
#endif

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

    delay(100);

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
    display.init();
#endif

    if (g_lora) {
        static LoRaTaskParams loraParams;
        loraParams.lora = g_lora;
        loraUpdateQueue = xQueueCreate(10, sizeof(LoRaPacket));
        BaseType_t result = xTaskCreatePinnedToCore(
            loraTask,
            "LoRa",
            8192,
            &loraParams,
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

    // Create main packet-receiving task
    BaseType_t mainTaskResult = xTaskCreatePinnedToCore(
        mainTask,
        "MainTask",
        4096,
        nullptr,
        10,
        nullptr,
        0
    );

    if (mainTaskResult == pdPASS) {
        LOGI(TAG, "Main task created on core 0 (priority 10)");
    } else {
        LOGE(TAG, "Failed to create main task");
    }

    LOGI(TAG, "===========================================");
    LOGI(TAG, "All tasks spawned - setup complete");
    LOGI(TAG, "===========================================");
}

void loop() {}
