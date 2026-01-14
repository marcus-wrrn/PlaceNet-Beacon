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
#include "tasks/main_task.h"

#ifdef HAS_GPS
#include "GPSModule.h"
#include "tasks/location_task.h"
#endif

#ifdef HAS_HTTP_SERVER
#include "SDCardModule.h"
#endif

static const char *TAG = "INIT";

#ifdef HAS_PMU
static PMUModule* g_pmu = nullptr;
#endif

#ifdef DISPLAY_MODEL
DisplayModule display;
#endif

static LoRaModule* g_lora = nullptr;

#ifdef HAS_GPS
static GPSModule* g_gps = nullptr;
#endif

int pktCount = 0;

#ifdef HAS_HTTP_SERVER
static SDCardModule* g_sdCard = nullptr;
static HTTPServerModule* g_httpServer = nullptr;
#endif

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

    g_lora = new LoRaModule();
    if (!g_lora) {
        LOGE(TAG, "Failed to create LoRaModule");
    }

#ifdef HAS_GPS
    g_gps = new GPSModule();
    if (!g_gps || !g_gps->init()) {
        LOGE(TAG, "Failed to create or initialize GPSModule");
    }
#endif

#ifdef HAS_HTTP_SERVER
    g_sdCard = new SDCardModule();
    if (!g_sdCard) {
        LOGE(TAG, "Failed to create SDCardModule");
    } else {
        g_httpServer = new HTTPServerModule(g_sdCard);
        if (!g_httpServer) {
            LOGE(TAG, "Failed to create HTTPServerModule");
        }
    }
#endif

    LOGI(TAG, "Spawning FreeRTOS tasks...");

#ifdef HAS_PMU
    if (g_pmu) {
        pmuStateQueue = xQueueCreate(5, sizeof(PMUState));
        TaskHandle_t pmuTaskHandle = nullptr;
        BaseType_t result = xTaskCreatePinnedToCore(
            pmuTask,
            "PMU",
            4096/2,
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

#ifdef HAS_GPS
    if (g_gps) {
        static LocationTaskParams locationParams;
        locationParams.gps = g_gps;
        locationUpdateQueue = xQueueCreate(5, sizeof(GPSData));
        BaseType_t result = xTaskCreatePinnedToCore(
            locationTask,
            "Location",
            4096,
            &locationParams,
            9,
            nullptr,
            1
        );

        if (result == pdPASS) {
            LOGI(TAG, "Location task created on core 1 (priority 7)");
        } else {
            LOGE(TAG, "Failed to create Location task");
        }
    }
#endif

    if (g_lora) {
        static LoRaTaskParams loraParams;
        loraParams.lora = g_lora;
        loraUpdateQueue = xQueueCreate(10, sizeof(LoRaPacket));
        BaseType_t result = xTaskCreatePinnedToCore(
            loraTask,
            "LoRa",
            4096,
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



#ifdef HAS_HTTP_SERVER
    if (g_sdCard && g_httpServer) {
        static HTTPServerTaskParams httpParams;
        httpParams.sdCard = g_sdCard;
        httpParams.httpServer = g_httpServer;

        BaseType_t result = xTaskCreatePinnedToCore(
            httpServerTask,
            "HTTP",
            8192,
            &httpParams,
            7,
            nullptr,
            1
        );

        if (result == pdPASS) {
            LOGI(TAG, "HTTP server task created on core 1 (priority 7)");
        } else {
            LOGE(TAG, "Failed to create HTTP server task");
        }
    }
#endif

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
