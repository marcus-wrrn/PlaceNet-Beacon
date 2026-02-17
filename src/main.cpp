#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>

#include "config.h"
#include "logger.h"
#include "BoardUtility.h"

#ifdef HAS_PMU
#include "PMUModule.h"
#include "tasks/pmu_task.h"
#endif

#ifdef HAS_GPS
#include "GPSModule.h"
#endif

#ifdef DISPLAY_MODEL
#include "DisplayModule.h"
#endif

#include "LoRaModule.h"
#include "BLEModule.h"
#include "PlaceNetConfig.h"
#include "SDCardModule.h"
#include "tasks/main_task.h"

static const char* TAG = "INIT";

#ifdef HAS_PMU
static PMUModule* g_pmu = nullptr;
#endif

#ifdef DISPLAY_MODEL
DisplayModule display;
#endif

static LoRaModule*     g_lora   = nullptr;
static BLEModule*      g_ble    = nullptr;
static PlaceNetConfig  g_config;
static SDCardModule*   g_sd     = nullptr;

#ifdef HAS_GPS
static GPSModule* g_gps = nullptr;
#endif

int pktCount = 0;

static bool setupPMU() {
#ifdef HAS_PMU
    g_pmu = new PMUModule(PMU_WIRE_PORT, PMU_IRQ);
    if (!g_pmu->initialize()) {
        LOGE(TAG, "PMU initialization failed!");
        delete g_pmu;
        g_pmu = nullptr;
        return false;
    }
    LOGI(TAG, "PMU initialized successfully");
    setupPMUTask(g_pmu, 2048);
#else
    LOGI(TAG, "No PMU configured");
#endif
    return true;
}

static bool setupGPS() {
#ifdef HAS_GPS
    g_gps = new GPSModule();
    if (!g_gps || !g_gps->init()) {
        LOGE(TAG, "Failed to create or initialize GPSModule");
        delete g_gps;
        g_gps = nullptr;
        return false;
    }
    LOGI(TAG, "GPS initialized successfully");
#else
    LOGI(TAG, "No GPS configured");
#endif
    return true;
}

static bool setupDisplay() {
    bool result = true;
#ifdef DISPLAY_MODEL
    result = display.init();
    if (result) {
        LOGI(TAG, "Display initialized successfully");
    } else {
        LOGI(TAG, "Display failed to initialize");
    }
#else
    LOGI(TAG, "No display configured");
#endif
    return result;
}

static bool setupSD() {
#ifdef HAS_SDCARD
    g_sd = new SDCardModule();
    if (!g_sd->init()) {
        LOGE(TAG, "SD card init failed — no persistent config");
        delete g_sd;
        g_sd = nullptr;
        return false;
    }
    LOGI(TAG, "SD card initialized");
    if (g_sd->loadConfig(&g_config)) {
        LOGI(TAG, "Config loaded from SD");
    } else {
        LOGI(TAG, "No config on SD — saving defaults");
        g_sd->saveConfig(&g_config);
    }
#else
    LOGI(TAG, "No SD card configured");
#endif
    g_config.print();
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    LOGI(TAG, "===========================================");
    LOGI(TAG, "PlaceNet Beacon");
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

    setupPMU();
    delay(100);

#ifdef I2C_SDA
    Wire.begin(I2C_SDA, I2C_SCL);
    LOGI(TAG, "Scan Wire (I2C0)...");
    BoardUtility::scanI2C(&Wire);
#endif

    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    LOGI(TAG, "SPI bus initialized");

    setupDisplay();
    setupGPS();
    setupSD();

    // Create hardware module objects — not yet initialized
    g_lora = new LoRaModule();
    g_ble  = new BLEModule();

    // Build supervisor context and spawn only main_task
    static SupervisorContext ctx;
    ctx.lora   = g_lora;
    ctx.ble    = g_ble;
    ctx.config = &g_config;
    ctx.sd     = g_sd;
#ifdef HAS_GPS
    ctx.gps = g_gps;
#endif

    setupMainTask(&ctx, 8192);

    LOGI(TAG, "===========================================");
    LOGI(TAG, "Setup complete");
    LOGI(TAG, "Free heap: %u bytes", esp_get_free_heap_size());
    LOGI(TAG, "Minimum free heap: %u bytes", esp_get_minimum_free_heap_size());
    LOGI(TAG, "Free internal RAM: %u bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    LOGI(TAG, "Free PSRAM: %u bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    LOGI(TAG, "===========================================");
}

void loop() {}
