#include "main_task.h"
#include "lora_task.h"
#include "managers/network_manager.h"
#include "LoRaModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>

#ifdef HAS_PMU
#include "pmu_task.h"
#include "PMUModule.h"
#include "managers/pmu_manager.h"
#endif

#ifdef HAS_GPS
#include "location_task.h"
#include "GPSModule.h"
#include "managers/gps_manager.h"
#endif

static const char* TAG = "MAIN";

static SupervisorContext* g_ctx = nullptr;
static std::atomic<BeaconState> g_state{STATE_SETUP};

// Credentials received from BLE callback (set from BLE ISR/task context)
static volatile bool g_credsReceived = false;
static char g_pendingSsid[MAX_SSID_LENGTH]     = {};
static char g_pendingPassword[MAX_PASSWORD_LENGTH] = {};

static void onCredentialsReceived(const char* ssid, const char* pass) {
    strncpy(g_pendingSsid, ssid, MAX_SSID_LENGTH - 1);
    g_pendingSsid[MAX_SSID_LENGTH - 1] = '\0';
    strncpy(g_pendingPassword, pass, MAX_PASSWORD_LENGTH - 1);
    g_pendingPassword[MAX_PASSWORD_LENGTH - 1] = '\0';
    g_credsReceived = true;
}

static void enterSetup(SupervisorContext* ctx) {
    LOGI(TAG, "Entering SETUP state: starting BLE provisioning");
    ctx->ble->setCredentialsCallback(onCredentialsReceived);
    ctx->ble->init();
    ctx->ble->startAdvertising();
    g_state = STATE_PROVISIONING;
    LOGI(TAG, "BLE advertising started, waiting for credentials");
}

static void enterOperational(SupervisorContext* ctx) {
    LOGI(TAG, "Entering OPERATIONAL state");

    if (ctx->lora) {
        setupLoRaTask(ctx->lora, 4096);
    }

#ifdef HAS_GPS
    if (ctx->gps && ctx->config->beacon.gpsEnabled) {
        setupLocationTask(ctx->gps, 4096);
    }
#endif

    NetworkManager networkManager(ctx->config, ctx->sd, ctx->ble);
    networkManager.setup();

    g_state = STATE_OPERATIONAL;
    LOGI(TAG, "Worker tasks spawned");
}

static void runOperationalLoop(SupervisorContext* ctx) {
    LoRaPacket pkt;
    DisplayState currentState  = {};
    DisplayState lastDisplayed = {};
    currentState.beaconState   = STATE_OPERATIONAL;
    TickType_t lastDisplayUpdate = 0;
    const TickType_t displayUpdateInterval = pdMS_TO_TICKS(5000);

#ifdef HAS_PMU
    PMUManager pmuManager;
#endif

#ifdef HAS_GPS
    GPSManager gpsManager(locationTaskUpdateQueue);
#endif

    while (true) {
#ifdef HAS_PMU
        if (pmuManager.updatePMU()) {
            currentState.batteryVoltage = pmuManager.getState().battery_voltage;
        }
#endif

#ifdef HAS_GPS
        if (gpsManager.updateGPS()) {
            const GPSData& gpsData = gpsManager.getData();
            currentState.latitude      = gpsData.position.latitude;
            currentState.longitude     = gpsData.position.longitude;
            currentState.satelliteCount = gpsData.metadata.satelliteCount;
            currentState.altitude      = gpsData.metadata.altitude;
        }
#endif

        while (loraUpdateQueue && xQueueReceive(loraUpdateQueue, &pkt, 0) == pdPASS) {
            pktCount++;
            bool isSentPacket = (pkt.rssi == 0 && pkt.snr == 0.0f);

            currentState.packetCount     = pktCount;
            currentState.lastRssi        = pkt.rssi;
            currentState.lastSnr         = pkt.snr;
            currentState.lastPacketWasSent = isSentPacket;
            memcpy(currentState.receivedData, pkt.data, pkt.length);
            currentState.receivedData[pkt.length] = '\0';

            if (isSentPacket) {
                LOGI(TAG, "Packet #%d sent\n%.*s", pktCount, pkt.length, (const char*)pkt.data);
            } else {
                LOGI(TAG, "#%d Received packet: RSSI=%d dBm, SNR=%.2f dB, len=%d",
                     pktCount, pkt.rssi, pkt.snr, pkt.length);
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (now - lastDisplayUpdate >= displayUpdateInterval) {
            if (currentState != lastDisplayed) {
#ifdef DISPLAY_MODEL
                display.clearBuffer();

                if (currentState.packetCount > 0) {
                    if (currentState.lastPacketWasSent) {
                        display.drawLine("Packet #%d sent", currentState.packetCount);
                    } else {
                        display.drawLine("#%d, RSSI: %d, SNR: %.2f",
                            currentState.packetCount, currentState.lastRssi, currentState.lastSnr);
                        display.drawLine((char*)currentState.receivedData);
                    }
                }

#ifdef HAS_PMU
                pmuManager.logPMU(&display);
#endif

#ifdef HAS_GPS
                gpsManager.logGPS(&display);
#endif

                display.sendBuffer();
#endif
                lastDisplayed = currentState;
            }
            lastDisplayUpdate = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

bool setupMainTask(SupervisorContext* ctx, uint32_t stackDepth) {
    g_ctx = ctx;

    BaseType_t result = xTaskCreatePinnedToCore(
        mainTask,
        "MainTask",
        stackDepth,
        ctx,
        10,
        nullptr,
        0
    );

    if (result == pdPASS) {
        LOGI(TAG, "Main task created on core 0 (priority 10)");
        return true;
    } else {
        LOGE(TAG, "Failed to create main task");
        return false;
    }
}

void mainTask(void* pvParameters) {
    LOGI(TAG, "Main task starting...");

    SupervisorContext* ctx = static_cast<SupervisorContext*>(pvParameters);

    // Determine initial state from config
    if (ctx->config && ctx->config->isSetUp()) {
        LOGI(TAG, "WiFi configured — booting directly into operational mode");
        enterOperational(ctx);
    } else {
        LOGI(TAG, "No WiFi configured — entering setup/provisioning mode");
        enterSetup(ctx);
    }

    // State machine loop
    while (g_state != STATE_OPERATIONAL) {
        switch (g_state.load()) {
            case STATE_PROVISIONING: {
#ifdef DISPLAY_MODEL
                static bool provDisplayShown = false;
                if (!provDisplayShown) {
                    display.clearBuffer();
                    display.drawLine("Awaiting BLE config");
                    display.sendBuffer();
                    provDisplayShown = true;
                }
#endif
                if (g_credsReceived) {
                    LOGI(TAG, "Credentials received: SSID='%s'", g_pendingSsid);
                    // Write credentials into config
                    strncpy(ctx->config->wifi[0].ssid, g_pendingSsid, MAX_SSID_LENGTH - 1);
                    ctx->config->wifi[0].ssid[MAX_SSID_LENGTH - 1] = '\0';
                    strncpy(ctx->config->wifi[0].password, g_pendingPassword, MAX_PASSWORD_LENGTH - 1);
                    ctx->config->wifi[0].password[MAX_PASSWORD_LENGTH - 1] = '\0';
                    ctx->config->wifi[0].enabled = true;
                    g_credsReceived = false;

#ifdef HAS_SDCARD
                    if (ctx->sd && ctx->sd->isInitialized()) {
                        if (ctx->sd->saveConfig(ctx->config)) {
                            LOGI(TAG, "Config saved to SD");
                        } else {
                            LOGE(TAG, "Failed to save config to SD");
                        }
                    }
#endif
                    g_state = STATE_TRANSITIONING;
                }
                break;
            }

            case STATE_TRANSITIONING: {
                LOGI(TAG, "Transitioning: entering operational mode");
                enterOperational(ctx);
                break;
            }

            default:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Once operational, run the packet-processing loop (never returns)
    runOperationalLoop(ctx);
}
