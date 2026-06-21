#include "main_task.h"
#include "lora_task.h"
#include "managers/network_manager.h"
#include "LoRaModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include <esp_system.h>
#include <esp_attr.h>

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

static const uint32_t FORCE_SETUP_MAGIC = 0x5E7C0DEu; // arbitrary value
RTC_NOINIT_ATTR static uint32_t g_forceSetupMagic;

void requestSetupModeReboot() {
    LOGW(TAG, "Setup-mode reboot requested — restarting into BLE provisioning");
    g_forceSetupMagic = FORCE_SETUP_MAGIC;
    vTaskDelay(pdMS_TO_TICKS(100));  // give the log line time to flush over UART
    esp_restart();
}

// Credentials received from BLE callback (set from BLE ISR/task context)
static volatile bool g_credsReceived = false;
static char g_pendingSsid[MAX_SSID_LENGTH]     = {};
static char g_pendingPassword[MAX_PASSWORD_LENGTH] = {};

// Radio / server config received from BLE callbacks (BLE task context).
static volatile bool    g_loraCfgReceived   = false;
static BLELoRaConfig    g_pendingLora       = {};
static volatile bool    g_serverCfgReceived = false;
static BLEServerConfig  g_pendingServer     = {};

static void onCredentialsReceived(const char* ssid, const char* pass) {
    strncpy(g_pendingSsid, ssid, MAX_SSID_LENGTH - 1);
    g_pendingSsid[MAX_SSID_LENGTH - 1] = '\0';
    strncpy(g_pendingPassword, pass, MAX_PASSWORD_LENGTH - 1);
    g_pendingPassword[MAX_PASSWORD_LENGTH - 1] = '\0';
    g_credsReceived = true;
}

static void onLoRaConfigReceived(const BLELoRaConfig& cfg) {
    g_pendingLora     = cfg;
    g_loraCfgReceived = true;
}

static void onServerConfigReceived(const BLEServerConfig& cfg) {
    g_pendingServer     = cfg;
    g_serverCfgReceived = true;
}

static void enterSetup(SupervisorContext* ctx) {
    LOGI(TAG, "Entering SETUP state: starting BLE provisioning");
    ctx->ble->setCredentialsCallback(onCredentialsReceived);
    ctx->ble->setLoRaConfigCallback(onLoRaConfigReceived);
    ctx->ble->setServerConfigCallback(onServerConfigReceived);
    ctx->ble->init();

    // Seed the readable config characteristics with the values currently in
    // effect so the app shows the real configuration on connect.
    ctx->ble->setCurrentLoRaConfig(ctx->config->lora.frequency,
                                   ctx->config->lora.bandwidth,
                                   ctx->config->lora.spreadingFactor,
                                   ctx->config->lora.codingRate,
                                   ctx->config->lora.syncWord);
    ctx->ble->setCurrentServerConfig(ctx->config->httpServer.url,
                                     ctx->config->httpServer.port);

    ctx->ble->startAdvertising();
    g_state = STATE_PROVISIONING;
    LOGI(TAG, "BLE advertising started, waiting for credentials");
}

static void enterOperational(SupervisorContext* ctx) {
    LOGI(TAG, "Entering OPERATIONAL state");

    if (ctx->lora) {
        // Apply the provisioned radio PHY profile before the radio is brought up.
        ctx->lora->setRadioParams(ctx->config->lora.frequency,
                                  ctx->config->lora.bandwidth,
                                  ctx->config->lora.spreadingFactor,
                                  ctx->config->lora.codingRate,
                                  ctx->config->lora.syncWord);
        setupLoRaTask(ctx->lora, 4096);
    }

#ifdef HAS_GPS
    if (ctx->gps && ctx->config->beacon.gpsEnabled) {
        setupLocationTask(ctx->gps, 4096);
    }
#endif

    NetworkManager networkManager(ctx->config, ctx->sd, ctx->ble);
    networkManager.setup();
    ctx->mqtt = networkManager.takeMqttManager();

    g_state = STATE_OPERATIONAL;
    LOGI(TAG, "Worker tasks spawned");
}

static void runOperationalLoop(SupervisorContext* ctx) {
    LoRaRxMsg    rxMsg;
    DisplayState currentState  = {};
    DisplayState lastDisplayed = {};
    currentState.beaconState   = STATE_OPERATIONAL;
    TickType_t lastDisplayUpdate = 0;
    const TickType_t displayUpdateInterval = pdMS_TO_TICKS(5000);
    TickType_t lastAliveTime = 0;
    const TickType_t aliveInterval = pdMS_TO_TICKS(60000);

#ifdef HAS_PMU
    PMUManager pmuManager;
#endif

#ifdef HAS_GPS
    GPSManager gpsManager(locationTaskUpdateQueue);
#endif

    while (true) {
        if (ctx->mqtt) {
            ctx->mqtt->loop();
        }

#ifdef HAS_PMU
        if (pmuManager.updatePMU()) {
            currentState.batteryVoltage = pmuManager.getState().battery_voltage;
        }
#endif

#ifdef HAS_GPS
        gpsManager.updateGPS();  // keep module ticking; GPS no longer shown on display
#endif

        while (loraRxQueue && xQueueReceive(loraRxQueue, &rxMsg, 0) == pdPASS) {
            pktCount++;

            if (rxMsg.isEcho) {
                currentState.sentCount++;
                LOGI(TAG, "Packet #%d sent (payloadType=%d)", pktCount, rxMsg.packet.payloadType);
            } else {
                currentState.receivedCount++;
                currentState.lastRssi        = rxMsg.rssi;
                currentState.lastSnr         = rxMsg.snr;
                currentState.lastPayloadType = rxMsg.packet.payloadType;

                if (rxMsg.packet.payloadType == meshcore::PAYLOAD_TYPE_ADVERT) {
                    meshcore::Advert advert;
                    if (ctx->lora->parseAdvert(rxMsg.packet, advert)) {
                        strncpy(currentState.lastAdvertName, advert.name,
                                sizeof(currentState.lastAdvertName) - 1);
                        currentState.lastAdvertName[sizeof(currentState.lastAdvertName) - 1] = '\0';
                        LOGI(TAG, "#%d RX ADVERT: name='%s' type=%d RSSI=%d dBm SNR=%.2f dB",
                             pktCount, advert.name, advert.type, rxMsg.rssi, rxMsg.snr);
                    } else {
                        currentState.lastAdvertName[0] = '\0';
                        LOGW(TAG, "#%d RX ADVERT: parse failed", pktCount);
                    }
                } else {
                    currentState.lastAdvertName[0] = '\0';
                    LOGI(TAG, "#%d RX packet: payloadType=%d RSSI=%d dBm SNR=%.2f dB payloadLen=%d",
                         pktCount, rxMsg.packet.payloadType, rxMsg.rssi, rxMsg.snr, rxMsg.packet.payloadLen);
                }
            }
        }

        TickType_t now = xTaskGetTickCount();

        if (now - lastAliveTime >= aliveInterval) {
            if (ctx->mqtt) {
                ctx->mqtt->publishAlive();
            }
            lastAliveTime = now;
        }

        if (now - lastDisplayUpdate >= displayUpdateInterval) {
            if (currentState != lastDisplayed) {
#ifdef DISPLAY_MODEL
                display.clearBuffer();

#ifdef HAS_PMU
                display.drawLine("Bat: %u mV", currentState.batteryVoltage);
#endif
                display.drawLine("Sent: #%d", currentState.sentCount);
                display.drawLine("Recv: #%d", currentState.receivedCount);
                display.drawLine("RSSI: %d  SNR: %.1f", currentState.lastRssi, currentState.lastSnr);
                display.drawLine("Type: %d", currentState.lastPayloadType);
                display.drawLine("Name: %s",
                    currentState.lastAdvertName[0] ? currentState.lastAdvertName : "-");

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

    // check to see if setup state has been reinforced
    bool forceSetup = (g_forceSetupMagic == FORCE_SETUP_MAGIC);
    g_forceSetupMagic = 0;

    if (!forceSetup && ctx->config && ctx->config->isReadyForOperation()) {
        LOGI(TAG, "Config valid — booting directly into operational mode");
        enterOperational(ctx);
    } else {
        if (forceSetup) {
            LOGI(TAG, "Setup mode forced by button gesture");
        } else {
            LOGI(TAG, "Config invalid or incomplete — entering setup/provisioning mode");
        }
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
                // Apply radio config as soon as it arrives so it persists even
                // if the user configures it without (re)sending WiFi credentials.
                if (g_loraCfgReceived) {
                    ctx->config->lora.frequency       = g_pendingLora.frequency;
                    ctx->config->lora.bandwidth       = g_pendingLora.bandwidth;
                    ctx->config->lora.spreadingFactor = g_pendingLora.spreadingFactor;
                    ctx->config->lora.codingRate      = g_pendingLora.codingRate;
                    ctx->config->lora.syncWord        = g_pendingLora.syncWord;
                    g_loraCfgReceived = false;
                    LOGI(TAG, "Applied LoRa config from BLE");
#ifdef HAS_SDCARD
                    if (ctx->sd && ctx->sd->isInitialized()) {
                        ctx->sd->saveConfig(ctx->config);
                    }
#endif
                }

                if (g_serverCfgReceived) {
                    strncpy(ctx->config->httpServer.url, g_pendingServer.address,
                            MAX_HTTP_SERVER_LENGTH - 1);
                    ctx->config->httpServer.url[MAX_HTTP_SERVER_LENGTH - 1] = '\0';
                    ctx->config->httpServer.port = g_pendingServer.port;
                    ctx->config->httpServer.enabled =
                        strlen(ctx->config->httpServer.url) > 0;
                    g_serverCfgReceived = false;
                    LOGI(TAG, "Applied server config from BLE: %s:%u",
                         ctx->config->httpServer.url, ctx->config->httpServer.port);
#ifdef HAS_SDCARD
                    if (ctx->sd && ctx->sd->isInitialized()) {
                        ctx->sd->saveConfig(ctx->config);
                    }
#endif
                }

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
