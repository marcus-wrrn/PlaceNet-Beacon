#include "display_task.h"

#ifdef DISPLAY_MODEL

#include "DisplayModule.h"
#include "DisplayEvents.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "DISPLAY_TASK";

#define BOOT_SPLASH_DURATION_MS 3000
#define PACKET_VIEW_DURATION_MS 5000
#define LOW_BATTERY_THRESHOLD_MV 3300
#define STATUS_UPDATE_INTERVAL_MS 100

static void renderBootSplash(DISPLAY_MODEL* u8g2) {
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_inb19_mr);
    u8g2->drawStr(0, 30, "PlaceNet");
    u8g2->drawHLine(2, 50, 47);
    u8g2->drawHLine(3, 54, 47);
    u8g2->setFont(u8g2_font_inb19_mf);
    u8g2->drawStr(30, 60, "Beacon");
    u8g2->sendBuffer();
}

static void renderStatusScreen(DISPLAY_MODEL* u8g2, const DisplayState* state) {
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_fur11_tf);

    char buffer[64];
    uint8_t y_pos = 12;

    unsigned long uptimeSeconds = (millis() - state->boot_time) / 1000;
    unsigned long uptimeMinutes = uptimeSeconds / 60;
    unsigned long uptimeHours = uptimeMinutes / 60;

    if (uptimeHours > 0) {
        snprintf(buffer, sizeof(buffer), "Up: %luh %lum", uptimeHours, uptimeMinutes % 60);
    } else if (uptimeMinutes > 0) {
        snprintf(buffer, sizeof(buffer), "Up: %lum %lus", uptimeMinutes, uptimeSeconds % 60);
    } else {
        snprintf(buffer, sizeof(buffer), "Up: %lus", uptimeSeconds);
    }
    u8g2->drawStr(0, y_pos, buffer);
    y_pos += 12;

    if (state->battery_present) {
        float voltage = state->battery_voltage / 1000.0f;
        if (state->battery_charging) {
            snprintf(buffer, sizeof(buffer), "Bat: %.2fV CHG", voltage);
        } else {
            snprintf(buffer, sizeof(buffer), "Bat: %.2fV", voltage);
        }
        u8g2->drawStr(0, y_pos, buffer);
        y_pos += 12;
    }

#ifdef LORA_MODE_RECEIVER
    snprintf(buffer, sizeof(buffer), "RX: %lu pkts", state->rx_packet_count);
    u8g2->drawStr(0, y_pos, buffer);
    y_pos += 12;

    if (state->rx_packet_count > 0) {
        snprintf(buffer, sizeof(buffer), "RSSI: %ddBm", state->last_rssi);
        u8g2->drawStr(0, y_pos, buffer);
        y_pos += 12;

        snprintf(buffer, sizeof(buffer), "SNR: %.1fdB", state->last_snr);
        u8g2->drawStr(0, y_pos, buffer);
    }
#endif

#ifdef LORA_MODE_BEACON
    snprintf(buffer, sizeof(buffer), "TX: %lu beacons", state->tx_beacon_count);
    u8g2->drawStr(0, y_pos, buffer);
    y_pos += 12;

    if (state->duty_cycle_1m > 0.0f) {
        snprintf(buffer, sizeof(buffer), "DC 1m: %.2f%%", state->duty_cycle_1m);
        u8g2->drawStr(0, y_pos, buffer);
    }
#endif

    u8g2->sendBuffer();
}

static void renderPacketViewScreen(DISPLAY_MODEL* u8g2, const DisplayState* state) {
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_fur11_tf);

    char buffer[64];

    u8g2->drawStr(0, 12, "PACKET RX:");

    memset(buffer, 0, sizeof(buffer));
    uint8_t displayLen = (state->last_packet_length < 20) ? state->last_packet_length : 20;
    memcpy(buffer, state->last_packet_data, displayLen);
    buffer[displayLen] = '\0';

    for (uint8_t i = 0; i < displayLen; i++) {
        if (buffer[i] < 32 || buffer[i] > 126) {
            buffer[i] = '.';
        }
    }

    u8g2->drawStr(0, 28, buffer);

    snprintf(buffer, sizeof(buffer), "RSSI: %ddBm", state->last_rssi);
    u8g2->drawStr(0, 44, buffer);

    snprintf(buffer, sizeof(buffer), "SNR: %.1fdB L:%d", state->last_snr, state->last_packet_length);
    u8g2->drawStr(0, 60, buffer);

    u8g2->sendBuffer();
}

static void renderLowBatteryScreen(DISPLAY_MODEL* u8g2, const DisplayState* state) {
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_inb19_mr);

    u8g2->drawStr(5, 30, "LOW");
    u8g2->drawStr(5, 55, "BATTERY");

    u8g2->setFont(u8g2_font_fur11_tf);
    char buffer[32];
    float voltage = state->battery_voltage / 1000.0f;
    snprintf(buffer, sizeof(buffer), "%.2fV", voltage);
    u8g2->drawStr(45, 60, buffer);

    u8g2->sendBuffer();
}

static void updateStateFromEvent(DisplayState* state, const DisplayEvent* evt) {
    switch (evt->type) {
        case DISP_EVT_BATTERY_UPDATE:
            state->battery_voltage = evt->data.battery.voltage_mv;
            state->battery_charging = evt->data.battery.charging;
            state->battery_present = evt->data.battery.has_battery;
            LOGD(TAG, "Battery: %dmV %s", state->battery_voltage,
                 state->battery_charging ? "CHG" : "");
            break;

        case DISP_EVT_LORA_PACKET_RX:
            state->last_rssi = evt->data.lora_rx.rssi;
            state->last_snr = evt->data.lora_rx.snr;
            state->last_packet_length = evt->data.lora_rx.length;
            state->rx_packet_count = evt->data.lora_rx.packet_count;
            memcpy(state->last_packet_data, evt->data.lora_rx.data,
                   evt->data.lora_rx.length);
            state->last_packet_data[evt->data.lora_rx.length] = '\0';
            LOGI(TAG, "LoRa RX #%lu: RSSI=%d SNR=%.1f Len=%d",
                 state->rx_packet_count, state->last_rssi, state->last_snr,
                 state->last_packet_length);
            break;

        case DISP_EVT_LORA_PACKET_TX:
            state->tx_beacon_count = evt->data.lora_tx.beacon_count;
            state->duty_cycle_1m = evt->data.lora_tx.duty_cycle_1m;
            state->duty_cycle_10m = evt->data.lora_tx.duty_cycle_10m;
            LOGI(TAG, "LoRa TX #%lu: DC 1m=%.2f%% 10m=%.2f%%",
                 state->tx_beacon_count, state->duty_cycle_1m, state->duty_cycle_10m);
            break;

        case DISP_EVT_GPS_UPDATE:
            state->gps_has_fix = evt->data.gps.has_fix;
            state->gps_lat = evt->data.gps.latitude;
            state->gps_lon = evt->data.gps.longitude;
            state->gps_sats = evt->data.gps.satellites;
            break;

        case DISP_EVT_MODE_CHANGE:
            state->current_mode = evt->data.mode;
            LOGI(TAG, "Mode changed to: %d", state->current_mode);
            break;

        default:
            break;
    }
}

static void updateDisplayMode(DisplayState* state) {
    uint32_t now = millis();

    if (state->current_mode == DISP_MODE_BOOT_SPLASH) {
        if (now - state->boot_time >= BOOT_SPLASH_DURATION_MS) {
            state->current_mode = DISP_MODE_STATUS;
            state->mode_timeout_ms = 0;
            LOGI(TAG, "Boot splash complete, switching to status");
        }
        return;
    }

    if (state->battery_present && state->battery_voltage < LOW_BATTERY_THRESHOLD_MV) {
        if (state->current_mode != DISP_MODE_LOW_BATTERY) {
            state->current_mode = DISP_MODE_LOW_BATTERY;
            state->mode_timeout_ms = 0;
            LOGI(TAG, "Low battery detected, switching to warning screen");
        }
        return;
    }

    if (state->current_mode == DISP_MODE_LOW_BATTERY &&
        state->battery_voltage >= LOW_BATTERY_THRESHOLD_MV + 100) {
        state->current_mode = DISP_MODE_STATUS;
        state->mode_timeout_ms = 0;
        LOGI(TAG, "Battery recovered, switching to status");
        return;
    }

    if (state->current_mode == DISP_MODE_PACKET_VIEW) {
        if (state->mode_timeout_ms > 0 && now >= state->mode_timeout_ms) {
            state->current_mode = DISP_MODE_STATUS;
            state->mode_timeout_ms = 0;
            LOGI(TAG, "Packet view timeout, returning to status");
        }
    }
}

void displayTask(void* pvParameters) {
    DisplayTaskParams* params = static_cast<DisplayTaskParams*>(pvParameters);

    if (!params) {
        LOGE(TAG, "Display task parameters are null, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    if (!params->eventQueue) {
        LOGE(TAG, "Event queue is null, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "Display task started - initializing display module");

    DisplayModule display;
    if (!display.init()) {
        LOGE(TAG, "Display hardware initialization failed, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "Display task - event-driven architecture ready");

    DisplayState state;
    initDisplayState(&state);

    QueueHandle_t eventQueue = params->eventQueue;
    DISPLAY_MODEL* u8g2 = display.getDisplay();

    if (!u8g2) {
        LOGE(TAG, "U8g2 display is null!");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "Rendering boot splash...");
    renderBootSplash(u8g2);
    LOGI(TAG, "Boot splash rendered");

    DisplayEvent evt;
    TickType_t lastStatusUpdate = xTaskGetTickCount();

    uint32_t loopCount = 0;
    while (1) {
        bool eventReceived = false;

        if (xQueueReceive(eventQueue, &evt, pdMS_TO_TICKS(10)) == pdTRUE) {
            updateStateFromEvent(&state, &evt);
            eventReceived = true;

            if (evt.type == DISP_EVT_LORA_PACKET_RX &&
                state.current_mode != DISP_MODE_LOW_BATTERY) {
                state.current_mode = DISP_MODE_PACKET_VIEW;
                state.mode_timeout_ms = millis() + PACKET_VIEW_DURATION_MS;
                LOGI(TAG, "New packet, switching to packet view");
            }
        }

        updateDisplayMode(&state);

        TickType_t now = xTaskGetTickCount();
        if (eventReceived || (now - lastStatusUpdate >= pdMS_TO_TICKS(STATUS_UPDATE_INTERVAL_MS))) {
            lastStatusUpdate = now;

            loopCount++;
            if (loopCount % 50 == 0) {
                LOGD(TAG, "Display loop #%lu, mode=%d, bat=%dmV", loopCount, state.current_mode, state.battery_voltage);
            }

            switch (state.current_mode) {
                case DISP_MODE_BOOT_SPLASH:
                    break;

                case DISP_MODE_STATUS:
                    renderStatusScreen(u8g2, &state);
                    break;

                case DISP_MODE_PACKET_VIEW:
                    renderPacketViewScreen(u8g2, &state);
                    break;

                case DISP_MODE_LOW_BATTERY:
                    renderLowBatteryScreen(u8g2, &state);
                    break;
            }
        }
    }
}

#endif
