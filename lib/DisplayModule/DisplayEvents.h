#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

enum DisplayEventType {
    DISP_EVT_BATTERY_UPDATE,
    DISP_EVT_LORA_PACKET_RX,
    DISP_EVT_LORA_PACKET_TX,
    DISP_EVT_GPS_UPDATE,
    DISP_EVT_SYSTEM_BOOT,
    DISP_EVT_MODE_CHANGE
};

enum DisplayMode {
    DISP_MODE_STATUS,
    DISP_MODE_PACKET_VIEW,
    DISP_MODE_LOW_BATTERY,
    DISP_MODE_BOOT_SPLASH
};

struct BatteryEventData {
    uint16_t voltage_mv;
    bool charging;
    bool has_battery;
};

struct LoRaRxEventData {
    int16_t rssi;
    float snr;
    uint8_t length;
    char data[64];
    uint32_t packet_count;
};

struct LoRaTxEventData {
    bool success;
    uint32_t beacon_count;
    float duty_cycle_1m;
    float duty_cycle_10m;
};

struct GPSEventData {
    bool has_fix;
    float latitude;
    float longitude;
    uint8_t satellites;
};

struct DisplayEvent {
    DisplayEventType type;
    uint32_t timestamp;
    union {
        BatteryEventData battery;
        LoRaRxEventData lora_rx;
        LoRaTxEventData lora_tx;
        GPSEventData gps;
        DisplayMode mode;
    } data;
};

struct DisplayState {
    DisplayMode current_mode;
    uint32_t mode_timeout_ms;

    uint16_t battery_voltage;
    bool battery_charging;
    bool battery_present;

    int16_t last_rssi;
    float last_snr;
    uint32_t rx_packet_count;
    char last_packet_data[64];
    uint8_t last_packet_length;

    uint32_t tx_beacon_count;
    float duty_cycle_1m;
    float duty_cycle_10m;

    bool gps_has_fix;
    float gps_lat;
    float gps_lon;
    uint8_t gps_sats;

    uint32_t boot_time;
};

inline void initDisplayState(DisplayState* state) {
    if (!state) return;

    state->current_mode = DISP_MODE_BOOT_SPLASH;
    state->mode_timeout_ms = 0;

    state->battery_voltage = 0;
    state->battery_charging = false;
    state->battery_present = false;

    state->last_rssi = 0;
    state->last_snr = 0.0f;
    state->rx_packet_count = 0;
    memset(state->last_packet_data, 0, sizeof(state->last_packet_data));
    state->last_packet_length = 0;

    state->tx_beacon_count = 0;
    state->duty_cycle_1m = 0.0f;
    state->duty_cycle_10m = 0.0f;

    state->gps_has_fix = false;
    state->gps_lat = 0.0f;
    state->gps_lon = 0.0f;
    state->gps_sats = 0;

    state->boot_time = millis();
}

inline bool sendDisplayEvent(QueueHandle_t queue, const DisplayEvent& event) {
    if (!queue) return false;
    return xQueueSend(queue, &event, pdMS_TO_TICKS(10)) == pdTRUE;
}

inline DisplayEvent createBatteryEvent(uint16_t voltage_mv, bool charging, bool has_battery) {
    DisplayEvent evt;
    evt.type = DISP_EVT_BATTERY_UPDATE;
    evt.timestamp = millis();
    evt.data.battery.voltage_mv = voltage_mv;
    evt.data.battery.charging = charging;
    evt.data.battery.has_battery = has_battery;
    return evt;
}

inline DisplayEvent createLoRaRxEvent(int16_t rssi, float snr, const uint8_t* payload,
                                       uint8_t length, uint32_t count) {
    DisplayEvent evt;
    evt.type = DISP_EVT_LORA_PACKET_RX;
    evt.timestamp = millis();
    evt.data.lora_rx.rssi = rssi;
    evt.data.lora_rx.snr = snr;
    evt.data.lora_rx.length = (length < 64) ? length : 63;
    evt.data.lora_rx.packet_count = count;

    memset(evt.data.lora_rx.data, 0, sizeof(evt.data.lora_rx.data));
    if (payload && length > 0) {
        memcpy(evt.data.lora_rx.data, payload, evt.data.lora_rx.length);
        evt.data.lora_rx.data[evt.data.lora_rx.length] = '\0';
    }

    return evt;
}

inline DisplayEvent createLoRaTxEvent(bool success, uint32_t count,
                                       float dc_1m, float dc_10m) {
    DisplayEvent evt;
    evt.type = DISP_EVT_LORA_PACKET_TX;
    evt.timestamp = millis();
    evt.data.lora_tx.success = success;
    evt.data.lora_tx.beacon_count = count;
    evt.data.lora_tx.duty_cycle_1m = dc_1m;
    evt.data.lora_tx.duty_cycle_10m = dc_10m;
    return evt;
}

inline DisplayEvent createGPSEvent(bool has_fix, float lat, float lon, uint8_t sats) {
    DisplayEvent evt;
    evt.type = DISP_EVT_GPS_UPDATE;
    evt.timestamp = millis();
    evt.data.gps.has_fix = has_fix;
    evt.data.gps.latitude = lat;
    evt.data.gps.longitude = lon;
    evt.data.gps.satellites = sats;
    return evt;
}
