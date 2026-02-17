#include <Arduino.h>
#include <unity.h>
#include "PlaceNetConfig.h"

static PlaceNetConfig config;

void setUp(void) {
    config = PlaceNetConfig();
}

void tearDown(void) {}

void test_defaults_beacon_interval(void) {
    TEST_ASSERT_EQUAL_UINT32(30000, config.beacon.beaconIntervalMs);
}

void test_defaults_lora_enabled(void) {
    TEST_ASSERT_TRUE(config.beacon.loraEnabled);
}

void test_defaults_mqtt_disabled(void) {
    TEST_ASSERT_FALSE(config.mqtt.enabled);
}

void test_defaults_wifi_disabled(void) {
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        TEST_ASSERT_FALSE(config.wifi[i].enabled);
    }
}

void test_validate_default_passes(void) {
    TEST_ASSERT_TRUE(config.validate());
}

void test_validate_bad_wifi_fails(void) {
    config.wifi[0].enabled = true;
    // empty SSID
    TEST_ASSERT_FALSE(config.validate());
}

void test_reset_restores_defaults(void) {
    config.beacon.beaconIntervalMs = 999;
    config.mqtt.port = 5555;
    config.reset();
    TEST_ASSERT_EQUAL_UINT32(30000, config.beacon.beaconIntervalMs);
    TEST_ASSERT_EQUAL_UINT16(1883, config.mqtt.port);
}

void setup() {
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(test_defaults_beacon_interval);
    RUN_TEST(test_defaults_lora_enabled);
    RUN_TEST(test_defaults_mqtt_disabled);
    RUN_TEST(test_defaults_wifi_disabled);
    RUN_TEST(test_validate_default_passes);
    RUN_TEST(test_validate_bad_wifi_fails);
    RUN_TEST(test_reset_restores_defaults);

    UNITY_END();
}

void loop() {}
