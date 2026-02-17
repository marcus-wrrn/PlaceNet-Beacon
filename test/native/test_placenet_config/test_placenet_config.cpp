#include <unity.h>
#include "PlaceNetConfig.h"

static PlaceNetConfig config;

void setUp(void) {
    config = PlaceNetConfig();
}

void tearDown(void) {}

// --- WiFiCredentials defaults ---

void test_wifi_defaults_ssid_empty(void) {
    TEST_ASSERT_EQUAL_STRING("", config.wifi[0].ssid);
}

void test_wifi_defaults_password_empty(void) {
    TEST_ASSERT_EQUAL_STRING("", config.wifi[0].password);
}

void test_wifi_defaults_disabled(void) {
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        TEST_ASSERT_FALSE(config.wifi[i].enabled);
    }
}

// --- MQTTConfig defaults ---

void test_mqtt_defaults_port(void) {
    TEST_ASSERT_EQUAL_UINT16(1883, config.mqtt.port);
}

void test_mqtt_defaults_disabled(void) {
    TEST_ASSERT_FALSE(config.mqtt.enabled);
}

void test_mqtt_defaults_tls_off(void) {
    TEST_ASSERT_FALSE(config.mqtt.useTLS);
}

void test_mqtt_defaults_broker_empty(void) {
    TEST_ASSERT_EQUAL_STRING("", config.mqtt.broker);
}

// --- HTTPServerConfig defaults ---

void test_http_defaults_port(void) {
    TEST_ASSERT_EQUAL_UINT16(80, config.httpServer.port);
}

void test_http_defaults_disabled(void) {
    TEST_ASSERT_FALSE(config.httpServer.enabled);
}

void test_http_defaults_url_empty(void) {
    TEST_ASSERT_EQUAL_STRING("", config.httpServer.url);
}

// --- BeaconConfig defaults ---

void test_beacon_defaults_interval(void) {
    TEST_ASSERT_EQUAL_UINT32(30000, config.beacon.beaconIntervalMs);
}

void test_beacon_defaults_lora_enabled(void) {
    TEST_ASSERT_TRUE(config.beacon.loraEnabled);
}

void test_beacon_defaults_gps_enabled(void) {
    TEST_ASSERT_TRUE(config.beacon.gpsEnabled);
}

void test_beacon_defaults_ble_disabled(void) {
    TEST_ASSERT_FALSE(config.beacon.bleEnabled);
}

// --- reset() ---

void test_reset_restores_defaults(void) {
    strcpy(config.wifi[0].ssid, "MyNetwork");
    config.wifi[0].enabled = true;
    config.mqtt.port = 9999;
    config.beacon.beaconIntervalMs = 1;

    config.reset();

    TEST_ASSERT_EQUAL_STRING("", config.wifi[0].ssid);
    TEST_ASSERT_FALSE(config.wifi[0].enabled);
    TEST_ASSERT_EQUAL_UINT16(1883, config.mqtt.port);
    TEST_ASSERT_EQUAL_UINT32(30000, config.beacon.beaconIntervalMs);
}

// --- validate() ---

void test_validate_default_config_passes(void) {
    TEST_ASSERT_TRUE(config.validate());
}

void test_validate_valid_wifi(void) {
    strcpy(config.wifi[0].ssid, "TestNetwork");
    strcpy(config.wifi[0].password, "password123");
    config.wifi[0].enabled = true;
    TEST_ASSERT_TRUE(config.validate());
}

void test_validate_wifi_empty_ssid_fails(void) {
    config.wifi[0].enabled = true;
    // ssid is empty by default
    TEST_ASSERT_FALSE(config.validate());
}

void test_validate_wifi_short_password_fails(void) {
    strcpy(config.wifi[0].ssid, "TestNetwork");
    strcpy(config.wifi[0].password, "short");  // < 8 chars
    config.wifi[0].enabled = true;
    TEST_ASSERT_FALSE(config.validate());
}

void test_validate_wifi_open_network_passes(void) {
    strcpy(config.wifi[0].ssid, "OpenNet");
    // password left empty (open network)
    config.wifi[0].enabled = true;
    TEST_ASSERT_TRUE(config.validate());
}

void test_validate_mqtt_enabled_no_broker_fails(void) {
    config.mqtt.enabled = true;
    // broker is empty
    TEST_ASSERT_FALSE(config.validate());
}

void test_validate_mqtt_enabled_valid(void) {
    config.mqtt.enabled = true;
    strcpy(config.mqtt.broker, "mqtt.example.com");
    config.mqtt.port = 1883;
    TEST_ASSERT_TRUE(config.validate());
}

void test_validate_mqtt_port_zero_fails(void) {
    config.mqtt.enabled = true;
    strcpy(config.mqtt.broker, "mqtt.example.com");
    config.mqtt.port = 0;
    TEST_ASSERT_FALSE(config.validate());
}

void test_validate_http_enabled_no_url_fails(void) {
    config.httpServer.enabled = true;
    // url is empty
    TEST_ASSERT_FALSE(config.validate());
}

void test_validate_http_enabled_valid(void) {
    config.httpServer.enabled = true;
    strcpy(config.httpServer.url, "http://example.com");
    config.httpServer.port = 8080;
    TEST_ASSERT_TRUE(config.validate());
}

void test_validate_http_port_zero_fails(void) {
    config.httpServer.enabled = true;
    strcpy(config.httpServer.url, "http://example.com");
    config.httpServer.port = 0;
    TEST_ASSERT_FALSE(config.validate());
}

int main(int argc, char** argv) {
    UNITY_BEGIN();

    // WiFi defaults
    RUN_TEST(test_wifi_defaults_ssid_empty);
    RUN_TEST(test_wifi_defaults_password_empty);
    RUN_TEST(test_wifi_defaults_disabled);

    // MQTT defaults
    RUN_TEST(test_mqtt_defaults_port);
    RUN_TEST(test_mqtt_defaults_disabled);
    RUN_TEST(test_mqtt_defaults_tls_off);
    RUN_TEST(test_mqtt_defaults_broker_empty);

    // HTTP defaults
    RUN_TEST(test_http_defaults_port);
    RUN_TEST(test_http_defaults_disabled);
    RUN_TEST(test_http_defaults_url_empty);

    // Beacon defaults
    RUN_TEST(test_beacon_defaults_interval);
    RUN_TEST(test_beacon_defaults_lora_enabled);
    RUN_TEST(test_beacon_defaults_gps_enabled);
    RUN_TEST(test_beacon_defaults_ble_disabled);

    // reset()
    RUN_TEST(test_reset_restores_defaults);

    // validate()
    RUN_TEST(test_validate_default_config_passes);
    RUN_TEST(test_validate_valid_wifi);
    RUN_TEST(test_validate_wifi_empty_ssid_fails);
    RUN_TEST(test_validate_wifi_short_password_fails);
    RUN_TEST(test_validate_wifi_open_network_passes);
    RUN_TEST(test_validate_mqtt_enabled_no_broker_fails);
    RUN_TEST(test_validate_mqtt_enabled_valid);
    RUN_TEST(test_validate_mqtt_port_zero_fails);
    RUN_TEST(test_validate_http_enabled_no_url_fails);
    RUN_TEST(test_validate_http_enabled_valid);
    RUN_TEST(test_validate_http_port_zero_fails);

    return UNITY_END();
}
