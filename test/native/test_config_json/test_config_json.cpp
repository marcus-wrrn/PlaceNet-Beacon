#include <unity.h>
#include <ArduinoJson.h>
#include "PlaceNetConfig.h"

// Free-function replicas of SDCardModule::parseConfigJSON / createConfigJSON
// so we can test JSON logic without SD card hardware or #ifdef HAS_SDCARD.

static bool parseConfigJSON(const char* json, PlaceNetConfig* config) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);
    if (error) return false;

    config->reset();

    JsonArray wifiArray = doc["wifi"];
    if (wifiArray) {
        int index = 0;
        for (JsonObject wifiObj : wifiArray) {
            if (index >= MAX_WIFI_NETWORKS) break;
            const char* ssid = wifiObj["ssid"];
            const char* password = wifiObj["password"];
            bool enabled = wifiObj["enabled"] | false;
            if (ssid) strncpy(config->wifi[index].ssid, ssid, MAX_SSID_LENGTH - 1);
            if (password) strncpy(config->wifi[index].password, password, MAX_PASSWORD_LENGTH - 1);
            config->wifi[index].enabled = enabled;
            index++;
        }
    }

    JsonObject mqttObj = doc["mqtt"];
    if (mqttObj) {
        const char* broker = mqttObj["broker"];
        if (broker) strncpy(config->mqtt.broker, broker, MAX_MQTT_BROKER_LENGTH - 1);
        config->mqtt.port = mqttObj["port"] | 1883;
        const char* username = mqttObj["username"];
        if (username) strncpy(config->mqtt.username, username, MAX_PASSWORD_LENGTH - 1);
        const char* password = mqttObj["password"];
        if (password) strncpy(config->mqtt.password, password, MAX_PASSWORD_LENGTH - 1);
        const char* clientId = mqttObj["clientId"];
        if (clientId) strncpy(config->mqtt.clientId, clientId, MAX_MQTT_CLIENT_ID_LENGTH - 1);
        const char* baseTopic = mqttObj["baseTopic"];
        if (baseTopic) strncpy(config->mqtt.baseTopic, baseTopic, MAX_MQTT_TOPIC_LENGTH - 1);
        config->mqtt.enabled = mqttObj["enabled"] | false;
        config->mqtt.useTLS = mqttObj["useTLS"] | false;
    }

    JsonObject httpObj = doc["httpServer"];
    if (httpObj) {
        const char* url = httpObj["url"];
        if (url) strncpy(config->httpServer.url, url, MAX_HTTP_SERVER_LENGTH - 1);
        config->httpServer.port = httpObj["port"] | 80;
        config->httpServer.enabled = httpObj["enabled"] | false;
        config->httpServer.useTLS = httpObj["useTLS"] | false;
    }

    JsonObject beaconObj = doc["beacon"];
    if (beaconObj) {
        config->beacon.beaconIntervalMs = beaconObj["intervalMs"] | 30000;
        config->beacon.loraEnabled = beaconObj["loraEnabled"] | true;
        config->beacon.gpsEnabled = beaconObj["gpsEnabled"] | true;
        config->beacon.bleEnabled = beaconObj["bleEnabled"] | false;
    }

    return true;
}

static bool createConfigJSON(const PlaceNetConfig* config, char* buffer, size_t bufferSize) {
    JsonDocument doc;

    JsonArray wifiArray = doc["wifi"].to<JsonArray>();
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        if (config->wifi[i].enabled) {
            JsonObject wifiObj = wifiArray.add<JsonObject>();
            wifiObj["ssid"] = config->wifi[i].ssid;
            wifiObj["password"] = config->wifi[i].password;
            wifiObj["enabled"] = config->wifi[i].enabled;
        }
    }

    JsonObject mqttObj = doc["mqtt"].to<JsonObject>();
    mqttObj["broker"] = config->mqtt.broker;
    mqttObj["port"] = config->mqtt.port;
    mqttObj["username"] = config->mqtt.username;
    mqttObj["password"] = config->mqtt.password;
    mqttObj["clientId"] = config->mqtt.clientId;
    mqttObj["baseTopic"] = config->mqtt.baseTopic;
    mqttObj["enabled"] = config->mqtt.enabled;
    mqttObj["useTLS"] = config->mqtt.useTLS;

    JsonObject httpObj = doc["httpServer"].to<JsonObject>();
    httpObj["url"] = config->httpServer.url;
    httpObj["port"] = config->httpServer.port;
    httpObj["enabled"] = config->httpServer.enabled;
    httpObj["useTLS"] = config->httpServer.useTLS;

    JsonObject beaconObj = doc["beacon"].to<JsonObject>();
    beaconObj["intervalMs"] = config->beacon.beaconIntervalMs;
    beaconObj["loraEnabled"] = config->beacon.loraEnabled;
    beaconObj["gpsEnabled"] = config->beacon.gpsEnabled;
    beaconObj["bleEnabled"] = config->beacon.bleEnabled;

    size_t written = serializeJsonPretty(doc, buffer, bufferSize);
    if (written == 0 || written >= bufferSize - 1) return false;
    return true;
}

// --- Tests ---

static PlaceNetConfig config;

void setUp(void) {
    config = PlaceNetConfig();
}

void tearDown(void) {}

void test_parse_minimal_json(void) {
    const char* json = "{}";
    TEST_ASSERT_TRUE(parseConfigJSON(json, &config));
    // Should be reset to defaults
    TEST_ASSERT_EQUAL_UINT32(30000, config.beacon.beaconIntervalMs);
    TEST_ASSERT_FALSE(config.mqtt.enabled);
}

void test_parse_invalid_json(void) {
    const char* json = "not json at all";
    TEST_ASSERT_FALSE(parseConfigJSON(json, &config));
}

void test_parse_wifi_single(void) {
    const char* json = R"({
        "wifi": [
            {"ssid": "Home", "password": "secret1234", "enabled": true}
        ]
    })";
    TEST_ASSERT_TRUE(parseConfigJSON(json, &config));
    TEST_ASSERT_EQUAL_STRING("Home", config.wifi[0].ssid);
    TEST_ASSERT_EQUAL_STRING("secret1234", config.wifi[0].password);
    TEST_ASSERT_TRUE(config.wifi[0].enabled);
    TEST_ASSERT_FALSE(config.wifi[1].enabled);
}

void test_parse_mqtt_fields(void) {
    const char* json = R"({
        "mqtt": {
            "broker": "mqtt.example.com",
            "port": 8883,
            "username": "user",
            "password": "pass",
            "clientId": "beacon-01",
            "baseTopic": "placenet/beacons",
            "enabled": true,
            "useTLS": true
        }
    })";
    TEST_ASSERT_TRUE(parseConfigJSON(json, &config));
    TEST_ASSERT_EQUAL_STRING("mqtt.example.com", config.mqtt.broker);
    TEST_ASSERT_EQUAL_UINT16(8883, config.mqtt.port);
    TEST_ASSERT_EQUAL_STRING("user", config.mqtt.username);
    TEST_ASSERT_EQUAL_STRING("pass", config.mqtt.password);
    TEST_ASSERT_EQUAL_STRING("beacon-01", config.mqtt.clientId);
    TEST_ASSERT_EQUAL_STRING("placenet/beacons", config.mqtt.baseTopic);
    TEST_ASSERT_TRUE(config.mqtt.enabled);
    TEST_ASSERT_TRUE(config.mqtt.useTLS);
}

void test_parse_http_server(void) {
    const char* json = R"({
        "httpServer": {
            "url": "http://api.example.com",
            "port": 443,
            "enabled": true,
            "useTLS": true
        }
    })";
    TEST_ASSERT_TRUE(parseConfigJSON(json, &config));
    TEST_ASSERT_EQUAL_STRING("http://api.example.com", config.httpServer.url);
    TEST_ASSERT_EQUAL_UINT16(443, config.httpServer.port);
    TEST_ASSERT_TRUE(config.httpServer.enabled);
    TEST_ASSERT_TRUE(config.httpServer.useTLS);
}

void test_parse_beacon_fields(void) {
    const char* json = R"({
        "beacon": {
            "intervalMs": 60000,
            "loraEnabled": false,
            "gpsEnabled": false,
            "bleEnabled": true
        }
    })";
    TEST_ASSERT_TRUE(parseConfigJSON(json, &config));
    TEST_ASSERT_EQUAL_UINT32(60000, config.beacon.beaconIntervalMs);
    TEST_ASSERT_FALSE(config.beacon.loraEnabled);
    TEST_ASSERT_FALSE(config.beacon.gpsEnabled);
    TEST_ASSERT_TRUE(config.beacon.bleEnabled);
}

void test_create_json_default_config(void) {
    char buf[4096];
    TEST_ASSERT_TRUE(createConfigJSON(&config, buf, sizeof(buf)));
    TEST_ASSERT_GREATER_THAN(0, strlen(buf));
}

void test_create_json_buffer_too_small(void) {
    char buf[8];  // Way too small
    TEST_ASSERT_FALSE(createConfigJSON(&config, buf, sizeof(buf)));
}

void test_roundtrip_full_config(void) {
    strcpy(config.wifi[0].ssid, "RoundTrip");
    strcpy(config.wifi[0].password, "password123");
    config.wifi[0].enabled = true;

    config.mqtt.enabled = true;
    strcpy(config.mqtt.broker, "broker.test");
    config.mqtt.port = 8883;
    strcpy(config.mqtt.username, "admin");
    strcpy(config.mqtt.clientId, "dev-01");
    strcpy(config.mqtt.baseTopic, "test/topic");
    config.mqtt.useTLS = true;

    config.httpServer.enabled = true;
    strcpy(config.httpServer.url, "http://server.test");
    config.httpServer.port = 9090;
    config.httpServer.useTLS = true;

    config.beacon.beaconIntervalMs = 15000;
    config.beacon.loraEnabled = false;
    config.beacon.gpsEnabled = false;
    config.beacon.bleEnabled = true;

    char buf[4096];
    TEST_ASSERT_TRUE(createConfigJSON(&config, buf, sizeof(buf)));

    PlaceNetConfig restored;
    TEST_ASSERT_TRUE(parseConfigJSON(buf, &restored));

    TEST_ASSERT_EQUAL_STRING("RoundTrip", restored.wifi[0].ssid);
    TEST_ASSERT_EQUAL_STRING("password123", restored.wifi[0].password);
    TEST_ASSERT_TRUE(restored.wifi[0].enabled);

    TEST_ASSERT_EQUAL_STRING("broker.test", restored.mqtt.broker);
    TEST_ASSERT_EQUAL_UINT16(8883, restored.mqtt.port);
    TEST_ASSERT_EQUAL_STRING("admin", restored.mqtt.username);
    TEST_ASSERT_EQUAL_STRING("dev-01", restored.mqtt.clientId);
    TEST_ASSERT_EQUAL_STRING("test/topic", restored.mqtt.baseTopic);
    TEST_ASSERT_TRUE(restored.mqtt.enabled);
    TEST_ASSERT_TRUE(restored.mqtt.useTLS);

    TEST_ASSERT_EQUAL_STRING("http://server.test", restored.httpServer.url);
    TEST_ASSERT_EQUAL_UINT16(9090, restored.httpServer.port);
    TEST_ASSERT_TRUE(restored.httpServer.enabled);
    TEST_ASSERT_TRUE(restored.httpServer.useTLS);

    TEST_ASSERT_EQUAL_UINT32(15000, restored.beacon.beaconIntervalMs);
    TEST_ASSERT_FALSE(restored.beacon.loraEnabled);
    TEST_ASSERT_FALSE(restored.beacon.gpsEnabled);
    TEST_ASSERT_TRUE(restored.beacon.bleEnabled);
}

void test_roundtrip_only_enabled_wifi_serialized(void) {
    strcpy(config.wifi[0].ssid, "Enabled");
    strcpy(config.wifi[0].password, "password123");
    config.wifi[0].enabled = true;

    strcpy(config.wifi[1].ssid, "Disabled");
    strcpy(config.wifi[1].password, "password456");
    config.wifi[1].enabled = false;

    char buf[4096];
    TEST_ASSERT_TRUE(createConfigJSON(&config, buf, sizeof(buf)));

    PlaceNetConfig restored;
    TEST_ASSERT_TRUE(parseConfigJSON(buf, &restored));

    TEST_ASSERT_EQUAL_STRING("Enabled", restored.wifi[0].ssid);
    TEST_ASSERT_TRUE(restored.wifi[0].enabled);
    // Disabled network should not appear in JSON, so index 1 stays default
    TEST_ASSERT_FALSE(restored.wifi[1].enabled);
    TEST_ASSERT_EQUAL_STRING("", restored.wifi[1].ssid);
}

int main(int argc, char** argv) {
    UNITY_BEGIN();

    RUN_TEST(test_parse_minimal_json);
    RUN_TEST(test_parse_invalid_json);
    RUN_TEST(test_parse_wifi_single);
    RUN_TEST(test_parse_mqtt_fields);
    RUN_TEST(test_parse_http_server);
    RUN_TEST(test_parse_beacon_fields);
    RUN_TEST(test_create_json_default_config);
    RUN_TEST(test_create_json_buffer_too_small);
    RUN_TEST(test_roundtrip_full_config);
    RUN_TEST(test_roundtrip_only_enabled_wifi_serialized);

    return UNITY_END();
}
