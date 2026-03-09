#pragma once

#include <Arduino.h>
#include "config.h"

#ifdef HAS_SDCARD
class SDCardModule;
#endif

struct WiFiCredentials {
    char ssid[MAX_SSID_LENGTH];
    char password[MAX_PASSWORD_LENGTH];
    bool enabled;

    WiFiCredentials() : enabled(false) {
        memset(ssid, 0, sizeof(ssid));
        memset(password, 0, sizeof(password));
    }
};

struct MQTTTopic {
    char topic[MAX_MQTT_TOPIC_LENGTH];
    uint8_t qos;

    MQTTTopic() : qos(0) {
        memset(topic, 0, sizeof(topic));
    }
};

struct MQTTBrokerInfo {
    char address[MAX_MQTT_BROKER_LENGTH];
    uint16_t port;
    MQTTTopic topics[MAX_MQTT_TOPICS];
    uint8_t topicCount;

    MQTTBrokerInfo() : port(1883), topicCount(0) {
        memset(address, 0, sizeof(address));
    }
};

struct MQTTConfig {
    char broker[MAX_MQTT_BROKER_LENGTH];
    uint16_t port;
    char username[MAX_PASSWORD_LENGTH];
    char password[MAX_PASSWORD_LENGTH];
    char clientId[MAX_MQTT_CLIENT_ID_LENGTH];
    char baseTopic[MAX_MQTT_TOPIC_LENGTH];
    bool enabled;
    bool useTLS;

    MQTTConfig() : port(1883), enabled(false), useTLS(false) {
        memset(broker, 0, sizeof(broker));
        memset(username, 0, sizeof(username));
        memset(password, 0, sizeof(password));
        memset(clientId, 0, sizeof(clientId));
        memset(baseTopic, 0, sizeof(baseTopic));
    }
};

struct HTTPServerConfig {
    char url[MAX_HTTP_SERVER_LENGTH];
    uint16_t port;
    bool enabled;
    bool useTLS;

    HTTPServerConfig() : port(80), enabled(false), useTLS(false) {
        memset(url, 0, sizeof(url));
    }
};

struct BeaconConfig {
    uint32_t beaconIntervalMs;
    bool loraEnabled;
    bool gpsEnabled;
    bool bleEnabled;

    BeaconConfig() : beaconIntervalMs(30000), loraEnabled(true), gpsEnabled(true), bleEnabled(false) {}
};

class PlaceNetConfig {
public:
    PlaceNetConfig();

    WiFiCredentials wifi[MAX_WIFI_NETWORKS];
    MQTTConfig mqtt;
    HTTPServerConfig httpServer;
    BeaconConfig beacon;

    void reset();
    void print() const;

#ifdef HAS_SDCARD
    bool resetHard(SDCardModule* sd);
#endif
    bool validate() const;
    bool isSetUp() const;

private:
    bool isValidSSID(const char* ssid) const;
    bool isValidPassword(const char* password) const;
    bool isValidMQTTBroker(const char* broker) const;
};
