#pragma once

#include <Arduino.h>

#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64
#define MAX_MQTT_BROKER_LENGTH 128
#define MAX_MQTT_TOPIC_LENGTH 64
#define MAX_MQTT_CLIENT_ID_LENGTH 32
#define MAX_HTTP_SERVER_LENGTH 128
#define MAX_WIFI_NETWORKS 3

struct WiFiCredentials {
    char ssid[MAX_SSID_LENGTH];
    char password[MAX_PASSWORD_LENGTH];
    bool enabled;

    WiFiCredentials() : enabled(false) {
        memset(ssid, 0, sizeof(ssid));
        memset(password, 0, sizeof(password));
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
    bool validate() const;

private:
    bool isValidSSID(const char* ssid) const;
    bool isValidPassword(const char* password) const;
    bool isValidMQTTBroker(const char* broker) const;
};
