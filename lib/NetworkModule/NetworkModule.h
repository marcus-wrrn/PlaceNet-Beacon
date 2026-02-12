#pragma once

#include <Arduino.h>
#include "WiFi.h"
#include "PubSubClient.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define MQTT_CONNECTED_BIT  BIT2
#define WIFI_MAX_RETRY      5

struct MQTTMessage {
    char topic[128];
    char data[256];
    int data_len;
    int topic_len;
};

typedef void (*MQTTMessageCallback)(const MQTTMessage* message);

class NetworkModule {
public:
    NetworkModule();
    ~NetworkModule();

    // Initialize WiFi connection with credentials, waits up to 5s for connection
    bool initWiFi(const char* ssid, const char* password);

    // Initialize MQTT client without authentication
    bool initMQTT(const char* broker, uint16_t port, const char* clientId);

    // Initialize MQTT client with username/password authentication
    bool initMQTT(const char* broker, uint16_t port, const char* clientId,
                  const char* username, const char* password);

    // Check if WiFi is currently connected via event group bits
    bool isWiFiConnected();

    // Check if MQTT client is currently connected via event group bits
    bool isMQTTConnected();

    // Publish MQTT message to specified topic with optional QoS and retain flag
    bool publishMessage(const char* topic, const char* message, int qos = 0, bool retain = false);

    // Subscribe to MQTT topic with optional QoS level
    bool subscribeTopic(const char* topic, int qos = 0);

    // Set callback function to handle incoming MQTT messages
    void setMessageCallback(MQTTMessageCallback callback);

    // Attempt to reconnect WiFi if disconnected, waits up to 5s
    void reconnectWiFi();

    // Attempt to reconnect MQTT if WiFi is connected and MQTT is disconnected
    void reconnectMQTT();

    // Maintain WiFi and MQTT connections, call regularly from main loop
    void loop();

    // Enable WiFi radio and attempt connection with stored credentials
    void enableWiFi();

    // Disconnect and disable WiFi radio, also disables MQTT if connected
    void disableWiFi();

    // Reconnect to MQTT broker if WiFi is connected
    void enableMQTT();

    // Disconnect from MQTT broker
    void disableMQTT();

    // Stop all network services (MQTT and WiFi)
    void stop();

private:
    WiFiClient* wifiClient;
    PubSubClient* mqttClient;

    EventGroupHandle_t eventGroup;
    MQTTMessageCallback messageCallback;

    char mqttBroker[128];
    uint16_t mqttPort;
    char mqttClientId[64];
    char mqttUsername[64];
    char mqttPassword[64];
    bool mqttHasAuth;

    char wifiSsid[32];
    char wifiPassword[64];
    int wifiRetryCount;

    static void mqttCallbackWrapper(char* topic, byte* payload, unsigned int length);
    static NetworkModule* instance;

    void handleMQTTMessage(char* topic, byte* payload, unsigned int length);
};
