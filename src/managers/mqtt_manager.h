#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "PlaceNetConfig.h"

class MQTTManager {
public:
    MQTTManager();

    // Connect to the broker described by brokerInfo and subscribe to its topics.
    // clientId    — MQTT client identifier (e.g. resolved mDNS hostname).
    // deviceAddress / mdnsHostname / mdnsPort — included in the registration
    //   message published to the "registration" topic on every (re)connect so
    //   placenet-home can track the device.
    //
    // Returns true if the initial connection succeeded.  loop() will retry
    // automatically when the connection drops.
    bool connect(const MQTTBrokerInfo& brokerInfo,
                 const char* clientId,
                 const char* deviceAddress,
                 const char* mdnsHostname,
                 uint16_t mdnsPort);

    // Must be called frequently (e.g. every 100 ms) from the main loop to
    // service incoming messages and trigger reconnect attempts when disconnected.
    void loop();

    bool publish(const char* topic, const char* payload, bool retained = false);
    bool isConnected();

private:
    bool attemptConnect();
    void subscribeTopics();
    void publishRegistration();

    WiFiClient   wifiClient_;   // must be declared before mqttClient_
    PubSubClient mqttClient_;

    MQTTBrokerInfo brokerInfo_;

    char     clientId_[MAX_MQTT_CLIENT_ID_LENGTH] = {};
    char     deviceAddress_[64]                   = {};
    char     mdnsHostname_[32]                    = {};
    uint16_t mdnsPort_                            = 0;

    unsigned long lastReconnectAttemptMs_           = 0;
    static constexpr unsigned long kReconnectIntervalMs = 5000;
};
