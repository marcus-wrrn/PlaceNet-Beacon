#pragma once

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "PlaceNetConfig.h"

class MQTTManager {
public:
    MQTTManager();

    // Connect to the broker described by brokerInfo using mutual TLS.
    // clientId      — MQTT client identifier (e.g. resolved mDNS hostname).
    // deviceAddress / mdnsHostname / mdnsPort — included in the registration
    //   message published to the "registration" topic on every (re)connect so
    //   placenet-home can track the device.
    // caCertPem     — PEM-encoded CA certificate to verify the broker's TLS cert.
    // deviceCertPem — PEM-encoded device certificate for client identity.
    // deviceKeyPem  — PEM-encoded device private key matching deviceCertPem.
    //
    // Returns true if the initial connection succeeded.  loop() will retry
    // automatically when the connection drops.
    bool connect(const MQTTBrokerInfo& brokerInfo,
                 const char* clientId,
                 const char* deviceAddress,
                 const char* mdnsHostname,
                 uint16_t mdnsPort,
                 const char* caCertPem,
                 const char* deviceCertPem,
                 const char* deviceKeyPem);

    // Must be called frequently (e.g. every 100 ms) from the main loop to
    // service incoming messages and trigger reconnect attempts when disconnected.
    void loop();

    bool publish(const char* topic, const char* payload, bool retained = false);
    bool isConnected();

private:
    bool attemptConnect();
    void subscribeTopics();
    void publishRegistration();

    WiFiClientSecure wifiClient_;   // must be declared before mqttClient_
    PubSubClient     mqttClient_;

    MQTTBrokerInfo brokerInfo_;

    char     clientId_[MAX_MQTT_CLIENT_ID_LENGTH] = {};
    char     deviceAddress_[64]                   = {};
    char     mdnsHostname_[32]                    = {};
    uint16_t mdnsPort_                            = 0;

    // TLS credentials — stored so that attemptConnect() can re-apply them on
    // every reconnection attempt.
    String caCertPem_;
    String deviceCertPem_;
    String deviceKeyPem_;

    unsigned long lastReconnectAttemptMs_           = 0;
    static constexpr unsigned long kReconnectIntervalMs = 5000;
};
