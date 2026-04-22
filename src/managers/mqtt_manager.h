#pragma once

#include <Arduino.h>
#include <atomic>
#include "mqtt_client.h"
#include "PlaceNetConfig.h"

class MQTTManager {
public:
    MQTTManager();
    ~MQTTManager();

    // Connect to the broker described by brokerInfo using mutual TLS.
    // Starts the esp_mqtt internal task; returns true if the client started
    // successfully (actual TCP/TLS connection is asynchronous).
    bool connect(const MQTTBrokerInfo& brokerInfo,
                 const char* clientId,
                 const char* deviceAddress,
                 const char* mdnsHostname,
                 uint16_t mdnsPort,
                 const char* caCertPem,
                 const char* deviceCertPem,
                 const char* deviceKeyPem);

    // No-op: esp_mqtt runs its own internal FreeRTOS task.
    // Kept so callers don't need to change.
    void loop();

    bool publish(const char* topic, const char* payload, bool retained = false);
    bool isConnected();

private:
    static void eventHandler(void* handlerArgs, esp_event_base_t base,
                             int32_t eventId, void* eventData);
    void onEvent(esp_mqtt_event_handle_t event);
    void subscribeTopics();
    void publishRegistration();

    esp_mqtt_client_handle_t client_ = nullptr;
    std::atomic<bool>        connected_{false};

    MQTTBrokerInfo brokerInfo_;

    char     clientId_[MAX_MQTT_CLIENT_ID_LENGTH] = {};
    char     deviceAddress_[64]                   = {};
    char     mdnsHostname_[32]                    = {};
    uint16_t mdnsPort_                            = 0;

    // PEM strings must outlive the client — stored as members so pointers
    // passed to esp_mqtt_client_init remain valid.
    String caCertPem_;
    String deviceCertPem_;
    String deviceKeyPem_;
};
