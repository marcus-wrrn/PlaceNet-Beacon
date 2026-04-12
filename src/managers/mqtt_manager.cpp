#include "mqtt_manager.h"
#include "logger.h"
#include <ArduinoJson.h>

static const char* TAG = "MQTT-MANAGER";

// PubSubClient requires a plain C-style callback; this trampoline lets us log
// received messages without needing a full instance dispatch mechanism.
static void onMessageReceived(char* topic, byte* payload, unsigned int length) {
    // Guard against oversized payloads trashing the stack.
    const unsigned int kMaxLog = 200;
    char buf[kMaxLog + 1] = {};
    unsigned int len = (length < kMaxLog) ? length : kMaxLog;
    memcpy(buf, payload, len);
    LOGI("MQTT-CB", "[%s] %.*s%s", topic, (int)len, buf, length > kMaxLog ? "…" : "");
}

// ── MQTTManager ───────────────────────────────────────────────────────────────

MQTTManager::MQTTManager() : mqttClient_(wifiClient_) {
    mqttClient_.setCallback(onMessageReceived);
}

bool MQTTManager::connect(const MQTTBrokerInfo& brokerInfo,
                           const char* clientId,
                           const char* deviceAddress,
                           const char* mdnsHostname,
                           uint16_t mdnsPort) {
    brokerInfo_ = brokerInfo;
    strncpy(clientId_,      clientId,      sizeof(clientId_) - 1);
    strncpy(deviceAddress_, deviceAddress, sizeof(deviceAddress_) - 1);
    strncpy(mdnsHostname_,  mdnsHostname,  sizeof(mdnsHostname_) - 1);
    mdnsPort_ = mdnsPort;

    // TODO: implement MQTTS — swap wifiClient_ for a WiFiClientSecure, load
    // the CA cert returned in the handshake response, and switch to the MQTTS
    // port (brokerInfo_.port will already reflect the correct port once
    // placenet-home sets MQTT_TLS_ENABLED=true).

    mqttClient_.setServer(brokerInfo_.address, brokerInfo_.port);
    mqttClient_.setKeepAlive(60);
    mqttClient_.setSocketTimeout(10);

    LOGI(TAG, "Connecting to MQTT broker %s:%u as '%s'",
         brokerInfo_.address, brokerInfo_.port, clientId_);

    return attemptConnect();
}

bool MQTTManager::attemptConnect() {
    if (!mqttClient_.connect(clientId_)) {
        LOGW(TAG, "MQTT connect failed (rc=%d) — will retry in %lums",
             mqttClient_.state(), kReconnectIntervalMs);
        lastReconnectAttemptMs_ = millis();
        return false;
    }

    LOGI(TAG, "MQTT connected");
    subscribeTopics();
    publishRegistration();
    return true;
}

void MQTTManager::subscribeTopics() {
    for (uint8_t i = 0; i < brokerInfo_.topicCount; i++) {
        const char* topic = brokerInfo_.topics[i].topic;
        uint8_t     qos   = brokerInfo_.topics[i].qos;
        if (mqttClient_.subscribe(topic, qos)) {
            LOGI(TAG, "Subscribed to '%s' (QoS %u)", topic, qos);
        } else {
            LOGW(TAG, "Failed to subscribe to '%s'", topic);
        }
    }
}

void MQTTManager::publishRegistration() {
    JsonDocument doc;
    doc["address"]           = deviceAddress_;
    JsonObject mdns          = doc["mdns"].to<JsonObject>();
    mdns["hostname"]         = mdnsHostname_;
    mdns["port"]             = mdnsPort_;

    char payload[256] = {};
    serializeJson(doc, payload, sizeof(payload));

    if (mqttClient_.publish("registration", payload)) {
        LOGI(TAG, "Published registration: %s", payload);
    } else {
        LOGW(TAG, "Failed to publish registration message");
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void MQTTManager::loop() {
    if (mqttClient_.connected()) {
        mqttClient_.loop();
        return;
    }

    unsigned long now = millis();
    if (now - lastReconnectAttemptMs_ >= kReconnectIntervalMs) {
        LOGI(TAG, "MQTT disconnected — attempting reconnect...");
        lastReconnectAttemptMs_ = now;
        attemptConnect();
    }
}

bool MQTTManager::publish(const char* topic, const char* payload, bool retained) {
    if (!mqttClient_.connected()) {
        LOGW(TAG, "publish('%s') dropped — not connected", topic);
        return false;
    }
    return mqttClient_.publish(topic, payload, retained);
}

bool MQTTManager::isConnected() {
    return mqttClient_.connected();
}
