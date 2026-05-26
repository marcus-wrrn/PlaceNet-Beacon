#include "mqtt_manager.h"
#include "logger.h"
#include <ArduinoJson.h>
#include "../tasks/lora_task.h"
#include "LoRaModule.h"

static const char* TAG = "MQTT-MANAGER";

MQTTManager::MQTTManager() {}

MQTTManager::~MQTTManager() {
    if (client_) {
        esp_mqtt_client_stop(client_);
        esp_mqtt_client_destroy(client_);
    }
}

bool MQTTManager::connect(const MQTTBrokerInfo& brokerInfo,
                           const char* clientId,
                           const char* deviceAddress,
                           const char* mdnsHostname,
                           uint16_t mdnsPort,
                           const char* caCertPem,
                           const char* deviceCertPem,
                           const char* deviceKeyPem) {
    brokerInfo_ = brokerInfo;
    strncpy(clientId_,      clientId,      sizeof(clientId_) - 1);
    strncpy(deviceAddress_, deviceAddress, sizeof(deviceAddress_) - 1);
    strncpy(mdnsHostname_,  mdnsHostname,  sizeof(mdnsHostname_) - 1);
    strncpy(broadcastTopic, brokerInfo_.broadcast.topic, sizeof(broadcastTopic) - 1);
    strncpy(commandTopic,   brokerInfo_.receive.topic,   sizeof(commandTopic) - 1);
    mdnsPort_ = mdnsPort;

    caCertPem_     = caCertPem;
    deviceCertPem_ = deviceCertPem;
    deviceKeyPem_  = deviceKeyPem;

    char uri[128] = {};
    snprintf(uri, sizeof(uri), "mqtts://%s:%u", brokerInfo_.address, brokerInfo_.port);

    esp_mqtt_client_config_t config = {};
    config.uri         = uri;
    config.client_id   = clientId_;
    config.cert_pem    = caCertPem_.c_str();

    // TODO: This is enabled to skip checking for the proper SAN name - Eventually this should be included as part of the protocol - remove later
    config.skip_cert_common_name_check = true;

    if (deviceCertPem_.length() > 0) {
        config.client_cert_pem = deviceCertPem_.c_str();
    }
    if (deviceKeyPem_.length() > 0) {
        config.client_key_pem = deviceKeyPem_.c_str();
    }

    client_ = esp_mqtt_client_init(&config);
    if (!client_) {
        LOGE(TAG, "Failed to init MQTT client");
        return false;
    }

    esp_mqtt_client_register_event(client_, MQTT_EVENT_ANY, eventHandler, this);

    esp_err_t err = esp_mqtt_client_start(client_);
    if (err != ESP_OK) {
        LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(client_);
        client_ = nullptr;
        return false;
    }

    LOGI(TAG, "MQTT client started — connecting to %s as '%s'", uri, clientId_);
    return true;
}

void MQTTManager::eventHandler(void* handlerArgs, esp_event_base_t base,
                                int32_t eventId, void* eventData) {
    auto* self = static_cast<MQTTManager*>(handlerArgs);
    self->onEvent(static_cast<esp_mqtt_event_handle_t>(eventData));
}

void MQTTManager::onEvent(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            LOGI(TAG, "MQTT connected");
            connected_ = true;
            subscribeTopics();
            publishRegistration();
            break;

        case MQTT_EVENT_DISCONNECTED:
            LOGW(TAG, "MQTT disconnected - client will retry automatically");
            connected_ = false;
            break;

        case MQTT_EVENT_DATA: {
            // esp_mqtt does not null-terminate topic or data
            char topic[MAX_MQTT_TOPIC_LENGTH] = {};
            char data[512] = {};
            int tlen = std::min(event->topic_len, (int)sizeof(topic) - 1);
            int dlen = std::min(event->data_len,  (int)sizeof(data)  - 1);
            memcpy(topic, event->topic, tlen);
            memcpy(data,  event->data,  dlen);

            LOGI("MQTT-CB", "[%s] %s", topic, data);

            if (strcmp(topic, commandTopic) == 0) {
                handleCommand(data);
            } else if (strcmp(topic, broadcastTopic) == 0) {
                handleBroadcast(data);
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            LOGE(TAG, "MQTT error type=%d", event->error_handle->error_type);
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                LOGE(TAG, "  esp_tls_err=0x%x  tls_stack_err=0x%x",
                     event->error_handle->esp_tls_last_esp_err,
                     event->error_handle->esp_tls_stack_err);
            }
            break;

        default:
            break;
    }
}

void MQTTManager::handleCommand(const char* payload) {
    JsonDocument doc;
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {
        LOGW(TAG, "handleCommand: JSON parse failed");
        return;
    }

    CommandPayload cmd = {};
    strncpy(cmd.command, doc["command"] | "", sizeof(cmd.command) - 1);
    strncpy(cmd.params,  doc["params"]  | "", sizeof(cmd.params)  - 1);
    cmd.ok = cmd.command[0] != '\0';

    if (!cmd.ok) {
        LOGW(TAG, "handleCommand: missing 'command' field");
        return;
    }

    LOGI(TAG, "Command received: %s params=%s", cmd.command, cmd.params);
}

void MQTTManager::handleBroadcast(const char* payload) {
    JsonDocument doc;
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {
        LOGW(TAG, "handleBroadcast: JSON parse failed");
        return;
    }

    BroadcastPayload bcast = {};
    strncpy(bcast.beaconId, doc["beaconId"] | "", sizeof(bcast.beaconId) - 1);
    strncpy(bcast.address,  doc["address"]  | "", sizeof(bcast.address)  - 1);
    bcast.ok = bcast.beaconId[0] != '\0';

    if (!bcast.ok) {
        LOGW(TAG, "handleBroadcast: missing 'beaconId' field");
        return;
    }

    LOGI(TAG, "Broadcast received from %s at %s — relaying over LoRa", bcast.beaconId, bcast.address);

    if (!loraTxQueue) {
        LOGW(TAG, "handleBroadcast: loraTxQueue not ready, dropping LoRa relay");
        return;
    }

    // TODO: transmit only the fields needed by the LoRa protocol rather than
    //       the full JSON payload (e.g. address only, or a compact binary frame).
    LoRaPacket pkt = {};
    uint8_t len = static_cast<uint8_t>(
        std::min(strlen(payload), (size_t)(LORA_MAX_PACKET_SIZE - 1)));
    memcpy(pkt.data, payload, len);
    pkt.length = len;

    if (xQueueSend(loraTxQueue, &pkt, 0) != pdPASS) {
        LOGW(TAG, "handleBroadcast: loraTxQueue full, LoRa relay dropped");
    } else {
        LOGI(TAG, "Broadcast queued for LoRa TX (%u bytes)", len);
    }
}

void MQTTManager::subscribeTopics() {
    const MQTTTopic* subs[] = { &brokerInfo_.broadcast, &brokerInfo_.receive };
    for (const MQTTTopic* t : subs) {
        if (t->topic[0] == '\0') continue;
        int msgId = esp_mqtt_client_subscribe(client_, t->topic, t->qos);
        if (msgId >= 0) {
            LOGI(TAG, "Subscribed to '%s' (QoS %u, msg_id=%d)", t->topic, t->qos, msgId);
        } else {
            LOGW(TAG, "Failed to subscribe to '%s'", t->topic);
        }
    }
}

void MQTTManager::publishRegistration() {
    JsonDocument doc;
    doc["address"]       = deviceAddress_;
    JsonObject mdns      = doc["mdns"].to<JsonObject>();
    mdns["hostname"]     = mdnsHostname_;
    mdns["port"]         = mdnsPort_;

    char payload[256] = {};
    serializeJson(doc, payload, sizeof(payload));

    int msgId = esp_mqtt_client_publish(client_, "registration", payload, 0, 1, 0);
    if (msgId >= 0) {
        LOGI(TAG, "Published registration: %s", payload);
    } else {
        LOGW(TAG, "Failed to publish registration message");
    }
}

void MQTTManager::loop() {}

bool MQTTManager::publishAlive() {
    if (!connected_) {
        LOGW(TAG, "publishAlive dropped — not connected");
        return false;
    }

    JsonDocument doc;
    doc["type"]     = "alive";
    doc["beaconId"] = clientId_;

    char payload[128] = {};
    serializeJson(doc, payload, sizeof(payload));

    int msgId = esp_mqtt_client_publish(client_, commandTopic, payload, 0, 1, 0);
    if (msgId >= 0) {
        LOGI(TAG, "Alive published to '%s': %s", commandTopic, payload);
        return true;
    } else {
        LOGW(TAG, "Failed to publish alive message to '%s'", commandTopic);
        return false;
    }
}

bool MQTTManager::publish(const char* topic, const char* payload, bool retained) {
    if (!connected_) {
        LOGW(TAG, "publish('%s') dropped — not connected", topic);
        return false;
    }
    return esp_mqtt_client_publish(client_, topic, payload, 0, 1, retained ? 1 : 0) >= 0;
}

bool MQTTManager::isConnected() {
    return connected_.load();
}
