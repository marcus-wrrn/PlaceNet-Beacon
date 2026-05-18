#include "mqtt_manager.h"
#include "logger.h"
#include <ArduinoJson.h>

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
            LOGW(TAG, "MQTT disconnected — client will retry automatically");
            connected_ = false;
            break;

        case MQTT_EVENT_DATA:
            LOGI("MQTT-CB", "[%.*s] %.*s",
                 event->topic_len, event->topic,
                 event->data_len, event->data);
            break;

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
