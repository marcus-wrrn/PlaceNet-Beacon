#include "NetworkModule.h"
#include "logger.h"

static const char* TAG = "Network";

NetworkModule* NetworkModule::instance = nullptr;

NetworkModule::NetworkModule()
    : wifiClient(nullptr),
      mqttClient(nullptr),
      eventGroup(nullptr),
      messageCallback(nullptr),
      mqttPort(1883),
      mqttHasAuth(false),
      wifiRetryCount(0)
{
    instance = this;
    eventGroup = xEventGroupCreate();

    memset(mqttBroker, 0, sizeof(mqttBroker));
    memset(mqttClientId, 0, sizeof(mqttClientId));
    memset(mqttUsername, 0, sizeof(mqttUsername));
    memset(mqttPassword, 0, sizeof(mqttPassword));
    memset(wifiSsid, 0, sizeof(wifiSsid));
    memset(wifiPassword, 0, sizeof(wifiPassword));
}

NetworkModule::~NetworkModule() {
    stop();
    if (eventGroup) {
        vEventGroupDelete(eventGroup);
    }
    if (mqttClient) {
        delete mqttClient;
    }
    if (wifiClient) {
        delete wifiClient;
    }
    instance = nullptr;
}

bool NetworkModule::initWiFi(const char* ssid, const char* password) {
    LOGI(TAG, "Initializing WiFi...");

    strncpy(wifiSsid, ssid, sizeof(wifiSsid) - 1);
    strncpy(wifiPassword, password, sizeof(wifiPassword) - 1);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int maxWait = 10;
    int count = 0;
    while (WiFi.status() != WL_CONNECTED && count < maxWait) {
        delay(500);
        LOGI(TAG, "Connecting to WiFi... (%d/%d)", count + 1, maxWait);
        count++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        LOGI(TAG, "WiFi connected");
        LOGI(TAG, "IP address: %s", WiFi.localIP().toString().c_str());
        xEventGroupSetBits(eventGroup, WIFI_CONNECTED_BIT);
        wifiRetryCount = 0;
        return true;
    } else {
        LOGE(TAG, "WiFi connection failed");
        xEventGroupSetBits(eventGroup, WIFI_FAIL_BIT);
        return false;
    }
}

bool NetworkModule::initMQTT(const char* broker, uint16_t port, const char* clientId) {
    return initMQTT(broker, port, clientId, nullptr, nullptr);
}

bool NetworkModule::initMQTT(const char* broker, uint16_t port, const char* clientId,
                             const char* username, const char* password) {
    LOGI(TAG, "Initializing MQTT...");

    strncpy(mqttBroker, broker, sizeof(mqttBroker) - 1);
    mqttPort = port;
    strncpy(mqttClientId, clientId, sizeof(mqttClientId) - 1);

    if (username && password) {
        strncpy(mqttUsername, username, sizeof(mqttUsername) - 1);
        strncpy(mqttPassword, password, sizeof(mqttPassword) - 1);
        mqttHasAuth = true;
    } else {
        mqttHasAuth = false;
    }

    if (!wifiClient) {
        wifiClient = new WiFiClient();
    }

    if (!mqttClient) {
        mqttClient = new PubSubClient(*wifiClient);
    }

    mqttClient->setServer(broker, port);
    mqttClient->setCallback(mqttCallbackWrapper);

    reconnectMQTT();

    return isMQTTConnected();
}

bool NetworkModule::isWiFiConnected() {
    if (!eventGroup) return false;
    EventBits_t bits = xEventGroupGetBits(eventGroup);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

bool NetworkModule::isMQTTConnected() {
    if (!eventGroup) return false;
    EventBits_t bits = xEventGroupGetBits(eventGroup);
    return (bits & MQTT_CONNECTED_BIT) != 0;
}

void NetworkModule::reconnectWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        LOGI(TAG, "Reconnecting to WiFi...");
        xEventGroupClearBits(eventGroup, WIFI_CONNECTED_BIT);

        WiFi.disconnect();
        delay(100);
        WiFi.begin(wifiSsid, wifiPassword);

        int maxWait = 10;
        int count = 0;
        while (WiFi.status() != WL_CONNECTED && count < maxWait) {
            delay(500);
            count++;
        }

        if (WiFi.status() == WL_CONNECTED) {
            LOGI(TAG, "WiFi reconnected");
            xEventGroupSetBits(eventGroup, WIFI_CONNECTED_BIT);
            wifiRetryCount = 0;
        } else {
            wifiRetryCount++;
            LOGW(TAG, "WiFi reconnection failed (attempt %d)", wifiRetryCount);
            if (wifiRetryCount >= WIFI_MAX_RETRY) {
                LOGE(TAG, "WiFi reconnection failed after %d attempts", WIFI_MAX_RETRY);
                xEventGroupSetBits(eventGroup, WIFI_FAIL_BIT);
            }
        }
    }
}

void NetworkModule::reconnectMQTT() {
    if (!mqttClient) {
        LOGE(TAG, "MQTT client not initialized");
        return;
    }

    if (!isWiFiConnected()) {
        LOGW(TAG, "Cannot connect to MQTT: WiFi not connected");
        return;
    }

    if (!mqttClient->connected()) {
        LOGI(TAG, "Connecting to MQTT broker %s:%d...", mqttBroker, mqttPort);

        bool connected = false;
        if (mqttHasAuth) {
            connected = mqttClient->connect(mqttClientId, mqttUsername, mqttPassword);
        } else {
            connected = mqttClient->connect(mqttClientId);
        }

        if (connected) {
            LOGI(TAG, "MQTT connected");
            xEventGroupSetBits(eventGroup, MQTT_CONNECTED_BIT);
        } else {
            LOGE(TAG, "MQTT connection failed, state=%d", mqttClient->state());
            xEventGroupClearBits(eventGroup, MQTT_CONNECTED_BIT);
        }
    }
}

void NetworkModule::loop() {
    if (WiFi.status() != WL_CONNECTED) {
        reconnectWiFi();
    }

    if (mqttClient && !mqttClient->connected()) {
        reconnectMQTT();
    }

    if (mqttClient) {
        mqttClient->loop();
    }
}

bool NetworkModule::publishMessage(const char* topic, const char* message, int qos, bool retain) {
    if (!mqttClient || !mqttClient->connected()) {
        LOGW(TAG, "Cannot publish: MQTT not connected");
        return false;
    }

    LOGI(TAG, "Publishing to topic '%s': %s", topic, message);
    bool result = mqttClient->publish(topic, message, retain);

    if (result) {
        LOGI(TAG, "Message published successfully");
    } else {
        LOGE(TAG, "Failed to publish message");
    }

    return result;
}

bool NetworkModule::subscribeTopic(const char* topic, int qos) {
    if (!mqttClient || !mqttClient->connected()) {
        LOGW(TAG, "Cannot subscribe: MQTT not connected");
        return false;
    }

    LOGI(TAG, "Subscribing to topic '%s'", topic);
    bool result = mqttClient->subscribe(topic, qos);

    if (result) {
        LOGI(TAG, "Subscribed successfully");
    } else {
        LOGE(TAG, "Failed to subscribe");
    }

    return result;
}

void NetworkModule::setMessageCallback(MQTTMessageCallback callback) {
    messageCallback = callback;
}

void NetworkModule::mqttCallbackWrapper(char* topic, byte* payload, unsigned int length) {
    if (instance) {
        instance->handleMQTTMessage(topic, payload, length);
    }
}

void NetworkModule::handleMQTTMessage(char* topic, byte* payload, unsigned int length) {
    if (!messageCallback) {
        LOGW(TAG, "No message callback set, message ignored");
        return;
    }

    MQTTMessage message = {0};

    int topicLen = strlen(topic);
    if (topicLen < sizeof(message.topic) - 1) {
        strncpy(message.topic, topic, topicLen);
        message.topic[topicLen] = '\0';
        message.topic_len = topicLen;
    } else {
        LOGW(TAG, "Topic too long, truncating");
        strncpy(message.topic, topic, sizeof(message.topic) - 1);
        message.topic[sizeof(message.topic) - 1] = '\0';
        message.topic_len = sizeof(message.topic) - 1;
    }

    if (length < sizeof(message.data) - 1) {
        memcpy(message.data, payload, length);
        message.data[length] = '\0';
        message.data_len = length;
    } else {
        LOGW(TAG, "Data too long, truncating");
        memcpy(message.data, payload, sizeof(message.data) - 1);
        message.data[sizeof(message.data) - 1] = '\0';
        message.data_len = sizeof(message.data) - 1;
    }

    LOGI(TAG, "Received message on topic '%s': '%s' (len: %d)",
         message.topic, message.data, message.data_len);

    messageCallback(&message);
}

void NetworkModule::enableWiFi() {
    LOGI(TAG, "Enabling WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSsid, wifiPassword);

    int maxWait = 10;
    int count = 0;
    while (WiFi.status() != WL_CONNECTED && count < maxWait) {
        delay(500);
        count++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        LOGI(TAG, "WiFi enabled and connected");
        LOGI(TAG, "IP address: %s", WiFi.localIP().toString().c_str());
        xEventGroupSetBits(eventGroup, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(eventGroup, WIFI_FAIL_BIT);
        wifiRetryCount = 0;
    } else {
        LOGW(TAG, "WiFi enabled but connection failed");
    }
}

void NetworkModule::disableWiFi() {
    LOGI(TAG, "Disabling WiFi...");

    if (mqttClient && mqttClient->connected()) {
        disableMQTT();
    }

    if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        xEventGroupClearBits(eventGroup, WIFI_CONNECTED_BIT);
        LOGI(TAG, "WiFi disabled");
    }
}

void NetworkModule::enableMQTT() {
    if (!isWiFiConnected()) {
        LOGW(TAG, "Cannot enable MQTT: WiFi not connected");
        return;
    }

    LOGI(TAG, "Enabling MQTT...");
    reconnectMQTT();
}

void NetworkModule::disableMQTT() {
    LOGI(TAG, "Disabling MQTT...");

    if (mqttClient && mqttClient->connected()) {
        mqttClient->disconnect();
        xEventGroupClearBits(eventGroup, MQTT_CONNECTED_BIT);
        LOGI(TAG, "MQTT disabled");
    }
}

void NetworkModule::stop() {
    disableMQTT();
    disableWiFi();
}
