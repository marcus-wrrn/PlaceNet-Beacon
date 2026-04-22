#include "http_manager.h"
#include "logger.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>

static const char* TAG = "HTTP-MANAGER";

// ── HTTPManager ──────────────────────────────────────────────────────────────

HTTPManager::HTTPManager(const char* serverIp, uint16_t serverPort)
    : serverIp_(serverIp), serverPort_(serverPort) {}

String HTTPManager::baseUrl() const {
    String url = "http://";
    url += serverIp_;
    url += ":";
    url += serverPort_;
    return url;
}

bool HTTPManager::checkHealth() {
    String url = baseUrl() + "/health";

    LOGI(TAG, "Checking server health at %s", url.c_str());

    HTTPClient http;
    http.begin(url);
    http.addHeader("X-PlaceNet-Health", "0.0.1");
    http.setTimeout(5000);

    int httpCode = http.GET();

    if (httpCode < 0) {
        LOGE(TAG, "GET /health failed: %s", http.errorToString(httpCode).c_str());
        http.end();
        return false;
    }

    String body = http.getString();
    LOGI(TAG, "GET /health -> HTTP %d: %s", httpCode, body.c_str());
    http.end();

    if (httpCode == 200) {
        LOGI(TAG, "Home server is healthy");
        return true;
    }

    LOGW(TAG, "Unexpected health response: HTTP %d", httpCode);
    return false;
}

bool HTTPManager::performHandshake(const char* deviceAddress,
                                   const char* mdnsHostname,
                                   uint16_t mdnsPort,
                                   const char* csrPem,
                                   String& responseBody) {
    JsonDocument doc;
    doc["address"]           = deviceAddress;
    JsonObject mdns          = doc["mdns"].to<JsonObject>();
    mdns["hostname"]         = mdnsHostname;
    mdns["port"]             = mdnsPort;
    if (csrPem && csrPem[0] != '\0') {
        doc["csr_pem"]       = csrPem;
    }

    String payload;
    serializeJson(doc, payload);

    String url = baseUrl() + "/";

    LOGI(TAG, "Sending PlaceNet registration to %s", url.c_str());

    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-PlaceNet-Init", "0.0.1");
    http.setTimeout(10000);

    int httpCode = http.POST(payload);

    if (httpCode < 0) {
        LOGE(TAG, "POST / failed: %s", http.errorToString(httpCode).c_str());
        http.end();
        return false;
    }

    responseBody = http.getString();
    LOGI(TAG, "POST / -> HTTP %d: %s", httpCode, responseBody.c_str());
    http.end();

    if (httpCode == 200) {
        LOGI(TAG, "Device verified by home server");
        return true;
    }

    LOGE(TAG, "Handshake rejected (HTTP %d)", httpCode);
    return false;
}

bool HTTPManager::parseMQTTBrokerResponse(const String& body, MQTTBrokerInfo* out, String& certPem, String& caCertPem) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, body);
    if (error) {
        LOGE(TAG, "Failed to parse handshake response: %s", error.c_str());
        return false;
    }

    const char* cert = doc["cert_pem"];
    if (!cert) {
        LOGE(TAG, "Handshake response missing 'cert_pem'");
        return false;
    }
    certPem = cert;

    JsonObject brokerage = doc["brokerage"];
    if (brokerage.isNull()) {
        LOGE(TAG, "Handshake response missing 'brokerage'");
        return false;
    }

    const char* caCert = brokerage["ca_cert_pem"];
    if (!caCert) {
        LOGE(TAG, "Brokerage missing 'ca_cert_pem'");
        return false;
    }
    caCertPem = caCert;

    const char* address = brokerage["address"];
    if (!address) {
        LOGE(TAG, "Brokerage missing 'address'");
        return false;
    }

    *out = MQTTBrokerInfo();
    strncpy(out->address, address, MAX_MQTT_BROKER_LENGTH - 1);
    out->port = brokerage["port"] | 1883;

    JsonArray topics = brokerage["topics"];
    if (topics) {
        for (JsonObject t : topics) {
            if (out->topicCount >= MAX_MQTT_TOPICS) break;
            const char* topic = t["topic"];
            if (topic) {
                strncpy(out->topics[out->topicCount].topic, topic, MAX_MQTT_TOPIC_LENGTH - 1);
            }
            out->topics[out->topicCount].qos = t["qos"] | 0;
            out->topicCount++;
        }
    }

    LOGI(TAG, "Parsed MQTT broker: %s:%u (%u topics)", out->address, out->port, out->topicCount);
    return true;
}
