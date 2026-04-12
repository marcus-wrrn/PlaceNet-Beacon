#pragma once

#include <Arduino.h>
#include "PlaceNetConfig.h"

class HTTPManager {
public:
    HTTPManager(const char* serverIp, uint16_t serverPort);

    bool checkHealth();
    bool performHandshake(const char* deviceAddress,
                          const char* mdnsHostname,
                          uint16_t mdnsPort,
                          const char* csrPem,
                          String& responseBody);

    bool parseMQTTBrokerResponse(const String& body, MQTTBrokerInfo* out, String& certPem);

private:
    String baseUrl() const;

    const char* serverIp_;
    uint16_t    serverPort_;
};
