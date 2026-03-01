#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class PlaceNetKeyPair;

class HTTPManager {
public:
    HTTPManager(const char* serverIp, uint16_t serverPort);

    bool checkHealth();
    bool performHandshake(const PlaceNetKeyPair& keyPair,
                          const char* deviceAddress,
                          const char* mdnsHostname,
                          uint16_t mdnsPort);

private:
    String baseUrl() const;

    const char* serverIp_;
    uint16_t    serverPort_;
};

// TLS server: listens on a port and accepts inbound TLS connections from the
// home server using the device's EC P-256 identity key.
class TLSServer {
public:
    // Initialise with the port to listen on and the device key pair to use.
    // The key pair must remain valid for the lifetime of the TLSServer.
    TLSServer(uint16_t port, const PlaceNetKeyPair& keyPair);
    ~TLSServer();

    // Generate a self-signed certificate and start the listener task.
    // Returns false if cert generation or socket setup fails.
    bool start();

    // Stop the listener task and release resources.
    void stop();

    bool isRunning() const { return taskHandle_ != nullptr; }

private:
    static void taskEntry(void* pvParameters);
    void        run();

    bool buildSelfSignedCert();

    uint16_t                port_;
    const PlaceNetKeyPair&  keyPair_;
    TaskHandle_t            taskHandle_ = nullptr;

    // DER-encoded self-signed certificate (heap-allocated).
    unsigned char* certDer_    = nullptr;
    size_t         certDerLen_ = 0;
};
