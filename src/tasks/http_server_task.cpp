#include "http_server_task.h"

#ifdef HAS_HTTP_SERVER

#include "SDCardModule.h"
#include "HTTPServerModule.h"
#include "logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "HTTP_TASK";

void httpServerTask(void* pvParameters) {
    HTTPServerTaskParams* params = static_cast<HTTPServerTaskParams*>(pvParameters);

    if (!params || !params->sdCard || !params->httpServer) {
        LOGE(TAG, "HTTP server task parameters invalid, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    SDCardModule* sdCard = params->sdCard;
    HTTPServerModule* httpServer = params->httpServer;

    LOGI(TAG, "HTTP server task starting...");

    if (!sdCard->init()) {
        LOGE(TAG, "SD card initialization failed, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    sdCard->listDirectory("/");

    if (!httpServer->init()) {
        LOGE(TAG, "HTTP server initialization failed, task exiting");
        vTaskDelete(nullptr);
        return;
    }

    LOGI(TAG, "HTTP server ready at http://%s/", httpServer->getIPAddress().c_str());

    uint32_t statusReportInterval = 60000;
    uint32_t lastStatusReport = millis();

    while (1) {
        httpServer->maintainWiFiConnection();

        uint32_t currentTime = millis();
        if (currentTime - lastStatusReport >= statusReportInterval) {
            if (httpServer->isWiFiConnected()) {
                LOGI(TAG, "Status: WiFi connected, IP: %s",
                     httpServer->getIPAddress().c_str());
            } else {
                LOGW(TAG, "Status: WiFi disconnected");
            }
            lastStatusReport = currentTime;
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

#endif // HAS_HTTP_SERVER
