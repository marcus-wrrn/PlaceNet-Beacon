#pragma once

#include "config.h"

#ifdef HAS_HTTP_SERVER

class SDCardModule;
class HTTPServerModule;

struct HTTPServerTaskParams {
    SDCardModule* sdCard;
    HTTPServerModule* httpServer;
};

void httpServerTask(void* pvParameters);

#endif // HAS_HTTP_SERVER
