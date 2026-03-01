#include "http_manager.h"

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
    return false;
}

bool HTTPManager::performHandshake(const char* deviceAddress,
                                   const char* mdnsHostname,
                                   uint16_t mdnsPort) {
    return false;
}
