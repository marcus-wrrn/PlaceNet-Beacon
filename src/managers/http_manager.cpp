#include "http_manager.h"
#include "logger.h"
#include "placenet_keys.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include <mbedtls/ssl.h>
#include <mbedtls/net_sockets.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/error.h>

#include <lwip/sockets.h>

static const char* TAG      = "HTTP-MANAGER";
static const char* TAG_TLS  = "TLS-SERVER";

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

bool HTTPManager::performHandshake(const PlaceNetKeyPair& keyPair,
                                   const char* deviceAddress,
                                   const char* mdnsHostname,
                                   uint16_t mdnsPort) {
    JsonDocument doc;
    doc["public_key"] = keyPair.publicKeyPem;
    doc["address"]    = deviceAddress;
    JsonObject mdns   = doc["mdns"].to<JsonObject>();
    mdns["hostname"]  = mdnsHostname;
    mdns["port"]      = mdnsPort;

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

    String body = http.getString();
    LOGI(TAG, "POST / -> HTTP %d: %s", httpCode, body.c_str());
    http.end();

    if (httpCode == 200) {
        LOGI(TAG, "Handshake accepted by home server");
        return true;
    }

    LOGE(TAG, "Handshake rejected (HTTP %d)", httpCode);
    return false;
}

// ── TLSServer ────────────────────────────────────────────────────────────────

// Scratch buffer for DER cert generation.  mbedtls writes from the end of the
// buffer backwards, so it needs to be larger than the final certificate.
static constexpr size_t CERT_BUF_SIZE = 2048;

TLSServer::TLSServer(uint16_t port, const PlaceNetKeyPair& keyPair)
    : port_(port), keyPair_(keyPair) {}

TLSServer::~TLSServer() {
    stop();
}

bool TLSServer::buildSelfSignedCert() {
    mbedtls_x509write_cert  crt;
    mbedtls_mpi             serial;

    mbedtls_x509write_crt_init(&crt);
    mbedtls_mpi_init(&serial);

    // Use the key pair's own RNG for signing.
    mbedtls_ctr_drbg_context* rng = const_cast<mbedtls_ctr_drbg_context*>(&keyPair_.ctr_drbg);

    mbedtls_pk_context* pk = const_cast<mbedtls_pk_context*>(&keyPair_.pk);

    // Serial = 1
    int ret = mbedtls_mpi_lset(&serial, 1);
    if (ret != 0) { goto fail; }

    mbedtls_x509write_crt_set_version(&crt, MBEDTLS_X509_CRT_VERSION_3);
    mbedtls_x509write_crt_set_md_alg(&crt, MBEDTLS_MD_SHA256);
    mbedtls_x509write_crt_set_subject_key(&crt, pk);
    mbedtls_x509write_crt_set_issuer_key(&crt, pk);

    ret = mbedtls_x509write_crt_set_subject_name(&crt, "CN=placenet-beacon");
    if (ret != 0) { goto fail; }

    ret = mbedtls_x509write_crt_set_issuer_name(&crt, "CN=placenet-beacon");
    if (ret != 0) { goto fail; }

    // Validity: Jan 1 2025 – Jan 1 2035
    ret = mbedtls_x509write_crt_set_validity(&crt, "20250101000000", "20350101000000");
    if (ret != 0) { goto fail; }

    ret = mbedtls_x509write_crt_set_serial(&crt, &serial);
    if (ret != 0) { goto fail; }

    {
        // Write DER into a temporary stack buffer, then copy the live portion.
        unsigned char tmp[CERT_BUF_SIZE];
        ret = mbedtls_x509write_crt_der(&crt, tmp, sizeof(tmp),
                                         mbedtls_ctr_drbg_random, rng);
        if (ret < 0) { goto fail; }

        certDerLen_ = (size_t)ret;
        certDer_    = (unsigned char*)malloc(certDerLen_);
        if (!certDer_) { ret = -1; goto fail; }

        // mbedtls writes DER at the *end* of the buffer.
        memcpy(certDer_, tmp + sizeof(tmp) - certDerLen_, certDerLen_);
    }

    mbedtls_x509write_crt_free(&crt);
    mbedtls_mpi_free(&serial);
    LOGI(TAG_TLS, "Self-signed certificate generated (%u bytes)", (unsigned)certDerLen_);
    return true;

fail:
    {
        char errbuf[128];
        mbedtls_strerror(ret, errbuf, sizeof(errbuf));
        LOGE(TAG_TLS, "Certificate generation failed: %s", errbuf);
    }
    mbedtls_x509write_crt_free(&crt);
    mbedtls_mpi_free(&serial);
    free(certDer_);
    certDer_    = nullptr;
    certDerLen_ = 0;
    return false;
}

bool TLSServer::start() {
    if (taskHandle_) {
        LOGW(TAG_TLS, "Already running");
        return true;
    }

    if (!buildSelfSignedCert()) {
        return false;
    }

    // Pass `this` as the task parameter.
    BaseType_t rc = xTaskCreate(taskEntry, "tls_srv", 8192, this, 5, &taskHandle_);
    if (rc != pdPASS) {
        LOGE(TAG_TLS, "Failed to create TLS server task");
        free(certDer_);
        certDer_    = nullptr;
        certDerLen_ = 0;
        return false;
    }

    LOGI(TAG_TLS, "TLS server task started on port %u", port_);
    return true;
}

void TLSServer::stop() {
    if (taskHandle_) {
        vTaskDelete(taskHandle_);
        taskHandle_ = nullptr;
    }
    free(certDer_);
    certDer_    = nullptr;
    certDerLen_ = 0;
}

// Static trampoline.
void TLSServer::taskEntry(void* pvParameters) {
    static_cast<TLSServer*>(pvParameters)->run();
    vTaskDelete(nullptr);
}

// ── TLS server accept loop ────────────────────────────────────────────────────
//
// The home server TLS-connects back to the beacon after the registration
// handshake.  This loop accepts those connections and performs the TLS
// handshake so that the channel is established.  The connection object is
// handed off to further processing once the handshake is complete.

void TLSServer::run() {
    mbedtls_net_context      listenCtx;
    mbedtls_net_context      clientCtx;
    mbedtls_ssl_context      ssl;
    mbedtls_ssl_config       conf;
    mbedtls_x509_crt         srvcert;
    mbedtls_entropy_context  entropy;
    mbedtls_ctr_drbg_context ctr_drbg;

    mbedtls_net_init(&listenCtx);
    mbedtls_net_init(&clientCtx);
    mbedtls_ssl_init(&ssl);
    mbedtls_ssl_config_init(&conf);
    mbedtls_x509_crt_init(&srvcert);
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&ctr_drbg);

    int ret;
    const char* pers = "placenet_tls_srv";

    ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                  (const unsigned char*)pers, strlen(pers));
    if (ret != 0) {
        LOGE(TAG_TLS, "ctr_drbg_seed failed: -0x%04X", (unsigned)(-ret));
        goto cleanup;
    }

    // Load the self-signed certificate DER.
    ret = mbedtls_x509_crt_parse_der(&srvcert, certDer_, certDerLen_);
    if (ret != 0) {
        LOGE(TAG_TLS, "x509_crt_parse_der failed: -0x%04X", (unsigned)(-ret));
        goto cleanup;
    }

    // Bind the listening socket.
    {
        char portStr[8];
        snprintf(portStr, sizeof(portStr), "%u", port_);
        ret = mbedtls_net_bind(&listenCtx, nullptr, portStr, MBEDTLS_NET_PROTO_TCP);
        if (ret != 0) {
            LOGE(TAG_TLS, "net_bind failed: -0x%04X", (unsigned)(-ret));
            goto cleanup;
        }
    }

    // Configure TLS server defaults.
    ret = mbedtls_ssl_config_defaults(&conf,
                                       MBEDTLS_SSL_IS_SERVER,
                                       MBEDTLS_SSL_TRANSPORT_STREAM,
                                       MBEDTLS_SSL_PRESET_DEFAULT);
    if (ret != 0) {
        LOGE(TAG_TLS, "ssl_config_defaults failed: -0x%04X", (unsigned)(-ret));
        goto cleanup;
    }

    mbedtls_ssl_conf_rng(&conf, mbedtls_ctr_drbg_random, &ctr_drbg);

    // We don't verify the client's certificate (the home server doesn't send one
    // at this stage; mutual auth is a future concern).
    mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_NONE);

    ret = mbedtls_ssl_conf_own_cert(&conf, &srvcert,
                                     const_cast<mbedtls_pk_context*>(&keyPair_.pk));
    if (ret != 0) {
        LOGE(TAG_TLS, "ssl_conf_own_cert failed: -0x%04X", (unsigned)(-ret));
        goto cleanup;
    }

    LOGI(TAG_TLS, "Listening for TLS connections on port %u", port_);

    // Accept loop.
    while (true) {
        mbedtls_net_free(&clientCtx);
        mbedtls_ssl_free(&ssl);
        mbedtls_ssl_init(&ssl);
        mbedtls_net_init(&clientCtx);

        ret = mbedtls_net_accept(&listenCtx, &clientCtx, nullptr, 0, nullptr);
        if (ret != 0) {
            LOGE(TAG_TLS, "net_accept failed: -0x%04X", (unsigned)(-ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        LOGI(TAG_TLS, "Incoming connection — performing TLS handshake");

        ret = mbedtls_ssl_setup(&ssl, &conf);
        if (ret != 0) {
            LOGE(TAG_TLS, "ssl_setup failed: -0x%04X", (unsigned)(-ret));
            continue;
        }

        mbedtls_ssl_set_bio(&ssl, &clientCtx,
                             mbedtls_net_send,
                             mbedtls_net_recv,
                             nullptr);

        // Perform TLS handshake (blocking, non-preemptive).
        while ((ret = mbedtls_ssl_handshake(&ssl)) != 0) {
            if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
                char errbuf[128];
                mbedtls_strerror(ret, errbuf, sizeof(errbuf));
                LOGE(TAG_TLS, "TLS handshake failed: %s", errbuf);
                break;
            }
        }

        if (ret == 0) {
            LOGI(TAG_TLS, "TLS handshake complete — secure channel established");
            // TODO: dispatch connection to a session handler.
            // For now, gracefully close.
            mbedtls_ssl_close_notify(&ssl);
        }

        mbedtls_ssl_free(&ssl);
        mbedtls_net_free(&clientCtx);
        mbedtls_ssl_init(&ssl);
        mbedtls_net_init(&clientCtx);
    }

cleanup:
    mbedtls_net_free(&clientCtx);
    mbedtls_net_free(&listenCtx);
    mbedtls_ssl_free(&ssl);
    mbedtls_ssl_config_free(&conf);
    mbedtls_x509_crt_free(&srvcert);
    mbedtls_entropy_free(&entropy);
    mbedtls_ctr_drbg_free(&ctr_drbg);
}
