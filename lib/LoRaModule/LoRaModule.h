#pragma once

#include "config.h"
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstdint>
#include "MeshPacket.h"
#include "MeshAdvert.h"

#define LORA_MAX_PACKET_SIZE 255

struct LoRaPacket {
    // Raw bytes written to / read from the radio.
    uint8_t data[LORA_MAX_PACKET_SIZE];
    uint8_t length;

    // Radio metadata (populated on RX; rssi==0 && snr==0 marks a TX echo).
    int16_t rssi;
    float   snr;

    // Structured broadcast fields parsed from / serialised into data[].
    char    url[128];
    uint8_t kid[4];
    uint8_t tok[4];
};

enum LoRaMode {
    LORA_MODE_IDLE,
    LORA_MODE_TX,
    LORA_MODE_RX,
    LORA_MODE_SLEEP
};

class LoRaModule {
public:
    LoRaModule();
    ~LoRaModule();
    bool init();
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Transmit data immediately (blocking)
     * @param data Pointer to data buffer
     * @param length Length of data to transmit
     * @return true if transmission successful
     */
    bool transmit(const uint8_t* data, uint8_t length);

    /**
     * @brief Start continuous receive mode
     * @return true if receive mode started successfully
     */
    bool startListening();

    /**
     * @brief Non-blocking packet read driven by DIO1 interrupt flag.
     *        Call after startListening(); returns true if a complete packet
     *        was waiting.  Re-arms continuous receive automatically.
     * @param packet Pointer to LoRaPacket struct to fill
     * @return true if a packet was ready and read successfully
     */
    bool readPacket(LoRaPacket* packet);

    /**
     * @brief Serialize and transmit a MeshPacket.
     * @param packet The MeshPacket to send.
     * @return true if the packet was serialized and transmitted successfully.
     */
    bool transmitMeshCorePacket(const meshcore::MeshPacket& packet);

    /**
     * @brief Wrap an Advert in a PAYLOAD_TYPE_ADVERT MeshPacket and transmit it.
     * @param advert The Advert to send (appData must already be encoded / signature set).
     * @return true if the advert was packed and transmitted successfully.
     */
    bool transmitMeshCoreAdvert(const meshcore::Advert& advert);

    /**
     * @brief Parse a raw LoRaPacket into a MeshPacket.
     * @param raw  Source LoRaPacket (data[] + length filled by readPacket()).
     * @param out  MeshPacket to populate.
     * @return true if parsing succeeded.
     */
    bool parsePacket(const LoRaPacket& raw, meshcore::MeshPacket& out);

    /**
     * @brief Parse the payload of a PAYLOAD_TYPE_ADVERT MeshPacket into an Advert.
     * @param packet Source MeshPacket (payloadType must be PAYLOAD_TYPE_ADVERT).
     * @param out    Advert to populate.
     * @return true if the payload was a valid advert and parsing succeeded.
     */
    bool parseAdvert(const meshcore::MeshPacket& packet, meshcore::Advert& out);

    /** @brief Returns true if the DIO1 ISR has flagged a received packet. */
    bool hasPacket() const { return rxFlag_; }

    /**
     * @brief Set radio mode
     * @param mode LoRa mode (IDLE, TX, RX, SLEEP)
     * @return true if mode set successfully
     */
    bool setMode(LoRaMode mode);

    LoRaMode getMode() const { return mode_; }

    /**
     * @brief Get duty cycle percentage over specified window
     * @param windowMs Time window in milliseconds (default: 3600000 = 1 hour)
     * @return Duty cycle as percentage (0.0 - 100.0)
     */
    float getDutyCycle(uint32_t windowMs = 3600000);

private:
    // ── Interrupt-driven receive ──────────────────────────────────────────────
    /** Set by DIO1 ISR when a full packet has been received. */
    volatile bool rxFlag_;

    /** Singleton pointer used by the static ISR callback. */
    static LoRaModule* instance_;

    /** DIO1 ISR — must be in IRAM and have no-argument signature. */
    static void IRAM_ATTR onDio1Interrupt();
    // ─────────────────────────────────────────────────────────────────────────

    struct TransmissionRecord {
        uint32_t timestamp;
        uint32_t timeOnAir;
    };

    static const uint8_t MAX_TX_RECORDS = 100;

    SX1262* radio_;

    LoRaMode mode_;
    bool initialized_;

    float frequency_;
    float bandwidth_;
    uint8_t spreadingFactor_;
    uint8_t codingRate_;
    int8_t txPower_;
    uint8_t syncWord_;

    TransmissionRecord txRecords_[MAX_TX_RECORDS];
    uint8_t txRecordIndex_;
    uint8_t txRecordCount_;

    void recordTransmission(uint32_t timeOnAir);
};
