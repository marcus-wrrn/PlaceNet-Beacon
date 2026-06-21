#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "ContactStore.h"
#include "MeshIdentity.h"
#include "MeshGroupChannel.h"
#include "MeshPacket.h"
#include "MeshAdvert.h"
#include "MeshDirectMessage.h"

struct DisplayState;  // defined in tasks/main_task.h

// Owns this beacon's MeshCore messaging state — the stable Ed25519 identity, the
// table of known peers, and the cached public group channel — and drives all
// MeshCore TX/RX framing. Producers hand it the LoRa TX queue at construction;
// the packet-processing loop calls the handle*() entry points for received
// frames. NOT thread-safe: intended to be owned by a single task.
class MeshMessenger {
public:
    MeshMessenger();

    // Bind the LoRa TX queue and allocate the contact-table backing storage
    // (PSRAM-preferred). Call once the radio is up and loraTxQueue exists.
    // Returns false on OOM, in which case direct messaging is disabled.
    // Idempotent.
    bool begin(QueueHandle_t txQueue);

    // True once begin() has bound a usable TX queue. The handle*() and send*()
    // paths are no-ops (safe) before this is true.
    bool ready() const { return txQueue_ != nullptr; }

    // This beacon's stable identity, derived deterministically from the factory
    // eFuse MAC so the public key is the same across reboots.
    const meshcore::MeshIdentity& identity();

    // Broadcast a signed MeshCore ADVERT announcing this beacon.
    void sendStartupAdvert();

    // Send a MeshCore-compatible GRP_TXT "Hello" on the public group channel so
    // any MeshCore chat node in range sees a reply.
    void sendHelloReply();

    // Learn a received ADVERT: update the display name, and if the signature
    // verifies, track the peer so its direct messages can be decrypted later.
    void handleAdvert(const meshcore::Advert& advert, DisplayState& st, int pktCount);

    // Handle a received direct text message: confirm it's addressed to us, find
    // the sending contact, decrypt, update the display, ACK, and reply.
    void handleDirectMessage(const meshcore::MeshPacket& pkt, DisplayState& st, int pktCount);

private:
    // Queue a PAYLOAD_TYPE_ACK carrying the 6-byte ack hash so the sender's UI
    // can mark the message delivered.
    void sendDirectAck(const uint8_t ack[meshcore::DM_ACK_SIZE]);

    // Queue an encrypted direct text to a single peer. `secret` is the shared
    // secret with that peer; destHash/srcHash are pubKey[0] of recipient / us.
    void sendDirectText(const uint8_t secret[meshcore::DM_SHARED_SECRET_SIZE],
                        uint8_t destHash, uint8_t srcHash, const char* text);

    QueueHandle_t          txQueue_ = nullptr;
    meshcore::ContactStore contacts_;
    meshcore::MeshIdentity identity_;
    bool                   identityReady_ = false;
    meshcore::GroupChannel publicChannel_;
    bool                   channelReady_ = false;
};
