#include "mesh_messenger.h"

#include "tasks/main_task.h"   // DisplayState
#include "tasks/lora_task.h"   // LoRaTxMsg, LoRaTxType
#include "config.h"            // MESHCORE_PUBLIC_CHANNEL_PSK
#include "logger.h"

#include <esp_random.h>
#include <esp_mac.h>
#include <ctime>
#include <cstdio>
#include <cstring>

static const char* TAG = "MESH";

MeshMessenger::MeshMessenger() : contacts_(64) {}

bool MeshMessenger::begin(QueueHandle_t txQueue) {
    txQueue_ = txQueue;
    return contacts_.begin();
}

// Identity is derived deterministically between reboots based on device MAC 
// Could potentially be a problem in the future but probably fine for now
const meshcore::MeshIdentity& MeshMessenger::identity() {
    if (!identityReady_) {
        uint8_t mac[6] = {0};
        esp_efuse_mac_get_default(mac);
        uint8_t seed[32] = {0};
        memcpy(seed, mac, sizeof(mac));
        static const char tag[] = "PlaceNetBeacon";
        memcpy(seed + sizeof(mac), tag, sizeof(tag) - 1);
        identity_.fromSeed(seed);
        identityReady_ = true;
    }
    return identity_;
}

void MeshMessenger::sendStartupAdvert() {
    if (!txQueue_) {
        LOGW(TAG, "ADVERT: txQueue not ready, skipping startup advert");
        return;
    }

    LoRaTxMsg txMsg;
    txMsg.type = LoRaTxType::ADVERT;
    meshcore::Advert& adv = txMsg.advert;

    adv.setType(meshcore::ADV_TYPE_CHAT);

    // Name = "PlaceNet-XXXX" using the low 2 MAC bytes for per-device uniqueness.
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    char name[meshcore::ADV_MAX_APP_DATA + 1];
    snprintf(name, sizeof(name), "PlaceNet-%02X%02X", mac[4], mac[5]);
    adv.setName(name);

    // time(nullptr) is fine without an RTC sync — MeshCore only uses the
    // timestamp for replay/freshness ordering and packet-hash uniqueness.
    uint32_t ts = static_cast<uint32_t>(time(nullptr));
    if (!identity().signAdvert(adv, ts)) {
        LOGE(TAG, "ADVERT: failed to sign startup advert");
        return;
    }

    if (xQueueSend(txQueue_, &txMsg, 0) != pdPASS) {
        LOGW(TAG, "ADVERT: txQueue full, startup advert dropped");
    } else {
        LOGI(TAG, "ADVERT: queued startup advert name='%s' type=%d",
             adv.name, adv.type);
    }
}

void MeshMessenger::sendHelloReply() {
    if (!channelReady_) {
        if (!publicChannel_.fromBase64PSK(MESHCORE_PUBLIC_CHANNEL_PSK)) {
            LOGE(TAG, "GRP_TXT: invalid public channel PSK — cannot reply");
            return;
        }
        channelReady_ = true;
    }

    if (!txQueue_) {
        LOGW(TAG, "GRP_TXT: txQueue not ready, dropping Hello reply");
        return;
    }

    LoRaTxMsg txMsg;
    txMsg.type = LoRaTxType::MESH_PACKET;
    // time(nullptr) is fine even without an RTC sync — MeshCore only uses the
    // timestamp to keep the packet hash unique.
    uint32_t ts = static_cast<uint32_t>(time(nullptr));
    if (publicChannel_.encode(txMsg.packet, ts, "PlaceNet", "Hello") == 0) {
        LOGW(TAG, "GRP_TXT: failed to encode Hello reply");
        return;
    }

    if (xQueueSend(txQueue_, &txMsg, 0) != pdPASS) {
        LOGW(TAG, "GRP_TXT: txQueue full, Hello reply dropped");
    } else {
        LOGI(TAG, "GRP_TXT: queued Hello reply (%u bytes)", txMsg.packet.payloadLen);
    }
}

void MeshMessenger::handleAdvert(const meshcore::Advert& advert, DisplayState& st, int pktCount) {
    strncpy(st.lastAdvertName, advert.name, sizeof(st.lastAdvertName) - 1);
    st.lastAdvertName[sizeof(st.lastAdvertName) - 1] = '\0';

    // Learn the peer so we can decrypt direct messages from it later. Only
    // store verified identities.
    if (meshcore::MeshIdentity::verifyAdvert(advert)) {
        if (contacts_.upsert(advert.pubKey, advert.name, advert.timestamp) < 0) {
            LOGW(TAG, "ContactStore full — not tracking %s", advert.name);
        }
    } else {
        LOGW(TAG, "#%d ADVERT signature invalid — not stored", pktCount);
    }
}

void MeshMessenger::sendDirectAck(const uint8_t ack[meshcore::DM_ACK_SIZE]) {
    if (!txQueue_) return;

    LoRaTxMsg txMsg;
    txMsg.type = LoRaTxType::MESH_PACKET;
    meshcore::MeshPacket& pkt = txMsg.packet;
    pkt.payloadType = meshcore::PAYLOAD_TYPE_ACK;
    pkt.routeType   = meshcore::ROUTE_TYPE_FLOOD;
    memcpy(pkt.payload, ack, meshcore::DM_ACK_SIZE);
    pkt.payloadLen  = meshcore::DM_ACK_SIZE;

    if (xQueueSend(txQueue_, &txMsg, 0) != pdPASS) {
        LOGW(TAG, "ACK: txQueue full, ack dropped");
    }
}

void MeshMessenger::sendDirectText(const uint8_t secret[meshcore::DM_SHARED_SECRET_SIZE],
                                   uint8_t destHash, uint8_t srcHash, const char* text) {
    if (!txQueue_) {
        LOGW(TAG, "DM: txQueue not ready, dropping '%s'", text);
        return;
    }

    LoRaTxMsg txMsg;
    txMsg.type = LoRaTxType::MESH_PACKET;
    // time(nullptr) is fine without an RTC sync — the timestamp only keeps the
    // packet hash unique and orders messages.
    uint32_t ts = static_cast<uint32_t>(time(nullptr));
    if (meshcore::DirectMessage::encode(txMsg.packet, secret, destHash, srcHash,
                                        ts, 0, text) == 0) {
        LOGW(TAG, "DM: failed to encode '%s'", text);
        return;
    }

    if (xQueueSend(txQueue_, &txMsg, 0) != pdPASS) {
        LOGW(TAG, "DM: txQueue full, '%s' dropped", text);
    } else {
        LOGI(TAG, "DM: queued direct text '%s' (%u bytes)", text, txMsg.packet.payloadLen);
    }
}

void MeshMessenger::handleDirectMessage(const meshcore::MeshPacket& pkt, DisplayState& st, int pktCount) {
    uint8_t destHash, srcHash;
    if (!meshcore::DirectMessage::readHashes(pkt, destHash, srcHash)) {
        LOGW(TAG, "#%d TXT_MSG: malformed", pktCount);
        return;
    }

    const meshcore::MeshIdentity& self = identity();
    if (destHash != self.pubKey[0]) {
        LOGI(TAG, "#%d TXT_MSG not addressed to us (dest=%02X)", pktCount, destHash);
        return;
    }

    char text[meshcore::DM_MAX_TEXT_LEN + 1];
    uint32_t ts;
    uint8_t txtType, attempt;
    size_t cursor = 0;
    int idx;
    while ((idx = contacts_.nextByHash(srcHash, &cursor)) >= 0) {
        const uint8_t* secret = contacts_.sharedSecret(static_cast<size_t>(idx), self);
        if (!secret) continue;
        if (!meshcore::DirectMessage::decode(pkt, secret, ts, txtType, attempt,
                                             text, sizeof(text))) {
            continue;  // wrong contact for this prefix collision — keep trying
        }

        meshcore::Contact* c = contacts_.at(static_cast<size_t>(idx));
        const char* from = (c && c->name[0]) ? c->name : "?";
        LOGI(TAG, "#%d RX DM from '%s': \"%s\" (ts=%u type=%d)",
             pktCount, from, text, ts, txtType);
        strncpy(st.lastAdvertName, from, sizeof(st.lastAdvertName) - 1);
        st.lastAdvertName[sizeof(st.lastAdvertName) - 1] = '\0';

        // Reply with a MeshCore ACK (hash bound to the sender's pubkey), then a
        // direct "Hello World" text back to the sender.
        if (c) {
            uint8_t ack[meshcore::DM_ACK_SIZE];
            meshcore::DirectMessage::buildAck(ack, ts, txtType, attempt, text,
                                              c->pubKey,
                                              static_cast<uint8_t>(esp_random() & 0xFF));
            sendDirectAck(ack);

            // Reply goes back to the sender: dest = their hash, src = ours.
            sendDirectText(secret, c->pubKey[0], self.pubKey[0], "Hello World");
        }
        return;
    }

    LOGW(TAG, "#%d TXT_MSG: no known contact matches src=%02X (MAC unverified)",
         pktCount, srcHash);
}
