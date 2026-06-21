#include "MeshDirectMessage.h"

#include <cstring>

namespace meshcore {

bool DirectMessage::readHashes(const MeshPacket& pkt, uint8_t& destHash, uint8_t& srcHash) {
    if (pkt.payloadType != PAYLOAD_TYPE_TXT_MSG) return false;
    if (pkt.payloadLen < DM_HASH_PREFIX_SIZE + CIPHER_MAC_SIZE) return false;
    destHash = pkt.payload[0];
    srcHash  = pkt.payload[1];
    return true;
}

size_t DirectMessage::encode(MeshPacket& pkt, const uint8_t secret[DM_SHARED_SECRET_SIZE],
                             uint8_t destHash, uint8_t srcHash,
                             uint32_t timestamp, uint8_t attempt, const char* text) {
    if (secret == nullptr || text == nullptr) return 0;

    // Build plaintext: [ts:4][flags:1][text…]. No "<sender>: " prefix (unlike GRP_TXT).
    uint8_t plain[DM_HEADER_SIZE + DM_MAX_TEXT_LEN];
    plain[0] = static_cast<uint8_t>(timestamp & 0xFF);
    plain[1] = static_cast<uint8_t>((timestamp >> 8) & 0xFF);
    plain[2] = static_cast<uint8_t>((timestamp >> 16) & 0xFF);
    plain[3] = static_cast<uint8_t>((timestamp >> 24) & 0xFF);
    plain[4] = static_cast<uint8_t>((attempt & 0x03) | (DM_TXT_TYPE_PLAIN << 2));

    size_t n = 0;
    for (const char* t = text; *t && n < DM_MAX_TEXT_LEN; ++t) {
        plain[DM_HEADER_SIZE + n] = static_cast<uint8_t>(*t);
        ++n;
    }
    const size_t plainLen = DM_HEADER_SIZE + n;

    // [dest_hash:1][src_hash:1][mac:2][ciphertext]
    const size_t blocks     = (plainLen + CIPHER_BLOCK_SIZE - 1) / CIPHER_BLOCK_SIZE;
    const size_t payloadLen = DM_HASH_PREFIX_SIZE + CIPHER_MAC_SIZE + blocks * CIPHER_BLOCK_SIZE;
    if (payloadLen > MESH_MAX_PAYLOAD_SIZE) return 0;

    pkt.payloadType = PAYLOAD_TYPE_TXT_MSG;
    pkt.routeType   = ROUTE_TYPE_FLOOD;

    pkt.payload[0] = destHash;
    pkt.payload[1] = srcHash;
    size_t macEnc  = encryptThenMAC(secret, &pkt.payload[DM_HASH_PREFIX_SIZE], plain, plainLen);
    pkt.payloadLen = static_cast<uint8_t>(DM_HASH_PREFIX_SIZE + macEnc);
    return pkt.payloadLen;
}

bool DirectMessage::decode(const MeshPacket& pkt, const uint8_t secret[DM_SHARED_SECRET_SIZE],
                           uint32_t& outTimestamp, uint8_t& outTxtType, uint8_t& outAttempt,
                           char* outText, size_t cap) {
    if (secret == nullptr || outText == nullptr || cap == 0) return false;
    if (pkt.payloadType != PAYLOAD_TYPE_TXT_MSG) return false;
    if (pkt.payloadLen < DM_HASH_PREFIX_SIZE + CIPHER_MAC_SIZE) return false;

    uint8_t plain[MESH_MAX_PAYLOAD_SIZE];
    size_t len = MACThenDecrypt(secret, plain, &pkt.payload[DM_HASH_PREFIX_SIZE],
                                pkt.payloadLen - DM_HASH_PREFIX_SIZE);
    if (len < DM_HEADER_SIZE) return false;  // MAC failed (len==0) or too short

    outTimestamp = static_cast<uint32_t>(plain[0]) |
                   (static_cast<uint32_t>(plain[1]) << 8) |
                   (static_cast<uint32_t>(plain[2]) << 16) |
                   (static_cast<uint32_t>(plain[3]) << 24);
    outAttempt = plain[4] & 0x03;
    outTxtType = plain[4] >> 2;

    // Text runs from offset 5 up to the first NUL (zero padding) or `len`.
    size_t textLen = 0;
    while (DM_HEADER_SIZE + textLen < len && plain[DM_HEADER_SIZE + textLen] != 0) {
        ++textLen;
    }
    size_t copyLen = textLen < (cap - 1) ? textLen : (cap - 1);
    memcpy(outText, &plain[DM_HEADER_SIZE], copyLen);
    outText[copyLen] = 0;
    return true;
}

void DirectMessage::buildAck(uint8_t out[DM_ACK_SIZE], uint32_t timestamp, uint8_t txtType,
                             uint8_t attempt, const char* text,
                             const uint8_t senderPubKey[32], uint8_t randByte) {
    // MeshCore hashes (timestamp || flags || text) concatenated with the
    // sender's public key, truncated to 4 bytes.
    uint8_t msg[DM_HEADER_SIZE + DM_MAX_TEXT_LEN + 32];
    msg[0] = static_cast<uint8_t>(timestamp & 0xFF);
    msg[1] = static_cast<uint8_t>((timestamp >> 8) & 0xFF);
    msg[2] = static_cast<uint8_t>((timestamp >> 16) & 0xFF);
    msg[3] = static_cast<uint8_t>((timestamp >> 24) & 0xFF);
    msg[4] = static_cast<uint8_t>((attempt & 0x03) | (txtType << 2));

    size_t textLen = 0;
    if (text != nullptr) {
        while (text[textLen] && textLen < DM_MAX_TEXT_LEN) ++textLen;
        memcpy(&msg[DM_HEADER_SIZE], text, textLen);
    }
    memcpy(&msg[DM_HEADER_SIZE + textLen], senderPubKey, 32);

    uint8_t h[4];
    sha256(h, sizeof(h), msg, DM_HEADER_SIZE + textLen + 32);
    memcpy(out, h, 4);
    out[4] = 0;          // ext_attempt (attempt > 3 retransmits unsupported)
    out[5] = randByte;   // keeps the ACK packet hash unique
}

}  // namespace meshcore
