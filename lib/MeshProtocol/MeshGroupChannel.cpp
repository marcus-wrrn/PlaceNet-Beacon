#include "MeshGroupChannel.h"

#include <cstring>

namespace meshcore {

GroupChannel::GroupChannel() : secretLen(0), hash(0) {
    memset(secret, 0, sizeof(secret));
}

bool GroupChannel::fromSecret(const uint8_t* key, size_t len) {
    if (key == nullptr || (len != 16 && len != 32)) return false;
    memset(secret, 0, sizeof(secret));
    memcpy(secret, key, len);
    secretLen = static_cast<uint8_t>(len);
    // Channel hash is SHA256 over the actual key length (matches MeshCore).
    uint8_t h[1];
    sha256(h, sizeof(h), secret, secretLen);
    hash = h[0];
    return true;
}

bool GroupChannel::fromBase64PSK(const char* base64) {
    uint8_t key[CIPHER_SECRET_SIZE];
    size_t len = decodeBase64(base64, key, sizeof(key));
    return fromSecret(key, len);
}

size_t GroupChannel::encode(MeshPacket& pkt, uint32_t timestamp, const char* sender, const char* text) const {
    if (!valid() || sender == nullptr || text == nullptr) return 0;

    // Build plaintext: [ts:4][txt_type:1]["<sender>: <text>"]
    uint8_t plain[GRP_TXT_HEADER_SIZE + GRP_TXT_MAX_TEXT_LEN];
    plain[0] = static_cast<uint8_t>(timestamp & 0xFF);
    plain[1] = static_cast<uint8_t>((timestamp >> 8) & 0xFF);
    plain[2] = static_cast<uint8_t>((timestamp >> 16) & 0xFF);
    plain[3] = static_cast<uint8_t>((timestamp >> 24) & 0xFF);
    plain[4] = GRP_TXT_TYPE_PLAIN;

    char* body = reinterpret_cast<char*>(&plain[GRP_TXT_HEADER_SIZE]);
    const size_t cap = GRP_TXT_MAX_TEXT_LEN;

    // "<sender>: " prefix, then as much of <text> as fits.
    size_t n = 0;
    for (const char* s = sender; *s && n < cap; ++s) body[n++] = *s;
    if (n + 2 <= cap) { body[n++] = ':'; body[n++] = ' '; }
    for (const char* t = text; *t && n < cap; ++t) body[n++] = *t;

    const size_t plainLen = GRP_TXT_HEADER_SIZE + n;

    // [hash:1][mac:2][ciphertext]
    const size_t blocks = (plainLen + CIPHER_BLOCK_SIZE - 1) / CIPHER_BLOCK_SIZE;
    const size_t payloadLen = 1 + CIPHER_MAC_SIZE + blocks * CIPHER_BLOCK_SIZE;
    if (payloadLen > MESH_MAX_PAYLOAD_SIZE) return 0;

    pkt.payloadType = PAYLOAD_TYPE_GRP_TXT;
    pkt.routeType   = ROUTE_TYPE_FLOOD;

    pkt.payload[0] = hash;
    size_t macEnc = encryptThenMAC(secret, &pkt.payload[1], plain, plainLen);
    pkt.payloadLen = static_cast<uint8_t>(1 + macEnc);
    return pkt.payloadLen;
}

bool GroupChannel::decode(const MeshPacket& pkt, uint32_t& outTimestamp, char* outText, size_t cap) const {
    if (!valid() || outText == nullptr || cap == 0) return false;
    if (pkt.payloadType != PAYLOAD_TYPE_GRP_TXT) return false;
    if (pkt.payloadLen < 1 + CIPHER_MAC_SIZE) return false;
    if (pkt.payload[0] != hash) return false;

    uint8_t plain[MESH_MAX_PAYLOAD_SIZE];
    size_t len = MACThenDecrypt(secret, plain, &pkt.payload[1], pkt.payloadLen - 1);
    if (len < GRP_TXT_HEADER_SIZE) return false;       // need ts + txt_type + (text)
    if ((plain[4] >> 2) != 0) return false;            // unsupported txt_type

    outTimestamp = static_cast<uint32_t>(plain[0]) |
                   (static_cast<uint32_t>(plain[1]) << 8) |
                   (static_cast<uint32_t>(plain[2]) << 16) |
                   (static_cast<uint32_t>(plain[3]) << 24);

    // Text runs from offset 5 up to the first NUL (zero padding) or `len`.
    size_t textLen = 0;
    while (GRP_TXT_HEADER_SIZE + textLen < len && plain[GRP_TXT_HEADER_SIZE + textLen] != 0) {
        ++textLen;
    }
    size_t copyLen = textLen < (cap - 1) ? textLen : (cap - 1);
    memcpy(outText, &plain[GRP_TXT_HEADER_SIZE], copyLen);
    outText[copyLen] = 0;
    return true;
}

// ── base64 ────────────────────────────────────────────────────────────────
static int b64Val(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

size_t decodeBase64(const char* in, uint8_t* out, size_t cap) {
    if (in == nullptr || out == nullptr) return 0;

    uint32_t acc = 0;
    int bits = 0;
    size_t n = 0;
    for (const char* p = in; *p; ++p) {
        if (*p == '=' || *p == '\n' || *p == '\r' || *p == ' ') continue;
        int v = b64Val(*p);
        if (v < 0) return 0;  // invalid character
        acc = (acc << 6) | static_cast<uint32_t>(v);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (n >= cap) return 0;  // overflow
            out[n++] = static_cast<uint8_t>((acc >> bits) & 0xFF);
        }
    }
    return n;
}

}  // namespace meshcore
