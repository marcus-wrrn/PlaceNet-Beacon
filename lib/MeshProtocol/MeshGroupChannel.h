#pragma once

#include <cstddef>
#include <cstdint>

#include "MeshCrypto.h"
#include "MeshPacket.h"

// MeshCore-compatible group/channel text (PAYLOAD_TYPE_GRP_TXT, 0x05) codec.
//
// A channel is identified by a shared secret (a 16- or 32-byte PSK, often
// distributed base64-encoded as in MeshCore). The on-wire payload of a GRP_TXT
// packet is:
//
//   [channel_hash:1][mac:2][ciphertext:16*n]
//
//   channel_hash  = SHA256(secret)[0]
//   mac           = first 2 bytes of HMAC-SHA256(secret_32, ciphertext)
//   ciphertext    = AES-128-ECB(secret_16, plaintext), zero-padded
//
// Decrypted plaintext:
//
//   [timestamp:4 LE][txt_type:1 = 0x00]["<sender>: <message>"]   (NUL-padded)
//
// This codec only frames and (de)crypts GRP_TXT payloads — it does not send,
// route, or manage channel membership.
namespace meshcore {

// Maximum "<sender>: <message>" content length (MeshCore MAX_TEXT_LEN).
static constexpr size_t GRP_TXT_MAX_TEXT_LEN = 160;
// Plaintext header: timestamp (4) + txt_type (1).
static constexpr size_t GRP_TXT_HEADER_SIZE  = 5;
static constexpr uint8_t GRP_TXT_TYPE_PLAIN  = 0x00;

class GroupChannel {
public:
    uint8_t secret[CIPHER_SECRET_SIZE];  // 32 bytes, zero-padded if PSK is 16
    uint8_t secretLen;                   // actual key length: 16 or 32
    uint8_t hash;                         // SHA256(secret, secretLen)[0]

    GroupChannel();

    // Build a channel from a raw 16- or 32-byte secret. Returns false on an
    // unsupported length.
    bool fromSecret(const uint8_t* key, size_t len);

    // Build a channel from a base64-encoded PSK (decodes to 16 or 32 bytes,
    // matching MeshCore channel config). Returns false on bad input/length.
    bool fromBase64PSK(const char* base64);

    bool valid() const { return secretLen == 16 || secretLen == 32; }

    // Encode a GRP_TXT message into `pkt` (sets payload, payloadType,
    // routeType). `sender` and `text` are NUL-terminated C strings; the wire
    // form is "<sender>: <text>", truncated to GRP_TXT_MAX_TEXT_LEN. Returns
    // the payload length written, or 0 on error.
    size_t encode(MeshPacket& pkt, uint32_t timestamp, const char* sender, const char* text) const;

    // Decode a GRP_TXT packet whose channel_hash matches this channel and whose
    // MAC verifies under this channel's secret. On success writes the message
    // ("<sender>: <text>") into `outText` (NUL-terminated, bounded by `cap`)
    // and the timestamp into `outTimestamp`, returning true. Returns false if
    // the packet is not GRP_TXT, the hash doesn't match, the MAC fails, or the
    // plaintext is malformed.
    bool decode(const MeshPacket& pkt, uint32_t& outTimestamp, char* outText, size_t cap) const;
};

// Decode a standard base64 string into `out` (capacity `cap`). Returns the
// number of bytes decoded, or 0 on invalid input / overflow.
size_t decodeBase64(const char* in, uint8_t* out, size_t cap);

}  // namespace meshcore
