#pragma once

#include <cstddef>
#include <cstdint>

#include "MeshCrypto.h"
#include "MeshPacket.h"

// MeshCore-compatible direct (1-to-1) encrypted text (PAYLOAD_TYPE_TXT_MSG, 0x02).
//
// Unlike GRP_TXT (which uses a shared channel PSK), a TXT_MSG is encrypted with
// the per-pair X25519 ECDH shared secret between sender and recipient
// (MeshIdentity::calcSharedSecret). The same AES-128-ECB + 2-byte HMAC scheme
// as GRP_TXT is reused — only the key source and payload prefix differ.
//
// On-wire payload:
//
//   [dest_hash:1][src_hash:1][mac:2][ciphertext:16*n]
//
//   dest_hash / src_hash = pubKey[0] of recipient / sender (1-byte prefix)
//   mac + ciphertext     = encryptThenMAC(shared_secret, plaintext)
//
// Decrypted plaintext:
//
//   [timestamp:4 LE][flags:1][text… NUL-terminated, zero-padded]
//
//   flags = (attempt & 0x03) | (txt_type << 2)
//
// NOTE: the 1-byte hashes are ambiguous, so a receiver must try every known
// contact whose pubKey[0] matches src_hash and keep the one whose MAC verifies.
// Retransmits with attempt > 3 (MeshCore's trailing extended-attempt byte) are
// not produced here.
namespace meshcore {

static constexpr size_t  DM_SHARED_SECRET_SIZE = 32;
static constexpr size_t  DM_MAX_TEXT_LEN       = 160;  // MeshCore MAX_TEXT_LEN
static constexpr size_t  DM_HEADER_SIZE        = 5;    // timestamp(4) + flags(1)
static constexpr size_t  DM_HASH_PREFIX_SIZE   = 2;    // dest_hash + src_hash
static constexpr size_t  DM_ACK_SIZE           = 6;    // hash(4) + attempt(1) + rand(1)

static constexpr uint8_t DM_TXT_TYPE_PLAIN     = 0;
static constexpr uint8_t DM_TXT_TYPE_CLI_DATA  = 1;
static constexpr uint8_t DM_TXT_TYPE_SIGNED    = 2;

struct DirectMessage {
    // Read the dest/src hash prefix from a TXT_MSG packet. Returns false if the
    // packet is not a TXT_MSG or is too short to hold the prefix + a MAC.
    static bool readHashes(const MeshPacket& pkt, uint8_t& destHash, uint8_t& srcHash);

    // Encode a TXT_MSG into `pkt` (sets payload, payloadType, routeType=FLOOD).
    // `secret` is the 32-byte shared secret; destHash/srcHash are pubKey[0] of
    // recipient/sender; `attempt` is 0..3. Returns the payload length written,
    // or 0 on error (bad args / oversized).
    static size_t encode(MeshPacket& pkt, const uint8_t secret[DM_SHARED_SECRET_SIZE],
                         uint8_t destHash, uint8_t srcHash,
                         uint32_t timestamp, uint8_t attempt, const char* text);

    // Decrypt a TXT_MSG using `secret`, verifying the MAC. On success writes the
    // message text (NUL-terminated, bounded by `cap`), timestamp, txt_type and
    // attempt, returning true. Returns false if `pkt` is not a TXT_MSG, the MAC
    // fails (wrong secret / not addressed to this key), or the plaintext is
    // malformed.
    static bool decode(const MeshPacket& pkt, const uint8_t secret[DM_SHARED_SECRET_SIZE],
                       uint32_t& outTimestamp, uint8_t& outTxtType, uint8_t& outAttempt,
                       char* outText, size_t cap);

    // Build the 6-byte MeshCore ACK the recipient returns to confirm delivery:
    //   [sha256(timestamp||flags||text || senderPubKey)[0..3]][ext_attempt:1][rand:1]
    // `txtType`/`attempt` reconstruct the flags byte; `senderPubKey` is the
    // original sender's 32-byte public key; `randByte` makes the packet hash
    // unique. ext_attempt is 0 (attempt > 3 retransmits unsupported). The ACK
    // is sent in a PAYLOAD_TYPE_ACK packet's payload.
    static void buildAck(uint8_t out[DM_ACK_SIZE], uint32_t timestamp, uint8_t txtType,
                         uint8_t attempt, const char* text,
                         const uint8_t senderPubKey[32], uint8_t randByte);
};

}  // namespace meshcore
