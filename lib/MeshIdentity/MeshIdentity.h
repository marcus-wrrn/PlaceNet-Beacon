#pragma once

#include <cstddef>
#include <cstdint>

#include "MeshAdvert.h"

// Ed25519 identity for MeshCore interop
//
// Wraps the vendored orlp/ed25519 implementation (lib/ed25519). Key sizes match
// MeshCore exactly: 32-byte public key, 64-byte expanded private key (the
// SHA-512 of the seed, clamped per RFC 8032). Signatures produced here are
// standard Ed25519 and verify under the rweather Crypto Ed25519::verify that
// real MeshCore nodes use, so signed ADVERTs are accepted as genuine.
namespace meshcore {

class MeshIdentity {
public:
    uint8_t pubKey[ADV_PUB_KEY_SIZE];  // 32
    uint8_t prvKey[64];                // expanded private key

    MeshIdentity();

    // Derive a keypair from a 32-byte seed (deterministic). Always available.
    void fromSeed(const uint8_t seed[32]);
    bool generate();
    void fromPrivateKey(const uint8_t prv[64]);
    void sign(uint8_t sigOut[ADV_SIGNATURE_SIZE], const uint8_t* msg, size_t len) const;

    // Raw verification against an arbitrary public key. Returns true if valid.
    static bool verify(const uint8_t sig[ADV_SIGNATURE_SIZE], const uint8_t pubKey[ADV_PUB_KEY_SIZE],
                       const uint8_t* msg, size_t len);

    // ── High-level ADVERT helpers ────────────────────────────────────────────
    // Fill `adv` with this identity's public key + given timestamp, (re)encode
    // its app_data from its current fields, and sign it. After this the advert
    // is ready to serialize() into a PAYLOAD_TYPE_ADVERT packet. Returns false
    // if app_data encoding overflows.
    bool signAdvert(Advert& adv, uint32_t timestamp) const;

    // Verify a parsed advert's signature against the public key it carries.
    static bool verifyAdvert(const Advert& adv);
};

}  // namespace meshcore
