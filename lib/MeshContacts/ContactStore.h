#pragma once

#include <cstddef>
#include <cstdint>

#include "MeshIdentity.h"

// RAM-resident table of known peer identities for MeshCore direct messaging.
//
// A direct PAYLOAD_TYPE_TXT_MSG only carries a 1-byte src_hash, so decrypting it
// requires the sender's full 32-byte public key (to derive the X25519 shared
// secret). This store keeps those keys — learned from received ADVERTs — and
// caches the per-peer shared secret on first use so the radio RX path never
// recomputes the (expensive) ECDH per packet.
//
// The backing array is allocated from PSRAM when available (both supported
// boards carry 8 MB), falling back to internal RAM, so a large contact table
// does not consume scarce internal SRAM. Per-packet lookup is a handful of
// 1-byte prefix compares plus a cached-secret AES/HMAC, so PSRAM latency is
// irrelevant here.
//
// NOT thread-safe: intended to be owned and accessed by a single task (the
// packet-processing loop in main_task).
namespace meshcore {

struct Contact {
    uint8_t  pubKey[32];
    uint8_t  sharedSecret[32];
    bool     secretValid;
    uint32_t lastSeen;        // timestamp from the most recent ADVERT
    char     name[33];        // NUL-terminated advert name ("" if unknown)
};

class ContactStore {
public:
    explicit ContactStore(size_t capacity = 64);
    ~ContactStore();

    ContactStore(const ContactStore&) = delete;
    ContactStore& operator=(const ContactStore&) = delete;

    // Allocate the backing storage (PSRAM-preferred). Returns false on OOM.
    // Idempotent.
    bool begin();
    bool ready() const { return contacts_ != nullptr; }

    size_t count() const { return count_; }
    size_t capacity() const { return capacity_; }

    // Insert a new contact, or refresh the existing one with the same pubkey
    // (updates name + lastSeen, keeps the cached secret). Returns the contact
    // index, or -1 if storage is uninitialised or full.
    int upsert(const uint8_t pubKey[32], const char* name, uint32_t lastSeen);

    Contact*       at(size_t idx);
    const Contact* at(size_t idx) const;

    // Return the cached shared secret for `idx`, deriving it from `self` on
    // first use. Returns nullptr on a bad index.
    const uint8_t* sharedSecret(size_t idx, const MeshIdentity& self);

    // Iterate contacts whose pubKey[0] == hash: start with *cursor == 0, advance
    // *cursor each call, returns the next matching index or -1 when exhausted.
    int nextByHash(uint8_t hash, size_t* cursor) const;

private:
    Contact* contacts_;
    size_t   capacity_;
    size_t   count_;
};

}  // namespace meshcore
