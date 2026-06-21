#pragma once

#include <cstddef>
#include <cstdint>
namespace meshcore {

static constexpr size_t ADV_PUB_KEY_SIZE     = 32;
static constexpr size_t ADV_TIMESTAMP_SIZE   = 4;
static constexpr size_t ADV_SIGNATURE_SIZE   = 64;
static constexpr size_t ADV_MAX_APP_DATA     = 32;  // MeshCore MAX_ADVERT_DATA_SIZE
static constexpr size_t ADV_FIXED_SIZE       = ADV_PUB_KEY_SIZE + ADV_TIMESTAMP_SIZE +
                                               ADV_SIGNATURE_SIZE;  // 100

enum AdvertType : uint8_t {
    ADV_TYPE_NONE               = 0,
    ADV_TYPE_CHAT               = 1,
    ADV_TYPE_REPEATER           = 2,
    ADV_TYPE_ROOM               = 3,
    ADV_TYPE_SENSOR             = 4,
    ADV_TYPE_PLACENET_BEACON    = 12,   // type is non-Meshcore, goes up to 15 but decided with 12 to minimize potential conflicts in the future 
};

// app_data flag bits — high nibble of the flags byte.
static constexpr uint8_t ADV_LATLON_MASK = 0x10;
static constexpr uint8_t ADV_FEAT1_MASK  = 0x20;
static constexpr uint8_t ADV_FEAT2_MASK  = 0x40;
static constexpr uint8_t ADV_NAME_MASK   = 0x80;
static constexpr uint8_t ADV_TYPE_MASK   = 0x0F;

class Advert {
public:
    uint8_t  pubKey[ADV_PUB_KEY_SIZE];
    uint32_t timestamp;
    uint8_t  signature[ADV_SIGNATURE_SIZE];
    uint8_t  type;       // ADV_TYPE_* (low nibble of flags)
    bool     hasLoc;
    int32_t  latE6;      // latitude  * 1e6
    int32_t  lonE6;      // longitude * 1e6
    uint16_t feat1;
    uint16_t feat2;
    char     name[ADV_MAX_APP_DATA + 1];  // NUL-terminated; "" if absent

    // Raw app_data bytes exactly as parsed/encoded. This is what the signature
    // covers, so verification must use these bytes (not a re-encode).
    uint8_t  appData[ADV_MAX_APP_DATA];
    uint8_t  appDataLen;

    Advert();

    bool   hasName() const { return name[0] != 0; }
    double lat() const { return static_cast<double>(latE6) / 1e6; }
    double lon() const { return static_cast<double>(lonE6) / 1e6; }

    // ── Phase 2: decode (no crypto) ──────────────────────────────────────────
    // Parse a full ADVERT payload into this object. Returns false if the buffer
    // is too short for the fixed header or the app_data exceeds limits. Does NOT
    // verify the signature (see MeshIdentity::verifyAdvert).
    bool parse(const uint8_t* payload, size_t len);

    // ── Building (used by Phase 3 signing) ───────────────────────────────────
    // Setters mutate the decoded fields; call encodeAppData() afterwards (or let
    // MeshIdentity::signAdvert do it) to refresh appData[].
    void setType(uint8_t t)            { type = t & ADV_TYPE_MASK; }
    void setLocation(double la, double lo);
    void clearLocation()               { hasLoc = false; }
    void setFeat1(uint16_t v)          { feat1 = v; }
    void setFeat2(uint16_t v)          { feat2 = v; }
    void setName(const char* n);

    // Encode the decoded fields into appData[]/appDataLen (mirrors MeshCore's
    // AdvertDataBuilder). Returns the encoded length, or 0 on overflow.
    size_t encodeAppData();

    // Build the signable message (public_key || timestamp || app_data) into
    // `out`. Uses the current appData[]. Returns bytes written, or 0 if too big.
    size_t buildSignableMessage(uint8_t* out, size_t cap) const;

    // Serialize the full ADVERT payload. `signature` must already be set.
    // Returns bytes written, or 0 if it would exceed `cap`.
    size_t serialize(uint8_t* out, size_t cap) const;

    // Total serialized size for the current app_data.
    size_t payloadSize() const { return ADV_FIXED_SIZE + appDataLen; }
};

}  // namespace meshcore
