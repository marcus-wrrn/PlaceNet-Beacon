#include "MeshAdvert.h"

#include <cstring>

namespace meshcore {

// Little-endian helpers (MeshCore is little-endian on the wire, ESP32 too, but
// we stay explicit so the codec is portable and not memcpy-aliasing dependent).
static inline void putU16LE(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
static inline void putU32LE(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}
static inline uint16_t getU16LE(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
static inline uint32_t getU32LE(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

Advert::Advert()
    : timestamp(0),
      type(ADV_TYPE_NONE),
      hasLoc(false),
      latE6(0),
      lonE6(0),
      feat1(0),
      feat2(0),
      appDataLen(0) {
    memset(pubKey, 0, sizeof(pubKey));
    memset(signature, 0, sizeof(signature));
    memset(appData, 0, sizeof(appData));
    name[0] = 0;
}

void Advert::setLocation(double la, double lo) {
    hasLoc = true;
    latE6 = static_cast<int32_t>(la * 1e6);
    lonE6 = static_cast<int32_t>(lo * 1e6);
}

void Advert::setName(const char* n) {
    if (n == nullptr) { name[0] = 0; return; }
    size_t i = 0;
    for (; n[i] != 0 && i < ADV_MAX_APP_DATA; ++i) name[i] = n[i];
    name[i] = 0;
}

size_t Advert::encodeAppData() {
    // Build flags byte as we go (mirrors AdvertDataBuilder::encodeTo). Note that
    // MeshCore only sets a feature flag when its value is non-zero.
    uint8_t  buf[ADV_MAX_APP_DATA];
    uint8_t  flags = type & ADV_TYPE_MASK;
    size_t   i = 1;  // byte 0 is flags, filled at the end

    if (hasLoc) {
        if (i + 8 > ADV_MAX_APP_DATA) return 0;
        flags |= ADV_LATLON_MASK;
        putU32LE(&buf[i], static_cast<uint32_t>(latE6)); i += 4;
        putU32LE(&buf[i], static_cast<uint32_t>(lonE6)); i += 4;
    }
    if (feat1) {
        if (i + 2 > ADV_MAX_APP_DATA) return 0;
        flags |= ADV_FEAT1_MASK;
        putU16LE(&buf[i], feat1); i += 2;
    }
    if (feat2) {
        if (i + 2 > ADV_MAX_APP_DATA) return 0;
        flags |= ADV_FEAT2_MASK;
        putU16LE(&buf[i], feat2); i += 2;
    }
    if (name[0] != 0) {
        flags |= ADV_NAME_MASK;
        for (size_t k = 0; name[k] != 0 && i < ADV_MAX_APP_DATA; ++k) {
            buf[i++] = static_cast<uint8_t>(name[k]);
        }
    }

    buf[0] = flags;
    memcpy(appData, buf, i);
    appDataLen = static_cast<uint8_t>(i);
    return i;
}

size_t Advert::buildSignableMessage(uint8_t* out, size_t cap) const {
    const size_t total = ADV_PUB_KEY_SIZE + ADV_TIMESTAMP_SIZE + appDataLen;
    if (out == nullptr || cap < total) return 0;

    size_t pos = 0;
    memcpy(out + pos, pubKey, ADV_PUB_KEY_SIZE); pos += ADV_PUB_KEY_SIZE;
    putU32LE(out + pos, timestamp); pos += ADV_TIMESTAMP_SIZE;
    memcpy(out + pos, appData, appDataLen); pos += appDataLen;
    return pos;
}

size_t Advert::serialize(uint8_t* out, size_t cap) const {
    if (appDataLen > ADV_MAX_APP_DATA) return 0;
    const size_t total = payloadSize();
    if (out == nullptr || cap < total) return 0;

    size_t pos = 0;
    memcpy(out + pos, pubKey, ADV_PUB_KEY_SIZE); pos += ADV_PUB_KEY_SIZE;
    putU32LE(out + pos, timestamp); pos += ADV_TIMESTAMP_SIZE;
    memcpy(out + pos, signature, ADV_SIGNATURE_SIZE); pos += ADV_SIGNATURE_SIZE;
    memcpy(out + pos, appData, appDataLen); pos += appDataLen;
    return pos;
}

bool Advert::parse(const uint8_t* payload, size_t len) {
    if (payload == nullptr || len < ADV_FIXED_SIZE) return false;

    size_t pos = 0;
    memcpy(pubKey, payload + pos, ADV_PUB_KEY_SIZE); pos += ADV_PUB_KEY_SIZE;
    timestamp = getU32LE(payload + pos); pos += ADV_TIMESTAMP_SIZE;
    memcpy(signature, payload + pos, ADV_SIGNATURE_SIZE); pos += ADV_SIGNATURE_SIZE;

    // Remainder is app_data. MeshCore clamps to MAX_ADVERT_DATA_SIZE.
    size_t adLen = len - pos;
    if (adLen > ADV_MAX_APP_DATA) adLen = ADV_MAX_APP_DATA;
    memcpy(appData, payload + pos, adLen);
    appDataLen = static_cast<uint8_t>(adLen);

    // Decode app_data fields (mirrors AdvertDataParser). Reset to defaults first.
    type = ADV_TYPE_NONE;
    hasLoc = false;
    latE6 = lonE6 = 0;
    feat1 = feat2 = 0;
    name[0] = 0;

    if (adLen == 0) return true;  // no app_data is still a structurally valid advert

    const uint8_t flags = appData[0];
    type = flags & ADV_TYPE_MASK;
    size_t i = 1;

    if (flags & ADV_LATLON_MASK) {
        if (i + 8 > adLen) return false;
        latE6 = static_cast<int32_t>(getU32LE(&appData[i])); i += 4;
        lonE6 = static_cast<int32_t>(getU32LE(&appData[i])); i += 4;
        hasLoc = true;
    }
    if (flags & ADV_FEAT1_MASK) {
        if (i + 2 > adLen) return false;
        feat1 = getU16LE(&appData[i]); i += 2;
    }
    if (flags & ADV_FEAT2_MASK) {
        if (i + 2 > adLen) return false;
        feat2 = getU16LE(&appData[i]); i += 2;
    }
    if (flags & ADV_NAME_MASK) {
        size_t nlen = (adLen > i) ? (adLen - i) : 0;  // name is the remainder
        if (nlen > ADV_MAX_APP_DATA) nlen = ADV_MAX_APP_DATA;
        memcpy(name, &appData[i], nlen);
        name[nlen] = 0;
    }
    return true;
}

}  // namespace meshcore
