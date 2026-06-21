#include "MeshIdentity.h"

#include <cstring>

extern "C" {
#include "ed_25519.h"
}

#if defined(ESP_PLATFORM) || defined(ARDUINO)
#include <esp_random.h>
#endif

namespace meshcore {

MeshIdentity::MeshIdentity() {
    memset(pubKey, 0, sizeof(pubKey));
    memset(prvKey, 0, sizeof(prvKey));
}

void MeshIdentity::fromSeed(const uint8_t seed[32]) {
    ed25519_create_keypair(pubKey, prvKey, seed);
}

void MeshIdentity::fromPrivateKey(const uint8_t prv[64]) {
    memcpy(prvKey, prv, 64);
    ed25519_derive_pub(pubKey, prvKey);
}

bool MeshIdentity::generate() {
#if defined(ESP_PLATFORM) || defined(ARDUINO)
    uint8_t seed[32];
    esp_fill_random(seed, sizeof(seed));
    fromSeed(seed);
    return true;
#else
    return false;  // no hardware RNG on host builds; caller should use fromSeed()
#endif
}

void MeshIdentity::sign(uint8_t sigOut[ADV_SIGNATURE_SIZE], const uint8_t* msg, size_t len) const {
    ed25519_sign(sigOut, msg, len, pubKey, prvKey);
}

bool MeshIdentity::verify(const uint8_t sig[ADV_SIGNATURE_SIZE], const uint8_t pubKey[ADV_PUB_KEY_SIZE],
                          const uint8_t* msg, size_t len) {
    return ed25519_verify(sig, msg, len, pubKey) != 0;
}

bool MeshIdentity::signAdvert(Advert& adv, uint32_t timestamp) const {
    memcpy(adv.pubKey, pubKey, ADV_PUB_KEY_SIZE);
    adv.timestamp = timestamp;

    if (adv.encodeAppData() == 0) return false;  // app_data overflowed

    // Signed message = public_key || timestamp || app_data.
    uint8_t msg[ADV_PUB_KEY_SIZE + ADV_TIMESTAMP_SIZE + ADV_MAX_APP_DATA];
    size_t msgLen = adv.buildSignableMessage(msg, sizeof(msg));
    if (msgLen == 0) return false;

    sign(adv.signature, msg, msgLen);
    return true;
}

bool MeshIdentity::verifyAdvert(const Advert& adv) {
    uint8_t msg[ADV_PUB_KEY_SIZE + ADV_TIMESTAMP_SIZE + ADV_MAX_APP_DATA];
    size_t msgLen = adv.buildSignableMessage(msg, sizeof(msg));
    if (msgLen == 0) return false;
    return verify(adv.signature, adv.pubKey, msg, msgLen);
}

}  // namespace meshcore
