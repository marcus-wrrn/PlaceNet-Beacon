#include "MeshCrypto.h"

#include <cstring>

#include <AES.h>
#include <SHA256.h>

namespace meshcore {

void sha256(uint8_t* hash, size_t hashLen, const uint8_t* msg, size_t msgLen) {
    SHA256 sha;
    sha.update(msg, msgLen);
    sha.finalize(hash, hashLen);
}

size_t aesEcbEncrypt(const uint8_t* secret, uint8_t* dest, const uint8_t* src, size_t srcLen) {
    AES128 aes;
    aes.setKey(secret, CIPHER_KEY_SIZE);

    uint8_t* dp = dest;
    while (srcLen >= CIPHER_BLOCK_SIZE) {
        aes.encryptBlock(dp, src);
        dp += CIPHER_BLOCK_SIZE;
        src += CIPHER_BLOCK_SIZE;
        srcLen -= CIPHER_BLOCK_SIZE;
    }
    if (srcLen > 0) {  // zero-pad the trailing partial block
        uint8_t tmp[CIPHER_BLOCK_SIZE];
        memset(tmp, 0, sizeof(tmp));
        memcpy(tmp, src, srcLen);
        aes.encryptBlock(dp, tmp);
        dp += CIPHER_BLOCK_SIZE;
    }
    return static_cast<size_t>(dp - dest);
}

size_t aesEcbDecrypt(const uint8_t* secret, uint8_t* dest, const uint8_t* src, size_t srcLen) {
    AES128 aes;
    aes.setKey(secret, CIPHER_KEY_SIZE);

    uint8_t* dp = dest;
    while (srcLen >= CIPHER_BLOCK_SIZE) {
        aes.decryptBlock(dp, src);
        dp += CIPHER_BLOCK_SIZE;
        src += CIPHER_BLOCK_SIZE;
        srcLen -= CIPHER_BLOCK_SIZE;
    }
    return static_cast<size_t>(dp - dest);
}

size_t encryptThenMAC(const uint8_t* secret, uint8_t* dest, const uint8_t* src, size_t srcLen) {
    size_t encLen = aesEcbEncrypt(secret, dest + CIPHER_MAC_SIZE, src, srcLen);

    SHA256 sha;
    sha.resetHMAC(secret, CIPHER_SECRET_SIZE);
    sha.update(dest + CIPHER_MAC_SIZE, encLen);
    sha.finalizeHMAC(secret, CIPHER_SECRET_SIZE, dest, CIPHER_MAC_SIZE);

    return CIPHER_MAC_SIZE + encLen;
}

size_t MACThenDecrypt(const uint8_t* secret, uint8_t* dest, const uint8_t* src, size_t srcLen) {
    if (srcLen <= CIPHER_MAC_SIZE) return 0;

    const size_t encLen = srcLen - CIPHER_MAC_SIZE;
    if (encLen % CIPHER_BLOCK_SIZE != 0) return 0;

    uint8_t mac[CIPHER_MAC_SIZE];
    SHA256 sha;
    sha.resetHMAC(secret, CIPHER_SECRET_SIZE);
    sha.update(src + CIPHER_MAC_SIZE, encLen);
    sha.finalizeHMAC(secret, CIPHER_SECRET_SIZE, mac, CIPHER_MAC_SIZE);

    if (memcmp(mac, src, CIPHER_MAC_SIZE) != 0) return 0;  // invalid HMAC

    return aesEcbDecrypt(secret, dest, src + CIPHER_MAC_SIZE, encLen);
}

}  // namespace meshcore
