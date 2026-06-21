#pragma once

#include <cstddef>
#include <cstdint>

// MeshCore-compatible symmetric crypto primitives for group/channel messages.
//
// Mirrors MeshCore's Utils::encryptThenMAC / MACThenDecrypt (src/Utils.cpp):
//   - AES-128-ECB with zero-padding to a 16-byte boundary
//   - 2-byte truncated HMAC-SHA256 prepended to the ciphertext
//   - HMAC key is the full 32-byte channel secret; the AES key is its first
//     16 bytes
//
// These are pure byte-in/byte-out helpers (no radio, no packet framing) built
// on the rweather/Crypto library that real MeshCore firmware also uses.
namespace meshcore {

static constexpr size_t CIPHER_KEY_SIZE   = 16;  // AES-128 key (bytes)
static constexpr size_t CIPHER_BLOCK_SIZE = 16;  // AES block (bytes)
static constexpr size_t CIPHER_MAC_SIZE   = 2;   // truncated HMAC (bytes)
static constexpr size_t CIPHER_SECRET_SIZE = 32; // full channel secret / HMAC key (PUB_KEY_SIZE)

// SHA-256 of `msg` into `hash`, truncated to `hashLen` (<= 32) bytes.
void sha256(uint8_t* hash, size_t hashLen, const uint8_t* msg, size_t msgLen);

// AES-128-ECB encrypt `srcLen` bytes of `src` into `dest`, zero-padding the
// final partial block. `dest` must hold ceil(srcLen/16)*16 bytes (>= 16).
// Returns the number of bytes written (always a multiple of 16).
size_t aesEcbEncrypt(const uint8_t* secret, uint8_t* dest, const uint8_t* src, size_t srcLen);

// AES-128-ECB decrypt `srcLen` (multiple of 16) bytes. Returns bytes written.
size_t aesEcbDecrypt(const uint8_t* secret, uint8_t* dest, const uint8_t* src, size_t srcLen);

// Encrypt `src` then prepend a 2-byte HMAC of the ciphertext. Layout of `dest`:
//   [mac:2][ciphertext:16*n]
// Returns CIPHER_MAC_SIZE + ciphertext length.
size_t encryptThenMAC(const uint8_t* secret, uint8_t* dest, const uint8_t* src, size_t srcLen);

// Verify the 2-byte HMAC over `src` ([mac:2][ciphertext...]) then decrypt.
// Returns the decrypted length, or 0 if the MAC is invalid or `src` is too
// short. `dest` must hold at least (srcLen - CIPHER_MAC_SIZE) bytes.
size_t MACThenDecrypt(const uint8_t* secret, uint8_t* dest, const uint8_t* src, size_t srcLen);

}  // namespace meshcore
