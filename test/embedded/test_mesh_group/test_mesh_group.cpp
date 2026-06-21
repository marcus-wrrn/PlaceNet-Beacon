#include <Arduino.h>
#include <unity.h>

#include "MeshCrypto.h"
#include "MeshGroupChannel.h"
#include "MeshPacket.h"

using namespace meshcore;

void setUp(void) {}
void tearDown(void) {}

// MeshCore "public" channel PSK (base64 of the well-known 16-byte key).
static const char* PUBLIC_PSK = "izOH6cXN6mrJ5e26oRXNcg==";

// ── base64 ──────────────────────────────────────────────────────────────────
void test_base64_decode_16(void) {
    uint8_t out[32];
    size_t n = decodeBase64(PUBLIC_PSK, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(16, n);
}

void test_base64_rejects_garbage(void) {
    uint8_t out[32];
    TEST_ASSERT_EQUAL_size_t(0, decodeBase64("not*base*64", out, sizeof(out)));
}

// ── channel derivation ────────────────────────────────────────────────────────
void test_channel_hash_deterministic(void) {
    GroupChannel a, b;
    TEST_ASSERT_TRUE(a.fromBase64PSK(PUBLIC_PSK));
    TEST_ASSERT_TRUE(b.fromBase64PSK(PUBLIC_PSK));
    TEST_ASSERT_TRUE(a.valid());
    TEST_ASSERT_EQUAL_UINT8(a.hash, b.hash);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a.secret, b.secret, CIPHER_SECRET_SIZE);
}

void test_channel_rejects_bad_length(void) {
    GroupChannel c;
    uint8_t key[20] = {0};
    TEST_ASSERT_FALSE(c.fromSecret(key, sizeof(key)));
    TEST_ASSERT_FALSE(c.valid());
}

// ── crypto round-trip ─────────────────────────────────────────────────────────
void test_encrypt_then_mac_roundtrip(void) {
    GroupChannel c;
    c.fromBase64PSK(PUBLIC_PSK);

    const char* msg = "hello mesh world, this spans two blocks";
    uint8_t enc[MESH_MAX_PAYLOAD_SIZE];
    size_t encLen = encryptThenMAC(c.secret, enc, (const uint8_t*)msg, strlen(msg));
    TEST_ASSERT_EQUAL_size_t(0, (encLen - CIPHER_MAC_SIZE) % CIPHER_BLOCK_SIZE);

    uint8_t dec[MESH_MAX_PAYLOAD_SIZE];
    size_t decLen = MACThenDecrypt(c.secret, dec, enc, encLen);
    TEST_ASSERT_TRUE(decLen >= strlen(msg));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(msg, dec, strlen(msg));
}

void test_mac_rejects_tamper(void) {
    GroupChannel c;
    c.fromBase64PSK(PUBLIC_PSK);
    uint8_t enc[64];
    size_t encLen = encryptThenMAC(c.secret, enc, (const uint8_t*)"abc", 3);
    enc[encLen - 1] ^= 0x01;  // flip a ciphertext bit
    uint8_t dec[64];
    TEST_ASSERT_EQUAL_size_t(0, MACThenDecrypt(c.secret, dec, enc, encLen));
}

// ── GRP_TXT packet codec ──────────────────────────────────────────────────────
void test_grp_txt_roundtrip(void) {
    GroupChannel c;
    c.fromBase64PSK(PUBLIC_PSK);

    MeshPacket pkt;
    size_t n = c.encode(pkt, 0x12345678, "alice", "hi everyone");
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_EQUAL(PAYLOAD_TYPE_GRP_TXT, pkt.payloadType);
    TEST_ASSERT_EQUAL(ROUTE_TYPE_FLOOD, pkt.routeType);
    TEST_ASSERT_EQUAL_UINT8(c.hash, pkt.payload[0]);

    uint32_t ts = 0;
    char text[200];
    TEST_ASSERT_TRUE(c.decode(pkt, ts, text, sizeof(text)));
    TEST_ASSERT_EQUAL_HEX32(0x12345678, ts);
    TEST_ASSERT_EQUAL_STRING("alice: hi everyone", text);
}

void test_grp_txt_wrong_channel_rejected(void) {
    GroupChannel sender, other;
    sender.fromBase64PSK(PUBLIC_PSK);
    uint8_t k2[16];
    for (int i = 0; i < 16; i++) k2[i] = (uint8_t)(i + 1);
    other.fromSecret(k2, sizeof(k2));

    MeshPacket pkt;
    sender.encode(pkt, 1000, "bob", "secret");

    uint32_t ts;
    char text[64];
    // Different hash → rejected outright (unless the 1-byte hashes happen to
    // collide, in which case the MAC check still rejects it).
    bool ok = other.decode(pkt, ts, text, sizeof(text));
    TEST_ASSERT_FALSE(ok);
}

// ── runner ────────────────────────────────────────────────────────────────────
void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_base64_decode_16);
    RUN_TEST(test_base64_rejects_garbage);
    RUN_TEST(test_channel_hash_deterministic);
    RUN_TEST(test_channel_rejects_bad_length);
    RUN_TEST(test_encrypt_then_mac_roundtrip);
    RUN_TEST(test_mac_rejects_tamper);
    RUN_TEST(test_grp_txt_roundtrip);
    RUN_TEST(test_grp_txt_wrong_channel_rejected);
    UNITY_END();
}

void loop() {}
