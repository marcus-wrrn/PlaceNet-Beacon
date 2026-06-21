#include <Arduino.h>
#include <unity.h>

#include "MeshPacket.h"
#include "MeshAdvert.h"
#include "MeshIdentity.h"

using namespace meshcore;

void setUp(void) {}
void tearDown(void) {}

// ── Packet framing ──────────────────────────────────────────────────────────
void test_packet_roundtrip_flood(void) {
    MeshPacket p;
    p.routeType = ROUTE_TYPE_FLOOD;
    p.payloadType = PAYLOAD_TYPE_RAW_CUSTOM;
    p.hashSize = 1; p.hopCount = 3;
    p.path[0] = 0xAA; p.path[1] = 0xBB; p.path[2] = 0xCC;
    const char* m = "placenet";
    p.payloadLen = strlen(m); memcpy(p.payload, m, p.payloadLen);

    uint8_t buf[256];
    size_t n = p.serialize(buf, sizeof(buf));
    TEST_ASSERT_EQUAL_size_t(p.encodedSize(), n);
    TEST_ASSERT_EQUAL_HEX8(0x3D, buf[0]);  // route1 type0xF ver0 => (0xF<<2)|1
    TEST_ASSERT_EQUAL_HEX8(0x03, buf[1]);  // 3 hops, 1-byte hashes

    MeshPacket q;
    TEST_ASSERT_TRUE(q.parse(buf, n));
    TEST_ASSERT_EQUAL(ROUTE_TYPE_FLOOD, q.routeType);
    TEST_ASSERT_EQUAL(3, q.hopCount);
    TEST_ASSERT_EQUAL(1, q.hashSize);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(p.path, q.path, 3);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(p.payload, q.payload, q.payloadLen);
}

void test_packet_rejects_reserved_hashsize(void) {
    uint8_t bad[3] = {0x01, 0xC0, 0x00};  // path hash-size code 0b11 (reserved)
    MeshPacket q;
    TEST_ASSERT_FALSE(q.parse(bad, sizeof(bad)));
}

void test_packet_transport_codes_le(void) {
    MeshPacket p;
    p.routeType = ROUTE_TYPE_TRANSPORT_FLOOD;
    p.payloadType = PAYLOAD_TYPE_TXT_MSG;
    p.transportCode1 = 0x1234; p.transportCode2 = 0xABCD;
    p.hashSize = 1; p.hopCount = 0; p.payloadLen = 0;

    uint8_t buf[64];
    size_t n = p.serialize(buf, sizeof(buf));
    TEST_ASSERT_EQUAL_HEX8(0x34, buf[1]);  // LE low byte of tc1
    TEST_ASSERT_EQUAL_HEX8(0x12, buf[2]);

    MeshPacket q;
    TEST_ASSERT_TRUE(q.parse(buf, n));
    TEST_ASSERT_TRUE(q.hasTransportCodes());
    TEST_ASSERT_EQUAL_HEX16(0x1234, q.transportCode1);
    TEST_ASSERT_EQUAL_HEX16(0xABCD, q.transportCode2);
}

// ── ADVERT decode (Phase 2, no crypto) ──────────────────────────────────────
void test_advert_parse_structure(void) {
    // Hand-build an advert payload and confirm field decode.
    Advert a;
    a.setType(ADV_TYPE_REPEATER);
    a.setName("rptr");
    a.setLocation(1.5, -2.25);
    a.timestamp = 0x11223344;
    a.encodeAppData();

    uint8_t buf[256];
    size_t n = a.serialize(buf, sizeof(buf));

    Advert b;
    TEST_ASSERT_TRUE(b.parse(buf, n));
    TEST_ASSERT_EQUAL(ADV_TYPE_REPEATER, b.type);
    TEST_ASSERT_TRUE(b.hasLoc);
    TEST_ASSERT_EQUAL_INT32(1500000, b.latE6);
    TEST_ASSERT_EQUAL_INT32(-2250000, b.lonE6);
    TEST_ASSERT_EQUAL_STRING("rptr", b.name);
    TEST_ASSERT_EQUAL_HEX32(0x11223344, b.timestamp);
}

// ── ADVERT signing/verify (Phase 3, crypto) ─────────────────────────────────
void test_advert_sign_and_verify(void) {
    MeshIdentity id;
    TEST_ASSERT_TRUE(id.generate());  // hardware RNG on ESP32

    Advert a;
    a.setType(ADV_TYPE_CHAT);
    a.setName("beacon");
    TEST_ASSERT_TRUE(id.signAdvert(a, 0xDEADBEEF));

    uint8_t buf[256];
    size_t n = a.serialize(buf, sizeof(buf));

    Advert rx;
    TEST_ASSERT_TRUE(rx.parse(buf, n));
    TEST_ASSERT_TRUE(MeshIdentity::verifyAdvert(rx));

    // Tamper -> verification fails.
    rx.appData[rx.appDataLen - 1] ^= 0x01;
    TEST_ASSERT_FALSE(MeshIdentity::verifyAdvert(rx));
}

void test_identity_deterministic_seed(void) {
    uint8_t seed[32];
    for (int i = 0; i < 32; i++) seed[i] = i;
    MeshIdentity a, b;
    a.fromSeed(seed);
    b.fromSeed(seed);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a.pubKey, b.pubKey, 32);

    // Round-trip via private key restore.
    MeshIdentity c;
    c.fromPrivateKey(a.prvKey);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a.pubKey, c.pubKey, 32);
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_packet_roundtrip_flood);
    RUN_TEST(test_packet_rejects_reserved_hashsize);
    RUN_TEST(test_packet_transport_codes_le);
    RUN_TEST(test_advert_parse_structure);
    RUN_TEST(test_advert_sign_and_verify);
    RUN_TEST(test_identity_deterministic_seed);
    UNITY_END();
}

void loop() {}
