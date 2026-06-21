#include <Arduino.h>
#include <unity.h>

#include <cstring>

#include "ContactStore.h"
#include "MeshDirectMessage.h"
#include "MeshIdentity.h"
#include "MeshPacket.h"

using namespace meshcore;

void setUp(void) {}
void tearDown(void) {}

// Two deterministic identities (Alice = sender, Bob = receiver).
static MeshIdentity makeIdentity(uint8_t fill) {
    uint8_t seed[32];
    memset(seed, fill, sizeof(seed));
    MeshIdentity id;
    id.fromSeed(seed);
    return id;
}

// ── ECDH ─────────────────────────────────────────────────────────────────────
// Both ends must derive the same secret, or no direct message can decrypt.
void test_shared_secret_symmetric(void) {
    MeshIdentity alice = makeIdentity(0xA1);
    MeshIdentity bob   = makeIdentity(0xB2);

    uint8_t s1[32], s2[32];
    alice.calcSharedSecret(s1, bob.pubKey);
    bob.calcSharedSecret(s2, alice.pubKey);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(s1, s2, 32);
}

// ── TXT_MSG packet codec ───────────────────────────────────────────────────────
void test_txt_msg_roundtrip(void) {
    MeshIdentity alice = makeIdentity(0xA1);
    MeshIdentity bob   = makeIdentity(0xB2);
    uint8_t secret[32];
    alice.calcSharedSecret(secret, bob.pubKey);

    MeshPacket pkt;
    size_t n = DirectMessage::encode(pkt, secret, bob.pubKey[0], alice.pubKey[0],
                                     0xCAFEBABE, 0, "ping over a direct link");
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_EQUAL(PAYLOAD_TYPE_TXT_MSG, pkt.payloadType);
    TEST_ASSERT_EQUAL_UINT8(bob.pubKey[0], pkt.payload[0]);
    TEST_ASSERT_EQUAL_UINT8(alice.pubKey[0], pkt.payload[1]);

    uint8_t bobSecret[32];
    bob.calcSharedSecret(bobSecret, alice.pubKey);

    uint32_t ts;
    uint8_t txtType, attempt;
    char text[DM_MAX_TEXT_LEN + 1];
    TEST_ASSERT_TRUE(DirectMessage::decode(pkt, bobSecret, ts, txtType, attempt,
                                           text, sizeof(text)));
    TEST_ASSERT_EQUAL_HEX32(0xCAFEBABE, ts);
    TEST_ASSERT_EQUAL_UINT8(DM_TXT_TYPE_PLAIN, txtType);
    TEST_ASSERT_EQUAL_UINT8(0, attempt);
    TEST_ASSERT_EQUAL_STRING("ping over a direct link", text);
}

void test_txt_msg_wrong_secret_rejected(void) {
    MeshIdentity alice = makeIdentity(0xA1);
    MeshIdentity bob   = makeIdentity(0xB2);
    MeshIdentity eve   = makeIdentity(0xEE);
    uint8_t secret[32];
    alice.calcSharedSecret(secret, bob.pubKey);

    MeshPacket pkt;
    DirectMessage::encode(pkt, secret, bob.pubKey[0], alice.pubKey[0], 1, 0, "secret");

    // Eve derives a secret with Alice that does not match — MAC must reject.
    uint8_t eveSecret[32];
    eve.calcSharedSecret(eveSecret, alice.pubKey);
    uint32_t ts; uint8_t tt, at; char text[64];
    TEST_ASSERT_FALSE(DirectMessage::decode(pkt, eveSecret, ts, tt, at, text, sizeof(text)));
}

// ── ContactStore ───────────────────────────────────────────────────────────────
void test_contact_store_lookup_and_decode(void) {
    MeshIdentity alice = makeIdentity(0xA1);   // remote sender
    MeshIdentity self  = makeIdentity(0xB2);   // us

    ContactStore store(8);
    TEST_ASSERT_TRUE(store.begin());
    int idx = store.upsert(alice.pubKey, "alice", 42);
    TEST_ASSERT_TRUE(idx >= 0);
    TEST_ASSERT_EQUAL_size_t(1, store.count());

    // Encode a message from Alice to us.
    uint8_t aliceSecret[32];
    alice.calcSharedSecret(aliceSecret, self.pubKey);
    MeshPacket pkt;
    DirectMessage::encode(pkt, aliceSecret, self.pubKey[0], alice.pubKey[0], 7, 0, "hi self");

    // Find the contact by src hash and decode with the cached secret.
    uint8_t srcHash = pkt.payload[1];
    size_t cursor = 0;
    int found = store.nextByHash(srcHash, &cursor);
    TEST_ASSERT_EQUAL_INT(idx, found);

    const uint8_t* secret = store.sharedSecret(found, self);
    TEST_ASSERT_NOT_NULL(secret);
    uint32_t ts; uint8_t tt, at; char text[64];
    TEST_ASSERT_TRUE(DirectMessage::decode(pkt, secret, ts, tt, at, text, sizeof(text)));
    TEST_ASSERT_EQUAL_STRING("hi self", text);

    // Second call returns the same cached secret bytes.
    const uint8_t* secret2 = store.sharedSecret(found, self);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(secret, secret2, 32);
}

void test_contact_store_upsert_dedupes(void) {
    MeshIdentity alice = makeIdentity(0xA1);
    ContactStore store(4);
    store.begin();
    store.upsert(alice.pubKey, "alice", 1);
    store.upsert(alice.pubKey, "alice-renamed", 2);
    TEST_ASSERT_EQUAL_size_t(1, store.count());
    Contact* c = store.at(0);
    TEST_ASSERT_NOT_NULL(c);
    TEST_ASSERT_EQUAL_STRING("alice-renamed", c->name);
    TEST_ASSERT_EQUAL_UINT32(2, c->lastSeen);
}

// ── ACK ────────────────────────────────────────────────────────────────────────
// Receiver's ACK hash (over sender's pubkey) matches the value the sender
// independently computes from the same plaintext + its own pubkey.
void test_ack_matches_sender_expectation(void) {
    MeshIdentity alice = makeIdentity(0xA1);  // sender

    const uint32_t ts = 0x11223344;
    const char* text  = "deliver me";

    uint8_t ack[DM_ACK_SIZE];
    DirectMessage::buildAck(ack, ts, DM_TXT_TYPE_PLAIN, 0, text, alice.pubKey, 0x99);

    // Recompute the sender-side expected 4-byte value the same way: the message
    // is ts(4) || flags(1) || text, concatenated with the sender's pubkey.
    uint8_t msg[5 + 64 + 32];
    msg[0] = ts & 0xFF; msg[1] = (ts >> 8) & 0xFF;
    msg[2] = (ts >> 16) & 0xFF; msg[3] = (ts >> 24) & 0xFF;
    msg[4] = 0;  // attempt 0, TXT_TYPE_PLAIN
    size_t tl = strlen(text);
    memcpy(&msg[5], text, tl);
    memcpy(&msg[5 + tl], alice.pubKey, 32);
    uint8_t expect[4];
    sha256(expect, sizeof(expect), msg, 5 + tl + 32);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, ack, 4);
    TEST_ASSERT_EQUAL_UINT8(0x99, ack[5]);
}

// ── runner ────────────────────────────────────────────────────────────────────
void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_shared_secret_symmetric);
    RUN_TEST(test_txt_msg_roundtrip);
    RUN_TEST(test_txt_msg_wrong_secret_rejected);
    RUN_TEST(test_contact_store_lookup_and_decode);
    RUN_TEST(test_contact_store_upsert_dedupes);
    RUN_TEST(test_ack_matches_sender_expectation);
    UNITY_END();
}

void loop() {}
