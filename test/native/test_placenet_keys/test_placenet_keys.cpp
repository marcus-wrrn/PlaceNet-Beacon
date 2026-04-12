#include <unity.h>
#include "placenet_keys.h"
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

// ── placenet_keygen ───────────────────────────────────────────────────────────

void test_keygen_returns_true(void) {
    PlaceNetKeyPair kp;
    bool ok = placenet_keygen(&kp);
    TEST_ASSERT_TRUE(ok);
    placenet_keyfree(&kp);
}

void test_keygen_sets_valid(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    TEST_ASSERT_TRUE(kp.valid);
    placenet_keyfree(&kp);
}

void test_keygen_public_key_pem_not_empty(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    TEST_ASSERT_GREATER_THAN(0, (int)strlen(kp.publicKeyPem));
    placenet_keyfree(&kp);
}

void test_keygen_public_key_pem_header(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    TEST_ASSERT_NOT_NULL(strstr(kp.publicKeyPem, "-----BEGIN PUBLIC KEY-----"));
    placenet_keyfree(&kp);
}

void test_keygen_public_key_pem_footer(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    TEST_ASSERT_NOT_NULL(strstr(kp.publicKeyPem, "-----END PUBLIC KEY-----"));
    placenet_keyfree(&kp);
}

void test_keygen_null_returns_false(void) {
    TEST_ASSERT_FALSE(placenet_keygen(nullptr));
}

void test_keygen_produces_unique_keys(void) {
    PlaceNetKeyPair kp1, kp2;
    placenet_keygen(&kp1);
    placenet_keygen(&kp2);
    // Two independent key pairs should have different public keys.
    bool same = (strcmp(kp1.publicKeyPem, kp2.publicKeyPem) == 0);
    TEST_ASSERT_FALSE(same);
    placenet_keyfree(&kp1);
    placenet_keyfree(&kp2);
}

// ── placenet_keyfree ──────────────────────────────────────────────────────────

void test_keyfree_clears_valid(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    placenet_keyfree(&kp);
    TEST_ASSERT_FALSE(kp.valid);
}

void test_keyfree_null_is_safe(void) {
    // Must not crash.
    placenet_keyfree(nullptr);
    TEST_PASS();
}

// ── placenet_csr_generate ─────────────────────────────────────────────────────

void test_csr_generate_returns_true(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    char buf[PLACENET_CSR_PEM_SIZE];
    bool ok = placenet_csr_generate(&kp, "CN=placenet-beacon", buf, sizeof(buf));
    TEST_ASSERT_TRUE(ok);
    placenet_keyfree(&kp);
}

void test_csr_pem_not_empty(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    char buf[PLACENET_CSR_PEM_SIZE];
    placenet_csr_generate(&kp, "CN=placenet-beacon", buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN(0, (int)strlen(buf));
    placenet_keyfree(&kp);
}

void test_csr_pem_header(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    char buf[PLACENET_CSR_PEM_SIZE];
    placenet_csr_generate(&kp, "CN=placenet-beacon", buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "-----BEGIN CERTIFICATE REQUEST-----"));
    placenet_keyfree(&kp);
}

void test_csr_pem_footer(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    char buf[PLACENET_CSR_PEM_SIZE];
    placenet_csr_generate(&kp, "CN=placenet-beacon", buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "-----END CERTIFICATE REQUEST-----"));
    placenet_keyfree(&kp);
}

void test_csr_subject_roundtrip(void) {
    // Two CSRs from different key pairs should not be byte-for-byte identical
    // (different keys → different signatures).
    PlaceNetKeyPair kp1, kp2;
    placenet_keygen(&kp1);
    placenet_keygen(&kp2);
    char buf1[PLACENET_CSR_PEM_SIZE];
    char buf2[PLACENET_CSR_PEM_SIZE];
    placenet_csr_generate(&kp1, "CN=placenet-beacon", buf1, sizeof(buf1));
    placenet_csr_generate(&kp2, "CN=placenet-beacon", buf2, sizeof(buf2));
    TEST_ASSERT_FALSE(strcmp(buf1, buf2) == 0);
    placenet_keyfree(&kp1);
    placenet_keyfree(&kp2);
}

void test_csr_null_keypair_fails(void) {
    char buf[PLACENET_CSR_PEM_SIZE];
    TEST_ASSERT_FALSE(placenet_csr_generate(nullptr, "CN=test", buf, sizeof(buf)));
}

void test_csr_invalid_keypair_fails(void) {
    PlaceNetKeyPair kp;
    // Manually construct a keypair with valid=false (never called keygen).
    mbedtls_pk_init(&kp.pk);
    mbedtls_entropy_init(&kp.entropy);
    mbedtls_ctr_drbg_init(&kp.ctr_drbg);
    kp.valid = false;
    char buf[PLACENET_CSR_PEM_SIZE];
    TEST_ASSERT_FALSE(placenet_csr_generate(&kp, "CN=test", buf, sizeof(buf)));
    mbedtls_pk_free(&kp.pk);
    mbedtls_entropy_free(&kp.entropy);
    mbedtls_ctr_drbg_free(&kp.ctr_drbg);
}

void test_csr_null_buffer_fails(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    TEST_ASSERT_FALSE(placenet_csr_generate(&kp, "CN=test", nullptr, 128));
    placenet_keyfree(&kp);
}

void test_csr_zero_size_fails(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    char buf[PLACENET_CSR_PEM_SIZE];
    TEST_ASSERT_FALSE(placenet_csr_generate(&kp, "CN=test", buf, 0));
    placenet_keyfree(&kp);
}

void test_csr_buffer_too_small_fails(void) {
    PlaceNetKeyPair kp;
    placenet_keygen(&kp);
    char buf[16];  // Far too small for a CSR PEM.
    TEST_ASSERT_FALSE(placenet_csr_generate(&kp, "CN=placenet-beacon", buf, sizeof(buf)));
    placenet_keyfree(&kp);
}

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    UNITY_BEGIN();

    // keygen
    RUN_TEST(test_keygen_returns_true);
    RUN_TEST(test_keygen_sets_valid);
    RUN_TEST(test_keygen_public_key_pem_not_empty);
    RUN_TEST(test_keygen_public_key_pem_header);
    RUN_TEST(test_keygen_public_key_pem_footer);
    RUN_TEST(test_keygen_null_returns_false);
    RUN_TEST(test_keygen_produces_unique_keys);

    // keyfree
    RUN_TEST(test_keyfree_clears_valid);
    RUN_TEST(test_keyfree_null_is_safe);

    // csr_generate
    RUN_TEST(test_csr_generate_returns_true);
    RUN_TEST(test_csr_pem_not_empty);
    RUN_TEST(test_csr_pem_header);
    RUN_TEST(test_csr_pem_footer);
    RUN_TEST(test_csr_subject_roundtrip);
    RUN_TEST(test_csr_null_keypair_fails);
    RUN_TEST(test_csr_invalid_keypair_fails);
    RUN_TEST(test_csr_null_buffer_fails);
    RUN_TEST(test_csr_zero_size_fails);
    RUN_TEST(test_csr_buffer_too_small_fails);

    return UNITY_END();
}
