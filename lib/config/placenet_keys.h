#pragma once

// PlaceNet device identity key pair.
//
// An EC P-256 key pair is generated once at runtime using mbedTLS and held in
// these globals.  The public key is exported as a PEM string so it can be
// included in the registration payload sent to the home server.
//
// NOTE: These are in-RAM keys generated fresh every boot.  Persistent key
// storage (e.g. NVS / SD card) is a future concern; for now regenerate on
// every power-cycle.

#include <mbedtls/pk.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/error.h>
#include <string.h>

// Maximum size of a PEM-encoded EC P-256 public key (including null terminator).
#define PLACENET_PUBLIC_KEY_PEM_SIZE  256

// Holds the generated key pair for the lifetime of the session.
struct PlaceNetKeyPair {
    mbedtls_pk_context      pk;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    char publicKeyPem[PLACENET_PUBLIC_KEY_PEM_SIZE];
    bool valid;
};

// Generate a fresh EC P-256 key pair.
// Returns true on success; publicKeyPem is populated with the PEM string.
inline bool placenet_keygen(PlaceNetKeyPair* kp) {
    if (!kp) return false;

    mbedtls_pk_init(&kp->pk);
    mbedtls_entropy_init(&kp->entropy);
    mbedtls_ctr_drbg_init(&kp->ctr_drbg);
    memset(kp->publicKeyPem, 0, sizeof(kp->publicKeyPem));
    kp->valid = false;

    const char* pers = "placenet_keygen";
    int ret;

    ret = mbedtls_ctr_drbg_seed(&kp->ctr_drbg, mbedtls_entropy_func,
                                  &kp->entropy,
                                  (const unsigned char*)pers, strlen(pers));
    if (ret != 0) return false;

    ret = mbedtls_pk_setup(&kp->pk, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
    if (ret != 0) return false;

    ret = mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1,
                               mbedtls_pk_ec(kp->pk),
                               mbedtls_ctr_drbg_random, &kp->ctr_drbg);
    if (ret != 0) return false;

    ret = mbedtls_pk_write_pubkey_pem(&kp->pk,
                                       (unsigned char*)kp->publicKeyPem,
                                       sizeof(kp->publicKeyPem));
    if (ret != 0) return false;

    kp->valid = true;
    return true;
}

// Free resources held by a PlaceNetKeyPair.
inline void placenet_keyfree(PlaceNetKeyPair* kp) {
    if (!kp) return;
    mbedtls_pk_free(&kp->pk);
    mbedtls_entropy_free(&kp->entropy);
    mbedtls_ctr_drbg_free(&kp->ctr_drbg);
    kp->valid = false;
}
