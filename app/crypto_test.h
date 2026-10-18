#ifndef __CRYPTO_TEST_H__
#define __CRYPTO_TEST_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define HDR_CRYPTO_TEST 12

/* individual timings in CPU cycles */
enum
{
  CT_X25519_PUB,  /* ephemeral public key */
  CT_SESSION,     /* X25519 shared secret + HKDF-SHA512 */
  CT_ED_SIGN,     /* Ed25519 sign OPEN (92 B) */
  CT_ED_VERIFY,   /* Ed25519 verify OPEN (92 B) */
  CT_AEAD_100,    /* encrypt 100 B packet */
  CT_AEAD_1K,     /* encrypt 1 KiB */
  CT_COUNT
};

typedef struct
{
  uint32_t failed; /* bit mask of failed checks, 0 = all passed */
  uint32_t cycles[CT_COUNT];
} crypto_test_result_t;

/* known answer tests against udp-proxy (OpenSSL) vectors + timing, blocking ~1 s */
void crypto_test_run(crypto_test_result_t *res);

/* HID: request from USB callback, run and reply from main loop */
void crypto_test_request(void);
void crypto_test_task(void);

#ifdef __cplusplus
}
#endif
#endif /* __CRYPTO_TEST_H__ */
