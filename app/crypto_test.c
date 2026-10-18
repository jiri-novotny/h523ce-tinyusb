#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include CMSIS_device_header

#include "crypto_test.h"
#include "crypto_test_vectors.h"
#include "monocypher.h"
#include "rng.h"
#include "tusb.h"
#include "udp_crypto.h"
#include "watchdog.h"

/* failed bits */
#define F_RNG       (1U << 0)
#define F_ED_PK     (1U << 1)
#define F_ED_SIGN   (1U << 2)
#define F_ED_VERIFY (1U << 3)
#define F_ED_REJECT (1U << 4)
#define F_X_PUB     (1U << 5)
#define F_SESSION   (1U << 6)
#define F_LOW_ORDER (1U << 7)
#define F_ENCRYPT   (1U << 8)
#define F_DECRYPT   (1U << 9)
#define F_TAMPER    (1U << 10)

static bool test_pending = false;

static void cycles_start(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t cycles(void)
{
  return DWT->CYCCNT;
}

/* inputs used by udp-proxy test vector generator */
static void tv_fill(uint8_t *buf, size_t len, uint8_t base, uint8_t step)
{
  for (size_t i = 0; i < len; i++) buf[i] = (uint8_t) (base + i * step);
}

void crypto_test_run(crypto_test_result_t *res)
{
  uint8_t seed[32], sk[64], pk[32], sig[64], msg[TV_MSG_LEN];
  uint8_t a[32], b[32], pub[32], cliHello[UDP_HELLO_LEN], srvHello[UDP_HELLO_LEN], keys[SESSION_KEYS_LEN];
  uint8_t key[32], pkt[UDP_HDR_LEN + 1024 + UDP_TAG_LEN];
  udpHeader_t *hdr = (udpHeader_t *) pkt;
  uint32_t t;

  memset(res, 0, sizeof(*res));
  cycles_start();

  /* RNG: two reads must differ */
  if (!rng_read(a, sizeof(a)) || !rng_read(b, sizeof(b)) || memcmp(a, b, sizeof(a)) == 0)
  {
    res->failed |= F_RNG;
  }

  /* Ed25519 */
  tv_fill(seed, sizeof(seed), 0, 1);
  tv_fill(msg, sizeof(msg), 0, 7);
  sigKeyPair(seed, sk, pk);
  if (memcmp(pk, tv_ed25519_pk, sizeof(pk)) != 0) res->failed |= F_ED_PK;

  t = cycles();
  sigSign(sk, msg, sizeof(msg), NULL, 0, sig);
  res->cycles[CT_ED_SIGN] = cycles() - t;
  if (memcmp(sig, tv_ed25519_sig, sizeof(sig)) != 0) res->failed |= F_ED_SIGN;

  t = cycles();
  if (!sigVerify(tv_ed25519_pk, msg, sizeof(msg), NULL, 0, tv_ed25519_sig)) res->failed |= F_ED_VERIFY;
  res->cycles[CT_ED_VERIFY] = cycles() - t;
  msg[0] ^= 1;
  if (sigVerify(tv_ed25519_pk, msg, sizeof(msg), NULL, 0, tv_ed25519_sig)) res->failed |= F_ED_REJECT;
  cryptoWipe(sk, sizeof(sk));
  WDI();

  /* X25519 + HKDF-SHA512, device side of the handshake */
  tv_fill(a, sizeof(a), 0x40, 1);
  t = cycles();
  crypto_x25519_public_key(pub, a);
  res->cycles[CT_X25519_PUB] = cycles() - t;
  if (memcmp(pub, tv_x25519_dev_pub, sizeof(pub)) != 0) res->failed |= F_X_PUB;

  memcpy(cliHello, tv_x25519_dev_pub, 32);
  memcpy(srvHello, tv_x25519_srv_pub, 32);
  tv_fill(cliHello + 32, UDP_NONCE_LEN, 0x10, 1);
  tv_fill(srvHello + 32, UDP_NONCE_LEN, 0x20, 1);
  t = cycles();
  bool ok = sessionKeys(a, tv_x25519_srv_pub, TV_SN, cliHello, srvHello, keys);
  res->cycles[CT_SESSION] = cycles() - t;
  if (!ok || memcmp(keys, tv_session_keys, sizeof(keys)) != 0) res->failed |= F_SESSION;

  memset(b, 0, sizeof(b)); /* low order point */
  if (sessionKeys(a, b, TV_SN, cliHello, srvHello, keys)) res->failed |= F_LOW_ORDER;
  cryptoWipe(keys, sizeof(keys));
  WDI();

  /* ChaCha20-Poly1305 packet */
  tv_fill(key, sizeof(key), 0xC0, 1);
  memset(pkt, 0, UDP_HDR_LEN);
  hdr->cmd = UDP_DATA;
  hdr->ver = UDP_VER;
  seqPut(hdr, 0x123456789AULL);
  hdr->sn = TV_SN;
  tv_fill(pkt + UDP_HDR_LEN, TV_PLAIN_LEN, 0, 1);
  t = cycles();
  int n = encryptPacket(key, pkt, TV_PLAIN_LEN);
  res->cycles[CT_AEAD_100] = cycles() - t;
  if (n != (int) sizeof(tv_packet) || memcmp(pkt, tv_packet, sizeof(tv_packet)) != 0) res->failed |= F_ENCRYPT;

  if (decryptPacket(key, pkt, n) != TV_PLAIN_LEN) res->failed |= F_DECRYPT;
  for (int i = 0; i < TV_PLAIN_LEN; i++)
  {
    if (pkt[UDP_HDR_LEN + i] != i) res->failed |= F_DECRYPT;
  }
  memcpy(pkt, tv_packet, sizeof(tv_packet));
  pkt[UDP_HDR_LEN + 5] ^= 1;
  if (decryptPacket(key, pkt, sizeof(tv_packet)) >= 0) res->failed |= F_TAMPER;

  memset(pkt + UDP_HDR_LEN, 0xA5, 1024);
  t = cycles();
  encryptPacket(key, pkt, 1024);
  res->cycles[CT_AEAD_1K] = cycles() - t;
  WDI();
}

void crypto_test_request(void)
{
  test_pending = true;
}

void crypto_test_task(void)
{
  static crypto_test_result_t res;
  static bool reply_pending = false;
  uint8_t pkt[2 + sizeof(res)];

  if (test_pending)
  {
    test_pending = false;
    crypto_test_run(&res);
    reply_pending = true;
  }

  if (reply_pending && tud_hid_ready())
  {
    pkt[0] = HDR_CRYPTO_TEST;
    pkt[1] = sizeof(res);
    memcpy(&pkt[2], &res, sizeof(res));
    if (tud_hid_report(0, pkt, sizeof(pkt)))
    {
      reply_pending = false;
    }
  }
}
