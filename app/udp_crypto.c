#include <string.h>

#include "monocypher-ed25519.h"
#include "monocypher.h"

#include "rng.h"
#include "udp_crypto.h"

#define KDF_LABEL "udp-proxy v5 keys"

/* ChaCha20-Poly1305 (RFC 8439), header (filled, len excluding tag) is 96 bit nonce and AAD,
 * payload encrypted in place, tag appended */
int encryptPacket(const uint8_t *key, uint8_t *pkt, int mlen)
{
  udpHeader_t *hdr = (udpHeader_t *) pkt;
  crypto_aead_ctx ctx;

  if (mlen < 0) return -1;
  hdr->len = mlen + UDP_TAG_LEN;

  /* fresh context per packet, Monocypher rekeys the context after each message */
  crypto_aead_init_ietf(&ctx, key, pkt);
  crypto_aead_write(&ctx, pkt + UDP_HDR_LEN, pkt + UDP_HDR_LEN + mlen, pkt, UDP_HDR_LEN, pkt + UDP_HDR_LEN, mlen);
  crypto_wipe(&ctx, sizeof(ctx));

  return UDP_HDR_LEN + mlen + UDP_TAG_LEN;
}

/* decrypts in place, returns plaintext length or -1, payload is untouched on failure */
int decryptPacket(const uint8_t *key, uint8_t *pkt, int len)
{
  int mlen = len - UDP_HDR_LEN - UDP_TAG_LEN;
  crypto_aead_ctx ctx;
  int ret;

  if (mlen < 0) return -1;

  crypto_aead_init_ietf(&ctx, key, pkt);
  ret = crypto_aead_read(&ctx, pkt + UDP_HDR_LEN, pkt + UDP_HDR_LEN + mlen, pkt, UDP_HDR_LEN, pkt + UDP_HDR_LEN, mlen);
  crypto_wipe(&ctx, sizeof(ctx));

  return ret == 0 ? mlen : -1;
}

void seqPut(udpHeader_t *hdr, uint64_t seq)
{
  for (int i = 0; i < UDP_SEQ_LEN; i++) hdr->seq[i] = (uint8_t) (seq >> (8 * i));
}

uint64_t seqGet(const udpHeader_t *hdr)
{
  uint64_t seq = 0;

  for (int i = 0; i < UDP_SEQ_LEN; i++) seq |= (uint64_t) hdr->seq[i] << (8 * i);
  return seq;
}

/* true when seq was not seen yet and is not older than the window */
bool replayCheck(const replay_t *r, uint64_t seq)
{
  if (seq >= r->top) return true;
  if (r->top - 1 - seq >= 64) return false;
  return !(r->window & (1ULL << (r->top - 1 - seq)));
}

/* call only for authenticated packets, returns true when seq is the newest */
bool replayUpdate(replay_t *r, uint64_t seq)
{
  if (seq >= r->top)
  {
    uint64_t shift = seq + 1 - r->top;
    r->window = (shift >= 64) ? 0 : (r->window << shift);
    r->window |= 1;
    r->top = seq + 1;
    return true;
  }
  r->window |= 1ULL << (r->top - 1 - seq);
  return false;
}

void sigKeyPair(uint8_t seed[UDP_SIG_SEED_LEN], uint8_t sk[UDP_SIG_SK_LEN], uint8_t pk[UDP_SIG_PK_LEN])
{
  crypto_ed25519_key_pair(sk, pk, seed);
}

/* Ed25519 is one-shot only, m1 | m2 joined in a buffer large enough for the handshake */
static size_t sigJoin(uint8_t *bfr, size_t size, const uint8_t *m1, size_t l1, const uint8_t *m2, size_t l2)
{
  if (l1 + l2 > size) return 0;
  memcpy(bfr, m1, l1);
  if (l2) memcpy(bfr + l1, m2, l2);
  return l1 + l2;
}

/* Ed25519 over m1 | m2, sig is UDP_SIG_LEN */
bool sigSign(const uint8_t sk[UDP_SIG_SK_LEN], const uint8_t *m1, size_t l1, const uint8_t *m2, size_t l2, uint8_t *sig)
{
  uint8_t msg[2 * UDP_HDR_LEN + UDP_OPEN_SIGNED_LEN + UDP_HELLO_LEN];
  size_t mlen = sigJoin(msg, sizeof(msg), m1, l1, m2, l2);

  if (mlen == 0) return false;
  crypto_ed25519_sign(sig, sk, msg, mlen);
  return true;
}

bool sigVerify(const uint8_t pk[UDP_SIG_PK_LEN], const uint8_t *m1, size_t l1, const uint8_t *m2, size_t l2, const uint8_t *sig)
{
  uint8_t msg[2 * UDP_HDR_LEN + UDP_OPEN_SIGNED_LEN + UDP_HELLO_LEN];
  size_t mlen = sigJoin(msg, sizeof(msg), m1, l1, m2, l2);

  return mlen != 0 && crypto_ed25519_check(sig, pk, msg, mlen) == 0;
}

bool ecdhKeygen(uint8_t eph[32], uint8_t *pub)
{
  if (!rng_read(eph, 32)) return false;
  crypto_x25519_public_key(pub, eph);
  return true;
}

/* X25519 + HKDF-SHA512 (no salt), hellos are [eph][nonce] as sent */
bool sessionKeys(const uint8_t eph[32], const uint8_t *peerPub, uint32_t sn, const uint8_t *cliHello, const uint8_t *srvHello, uint8_t *keys)
{
  static const uint8_t zero[32];
  uint8_t ss[32];
  uint8_t info[sizeof(KDF_LABEL) - 1 + 4 + 2 * UDP_HELLO_LEN];
  bool ret = false;

  crypto_x25519(ss, eph, peerPub);

  /* reject all-zero shared secret (low order peer point), same as OpenSSL on the server */
  if (crypto_verify32(ss, zero) != 0)
  {
    memcpy(info, KDF_LABEL, sizeof(KDF_LABEL) - 1);
    memcpy(info + sizeof(KDF_LABEL) - 1, &sn, 4);
    memcpy(info + sizeof(KDF_LABEL) - 1 + 4, cliHello, UDP_HELLO_LEN);
    memcpy(info + sizeof(KDF_LABEL) - 1 + 4 + UDP_HELLO_LEN, srvHello, UDP_HELLO_LEN);

    crypto_sha512_hkdf(keys, SESSION_KEYS_LEN, ss, sizeof(ss), NULL, 0, info, sizeof(info));
    ret = true;
  }
  crypto_wipe(ss, sizeof(ss));

  return ret;
}

void cryptoWipe(void *secret, size_t size)
{
  crypto_wipe(secret, size);
}
