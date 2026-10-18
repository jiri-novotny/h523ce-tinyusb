#ifndef UDP_CRYPTO_H
#define UDP_CRYPTO_H

/* udp-proxy v5 device crypto on Monocypher, wire compatible with udp-proxy/crypto.c (OpenSSL) */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "udp_proto.h"

/* session key material: [c2s key][s2c key][srv ctrOut:2][srv ctrIn:2] */
#define SESSION_KEYS_LEN (2 * UDP_KEY_LEN + 4)

#define UDP_SIG_SEED_LEN 32 /* Ed25519 private key (RFC 8032 seed, raw part of PKCS#8 PEM) */
#define UDP_SIG_SK_LEN 64   /* Monocypher expanded secret key: [seed][public key] */
#define UDP_SIG_PK_LEN 32

typedef struct
{
  uint64_t top;    /* highest accepted seq + 1, 0 = nothing received */
  uint64_t window; /* bit n set = seq (top - 1 - n) received */
} replay_t;

int encryptPacket(const uint8_t *key, uint8_t *pkt, int mlen);
int decryptPacket(const uint8_t *key, uint8_t *pkt, int len);

void seqPut(udpHeader_t *hdr, uint64_t seq);
uint64_t seqGet(const udpHeader_t *hdr);
bool replayCheck(const replay_t *r, uint64_t seq);
bool replayUpdate(replay_t *r, uint64_t seq);

/* seed is wiped, sk keeps the key pair for signing */
void sigKeyPair(uint8_t seed[UDP_SIG_SEED_LEN], uint8_t sk[UDP_SIG_SK_LEN], uint8_t pk[UDP_SIG_PK_LEN]);
bool sigSign(const uint8_t sk[UDP_SIG_SK_LEN], const uint8_t *m1, size_t l1, const uint8_t *m2, size_t l2, uint8_t *sig);
bool sigVerify(const uint8_t pk[UDP_SIG_PK_LEN], const uint8_t *m1, size_t l1, const uint8_t *m2, size_t l2, const uint8_t *sig);

/* ephemeral X25519 key, raw public part written to pub; eph must be wiped by caller after sessionKeys() */
bool ecdhKeygen(uint8_t eph[32], uint8_t *pub);
bool sessionKeys(const uint8_t eph[32], const uint8_t *peerPub, uint32_t sn, const uint8_t *cliHello, const uint8_t *srvHello, uint8_t *keys);

void cryptoWipe(void *secret, size_t size);

#endif /* UDP_CRYPTO_H */
