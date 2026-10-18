#ifndef UDP_PROTO_H
#define UDP_PROTO_H

/* udp-proxy protocol definitions, keep in sync with udp-proxy/const.h */

#include <stdint.h>

#define UDP_VER 5
#define UDP_SEQ_LEN 6
#define UDP_HDR_LEN 12
#define UDP_TAG_LEN 16
#define UDP_KEY_LEN 32   /* ChaCha20-Poly1305, one key per direction */
#define UDP_EPH_LEN 32   /* X25519 public key */
#define UDP_NONCE_LEN 32
#define UDP_SIG_LEN 64   /* Ed25519 */
#define UDP_COOKIE_LEN 16
#define UDP_HELLO_LEN (UDP_EPH_LEN + UDP_NONCE_LEN)
#define UDP_OPEN_SIGNED_LEN (UDP_HELLO_LEN + UDP_COOKIE_LEN)
#define UDP_OPEN_LEN (UDP_OPEN_SIGNED_LEN + UDP_SIG_LEN)
#define UDP_AUTH_LEN (UDP_HELLO_LEN + UDP_SIG_LEN)
#define UDP_COOKIE_MSG_LEN (2 * UDP_COOKIE_LEN) /* [cookie][nC prefix] */
#define UDP_SEQ_MAX (1ULL << (8 * UDP_SEQ_LEN))

/*
 * Handshake (plaintext, signed with Ed25519):
 *   OPEN    dev->srv  [hdr][eC][nC][0..0][0..0]         unsigned cookie request, full size against amplification
 *   COOKIE  srv->dev  [hdr][cookie][nC:16]              stateless HMAC(secret, epoch | ip | port | sn)
 *   OPEN    dev->srv  [hdr][eC][nC][cookie][sig dev over hdr..cookie]
 *   AUTH    srv->dev  [hdr][eS][nS][sig srv over OPEN hdr..cookie | hdr..nS]
 * Keys = HKDF-SHA512(X25519(eC, eS), info = label | sn | eC nC | eS nS)
 * Encrypted with ChaCha20-Poly1305 (header is nonce and AAD, seq per direction):
 *   AUTH    dev->srv  []   key confirmation
 *   CONNECT srv->dev  []
 */
typedef enum
{
  UDP_OPEN,
  UDP_REJECT,
  UDP_AUTH,
  UDP_CONNECT,
  UDP_KEEPALIVE,
  UDP_CLIENT,
  UDP_DATA,
  UDP_DATA_ACK, /* [ack:2][ctr:2][data] - ACK piggybacked on data */
  UDP_CLOSE,
  UDP_COOKIE,
} udpCmd_t;

typedef struct
{
  uint16_t cmd : 4;
  uint16_t ver : 3;
  uint16_t len : 9;
  uint8_t seq[UDP_SEQ_LEN]; /* per direction packet counter, random in handshake */
  uint32_t sn;
} udpHeader_t;

_Static_assert(sizeof(udpHeader_t) == UDP_HDR_LEN, "udpHeader_t layout");

#endif /* UDP_PROTO_H */
