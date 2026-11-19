// tiny_hash.h - SHA-1 and base64 for the WebSocket accept handshake only.
//
// Why not mbedtls: the ESP32 Arduino core 4.x moved sha1.h into
// tf-psa-crypto's private headers (mbedtls 4 reorganisation), so
// #include "mbedtls/sha1.h" no longer resolves; the classic path only works
// on cores 2/3. The WebSocket handshake needs exactly one SHA-1 of a short
// ASCII string and one base64 of the 20-byte result - about 110 lines - so
// this sketch carries its own and never breaks with a core upgrade again.
// Not used for anything security-sensitive: the WS handshake key is a
// well-known obfuscation (RFC 6455 §1.3), not a credential.
//
// Verified on the host against FIPS 180-4 / RFC 6455 vectors:
//   SHA1("")    = da39a3ee5e6b4b0d3255bfef95601890afd80709
//   SHA1("abc") = a9993e364706816aba3e25717850c26c9cd0d89d
//   WS: key "dGhlIHNhbXBsZSBub25jZQ==" ->
//       accept "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// One 512-bit block -> mix into the chaining value (FIPS 180-4 §6.1).
static inline void tinySha1Block(uint32_t h[5], const uint8_t block[64]) {
  uint32_t w[80];
  for (int i = 0; i < 16; i++)
    w[i] = ((uint32_t)block[i * 4] << 24) | ((uint32_t)block[i * 4 + 1] << 16) |
           ((uint32_t)block[i * 4 + 2] << 8) | (uint32_t)block[i * 4 + 3];
  for (int i = 16; i < 80; i++) {
    const uint32_t x = w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16];
    w[i] = (x << 1) | (x >> 31);
  }

  uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
  for (int i = 0; i < 80; i++) {
    uint32_t f, k;
    if (i < 20)      { f = (b & c) | ((~b) & d); k = 0x5A827999u; }
    else if (i < 40) { f = b ^ c ^ d;            k = 0x6ED9EBA1u; }
    else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCu; }
    else             { f = b ^ c ^ d;            k = 0xCA62C1D6u; }
    const uint32_t t = ((a << 5) | (a >> 27)) + f + e + k + w[i];
    e = d;
    d = c;
    c = (b << 30) | (b >> 2);
    b = a;
    a = t;
  }
  h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
}

// SHA-1 of data[0..len) into out[20].
static inline void tinySha1(const uint8_t * data, size_t len, uint8_t out[20]) {
  uint32_t h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u,
                   0xC3D2E1F0u};
  const uint64_t total_bits = (uint64_t)len * 8u;

  // Full 64-byte blocks straight from the input.
  size_t off = 0;
  while (len - off >= 64) {
    tinySha1Block(h, data + off);
    off += 64;
  }

  // Tail: remainder + 0x80 + zeros + 64-bit big-endian bit length. The
  // length may spill into a second tail block when rem >= 56.
  uint8_t tail[128];
  memset(tail, 0, sizeof(tail));
  const size_t rem = len - off;
  if (rem) memcpy(tail, data + off, rem);
  tail[rem] = 0x80;
  const size_t tail_blocks = (rem < 56) ? 64u : 128u;
  for (int i = 0; i < 8; i++)
    tail[tail_blocks - 8 + i] = (uint8_t)(total_bits >> (56 - 8 * i));
  tinySha1Block(h, tail);
  if (tail_blocks == 128) tinySha1Block(h, tail + 64);

  for (int i = 0; i < 5; i++) {
    out[i * 4]     = (uint8_t)(h[i] >> 24);
    out[i * 4 + 1] = (uint8_t)(h[i] >> 16);
    out[i * 4 + 2] = (uint8_t)(h[i] >> 8);
    out[i * 4 + 3] = (uint8_t)h[i];
  }
}

// ----------------------------------------------------------------- base64
// RFC 4648. out must hold ceil(len/3)*4 + 1 bytes (NUL included).
// Returns the number of characters written, excluding the NUL; 0 if the
// output does not fit.
static inline size_t tinyBase64(const uint8_t * data, size_t len,
                                char * out, size_t out_cap) {
  static const char tbl[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t o = 0;
  size_t i = 0;
  while (i + 3 <= len) {
    if (o + 5 > out_cap) return 0;
    const uint32_t v = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8) |
                       data[i + 2];
    out[o++] = tbl[(v >> 18) & 63];
    out[o++] = tbl[(v >> 12) & 63];
    out[o++] = tbl[(v >> 6) & 63];
    out[o++] = tbl[v & 63];
    i += 3;
  }
  const size_t rem = len - i;
  if (rem == 1) {
    if (o + 5 > out_cap) return 0;
    const uint32_t v = (uint32_t)data[i] << 16;
    out[o++] = tbl[(v >> 18) & 63];
    out[o++] = tbl[(v >> 12) & 63];
    out[o++] = '=';
    out[o++] = '=';
  } else if (rem == 2) {
    if (o + 5 > out_cap) return 0;
    const uint32_t v = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8);
    out[o++] = tbl[(v >> 18) & 63];
    out[o++] = tbl[(v >> 12) & 63];
    out[o++] = tbl[(v >> 6) & 63];
    out[o++] = '=';
  }
  out[o] = '\0';
  return o;
}
