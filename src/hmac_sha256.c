/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "hmac_sha256.h"

#include <string.h>

struct sha256 {
	uint32_t h[8];
	uint8_t  buf[64];
	uint64_t total;      /* bytes consumed */
	size_t   fill;
};

static const uint32_t K[64] = {
	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
	0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
	0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
	0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
	0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
	0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

#define ROR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

static void block(struct sha256 *s, const uint8_t *p)
{
	uint32_t w[64], a, b, c, d, e, f, g, h;

	for (int i = 0; i < 16; i++) {
		w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) |
		       ((uint32_t)p[4 * i + 2] << 8) | (uint32_t)p[4 * i + 3];
	}
	for (int i = 16; i < 64; i++) {
		uint32_t s0 = ROR(w[i - 15], 7) ^ ROR(w[i - 15], 18) ^ (w[i - 15] >> 3);
		uint32_t s1 = ROR(w[i - 2], 17) ^ ROR(w[i - 2], 19) ^ (w[i - 2] >> 10);

		w[i] = w[i - 16] + s0 + w[i - 7] + s1;
	}
	a = s->h[0]; b = s->h[1]; c = s->h[2]; d = s->h[3];
	e = s->h[4]; f = s->h[5]; g = s->h[6]; h = s->h[7];
	for (int i = 0; i < 64; i++) {
		uint32_t S1 = ROR(e, 6) ^ ROR(e, 11) ^ ROR(e, 25);
		uint32_t ch = (e & f) ^ (~e & g);
		uint32_t t1 = h + S1 + ch + K[i] + w[i];
		uint32_t S0 = ROR(a, 2) ^ ROR(a, 13) ^ ROR(a, 22);
		uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
		uint32_t t2 = S0 + mj;

		h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
	}
	s->h[0] += a; s->h[1] += b; s->h[2] += c; s->h[3] += d;
	s->h[4] += e; s->h[5] += f; s->h[6] += g; s->h[7] += h;
}

static void sha_init(struct sha256 *s)
{
	static const uint32_t iv[8] = {
		0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
		0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
	};

	memcpy(s->h, iv, sizeof(iv));
	s->total = 0;
	s->fill = 0;
}

static void sha_update(struct sha256 *s, const uint8_t *p, size_t n)
{
	s->total += n;
	while (n > 0) {
		size_t take = 64 - s->fill;

		if (take > n) {
			take = n;
		}
		memcpy(s->buf + s->fill, p, take);
		s->fill += take;
		p += take;
		n -= take;
		if (s->fill == 64) {
			block(s, s->buf);
			s->fill = 0;
		}
	}
}

static void sha_final(struct sha256 *s, uint8_t out[32])
{
	uint64_t bits = s->total * 8u;
	uint8_t pad = 0x80;

	sha_update(s, &pad, 1);
	pad = 0;
	while (s->fill != 56) {
		sha_update(s, &pad, 1);
	}
	uint8_t len[8];

	for (int i = 0; i < 8; i++) {
		len[i] = (uint8_t)(bits >> (56 - 8 * i));
	}
	sha_update(s, len, 8);
	for (int i = 0; i < 8; i++) {
		out[4 * i]     = (uint8_t)(s->h[i] >> 24);
		out[4 * i + 1] = (uint8_t)(s->h[i] >> 16);
		out[4 * i + 2] = (uint8_t)(s->h[i] >> 8);
		out[4 * i + 3] = (uint8_t)s->h[i];
	}
}

void ancla_sha256(const uint8_t *msg, size_t len, uint8_t out[32])
{
	struct sha256 s;

	sha_init(&s);
	sha_update(&s, msg, len);
	sha_final(&s, out);
}

void ancla_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg, size_t msg_len,
		 uint8_t out[HMAC_SHA256_LEN])
{
	uint8_t k[64] = {0}, ipad[64], opad[64], inner[32];
	struct sha256 s;

	if (key_len > 64) {                       /* RFC 2104: a long key is hashed first */
		sha_init(&s);
		sha_update(&s, key, key_len);
		sha_final(&s, k);
	} else if (key_len > 0) {
		memcpy(k, key, key_len);
	}
	for (int i = 0; i < 64; i++) {
		ipad[i] = k[i] ^ 0x36;
		opad[i] = k[i] ^ 0x5c;
	}
	sha_init(&s);
	sha_update(&s, ipad, 64);
	sha_update(&s, msg, msg_len);
	sha_final(&s, inner);

	sha_init(&s);
	sha_update(&s, opad, 64);
	sha_update(&s, inner, 32);
	sha_final(&s, out);
}

bool ancla_hmac_sha256_equal(const uint8_t a[HMAC_SHA256_LEN], const uint8_t b[HMAC_SHA256_LEN])
{
	uint8_t diff = 0;

	for (size_t i = 0; i < HMAC_SHA256_LEN; i++) {
		diff |= (uint8_t)(a[i] ^ b[i]);
	}
	return diff == 0;
}
