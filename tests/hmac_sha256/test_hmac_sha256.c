/*
 * Host tests for hmac_sha256: RFC 4231 vectors plus block-boundary lengths checked against
 * Python's hmac module (the same library the signing tool uses).
 *
 *   gcc -Wall -Wextra -D__USE_MINGW_ANSI_STDIO=1 -Isrc -o tests/hmac_sha256/t.exe \
 *       tests/hmac_sha256/test_hmac_sha256.c src/hmac_sha256.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hmac_sha256.h"

static int failures;

#define CHECK(cond)                                                            \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                            \
		}                                                              \
	} while (0)

static void hex(const uint8_t *p, size_t n, char *out)
{
	for (size_t i = 0; i < n; i++) {
		sprintf(out + 2 * i, "%02x", p[i]);
	}
}

static void expect(const uint8_t *key, size_t kl, const uint8_t *msg, size_t ml, const char *want)
{
	uint8_t mac[32];
	char got[65];

	ancla_hmac_sha256(key, kl, msg, ml, mac);
	hex(mac, 32, got);
	if (strcmp(got, want) != 0) {
		printf("  got  %s\n  want %s\n", got, want);
	}
	CHECK(strcmp(got, want) == 0);
}

static void test_rfc4231(void)
{
	uint8_t k20[20], k131[131], d50[50];

	memset(k20, 0x0b, sizeof(k20));
	expect(k20, 20, (const uint8_t *)"Hi There", 8,
	       "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7");
	expect((const uint8_t *)"Jefe", 4, (const uint8_t *)"what do ya want for nothing?", 28,
	       "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843");
	memset(k20, 0xaa, sizeof(k20));
	memset(d50, 0xdd, sizeof(d50));
	expect(k20, 20, d50, 50, "773ea91e36800e46854db8ebd09181a72959098b3ef8c122d9635514ced565fe");
	memset(k131, 0xaa, sizeof(k131));
	expect(k131, 131,
	       (const uint8_t *)"Test Using Larger Than Block-Size Key - Hash Key First", 54,
	       "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54");
}

static const struct { size_t len; const char *mac; } boundary[] = {
	{ 0, "feffb2318c2191c4f11e1fc0ee3d808697d6073ae8a1c30be5f1a169ceff13e6" },
	{ 1, "1887a7e7c9c15938b942006b573e26f129166114a170fddc6f9ca7230d68406d" },
	{ 55, "76f9c1ccec74b5f25419c8154ff80648db3d78e9ff4d719e553268bb5c02fb61" },
	{ 56, "fdac98fd6d99e2ca2a96c4d0905ab3542da35f15e8d7ad9332e2038830fad12d" },
	{ 57, "c2e36f3eef182cabf19545d0d12f364b49f40f2ab42acc2ce6670a7182dc4d82" },
	{ 63, "2c33cf78f9387b30861514222e07809f63d83501a638140fa49d7036d0d04951" },
	{ 64, "ca9e37de8ad88576af0c14df4ba027641d73dc33f4d6c405666607054844db9c" },
	{ 65, "d37dafe0229386f785fdb6fc1d00fc8f515a0bc89e3032a95b8d1995e3ee8fc4" },
	{ 119, "3bcebdecfcb1ed1090f1d34286ad0145a5e1281c73d3c0ccad51c0085174d136" },
	{ 120, "0f577fbb2e85c86bb2b6be4f6eaa34d0d959cdcf8eabd5327f781a2e4c587cf6" },
	{ 121, "4551a3b2e626f1b28918eee1480137f3ced01440963ebf6957d9805405d8c02c" },
	{ 200, "f55e1c5e4fc207c94ec7e77e620d2989be6defab3366b48e70c52a7090174bc4" },
};

static void test_block_boundaries(void)
{
	uint8_t key[32], msg[256];

	for (int i = 0; i < 32; i++) key[i] = (uint8_t)((7 * i + 3) & 0xff);
	for (int i = 0; i < 256; i++) msg[i] = (uint8_t)((13 * i + 1) & 0xff);
	for (size_t i = 0; i < sizeof(boundary) / sizeof(boundary[0]); i++) {
		expect(key, 32, msg, boundary[i].len, boundary[i].mac);
	}
}

static void test_sha256_vectors_and_fingerprint(void)
{
	uint8_t d[32];
	char got[65];

	ancla_sha256((const uint8_t *)"", 0, d);
	hex(d, 32, got);
	CHECK(strcmp(got, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855") == 0);
	ancla_sha256((const uint8_t *)"abc", 3, d);
	hex(d, 32, got);
	CHECK(strcmp(got, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad") == 0);

	/* the key fingerprint both sides print: first 4 bytes of SHA-256(key), for the key 00..1f */
	uint8_t key[32];

	for (int i = 0; i < 32; i++) key[i] = (uint8_t)i;
	ancla_sha256(key, 32, d);
	hex(d, 4, got);
	CHECK(strcmp(got, "630dcd29") == 0);
}

static void test_equal_is_exact(void)
{
	uint8_t a[32], b[32];

	memset(a, 0x5a, 32);
	memcpy(b, a, 32);
	CHECK(ancla_hmac_sha256_equal(a, b));
	for (int i = 0; i < 32; i++) {                 /* one flipped bit anywhere must fail */
		b[i] ^= 0x01;
		CHECK(!ancla_hmac_sha256_equal(a, b));
		b[i] ^= 0x01;
	}
}

int main(void)
{
	test_rfc4231();
	test_block_boundaries();
	test_sha256_vectors_and_fingerprint();
	test_equal_is_exact();
	if (failures) {
		printf("\n%d CHECK(s) FAILED\n", failures);
		return EXIT_FAILURE;
	}
	printf("hmac_sha256: ALL TESTS PASSED\n");
	return EXIT_SUCCESS;
}
