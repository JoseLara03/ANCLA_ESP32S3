/*
 * Host tests for cmd_proto. The signature vectors below were produced by Python's hmac module --
 * the same code path edge/tools/nodecmd.py signs with -- so a mismatch in the canonical string
 * between the two languages fails here.
 *
 *   gcc -Wall -Wextra -D__USE_MINGW_ANSI_STDIO=1 -Isrc -o tests/cmd_proto/t.exe \
 *       tests/cmd_proto/test_cmd_proto.c src/cmd_proto.c src/hmac_sha256.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmd_proto.h"
#include "hmac_sha256.h"

static int failures;

#define CHECK(cond)                                                            \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                            \
		}                                                              \
	} while (0)

static uint8_t KEY[32];
#define ZONE "852541"
#define CH "a1b2c3d4e5f60718"

static int parse(const char *s, struct cmd_msg *m)
{
	return cmd_parse(s, strlen(s), m);
}

static const char *SIG_TAU = "3dbf20d755345f70100590e7e59556893055348e6f01c93004102403100d34a9";
static const char *SIG_ANT = "a1462f949d1b0d9d4dda1231503367d98a036f7bd6481ef3d2a91e04fb33933f";
static const char *SIG_REB = "a385df6ea492394f078ba6e9eebec7185976f2409536391b6e5bb5785e24ab09";
static const char *SIG_NEG = "2be60c67357e747a90f83b675a851f5d59db70f48bcc73321d54c675de229ffc";
/* Since 2026-10-21 an anchor's node name is its EUI's 16 hex digits, not its id. */
#define EUI_NODE "7CDFA1B2C3D40000"
static const char *SIG_EUI = "d42ad06a0e8edbe43a984088e6b47fd28cf9b3955cc6051eb20fb2b17225797a";

static void mk(char *buf, size_t n, const char *c, const char *args, const char *sig)
{
	snprintf(buf, n, "{\"c\":\"%s\"%s,\"ch\":\"" CH "\",\"sig\":\"%s\"}", c, args, sig);
}

static void test_cross_language_vectors(void)
{
	char s[300];
	struct cmd_msg m;
	char canon[128];

	mk(s, sizeof(s), "tau_set", ",\"a\":312", SIG_TAU);
	CHECK(parse(s, &m) == CMD_OK);
	CHECK(m.id == CMD_TAU_SET && m.has_a && m.a == 312 && !m.has_b);
	CHECK(cmd_canonical(canon, sizeof(canon), ZONE, "1", &m) > 0);
	CHECK(strcmp(canon, "852541|1|a1b2c3d4e5f60718|tau_set|312|") == 0);
	CHECK(cmd_verify(&m, KEY, sizeof(KEY), ZONE, "1"));

	mk(s, sizeof(s), "ant_set", ",\"a\":16453,\"b\":16385", SIG_ANT);
	CHECK(parse(s, &m) == CMD_OK && m.id == CMD_ANT_SET && m.a == 16453 && m.b == 16385);
	CHECK(cmd_verify(&m, KEY, sizeof(KEY), ZONE, "1"));

	mk(s, sizeof(s), "reboot", "", SIG_REB);
	CHECK(parse(s, &m) == CMD_OK && m.id == CMD_REBOOT && !m.has_a && !m.has_b);
	CHECK(cmd_verify(&m, KEY, sizeof(KEY), ZONE, "1"));

	mk(s, sizeof(s), "tau_set", ",\"a\":-45", SIG_NEG);
	CHECK(parse(s, &m) == CMD_OK && m.a == -45);
	CHECK(cmd_verify(&m, KEY, sizeof(KEY), ZONE, "1"));

	mk(s, sizeof(s), "tau_set", ",\"a\":312", SIG_EUI);
	CHECK(parse(s, &m) == CMD_OK);
	CHECK(cmd_canonical(canon, sizeof(canon), ZONE, EUI_NODE, &m) > 0);
	CHECK(strcmp(canon, "852541|" EUI_NODE "|a1b2c3d4e5f60718|tau_set|312|") == 0);
	CHECK(cmd_verify(&m, KEY, sizeof(KEY), ZONE, EUI_NODE));
	CHECK(!cmd_verify(&m, KEY, sizeof(KEY), ZONE, "1"));   /* the id is not the name */
}

static void test_every_signed_part_matters(void)
{
	char s[300];
	struct cmd_msg m;
	uint8_t k2[32];

	mk(s, sizeof(s), "tau_set", ",\"a\":312", SIG_TAU);
	CHECK(parse(s, &m) == CMD_OK);

	CHECK(!cmd_verify(&m, KEY, sizeof(KEY), "852542", "1"));      /* other zone */
	CHECK(!cmd_verify(&m, KEY, sizeof(KEY), ZONE, "2"));          /* other node */
	CHECK(!cmd_verify(&m, KEY, sizeof(KEY), ZONE, "gw"));
	{ struct cmd_msg x = m; x.a = 313;  CHECK(!cmd_verify(&x, KEY, sizeof(KEY), ZONE, "1")); }
	{ struct cmd_msg x = m; x.has_b = true; x.b = 0; CHECK(!cmd_verify(&x, KEY, sizeof(KEY), ZONE, "1")); }
	{ struct cmd_msg x = m; strcpy(x.name, "reboot"); CHECK(!cmd_verify(&x, KEY, sizeof(KEY), ZONE, "1")); }
	{ struct cmd_msg x = m; x.ch[0] = 'b'; CHECK(!cmd_verify(&x, KEY, sizeof(KEY), ZONE, "1")); }
	{ struct cmd_msg x = m; x.sig[31] ^= 1; CHECK(!cmd_verify(&x, KEY, sizeof(KEY), ZONE, "1")); }
	memcpy(k2, KEY, 32); k2[0] ^= 1;
	CHECK(!cmd_verify(&m, k2, sizeof(k2), ZONE, "1"));            /* other key */
	CHECK(!cmd_verify(&m, KEY, 0, ZONE, "1"));                    /* no key: fail closed */
	CHECK(!cmd_verify(&m, NULL, 0, ZONE, "1"));
}

#define ZSIG "0000000000000000000000000000000000000000000000000000000000000000"

static void test_parser_is_strict(void)
{
	struct cmd_msg m;
	char big[400];
	const char *tail = ",\"ch\":\"" CH "\",\"sig\":\"" ZSIG "\"}";
	char s[400];

	CHECK(parse("", &m) == CMD_E_FORMAT);
	CHECK(parse("{}", &m) == CMD_E_FORMAT);
	CHECK(parse("not json", &m) == CMD_E_FORMAT);
	CHECK(parse("{\"c\":\"reboot\"", &m) == CMD_E_FORMAT);                    /* truncated */
	snprintf(s, sizeof(s), "{\"c\":\"reboot\"%s", tail);
	CHECK(parse(s, &m) == CMD_OK);
	snprintf(s, sizeof(s), "{\"c\":\"reboot\"%s x", tail);
	CHECK(parse(s, &m) == CMD_E_FORMAT);                                       /* trailing junk */
	snprintf(s, sizeof(s), "{\"c\":\"reboot\",\"c\":\"status\"%s", tail);
	CHECK(parse(s, &m) == CMD_E_FORMAT);                                       /* duplicate key */
	snprintf(s, sizeof(s), "{\"c\":\"reboot\",\"a\":1,\"a\":2%s", tail);
	CHECK(parse(s, &m) == CMD_E_FORMAT);
	snprintf(s, sizeof(s), "{\"c\":\"reboot\",\"zz\":1%s", tail);
	CHECK(parse(s, &m) == CMD_E_FORMAT);                                       /* unknown key */
	snprintf(s, sizeof(s), "{\"c\":\"re\\\"boot\"%s", tail);
	CHECK(parse(s, &m) == CMD_E_FORMAT);                                       /* escape */
	snprintf(s, sizeof(s), "{\"c\":\"reboot\",\"a\":99999999999%s", tail);
	CHECK(parse(s, &m) == CMD_E_FORMAT);                                       /* int overflow */
	snprintf(s, sizeof(s), "{\"c\":\"reboot\",\"a\":2147483648%s", tail);
	CHECK(parse(s, &m) == CMD_E_FORMAT);
	snprintf(s, sizeof(s), "{\"c\":\"reboot\",\"a\":-2147483648%s", tail);
	CHECK(parse(s, &m) == CMD_OK && m.a == INT32_MIN);
	snprintf(s, sizeof(s), "{\"c\":\"reboot\",\"a\":-%s", tail);
	CHECK(parse(s, &m) == CMD_E_FORMAT);
	snprintf(s, sizeof(s), "{\"c\":\"reboot\",\"ch\":\"zzzzzzzzzzzzzzzz\",\"sig\":\"" ZSIG "\"}");
	CHECK(parse(s, &m) == CMD_E_FORMAT);                                       /* non-hex challenge */
	snprintf(s, sizeof(s), "{\"c\":\"reboot\",\"ch\":\"abcd\",\"sig\":\"" ZSIG "\"}");
	CHECK(parse(s, &m) == CMD_E_FORMAT);                                       /* short challenge */
	snprintf(s, sizeof(s), "{\"c\":\"reboot\",\"ch\":\"" CH "\",\"sig\":\"%063d\"}", 0);
	CHECK(parse(s, &m) == CMD_E_FORMAT);                                       /* short sig */
	snprintf(s, sizeof(s), "{\"c\":\"reboot\"}");
	CHECK(parse(s, &m) == CMD_E_FORMAT);                                       /* no ch / sig */
	snprintf(s, sizeof(s), "{\"c\":\"selfdestruct\"%s", tail);
	CHECK(parse(s, &m) == CMD_E_UNKNOWN);                                      /* not whitelisted */
	snprintf(s, sizeof(s), "{\"c\":\"abcdefghijklmnopqrstuvwxyz\"%s", tail);
	CHECK(parse(s, &m) == CMD_E_FORMAT);                                       /* name too long */

	memset(big, ' ', sizeof(big));
	memcpy(big, "{}", 2);
	CHECK(cmd_parse(big, CMD_MAX_LEN + 1, &m) == CMD_E_FORMAT);                /* oversize */
	CHECK(cmd_parse(NULL, 5, &m) == CMD_E_FORMAT);
	CHECK(cmd_parse("{}", 2, NULL) == CMD_E_FORMAT);
}

/* Random and mutated input must never crash or read out of bounds, and a parse that says OK must
 * be a fully formed command. */
static void test_fuzz_never_misparses(void)
{
	uint32_t st = 12345u;
	char seed[300];
	struct cmd_msg m;

	mk(seed, sizeof(seed), "tau_set", ",\"a\":312", SIG_TAU);
	for (int iter = 0; iter < 60000; iter++) {
		char buf[300];
		size_t n = strlen(seed);

		memcpy(buf, seed, n);
		int flips = 1 + (int)(st % 4u);

		for (int f = 0; f < flips; f++) {
			st = st * 1664525u + 1013904223u;
			buf[(st >> 8) % n] = (char)(st >> 24);
		}
		st = st * 1664525u + 1013904223u;
		size_t len = (st & 7u) == 0 ? ((st >> 8) % n) : n;
		int r = cmd_parse(buf, len, &m);

		CHECK(r == CMD_OK || r == CMD_E_FORMAT || r == CMD_E_UNKNOWN);
		if (r == CMD_OK) {
			CHECK(m.id != CMD_NONE && strlen(m.ch) == CMD_CH_HEX_LEN);
		}
	}
}

static void test_challenge_lifecycle(void)
{
	struct cmd_challenge c;

	cmd_challenge_init(&c);
	CHECK(!cmd_challenge_accepts(&c, CH));                         /* nothing issued yet */
	cmd_challenge_rotate(&c, 0xa1b2c3d4u, 0xe5f60718u);
	CHECK(strcmp(c.cur, CH) == 0);
	CHECK(cmd_challenge_accepts(&c, CH));
	CHECK(!cmd_challenge_accepts(&c, "a1b2c3d4e5f60719"));
	CHECK(!cmd_challenge_accepts(&c, "a1b2"));
	CHECK(!cmd_challenge_accepts(&c, NULL));

	cmd_challenge_rotate(&c, 1u, 2u);                              /* next status report */
	CHECK(cmd_challenge_accepts(&c, CH));                          /* previous still honoured */
	CHECK(cmd_challenge_accepts(&c, "0000000100000002"));
	cmd_challenge_rotate(&c, 3u, 4u);
	CHECK(!cmd_challenge_accepts(&c, CH));                         /* two periods old: dead */

	cmd_challenge_burn(&c);                                        /* a command was accepted */
	CHECK(!cmd_challenge_accepts(&c, "0000000300000004"));
	CHECK(!cmd_challenge_accepts(&c, "0000000000000000"));         /* a zeroed slot is not a key */
	cmd_challenge_rotate(&c, 5u, 6u);
	CHECK(cmd_challenge_accepts(&c, "0000000500000006"));
	CHECK(!cmd_challenge_accepts(&c, "0000000300000004"));         /* burned for good */
}

static void test_ack(void)
{
	char b[96];

	CHECK(cmd_ack_json(b, sizeof(b), "tau_set", true, NULL, true, 312) > 0);
	CHECK(strcmp(b, "{\"c\":\"tau_set\",\"ok\":1,\"v\":312}") == 0);
	CHECK(cmd_ack_json(b, sizeof(b), "reboot", true, NULL, false, 0) > 0);
	CHECK(strcmp(b, "{\"c\":\"reboot\",\"ok\":1}") == 0);
	CHECK(cmd_ack_json(b, sizeof(b), "tau_set", false, "bad_sig", false, 0) > 0);
	CHECK(strcmp(b, "{\"c\":\"tau_set\",\"ok\":0,\"err\":\"bad_sig\"}") == 0);
	CHECK(cmd_ack_json(b, sizeof(b), "a\"b", false, "bad_format", false, 0) > 0);
	CHECK(strstr(b, "\"c\":\"?\""));                               /* hostile name is not echoed */
	CHECK(cmd_ack_json(b, 10, "tau_set", true, NULL, true, 312) == -1);
}

int main(void)
{
	for (int i = 0; i < 32; i++) KEY[i] = (uint8_t)i;
	test_cross_language_vectors();
	test_every_signed_part_matters();
	test_parser_is_strict();
	test_fuzz_never_misparses();
	test_challenge_lifecycle();
	test_ack();
	if (failures) {
		printf("\n%d CHECK(s) FAILED\n", failures);
		return EXIT_FAILURE;
	}
	printf("cmd_proto: ALL TESTS PASSED\n");
	return EXIT_SUCCESS;
}
