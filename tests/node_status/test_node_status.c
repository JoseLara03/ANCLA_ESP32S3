/*
 * Host tests for node_status: the JSON every node publishes about itself.
 *
 *   gcc -Wall -Wextra -D__USE_MINGW_ANSI_STDIO=1 -Isrc -o tests/node_status/t.exe \
 *       tests/node_status/test_node_status.c src/node_status.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "node_status.h"

static int failures;

#define CHECK(cond)                                                            \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                            \
		}                                                              \
	} while (0)

/* A structural validator, not a parser: braces and quotes balance, no control characters, no
 * trailing comma before a closing brace, and the whole thing is one object. It rejects exactly
 * the failure this module can have -- truncation and stray punctuation. */
static int looks_like_json_object(const char *s)
{
	int depth = 0, in_str = 0;
	size_t n = strlen(s);

	if (n < 2 || s[0] != '{' || s[n - 1] != '}') return 0;
	for (size_t i = 0; i < n; i++) {
		char c = s[i];

		if ((unsigned char)c < 0x20) return 0;
		if (c == '"') in_str = !in_str;
		if (in_str) continue;
		if (c == '{') depth++;
		if (c == '}') {
			if (i > 0 && s[i - 1] == ',') return 0;
			depth--;
			if (depth < 0) return 0;
			if (depth == 0 && i != n - 1) return 0;
		}
		if (c == ',' && (s[i + 1] == ',' || s[i + 1] == '}')) return 0;
	}
	return depth == 0 && !in_str;
}

static struct node_status slave(void)
{
	struct node_status s = {0};

	s.is_gateway = false; s.anchor_id = 1; s.short_addr = 0x0002; s.uptime_s = 12498;
	s.reset = "brownout"; s.fw = "Oct 15 2026 12:00:00";
	s.has_rssi = true; s.rssi_dbm = -58;
	s.wifi_up = 2; s.mqtt_up = 3; s.obs_pub = 55000; s.obs_drop = 4;
	s.has_sync = true; s.sync_valid = true; s.sigma_dtu = 22; s.jitter_dtu = 18; s.rms_dtu = 29;
	s.max_dtu = 411; s.n_obs = 77; s.rebaselines = 5; s.sync_gaps = 54; s.outliers = 1;
	s.ccp_rx = 55125; s.ccp_rejected = 21; s.drift_ppb = -6681; s.clock_fresh = true;
	s.tau_used_dtu = 312; s.tau_mode = "survey"; s.tau_link = true;
	s.blink_rx = 9000; s.blink_no_sync = 2; s.blink_stale = 0; s.blink_sent = 8998;
	return s;
}

static void test_slave_report(void)
{
	char b[1024];
	struct node_status s = slave();
	int n = node_status_json(b, sizeof(b), &s);

	CHECK(n > 0 && (size_t)n == strlen(b));
	CHECK(looks_like_json_object(b));
	CHECK(strstr(b, "\"role\":\"an\",\"id\":1"));
	CHECK(strstr(b, "\"addr\":\"0x0002\""));
	CHECK(strstr(b, "\"reset\":\"brownout\""));
	CHECK(strstr(b, "\"rssi\":-58"));
	CHECK(strstr(b, "\"sigma\":22"));
	CHECK(strstr(b, "\"outliers\":1"));
	CHECK(strstr(b, "\"drift_ppb\":-6681"));
	CHECK(strstr(b, "\"tau\":{\"mode\":\"survey\",\"used\":312,\"link\":1}"));
	CHECK(!strstr(b, "\"ccp\":"));                      /* gateway-only */
	printf("  slave report is %d bytes\n", n);
	CHECK(n < 700);                                      /* leaves room in POS_JSON_MAX_LEN */
}

static void test_gateway_report(void)
{
	char b[1024];
	struct node_status s = {0};

	s.is_gateway = true; s.short_addr = 0x0000; s.uptime_s = 5; s.reset = "power"; s.fw = "x";
	s.has_master = true; s.ccp_sent = 4808; s.ccp_dropped = 0;
	int n = node_status_json(b, sizeof(b), &s);

	CHECK(n > 0);
	CHECK(looks_like_json_object(b));
	CHECK(strstr(b, "\"role\":\"gw\""));
	CHECK(!strstr(b, "\"id\":"));                        /* the gateway has no anchor id */
	CHECK(!strstr(b, "\"sync\":"));                      /* nor a slave sync model */
	CHECK(strstr(b, "\"ccp\":{\"sent\":4808,\"dropped\":0}"));
	CHECK(!strstr(b, "\"rssi\""));                       /* absent, not zero, when unknown */
}

static void test_challenge_is_published_only_when_commands_are_enabled(void)
{
	char b[1024];
	struct node_status s = slave();

	s.cmd_enabled = true;
	s.challenge = "a1b2c3d4e5f60718";
	CHECK(node_status_json(b, sizeof(b), &s) > 0);
	CHECK(looks_like_json_object(b));
	CHECK(strstr(b, "\"cmd\":{\"ch\":\"a1b2c3d4e5f60718\"}"));

	s.cmd_enabled = false;                                 /* no key stored: nothing to sign with */
	CHECK(node_status_json(b, sizeof(b), &s) > 0);
	CHECK(!strstr(b, "\"cmd\""));

	s.cmd_enabled = true;
	s.challenge = "";                                      /* enabled but none issued yet */
	CHECK(node_status_json(b, sizeof(b), &s) > 0);
	CHECK(!strstr(b, "\"cmd\""));
	s.challenge = NULL;
	CHECK(node_status_json(b, sizeof(b), &s) > 0);
	CHECK(!strstr(b, "\"cmd\""));
}

static void test_missing_rssi_is_omitted_not_zero(void)
{
	char b[1024];
	struct node_status s = slave();

	s.has_rssi = false;
	CHECK(node_status_json(b, sizeof(b), &s) > 0);
	CHECK(looks_like_json_object(b));
	CHECK(!strstr(b, "rssi"));
}

/* The property that protects the retained topic: for EVERY buffer size smaller than the
 * document, the call fails -- it never returns a truncated object. */
static void test_never_truncates(void)
{
	char full[1024], small[1024];
	struct node_status s = slave();
	int n = node_status_json(full, sizeof(full), &s);

	CHECK(n > 0);
	for (int len = 1; len <= n; len++) {
		memset(small, 'Z', sizeof(small));
		int r = node_status_json(small, (size_t)len, &s);

		CHECK(r == -1);
	}
	CHECK(node_status_json(small, (size_t)n + 1, &s) == n);
	CHECK(strcmp(small, full) == 0);
}

static void test_null_and_zero_length(void)
{
	char b[8];
	struct node_status s = slave();

	CHECK(node_status_json(NULL, 100, &s) == -1);
	CHECK(node_status_json(b, 0, &s) == -1);
	CHECK(node_status_json(b, sizeof(b), NULL) == -1);
}

static void test_null_strings_do_not_crash(void)
{
	char b[1024];
	struct node_status s = slave();

	s.reset = NULL; s.fw = NULL; s.tau_mode = NULL;
	CHECK(node_status_json(b, sizeof(b), &s) > 0);
	CHECK(looks_like_json_object(b));
}

static void test_offline_will(void)
{
	char b[64];

	CHECK(node_status_offline_json(b, sizeof(b), false, 3) > 0);
	CHECK(strcmp(b, "{\"online\":0,\"role\":\"an\",\"id\":3}") == 0);
	CHECK(looks_like_json_object(b));
	CHECK(node_status_offline_json(b, sizeof(b), true, 9) > 0);
	CHECK(strcmp(b, "{\"online\":0,\"role\":\"gw\"}") == 0);
	CHECK(node_status_offline_json(b, 5, false, 3) == -1);
}

int main(void)
{
	test_slave_report();
	test_gateway_report();
	test_missing_rssi_is_omitted_not_zero();
	test_challenge_is_published_only_when_commands_are_enabled();
	test_never_truncates();
	test_null_and_zero_length();
	test_null_strings_do_not_crash();
	test_offline_will();

	if (failures) {
		printf("\n%d CHECK(s) FAILED\n", failures);
		return EXIT_FAILURE;
	}
	printf("node_status: ALL TESTS PASSED\n");
	return EXIT_SUCCESS;
}
