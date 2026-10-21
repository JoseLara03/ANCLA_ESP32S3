/*
 * Host tests for the pure half of node_eui.h: the canonical text form every
 * consumer (MQTT topics, command signatures, BLINK observations, the edge
 * server) must agree on byte for byte.
 *
 *   gcc -Wall -Wextra -Isrc -o tests/node_eui/t.exe tests/node_eui/test_node_eui.c
 */
#include "node_eui.h"

#include <stdio.h>
#include <string.h>

static int g_fail;

#define CHECK(cond) do { \
	if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static void test_fmt(void)
{
	/* An ESP32-S3 MAC, zero-padded to 8 bytes exactly as node_eui.c pads it. */
	const uint8_t mac[8] = { 0x7C, 0xDF, 0xA1, 0xB2, 0xC3, 0xD4, 0x00, 0x00 };
	const uint8_t all[8] = { 0x00, 0x0F, 0xF0, 0xFF, 0x09, 0xA0, 0x5A, 0xA5 };
	char out[NODE_EUI_HEX_LEN + 1u];

	memset(out, 'x', sizeof(out));
	node_eui_fmt(mac, out);
	CHECK(strcmp(out, "7CDFA1B2C3D40000") == 0);   /* upper case, byte order */
	CHECK(strlen(out) == NODE_EUI_HEX_LEN);

	node_eui_fmt(all, out);
	CHECK(strcmp(out, "000FF0FF09A05AA5") == 0);   /* nibble order */
}

static void test_parse(void)
{
	const uint8_t want[8] = { 0x7C, 0xDF, 0xA1, 0xB2, 0xC3, 0xD4, 0x00, 0x00 };
	uint8_t got[8];
	uint8_t keep[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	char back[NODE_EUI_HEX_LEN + 1u];

	CHECK(node_eui_parse("7CDFA1B2C3D40000", got));
	CHECK(memcmp(got, want, 8) == 0);
	CHECK(node_eui_parse("7cdfa1b2c3d40000", got));   /* lower case accepted */
	CHECK(memcmp(got, want, 8) == 0);

	node_eui_fmt(got, back);                          /* round trip */
	CHECK(strcmp(back, "7CDFA1B2C3D40000") == 0);

	memcpy(got, keep, 8);
	CHECK(!node_eui_parse("7CDFA1B2C3D4000", got));   /* 15 digits */
	CHECK(!node_eui_parse("7CDFA1B2C3D400000", got)); /* 17 digits */
	CHECK(!node_eui_parse("7CDFA1B2C3D4000G", got));  /* non-hex */
	CHECK(!node_eui_parse("7C:DF:A1:B2:C3:D4", got)); /* separators */
	CHECK(!node_eui_parse("", got));
	CHECK(!node_eui_parse(NULL, got));
	CHECK(memcmp(got, keep, 8) == 0);                 /* untouched on failure */
}

static void test_is_zero(void)
{
	const uint8_t z[8] = { 0 };
	const uint8_t nz[8] = { 0, 0, 0, 0, 0, 0, 0, 1 };

	CHECK(node_eui_is_zero(z));
	CHECK(!node_eui_is_zero(nz));
}

int main(void)
{
	test_fmt();
	test_parse();
	test_is_zero();
	if (g_fail) {
		printf("node_eui: %d FAILED\n", g_fail);
		return 1;
	}
	printf("node_eui: ALL TESTS PASSED\n");
	return 0;
}
