/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This board's EUI-64: the ONE identity of a node everywhere off the air.
 *
 * The survey, the static sync links, the MQTT topics, the command signatures
 * and the BLINK observations all name a node by this value. The 16-bit short
 * address is only a radio handle the gateway hands out (apos assign), and must
 * never be used to tell two boards apart -- see
 * docs/superpowers/specs/2026-10-21-anchor-eui-identity-design.md.
 *
 * On the ESP32-S3 hwinfo returns the 6-byte eFuse MAC, not a real EUI-64, so
 * bytes 6-7 are zero-padded here. The hex form is therefore the MAC printed on
 * the module label followed by "0000", which an installer can read without a
 * cable. The padding is not cosmetic: the tail is hashed for the enumeration
 * stagger and compared for identity, so uninitialised bytes would change this
 * board's identity between boots.
 *
 * Read once (node_eui_init() from main(), or lazily on first use, which is what
 * lets the host tests link this file against a fake hwinfo).
 */

#ifndef NODE_EUI_H
#define NODE_EUI_H

#include <stdbool.h>
#include <stdint.h>

#define NODE_EUI_LEN     8u
#define NODE_EUI_HEX_LEN 16u   /* characters, excluding the NUL */

/* Read hwinfo and format the hex string. Idempotent. Call first in main(),
 * before anything that publishes or transmits an identity. */
void node_eui_init(void);

/* False when hwinfo failed or returned all zeros. An all-zero EUI is accepted
 * by nothing that keys on identity (the gateway refuses it in apos assign): it
 * is what every board with an unprogrammed eFuse would share. */
bool node_eui_valid(void);

/* The 8 bytes, zero-padded. Always readable; all zeros when !node_eui_valid(). */
const uint8_t *node_eui(void);

/* 16 upper-case hex digits plus NUL, e.g. "7CDFA1B2C3D40000". */
const char *node_eui_hex(void);

/* The canonical text form of ANY EUI: 16 upper-case hex digits, the 8 bytes in
 * order, no separators. Pure, so every consumer formats a foreign EUI exactly
 * the way this board formats its own. `out` must hold 17 bytes. */
static inline void node_eui_fmt(const uint8_t eui[NODE_EUI_LEN],
				char out[NODE_EUI_HEX_LEN + 1u])
{
	static const char hex[] = "0123456789ABCDEF";

	for (unsigned int i = 0; i < NODE_EUI_LEN; i++) {
		out[2u * i]      = hex[eui[i] >> 4];
		out[2u * i + 1u] = hex[eui[i] & 0x0Fu];
	}
	out[NODE_EUI_HEX_LEN] = '\0';
}

/* Inverse of node_eui_fmt(): exactly 16 hex digits (either case), nothing
 * else. Returns false, leaving `out` untouched, on any other input. */
static inline bool node_eui_parse(const char *s, uint8_t out[NODE_EUI_LEN])
{
	uint8_t tmp[NODE_EUI_LEN];

	if (s == 0) {
		return false;
	}
	for (unsigned int i = 0; i < NODE_EUI_HEX_LEN; i++) {
		char c = s[i];
		uint8_t v;

		if (c >= '0' && c <= '9') {
			v = (uint8_t)(c - '0');
		} else if (c >= 'A' && c <= 'F') {
			v = (uint8_t)(c - 'A' + 10);
		} else if (c >= 'a' && c <= 'f') {
			v = (uint8_t)(c - 'a' + 10);
		} else {
			return false;   /* includes an early NUL */
		}
		if ((i & 1u) == 0u) {
			tmp[i / 2u] = (uint8_t)(v << 4);
		} else {
			tmp[i / 2u] |= v;
		}
	}
	if (s[NODE_EUI_HEX_LEN] != '\0') {
		return false;
	}
	for (unsigned int i = 0; i < NODE_EUI_LEN; i++) {
		out[i] = tmp[i];
	}
	return true;
}

/* True when every byte is zero -- the unprogrammed-eFuse value. */
static inline bool node_eui_is_zero(const uint8_t eui[NODE_EUI_LEN])
{
	for (unsigned int i = 0; i < NODE_EUI_LEN; i++) {
		if (eui[i] != 0u) {
			return false;
		}
	}
	return true;
}

#endif /* NODE_EUI_H */
