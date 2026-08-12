/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pos_json.h"

#include <stdio.h>

int pos_json_fix(char *buf, size_t len, const struct pos_fix *fix)
{
	int n;

	/* tagId is BARE UPPERCASE HEX: four digits, zero-padded, no "0x" prefix.
	 * 0x00AB formats as "00AB". This matches an existing Python consumer, so
	 * it is not free to change. Note it reads as decimal to a human --
	 * "1234" is 0x1234 = 4660, not one thousand two hundred thirty-four.
	 *
	 * z is the integer literal 0, not %.2f: the solver is 2D and the
	 * consumer expects 0.
	 *
	 * residual_m, n_anchors and batt_soc are deliberately absent. They stay
	 * on pos_sink.c's console log line. */
	n = snprintf(buf, len,
		     "{\"tagId\":\"%04X\",\"x\":%.2f,\"y\":%.2f,\"z\":0,"
		     "\"zoneName\":\"" POS_JSON_ZONE_NAME "\"}",
		     (unsigned int)fix->src_addr,
		     (double)fix->x, (double)fix->y);

	if (n < 0 || (size_t)n >= len) {
		return -1;
	}
	return n;
}

int pos_json_anchors(char *buf, size_t len)
{
	/* Stub: the real anchor positions need a survey step that does not exist
	 * yet. The schema is final; only the numbers are placeholders, so a
	 * downstream consumer written against this stays valid. */
	static const char doc[] =
		"{\"name\":\"" POS_JSON_ZONE_NAME "\",\"anchors\":["
		"{\"name\":\"A0\",\"isAxis\":true,\"isReferenceAxis\":true,"
		"\"latitude\":0.0,\"longitude\":0.0,\"x\":0.0,\"y\":0.0,\"z\":0.0},"
		"{\"name\":\"A1\",\"isAxis\":true,\"isReferenceAxis\":false,"
		"\"latitude\":0.0,\"longitude\":0.0,\"x\":0.0,\"y\":0.0,\"z\":0.0},"
		"{\"name\":\"A2\",\"isAxis\":false,\"isReferenceAxis\":false,"
		"\"latitude\":0.0,\"longitude\":0.0,\"x\":0.0,\"y\":0.0,\"z\":0.0}"
		"]}";
	int n = snprintf(buf, len, "%s", doc);

	if (n < 0 || (size_t)n >= len) {
		return -1;
	}
	return n;
}
