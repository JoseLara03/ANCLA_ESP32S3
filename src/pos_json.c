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

	/* Tid is fix->src_addr as a PLAIN DECIMAL NUMBER, not hex and not a
	 * quoted string -- 0x1234 formats as 4660, not "1234". This matches the
	 * downstream consumer's actual schema; there is no zoneName here, the
	 * consumer gets the zone from the anchors topic instead.
	 *
	 * z is the integer literal 0, not %.2f: the solver is 2D and there is
	 * no z measurement yet.
	 *
	 * residual_m, n_anchors and batt_soc are deliberately absent. They stay
	 * on pos_sink.c's console log line. */
	n = snprintf(buf, len,
		     "{\"Tid\":%u,\"x\":%.2f,\"y\":%.2f,\"z\":0}",
		     (unsigned int)fix->src_addr,
		     (double)fix->x, (double)fix->y);

	if (n < 0 || (size_t)n >= len) {
		return -1;
	}
	return n;
}

int pos_json_anchors(char *buf, size_t len)
{
	/* Stub: the real anchor positions need a survey step that does not
	 * exist yet, so the four anchors are placed at the corners of a 2 m x
	 * 2 m square. ANC-LOBBY-001 is the origin and the sole axis/reference
	 * anchor, and the only one carrying a real (building-level) lat/long --
	 * the other three are local-only until they get their own survey. The
	 * schema is final; only the numbers are placeholders, so a downstream
	 * consumer written against this stays valid. */
	static const char doc[] =
		"{\"name\":\"" POS_JSON_ZONE_NAME "\",\"anchors\":["
		"{\"name\":\"ANC-LOBBY-001\",\"isAxis\":true,\"isReferenceAxis\":true,"
		"\"latitude\":21.01604164655441,\"longitude\":-89.6521292940793,"
		"\"x\":0.0,\"y\":0.0,\"z\":0.0},"
		"{\"name\":\"ANC-LOBBY-002\",\"isAxis\":false,\"isReferenceAxis\":false,"
		"\"latitude\":0,\"longitude\":0,\"x\":2.0,\"y\":0.0,\"z\":0.0},"
		"{\"name\":\"ANC-LOBBY-003\",\"isAxis\":false,\"isReferenceAxis\":false,"
		"\"latitude\":0,\"longitude\":0,\"x\":2.0,\"y\":2.0,\"z\":0.0},"
		"{\"name\":\"ANC-LOBBY-004\",\"isAxis\":false,\"isReferenceAxis\":false,"
		"\"latitude\":0,\"longitude\":0,\"x\":0.0,\"y\":2.0,\"z\":0.0}"
		"]}";
	int n = snprintf(buf, len, "%s", doc);

	if (n < 0 || (size_t)n >= len) {
		return -1;
	}
	return n;
}
