/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pos_json.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
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

/* The original four-anchor placeholder. Kept as the fallback for a gateway that
 * has never been surveyed: a schema-valid document with placeholder numbers is
 * strictly better than no document, because a downstream consumer written
 * against this stays valid either way. */
static int anchors_stub(char *buf, size_t len)
{
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

int pos_json_anchors(char *buf, size_t len, const struct apos_survey *s)
{
	if (!buf || len == 0) {
		return -1;
	}
	if (!s || !s->valid || s->n_nodes == 0) {
		return anchors_stub(buf, len);
	}

	/* A non-finite coordinate would still format under %f (as a bare `nan`
	 * or `inf` token, which is not valid JSON) and this function would
	 * still report success -- the retained document would then be rejected
	 * whole by the platform's parser, which is worse than the stub it
	 * replaced. Refuse the whole document up front, for the same reason
	 * truncation below is refused rather than published partially.
	 * APOS_MIN_NODES (4) is isostatic (6 edges == 3N-6 free parameters), so
	 * a degenerate/near-collinear solve can be accepted by the solver with
	 * a non-finite coordinate in it -- this is the last check before MQTT. */
	if (s->ref_valid && (!isfinite(s->ref_lat) || !isfinite(s->ref_lon))) {
		return -1;
	}
	for (uint8_t k = 0; k < s->n_nodes; k++) {
		if (!isfinite(s->node[k].x) || !isfinite(s->node[k].y) ||
		    !isfinite(s->node[k].z)) {
			return -1;
		}
	}

	/* Accumulated with a running offset rather than one giant snprintf: the
	 * node count is variable. Every append is bounds-checked, and any
	 * overflow returns -1 for the whole document -- a truncated retained
	 * publish would poison the topic until the next connect. */
	size_t off = 0;
	int n = snprintf(buf, len, "{\"name\":\"" POS_JSON_ZONE_NAME
				   "\",\"anchors\":[");

	if (n < 0 || (size_t)n >= len) {
		return -1;
	}
	off = (size_t)n;

	for (uint8_t k = 0; k < s->n_nodes; k++) {
		bool is_ref = (k == 0);

		/* node[0] is the gauge origin by construction (apos_store.h),
		 * so it is the axis anchor and the only one carrying a real
		 * lat/long. Everything else is local-only, in metres relative
		 * to it -- which is what the schema already meant. */
		/* The `ANC-LOBBY-%03u` prefix is deliberately UNCHANGED from the
		 * stub below, even though this branch publishes the real
		 * survey. The retained anchors document is a live contract with
		 * a customer platform that probably keys anchor records by
		 * `name`: renaming them at the first `apos apply` would orphan
		 * the four existing records and silently create four new ones,
		 * one-way and customer-visible. Keeping the name means the first
		 * apply changes only the coordinates, which is the reversible
		 * direction. Switching to a zone-derived name is a decision for
		 * whoever owns the platform integration, not a side effect of
		 * surveying. */
		n = snprintf(buf + off, len - off,
			     "%s{\"name\":\"ANC-LOBBY-%03u\","
			     "\"isAxis\":%s,\"isReferenceAxis\":%s,"
			     "\"latitude\":%.8f,\"longitude\":%.8f,"
			     "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
			     (k == 0) ? "" : ",",
			     (unsigned int)s->node[k].short_addr,
			     is_ref ? "true" : "false",
			     is_ref ? "true" : "false",
			     (is_ref && s->ref_valid) ? s->ref_lat : 0.0,
			     (is_ref && s->ref_valid) ? s->ref_lon : 0.0,
			     (double)s->node[k].x, (double)s->node[k].y,
			     (double)s->node[k].z);

		if (n < 0 || (size_t)n >= len - off) {
			return -1;
		}
		off += (size_t)n;
	}

	n = snprintf(buf + off, len - off, "]}");
	if (n < 0 || (size_t)n >= len - off) {
		return -1;
	}
	return (int)(off + (size_t)n);
}
