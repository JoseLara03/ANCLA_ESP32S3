/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pos_json.h"

#include "uwb_frame_802_15_4z.h"   /* UWB_FRAME_POS_SOC_CONNECTED */

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int pos_json_fix(char *buf, size_t len, const struct pos_fix *fix)
{
	int n;

	/* Tid is fix->tag_id as a PLAIN DECIMAL NUMBER, not hex and not a
	 * quoted string. tag_id is a stable per-physical-tag value derived
	 * from the tag's EUI (tag_id_from_eui(), src/tag_id.c) -- NOT
	 * fix->src_addr, which is only the tag's current MAC short address
	 * and gets reallocated across a rejoin (gw_core.c's seat table wipes
	 * a tag's record, EUI included, once its lease expires). Publishing
	 * src_addr here would make the platform see one physical tag as many
	 * different Tid values over its lifetime -- this is why the fixed
	 * contract's *format* (plain decimal, unquoted) survives unchanged
	 * while its *source field* does not. This matches the downstream
	 * consumer's actual schema; there is no zoneName here, the consumer
	 * gets the zone from the anchors topic instead.
	 *
	 * z is the integer literal 0, not %.2f: the solver is 2D and there is
	 * no z measurement yet.
	 *
	 * residual_m and n_anchors are deliberately absent. They stay on
	 * pos_sink.c's console log line.
	 *
	 * ---- Battery, added 2026-09-03 -------------------------------------
	 *
	 * TWO fields, not one, and both are always present and always the same
	 * JSON type -- a schema that changes shape per message is what made the
	 * Tid int32 truncation so slow to find (see CLAUDE.md): this consumer
	 * drops what it cannot parse, silently.
	 *
	 *   "batt"  0..100, or -1 when the tag reported no percentage.
	 *   "chg"   1 when there is no percentage reading, else 0.
	 *
	 * -1 rather than 255 for the no-reading case, because 255 is IN RANGE
	 * for a byte and reads as a percentage; -1 cannot be mistaken for one.
	 * Never emitted as JSON null, so the column type stays integer.
	 *
	 * HONESTY NOTE ON "chg", because the name promises more than the wire
	 * can deliver: both fields are derived from the SAME single sentinel,
	 * batt_soc == UWB_FRAME_POS_SOC_CONNECTED (0xFF). There is exactly one
	 * bit of information here, so "chg" carries none of its own -- it is a
	 * restatement of "batt == -1", not an independent measurement. The tag
	 * DOES distinguish the causes internally (batt.c returns -EBUSY
	 * specifically for a connected charger, and other failures separately),
	 * but every one of them collapses to 0xFF before transmission. So a
	 * FAILED FUEL GAUGE also publishes chg = 1. Making this field mean what
	 * its name says needs a flag on the wire -- the POS frame and
	 * blink_frame.flags both have room -- which is a protocol change, not a
	 * formatting one. */
	{
		bool no_reading =
			(fix->batt_soc == UWB_FRAME_POS_SOC_CONNECTED);

		n = snprintf(buf, len,
			     "{\"Tid\":%u,\"x\":%.2f,\"y\":%.2f,\"z\":0,"
			     "\"batt\":%d,\"chg\":%u}",
			     (unsigned int)fix->tag_id,
			     (double)fix->x, (double)fix->y,
			     no_reading ? -1 : (int)fix->batt_soc,
			     no_reading ? 1u : 0u);
	}

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
	 * APOS_MIN_NODES_3D (4) is isostatic (6 edges == 3N-6 free parameters), so
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

/* A minimal scanner, not a JSON parser. Justified because the producer is
 * pos_json_blink() in this same file and the format is pinned by
 * tests/pos_json_blink/: pulling cJSON into the firmware for six scalar fields
 * would cost flash and a dependency for nothing. Finds "key": and returns the
 * first character of the value, skipping an opening quote if there is one. */
static const char *scan_value(const char *json, const char *key)
{
	char pat[16];
	const char *p;
	int n = snprintf(pat, sizeof(pat), "\"%s\":", key);

	if (n < 0 || (size_t)n >= sizeof(pat)) {
		return NULL;
	}
	p = strstr(json, pat);
	if (p == NULL) {
		return NULL;
	}
	p += (size_t)n;
	if (*p == '"') {
		p++;
	}
	return p;
}

static bool scan_u32(const char *json, const char *key, uint32_t *out)
{
	const char *p = scan_value(json, key);

	if (p == NULL || *p < '0' || *p > '9') {
		return false;
	}
	*out = (uint32_t)strtoul(p, NULL, 10);
	return true;
}

static bool scan_i64(const char *json, const char *key, int64_t *out)
{
	const char *p = scan_value(json, key);
	long long v;

	if (p == NULL) {
		return false;
	}
	if (*p != '-' && (*p < '0' || *p > '9')) {
		return false;
	}

	/* errno is checked, not ignored: strtoll() SATURATES on overflow and
	 * still returns a perfectly plausible-looking number, so without this
	 * a 20-digit ts parses as LLONG_MAX and reports success. errno must be
	 * cleared first -- strtoll() only ever sets it, never clears it. */
	errno = 0;
	v = strtoll(p, NULL, 10);
	if (errno == ERANGE) {
		return false;
	}

	*out = (int64_t)v;
	return true;
}

int pos_json_blink(char *buf, size_t len, const struct pos_blink_obs *o)
{
	int n;

	if (buf == NULL || o == NULL) {
		return -1;
	}

	n = snprintf(buf, len,
		     "{\"a\":%u,\"t\":%u,\"s\":%u,\"ts\":\"%lld\",\"q\":%u,\"b\":%u}",
		     (unsigned int)o->anchor_id, (unsigned int)o->tag_addr,
		     (unsigned int)o->blink_seq, (long long)o->t_dtu,
		     (unsigned int)o->quality, (unsigned int)o->batt_soc);

	if (n < 0 || (size_t)n >= len) {
		return -1;
	}
	return n;
}

int pos_json_blink_parse(const char *buf, size_t len, struct pos_blink_obs *out)
{
	/* NUL-terminate a bounded copy: an MQTT payload does not arrive
	 * terminated, and strstr() over unterminated memory would read out of
	 * bounds. */
	char scratch[POS_JSON_BLINK_MAX_LEN];
	uint32_t a, t, s, q, b;
	int64_t ts;

	if (buf == NULL || out == NULL) {
		return -1;
	}
	if (len == 0u || len >= sizeof(scratch)) {
		return -1;
	}
	memcpy(scratch, buf, len);
	scratch[len] = '\0';

	if (!scan_u32(scratch, "a", &a) || !scan_u32(scratch, "t", &t) ||
	    !scan_u32(scratch, "s", &s) || !scan_u32(scratch, "q", &q) ||
	    !scan_u32(scratch, "b", &b) || !scan_i64(scratch, "ts", &ts)) {
		return -1;
	}
	if (a > 0xFFu || t > 0xFFFFu || s > 0xFFu || q > 0xFFFFu || b > 0xFFu) {
		return -1;
	}
	/* t_dtu is a DW3220 device time, so it is bounded on BOTH sides: never
	 * negative, and never past the counter's 40-bit domain. Every other
	 * field here is already bounded; leaving this one open would be the
	 * hole, since it is the only field the solver actually consumes. An
	 * out-of-domain value is a broken publisher or a corrupt payload, and
	 * accepting it would hand Task 6 a difference against a real timestamp
	 * that yields a confidently wrong position -- or a NaN -- from a call
	 * that returned success. */
	if (ts < 0 || ts > POS_JSON_BLINK_TS_MAX) {
		return -1;
	}

	out->anchor_id = (uint8_t)a;
	out->blink_seq = (uint8_t)s;
	out->batt_soc  = (uint8_t)b;
	out->tag_addr  = (uint16_t)t;
	out->quality   = (uint16_t)q;
	out->t_dtu     = ts;
	return 0;
}
