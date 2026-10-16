/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "cmd_proto.h"

#include <stdio.h>
#include <string.h>

#include "hmac_sha256.h"

static const struct { const char *name; enum cmd_id id; } table[] = {
	{ "status",     CMD_STATUS },
	{ "sync_reset", CMD_SYNC_RESET },
	{ "tau_set",    CMD_TAU_SET },
	{ "tau_mode",   CMD_TAU_MODE },
	{ "ant_set",    CMD_ANT_SET },
	{ "reboot",     CMD_REBOOT },
};

/* ---- A deliberately tiny JSON reader ---------------------------------------
 * One flat object; keys and string values are plain ASCII with no escapes; numbers are signed
 * decimal integers. Anything else is a format error -- including a duplicated key, which would
 * otherwise let a later value silently override an earlier, signed one. */

struct rd {
	const char *p;
	const char *end;
};

static void skip_ws(struct rd *r)
{
	while (r->p < r->end && (*r->p == ' ' || *r->p == '\t' || *r->p == '\r' || *r->p == '\n')) {
		r->p++;
	}
}

static bool eat(struct rd *r, char c)
{
	skip_ws(r);
	if (r->p < r->end && *r->p == c) {
		r->p++;
		return true;
	}
	return false;
}

/* A quoted string without escapes into out (NUL terminated). False if it does not fit. */
static bool read_str(struct rd *r, char *out, size_t cap)
{
	size_t n = 0;

	if (!eat(r, '"')) {
		return false;
	}
	while (r->p < r->end && *r->p != '"') {
		unsigned char c = (unsigned char)*r->p;

		if (c < 0x20 || c == '\\' || c >= 0x7f || n + 1 >= cap) {
			return false;
		}
		out[n++] = (char)c;
		r->p++;
	}
	if (r->p >= r->end) {
		return false;
	}
	r->p++;                         /* closing quote */
	out[n] = '\0';
	return true;
}

static bool read_int(struct rd *r, int32_t *out)
{
	int64_t v = 0;
	bool neg = false;
	size_t digits = 0;

	skip_ws(r);
	if (r->p < r->end && *r->p == '-') {
		neg = true;
		r->p++;
	}
	while (r->p < r->end && *r->p >= '0' && *r->p <= '9') {
		v = v * 10 + (*r->p - '0');
		if (++digits > 10 || v > 2147483648LL) {
			return false;
		}
		r->p++;
	}
	if (digits == 0) {
		return false;
	}
	if (neg) {
		v = -v;
	}
	if (v > INT32_MAX || v < INT32_MIN) {
		return false;
	}
	*out = (int32_t)v;
	return true;
}

static int hexval(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

int cmd_parse(const char *payload, size_t len, struct cmd_msg *out)
{
	struct rd r = { payload, payload + len };
	bool got_c = false, got_ch = false, got_sig = false;
	char key[8];

	if (payload == NULL || out == NULL || len == 0 || len > CMD_MAX_LEN) {
		return CMD_E_FORMAT;
	}
	memset(out, 0, sizeof(*out));

	if (!eat(&r, '{')) {
		return CMD_E_FORMAT;
	}
	skip_ws(&r);
	if (r.p < r.end && *r.p == '}') {
		return CMD_E_FORMAT;                        /* empty object */
	}
	for (;;) {
		if (!read_str(&r, key, sizeof(key)) || !eat(&r, ':')) {
			return CMD_E_FORMAT;
		}
		if (strcmp(key, "c") == 0) {
			if (got_c || !read_str(&r, out->name, sizeof(out->name))) return CMD_E_FORMAT;
			got_c = true;
		} else if (strcmp(key, "a") == 0) {
			if (out->has_a || !read_int(&r, &out->a)) return CMD_E_FORMAT;
			out->has_a = true;
		} else if (strcmp(key, "b") == 0) {
			if (out->has_b || !read_int(&r, &out->b)) return CMD_E_FORMAT;
			out->has_b = true;
		} else if (strcmp(key, "ch") == 0) {
			if (got_ch || !read_str(&r, out->ch, sizeof(out->ch)) ||
			    strlen(out->ch) != CMD_CH_HEX_LEN) return CMD_E_FORMAT;
			for (size_t i = 0; i < CMD_CH_HEX_LEN; i++) {
				if (hexval(out->ch[i]) < 0) return CMD_E_FORMAT;
			}
			got_ch = true;
		} else if (strcmp(key, "sig") == 0) {
			char hex[65];

			if (got_sig || !read_str(&r, hex, sizeof(hex)) || strlen(hex) != 64) {
				return CMD_E_FORMAT;
			}
			for (size_t i = 0; i < 32; i++) {
				int hi = hexval(hex[2 * i]), lo = hexval(hex[2 * i + 1]);

				if (hi < 0 || lo < 0) return CMD_E_FORMAT;
				out->sig[i] = (uint8_t)((hi << 4) | lo);
			}
			got_sig = true;
		} else {
			return CMD_E_FORMAT;                    /* unknown key */
		}
		if (eat(&r, ',')) {
			continue;
		}
		if (eat(&r, '}')) {
			break;
		}
		return CMD_E_FORMAT;
	}
	skip_ws(&r);
	if (r.p != r.end || !got_c || !got_ch || !got_sig) {
		return CMD_E_FORMAT;                        /* trailing junk or a missing field */
	}

	for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
		if (strcmp(out->name, table[i].name) == 0) {
			out->id = table[i].id;
			return CMD_OK;
		}
	}
	return CMD_E_UNKNOWN;
}

int cmd_canonical(char *buf, size_t len, const char *zone, const char *node,
		  const struct cmd_msg *m)
{
	char a[16] = "", b[16] = "";
	int n;

	if (buf == NULL || zone == NULL || node == NULL || m == NULL) {
		return -1;
	}
	if (m->has_a) snprintf(a, sizeof(a), "%ld", (long)m->a);
	if (m->has_b) snprintf(b, sizeof(b), "%ld", (long)m->b);
	n = snprintf(buf, len, "%s|%s|%s|%s|%s|%s", zone, node, m->ch, m->name, a, b);
	return (n < 0 || (size_t)n >= len) ? -1 : n;
}

bool cmd_verify(const struct cmd_msg *m, const uint8_t *key, size_t key_len, const char *zone,
		const char *node)
{
	char canon[128];
	uint8_t mac[HMAC_SHA256_LEN];
	int n;

	if (key == NULL || key_len == 0) {
		return false;                               /* no key => nothing is authentic */
	}
	n = cmd_canonical(canon, sizeof(canon), zone, node, m);
	if (n < 0) {
		return false;
	}
	ancla_hmac_sha256(key, key_len, (const uint8_t *)canon, (size_t)n, mac);
	return ancla_hmac_sha256_equal(mac, m->sig);
}

/* ---- Challenge -------------------------------------------------------------- */

void cmd_challenge_init(struct cmd_challenge *c)
{
	memset(c, 0, sizeof(*c));
}

void cmd_challenge_rotate(struct cmd_challenge *c, uint32_t rand_hi, uint32_t rand_lo)
{
	if (c->have_cur) {
		memcpy(c->prev, c->cur, sizeof(c->prev));
		c->have_prev = true;
	}
	snprintf(c->cur, sizeof(c->cur), "%08x%08x", (unsigned int)rand_hi, (unsigned int)rand_lo);
	c->have_cur = true;
}

bool cmd_challenge_accepts(const struct cmd_challenge *c, const char *ch_hex)
{
	if (ch_hex == NULL || strlen(ch_hex) != CMD_CH_HEX_LEN) {
		return false;
	}
	/* Both compared in full every time: no early exit on the first mismatching byte. */
	uint8_t d_cur = c->have_cur ? 0 : 1, d_prev = c->have_prev ? 0 : 1;

	for (size_t i = 0; i < CMD_CH_HEX_LEN; i++) {
		d_cur  |= (uint8_t)(c->cur[i] ^ ch_hex[i]);
		d_prev |= (uint8_t)(c->prev[i] ^ ch_hex[i]);
	}
	return d_cur == 0 || d_prev == 0;
}

void cmd_challenge_burn(struct cmd_challenge *c)
{
	c->have_cur = false;
	c->have_prev = false;
	memset(c->cur, 0, sizeof(c->cur));
	memset(c->prev, 0, sizeof(c->prev));
}

/* ---- Replies ----------------------------------------------------------------- */

int cmd_ack_json(char *buf, size_t len, const char *name, bool ok, const char *err,
		 bool has_value, int32_t value)
{
	char safe[CMD_NAME_MAX + 1] = "?";
	int n;

	if (buf == NULL || len == 0) {
		return -1;
	}
	if (name != NULL && name[0] != '\0' && strlen(name) < sizeof(safe)) {
		bool printable = true;

		for (const char *p = name; *p; p++) {
			if ((unsigned char)*p < 0x21 || (unsigned char)*p >= 0x7f || *p == '"' ||
			    *p == '\\') {
				printable = false;
			}
		}
		if (printable) {
			strcpy(safe, name);
		}
	}
	if (ok) {
		if (has_value) {
			n = snprintf(buf, len, "{\"c\":\"%s\",\"ok\":1,\"v\":%ld}", safe, (long)value);
		} else {
			n = snprintf(buf, len, "{\"c\":\"%s\",\"ok\":1}", safe);
		}
	} else {
		n = snprintf(buf, len, "{\"c\":\"%s\",\"ok\":0,\"err\":\"%s\"}", safe,
			     err ? err : "error");
	}
	return (n < 0 || (size_t)n >= len) ? -1 : n;
}
