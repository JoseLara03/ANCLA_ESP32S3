/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "node_status.h"

#include <stdarg.h>
#include <stdio.h>

/* Append to buf at *off; false (and nothing more is written) once it would not fit. */
static bool put(char *buf, size_t len, size_t *off, const char *fmt, ...)
	__attribute__((format(printf, 4, 5)));

static bool put(char *buf, size_t len, size_t *off, const char *fmt, ...)
{
	va_list ap;
	int n;

	if (*off >= len) {
		return false;
	}
	va_start(ap, fmt);
	n = vsnprintf(buf + *off, len - *off, fmt, ap);
	va_end(ap);
	if (n < 0 || (size_t)n >= len - *off) {
		return false;
	}
	*off += (size_t)n;
	return true;
}

int node_status_json(char *buf, size_t len, const struct node_status *s)
{
	size_t o = 0;
	bool ok = true;

	if (buf == NULL || s == NULL || len == 0u) {
		return -1;
	}

	ok = ok && put(buf, len, &o, "{\"online\":1,\"role\":\"%s\"", s->is_gateway ? "gw" : "an");
	if (!s->is_gateway) {
		ok = ok && put(buf, len, &o, ",\"id\":%u", (unsigned int)s->anchor_id);
	}
	ok = ok && put(buf, len, &o, ",\"addr\":\"0x%04X\",\"up_s\":%u,\"reset\":\"%s\",\"fw\":\"%s\"",
		       (unsigned int)s->short_addr, (unsigned int)s->uptime_s,
		       s->reset ? s->reset : "?", s->fw ? s->fw : "?");

	ok = ok && put(buf, len, &o, ",\"net\":{");
	if (s->has_rssi) {
		ok = ok && put(buf, len, &o, "\"rssi\":%d,", (int)s->rssi_dbm);
	}
	ok = ok && put(buf, len, &o, "\"wifi_up\":%u,\"mqtt_up\":%u,\"pub\":%u,\"drop\":%u}",
		       (unsigned int)s->wifi_up, (unsigned int)s->mqtt_up,
		       (unsigned int)s->obs_pub, (unsigned int)s->obs_drop);

	if (s->has_sync) {
		ok = ok && put(buf, len, &o,
			       ",\"sync\":{\"valid\":%u,\"sigma\":%u,\"jitter\":%u,\"rms\":%u,"
			       "\"max\":%u,\"n_obs\":%u,\"rebase\":%u,\"gaps\":%u,\"outliers\":%u,"
			       "\"rx\":%u,\"rej\":%u,\"drift_ppb\":%d,\"fresh\":%u}",
			       s->sync_valid ? 1u : 0u, (unsigned int)s->sigma_dtu,
			       (unsigned int)s->jitter_dtu, (unsigned int)s->rms_dtu,
			       (unsigned int)s->max_dtu, (unsigned int)s->n_obs,
			       (unsigned int)s->rebaselines, (unsigned int)s->sync_gaps,
			       (unsigned int)s->outliers, (unsigned int)s->ccp_rx,
			       (unsigned int)s->ccp_rejected, (int)s->drift_ppb,
			       s->clock_fresh ? 1u : 0u);
		ok = ok && put(buf, len, &o,
			       ",\"tau\":{\"mode\":\"%s\",\"used\":%d,\"link\":%u}",
			       s->tau_mode ? s->tau_mode : "?", (int)s->tau_used_dtu,
			       s->tau_link ? 1u : 0u);
		ok = ok && put(buf, len, &o,
			       ",\"blink\":{\"rx\":%u,\"no_sync\":%u,\"stale\":%u,\"sent\":%u}",
			       (unsigned int)s->blink_rx, (unsigned int)s->blink_no_sync,
			       (unsigned int)s->blink_stale, (unsigned int)s->blink_sent);
	}

	if (s->cmd_enabled && s->challenge != NULL && s->challenge[0] != '\0') {
		ok = ok && put(buf, len, &o, ",\"cmd\":{\"ch\":\"%s\"}", s->challenge);
	}

	if (s->has_master) {
		ok = ok && put(buf, len, &o, ",\"ccp\":{\"sent\":%u,\"dropped\":%u}",
			       (unsigned int)s->ccp_sent, (unsigned int)s->ccp_dropped);
	}

	ok = ok && put(buf, len, &o, "}");
	return ok ? (int)o : -1;
}

int node_status_offline_json(char *buf, size_t len, bool is_gateway, uint8_t id)
{
	int n;

	if (buf == NULL || len == 0u) {
		return -1;
	}
	if (is_gateway) {
		n = snprintf(buf, len, "{\"online\":0,\"role\":\"gw\"}");
	} else {
		n = snprintf(buf, len, "{\"online\":0,\"role\":\"an\",\"id\":%u}", (unsigned int)id);
	}
	return (n < 0 || (size_t)n >= len) ? -1 : n;
}
