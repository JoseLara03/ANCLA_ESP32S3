/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_store.h.
 *
 * Written as two blobs under "apos/", deliberately unlike uwb_store.c's
 * one-key-per-field scheme. That scheme exists so a new field does not force a
 * version byte and a wipe -- the right call for a config struct that grows. A
 * survey is different: it is one atomic measurement result in which a partially
 * applied set of coordinates is worse than none, and the whole record is always
 * written and read together. The geographic reference is the second blob because
 * it is entered once per site and must survive a re-survey.
 */

#include "apos_store.h"

#include "apos_addrmap.h"
#include "apos_links.h"

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(apos_store, LOG_LEVEL_INF);

#define KEY_SURVEY "apos/survey"
#define KEY_REF    "apos/ref"
#define KEY_TAGZ   "apos/tagz"
#define KEY_LINKS   "apos/links"
#define KEY_PENDING "apos/pending"
#define KEY_ADDRMAP "apos/addrmap"

/* Only the geometry, so adding a field to struct apos_survey does not silently
 * change the stored layout: this record is what is written, and the size check
 * on load is what catches a layout change. */
struct stored_survey {
	struct apos_survey_node node[APOS_MAX_NODES];
	uint8_t                 n_nodes;
	uint8_t                 valid;
	uint8_t                 dim; /* enum apos_geom_dim, stored as a plain
				      * uint8_t like valid above */
};

struct stored_ref {
	double  lat;
	double  lon;
	uint8_t valid;
};

/* Own key, own record: the tag plane's z is a fact about the SITE, not a
 * result of the survey, so it must survive apos_store_save() and
 * apos_store_clear() -- exactly like stored_ref. */
struct stored_tagz {
	float z;
};

static struct apos_survey g_survey;

/* Bumped on every change to what pos_json_anchors() publishes (survey, ref), so
 * the uplink can republish the retained anchors topic without being told. */
static volatile uint32_t g_survey_changes;

/* The static sync links and the apply-in-progress marker. Separate keys and a
 * separate, versioned codec (apos_links.c): the survey blob above is validated
 * by sizeof alone and is deliberately left byte-for-byte as it was, so a board
 * upgraded in place keeps its survey. */
static struct apos_links g_links;
static struct apos_pending g_pending;
static bool g_links_rejected;   /* a record was stored but could not be read */
static bool g_pending_rejected;

/* The EUI -> address map `apos assign` keeps (apos_addrmap.h). Same discipline
 * as the links: versioned codec, an unreadable record is kept on flash and
 * reported, never reinterpreted. */
static struct apos_addrmap g_addrmap;
static bool g_addrmap_rejected;

static int read_val(settings_read_cb read_cb, void *cb_arg, void *dst,
		    size_t len)
{
	ssize_t n = read_cb(cb_arg, dst, len);

	if (n != (ssize_t)len) {
		return -EINVAL;
	}
	return 0;
}

static int apos_settings_set(const char *key, size_t len,
			     settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(key, "survey") == 0) {
		struct stored_survey s;

		if (len != sizeof(s)) {
			LOG_WRN("stored survey size %u invalid — expected %u, "
				"treating as no survey",
				(unsigned int)len, (unsigned int)sizeof(s));
			return -EINVAL;
		}
		if (read_val(read_cb, cb_arg, &s, sizeof(s))) {
			return -EINVAL;
		}
		if (s.n_nodes > APOS_MAX_NODES) {
			LOG_WRN("stored survey claims %u nodes, max is %u — "
				"treating as no survey", s.n_nodes,
				APOS_MAX_NODES);
			return -EINVAL;
		}
		memcpy(g_survey.node, s.node, sizeof(g_survey.node));
		g_survey.n_nodes = s.n_nodes;
		g_survey.valid = s.valid != 0u;
		g_survey.dim = (enum apos_geom_dim)s.dim;
		return 0;
	}

	if (strcmp(key, "ref") == 0) {
		struct stored_ref r;

		if (len != sizeof(r)) {
			LOG_WRN("stored ref size %u invalid — expected %u, "
				"keeping none",
				(unsigned int)len, (unsigned int)sizeof(r));
			return -EINVAL;
		}
		if (read_val(read_cb, cb_arg, &r, sizeof(r))) {
			return -EINVAL;
		}
		g_survey.ref_lat = r.lat;
		g_survey.ref_lon = r.lon;
		g_survey.ref_valid = r.valid != 0u;
		return 0;
	}

	if (strcmp(key, "tagz") == 0) {
		struct stored_tagz t;

		if (len != sizeof(t)) {
			LOG_WRN("stored tagz size %u invalid — expected %u, "
				"keeping 0",
				(unsigned int)len, (unsigned int)sizeof(t));
			return -EINVAL;
		}
		if (read_val(read_cb, cb_arg, &t, sizeof(t))) {
			return -EINVAL;
		}
		g_survey.tag_z_m = t.z;
		return 0;
	}

	if (strcmp(key, "links") == 0) {
		uint8_t buf[APOS_LINKS_MAX_LEN + 1];
		struct apos_links tmp;

		/* One byte more than the largest legal record, so an over-long one
		 * is READ as over-long and refused by the length check rather than
		 * silently truncated to a plausible size. */
		if (len > sizeof(buf)) {
			g_links_rejected = true;
			LOG_ERR("{\"apos_store\":\"stored links record is %u bytes, "
				"larger than any this firmware writes — ignored, "
				"NOT erased\"}", (unsigned int)len);
			return 0;
		}
		if (read_val(read_cb, cb_arg, buf, len)) {
			return -EINVAL;
		}
		int rc = apos_links_decode(buf, len, &tmp);

		if (rc) {
			/* Rejected, kept on flash, and said out loud: an unknown
			 * version is the case a rollback of this firmware would hit,
			 * and wiping it would destroy the install. */
			g_links_rejected = true;
			LOG_ERR("{\"apos_store\":\"stored links record unreadable "
				"(%d) — links treated as ABSENT (needs_survey), the "
				"record is left on flash\"}", rc);
			return 0;
		}
		g_links = tmp;
		g_links_rejected = false;
		return 0;
	}

	if (strcmp(key, "addrmap") == 0) {
		uint8_t buf[APOS_ADDRMAP_MAX_LEN + 1];
		struct apos_addrmap tmp;

		if (len > sizeof(buf)) {
			g_addrmap_rejected = true;
			LOG_ERR("{\"apos_store\":\"stored address map is %u bytes, "
				"larger than any this firmware writes — ignored, "
				"NOT erased\"}", (unsigned int)len);
			return 0;
		}
		if (read_val(read_cb, cb_arg, buf, len)) {
			return -EINVAL;
		}
		int rc = apos_addrmap_decode(buf, len, &tmp);

		if (rc) {
			g_addrmap_rejected = true;
			LOG_ERR("{\"apos_store\":\"stored address map unreadable "
				"(%d) — `apos assign` refuses until `apos addr clear`, "
				"the record is left on flash\"}", rc);
			return 0;
		}
		g_addrmap = tmp;
		g_addrmap_rejected = false;
		return 0;
	}

	if (strcmp(key, "pending") == 0) {
		uint8_t buf[APOS_PENDING_LEN + 1];
		struct apos_pending tmp;

		if (len > sizeof(buf)) {
			g_pending_rejected = true;
			return 0;
		}
		if (read_val(read_cb, cb_arg, buf, len)) {
			return -EINVAL;
		}
		if (apos_pending_decode(buf, len, &tmp) != 0) {
			/* An unreadable marker is treated as PENDING: the safe reading
			 * of "I cannot tell whether an apply was interrupted" is that
			 * it was. */
			g_pending_rejected = true;
			LOG_ERR("{\"apos_store\":\"stored apply marker unreadable — "
				"treated as an INTERRUPTED apply\"}");
			return 0;
		}
		g_pending = tmp;
		g_pending_rejected = false;
		return 0;
	}

	/* An unrecognised key is a field from a newer firmware; ignore it. */
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(apos, "apos", NULL, apos_settings_set, NULL,
			       NULL);

void apos_store_init(void)
{
	/* No settings_subsys_init()/settings_load() here: uwb_store_init()
	 * already does both from main(), and settings_load() runs EVERY
	 * registered handler including this module's. Calling it again would
	 * re-read the whole subtree for no gain. This function exists so the
	 * ordering requirement is explicit at the call site rather than
	 * implicit in a static initialiser. */
	if (g_survey.valid) {
		LOG_INF("{\"apos_store\":{\"nodes\":%u,\"ref_valid\":%u}}",
			g_survey.n_nodes, g_survey.ref_valid ? 1u : 0u);
	} else {
		LOG_INF("{\"apos_store\":\"no survey — anchors payload will "
			"use the stub\"}");
	}
}

const struct apos_survey *apos_store_get(void)
{
	return &g_survey;
}

int apos_store_save(const struct apos_survey *s)
{
	if (!s || s->n_nodes > APOS_MAX_NODES) {
		return -EINVAL;
	}

	struct stored_survey rec;

	memset(&rec, 0, sizeof(rec));
	memcpy(rec.node, s->node, sizeof(rec.node));
	rec.n_nodes = s->n_nodes;
	rec.valid = s->valid ? 1u : 0u;
	rec.dim = (uint8_t)s->dim;

	int ret = settings_save_one(KEY_SURVEY, &rec, sizeof(rec));

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", KEY_SURVEY, ret);
		return ret;
	}

	/* Cache updated only after the write succeeded, so a failed save leaves
	 * apos_store_get() reporting what is actually on flash. */
	memcpy(g_survey.node, s->node, sizeof(g_survey.node));
	g_survey.n_nodes = s->n_nodes;
	g_survey.valid = s->valid;
	g_survey.dim = s->dim;
	g_survey_changes++;

	return 0;
}

uint32_t apos_store_survey_changes(void)
{
	return g_survey_changes;
}

int apos_store_set_ref(double lat, double lon)
{
	struct stored_ref rec = {.lat = lat, .lon = lon, .valid = 1u};

	int ret = settings_save_one(KEY_REF, &rec, sizeof(rec));

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", KEY_REF, ret);
		return ret;
	}

	g_survey.ref_lat = lat;
	g_survey.ref_lon = lon;
	g_survey.ref_valid = true;
	g_survey_changes++;
	return 0;
}

int apos_store_set_tag_z(float z)
{
	struct stored_tagz rec = {.z = z};

	int ret = settings_save_one(KEY_TAGZ, &rec, sizeof(rec));

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", KEY_TAGZ, ret);
		return ret;
	}

	/* Cache updated only after the write succeeded, same discipline as
	 * apos_store_save() -- a failed write must leave apos_store_get()
	 * reporting what is actually on flash. */
	g_survey.tag_z_m = z;
	return 0;
}

int apos_store_clear(void)
{
	struct stored_survey rec;

	memset(&rec, 0, sizeof(rec));

	int ret = settings_save_one(KEY_SURVEY, &rec, sizeof(rec));

	if (ret) {
		LOG_ERR("failed to clear %s (%d)", KEY_SURVEY, ret);
		return ret;
	}

	memset(g_survey.node, 0, sizeof(g_survey.node));
	g_survey.n_nodes = 0;
	g_survey.valid = false;
	g_survey_changes++;
	return 0;
}

const struct apos_links *apos_store_links(void)
{
	return &g_links;
}

bool apos_store_links_rejected(void)
{
	return g_links_rejected;
}

int apos_store_save_links(const struct apos_links *l)
{
	uint8_t buf[APOS_LINKS_MAX_LEN];
	int n = apos_links_encode(l, buf, sizeof(buf));

	if (n < 0) {
		return n;
	}

	int ret = settings_save_one(KEY_LINKS, buf, (size_t)n);

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", KEY_LINKS, ret);
		return ret;
	}
	/* Cache only after the write succeeded, same discipline as the survey. */
	g_links = *l;
	g_links_rejected = false;
	return 0;
}

bool apos_store_apply_pending(void)
{
	return g_pending_rejected || g_pending.state == APOS_PENDING_APPLYING;
}

uint32_t apos_store_pending_generation(void)
{
	return g_pending.generation;
}

int apos_store_set_pending(uint32_t generation)
{
	struct apos_pending p = {.generation = generation,
				 .state = APOS_PENDING_APPLYING};
	uint8_t buf[APOS_PENDING_LEN];
	int n = apos_pending_encode(&p, buf, sizeof(buf));

	if (n < 0) {
		return n;
	}

	int ret = settings_save_one(KEY_PENDING, buf, (size_t)n);

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", KEY_PENDING, ret);
		return ret;
	}
	g_pending = p;
	g_pending_rejected = false;
	return 0;
}

int apos_store_clear_pending(void)
{
	struct apos_pending p = {.generation = g_pending.generation,
				 .state = APOS_PENDING_NONE};
	uint8_t buf[APOS_PENDING_LEN];
	int n = apos_pending_encode(&p, buf, sizeof(buf));

	if (n < 0) {
		return n;
	}

	int ret = settings_save_one(KEY_PENDING, buf, (size_t)n);

	if (ret) {
		LOG_ERR("failed to clear %s (%d)", KEY_PENDING, ret);
		return ret;
	}
	g_pending = p;
	g_pending_rejected = false;
	return 0;
}

const struct apos_addrmap *apos_store_addrmap(void)
{
	return &g_addrmap;
}

bool apos_store_addrmap_rejected(void)
{
	return g_addrmap_rejected;
}

int apos_store_save_addrmap(const struct apos_addrmap *m)
{
	uint8_t buf[APOS_ADDRMAP_MAX_LEN];
	int n = apos_addrmap_encode(m, buf, sizeof(buf));

	if (n < 0) {
		return n;
	}

	int ret = settings_save_one(KEY_ADDRMAP, buf, (size_t)n);

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", KEY_ADDRMAP, ret);
		return ret;
	}
	g_addrmap = *m;
	g_addrmap_rejected = false;
	return 0;
}
