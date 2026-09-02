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

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(apos_store, LOG_LEVEL_INF);

#define KEY_SURVEY "apos/survey"
#define KEY_REF    "apos/ref"
#define KEY_TAGZ   "apos/tagz"

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

	return 0;
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
	return 0;
}
