/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The gateway's record of the last completed anchor survey, persisted so the
 * anchors MQTT payload survives a reboot and so a replaced board can be
 * recognised as new.
 *
 * Keyed by EUI-64, not by anchor_id or short address. That is the point: an
 * `anchor id` swap once left a coordinate stranded on the wrong board because
 * the deployment's state was keyed by id. EUI-64 travels with the board.
 *
 * The header is deliberately pure C with no Zephyr includes: pos_json.c consumes
 * struct apos_survey and is host-tested. Only apos_store.c touches settings.
 */

#ifndef APOS_STORE_H
#define APOS_STORE_H

#include "apos_geom.h"    /* APOS_MAX_NODES */
#include "apos_table.h"   /* APOS_EUI_LEN   */

#include <stdbool.h>
#include <stdint.h>

struct apos_survey_node {
	uint8_t  eui[APOS_EUI_LEN];
	uint16_t short_addr;
	float    x, y, z;
};

struct apos_survey {
	struct apos_survey_node node[APOS_MAX_NODES];
	uint8_t  n_nodes;
	bool     valid;      /* false until a survey has been applied */
	enum apos_geom_dim dim; /* whether this survey solved z, or fixed it
				 * at 0 -- see apos_geom.h */

	/* Geographic anchor for the local frame, from `apos ref`. The platform
	 * needs one real lat/long to place the survey on a map; every other
	 * anchor is positioned relative to it in metres. Belongs to the origin
	 * node, which is node[0] by construction (see apos_store_save). */
	double   ref_lat, ref_lon;
	bool     ref_valid;
};

/* Load the persisted survey into the module cache. Call from main() before the
 * uplink thread starts -- net_uplink publishes the anchors payload on connect,
 * which reads this. */
void apos_store_init(void);

/* The cached survey. Never NULL; check ->valid. */
const struct apos_survey *apos_store_get(void);

/* Replace the cached survey and persist it. node[0] MUST be the gauge origin:
 * the geographic reference belongs to it, and pos_json relies on that ordering.
 *
 * ref_lat/ref_lon/ref_valid in *s are ignored -- the reference is set
 * independently by apos_store_set_ref() and survives a re-survey, because
 * re-measuring the geometry does not move the building. Returns 0 or a negative
 * errno from the settings layer. */
int apos_store_save(const struct apos_survey *s);

/* Set and persist the geographic reference. Independent of the survey so it can
 * be entered once for a site and then left alone. */
int apos_store_set_ref(double lat, double lon);

/* Invalidate and erase. `apos_store_get()->valid` becomes false, so the anchors
 * payload falls back to the stub. Returns 0 or a negative errno. */
int apos_store_clear(void);

#endif /* APOS_STORE_H */
