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

	/*
	 * The TAG plane's z, in this survey's own coordinate frame, metres.
	 * From `apos tagz`. Consumed by src/tdoa_gw.c as
	 * dz_i = node[i].z - tag_z_m, which is struct pos_meas's documented
	 * convention (anchor z minus TAG z) with the tag's z no longer assumed
	 * to be zero.
	 *
	 * WHY IT MATTERS, because the intuitive answer is wrong: with every
	 * anchor at the same height this looks like a constant that cancels in
	 * a range DIFFERENCE. It does not. The model is
	 *
	 *     r_i     = sqrt(rho_i^2 + dz^2)          (rho = horizontal range)
	 *     r_i-r_0 = sqrt(rho_i^2 + dz^2) - sqrt(rho_0^2 + dz^2)
	 *
	 * which is NOT rho_i - rho_0. The square root is nonlinear, so a
	 * uniform dz COMPRESSES the range differences, and a model that assumes
	 * dz = 0 must move the estimate to where the horizontal differences are
	 * smaller -- i.e. INWARD, toward the anchor centroid. A real geometric
	 * bias, not an offset.
	 *
	 * MEASURED 2026-09-03 (tests/tdoa_solve/test_height_model), worst point
	 * on the mid-line:
	 *
	 *     10.0 m array, H = 1.5 m  ->  0.098 m  (offset 4.000 -> 3.902)
	 *      2.5 m array, H = 1.4 m  ->  0.230 m  (offset 1.000 -> 0.770)
	 *      2.5 m array, H = 0.0 m  ->  0.002 m  (the default is a no-op)
	 *
	 * The middle row is this project's array (1.2-2.5 m edges): an
	 * unmodelled 1.4 m separation shrinks the reported offset from centre
	 * by ~23%, the same order as the whole ~45 cm accuracy target. On a
	 * 10 m array the identical H is a 2.5% effect, which is how this stays
	 * easy to dismiss on paper.
	 *
	 * SIGN: this is a z coordinate, not a height, so it is negative when
	 * the tags are BELOW the anchors -- which is the usual case, and in a
	 * 2D survey (APOS_GEOM_2D pins every anchor at z = 0) it is the only
	 * case. Ceiling anchors with tags on people is roughly -1.4.
	 * Expressing it as a coordinate rather than a "height below" is what
	 * makes it correct whether or not `apos zoff` has moved z = 0 to the
	 * floor.
	 *
	 * DEFAULT 0.0, which reproduces the previous behaviour exactly. It is
	 * deliberately NOT seeded with a plausible ceiling figure: a wrong
	 * value here is a NEW bias, and a guessed default would silently
	 * change the numbers every existing deployment reports. Someone has to
	 * measure the site.
	 *
	 * Persisted under its own settings key, so `apos apply` and
	 * `apos_store_clear()` leave it alone -- same treatment as ref_lat /
	 * ref_lon, and for the same reason: it is a fact about the SITE, not a
	 * result of the survey.
	 */
	float    tag_z_m;
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

/* Persist the tag plane's z in the survey frame -- see tag_z_m above for the
 * sign convention and for why a uniform value does not cancel out. Stored
 * under its own key, so it survives `apos apply` and apos_store_clear(). */
int apos_store_set_tag_z(float z);

/* Invalidate and erase. `apos_store_get()->valid` becomes false, so the anchors
 * payload falls back to the stub. Returns 0 or a negative errno. */
int apos_store_clear(void);

#endif /* APOS_STORE_H */
