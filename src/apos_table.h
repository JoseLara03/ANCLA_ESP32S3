/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The gateway's working set for one survey: which anchors answered enumeration,
 * every directed range measurement reported back, and the reduction of those
 * into the undirected edges apos_geom.c fits.
 *
 * Peers are keyed by EUI-64, not by anchor_id or short address. That is the
 * whole point: an `anchor id` swap once left coordinates stranded on the wrong
 * board because the deployment's state was keyed by id. EUI-64 is assigned at
 * manufacture and travels with the board.
 *
 * Pure C -- no Zephyr -- so the accumulation and symmetrisation rules are
 * host-testable.
 */

#ifndef APOS_TABLE_H
#define APOS_TABLE_H

#include "apos_geom.h"

#include <stdbool.h>
#include <stdint.h>

#define APOS_EUI_LEN 8

/* Every ordered pair, since both directions of each pair are measured
 * separately: N*(N-1) = 56 at APOS_MAX_NODES. */
#define APOS_MAX_MEAS (APOS_MAX_NODES * (APOS_MAX_NODES - 1))

/* A reported sd of 0 would give an edge infinite weight and let one pair
 * dictate the entire fit. 1 mm is far below this hardware's real resolution, so
 * the floor never binds on an honest measurement. */
#define APOS_SD_FLOOR_MM 1u

/* Applied to an edge measured in only ONE direction. Averaging A->B with B->A
 * is what cancels antenna-delay asymmetry; with a single direction that error
 * is still in the number, so the edge is trusted less rather than being
 * silently treated as equal in quality to a symmetrised one. */
#define APOS_ONEWAY_SD_INFLATE 2.0f

struct apos_peer {
	uint8_t  eui[APOS_EUI_LEN];
	uint16_t short_addr;
	bool     pos_valid; /* what the anchor reported at enumeration */
	float    x, y, z;   /* its position BEFORE this survey */
};

/* One anchor's report of ranging one peer: `from` polled `to`. */
struct apos_dir_meas {
	uint8_t  from;    /* node index of the initiator */
	uint8_t  to;      /* node index of the responder */
	int32_t  mean_mm;
	uint16_t sd_mm;
	uint8_t  n_ok;    /* exchanges that produced a distance */
};

struct apos_table {
	struct apos_peer     peer[APOS_MAX_NODES];
	uint8_t              n_peers;
	struct apos_dir_meas meas[APOS_MAX_MEAS];
	uint16_t             n_meas;
};

void apos_table_init(struct apos_table *t);

/* Register an enumerated anchor. Returns its node index (>= 0).
 *
 * Idempotent on EUI: re-registering the same EUI updates its fields and returns
 * the same index, so repeated SURVEY_BEGIN broadcasts are harmless.
 *
 * Returns -EADDRINUSE if short_addr is already claimed by a DIFFERENT EUI --
 * two boards configured with the same `anchor id`. That is a fault which today
 * produces silently wrong ranging with no indication anywhere, so it is
 * reported rather than tolerated. Returns -ENOSPC when the table is full. */
int apos_table_add_peer(struct apos_table *t, const uint8_t eui[APOS_EUI_LEN],
			uint16_t short_addr, bool pos_valid,
			float x, float y, float z);

/* Node index, or -ENOENT. */
int apos_table_find_addr(const struct apos_table *t, uint16_t short_addr);
int apos_table_find_eui(const struct apos_table *t,
			const uint8_t eui[APOS_EUI_LEN]);

/* Record one directed measurement. A repeat of the same (from, to) replaces the
 * earlier one, so a retried RANGE_CMD does not double-count.
 *
 * Returns 0, -EINVAL on a bad index pair, or -ENOSPC. */
int apos_table_add_meas(struct apos_table *t, uint8_t from, uint8_t to,
			int32_t mean_mm, uint16_t sd_mm, uint8_t n_ok);

/* Reduce the directed measurements to undirected edges in metres.
 *
 * Measurements with n_ok < min_n_ok are discarded as too thin to trust. A pair
 * with both directions surviving is combined inverse-variance weighted; a pair
 * with one direction is kept with its sd inflated by APOS_ONEWAY_SD_INFLATE; a
 * pair with none produces no edge at all.
 *
 * Returns the number of edges written, never more than out_cap. */
uint16_t apos_table_symmetrise(const struct apos_table *t,
			       struct apos_edge *out, uint16_t out_cap,
			       uint8_t min_n_ok);

/* Ranging-quality maxima over the usable directed measurements.
 *
 * *max_recip_mm receives the largest |d(A->B) - d(B->A)| across every pair that
 * was measured in BOTH directions, or -1 if no such pair exists. *max_sd_mm
 * receives the largest per-measurement sd_mm, or 0 if there is nothing usable.
 * Either output pointer may be NULL.
 *
 * These describe the RANGING, and are the only quality signals that carry
 * information on a four-anchor array, where the fit's rms_m is vacuous (see
 * apos_gw_result_unverified()). They do NOT make the geometry over-determined:
 * a rigid 4-node framework is isostatic whatever its edges measure. Reciprocal
 * disagreement is computed here rather than inside apos_table_symmetrise(),
 * whose averaging is exactly what throws the information away -- the averaging
 * itself is unchanged. */
void apos_table_quality(const struct apos_table *t, uint8_t min_n_ok,
			int32_t *max_recip_mm, uint16_t *max_sd_mm);

/* Unordered pairs with no usable measurement in either direction. These are the
 * holes the fit works around, and the number an operator needs to see before
 * deciding whether to move an anchor. */
uint16_t apos_table_missing_pairs(const struct apos_table *t, uint8_t min_n_ok);

#endif /* APOS_TABLE_H */
