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
#include "uwb_config.h" /* UWB_ANCHOR_ADDR_BASE, UWB_MAX_ANCHORS */

#include <stdbool.h>
#include <stdint.h>

#define APOS_EUI_LEN 8

/* Every ordered pair, since both directions of each pair are measured
 * separately: N*(N-1) = 992 at APOS_MAX_NODES (32).
 *
 * This is 992 rows of 12 bytes, so a struct apos_table is ~12.7 kB. It must
 * live in static storage, never on the gateway loop's 4 kB stack -- apos_gw.c
 * holds the one working table at file scope, and the host tests are on a host
 * stack. */
#define APOS_MAX_MEAS (APOS_MAX_NODES * (APOS_MAX_NODES - 1))

/* Width of struct apos_peer.heard_ids, one bit per ANCHOR ID. Ids are
 * 0..UWB_MAX_ANCHORS-1 and the bitmap is a uint32_t, so the two must not
 * diverge silently. */
#define APOS_HEARD_BITS 32u
_Static_assert(UWB_MAX_ANCHORS <= (int)APOS_HEARD_BITS,
	       "UWB_MAX_ANCHORS > 32 needs struct apos_peer.heard_ids and the "
	       "ENUM_RSP neighbour field in apos_frame.c widened first");

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

	/* Which peers this anchor reported HEARING during enumeration, as a
	 * bitmap of anchor ids (bit k = short address UWB_ANCHOR_ADDR_BASE + k).
	 *
	 * By anchor id, not node index: node indices are a gateway-side artefact
	 * of the order anchors happened to answer in, and the anchor filling
	 * this in has never seen them. Unioned across enumeration rounds by
	 * apos_table_add_peer(), because each round is an independent stagger
	 * draw and hears a different subset. Zero means "this board reported
	 * hearing nobody", which apos_table_is_candidate() treats as no evidence
	 * rather than as evidence of isolation. */
	uint32_t heard_ids;
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
 * heard_ids is UNIONED into whatever the peer already had, not assigned: each
 * enumeration round is an independent stagger draw in which a board hears a
 * different subset of its peers, so the union across rounds is the adjacency
 * evidence and any single round's bitmap is only part of it.
 *
 * Returns -EADDRINUSE if short_addr is already claimed by a DIFFERENT EUI --
 * two boards configured with the same `anchor id`. That is a fault which today
 * produces silently wrong ranging with no indication anywhere, so it is
 * reported rather than tolerated. Returns -ENOSPC when the table is full. */
int apos_table_add_peer(struct apos_table *t, const uint8_t eui[APOS_EUI_LEN],
			uint16_t short_addr, bool pos_valid,
			float x, float y, float z, uint32_t heard_ids);

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

/* ---- Candidate pairs ---- */

/* Whether the pair (i, j) is worth commanding a range for.
 *
 * A full mesh is N*(N-1) = 992 ordered pairs at 32 anchors, and a commanded
 * pair costs a RANGE_CMD, a ~200 ms ranging batch and a RANGE_RSP -- so a full
 * 32-anchor mesh is many minutes of survey, most of it spent on pairs 100 m
 * apart that were never going to range. This is the filter that removes them.
 *
 * The evidence is what `apos enum` OBSERVED, nothing else: each anchor reports
 * the peers whose ENUM_RSP frames it actually received (struct
 * apos_peer.heard_ids), and a pair is a candidate if EITHER endpoint heard the
 * other. Either, not both, because the enumeration stagger gives one direction
 * per round -- a board is asleep through its own slot delay and so can only
 * hear peers whose slot came later (apos_node.c's handle_survey_begin) -- and
 * because a link that works one way is worth trying both ways.
 *
 * Two deliberate conservative cases, both of which keep the filter from
 * dropping a pair that could have ranged:
 *
 *   - A peer whose heard_ids is ZERO reported hearing nobody at all. That is
 *     absence of evidence, not evidence of isolation (a board that drew the
 *     last stagger slot in every round hears nothing by construction, and one
 *     running firmware without the neighbour field reports nothing either), so
 *     every one of its pairs stays a candidate. Without this a board with a
 *     deaf receiver would be silently dropped from the survey it is the subject
 *     of.
 *   - i == j and out-of-range indices are refused rather than defaulted true,
 *     so a caller cannot walk itself into commanding a self-range.
 *
 * What this does NOT promise is that enumeration observed every real link.
 * A pair is missed when the later-slot board's reply collided in every round;
 * see apos_node.h for the arithmetic. That costs one edge, is counted in the
 * apos_edges log line, and is exactly what apos_geom_rigidity() exists to catch
 * downstream -- it is not silent.
 *
 * Symmetric by construction: is_candidate(i, j) == is_candidate(j, i). */
bool apos_table_is_candidate(const struct apos_table *t, uint8_t i, uint8_t j);

/* ORDERED candidate pairs -- both directions of each, since both are measured.
 * Exactly twice the unordered count, because apos_table_is_candidate() is
 * symmetric, so a caller wanting unordered can halve it. */
uint16_t apos_table_candidate_pairs(const struct apos_table *t);

/* CANDIDATE unordered pairs with no usable measurement in either direction.
 * These are the holes the fit works around, and the number an operator needs to
 * see before deciding whether to move an anchor.
 *
 * Non-candidate pairs are excluded: they were never commanded, so counting them
 * here would bury the handful of pairs that were asked to range and could not
 * under the hundreds that were never asked. The gateway reports the excluded
 * count separately. */
uint16_t apos_table_missing_pairs(const struct apos_table *t, uint8_t min_n_ok);

#endif /* APOS_TABLE_H */
