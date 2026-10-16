/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Static clock-sync links: the per-anchor flight-time compensation `tau_dtu`
 * that B14 applies to the master's declared TX time, derived once from an
 * ACCEPTED survey geometry rather than measured continuously on the air.
 *
 *     tau(parent, child) = round(distance_3d(parent, child) / mm_per_dtu)
 *
 * The same conversion gw_selftau uses (SS_INITIATOR_MM_PER_DTU_X1000), so a
 * geometric tau and an on-air tau for the same pair are directly comparable
 * and cannot drift apart by a unit-conversion constant.
 *
 * A link is keyed by the EUI-64 of its two ends, never by a short address: an
 * `anchor id` swap must not move a compensation onto the wrong board, the same
 * reason apos_table.h keys peers by EUI. In this delivery every parent is the
 * gateway (the CCP root) and the tree is one level deep; the parent field and
 * the generation exist so a later multi-hop tree has somewhere to put a real
 * parent without a format change. Nothing here selects or routes a parent.
 *
 * The measured one-way range is EVIDENCE for the geometry, never a source that
 * rewrites tau at run time: the discrepancy is recorded per link and judged
 * against the heritage B14 gate (0.5 ns, ~32 DTU, ~150 mm) by the caller.
 *
 * Pure C, no Zephyr: host-tested in tests/sync_link/.
 */

#ifndef SYNC_LINK_H
#define SYNC_LINK_H

#include "apos_geom.h"
#include "apos_table.h"
#include "ss_initiator.h"   /* SS_INITIATOR_MM_PER_DTU_X1000 */

#include <stdbool.h>
#include <stdint.h>

#define SYNC_LINK_MAX APOS_MAX_NODES

/* Where a link's tau came from. 0 is "nothing", so a zero-initialised record
 * can never be mistaken for a calibrated one. */
enum sync_link_source {
	SYNC_LINK_SRC_NONE          = 0,
	SYNC_LINK_SRC_SURVEY_STATIC = 1,
};

/* The B14 Gate B heritage: tau must agree with an independent reference
 * within 0.5 ns, i.e. 32 DTU (SYNC_DTU_PER_NS = 64) = 32 * 4.69 mm. Restated
 * in millimetres because the check is on distances. */
#define SYNC_LINK_GATE_DTU 32
#define SYNC_LINK_GATE_MM \
	((SYNC_LINK_GATE_DTU * SS_INITIATOR_MM_PER_DTU_X1000 + 500) / 1000)

struct sync_link {
	uint8_t  child_eui[APOS_EUI_LEN];
	uint8_t  parent_eui[APOS_EUI_LEN];
	uint16_t child_addr;   /* radio address at derivation time; NOT the key */
	int32_t  tau_dtu;
	int32_t  geom_mm;      /* parent->child distance from the solved geometry */
	int32_t  meas_mm;      /* direct parent->child ranging, 0 when !checked */
	int32_t  err_mm;       /* meas_mm - geom_mm, 0 when !checked */
	uint16_t sd_mm;        /* sd of that direct ranging */
	uint8_t  source;       /* enum sync_link_source */
	bool     valid;        /* tau_dtu was computed from placed geometry;
			        * explicit, because tau == 0 is a legal value */
	bool     checked;      /* a direct measurement existed to compare with */
	bool     within_gate;  /* checked && |err_mm| <= SYNC_LINK_GATE_MM */
};

/* mm -> DTU, one integer round-half-up, never a float. Identical to
 * gw_selftau_mm_to_dtu(); kept here so this pure module does not need the
 * gateway's header. A negative distance is not physical and returns 0. */
static inline int32_t sync_link_mm_to_dtu(int32_t mm)
{
	if (mm < 0) {
		return 0;
	}
	return (int32_t)(((int64_t)mm * 1000 +
			  (int64_t)(SS_INITIATOR_MM_PER_DTU_X1000 / 2)) /
			 SS_INITIATOR_MM_PER_DTU_X1000);
}

/* Derive one link per peer other than `root_idx` (the CCP root, i.e. the
 * gateway) from a SOLVED result. Returns the number of links written (never
 * more than cap), or a negative errno: -EINVAL on a NULL argument, a root index
 * out of range, or a result that does not describe the table's peers.
 *
 * A peer the solve left unplaced yields a link with valid == false rather than
 * being skipped, so the caller can count it and refuse. Links are written in
 * peer order, which is arbitrary; consumers must look them up by EUI. */
int sync_link_derive(const struct apos_table *t, const struct apos_result *r,
		     uint8_t root_idx, uint8_t min_n_ok, struct sync_link *out,
		     uint8_t cap);

/* How many of n links are valid, checked and within the gate. */
uint8_t sync_link_count_good(const struct sync_link *l, uint8_t n);

#endif /* SYNC_LINK_H */
