/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Groups the observations different anchors make of the SAME tag BLINK.
 * A BLINK carries no anchor-side arithmetic at all -- every anchor that hears
 * one hands the gateway a timestamped struct tdoa_meas, and this module is
 * what decides which of those belong together before tdoa_solve() ever sees
 * them. Pure C, no radio and no clock of its own: `now_ms` is supplied by the
 * caller on every call, exactly like gw_core.h's frame_counter discipline.
 *
 * ---- The key is (tag_addr, blink_seq) ------------------------------------
 *
 * blink_seq wraps at 256. At 5 Hz (the fastest BLINK rate this project uses)
 * that is 51.2 s between two blinks from the same tag sharing a seq value --
 * far longer than TDOA_COLLECT_WINDOW_MS (150 ms), so a group closes or times
 * out long before its seq value could recur. No extra disambiguator is
 * needed.
 *
 * ---- Slot exhaustion: drop the OLDEST group, not the new one -------------
 *
 * TDOA_COLLECT_SLOTS (16) open groups is already generous for one BLINK
 * period, so this only fires under packet loss or backhaul jitter piling up
 * stale groups. The oldest group is furthest from completion and closest to
 * its own window timeout anyway -- it would very likely be discarded on the
 * next tdoa_collect_take_ready() regardless. Evicting it to make room for a
 * fresh blink is the same call net_uplink_submit() makes for its bounded
 * queue, and for the same reason: an old, probably-dead item is worth less
 * than telemetry that just arrived.
 *
 * ---- Release rule ---------------------------------------------------------
 *
 * A group is handed to the caller (tdoa_collect_take_ready() returns true)
 * as soon as it is COMPLETE -- POS_MAX_ANCHORS distinct anchors have all
 * reported -- without waiting out the rest of the window, since nothing more
 * can arrive for it. Short of that, it is released once the window has
 * expired PROVIDED at least TDOA_MIN_ANCHORS observations were gathered;
 * fewer than that is not resolvable in 2D (see tdoa_solve.h) and the group is
 * silently discarded, freeing its slot.
 *
 * ---- Clock discipline ------------------------------------------------------
 *
 * All comparisons against `now_ms` are signed differences,
 * (int32_t)(now_ms - first_ms), so a wrapping or otherwise non-monotonic
 * 32-bit millisecond counter cannot corrupt state -- any single interval
 * under ~24.8 days (2^31 ms) reads correctly. A caller that rewinds `now_ms`
 * by more than that is out of contract; nothing here can detect that case,
 * same as gw_core.c's frame_counter and beacon_guard.c's hi32.
 */

#ifndef TDOA_COLLECT_H
#define TDOA_COLLECT_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "tdoa_solve.h"   /* struct tdoa_meas */
#include "pos_solver.h"   /* POS_MAX_ANCHORS */

/* Concurrently open blink groups. See the slot-exhaustion note above for what
 * happens on the 17th. */
#define TDOA_COLLECT_SLOTS       16u

/* Deliberately less than one 200 ms superframe: a blink is resolved or
 * discarded before the next one is due. A late fix is worthless in an RTLS. */
#define TDOA_COLLECT_WINDOW_MS   150u

/* One anchor's report of one blink. anchor_id must be < POS_MAX_ANCHORS --
 * it both indexes the group's meas[] array and sets a bit in anchor_bits. */
struct tdoa_obs {
	uint16_t          tag_addr;
	uint8_t           blink_seq;
	uint8_t           anchor_id;
	struct tdoa_meas  meas;
};

struct tdoa_group {
	uint16_t          tag_addr;
	uint8_t           blink_seq;
	uint8_t           n;             /* observations gathered */
	uint32_t          anchor_bits;   /* anchor_id already seen, one bit each */
	uint32_t          first_ms;      /* arrival of the first observation */
	bool              used;
	struct tdoa_meas  meas[POS_MAX_ANCHORS];
};

struct tdoa_collect {
	struct tdoa_group slot[TDOA_COLLECT_SLOTS];
};

void tdoa_collect_init(struct tdoa_collect *c);

/*
 * Fold one anchor's observation into its blink's group, opening a new group
 * for a (tag_addr, blink_seq) pair not currently held.
 *
 * Returns false, and changes nothing, when the observation cannot be used:
 * anchor_id >= POS_MAX_ANCHORS, or the anchor has already reported for this
 * exact blink (a duplicate -- an MQTT redelivery or a retry -- which would
 * otherwise feed tdoa_solve() the same range difference twice and make its
 * normal matrix singular against itself). Returns true once the observation
 * is stored, whether or not the group is ready yet.
 *
 * O(TDOA_COLLECT_SLOTS) linear search, bounded and allocation-free -- safe to
 * call from the K_PRIO_COOP(0) gateway loop.
 */
bool tdoa_collect_add(struct tdoa_collect *c, const struct tdoa_obs *o,
		      uint32_t now_ms);

/*
 * Release at most one ready or expired group per call. A group is ready when
 * it holds POS_MAX_ANCHORS observations, or when its window has expired
 * (see TDOA_COLLECT_WINDOW_MS) with at least TDOA_MIN_ANCHORS gathered; on
 * release, `out` receives up to POS_MAX_ANCHORS struct tdoa_meas, `*n_out`
 * the count, and `*tag_addr_out` the tag. A group whose window has expired
 * with fewer than TDOA_MIN_ANCHORS observations is discarded silently
 * (returns nothing for it) and its slot freed.
 *
 * Every call scans the full table once, so an expired-and-discarded group is
 * freed as soon as some call reaches it, even if that same call also returns
 * a different, ready group. Returns false when nothing was ready to release
 * (though stale groups may still have been discarded during the scan).
 *
 * O(TDOA_COLLECT_SLOTS), bounded and allocation-free -- safe to call from the
 * K_PRIO_COOP(0) gateway loop once per superframe.
 */
bool tdoa_collect_take_ready(struct tdoa_collect *c, uint32_t now_ms,
			     struct tdoa_meas *out, size_t *n_out,
			     uint16_t *tag_addr_out);

#endif /* TDOA_COLLECT_H */
