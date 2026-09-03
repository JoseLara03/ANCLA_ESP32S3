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
 * ---- Slot exhaustion: drop the LEAST COMPLETE group, not the oldest -------
 *
 * TDOA_COLLECT_SLOTS (16) open groups is already generous for one BLINK
 * period, so this only fires under packet loss or backhaul jitter piling up
 * stale groups. An earlier revision of this module evicted the OLDEST group
 * outright, on the theory that it was "furthest from completion and closest
 * to its own timeout anyway" -- that reasoning is backwards. The oldest
 * group has had the MOST time to fill, so it is the group MOST likely to
 * already be complete, not least. Evicting strictly by age can therefore
 * destroy an already-resolvable fix -- one sitting complete, merely waiting
 * for the next tdoa_collect_take_ready() drain -- to make room for a blink
 * that has not gathered a single observation yet and may never complete.
 * That is a materially worse loss than dropping a still-forming group, and
 * with take_ready() drained only once per superframe (200 ms) against a
 * 150 ms window, a complete group can genuinely sit waiting when the 17th
 * distinct blink arrives.
 *
 * So the victim is chosen by COMPLETENESS first, age second: the group with
 * the fewest observations gathered, ties broken by oldest first_ms. Any
 * group already RELEASABLE (n >= TDOA_MIN_ANCHORS -- a fix the gateway can
 * actually produce) is never evicted while a less-complete group exists to
 * take its place instead. If every slot already holds a releasable group,
 * the new observation is REJECTED rather than destroying one of them: all
 * TDOA_COLLECT_SLOTS pending fixes are about to be drained by the next
 * take_ready() call regardless, so refusing one more inbound observation
 * costs less than discarding a fix that is already resolvable.
 *
 * The net_uplink_submit() precedent for "evict to make room" is real but
 * does not transfer whole: that queue only ever holds ALREADY-COMPLETED
 * fixes, so its worst case is dropping one finished reading. This module's
 * groups span the full range from zero observations to fully resolvable, so
 * an eviction policy here has to protect completeness explicitly rather than
 * relying on age as a proxy for it.
 *
 * ---- The reference anchor is meas[0], and it must be DETERMINISTIC -------
 *
 * Everything downstream treats meas[0] as the reference: tdoa_dtu_rebase()
 * differences against it, tdoa_solve() linearises against it, and
 * pos_ekf_update_tdoa() writes its n-1 scalar equations against it. Since
 * 2026-09-02 tdoa_gw.c ALSO takes the filter's dt from meas[0]'s absolute
 * t_dtu.
 *
 * A first revision appended observations in arrival order, so meas[0] was
 * whichever anchor's MQTT message happened to land first -- different between
 * consecutive blinks from the same tag, for reasons that have nothing to do
 * with where the tag is. That makes the linearisation reference wander, so the
 * conditioning of the solve changes fix to fix with no physical cause: jitter
 * manufactured by bookkeeping. It also means dt is measured against a
 * different anchor each time (harmless in magnitude -- the array's propagation
 * spread is ~8 ns against a 200 ms dt -- but pointless).
 *
 * So this module keeps meas[] sorted by anchor_id ascending, making the
 * reference the LOWEST anchor_id present in the group. Deterministic given the
 * set of anchors that reported, which is the property downstream needs.
 *
 * Note the honest limit: it is deterministic given the SET, not invariant. If
 * anchor 0 misses a blink, that blink's reference is anchor 1. That is
 * unavoidable without inventing a reference the group does not contain, and
 * both the solve and dt tolerate it.
 *
 * Alternatives rejected: picking the best-`quality` anchor flips between fixes
 * whenever two are close, reintroducing exactly the instability this removes;
 * picking the anchor nearest the previous fix conditions the solve better but
 * makes the geometry depend on the filter's own previous output, which is one
 * more closed loop.
 *
 * ---- Release rule ---------------------------------------------------------
 *
 * A group is handed to the caller (tdoa_collect_take_ready() returns true)
 * as soon as it is COMPLETE -- `expected` distinct anchors have all reported
 * (see tdoa_collect_set_expected(); POS_MAX_ANCHORS until that is called) --
 * without waiting out the rest of the window, since nothing more can arrive
 * for it. Short of that, it is released once the window has expired PROVIDED
 * at least TDOA_MIN_ANCHORS observations were gathered; fewer than that is
 * not resolvable in 2D (see tdoa_solve.h) and the group is silently
 * discarded, freeing its slot.
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
	/* meas[] is kept sorted by anchor_id ASCENDING, so meas[0] is always the
	 * lowest anchor_id present -- see the reference-anchor note above.
	 * aid[i] is the anchor_id that produced meas[i]; it exists ONLY to
	 * maintain that order, and is deliberately not exposed through
	 * tdoa_collect_take_ready(). struct tdoa_meas has no anchor_id field and
	 * must not grow one: nothing downstream of the collector has any use for
	 * it, and the solver indexes its own equations. */
	uint8_t           aid[POS_MAX_ANCHORS];
	struct tdoa_meas  meas[POS_MAX_ANCHORS];
};

struct tdoa_collect {
	struct tdoa_group slot[TDOA_COLLECT_SLOTS];
	/* Anchors expected to report per blink, i.e. the release threshold used
	 * in place of a hardcoded POS_MAX_ANCHORS. See
	 * tdoa_collect_set_expected() below. */
	uint8_t           expected;
};

void tdoa_collect_init(struct tdoa_collect *c);

/*
 * Set how many anchors are expected to report each blink -- a DEPLOYMENT
 * fact (how many anchors the survey actually placed), not a compile-time one.
 * Without this call, tdoa_collect_take_ready() only releases a group early at
 * POS_MAX_ANCHORS observations, which is correct only when every possible
 * anchor slot is populated. On a deployment with fewer anchors than
 * POS_MAX_ANCHORS, that early-release condition is never reached and every
 * group pays the full TDOA_COLLECT_WINDOW_MS wait.
 *
 * `n` is clamped to [TDOA_MIN_ANCHORS, POS_MAX_ANCHORS]; a value outside that
 * range is pulled to the nearest bound rather than rejected, since a caller
 * has no better fallback to offer for an out-of-range anchor count. Calling
 * this does not affect any group already open -- only take_ready()'s
 * early-release check on subsequent calls.
 *
 * tdoa_collect_init() sets `expected` to POS_MAX_ANCHORS, so a caller that
 * never invokes this setter keeps exactly today's behaviour.
 */
void tdoa_collect_set_expected(struct tdoa_collect *c, uint8_t n);

/*
 * Fold one anchor's observation into its blink's group, opening a new group
 * for a (tag_addr, blink_seq) pair not currently held.
 *
 * Returns false, and changes nothing, when the observation cannot be used:
 * anchor_id >= POS_MAX_ANCHORS, the anchor has already reported for this
 * exact blink (a duplicate -- an MQTT redelivery or a retry -- which would
 * otherwise feed tdoa_solve() the same range difference twice and make its
 * normal matrix singular against itself), or every slot is full AND already
 * holds a releasable group (n >= TDOA_MIN_ANCHORS) -- see the slot-exhaustion
 * note above for why a new, unformed blink loses that contest rather than
 * displacing a fix that already exists. Returns true once the observation is
 * stored, whether or not the group is ready yet.
 *
 * O(TDOA_COLLECT_SLOTS) linear search, bounded and allocation-free -- safe to
 * call from the K_PRIO_COOP(0) gateway loop.
 */
bool tdoa_collect_add(struct tdoa_collect *c, const struct tdoa_obs *o,
		      uint32_t now_ms);

/*
 * Release at most one ready or expired group per call. A group is ready when
 * it holds `expected` observations (see tdoa_collect_set_expected()) or when
 * its window has expired (see TDOA_COLLECT_WINDOW_MS) with at least
 * TDOA_MIN_ANCHORS gathered; on release, `out` receives up to
 * POS_MAX_ANCHORS struct tdoa_meas, `*n_out` the count, and `*tag_addr_out`
 * the tag. A group whose window has expired
 * with fewer than TDOA_MIN_ANCHORS observations is discarded silently
 * (returns nothing for it) and its slot freed.
 *
 * When SEVERAL groups are releasable at once -- routine, since the gateway
 * drains up to TDOA_GW_SOLVE_MAX groups per superframe -- the OLDEST by
 * `first_ms` is released first. That ordering is part of this function's
 * contract, not an implementation detail: the caller derives its filter's
 * `dt` from consecutive groups' reference timestamps, so releasing them out
 * of order feeds a constant-velocity filter negative and double-counted time
 * steps. An earlier revision released the first releasable group the table
 * scan reached, which is blink_seq/slot-allocation order and unrelated to
 * time; measured on hardware 2026-09-03 it drained three groups in one
 * gateway cycle with dt of 0.400, 0.200 and 0.600 s (1.2 s of prediction for
 * 200 ms of elapsed time), and the filtered fixes came out NOISIER than the
 * unfiltered solve (dispersion RMS 0.512 m against 0.345 m).
 *
 * `first_ms` is gateway ARRIVAL, not tag emission, and is the only key
 * available here -- the reference timestamp lives in `meas[].t_dtu`, is
 * comparable only within one tag, and this function is tag-agnostic by
 * design. Interleaved MQTT arrivals can therefore still invert two adjacent
 * blinks; the caller keeps its own guard for that residue. Strictly better
 * than table order, not a total order.
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
