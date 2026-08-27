/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ccp_slave.h"

#include "ccp_frame.h"
#include "uwb_debug.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ccp_slave, ANCLA_LOG_LEVEL);

/* static, not automatic. 72 bytes would fit anywhere, but CLAUDE.md records a
 * 2588-byte automatic silently overflowing the 4096-byte main stack, and the
 * rule that came out of it does not take exceptions for size. */
static struct sync_model model;

static uint32_t cur_root;

/* The last CCP RECEIVED (not yet necessarily paired into an observation):
 * its sequence number and the local RX timestamp it arrived at. Every
 * received, parseable CCP updates this, unconditionally -- it is what lets
 * the NEXT CCP's announced timestamp (which always describes THIS one, by
 * construction: a CCP with sequence S announces sequence S-1's timestamp) be
 * matched back to a local RX time. */
static uint8_t  prev_seq;
static uint64_t prev_rx_ts;
static bool     have_prev;

/* The sequence number of the last frame actually FED to sync_model as an
 * observation (not merely received) -- i.e. the frame whose PAIR (its own RX
 * timestamp, and the NEXT CCP's announcement of its TX timestamp) has both
 * arrived and been consumed. Used to size the miss run between two
 * observations, which is now the gap between PAIRED frames, not between
 * received ones -- a received-but-unpaired CCP (e.g. one immediately
 * following a lost frame, or the sentinel/txtest cases below) does not by
 * itself advance this. */
static uint8_t  last_obs_seq;
static uint64_t last_obs_rx_ts;
static bool     have_obs;

static uint32_t n_rx;
static uint32_t n_paired;
static uint32_t n_gap;
static uint32_t n_reject;
static uint32_t n_no_announce;

/* A genuine forward run of `intervals` observations should show local elapsed
 * time within a small fraction of intervals * SYNC_CCP_INTERVAL_DTU -- the
 * only thing that can move it off that mark is the two crystals' combined
 * drift, which CLAUDE.md's own cross-check bounds at roughly +/-40000 ppb,
 * i.e. under 0.005% even over the largest run this path handles (128
 * intervals). A factor of 4 either side is therefore not a tight statistical
 * bound -- it has three-plus orders of magnitude of margin over anything a
 * real clock pair can produce, so it cannot fire on ordinary jitter or on a
 * genuinely long outage (a real outage of N superframes advances both
 * `intervals` and the local clock by the SAME N, so the ratio stays near 1
 * regardless of how large N is). What it DOES catch is a gateway reboot: the
 * local elapsed time since the last accepted CCP is however long the reboot
 * actually took, which has no relationship at all to what the wrapped
 * sequence-number arithmetic computes for `intervals`, so the two landing
 * within a factor of 4 of each other by coincidence is not realistic --
 * including the case a plain intervals>128 check cannot see at all:
 * intervals == 1 (last_obs_seq in 229..255, wrapping to a low post-reboot
 * seq) with several REAL seconds elapsed locally, which is a discontinuity by
 * construction and not merely a slow superframe. */
#define CCP_SLAVE_GAP_TOL_FACTOR 4u

/* Known residual limitation, disclosed rather than hidden: local_delta is
 * derived from two 40-bit hardware timestamps, so it is only ever
 * TRUE_ELAPSED mod 2^40 (~17.18 s) -- there is no way to recover an elapsed
 * span longer than that from two raw DW3220 stamps alone. For a GENUINE
 * forward run whose true elapsed time straddles a multiple of that wrap
 * (roughly intervals in 86..114 at this superframe period, i.e.
 * 17.2..22.8 s of real, consecutive CCP loss with no reboot), the wrapped
 * local_delta can legitimately fall outside this factor-of-4 band, so this
 * branch can mislabel that one case as a discontinuity (n_reject, logged as
 * a probable reboot) instead of a gap (n_gap). That costs only the
 * DIAGNOSTIC label: this whole branch already calls sync_model_init() for
 * both a genuine large gap and a reboot (see the `else if` below), so the
 * sync estimate itself is identical either way -- nothing is poisoned, and
 * nothing here changes what the model does, only what an operator reads
 * about why. */

/* Signed difference between two 40-bit DW3220 timestamps. Pattern-matched
 * from sync_model.c's sdelta40() (that file must not be modified, so this is
 * a deliberate local copy, not a shared helper): the counter wraps every
 * 2^40 DTU (~17.2 s), so an unsigned compare is wrong across the wrap and the
 * failure is rare, timing-dependent, and looks like a radio fault. Correct for
 * any interval under 2^39 DTU (~8.6 s) -- far past the largest run (128
 * intervals = 25.6 s) this function differences in one step, but that ceiling
 * only matters for a genuine multi-second reboot gap landing exactly on a
 * multiple of ~8.6 s, which the factor-of-4 tolerance above does not depend on
 * being exact for: a wrapped-and-therefore-wrong delta is just as far from
 * `expected` as the true one would have been. */
static int64_t ccp_slave_sdelta40(uint64_t a, uint64_t b)
{
	uint64_t d = (a - b) & SYNC_DTU_MASK;

	if (d & (1ULL << (SYNC_DTU_BITS - 1))) {
		return (int64_t)d - (int64_t)(1ULL << SYNC_DTU_BITS);
	}
	return (int64_t)d;
}

/* Save the most recently RECEIVED frame's identity, unconditionally. Every
 * exit path from ccp_slave_on_rx() that accepted a parseable CCP calls this
 * last, which is what lets the NEXT frame's announcement -- describing THIS
 * one -- be matched back to a local RX time regardless of whether THIS frame
 * itself got paired into an observation. */
static void save_prev(uint8_t seq, uint64_t rx_ts)
{
	prev_seq = seq;
	prev_rx_ts = rx_ts;
	have_prev = true;
}

void ccp_slave_init(void)
{
	sync_model_init(&model);
	cur_root = 0u;
	prev_seq = 0u;
	prev_rx_ts = 0u;
	have_prev = false;
	last_obs_seq = 0u;
	last_obs_rx_ts = 0u;
	have_obs = false;
	n_rx = 0u;
	n_paired = 0u;
	n_gap = 0u;
	n_reject = 0u;
	n_no_announce = 0u;
}

bool ccp_slave_on_rx(const uint8_t *buf, uint16_t plen, uint64_t rx_ts)
{
	struct ccp_frame f;

	if (!ccp_frame_is_ccp(buf, plen)) {
		return false;
	}

	/* ccp_frame_parse() also rejects a hop this node must not adopt, so the
	 * CCP_HOP_MAX check cannot be forgotten here. That said, f.hop is never
	 * read below once parsing accepts it -- harmless in Phase 2, where
	 * there is exactly one hop-0 transmitter on the air and nothing else
	 * this node could adopt. It stops being harmless the day a Phase 3
	 * hop-1 repeater exists: a repeater relaying the root's root_id while
	 * stamping its OWN tx_dtu would be indistinguishable here from the
	 * root itself, get folded into the same model as the root's direct
	 * CCPs, and the root-change branch below -- which keys only on
	 * root_id -- would have no way to see the substitution. */
	if (ccp_frame_parse(buf, plen, &f) != 0) {
		n_reject++;
		return true;
	}

	uint64_t this_rx_ts = rx_ts & SYNC_DTU_MASK;

	n_rx++;

	if (!have_prev || f.root_id != cur_root) {
		/* A different root is a different time base, and the baseline
		 * built against the old one describes nothing. Start over
		 * rather than mixing two clocks into one rate estimate. This
		 * frame becomes the new `prev` with no observation yet -- its
		 * OWN announced tx_dtu describes some earlier frame under
		 * whatever the OLD root was (or nothing, if it is the sentinel),
		 * and either way it is not usable now. */
		if (have_prev && f.root_id != cur_root) {
			LOG_WRN("{\"ccp_slave\":{\"root_changed\":%u,"
				"\"was\":%u}}", f.root_id, cur_root);
		}
		sync_model_init(&model);
		cur_root = f.root_id;
		have_obs = false;
		save_prev(f.seq, this_rx_ts);
		return true;
	}

	/* tx_dtu == 0 is the sentinel for "no announcement": either the
	 * master's first CCP after its own boot (it has no previous confirmed
	 * transmit to report), or a "sync txtest" probe (seq 0xFF, which never
	 * carries an announcement either -- see ccp_master.c). Nothing to pair
	 * this frame's PREDECESSOR against; just remember this frame as the
	 * new `prev` and wait for the next one. */
	if (f.tx_dtu == 0u) {
		n_no_announce++;
		save_prev(f.seq, this_rx_ts);
		return true;
	}

	if (prev_seq != (uint8_t)(f.seq - 1u)) {
		/* This frame's announcement describes sequence f.seq-1, but
		 * the frame this node actually last received was some OTHER
		 * sequence -- the announced frame was never received here
		 * (lost in the air, or this is the very first CCP after a
		 * root change/init and there is nothing to check it against),
		 * so there is no local RX timestamp to pair the announcement
		 * with. Not usable as an observation; still worth keeping as
		 * the new `prev` in case the frame AFTER this one announces
		 * f.seq itself. */
		n_reject++;
		save_prev(f.seq, this_rx_ts);
		return true;
	}

	/* Pair: f.tx_dtu is the ACTUAL TX timestamp of the frame this node
	 * received at prev_rx_ts (sequence prev_seq). That is the observation
	 * -- one superframe lagged behind the frame that carried it, by
	 * construction of the deferred-timestamp design (see ccp_master.c and
	 * ccp_frame.h). */
	if (have_obs) {
		/* intervals is the gap between this observation and the last
		 * one PAIRED, not between receptions -- a received-but-
		 * unpaired frame (sentinel, or an announcement whose target
		 * was never received) does not itself advance last_obs_seq,
		 * so the count below can span more than one CCP even though
		 * every intervening frame was individually accounted for
		 * above. */
		uint8_t intervals = (uint8_t)(prev_seq - last_obs_seq);

		if (intervals == 0u) {
			/* Same sequence paired twice -- cannot happen via the
			 * checks above (prev_seq only reaches this point once
			 * per distinct received sequence), kept only as a
			 * guard against folding one interval in as two misses
			 * below if that invariant is ever violated by a future
			 * change. */
			n_reject++;
		} else if (intervals > 128u) {
			/* A run this large is far more likely to be the
			 * sequence number moving BACKWARDS than 129+
			 * superframes of genuine forward misses. The ordinary
			 * trigger is a gateway reboot: ccp_master_init()
			 * re-seeds ccp_seq at 0 while root_id is derived from
			 * the board and does NOT change, so the root-change
			 * branch above never sees it either. Re-baselining the
			 * model is still correct here -- the old baseline
			 * really doesn't describe the new sequence -- but
			 * counting this as n_gap would tell the operator the
			 * LINK is dropping CCPs when nothing was lost at all.
			 * n_reject is what it actually is: a pairing this node
			 * could not fold into its running count. */
			n_reject++;
			sync_model_init(&model);
			have_obs = false;
		} else {
			/* intervals in 1..128. Sequence number alone cannot
			 * tell a genuine forward gap of this size from a
			 * gateway reboot that happens to wrap last_obs_seq
			 * down to a small apparent `intervals` (see the
			 * comment above). The local clock can, because it has
			 * no idea what the sequence number claims -- a genuine
			 * gap of `intervals` superframes elapses
			 * `intervals * SYNC_CCP_INTERVAL_DTU` of REAL local
			 * time (up to the crystals' negligible drift), while a
			 * reboot's elapsed local time is however long the
			 * reboot took, with no relationship to `intervals` at
			 * all. See CCP_SLAVE_GAP_TOL_FACTOR for why that
			 * comparison is safe at a generous tolerance. */
			int64_t local_delta =
				ccp_slave_sdelta40(prev_rx_ts, last_obs_rx_ts);

			uint64_t expected =
				(uint64_t)intervals * SYNC_CCP_INTERVAL_DTU;
			uint64_t lo = expected / CCP_SLAVE_GAP_TOL_FACTOR;
			uint64_t hi = expected * CCP_SLAVE_GAP_TOL_FACTOR;

			if (local_delta < 0 || (uint64_t)local_delta < lo ||
			    (uint64_t)local_delta > hi) {
				/* Reported in microseconds, not raw DTU: no
				 * LOG_* call anywhere else in this project
				 * prints a bare uint64_t/int64_t, and 1 DTU =
				 * 1/64 ns keeps both well inside int32_t/
				 * uint32_t at any run this branch handles
				 * (128 intervals is ~25.6 s, ~25.6e6 us,
				 * comfortably inside 32 bits either way). */
				int32_t local_delta_us = (int32_t)(
					local_delta /
					(int64_t)(SYNC_DTU_PER_NS * 1000u));
				uint32_t expected_us = (uint32_t)(
					expected /
					(uint64_t)(SYNC_DTU_PER_NS * 1000u));

				LOG_WRN("{\"ccp_slave\":{\"seq_discontinuity\":1,"
					"\"intervals\":%u,\"expected_us\":%u,"
					"\"local_delta_us\":%d}}",
					intervals, expected_us,
					local_delta_us);
				n_reject++;
				sync_model_init(&model);
				have_obs = false;
			} else if (intervals >
				   (uint8_t)(SYNC_MISS_MAX + 1u)) {
				/* Already coasted past the model's own limit,
				 * so its estimate is invalid whatever we do. A
				 * fresh baseline beats feeding it hundreds of
				 * misses one call at a time on the SLAVE
				 * loop. */
				n_gap += (uint32_t)intervals - 1u;
				sync_model_init(&model);
				have_obs = false;
			} else {
				for (uint8_t k = 1u; k < intervals; k++) {
					sync_model_miss(&model);
					n_gap++;
				}
			}
		}
	}

	sync_model_observe(&model, f.tx_dtu, prev_rx_ts);
	n_paired++;
	last_obs_seq = prev_seq;
	last_obs_rx_ts = prev_rx_ts;
	have_obs = true;

	save_prev(f.seq, this_rx_ts);
	return true;
}

const struct sync_model *ccp_slave_model(void)
{
	return &model;
}

void ccp_slave_residual_reset(void)
{
	/* k_sched_lock()/k_sched_unlock() around exactly the three stores
	 * sync_model_residual_reset() makes -- res_sq_sum, res_n, res_max --
	 * and nothing else. The fence is here, not in sync_model.c, because the
	 * race is specific to THIS caller: the shell thread can be preempted by
	 * the SLAVE loop's own sync_model_observe() mid-update (see
	 * ccp_slave_model()'s comment on why a SLAVE runs main() at the default,
	 * preemptible priority). k_sched_lock() is legal from any thread,
	 * including the shell, and this region does no blocking call and no log
	 * write, so it cannot hold the lock across either. */
	k_sched_lock();
	sync_model_residual_reset(&model);
	k_sched_unlock();
}

void ccp_slave_stats(uint32_t *rx, uint32_t *gap, uint32_t *reject,
		     uint32_t *root)
{
	if (rx) {
		*rx = n_rx;
	}
	if (gap) {
		*gap = n_gap;
	}
	if (reject) {
		*reject = n_reject;
	}
	if (root) {
		*root = cur_root;
	}
}

void ccp_slave_stats_ex(uint32_t *paired, uint32_t *no_announce)
{
	if (paired) {
		*paired = n_paired;
	}
	if (no_announce) {
		*no_announce = n_no_announce;
	}
}
