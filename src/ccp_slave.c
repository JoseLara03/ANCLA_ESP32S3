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
static uint8_t  last_seq;
static bool     have_seq;
static uint32_t n_rx;
static uint32_t n_gap;
static uint32_t n_reject;

/* Previous accepted frame's RX timestamp, 40-bit masked. Used only by the
 * discontinuity check below -- sync_model.c already keeps its own l_raw for
 * the estimator itself, and this is deliberately a SEPARATE copy rather than
 * reaching into that struct: this module owns no arithmetic of its own except
 * this one check, and sync_model.{c,h} must not be touched to add it. */
static uint64_t prev_rx_ts;

/* A genuine forward gap of `gap` superframes should show local elapsed time
 * within a small fraction of gap * SYNC_CCP_INTERVAL_DTU -- the only thing
 * that can move it off that mark is the two crystals' combined drift, which
 * CLAUDE.md's own cross-check bounds at roughly +/-40000 ppb, i.e. under
 * 0.005% even over the largest gap this path handles (128 intervals). A
 * factor of 4 either side is therefore not a tight statistical bound -- it
 * has three-plus orders of magnitude of margin over anything a real clock
 * pair can produce, so it cannot fire on ordinary jitter or on a genuinely
 * long outage (a real outage of N superframes advances both `gap` and the
 * local clock by the SAME N, so the ratio stays near 1 regardless of how
 * large N is). What it DOES catch is a gateway reboot: the local elapsed
 * time since the last accepted CCP is however long the reboot actually took,
 * which has no relationship at all to what the wrapped sequence-number
 * arithmetic computes for `gap`, so the two landing within a factor of 4 of
 * each other by coincidence is not realistic -- including the case a plain
 * gap>128 check cannot see at all: gap == 1 (last_seq in 229..255, wrapping
 * to a low post-reboot seq) with several REAL seconds elapsed locally, which
 * is a discontinuity by construction and not merely a slow superframe. */
#define CCP_SLAVE_GAP_TOL_FACTOR 4u

/* Known residual limitation, disclosed rather than hidden: local_delta is
 * derived from two 40-bit hardware timestamps, so it is only ever
 * TRUE_ELAPSED mod 2^40 (~17.18 s) -- there is no way to recover an elapsed
 * span longer than that from two raw DW3220 stamps alone. For a GENUINE
 * forward gap whose true elapsed time straddles a multiple of that wrap
 * (roughly gap in 86..114 at this superframe period, i.e. 17.2..22.8 s of
 * real, consecutive CCP loss with no reboot), the wrapped local_delta can
 * legitimately fall outside this factor-of-4 band, so this branch can
 * mislabel that one case as a discontinuity (n_reject, logged as a probable
 * reboot) instead of a gap (n_gap). That costs only the DIAGNOSTIC label:
 * this whole branch already calls sync_model_init() for both a genuine
 * large gap and a reboot (see the `else if` below), so the sync estimate
 * itself is identical either way -- nothing is poisoned, and nothing here
 * changes what the model does, only what an operator reads about why. */

/* Signed difference between two 40-bit DW3220 timestamps. Pattern-matched
 * from sync_model.c's sdelta40() (that file must not be modified, so this is
 * a deliberate local copy, not a shared helper): the counter wraps every
 * 2^40 DTU (~17.2 s), so an unsigned compare is wrong across the wrap and the
 * failure is rare, timing-dependent, and looks like a radio fault. Correct for
 * any interval under 2^39 DTU (~8.6 s) -- far past the largest gap (128
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

void ccp_slave_init(void)
{
	sync_model_init(&model);
	cur_root = 0u;
	last_seq = 0u;
	have_seq = false;
	n_rx = 0u;
	n_gap = 0u;
	n_reject = 0u;
	prev_rx_ts = 0u;
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

	if (!have_seq || f.root_id != cur_root) {
		/* A different root is a different time base, and the baseline
		 * built against the old one describes nothing. Start over
		 * rather than mixing two clocks into one rate estimate. */
		if (have_seq) {
			LOG_WRN("{\"ccp_slave\":{\"root_changed\":%u,"
				"\"was\":%u}}", f.root_id, cur_root);
		}
		sync_model_init(&model);
		cur_root = f.root_id;
		have_seq = true;
		last_seq = f.seq;
		sync_model_observe(&model, f.tx_dtu, rx_ts & SYNC_DTU_MASK);
		n_rx++;
		prev_rx_ts = rx_ts & SYNC_DTU_MASK;
		return true;
	}

	uint8_t gap = (uint8_t)(f.seq - last_seq); /* wraps at 256, as ccp_seq does */
	uint64_t this_rx_ts = rx_ts & SYNC_DTU_MASK;

	last_seq = f.seq;

	if (gap == 0u) {
		/* Same sequence number twice. Not a miss and not an
		 * observation: folding a duplicate in would count one interval
		 * as two and bias the rate. */
		n_reject++;
		return true;
	}

	if (gap > 128u) {
		/* A "gap" this large is far more likely to be the sequence
		 * number moving BACKWARDS than 129+ superframes of genuine
		 * forward misses. The ordinary trigger is a gateway reboot:
		 * ccp_master_init() re-seeds ccp_seq at 0 while root_id is
		 * derived from the board and does NOT change, so the
		 * root-change branch above never sees it either. Re-baselining
		 * the model is still correct here -- the old baseline really
		 * doesn't describe the new sequence -- but counting this as
		 * n_gap would tell the operator the LINK is dropping CCPs
		 * (Step 7 item 3's exact reading) when nothing was lost at
		 * all. n_reject is what it actually is: a frame this node
		 * could not fold into its running count.
		 *
		 * This branch covers only the HALF of a gateway reboot where
		 * the wrapped arithmetic happens to land above 128 (i.e. the
		 * slave's last_seq was itself small when the reboot hit). The
		 * other half -- last_seq large enough that the wrap lands the
		 * apparent gap back down at 1..128 -- looks exactly like an
		 * ordinary forward gap or miss run by sequence number alone,
		 * and is what the local-clock check below is for. */
		n_reject++;
		sync_model_init(&model);
	} else {
		/* gap in 1..128. Sequence number alone cannot tell a genuine
		 * forward gap of this size from the OTHER half of a gateway
		 * reboot (see the comment above): a reboot can wrap last_seq
		 * down to a small apparent `gap` just as easily as a large
		 * one. The local clock can, because it has no idea what the
		 * sequence number claims -- a genuine gap of `gap`
		 * superframes elapses `gap * SYNC_CCP_INTERVAL_DTU` of REAL
		 * local time (up to the crystals' negligible drift), while a
		 * reboot's elapsed local time is however long the reboot took,
		 * with no relationship to `gap` at all. See
		 * CCP_SLAVE_GAP_TOL_FACTOR for why that comparison is safe at
		 * a generous tolerance. */
		int64_t local_delta = ccp_slave_sdelta40(this_rx_ts, prev_rx_ts);
		uint64_t expected = (uint64_t)gap * SYNC_CCP_INTERVAL_DTU;
		uint64_t lo = expected / CCP_SLAVE_GAP_TOL_FACTOR;
		uint64_t hi = expected * CCP_SLAVE_GAP_TOL_FACTOR;

		if (local_delta < 0 || (uint64_t)local_delta < lo ||
		    (uint64_t)local_delta > hi) {
			/* Reported in microseconds, not raw DTU: no LOG_* call
			 * anywhere else in this project prints a bare uint64_t/
			 * int64_t, and 1 DTU = 1/64 ns keeps both well inside
			 * int32_t/uint32_t at any gap this branch handles
			 * (128 intervals is ~25.6 ms... no, ~25.6 s, ~25.6e6 us,
			 * comfortably inside 32 bits either way). */
			int32_t local_delta_us = (int32_t)(local_delta /
							    (int64_t)(SYNC_DTU_PER_NS * 1000u));
			uint32_t expected_us = (uint32_t)(expected /
							   (uint64_t)(SYNC_DTU_PER_NS * 1000u));

			LOG_WRN("{\"ccp_slave\":{\"seq_discontinuity\":1,"
				"\"gap\":%u,\"expected_us\":%u,"
				"\"local_delta_us\":%d}}",
				gap, expected_us, local_delta_us);
			n_reject++;
			sync_model_init(&model);
		} else if (gap > (uint8_t)(SYNC_MISS_MAX + 1u)) {
			/* Already coasted past the model's own limit, so its
			 * estimate is invalid whatever we do. A fresh baseline
			 * beats feeding it hundreds of misses one call at a
			 * time on the SLAVE loop. */
			n_gap += (uint32_t)gap - 1u;
			sync_model_init(&model);
		} else {
			for (uint8_t k = 1u; k < gap; k++) {
				sync_model_miss(&model);
				n_gap++;
			}
		}
	}

	sync_model_observe(&model, f.tx_dtu, this_rx_ts);
	n_rx++;
	prev_rx_ts = this_rx_ts;
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
