/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The SLAVE side of anchor clock sync: receive CCPs, feed sync_model, and own
 * the one instance of it.
 *
 * This module holds no arithmetic of its own. Everything that could be wrong
 * about the estimator lives in sync_model.c, which is pure C and host-tested;
 * what lives here is the part a host test cannot reach -- a real RX timestamp
 * from a real DW3220 -- and the sequence bookkeeping that tells the model when
 * an expected CCP did not arrive.
 */

#ifndef CCP_SLAVE_H
#define CCP_SLAVE_H

#include "sync_model.h"

#include <stdbool.h>
#include <stdint.h>

/* Clear to the no-observations state. Call once before the SLAVE loop starts. */
void ccp_slave_init(void);

/* Offer one received frame. Returns true if it WAS a CCP -- consumed either
 * way, so the caller can stop its dispatch chain -- and false if the frame is
 * for someone else.
 *
 * `rx_ts` is the full 40-bit DW3220 RX timestamp, exactly as
 * uwb_get_rx_timestamp_u64() returns it. Do not shift it: sync_model works in
 * whole DTU, and handing it a hi32 value would throw away the bottom 8 bits --
 * 255 DTU, ~4 ns, against a 1 ns gate. */
bool ccp_slave_on_rx(const uint8_t *buf, uint16_t plen, uint64_t rx_ts);

/* The live model, for the `sync` shell command. Read-only: on a SLAVE, main()
 * is left at the default (preemptible) priority -- see main.c's comment on why
 * only GATEWAY mode is promoted to K_PRIO_COOP(0) -- so the SLAVE loop can
 * preempt the shell at any point, including mid-update of `model`. A const
 * pointer is what makes that safe for a READER: the shell can only ever see a
 * complete previous state or a complete next one, never a half-written one,
 * because it has no way to write through this pointer at all. It does NOT make
 * the model safe to mutate from the shell -- see ccp_slave_residual_reset()
 * for the one write path reachable from there, and why it is fenced instead of
 * exposed here. */
const struct sync_model *ccp_slave_model(void);

/* Clear the residual statistics (RMS/max/count), keeping the rate estimate and
 * baseline untouched -- the `sync reset` command's write path.
 *
 * Fenced with k_sched_lock()/k_sched_unlock() because the SLAVE loop is a
 * preemptible-priority writer that can interrupt the shell thread at any
 * instruction boundary (see ccp_slave_model()'s comment). Without the fence,
 * sync_model_residual_reset()'s three stores -- one of them a 64-bit
 * res_sq_sum, two stores on this target -- can interleave with
 * sync_model_observe()'s `res_sq_sum += mag*mag; res_n++` and leave a stale
 * high half of a CUMULATIVE sum standing next to a restarted res_n, inflating
 * rms_dtu/jitter_est_dtu until the next reset -- exactly the misreading this
 * command exists to prevent, on hardware that actually passes. */
void ccp_slave_residual_reset(void);

/* Counters. `n_gap` is expected CCPs that never arrived, `n_reject` is frames
 * that were CCPs but were not usable -- a parse failure, a hop this node may
 * not adopt, a duplicate sequence number, a sequence number that moved
 * BACKWARDS by more than 128 (see ccp_slave.c: the ordinary trigger is a
 * gateway reboot, which re-seeds ccp_seq at 0 without changing root_id), OR a
 * sequence number that looks like an ordinary small forward gap but whose
 * LOCAL elapsed time disagrees with it -- the other half of that same
 * gateway-reboot trigger, wrapped down instead of up, which the sequence
 * number alone cannot distinguish from a genuine gap and the local DW3220
 * clock can (see ccp_slave.c's CCP_SLAVE_GAP_TOL_FACTOR).
 *
 * Every one of these rejection paths re-baselines via sync_model_init(),
 * which also clears the residual statistics (res_sq_sum/res_n/res_max) along
 * with the rate estimate -- sync_model.h documents those as surviving
 * "everything except an explicit reset", and that is still true of
 * sync_model.c's own API; ccp_slave.c is simply the first caller to invoke
 * sync_model_init() itself mid-life rather than only at ccp_slave_init(), so
 * from this module's own callers' point of view a re-baseline IS a second,
 * implicit reset. A `sync stats` read straight after one of these will show
 * `count` restarted and the verdict back at "insufficient", with no other
 * symptom -- expected, not a bug. */
void ccp_slave_stats(uint32_t *n_rx, uint32_t *n_gap, uint32_t *n_reject,
		     uint32_t *root_id);

#endif /* CCP_SLAVE_H */
