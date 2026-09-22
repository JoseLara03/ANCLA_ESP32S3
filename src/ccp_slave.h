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
 * from a real DW3220 -- and the sequence/pairing bookkeeping that tells the
 * model when an expected CCP did not arrive, or could not be turned into an
 * observation.
 *
 * ---- The deferred-timestamp protocol (Option B) -------------------------
 *
 * A CCP no longer carries its OWN transmit time -- it cannot, since an
 * immediate TX's actual RMARKER is only known after the frame is already on
 * air (see ccp_master.c). Instead, a CCP with sequence S carries the ACTUAL
 * measured TX timestamp of the PREVIOUS CCP, sequence S-1. So this module's
 * observation of frame S-1 is only completed once frame S has been received:
 * an observation now LAGS the frame it describes by one superframe, and a
 * CCP lost in the air costs its receiver TWO things, not one -- the missed
 * frame itself, AND the announcement that would have completed the PRIOR
 * frame's observation (since that announcement travelled only in the frame
 * that was lost).
 *
 * tx_dtu == 0 is a reserved sentinel meaning "no announcement": either the
 * master's first CCP after its own boot (nothing earlier to announce), or a
 * one-shot "sync txtest" probe (seq 0xFF), which never carries an
 * announcement either. Both are handled identically here -- remembered as
 * the new "last received" frame, with nothing paired this call.
 *
 * The pairing itself needs no second sequence field on the wire: the
 * announced timestamp always belongs to sequence S-1 by construction, so
 * matching it to a local RX time only requires remembering the last frame
 * this node actually received (sequence and local RX timestamp) and checking
 * that IT was sequence S-1.
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

/* Counters, receive side. `n_rx` is every CCP frame RECEIVED and successfully
 * parsed -- it advances whether or not that frame could be turned into an
 * observation, which is exactly the distinction an operator needs: `rx`
 * climbing while `count` (sync_model_residual_count(), the paired-observation
 * count the `sync stats` verdict is gated on) stays flat means frames are
 * arriving but not pairing -- see ccp_slave_stats_ex() below for why, and
 * check `no_announce` there first (a healthy link with a freshly-rebooted
 * master, or one running only "sync txtest" probes, looks exactly like this).
 *
 * `n_gap` is expected CCPs that never arrived, `n_reject` is frames that were
 * CCPs but were not usable for pairing -- a parse failure, a hop this node
 * may not adopt, a frame whose announcement's target was never received (the
 * announced predecessor was lost in the air, or this is the first CCP after a
 * root change), a run of intervals that moved BACKWARDS by more than 128 (the
 * ordinary trigger is a gateway reboot, which re-seeds ccp_seq at 0 without
 * changing root_id -- see ccp_slave.c), OR a run that looks like an ordinary
 * small forward gap but whose LOCAL elapsed time disagrees with it -- the
 * other half of that same gateway-reboot trigger, wrapped down instead of up,
 * which the sequence number alone cannot distinguish from a genuine gap and
 * the local DW3220 clock can (see ccp_slave.c's CCP_SLAVE_GAP_TOL_FACTOR).
 *
 * Every one of the re-baseline paths above calls sync_model_rebaseline(),
 * which as of 2026-09-18 KEEPS the residual statistics and the lifetime
 * counters and clears only the baseline. An earlier revision called
 * sync_model_init() on these paths, which wiped res_sq_sum/res_n/res_max too,
 * so a `sync stats` read straight after a discontinuity showed `count`
 * restarted and the verdict back at "insufficient" with no other symptom.
 * That is no longer the behaviour, and the change is deliberate twice over:
 * the residuals measure the LINK's jitter, which a sequence discontinuity
 * does not alter, and a re-baseline counter that a re-baseline resets could
 * never have counted one.
 *
 * What a re-baseline DOES still cost is the rate estimate. Reading it back:
 * `n_obs` restarts at 0, `rebaselines` increments, and the published
 * `sigma_dtu` jumps (sync_model_sigma_dtu()) until the fresh baseline reaches
 * SYNC_BASELINE_USEFUL -- see sync_model.h's B13 note. Conversion continues
 * across the gap only while the carried skew is young enough
 * (SYNC_SKEW_MAX_AGE_DTU); past that, this anchor stops publishing
 * observations until it re-converges, which is the correct outcome and shows
 * up on the gateway as that anchor simply going quiet for ~2 s. */
void ccp_slave_stats(uint32_t *n_rx, uint32_t *n_gap, uint32_t *n_reject,
		     uint32_t *root_id);

/* Counters, receive side, part 2 -- added for the deferred-timestamp design
 * and kept as a separate call so every existing caller of ccp_slave_stats()
 * above keeps compiling unchanged.
 *
 * `n_paired` is the number of CCPs actually turned into a sync_model
 * observation (sync_model_observe() called) -- this is the number that,
 * together with sync_model_residual_count(), an operator should read as "is
 * the pairing working", NOT n_rx. `n_no_announce` is frames received with
 * tx_dtu == 0 -- the boot sentinel or a "sync txtest" probe, both of which
 * are, by design, receptions that were never going to pair into anything (see
 * ccp_slave.h's top-of-file comment). A steadily climbing `n_no_announce`
 * alongside a flat `n_rx` would mean something else entirely: a master stuck
 * repeatedly re-sending its own boot sentinel, which cannot happen from a
 * single ccp_master_after_beacon() call site but is worth ruling out before
 * suspecting the pairing logic itself. */
void ccp_slave_stats_ex(uint32_t *n_paired, uint32_t *n_no_announce);

/* B14 flight-compensation wiring (see sync_model.h's sync_model_set_tau()):
 * install or change this anchor's tau live. ccp_slave_init() calls this once
 * at boot with uwb_config's resolved effective tau; the `sync tau` shell
 * commands call it again on every change (FR-5's feed-forward). Fenced with
 * k_sched_lock()/k_sched_unlock() for the same reason
 * ccp_slave_residual_reset() is: the SLAVE loop is a preemptible-priority
 * writer that can interrupt the shell mid-call. */
void ccp_slave_set_tau(int32_t tau_dtu);

/* The tau currently applied to the live model. */
int32_t ccp_slave_get_tau(void);

#endif /* CCP_SLAVE_H */
