/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * A GATEWAY-only, one-shot alternative to measuring the gateway's own
 * position with a tape and running tools/sync_tau_from_survey.py: range the
 * gateway directly to an already-surveyed anchor over the air, using the
 * SAME SS-TWR exchange (src/ss_initiator.c) and the SAME WAVE responder
 * every anchor already answers (src/anchor_respond.c) -- no new wire format,
 * no new frame code, no coordinate math and no tape measurement of the
 * gateway's own position at all. tau_dtu = round(one-way distance / DTU),
 * directly.
 *
 * This is B14's Fase A (docs/superpowers/plans/2026-09-22-b14-flight-compensation.md),
 * not Fase B: it is a MANUAL, operator-triggered, one-shot measurement --
 * not the round-robin, filtered, auto-tracking on-air measurement FR-6..FR-9
 * describe. The operator still copies the printed tau_dtu into `sync tau
 * set` on the target anchor's own console by hand; nothing here writes to
 * another board.
 *
 * A STEP MACHINE, not a blocking call from the shell: the gateway's main
 * loop runs at K_PRIO_COOP(0) and the beacon is the whole network's time
 * base (same constraint apos_gw.h documents for its own step machine).
 * gw_selftau_step() advances a request by AT MOST ONE ss_initiator_range()
 * exchange and returns, so a request averaged over `n` exchanges spans up
 * to `n` superframes. That is deliberate, not an oversight: unlike
 * cal_shell.c's `cal link` (its own dedicated calibration-image loop, no
 * beacon to protect, exchanges run back-to-back), this shares the
 * production gateway's loop with the beacon and must never hold it for more
 * than one bounded exchange at a time.
 */

#ifndef GW_SELFTAU_H
#define GW_SELFTAU_H

#include "ss_initiator.h"   /* SS_INITIATOR_MM_PER_DTU_X1000 */

#include <stdbool.h>
#include <stdint.h>

/* Worst-case airtime of ONE ss_initiator_range() exchange that must be safe
 * to spend inside uwb_gateway_run()'s inner loop: ss_initiator.c's own
 * TX_DONE_TIMEOUT_MS (10) + RX_DONE_TIMEOUT_MS (25) = 35 ms, rounded up for
 * ss_initiator_enter()/leave() overhead and the mean/tau arithmetic.
 * Compared against `to_beacon` the exact same way APOS_GW_STEP_BUDGET_UUS
 * is in uwb_gateway.c's inner loop. Far under LOOP_STALL_MS (300 ms). */
#define GW_SELFTAU_STEP_BUDGET_UUS 40000u

#define GW_SELFTAU_MAX_N     32u
#define GW_SELFTAU_DEFAULT_N 8u

/* Pure conversion, no Zephyr dependency, so a host test can call it directly:
 * round(mean_mm / (SS_INITIATOR_MM_PER_DTU_X1000/1000)) with the division
 * done as one integer round-half-up, never a float. `mean_mm` is assumed
 * non-negative -- a physical one-way distance from ss_initiator_range() --
 * and is not defended against a negative input beyond returning a
 * (meaningless but non-crashing) value for it, since no caller here can
 * produce one. */
static inline int32_t gw_selftau_mm_to_dtu(int32_t mean_mm)
{
	int64_t num = (int64_t)mean_mm * 1000 +
		      (int64_t)(SS_INITIATOR_MM_PER_DTU_X1000 / 2);

	return (int32_t)(num / SS_INITIATOR_MM_PER_DTU_X1000);
}

/* GATEWAY-only. Queue a request to range this board against anchor <id>
 * (0..UWB_MAX_ANCHORS-1), averaged over `n` exchanges (1..GW_SELFTAU_MAX_N).
 * Returns false and queues nothing if a request is already in progress, `id`
 * or `n` is out of range. Does not itself validate that anchor <id> is
 * surveyed/positioned -- an unpositioned anchor simply will not answer (see
 * CLAUDE.md's "unpositioned anchor is silent to WAVE polls" fact), which
 * shows up as attempted > valid, not as a rejected request here. */
bool gw_selftau_request(uint8_t anchor_id, uint32_t n);

/* True while a request is queued or in progress (i.e. gw_selftau_step()
 * still has exchanges left to run for it). */
bool gw_selftau_busy(void);

/* Called from uwb_gateway_run()'s inner loop, at the same point and under
 * the same beacon-margin reserve check as apos_gw_step(). Advances the
 * pending request by at most one ss_initiator_range() exchange and returns.
 * A no-op when gw_selftau_busy() is false. */
void gw_selftau_step(void);

struct gw_selftau_result {
	uint8_t  anchor_id;
	uint32_t attempted;   /* exchanges tried so far (or in total, once done) */
	uint32_t valid;       /* of those, how many got a response */
	int32_t  mean_mm;     /* mean of the VALID exchanges; 0 if valid == 0 */
	int32_t  min_mm;
	int32_t  max_mm;
	int32_t  tau_dtu;     /* gw_selftau_mm_to_dtu(mean_mm); 0 if valid == 0 */
	bool     done;        /* the most recently REQUESTED run has finished */
};

/* The last requested run's state -- valid to read whether it is still in
 * progress (done == false, attempted/valid/mean_mm are the running partial
 * tally) or finished (done == true, and the fields above are final). No
 * locking: gw_selftau_step() runs on the GATEWAY's K_PRIO_COOP(0) main
 * thread, which nothing at a lower priority -- including the shell -- can
 * preempt mid-update (same reasoning src/ccp_slave.h's ccp_slave_model()
 * gives for why a SLAVE needs a fence here and a GATEWAY does not: the two
 * roles run main() at different priorities). */
void gw_selftau_get_result(struct gw_selftau_result *out);

#endif /* GW_SELFTAU_H */
