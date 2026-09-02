/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The gateway side of TDoA: drain the anchors' observations, group them, solve,
 * and hand a struct pos_fix to pos_sink_publish().
 *
 * NOTHING DOWNSTREAM CHANGES. This module produces exactly the struct pos_fix
 * the 0xEA dispatch path in uwb_gateway.c already produces -- same Tid
 * derivation (gw_core_find_eui() + tag_id_from_eui()), same pos_sink_publish(),
 * same pos_json_fix() payload, same frozen contract with the customer platform.
 * The measurement model changed; the telemetry did not.
 *
 * ---- Bounded, and called from K_PRIO_COOP(0) ------------------------------
 *
 * tdoa_gw_step() runs on the loop that arms the beacon, where no lower-priority
 * thread -- including the shell -- can run while it spins, and where an
 * unbounded busy-wait once froze this board permanently. So it is bounded twice
 * over: at most TDOA_GW_INGEST_MAX observations folded in and at most
 * TDOA_GW_SOLVE_MAX solves per call, both constants and neither derived from
 * anything an anchor or a broker controls. It never blocks (net_uplink_get_obs()
 * is K_NO_WAIT), never transmits, and never writes flash.
 *
 * It is called ONCE PER SUPERFRAME, from the top of the gateway's outer loop --
 * i.e. just after a beacon has gone out, where nearly the whole superframe of
 * margin remains. Deliberately NOT from the inner RX loop: there it would run
 * hundreds of times per superframe for no gain, since observations arrive on the
 * uplink thread's cadence (POLL_TIMEOUT_MS, 50 ms) and not on RX events.
 *
 * ---- The thread boundary is the queue, so no lock exists here --------------
 *
 * MQTT PUBLISHes are parsed on the uplink thread (priority 10) and pushed into
 * net_uplink's obs_q; struct tdoa_collect is owned exclusively by THIS module,
 * touched only from tdoa_gw_step(), i.e. only from the gateway loop. That is
 * why there is no mutex anywhere in this path: the two threads share a
 * k_msgq and nothing else.
 */

#ifndef TDOA_GW_H
#define TDOA_GW_H

#include <stdint.h>

#include "gw_core.h"

/* ---- Per-call bounds, and the tag capacity they buy -----------------------
 *
 * THE ARRIVAL RATE. Re-derive this for your own site rather than trusting the
 * numbers below; every term is a deployment parameter:
 *
 *   observations per superframe = anchors x blink_rate_hz x tags x 0.2
 *   groups       per superframe =           blink_rate_hz x tags x 0.2
 *
 * (0.2 because T_SUPERFRAME_UUS is 200 ms, i.e. five superframes a second.)
 * At this project's deployment -- 4 surveyed anchors, 5 Hz BLINK -- that is
 * 4 observations and 1 group per superframe PER TAG. An earlier revision of
 * this comment said 8 observations per superframe at "4 anchors x 5 Hz x 8
 * movers"; that is the arithmetic for TWO tags, wrong by 4x, and it made a
 * hard 2-tag ceiling look like generous burst headroom.
 *
 * WHAT tdoa_gw_step() MUST SUSTAIN, then, is INGEST_MAX >= 4 x tags and --
 * because tdoa_collect_take_ready() releases AT MOST ONE group per call --
 * SOLVE_MAX >= tags. Under either bound the shortfall does not queue up
 * politely: it accumulates in net_uplink's obs_q (OBS_QUEUE_DEPTH, 32) which
 * EVICTS THE OLDEST when full, so the loss lands upstream of this module and
 * shows on `blink stats` as rx_drop_evict, not as anything here.
 *
 * THE BUDGET. The binding constraint is NOT BEACON_ARM_MARGIN_UUS: this runs
 * at the TOP of the outer loop, immediately after a beacon went out, so the
 * next arm is a whole T_SUPERFRAME_UUS (200 ms) away rather than 5 ms. What is
 * actually at stake is the CAP service window -- every microsecond spent here
 * is a microsecond the inner loop is not servicing JOIN/GRANT/POS RX -- so the
 * self-imposed budget is 1 % of a superframe, 2 ms. Against that:
 *
 *   ingest: one k_msgq_get + a <= APOS_MAX_NODES survey scan + two bounded
 *           TDOA_COLLECT_SLOTS scans, all integer  --  order 10 us, call it
 *           32 x 10 us = 320 us
 *   solve:  one Gauss-Newton fit over <= 4 anchors, POS_GN_MAX_ITERS of
 *           single-precision sqrtf on the S3's FPU  --  conservatively 100 us,
 *           call it 8 x 100 us = 800 us
 *
 * ~1.1 ms of the 2 ms budget, and both figures are ESTIMATES: they have never
 * been measured on hardware. The instrument that settles it is the gw_sf
 * heartbeat staying at exactly 200.0 ms with no "beacon started but TXFRS
 * never completed", which is what turned APOS_GW_SOLVE_BUDGET_UUS from an
 * estimate into a measured number. Raising either constant without re-reading
 * that heartbeat is exactly the mistake CLAUDE.md's TX_COMPLETE_TIMEOUT_MS
 * entries document.
 *
 * THE VALUES, AND THE CEILING THEY DOCUMENT. INGEST_MAX is set equal to
 * OBS_QUEUE_DEPTH so that one step can always drain a full queue -- this
 * module can then never be the reason the queue evicts. SOLVE_MAX follows from
 * the same tag count:
 *
 *   4 anchors, 5 Hz  ->  32 / 4 = 8 TAGS, and 8 groups per superframe = 8 tags
 *
 * So this gateway sustains EIGHT tags at 5 Hz, and that is the ceiling to
 * check against a site, not a burst allowance. It is also where obs_q's own
 * depth of 32 puts the ceiling, so raising these two alone would buy nothing.
 * At more than 8 tags, or a faster BLINK rate, the honest fix is to raise
 * OBS_QUEUE_DEPTH and both of these together, and then re-read the heartbeat.
 * The previous values (8 / 2) capped the system at TWO tags. */
#define TDOA_GW_INGEST_MAX  32u
#define TDOA_GW_SOLVE_MAX   8u

/* ---- The per-tag EKF (2026-09-02 accuracy/smoothing work) -----------------
 *
 * Largest gap accepted between a tag's consecutive fixes before the filter's
 * dt is treated as meaningless and a fresh tdoa_solve()+pos_ekf_seed() runs
 * instead of a predict(). Same order as TDOA_GW_SEED_AGE_MS (ten blinks at
 * 5 Hz): a gateway reboot re-bases sync_model and the master clock jumps, and
 * a tag reappearing after minutes has no velocity worth extrapolating either
 * way. See docs/superpowers/specs/2026-09-02-tdoa-accuracy-filter-design.md
 * section 3.2.
 *
 * **Observed live on hardware, 2026-09-02, on this project's own deployed
 * gateway (real anchors, real MQTT broker, one real joined tag): `n_filtered`
 * stayed low against `n_dt_invalid`.** Diagnosed with temporary logging
 * (removed once root-caused, not left in-tree) rather than guessed at.
 * Two DISTINCT causes, both already correctly handled, neither a defect:
 * (1) a NEGATIVE raw_dt with no 40-bit wrap crossed -- two of the same tag's
 * groups draining out of blink-chronology order in one tdoa_gw_step() call,
 * because tdoa_collect_take_ready() scans slots in table order, not time
 * order, and a tag with patchy anchor coverage can have more than one of its
 * groups pending at once; (2) a raw_dt of several SECONDS, wrap-corrected --
 * long gaps between the RARE blinks that actually reached TDOA_MIN_ANCHORS
 * anchors for that tag, i.e. marginal RF coverage upstream in
 * tdoa_collect, not a timing bug here. Both are exactly the "dt <= 0
 * (reordering)" and "gap too large" cases this section already documents,
 * and the gate routing them to a fresh solve instead of a bogus predict is
 * the CORRECT behaviour, not the anomaly. Recorded here so nobody re-chases
 * this as a Task 4 defect: a dt_invalid-heavy site is a coverage/RF
 * observation about THAT deployment, not evidence against this constant or
 * the gate around it. */
#define TDOA_DT_MAX_MS  2000

/* TDOA_GW_MOVING_ENTER_MPS / _EXIT_MPS lived here until 2026-09-03. They
 * scheduled the EKF's process noise from the FILTER'S OWN velocity estimate,
 * because at the time nothing on this path carried an accelerometer reading.
 * BLINK_FLAG_MOVING (proto 5) now does, so the closed loop is gone rather
 * than kept as a fallback -- see the removal note above solve_one() in
 * tdoa_gw.c for why having both would be worse than either. */

/* Clear the collector and every cache. Call once, before the gateway loop. */
void tdoa_gw_init(void);

/* Ingest up to TDOA_GW_INGEST_MAX observations and publish up to
 * TDOA_GW_SOLVE_MAX fixes. `ctx` is the live seat table, read-only, for the
 * EUI lookup that Tid needs; `now_ms` is k_uptime_get_32(). */
void tdoa_gw_step(const struct gw_core_ctx *ctx, uint32_t now_ms);

/* ---- What this module does NOT read: out->residual_m at three anchors -----
 *
 * tdoa_solve.h states it as a hard caller contract, and this module is that
 * caller: n anchors give n-1 range-difference equations against 2 unknowns, so
 * at TDOA_MIN_ANCHORS (3) the system is exactly determined and residual_m comes
 * back numerically zero however wrong the input is -- a 2 m timestamp error
 * moves the fix 2.4 m while the residual stays under a millimetre. So the
 * mirror-branch/plausibility defence here is TDOA_GW_MAX_JUMP_M against a recent
 * previous fix plus tdoa_dtu_plausible(), never the residual; and a three-anchor
 * fix is published with residual_m explicitly zeroed rather than carrying a
 * number that reads like a quality figure on pos_sink's console line. The
 * `"n":3` on that same line is the only flag struct pos_fix has room for.
 *
 * ---- The gateway contributes no observation of its own, deliberately -------
 *
 * blink_rx_init() is called only from uwb_slave.c, so a GATEWAY never stamps a
 * BLINK it hears itself. That is correct and not a gap: the gateway holds
 * reserved short address 0x0000, consumes no anchor_id, and is therefore not in
 * the applied survey at all -- anchor_xyz() could not position its observation
 * even if it made one. Four surveyed anchors are the observers; the gateway is
 * the solver.
 */

/* Counters, for `net show` and for the bench. `n_obs` are observations folded
 * into the collector, `n_reject` observations the collector REFUSED (a duplicate
 * anchor for a blink, or every slot already holding a releasable group -- see
 * tdoa_collect.h's slot-exhaustion note; a climbing n_reject is load shedding,
 * which otherwise looks exactly like anchors going quiet), `n_fix` fixes
 * published, `n_no_anchor` observations
 * dropped because no surveyed anchor matches their anchor_id, `n_implausible`
 * groups rejected by tdoa_dtu_plausible(), `n_solve_fail` groups tdoa_solve()
 * refused, and `n_jump` fixes rejected for moving impossibly far from a recent
 * previous fix for the same tag. */
void tdoa_gw_stats(uint32_t *n_obs, uint32_t *n_reject, uint32_t *n_fix,
		   uint32_t *n_no_anchor, uint32_t *n_implausible,
		   uint32_t *n_solve_fail, uint32_t *n_jump);

/* n_reject above, broken out -- and it must be, because its two causes call for
 * opposite actions. `n_dup` is a duplicate report of one blink by one anchor (an
 * MQTT redelivery or a retry): harmless, and folding it in twice would hand
 * tdoa_solve() a zero-difference equation against itself and make its normal
 * matrix singular, which is why tdoa_collect refuses it. `n_shed` is LOAD
 * SHEDDING: every collector slot already held a releasable group, so the new
 * observation lost that contest rather than destroying a fix that already
 * exists (see tdoa_collect.h's slot-exhaustion note). A climbing n_shed means
 * tdoa_gw_step() is not draining fast enough -- TDOA_GW_SOLVE_MAX is the knob --
 * and is otherwise indistinguishable at the console from anchors going quiet.
 * Same split, and the same reason, as net_uplink_obs_rx_drops(). */
void tdoa_gw_reject_detail(uint32_t *n_dup, uint32_t *n_shed);

/* The per-tag EKF's own counters, distinct from the ingest/solve counters
 * above: those are about whether a GROUP made it to a solve attempt at all,
 * these are about what the FILTER did with it once solved. Without these
 * the filter is a black box on the bench -- no way to tell "no tags" from
 * "the filter rejects everything".
 *
 * `n_seeded` fixes produced by a COLD-START tdoa_solve()+pos_ekf_seed()
 * (no filter running yet for that tag); `n_reseed` fixes produced the same
 * way but recovering an ALREADY-seeded filter whose gate_streak reached
 * cfg.reset_after (pos_ekf_needs_reseed()); `n_filtered` fixes produced by
 * predict()+update_tdoa() with no solve at all -- the steady-state case;
 * `n_dt_invalid` cycles where an ALREADY-seeded filter could not get a
 * usable dt (forcing the reseed path instead of a predict -- a fresh
 * filter's first-ever group is NOT counted here, since it has no previous
 * dt to have been invalid); `n_gate_rejected` the cumulative count of
 * individual range-difference equations pos_ekf_update_tdoa() gated out
 * (out of n-1 attempted per filtered fix); `n_no_update` cycles where
 * NOTHING published -- dt was invalid AND the solve+seed fallback also
 * failed (solve_fail or the mirror-branch jump gate), so the filter's
 * state did not move and publishing its unchanged prior value would have
 * republished stale data as a live fix (found live on hardware
 * 2026-09-02: the same tag published the exact same (x, y) to two
 * decimals, minutes apart -- see solve_one()'s `state_changed` comment).
 * `n_no_update` is a SUBSET of `n_dt_invalid`, not additional to it: every
 * no-update cycle already incremented n_dt_invalid too.
 *
 * `n_zupt` counts zero-velocity updates applied, i.e. filtered cycles where
 * the tag's accelerometer reported still. A SUBSET of `n_filtered`. Read it
 * to tell "the MOVING bit never arrives" -- an anchor still on proto 4 sends
 * no flags field, which parses as 0, so n_zupt would equal n_filtered --
 * from "it arrives and says moving", which no other counter distinguishes. */
void tdoa_gw_ekf_stats(uint32_t *n_seeded, uint32_t *n_reseed,
		       uint32_t *n_filtered, uint32_t *n_dt_invalid,
		       uint32_t *n_gate_rejected, uint32_t *n_no_update,
		       uint32_t *n_zupt);

#endif /* TDOA_GW_H */
