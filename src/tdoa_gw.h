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

/* ---- The per-tag alpha-beta-gamma filter (2026-09-06) ----------------------
 *
 * Largest FORWARD dt the filter predicts through. Beyond it the tag's filter
 * is reseeded on the fresh solve instead (counted `dt_reseed`): a gateway
 * reboot re-bases sync_model and the master clock jumps, a tag in a slow
 * reporting tier goes seconds between blinks, and a tag that vanished for
 * that long has no velocity worth extrapolating. 1000 ms, not the EKF's 2000:
 * the measured dt distribution (2026-09-03, tools/pos_trace.py) has p90 at
 * 0.8 s, so 1 s keeps ~90 % of cycles on the filtered path while refusing
 * to coast a walking tag more than ~1.5 m. The part-2 plan's 600 ms would
 * reseed on >10 % of cycles -- reintroducing jumps at exactly the cadence the
 * filter exists to hide. An aliased large-NEGATIVE dt (see the next constant)
 * is treated the same way: genuinely new group, reseed. */
#define TDOA_DT_MAX_MS  1000

/* The largest BACKWARDS dt that can still be genuine group reordering rather
 * than a forward gap that aliased through sdelta40()'s sign boundary.
 *
 * Two different things produce a negative dt and they need opposite handling:
 *
 *  - REORDERING. Two adjacent blinks whose observations interleave over MQTT
 *    can be released inverted, so the second group processed describes an
 *    EARLIER instant. tdoa_collect_take_ready() releases oldest-first
 *    precisely to make this rare, but it orders by gateway ARRIVAL and cannot
 *    make it impossible (see its own contract). Such a group is STALE: it must
 *    not be published (the trace would step backwards in time), and above all
 *    it must not be allowed to rewind `last_ref_t_dtu`, or a filter's next
 *    dt then covers time already integrated -- the 2026-09-03 defect, found
 *    with the EKF and just as real for the filter that replaces it. Bounded
 *    by TDOA_COLLECT_WINDOW_MS (150 ms) plus the drain
 *    latency of one superframe -- the largest inversion actually measured on
 *    hardware 2026-09-03 was 800 ms.
 *
 *  - AN ALIASED FORWARD GAP. sdelta40() resolves +/-2^39 DTU, i.e. +/-8.6 s,
 *    and a tag in a slow reporting tier legitimately goes 9-15 s between
 *    blinks that reach TDOA_MIN_ANCHORS anchors. Those read NEGATIVE: a real
 *    10.2 s gap comes back as -7.007 s, which is exactly what the 2026-09-03
 *    trace shows. The group is genuinely NEW; its reference timestamp must
 *    advance, and a filter must reseed rather than predict through it.
 *    There is no un-wrapped clock available here to disambiguate these by
 *    arithmetic -- t_dtu wraps at 17.2 s, full stop -- so the discriminator is
 *    the magnitude, and it works only because the two populations are three
 *    orders of magnitude apart.
 *
 * 1000 ms sits above the measured reordering worst case with margin and an
 * order of magnitude below the aliasing boundary. Getting this wrong in the
 * generous direction (too large) makes a real short gap look like reordering
 * and drops a valid fix; too small lets an aliased gap rewind the reference,
 * which is the defect this exists to close. */
#define TDOA_DT_REORDER_MAX_MS  1000

/* The per-tag EKF that ran here 2026-09-02..06 is gone; its replacement is
 * pos_abg (src/pos_abg.h), an alpha-beta-gamma filter on the SOLVED position.
 * docs/superpowers/specs/2026-09-06-abg-position-filter-design.md. */

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


/* Groups discarded for arriving out of order (see TDOA_DT_REORDER_MAX_MS).
 * Not a subset of any counter in tdoa_gw_stats(): such a group is never
 * solved, so it is neither a fix nor a solve_fail. Expected in small numbers;
 * a count approaching `fixes` means the backhaul is reordering heavily and the
 * published stream carries far less of the data than `ingested` suggests. */
uint32_t tdoa_gw_reorder_count(void);

/* The filter's own counters, the third `blink stats` line. Publish identity
 * that MUST hold, and that tools/pos_trace.py checks:
 *
 *     fixes == seeded + dt_reseed + filtered + reseed
 *
 * `seeded`: a tag's first group (fresh memo slot) -- solve, seed, publish.
 * `dt_reseed`: an already-seeded filter whose dt exceeded TDOA_DT_MAX_MS or
 * aliased negative -- seed on the fresh solve, publish. `filtered`: predict +
 * accepted correction -- publish. `gate_rejected`: predict only, correction
 * gated -- publish NOTHING (no evidence arrived; republishing the prediction
 * would be the 2026-09-02 stale-republish bug by design). `reseed`: the
 * rejection streak reached cfg.reset_after, state replaced by the solve --
 * publish. `still`: filtered cycles that took the still branch; a SUBSET of
 * `filtered`, and `still == filtered` on a live fleet means the MOVING bit is
 * not arriving (proto-4 anchors), same trap the EKF's `zupt` counter caught. */
void tdoa_gw_abg_stats(uint32_t *n_seeded, uint32_t *n_dt_reseed,
		       uint32_t *n_filtered, uint32_t *n_gate_rejected,
		       uint32_t *n_reseed, uint32_t *n_still);

/* Runtime tuning, NOT persisted, for the visual lambda sweep (spec §4.5). A
 * reboot returns to pos_abg_cfg_defaults(). The setters are fenced with
 * k_sched_lock()/k_sched_unlock(): the shell thread writes while the
 * K_PRIO_COOP(0) gateway loop reads field by field, and the loop CAN preempt
 * the shell between two field stores. */
struct pos_abg_cfg;
const struct pos_abg_cfg *tdoa_gw_abg_cfg(void);
float tdoa_gw_abg_lambda(void);
void tdoa_gw_abg_set_lambda(float lambda);
void tdoa_gw_abg_set_gamma(float gamma);       /* 0 makes it alpha-beta; spec §3.3 */
void tdoa_gw_abg_set_gate(float gate_m);
void tdoa_gw_abg_set_alpha_still(float alpha_still);
void tdoa_gw_abg_set_reset_after(uint8_t n);

#endif /* TDOA_GW_H */
