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

/* Per-call bounds. INGEST_MAX x (one k_msgq_get + one linear scan of
 * TDOA_COLLECT_SLOTS) and SOLVE_MAX x (one Gauss-Newton solve over <= 4
 * anchors) are both tens of microseconds against BEACON_ARM_MARGIN_UUS
 * (5000 uus) -- and this runs a whole superframe away from the next arm, not
 * next to it. At 4 anchors x 5 Hz x 8 movers, 8 observations per superframe is
 * the steady-state arrival rate, so the bound throttles a burst rather than
 * the normal case. Raising either without re-reading the beacon heartbeat on
 * hardware is exactly the mistake CLAUDE.md's TX_COMPLETE_TIMEOUT_MS entries
 * document. */
#define TDOA_GW_INGEST_MAX  8u
#define TDOA_GW_SOLVE_MAX   2u

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

#endif /* TDOA_GW_H */
