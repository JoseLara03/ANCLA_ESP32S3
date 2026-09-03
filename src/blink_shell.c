/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `blink` command tree: the Phase 3 observation path, read off a console.
 *
 * This tree exists because every failure on that path is otherwise INVISIBLE.
 * The anchor half stamps a BLINK and enqueues it; the gateway half subscribes
 * and drains. Neither half logs anything in production once it is working, and
 * every way it can stop working looks, from every other console surface, like
 * a healthy system with no tags in it:
 *
 *   - an anchor hearing tags fine but with its CCP link down drops every
 *     observation (n_no_sync == n_rx) and says nothing;
 *   - a gateway whose broker ACL refused the subscription is DEAF, and used to
 *     keep reporting "connected" while receiving nothing (n_sub_fail);
 *   - a publisher newer than this gateway has every payload rejected on length
 *     rather than tolerated (rx_drop_oversize -- see POS_JSON_BLINK_MAX_LEN's
 *     versioning note);
 *   - a gateway loop too slow to drain obs_q silently evicts (rx_drop_evict).
 *
 * The first two are the ones that cost bench time, and the drop counters are
 * split three ways precisely so a format incompatibility cannot be mistaken
 * for saturation.
 *
 * ---- Registered unconditionally, and it prints a `role` --------------
 *
 * Same precedent, and the same reason, as src/sync_shell.c (see its lines
 * 17-30): this tree is in the production image on EVERY role and in the
 * calibration image too, because blink_rx.c and net_uplink.c compile into
 * every image regardless of CONFIG_ANCLA_CAL_MODE -- only cal_run.c's own loop
 * never drives either. `stamped` is the anchor half, meaningful on a SLAVE;
 * `received`/`sub_fail` are the gateway half. On a role that never exercises
 * a counter it simply never moved, and a static all-zero line is
 * indistinguishable on its own from a dead link. Read the role field before
 * concluding anything about the numbers beside it.
 */

#include "blink_rx.h"
#include "net_uplink.h"
#include "tdoa_gw.h"

#include <string.h>
#include "uwb_config.h"

#include <zephyr/shell/shell.h>

#include <stdint.h>

#ifdef CONFIG_ANCLA_CAL_MODE
/* The cal image runs cal_run()'s own loop, never uwb_slave_run() or
 * uwb_gateway_run(), so cfg->mode -- persisted NVS state, unrelated to what
 * actually executes here -- would actively mislead. Say what it really is.
 * Copied from sync_shell.c deliberately rather than shared: two three-line
 * functions are cheaper than a header whose only purpose is this. */
static const char *board_role(void)
{
	return "cal";
}
#else
static const char *board_role(void)
{
	return uwb_config_mode_name(uwb_config_get()->mode);
}
#endif

static int cmd_stats(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t n_rx = 0, n_no_sync = 0, n_bad = 0, n_sent = 0;
	uint32_t n_pub = 0, n_pub_drop = 0;
	uint32_t n_obs_rx = 0, n_obs_drop = 0, n_sub_fail = 0;
	uint32_t d_oversize = 0, d_parse = 0, d_evict = 0;
	uint32_t s_obs = 0, s_reject = 0, s_fix = 0, s_no_anchor = 0;
	uint32_t s_implaus = 0, s_solve_fail = 0, s_jump = 0;
	uint32_t s_dup = 0, s_shed = 0;
	uint32_t e_seeded = 0, e_reseed = 0, e_filtered = 0, e_dt_invalid = 0;
	uint32_t e_gate_rejected = 0, e_no_update = 0, e_zupt = 0;
	uint32_t e_reorder = 0;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	blink_rx_stats(&n_rx, &n_no_sync, &n_bad, &n_sent);
	net_uplink_obs_stats(&n_pub, &n_pub_drop, &n_obs_rx, &n_obs_drop,
			     &n_sub_fail);
	net_uplink_obs_rx_drops(&d_oversize, &d_parse, &d_evict);
	tdoa_gw_stats(&s_obs, &s_reject, &s_fix, &s_no_anchor, &s_implaus,
		      &s_solve_fail, &s_jump);
	tdoa_gw_reject_detail(&s_dup, &s_shed);
	tdoa_gw_ekf_stats(&e_seeded, &e_reseed, &e_filtered, &e_dt_invalid,
			  &e_gate_rejected, &e_no_update, &e_zupt, &e_reorder);

	shell_print(sh,
		    "{\"blink\":{\"role\":\"%s\","
		    "\"rx\":%u,\"no_sync\":%u,\"bad\":%u,\"stamped\":%u,"
		    "\"published\":%u,\"pub_drop\":%u,"
		    "\"received\":%u,\"rx_drop\":%u,"
		    "\"rx_drop_oversize\":%u,\"rx_drop_parse\":%u,"
		    "\"rx_drop_evict\":%u,\"sub_fail\":%u}}",
		    board_role(), n_rx, n_no_sync, n_bad, n_sent,
		    n_pub, n_pub_drop, n_obs_rx, n_obs_drop,
		    d_oversize, d_parse, d_evict, n_sub_fail);

	/* The solve half (Task 6), on the SAME tree deliberately: it is the
	 * downstream end of the very counters above, and splitting it into its
	 * own command would make an operator read two places to tell "no
	 * observations arriving" from "observations arriving and not solving".
	 * Every counter here only ever moves on a GATEWAY -- read the role
	 * field before concluding anything from a line of zeros. */
	shell_print(sh,
		    "{\"tdoa\":{\"role\":\"%s\","
		    "\"ingested\":%u,\"rejected\":%u,\"reject_dup\":%u,"
		    "\"reject_shed\":%u,\"fixes\":%u,\"no_anchor\":%u,"
		    "\"implausible\":%u,\"solve_fail\":%u,\"jump\":%u}}",
		    board_role(), s_obs, s_reject, s_dup, s_shed, s_fix,
		    s_no_anchor, s_implaus, s_solve_fail, s_jump);

	/* The per-tag EKF's own counters (Task 4 of
	 * docs/superpowers/plans/2026-09-02-tdoa-accuracy-filter.md), on their
	 * own line for the same reason the solve half got its own line above:
	 * without this there is no way to tell "no tags" from "the filter
	 * rejects everything". Same role field, same reason. */
	shell_print(sh,
		    "{\"tdoa_ekf\":{\"role\":\"%s\","
		    "\"seeded\":%u,\"reseed\":%u,\"filtered\":%u,"
		    "\"dt_invalid\":%u,\"gate_rejected\":%u,"
		    "\"no_update\":%u,\"zupt\":%u,\"reorder\":%u}}",
		    board_role(), e_seeded, e_reseed, e_filtered,
		    e_dt_invalid, e_gate_rejected, e_no_update, e_zupt,
		    e_reorder);

	/* The one reading an operator cannot get from any other number here.
	 * An anchor still on proto 4 sends no flags field, which parses as 0 =
	 * "not moving", so EVERY filtered cycle applies a ZUPT. That looks like
	 * a healthy stationary fleet and is actually a firmware version
	 * mismatch, so it is called out rather than left to be spotted. */
	if (e_filtered > 0u && e_zupt == e_filtered) {
		shell_warn(sh,
			   "every filtered cycle (%u) applied a zero-velocity "
			   "update. Either nothing is moving, or the anchors are "
			   "older than proto 5 and send no MOVING bit at all - "
			   "which parses as \"still\" and looks identical from "
			   "here. Check the anchors' firmware before trusting a "
			   "still fleet.", e_zupt);
	}

	if (e_reorder > 0u) {
		shell_print(sh,
			    "%u group(s) arrived out of order and were "
			    "discarded rather than stepping the filter "
			    "backwards. Expected in small numbers: the "
			    "collector orders releases by gateway ARRIVAL, "
			    "which interleaved MQTT delivery can still "
			    "invert. A count approaching `filtered` means "
			    "the backhaul is reordering heavily and the "
			    "filter is seeing far less of the data than "
			    "`ingested` suggests.", e_reorder);
	}

	if (e_no_update > 0u) {
		shell_warn(sh,
			   "%u cycle(s) published NOTHING because dt was "
			   "invalid and the solve+seed fallback also failed: "
			   "no stale fix was republished in their place "
			   "(fixed 2026-09-02 - an earlier build would have "
			   "silently republished the filter's unchanged prior "
			   "position as if it were a live fix). A high count "
			   "here alongside a high dt_invalid means marginal "
			   "anchor coverage for that tag, not a bug.",
			   e_no_update);
	}

	if (s_no_anchor > 0u) {
		shell_warn(sh,
			   "%u observation(s) named an anchor that is NOT in "
			   "the applied survey, so they were dropped and can "
			   "never produce a fix. Read `apos show`: an "
			   "unsurveyed gateway solves nothing at all.",
			   s_no_anchor);
	}
	/* Overload shows up UPSTREAM of tdoa_gw, so this verdict reads the
	 * evict counter on the line above, not `reject_shed`. tdoa_gw_step()
	 * ingests at most TDOA_GW_INGEST_MAX per superframe; anything beyond
	 * that never leaves net_uplink's obs_q, which drops the OLDEST when
	 * full. So the collector's slots stay comfortable and `reject_shed`
	 * can sit at zero through a total overload -- the loss is already
	 * counted, as rx_drop_evict, before this module ever sees it. */
	if (d_evict > 0u) {
		shell_warn(sh,
			   "%u observation(s) were EVICTED from the receive "
			   "queue before the gateway could ingest them: this "
			   "board is not draining fast enough for the tag "
			   "count. observations/superframe = anchors x "
			   "blink_rate x tags x 0.2; TDOA_GW_INGEST_MAX (%u) "
			   "and TDOA_GW_SOLVE_MAX (%u) sustain 8 tags at 5 Hz "
			   "over 4 anchors, and OBS_QUEUE_DEPTH caps it there "
			   "too. Re-read the gw_sf heartbeat after raising "
			   "any of them.",
			   d_evict, (unsigned int)TDOA_GW_INGEST_MAX,
			   (unsigned int)TDOA_GW_SOLVE_MAX);
	}
	if (s_shed > 0u) {
		shell_warn(sh,
			   "%u observation(s) were SHED by the collector (NOT "
			   "duplicates): every slot already held a releasable "
			   "group. Rarer than the evict case above and it "
			   "means the same thing - TDOA_GW_SOLVE_MAX is the "
			   "knob, and the gw_sf heartbeat must be re-read "
			   "after touching it.",
			   s_shed);
	}
	if (s_implaus > 0u) {
		shell_warn(sh,
			   "%u blink group(s) failed the physical spread bound "
			   "(TDOA_DTU_MAX_SPREAD, 153.7 m of path difference). "
			   "That is broken clock sync or a corrupt timestamp, "
			   "not a TDoA geometry problem - read `sync stats`.",
			   s_implaus);
	}

	/* The two verdicts worth stating rather than leaving to be spotted.
	 * Both are conditions an operator would otherwise read as "no tags". */
	if (n_rx > 0u && n_no_sync == n_rx) {
		shell_warn(sh,
			   "every BLINK heard was DROPPED for want of a common "
			   "time base — this is the CCP link, not the BLINK "
			   "path. Read `sync stats`, and check the gateway is "
			   "on USB-C: on battery its PA cannot sustain the "
			   "CCP's second transmission per superframe.");
	}
	if (n_sub_fail > 0u) {
		shell_warn(sh,
			   "%u subscription attempt(s) never got a granted "
			   "SUBACK — a gateway in that state is DEAF and hears "
			   "no observation at all. Check the broker ACL for "
			   "the observation topic.",
			   n_sub_fail);
	}

	return 0;
}

#if defined(CONFIG_ANCLA_TDOA_TRACE)
/*
 * TEMPORARY -- Task 7's instrumentation. DELETE with the rest of
 * CONFIG_ANCLA_TDOA_TRACE once the runaway is explained.
 *
 * shell_print and NOT LOG_INF, deliberately and for the same reason the data
 * lives in a ring: the console loses ~80 % of records at 2 fixes/s, and a
 * 128-line dump through the deferred log pool (8192 B in debug.conf) would
 * drop most of itself. Shell output is synchronous and flow-controlled.
 *
 * Read `gs` (gate_streak) first: it is the quantity pos_ekf_needs_reseed()
 * turns on, and the whole question is whether it ever reaches reset_after (3)
 * during an escape. `acc` is how many of the n-1 equations the gate let
 * through -- if that stays >= 1 while x,y walk away, the streak is being
 * reset every cycle and the recovery can never arm, which is the hypothesis.
 */
static int cmd_trace(const struct shell *sh, size_t argc, char **argv)
{
	const struct tdoa_trace_entry *ring = NULL;
	uint32_t n = 0u, dropped = 0u, head = 0u;

	if (argc == 2 && strcmp(argv[1], "clear") == 0) {
		tdoa_gw_trace_clear();
		shell_print(sh, "trace cleared");
		return 0;
	}

	tdoa_gw_trace_snapshot(&ring, &n, &dropped, &head);
	if (ring == NULL || n == 0u) {
		shell_print(sh, "trace empty (role %s -- only a GATEWAY fills "
				"it)", board_role());
		return 0;
	}

	shell_print(sh, "%u entries, %u overwritten%s", n, dropped,
		    dropped ? "  <-- WINDOW TOO SHORT, dump sooner" : "");
	shell_print(sh, "     t_ms  tag  path  n acc gs z r    dt_s      x"
			"      y     vx     vy  sigma");

	/* Oldest first. Once the ring has wrapped, the oldest entry is at the
	 * write cursor; before that it is at 0. Getting this wrong reorders a
	 * trace whose entire value is the time evolution of gate_streak. */
	uint32_t start = (n == TDOA_TRACE_SLOTS) ? head : 0u;
	static const char *const pname[3] = {"filt", "seed", "none"};

	for (uint32_t k = 0; k < n; k++) {
		const struct tdoa_trace_entry *e =
			&ring[(start + k) % TDOA_TRACE_SLOTS];

		shell_print(sh, "%9u %04X  %-4s %2u %3u %2u %1u %1u %7.3f "
				"%6.2f %6.2f %6.2f %6.2f %6.2f",
			    e->t_ms, e->tag_addr,
			    pname[(e->path < 3u) ? e->path : 2u],
			    e->n, e->accepted, e->gate_streak, e->zupt,
			    e->reseed, (double)e->dt_s, (double)e->x,
			    (double)e->y, (double)e->vx, (double)e->vy,
			    (double)e->sigma);
	}
	return 0;
}
#endif /* CONFIG_ANCLA_TDOA_TRACE */

SHELL_STATIC_SUBCMD_SET_CREATE(sub_blink,
	SHELL_CMD_ARG(stats, NULL,
		      "stats — the TDoA path as JSON, in three lines: the "
		      "anchor's stamping counters plus the uplink's "
		      "publish/subscribe counters, then the gateway's "
		      "ingest/solve/publish counters, then the per-tag EKF's "
		      "own counters. All three carry a role field",
		      cmd_stats, 1, 0),
#if defined(CONFIG_ANCLA_TDOA_TRACE)
	SHELL_CMD_ARG(trace, NULL,
		      "trace [clear] — TEMPORARY per-cycle filter trace ring "
		      "(Task 7). Read gate_streak (gs) against acc: a streak "
		      "stuck at 0 while acc stays >= 1 is the runaway "
		      "hypothesis",
		      cmd_trace, 1, 1),
#endif
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(blink, &sub_blink, "TDoA observation path", NULL);
