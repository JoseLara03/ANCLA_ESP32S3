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

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	blink_rx_stats(&n_rx, &n_no_sync, &n_bad, &n_sent);
	net_uplink_obs_stats(&n_pub, &n_pub_drop, &n_obs_rx, &n_obs_drop,
			     &n_sub_fail);
	net_uplink_obs_rx_drops(&d_oversize, &d_parse, &d_evict);
	tdoa_gw_stats(&s_obs, &s_reject, &s_fix, &s_no_anchor, &s_implaus,
		      &s_solve_fail, &s_jump);
	tdoa_gw_reject_detail(&s_dup, &s_shed);

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

	if (s_no_anchor > 0u) {
		shell_warn(sh,
			   "%u observation(s) named an anchor that is NOT in "
			   "the applied survey, so they were dropped and can "
			   "never produce a fix. Read `apos show`: an "
			   "unsurveyed gateway solves nothing at all.",
			   s_no_anchor);
	}
	if (s_shed > 0u) {
		shell_warn(sh,
			   "%u observation(s) were SHED, not duplicated: every "
			   "collector slot already held a releasable group, so "
			   "this gateway is not draining fast enough. "
			   "TDOA_GW_SOLVE_MAX is the knob, and the gw_sf "
			   "heartbeat must be re-read after touching it.",
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

SHELL_STATIC_SUBCMD_SET_CREATE(sub_blink,
	SHELL_CMD_ARG(stats, NULL,
		      "stats — the TDoA path as JSON, in two lines: the "
		      "anchor's stamping counters plus the uplink's "
		      "publish/subscribe counters, then the gateway's "
		      "ingest/solve/publish counters. Both carry a role field",
		      cmd_stats, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(blink, &sub_blink, "TDoA observation path", NULL);
