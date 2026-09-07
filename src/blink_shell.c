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
#include "pos_abg.h"
#include "tdoa_gw.h"

#include "uwb_config.h"

#include <zephyr/shell/shell.h>

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

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
	uint32_t s_dup = 0, s_shed = 0, s_reorder = 0;
	uint32_t a_seeded = 0, a_dt_reseed = 0, a_filtered = 0;
	uint32_t a_gate = 0, a_reseed = 0, a_still = 0;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	blink_rx_stats(&n_rx, &n_no_sync, &n_bad, &n_sent);
	net_uplink_obs_stats(&n_pub, &n_pub_drop, &n_obs_rx, &n_obs_drop,
			     &n_sub_fail);
	net_uplink_obs_rx_drops(&d_oversize, &d_parse, &d_evict);
	tdoa_gw_stats(&s_obs, &s_reject, &s_fix, &s_no_anchor, &s_implaus,
		      &s_solve_fail, &s_jump);
	tdoa_gw_reject_detail(&s_dup, &s_shed);
	s_reorder = tdoa_gw_reorder_count();
	tdoa_gw_abg_stats(&a_seeded, &a_dt_reseed, &a_filtered, &a_gate,
			  &a_reseed, &a_still);

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
		    "\"implausible\":%u,\"solve_fail\":%u,\"jump\":%u,"
		    "\"reorder\":%u}}",
		    board_role(), s_obs, s_reject, s_dup, s_shed, s_fix,
		    s_no_anchor, s_implaus, s_solve_fail, s_jump, s_reorder);

	/* The filter's own counters (spec §4.4). Its own line for the same
	 * reason the solve half got one: without it there is no way to tell
	 * "no tags" from "the filter rejects everything". Publish identity:
	 * fixes == seeded + dt_reseed + filtered + reseed. */
	shell_print(sh,
		    "{\"tdoa_abg\":{\"role\":\"%s\","
		    "\"seeded\":%u,\"dt_reseed\":%u,\"filtered\":%u,"
		    "\"gate_rejected\":%u,\"reseed\":%u,\"still\":%u}}",
		    board_role(), a_seeded, a_dt_reseed, a_filtered,
		    a_gate, a_reseed, a_still);

	/* The proto-4 trap: an anchor older than proto 5 sends no flags
	 * field, which parses as 0 = "not moving", so EVERY filtered cycle
	 * takes the still branch. That looks like a healthy stationary fleet
	 * and is a firmware version mismatch. */
	if (a_filtered > 0u && a_still == a_filtered) {
		shell_warn(sh,
			   "every filtered cycle (%u) took the STILL branch. "
			   "Either nothing is moving, or the anchors are older "
			   "than proto 5 and send no MOVING bit - which parses "
			   "as \"still\" and looks identical from here. Check "
			   "the anchors' firmware before trusting a still "
			   "fleet.", a_still);
	}
	if (a_filtered > 0u && a_gate > a_filtered / 4u) {
		shell_warn(sh,
			   "the innovation gate rejected %u cycle(s) against "
			   "%u filtered - a quarter or more of the evidence. "
			   "Either gate_m (%.2f m) is too tight for this "
			   "site's raw dispersion, or the solve is producing "
			   "outliers at a rate no filter should hide: read "
			   "`implausible`, `jump` and `sync stats` before "
			   "loosening the gate.", a_gate, a_filtered,
			   (double)tdoa_gw_abg_cfg()->gate_m);
	}

	if (s_reorder > 0u) {
		shell_print(sh,
			    "%u group(s) arrived out of order and were "
			    "discarded rather than published as a fix that "
			    "steps backwards in time. Expected in small "
			    "numbers: the collector orders releases by gateway "
			    "ARRIVAL, which interleaved MQTT delivery can still "
			    "invert. A count approaching `fixes` means the "
			    "backhaul is reordering heavily and the published "
			    "stream carries far less of the data than "
			    "`ingested` suggests.", s_reorder);
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

/* `blink abg [lambda|gamma|gate|still|reset <value>]` -- runtime tuning of the
 * position filter for the visual lambda sweep (spec §4.5). NOT persisted: a
 * reboot returns to pos_abg_cfg_defaults(), and a value the maintainer picks
 * goes into those defaults, in source, where it can be seen. Refuses on a
 * non-gateway role for the same reason `apos` does: the filter only runs
 * there, and a setting on a SLAVE would be a value nobody can observe. */
static int cmd_abg(const struct shell *sh, size_t argc, char **argv)
{
	const struct pos_abg_cfg *c = tdoa_gw_abg_cfg();

	if (argc == 1) {
		shell_print(sh,
			    "{\"abg\":{\"role\":\"%s\",\"lambda\":%.4f,"
			    "\"alpha\":%.4f,\"beta\":%.4f,\"gamma\":%.5f,"
			    "\"alpha_still\":%.3f,\"gate_m\":%.2f,"
			    "\"reset_after\":%u}}",
			    board_role(), (double)tdoa_gw_abg_lambda(),
			    (double)c->alpha, (double)c->beta,
			    (double)c->gamma, (double)c->alpha_still,
			    (double)c->gate_m, (unsigned int)c->reset_after);
		return 0;
	}
	if (argc != 3) {
		shell_error(sh, "usage: blink abg [lambda|gamma|gate|still|reset "
				"<value>]");
		return -EINVAL;
	}
#ifndef CONFIG_ANCLA_CAL_MODE
	if (uwb_config_get()->mode != UWB_MODE_GATEWAY) {
		shell_error(sh, "error: the position filter runs on the "
				"GATEWAY - this board is a %s", board_role());
		return -EPERM;
	}
#endif

	char *end = NULL;
	/* strtof, not strtod: same choice apos_shell.c's cmd_zoff() makes. */
	float v = strtof(argv[2], &end);

	if (end == argv[2] || *end != '\0') {
		shell_error(sh, "error: \"%s\" is not a number", argv[2]);
		return -EINVAL;
	}

	if (strcmp(argv[1], "lambda") == 0) {
		if (!(v > 0.0f) || v > 1.0f) {
			shell_error(sh, "error: lambda must be in (0, 1]; "
					"0.005..0.5 is the sensible range");
			return -EINVAL;
		}
		tdoa_gw_abg_set_lambda(v);
	} else if (strcmp(argv[1], "gamma") == 0) {
		if (v < 0.0f || v > 1.0f) {
			shell_error(sh, "error: gamma must be in [0, 1]; 0 makes "
					"the filter alpha-beta");
			return -EINVAL;
		}
		tdoa_gw_abg_set_gamma(v);
	} else if (strcmp(argv[1], "gate") == 0) {
		if (!(v > 0.0f)) {
			shell_error(sh, "error: gate must be > 0 m");
			return -EINVAL;
		}
		tdoa_gw_abg_set_gate(v);
	} else if (strcmp(argv[1], "still") == 0) {
		if (!(v > 0.0f) || v > 1.0f) {
			shell_error(sh, "error: still alpha must be in (0, 1]");
			return -EINVAL;
		}
		tdoa_gw_abg_set_alpha_still(v);
	} else if (strcmp(argv[1], "reset") == 0) {
		if (v < 0.0f || v > 255.0f || v != (float)(int)v) {
			shell_error(sh, "error: reset must be an integer "
					"0..255 (0 = never reseed on the "
					"streak)");
			return -EINVAL;
		}
		tdoa_gw_abg_set_reset_after((uint8_t)v);
	} else {
		shell_error(sh, "error: unknown field \"%s\" (lambda|gamma|"
				"gate|still|reset)", argv[1]);
		return -EINVAL;
	}
	shell_print(sh, "applied to every tag's filter on the next cycle. NOT "
			"persisted: a reboot returns to the compiled-in "
			"defaults");
	return cmd_abg(sh, 1, argv);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_blink,
	SHELL_CMD_ARG(stats, NULL,
		      "stats — the TDoA path as JSON, in three lines: the "
		      "anchor's stamping counters plus the uplink's "
		      "publish/subscribe counters, then the gateway's "
		      "ingest/solve/publish counters, then "
		      "the position filter's own counters. All three carry a role field",
		      cmd_stats, 1, 0),
	SHELL_CMD_ARG(abg, NULL,
		      "abg [lambda|gamma|gate|still|reset <value>] — read or "
		      "tune the alpha-beta-gamma position filter at runtime "
		      "(gamma 0 = alpha-beta). NOT persisted; GATEWAY only "
		      "for the setters",
		      cmd_abg, 1, 2),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(blink, &sub_blink, "TDoA observation path", NULL);
