/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `sync` command tree: the Phase 2 gate, read off a console.
 *
 * The whole point is that an operator reads ONE number and gets a verdict,
 * because the natural mistake here is expensive and silent. sync_model.h spells
 * it out: the residual RMS is NOT the jitter -- the prediction consumes two
 * noisy local timestamps, so the RMS sits at about 1.55x the real jitter. An
 * operator reading 64 DTU of RMS as 1 ns would be looking at ~0.65 ns and
 * REJECTING hardware that passes, killing the TDoA migration on a
 * misinterpretation. So this command prints jitter_est first, prints the
 * verdict itself, and prints rms only as a diagnostic beside it.
 *
 * This tree is registered UNCONDITIONALLY -- in the production image on
 * every role, AND in the calibration image (CONFIG_ANCLA_CAL_MODE; see
 * CLAUDE.md's Build & flash section -- ccp_slave.c/ccp_master.c compile into
 * every image regardless of that config, only cal_run.c's loop never calls
 * either module's init or per-superframe entry point). `sync stats` reads the
 * receive half, meaningful only on a SLAVE; `sync master` below reads the
 * transmit half, meaningful only on a GATEWAY. On every OTHER role -- a
 * GATEWAY reading `sync stats`, a SLAVE reading `sync master`, or the cal
 * image reading either -- the underlying counters simply never moved, and
 * `root:0, rx:0, valid:0, verdict:"no-lock"` / `sent:0, dropped:0` is
 * indistinguishable on its own from a dead link. Both commands print a `role`
 * field for exactly that reason: read it before concluding anything about the
 * number next to it.
 */

#include "ccp_master.h"
#include "ccp_slave.h"
#include "gw_selftau.h"
#include "sync_model.h"
#include "uwb_config.h"
#include "uwb_store.h"

#include <zephyr/shell/shell.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#ifdef CONFIG_ANCLA_CAL_MODE
/* The cal image runs cal_run()'s own loop, never uwb_slave_run() or
 * uwb_gateway_run() -- so cfg->mode (persisted NVS state, unrelated to what
 * actually executes here) would be actively misleading if reported as this
 * board's role. Say what it really is instead. */
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

/* Thresholds from docs/anchor-sync-measurement.md section 4. 64 DTU = 1 ns
 * (SYNC_DTU_PER_NS), 32 DTU = 0.5 ns. */
#define SYNC_GATE_PASS_DTU      32u
#define SYNC_GATE_FAIL_DTU      64u

/* The statistic is an RMS and a short sample is a noisy one. The simulation in
 * tests/sync_model uses 400 observations and section 3 of the doc asks the
 * operator for the same, so refuse to render a verdict below it rather than
 * letting a 20-sample reading look authoritative. */
#define SYNC_GATE_MIN_COUNT     400u

static const char *verdict_of(uint32_t jitter_dtu, uint32_t count, bool valid)
{
	if (!valid) {
		return "no-lock";
	}
	if (count < SYNC_GATE_MIN_COUNT) {
		return "insufficient";
	}
	if (jitter_dtu < SYNC_GATE_PASS_DTU) {
		return "pass";
	}
	if (jitter_dtu <= SYNC_GATE_FAIL_DTU) {
		return "marginal";
	}
	return "fail";
}

static int cmd_stats(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct sync_model *m = ccp_slave_model();
	uint32_t rx = 0, gap = 0, reject = 0, root = 0;
	uint32_t paired = 0, no_announce = 0;

	ccp_slave_stats(&rx, &gap, &reject, &root);
	ccp_slave_stats_ex(&paired, &no_announce);

	/* sync_model_error_dtu() returns UINT32_MAX when the model has no rate
	 * estimate. That is the documented public way to ask, rather than
	 * reaching into the struct's `valid` field. */
	bool valid = sync_model_error_dtu(m, 0u) != UINT32_MAX;

	uint32_t jit = sync_model_jitter_est_dtu(m);
	uint32_t cnt = sync_model_residual_count(m);

	/* 1 DTU = 15.65 ps. Printed because nobody reasons in DTU. */
	uint32_t jit_ps = (uint32_t)(((uint64_t)jit * 1565u) / 100u);

	/* The B13 trio. `n_obs` is the CURRENT baseline's length, so it reads 0
	 * straight after a discontinuity and climbs to SYNC_BASELINE_USEFUL;
	 * `rebaselines` and `sync_gaps` are lifetime counters that no
	 * re-baseline clears. Without these the transient is invisible: every
	 * other number on this line looks identical during it. */
	uint32_t n_obs       = sync_model_n_obs(m);
	uint32_t rebaselines = sync_model_rebaseline_count(m);

	/* The DYNAMIC sigma -- what this anchor actually publishes with each
	 * blink observation, over a full CCP interval of extrapolation. Unlike
	 * jitter_est (a long-run link statistic that does not move during a
	 * transient) this rises after a re-baseline and falls as the baseline
	 * converges, which is the whole point of surfacing it here next to
	 * n_obs. */
	uint32_t sigma = sync_model_sigma_dtu(m);

	shell_print(sh,
		    "{\"sync\":{\"role\":\"%s\",\"root\":%u,\"rx\":%u,"
		    "\"paired\":%u,\"no_announce\":%u,\"gaps\":%u,"
		    "\"rejected\":%u,\"count\":%u,\"valid\":%u,"
		    "\"n_obs\":%u,\"rebaselines\":%u,\"sync_gaps\":%u,"
		    "\"sigma_dtu\":%u,\"sigma_ps\":%u,"
		    "\"jitter_est_dtu\":%u,\"jitter_est_ps\":%u,"
		    "\"rms_dtu\":%u,\"max_dtu\":%u,\"drift_ppb\":%d,"
		    "\"verdict\":\"%s\"}}",
		    board_role(), root, rx, paired, no_announce, gap, reject,
		    cnt, valid ? 1u : 0u,
		    n_obs, rebaselines, sync_model_miss_count(m),
		    sigma, (uint32_t)(((uint64_t)sigma * 1565u) / 100u),
		    jit, jit_ps,
		    sync_model_residual_rms_dtu(m),
		    sync_model_residual_max_dtu(m),
		    sync_model_drift_ppb(m),
		    verdict_of(jit, cnt, valid));

	if (valid && n_obs < SYNC_BASELINE_USEFUL) {
		shell_warn(sh,
			   "n_obs is %u of %u — this anchor is mid-re-baseline "
			   "and is converting on its CARRIED skew, with "
			   "sigma_dtu raised to say so. Transient, not a "
			   "fault; if it persists, `rebaselines` and `gaps` "
			   "say which discontinuity keeps firing.",
			   n_obs, SYNC_BASELINE_USEFUL);
	} else if (!valid && rx > 0u && rebaselines > 0u) {
		shell_warn(sh,
			   "rx is climbing but this model cannot convert: it "
			   "has re-baselined %u time(s) and the current "
			   "baseline is %u of %u, with no carried skew young "
			   "enough to bridge it. This anchor is publishing NO "
			   "observations right now — correct behaviour after a "
			   "gateway reboot, and it clears itself within ~2 s.",
			   rebaselines, n_obs, SYNC_BASELINE_USEFUL);
	}

	if (valid && cnt >= SYNC_GATE_MIN_COUNT) {
		uint32_t mx = sync_model_residual_max_dtu(m);
		uint32_t rms = sync_model_residual_rms_dtu(m);

		/* Section 4's cross-check, applied rather than left to the
		 * operator: a max far above the RMS means the distribution has
		 * a tail, which is multipath or an intermittent obstruction --
		 * not clock noise. Fix the setup and re-measure before
		 * believing the verdict. */
		if (rms > 0u && mx > 5u * rms) {
			shell_warn(sh,
				   "max is %ux the RMS — that is a TAIL, not "
				   "Gaussian clock noise. Almost certainly "
				   "multipath or an intermittent obstruction. "
				   "Fix the setup and re-measure before "
				   "trusting the verdict.",
				   mx / rms);
		}
	}

	shell_print(sh,
		    "read jitter_est, NOT rms: the residual differences two "
		    "noisy timestamps and its RMS is ~1.55x the real jitter. "
		    "Thresholds: <%u DTU pass, <=%u marginal, above that fail. "
		    "rx/paired/no_announce/gaps/rejected/valid are only "
		    "meaningful on a SLAVE -- a GATEWAY or the cal image never "
		    "feed this model, so root:0 rx:0 valid:0 \"no-lock\" there "
		    "means \"not a slave\", not \"dead link\". See \"role\" "
		    "above. rx climbing while count stays 0: check "
		    "no_announce first (a freshly-rebooted or "
		    "txtest-only master looks exactly like this by design, "
		    "not a fault) before suspecting the pairing logic.",
		    SYNC_GATE_PASS_DTU, SYNC_GATE_FAIL_DTU);
	return 0;
}

static int cmd_master(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint32_t sent = 0, dropped = 0, root = 0;
	int32_t offset_min = 0, offset_max = 0, offset_last = 0;
	int32_t arm_cost_last = 0;
	bool tt_pending = false, tt_done = false, tt_tx_ok = false, tt_txfrs_ok = false;

	ccp_master_stats(&sent, &dropped, &root, &offset_min, &offset_max,
			 &offset_last, &arm_cost_last);
	ccp_master_txtest_stats(&tt_pending, &tt_done, &tt_tx_ok, &tt_txfrs_ok);

	shell_print(sh,
		    "{\"ccp_master\":{\"role\":\"%s\",\"root\":%u,"
		    "\"sent\":%u,\"dropped\":%u,"
		    "\"offset_ns_min\":%d,\"offset_ns_max\":%d,"
		    "\"offset_ns_last\":%d,\"arm_cost_ns_last\":%d,"
		    "\"txtest\":{\"pending\":%d,\"done\":%d,\"tx_ok\":%d,"
		    "\"txfrs_ok\":%d}}}",
		    board_role(), root, sent, dropped,
		    (int)offset_min, (int)offset_max, (int)offset_last,
		    (int)arm_cost_last,
		    (int)tt_pending, (int)tt_done, (int)tt_tx_ok,
		    (int)tt_txfrs_ok);

	if (strcmp(board_role(), "gateway") != 0) {
		shell_warn(sh,
			   "sent/dropped are only meaningful on a GATEWAY -- "
			   "ccp_master_after_beacon() is called from nowhere "
			   "else, so sent:0 dropped:0 here means \"not a "
			   "gateway\", not \"the link is fine\".");
	} else if (dropped != 0u) {
		/* Makes a real risk readable rather than silent: a dropped CCP
		 * costs the receiver both an observation AND the following
		 * CCP's ability to make one (the deferred-timestamp design
		 * only announces a CONFIRMED transmit's timestamp), so
		 * `sync stats` on the peer can sit well short of `rx` even
		 * on a mostly-healthy link. This is the first place that says
		 * why. */
		shell_warn(sh,
			   "%u of %u CCPs dropped. If this keeps climbing, "
			   "check docs/anchor-sync-measurement.md section 5 "
			   "item \"there is no number at all\" before "
			   "suspecting the RF link.",
			   dropped, sent + dropped);
	}
	return 0;
}

static int cmd_txtest(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* Gateway mode only -- the same refusal pattern as the apos tree.
	 * ccp_master_after_beacon()'s per-superframe CCP only ever runs from
	 * uwb_gateway_run()'s loop, and this test rides the exact same
	 * mechanism (a flag consumed by that loop), so it means nothing
	 * anywhere else. */
	if (strcmp(board_role(), "gateway") != 0) {
		shell_error(sh, "sync txtest: gateway mode only (this board is "
				"\"%s\")", board_role());
		return -ENOTSUP;
	}

	/* BENCH DIAGNOSTIC ONLY: an immediate TX lands at an arbitrary time
	 * within the superframe, wherever the gateway loop happens to be when
	 * it consumes this request -- it can collide with beacon, GRANT, or
	 * ranging traffic on a live network. Never use this on a deployed
	 * gateway.
	 *
	 * This only SETS A FLAG -- see ccp_master.h. The actual transmit runs
	 * on the gateway loop, on its next iteration (i.e. after the current
	 * superframe's beacon and CCP), never from this shell thread. Check
	 * the result with `sync master` afterwards; \"pending\":1 means the
	 * gateway loop has not consumed it yet. */
	ccp_master_request_txtest();
	shell_print(sh,
		    "queued one immediate-TX CCP (BENCH DIAGNOSTIC — not for a "
		    "live deployment). The gateway loop transmits it on its "
		    "next iteration; run `sync master` afterwards for the "
		    "result (see its \"txtest\" field).");
	return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ccp_slave_residual_reset();
	shell_print(sh, "residual statistics cleared — the rate estimate and "
			"its baseline are NOT touched, so `count` restarts "
			"while the lock is kept");
	return 0;
}

/* Re-apply uwb_config's resolved tau to the live model. Shared by `tau mode`
 * and `tau set` so a change from either one takes effect immediately
 * (FR-5) instead of waiting for the next reboot -- deliberately NOT the
 * snapshot-at-mode-entry contract uwb_slave_run() otherwise applies to
 * ant_delay and friends. */
static void apply_tau_live(void)
{
	ccp_slave_set_tau(uwb_config_sync_tau_effective(uwb_config_get()));
}

static int cmd_tau_mode(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	uwb_config_t *cfg = uwb_config_get();
	uint8_t mode;

	if (!uwb_config_sync_tau_mode_from_name(argv[1], &mode)) {
		shell_error(sh, "sync tau mode: unknown mode \"%s\" (want "
				"legacy|survey|twr|auto)", argv[1]);
		return -EINVAL;
	}
	uwb_config_set_sync_tau_mode(cfg, mode);
	if (uwb_store_save_sync_tau_mode()) {
		shell_warn(sh, "mode applied but NOT persisted — will revert "
				"to \"%s\" on reboot",
			   uwb_config_sync_tau_mode_name(cfg->sync_tau_mode));
	}
	apply_tau_live();
	shell_print(sh, "{\"sync_tau\":{\"mode\":\"%s\",\"tau_used_dtu\":%d}}",
		    uwb_config_sync_tau_mode_name(cfg->sync_tau_mode),
		    ccp_slave_get_tau());
	if (mode == SYNC_TAU_MODE_TWR || mode == SYNC_TAU_MODE_AUTO) {
		shell_warn(sh, "Fase B (on-air TWR measurement) is not "
				"implemented on this build — \"%s\" resolves "
				"to the stored SURVEY value (tau_survey_dtu), "
				"same as \"survey\".",
			   uwb_config_sync_tau_mode_name(mode));
	}
	return 0;
}

static int cmd_tau_set(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	uwb_config_t *cfg = uwb_config_get();
	char *endp = NULL;
	long v = strtol(argv[1], &endp, 10);

	if (endp == argv[1] || *endp != '\0' || v < INT32_MIN || v > INT32_MAX) {
		shell_error(sh, "sync tau set: \"%s\" is not a valid signed "
				"DTU value", argv[1]);
		return -EINVAL;
	}
	uwb_config_set_sync_tau(cfg, (int32_t)v);
	if (uwb_store_save_sync_tau()) {
		shell_warn(sh, "tau_survey_dtu applied but NOT persisted — "
				"will revert to %d on reboot",
			   cfg->sync_tau_dtu);
	}
	apply_tau_live();
	shell_print(sh, "{\"sync_tau\":{\"tau_survey_dtu\":%d,\"mode\":\"%s\","
			"\"tau_used_dtu\":%d}}",
		    cfg->sync_tau_dtu,
		    uwb_config_sync_tau_mode_name(cfg->sync_tau_mode),
		    ccp_slave_get_tau());
	if (cfg->sync_tau_mode == SYNC_TAU_MODE_LEGACY) {
		shell_warn(sh, "mode is \"legacy\" — this value is stored but "
				"has NO effect until `sync tau mode survey` "
				"(or twr/auto).");
	}
	return 0;
}

static int cmd_tau_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const uwb_config_t *cfg = uwb_config_get();

	/* tau_twr_* fields are wire-compatible placeholders for FR-6..FR-9
	 * (Fase B): always "no source yet" on this build, since no on-air
	 * measurement exists to report. tau_used_dtu is read from the LIVE
	 * model (ccp_slave_get_tau()), not recomputed from cfg here, so this
	 * command shows what is actually being applied, not merely what is
	 * configured -- the two can differ for one superframe if a caller
	 * changed cfg without going through this file's apply_tau_live(). */
	shell_print(sh,
		    "{\"sync_tau\":{\"mode\":\"%s\",\"tau_survey_dtu\":%d,"
		    "\"tau_used_dtu\":%d,\"tau_twr_dtu\":null,"
		    "\"tau_twr_age_s\":null,\"twr_quality\":null,"
		    "\"tau_jump_events\":0}}",
		    uwb_config_sync_tau_mode_name(cfg->sync_tau_mode),
		    cfg->sync_tau_dtu, ccp_slave_get_tau());
	return 0;
}

/* strtol(), not the config-layer helpers: `id` and `n` here are console
 * arguments to THIS gateway-side command, not uwb_config_t fields. `endp ==
 * arg` (empty/non-numeric) and a trailing non-NUL both reject, same
 * discipline cmd_tau_set() above already applies. */
static bool parse_arg(const char *arg, long *out)
{
	char *endp = NULL;
	long v = strtol(arg, &endp, 10);

	if (endp == arg || *endp != '\0') {
		return false;
	}
	*out = v;
	return true;
}

static int cmd_tau_measure(const struct shell *sh, size_t argc, char **argv)
{
	long id, n = GW_SELFTAU_DEFAULT_N;

	/* Same refusal pattern as apos_shell.c and `sync txtest`: this rides
	 * the gateway's own K_PRIO_COOP(0) loop (gw_selftau_step(), called
	 * from uwb_gateway_run()) and means nothing anywhere else -- a SLAVE
	 * has no ss_initiator caller wired to consume the request at all, so
	 * it would sit "pending" forever with nothing to report. */
	if (strcmp(board_role(), "gateway") != 0) {
		shell_error(sh, "sync tau measure: gateway mode only (this "
				"board is \"%s\")", board_role());
		return -ENOTSUP;
	}

	if (!parse_arg(argv[1], &id) || id < 0 || id >= UWB_MAX_ANCHORS) {
		shell_error(sh, "sync tau measure: id must be 0..%u",
			    UWB_MAX_ANCHORS - 1);
		return -EINVAL;
	}
	if (argc > 2) {
		if (!parse_arg(argv[2], &n) || n < 1 || n > GW_SELFTAU_MAX_N) {
			shell_error(sh, "sync tau measure: n must be 1..%u",
				    GW_SELFTAU_MAX_N);
			return -EINVAL;
		}
	}

	if (!gw_selftau_request((uint8_t)id, (uint32_t)n)) {
		shell_error(sh, "sync tau measure: a request is already in "
				"progress -- check `sync tau result` first");
		return -EBUSY;
	}

	shell_print(sh, "queued: %ld exchange(s) against anchor %ld, spread "
			"over up to %ld superframes (~%ld ms) so the beacon "
			"is never held for more than one bounded exchange at "
			"a time. Read the result with `sync tau result`.",
		    n, id, n, n * 200L);
	return 0;
}

static int cmd_tau_result(const struct shell *sh, size_t argc, char **argv)
{
	struct gw_selftau_result r;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (strcmp(board_role(), "gateway") != 0) {
		shell_error(sh, "sync tau result: gateway mode only (this "
				"board is \"%s\")", board_role());
		return -ENOTSUP;
	}

	gw_selftau_get_result(&r);
	shell_print(sh,
		    "{\"gw_selftau\":{\"anchor_id\":%u,\"done\":%u,"
		    "\"attempted\":%u,\"valid\":%u,\"mean_mm\":%d,"
		    "\"min_mm\":%d,\"max_mm\":%d,\"tau_dtu\":%d}}",
		    r.anchor_id, r.done ? 1u : 0u, r.attempted, r.valid,
		    r.mean_mm, r.min_mm, r.max_mm, r.tau_dtu);

	if (!r.done) {
		shell_print(sh, "still running (%u of the requested "
				"exchanges so far) -- try again shortly",
			    r.attempted);
	} else if (r.valid == 0u) {
		shell_warn(sh, "0 of %u exchanges got a response -- check the "
				"anchor is powered, positioned "
				"(`anchor show`'s pos_valid) and in range "
				"before trusting tau_dtu:0",
			   r.attempted);
	} else if (r.valid < r.attempted) {
		shell_warn(sh, "only %u of %u exchanges succeeded -- tau_dtu "
				"is a mean over the %u that did, treat it "
				"with more caution than a clean run",
			   r.valid, r.attempted, r.valid);
	}

	/* Caught in the field 2026-09-23: a plain arithmetic mean has no
	 * outlier rejection, and ss_initiator_range() only returns
	 * INT32_MIN on a frame that never arrived -- a frame that arrived
	 * but carries a MULTIPATH-corrupted distance still counts as
	 * "valid" and drags the mean with it. One anchor read min/max/mean
	 * 1920/2757/2578 mm (837 mm spread) against three clean anchors at
	 * 25-63 mm, and the contaminated mean produced a tau_dtu visibly
	 * off from the clean cluster. 20% of the mean is a deliberately
	 * generous threshold -- every clean anchor observed so far sits
	 * under 6% -- same "catches the gross case, not a tuned
	 * sync-quality gate" philosophy as TDOA_DTU_MAX_SPREAD. */
	if (r.done && r.valid > 0u &&
	    (int64_t)(r.max_mm - r.min_mm) * 5 > (int64_t)r.mean_mm) {
		shell_warn(sh,
			   "spread is %d mm on a %d mm mean (%d..%d) -- that "
			   "is NOT ordinary link jitter (compare the other "
			   "anchors' spreads), it looks like one or more "
			   "exchanges hit multipath/a reflection. tau_dtu "
			   "may be dragged well off the true value. Check "
			   "line-of-sight to this anchor and re-run with a "
			   "larger n before trusting this result.",
			   r.max_mm - r.min_mm, r.mean_mm, r.min_mm, r.max_mm);
	}

	if (r.done && r.valid == r.attempted && r.valid > 0u) {
		shell_print(sh, "copy `sync tau set %d` into anchor %u's OWN "
				"console -- this value is not applied here, "
				"it describes THAT anchor's distance from "
				"this gateway",
			    r.tau_dtu, r.anchor_id);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sync_tau,
	SHELL_CMD_ARG(mode, NULL,
		      "mode <legacy|survey|twr|auto> — select the B14 tau "
		      "source, persist, apply live",
		      cmd_tau_mode, 2, 0),
	SHELL_CMD_ARG(set, NULL,
		      "set <dtu> — write tau_survey_dtu (signed DTU, from the "
		      "survey tool), persist, apply live",
		      cmd_tau_set, 2, 0),
	SHELL_CMD_ARG(show, NULL,
		      "show — mode, configured and live tau as JSON",
		      cmd_tau_show, 1, 0),
	SHELL_CMD_ARG(measure, NULL,
		      "measure <id> [n] — GATEWAY-only: range this gateway "
		      "directly to anchor <id> over SS-TWR, averaged over n "
		      "exchanges (default 8, max 32), and compute tau_dtu "
		      "with no coordinates or tape measurement needed. "
		      "Queues; read the result with `sync tau result`.",
		      cmd_tau_measure, 2, 1),
	SHELL_CMD_ARG(result, NULL,
		      "result — the last `sync tau measure` run, as JSON",
		      cmd_tau_result, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sync,
	SHELL_CMD_ARG(stats, NULL,
		      "stats — the Phase 2 gate as JSON, with a verdict "
		      "(receive half; meaningful on a SLAVE)",
		      cmd_stats, 1, 0),
	SHELL_CMD_ARG(reset, NULL,
		      "reset — clear the residual statistics, keeping the lock",
		      cmd_reset, 1, 0),
	SHELL_CMD_ARG(master, NULL,
		      "master — CCP transmit stats as JSON (meaningful on a "
		      "GATEWAY)",
		      cmd_master, 1, 0),
	SHELL_CMD_ARG(txtest, NULL,
		      "txtest — BENCH DIAGNOSTIC: queue one immediate-TX CCP "
		      "(GATEWAY only); read the result with `sync master`",
		      cmd_txtest, 1, 0),
	SHELL_CMD(tau, &sub_sync_tau,
		  "tau — B14 master->anchor flight-time compensation "
		  "(mode/set/show)",
		  NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sync, &sub_sync, "Anchor clock synchronisation", NULL);
