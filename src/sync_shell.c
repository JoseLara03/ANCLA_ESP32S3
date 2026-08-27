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
#include "sync_model.h"
#include "uwb_config.h"

#include <zephyr/shell/shell.h>

#include <errno.h>
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

	ccp_slave_stats(&rx, &gap, &reject, &root);

	/* sync_model_error_dtu() returns UINT32_MAX when the model has no rate
	 * estimate. That is the documented public way to ask, rather than
	 * reaching into the struct's `valid` field. */
	bool valid = sync_model_error_dtu(m, 0u) != UINT32_MAX;

	uint32_t jit = sync_model_jitter_est_dtu(m);
	uint32_t cnt = sync_model_residual_count(m);

	/* 1 DTU = 15.65 ps. Printed because nobody reasons in DTU. */
	uint32_t jit_ps = (uint32_t)(((uint64_t)jit * 1565u) / 100u);

	shell_print(sh,
		    "{\"sync\":{\"role\":\"%s\",\"root\":%u,\"rx\":%u,\"gaps\":%u,"
		    "\"rejected\":%u,\"count\":%u,\"valid\":%u,"
		    "\"jitter_est_dtu\":%u,\"jitter_est_ps\":%u,"
		    "\"rms_dtu\":%u,\"max_dtu\":%u,\"drift_ppb\":%d,"
		    "\"verdict\":\"%s\"}}",
		    board_role(), root, rx, gap, reject, cnt, valid ? 1u : 0u,
		    jit, jit_ps,
		    sync_model_residual_rms_dtu(m),
		    sync_model_residual_max_dtu(m),
		    sync_model_drift_ppb(m),
		    verdict_of(jit, cnt, valid));

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
		    "rx/gaps/rejected/valid are only meaningful on a SLAVE -- "
		    "a GATEWAY or the cal image never feed this model, so "
		    "root:0 rx:0 valid:0 \"no-lock\" there means \"not a "
		    "slave\", not \"dead link\". See \"role\" above.",
		    SYNC_GATE_PASS_DTU, SYNC_GATE_FAIL_DTU);
	return 0;
}

static int cmd_master(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint32_t sent = 0, dropped = 0, root = 0;
	int32_t late_min = 0, late_max = 0, late_last = 0;
	int32_t late_signed_last = 0, late_signed_min = 0;
	uint32_t sys_status_lo = 0;
	bool hpdwarn_seen = false;
	bool tt_pending = false, tt_done = false, tt_tx_ok = false, tt_txfrs_ok = false;

	ccp_master_stats(&sent, &dropped, &root, &late_min, &late_max,
			 &late_last);
	ccp_master_diag_stats(&late_signed_last, &late_signed_min,
			      &sys_status_lo, &hpdwarn_seen);
	ccp_master_txtest_stats(&tt_pending, &tt_done, &tt_tx_ok, &tt_txfrs_ok);

	shell_print(sh,
		    "{\"ccp_master\":{\"role\":\"%s\",\"root\":%u,"
		    "\"sent\":%u,\"dropped\":%u,"
		    "\"late_ns_min\":%d,\"late_ns_max\":%d,"
		    "\"late_ns_last\":%d,"
		    "\"late_ns_signed_last\":%d,\"late_ns_signed_min\":%d,"
		    "\"sys_status_lo\":\"0x%08X\",\"hpdwarn_seen\":%d,"
		    "\"txtest\":{\"pending\":%d,\"done\":%d,\"tx_ok\":%d,"
		    "\"txfrs_ok\":%d}}}",
		    board_role(), root, sent, dropped,
		    (int)late_min, (int)late_max, (int)late_last,
		    (int)late_signed_last, (int)late_signed_min,
		    (unsigned int)sys_status_lo, (int)hpdwarn_seen,
		    (int)tt_pending, (int)tt_done, (int)tt_tx_ok,
		    (int)tt_txfrs_ok);

	if (strcmp(board_role(), "gateway") != 0) {
		shell_warn(sh,
			   "sent/dropped are only meaningful on a GATEWAY -- "
			   "ccp_master_after_beacon() is called from nowhere "
			   "else, so sent:0 dropped:0 here means \"not a "
			   "gateway\", not \"the link is fine\".");
	} else if (dropped != 0u) {
		/* Makes a real risk readable rather than silent: if the CCP's
		 * arm budget (ccp_sched.h's CCP_SCHED_ARM_BUDGET_NS, unmeasured
		 * on this exact path) is too tight on this hardware, every CCP
		 * is dropped and `sync stats` on the peer just sits at `rx:0`
		 * forever, with nothing there to say why. This is the first
		 * place that does. */
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
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sync, &sub_sync, "Anchor clock synchronisation", NULL);
