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
 */

#include "ccp_slave.h"
#include "sync_model.h"

#include <zephyr/shell/shell.h>

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
		    "{\"sync\":{\"root\":%u,\"rx\":%u,\"gaps\":%u,"
		    "\"rejected\":%u,\"count\":%u,\"valid\":%u,"
		    "\"jitter_est_dtu\":%u,\"jitter_est_ps\":%u,"
		    "\"rms_dtu\":%u,\"max_dtu\":%u,\"drift_ppb\":%d,"
		    "\"verdict\":\"%s\"}}",
		    root, rx, gap, reject, cnt, valid ? 1u : 0u,
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
		    "Thresholds: <%u DTU pass, <=%u marginal, above that fail.",
		    SYNC_GATE_PASS_DTU, SYNC_GATE_FAIL_DTU);
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
		      "stats — the Phase 2 gate as JSON, with a verdict",
		      cmd_stats, 1, 0),
	SHELL_CMD_ARG(reset, NULL,
		      "reset — clear the residual statistics, keeping the lock",
		      cmd_reset, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sync, &sub_sync, "Anchor clock synchronisation", NULL);
