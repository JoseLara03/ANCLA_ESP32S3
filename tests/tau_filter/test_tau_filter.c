/*
 * Host test for src/tau_filter.c -- the B14 Fase B, Task 4 anchor-side
 * filter (NLOS/quality gate -> slow EMA -> jump detection). Pure C, no
 * Zephyr dependency, so every branch is checked with hand-computed exact
 * expected values, same discipline as tests/gw_selftau/ and
 * tests/sync_model/.
 *
 * Build:
 *   gcc -Wall -Wextra -Isrc -o tests/tau_filter/test_tau_filter.exe \
 *       tests/tau_filter/test_tau_filter.c src/tau_filter.c
 */

#include "tau_filter.h"

#include <stdio.h>

static int failures;

#define CHECK(cond)                                                          \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                            \
		}                                                              \
	} while (0)

static void test_seeds_directly_on_first_sample(void)
{
	struct tau_filter f;
	int32_t out = -1;
	enum tau_filter_outcome oc;

	tau_filter_init(&f);
	oc = tau_filter_step(&f, 1000, 200, &out);

	CHECK(oc == TAU_FILTER_SMOOTHED);
	CHECK(out == 1000);
	CHECK(f.value_dtu == 1000);
	CHECK(f.have_estimate);
	CHECK(f.n_accepted == 1u);
	CHECK(f.n_rejected_quality == 0u);
}

/* GW_SELFTAU_AUTO_QUALITY_NONE (0) is the only case rejected -- see
 * tau_filter.h's top comment for why this is deliberately narrow.
 * TAU_FILTER_MIN_QUALITY itself must be ACCEPTED (the check is `quality <
 * MIN_QUALITY`, not `<=`). */
static void test_quality_gate_rejects_only_none(void)
{
	struct tau_filter f;
	int32_t out = -1;
	enum tau_filter_outcome oc;

	tau_filter_init(&f);
	oc = tau_filter_step(&f, 5000, 0, &out);

	CHECK(oc == TAU_FILTER_REJECTED_QUALITY);
	CHECK(!f.have_estimate);
	CHECK(f.n_rejected_quality == 1u);
	CHECK(f.n_accepted == 0u);
	CHECK(out == -1); /* untouched, per the header's own contract */

	CHECK(TAU_FILTER_MIN_QUALITY == 1u);
	oc = tau_filter_step(&f, 5000, (uint8_t)TAU_FILTER_MIN_QUALITY, &out);
	CHECK(oc == TAU_FILTER_SMOOTHED);
	CHECK(out == 5000);
	CHECK(f.n_accepted == 1u);
}

/* Every delta here stays under TAU_FILTER_JUMP_THRESHOLD_DTU (128), so only
 * the EMA branch runs. alpha = 5/100, integer truncation toward zero. */
static void test_ema_converges_by_five_percent_steps(void)
{
	struct tau_filter f;
	int32_t out = -1;
	enum tau_filter_outcome oc;

	tau_filter_init(&f);
	tau_filter_step(&f, 1000, 255, &out);
	CHECK(out == 1000);

	/* delta=50: 50*5/100 = 2 (2.5 truncated). */
	oc = tau_filter_step(&f, 1050, 255, &out);
	CHECK(oc == TAU_FILTER_SMOOTHED);
	CHECK(out == 1002);
	CHECK(f.value_dtu == 1002);
	CHECK(f.jump_run == 0u);

	/* delta=48: 48*5/100 = 2 (2.4 truncated). */
	oc = tau_filter_step(&f, 1050, 255, &out);
	CHECK(oc == TAU_FILTER_SMOOTHED);
	CHECK(out == 1004);
}

/* Five consecutive same-direction over-threshold samples complete a jump,
 * snapped to the MEAN of the confirming run (not just the last sample). */
static void test_sustained_jump_snaps_to_run_mean(void)
{
	struct tau_filter f;
	int32_t out = -1;
	enum tau_filter_outcome oc;
	/* Sums to 5000 over 5 samples -> mean 1000, deliberately non-uniform
	 * so this cannot pass by accident on a "just echoes the last value"
	 * bug. */
	static const int32_t samples[5] = {1000, 1010, 990, 1005, 995};

	tau_filter_init(&f);
	tau_filter_step(&f, 0, 255, &out); /* seed */
	CHECK(out == 0);

	for (int i = 0; i < 4; i++) {
		oc = tau_filter_step(&f, samples[i], 255, &out);
		CHECK(oc == TAU_FILTER_SMOOTHED);
		CHECK(out == 0); /* unchanged while the run builds */
		CHECK(f.jump_run == (uint8_t)(i + 1));
	}

	oc = tau_filter_step(&f, samples[4], 255, &out);
	CHECK(oc == TAU_FILTER_JUMPED);
	CHECK(out == 1000);
	CHECK(f.value_dtu == 1000);
	CHECK(f.jump_run == 0u);
	CHECK(f.n_jumps == 1u);
}

/* A sample back within tolerance breaks an in-progress run (folds into the
 * EMA instead), and the run must restart from 1, not resume, afterward. */
static void test_in_tolerance_sample_breaks_the_run(void)
{
	struct tau_filter f;
	int32_t out = -1;
	enum tau_filter_outcome oc;

	tau_filter_init(&f);
	tau_filter_step(&f, 0, 255, &out); /* seed, value=0 */

	for (int i = 0; i < 3; i++) {
		oc = tau_filter_step(&f, 1000, 255, &out);
		CHECK(oc == TAU_FILTER_SMOOTHED);
		CHECK(f.jump_run == (uint8_t)(i + 1));
	}
	CHECK(f.value_dtu == 0); /* run never confirmed, value untouched */

	/* delta=50 (<=128): breaks the run AND folds into the EMA:
	 * 50*5/100 = 2. */
	oc = tau_filter_step(&f, 50, 255, &out);
	CHECK(oc == TAU_FILTER_SMOOTHED);
	CHECK(f.jump_run == 0u);
	CHECK(out == 2);
	CHECK(f.value_dtu == 2);

	/* Only 4 more over-threshold samples follow -- one short of
	 * TAU_FILTER_JUMP_SUSTAIN_COUNT (5), so this must NOT complete. */
	for (int i = 0; i < 4; i++) {
		oc = tau_filter_step(&f, 1000, 255, &out);
		CHECK(oc == TAU_FILTER_SMOOTHED);
	}
	CHECK(f.jump_run == 4u);
	CHECK(f.value_dtu == 2);
	CHECK(f.n_jumps == 0u);
}

/* A sample on the OPPOSITE side of the applied value also breaks a run --
 * FR-8 requires the sustained deviation to be consistent, not merely
 * "far enough away" -- but the breaking sample itself starts a fresh run
 * (count 1), not a discard back to 0. */
static void test_sign_flip_restarts_the_run_at_one(void)
{
	struct tau_filter f;
	int32_t out = -1;
	enum tau_filter_outcome oc;

	tau_filter_init(&f);
	tau_filter_step(&f, 0, 255, &out); /* seed, value=0 */

	for (int i = 0; i < 3; i++) {
		tau_filter_step(&f, 1000, 255, &out);
	}
	CHECK(f.jump_run == 3u);
	CHECK(f.jump_positive);

	/* Opposite direction, still over threshold: restarts at 1. */
	oc = tau_filter_step(&f, -1000, 255, &out);
	CHECK(oc == TAU_FILTER_SMOOTHED);
	CHECK(f.jump_run == 1u);
	CHECK(!f.jump_positive);
	CHECK(f.value_dtu == 0); /* still unconfirmed */

	for (int i = 0; i < 3; i++) {
		oc = tau_filter_step(&f, -1000, 255, &out);
		CHECK(oc == TAU_FILTER_SMOOTHED);
	}
	CHECK(f.jump_run == 4u);

	oc = tau_filter_step(&f, -1000, 255, &out);
	CHECK(oc == TAU_FILTER_JUMPED);
	CHECK(out == -1000);
	CHECK(f.value_dtu == -1000);
	CHECK(f.n_jumps == 1u);
}

/* A quality-rejected sample in the middle of a building run is simply
 * ABSENT -- it must neither advance nor reset jump_run, matching FR-8's
 * "mediciones" meaning usable measurements only. */
static void test_quality_rejection_does_not_disturb_a_run(void)
{
	struct tau_filter f;
	int32_t out = -1;
	enum tau_filter_outcome oc;

	tau_filter_init(&f);
	tau_filter_step(&f, 0, 255, &out); /* seed, value=0 */

	tau_filter_step(&f, 1000, 255, &out);
	tau_filter_step(&f, 1000, 255, &out);
	CHECK(f.jump_run == 2u);

	oc = tau_filter_step(&f, 9999, 0, &out); /* rejected */
	CHECK(oc == TAU_FILTER_REJECTED_QUALITY);
	CHECK(f.jump_run == 2u); /* untouched */

	tau_filter_step(&f, 1000, 255, &out);
	CHECK(f.jump_run == 3u);
	tau_filter_step(&f, 1000, 255, &out);
	CHECK(f.jump_run == 4u);

	oc = tau_filter_step(&f, 1000, 255, &out);
	CHECK(oc == TAU_FILTER_JUMPED);
	CHECK(out == 1000); /* mean of the five ACCEPTED 1000s in the run,
			      * none of the rejected sample */
	CHECK(f.n_accepted == 6u); /* the seed (0) plus the five 1000s */
	CHECK(f.n_rejected_quality == 1u);
}

int main(void)
{
	test_seeds_directly_on_first_sample();
	test_quality_gate_rejects_only_none();
	test_ema_converges_by_five_percent_steps();
	test_sustained_jump_snaps_to_run_mean();
	test_in_tolerance_sample_breaks_the_run();
	test_sign_flip_restarts_the_run_at_one();
	test_quality_rejection_does_not_disturb_a_run();

	if (failures) {
		printf("FAILED (%d)\n", failures);
		return 1;
	}
	printf("tau_filter: ALL TESTS PASSED\n");
	return 0;
}
