/*
 * Host tests for sync_model.
 *
 * The point of this suite is to answer, before any radio work, the question the
 * whole TDoA migration turns on: does a wirelessly-synchronised anchor reach
 * sub-nanosecond agreement with its master? The MAC contract section 1 assumed
 * not. These tests simulate two independent crystals with drift and timestamp
 * noise and measure what the model actually achieves.
 *
 * Build:
 *   gcc -Wall -Wextra -Isrc -o tests/sync_model/test_sync_model.exe \
 *       tests/sync_model/test_sync_model.c src/sync_model.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sync_model.h"

static int failures;

#define CHECK(cond)                                                            \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                            \
		}                                                              \
	} while (0)

/* One nanosecond, the target. */
#define NS  ((int64_t)SYNC_DTU_PER_NS)

/* Deterministic PRNG. Reproducibility matters more than statistical quality:
 * a test that fails only on some runs is worse than no test. */
static uint32_t rng_state = 0x13579BDFu;

static void rng_seed(uint32_t s) { rng_state = s; }

static int32_t rng_noise(int32_t amplitude)
{
	if (amplitude == 0) return 0;
	rng_state = rng_state * 1664525u + 1013904223u;
	/* Roughly uniform on [-amplitude, +amplitude]. */
	return (int32_t)((rng_state >> 8) % (uint32_t)(2 * amplitude + 1)) -
	       amplitude;
}

/* ---- The simulated pair of clocks ---------------------------------------
 *
 *   master  M(k) = m0 + k * T
 *   local   L(k) = l0 + k * T * (1 + drift)
 *
 * An observation hands the model (M(k), L(k) + noise). Conversion is then
 * checked at points where the true answer is known exactly.
 */
struct sim {
	uint64_t m0, l0;
	int32_t  ppb;          /* local runs fast by this many ppb */
	int32_t  jitter_dtu;
	uint64_t k;
};

static void sim_init(struct sim *s, uint64_t m0, uint64_t l0, int32_t ppb,
		     int32_t jitter)
{
	s->m0 = m0; s->l0 = l0; s->ppb = ppb; s->jitter_dtu = jitter; s->k = 0;
}

/* True local time corresponding to a master-time offset of `x` DTU from m0. */
static uint64_t sim_local_at(const struct sim *s, uint64_t x)
{
	int64_t skew = (int64_t)((__int128)x * s->ppb / 1000000000LL);

	return (s->l0 + x + (uint64_t)skew) & SYNC_DTU_MASK;
}

static void sim_step(struct sim *s, struct sync_model *m)
{
	uint64_t x   = s->k * SYNC_CCP_INTERVAL_DTU;
	uint64_t mst = (s->m0 + x) & SYNC_DTU_MASK;
	uint64_t loc = (sim_local_at(s, x) +
			(uint64_t)(int64_t)rng_noise(s->jitter_dtu)) &
		       SYNC_DTU_MASK;

	sync_model_observe(m, mst, loc);
	s->k++;
}

/* Signed error, DTU, of converting the local time at master-offset `x`. */
static int64_t sim_conv_err(const struct sim *s, const struct sync_model *m,
			    uint64_t x)
{
	uint64_t got = 0;
	uint64_t want = (s->m0 + x) & SYNC_DTU_MASK;

	if (!sync_model_to_master(m, sim_local_at(s, x), &got)) return INT64_MAX;

	int64_t d = (int64_t)((got - want) & SYNC_DTU_MASK);

	if (d & (1LL << 39)) d -= (1LL << 40);
	return d;
}

/* ---- 1. Lifecycle -------------------------------------------------------- */

static void test_needs_two_observations(void)
{
	struct sync_model m;
	struct sim s;
	uint64_t out;

	sync_model_init(&m);
	sim_init(&s, 1000000, 5000000, 0, 0);

	CHECK(!sync_model_to_master(&m, 5000000, &out));
	CHECK(sync_model_error_dtu(&m, 0) == UINT32_MAX);

	sim_step(&s, &m);                       /* one point is not a baseline */
	CHECK(!sync_model_to_master(&m, 5000000, &out));

	sim_step(&s, &m);
	CHECK(sync_model_to_master(&m, 5000000, &out));
	CHECK(sync_model_error_dtu(&m, 0) != UINT32_MAX);
}

/* ---- 2. Noiseless cases must be EXACT, not merely close ----------------- */

static void test_identical_clocks_are_exact(void)
{
	struct sync_model m;
	struct sim s;

	sync_model_init(&m);
	sim_init(&s, 0x1122334455ULL, 0x00AABBCCDDULL, 0, 0);

	for (int i = 0; i < 5; i++) sim_step(&s, &m);

	/* Anywhere inside the next interval, to the DTU. */
	for (uint64_t x = 0; x < SYNC_CCP_INTERVAL_DTU * 5; x += 997000000ULL) {
		CHECK(sim_conv_err(&s, &m, x) == 0);
	}
	CHECK(sync_model_drift_ppb(&m) == 0);
}

static void test_constant_drift_no_noise_is_exact(void)
{
	const int32_t ppbs[] = { 40000, -40000, 1, -1, 20000 };

	for (unsigned int k = 0; k < sizeof(ppbs) / sizeof(ppbs[0]); k++) {
		struct sync_model m;
		struct sim s;

		sync_model_init(&m);
		sim_init(&s, 0x0102030405ULL, 0x0908070605ULL, ppbs[k], 0);
		for (int i = 0; i < 6; i++) sim_step(&s, &m);

		/* A perfect linear clock is a linear model's exact fit. Allow
		 * 1 DTU for the integer division in the correction term -- that
		 * is 15.65 ps, four orders inside the target. */
		int64_t e = sim_conv_err(&s, &m, SYNC_CCP_INTERVAL_DTU * 6);

		CHECK(e > -2 && e < 2);

		/* And the drift is reported back, POSITIVE when the local clock
		 * runs fast -- the opposite sign from the `num` the conversion
		 * uses internally, which is easy to confuse and so is pinned in
		 * both directions here.
		 *
		 * The tolerance is set by the SIMULATION, not the model:
		 * sim_local_at() truncates x*ppb/1e9 toward zero, so at small
		 * ppb the simulated clock's real drift sits a fraction of a ppb
		 * below nominal and the model correctly reports what it was
		 * actually shown. */
		int32_t got = sync_model_drift_ppb(&m);

		if (!(got > ppbs[k] - 2 && got < ppbs[k] + 2)) {
			printf("    drift: asked %d ppb, model reports %d ppb\n",
			       ppbs[k], got);
		}
		CHECK(got > ppbs[k] - 2 && got < ppbs[k] + 2);
	}
}

/* ---- 3. The question the migration turns on ---------------------------- */

/* Run `n_ccp` observations with jitter, then measure the worst conversion error
 * over the following interval. Returns the worst |error| in DTU. */
static int64_t worst_error(int32_t ppb, int32_t jitter, unsigned int n_ccp,
			   uint32_t seed)
{
	struct sync_model m;
	struct sim s;
	int64_t worst = 0;

	rng_seed(seed);
	sync_model_init(&m);
	sim_init(&s, 0x2000000000ULL, 0x0400000000ULL, ppb, jitter);

	for (unsigned int i = 0; i < n_ccp; i++) sim_step(&s, &m);

	uint64_t base = (uint64_t)n_ccp * SYNC_CCP_INTERVAL_DTU;

	for (uint64_t off = 0; off < SYNC_CCP_INTERVAL_DTU;
	     off += SYNC_CCP_INTERVAL_DTU / 16) {
		int64_t e = sim_conv_err(&s, &m, base + off);

		if (e == INT64_MAX) return INT64_MAX;
		if (e < 0) e = -e;
		if (e > worst) worst = e;
	}
	return worst;
}

static void test_sub_nanosecond_is_reached(void)
{
	/* SYNC_JITTER_DTU (~100 ps) is the assumed hardware figure. With a
	 * baseline past SYNC_BASELINE_USEFUL the model must be comfortably
	 * inside 1 ns. */
	int64_t w = 0;

	for (uint32_t seed = 1; seed <= 8; seed++) {
		int64_t e = worst_error(40000, SYNC_JITTER_DTU,
					SYNC_BASELINE_USEFUL * 2, seed);

		CHECK(e != INT64_MAX);
		if (e > w) w = e;
	}
	printf("  sub-ns check: worst error over 8 seeds = %" PRId64
	       " DTU (%.3f ns), target < %" PRId64 " DTU\n",
	       w, (double)w / (double)NS, NS);
	CHECK(w < NS);
}

/* How does the error actually scale? Rather than assert a shape I guessed at,
 * measure it and assert only what the numbers support. The first version of
 * this test asserted a plateau at SYNC_BASELINE_USEFUL and failed: the error
 * keeps improving past it, because the phase EMA is an IIR that goes on
 * averaging and at n=10 has not finished settling from its zero start. */
static void test_error_scaling_is_measured_not_assumed(void)
{
	const unsigned int ns[] = { 2, 5, 10, 25, 100, 300 };
	int64_t mean[6];

	printf("  observations -> worst error, DTU, mean of 12 seeds"
	       " (jitter = %u DTU):\n", SYNC_JITTER_DTU);
	for (unsigned int i = 0; i < sizeof(ns) / sizeof(ns[0]); i++) {
		int64_t sum = 0;

		/* Averaged over seeds. One seed is far too noisy to judge a
		 * trend on: worst-of-16 probes came in at 2 DTU for n=5 and
		 * 10 DTU for n=10 purely by luck of the draw, which failed a
		 * monotonicity assertion the underlying behaviour does in
		 * fact satisfy. */
		for (uint32_t seed = 1; seed <= 12; seed++) {
			int64_t e = worst_error(40000, SYNC_JITTER_DTU,
						ns[i], seed);

			CHECK(e != INT64_MAX);
			CHECK(e < NS);        /* sub-ns from n=2 onward */
			sum += e;
		}
		mean[i] = sum / 12;
		printf("      n=%-4u %" PRId64 "\n", ns[i], mean[i]);
	}

	/* A long baseline must beat a short one. Compared end to end rather
	 * than step by step, since adjacent points differ by less than the
	 * sampling spread even across 12 seeds. */
	CHECK(mean[5] <= mean[0]);
	CHECK(mean[4] <= mean[1]);
}

/* Isolate the DETERMINISTIC floor: no jitter at all, so whatever error remains
 * is the model's own integer truncation plus the simulation's.
 *
 * This exists because of a surprise in the first run: at ten times the assumed
 * jitter the error came back SMALLER than at one times it (7 vs 8 DTU). Error
 * that does not scale with jitter is not jitter-limited, and a de-risk test
 * that cannot see its own dominant term is not much of a de-risk. */
static void test_deterministic_floor(void)
{
	int64_t floor_err = worst_error(40000, 0, 50, 1);

	printf("  deterministic floor (zero jitter, n=50): %" PRId64 " DTU\n",
	       floor_err);
	CHECK(floor_err != INT64_MAX);

	/* Truncation in the conversion's corr term and in the simulated clock is
	 * one DTU each, so the floor must be small single digits. If this grows,
	 * the arithmetic has a real precision bug rather than a noise problem. */
	CHECK(floor_err <= 4);
}

/* Now the sensitivity that matters for Phase 2: sweep jitter across three
 * orders of magnitude and print what the model does. The purpose is NOT to
 * pass -- it is to show which jitter figure would break the sub-ns target, so
 * the hardware measurement in A7 has something to be compared against. */
static void test_jitter_sensitivity_sweep(void)
{
	const int32_t jits[] = { 0, 6, 32, 64, 320, 640, 3200 };

	printf("  jitter sweep (n=%u, worst over 16 probes):\n",
	       SYNC_BASELINE_USEFUL * 2);
	for (unsigned int i = 0; i < sizeof(jits) / sizeof(jits[0]); i++) {
		int64_t e = worst_error(40000, jits[i], SYNC_BASELINE_USEFUL * 2,
					11);

		printf("      jitter %5d DTU (%6.2f ns) -> error %5" PRId64
		       " DTU (%6.3f ns)%s\n",
		       jits[i], (double)jits[i] / (double)NS, e,
		       (double)e / (double)NS, e < NS ? "" : "   OVER 1 ns");
		CHECK(e != INT64_MAX);
	}

	/* The one assertion worth making: at the ASSUMED figure it is sub-ns,
	 * and at a figure fifty times worse it is not. Both halves matter --
	 * the second is what stops this suite from being a rubber stamp. */
	CHECK(worst_error(40000, SYNC_JITTER_DTU, SYNC_BASELINE_USEFUL * 2, 11)
	      < NS);
	CHECK(worst_error(40000, SYNC_JITTER_DTU * 50,
			  SYNC_BASELINE_USEFUL * 2, 11) >= NS);
}

/* ---- 4. Wrap, overflow and coasting ------------------------------------ */

static void test_survives_the_40_bit_wrap(void)
{
	struct sync_model m;
	struct sim s;

	rng_seed(99);
	sync_model_init(&m);
	/* Start both clocks so the 2^40 wrap lands mid-run. */
	sim_init(&s, (1ULL << 40) - SYNC_CCP_INTERVAL_DTU * 3,
		 (1ULL << 40) - SYNC_CCP_INTERVAL_DTU * 2, 40000,
		 SYNC_JITTER_DTU);

	for (int i = 0; i < 12; i++) sim_step(&s, &m);

	int64_t e = sim_conv_err(&s, &m, SYNC_CCP_INTERVAL_DTU * 12);

	CHECK(e != INT64_MAX);
	if (e < 0) e = -e;
	printf("  across the 2^40 wrap: error = %" PRId64 " DTU\n", e);
	CHECK(e < NS);
}

static void test_max_baseline_does_not_overflow(void)
{
	struct sync_model m;
	struct sim s;

	rng_seed(4242);
	sync_model_init(&m);
	sim_init(&s, 0x0800000000ULL, 0x0100000000ULL, 40000, SYNC_JITTER_DTU);

	/* Past SYNC_BASELINE_MAX, so the roll fires repeatedly and the largest
	 * products the conversion can form are exercised. */
	for (unsigned int i = 0; i < SYNC_BASELINE_MAX * 3; i++) sim_step(&s, &m);

	CHECK(m.n_obs <= SYNC_BASELINE_MAX);

	int64_t e = sim_conv_err(&s, &m,
				 (uint64_t)SYNC_BASELINE_MAX * 3 *
					 SYNC_CCP_INTERVAL_DTU);

	CHECK(e != INT64_MAX);
	if (e < 0) e = -e;
	printf("  after %u observations (rolls included): error = %" PRId64
	       " DTU\n", SYNC_BASELINE_MAX * 3, e);
	CHECK(e < NS);

	/* Drift is still reported sanely, i.e. the roll preserved the ratio. */
	int32_t d = sync_model_drift_ppb(&m);

	CHECK(d > 39000 && d < 41000);
}

static void test_coasting_then_giving_up(void)
{
	struct sync_model m;
	struct sim s;
	uint64_t out;

	sync_model_init(&m);
	sim_init(&s, 0x0011223344ULL, 0x0055667788ULL, 40000, 0);
	for (int i = 0; i < 12; i++) sim_step(&s, &m);

	/* A few missed CCPs must NOT invalidate: after drift correction the
	 * residual is second order, so coasting seconds is safe. */
	for (unsigned int i = 0; i < SYNC_MISS_MAX; i++) sync_model_miss(&m);
	CHECK(sync_model_to_master(&m, s.l0, &out));

	/* Past the bound it must stop claiming validity rather than quietly
	 * converting against a stale model. */
	sync_model_miss(&m);
	CHECK(!sync_model_to_master(&m, s.l0, &out));
	CHECK(sync_model_error_dtu(&m, 0) == UINT32_MAX);

	/* And a fresh observation revives it. */
	sim_step(&s, &m);
	sim_step(&s, &m);
	CHECK(sync_model_to_master(&m, s.l0, &out));
}

/* ---- 5. The Phase 2 gate itself ---------------------------------------- */

/* The claim the whole hardware gate rests on: the RMS of the model's own
 * prediction residuals equals the timestamp jitter feeding it. If that does
 * not hold, reading `sync stats` on a bench anchor measures nothing and the
 * gate needs external instrumentation after all.
 *
 * The simulated noise is uniform on [-A, +A], whose sigma is A/sqrt(3), so
 * that -- not A -- is what the RMS must land on. */
static void test_residual_rms_measures_the_jitter(void)
{
	const int32_t amps[] = { 6, 32, 64, 320, 640 };

	printf("  residual RMS vs true jitter sigma:\n");
	for (unsigned int i = 0; i < sizeof(amps) / sizeof(amps[0]); i++) {
		struct sync_model m;
		struct sim s;

		rng_seed(500u + i);
		sync_model_init(&m);
		sim_init(&s, 0x1800000000ULL, 0x0200000000ULL, 40000, amps[i]);

		for (unsigned int k = 0; k < 400u; k++) sim_step(&s, &m);

		/* sigma of a uniform distribution on [-A, A]: A/sqrt(3). Kept in
		 * thousandths as well as rounded, so comparisons below need not
		 * truncate twice. 1000/sqrt(3) = 577. */
		uint32_t sigma_x1000 = (uint32_t)((int64_t)amps[i] * 577);
		uint32_t sigma       = sigma_x1000 / 1000u;
		uint32_t rms   = sync_model_residual_rms_dtu(&m);

		printf("      amp %4d -> sigma %4u, measured RMS %4u, max %5u\n",
		       amps[i], sigma, rms,
		       sync_model_residual_max_dtu(&m));

		CHECK(sync_model_residual_count(&m) > 390u);

		/* The RMS itself sits at SYNC_RESIDUAL_TO_JITTER x sigma, not at
		 * sigma -- the residual differences two independent noisy
		 * timestamps. Pinned so the factor cannot drift unnoticed. */
		uint32_t est = sync_model_jitter_est_dtu(&m);

		printf("                inferred jitter %4u (true sigma %4u)\n",
		       est, sigma);

		/* The RAW rms tracks SYNC_RESIDUAL_TO_JITTER x sigma. Compared in
		 * thousandths, because truncating sigma to an integer and then
		 * truncating the product again costs 25 % at the smallest
		 * amplitude and fails a band the behaviour actually satisfies. */
		uint32_t want_x1000 =
			(sigma_x1000 * SYNC_RESIDUAL_TO_JITTER) / 1000u;

		CHECK(rms * 1000u * 100u >= want_x1000 * 70u);
		CHECK(rms * 1000u * 100u <= want_x1000 * 135u);

		/* And the INFERRED jitter recovers sigma, which is the claim the
		 * gate actually rests on. Widest at the smallest amplitude,
		 * where integer truncation of sigma itself dominates. */
		CHECK(est * 100u >= sigma * 70u);
		CHECK(est * 100u <= sigma * 135u);
	}
}

/* A reset clears the statistics without disturbing the sync itself -- the
 * operator will want a fresh measurement without re-acquiring. */
static void test_residual_reset_keeps_sync(void)
{
	struct sync_model m;
	struct sim s;
	uint64_t out;

	rng_seed(77);
	sync_model_init(&m);
	sim_init(&s, 0x0300000000ULL, 0x0900000000ULL, 40000, SYNC_JITTER_DTU);
	for (unsigned int k = 0; k < 60u; k++) sim_step(&s, &m);

	CHECK(sync_model_residual_count(&m) > 50u);
	int32_t drift_before = sync_model_drift_ppb(&m);

	sync_model_residual_reset(&m);
	CHECK(sync_model_residual_count(&m) == 0u);
	CHECK(sync_model_residual_rms_dtu(&m) == 0u);
	CHECK(sync_model_residual_max_dtu(&m) == 0u);

	/* Still synced, still the same rate. */
	CHECK(sync_model_to_master(&m, s.l0, &out));
	CHECK(sync_model_drift_ppb(&m) == drift_before);
}

int main(void)
{
	printf("sync_model: 1 ns = %" PRId64 " DTU, CCP interval = %llu DTU\n",
	       NS, (unsigned long long)SYNC_CCP_INTERVAL_DTU);

	test_needs_two_observations();
	test_identical_clocks_are_exact();
	test_constant_drift_no_noise_is_exact();
	test_sub_nanosecond_is_reached();
	test_error_scaling_is_measured_not_assumed();
	test_deterministic_floor();
	test_jitter_sensitivity_sweep();
	test_survives_the_40_bit_wrap();
	test_max_baseline_does_not_overflow();
	test_coasting_then_giving_up();
	test_residual_rms_measures_the_jitter();
	test_residual_reset_keeps_sync();

	if (failures) {
		printf("\n%d CHECK(s) FAILED\n", failures);
		return EXIT_FAILURE;
	}
	printf("sync_model: ALL TESTS PASSED\n");
	return EXIT_SUCCESS;
}
