# Alpha-beta-gamma position filter — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Smooth and jump-damp the gateway's published TDoA positions with a
per-tag alpha-beta-gamma filter on the solved (x, y), replacing the EKF that
was removed on 2026-09-06.

**Architecture:** A new pure-C module `src/pos_abg.{c,h}` (state
`[x,y,vx,vy,ax,ay]`, fixed gains from one tracking index, Euclidean
innovation gate, streak-based reseed, still-freeze) is host-tested in
isolation, then wired into `tdoa_gw.c`'s `solve_one()` after the existing
`resolve_one()` solve, with its own counters on a third `blink stats` line and
a non-persisted `blink abg` tuning knob. Nothing upstream of the solve and
nothing downstream of `pos_sink_publish()` changes.

**Tech Stack:** Zephyr 4.4.x firmware for ESP32-S3 (`west build`), plain gcc
host tests (`-Wall -Wextra -Isrc ... -lm`), Python 3 for `tools/pos_trace.py`.

**Spec:** `docs/superpowers/specs/2026-09-06-abg-position-filter-design.md` —
read it first; every constant and every decision below argues from it.

## Global Constraints

- Filter module is pure C: `<math.h>`, `<stdbool.h>`, `<stdint.h>`,
  `<string.h>` only. No Zephyr headers, no CMSIS-DSP, no allocation.
- Float only; `double` allowed ONLY inside `pos_abg_cfg_from_index()` (runs
  once per config change, never per fix).
- Everything called from `tdoa_gw_step()` runs on the `K_PRIO_COOP(0)`
  gateway loop: bounded, never blocks, never transmits, never writes flash,
  `LOG_WRN` at most once per boot per condition.
- All new module state in `tdoa_gw.c` is `static` (4096 B main stack; see
  CLAUDE.md's `gw_core_ctx` overflow entry).
- The acceleration update is `a += (gamma / dt^2) * r` — NOT `2*gamma/dt^2`.
  Spec §3.3 explains the factor-of-two trap; the Kalman cross-check test in
  Task 1 pins it.
- `TDOA_DT_MAX_MS = 1000`, `POS_ABG_DT_MIN_S = 0.05f`,
  `POS_ABG_T_NOM_S = 0.2f`, `POS_ABG_GATE_DT_CAP = 3.0f`; defaults λ = 0.02,
  `alpha_still = 0.15f`, `gate_m = 1.5f`, `reset_after = 5`. The effective
  gate is `gate_m * min(CAP, max(1, dt/T_nom))`.
- Spec §3.3 "Finding": γ is expected to HURT under this site's dt spread.
  The default keeps the λ-derived γ as requested; `blink abg gamma 0` and
  Task 6 Step 3 are where that is settled on hardware. Do not pre-empt it in
  code, and do not drop the test that pins it.
- Counter identity that must hold at all times:
  `fixes == seeded + dt_reseed + filtered + reseed`, `still <= filtered`.
- Commit after every task; do not commit the maintainer's unrelated
  uncommitted files (`src/cal_run.*`, `src/cal_shell.c`, `src/uwb_radio.c`,
  the untracked `docs/*.md` and `tools/b_of_p.py`) — stage by path.
- Build both images before claiming a task that touches `src/` is done:

  ```powershell
  $env:ZEPHYR_BASE = "C:\Users\jolap\zephyrproject\zephyr"
  $env:ZEPHYR_SDK_INSTALL_DIR = "C:\Users\jolap\zephyr-sdk-1.0.1"
  west build -b ancla_esp32s3/esp32s3/procpu
  west build -b ancla_esp32s3/esp32s3/procpu --pristine -d build_cal -- "-DEXTRA_CONF_FILE=cal.conf"
  ```

  (Paths are this machine's; `build/CMakeCache.txt` is the source of truth
  on any other. Quote the `-D`.) Baseline after the EKF removal, 2026-09-06:
  production `dram0_0_seg` **271376 B**, cal **78432 B**, zero warnings.

---

## File structure

| file | responsibility | task |
|------|----------------|------|
| `src/pos_abg.h` | public types, constants, API contract | 1 |
| `src/pos_abg.c` | gains from index, seed/get/reset, predict-gate-correct step | 1, 2 |
| `tests/pos_abg/test_pos_abg.c` | host suite: gains vs Kalman, stability, noise, lag, outlier, gate, streak, still, dt guards, long run | 1, 2 |
| `src/tdoa_gw.h` | `TDOA_DT_MAX_MS`, `tdoa_gw_abg_stats()`, `tdoa_gw_abg_cfg()`/`tdoa_gw_abg_set_*()` | 3 |
| `src/tdoa_gw.c` | memo fields, `solve_one()` wiring, counters | 3 |
| `src/blink_shell.c` | `tdoa_abg` stats line, verdicts, `blink abg` knob | 3 |
| `CMakeLists.txt` | add `src/pos_abg.c` to the unconditional block | 3 |
| `tools/pos_trace.py` | parse `tdoa_abg`, check the fixes identity | 4 |
| `CLAUDE.md`, spec status line | record what shipped | 5 |
| (bench) | hardware verification and the λ pick | 6 |

---

### Task 1: `pos_abg` core — gains, seed/get, predict + correct

**Files:**
- Create: `src/pos_abg.h`
- Create: `src/pos_abg.c`
- Create: `tests/pos_abg/test_pos_abg.c`

**Interfaces:**
- Produces (used by Tasks 2 and 3):

  ```c
  #define POS_ABG_T_NOM_S      0.2f
  #define POS_ABG_DT_MIN_S     0.05f
  #define POS_ABG_GATE_DT_CAP  3.0f
  struct pos_abg { float x, y, vx, vy, ax, ay; bool init; uint8_t reject_streak; };
  struct pos_abg_cfg { float alpha, beta, gamma, alpha_still, gate_m; uint8_t reset_after; };
  enum pos_abg_result { POS_ABG_ACCEPTED, POS_ABG_REJECTED, POS_ABG_RESEEDED, POS_ABG_BAD_INPUT };
  void pos_abg_cfg_defaults(struct pos_abg_cfg *c);
  void pos_abg_cfg_from_index(struct pos_abg_cfg *c, float lambda);
  void pos_abg_reset(struct pos_abg *f);
  void pos_abg_seed(struct pos_abg *f, float x, float y);
  enum pos_abg_result pos_abg_step(struct pos_abg *f, const struct pos_abg_cfg *c,
                                   float dt_s, float zx, float zy, bool still);
  bool pos_abg_get(const struct pos_abg *f, float *x, float *y, float *vx, float *vy);
  ```

  In THIS task `pos_abg_step()` implements predict + correct only and ignores
  `gate_m`, `reset_after` and `still` (Task 2 adds those). The signature is
  final from the start so Task 3 never changes.

- [ ] **Step 1: Write the header**

`src/pos_abg.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Alpha-beta-gamma filter on a SOLVED position.
 *
 * gamma may be 0, which makes this an alpha-beta filter -- and spec §3.3's
 * simulation says that is the configuration to expect to ship: the
 * acceleration state random-walks on measurement noise and a long dt then
 * contributes 1/2 a dt^2, which under this site's 0.2-1.0 s dt spread made
 * the filtered output WORSE than the raw solve. Kept as alpha-beta-gamma
 * because that is what was asked for and the bench, not a simulation, is
 * where it is decided; `blink abg gamma 0` flips it live.
 *
 * Replaces the per-tag EKF removed on 2026-09-06 (see CLAUDE.md, "Precisión
 * y suavizado"). It sees only the (x, y) tdoa_solve() produced -- no ranges,
 * no range differences, no per-anchor sigma -- and its whole job is to make
 * consecutive published fixes of one tag temporally coherent and to keep a
 * single gross outlier off the map. It does not, and cannot, improve
 * accuracy: docs/superpowers/specs/2026-09-06-abg-position-filter-design.md §0.
 *
 * State [x, y, vx, vy, ax, ay]; fixed gains designed at POS_ABG_T_NOM_S and
 * applied with the ACTUAL dt (beta/dt, gamma/dt^2). Pure C, no Zephyr,
 * host-tested in tests/pos_abg/.
 *
 * GAIN CONVENTION, because it is the easy thing to get wrong by 2x: the
 * acceleration correction is  a += (gamma / dt^2) * r.  With that form the
 * Gray-Murray closed form in pos_abg_cfg_from_index() yields exactly the
 * steady-state Kalman gains (tests/pos_abg/ pins it against the Kalman
 * recursion). The other common form, 2*gamma/dt^2, doubles gamma.
 */

#ifndef POS_ABG_H
#define POS_ABG_H

#include <stdbool.h>
#include <stdint.h>

/* Nominal sample interval the gains are designed for: one BLINK per 200 ms
 * superframe. */
#define POS_ABG_T_NOM_S   0.2f

/* Floor under which beta/dt and gamma/dt^2 stop meaning anything. No two
 * distinct blinks of one tag can be closer than a superframe, so a dt below
 * this is a duplicate group, and the CALLER drops it (tdoa_gw.c counts it as
 * `reorder`). The filter itself only refuses dt <= 0. */
#define POS_ABG_DT_MIN_S  0.05f

/* The innovation gate grows with the interval the prediction spans:
 *   effective gate = cfg->gate_m * min(POS_ABG_GATE_DT_CAP, max(1, dt / T_nom))
 * i.e. 1x at the nominal 200 ms, 2x at 400 ms, capped at 3x from 600 ms up.
 * Measured in simulation (spec §3.3): a fixed gate rejected a walking tag on
 * ~500 of 1e5 cycles under this site's dt spread; the scaled gate on 3. */
#define POS_ABG_GATE_DT_CAP  3.0f

struct pos_abg {
	float   x, y;          /* m */
	float   vx, vy;        /* m/s */
	float   ax, ay;        /* m/s^2 */
	bool    init;          /* false until pos_abg_seed() */
	uint8_t reject_streak; /* consecutive gated cycles; saturates at 255 */
};

struct pos_abg_cfg {
	float   alpha;        /* position gain */
	float   beta;         /* velocity gain (applied as beta/dt) */
	float   gamma;        /* acceleration gain (applied as gamma/dt^2) */
	float   alpha_still;  /* position gain while the tag reports still */
	float   gate_m;       /* innovation gate on |z - x_pred| at nominal dt,
	                       * metres; scaled by dt, see POS_ABG_GATE_DT_CAP */
	uint8_t reset_after;  /* gated cycles before reseeding on z; 0 = never */
};

enum pos_abg_result {
	POS_ABG_ACCEPTED,   /* predicted and corrected: publish */
	POS_ABG_REJECTED,   /* predicted, correction gated: publish NOTHING */
	POS_ABG_RESEEDED,   /* streak reached, state replaced by z: publish */
	POS_ABG_BAD_INPUT,  /* unseeded, dt <= 0, or non-finite dt/z: state
	                     * untouched, caller's bug */
};

/* lambda = 0.02 via pos_abg_cfg_from_index(), alpha_still 0.15, gate_m 1.5,
 * reset_after 5. The spec's §3.3 table is where those come from. */
void pos_abg_cfg_defaults(struct pos_abg_cfg *c);

/* Gray & Murray (1993): alpha/beta/gamma from the Kalata tracking index
 * lambda = sigma_w * T^2 / sigma_v. Writes ONLY the three gains; the other
 * fields are untouched. lambda must be > 0; values in 0.005..0.5 are the
 * sensible range for this application. */
void pos_abg_cfg_from_index(struct pos_abg_cfg *c, float lambda);

/* init = false; everything zeroed. */
void pos_abg_reset(struct pos_abg *f);

/* Position from a fresh solve, velocity and acceleration zero, streak 0. */
void pos_abg_seed(struct pos_abg *f, float x, float y);

/* One cycle: predict over dt_s, gate on |z - x_pred| > cfg->gate_m, correct.
 * `still` selects cfg->alpha_still and zeroes velocity and acceleration after
 * the correction. See the enum for what the caller does with each result. */
enum pos_abg_result pos_abg_step(struct pos_abg *f, const struct pos_abg_cfg *c,
				 float dt_s, float zx, float zy, bool still);

/* Current estimate. false until seeded (out-pointers untouched); any pointer
 * may be NULL. */
bool pos_abg_get(const struct pos_abg *f, float *x, float *y,
		 float *vx, float *vy);

#endif /* POS_ABG_H */
```

- [ ] **Step 2: Write the failing tests for gains, seed/get, predict+correct**

`tests/pos_abg/test_pos_abg.c` (the whole file for this task; Task 2 appends):

```c
#include "pos_abg.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
	if (!(cond)) { \
		printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		g_fail++; \
	} \
} while (0)

/* ---- Steady-state Kalman filter for the jerk model, the reference the
 * closed-form gains must match. State [x, v, a], F = [[1,T,T^2/2],[0,1,T],
 * [0,0,1]], Q = sw^2 * g g^T with g = [T^2/2, T, 1], H = [1 0 0], R = sv^2.
 * Iterated to convergence; returns alpha = K0, beta = K1*T, gamma = K2*T^2
 * (the LAST one is the convention check -- see pos_abg.h). */
static void kalman_ss_gains(double sw, double sv, double T,
			    double *alpha, double *beta, double *gamma)
{
	double F[3][3] = {{1, T, T * T / 2}, {0, 1, T}, {0, 0, 1}};
	double g[3] = {T * T / 2, T, 1};
	double P[3][3] = {{10, 0, 0}, {0, 10, 0}, {0, 0, 10}};
	double K[3] = {0, 0, 0};

	for (int it = 0; it < 5000; it++) {
		double FP[3][3], Pp[3][3];
		int i, j, k;

		for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
			FP[i][j] = 0;
			for (k = 0; k < 3; k++) FP[i][j] += F[i][k] * P[k][j];
		}
		for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
			Pp[i][j] = sw * sw * g[i] * g[j];
			for (k = 0; k < 3; k++) Pp[i][j] += FP[i][k] * F[j][k];
		}
		double S = Pp[0][0] + sv * sv;
		for (i = 0; i < 3; i++) K[i] = Pp[i][0] / S;
		for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)
			P[i][j] = Pp[i][j] - K[i] * Pp[0][j];
	}
	*alpha = K[0];
	*beta  = K[1] * T;
	*gamma = K[2] * T * T;
}

static void test_gains_match_steady_state_kalman(void)
{
	const double T = 0.2, sv = 0.35;
	const double lams[3] = {0.01, 0.02, 0.08};

	for (int i = 0; i < 3; i++) {
		struct pos_abg_cfg c;
		double sw = lams[i] * sv / (T * T);
		double a, b, g;

		memset(&c, 0, sizeof(c));
		pos_abg_cfg_from_index(&c, (float)lams[i]);
		kalman_ss_gains(sw, sv, T, &a, &b, &g);
		CHECK(fabs(c.alpha - a) < 1e-3);
		CHECK(fabs(c.beta  - b) < 1e-3);
		CHECK(fabs(c.gamma - g) < 1e-3);   /* the 2x convention trap */
		/* from_index must not touch the other fields */
		CHECK(c.alpha_still == 0.0f && c.gate_m == 0.0f &&
		      c.reset_after == 0u);
	}
}

static void test_defaults(void)
{
	struct pos_abg_cfg c, ref;

	pos_abg_cfg_defaults(&c);
	memset(&ref, 0, sizeof(ref));
	pos_abg_cfg_from_index(&ref, 0.02f);
	CHECK(c.alpha == ref.alpha && c.beta == ref.beta && c.gamma == ref.gamma);
	CHECK(fabsf(c.alpha - 0.419f) < 0.005f);   /* spec §3.3 table, lambda 0.02 */
	CHECK(fabsf(c.beta  - 0.113f) < 0.005f);
	CHECK(fabsf(c.gamma - 0.0152f) < 0.001f);
	CHECK(c.alpha_still == 0.15f);
	CHECK(c.gate_m == 1.5f);
	CHECK(c.reset_after == 5u);
}

static void test_get_before_seed_and_null_pointers(void)
{
	struct pos_abg f;
	float x = 7.0f, y = 7.0f;

	pos_abg_reset(&f);
	CHECK(!pos_abg_get(&f, &x, &y, NULL, NULL));
	CHECK(x == 7.0f && y == 7.0f);   /* untouched */

	pos_abg_seed(&f, 1.0f, 2.0f);
	CHECK(pos_abg_get(&f, NULL, NULL, NULL, NULL));
	CHECK(pos_abg_get(&f, &x, &y, NULL, NULL));
	CHECK(x == 1.0f && y == 2.0f);
	CHECK(f.vx == 0.0f && f.vy == 0.0f && f.ax == 0.0f && f.ay == 0.0f);
	CHECK(f.reject_streak == 0u);
}

/* Filter that is the identity (alpha 1, beta 0, gamma 0) must return z. */
static void test_identity_gains_pass_measurement_through(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;
	float x, y;

	pos_abg_cfg_defaults(&c);
	c.alpha = 1.0f; c.beta = 0.0f; c.gamma = 0.0f; c.gate_m = 1e9f;
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 3.0f, -4.0f, false) == POS_ABG_ACCEPTED);
	CHECK(pos_abg_get(&f, &x, &y, NULL, NULL));
	CHECK(fabsf(x - 3.0f) < 1e-6f && fabsf(y + 4.0f) < 1e-6f);
}

/* A 1 m step in x: settles within 5 % in under 40 steps at every lambda in
 * the spec's table, and the impulse response decays -- stability by
 * simulation, the spec's stated test. */
static void test_step_response_settles(void)
{
	const float lams[5] = {0.01f, 0.02f, 0.03f, 0.05f, 0.08f};

	for (int i = 0; i < 5; i++) {
		struct pos_abg_cfg c;
		struct pos_abg f;
		float hist[400];
		int settle = -1;

		pos_abg_cfg_defaults(&c);
		pos_abg_cfg_from_index(&c, lams[i]);
		c.gate_m = 1e9f;
		pos_abg_reset(&f);
		pos_abg_seed(&f, 0.0f, 0.0f);
		for (int k = 0; k < 400; k++) {
			CHECK(pos_abg_step(&f, &c, 0.2f, 1.0f, 0.0f, false) ==
			      POS_ABG_ACCEPTED);
			hist[k] = f.x;
		}
		for (int k = 0; k < 400 && settle < 0; k++) {
			int ok = 1;
			for (int j = k; j < 400; j++) {
				if (fabsf(hist[j] - 1.0f) >= 0.05f) { ok = 0; break; }
			}
			if (ok) settle = k;
		}
		CHECK(settle >= 0 && settle < 40);
		CHECK(fabsf(hist[399] - 1.0f) < 1e-3f);
		CHECK(fabsf(f.vx) < 1e-3f && fabsf(f.ax) < 1e-3f);
	}
}

/* xorshift32, so the noise is reproducible across platforms. */
static uint32_t g_rng = 0xA5A5A5A5u;
static float noise(float sigma)
{
	/* sum of 12 uniforms -> approx N(0,1) */
	float s = 0.0f;
	for (int i = 0; i < 12; i++) {
		g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
		s += (float)(g_rng & 0xFFFFu) / 65535.0f;
	}
	return (s - 6.0f) * sigma;
}

/* Stationary tag, white noise: output RMS / input RMS ~ 0.60 at lambda 0.02
 * (spec §3.3 table), checked to +/-0.05. */
static void test_stationary_noise_reduction(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;
	const int N = 20000, warm = 500;
	const float sv = 0.35f;
	double acc = 0.0;

	pos_abg_cfg_defaults(&c);
	c.gate_m = 1e9f;
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (int k = 0; k < N; k++) {
		pos_abg_step(&f, &c, 0.2f, noise(sv), noise(sv), false);
		if (k >= warm) acc += (double)f.x * f.x;
	}
	double ratio = sqrt(acc / (N - warm)) / sv;
	CHECK(ratio > 0.55 && ratio < 0.65);
}

/* Constant velocity: an alpha-beta-gamma filter has ZERO steady-state lag
 * on a constant-velocity target. |error| < 2 cm after settling. */
static void test_constant_velocity_no_lag(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;
	const float v = 1.2f, T = 0.2f;

	pos_abg_cfg_defaults(&c);
	c.gate_m = 1e9f;
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (int k = 1; k <= 300; k++) {
		float t = (float)k * T;
		pos_abg_step(&f, &c, T, v * t, 0.0f, false);
		if (k > 100) {
			CHECK(fabsf(f.x - v * t) < 0.02f);
			CHECK(fabsf(f.vx - v) < 0.05f);
		}
	}
}

static void test_bad_input_leaves_state_untouched(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f, before;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.0f, 1.0f, false) == POS_ABG_BAD_INPUT);
	CHECK(!f.init);

	pos_abg_seed(&f, 1.0f, 2.0f);
	f.vx = 0.3f; f.ax = -0.1f;
	before = f;
	CHECK(pos_abg_step(&f, &c, 0.0f, 1.0f, 2.0f, false) == POS_ABG_BAD_INPUT);
	CHECK(memcmp(&f, &before, sizeof(f)) == 0);
	CHECK(pos_abg_step(&f, &c, -0.2f, 1.0f, 2.0f, false) == POS_ABG_BAD_INPUT);
	CHECK(memcmp(&f, &before, sizeof(f)) == 0);
	CHECK(pos_abg_step(&f, &c, NAN, 1.0f, 2.0f, false) == POS_ABG_BAD_INPUT);
	CHECK(memcmp(&f, &before, sizeof(f)) == 0);
	CHECK(pos_abg_step(&f, &c, 0.2f, NAN, 2.0f, false) == POS_ABG_BAD_INPUT);
	CHECK(memcmp(&f, &before, sizeof(f)) == 0);
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.0f, INFINITY, false) == POS_ABG_BAD_INPUT);
	CHECK(memcmp(&f, &before, sizeof(f)) == 0);
}

/* dt of 1.0 s (five nominal steps) must not blow up beta/dt, gamma/dt^2. */
static void test_long_dt_is_bounded(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	c.gate_m = 1e9f;
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 1.0f, 1.0f, 0.0f, false) == POS_ABG_ACCEPTED);
	CHECK(f.x > 0.0f && f.x <= 1.0f);
	CHECK(fabsf(f.vx) <= c.beta / 1.0f * 1.0f + 1e-6f);
	CHECK(fabsf(f.ax) <= c.gamma / 1.0f * 1.0f + 1e-6f);
}

int main(void)
{
	test_gains_match_steady_state_kalman();
	test_defaults();
	test_get_before_seed_and_null_pointers();
	test_identity_gains_pass_measurement_through();
	test_step_response_settles();
	test_stationary_noise_reduction();
	test_constant_velocity_no_lag();
	test_bad_input_leaves_state_untouched();
	test_long_dt_is_bounded();

	if (g_fail) {
		printf("pos_abg: %d FAILED\n", g_fail);
		return 1;
	}
	printf("pos_abg: ALL TESTS PASSED\n");
	return 0;
}
```

- [ ] **Step 3: Run to verify it fails to build (no implementation yet)**

```bash
gcc -Wall -Wextra -Isrc -o tests/pos_abg/test_pos_abg.exe tests/pos_abg/test_pos_abg.c src/pos_abg.c -lm
```

Expected: fails, `src/pos_abg.c: No such file or directory`.

- [ ] **Step 4: Write the implementation (predict + correct; gate/still come in Task 2)**

`src/pos_abg.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pos_abg.h"

#include <math.h>
#include <string.h>

void pos_abg_cfg_from_index(struct pos_abg_cfg *c, float lambda)
{
	/* Gray & Murray, "A derivation of an analytic expression for the
	 * tracking index for the alpha-beta-gamma filter", IEEE TAES 1993.
	 * Double on purpose, and the one place this module uses it: the cubic
	 * cancels badly at small lambda in float, and this runs once per config
	 * change, never per fix. */
	double l  = (double)lambda;
	double b  = l / 2.0 - 3.0;
	double cc = l / 2.0 + 3.0;
	double d  = -1.0;
	double p  = cc - b * b / 3.0;
	double q  = 2.0 * b * b * b / 27.0 - b * cc / 3.0 + d;
	double v  = sqrt(q * q + 4.0 * p * p * p / 27.0);
	double s  = -cbrt((q + v) / 2.0) - cbrt((q - v) / 2.0) - b / 3.0;
	double alpha = 1.0 - s * s;
	double beta  = 2.0 * (1.0 - s) * (1.0 - s);

	c->alpha = (float)alpha;
	c->beta  = (float)beta;
	/* beta^2 / (2 alpha) is the steady-state Kalman gamma UNDER the
	 * a += (gamma/dt^2) r convention pos_abg_step() uses. See pos_abg.h. */
	c->gamma = (float)(beta * beta / (2.0 * alpha));
}

void pos_abg_cfg_defaults(struct pos_abg_cfg *c)
{
	memset(c, 0, sizeof(*c));
	pos_abg_cfg_from_index(c, 0.02f);
	c->alpha_still = 0.15f;
	c->gate_m      = 1.5f;
	c->reset_after = 5u;
}

void pos_abg_reset(struct pos_abg *f)
{
	memset(f, 0, sizeof(*f));
}

void pos_abg_seed(struct pos_abg *f, float x, float y)
{
	f->x = x;  f->y = y;
	f->vx = 0.0f; f->vy = 0.0f;
	f->ax = 0.0f; f->ay = 0.0f;
	f->reject_streak = 0u;
	f->init = true;
}

bool pos_abg_get(const struct pos_abg *f, float *x, float *y,
		 float *vx, float *vy)
{
	if (!f->init) {
		return false;
	}
	if (x != NULL)  { *x = f->x; }
	if (y != NULL)  { *y = f->y; }
	if (vx != NULL) { *vx = f->vx; }
	if (vy != NULL) { *vy = f->vy; }
	return true;
}

enum pos_abg_result pos_abg_step(struct pos_abg *f, const struct pos_abg_cfg *c,
				 float dt_s, float zx, float zy, bool still)
{
	float T, T2, xp, yp, vxp, vyp, rx, ry;

	if (!f->init || !(dt_s > 0.0f) || !isfinite(dt_s) ||
	    !isfinite(zx) || !isfinite(zy)) {
		return POS_ABG_BAD_INPUT;
	}
	(void)still;   /* Task 2 */

	T  = dt_s;
	T2 = T * T;

	/* Predict to the measurement's instant. */
	xp  = f->x + f->vx * T + 0.5f * f->ax * T2;
	yp  = f->y + f->vy * T + 0.5f * f->ay * T2;
	vxp = f->vx + f->ax * T;
	vyp = f->vy + f->ay * T;

	rx = zx - xp;
	ry = zy - yp;

	/* Correct. Fixed gains designed at POS_ABG_T_NOM_S, applied with the
	 * actual dt: for dt longer than nominal the velocity and acceleration
	 * corrections shrink, which is the conservative direction. */
	f->x  = xp  + c->alpha * rx;
	f->y  = yp  + c->alpha * ry;
	f->vx = vxp + (c->beta / T) * rx;
	f->vy = vyp + (c->beta / T) * ry;
	f->ax = f->ax + (c->gamma / T2) * rx;
	f->ay = f->ay + (c->gamma / T2) * ry;
	f->reject_streak = 0u;

	return POS_ABG_ACCEPTED;
}
```

- [ ] **Step 5: Build and run the suite**

```bash
gcc -Wall -Wextra -Isrc -o tests/pos_abg/test_pos_abg.exe tests/pos_abg/test_pos_abg.c src/pos_abg.c -lm && ./tests/pos_abg/test_pos_abg.exe
```

Expected: `pos_abg: ALL TESTS PASSED`, exit 0, no compiler warnings. If
`test_gains_match_steady_state_kalman` fails only on gamma by a factor of
~2, the convention in `pos_abg_step()` is wrong (see pos_abg.h) — fix the
step, not the test.

- [ ] **Step 6: Commit**

```bash
git add src/pos_abg.h src/pos_abg.c tests/pos_abg/test_pos_abg.c
git commit -m "feat(pos_abg): alpha-beta-gamma position filter core, gains from the tracking index"
```

---

### Task 2: `pos_abg` gate, streak reseed, still-freeze

**Files:**
- Modify: `src/pos_abg.c` (`pos_abg_step()` only)
- Modify: `tests/pos_abg/test_pos_abg.c` (append tests, extend `main()`)

**Interfaces:**
- Consumes: Task 1's types and `pos_abg_step()` signature, unchanged.
- Produces: the full `pos_abg_step()` contract of spec §3.2 —
  `POS_ABG_REJECTED` when `hypot(rx, ry) > cfg->gate_m` (state = prediction,
  streak++), `POS_ABG_RESEEDED` when `reset_after != 0 && streak >=
  reset_after` (state = `pos_abg_seed(z)`), `still` ⇒ `alpha_still` for the
  position and `vx = vy = ax = ay = 0` after the correction.

- [ ] **Step 1: Append the failing tests**

Add before `main()` in `tests/pos_abg/test_pos_abg.c`:

```c
/* A single 3 m outlier is REJECTED: state equals the prediction (which for a
 * stationary seeded filter is the seed), init still true, streak 1. */
static void test_gate_rejects_outlier_and_holds_prediction(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);
	pos_abg_seed(&f, 1.0f, 1.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 4.0f, 1.0f, false) == POS_ABG_REJECTED);
	CHECK(f.init);
	CHECK(f.x == 1.0f && f.y == 1.0f);
	CHECK(f.vx == 0.0f && f.ax == 0.0f);
	CHECK(f.reject_streak == 1u);

	/* A moving filter coasts on its velocity while rejecting. */
	pos_abg_seed(&f, 0.0f, 0.0f);
	f.vx = 1.0f;
	CHECK(pos_abg_step(&f, &c, 0.2f, 10.0f, 0.0f, false) == POS_ABG_REJECTED);
	CHECK(fabsf(f.x - 0.2f) < 1e-6f);
	CHECK(f.vx == 1.0f);
}

/* Exactly at the gate is accepted (the test is `>`), just past it rejected. */
static void test_gate_boundary(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.5f, 0.0f, false) == POS_ABG_ACCEPTED);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.5001f, 0.0f, false) == POS_ABG_REJECTED);
	/* Euclidean, not per-axis: 1.2 in x and 1.2 in y is 1.70 m. */
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.2f, 1.2f, false) == POS_ABG_REJECTED);
}

/* The gate grows with dt: 2x at 0.4 s, capped at 3x (4.5 m) from 0.6 s up.
 * Below nominal dt it stays at gate_m. */
static void test_gate_scales_with_dt(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);

	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.4f, 2.5f, 0.0f, false) == POS_ABG_ACCEPTED);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.4f, 3.1f, 0.0f, false) == POS_ABG_REJECTED);

	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 1.0f, 4.4f, 0.0f, false) == POS_ABG_ACCEPTED);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 1.0f, 4.6f, 0.0f, false) == POS_ABG_REJECTED);

	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.1f, 1.6f, 0.0f, false) == POS_ABG_REJECTED);
}

/* Five consecutive far measurements: RESEEDED on the fifth at the new spot,
 * velocity zero, streak 0. Four then a return: no reseed, streak back to 0. */
static void test_streak_reseeds_after_reset_after(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;
	int k;

	pos_abg_cfg_defaults(&c);
	CHECK(c.reset_after == 5u);
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (k = 0; k < 4; k++) {
		CHECK(pos_abg_step(&f, &c, 0.2f, 5.0f, 5.0f, false) == POS_ABG_REJECTED);
		CHECK(f.reject_streak == (uint8_t)(k + 1));
	}
	CHECK(pos_abg_step(&f, &c, 0.2f, 5.0f, 5.0f, false) == POS_ABG_RESEEDED);
	CHECK(f.x == 5.0f && f.y == 5.0f);
	CHECK(f.vx == 0.0f && f.vy == 0.0f && f.ax == 0.0f && f.ay == 0.0f);
	CHECK(f.reject_streak == 0u);

	pos_abg_seed(&f, 0.0f, 0.0f);
	for (k = 0; k < 4; k++) {
		CHECK(pos_abg_step(&f, &c, 0.2f, 5.0f, 5.0f, false) == POS_ABG_REJECTED);
	}
	CHECK(pos_abg_step(&f, &c, 0.2f, 0.1f, 0.0f, false) == POS_ABG_ACCEPTED);
	CHECK(f.reject_streak == 0u);
	CHECK(f.x > 0.0f && f.x < 0.1f);
}

/* reset_after == 0 means never reseed on the streak; the streak saturates
 * at 255 instead of wrapping to 0. */
static void test_reset_after_zero_never_reseeds_and_streak_saturates(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	c.reset_after = 0u;
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (int k = 0; k < 300; k++) {
		CHECK(pos_abg_step(&f, &c, 0.2f, 5.0f, 5.0f, false) == POS_ABG_REJECTED);
	}
	CHECK(f.reject_streak == 255u);
	CHECK(f.x == 0.0f && f.y == 0.0f);
}

/* still: velocity and acceleration are exactly zero after the step, and the
 * position moves by alpha_still * r, not alpha * r. */
static void test_still_freezes_velocity_and_uses_alpha_still(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	f.vx = 0.5f; f.ax = 0.2f;   /* pretend it had been moving */
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.0f, 0.0f, true) == POS_ABG_ACCEPTED);
	CHECK(f.vx == 0.0f && f.vy == 0.0f && f.ax == 0.0f && f.ay == 0.0f);
	/* prediction was 0 + 0.5*0.2 + 0.5*0.2*0.04 = 0.104; correction 0.15*(1-0.104) */
	CHECK(fabsf(f.x - (0.104f + 0.15f * (1.0f - 0.104f))) < 1e-5f);

	/* A gated cycle while still still counts as rejected, not accepted. */
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 5.0f, 0.0f, true) == POS_ABG_REJECTED);
}

/* One-sample 1.4 m outlier (just under the gate) on a stationary tag at the
 * default lambda: peak output deviation < 0.65 m and back within 5 cm in 30
 * steps -- spec §3.3 table row for lambda 0.02 says 0.59 m. */
static void test_sub_gate_outlier_is_attenuated(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;
	float peak = 0.0f;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (int k = 0; k < 60; k++) {
		float z = (k == 5) ? 1.4f : 0.0f;

		CHECK(pos_abg_step(&f, &c, 0.2f, z, 0.0f, false) == POS_ABG_ACCEPTED);
		if (fabsf(f.x) > peak) peak = fabsf(f.x);
		if (k >= 35) CHECK(fabsf(f.x) < 0.05f);
	}
	CHECK(peak > 0.5f && peak < 0.65f);
}

/* The spec §3.3 "Finding" scenario: a stationary tag, 10^5 steps of random
 * dt in [0.2, 1.0] s, sigma 0.35 m noise, a 4 m outlier every 997 cycles,
 * never `still` (a worn tag). Returns the worst |estimate| and counts
 * reseeds. Shared by the two tests below. */
static float long_run(const struct pos_abg_cfg *c, uint32_t *reseeds_out)
{
	struct pos_abg f;
	float worst = 0.0f;
	uint32_t reseeds = 0u;

	g_rng = 0xC0FFEE11u;   /* same draw for every caller */
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (int k = 0; k < 100000; k++) {
		g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
		float dt = 0.2f + 0.8f * (float)(g_rng & 0xFFu) / 255.0f;
		float zx = noise(0.35f), zy = noise(0.35f);

		if ((k % 997) == 0) { zx += 4.0f; }       /* gross outlier */
		if (pos_abg_step(&f, c, dt, zx, zy, false) == POS_ABG_RESEEDED) {
			reseeds++;
		}
		CHECK(isfinite(f.x) && isfinite(f.y) && isfinite(f.vx) &&
		      isfinite(f.vy) && isfinite(f.ax) && isfinite(f.ay));
		if (fabsf(f.x) > worst) worst = fabsf(f.x);
		if (fabsf(f.y) > worst) worst = fabsf(f.y);
	}
	*reseeds_out = reseeds;
	return worst;
}

/* Default config: finite throughout, streak reseeds under 1 % of cycles.
 * NOTE the bound this test does NOT make: the estimate is not asserted to
 * stay near the truth, because with the default gamma it does not (spec
 * §3.3 table: worst 8.6 m). That is the finding the next test pins. */
static void test_long_run_stays_finite(void)
{
	struct pos_abg_cfg c;
	uint32_t reseeds = 0u;

	pos_abg_cfg_defaults(&c);
	(void)long_run(&c, &reseeds);
	CHECK(reseeds < 1000u);
}

/* Spec §3.3 "Finding", pinned: under this dt spread the alpha-beta filter
 * (gamma = 0) has a smaller worst excursion than the default alpha-beta-gamma,
 * and no more reseeds. If this starts failing, the finding has been
 * re-measured -- update spec §3.3 with the new numbers, do not delete this. */
static void test_gamma_zero_is_more_robust_under_dt_jitter(void)
{
	struct pos_abg_cfg c_abg, c_ab;
	uint32_t r_abg = 0u, r_ab = 0u;
	float w_abg, w_ab;

	pos_abg_cfg_defaults(&c_abg);
	c_ab = c_abg;
	c_ab.gamma = 0.0f;
	w_abg = long_run(&c_abg, &r_abg);
	w_ab  = long_run(&c_ab, &r_ab);
	printf("  finding: worst |x| abg %.2f m (%u reseeds) vs ab %.2f m "
	       "(%u reseeds)\n", (double)w_abg, r_abg, (double)w_ab, r_ab);
	CHECK(w_ab < w_abg);
	CHECK(r_ab <= r_abg);
	CHECK(w_ab < 3.0f);   /* spec table: ~2.3 m */
}
```

And add to `main()`, after `test_long_dt_is_bounded();`:

```c
	test_gate_rejects_outlier_and_holds_prediction();
	test_gate_boundary();
	test_gate_scales_with_dt();
	test_streak_reseeds_after_reset_after();
	test_reset_after_zero_never_reseeds_and_streak_saturates();
	test_still_freezes_velocity_and_uses_alpha_still();
	test_sub_gate_outlier_is_attenuated();
	test_long_run_stays_finite();
	test_gamma_zero_is_more_robust_under_dt_jitter();
```

- [ ] **Step 2: Run to verify the new tests fail**

```bash
gcc -Wall -Wextra -Isrc -o tests/pos_abg/test_pos_abg.exe tests/pos_abg/test_pos_abg.c src/pos_abg.c -lm && ./tests/pos_abg/test_pos_abg.exe
```

Expected: FAIL lines from `test_gate_rejects_outlier_and_holds_prediction`,
`test_gate_boundary`, `test_gate_scales_with_dt`, `test_streak_*`,
`test_still_*`; exit 1. (`test_gamma_zero_*` may already pass or fail --
irrelevant until the gate exists.)

- [ ] **Step 3: Implement gate, streak and still in `pos_abg_step()`**

Replace the body of `pos_abg_step()` from `(void)still;   /* Task 2 */` to the
end of the function with the code below, and add `float gate;` to the
function's declarations:

```c
	T  = dt_s;
	T2 = T * T;

	/* Predict to the measurement's instant. */
	xp  = f->x + f->vx * T + 0.5f * f->ax * T2;
	yp  = f->y + f->vy * T + 0.5f * f->ay * T2;
	vxp = f->vx + f->ax * T;
	vyp = f->vy + f->ay * T;

	rx = zx - xp;
	ry = zy - yp;

	/* Innovation gate: Euclidean, one threshold, grown with dt. Protection
	 * against a gross outlier (mirror-branch solve, one badly stamped
	 * anchor), not a statistical test -- gate_m sits ~4x the raw stationary
	 * RMS at the nominal 200 ms, and the prediction it is measured against
	 * is less certain the longer the interval it spans. */
	{
		float scale = T / POS_ABG_T_NOM_S;

		if (scale < 1.0f) {
			scale = 1.0f;
		} else if (scale > POS_ABG_GATE_DT_CAP) {
			scale = POS_ABG_GATE_DT_CAP;
		}
		gate = c->gate_m * scale;
	}
	if (sqrtf(rx * rx + ry * ry) > gate) {
		/* Predict ALWAYS, correct only when accepted: the caller
		 * advances the clock for this group whatever we return, so the
		 * state has to be at this instant or the next accepted group's
		 * dt covers time never integrated (the 2026-09-03 rewind
		 * defect, from the other side). */
		f->x = xp;  f->y = yp;
		f->vx = vxp; f->vy = vyp;
		if (f->reject_streak < 255u) {
			f->reject_streak++;
		}
		/* reset_after == 0: never reseed on the streak. */
		if (c->reset_after != 0u && f->reject_streak >= c->reset_after) {
			pos_abg_seed(f, zx, zy);
			return POS_ABG_RESEEDED;
		}
		return POS_ABG_REJECTED;
	}

	/* Correct. Fixed gains designed at POS_ABG_T_NOM_S, applied with the
	 * actual dt: for dt longer than nominal the velocity and acceleration
	 * corrections shrink, which is the conservative direction. */
	{
		float a = still ? c->alpha_still : c->alpha;

		f->x = xp + a * rx;
		f->y = yp + a * ry;
	}
	if (still) {
		/* The ZUPT-equivalent: the tag's accelerometer says it is not
		 * moving, so the next prediction is this point and alpha_still
		 * turns the filter into a slow average of the solves. */
		f->vx = 0.0f; f->vy = 0.0f;
		f->ax = 0.0f; f->ay = 0.0f;
	} else {
		f->vx = vxp + (c->beta / T) * rx;
		f->vy = vyp + (c->beta / T) * ry;
		f->ax = f->ax + (c->gamma / T2) * rx;
		f->ay = f->ay + (c->gamma / T2) * ry;
	}
	f->reject_streak = 0u;

	return POS_ABG_ACCEPTED;
}
```

- [ ] **Step 4: Run the full suite**

```bash
gcc -Wall -Wextra -Isrc -o tests/pos_abg/test_pos_abg.exe tests/pos_abg/test_pos_abg.c src/pos_abg.c -lm && ./tests/pos_abg/test_pos_abg.exe
```

Expected: `pos_abg: ALL TESTS PASSED`, exit 0, no warnings. If
`test_sub_gate_outlier_is_attenuated` fails on the peak bound, re-read
the table in spec §3.3 before touching the threshold: the peak is a property
of the gains, so a mismatch means the gains or the convention are off, not
the test.

- [ ] **Step 5: Commit**

```bash
git add src/pos_abg.c tests/pos_abg/test_pos_abg.c
git commit -m "feat(pos_abg): innovation gate, streak reseed and still-freeze"
```

---

### Task 3: Wire the filter into `tdoa_gw.c`, counters, `blink stats`, `blink abg`

**Files:**
- Modify: `CMakeLists.txt` (unconditional `target_sources` block, alphabetical
  after `src/net_uplink.c`)
- Modify: `src/tdoa_gw.h` (constants + three declarations)
- Modify: `src/tdoa_gw.c` (memo, static cfg, counters, `ingest_one()`,
  `solve_one()`, stats/cfg accessors)
- Modify: `src/blink_shell.c` (third stats line, verdicts, `blink abg` tree)

**Interfaces:**
- Consumes: everything Task 1/2 export from `pos_abg.h`; `BLINK_FLAG_MOVING`
  from `blink_frame.h`; existing `resolve_one()`, `memo_claim()`,
  `sdelta40()`, `TDOA_GW_S_PER_DTU`, `TDOA_DT_REORDER_MAX_MS` in `tdoa_gw.c`.
- Produces (used by the shell and by Task 4's tool):

  ```c
  #define TDOA_DT_MAX_MS  1000
  void tdoa_gw_abg_stats(uint32_t *n_seeded, uint32_t *n_dt_reseed,
                         uint32_t *n_filtered, uint32_t *n_gate_rejected,
                         uint32_t *n_reseed, uint32_t *n_still);
  const struct pos_abg_cfg *tdoa_gw_abg_cfg(void);     /* read-only view */
  float tdoa_gw_abg_lambda(void);                       /* last index set, 0.02 at boot */
  void tdoa_gw_abg_set_lambda(float lambda);            /* k_sched_lock()-fenced */
  void tdoa_gw_abg_set_gamma(float gamma);              /* 0 = alpha-beta */
  void tdoa_gw_abg_set_gate(float gate_m);
  void tdoa_gw_abg_set_alpha_still(float alpha_still);
  void tdoa_gw_abg_set_reset_after(uint8_t n);
  ```

  `blink stats` third line: `{"tdoa_abg":{"role":"…","seeded":N,"dt_reseed":N,"filtered":N,"gate_rejected":N,"reseed":N,"still":N}}`.

- [ ] **Step 1: CMake**

In `CMakeLists.txt`, after the line `\tsrc/net_uplink.c`, add:

```
	src/pos_abg.c
```

- [ ] **Step 2: Header additions in `src/tdoa_gw.h`**

Directly above `#define TDOA_DT_REORDER_MAX_MS  1000`, add:

```c
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
```

Replace the `/* ---- REMOVED 2026-09-06: the per-tag EKF ---...` block (ending
`...the next one needs them. */`) with:

```c
/* The per-tag EKF that ran here 2026-09-02..06 is gone; its replacement is
 * pos_abg (src/pos_abg.h), an alpha-beta-gamma filter on the SOLVED position.
 * docs/superpowers/specs/2026-09-06-abg-position-filter-design.md. */
```

After `uint32_t tdoa_gw_reorder_count(void);` add:

```c
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
```

- [ ] **Step 3: `tdoa_gw.c` — includes, memo, cfg, counters, init**

Add includes (alphabetical, after `#include "apos_store.h"` and after
`#include "net_uplink.h"` respectively):

```c
#include "blink_frame.h"   /* BLINK_FLAG_MOVING */
```
```c
#include "pos_abg.h"
```

In `struct tag_memo`, after `bool     has_ref_t;`, add:

```c
	/* The filter (spec §4.1). x/y/pos_ms/has_pos above keep their meaning:
	 * the last PUBLISHED position, which is what seeds tdoa_solve() and
	 * jump-gates it; they follow the filter's output at publish time. */
	struct pos_abg abg;
	/* Latched from the last observation's BLINK_FLAG_MOVING in
	 * ingest_one(). Every observation of one blink carries the same flags
	 * byte (one frame, several anchors), so the last to arrive is as good
	 * as any. Its ABSENCE (proto-4 anchor) reads as still -- see the
	 * `still` counter. */
	bool     tag_moving;
```

After `static struct tag_memo     memo[TDOA_GW_SEED_SLOTS];`, add:

```c
/* One shared filter config for every tag -- there is no per-tag tuning.
 * Written only by the shell setters below (k_sched_lock()-fenced) and read
 * by solve_one(). abg_lambda is what the last set_lambda() was given, kept
 * only so `blink abg` can print it back. */
static struct pos_abg_cfg abg_cfg;
static float abg_lambda = 0.02f;
```

After `static bool warned_reorder;`, add:

```c
static uint32_t n_abg_seeded;
static uint32_t n_abg_dt_reseed;
static uint32_t n_abg_filtered;
static uint32_t n_abg_gate_rejected;
static uint32_t n_abg_reseed;
static uint32_t n_abg_still;
static bool warned_gate;
static bool warned_dt_reseed;
```

In `tdoa_gw_init()`, after `memset(memo, 0, sizeof(memo));` add
`pos_abg_cfg_defaults(&abg_cfg);` and `abg_lambda = 0.02f;`; after
`warned_reorder = false;` add:

```c
	n_abg_seeded        = 0u;
	n_abg_dt_reseed     = 0u;
	n_abg_filtered      = 0u;
	n_abg_gate_rejected = 0u;
	n_abg_reseed        = 0u;
	n_abg_still         = 0u;
	warned_gate         = false;
	warned_dt_reseed    = false;
```

- [ ] **Step 4: `ingest_one()` — latch the MOVING bit**

Replace the comment block that begins `/* obs.flags (BLINK_FLAG_MOVING,
BLINK_FLAG_ALERT) is parsed and NOT` (ending `last to arrive is as good as
any). */`) with:

```c
	/* Anything that rides on the OBSERVATION rather than on the geometry
	 * is remembered per tag here and read back when the fix is built --
	 * the collector carries only struct tdoa_meas. */
	mm->tag_moving = (obs.flags & BLINK_FLAG_MOVING) != 0u;
```

- [ ] **Step 5: `solve_one()` — dt classification and the filter step**

Replace, inside `solve_one()`, everything from `if (mm->has_ref_t) {` through
`mm->has_ref_t      = true;` with:

```c
	/* dt classification. dt_ok means "predict through it"; anything else
	 * that is not dropped outright reseeds. All from the DTU clock -- never
	 * now_ms, which is quantized to the superframe and would fabricate
	 * velocity. */
	bool  dt_ok = false;
	float dt_s  = 0.0f;

	if (mm->has_ref_t) {
		int64_t raw_dt = sdelta40((uint64_t)fix_t_dtu,
					 (uint64_t)mm->last_ref_t_dtu);

		dt_s = (float)raw_dt * TDOA_GW_S_PER_DTU;

		/* A small NEGATIVE dt is a group released out of order -- it
		 * describes an instant EARLIER than this tag's last fix.
		 * Discard it whole: do not publish it (the trace would step
		 * backwards in time), do not step the filter, and do NOT
		 * advance last_ref_t_dtu. Letting it advance rewinds the
		 * reference, so the next group's dt spans time already
		 * integrated -- measured 2026-09-03 as three groups in one
		 * cycle predicting 1.2 s for 200 ms of real elapsed time.
		 *
		 * A dt under POS_ABG_DT_MIN_S is the same case from the other
		 * side: no two distinct blinks of one tag are that close, so it
		 * is a duplicate group of one blink, and beta/dt would blow up
		 * on it. Same treatment, same counter.
		 *
		 * A LARGE negative dt is a forward gap that aliased through
		 * sdelta40()'s sign boundary: the group is genuinely new, the
		 * reference must advance, and the filter reseeds rather than
		 * predicting. TDOA_DT_REORDER_MAX_MS carries the derivation of
		 * where the two populations separate. */
		if ((raw_dt <= 0 &&
		     -dt_s <= ((float)TDOA_DT_REORDER_MAX_MS / 1000.0f)) ||
		    (raw_dt > 0 && dt_s < POS_ABG_DT_MIN_S)) {
			n_reorder++;
			if (!warned_reorder) {
				warned_reorder = true;
				LOG_WRN("blink from 0x%04X released out of "
					"order (dt %d ms): group discarded, "
					"time reference held. Warned ONCE; "
					"`blink stats` carries the count",
					tag_addr, (int)(dt_s * 1000.0f));
			}
			return true;
		}
		dt_ok = (raw_dt > 0) &&
			(dt_s <= ((float)TDOA_DT_MAX_MS / 1000.0f));
	}
	mm->last_ref_t_dtu = fix_t_dtu;
	mm->has_ref_t      = true;
```

Then replace everything from `if (!resolve_one(mm, m, n, tag_addr, now_ms,
&res)) {` through `fix.batt_soc   = mm->batt_soc;` with:

```c
	if (!resolve_one(mm, m, n, tag_addr, now_ms, &res)) {
		/* Already counted and warned (solve_fail or jump). Nothing is
		 * published for this group and the filter is not stepped: a
		 * republished previous fix would be the stale-republish bug of
		 * 2026-09-02 all over again. */
		return true;
	}

	/* The filter (spec §4.2). Every path that publishes goes through
	 * pos_abg_get() below; every path that does not returns here. */
	if (!pos_abg_get(&mm->abg, NULL, NULL, NULL, NULL)) {
		pos_abg_seed(&mm->abg, res.x, res.y);
		n_abg_seeded++;
	} else if (!dt_ok) {
		pos_abg_seed(&mm->abg, res.x, res.y);
		n_abg_dt_reseed++;
		if (!warned_dt_reseed) {
			warned_dt_reseed = true;
			LOG_WRN("blink from 0x%04X: dt %d ms outside "
				"(%d..%d] ms, filter reseeded on the fresh "
				"solve. Warned ONCE; `blink stats` carries "
				"the count. A high count means slow-tier "
				"tags or marginal coverage, not a bug",
				tag_addr, (int)(dt_s * 1000.0f),
				(int)(POS_ABG_DT_MIN_S * 1000.0f),
				(int)TDOA_DT_MAX_MS);
		}
	} else {
		bool still = !mm->tag_moving;

		switch (pos_abg_step(&mm->abg, &abg_cfg, dt_s, res.x, res.y,
				     still)) {
		case POS_ABG_ACCEPTED:
			n_abg_filtered++;
			if (still) {
				n_abg_still++;
			}
			break;
		case POS_ABG_RESEEDED:
			n_abg_reseed++;
			break;
		case POS_ABG_REJECTED:
			n_abg_gate_rejected++;
			if (!warned_gate) {
				warned_gate = true;
				LOG_WRN("blink from 0x%04X: solve (%.2f, "
					"%.2f) is more than %.1f m from the "
					"filter's prediction - not published. "
					"Warned ONCE; `blink stats` carries "
					"the count", tag_addr, (double)res.x,
					(double)res.y,
					(double)abg_cfg.gate_m);
			}
			return true;
		case POS_ABG_BAD_INPUT:
		default:
			/* dt_ok guarantees dt > 0 and the solve guarantees a
			 * finite (x, y), so this is unreachable; counted as a
			 * solve failure rather than silently published. */
			n_solve_fail++;
			return true;
		}
	}

	if (!pos_abg_get(&mm->abg, &fix.x, &fix.y, NULL, NULL)) {
		/* Defensive only: every branch above leaves the filter seeded. */
		return true;
	}

	fix.src_addr   = tag_addr;
	fix.n_anchors  = (uint8_t)n;
	fix.batt_soc   = mm->batt_soc;
```

And replace the memo update:

```c
	/* The seed/jump-gate memo follows the position actually published. */
	mm->x       = res.x;
	mm->y       = res.y;
```

with

```c
	/* The seed/jump-gate memo follows the position actually PUBLISHED --
	 * the filter's, not the raw solve's. */
	mm->x       = fix.x;
	mm->y       = fix.y;
```

- [ ] **Step 6: `tdoa_gw.c` — stats and config accessors**

After `uint32_t tdoa_gw_reorder_count(void) { ... }` add:

```c
void tdoa_gw_abg_stats(uint32_t *n_seeded, uint32_t *n_dt_reseed,
		       uint32_t *n_filtered, uint32_t *n_gate_rejected,
		       uint32_t *n_reseed, uint32_t *n_still)
{
	if (n_seeded != NULL)        { *n_seeded = n_abg_seeded; }
	if (n_dt_reseed != NULL)     { *n_dt_reseed = n_abg_dt_reseed; }
	if (n_filtered != NULL)      { *n_filtered = n_abg_filtered; }
	if (n_gate_rejected != NULL) { *n_gate_rejected = n_abg_gate_rejected; }
	if (n_reseed != NULL)        { *n_reseed = n_abg_reseed; }
	if (n_still != NULL)         { *n_still = n_abg_still; }
}

const struct pos_abg_cfg *tdoa_gw_abg_cfg(void)
{
	return &abg_cfg;
}

float tdoa_gw_abg_lambda(void)
{
	return abg_lambda;
}

/* The shell (preemptible) writes; the K_PRIO_COOP(0) loop reads the fields
 * one by one and CAN preempt the shell between two stores, so a three-field
 * lambda change is fenced -- same fence, same reason as
 * ccp_slave_residual_reset(). Single-field setters get the same fence for
 * uniformity; it costs nothing. */
void tdoa_gw_abg_set_lambda(float lambda)
{
	struct pos_abg_cfg tmp = abg_cfg;

	pos_abg_cfg_from_index(&tmp, lambda);
	k_sched_lock();
	abg_cfg.alpha = tmp.alpha;
	abg_cfg.beta  = tmp.beta;
	abg_cfg.gamma = tmp.gamma;
	abg_lambda    = lambda;
	k_sched_unlock();
}

void tdoa_gw_abg_set_gamma(float gamma)
{
	k_sched_lock();
	abg_cfg.gamma = gamma;
	k_sched_unlock();
}

void tdoa_gw_abg_set_gate(float gate_m)
{
	k_sched_lock();
	abg_cfg.gate_m = gate_m;
	k_sched_unlock();
}

void tdoa_gw_abg_set_alpha_still(float alpha_still)
{
	k_sched_lock();
	abg_cfg.alpha_still = alpha_still;
	k_sched_unlock();
}

void tdoa_gw_abg_set_reset_after(uint8_t n)
{
	k_sched_lock();
	abg_cfg.reset_after = n;
	k_sched_unlock();
}
```

- [ ] **Step 7: `blink_shell.c` — third line, verdicts, `blink abg`**

Add includes: `#include "pos_abg.h"` (after `#include "net_uplink.h"`) and
`#include <stdlib.h>` (for `strtof`, next to `<stdint.h>`).

In `cmd_stats()`, extend the declarations with:

```c
	uint32_t a_seeded = 0, a_dt_reseed = 0, a_filtered = 0;
	uint32_t a_gate = 0, a_reseed = 0, a_still = 0;
```

after `s_reorder = tdoa_gw_reorder_count();` add:

```c
	tdoa_gw_abg_stats(&a_seeded, &a_dt_reseed, &a_filtered, &a_gate,
			  &a_reseed, &a_still);
```

Replace the comment beginning `/* A third line, \`tdoa_ekf\`, carried the
per-tag EKF's counters` (through `tolerates the missing line. */`) with:

```c
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
```

Add the `blink abg` handler before `SHELL_STATIC_SUBCMD_SET_CREATE(sub_blink,`:

```c
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
```

(`strcmp` needs `#include <string.h>`; add it back next to `<stdlib.h>`.
`UWB_MODE_GATEWAY` comes from `uwb_config.h`, already included; `EINVAL`/
`EPERM` from `<errno.h>`, add it.)

In `SHELL_STATIC_SUBCMD_SET_CREATE(sub_blink, ...)`, change the `stats` help
text `"in two lines"` → `"in three lines"` and `"ingest/solve/publish
counters. Both carry a role field"` → `"ingest/solve/publish counters, then
the position filter's own counters. All three carry a role field"`, and add
after the `stats` entry:

```c
	SHELL_CMD_ARG(abg, NULL,
		      "abg [lambda|gamma|gate|still|reset <value>] — read or "
		      "tune the alpha-beta-gamma position filter at runtime "
		      "(gamma 0 = alpha-beta). NOT persisted; GATEWAY only "
		      "for the setters",
		      cmd_abg, 1, 2),
```

- [ ] **Step 8: Build both images, check size and warnings**

```powershell
$env:ZEPHYR_BASE = "C:\Users\jolap\zephyrproject\zephyr"
$env:ZEPHYR_SDK_INSTALL_DIR = "C:\Users\jolap\zephyr-sdk-1.0.1"
west build -b ancla_esp32s3/esp32s3/procpu 2>&1 | Select-String -Pattern "warning|dram0_0_seg|error"
west build -b ancla_esp32s3/esp32s3/procpu --pristine -d build_cal -- "-DEXTRA_CONF_FILE=cal.conf" 2>&1 | Select-String -Pattern "warning|dram0_0_seg|error"
```

Expected: both link, zero warnings; production `dram0_0_seg` within
**271376 + ~600 B** (16 × 32 B memo growth + cfg + counters). If `cbrt` is
undefined at link time, the libc lacks it: add `CONFIG_FPU=y`-independent
fallback `pow(x, 1.0/3.0)` with sign handling in `pos_abg_cfg_from_index()`
and re-run the Task 1 host suite (the Kalman cross-check will catch any
precision loss).

- [ ] **Step 9: Re-run every host suite on the path**

```bash
for t in pos_abg tdoa_collect tdoa_solve tdoa_dtu pos_json_blink blink_frame; do echo "== $t"; done
gcc -Wall -Wextra -Isrc -o tests/pos_abg/test_pos_abg.exe tests/pos_abg/test_pos_abg.c src/pos_abg.c -lm && ./tests/pos_abg/test_pos_abg.exe
gcc -Wall -Wextra -Isrc -o tests/tdoa_collect/test_tdoa_collect.exe tests/tdoa_collect/test_tdoa_collect.c src/tdoa_collect.c src/tdoa_solve.c src/pos_residual.c -lm && ./tests/tdoa_collect/test_tdoa_collect.exe
gcc -Wall -Wextra -Isrc -o tests/tdoa_solve/test_tdoa_solve.exe tests/tdoa_solve/test_tdoa_solve.c src/tdoa_solve.c src/pos_residual.c -lm && ./tests/tdoa_solve/test_tdoa_solve.exe
```

Expected: all `ALL TESTS PASSED`.

- [ ] **Step 10: Commit**

```bash
git add CMakeLists.txt src/tdoa_gw.h src/tdoa_gw.c src/blink_shell.c
git commit -m "feat(tdoa): alpha-beta-gamma filter on the solved position, tdoa_abg stats and blink abg knob"
```

---

### Task 4: `tools/pos_trace.py` learns the `tdoa_abg` line

**Files:**
- Modify: `tools/pos_trace.py:57` (`STATS_RE`) and `:349-366` (the stats
  summary block in `main()`)

**Interfaces:**
- Consumes: the `tdoa_abg` JSON line from Task 3 with keys `seeded`,
  `dt_reseed`, `filtered`, `gate_rejected`, `reseed`, `still`; the `tdoa`
  line's `fixes`.

- [ ] **Step 1: Write a failing check**

Create a two-line fixture `tools/testdata/abg_stats_sample.txt` (new
directory) containing exactly:

```
[00:01:00.000,000] <inf> gw: {"tdoa":{"role":"gateway","ingested":40,"rejected":0,"reject_dup":0,"reject_shed":0,"fixes":10,"no_anchor":0,"implausible":0,"solve_fail":0,"jump":0,"reorder":1}}
[00:01:00.100,000] <inf> gw: {"tdoa_abg":{"role":"gateway","seeded":2,"dt_reseed":1,"filtered":6,"gate_rejected":3,"reseed":1,"still":4}}
```

Run: `python tools/pos_trace.py tools/testdata/abg_stats_sample.txt`

Expected today: the `tdoa_abg` line is not listed under "last stats line of
each kind" and no identity line is printed (the regex does not match it).

- [ ] **Step 2: Implement**

Change line 57 to:

```python
STATS_RE = re.compile(r'\{"(blink|tdoa|tdoa_abg)":\{(.+?)\}\}')
```

Replace the block from `for kind in ("blink", "tdoa", "tdoa_ekf"):` through
the `except (KeyError, ValueError):\n                pass` with:

```python
        for kind in ("blink", "tdoa", "tdoa_abg"):
            if kind in seen:
                print("  %-9s %s" % (kind, seen[kind]))
        a = seen.get("tdoa_abg")
        t = seen.get("tdoa")
        if a and t:
            try:
                s, d, f, r = (int(a["seeded"]), int(a["dt_reseed"]),
                              int(a["filtered"]), int(a["reseed"]))
                g, st = int(a["gate_rejected"]), int(a["still"])
                fx = int(t["fixes"])
                print("  fixes(%d) vs seeded+dt_reseed+filtered+reseed(%d): %s"
                      % (fx, s + d + f + r,
                         "balances" if fx == s + d + f + r
                         else "MISMATCH -- a published fix with no filter "
                              "step behind it, see tdoa_gw.c solve_one()"))
                if f and st == f:
                    print("  still == filtered (%d): the MOVING bit is not "
                          "arriving -- proto-4 anchors?" % f)
                if f and g > f // 4:
                    print("  gate_rejected (%d) is over a quarter of filtered "
                          "(%d): gate too tight or the solve is throwing "
                          "outliers" % (g, f))
            except (KeyError, ValueError):
                pass
```

- [ ] **Step 3: Run the check**

Run: `python tools/pos_trace.py tools/testdata/abg_stats_sample.txt`

Expected output contains:

```
  tdoa_abg  {'role': 'gateway', 'seeded': '2', ...}
  fixes(10) vs seeded+dt_reseed+filtered+reseed(10): balances
  gate_rejected (3) is over a quarter of filtered (6): gate too tight or the solve is throwing outliers
```

(the exact dict rendering follows whatever `parse()` already produces for the
other kinds). Then edit the fixture's `"fixes":10` to `11`, re-run, and
confirm `MISMATCH` prints; restore it to 10.

- [ ] **Step 4: Commit**

```bash
git add tools/pos_trace.py tools/testdata/abg_stats_sample.txt
git commit -m "tools(pos_trace): parse the tdoa_abg line and check the publish identity"
```

---

### Task 5: Record what shipped

**Files:**
- Modify: `CLAUDE.md` — the "STATUS 2026-09-06" paragraph under "Precisión y
  suavizado", the `src/tdoa_gw.{c,h}` layout entry's "REMOVED 2026-09-06"
  paragraph, the `src/pos_ekf.{c,h}` REMOVED entry, the Console section's
  `blink` tree, the Host tests list
- Modify: `docs/superpowers/specs/2026-09-06-abg-position-filter-design.md`
  header (status line)

- [ ] **Step 1: CLAUDE.md, "Precisión y suavizado" status paragraph**

Change `**The new filter is NOT implemented; the removal is.**` to
`**The new filter is implemented and build-verified (Tasks 1-4 of the plan);
Task 6, the bench evaluation and the λ pick, is [open / done on <date>] —
update this sentence when it closes.**` — fill in whichever is true at the
time of the edit; do not leave both alternatives.

- [ ] **Step 2: CLAUDE.md, layout**

Add after the `src/tdoa_gw.{c,h}` entry's REMOVED paragraph:

```markdown
  **Since 2026-09-06 (implemented per
  `docs/superpowers/plans/2026-09-06-abg-position-filter.md`), each tag's
  memo carries a `struct pos_abg`** and `solve_one()` runs solve → dt
  classification → `pos_abg_step()` → publish `pos_abg_get()`. Publish
  identity `fixes == seeded + dt_reseed + filtered + reseed` (third
  `blink stats` line, `tdoa_abg`; `tools/pos_trace.py` checks it). A
  gate-rejected cycle publishes NOTHING. `TDOA_DT_MAX_MS` is 1000 (p90 of
  the measured dt is 0.8 s); a dt under `POS_ABG_DT_MIN_S` (50 ms) is a
  duplicate group and counts as `reorder`. `blink abg` tunes the shared cfg
  at runtime, not persisted, `k_sched_lock()`-fenced.
```

Add a new layout entry after the `src/pos_ekf.{c,h}` REMOVED entry:

```markdown
- `src/pos_abg.{c,h}` — alpha-beta-gamma filter on a SOLVED position, state
  `[x,y,vx,vy,ax,ay]`, fixed gains from ONE tracking index via Gray-Murray
  (`pos_abg_cfg_from_index()`), Euclidean innovation gate, streak reseed,
  still-freeze. **Gain convention: `a += (gamma/dt^2)·r`** — the other
  common form doubles gamma; `tests/pos_abg/` pins the closed form against a
  steady-state Kalman recursion so the trap cannot come back silently. Pure
  C, `<math.h>` only (`double` in the one-shot index conversion, `float`
  everywhere else), host-tested in `tests/pos_abg/`. Spec:
  `docs/superpowers/specs/2026-09-06-abg-position-filter-design.md`.
```

- [ ] **Step 3: CLAUDE.md, Console and Host tests**

In the `blink` tree description, after the `blink stats` entry, add:

```
blink abg [lambda|gamma|gate|still|reset <v>]
                               read or tune the position filter at runtime
                               (gamma 0 = alpha-beta, see the spec's §3.3
                               finding).
                               NOT persisted — a reboot returns to
                               pos_abg_cfg_defaults(); a chosen value goes
                               into those defaults, in source. Setters are
                               GATEWAY-only
```

In the Host tests list, before the closing ``` of the gcc block, add:

```
gcc -Wall -Wextra -Isrc -o tests/pos_abg/test_pos_abg.exe tests/pos_abg/test_pos_abg.c src/pos_abg.c -lm
./tests/pos_abg/test_pos_abg.exe                # pos_abg: ALL TESTS PASSED, exits 0
```

and delete the sentence about `tests/pos_ekf/` having been removed if it
reads oddly next to it (keep the fact; it may become one clause).

- [ ] **Step 4: Spec status line**

Under the spec's `**Branch:**` line, replace the parenthetical with:
`(implemented 2026-09-xx per the plan; hardware evaluation status in CLAUDE.md)`.

- [ ] **Step 5: Commit**

```bash
git add CLAUDE.md docs/superpowers/specs/2026-09-06-abg-position-filter-design.md
git commit -m "docs: record the alpha-beta-gamma filter as shipped, blink abg, pos_abg host test"
```

---

### Task 6: Hardware evaluation and the λ pick *(bench, no code unless it fails)*

**Files:** none unless a defect is found. Results go into CLAUDE.md's status
paragraph (Task 5, Step 1) and, if a default changes, into
`pos_abg_cfg_defaults()` with the Task 1 `test_defaults()` numbers updated to
match.

**Prerequisites:** the deployed gateway on USB-C (not battery — CCP
constraint), the four surveyed anchors up, two tags: one placed at the
anchor triangle's CENTRE (not on the base line — CLAUDE.md's unobservable-y
note), one worn. `tools/pos_trace.py` on the PC, the console captured to a
file.

- [ ] **Step 1: Baseline the RAW solve, BEFORE flashing this build**

The identity filter (`alpha = 1, beta = gamma = 0`) is not reachable through
`blink abg` (`from_index` never produces it), so the raw baseline is taken on
the image currently deployed — the post-EKF-removal build, which publishes
raw solves. Five-minute stationary capture of the console, then
`tools/pos_trace.py <capture>`. Record: RMS about the mean of (x, y), and the
count of consecutive-fix jumps > 1.5 m. Without this number nothing in the
steps below can be called an improvement.

- [ ] **Step 2: Flash, cold boot, confirm the identity**

`west flash`, `kernel reboot cold`, wait 2 min with both tags live, then
`blink stats`. Confirm `fixes == seeded + dt_reseed + filtered + reseed` and
`still < filtered` (the worn tag must produce non-still cycles; `still ==
filtered` here means the MOVING bit is not on the air — stop and check the
anchors' `UWB_PROTO_VER`).

- [ ] **Step 3a: γ on versus γ off — the spec §3.3 finding, on hardware**

At the default λ = 0.02, two five-minute stationary captures back to back:
first as flashed (γ ≈ 0.015), then after `blink abg gamma 0`. Same tag, same
spot. Record for each: RMS about the mean, jumps > 1.5 m, and the `tdoa_abg`
deltas (`gate_rejected`, `reseed`). The simulation predicts γ = 0 wins on
every one of those numbers (table in spec §3.3). Then one lap of the room with
the worn tag under each setting, watching the map for overshoot at the end of
the walk and for excursions. **Whichever γ setting wins is the one every step
below runs under; if it is γ = 0, note it now for Step 6.**

- [ ] **Step 3b: Stationary dispersion at three λ**

Under the γ setting chosen in 3a, for each of `blink abg lambda 0.01`,
`0.02`, `0.05` (re-apply `blink abg gamma 0` after each `lambda`, since
`lambda` recomputes γ): five minutes with the stationary tag, capture,
`tools/pos_trace.py`. Record RMS and jump count against Step 1's raw
baseline. Expect RMS ≈ 0.6–0.9 × raw under real dt jitter (0.55–0.70 is the
constant-dt figure) and zero jumps > 1.5 m. Check `gw_sf` heartbeat stays at
200.0 ms with no `"beacon started but TXFRS never completed"` in the whole
capture.

- [ ] **Step 4: Walking trace at the same three λ**

Two laps of the room per λ with the worn tag, under the γ setting from 3a. On the platform map and in the
capture: every consecutive 200 ms step under ~30 cm, no back-steps, no
gap-then-teleport inside the hull. Note the overshoot at the end of a walk
(the spec predicts ~20–25 % of the last step's displacement); if it is
visually objectionable at every λ, `alpha_still` and the gate are the levers,
not λ.

- [ ] **Step 5: Injected outlier, via one anchor's clock**

The gateway positions anchors from its own `apos` store, which has no runtime
setter, so a coordinate error cannot be injected cleanly; a hand over an
antenna moves a LOS timestamp by centimetres, not metres. The reproducible
injection is a **sync disturbance on one anchor**: with the stationary tag
being tracked, run `sync reset` on one anchor's console, then within a few
seconds `kernel reboot cold` on that same anchor. Its `sync_model`
re-baselines from zero, and its first observations after re-lock carry a
re-converging clock (CLAUDE.md's "un-reset reading came back 3.7x too
high"). Expected on the gateway: `gate_rejected` moves by a handful of
cycles, `implausible` may move too (the DTU bound upstream doing its job —
expected, not a fault), and the map shows NO excursion beyond `gate_m`.
Record the counter deltas.

- [ ] **Step 6: Pick, and close the loop**

Choose λ, γ (on or 0, from 3a) and `alpha_still`/`gate_m` if either moved,
by eye plus the dispersion numbers. Put the chosen values into
`pos_abg_cfg_defaults()` (a γ = 0 default is `c->gamma = 0.0f;` AFTER the
`pos_abg_cfg_from_index()` call, with a comment naming the bench date), fix
`test_defaults()` in `tests/pos_abg/test_pos_abg.c` to the new numbers,
re-run the host suite, rebuild both images, update CLAUDE.md's status
paragraph with the figures (RMS raw vs filtered at the chosen λ, jump counts,
the `blink stats` identity holding over the run), and commit:

```bash
git add src/pos_abg.c tests/pos_abg/test_pos_abg.c CLAUDE.md
git commit -m "tune(pos_abg): lambda <value> from the 2026-09-xx bench evaluation"
```

If no default changed, commit CLAUDE.md alone with the figures.

---

## Self-review against the spec

- §2 solve every group, seed/jump gate on the raw solve, dt from DTU — Task 3
  Step 5 (`resolve_one()` unchanged, `dt_ok` from `sdelta40`).
- §3.1 state/cfg — Task 1 header. §3.2 predict-always/correct-if-accepted,
  no publish on reject, Euclidean gate, streak reseed with `reset_after = 0`
  = never, still-freeze, fixed gains with actual dt, `BAD_INPUT` — Task 2
  Step 3 and Task 3 Step 5 (`REJECTED → return true` before the publish).
- §3.3 gains from index, Kalman cross-check, table rows — Task 1 Steps 2/4;
  the "Finding" — Task 2's `test_gamma_zero_*`, Task 3's `gamma` knob, Task 6
  Step 3a. §3.2 dt-scaled gate — Task 2 Step 3 and `test_gate_scales_with_dt`.
- §3.4 API — Task 1 header; no `pos_sigma()`.
- §4.1 memo fields and shared cfg — Task 3 Step 3. §4.2 flow and the three
  dt outcomes incl. `< POS_ABG_DT_MIN_S` as reorder — Task 3 Step 5.
  §4.3 residual and memo follow the published position — Task 3 Step 5.
  §4.4 stats line and two verdicts — Task 3 Step 7. §4.5 knob with
  `k_sched_lock()` — Task 3 Steps 6/7.
- §5 edge table — every row maps to a branch in Task 3 Step 5 or Task 2;
  memo eviction and reboot need no code (memo zeroed ⇒ `pos_abg_get()`
  false ⇒ `seeded`).
- §6 host tests 1–10 — Task 1 (1, 2, 3, 4, 9) and Task 2 (5, 6, 7, 8, 10).
  §6 integration — Task 6. §7 budget — Task 3 Step 8's size check.
- Type consistency: `tdoa_gw_abg_stats()` six `uint32_t *` in the order
  seeded, dt_reseed, filtered, gate_rejected, reseed, still — identical in
  Task 3 Steps 2, 6, 7 and in Task 4's key names.
