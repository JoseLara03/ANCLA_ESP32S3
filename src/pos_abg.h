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
