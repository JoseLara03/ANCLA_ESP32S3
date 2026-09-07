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
	float T, T2, xp, yp, vxp, vyp, rx, ry, gate;

	if (!f->init || !(dt_s > 0.0f) || !isfinite(dt_s) ||
	    !isfinite(zx) || !isfinite(zy)) {
		return POS_ABG_BAD_INPUT;
	}

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
