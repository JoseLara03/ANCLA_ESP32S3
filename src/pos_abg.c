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
