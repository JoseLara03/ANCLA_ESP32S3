#ifndef POS_RESIDUAL_H_
#define POS_RESIDUAL_H_

#include <stddef.h>
#include "pos_solver.h"

/* RMS of (predicted range - measured range) over the first `n` measurements,
 * in metres, for the solved position (x, y). The predicted range is the 3D
 * slant distance sqrt(dx^2 + dy^2 + dz_i^2) -- see struct pos_meas in
 * pos_solver.h for why dz lives inside the model rather than being projected
 * out beforehand. Zero means the ranges are exactly self-consistent; a large
 * value means at least one range disagrees with the fix, which is the signal
 * a backend needs to decide whether to trust it.
 *
 * Returns 0.0f when n == 0.
 *
 * Deliberately separate from pos_solver.c: that file used to include
 * arm_math.h, which cannot compile on the host. pos_solver.c no longer needs
 * CMSIS-DSP either (the 2x2 normal equations are inverted in closed form),
 * but the split is kept -- this needs only sqrtf, and is host tested in
 * tests/pos_residual/ independently of the solver's Gauss-Newton loop. */
float pos_residual_rms(const struct pos_meas *m, size_t n, float x, float y);

#endif /* POS_RESIDUAL_H_ */
