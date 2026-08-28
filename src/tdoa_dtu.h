/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Turn a group's ABSOLUTE 40-bit master-base timestamps into signed
 * differences, and bound them.
 *
 * struct tdoa_meas.t_dtu as it leaves blink_rx.c is an absolute DW3220
 * timestamp in the master's base: 40 bits, wrapping every ~17.2 s. tdoa_solve()
 * consumes only (t_i - t_0), so two observations of the SAME blink landing on
 * opposite sides of that wrap differ by ~2^40 DTU -- about 5.16 MILLION km of
 * path
 * difference -- and the solver has no way to tell that from a measurement.
 *
 * Neither tdoa_collect (which only groups) nor tdoa_solve (already delivered
 * and tested against rebased inputs) is the right place for this, so it lives
 * here: the project's global "every timestamp comparison is a signed
 * difference" rule, applied at modulo 2^40 instead of 2^32.
 *
 * Pure C, no Zephyr, host-tested in tests/tdoa_dtu/.
 */

#ifndef TDOA_DTU_H
#define TDOA_DTU_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "tdoa_solve.h"   /* struct tdoa_meas */

/* The DW3220 timestamp counter is 40 bits. */
#define TDOA_DTU_MODULO  (1LL << 40)

/* Largest |t_dtu| a rebased group may carry, INCLUSIVE. 32768 DTU x
 * TDOA_M_PER_DTU = 153.7 m of path difference: far above any cell this MAC can
 * cover (four anchors inside one 200 ms superframe's ranging budget) and far
 * below any wrap artefact or broken-sync value. This is the ONLY sanity filter
 * standing in front of the solver, and it is needed precisely because
 * tdoa_solve()'s own residual cannot be one -- it is zero by construction at
 * TDOA_MIN_ANCHORS, and convergence there is judged by step size only, so a
 * tag outside the anchor hull can settle on the mirror branch and still report
 * valid. A residual filters none of that; a physical bound filters the gross
 * case. */
#define TDOA_DTU_MAX_SPREAD  32768LL

/* Rebase every t_dtu on m[0].t_dtu, reducing modulo 2^40 into
 * [-2^39, 2^39) so the result is the SIGNED difference. m[0].t_dtu becomes 0.
 * Geometry fields (x, y, dz) are untouched. No-op for m == NULL or n == 0.
 *
 * Idempotent for an already-rebased group (m[0] is 0, so the base is 0). */
void tdoa_dtu_rebase(struct tdoa_meas *m, size_t n);

/* True when every |t_dtu| is <= TDOA_DTU_MAX_SPREAD. Call AFTER
 * tdoa_dtu_rebase(); on a raw group it will (correctly) reject anything that
 * crossed the wrap, which is the defence-in-depth that makes a forgotten
 * rebase visible instead of silent. False for m == NULL or n == 0. */
bool tdoa_dtu_plausible(const struct tdoa_meas *m, size_t n);

#endif /* TDOA_DTU_H */
