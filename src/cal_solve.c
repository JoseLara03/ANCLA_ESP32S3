/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cal_solve.h"

#include "cal_math.h"

#include <errno.h>

int cal_solve_tx_delay(int32_t measured_mm, int32_t ref_mm,
		       uint16_t cur_tx, uint16_t cur_rx, uint16_t *out_tx)
{
	/* Saturation check FIRST. cal_solve_step() bounds one correction to
	 * +/-CAL_MAX_STEP_UNITS, and a saturated step lands inside the
	 * CAL_TX_DLY_MIN..MAX window below -- so checking the range alone would
	 * report a bounded nudge as a converged solution. See CAL_MAX_STEP_MM
	 * in cal_solve.h for the worked example that motivated this.
	 *
	 * Reported as the bound the measurement was pushing toward: a reading
	 * that is too FAR means the delay is too small, so tx must rise. */
	int32_t err_mm = measured_mm - ref_mm;

	if (err_mm > CAL_MAX_STEP_MM) {
		*out_tx = (uint16_t)CAL_TX_DLY_MAX;
		return -ERANGE;
	}
	if (err_mm < -CAL_MAX_STEP_MM) {
		*out_tx = (uint16_t)CAL_TX_DLY_MIN;
		return -ERANGE;
	}

	/* The solve is on the combined delay -- that is the only observable
	 * quantity -- and the whole correction is then applied to tx, because
	 * cur_rx is pinned at UWB_ANT_DELAY_DEFAULT by the calling procedure. */
	uint32_t cur_total = (uint32_t)cur_tx + (uint32_t)cur_rx;
	uint16_t new_total = cal_solve_step(measured_mm, ref_mm,
					    (uint16_t)cur_total);

	int32_t new_tx = (int32_t)new_total - (int32_t)cur_rx;

	if (new_tx < (int32_t)CAL_TX_DLY_MIN) {
		*out_tx = (uint16_t)CAL_TX_DLY_MIN;
		return -ERANGE;
	}
	if (new_tx > (int32_t)CAL_TX_DLY_MAX) {
		*out_tx = (uint16_t)CAL_TX_DLY_MAX;
		return -ERANGE;
	}

	*out_tx = (uint16_t)new_tx;
	return 0;
}
