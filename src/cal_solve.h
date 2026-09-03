/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The one piece of arithmetic this project adds on top of the tag's solver
 * (cal_math.c, copied verbatim): convert a solved COMBINED antenna delay back
 * into a TX-only value, holding ant_delay_rx fixed.
 *
 * Splitting is legitimate because only ant_tx + ant_rx is observable. The
 * responder derives its delayed TX time from poll_rx_ts, so raising ant_rx
 * makes it physically transmit earlier by exactly as much as raising ant_tx
 * makes it report a longer turnaround -- both move the initiator's result by
 * half a tick per unit. See the design spec, section 3.1; note that CLAUDE.md's
 * "RX_ANT_DLY cancels in RTD_resp" reaches the right conclusion by the wrong
 * route, and the wrong route makes ant_delay_rx look like a free parameter.
 *
 * Pure C with no Zephyr and no driver dependency, so it is host-testable like
 * uwb_config.c and beacon_guard.c.
 */

#ifndef CAL_SOLVE_H
#define CAL_SOLVE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Accepted ant_delay_tx range, about +/- 5.9 m of correction around the 16385
 * factory seed. Matches the sibling ESP-IDF project's DLY_MIN/DLY_MAX
 * (ESP32S3UWB/src/anchor_cal.c). A result outside it means the measurement was
 * wrong, not that the board needs that much trim. */
#define CAL_TX_DLY_MIN 14000u
#define CAL_TX_DLY_MAX 19000u

/* Largest measurement error a SINGLE cal_solve_step() call can fully correct:
 * CAL_MAX_STEP_UNITS at CAL_MM_PER_UNIT_X1000/1000 mm per unit = 4680 mm.
 *
 * Derived from cal_math.h's own exported constants rather than restated, so
 * this cannot drift from the solver it describes -- which matters because
 * cal_math.{c,h} is a verbatim copy owned by the tag and may change there.
 *
 * Why this guard has to exist at all, and why it must be checked BEFORE the
 * CAL_TX_DLY_MIN/MAX range test:
 *
 * cal_solve_step() bounds one iteration's correction to +/-CAL_MAX_STEP_UNITS
 * so a corrupted sample cannot swing the delay by ~32000 units in one step.
 * Good. But a SATURATED step does not return the solution -- it returns a
 * bounded nudge toward it, which lands comfortably inside
 * CAL_TX_DLY_MIN..CAL_TX_DLY_MAX and is therefore indistinguishable from a
 * converged result by a range check alone.
 *
 * Concretely, before this guard existed: cal_solve_tx_delay(12000, 2000, ...)
 * is a 10 m measurement error. Unclamped it produced ~20659, correctly refused
 * with -ERANGE. With the clamp it produces 18385 -- in range -- and returned 0,
 * i.e. a SUCCESS carrying a delay that is 2000 units (~4.7 m) wrong. The clamp
 * turned a loudly rejected calibration into a silently accepted broken one, and
 * this header's contract ("the caller must treat -ERANGE as a failed
 * calibration and NOT persist the value") had no way to fire.
 *
 * So the two guards are complementary, not redundant: the clamp bounds the
 * damage, and this reports that bounding happened. Removing either one
 * reintroduces a way to persist a wrong antenna delay with no error anywhere.
 */
#define CAL_MAX_STEP_MM \
	((int32_t)(((int32_t)CAL_MAX_STEP_UNITS * CAL_MM_PER_UNIT_X1000) / 1000))

/* Solve a new ant_delay_tx from a batch mean, holding cur_rx fixed.
 *
 *   measured_mm  mean of the accepted samples
 *   ref_mm       true antenna-to-antenna distance
 *   cur_tx/cur_rx  delays currently programmed into the radio
 *   out_tx       receives the new value -- written even when the result is
 *                out of range, so the caller can report what was rejected
 *
 * Returns 0 on success, or -ERANGE if the value must not be trusted. There are
 * now two independent reasons for -ERANGE, and the caller does not need to
 * distinguish them -- both mean "do not persist":
 *
 *   1. The solved delay falls outside CAL_TX_DLY_MIN..CAL_TX_DLY_MAX.
 *   2. The measurement error exceeds CAL_MAX_STEP_MM, so cal_solve_step()
 *      saturated and what it returned is not a solution. See CAL_MAX_STEP_MM.
 *
 * In both cases *out_tx is still written -- with the bound that was violated --
 * so the caller can report what was rejected. */
int cal_solve_tx_delay(int32_t measured_mm, int32_t ref_mm,
		       uint16_t cur_tx, uint16_t cur_rx, uint16_t *out_tx);


/* ---- Link statistics (the RANGE test, not the calibration) ---------------
 *
 * A range test needs NO reference distance: what it measures is whether the
 * exchange completes and how repeatable the distance is, not whether the
 * distance is right. Both of those are the caller's, not cal_math's -- these
 * live here for the same reason cal_solve_tx_delay() does, and deliberately
 * NOT in cal_math.c, which is a verbatim copy owned by the tag.
 */

struct cal_link_stats {
	int32_t  mean_mm;
	int32_t  sd_mm;
	int32_t  min_mm;
	int32_t  max_mm;
};

/* Population mean, standard deviation and extremes of `n` samples. Returns
 * false for n == 0. Integer-only: the sum of n <= 128 samples of a distance
 * in mm cannot overflow int64, and the variance is accumulated as int64
 * before the single sqrt, so no intermediate is ever squared in 32 bits.
 *
 * POPULATION sd (divide by n), not sample sd (n-1): these are all the
 * exchanges the batch got, not a sample drawn from a larger set, and at
 * n >= 30 the difference is under 2 % anyway. Stated so a later reader does
 * not "fix" it. */
bool cal_link_stats_compute(const int32_t *samples, size_t n,
			    struct cal_link_stats *out);

#endif /* CAL_SOLVE_H */
