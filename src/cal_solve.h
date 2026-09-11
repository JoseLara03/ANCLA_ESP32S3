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

/* Solve a new ant_delay_tx from a batch mean, holding cur_rx fixed.
 *
 *   measured_mm  mean of the accepted samples
 *   ref_mm       true antenna-to-antenna distance
 *   cur_tx/cur_rx  delays currently programmed into the radio
 *   out_tx       receives the new value -- written even when the result is
 *                out of range, so the caller can report what was rejected
 *
 * Returns 0 on success, or -ERANGE if the result had to be clamped. The caller
 * must treat -ERANGE as a failed calibration and NOT persist the value. */
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

/* Received signal level in dBm x10 (so -81.2 dBm reads -812), from the
 * DW3000's Ipatov CIR power and preamble accumulation count:
 *
 *     level = 10 * log10(C * 2^21 / N^2) - A
 *
 * with A = 121.7 dB at the 64 MHz PRF this PHY runs (channel 5, preamble
 * code 9). This is the DW3000 user manual's own formula.
 *
 * dBm x10 rather than a float because the value is a report, and an integer
 * cannot pick up a different last digit between two builds. Returns
 * INT32_MIN when `accum_count` is 0 or `cir_power` is 0 -- i.e. when the CIA
 * had not finished when the diagnostics were read, which is a real outcome on
 * a polled (interrupt-free) receive path and must not be reported as a very
 * low signal level. */
#define CAL_RX_LEVEL_INVALID  INT32_MIN
int32_t cal_rx_level_dbm_x10(uint32_t cir_power, uint16_t accum_count);

#endif /* CAL_SOLVE_H */
