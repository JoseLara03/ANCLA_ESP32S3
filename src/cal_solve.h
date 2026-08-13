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

#endif /* CAL_SOLVE_H */
