/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See gw_selftau.h.
 */

#include "gw_selftau.h"

#include "ss_initiator.h"
#include "uwb_config.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gw_selftau, LOG_LEVEL_INF);

static struct {
	bool     pending;
	uint8_t  anchor_id;
	uint32_t target_n;
	uint32_t attempted;
	uint32_t valid;
	int64_t  sum_mm;
	int32_t  min_mm;
	int32_t  max_mm;
} st;

static struct gw_selftau_result last;

bool gw_selftau_request(uint8_t anchor_id, uint32_t n)
{
	if (st.pending) {
		return false;
	}
	if (anchor_id >= UWB_MAX_ANCHORS) {
		return false;
	}
	if (n < 1u || n > GW_SELFTAU_MAX_N) {
		return false;
	}

	st.pending    = true;
	st.anchor_id  = anchor_id;
	st.target_n   = n;
	st.attempted  = 0u;
	st.valid      = 0u;
	st.sum_mm     = 0;
	st.min_mm     = INT32_MAX;
	st.max_mm     = INT32_MIN;

	/* Mark the new run as in-progress immediately, not just when the first
	 * exchange completes: `sync tau result` read in the gap between
	 * queueing and the first gw_selftau_step() call must not show the
	 * PREVIOUS run's `done:1` as if it described this one. */
	last.anchor_id = anchor_id;
	last.attempted = 0u;
	last.valid     = 0u;
	last.mean_mm   = 0;
	last.min_mm    = 0;
	last.max_mm    = 0;
	last.tau_dtu   = 0;
	last.done      = false;

	return true;
}

bool gw_selftau_busy(void)
{
	return st.pending;
}

static void finalize(void)
{
	last.anchor_id = st.anchor_id;
	last.attempted = st.attempted;
	last.valid     = st.valid;

	if (st.valid > 0u) {
		last.mean_mm = (int32_t)(st.sum_mm / (int64_t)st.valid);
		last.min_mm  = st.min_mm;
		last.max_mm  = st.max_mm;
		last.tau_dtu = gw_selftau_mm_to_dtu(last.mean_mm);
	} else {
		last.mean_mm = 0;
		last.min_mm  = 0;
		last.max_mm  = 0;
		last.tau_dtu = 0;
	}
	last.done = true;

	LOG_INF("{\"gw_selftau\":{\"anchor_id\":%u,\"attempted\":%u,"
		"\"valid\":%u,\"mean_mm\":%d,\"min_mm\":%d,\"max_mm\":%d,"
		"\"tau_dtu\":%d}}",
		last.anchor_id, last.attempted, last.valid, last.mean_mm,
		last.min_mm, last.max_mm, last.tau_dtu);
}

void gw_selftau_step(void)
{
	if (!st.pending) {
		return;
	}

	int32_t mm;

	/* Same bracketing every other ss_initiator_range() call site in this
	 * tree uses. ss_initiator_leave() restores DWT_INT_RX enabled, which
	 * is the GATEWAY's own normal interrupt-driven RX state (set once in
	 * uwb_gateway_run(), same as uwb_slave.c) -- unlike a caller that runs
	 * polled RX, there is nothing to restore afterward here. */
	ss_initiator_enter();
	mm = ss_initiator_range((uint8_t)(UWB_ANCHOR_ADDR_BASE + st.anchor_id));
	ss_initiator_leave();

	st.attempted++;
	if (mm != INT32_MIN) {
		st.valid++;
		st.sum_mm += mm;
		if (mm < st.min_mm) st.min_mm = mm;
		if (mm > st.max_mm) st.max_mm = mm;
	}

	if (st.attempted >= st.target_n) {
		finalize();
		st.pending = false;
	}
}

void gw_selftau_get_result(struct gw_selftau_result *out)
{
	*out = last;
	/* While a run is still in progress, report its RUNNING tally rather
	 * than the previous run's stale finals -- `last.mean_mm`/`tau_dtu`
	 * only get written by finalize(), so mid-run they would otherwise
	 * still show the last COMPLETED run's numbers next to a `done:0`,
	 * which reads as a contradiction rather than "not there yet". */
	if (st.pending) {
		out->attempted = st.attempted;
		out->valid     = st.valid;
		out->mean_mm   = (st.valid > 0u) ?
			(int32_t)(st.sum_mm / (int64_t)st.valid) : 0;
		out->min_mm    = (st.valid > 0u) ? st.min_mm : 0;
		out->max_mm    = (st.valid > 0u) ? st.max_mm : 0;
		out->tau_dtu   = (st.valid > 0u) ?
			gw_selftau_mm_to_dtu(out->mean_mm) : 0;
	}
}
