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

/* ---- Fase B: automatic round-robin ---------------------------------- */

/* volatile: written from the shell thread (gw_selftau_auto_set_enabled()),
 * read from the gateway loop (gw_selftau_step()/gw_selftau_has_work()) --
 * same single-flag-signal precedent as ccp_master.c's txtest_pending. */
static volatile bool auto_enabled = true;

/* Both touched ONLY from the gateway's K_PRIO_COOP(0) loop -- no fence
 * needed, same reasoning gw_selftau_get_result()'s own doc comment gives
 * for `last`. */
static uint8_t auto_next_anchor;
static bool    auto_armed_this_sf;

struct auto_entry {
	bool     valid;   /* an auto exchange has ever succeeded for this anchor */
	bool     fresh;   /* completed since the last gw_selftau_auto_take() */
	int32_t  tau_dtu;
	uint8_t  quality;
};
static struct auto_entry auto_latest[UWB_MAX_ANCHORS];

bool gw_selftau_auto_get_enabled(void)
{
	return auto_enabled;
}

void gw_selftau_auto_set_enabled(bool on)
{
	auto_enabled = on;
}

void gw_selftau_auto_new_superframe(void)
{
	auto_armed_this_sf = false;
}

bool gw_selftau_has_work(void)
{
	return st.pending || (auto_enabled && !auto_armed_this_sf);
}

bool gw_selftau_auto_take(uint16_t *anchor_addr, int32_t *tau_dtu,
			  uint8_t *quality)
{
	for (uint8_t i = 0; i < UWB_MAX_ANCHORS; i++) {
		if (auto_latest[i].fresh) {
			auto_latest[i].fresh = false;
			if (anchor_addr) {
				*anchor_addr = (uint16_t)(UWB_ANCHOR_ADDR_BASE + i);
			}
			if (tau_dtu) {
				*tau_dtu = auto_latest[i].tau_dtu;
			}
			if (quality) {
				*quality = auto_latest[i].quality;
			}
			return true;
		}
	}
	return false;
}

void gw_selftau_auto_get_anchor(uint8_t anchor_id, bool *valid,
				int32_t *tau_dtu, uint8_t *quality)
{
	if (anchor_id >= UWB_MAX_ANCHORS) {
		if (valid) { *valid = false; }
		if (tau_dtu) { *tau_dtu = 0; }
		if (quality) { *quality = 0; }
		return;
	}

	const struct auto_entry *e = &auto_latest[anchor_id];

	if (valid) { *valid = e->valid; }
	if (tau_dtu) { *tau_dtu = e->tau_dtu; }
	if (quality) { *quality = e->quality; }
}

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

/* One auto turn: exactly one exchange against the current round-robin
 * anchor, unaveraged (see gw_selftau.h's Fase B header comment for why).
 * Broken out of gw_selftau_step() only for readability -- it has exactly
 * one call site. */
static void auto_step(void)
{
	auto_armed_this_sf = true;

	uint8_t anchor_id = auto_next_anchor;

	auto_next_anchor = (uint8_t)((auto_next_anchor + 1u) % UWB_MAX_ANCHORS);

	int16_t rssi_q8;
	int32_t mm;

	/* Same bracketing as the manual path below and every other
	 * ss_initiator_range() call site in this tree. */
	ss_initiator_enter();
	mm = ss_initiator_range_ex((uint8_t)(UWB_ANCHOR_ADDR_BASE + anchor_id),
				   &rssi_q8);
	ss_initiator_leave();

	if (mm == INT32_MIN) {
		/* No response this turn -- leave the cached entry (if any)
		 * exactly as it was. An unanswered exchange must not
		 * overwrite a good measurement, and must not mark `fresh`:
		 * there is nothing new for a CCP to announce. This anchor
		 * simply gets no fresher tau until its NEXT turn, N
		 * superframes from now. */
		return;
	}

	struct auto_entry *e = &auto_latest[anchor_id];
	/* dBm = rssi_q8/256. Shifted into a small positive byte (roughly
	 * "dBm + 100") rather than passed through raw: CCP v2's twr_quality
	 * is one unsigned byte. This is a PLACEHOLDER encoding, not a
	 * calibrated confidence score -- CLAUDE.md already records that this
	 * project does not yet know whether a DW3000 quality figure carries
	 * real information at fixed PLEN (the ipatovAccumCount entry). It
	 * exists so Fase B's next task (the anchor-side EMA/jump-detect
	 * filter, FR-8) has a real, monotonic, already-hardware-validated
	 * signal to start from instead of a constant nobody could tune. */
	int32_t q = (int32_t)(rssi_q8 >> 8) + 100;

	if (rssi_q8 == SS_INITIATOR_RSSI_INVALID) {
		e->quality = GW_SELFTAU_AUTO_QUALITY_NONE;
	} else {
		if (q < 1) {
			q = 1;
		} else if (q > 255) {
			q = 255;
		}
		e->quality = (uint8_t)q;
	}

	e->valid   = true;
	e->fresh   = true;
	e->tau_dtu = gw_selftau_mm_to_dtu(mm);
}

void gw_selftau_step(void)
{
	if (st.pending) {
		int32_t mm;

		/* Same bracketing every other ss_initiator_range() call site
		 * in this tree uses. ss_initiator_leave() restores
		 * DWT_INT_RX enabled, which is the GATEWAY's own normal
		 * interrupt-driven RX state (set once in
		 * uwb_gateway_run(), same as uwb_slave.c) -- unlike a caller
		 * that runs polled RX, there is nothing to restore afterward
		 * here. */
		ss_initiator_enter();
		mm = ss_initiator_range((uint8_t)(UWB_ANCHOR_ADDR_BASE +
						   st.anchor_id));
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
		return;
	}

	if (!auto_enabled || auto_armed_this_sf) {
		return;
	}
	auto_step();
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
