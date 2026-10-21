/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "blink_rx.h"

#include "blink_frame.h"
#include "ccp_slave.h"
#include "net_uplink.h"
#include "node_eui.h"
#include "pos_json.h"
#include "sync_model.h"
#include "uwb_debug.h"

#include <zephyr/logging/log.h>

#include <string.h>

/* ANCLA_LOG_LEVEL, not LOG_LEVEL_INF: the per-BLINK verdict line below is the
 * only way to see WHY observations are not reaching the gateway, and an INF cap
 * would compile it out of reach of debug.conf. Still INF in production. */
LOG_MODULE_REGISTER(blink_rx, ANCLA_LOG_LEVEL);

/* Module state, static -- nothing on the stack. The SLAVE loop's stack is the
 * main stack (4096 B) and a 2588-byte automatic already overflowed it once. */
static uint8_t  my_anchor_id;
static uint32_t n_blink_rx;
static uint32_t n_blink_no_sync;
static uint32_t n_blink_bad;
static uint32_t n_blink_sent;
static uint32_t n_blink_stale;

void blink_rx_init(const uwb_config_t *cfg)
{
	my_anchor_id     = (cfg != NULL) ? cfg->anchor_id : 0u;
	n_blink_rx       = 0u;
	n_blink_no_sync  = 0u;
	n_blink_bad      = 0u;
	n_blink_sent     = 0u;
	n_blink_stale    = 0u;
}

uint32_t blink_rx_stale_count(void)
{
	return n_blink_stale;
}

bool blink_rx_on_rx(const uint8_t *buf, uint16_t plen, uint64_t rx_ts,
		    uint16_t quality)
{
	struct blink_frame bf;
	struct pos_blink_obs obs;
	uint64_t t_master;

	if (!blink_frame_is_blink(buf, plen)) {
		return false;
	}

	n_blink_rx++;

	if (blink_frame_parse(buf, plen, &bf) != 0) {
		n_blink_bad++;
		LOG_DBG("{\"blink\":{\"verdict\":\"malformed\"}}");
		return true;
	}

	/* The clock must be FRESH, not just present. sync_model_to_master() keeps
	 * extrapolating a converged model for as long as no CCP arrives to count a
	 * gap, so a link that has been silent for minutes still converts here
	 * "successfully" into a time that has drifted by however far the two
	 * crystals walked apart. The age bound is the existing holdover policy
	 * (SYNC_MISS_MAX superframes) expressed in time. A stale refusal also
	 * counts as no_sync, so `no_sync == rx` still reads as "CCP link down";
	 * `stale` says it was silence rather than a model that never converged. */
	if (!ccp_slave_clock_fresh()) {
		n_blink_no_sync++;
		n_blink_stale++;
		LOG_DBG("{\"blink\":{\"tag\":\"0x%04X\",\"seq\":%u,"
			"\"verdict\":\"stale clock (%u ms since the last CCP) -- "
			"dropped\"}}", bf.src_addr, bf.seq, ccp_slave_clock_age_ms());
		return true;
	}

	/* The one place this module can refuse. ccp_slave_model() is const by
	 * contract -- this is a pure read of the live model, which is exactly
	 * what that constness is for. */
	if (!sync_model_to_master(ccp_slave_model(), rx_ts, &t_master)) {
		n_blink_no_sync++;
		LOG_DBG("{\"blink\":{\"tag\":\"0x%04X\",\"seq\":%u,"
			"\"verdict\":\"no sync -- dropped\"}}",
			bf.src_addr, bf.seq);
		return true;
	}

	/* t_master is a 40-bit DTU value in the master's base, so the cast to
	 * int64_t is lossless and cannot go negative. The signed type exists
	 * because tdoa_dtu_rebase() (Task 6) turns these into SIGNED
	 * differences on the gateway. */
	obs.anchor_id = my_anchor_id;
	obs.blink_seq = bf.seq;
	obs.batt_soc  = bf.batt_soc;
	obs.tag_addr  = bf.src_addr;
	obs.quality   = quality;
	/* Forwarded WHOLE, not decoded: this module stamps and relays, it does
	 * not interpret. BLINK_FLAG_MOVING is what the gateway's filter wants
	 * out of it; blink_frame_parse() has already rejected any reserved
	 * bit, so whatever survives to here is a flag this build knows. */
	obs.flags     = bf.flags;
	/* The identity the gateway and the edge resolve this anchor by. */
	obs.has_eui   = node_eui_valid();
	memcpy(obs.eui, node_eui(), sizeof(obs.eui));

	/*
	 * This anchor's own MEASURED timestamp jitter, so the gateway can
	 * weight its equations against the other anchors' instead of assuming
	 * every link is equally good.
	 *
	 * sync_model_sigma_dtu(), since 2026-09-18. It used to be
	 * sync_model_jitter_est_dtu(), chosen over sync_model_error_dtu()
	 * because error_dtu() was built from SYNC_JITTER_DTU -- a hardcoded
	 * ASSUMPTION of ~100 ps against a Fase 2 hardware measurement of
	 * 782 ps to 1.53 ns -- and would have been roughly 15x too optimistic.
	 * error_dtu() no longer works that way: it is built from this anchor's
	 * OWN measured residuals with a pessimistic fallback before there are
	 * enough of them, so reaching past it is no longer necessary. See
	 * sync_model.h.
	 *
	 * The reason to move BACK to it is B13: jitter_est_dtu() is a long-run
	 * statistic that does not move at all during a re-baseline, so a
	 * freshly-discontinuous anchor published its usual confidence while
	 * converting on a carried or two-point rate. sync_model_sigma_dtu()
	 * is the conversion error over a full CCP interval and rises during
	 * that window, which is exactly what a consumer needs in order to
	 * down-weight it.
	 *
	 * WHAT IT IS NOT: a per-BLINK figure. Its jitter input is a long-run
	 * statistic for this anchor's CCP link, so it discriminates between
	 * ANCHORS (a weak or distant one counts less) and, apart from the
	 * baseline-state term, not between one blink and the next. Per-blink
	 * SNR is what `quality` would be for, and whether that carries any
	 * information is unmeasured -- see pos_json.h.
	 *
	 * 0 means "unknown" on the wire and must NOT reach the gateway as a
	 * sigma, since zero sigma reads as infinite confidence.
	 * sync_model_sigma_dtu() returns 0 only when the model cannot convert
	 * -- and in that case sync_model_to_master() above already refused and
	 * this line was never reached -- and clamps every other result to at
	 * least 1. Saturated rather than wrapped at 16 bits: 65535 DTU is
	 * 307 m of path error, so anything near it is already meaningless and
	 * the exact value does not matter, but a wrap would turn a terrible
	 * link into an excellent-looking one. */
	{
		uint32_t j = sync_model_sigma_dtu(ccp_slave_model());

		obs.sigma_dtu = (j > 0xFFFFu) ? 0xFFFFu : (uint16_t)j;
	}

	obs.t_dtu     = (int64_t)t_master;

	net_uplink_submit_blink(&obs);
	n_blink_sent++;

	LOG_DBG("{\"blink\":{\"tag\":\"0x%04X\",\"seq\":%u,\"anchor\":%u,"
		"\"verdict\":\"stamped\"}}",
		bf.src_addr, bf.seq, my_anchor_id);
	return true;
}

void blink_rx_stats(uint32_t *n_rx, uint32_t *n_no_sync, uint32_t *n_bad,
		    uint32_t *n_sent)
{
	if (n_rx != NULL) {
		*n_rx = n_blink_rx;
	}
	if (n_no_sync != NULL) {
		*n_no_sync = n_blink_no_sync;
	}
	if (n_bad != NULL) {
		*n_bad = n_blink_bad;
	}
	if (n_sent != NULL) {
		*n_sent = n_blink_sent;
	}
}
