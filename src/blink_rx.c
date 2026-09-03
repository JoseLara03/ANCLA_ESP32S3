/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "blink_rx.h"

#include "blink_frame.h"
#include "ccp_slave.h"
#include "net_uplink.h"
#include "pos_json.h"
#include "sync_model.h"
#include "uwb_debug.h"

#include <zephyr/logging/log.h>

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

void blink_rx_init(const uwb_config_t *cfg)
{
	my_anchor_id     = (cfg != NULL) ? cfg->anchor_id : 0u;
	n_blink_rx       = 0u;
	n_blink_no_sync  = 0u;
	n_blink_bad      = 0u;
	n_blink_sent     = 0u;
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

	/*
	 * This anchor's own MEASURED timestamp jitter, so the gateway can
	 * weight its equations against the other anchors' instead of assuming
	 * every link is equally good.
	 *
	 * sync_model_jitter_est_dtu(), NOT sync_model_error_dtu(), and the
	 * difference is not cosmetic: error_dtu() is built from
	 * SYNC_JITTER_DTU, a hardcoded ASSUMPTION of ~100 ps, while the Fase 2
	 * hardware measurement came in at 782 ps to 1.53 ns. Publishing
	 * error_dtu() would hand the gateway a sigma roughly 15x too
	 * optimistic and make its filter overconfident in every observation --
	 * worse than the flat r_tdoa fallback it would be replacing.
	 * jitter_est_dtu() is derived from this anchor's own residual
	 * statistics, i.e. from what its link actually did.
	 *
	 * WHAT IT IS NOT: a per-BLINK figure. It is a long-run statistic for
	 * this anchor's CCP link, so it discriminates between ANCHORS (a weak
	 * or distant one counts less) and not between one blink and the next.
	 * Per-blink SNR is what `quality` would be for, and whether that
	 * carries any information is unmeasured -- see pos_json.h.
	 *
	 * 0 means "not enough observations yet" (the RMS is 0 before any
	 * residual has been folded in), and it must NOT reach the gateway as a
	 * sigma: zero sigma reads as infinite confidence. It is forwarded as 0
	 * on purpose and the gateway treats 0 as "unknown, use the fallback".
	 * Saturated rather than wrapped at 16 bits: 65535 DTU is 307 m of path
	 * error, so anything near it is already meaningless and the exact
	 * value does not matter, but a wrap would turn a terrible link into an
	 * excellent-looking one. */
	{
		uint32_t j = sync_model_jitter_est_dtu(ccp_slave_model());

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
