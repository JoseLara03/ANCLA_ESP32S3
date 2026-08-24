/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GATEWAY mode: the TDMA beacon and the CAP seat protocol.
 *
 * MAC-only -- this node does NOT answer ranging polls, unlike the nRF5
 * gateway's dispatch() fall-through. That costs a board rather than an anchor
 * (the deployment is one gateway plus four slaves) and buys a much smaller
 * beacon arm margin: with no anchor_respond in the loop the worst-case service
 * latency is one GRANT, not a 16.5 ms discovery stagger.
 *
 * Interrupt-driven with the same callback shape as uwb_slave.c. The DW3000
 * system clock is authoritative over the beacon cadence -- the beacon IS the
 * network's time base, so it is scheduled against the radio's own clock and
 * never against the kernel's.
 */

#include "uwb_modes.h"

#include "apos_frame.h"
#include "apos_gw.h"
#include "gw_core.h"
#include "pos_sink.h"
#include "tag_id.h"
#include "uwb_dwtime.h"
#include "uwb_frame_802_15_4z.h"
#include "uwb_mac.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

LOG_MODULE_REGISTER(uwb_gateway, LOG_LEVEL_INF);

/* T_SUPERFRAME_UUS comes from uwb_mac.h — the slaves predict the beacon from
 * the same definition, and a local copy here would drift against theirs. */

/* Delay from a received CAP frame to our GRANT TX. Same budget class as the
 * responders' turnaround; see disc_schedule.h for why this port needs more
 * than the nRF5 anchor's 2000. */
#define RX_TO_TX_DLY_UUS 2000u

/* Stop servicing and arm the beacon when this close to it. MAC-only keeps this
 * small: worst-case service latency is one GRANT (RX_TO_TX_DLY_UUS + ~1.3 ms
 * airtime + the bounded TXFRS wait), not the nRF5 gateway's 8000, which had to
 * cover a discovery stagger this node no longer performs. */
#define BEACON_ARM_MARGIN_UUS 5000u

/* Bound for the post-dwt_starttx() TXFRS wait, shared by both tx_beacon()'s
 * and send_grant()'s delayed TX. Covers the scheduled delay itself plus
 * airtime -- dwt_starttx() returns when the TX is armed, not when it fires.
 *
 * The two delayed-TX call sites have different worst-case scheduled delays,
 * and this bound must cover the LARGER one:
 *   - send_grant(): RX_TO_TX_DLY_UUS (~2.05 ms).
 *   - tx_beacon(true, ...): up to BEACON_ARM_MARGIN_UUS (~5.13 ms) -- the
 *     main loop only breaks out of RX-servicing and calls tx_beacon() once
 *     `to_beacon <= BEACON_ARM_MARGIN_UUS`, so the delayed beacon can be
 *     armed with nearly the full margin still to run before it fires. A
 *     bound sized only from RX_TO_TX_DLY_UUS (an earlier version of this
 *     comment did exactly that, and set this to 5 ms) times out on almost
 *     every delayed beacon -- confirmed on hardware: the first, immediate,
 *     beacon transmits fine, and every subsequent delayed one logs "beacon
 *     started but TXFRS never completed" and gets forced off.
 *
 * BEACON_ARM_MARGIN_UUS (~5.13 ms) + ~1.5 ms beacon airtime is the true
 * worst case, ~6.6 ms. Same derivation style as the slave's
 * TX_COMPLETE_TIMEOUT_MS in anchor_respond.c: ceil(worst_uus * 1.0256/1000)
 * + 5 ms margin = ceil(5.13) + 5 = 11. There is no upper ceiling this needs
 * to clear -- BEACON_ARM_MARGIN_UUS is the lower bound this must exceed, not
 * an upper one it must stay under; the two constants aren't otherwise
 * coupled. Re-derive if RX_TO_TX_DLY_UUS or BEACON_ARM_MARGIN_UUS changes
 * such that either's worst case grows past what 11 ms covers. */
#define TX_COMPLETE_TIMEOUT_MS 11

/* Longest frame the contract defines is a 39-byte beacon; +FCS, rounded up. */
#define RX_BUF_LEN 64

static K_SEM_DEFINE(rx_sem, 0, 1);

/* Same rx_pending guard as uwb_slave.c: br101's IRQ-drain loop can call
 * dwt_isr() again before the main thread has consumed the previous event, and
 * the limit-1 semaphore would silently swallow the second give while the
 * second callback corrupted the still-unread values. */
static volatile uint32_t rx_status;
static volatile uint16_t rx_len;
static volatile bool rx_pending;

static uint8_t rx_buf[RX_BUF_LEN];
static uint8_t beacon_buf[UWB_FRAME_MAX_LEN];
static uint8_t gw_seq;

static void cb_rx_ok(const dwt_cb_data_t *cb_data)
{
	if (rx_pending) {
		return;
	}
	rx_status = cb_data->status;
	rx_len = cb_data->datalength;
	rx_pending = true;
	k_sem_give(&rx_sem);
}

static void cb_rx_fail(const dwt_cb_data_t *cb_data)
{
	if (rx_pending) {
		return;
	}
	rx_status = cb_data->status;
	rx_len = 0;
	rx_pending = true;
	k_sem_give(&rx_sem);
}

/* Transmit the beacon. Returns its 40-bit TX timestamp, or 0 if it did not go
 * out. delayed=0 for the very first beacon, which has no predecessor to
 * schedule against. */
static uint64_t tx_beacon(struct gw_core_ctx *ctx, bool delayed, uint32_t tx_at)
{
	uint16_t slot_map[GW_N_CFP];

	gw_core_build_slotmap(ctx, slot_map);

	/* GW_N_CFP == UWB_FRAME_N_CFP == 11, so need = 15 + 22 = 37 =
	 * UWB_FRAME_MAX_LEN and beacon_buf is exactly large enough.
	 * uwb_frame_beacon_build() does not bound n_slots itself -- a known
	 * defect in the frame module, left unfixed because that file is kept
	 * byte-identical to the tag's. It cannot fire here: the argument is a
	 * compile-time constant equal to the maximum. */
	int n = uwb_frame_beacon_build(beacon_buf, sizeof(beacon_buf),
				       ctx->frame_counter, slot_map, GW_N_CFP);
	if (n < 0) {
		LOG_ERR("beacon build failed (%d)", n);
		return 0;
	}
	uwb_frame_set_seq_num(beacon_buf, gw_seq++);

	if (delayed) {
		dwt_setdelayedtrxtime(tx_at);
	}
	dwt_writetxdata((uint16_t)n, beacon_buf, 0);
	dwt_writetxfctrl((uint16_t)(n + FCS_LEN), 0, 0);

	if (dwt_starttx(delayed ? DWT_START_TX_DELAYED : DWT_START_TX_IMMEDIATE)
	    != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("beacon missed its slot — re-basing cadence");
		return 0;
	}
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       TX_COMPLETE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		LOG_WRN("beacon started but TXFRS never completed — forced off");
		return 0;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

	return uwb_get_tx_timestamp_u64();
}

static void send_grant(const uint8_t eui[UWB_FRAME_EUI_LEN],
		       const struct gw_grant *g, uint64_t rx_ts)
{
	uint8_t buf[UWB_FRAME_LEN_GRANT];

	/* eui is non-NULL by construction: it comes from a successfully parsed
	 * JOIN. uwb_frame_grant_build() writes its header before checking the
	 * pointer -- another known frame-module defect left unfixed for
	 * byte-identity with the tag -- which cannot fire on this path. */
	int n = uwb_frame_grant_build(buf, sizeof(buf), eui, g->short_addr,
				      g->slot_index, g->tier, g->lease);
	if (n < 0) {
		LOG_WRN("grant build failed (%d)", n);
		return;
	}
	uwb_frame_set_seq_num(buf, gw_seq++);

	uint32_t tx_at = (uint32_t)((rx_ts +
		((uint64_t)RX_TO_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);

	dwt_setdelayedtrxtime(tx_at);
	dwt_writetxdata((uint16_t)n, buf, 0);
	dwt_writetxfctrl((uint16_t)(n + FCS_LEN), 0, 0);

	if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("grant missed its slot — tag will retry via CAP");
		return;
	}
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       TX_COMPLETE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		LOG_WRN("grant started but TXFRS never completed — forced off");
		return;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
}

static void dispatch(struct gw_core_ctx *ctx, const uint8_t *buf, uint16_t len,
		     uint64_t rx_ts)
{
	if (apos_frame_is_apos(buf, len)) {
		apos_gw_on_rx(buf, len);
	} else if (uwb_frame_is_join(buf, len)) {
		uint8_t eui[UWB_FRAME_EUI_LEN];
		uint8_t req_tier = 0;
		struct gw_grant g;

		if (uwb_frame_parse_join(buf, len, eui, &req_tier) != 0) {
			return;
		}
		if (!gw_core_join(ctx, eui, req_tier, &g)) {
			LOG_WRN("JOIN refused — all %u seats occupied", GW_N_CFP);
			return;
		}
		LOG_INF("GRANT addr=0x%04X slot=%u tier=%u lease=%u",
			g.short_addr, g.slot_index, g.tier, g.lease);
		send_grant(eui, &g, rx_ts);
	} else if (uwb_frame_is_keepalive(buf, len)) {
		uint16_t sa = 0;
		uint8_t rt = 0, si = 0;

		if (uwb_frame_parse_keepalive(buf, len, &sa, &rt, &si) == 0) {
			gw_core_keepalive(ctx, sa, rt);
		}
	} else if (uwb_frame_is_release(buf, len)) {
		uint16_t sa = uwb_frame_get_src_addr(buf);

		LOG_INF("RELEASE addr=0x%04X", sa);
		gw_core_release(ctx, sa);
	} else if (uwb_frame_is_pos(buf, len)) {
		struct pos_fix fix;
		uint8_t eui[UWB_FRAME_EUI_LEN];

		/* Deliberately not gated on gw_core seat state: a fix from a tag
		 * whose lease just expired is still a real measurement, and
		 * silently dropping it would be close to undebuggable from the
		 * broker's side. */
		uwb_frame_parse_pos(buf, len, &fix.src_addr, &fix.x, &fix.y,
				    &fix.residual_m, &fix.n_anchors,
				    &fix.batt_soc);

		/* Tid must be the tag's stable EUI-derived id, not its
		 * reallocatable short address (see pos_json.h). The seat table
		 * is the only place that EUI lives -- look it up by the
		 * address this frame just arrived from. A miss here means the
		 * sender's lease expired between its last KEEPALIVE and this
		 * POS frame (the "not gated on seat state" comment above): the
		 * fix is still real and must still be published, just without
		 * the stability guarantee for this one straggler. Falling back
		 * to fix.src_addr reproduces this path's old (pre-tag_id)
		 * per-frame VALUE exactly, but note the straggler's Tid
		 * (src_addr) will differ from every other fix this same tag
		 * has ever sent (hash(EUI)), so the platform sees a one-record
		 * phantom device for that single frame -- a narrow, accepted
		 * cost, not a dropped fix. */
		if (gw_core_find_eui(ctx, fix.src_addr, eui)) {
			fix.tag_id = tag_id_from_eui(eui, UWB_FRAME_EUI_LEN);
		} else {
			LOG_WRN("POS from 0x%04X: no live seat, Tid falls back to short address",
				fix.src_addr);
			fix.tag_id = fix.src_addr;
		}
		pos_sink_publish(&fix);
	}
	/* Anything else is tag<->anchor ranging traffic. MAC-only: not ours,
	 * and logging every frame on a busy network would flood the console. */
}

void uwb_gateway_run(const uwb_config_t *cfg)
{
	/* Snapshot at entry — see uwb_slave.c for why. It matters more here:
	 * this loop runs indefinitely and drives every other node's timing. */
	uwb_config_t cfg_snapshot = *cfg;

	cfg = &cfg_snapshot;

	if (!cfg->position_valid) {
		LOG_ERR("{\"error\":\"gateway not positioned\"} — "
			"set `anchor pos <x> <y> <z>` and reboot");
		return;
	}

	static dwt_callbacks_s cbs;

	cbs.cbRxOk = cb_rx_ok;
	cbs.cbRxTo = cb_rx_fail;
	cbs.cbRxErr = cb_rx_fail;
	dwt_setcallbacks(&cbs);

	/* RX events only. TXFRS must stay masked: tx_beacon() and send_grant()
	 * poll for it, and an ISR that cleared it first would make both wait
	 * out their full timeout on every single transmission. */
	dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT);

	struct gw_core_ctx ctx;

	gw_core_init(&ctx);
	apos_gw_init();

	LOG_INF("{\"status\":\"gateway\",\"x\":%.2f,\"y\":%.2f,"
		"\"superframe_ms\":200,\"slots\":%u}",
		(double)cfg->x, (double)cfg->y, GW_N_CFP);

	uint64_t beacon_tx_ts = tx_beacon(&ctx, false, 0);

	if (beacon_tx_ts == 0) {
		LOG_ERR("first beacon failed to transmit — cannot start");
		return;
	}

	while (1) {
		uint32_t next_beacon = (uint32_t)((beacon_tx_ts +
			((uint64_t)T_SUPERFRAME_UUS * UUS_TO_DWT_TIME)) >> 8);

		for (;;) {
			uint32_t now = dwt_readsystimestamphi32();
			int32_t to_beacon = (int32_t)(next_beacon - now);

			if (to_beacon <= (int32_t)UUS_TO_HI32(BEACON_ARM_MARGIN_UUS)) {
				break;
			}

			/* Advance any running survey by at most one frame.
			 *
			 * Deliberately at the TOP of the loop body, not after
			 * dispatch(): the two `continue`s below (no RX event
			 * within the window, and a runt frame) would otherwise
			 * skip the step entirely, and an RX timeout is the
			 * NORMAL outcome on a quiet network. Since the first
			 * SURVEY_BEGIN must go out before any anchor has
			 * anything to reply to, a step reachable only after a
			 * successful RX would never start an enumeration at
			 * all.
			 *
			 * to_beacon was just read, so no recomputation is
			 * needed before the step; it is re-read afterwards
			 * because the step may have transmitted, and arming
			 * the RX window from a stale figure would push its
			 * expiry past the beacon. */
			if (apos_gw_busy()) {
				int32_t reserve = (int32_t)UUS_TO_HI32(
					BEACON_ARM_MARGIN_UUS +
					APOS_GW_STEP_BUDGET_UUS);

				if (to_beacon > reserve) {
					uint32_t span = (uint32_t)to_beacon -
						UUS_TO_HI32(BEACON_ARM_MARGIN_UUS);
					uint32_t avail_uus = (uint32_t)(
						((uint64_t)span << 8) / UUS_TO_DWT_TIME);

					apos_gw_step(avail_uus, &gw_seq);

					now = dwt_readsystimestamphi32();
					to_beacon = (int32_t)(next_beacon - now);
					if (to_beacon <=
					    (int32_t)UUS_TO_HI32(BEACON_ARM_MARGIN_UUS)) {
						break;
					}
				}
			}

			/* Expire the RX window BEACON_ARM_MARGIN_UUS before the
			 * beacon, so the delayed-TX setup has a guaranteed
			 * window. Without the subtraction the timeout fires at
			 * exactly the beacon instant and the delayed TX fails
			 * every time. */
			uint32_t span_hi32 =
				(uint32_t)to_beacon - UUS_TO_HI32(BEACON_ARM_MARGIN_UUS);
			uint32_t rx_to_uus =
				(uint32_t)(((uint64_t)span_hi32 << 8) / UUS_TO_DWT_TIME);

			dwt_setpreambledetecttimeout(0);
			dwt_setrxtimeout(rx_to_uus);
			dwt_setrxaftertxdelay(0);
			dwt_rxenable(DWT_START_RX_IMMEDIATE);

			/* Bounded, not K_FOREVER. DWT_INT_RX includes RXFTO so a
			 * timeout normally arrives, but a MAC loop that can wedge
			 * takes the whole network down, not one range. One
			 * superframe of slack past the window is ample. */
			if (k_sem_take(&rx_sem, K_MSEC(400)) != 0) {
				LOG_WRN("no RX event within the window — re-arming");
				dwt_forcetrxoff();
				continue;
			}

			/* rx_status is captured by the callbacks for symmetry with
			 * uwb_slave.c but is not read here: the gateway needs no
			 * CIR, since it does not answer ranging polls. */
			uint16_t flen = rx_len;

			rx_pending = false;

			if (flen <= FCS_LEN || flen > RX_BUF_LEN) {
				continue;
			}

			dwt_readrxdata(rx_buf, flen, 0);

			uint64_t rx_ts = uwb_get_rx_timestamp_u64();

			dispatch(&ctx, rx_buf, (uint16_t)(flen - FCS_LEN), rx_ts);
		}

		gw_core_superframe_tick(&ctx);

		uint64_t ts = tx_beacon(&ctx, true, next_beacon);

		/* On a miss, re-base on the current time rather than compounding
		 * the error into every following superframe. */
		beacon_tx_ts = ts ? ts
				  : (((uint64_t)dwt_readsystimestamphi32()) << 8);
	}
}
