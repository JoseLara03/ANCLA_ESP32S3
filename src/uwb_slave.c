/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SLAVE mode: an SS-TWR responder that also observes the gateway beacon.
 *
 * Interrupt-driven rather than polled. br101 runs dwt_isr() from the system
 * workqueue (platform/dw3000_hw.c), so the callbacks below are in thread
 * context -- but they still do the minimum and hand off, because the main
 * thread is ours to block and the workqueue is shared. A polling loop was
 * rejected outright: Zephyr's main thread is priority 0 and the shell thread
 * is 14, so a loop that never yields would starve the console.
 */

#include "uwb_modes.h"

#include "anchor_respond.h"
#include "apos_node.h"
#include "beacon_guard.h"
#include "ccp_slave.h"
#include "uwb_debug.h"
#include "uwb_dwtime.h"
#include "uwb_frame_802_15_4z.h"
#include "uwb_mac.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <string.h>

/* ANCLA_LOG_LEVEL, not LOG_LEVEL_INF: observe_beacon()'s per-beacon line is a
 * LOG_DBG that an INF cap makes unreachable from a .conf overlay, and the
 * beacon-guard lock state it prints is an input to every suppression decision
 * in anchor_respond.c. Still LOG_LEVEL_INF in production -- see uwb_debug.h. */
LOG_MODULE_REGISTER(uwb_slave, ANCLA_LOG_LEVEL);

/* Longest frame the contract defines is a 37-byte beacon; +FCS, rounded up. */
#define RX_BUF_LEN 64

static K_SEM_DEFINE(rx_sem, 0, 1);

/* Written by the RX callbacks, read by the main thread. rx_pending is the
 * guard that makes this safe: br101's IRQ-drain loop (platform/dw3000_hw.c)
 * calls dwt_isr() repeatedly while the IRQ line is held, and DWT_INT_RX
 * includes DWT_INT_CIADONE_BIT_MASK, whose completion can trail the RXFCG
 * pass -- so a second cb_rx_ok/cb_rx_fail can fire (or a datalength==0 case
 * can dispatch cb_rx_fail) before the main thread has consumed the previous
 * event's rx_status/rx_len via k_sem_take. Without the guard that second
 * write corrupts the still-unread values, and the second k_sem_give is
 * silently swallowed by the limit-1 semaphore -- a real event lost or
 * corrupted. rx_pending makes each callback a no-op once an event is already
 * waiting for the main thread; the main thread clears it right after
 * k_sem_take, before touching rx_status/rx_len, so the next callback is free
 * to capture again. A plain flag suffices here: these callbacks run in
 * cooperative thread context on the system workqueue, not a preempting ISR. */
static volatile uint32_t rx_status;
static volatile uint16_t rx_len;
static volatile bool rx_pending;

static uint8_t rx_buf[RX_BUF_LEN];
static uint8_t frame_seq_nb;

static struct beacon_guard bguard;

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

static void rx_arm(void)
{
	dwt_setpreambledetecttimeout(0);
	dwt_setrxtimeout(0);
	dwt_setrxaftertxdelay(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/* Read the Ipatov CIR metrics, but only if CIA had finished for this frame.
 *
 * dwt_isr() clears SYS_STATUS_ALL_RX_GOOD -- which includes CIADONE -- before
 * invoking cb_rx_ok, so the register cannot be polled here; the pre-clear
 * snapshot in cb_data->status is the only evidence available. Reporting zero is
 * safe: the tag ranks anchors by these metrics, it does not accept or reject a
 * range on them. */
static void read_cir(uint32_t status, int32_t *cir_power, uint16_t *cir_quality)
{
	if (!(status & DWT_INT_CIADONE_BIT_MASK)) {
		*cir_power = 0;
		*cir_quality = 0;
		return;
	}

	dwt_rxdiag_t diag;

	memset(&diag, 0, sizeof(diag));
	dwt_readdiagnostics(&diag);
	/* Ipatov (preamble) metrics — STS is off in this PHY. */
	*cir_power = (int32_t)diag.ipatovPower;
	*cir_quality = diag.ipatovAccumCount;
}

static void observe_beacon(const uint8_t *buf, uint16_t len,
			   const uwb_config_t *cfg, uint64_t rx_ts)
{
	if (!uwb_frame_is_beacon(buf, len)) {
		return;
	}

	uint8_t proto_ver = 0;
	uint32_t frame_counter = 0;
	uint16_t slot_map[UWB_FRAME_N_CFP];
	uint8_t n_slots = 0;

	int rc = uwb_frame_parse_beacon(buf, len, &proto_ver, &frame_counter,
					slot_map, &n_slots);
	if (rc) {
		LOG_WRN("malformed beacon (%d)", rc);
		return;
	}
	/* slot_map cannot have overflowed above: parse_beacon derives n_slots as
	 * (len - 15) / 2, and uwb_frame_is_valid() already capped len at
	 * UWB_FRAME_MAX_LEN (37) == 15 + 2 * UWB_FRAME_N_CFP, so n_slots <= 11.
	 * This check exists to catch that identity being broken by a future
	 * change to either constant, not to protect the write. */
	if (n_slots > UWB_FRAME_N_CFP) {
		LOG_ERR("beacon parsed %u slots, buffer holds %u — "
			"UWB_FRAME_MAX_LEN and UWB_FRAME_N_CFP disagree",
			n_slots, UWB_FRAME_N_CFP);
		return;
	}

	int slot = uwb_frame_beacon_find_addr(slot_map, n_slots,
					      uwb_config_short_addr(cfg));

	/* Re-anchor on the beacon's own RX timestamp, in the same hi32 units
	 * every scheduled TX is expressed in -- no clock conversion, so no
	 * second clock to drift against. */
	beacon_guard_beacon(&bguard, (uint32_t)(rx_ts >> 8));

	/* Recorded and logged only. Gating RX windows on beacon timing needs a
	 * real gateway to develop against — that is spec D. LOG_DBG, not
	 * LOG_INF: this fires on every beacon (roughly every 195 ms) regardless
	 * of ranging activity, and at LOG_LEVEL_INF the call compiles out
	 * entirely — link-budget timing on the DISCOVERY/WAVE responders (see
	 * disc_schedule.h, anchor_respond.c) matters more than always-on beacon
	 * visibility. Bump this module's LOG_MODULE_REGISTER level to
	 * LOG_LEVEL_DBG to re-enable for debugging. */
	LOG_DBG("BEACON ver=%u counter=%u slots=%u our_slot=%d locked=%d",
		proto_ver, frame_counter, n_slots, slot,
		beacon_guard_locked(&bguard));
}

#ifdef CONFIG_ANCLA_RANGING_DEBUG

/* Type byte offset in every frame this network uses -- the module frames and
 * the legacy WAVE/VEWA pair alike (OFF_TYPE in uwb_frame_802_15_4z.c). Counted
 * here rather than inside each responder so one place sees every frame that
 * reached the loop, including the ones every responder declines. */
#define DBG_OFF_TYPE 9
#define DBG_HEARTBEAT_MS 1000

static struct {
	uint32_t rx_ok;
	uint32_t rx_err;
	uint32_t wave;   /* 0xE0 legacy poll, addressed to any anchor */
	uint32_t disc;   /* 0xE2 DISCOVERY broadcast */
	uint32_t beacon; /* 0xE5 */
	uint32_t apos;   /* 0xEB survey traffic */
	uint32_t other;
} dbg;

static void dbg_count(const uint8_t *buf, uint16_t plen)
{
	dbg.rx_ok++;
	if (plen <= DBG_OFF_TYPE) {
		dbg.other++;
		return;
	}
	switch (buf[DBG_OFF_TYPE]) {
	case 0xE0: dbg.wave++;   break;
	case 0xE2: dbg.disc++;   break;
	case 0xE5: dbg.beacon++; break;
	case 0xEB: dbg.apos++;   break;
	default:
		dbg.other++;
		/* Type and length of anything the responders will not claim. A
		 * DISCOVERY that is well-formed enough to carry 0xE2 but too
		 * short for UWB_FRAME_LEN_DISC counts as disc above and yet
		 * produces no {"disc":...} verdict line, so comparing the two is
		 * what separates a malformed frame from a refused one. */
		LOG_DBG("{\"rx_other\":{\"type\":\"0x%02X\",\"plen\":%u,"
			"\"src\":\"0x%04X\"}}",
			buf[DBG_OFF_TYPE], plen,
			uwb_frame_get_src_addr(buf));
		break;
	}
}

/* Emitted every second whether or not a single frame arrived.
 *
 * An all-zero line is the most decisive observation this board can make on its
 * own: it separates "deaf" from "hears the tag and declines to answer" with no
 * sniffer and no tag-side visibility. It also has to keep printing while the
 * board is deaf -- per-frame logging alone goes silent in exactly the failure it
 * was added to catch, and that silence is indistinguishable from a wedged
 * thread, a crashed board or a disconnected console.
 *
 * sys is SYS_STATUS_LO, the receiver's own account of itself. "A board that
 * transmits once per boot and then goes silent is a wedged DW3220" is already a
 * known failure on this hardware; a heartbeat that keeps ticking with rx_ok at
 * zero and an unchanging sys word says so directly instead of by elimination.
 */
static void dbg_heartbeat(const uwb_config_t *cfg, struct beacon_guard *bg)
{
	LOG_INF("{\"slave_hb\":{\"id\":%u,\"addr\":\"0x%04X\","
		"\"pos_valid\":%u,\"rx_ok\":%u,\"rx_err\":%u,\"wave\":%u,"
		"\"disc\":%u,\"beacon\":%u,\"apos\":%u,\"other\":%u,"
		"\"sys\":\"0x%08X\",\"locked\":%u,\"misses\":%u}}",
		cfg->anchor_id, uwb_config_short_addr(cfg),
		cfg->position_valid ? 1u : 0u,
		dbg.rx_ok, dbg.rx_err, dbg.wave, dbg.disc, dbg.beacon,
		dbg.apos, dbg.other, dwt_readsysstatuslo(),
		beacon_guard_locked(bg) ? 1u : 0u, beacon_guard_misses(bg));
	memset(&dbg, 0, sizeof(dbg));
}
#endif /* CONFIG_ANCLA_RANGING_DEBUG */

void uwb_slave_run(const uwb_config_t *cfg)
{
	/* Snapshot at mode-entry: anchor_shell.c's "anchor id/ant/pos" commands
	 * mutate the live uwb_config_get() singleton that cfg points at, and
	 * this loop runs indefinitely on the main thread with no synchronisation
	 * against the shell thread. Operating on a stable copy restores the
	 * documented "changes take effect only on reboot" contract -- notably
	 * for ant_delay_tx, which resp_tx_ts arithmetic must keep matching the
	 * antenna delay actually programmed into the radio at boot.
	 *
	 * Kept as a named MUTABLE object as well as the const view:
	 * apos_node_on_rx() writes a surveyed position straight into it, so
	 * `apos apply` takes effect without a reboot, while every other consumer
	 * keeps the const view and the reboot contract above for id and antenna
	 * delay. */
	static uwb_config_t cfg_snapshot;

	cfg_snapshot = *cfg;
	cfg = &cfg_snapshot;

	static dwt_callbacks_s cbs;

	cbs.cbRxOk = cb_rx_ok;
	cbs.cbRxTo = cb_rx_fail;
	cbs.cbRxErr = cb_rx_fail;
	dwt_setcallbacks(&cbs);

	/* DWT_INT_RX is the RX-only mask. TXFRS must stay out of it: tx_delayed()
	 * polls for transmit completion, and an ISR that cleared the bit first
	 * would hang that poll. */
	dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT);

	LOG_INF("{\"status\":\"listening\",\"mode\":\"slave\",\"id\":%u,"
		"\"short_addr\":\"0x%04X\"}",
		cfg->anchor_id, uwb_config_short_addr(cfg));

	apos_node_init();
	ccp_slave_init();

	beacon_guard_init(&bguard, UUS_TO_HI32(T_SUPERFRAME_UUS),
			  UUS_TO_HI32(BEACON_GUARD_UUS),
			  UUS_TO_HI32(BEACON_OCCUPANCY_UUS));

	rx_arm();

#ifdef CONFIG_ANCLA_RANGING_DEBUG
	int64_t next_hb = k_uptime_get() + DBG_HEARTBEAT_MS;
#endif

	while (1) {
#ifdef CONFIG_ANCLA_RANGING_DEBUG
		/* Bounded instead of K_FOREVER so the heartbeat still fires on a
		 * board that is receiving nothing -- which is the case being
		 * chased, and the one a purely event-driven log cannot report.
		 *
		 * The timeout path deliberately does NOT call rx_arm(). Re-arming
		 * a receiver on a timer would change the very radio behaviour this
		 * image exists to observe, and if RX really has been left off,
		 * the sys word in the heartbeat is the evidence for it; quietly
		 * repairing the symptom here would destroy that evidence. */
		int64_t wait_ms = next_hb - k_uptime_get();

		if (wait_ms < 0) {
			wait_ms = 0;
		}
		if (k_sem_take(&rx_sem, K_MSEC(wait_ms)) != 0) {
			dbg_heartbeat(cfg, &bguard);
			next_hb = k_uptime_get() + DBG_HEARTBEAT_MS;
			continue;
		}
		if (k_uptime_get() >= next_hb) {
			dbg_heartbeat(cfg, &bguard);
			next_hb = k_uptime_get() + DBG_HEARTBEAT_MS;
		}
#else
		k_sem_take(&rx_sem, K_FOREVER);
#endif

		uint32_t status = rx_status;
		uint16_t flen = rx_len;
		rx_pending = false;

		/* flen counts the FCS. Anything that cannot hold one is not a
		 * frame we can reason about. cb_rx_fail() reports flen 0, so this
		 * is also where an RX error or timeout lands. */
		if (flen <= FCS_LEN || flen > RX_BUF_LEN) {
#ifdef CONFIG_ANCLA_RANGING_DEBUG
			dbg.rx_err++;
#endif
			rx_arm();
			continue;
		}

		int32_t cir_power = 0;
		uint16_t cir_quality = 0;

		read_cir(status, &cir_power, &cir_quality);
		dwt_readrxdata(rx_buf, flen, 0);
		uint64_t rx_ts = uwb_get_rx_timestamp_u64();

		/* Payload length: every consumer below derives field counts from
		 * it, and two extra bytes corrupt them silently. */
		uint16_t plen = (uint16_t)(flen - FCS_LEN);

#ifdef CONFIG_ANCLA_RANGING_DEBUG
		dbg_count(rx_buf, plen);
#endif

		/* First in the chain, and cheap to reject: ccp_frame_is_ccp()
		 * is a type-and-length test. A CCP arrives once per superframe
		 * and is for nobody else, so there is no reason to walk it past
		 * the survey and ranging responders. */
		if (ccp_slave_on_rx(rx_buf, plen, rx_ts)) {
			rx_arm();
			continue;
		}

		/* Offered to each in turn; each ignores what is not its
		 * own. APOS goes first and short-circuits: a survey
		 * frame is never also a ranging poll or a beacon, and a
		 * RANGE_CMD blocks for a whole batch, so there is no
		 * point offering it to anyone else afterwards. */
		if (!apos_node_on_rx(rx_buf, plen, &cfg_snapshot,
				     &frame_seq_nb, &bguard)) {
			anchor_respond_wave_poll(rx_buf, plen, rx_ts, cfg,
						 &frame_seq_nb, &bguard,
						 apos_node_window_open());
			anchor_respond_discovery(rx_buf, plen, rx_ts, cfg,
						 &frame_seq_nb, cir_power,
						 cir_quality, &bguard);
			observe_beacon(rx_buf, plen, cfg, rx_ts);
		}

		rx_arm();
	}
}
