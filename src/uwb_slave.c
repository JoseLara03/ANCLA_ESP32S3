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
#include "uwb_dwtime.h"
#include "uwb_frame_802_15_4z.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <string.h>

LOG_MODULE_REGISTER(uwb_slave, LOG_LEVEL_INF);

/* Longest frame the contract defines is a 39-byte beacon; +FCS, rounded up. */
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

static void observe_beacon(const uint8_t *buf, uint16_t len, const uwb_config_t *cfg)
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
	 * UWB_FRAME_MAX_LEN (39) == 15 + 2 * UWB_FRAME_N_CFP, so n_slots <= 12.
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

	/* Recorded and logged only. Gating RX windows on beacon timing needs a
	 * real gateway to develop against — that is spec D. */
	LOG_INF("BEACON ver=%u counter=%u slots=%u our_slot=%d", proto_ver,
		frame_counter, n_slots, slot);
}

void uwb_slave_run(const uwb_config_t *cfg)
{
	/* Snapshot at mode-entry: anchor_shell.c's "anchor id/ant/pos" commands
	 * mutate the live uwb_config_get() singleton that cfg points at, and
	 * this loop runs indefinitely on the main thread with no synchronisation
	 * against the shell thread. Operating on a stable copy restores the
	 * documented "changes take effect only on reboot" contract -- notably
	 * for ant_delay_tx, which resp_tx_ts arithmetic must keep matching the
	 * antenna delay actually programmed into the radio at boot. */
	uwb_config_t cfg_snapshot = *cfg;
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

	rx_arm();

	while (1) {
		k_sem_take(&rx_sem, K_FOREVER);

		uint32_t status = rx_status;
		uint16_t flen = rx_len;
		rx_pending = false;

		/* flen counts the FCS. Anything that cannot hold one is not a
		 * frame we can reason about. */
		if (flen <= FCS_LEN || flen > RX_BUF_LEN) {
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

		/* Offered to each in turn; each ignores what is not its own. */
		anchor_respond_wave_poll(rx_buf, plen, rx_ts, cfg, &frame_seq_nb);
		anchor_respond_discovery(rx_buf, plen, rx_ts, cfg, &frame_seq_nb,
					 cir_power, cir_quality);
		observe_beacon(rx_buf, plen, cfg);

		rx_arm();
	}
}
