/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ported from the nRF5 anchor (fw-cre Src/anchor_respond.c). Both responders
 * keep their original shape; what changed is the short address on the wire
 * (the MAC contract reserves 0x0000 for the gateway) and the removal of
 * anchor_read_cir() -- under the callback API the CIA status bit is already
 * cleared by the time we run, so uwb_slave.c captures CIR instead.
 */

#include "anchor_respond.h"

#include "disc_schedule.h"
#include "uwb_dwtime.h"
#include "uwb_frame_802_15_4z.h"

#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <string.h>

LOG_MODULE_REGISTER(anchor_respond, LOG_LEVEL_INF);

/* Bound for tx_delayed()'s post-dwt_starttx() TXFRS wait. Must exceed the
 * WORST-CASE scheduled delay across both responders, not just a frame's
 * airtime -- dwt_starttx() returns almost immediately after arming a
 * delayed TX; the transmission itself doesn't happen until the full
 * scheduled delay has elapsed. Worst case today is DISCOVERY at anchor id 3:
 * disc_resp_delay_uus(3) = DISC_BASE_UUS + 3*DISC_SLOT_UUS = 2000 + 3*3500 =
 * 12500 uus (~12.5 ms). Re-derived after DISC_BASE_UUS dropped from 6000 to
 * 2000 on the fast (26.67 MHz) SPI bus:
 *   ceil(12500 * 1.0256 / 1000) + 5 = ceil(12.82) + 5 = 13 + 5 = 18.
 * An earlier version of this bound (10 ms) was shorter than the scheduled
 * delay for any anchor id >= 2, so it force-cancelled transmissions that
 * hadn't had a chance to fire yet, on every single attempt for those ids.
 * Revisit this constant if DISC_BASE_UUS/DISC_SLOT_UUS/
 * POLL_RX_TO_RESP_TX_DLY_UUS are ever tuned further. */
#define TX_COMPLETE_TIMEOUT_MS 18

/* Legacy responder turnaround. Was 2000, carried over unchanged from the
 * nRF5 anchor and the working (non-Zephyr) ESP32S3UWB responder; measured
 * ~1700 uus short on this ANCLA_ESP32S3/Zephyr port for the DISCOVERY path
 * (see DISC_BASE_UUS in disc_schedule.h for the full story, including that
 * dropping to 2400 after removing the per-frame logging still failed 100%
 * of the time -- logging was not the dominant overhead). 6000 confirmed
 * working on hardware for DISCOVERY; not yet independently exercised for
 * this WAVE path specifically, but both share tx_delayed() and the same
 * driver overhead, so matching it here rather than leaving this one at a
 * value already shown to fail.
 * 2000 confirmed working after the SPI bus was switched to fast rate
 * (26.67 MHz): the earlier 2000/2400 failures were the 2 MHz bus, not the
 * driver's per-call overhead -- read_cir()'s 216-byte transfer alone was
 * 864 uus of pure clock time at 2 MHz. Bench-confirmed for DISCOVERY via an
 * external sniffer (clean WAVE/DISCOVERY responses, no TX misses); matching
 * it here for WAVE as before, since both share tx_delayed() and the same
 * driver overhead. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2000u

/* ---- Legacy WAVE/0xE0 -> VEWA/0xE1 ---- */
static const uint8_t rx_poll_ref[] = {
	0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0
};

#define POS_ANCHOR_ID_IDX  10
#define POS_POLL_RX_TS_IDX 11
#define POS_RESP_TX_TS_IDX 15
#define POS_ANCHOR_X_IDX   19
#define POS_ANCHOR_Y_IDX   23

static uint8_t tx_resp_msg[] = {
	0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1,
	0,          /* anchor id  @10 */
	0, 0, 0, 0, /* poll_rx_ts @11..14 */
	0, 0, 0, 0, /* resp_tx_ts @15..18 */
	0, 0, 0, 0, /* x          @19..22 */
	0, 0, 0, 0, /* y          @23..26 */
	0, 0        /* FCS        @27..28 */
};

static void tx_delayed(const uint8_t *buf, uint16_t payload_len, uint32_t tx_time,
		       int ranging)
{
	dwt_setdelayedtrxtime(tx_time);
	dwt_writetxdata(payload_len, (uint8_t *)(uintptr_t)buf, 0);
	dwt_writetxfctrl((uint16_t)(payload_len + (ranging ? 0u : FCS_LEN)), 0, ranging);

	if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("delayed TX missed its slot — response dropped");
		return;
	}

	/* Safe to poll: TXFRS is not in the enabled interrupt mask (uwb_slave.c
	 * enables DWT_INT_RX only), so dwt_isr() never runs for it and never
	 * clears the bit ahead of us. Bounded, not a plain spin: dwt_starttx()'s
	 * own deadline check can race a borderline-late delayed TX and still
	 * report DWT_SUCCESS for a transmission that never completes -- an
	 * unbounded wait here hung the whole console (main thread is priority 0)
	 * the first time that race landed on real hardware. See
	 * TX_COMPLETE_TIMEOUT_MS for why the bound has to cover the scheduled
	 * delay itself, not just the frame's airtime. */
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK, TX_COMPLETE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		LOG_WRN("delayed TX started but TXFRS never completed — forced off");
		return;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
}

void anchor_respond_wave_poll(const uint8_t *buf, uint16_t len, uint64_t poll_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq,
			      struct beacon_guard *bg)
{
	/* The id byte the tag polls with is the low byte of our short address,
	 * because that is what it read out of our DISCOVERY response
	 * (tag uwb_net_runner.c: "anchor ID is taken from the low byte of
	 * src_addr"). Filter and reply must use the same value or the tag
	 * discards the response. */
	const uint8_t wire_id = (uint8_t)(uwb_config_short_addr(cfg) & 0xFFu);

	if (len < POS_ANCHOR_ID_IDX + 1) {
		return;
	}
	/* Byte 2 is the tag's per-poll sequence number — skip it. */
	if (memcmp(buf, rx_poll_ref, 2) != 0) {
		return;
	}
	if (memcmp(buf + 3, rx_poll_ref + 3, 7) != 0) {
		return;
	}
	if (buf[POS_ANCHOR_ID_IDX] != wire_id) {
		return;
	}

	uint32_t resp_tx_time = (uint32_t)((poll_rx_ts +
		((uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
	uint64_t resp_tx_ts =
		(((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + cfg->ant_delay_tx;

	if (bg && !beacon_guard_tx_allowed(bg, resp_tx_time)) {
		LOG_DBG("WAVE response suppressed — would land on the beacon");
		return;
	}

	tx_resp_msg[2] = (*seq)++;
	tx_resp_msg[POS_ANCHOR_ID_IDX] = wire_id;
	uwb_resp_msg_set_ts(&tx_resp_msg[POS_POLL_RX_TS_IDX], poll_rx_ts);
	uwb_resp_msg_set_ts(&tx_resp_msg[POS_RESP_TX_TS_IDX], resp_tx_ts);
	memcpy(&tx_resp_msg[POS_ANCHOR_X_IDX], &cfg->x, sizeof(float));
	memcpy(&tx_resp_msg[POS_ANCHOR_Y_IDX], &cfg->y, sizeof(float));

	/* tx_resp_msg already carries the two FCS placeholder bytes, hence
	 * ranging=1 and no extra FCS_LEN in writetxfctrl. */
	tx_delayed(tx_resp_msg, sizeof(tx_resp_msg), resp_tx_time, 1);
}

void anchor_respond_discovery(const uint8_t *buf, uint16_t len, uint64_t disc_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq,
			      int32_t cir_power, uint16_t cir_quality,
			      struct beacon_guard *bg)
{
	if (!uwb_frame_is_discovery(buf, len)) {
		return;
	}

	uint16_t tag_addr = uwb_frame_get_src_addr(buf);
	uint8_t out[UWB_FRAME_LEN_RESP];

	int n = uwb_frame_response_build(out, sizeof(out),
					 uwb_config_short_addr(cfg), tag_addr,
					 0u, /* tx_ts unused for anchor selection */
					 cir_power, cir_quality);
	if (n < 0) {
		LOG_WRN("response build failed (%d)", n);
		return;
	}
	uwb_frame_set_seq_num(out, (*seq)++);

	uint32_t delay_uus = disc_resp_delay_uus(cfg->anchor_id);
	uint32_t resp_tx_time = (uint32_t)((disc_rx_ts +
		((uint64_t)delay_uus * UUS_TO_DWT_TIME)) >> 8);

	if (bg && !beacon_guard_tx_allowed(bg, resp_tx_time)) {
		LOG_DBG("DISCOVERY response suppressed — would land on the beacon");
		return;
	}

	/* Module frames carry no FCS placeholder: ranging=0 adds FCS_LEN.
	 * Logged after, not before: a synchronous log call ahead of the
	 * delayed-TX setup would eat into DISC_BASE_UUS/DISC_SLOT_UUS's already
	 * tight budget on this port (see disc_schedule.h). */
	tx_delayed(out, (uint16_t)n, resp_tx_time, 0);

	/* LOG_DBG, not LOG_INF: compiles out at this module's LOG_LEVEL_INF.
	 * Link-budget timing matters more than per-DISCOVERY visibility here --
	 * bump the module's registered level to re-enable for debugging. */
	LOG_DBG("DISC from 0x%04X -> resp src 0x%04X pwr=%d q=%u", tag_addr,
		uwb_config_short_addr(cfg), cir_power, cir_quality);
}
