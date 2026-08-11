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

/* Legacy responder turnaround, unchanged from the nRF5 anchor and matching the
 * working ESP32S3UWB responder. */
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
	 * clears the bit ahead of us. */
	uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK);
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
}

void anchor_respond_wave_poll(const uint8_t *buf, uint16_t len, uint64_t poll_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq)
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
			      int32_t cir_power, uint16_t cir_quality)
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

	LOG_INF("DISC from 0x%04X -> resp src 0x%04X pwr=%d q=%u", tag_addr,
		uwb_config_short_addr(cfg), cir_power, cir_quality);

	uint32_t delay_uus = disc_resp_delay_uus(cfg->anchor_id);
	uint32_t resp_tx_time = (uint32_t)((disc_rx_ts +
		((uint64_t)delay_uus * UUS_TO_DWT_TIME)) >> 8);

	/* Module frames carry no FCS placeholder: ranging=0 adds FCS_LEN. */
	tx_delayed(out, (uint16_t)n, resp_tx_time, 0);
}
