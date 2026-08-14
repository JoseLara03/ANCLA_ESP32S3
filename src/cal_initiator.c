/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal_initiator.h.
 */

#include "cal_initiator.h"

#include "uwb_dwtime.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <string.h>

LOG_MODULE_REGISTER(cal_initiator, LOG_LEVEL_INF);

/* Speed of light in air as every peer on this network uses it: the tag
 * (uwb_ss_initiator.c) and the sibling ESP-IDF anchor (ranging.c) both use
 * 299702547.0. A different constant here would show up as a fixed scale error. */
#define SPEED_OF_LIGHT 299702547.0
#define DWT_TIME_UNITS_V (1.0 / 499.2e6 / 128.0)

/* RX opens this long after our poll finishes. The response preamble cannot
 * arrive sooner: the peer's RMARKER is 2000 uus after its own poll_rx_ts and
 * the PLEN_1024 preamble runs ~1.05 ms ahead of that. */
#define RX_AFTER_TX_UUS 300U

/* RX window. Both peer types turn around in 2000 uus (the DWM3001CDK's stock
 * 450 is raised to match -- at PLEN_1024, 450 is shorter than the preamble and
 * physically impossible), so one window covers both. */
#define RESP_TIMEOUT_UUS 4000U

#define TX_DONE_TIMEOUT_MS 10U
#define RX_DONE_TIMEOUT_MS 25U

#define ALL_MSG_COMMON_LEN 10U
#define ALL_MSG_SN_IDX 2U
#define POLL_PEER_ID_IDX 10U

/* Stock Qorvo ss_twr_responder response: 18 payload bytes. */
#define RESP_LEN_STOCK 18U
#define RESP_TS_IDX_STOCK_POLL_RX 10U
#define RESP_TS_IDX_STOCK_RESP_TX 14U

/* ANCLA VEWA response (anchor_respond.c:73): 27 payload bytes, both timestamp
 * fields pushed one byte later by the anchor-id byte at index 10. */
#define RESP_LEN_ANCLA 27U
#define RESP_TS_IDX_ANCLA_POLL_RX 11U
#define RESP_TS_IDX_ANCLA_RESP_TX 15U

#define RX_BUF_LEN 64

/* 11 bytes: the 10 stock Qorvo header bytes plus a peer-id byte. The stock
 * responder compares only the first ALL_MSG_COMMON_LEN and ignores the
 * eleventh; the ANCLA responder requires it and filters on it
 * (anchor_respond.c:124,134). One frame therefore addresses either peer, and
 * exactly one board ever answers. */
static uint8_t tx_poll[] = {
	0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0,
	0 /* peer id @10 */
};

static const uint8_t rx_resp_ref[] = {
	0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1
};

static uint8_t poll_seq;

void cal_initiator_enter(void)
{
	dwt_forcetrxoff();
	/* Every interrupt off: this path polls SYS_STATUS, and an enabled ISR
	 * would clear TXFRS/RXFCG before the poll could see them. */
	dwt_setinterrupt(0xFFFFFFFFU, 0xFFFFFFFFU, DWT_DISABLE_INT);
	dwt_setrxaftertxdelay(RX_AFTER_TX_UUS);
	dwt_setrxtimeout(RESP_TIMEOUT_UUS);
	dwt_setpreambledetecttimeout(0);
}

void cal_initiator_leave(void)
{
	dwt_forcetrxoff();
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	dwt_setpreambledetecttimeout(0);
	dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT);
}

/* Bounded wait for ANY bit in mask. uwb_wait_for_sysstatus_lo() waits for ALL
 * of them, which cannot express "RXFCG or RXTO or RXERR". */
static uint32_t wait_any_sysstatus_lo(uint32_t mask, uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + (int64_t)timeout_ms;

	for (;;) {
		uint32_t status = dwt_readsysstatuslo();

		if (status & mask) {
			return status & mask;
		}
		if (k_uptime_get() > deadline) {
			return 0;
		}
	}
}

static uint32_t ts_from(const uint8_t *p)
{
	uint32_t ts = 0;

	for (int i = 0; i < 4; i++) {
		ts |= (uint32_t)p[i] << (i * 8);
	}
	return ts;
}

int32_t cal_initiator_range(uint8_t peer_wire_id)
{
	tx_poll[ALL_MSG_SN_IDX] = poll_seq++;
	tx_poll[POLL_PEER_ID_IDX] = peer_wire_id;

	dwt_writetxdata(sizeof(tx_poll), tx_poll, 0);
	dwt_writetxfctrl(sizeof(tx_poll) + FCS_LEN, 0, 1);

	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

	if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) !=
	    DWT_SUCCESS) {
		return INT32_MIN;
	}

	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       TX_DONE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		return INT32_MIN;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

	uint32_t rx_mask = DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
			   SYS_STATUS_ALL_RX_ERR;
	uint32_t got = wait_any_sysstatus_lo(rx_mask, RX_DONE_TIMEOUT_MS);

	if (!(got & DWT_INT_RXFCG_BIT_MASK)) {
		dwt_forcetrxoff();
		dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		return INT32_MIN;
	}
	dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

	uint8_t rng = 0;
	uint16_t flen = dwt_getframelength(&rng);

	if (flen <= FCS_LEN || flen > RX_BUF_LEN) {
		return INT32_MIN;
	}

	uint8_t buf[RX_BUF_LEN];

	dwt_readrxdata(buf, flen, 0);

	/* flen includes the FCS -- always subtract before deriving anything
	 * from the length, which is exactly what the layout choice below does. */
	uint16_t plen = (uint16_t)(flen - FCS_LEN);

	buf[ALL_MSG_SN_IDX] = 0;
	if (memcmp(buf, rx_resp_ref, ALL_MSG_COMMON_LEN) != 0) {
		return INT32_MIN;
	}

	uint16_t poll_rx_idx, resp_tx_idx;

	if (plen == RESP_LEN_ANCLA) {
		poll_rx_idx = RESP_TS_IDX_ANCLA_POLL_RX;
		resp_tx_idx = RESP_TS_IDX_ANCLA_RESP_TX;
	} else if (plen == RESP_LEN_STOCK) {
		poll_rx_idx = RESP_TS_IDX_STOCK_POLL_RX;
		resp_tx_idx = RESP_TS_IDX_STOCK_RESP_TX;
	} else {
		LOG_DBG("unknown response layout, plen=%u", plen);
		return INT32_MIN;
	}

	uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
	uint32_t resp_rx_ts = dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
	uint32_t poll_rx_ts = ts_from(&buf[poll_rx_idx]);
	uint32_t resp_tx_ts = ts_from(&buf[resp_tx_idx]);

	/* Mandatory, not a refinement. SS-TWR range error from crystal offset
	 * is c/2 * ppm * T_reply; at this project's 2000 uus turnaround 5 ppm is
	 * ~1.5 m, which dwarfs the antenna-delay error being measured. Both
	 * existing peers correct for it identically -- tag uwb_ss_initiator.c
	 * and ESP32S3UWB ranging.c:301. */
	double clk_off = ((double)dwt_readclockoffset()) / (double)(1U << 26);
	int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
	int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);
	double tof = ((rtd_init - rtd_resp * (1.0 - clk_off)) / 2.0) *
		     DWT_TIME_UNITS_V;

	return (int32_t)(tof * SPEED_OF_LIGHT * 1000.0);
}
