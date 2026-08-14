/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_node.h.
 */

#include "apos_node.h"

#include "apos_frame.h"
#include "uwb_dwtime.h"
#include "uwb_store.h"

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <string.h>

LOG_MODULE_REGISTER(apos_node, LOG_LEVEL_INF);

/* Bound for the post-dwt_starttx() TXFRS wait. Every APOS transmission from
 * this module is IMMEDIATE, not delayed, so unlike anchor_respond.c's 18 ms
 * this only has to cover a frame's airtime -- the longest APOS frame is 34
 * bytes, well under 2 ms at 850 kbps with a PLEN_1024 preamble. 8 ms is
 * generous and still far short of anything that would matter to the loop. */
#define TX_COMPLETE_TIMEOUT_MS 8

/* How far ahead of an immediate TX to test the beacon guard. dwt_starttx()
 * plus the SPI write is well under this; it exists so the guard is asked about
 * the instant the frame actually leaves rather than the instant we asked. */
#define TX_GUARD_LOOKAHEAD_UUS 500u

/* The open window's deadline in kernel uptime, and the session it belongs to.
 * A session of 0 is never issued by the gateway, so it doubles as "no session". */
static int64_t window_deadline_ms;
static uint16_t window_session;

static uint8_t eui[APOS_EUI_LEN];
static bool eui_ready;

static uint8_t tx_buf[APOS_LEN_MAX];

void apos_node_init(void)
{
	window_deadline_ms = 0;
	window_session = 0;

	/* hwinfo may return fewer than 8 bytes. Zero-pad rather than leaving
	 * the tail uninitialised: the tail is hashed for the stagger and
	 * compared for identity, so uninitialised bytes would make this board's
	 * identity change between boots. */
	memset(eui, 0, sizeof(eui));

	ssize_t n = hwinfo_get_device_id(eui, sizeof(eui));

	if (n <= 0) {
		LOG_ERR("hwinfo_get_device_id failed (%d) — this board has no "
			"unique id and cannot take part in a survey",
			(int)n);
		eui_ready = false;
		return;
	}
	eui_ready = true;
	LOG_INF("{\"apos\":\"ready\",\"eui\":\"%02X%02X%02X%02X%02X%02X%02X%02X\"}",
		eui[0], eui[1], eui[2], eui[3], eui[4], eui[5], eui[6], eui[7]);
}

const uint8_t *apos_node_eui(void)
{
	return eui;
}

bool apos_node_window_open(void)
{
	if (window_session == 0) {
		return false;
	}
	if (k_uptime_get() > window_deadline_ms) {
		/* Self-closing: no timer and no tick, so a gateway that dies
		 * mid-survey cannot leave this board permanently willing to
		 * transmit. */
		window_session = 0;
		LOG_INF("{\"apos\":\"window closed\",\"reason\":\"expired\"}");
		return false;
	}
	return true;
}

static void window_open(uint16_t session, uint16_t window_s)
{
	window_session = session;
	window_deadline_ms = k_uptime_get() + (int64_t)window_s * 1000;
}

/* Push the deadline out on a SETPOS matching the current session, so a long
 * survey does not need the gateway to re-broadcast SURVEY_BEGIN just to keep
 * the window alive. Called only from handle_setpos(), after the session
 * check, so a stale-session SETPOS cannot extend a window it does not belong
 * to. */
static void window_refresh(void)
{
	if (window_session != 0) {
		window_deadline_ms = k_uptime_get() + (int64_t)1000 *
				     APOS_NODE_REFRESH_S;
	}
}

/* Transmit immediately. Returns false if the frame did not go out.
 *
 * Safe to poll TXFRS: uwb_slave.c enables DWT_INT_RX only, so dwt_isr() never
 * runs for TXFRS and cannot clear the bit ahead of this wait. Bounded anyway --
 * an unbounded wait here once froze the whole console. */
static bool tx_now(const uint8_t *buf, uint16_t len, struct beacon_guard *bg)
{
	uint32_t now = dwt_readsystimestamphi32();

	if (bg && !beacon_guard_tx_allowed(bg,
			now + UUS_TO_HI32(TX_GUARD_LOOKAHEAD_UUS))) {
		LOG_WRN("APOS TX suppressed — would land on the beacon");
		return false;
	}

	dwt_forcetrxoff();
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	/* dwt_writetxdata() only reads from buf; the driver's signature is not
	 * const-correct, hence the cast rather than a non-const parameter here. */
	dwt_writetxdata(len, (uint8_t *)(uintptr_t)buf, 0);
	dwt_writetxfctrl((uint16_t)(len + FCS_LEN), 0, 0);

	if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("APOS TX failed to start");
		return false;
	}
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       TX_COMPLETE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		LOG_WRN("APOS TX started but TXFRS never completed — forced off");
		return false;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	return true;
}

/* FNV-1a over the EUI, reduced to a slot. Any cheap avalanche would do; what
 * matters is that it depends on every byte, since boards from one batch differ
 * only in the last one or two. */
static uint32_t enum_slot(void)
{
	uint32_t h = 2166136261u;

	for (int i = 0; i < APOS_EUI_LEN; i++) {
		h ^= eui[i];
		h *= 16777619u;
	}
	return h % APOS_ENUM_SLOTS;
}

static void handle_survey_begin(const uint8_t *buf, uint16_t plen,
				const uwb_config_t *cfg, uint8_t *seq,
				struct beacon_guard *bg)
{
	uint16_t session = 0, window_s = 0;

	if (apos_frame_parse_survey_begin(buf, plen, &session, &window_s) != 0) {
		LOG_WRN("SURVEY_BEGIN parse failed: plen=%u expected=%u — check "
			"the caller is passing flen - FCS_LEN",
			plen, APOS_LEN_SURVEY_BEGIN);
		return;
	}
	if (session == 0 || !eui_ready) {
		return;
	}

	window_open(session, window_s);
	LOG_INF("{\"apos\":\"window open\",\"session\":%u,\"window_s\":%u}",
		session, window_s);

	/* Sleep out our stagger slot before replying. Blocking the SLAVE loop
	 * for up to 210 ms is acceptable here and nowhere else: this happens
	 * only when an operator triggers a survey, and the alternative -- a
	 * timer plus a deferred TX path -- would add a second thread touching
	 * the SPI bus. */
	k_sleep(K_MSEC(enum_slot() * APOS_ENUM_SLOT_MS));

	int n = apos_frame_enum_rsp_build(tx_buf, sizeof(tx_buf),
					 uwb_config_short_addr(cfg),
					 UWB_ADDR_GATEWAY_RESERVED, session,
					 eui, cfg->position_valid,
					 cfg->x, cfg->y, cfg->z);

	if (n < 0) {
		LOG_ERR("ENUM_RSP build failed (%d)", n);
		return;
	}
	apos_frame_set_seq(tx_buf, (*seq)++);
	tx_now(tx_buf, (uint16_t)n, bg);
}

static void handle_setpos(const uint8_t *buf, uint16_t plen,
			  uwb_config_t *cfg, uint8_t *seq,
			  struct beacon_guard *bg)
{
	uint16_t session = 0;
	float x = 0.0f, y = 0.0f, z = 0.0f;

	if (apos_frame_parse_setpos(buf, plen, &session, &x, &y, &z) != 0) {
		LOG_WRN("SETPOS parse failed: plen=%u expected=%u — check the "
			"caller is passing flen - FCS_LEN",
			plen, APOS_LEN_SETPOS);
		return;
	}
	if (session != window_session) {
		LOG_WRN("SETPOS for session %u ignored — current is %u",
			session, window_session);
		return;
	}
	window_refresh();

	/* Applied to the loop's snapshot AND to the shared singleton, then
	 * persisted. The snapshot is what anchor_respond.c reports to the tag,
	 * so updating it is what makes the new coordinates take effect without
	 * a reboot; the singleton is what uwb_store_save_pos() reads. */
	uwb_config_set_pos(cfg, x, y, z);
	uwb_config_set_pos(uwb_config_get(), x, y, z);

	int rc = uwb_store_save_pos();
	bool ok = (rc == 0);

	if (!ok) {
		LOG_ERR("SETPOS applied but NOT persisted (%d) — will be lost "
			"on reboot", rc);
	} else {
		LOG_INF("{\"apos_setpos\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
			(double)x, (double)y, (double)z);
	}

	/* The ACK reports what was actually stored, not what was asked for, so
	 * a failed NVS write is visible at the gateway rather than only in this
	 * board's log. A half-applied survey is the one outcome worth being
	 * loud about. */
	int n = apos_frame_setpos_ack_build(tx_buf, sizeof(tx_buf),
					   uwb_config_short_addr(cfg),
					   UWB_ADDR_GATEWAY_RESERVED, session,
					   cfg->x, cfg->y, cfg->z, ok);

	if (n < 0) {
		LOG_ERR("SETPOS_ACK build failed (%d)", n);
		return;
	}
	apos_frame_set_seq(tx_buf, (*seq)++);
	tx_now(tx_buf, (uint16_t)n, bg);
}

static void handle_survey_end(const uint8_t *buf, uint16_t plen)
{
	uint16_t session = 0;

	if (apos_frame_parse_survey_end(buf, plen, &session) != 0) {
		LOG_WRN("SURVEY_END parse failed: plen=%u expected=%u — check "
			"the caller is passing flen - FCS_LEN",
			plen, APOS_LEN_SURVEY_END);
		return;
	}
	if (session != window_session) {
		return;
	}
	window_session = 0;
	LOG_INF("{\"apos\":\"window closed\",\"reason\":\"survey end\"}");
}

bool apos_node_on_rx(const uint8_t *buf, uint16_t plen, uwb_config_t *cfg,
		     uint8_t *seq, struct beacon_guard *bg)
{
	if (!apos_frame_is_apos(buf, plen)) {
		return false;
	}

	/* Only the gateway issues survey traffic. Filtering here rather than
	 * per-subtype means a stray or spoofed APOS frame from another anchor
	 * cannot open a window or move this board's coordinates. */
	if (apos_frame_src(buf) != UWB_ADDR_GATEWAY_RESERVED) {
		return true;
	}

	uint16_t dest = apos_frame_dest(buf);

	if (dest != APOS_ADDR_BCAST && dest != uwb_config_short_addr(cfg)) {
		return true;
	}

	switch (apos_frame_subtype(buf)) {
	case APOS_SUB_SURVEY_BEGIN:
		handle_survey_begin(buf, plen, cfg, seq, bg);
		break;
	case APOS_SUB_SETPOS:
		handle_setpos(buf, plen, cfg, seq, bg);
		break;
	case APOS_SUB_SURVEY_END:
		handle_survey_end(buf, plen);
		break;
	default:
		/* RANGE_CMD lands here until Task 7. Every other subtype is
		 * anchor-to-gateway and should never arrive with src 0x0000. */
		break;
	}
	return true;
}
