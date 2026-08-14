/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_node.h.
 */

#include "apos_node.h"

#include "apos_frame.h"
#include "ss_initiator.h"
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

/* Static, not on the stack: CONFIG_MAIN_STACK_SIZE is 4096 and this is 256 B. */
static int32_t batch[APOS_MAX_EXCHANGES];

/* Mean and standard deviation of the batch, in mm. Returns the count kept.
 *
 * A median-absolute-deviation filter rather than cal_math.c's cal_filtered_mean:
 * that one is tuned for a batch measured against a known reference at a known
 * distance, and here nothing is known in advance. The sd is the whole point --
 * it becomes the edge weight in the fit and the operator's per-pair quality
 * signal -- so it must describe the kept samples, not the raw ones. */
static uint8_t batch_stats(const int32_t *s, uint8_t n, int32_t *mean_out,
			   uint16_t *sd_out)
{
	if (n == 0) {
		*mean_out = 0;
		*sd_out = 0;
		return 0;
	}

	/* Median via a partial insertion sort into a scratch copy: n <= 64, so
	 * O(n^2) is irrelevant and a real sort would be more code. */
	int32_t tmp[APOS_MAX_EXCHANGES];

	memcpy(tmp, s, (size_t)n * sizeof(tmp[0]));
	for (uint8_t i = 1; i < n; i++) {
		int32_t v = tmp[i];
		int8_t j = (int8_t)(i - 1);

		while (j >= 0 && tmp[j] > v) {
			tmp[j + 1] = tmp[j];
			j--;
		}
		tmp[j + 1] = v;
	}

	int32_t median = tmp[n / 2];

	/* Reject anything more than 300 mm from the median. A reflection or a
	 * half-decoded frame lands far outside that; honest ranging noise on
	 * this hardware is tens of millimetres. */
	int64_t sum = 0;
	uint8_t kept = 0;

	for (uint8_t i = 0; i < n; i++) {
		int32_t dev = s[i] - median;

		if (dev < 0) {
			dev = -dev;
		}
		if (dev <= 300) {
			sum += s[i];
			kept++;
		}
	}
	if (kept == 0) {
		*mean_out = median;
		*sd_out = 0;
		return 0;
	}

	int32_t mean = (int32_t)(sum / kept);
	int64_t var = 0;

	for (uint8_t i = 0; i < n; i++) {
		int32_t dev = s[i] - median;

		if (dev < 0) {
			dev = -dev;
		}
		if (dev <= 300) {
			int64_t d = (int64_t)s[i] - mean;

			var += d * d;
		}
	}

	/* Population sd over the kept samples. Divide by kept, not kept-1: with
	 * 40 samples the difference is under 2 % and this feeds a weight, not a
	 * hypothesis test. */
	uint32_t sd = 0;

	if (kept > 1) {
		int64_t v = var / kept;

		/* Integer sqrt: no float needed and this runs on the SLAVE
		 * loop's stack. */
		while ((int64_t)(sd + 1) * (sd + 1) <= v) {
			sd++;
		}
	}

	*mean_out = mean;
	*sd_out = (sd > UINT16_MAX) ? UINT16_MAX : (uint16_t)sd;
	return kept;
}

static void handle_range_cmd(const uint8_t *buf, uint16_t plen,
			     const uwb_config_t *cfg, uint8_t *seq,
			     struct beacon_guard *bg)
{
	uint16_t session = 0, peer_addr = 0;
	uint8_t n_exchanges = 0;

	if (apos_frame_parse_range_cmd(buf, plen, &session, &peer_addr,
				       &n_exchanges) != 0) {
		LOG_WRN("RANGE_CMD parse failed: plen=%u expected=%u — check "
			"the caller is passing flen - FCS_LEN",
			plen, APOS_LEN_RANGE_CMD);
		return;
	}

	/* Gate one: the window. Gate two: the session. src == 0x0000 was
	 * already checked by the caller. All three must hold before this board
	 * transmits a single unsolicited poll -- see apos_node.h. */
	if (!apos_node_window_open() || session != window_session) {
		LOG_WRN("RANGE_CMD refused — window closed or session mismatch");
		return;
	}
	if (peer_addr == UWB_ADDR_GATEWAY_RESERVED ||
	    peer_addr == uwb_config_short_addr(cfg)) {
		/* The gateway never answers a poll, and polling ourselves is
		 * meaningless. Either means the gateway's table is wrong. */
		LOG_WRN("RANGE_CMD refused — peer 0x%04X is not rangeable",
			peer_addr);
		return;
	}

	uint8_t want = (n_exchanges > APOS_MAX_EXCHANGES)
		       ? (uint8_t)APOS_MAX_EXCHANGES : n_exchanges;
	uint8_t valid = 0;

	/* The responder filters on the low byte of its short address, which is
	 * what the tag polls with too (anchor_respond.c:122). */
	uint8_t peer_wire_id = (uint8_t)(peer_addr & 0xFFu);

	ss_initiator_enter();
	for (uint8_t i = 0; i < want; i++) {
		int32_t mm = ss_initiator_range(peer_wire_id);

		if (mm != INT32_MIN) {
			batch[valid++] = mm;
		}
		/* Yields to the shell so the console survives the batch, and
		 * decorrelates consecutive exchanges slightly. */
		k_sleep(K_MSEC(2));
	}
	ss_initiator_leave();

	int32_t mean = 0;
	uint16_t sd = 0;
	uint8_t kept = batch_stats(batch, valid, &mean, &sd);

	LOG_INF("{\"apos_range\":{\"peer\":\"0x%04X\",\"tried\":%u,\"ok\":%u,"
		"\"kept\":%u,\"mean_mm\":%d,\"sd_mm\":%u}}",
		peer_addr, want, valid, kept, mean, sd);

	/* n_ok reports the KEPT count, not the raw count: the gateway's
	 * min_n_ok threshold is about how many samples the mean rests on, and
	 * outliers that were filtered out did not contribute to it. */
	int n = apos_frame_range_rsp_build(tx_buf, sizeof(tx_buf),
					  uwb_config_short_addr(cfg),
					  UWB_ADDR_GATEWAY_RESERVED, session,
					  peer_addr, mean, sd, kept);

	if (n < 0) {
		LOG_ERR("RANGE_RSP build failed (%d)", n);
		return;
	}
	apos_frame_set_seq(tx_buf, (*seq)++);

	/* ss_initiator_leave() has already restored the interrupt mask and
	 * cleared the RX timeouts, so a normal immediate TX is legal again. The
	 * SLAVE loop re-arms RX when this returns. */
	tx_now(tx_buf, (uint16_t)n, bg);
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
	case APOS_SUB_RANGE_CMD:
		window_refresh();
		handle_range_cmd(buf, plen, cfg, seq, bg);
		break;
	default:
		/* Every remaining subtype is anchor-to-gateway and should
		 * never arrive with src 0x0000. */
		break;
	}
	return true;
}
