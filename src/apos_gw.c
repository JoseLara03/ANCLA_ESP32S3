/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_gw.h.
 */

#include "apos_gw.h"

#include "apos_frame.h"
#include "apos_node.h"    /* APOS_MIN_N_OK, APOS_ENUM_* for the gap derivation */
#include "apos_store.h"
#include "uwb_config.h"
#include "uwb_dwtime.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include <deca_device_api.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(apos_gw, LOG_LEVEL_INF);

/* Same reasoning as apos_node's: every APOS transmission from here is
 * IMMEDIATE, so this only covers a frame's airtime, not a scheduled delay. */
#define TX_COMPLETE_TIMEOUT_MS 8

static struct apos_table tbl;
static struct apos_result res;
static bool have_result;

static uint8_t phase = APOS_GW_IDLE;
static uint16_t session;

/* Gauge as short addresses; 0 means unset, since 0x0000 is the gateway and can
 * never be a survey node. */
static uint16_t gauge_addr[4];

static float zoff_m;

/* ENUM phase */
static uint8_t enum_round;
static int64_t next_action_ms;

static uint8_t tx_buf[APOS_LEN_MAX];

void apos_gw_init(void)
{
	apos_table_init(&tbl);
	memset(&res, 0, sizeof(res));
	memset(gauge_addr, 0, sizeof(gauge_addr));
	have_result = false;
	phase = APOS_GW_IDLE;
	session = 0;
	zoff_m = 0.0f;
}

bool apos_gw_busy(void)
{
	return phase != APOS_GW_IDLE;
}

const struct apos_table *apos_gw_table(void)
{
	return &tbl;
}

bool apos_gw_gauge_set(void)
{
	return gauge_addr[0] != 0u && gauge_addr[1] != 0u &&
	       gauge_addr[2] != 0u && gauge_addr[3] != 0u;
}

void apos_gw_get_status(struct apos_gw_status *out)
{
	if (!out) {
		return;
	}
	memset(out, 0, sizeof(*out));
	out->phase = phase;
	out->session = session;
	out->n_peers = tbl.n_peers;
	out->have_result = have_result;
}

int apos_gw_set_gauge(uint16_t origin, uint16_t xaxis, uint16_t plane,
		      uint16_t up)
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}

	const uint16_t a[4] = {origin, xaxis, plane, up};

	for (int i = 0; i < 4; i++) {
		if (a[i] == 0u || a[i] == UWB_ADDR_GATEWAY_RESERVED) {
			return -EINVAL;
		}
		for (int j = i + 1; j < 4; j++) {
			if (a[i] == a[j]) {
				return -EINVAL;
			}
		}
	}
	memcpy(gauge_addr, a, sizeof(gauge_addr));
	return 0;
}

/* Transmit immediately. No beacon guard here, unlike apos_node: the caller only
 * calls apos_gw_step() when avail_uus leaves room before the beacon must be
 * armed, and the gateway owns the beacon rather than predicting it. */
static bool tx_now(uint16_t len, uint8_t *seq)
{
	apos_frame_set_seq(tx_buf, (*seq)++);

	dwt_forcetrxoff();
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	dwt_writetxdata(len, tx_buf, 0);
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

/* A session of 0 is reserved as "none", so retry until non-zero. */
static uint16_t new_session(void)
{
	uint16_t s = 0;

	while (s == 0u) {
		s = (uint16_t)sys_rand32_get();
	}
	return s;
}

static void send_survey_begin(uint8_t *seq)
{
	int n = apos_frame_survey_begin_build(tx_buf, sizeof(tx_buf),
					     UWB_ADDR_GATEWAY_RESERVED,
					     session, APOS_GW_WINDOW_S);

	if (n < 0) {
		LOG_ERR("SURVEY_BEGIN build failed (%d)", n);
		return;
	}
	tx_now((uint16_t)n, seq);
}

int apos_gw_start_enum(void)
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}

	apos_table_init(&tbl);
	have_result = false;
	session = new_session();
	enum_round = 0;
	next_action_ms = 0; /* fire on the very next step */
	phase = APOS_GW_ENUM;

	LOG_INF("{\"apos\":\"enumerating\",\"session\":%u}", session);
	return 0;
}

/* ---- RX ---- */

static void on_enum_rsp(const uint8_t *buf, uint16_t plen)
{
	uint16_t sess = 0;
	uint8_t eui[APOS_EUI_LEN];
	bool pv = false;
	float x = 0.0f, y = 0.0f, z = 0.0f;

	if (apos_frame_parse_enum_rsp(buf, plen, &sess, eui, &pv, &x, &y, &z)
	    != 0) {
		return;
	}
	if (sess != session) {
		return;
	}

	uint16_t addr = apos_frame_src(buf);

	/* apos_table deliberately does not filter this, so it is filtered here,
	 * BEFORE the table ever sees it: 0x0000 is the gateway's reserved
	 * address and a board answering with it is misconfigured. Letting it in
	 * would put a node into the survey geometry that no RANGE_CMD can ever
	 * be directed at, and whose SETPOS the gateway would address to itself. */
	if (addr == UWB_ADDR_GATEWAY_RESERVED) {
		LOG_WRN("{\"apos_error\":\"an anchor answered enumeration with "
			"the gateway's reserved address 0x0000 — ignored\"}");
		return;
	}

	int idx = apos_table_add_peer(&tbl, eui, addr, pv, x, y, z);

	if (idx == -EADDRINUSE) {
		/* Two distinct boards claiming one short address: both are
		 * configured with the same `anchor id`. Everything downstream
		 * addresses peers by short address, so continuing would range
		 * whichever board answered first and then push coordinates to
		 * whichever answered second. Abort loudly. */
		LOG_ERR("{\"apos_error\":\"duplicate short address 0x%04X — two "
			"boards share an `anchor id`. Fix with `anchor id` on "
			"one of them and re-run.\"}", addr);
		phase = APOS_GW_IDLE;
		return;
	}
	if (idx == -ENOSPC) {
		LOG_WRN("more than %u anchors answered — ignoring 0x%04X",
			APOS_MAX_NODES, addr);
		return;
	}
	if (idx < 0) {
		return;
	}

	LOG_INF("{\"apos_peer\":{\"idx\":%d,\"addr\":\"0x%04X\","
		"\"eui\":\"%02X%02X%02X%02X%02X%02X%02X%02X\",\"pos_valid\":%u,"
		"\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
		idx, addr, eui[0], eui[1], eui[2], eui[3], eui[4], eui[5],
		eui[6], eui[7], pv ? 1u : 0u, (double)x, (double)y, (double)z);
}

void apos_gw_on_rx(const uint8_t *buf, uint16_t plen)
{
	if (!apos_frame_is_apos(buf, plen)) {
		return;
	}
	if (phase == APOS_GW_IDLE) {
		return;
	}
	if (apos_frame_dest(buf) != UWB_ADDR_GATEWAY_RESERVED) {
		return;
	}

	switch (apos_frame_subtype(buf)) {
	case APOS_SUB_ENUM_RSP:
		on_enum_rsp(buf, plen);
		break;
	default:
		/* RANGE_RSP and SETPOS_ACK arrive in Tasks 11 and 12. */
		break;
	}
}

/* ---- Step ---- */

static void step_enum(uint8_t *seq)
{
	int64_t now = k_uptime_get();

	if (now < next_action_ms) {
		return;
	}

	if (enum_round >= APOS_GW_ENUM_ROUNDS) {
		LOG_INF("{\"apos_enum_done\":{\"peers\":%u}}", tbl.n_peers);
		phase = APOS_GW_IDLE;
		return;
	}

	send_survey_begin(seq);
	enum_round++;
	next_action_ms = now + APOS_GW_ENUM_GAP_MS;
}

void apos_gw_step(uint32_t avail_uus, uint8_t *seq)
{
	if (phase == APOS_GW_IDLE) {
		return;
	}
	if (avail_uus < APOS_GW_STEP_BUDGET_UUS) {
		/* Not enough room before the beacon. Skipping costs latency
		 * only: every deadline below is absolute wall-clock. */
		return;
	}

	switch (phase) {
	case APOS_GW_ENUM:
		step_enum(seq);
		break;
	default:
		/* APOS_GW_RANGE and APOS_GW_APPLY are Tasks 11 and 12's to
		 * implement. Unreachable until one of them sets those phases --
		 * but a phase with no handler means apos_gw_busy() stays true
		 * forever with nothing advancing it, which from the console is
		 * indistinguishable from a wedged radio. Say so loudly rather
		 * than hanging in silence. */
		LOG_ERR("{\"apos_error\":\"no step handler for phase %u — "
			"survey is stuck; `kernel reboot cold` to clear\"}",
			phase);
		phase = APOS_GW_IDLE;
		break;
	}
}
