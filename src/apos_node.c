/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_node.h.
 */

#include "apos_node.h"

#include "apos_frame.h"
#include "apos_stats.h"
#include "ccp_sched.h"
#include "ccp_slave.h"
#include "node_eui.h"
#include "ss_initiator.h"
#include "uwb_dwtime.h"
#include "uwb_store.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

#include <deca_device_api.h>

#include <string.h>

LOG_MODULE_REGISTER(apos_node, LOG_LEVEL_INF);

/* Bound for the post-dwt_starttx() TXFRS wait. Every APOS transmission from
 * this module is IMMEDIATE, not delayed, so unlike anchor_respond.c's 18 ms
 * this only has to cover a frame's airtime -- the longest APOS frame is 34
 * bytes, well under 2 ms at 850 kbps (PLEN_256 preamble since 2026-10-12; ~1.05 ms at PLEN_1024). 8 ms is
 * generous and still far short of anything that would matter to the loop. */
#define TX_COMPLETE_TIMEOUT_MS 8

/* How far ahead of an immediate TX to test the beacon guard. dwt_starttx()
 * plus the SPI write is well under this; it exists so the guard is asked about
 * the instant the frame actually leaves rather than the instant we asked. */
#define TX_GUARD_LOOKAHEAD_UUS 500u

/* The span around each beacon in which the GATEWAY is not receiving, relative
 * to the beacon's RMARKER. Every frame this module sends is addressed to the
 * gateway, and one sent in here is lost with no error on either side.
 *
 *   before: uwb_gateway.c expires its RX window BEACON_ARM_MARGIN_UUS ahead of
 *           the beacon and then sits in the armed delayed TX. The whole reply
 *           has to END before that, hence the extra 1 ms for its own airtime
 *           (a 22-byte RANGE_RSP is ~0.55 ms at PLEN_256) and the arm.
 *   after:  the beacon, the CCP held back to CCP_TX_PREAMBLE_DELAY_NS, then
 *           the gateway's own post-CCP bookkeeping before it re-arms RX.
 *           CCP_SCHED_GW_TX_EARLIEST_NS is where the gateway itself resumes
 *           transmitting; 2 ms more covers tdoa_gw_step() and the re-arm.
 *
 * Hit for real after the PLEN_1024 -> PLEN_256 change (2026-10-16): a
 * 40-exchange RANGE_CMD batch shrank from ~228 ms to ~194 ms, i.e. from
 * "ends 30 ms after the next beacon" to "ends ON the next beacon", and whole
 * runs of pairs came back as "no RANGE_RSP" with both retries failing the same
 * way, because a retry re-sends the RANGE_CMD at the same superframe phase. */
#define GW_DEAF_BEFORE_UUS (BEACON_ARM_MARGIN_UUS + 1000u)
#define GW_DEAF_AFTER_UUS  (CCP_SCHED_GW_TX_EARLIEST_NS / 1000u + 2000u)

/* Leaving the deaf window must also clear tx_now()'s own beacon guard, or the
 * deferred reply is then suppressed instead. */
BUILD_ASSERT(GW_DEAF_AFTER_UUS > BEACON_OCCUPANCY_UUS + BEACON_GUARD_UUS +
				  TX_GUARD_LOOKAHEAD_UUS,
	     "deferred APOS reply would land inside the beacon guard");

/* The open window's deadline in kernel uptime, and the session it belongs to.
 * A session of 0 is never issued by the gateway, so it doubles as "no session". */
static int64_t window_deadline_ms;
static uint16_t window_session;

/* SURVEY_BEGIN broadcasts seen so far for the CURRENT session, used to salt the
 * enumeration stagger so successive rounds draw independent slots. Reset when a
 * new session opens; see enum_slot(). */
static uint8_t begin_seen;

static uint8_t eui[APOS_EUI_LEN];
static bool eui_ready;

static uint8_t tx_buf[APOS_LEN_MAX];

/* How long after a confirmed SETADDR_ACK this board reboots into its new
 * address. Long enough for the deferred log thread to drain the "rebooting"
 * line; the ACK itself is already on air (TXFRS confirmed) before this starts. */
#define SETADDR_REBOOT_DELAY_MS 500

static struct k_work_delayable reboot_work;

static void reboot_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	/* The same reset the remote `reboot` command uses (net_uplink.c), so
	 * the WiFi driver sees one kind of restart, not a third. */
	sys_reboot(SYS_REBOOT_COLD);
}

void apos_node_init(void)
{
	window_deadline_ms = 0;
	window_session = 0;
	begin_seen = 0;
	k_work_init_delayable(&reboot_work, reboot_fn);

	/* node_eui owns the read, the zero padding and the all-zero check; a
	 * board without a valid identity cannot take part in a survey. */
	memcpy(eui, node_eui(), sizeof(eui));
	eui_ready = node_eui_valid();
	if (!eui_ready) {
		LOG_ERR("no valid EUI — this board cannot take part in a survey");
		return;
	}
	LOG_INF("{\"apos\":\"ready\",\"eui\":\"%s\"}", node_eui_hex());
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

/* Push the deadline out on an in-session SETPOS or RANGE_CMD, so a long
 * survey does not need the gateway to re-broadcast SURVEY_BEGIN just to keep
 * the window alive. Called only from handle_setpos() and handle_range_cmd(),
 * both AFTER their own session check, so a stale-session or malformed frame
 * of either kind cannot extend a window it does not belong to -- that check
 * is what keeps the window's self-closing property meaningful under a stream
 * of garbage RANGE_CMDs. */
static void window_refresh(void)
{
	if (window_session != 0) {
		window_deadline_ms = k_uptime_get() + (int64_t)1000 *
				     APOS_NODE_REFRESH_S;
	}
}

/* If "now" falls in the gateway's deaf window around the predicted beacon
 * (GW_DEAF_BEFORE_UUS / GW_DEAF_AFTER_UUS), sleep until it has passed.
 *
 * Read-only on the guard: the beacon is rolled forward locally rather than via
 * beacon_guard_tx_allowed(), which tx_now() still calls afterwards and which
 * owns the miss accounting. A board that has just spent a whole superframe in
 * ss_initiator mode never saw the beacon it ranged across, so next_beacon is
 * routinely one period behind here -- that is the case this exists for. With
 * no lock, or a prediction older than the guard itself would trust, there is
 * nothing to defer against and the frame goes now. Worst case ~11 ms of
 * sleep on the SLAVE loop, only during an operator-triggered survey. */
static void gw_rx_wait(const struct beacon_guard *bg)
{
	if (bg == NULL || !beacon_guard_locked(bg)) {
		return;
	}

	const uint32_t period = UUS_TO_HI32(T_SUPERFRAME_UUS);
	const uint32_t before = UUS_TO_HI32(GW_DEAF_BEFORE_UUS);
	const uint32_t after = UUS_TO_HI32(GW_DEAF_AFTER_UUS);
	uint32_t now = dwt_readsystimestamphi32();
	uint32_t nb = beacon_guard_next(bg);

	for (int i = 0; (int32_t)(now - (nb + after)) > 0; i++) {
		if (i >= BEACON_GUARD_MAX_MISSES) {
			return; /* prediction too stale to act on */
		}
		nb += period;
	}
	if ((int32_t)(now - (nb - before)) < 0) {
		return; /* clear of the window */
	}

	/* hi32 tick = 256 DTU ~= 4.006 ns; +1 us so the sleep never ends early. */
	uint32_t wait_us = (uint32_t)(((uint64_t)(nb + after - now) * 4006u) /
				      1000000u) + 1u;

	LOG_DBG("APOS TX deferred %u us past the gateway's beacon/CCP window",
		wait_us);
	k_sleep(K_USEC(wait_us));
}

/* Transmit immediately. Returns false if the frame did not go out.
 *
 * Every frame from here goes to the gateway, so it is first held out of the
 * span in which the gateway is not listening (gw_rx_wait()).
 *
 * Safe to poll TXFRS: uwb_slave.c enables DWT_INT_RX only, so dwt_isr() never
 * runs for TXFRS and cannot clear the bit ahead of this wait. Bounded anyway --
 * an unbounded wait here once froze the whole console. */
static bool tx_now(const uint8_t *buf, uint16_t len, struct beacon_guard *bg)
{
	gw_rx_wait(bg);

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

/* FNV-1a over a per-round salt and then the EUI, reduced to a slot. Depending
 * on every EUI byte is what makes boards from one batch -- which differ only in
 * the last byte or two -- land in unrelated slots.
 *
 * The salt is what makes the gateway's repeated SURVEY_BEGIN rounds worth
 * repeating. Hashing the EUI ALONE gives a board one fixed slot forever, so two
 * anchors that collide collide identically in every round and one of them is
 * simply never enumerated -- with 8 slots and 4 anchors that is a ~59 % chance
 * of a survey that reports a missing anchor and no operator action that fixes
 * it (the EUI is burned into eFuse). Mixing in the round counter makes each
 * round an independent draw; mixing in the session makes a re-run reshuffle too,
 * so an unlucky session is never repeatable either.
 *
 * Salt first, EUI after: FNV-1a's avalanche then carries the salt through every
 * subsequent byte, rather than letting it perturb only the final multiply. */
static uint32_t enum_slot(uint16_t sess, uint8_t round)
{
	const uint8_t salt[3] = {round, (uint8_t)sess, (uint8_t)(sess >> 8)};
	uint32_t h = 2166136261u;

	for (int i = 0; i < (int)sizeof(salt); i++) {
		h ^= salt[i];
		h *= 16777619u;
	}
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

	/* Round counter for the stagger salt. Observed BEFORE window_open()
	 * overwrites window_session: a SURVEY_BEGIN for a session we are not
	 * already in starts the count over, so round 0 of every survey is a
	 * fresh draw and the counter cannot creep across runs. */
	if (session != window_session) {
		begin_seen = 0;
	}

	uint8_t round = begin_seen;

	if (begin_seen < UINT8_MAX) {
		begin_seen++;
	}

	window_open(session, window_s);
	LOG_INF("{\"apos\":\"window open\",\"session\":%u,\"window_s\":%u,"
		"\"round\":%u}", session, window_s, round);

	/* Sleep out our stagger slot before replying. Blocking the SLAVE loop
	 * for up to 210 ms is acceptable here and nowhere else: this happens
	 * only when an operator triggers a survey, and the alternative -- a
	 * timer plus a deferred TX path -- would add a second thread touching
	 * the SPI bus. */
	k_sleep(K_MSEC(enum_slot(session, round) * APOS_ENUM_SLOT_MS));

	int n = apos_frame_enum_rsp_build(tx_buf, sizeof(tx_buf),
					 uwb_config_short_addr(cfg),
					 UWB_ADDR_GATEWAY_RESERVED, session,
					 eui, cfg->position_valid,
					 cfg->x, cfg->y, cfg->z);

	if (n < 0) {
		LOG_ERR("ENUM_RSP build failed (%d)", n);
		return;
	}

	/* A suppressed or failed TX here must not be silently lost: with the
	 * stagger spanning up to APOS_ENUM_WINDOW_MS (768 ms) against a 200 ms
	 * superframe, a board that draws one of the longer slots is landing
	 * its reply right against some beacon -- exactly
	 * where beacon_guard_tx_allowed() is designed to refuse. Without a
	 * retry that board simply never appears in this round's enumeration,
	 * indistinguishable from it never having answered at all. Same
	 * one-retry pattern as handle_range_cmd()'s RANGE_RSP below. */
	apos_frame_set_seq(tx_buf, (*seq)++);
	if (!tx_now(tx_buf, (uint16_t)n, bg)) {
		LOG_WRN("ENUM_RSP TX failed — retrying once");
		k_sleep(K_MSEC(5));
		apos_frame_set_seq(tx_buf, (*seq)++);
		if (!tx_now(tx_buf, (uint16_t)n, bg)) {
			LOG_ERR("ENUM_RSP TX failed twice — this board will be "
				"absent from session %u's enumeration", session);
		}
	}
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
	/* Every unassigned board answers from 0x00FF, so a SETPOS reaching one
	 * would move ALL of them. The gateway never sends one (apos run refuses
	 * with an unassigned board present); this is the anchor's half. */
	if (!uwb_config_is_assigned(cfg)) {
		LOG_WRN("SETPOS refused — this board has no assigned address");
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

/* B14 static survey: install this anchor's flight-time compensation for the
 * link to the CCP root, together with the survey generation and the root's
 * EUI.
 *
 * Idempotent by construction -- the same (generation, tau, parent) applied
 * twice leaves the same state -- because the gateway retries on a lost ACK and
 * cannot tell "the frame never arrived" from "it arrived and the ACK was
 * lost". Applied LIVE (ccp_slave_set_tau) so there is no reboot window in
 * which this anchor runs the new geometry's gateway against the old tau, and
 * persisted in the order tau, mode, link: a power cut part-way leaves the OLD
 * generation recorded against a tau that may already be new, which the gateway
 * reads as a mismatch and retries, whereas the reverse order would let a new
 * generation vouch for a tau that never landed. */
static void handle_settau(const uint8_t *buf, uint16_t plen,
			  uwb_config_t *cfg, uint8_t *seq,
			  struct beacon_guard *bg)
{
	uint16_t session = 0;
	uint8_t version = 0;
	uint32_t generation = 0;
	int32_t tau = 0;
	uint8_t parent[APOS_EUI_LEN];

	if (apos_frame_parse_settau(buf, plen, &session, &version, &generation,
				    &tau, parent) != 0) {
		LOG_WRN("SETTAU parse failed: plen=%u expected=%u — check the "
			"caller is passing flen - FCS_LEN",
			plen, APOS_LEN_SETTAU);
		return;
	}
	if (session != window_session) {
		LOG_WRN("SETTAU for session %u ignored — current is %u", session,
			window_session);
		return;
	}
	if (!uwb_config_is_assigned(cfg)) {
		LOG_WRN("SETTAU refused — this board has no assigned address");
		return;
	}
	window_refresh();

	uint8_t status = APOS_SETTAU_OK;

	if (version != APOS_SETTAU_VERSION) {
		status = APOS_SETTAU_BAD_VERSION;
		LOG_ERR("SETTAU version %u not supported (this firmware speaks "
			"%u) — link NOT installed", version, APOS_SETTAU_VERSION);
	} else {
		uwb_config_t *live = uwb_config_get();

		uwb_config_set_sync_tau(cfg, tau);
		uwb_config_set_sync_tau(live, tau);
		(void)uwb_config_set_sync_tau_mode(cfg, SYNC_TAU_MODE_SURVEY);
		(void)uwb_config_set_sync_tau_mode(live, SYNC_TAU_MODE_SURVEY);
		uwb_config_set_sync_link(cfg, generation, parent);
		uwb_config_set_sync_link(live, generation, parent);

		/* Live, same thread as ccp_slave_on_rx(). */
		ccp_slave_set_tau(uwb_config_sync_tau_effective(cfg));

		int rc = uwb_store_save_sync_tau();

		if (rc == 0) {
			rc = uwb_store_save_sync_tau_mode();
		}
		if (rc == 0) {
			rc = uwb_store_save_sync_link();
		}
		if (rc != 0) {
			status = APOS_SETTAU_PERSIST_ERR;
			LOG_ERR("SETTAU applied but NOT persisted (%d) — will be "
				"lost on reboot", rc);
		} else {
			LOG_INF("{\"apos_settau\":{\"tau_dtu\":%d,\"gen\":%u}}",
				(int)tau, (unsigned int)generation);
		}
	}

	/* The ACK reports the value actually held, like SETPOS_ACK does. */
	int n = apos_frame_settau_ack_build(tx_buf, sizeof(tx_buf),
					    uwb_config_short_addr(cfg),
					    UWB_ADDR_GATEWAY_RESERVED, session,
					    cfg->sync_link_gen, cfg->sync_tau_dtu,
					    status);

	if (n < 0) {
		LOG_ERR("SETTAU_ACK build failed (%d)", n);
		return;
	}
	apos_frame_set_seq(tx_buf, (*seq)++);
	tx_now(tx_buf, (uint16_t)n, bg);
}

/* Address assignment (docs/superpowers/specs/2026-10-21-anchor-eui-identity-
 * design.md D5). Addressed by EUI inside a BROADCAST frame, because the target
 * may have no address of its own yet, or share one.
 *
 * Gated like RANGE_CMD -- open window, matching session, src 0x0000 (checked by
 * the caller) -- because it ends in a reboot: nothing but the gateway that
 * opened this survey may make a deployed board restart.
 *
 * The comparison is against `cfg`, the loop's snapshot, i.e. the address this
 * board is RUNNING with, not the persisted one, so a gateway retry after an ACK
 * this board never got onto the air finds CHANGE again and can complete.
 *
 * The reboot is armed once the ACK has LEFT this radio (TXFRS), which says
 * nothing about whether the gateway received it. An ACK lost on air is followed
 * by retries that reach a rebooting board with its window closed; the gateway
 * does not count that as a failure but settles it by re-enumeration, seeing the
 * board at its new address (apos_gw.c, assign_pending()). */
static void handle_setaddr(const uint8_t *buf, uint16_t plen,
			   const uwb_config_t *cfg, uint8_t *seq,
			   struct beacon_guard *bg)
{
	uint16_t session = 0, addr = 0;
	uint8_t target[APOS_EUI_LEN];

	if (apos_frame_parse_setaddr(buf, plen, &session, target, &addr) != 0) {
		LOG_WRN("SETADDR parse failed: plen=%u expected=%u — check the "
			"caller is passing flen - FCS_LEN",
			plen, APOS_LEN_SETADDR);
		return;
	}
	if (!eui_ready) {
		return;
	}

	enum apos_setaddr_verdict v = apos_setaddr_check(
		eui, uwb_config_short_addr(cfg), target, addr,
		UWB_ANCHOR_ADDR_BASE, UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS);

	if (v == APOS_SETADDR_IGNORE) {
		return;   /* another board's: no reply, no log */
	}
	if (!apos_node_window_open() || session != window_session) {
		LOG_WRN("SETADDR refused — window closed or session mismatch");
		return;
	}
	window_refresh();

	uint8_t status = APOS_SETADDR_OK;
	uint16_t held = uwb_config_short_addr(cfg);
	bool reboot = false;

	if (v == APOS_SETADDR_V_BAD) {
		status = APOS_SETADDR_BAD_ADDR;
		LOG_ERR("SETADDR 0x%04X refused — outside 0x%04X..0x%04X", addr,
			UWB_ANCHOR_ADDR_BASE,
			UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS - 1u);
	} else if (v == APOS_SETADDR_V_CHANGE) {
		uwb_config_t *live = uwb_config_get();
		uint8_t prev = live->anchor_id;

		(void)uwb_config_set_id(live, (uint8_t)(addr - UWB_ANCHOR_ADDR_BASE));

		int rc = uwb_store_save_id();

		if (rc != 0) {
			(void)uwb_config_set_id(live, prev);
			status = APOS_SETADDR_PERSIST_ERR;
			LOG_ERR("SETADDR 0x%04X NOT persisted (%d) — keeping 0x%04X",
				addr, rc, held);
		} else {
			held = addr;
			reboot = true;
			LOG_INF("{\"apos_setaddr\":{\"from\":\"0x%04X\","
				"\"to\":\"0x%04X\"}}", uwb_config_short_addr(cfg),
				addr);
		}
	}

	/* Sent from the address this board is still running with; the gateway
	 * pairs the ACK by EUI, never by source. `held` is what is persisted. */
	int n = apos_frame_setaddr_ack_build(tx_buf, sizeof(tx_buf),
					     uwb_config_short_addr(cfg),
					     UWB_ADDR_GATEWAY_RESERVED, session,
					     eui, held, status);

	if (n < 0) {
		LOG_ERR("SETADDR_ACK build failed (%d)", n);
		return;
	}

	bool sent;

	apos_frame_set_seq(tx_buf, (*seq)++);
	sent = tx_now(tx_buf, (uint16_t)n, bg);
	if (!sent) {
		k_sleep(K_MSEC(5));
		apos_frame_set_seq(tx_buf, (*seq)++);
		sent = tx_now(tx_buf, (uint16_t)n, bg);
	}

	if (reboot && sent) {
		LOG_WRN("rebooting in %d ms to take address 0x%04X",
			SETADDR_REBOOT_DELAY_MS, held);
		k_work_reschedule(&reboot_work, K_MSEC(SETADDR_REBOOT_DELAY_MS));
	} else if (reboot) {
		/* The ACK never left: undo the persist, so the live config, the
		 * status report and NVS all keep saying what the radio runs. A
		 * later power cut must not move this board to an address the
		 * gateway never recorded; the gateway's retry redoes it. */
		uwb_config_t *live = uwb_config_get();

		(void)uwb_config_set_id(live, cfg->anchor_id);
		if (uwb_store_save_id() != 0) {
			LOG_ERR("SETADDR rollback NOT persisted — this board moves to "
				"0x%04X on its next reboot", held);
		}
		LOG_WRN("SETADDR_ACK could not be sent — staying on 0x%04X",
			uwb_config_short_addr(cfg));
	}
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
	begin_seen = 0;
	LOG_INF("{\"apos\":\"window closed\",\"reason\":\"survey end\"}");
}

/* apos_batch_stats() sizes its scratch from APOS_STATS_MAX; a larger batch cap
 * here would be silently truncated there. */
_Static_assert(APOS_MAX_EXCHANGES <= APOS_STATS_MAX,
	       "APOS_MAX_EXCHANGES exceeds apos_stats.h APOS_STATS_MAX");

/* Static, not on the stack: CONFIG_MAIN_STACK_SIZE is 4096 and this is 256 B. */
static int32_t batch[APOS_MAX_EXCHANGES];

/* The batch reduction (median-absolute-deviation filter plus population sd)
 * lives in apos_stats.c so the gateway ranging an anchor itself reduces its
 * exchanges identically. */

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
	/* Only after the session check: a stale-session or otherwise-refused
	 * RANGE_CMD must not keep the window alive, or a stream of garbage
	 * from 0x0000 defeats the window's self-closing property. See
	 * window_refresh()'s own comment. */
	window_refresh();

	/* An unassigned board shares 0x00FF with every other one: its polls and
	 * its RANGE_RSP could not be told apart. apos run refuses to start with
	 * one present; this is the anchor's half of that rule. */
	if (!uwb_config_is_assigned(cfg)) {
		LOG_WRN("RANGE_CMD refused — this board has no assigned address");
		return;
	}

	if (peer_addr == UWB_ADDR_GATEWAY_RESERVED ||
	    peer_addr == uwb_config_short_addr(cfg) ||
	    peer_addr < UWB_ANCHOR_ADDR_BASE ||
	    peer_addr >= UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS) {
		/* The gateway never answers a poll, polling ourselves is
		 * meaningless, and anything outside the valid anchor-address
		 * range would silently alias onto some other anchor's wire id
		 * once truncated to a byte below (e.g. 0x0102 -> id 1). Either
		 * means the gateway's table is wrong -- surface it rather than
		 * producing a plausible-looking wrong measurement. */
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

	/* Wall-clock deadline, not just the APOS_MAX_EXCHANGES count cap: a
	 * batch against an unreachable peer runs ~37 ms/exchange (both
	 * ss_initiator_range() timeouts plus the inter-exchange sleep), not
	 * the ~5 ms/exchange the count cap alone was sized against, so the
	 * count cap alone does not keep this inside beacon_guard's lock
	 * budget. See APOS_RANGE_BATCH_DEADLINE_MS in apos_node.h. */
	int64_t deadline = k_uptime_get() + (int64_t)APOS_RANGE_BATCH_DEADLINE_MS;

	ss_initiator_enter();
	for (uint8_t i = 0; i < want; i++) {
		if (k_uptime_get() >= deadline) {
			/* Not a failure: the exchanges gathered so far are
			 * still good data, and n_ok (kept, below) already
			 * reports the count honestly. */
			LOG_WRN("RANGE_CMD batch deadline hit — stopping at "
				"%u/%u exchanges", i, want);
			break;
		}
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
	uint8_t kept = apos_batch_stats(batch, valid, &mean, &sd);

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
	 * SLAVE loop re-arms RX when this returns.
	 *
	 * A completed batch's result must not silently vanish -- a suppressed
	 * TX (e.g. landing on the beacon right after a long batch, entirely
	 * plausible here) would otherwise look identical at the gateway to
	 * "the RANGE_CMD never arrived", which is exactly the conflation
	 * APOS_MIN_N_OK's comment says must not happen. One retry after a
	 * short sleep, then give up loudly. */
	if (!tx_now(tx_buf, (uint16_t)n, bg)) {
		LOG_WRN("RANGE_RSP TX failed — retrying once");
		k_sleep(K_MSEC(5));
		apos_frame_set_seq(tx_buf, (*seq)++);
		if (!tx_now(tx_buf, (uint16_t)n, bg)) {
			LOG_ERR("RANGE_RSP TX failed twice — batch result for "
				"peer 0x%04X lost", peer_addr);
		}
	}
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
	case APOS_SUB_SETTAU:
		handle_settau(buf, plen, cfg, seq, bg);
		break;
	case APOS_SUB_SURVEY_END:
		handle_survey_end(buf, plen);
		break;
	case APOS_SUB_SETADDR:
		handle_setaddr(buf, plen, cfg, seq, bg);
		break;
	case APOS_SUB_RANGE_CMD:
		/* window_refresh() happens inside handle_range_cmd(), after
		 * its own session check -- not here -- so a stale-session or
		 * malformed RANGE_CMD cannot extend a window it does not
		 * belong to. */
		handle_range_cmd(buf, plen, cfg, seq, bg);
		break;
	default:
		/* Every remaining subtype is anchor-to-gateway and should
		 * never arrive with src 0x0000. */
		break;
	}
	return true;
}
