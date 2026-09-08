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
#include "uwb_debug.h"
#include "uwb_frame_802_15_4z.h"
#include "uwb_mac.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <string.h>

/* ANCLA_LOG_LEVEL, not LOG_LEVEL_INF: the two LOG_DBG lines below (the
 * beacon-guard suppression verdicts) are the whole point of the debug image
 * and a per-module INF cap makes them unreachable from a .conf overlay. Still
 * LOG_LEVEL_INF in production -- see src/uwb_debug.h. */
LOG_MODULE_REGISTER(anchor_respond, ANCLA_LOG_LEVEL);

/* Bound for tx_delayed()'s post-dwt_starttx() TXFRS wait. Must exceed the
 * WORST-CASE scheduled delay across both responders, not just a frame's
 * airtime -- dwt_starttx() returns almost immediately after arming a
 * delayed TX; the transmission itself doesn't happen until the full
 * scheduled delay has elapsed. Worst case today is DISCOVERY at
 * rank = DISC_MAX_RANK (3) inside its group -- not anchor id 3 specifically:
 * network-scaling-v3's grouped DISCOVERY (disc_schedule.h) staggers by
 * rank = anchor_id / n_groups, and disc_resp_delay_uus_grouped() refuses to
 * schedule anything past DISC_MAX_RANK, so the worst case a grouped response
 * can ever reach is unchanged regardless of how many anchors or groups exist:
 * DISC_BASE_UUS + DISC_MAX_RANK*DISC_SLOT_UUS = 2000 + 3*3500 =
 * 12500 uus (~12.5 ms). Re-derived after DISC_BASE_UUS dropped from 6000 to
 * 2000 on the fast (26.67 MHz) SPI bus:
 *   ceil(12500 * 1.0256 / 1000) + 5 = ceil(12.82) + 5 = 13 + 5 = 18.
 * An earlier version of this bound (10 ms) was shorter than the scheduled
 * delay for any anchor id >= 2, so it force-cancelled transmissions that
 * hadn't had a chance to fire yet, on every single attempt for those ids.
 * ANNOUNCE (anchor_respond_announce(), T_ANNOUNCE_TX_DELAY_UUS in uwb_mac.h,
 * ~2500 uus) is comfortably inside this same bound and needs no separate
 * timeout. Revisit this constant if DISC_BASE_UUS/DISC_SLOT_UUS/
 * POLL_RX_TO_RESP_TX_DLY_UUS/T_ANNOUNCE_TX_DELAY_UUS are ever tuned further. */
#define TX_COMPLETE_TIMEOUT_MS 18

/* Minimum gap between two "WAVE poll refused" lines. A tag polls at superframe
 * rate, so on a cold (unsurveyed) deployment this refusal is continuous, not
 * occasional; 10 s keeps it visible without letting it own the log pool. */
#define UNPOSITIONED_LOG_GAP_MS 10000

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

/* Why a verdict rather than void: "the anchor did not answer" has four
 * distinct causes on this path -- the frame was not for us, the beacon guard
 * refused, dwt_starttx() rejected the slot, or the transmission was armed and
 * never completed -- and from the tag's side, from a sniffer, and in the log as
 * it stood, all four look identical. Separating them is the point of the debug
 * image; the caller pairs this with the beacon-guard state to name the cause. */
enum tx_verdict {
	TX_SENT = 0,
	TX_MISSED_SLOT,  /* dwt_starttx() rejected the scheduled time */
	TX_NO_TXFRS,     /* armed, but TXFRS never set within the bound */
};

/* The fourth cause, a beacon-guard refusal, never reaches this enum: the
 * callers check the guard first and return without touching the radio, so they
 * report "suppressed" themselves.
 *
 * Unconditional and static inline, not guarded on CONFIG_ANCLA_RANGING_DEBUG:
 * Zephyr's LOG_DBG expands its arguments even where the level is compiled out
 * (Z_LOG_TO_PRINTK), so a helper hidden behind the flag fails to compile in the
 * production image. static inline is what keeps it from warning as unused
 * there. Same reason for hi32_delta_uus() below. */
static inline const char *tx_verdict_name(enum tx_verdict v)
{
	switch (v) {
	case TX_SENT:        return "sent";
	case TX_MISSED_SLOT: return "missed_slot";
	case TX_NO_TXFRS:    return "no_txfrs";
	default:             return "?";
	}
}

static enum tx_verdict tx_delayed(const uint8_t *buf, uint16_t payload_len,
				  uint32_t tx_time, int ranging)
{
	dwt_setdelayedtrxtime(tx_time);
	dwt_writetxdata(payload_len, (uint8_t *)(uintptr_t)buf, 0);
	dwt_writetxfctrl((uint16_t)(payload_len + (ranging ? 0u : FCS_LEN)), 0, ranging);

	if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("delayed TX missed its slot — response dropped");
		return TX_MISSED_SLOT;
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
		return TX_NO_TXFRS;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	return TX_SENT;
}

/* Signed hi32 tick difference expressed in UUS, for "how far was this
 * transmission from the beacon". hi32 wraps every ~17.2 s, so the subtraction
 * has to be signed before it is scaled -- see UUS_TO_HI32 in uwb_dwtime.h.
 * UUS_TO_HI32(1) is 256, hence the divisor. */
static inline int32_t hi32_delta_uus(uint32_t a, uint32_t b)
{
	return (int32_t)(a - b) / 256;
}

void anchor_respond_wave_poll(const uint8_t *buf, uint16_t len,
			      uint64_t poll_rx_ts, const uwb_config_t *cfg,
			      uint8_t *seq, struct beacon_guard *bg,
			      bool allow_unpositioned)
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
		/* Not ours -- but proof that this board is hearing the tag poll
		 * SOMEBODY. That distinction is the first fork in diagnosing a
		 * silent anchor: a board logging polls for other ids has a
		 * working receiver and a live tag, so the fault is downstream
		 * of RX. A board logging nothing at all does not. */
		LOG_DBG("{\"wave_other\":{\"polled_id\":%u,\"our_id\":%u}}",
			buf[POS_ANCHOR_ID_IDX], wire_id);
		return;
	}

	/* Refusing is strictly better than answering with (0, 0): a silent
	 * wrong coordinate is undebuggable from the tag's side, while a silent
	 * anchor shows up immediately as a missing anchor. Mirrors the
	 * gateway's existing refusal to beacon unpositioned
	 * (uwb_gateway.c:253). Deliberately placed after the frame-match
	 * checks above (the brief's supplied code placed it before them):
	 * this function is offered every received frame regardless of type,
	 * so gating this early would LOG_WRN on every beacon/DISCOVERY/APOS
	 * frame too, not just on an actual WAVE poll addressed to us. */
	if (!cfg->position_valid && !allow_unpositioned) {
		/* Rate-limited, because this is not a rare condition: it fires
		 * on EVERY addressed poll during exactly the situation it was
		 * written for -- a cold deployment where all four anchors are
		 * unpositioned and a tag is polling at superframe rate. Deferred
		 * logging formats into a shared CONFIG_LOG_BUFFER_SIZE pool, the
		 * same pool the survey's own diagnostics use, so an unbounded
		 * version of this line would flush the messages an operator
		 * actually needs. First occurrence is immediate and full;
		 * repeats are folded into one line per interval. */
		static int64_t last_ms;
		static uint32_t suppressed;
		int64_t now = k_uptime_get();

		if (last_ms == 0 || now - last_ms >= UNPOSITIONED_LOG_GAP_MS) {
			if (suppressed) {
				LOG_WRN("WAVE poll refused — no surveyed "
					"position (%u more since the last "
					"line). Run `apos run` + `apos apply` "
					"on the gateway, or set `anchor pos`.",
					suppressed);
			} else {
				LOG_WRN("WAVE poll refused — no surveyed "
					"position. Run `apos run` + `apos "
					"apply` on the gateway, or set "
					"`anchor pos`.");
			}
			last_ms = now;
			suppressed = 0;
		} else {
			suppressed++;
		}
		return;
	}

	uint32_t resp_tx_time = (uint32_t)((poll_rx_ts +
		((uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
	uint64_t resp_tx_ts =
		(((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + cfg->ant_delay_tx;

	if (bg && !beacon_guard_tx_allowed(bg, resp_tx_time)) {
		LOG_DBG("{\"wave\":{\"id\":%u,\"verdict\":\"suppressed\","
			"\"to_beacon_uus\":%d,\"misses\":%u}}",
			wire_id, hi32_delta_uus(beacon_guard_next(bg), resp_tx_time),
			beacon_guard_misses(bg));
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
	enum tx_verdict verdict =
		tx_delayed(tx_resp_msg, sizeof(tx_resp_msg), resp_tx_time, 1);

	/* After the transmission, never before: this path has a 2000 uus
	 * turnaround budget (POLL_RX_TO_RESP_TX_DLY_UUS) and a formatting call
	 * ahead of dwt_starttx() spends it. Nothing logged here is needed to
	 * build the frame, so it all waits. */
	LOG_DBG("{\"wave\":{\"id\":%u,\"verdict\":\"%s\","
		"\"to_beacon_uus\":%d,\"locked\":%u}}",
		wire_id, tx_verdict_name(verdict),
		bg ? hi32_delta_uus(beacon_guard_next(bg), resp_tx_time) : 0,
		(bg && beacon_guard_locked(bg)) ? 1u : 0u);
}

void anchor_respond_discovery(const uint8_t *buf, uint16_t len, uint64_t disc_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq,
			      int32_t cir_power, uint16_t cir_quality,
			      struct beacon_guard *bg)
{
	if (!uwb_frame_is_discovery(buf, len)) {
		return;
	}

	uint16_t tag_addr;
	uint32_t tag_tx_ts;
	uint8_t group, n_groups;

	if (uwb_frame_parse_discovery(buf, len, &tag_addr, &tag_tx_ts,
				      &group, &n_groups) != 0) {
		return;
	}

	if (!disc_group_match(cfg->anchor_id, group, n_groups)) {
		/* Not our round -- silent, not a fault. Every other anchor in
		 * a different group sees the same broadcast and says nothing
		 * either; this is not the DISCOVERY-for-a-different-id case
		 * anchor_respond_wave_poll() logs, because a group miss is the
		 * expected common case for n_groups-1 out of every n_groups
		 * broadcasts, not a diagnostic signal. */
		return;
	}

	uint32_t delay_uus;

	if (!disc_resp_delay_uus_grouped(cfg->anchor_id, n_groups, &delay_uus)) {
		/* n_groups too small for this anchor's id -- our rank would
		 * land past DISC_MAX_RANK and therefore past
		 * TX_COMPLETE_TIMEOUT_MS. Refusing is the same policy as an
		 * unpositioned anchor refusing WAVE: silent-wrong is worse
		 * than silent-absent, and a missing anchor is visible on the
		 * tag's side while a late response is not. */
		LOG_WRN("DISCOVERY refused — rank exceeds DISC_MAX_RANK "
			"(id=%u, n_groups=%u)", cfg->anchor_id, n_groups);
		return;
	}

	uint8_t out[UWB_FRAME_LEN_RESP];

	int n = uwb_frame_response_build(out, sizeof(out),
					 uwb_config_short_addr(cfg), tag_addr,
					 0u, /* tx_ts unused for anchor selection */
					 cir_power, cir_quality);
	if (n < 0) {
		LOG_WRN("response build failed (%d)", n);
		return;
	}
	uint8_t used_seq = (*seq)++;

	uwb_frame_set_seq_num(out, used_seq);

	uint32_t resp_tx_time = (uint32_t)((disc_rx_ts +
		((uint64_t)delay_uus * UUS_TO_DWT_TIME)) >> 8);

	if (bg && !beacon_guard_tx_allowed(bg, resp_tx_time)) {
		/* Logged in full here rather than deferred: there is no
		 * transmission left to delay on this branch, so the whole
		 * turnaround budget is already forfeit.
		 *
		 * to_beacon_uus is the measurement that makes this branch
		 * actionable. The stagger puts anchor id N's reply at
		 * DISC_BASE_UUS + N*DISC_SLOT_UUS after the tag's broadcast, and
		 * the forbidden window is guard+occupancy+guard = 4500 uus wide
		 * against a 3500 uus slot pitch -- so where the tag's broadcast
		 * falls in the superframe decides WHICH id gets refused, and one
		 * id being refused every round while its neighbours pass is what
		 * this line proves. seq is consumed above, before this refusal,
		 * so a gap in this board's sequence numbers on a sniffer capture
		 * lines up against these lines one-for-one. */
		LOG_DBG("{\"disc\":{\"tag\":\"0x%04X\",\"id\":%u,\"seq\":%u,"
			"\"delay_uus\":%u,\"verdict\":\"suppressed\","
			"\"to_beacon_uus\":%d,\"misses\":%u}}",
			tag_addr, cfg->anchor_id, used_seq, delay_uus,
			hi32_delta_uus(beacon_guard_next(bg), resp_tx_time),
			beacon_guard_misses(bg));
		return;
	}

	/* Module frames carry no FCS placeholder: ranging=0 adds FCS_LEN.
	 * Logged after, not before: a synchronous log call ahead of the
	 * delayed-TX setup would eat into DISC_BASE_UUS/DISC_SLOT_UUS's already
	 * tight budget on this port (see disc_schedule.h). */
	enum tx_verdict verdict = tx_delayed(out, (uint16_t)n, resp_tx_time, 0);

	/* One line per DISCOVERY, carrying every input to the decision as well
	 * as its outcome, because "this anchor did not answer" has to be
	 * resolvable from the anchor's own console without a sniffer:
	 *   - the line existing at all proves the frame was received and
	 *     accepted as a DISCOVERY (the two silent early returns above);
	 *   - id/delay_uus is this board's stagger slot, which is what a
	 *     duplicate `anchor id` across two boards collapses -- two anchors
	 *     printing the same id here answer in the same slot and collide on
	 *     air, and neither board sees anything wrong locally;
	 *   - verdict separates sent / missed_slot / no_txfrs, which are three
	 *     different faults that used to share one silence;
	 *   - to_beacon_uus and locked place the reply against the beacon the
	 *     guard is predicting, so a suppression is readable as a timing
	 *     relationship rather than a verdict to be taken on faith. */
	LOG_DBG("{\"disc\":{\"tag\":\"0x%04X\",\"id\":%u,\"seq\":%u,"
		"\"delay_uus\":%u,\"verdict\":\"%s\",\"to_beacon_uus\":%d,"
		"\"locked\":%u,\"pwr\":%d,\"q\":%u}}",
		tag_addr, cfg->anchor_id, used_seq, delay_uus,
		tx_verdict_name(verdict),
		bg ? hi32_delta_uus(beacon_guard_next(bg), resp_tx_time) : 0,
		(bg && beacon_guard_locked(bg)) ? 1u : 0u,
		cir_power, cir_quality);
}

void anchor_respond_announce(uint32_t frame_counter, const uwb_config_t *cfg,
			     uint8_t *seq, uint64_t beacon_rx_ts,
			     int32_t cir_power, uint16_t cir_quality,
			     struct beacon_guard *bg)
{
	if ((frame_counter % ANNOUNCE_CYCLE_A) != cfg->anchor_id) {
		/* Not our rotation slot -- the expected case on
		 * (ANNOUNCE_CYCLE_A - 1) out of every ANNOUNCE_CYCLE_A
		 * beacons for any given anchor. Silent, same policy as a
		 * DISCOVERY group miss: this is routine scheduling, not a
		 * diagnostic signal. */
		return;
	}

	if (!cfg->position_valid) {
		/* Unlike anchor_respond_wave_poll(), there is no
		 * allow_unpositioned relaxation here: ANNOUNCE exists to
		 * advertise a real surveyed position to tags building their
		 * anchor pool, and there is no bootstrap case (analogous to
		 * the anchor survey's own window) where advertising a
		 * placeholder coordinate is useful. An unsurveyed anchor
		 * simply never announces; it still answers DISCOVERY, so it
		 * is not invisible, just silent on this one path. */
		return;
	}

	uint32_t resp_tx_time = (uint32_t)((beacon_rx_ts +
		((uint64_t)T_ANNOUNCE_TX_DELAY_UUS * UUS_TO_DWT_TIME)) >> 8);

	if (bg && !beacon_guard_tx_allowed(bg, resp_tx_time)) {
		LOG_DBG("{\"announce\":{\"id\":%u,\"verdict\":\"suppressed\","
			"\"to_beacon_uus\":%d,\"misses\":%u}}",
			cfg->anchor_id,
			hi32_delta_uus(beacon_guard_next(bg), resp_tx_time),
			beacon_guard_misses(bg));
		return;
	}

	/* a.addr duplicates the header's src_addr (uwb_frame_announce_build()
	 * writes both) -- payload convention shared with the other module
	 * frames, not a redundancy bug.
	 *
	 * z is cfg->z as configured (by `anchor pos` or a 2D/3D `apos apply`)
	 * rather than NaN-when-unset: this codebase has no state distinguishing
	 * "no height known" from "height is 0", since position_valid gates
	 * (x, y, z) atomically and a 2D survey's z = 0 is a real floor-relative
	 * value, not a placeholder. The design doc's NaN fallback exists for a
	 * per-anchor-height distinction this project does not currently model;
	 * pos_solver's dz fallback for NaN z simply never triggers via this
	 * path until/unless that state is added. */
	struct uwb_announce a = {
		.addr = uwb_config_short_addr(cfg),
		.x = cfg->x,
		.y = cfg->y,
		.z = cfg->z,
		.cir_power = cir_power,
		.cir_quality = cir_quality,
	};
	uint8_t out[UWB_FRAME_LEN_ANNOUNCE];
	int n = uwb_frame_announce_build(out, sizeof(out),
					 uwb_config_short_addr(cfg), &a);
	if (n < 0) {
		LOG_WRN("announce build failed (%d)", n);
		return;
	}
	uwb_frame_set_seq_num(out, (*seq)++);

	enum tx_verdict verdict = tx_delayed(out, (uint16_t)n, resp_tx_time, 0);

	LOG_DBG("{\"announce\":{\"id\":%u,\"verdict\":\"%s\","
		"\"to_beacon_uus\":%d,\"locked\":%u}}",
		cfg->anchor_id, tx_verdict_name(verdict),
		bg ? hi32_delta_uus(beacon_guard_next(bg), resp_tx_time) : 0,
		(bg && beacon_guard_locked(bg)) ? 1u : 0u);
}

void anchor_respond_multipoll(const uint8_t *buf, uint16_t len,
			      uint64_t poll_rx_ts, const uwb_config_t *cfg,
			      uint8_t *seq, struct beacon_guard *bg)
{
	if (!uwb_frame_is_multipoll(buf, len)) {
		return;
	}

	struct uwb_anchor_slot slots[UWB_FRAME_MAX_ANCHORS];
	uint8_t num_slots = 0;
	uint32_t tag_tx_ts; /* the tag's own bookkeeping -- unused here */

	if (uwb_frame_parse_multipoll(buf, len, slots, &num_slots, &tag_tx_ts) != 0) {
		return;
	}

	const uint16_t our_addr = uwb_config_short_addr(cfg);
	uint16_t delay_us = 0;
	bool found = false;

	for (uint8_t i = 0; i < num_slots; i++) {
		if (slots[i].addr == our_addr) {
			delay_us = slots[i].delay_us;
			found = true;
			break;
		}
	}
	if (!found) {
		/* Not named in this poll -- the expected case whenever a tag
		 * selects fewer than UWB_MAX_ANCHORS anchors or picks a
		 * different four. Same "proves the receiver works, fault is
		 * downstream" value as wave_other in
		 * anchor_respond_wave_poll(). */
		LOG_DBG("{\"mpol_other\":{\"our_addr\":\"0x%04X\"}}", our_addr);
		return;
	}

	if (!cfg->position_valid) {
		/* Same policy as anchor_respond_wave_poll(): a wrong-but-valid-
		 * looking (x, y, z) is undebuggable from the tag's side, so an
		 * unsurveyed anchor must stay silent rather than answer with a
		 * placeholder. Rate-limited for the identical reason -- a cold
		 * deployment makes this the common case, not the rare one. */
		static int64_t last_ms;
		static uint32_t suppressed;
		int64_t now = k_uptime_get();

		if (last_ms == 0 || now - last_ms >= UNPOSITIONED_LOG_GAP_MS) {
			LOG_WRN("MULTI-POLL refused — no surveyed position"
				"%s. Run `apos run` + `apos apply` on the "
				"gateway, or set `anchor pos`.",
				suppressed ? " (repeats suppressed)" : "");
			last_ms = now;
			suppressed = 0;
		} else {
			suppressed++;
		}
		return;
	}

	uint16_t tag_addr = uwb_frame_get_src_addr(buf);
	uint32_t resp_tx_time = (uint32_t)((poll_rx_ts +
		((uint64_t)delay_us * UUS_TO_DWT_TIME)) >> 8);

	if (bg && !beacon_guard_tx_allowed(bg, resp_tx_time)) {
		LOG_DBG("{\"mpol\":{\"tag\":\"0x%04X\",\"delay_uus\":%u,"
			"\"verdict\":\"suppressed\",\"to_beacon_uus\":%d,"
			"\"misses\":%u}}",
			tag_addr, delay_us,
			hi32_delta_uus(beacon_guard_next(bg), resp_tx_time),
			beacon_guard_misses(bg));
		return;
	}

	/* Same delayed-TX timestamp identity the legacy WAVE responder uses
	 * (tx_resp_msg's resp_tx_ts, POS_RESP_TX_TS_IDX above): the DW3000's
	 * actual TX instant for a delayed transmission is deterministically
	 * (scheduled_time & ~1) << 8 + antenna delay, so this can be computed
	 * before the transmission happens rather than read back after. Written
	 * as the frame module's own uint32_t (low 32 bits, matching
	 * uwb_resp_msg_set_ts()'s byte-for-byte convention), NOT the hi32
	 * (>>8) view resp_tx_time uses for scheduling -- the tag's ToF formula
	 * (uwb_ss_initiator.c) expects the same low-32 view for both
	 * poll_rx_ts and resp_tx_ts, matching the legacy WAVE path exactly. */
	uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) +
			      cfg->ant_delay_tx;
	uint8_t wire_id = (uint8_t)(our_addr & 0xFFu);
	uint8_t out[UWB_FRAME_LEN_MPOL_RESP];

	/* z is cfg->z as configured, not NaN-when-unset -- same reasoning as
	 * anchor_respond_announce() above: this codebase has no state
	 * distinguishing "no height known" from "height is 0". */
	int n = uwb_frame_mpol_resp_build(out, sizeof(out), our_addr, tag_addr,
					  wire_id, (uint32_t)poll_rx_ts,
					  (uint32_t)resp_tx_ts,
					  cfg->x, cfg->y, cfg->z);
	if (n < 0) {
		LOG_WRN("mpol_resp build failed (%d)", n);
		return;
	}
	uwb_frame_set_seq_num(out, (*seq)++);

	/* TX_COMPLETE_TIMEOUT_MS (18 ms, above) already covers this path's
	 * worst case without adjustment: the largest delay_us a tag can
	 * assign is MPOL_BASE_UUS + (UWB_FRAME_MAX_ANCHORS-1)*MPOL_SLOT_UUS =
	 * 2200 + 3*1700 = 7300 uus (tag_testting/src/uwb_ss_initiator.c),
	 * plus one MPOL_RESP's airtime (31 B, ~1.35 ms) -- under 9 ms total,
	 * comfortably inside the 12.5 ms DISCOVERY worst case this bound was
	 * already sized for. Do not confuse MPOL_BASE_UUS/MPOL_SLOT_UUS with
	 * DISC_BASE_UUS/DISC_SLOT_UUS above -- unrelated schedules that only
	 * happen to share one timeout. */
	enum tx_verdict verdict = tx_delayed(out, (uint16_t)n, resp_tx_time, 0);

	LOG_DBG("{\"mpol\":{\"tag\":\"0x%04X\",\"delay_uus\":%u,"
		"\"verdict\":\"%s\",\"to_beacon_uus\":%d,\"locked\":%u}}",
		tag_addr, delay_us, tx_verdict_name(verdict),
		bg ? hi32_delta_uus(beacon_guard_next(bg), resp_tx_time) : 0,
		(bg && beacon_guard_locked(bg)) ? 1u : 0u);
}
