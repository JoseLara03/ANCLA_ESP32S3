/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ccp_master.h"

#include "ccp_frame.h"
#include "ccp_sched.h"
#include "sync_model.h"
#include "tag_id.h"
#include "uwb_config.h"
#include "uwb_debug.h"
#include "uwb_dwtime.h"

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <string.h>

LOG_MODULE_REGISTER(ccp_master, ANCLA_LOG_LEVEL);

/* Bound for the post-dwt_starttx() TXFRS wait. The CCP is scheduled
 * CCP_OFFSET_UUS (1500 uus ~= 1.538 ms) after the beacon's RMARKER, and
 * ccp_master_after_beacon() is called once that beacon's TXFRS has already
 * been confirmed -- so by the time we arm, at most the FULL ~1.538 ms of that
 * offset can still remain before the CCP's own RMARKER (dwt_starttx() returns
 * almost immediately after arming a delayed TX; the transmission itself
 * happens only once the scheduled delay elapses, per CLAUDE.md). What is left
 * to wait out AFTER that RMARKER is not the CCP's whole-frame airtime -- an
 * earlier version of this comment said "1.29 ms of CCP airtime", which is
 * CCP_SCHED_SHR_NS + CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN)
 * (ccp_sched.h's `full`, ~1.289 ms), the SHR-through-FCS duration of the
 * whole frame -- but the SHR precedes the RMARKER and so is already inside
 * the ~1.538 ms above; counting it again double-charges it. Only
 * CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN) (PHR + payload + FCS, ~0.239 ms)
 * is genuinely still ahead once the RMARKER arrives. True worst case is
 * therefore ~1.538 ms + ~0.239 ms = ~1.78 ms, not the ~2.83 ms the old
 * (double-counted) derivation implied. The constant itself does not change:
 * 8 ms already clears ~1.78 ms with over 6 ms to spare, more slack than the
 * usual ceil(worst_ms) + 5 ms pattern gives the gateway's own
 * TX_COMPLETE_TIMEOUT_MS -- deliberately, given how little this exact path
 * has been measured (see ccp_sched.h's CCP_SCHED_ARM_BUDGET_NS).
 *
 * This is a SEPARATE constant from uwb_gateway.c's TX_COMPLETE_TIMEOUT_MS (11)
 * and anchor_respond.c's (18). All three cover different worst cases and
 * CLAUDE.md already records one bug caused by conflating two of them. */
#define CCP_MASTER_TX_TIMEOUT_MS 8

/* Rate limit for the production-visible drop summary below. Per-event detail
 * (a missed slot, a TXFRS timeout) stays at LOG_DBG -- compiled out at
 * ANCLA_LOG_LEVEL == LOG_LEVEL_INF -- because this loop runs at
 * K_PRIO_COOP(0) and one line per superframe (5/s) would be both console
 * noise and a timing cost on the beacon path. This WRN is the thing that
 * DOES survive production: if ccp_sched.h's unmeasured arm budget
 * (CCP_SCHED_ARM_BUDGET_NS) turns out to be too tight on real hardware, the
 * gateway drops every CCP silently unless something says so here.
 *
 * 25 superframes is ~5 s at 200 ms/superframe: short enough that a real fault
 * shows up within one bench observation, long enough that it cannot itself
 * become 5 Hz of console spam. Only reported when n_dropped has actually
 * moved since the last report, so a healthy gateway stays silent. */
#define CCP_MASTER_STATS_LOG_PERIOD_SF 25u

static uint8_t tx_buf[CCP_FRAME_LEN];
static uint32_t root_id;
static uint8_t  ccp_seq;
static uint32_t n_sent;
static uint32_t n_dropped;
static uint32_t sf_since_report;
static uint32_t last_reported_dropped;

void ccp_master_init(void)
{
	/* Zero-pad FIRST, mirroring apos_node.c's apos_node_init(): hwinfo may
	 * return fewer than 8 bytes on this part (the ESP32-S3 eFuse MAC read
	 * is 6, not 8), and an uninitialised tail would make root_id change
	 * between boots. The two derivations must agree -- the anchor survey
	 * already keys peers by EUI-64 the same way. */
	uint8_t eui[8];

	memset(eui, 0, sizeof(eui));

	ssize_t n = hwinfo_get_device_id(eui, sizeof(eui));

	if (n <= 0) {
		/* Not fatal for the measurement: root_id only has to be stable
		 * for this board's lifetime and distinct from other roots on
		 * the same air, and a single site has one root. Say so instead
		 * of pretending, because a receiver keys its baseline on it. */
		LOG_WRN("hwinfo_get_device_id failed (%d) — root_id falls "
			"back to a constant, which is only safe with ONE "
			"gateway on this air", (int)n);
		root_id = 0xC0FFEE01u;
	} else {
		/* FNV-1a over the whole 8-byte (zero-padded) buffer, reusing
		 * the tag's hash so this project has one derivation rather
		 * than two. Its 31-bit mask is irrelevant here -- root_id has
		 * no int32 consumer -- but harmless, and sharing the function
		 * is worth more than shaving a bit. */
		root_id = tag_id_from_eui(eui, sizeof(eui));
	}

	ccp_seq = 0;
	n_sent = 0;
	n_dropped = 0;
	sf_since_report = 0;
	last_reported_dropped = 0;

	LOG_INF("{\"ccp_master\":{\"root_id\":%u,\"offset_uus\":%u}}",
		root_id, (unsigned int)CCP_OFFSET_UUS);
}

void ccp_master_after_beacon(uint64_t beacon_tx_dtu, uint8_t *frame_seq)
{
	/* The production-visible drop summary. Deliberately at the TOP of the
	 * function, before this superframe's own CCP is even built: it reports
	 * on what has accumulated so far, and doing the bookkeeping first means
	 * every one of this function's several early returns below still
	 * leaves the counter advanced -- no return site needs its own copy of
	 * this. Bounded and cheap (a handful of integer ops and, rarely, one
	 * deferred LOG_WRN enqueue): safe on the K_PRIO_COOP(0) loop. */
	if (++sf_since_report >= CCP_MASTER_STATS_LOG_PERIOD_SF) {
		sf_since_report = 0;
		if (n_dropped != last_reported_dropped) {
			LOG_WRN("{\"ccp_master\":{\"sent\":%u,\"dropped\":%u}}",
				n_sent, n_dropped);
			last_reported_dropped = n_dropped;
		}
	}

	/* The delayed-TX register is programmed in hi32 units, so the hardware
	 * rounds the RMARKER DOWN to a 256-DTU boundary. The payload MUST carry
	 * the rounded value.
	 *
	 * This is the single most destructive mistake available in this file.
	 * Carrying the unrounded value puts up to 255 DTU -- ~4 ns -- of error
	 * into every observation, and the gate this whole phase exists to
	 * measure has a threshold of 1 ns. The measurement would fail, and it
	 * would fail in the direction that looks like a hardware verdict.
	 *
	 * What this payload does NOT carry, deliberately: ant_delay_tx. The
	 * hardware's actual TX timestamp differs from this programmed RMARKER
	 * by that fixed antenna delay, but it is a CONSTANT bias, absorbed
	 * whole by the receiver's phase reference on the very first
	 * observation. This gate measures jitter, not absolute time-of-flight,
	 * so adding it here would not improve the measurement -- it would just
	 * move a bias sync_model already cancels. Do not "fix" this. */
	uint64_t want = (beacon_tx_dtu +
			 (uint64_t)CCP_OFFSET_UUS * UUS_TO_DWT_TIME) &
			SYNC_DTU_MASK;
	uint32_t at_hi32 = (uint32_t)(want >> 8);
	uint64_t rmarker = ((uint64_t)at_hi32) << 8;

	struct ccp_frame f = {
		.seq = ccp_seq,
		.hop = CCP_HOP_ROOT,
		.tx_dtu = rmarker,
		.root_id = root_id,
	};

	/* Consumed at BUILD time, not on success, and that is deliberate. A CCP
	 * that is built and never transmitted must leave a GAP so the receiver
	 * calls sync_model_miss() for it. Incrementing only on success would
	 * hide the skipped superframe and silently corrupt the receiver's
	 * baseline -- it would fold two intervals in as one. Same convention as
	 * frame_seq_nb on the responder path; see CLAUDE.md on reading sniffer
	 * captures. */
	ccp_seq++;

	int n = ccp_frame_build(tx_buf, sizeof(tx_buf),
				UWB_ADDR_GATEWAY_RESERVED, (*frame_seq)++, &f);

	if (n < 0) {
		LOG_ERR("CCP build failed (%d)", n);
		n_dropped++;
		return;
	}

	dwt_setdelayedtrxtime(at_hi32);
	dwt_writetxdata((uint16_t)n, tx_buf, 0);
	dwt_writetxfctrl((uint16_t)(n + FCS_LEN), 0, 0);

	if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		n_dropped++;
		LOG_DBG("CCP missed its slot");
		return;
	}
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       CCP_MASTER_TX_TIMEOUT_MS)) {
		/* NOT optional, and not merely tidy. CLAUDE.md records that
		 * dwt_starttx() can report DWT_SUCCESS for a delayed TX that
		 * never happens. Counting such a CCP as sent would leave the
		 * receiver with no gap to notice, so it would fold a
		 * two-interval span in as one -- a fabricated observation in
		 * the one estimator the whole TDoA migration turns on. */
		dwt_forcetrxoff();
		/* TXFRS is write-1-to-clear, and this is the third bug on this
		 * board caused by not treating it that way -- CLAUDE.md records
		 * the other two (an unbounded TXFRS wait that froze the console,
		 * and TXFRS excluded from the enabled interrupt mask so
		 * tx_delayed()'s own poll cannot be pre-empted by an ISR clearing
		 * it first). The failure this one guards against: a delayed TX
		 * can complete for real an instant AFTER this timeout already
		 * decided to give up on it (the same race CLAUDE.md documents for
		 * dwt_starttx() reporting DWT_SUCCESS on a TX that never
		 * happens, just on the other side of the deadline). Left set,
		 * TXFRS stays up for the NEXT delayed TX on this radio -- the
		 * following superframe's beacon, in uwb_gateway.c's tx_beacon()
		 * -- whose bounded wait then returns IMMEDIATELY and reads this
		 * CCP's stale timestamp as if it were the beacon's own. That
		 * shifts beacon_tx_ts by a whole transmission, and every slave's
		 * beacon_guard then predicts against a schedule that moved with
		 * no warning anywhere: a network-wide timing fault reported as
		 * nothing. uwb_gateway.c's tx_beacon() and send_grant() clear it
		 * the same way on their own TXFRS timeouts, for the same reason. */
		dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
		n_dropped++;
		LOG_DBG("CCP started but TXFRS never completed — forced off");
		return;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	n_sent++;
}

void ccp_master_stats(uint32_t *sent, uint32_t *dropped, uint32_t *root)
{
	if (sent) {
		*sent = n_sent;
	}
	if (dropped) {
		*dropped = n_dropped;
	}
	if (root) {
		*root = root_id;
	}
}
