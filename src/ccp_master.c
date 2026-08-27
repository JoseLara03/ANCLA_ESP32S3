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

#include <stdbool.h>
#include <string.h>

LOG_MODULE_REGISTER(ccp_master, ANCLA_LOG_LEVEL);

/* Bound for the post-dwt_starttx() TXFRS wait. Unlike the old delayed-TX
 * design, an IMMEDIATE TX's frame is on air essentially as soon as
 * dwt_starttx() returns -- there is no scheduled delay left to wait out, only
 * the CCP's own airtime (CCP_SCHED_SHR_NS + CCP_SCHED_POST_RMARKER_NS,
 * ~1.29 ms total, ccp_sched.h) plus whatever the arm sequence itself costs.
 * 8 ms clears that with ample margin and is kept unchanged from the delayed-TX
 * design's value rather than re-derived down, since this path is still newly
 * measured and a generous bound costs nothing on a K_PRIO_COOP(0) loop that
 * only spins until the bit sets or the bound elapses.
 *
 * This is a SEPARATE constant from uwb_gateway.c's TX_COMPLETE_TIMEOUT_MS (11)
 * and anchor_respond.c's (18). All three cover different worst cases and
 * CLAUDE.md already records one bug caused by conflating two of them. */
#define CCP_MASTER_TX_TIMEOUT_MS 8

/* Rate limit for the production-visible drop/offset summary below. Per-event
 * detail stays at LOG_DBG -- compiled out at ANCLA_LOG_LEVEL == LOG_LEVEL_INF
 * -- because this loop runs at K_PRIO_COOP(0) and one line per superframe
 * (5/s) would be both console noise and a timing cost on the beacon path.
 *
 * 25 superframes is ~5 s at 200 ms/superframe: short enough that a real fault
 * shows up within one bench observation, long enough that it cannot itself
 * become 5 Hz of console spam. Only reported when n_dropped has actually
 * moved since the last report, so a healthy gateway stays silent. */
#define CCP_MASTER_STATS_LOG_PERIOD_SF 25u

/* Rate limit for the CAP-overlap warning below, independent of the drop
 * summary's period: an overlapping CCP is not a drop (it was sent and TXFRS
 * confirmed it), so it would never trip the "n_dropped moved" gate above and
 * needs its own throttle to avoid becoming 5 Hz of console spam on a
 * persistently-late arm. */
#define CCP_MASTER_OVERLAP_LOG_PERIOD_SF 25u

static uint8_t tx_buf[CCP_FRAME_LEN];
static uint32_t root_id;
static uint8_t  ccp_seq;
static uint32_t n_sent;
static uint32_t n_dropped;
static uint32_t sf_since_report;
static uint32_t last_reported_dropped;
static uint32_t sf_since_overlap_report;

/* The previous CONFIRMED CCP's measured TX timestamp (40-bit DTU, masked),
 * and whether one exists yet. This is the whole state the deferred-timestamp
 * design adds: the NEXT CCP built announces this value, then this value is
 * replaced by THAT CCP's own measured timestamp once ITS transmit is
 * confirmed -- so the invariant is that prev_tx_dtu always names the most
 * recent CCP that ACTUALLY went on air, never one that merely built. Advanced
 * ONLY on a confirmed transmit (see ccp_master_after_beacon()) -- if a CCP is
 * dropped, the next one re-announces the SAME prev_tx_dtu rather than
 * skipping ahead to a timestamp for a frame that never existed. have_prev_tx
 * starts false: the first CCP after boot has no previous frame to announce
 * and sends the tx_dtu=0 sentinel instead (see ccp_master.c's build site and
 * ccp_slave.c's sentinel handling). */
static uint64_t prev_tx_dtu;
static bool have_prev_tx;

/* Where the CCP's ACTUAL RMARKER landed, relative to the BEACON's RMARKER, in
 * nanoseconds -- tracked only on a CONFIRMED transmit, since a dropped CCP has
 * no landing spot to report. This is the direct replacement for the old
 * delayed-TX "lateness against a schedule" instrument: there is no schedule
 * left, so what is worth knowing is simply where the frame actually ended up,
 * so it can be checked against ccp_sched.h's CCP_SCHED_CAP_PREAMBLE_NS. */
static int32_t offset_ns_min;
static int32_t offset_ns_max;
static int32_t offset_ns_last;
static bool have_offset_sample;

/* offset_ns_last with the CCP's own SHR and the beacon's own post-RMARKER
 * airtime subtracted -- i.e. how long the arm sequence itself took between the
 * beacon's frame ending and this CCP's preamble starting. This is the number
 * to compare against ccp_sched.h's CCP_SCHED_MAX_ARM_NS (348300 ns today). */
static int32_t arm_cost_ns_last;

/* D3: the one-shot on-demand probe ("sync txtest"). Separate state from
 * everything above so the bench probe can never be confused with, or perturb,
 * the normal per-superframe CCP counters. txtest_pending is set by the shell
 * thread (ccp_master_request_txtest()) and consumed ONLY by the gateway loop
 * (ccp_master_txtest_step()), which is what makes this safe: see
 * ccp_master.h for the full reasoning on why the radio itself must never be
 * touched from the shell thread here. */
static volatile bool txtest_pending;
static bool txtest_done;
static bool txtest_tx_ok;
static bool txtest_txfrs_ok;
static uint8_t txtest_buf[CCP_FRAME_LEN];

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
	sf_since_overlap_report = 0;

	prev_tx_dtu = 0;
	have_prev_tx = false;

	offset_ns_min = 0;
	offset_ns_max = 0;
	offset_ns_last = 0;
	have_offset_sample = false;
	arm_cost_ns_last = 0;

	txtest_pending = false;
	txtest_done = false;
	txtest_tx_ok = false;
	txtest_txfrs_ok = false;

	LOG_INF("{\"ccp_master\":{\"root_id\":%u}}", root_id);
}

void ccp_master_after_beacon(uint32_t beacon_hi32, uint8_t *frame_seq)
{
	/* The production-visible drop/offset summary. Deliberately at the TOP
	 * of the function, before this superframe's own CCP is even built: it
	 * reports on what has accumulated so far, and doing the bookkeeping
	 * first means every one of this function's several early returns
	 * below still leaves the counter advanced -- no return site needs its
	 * own copy of this. Bounded and cheap (a handful of integer ops and,
	 * rarely, one deferred LOG_WRN enqueue): safe on the K_PRIO_COOP(0)
	 * loop. */
	if (++sf_since_report >= CCP_MASTER_STATS_LOG_PERIOD_SF) {
		sf_since_report = 0;
		if (n_dropped != last_reported_dropped) {
			LOG_WRN("{\"ccp_master\":{\"sent\":%u,\"dropped\":%u,"
				"\"offset_ns_min\":%d,\"offset_ns_max\":%d,"
				"\"offset_ns_last\":%d,"
				"\"arm_cost_ns_last\":%d}}",
				n_sent, n_dropped,
				(int)offset_ns_min, (int)offset_ns_max,
				(int)offset_ns_last, (int)arm_cost_ns_last);
			last_reported_dropped = n_dropped;
		}
	}

	/* The CCP this superframe announces the PREVIOUS confirmed CCP's
	 * measured TX timestamp -- 0 as a sentinel meaning "no announcement
	 * yet" when none exists (the first CCP after boot). A genuine 40-bit
	 * timestamp landing on exactly 0 is a 2^-40 event; ccp_slave.c treats
	 * tx_dtu==0 as "not an announcement" and simply waits for the next
	 * frame, so the entire cost of that coincidence is one skipped
	 * observation -- not a wrong one. This is also, conveniently, why a
	 * "sync txtest" probe (which always sends tx_dtu=0, since an
	 * on-demand immediate TX has nothing scheduled to announce for
	 * itself) is automatically ignored by a receiver: it looks exactly
	 * like the boot sentinel, and ccp_slave.c's seq handling separately
	 * refuses to pair anything against seq 0xFF (see ccp_slave.c). */
	uint64_t announce_tx_dtu = have_prev_tx ? prev_tx_dtu : 0u;

	struct ccp_frame f = {
		.seq = ccp_seq,
		.hop = CCP_HOP_ROOT,
		.tx_dtu = announce_tx_dtu,
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

	/* dwt_forcetrxoff() (ull_forcetrxoff() in the driver) issues
	 * CMD_TXRXOFF and, per ss_initiator.c's own comment on the identical
	 * call, skips even that write when the part is already idle -- so
	 * this is cheap, not a new wait, and does not touch the K_PRIO_COOP(0)
	 * bound.
	 *
	 * Still required here even though this is now an IMMEDIATE TX, not a
	 * delayed one: this is the ONLY transmit in the whole tree armed
	 * immediately after another TX (the beacon) rather than after an RX
	 * or from a cold start, and a completed TX -- unlike a completed RX --
	 * does NOT leave the Transmit Sequencing Engine in IDLE on its own
	 * (dw3000_device.c's ull_starttx() otherwise reads SYS_STATE_LO as
	 * DW_SYS_STATE_TXERR, a hard DWT_ERROR, exactly the state the chip is
	 * left in right after the beacon's own TX completes and before
	 * CMD_TXRXOFF has cleared it). See CLAUDE.md's "The gateway's
	 * TX_COMPLETE_TIMEOUT_MS" entry and ccp_sched.h's header comment for
	 * the full history of why this call exists and why raising a
	 * delayed-TX offset could never have substituted for it. */
	dwt_forcetrxoff();

	dwt_writetxdata((uint16_t)n, tx_buf, 0);
	dwt_writetxfctrl((uint16_t)(n + FCS_LEN), 0, 0);

	/* No dwt_setdelayedtrxtime() call: this is the whole point of the
	 * redesign. An immediate TX has no scheduled RMARKER to arm against,
	 * so there is nothing for arm jitter to be late for -- the previous
	 * design's entire failure mode does not exist on this path. */
	if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		n_dropped++;
		LOG_DBG("CCP dwt_starttx() failed");
		return;
	}

	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       CCP_MASTER_TX_TIMEOUT_MS)) {
		/* NOT optional, and not merely tidy. CLAUDE.md records that
		 * dwt_starttx() can report DWT_SUCCESS for a transmission that
		 * never actually happens. Counting such a CCP as sent would
		 * leave the receiver with no gap to notice, so it would fold a
		 * two-interval span in as one -- a fabricated observation in
		 * the one estimator the whole TDoA migration turns on. Just as
		 * important under THIS design: prev_tx_dtu/have_prev_tx below
		 * must NOT be advanced on this path, or the next CCP would
		 * announce a timestamp for a frame that never transmitted. */
		dwt_forcetrxoff();
		/* TXFRS is write-1-to-clear, and this is the same third-bug
		 * class CLAUDE.md records for the beacon and the old delayed
		 * CCP: left set, TXFRS stays up for the NEXT delayed/immediate
		 * TX on this radio, whose bounded wait then returns
		 * IMMEDIATELY against this CCP's stale timestamp. */
		dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
		n_dropped++;
		LOG_DBG("CCP started but TXFRS never completed — forced off");
		return;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	n_sent++;

	/* CONFIRMED transmit only, from here down. Read the CCP's own actual
	 * TX timestamp -- its true RMARKER -- and store it for the NEXT CCP to
	 * announce. This is the read the old design tried to avoid paying for
	 * on this critical path (ccp_sched.h's history); it is unavoidable
	 * now, because it IS the measurement this whole redesign exists to
	 * take, but it happens AFTER TXFRS, off the arming critical path
	 * entirely -- it cannot make this CCP itself late for anything. */
	uint64_t tx_ts = uwb_get_tx_timestamp_u64();

	prev_tx_dtu = tx_ts & SYNC_DTU_MASK;
	have_prev_tx = true;

	/* Instrumentation: where did this CCP's RMARKER actually land, relative
	 * to the beacon's RMARKER? Both are hi32 quantities (256 DTU/tick,
	 * ~4.006 ns/tick); hi32 wraps every ~17.2 s, so the difference MUST be
	 * taken as a signed 32-bit subtraction -- (int32_t)(a - b) -- per
	 * CLAUDE.md, or the arithmetic is wrong across the wrap and looks like
	 * a radio fault instead of a timing one. */
	uint32_t ccp_hi32 = (uint32_t)(tx_ts >> 8);
	int32_t offset_ticks = (int32_t)(ccp_hi32 - beacon_hi32);
	int32_t offset_ns = (int32_t)(((int64_t)offset_ticks * 4006) / 1000);

	offset_ns_last = offset_ns;
	if (!have_offset_sample) {
		offset_ns_min = offset_ns;
		offset_ns_max = offset_ns;
		have_offset_sample = true;
	} else {
		if (offset_ns < offset_ns_min) {
			offset_ns_min = offset_ns;
		}
		if (offset_ns > offset_ns_max) {
			offset_ns_max = offset_ns;
		}
	}

	/* The arm sequence's actual cost: how long it took, after the beacon's
	 * own frame ended, to get this CCP's preamble on air. offset_ns is
	 * measured to the RMARKER, which sits at the END of the CCP's own SHR
	 * -- so the preamble started CCP_SCHED_SHR_NS earlier than that, and
	 * the beacon's frame had already ended CCP_SCHED_BEACON_END_NS after
	 * ITS OWN RMARKER (the same instant this function was first able to
	 * run, since tx_beacon() does not return before then). */
	arm_cost_ns_last = offset_ns - (int32_t)CCP_SCHED_SHR_NS -
			   (int32_t)CCP_SCHED_BEACON_END_NS;

	/* Runtime check, not a BUILD_ASSERT: the quantity is measured, so it
	 * cannot be known at compile time whether it holds. If this CCP's
	 * frame end (its RMARKER plus its own post-RMARKER airtime) reaches
	 * or passes the earliest legitimate slave CAP preamble
	 * (ccp_sched.h's CCP_SCHED_CAP_PREAMBLE_NS), this CCP overlapped real
	 * ranging traffic on this superframe. Rate-limited the same way as the
	 * drop summary above -- this loop is K_PRIO_COOP(0) and cannot afford
	 * per-superframe logging. */
	int32_t frame_end_ns = offset_ns +
				(int32_t)CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN);

	if (frame_end_ns >= (int32_t)CCP_SCHED_CAP_PREAMBLE_NS) {
		/* Rate-limited independently of the drop summary above: an
		 * overlapping CCP is not a drop (TXFRS confirmed it), so it
		 * would never trip that gate and needs its own throttle so a
		 * persistently-late arm cannot become 5 Hz of console spam on
		 * the K_PRIO_COOP(0) loop. */
		if (++sf_since_overlap_report >=
		    CCP_MASTER_OVERLAP_LOG_PERIOD_SF) {
			sf_since_overlap_report = 0;
			LOG_WRN("{\"ccp_master\":{\"cap_overlap\":1,"
				"\"frame_end_ns\":%d,"
				"\"cap_preamble_ns\":%u}}",
				(int)frame_end_ns,
				(unsigned int)CCP_SCHED_CAP_PREAMBLE_NS);
		}
	}
}

void ccp_master_stats(uint32_t *sent, uint32_t *dropped, uint32_t *root,
		      int32_t *offset_min, int32_t *offset_max,
		      int32_t *offset_last, int32_t *arm_cost_last)
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
	if (offset_min) {
		*offset_min = offset_ns_min;
	}
	if (offset_max) {
		*offset_max = offset_ns_max;
	}
	if (offset_last) {
		*offset_last = offset_ns_last;
	}
	if (arm_cost_last) {
		*arm_cost_last = arm_cost_ns_last;
	}
}

void ccp_master_request_txtest(void)
{
	/* Only a request flag is touched here -- this function runs on the
	 * shell thread and must never reach the radio itself; see
	 * ccp_master.h and the hazard analysis in uwb_gateway.c's call site
	 * for why. txtest_pending is the sole field the shell thread writes;
	 * everything else below is written only by ccp_master_txtest_step(),
	 * which runs on the gateway loop. */
	txtest_done = false;
	txtest_tx_ok = false;
	txtest_txfrs_ok = false;
	txtest_pending = true;
}

bool ccp_master_txtest_pending(void)
{
	return txtest_pending;
}

void ccp_master_txtest_step(uint8_t *frame_seq)
{
	if (!txtest_pending) {
		return;
	}
	/* Cleared FIRST: whatever happens below, this is a one-shot -- the
	 * gateway loop must not retry it next superframe on its own. */
	txtest_pending = false;

	/* seq 0xFF marks this as the bench probe, not a real per-superframe
	 * CCP, so a sniffer capture (or ccp_slave's own receive path, which
	 * counts on ccp_seq being gapless) cannot mistake one for the other.
	 * tx_dtu is 0: this probe does not participate in the deferred-
	 * timestamp chain at all (it is not built from ccp_seq's own sequence
	 * and does not update prev_tx_dtu), so it has nothing of its own to
	 * announce and nothing else's announcement to carry either. Because
	 * this is the same tx_dtu==0 sentinel a genuine first-CCP-after-boot
	 * uses, ccp_slave.c's ordinary sentinel handling ignores it without
	 * any special-casing here. */
	struct ccp_frame f = {
		.seq = 0xFFu,
		.hop = CCP_HOP_ROOT,
		.tx_dtu = 0,
		.root_id = root_id,
	};

	int n = ccp_frame_build(txtest_buf, sizeof(txtest_buf),
				UWB_ADDR_GATEWAY_RESERVED, (*frame_seq)++, &f);

	if (n < 0) {
		LOG_ERR("txtest: CCP build failed (%d)", n);
		txtest_done = true;
		return;
	}

	/* Same TXERR precaution as the real per-superframe CCP: this probe can
	 * follow a beacon (and possibly a real CCP) TX, which per the comment
	 * in ccp_master_after_beacon() above leaves the TSE in
	 * DW_SYS_STATE_TXERR until CMD_TXRXOFF clears it. */
	dwt_forcetrxoff();

	dwt_writetxdata((uint16_t)n, txtest_buf, 0);
	dwt_writetxfctrl((uint16_t)(n + FCS_LEN), 0, 0);

	if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		txtest_tx_ok = false;
		txtest_txfrs_ok = false;
		txtest_done = true;
		return;
	}
	txtest_tx_ok = true;

	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       CCP_MASTER_TX_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
		txtest_txfrs_ok = false;
		txtest_done = true;
		return;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	txtest_txfrs_ok = true;
	txtest_done = true;
}

void ccp_master_txtest_stats(bool *pending, bool *done, bool *tx_ok,
			     bool *txfrs_ok)
{
	if (pending) {
		*pending = txtest_pending;
	}
	if (done) {
		*done = txtest_done;
	}
	if (tx_ok) {
		*tx_ok = txtest_tx_ok;
	}
	if (txfrs_ok) {
		*txfrs_ok = txtest_txfrs_ok;
	}
}
