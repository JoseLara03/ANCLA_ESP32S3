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

/* How late dwt_starttx() found the radio clock, on the failure path only --
 * see the BUILD_ASSERT-adjacent comment in ccp_master_after_beacon() for how
 * this is computed. Tracked only across OBSERVED positive latenesses; a
 * non-positive reading is counted separately (n_nonpositive_late) rather than
 * folded into min/max, because a negative "lateness" means the arm was NOT
 * late and the failure has some other cause -- reporting it as e.g.
 * late_ns_min would tell an operator the arm budget is fine when the data
 * says nothing of the kind. 0 (no positive observation yet) is the printed
 * default for all three; that is indistinguishable from "measured exactly
 * zero" only in principle, since a genuine zero-ns miss is not physically
 * distinguishable from "not yet observed" anyway and both are equally
 * uninteresting for sizing the budget. */
static int32_t late_ns_min;
static int32_t late_ns_max;
static int32_t late_ns_last;
static uint32_t n_nonpositive_late;
static bool have_late_sample;

/* D2: the SIGNED lateness, tracked unconditionally (including non-positive
 * values), unlike late_ns_{min,max} above which deliberately exclude them.
 * This is the decisive number for telling HPDWARN and TXERR apart: at the
 * instant of the arm we expect to be roughly one CCP-SHR *before* the
 * scheduled RMARKER, i.e. around -1.05 ms (~ -262000 hi32 ticks,
 * late_ns ~= -1050000). A value near that confirms at_hi32 is sane and the
 * failure has some other cause (consistent with TXERR); a wildly large
 * magnitude -- anything approaching the ~8.6 s half-period this hi32
 * arithmetic is valid within -- would mean at_hi32 itself is being computed
 * or written wrongly, which would BE the cause of HPDWARN, not merely
 * coincide with it. */
static int32_t late_ns_signed_last;
static int32_t late_ns_signed_min;
static bool have_signed_late_sample;

/* D1: the raw low system-status word and whether HPDWARN was set, read ONLY
 * on the dwt_starttx() failure path, immediately after the failure. HPDWARN is
 * explicitly cleared right before dwt_starttx() (see the call site) so that
 * any HPDWARN observed here was necessarily set by THIS CMD_DTX and is not
 * stale from an earlier transmission.
 *
 * This is suggestive, not proof: ull_starttx() (the vendored driver) issues
 * CMD_TXRXOFF on BOTH of its failure branches -- HPDWARN and TXERR -- before
 * we ever get to read anything, and that command may itself clear HPDWARN.
 * So "HPDWARN reads clear" is consistent with TXERR having fired, but it does
 * not rule out a HPDWARN that the TXRXOFF already cleared before this read.
 * Read alongside late_ns_signed_last/min above, which is the stronger
 * evidence. */
static uint32_t last_sys_status_lo;
static bool last_hpdwarn_seen;

/* D3: the one-shot immediate-TX control experiment ("sync txtest"). Separate
 * state from everything above so the bench probe can never be confused with,
 * or perturb, the normal per-superframe CCP counters. txtest_pending is set
 * by the shell thread (ccp_master_request_txtest()) and consumed ONLY by the
 * gateway loop (ccp_master_txtest_step()), which is what makes this safe: see
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
	late_ns_min = 0;
	late_ns_max = 0;
	late_ns_last = 0;
	n_nonpositive_late = 0;
	have_late_sample = false;

	late_ns_signed_last = 0;
	late_ns_signed_min = 0;
	have_signed_late_sample = false;

	last_sys_status_lo = 0;
	last_hpdwarn_seen = false;

	txtest_pending = false;
	txtest_done = false;
	txtest_tx_ok = false;
	txtest_txfrs_ok = false;

	LOG_INF("{\"ccp_master\":{\"root_id\":%u,\"offset_uus\":%u}}",
		root_id, (unsigned int)CCP_OFFSET_UUS);
}

void ccp_master_after_beacon(uint32_t beacon_hi32, uint8_t *frame_seq)
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
			LOG_WRN("{\"ccp_master\":{\"sent\":%u,\"dropped\":%u,"
				"\"late_ns_min\":%d,\"late_ns_max\":%d,"
				"\"late_ns_last\":%d,"
				"\"nonpositive_late\":%u,"
				"\"late_ns_signed_last\":%d,"
				"\"late_ns_signed_min\":%d,"
				"\"sys_status_lo\":\"0x%08X\","
				"\"hpdwarn_seen\":%d}}",
				n_sent, n_dropped,
				(int)late_ns_min, (int)late_ns_max,
				(int)late_ns_last, n_nonpositive_late,
				(int)late_ns_signed_last, (int)late_ns_signed_min,
				(unsigned int)last_sys_status_lo,
				(int)last_hpdwarn_seen);
			last_reported_dropped = n_dropped;
		}
	}

	/* Scheduled entirely in hi32 -- no SPI read, no 40-bit round trip. This
	 * is Change 1 of the arm-latency fix: the previous version took
	 * tx_beacon()'s MEASURED TX timestamp (uwb_get_tx_timestamp_u64(), a
	 * ~20 us SPI read) as its base. beacon_hi32 here is the PROGRAMMED hi32
	 * the gateway already computed to arm the beacon itself
	 * (uwb_gateway.c's `next_beacon`, passed by the caller before it is
	 * re-based) -- the gateway already knows this value for free, so
	 * deriving the CCP's time from it removes that read from a budget
	 * (CCP_SCHED_ARM_BUDGET_NS, ccp_sched.h) hardware has now shown is too
	 * tight: 100% of CCPs dropped on the bench with the old computation.
	 *
	 * CCP_OFFSET_UUS * UUS_TO_DWT_TIME is a compile-time constant, and the
	 * `>> 8` of it is EXACT for today's values: CCP_OFFSET_UUS is 1500
	 * (BEACON_OCCUPANCY_UUS) and UUS_TO_DWT_TIME is 65536, so the product
	 * is 98304000 = 384000 * 256 -- a multiple of 256 with a zero low
	 * byte, so the shift drops no bits. If either constant ever changes
	 * such that the product stops being a multiple of 256, this comment's
	 * claim needs re-checking; nothing here catches that automatically.
	 *
	 * Unsigned 32-bit wraparound of the addition below is correct and
	 * intended, not a bug: the hi32 counter itself wraps every ~17.2 s
	 * (CLAUDE.md), and dwt_setdelayedtrxtime() / dwt_readsystimestamphi32()
	 * both operate in that same wrapping 32-bit space, so arithmetic that
	 * wraps identically is what stays consistent with the hardware.
	 *
	 * What this does NOT carry, deliberately: ant_delay_tx. The hardware's
	 * actual TX timestamp differs from this programmed RMARKER by that
	 * fixed antenna delay, but it is a CONSTANT bias, absorbed whole by the
	 * receiver's phase reference on the very first observation. This gate
	 * measures jitter, not absolute time-of-flight, so adding it here would
	 * not improve the measurement -- it would just move a bias sync_model
	 * already cancels. Do not "fix" this. */
	uint32_t at_hi32 = beacon_hi32 + UUS_TO_HI32(CCP_OFFSET_UUS);
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

	/* Change 3, and the actual fix for the 100%-drop bug the Change-1/2
	 * instrumentation above diagnosed. dwt_forcetrxoff() (ull_forcetrxoff()
	 * in the driver) issues CMD_TXRXOFF and, per ss_initiator.c's own
	 * comment on the identical call, skips even that write when the part is
	 * already idle -- so this is cheap, not a new wait, and does not touch
	 * the K_PRIO_COOP(0) bound.
	 *
	 * WHY it is required here specifically: this is the ONLY delayed TX in
	 * the whole tree armed immediately after another TX rather than after
	 * an RX or from a cold start. Every other delayed-TX site already calls
	 * dwt_forcetrxoff() before arming (anchor_respond.c:133, apos_gw.c:207,
	 * apos_node.c:136, ss_initiator.c:99); tx_beacon() in uwb_gateway.c is
	 * the only OTHER exception, and it gets away with it only because a
	 * completed RX -- not a TX -- always precedes it, and RX leaves the
	 * Transmit Sequencing Engine in IDLE on its own. A completed TX does
	 * not: ull_starttx() (dw3000_device.c ~line 5061) reads SYS_STATE_LO
	 * and, absent HPDWARN, treats state DW_SYS_STATE_TXERR (0xD0000,
	 * documented in dw3000_deca_vals.h:154 as "TSE is in TX but TX is in
	 * IDLE in SYS_STATE_LO register") as a hard DWT_ERROR. That is exactly
	 * the state the chip is left in right after the beacon's own TX
	 * completes and before CMD_TXRXOFF has been issued to clear it -- which
	 * is precisely the gap between tx_beacon() returning and this function
	 * arming the CCP. The result was deterministic, not intermittent:
	 * dwt_starttx() failed for 100% of CCPs on the bench (sent:0, dropped
	 * climbing by one every superframe), while the lateness instrument just
	 * below reported nonpositive_late == dropped on all 249 samples with
	 * late_ns_max:0 -- proof the arm was NOT late and the HPDWARN branch was
	 * NOT what fired, which is what pointed at TXERR instead. Do not remove
	 * this call on the theory that the timing must be at fault; the
	 * instrumentation already ruled that out once. */
	dwt_forcetrxoff();

	dwt_setdelayedtrxtime(at_hi32);
	dwt_writetxdata((uint16_t)n, tx_buf, 0);
	dwt_writetxfctrl((uint16_t)(n + FCS_LEN), 0, 0);

	/* D1: clear HPDWARN immediately before arming, write-1-to-clear like
	 * every other status bit on this part. Any HPDWARN read after this
	 * point was necessarily set by THIS dwt_starttx()'s own CMD_DTX, not
	 * left over from an earlier transmission -- without this clear, a
	 * stale HPDWARN from a previous exchange would make the reading below
	 * meaningless. */
	dwt_writesysstatuslo(DWT_INT_HPDWARN_BIT_MASK);

	if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		n_dropped++;

		/* This is Change 2: on the already-failed path only, one more
		 * register read costs nothing (the CCP is dropped either way)
		 * and tells us how far past the scheduled arm we actually
		 * were -- the number that decides how much CCP_SCHED_ARM_-
		 * BUDGET_NS needs to grow, or whether the arm cost isn't the
		 * culprit at all.
		 *
		 * This does NOT distinguish ull_starttx()'s two failure
		 * branches (HPDWARN vs. DW_SYS_STATE_TXERR, dw3000_device.c
		 * ~line 5061) from each other -- both return plain DWT_ERROR
		 * and neither dwt_starttx() nor any other function in the
		 * public deca_device_api.h vtable hands back which one fired
		 * or the SYS_STATE_LO value itself, so there is no public
		 * accessor left to add here without reaching into driver
		 * internals CLAUDE.md says not to touch. A zero lateness with
		 * a nonzero drop count (as diagnosed above, for TXERR) is
		 * currently the only way to tell them apart from outside the
		 * driver.
		 *
		 * Signed difference, mandatory, not stylistic: hi32 wraps
		 * every ~17.2 s (256 DTU/tick, ~4.006 ns/tick), and CLAUDE.md
		 * records that a plain unsigned compare of two hi32 values is
		 * wrong across that wrap and looks like a radio fault instead
		 * of a timing one. (int32_t) casts of the raw subtraction give
		 * the correct signed delta on either side of a wrap. */
		uint32_t now_hi32 = dwt_readsystimestamphi32();
		int32_t late_ticks = (int32_t)(now_hi32 - at_hi32);

		/* 1 hi32 tick = 256 DTU ~= 4.006 ns. Integer ns, good enough
		 * for a diagnostic. */
		int32_t late_ns = (int32_t)(((int64_t)late_ticks * 4006) / 1000);

		/* D1: read the raw status, HPDWARN clear or not. This is the
		 * one extra register read Change 2's own comment above says
		 * has no public way to distinguish the two ull_starttx()
		 * failure branches -- this closes that gap as far as it CAN
		 * be closed from outside the driver. See the static's own
		 * comment for why a clear reading is suggestive, not proof. */
		uint32_t status_lo = dwt_readsysstatuslo();

		last_sys_status_lo = status_lo;
		last_hpdwarn_seen = (status_lo & DWT_INT_HPDWARN_BIT_MASK) != 0u;

		/* D2: track the SIGNED value unconditionally, independent of
		 * the non-positive/positive split the pre-existing fields
		 * below use. This is the number that separates the two
		 * hypotheses -- see the static's comment for the expected
		 * magnitude (~ -1.05 ms) and what a wild outlier would mean. */
		late_ns_signed_last = late_ns;
		if (!have_signed_late_sample || late_ns < late_ns_signed_min) {
			late_ns_signed_min = late_ns;
		}
		have_signed_late_sample = true;

		if (late_ns <= 0) {
			/* NOT late by this measure -- the arm actually
			 * completed at or before the scheduled hi32, so the
			 * arm-budget theory does not explain THIS failure.
			 * Folding this into late_ns_min would tell an operator
			 * the arm budget is fine when the data says nothing of
			 * the kind (a negative min reads as "up to Nns early",
			 * not "no evidence of lateness") -- so it is counted
			 * separately instead. */
			n_nonpositive_late++;
		} else if (!have_late_sample) {
			/* First-ever positive-lateness observation: seed
			 * min/max rather than comparing against the reset
			 * default of 0, which would otherwise clamp
			 * late_ns_min at 0 forever. */
			late_ns_min = late_ns;
			late_ns_max = late_ns;
			late_ns_last = late_ns;
			have_late_sample = true;
		} else {
			if (late_ns < late_ns_min) {
				late_ns_min = late_ns;
			}
			if (late_ns > late_ns_max) {
				late_ns_max = late_ns;
			}
			late_ns_last = late_ns;
		}

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

void ccp_master_stats(uint32_t *sent, uint32_t *dropped, uint32_t *root,
		      int32_t *late_min, int32_t *late_max, int32_t *late_last)
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
	if (late_min) {
		*late_min = late_ns_min;
	}
	if (late_max) {
		*late_max = late_ns_max;
	}
	if (late_last) {
		*late_last = late_ns_last;
	}
}

void ccp_master_diag_stats(int32_t *late_signed_last, int32_t *late_signed_min,
			   uint32_t *sys_status_lo, bool *hpdwarn_seen)
{
	if (late_signed_last) {
		*late_signed_last = late_ns_signed_last;
	}
	if (late_signed_min) {
		*late_signed_min = late_ns_signed_min;
	}
	if (sys_status_lo) {
		*sys_status_lo = last_sys_status_lo;
	}
	if (hpdwarn_seen) {
		*hpdwarn_seen = last_hpdwarn_seen;
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
	 * tx_dtu is meaningless for an immediate TX -- there is no scheduled
	 * RMARKER to report -- so it is left 0 rather than invented. */
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

	/* Same TXERR precaution as the real per-superframe CCP: this probe
	 * follows a beacon (and possibly a real CCP) TX, which per the
	 * Change-3 comment above leaves the TSE in DW_SYS_STATE_TXERR until
	 * CMD_TXRXOFF clears it. */
	dwt_forcetrxoff();
	dwt_writesysstatuslo(DWT_INT_HPDWARN_BIT_MASK);

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
