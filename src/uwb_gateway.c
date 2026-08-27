/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GATEWAY mode: the TDMA beacon and the CAP seat protocol.
 *
 * MAC-only -- this node does NOT answer ranging polls, unlike the nRF5
 * gateway's dispatch() fall-through. That costs a board rather than an anchor
 * (the deployment is one gateway plus four slaves) and buys a much smaller
 * beacon arm margin: with no anchor_respond in the loop the worst-case service
 * latency is one GRANT, not a 16.5 ms discovery stagger.
 *
 * Interrupt-driven with the same callback shape as uwb_slave.c. The DW3000
 * system clock is authoritative over the beacon cadence -- the beacon IS the
 * network's time base, so it is scheduled against the radio's own clock and
 * never against the kernel's.
 */

#include "uwb_modes.h"

#include "apos_frame.h"
#include "apos_gw.h"
#include "ccp_master.h"
#include "gw_core.h"
#include "pos_sink.h"
#include "tag_id.h"
#include "uwb_debug.h"
#include "uwb_dwtime.h"
#include "uwb_frame_802_15_4z.h"
#include "uwb_mac.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
/* log_panic() lives here, not in log.h -- it is log CONTROL, not log output. */
#include <zephyr/logging/log_ctrl.h>

#include <deca_device_api.h>

LOG_MODULE_REGISTER(uwb_gateway, ANCLA_LOG_LEVEL);

/* T_SUPERFRAME_UUS comes from uwb_mac.h — the slaves predict the beacon from
 * the same definition, and a local copy here would drift against theirs. */

/* Delay from a received CAP frame to our GRANT TX. Same budget class as the
 * responders' turnaround; see disc_schedule.h for why this port needs more
 * than the nRF5 anchor's 2000. */
#define RX_TO_TX_DLY_UUS 2000u

/* Stop servicing and arm the beacon when this close to it. MAC-only keeps this
 * small: worst-case service latency is one GRANT (RX_TO_TX_DLY_UUS + ~1.3 ms
 * airtime + the bounded TXFRS wait), not the nRF5 gateway's 8000, which had to
 * cover a discovery stagger this node no longer performs. */
#define BEACON_ARM_MARGIN_UUS 5000u

/* Bound for the post-dwt_starttx() TXFRS wait, shared by both tx_beacon()'s
 * and send_grant()'s delayed TX. Covers the scheduled delay itself plus
 * airtime -- dwt_starttx() returns when the TX is armed, not when it fires.
 *
 * The two delayed-TX call sites have different worst-case scheduled delays,
 * and this bound must cover the LARGER one:
 *   - send_grant(): RX_TO_TX_DLY_UUS (~2.05 ms).
 *   - tx_beacon(true, ...): up to BEACON_ARM_MARGIN_UUS (~5.13 ms) -- the
 *     main loop only breaks out of RX-servicing and calls tx_beacon() once
 *     `to_beacon <= BEACON_ARM_MARGIN_UUS`, so the delayed beacon can be
 *     armed with nearly the full margin still to run before it fires. A
 *     bound sized only from RX_TO_TX_DLY_UUS (an earlier version of this
 *     comment did exactly that, and set this to 5 ms) times out on almost
 *     every delayed beacon -- confirmed on hardware: the first, immediate,
 *     beacon transmits fine, and every subsequent delayed one logs "beacon
 *     started but TXFRS never completed" and gets forced off.
 *
 * BEACON_ARM_MARGIN_UUS (~5.13 ms) + ~1.5 ms beacon airtime is the true
 * worst case, ~6.6 ms. Same derivation style as the slave's
 * TX_COMPLETE_TIMEOUT_MS in anchor_respond.c: ceil(worst_uus * 1.0256/1000)
 * + 5 ms margin = ceil(5.13) + 5 = 11. There is no upper ceiling this needs
 * to clear -- BEACON_ARM_MARGIN_UUS is the lower bound this must exceed, not
 * an upper one it must stay under; the two constants aren't otherwise
 * coupled. Re-derive if RX_TO_TX_DLY_UUS or BEACON_ARM_MARGIN_UUS changes
 * such that either's worst case grows past what 11 ms covers. */
#define TX_COMPLETE_TIMEOUT_MS 11

/* Longest frame the contract defines is a 39-byte beacon; +FCS, rounded up. */
#define RX_BUF_LEN 64

/* How long the inner RX-servicing loop may run WITHOUT reaching the beacon
 * before it declares itself stalled.
 *
 * This exists because a loop that stops breaking out is the one failure this
 * gateway cannot notice or report, and its silence is total. Two things follow
 * from the priority layout: this loop is K_PRIO_COOP(0) = -16 and dwt_isr()
 * runs on the system workqueue at -1, while the shell and the log-processing
 * thread are both PREEMPTIBLE (the log thread at
 * K_LOWEST_APPLICATION_THREAD_PRIO). If those two stop going idle, nothing
 * preemptible is ever scheduled: the console dies, the log thread never drains,
 * and CONFIG_LOG_MODE_OVERFLOW then overwrites the queued records. The board
 * stops beaconing and says nothing -- no warning, no fatal dump. Observed on
 * the bench during an `apos run` ranging phase.
 *
 * Deliberately a WALL-CLOCK bound on the whole loop rather than a check that
 * the radio clock advanced. A frozen radio clock was the first hypothesis --
 * the beacon is scheduled from dwt_readsystimestamphi32(), so a stopped clock
 * traps the loop by construction -- and the heartbeat below REFUTED it: over
 * the runs captured, systime advanced dead-regularly (210.7 / 210.5 / 210.3 /
 * 211.5 / 210.8 ms). A clock-only check would therefore have reported nothing
 * for the fault actually seen. k_uptime_get() is driven by the SoC timer, which
 * no DW3220 state can stop, so it bounds the loop whatever the cause: frozen
 * radio clock, an IRQ storm the loop cannot drain, or arithmetic that never
 * satisfies the break test. The report prints both clocks so the next capture
 * says WHICH.
 *
 * 300 ms is 1.5 superframes: past any legitimate iteration (the loop must reach
 * the beacon every 200 ms by construction) and short enough to be caught in the
 * superframe it happened in.
 *
 * CLAUDE.md records this chip wedging before, diagnosed the long way. That
 * instance left the shell ALIVE -- an idle network gave the loop nothing to
 * service, so it sat in k_sem_take and yielded. Under the survey's sustained
 * ranging traffic it does not, which is why a similar fault now presents as a
 * total freeze instead of a silent radio with a live console. */
#define LOOP_STALL_MS 300u

/* ---- Breadcrumb: where the loop is, readable from outside the loop -------
 *
 * The wall-clock watchdog above only runs where it sits, at the top of the
 * inner loop. Anything that blocks INSIDE the loop body -- an SPI transfer that
 * never completes, a callee that spins -- is invisible to it, because the check
 * is never reached again. That is not a hypothetical gap: on the bench
 * 2026-08-26 the gateway stopped mid-survey with the last pair unreported and
 * NO gw_fault line, which is exactly what a body-blocking stall looks like.
 * (The solver was the first suspect and is exonerated: replaying that run's
 * exact edges through apos_geom_solve() on the host returns rc=0 instantly.)
 *
 * So the loop drops a crumb at every step and a k_timer reads it. The timer
 * expiry runs in ISR context, which is the only context that can observe this
 * thread while it is stuck inside a call -- an interrupt preempts a
 * K_PRIO_COOP(0) busy-wait, a lower-priority thread never will.
 *
 * Numbering is deliberately stable and gapless so a capture can be read
 * straight off the console without the source at hand. */
enum gw_crumb {
	GW_CRUMB_BOOT = 0,
	GW_CRUMB_LOOP_TOP,    /* 1: top of the inner loop, clocks just read */
	GW_CRUMB_APOS_STEP,   /* 2: inside apos_gw_step() -- may transmit */
	GW_CRUMB_RX_ARM,      /* 3: dwt_setrxtimeout + dwt_rxenable */
	GW_CRUMB_RX_WAIT,     /* 4: blocked on rx_sem (bounded, 400 ms) */
	GW_CRUMB_RX_READ,     /* 5: dwt_readrxdata + RX timestamp */
	GW_CRUMB_DISPATCH,    /* 6: inside dispatch() -- may send a GRANT */
	GW_CRUMB_SF_TICK,     /* 7: gw_core_superframe_tick() */
	GW_CRUMB_TX_BEACON,   /* 8: inside tx_beacon() -- delayed TX + TXFRS */
	GW_CRUMB_CCP_TX,      /* 9: inside ccp_master_after_beacon() */
};

static volatile uint32_t gw_crumb;
static volatile uint32_t gw_crumb_seq;

#define CRUMB(c) do { gw_crumb = (uint32_t)(c); gw_crumb_seq++; } while (0)

/* Long enough that a legitimate 400 ms rx_sem wait plus slack cannot trip it. */
#define CRUMB_STALL_MS 900u

static void gw_stall_expiry(struct k_timer *t)
{
	static uint32_t last_seq;
	static bool reported;

	ARG_UNUSED(t);

	if (gw_crumb_seq != last_seq) {
		last_seq = gw_crumb_seq;
		reported = false;
		return;
	}
	if (reported) {
		return;
	}
	reported = true;

	/* log_panic() switches the log subsystem to synchronous and flushes
	 * what is queued. It is the ONLY way this line reaches the console once
	 * the loop has starved the preemptible log thread -- and it is exactly
	 * what Zephyr's own fatal handler does for the same reason. Timing is
	 * wrecked afterwards, which costs nothing: the board is already stuck,
	 * and this fires once per stall, not twice a second forever. */
	log_panic();
	LOG_ERR("{\"gw_stuck\":{\"crumb\":%u,\"for_ms\":%u,\"seq\":%u,"
		"\"apos_busy\":%d,\"note\":\"1=loop_top 2=apos_step "
		"3=rx_arm 4=rx_wait 5=rx_read 6=dispatch 7=sf_tick "
		"8=tx_beacon 9=ccp_tx\"}}",
		gw_crumb, CRUMB_STALL_MS, gw_crumb_seq,
		(int)apos_gw_busy());
}

static K_TIMER_DEFINE(gw_stall_timer, gw_stall_expiry, NULL);

static K_SEM_DEFINE(rx_sem, 0, 1);

/* Same rx_pending guard as uwb_slave.c: br101's IRQ-drain loop can call
 * dwt_isr() again before the main thread has consumed the previous event, and
 * the limit-1 semaphore would silently swallow the second give while the
 * second callback corrupted the still-unread values. */
static volatile uint32_t rx_status;
static volatile uint16_t rx_len;
static volatile bool rx_pending;

static uint8_t rx_buf[RX_BUF_LEN];
static uint8_t beacon_buf[UWB_FRAME_MAX_LEN];
static uint8_t gw_seq;

static void cb_rx_ok(const dwt_cb_data_t *cb_data)
{
	if (rx_pending) {
		return;
	}
	rx_status = cb_data->status;
	rx_len = cb_data->datalength;
	rx_pending = true;
	k_sem_give(&rx_sem);
}

static void cb_rx_fail(const dwt_cb_data_t *cb_data)
{
	if (rx_pending) {
		return;
	}
	rx_status = cb_data->status;
	rx_len = 0;
	rx_pending = true;
	k_sem_give(&rx_sem);
}

/* Transmit the beacon. Returns its 40-bit TX timestamp, or 0 if it did not go
 * out. delayed=0 for the very first beacon, which has no predecessor to
 * schedule against. */
static uint64_t tx_beacon(struct gw_core_ctx *ctx, bool delayed, uint32_t tx_at)
{
	uint16_t slot_map[GW_N_CFP];

	gw_core_build_slotmap(ctx, slot_map);

	/* GW_N_CFP == UWB_FRAME_N_CFP == 11, so need = 15 + 22 = 37 =
	 * UWB_FRAME_MAX_LEN and beacon_buf is exactly large enough.
	 * uwb_frame_beacon_build() does not bound n_slots itself -- a known
	 * defect in the frame module, left unfixed because that file is kept
	 * byte-identical to the tag's. It cannot fire here: the argument is a
	 * compile-time constant equal to the maximum. */
	int n = uwb_frame_beacon_build(beacon_buf, sizeof(beacon_buf),
				       ctx->frame_counter, slot_map, GW_N_CFP);
	if (n < 0) {
		LOG_ERR("beacon build failed (%d)", n);
		return 0;
	}
	uwb_frame_set_seq_num(beacon_buf, gw_seq++);

	if (delayed) {
		dwt_setdelayedtrxtime(tx_at);
	}
	dwt_writetxdata((uint16_t)n, beacon_buf, 0);
	dwt_writetxfctrl((uint16_t)(n + FCS_LEN), 0, 0);

	if (dwt_starttx(delayed ? DWT_START_TX_DELAYED : DWT_START_TX_IMMEDIATE)
	    != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("beacon missed its slot — re-basing cadence");
		return 0;
	}
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       TX_COMPLETE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		LOG_WRN("beacon started but TXFRS never completed — forced off");
		return 0;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

	return uwb_get_tx_timestamp_u64();
}

static void send_grant(const uint8_t eui[UWB_FRAME_EUI_LEN],
		       const struct gw_grant *g, uint64_t rx_ts)
{
	uint8_t buf[UWB_FRAME_LEN_GRANT];

	/* eui is non-NULL by construction: it comes from a successfully parsed
	 * JOIN. uwb_frame_grant_build() writes its header before checking the
	 * pointer -- another known frame-module defect left unfixed for
	 * byte-identity with the tag -- which cannot fire on this path. */
	int n = uwb_frame_grant_build(buf, sizeof(buf), eui, g->short_addr,
				      g->seat_id, g->tier, g->lease);
	if (n < 0) {
		LOG_WRN("grant build failed (%d)", n);
		return;
	}
	uwb_frame_set_seq_num(buf, gw_seq++);

	uint32_t tx_at = (uint32_t)((rx_ts +
		((uint64_t)RX_TO_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);

	dwt_setdelayedtrxtime(tx_at);
	dwt_writetxdata((uint16_t)n, buf, 0);
	dwt_writetxfctrl((uint16_t)(n + FCS_LEN), 0, 0);

	if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("grant missed its slot — tag will retry via CAP");
		return;
	}
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       TX_COMPLETE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		LOG_WRN("grant started but TXFRS never completed — forced off");
		return;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
}

static void dispatch(struct gw_core_ctx *ctx, const uint8_t *buf, uint16_t len,
		     uint64_t rx_ts)
{
	if (apos_frame_is_apos(buf, len)) {
		apos_gw_on_rx(buf, len);
	} else if (uwb_frame_is_join(buf, len)) {
		uint8_t eui[UWB_FRAME_EUI_LEN];
		uint8_t req_tier = 0;
		struct gw_grant g;

		if (uwb_frame_parse_join(buf, len, eui, &req_tier) != 0) {
			return;
		}
		if (!gw_core_join(ctx, eui, req_tier, &g)) {
			/* Two distinct causes, and the numbers separate them:
			 * no free seat of GW_MAX_SEATS, or no slot-superframes
			 * left of GW_SCHED_CAPACITY even at IDLE. */
			LOG_WRN("JOIN refused — seats %u/%u, airtime %u/%u",
				gw_core_seats_used(ctx),
				(unsigned int)GW_MAX_SEATS,
				gw_core_cost_used(ctx),
				(unsigned int)GW_SCHED_CAPACITY);
			return;
		}
		/* `tier` is what was GRANTED, which may be below req_tier when
		 * the cell is busy -- log both, so a degraded grant is visible
		 * rather than looking like the tag asked for the slower rate. */
		LOG_INF("GRANT addr=0x%04X seat=%u tier=%u(req %u) lease=%u "
			"[seats %u/%u, airtime %u/%u]",
			g.short_addr, g.seat_id, g.tier, req_tier, g.lease,
			gw_core_seats_used(ctx), (unsigned int)GW_MAX_SEATS,
			gw_core_cost_used(ctx),
			(unsigned int)GW_SCHED_CAPACITY);
		send_grant(eui, &g, rx_ts);
	} else if (uwb_frame_is_keepalive(buf, len)) {
		uint16_t sa = 0;
		uint8_t rt = 0, si = 0;

		if (uwb_frame_parse_keepalive(buf, len, &sa, &rt, &si) == 0) {
			/* NULL: the tag learns its granted tier from the
			 * cadence of the beacon slot map, not from a
			 * KEEPALIVE reply -- the contract has no such frame. */
			gw_core_keepalive(ctx, sa, rt, NULL);
		}
	} else if (uwb_frame_is_release(buf, len)) {
		uint16_t sa = uwb_frame_get_src_addr(buf);

		LOG_INF("RELEASE addr=0x%04X", sa);
		gw_core_release(ctx, sa);
	} else if (uwb_frame_is_pos(buf, len)) {
		struct pos_fix fix;
		uint8_t eui[UWB_FRAME_EUI_LEN];

		/* Deliberately not gated on gw_core seat state: a fix from a tag
		 * whose lease just expired is still a real measurement, and
		 * silently dropping it would be close to undebuggable from the
		 * broker's side. */
		uwb_frame_parse_pos(buf, len, &fix.src_addr, &fix.x, &fix.y,
				    &fix.residual_m, &fix.n_anchors,
				    &fix.batt_soc);

		/* Tid must be the tag's stable EUI-derived id, not its
		 * reallocatable short address (see pos_json.h). The seat table
		 * is the only place that EUI lives -- look it up by the
		 * address this frame just arrived from. A miss here means the
		 * sender's lease expired between its last KEEPALIVE and this
		 * POS frame (the "not gated on seat state" comment above): the
		 * fix is still real and must still be published, just without
		 * the stability guarantee for this one straggler. Falling back
		 * to fix.src_addr reproduces this path's old (pre-tag_id)
		 * per-frame VALUE exactly, but note the straggler's Tid
		 * (src_addr) will differ from every other fix this same tag
		 * has ever sent (hash(EUI)), so the platform sees a one-record
		 * phantom device for that single frame -- a narrow, accepted
		 * cost, not a dropped fix. */
		if (gw_core_find_eui(ctx, fix.src_addr, eui)) {
			fix.tag_id = tag_id_from_eui(eui, UWB_FRAME_EUI_LEN);
		} else {
			LOG_WRN("POS from 0x%04X: no live seat, Tid falls back to short address",
				fix.src_addr);
			fix.tag_id = fix.src_addr;
		}
		pos_sink_publish(&fix);
	}
	/* Anything else is tag<->anchor ranging traffic. MAC-only: not ours,
	 * and logging every frame on a busy network would flood the console. */
}

void uwb_gateway_run(const uwb_config_t *cfg)
{
	/* Snapshot at entry — see uwb_slave.c for why. It matters more here:
	 * this loop runs indefinitely and drives every other node's timing. */
	uwb_config_t cfg_snapshot = *cfg;

	cfg = &cfg_snapshot;

	if (!cfg->position_valid) {
		LOG_ERR("{\"error\":\"gateway not positioned\"} — "
			"set `anchor pos <x> <y> <z>` and reboot");
		return;
	}

	static dwt_callbacks_s cbs;

	cbs.cbRxOk = cb_rx_ok;
	cbs.cbRxTo = cb_rx_fail;
	cbs.cbRxErr = cb_rx_fail;
	dwt_setcallbacks(&cbs);

	/* RX events only. TXFRS must stay masked: tx_beacon() and send_grant()
	 * poll for it, and an ISR that cleared it first would make both wait
	 * out their full timeout on every single transmission. */
	dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT);

	/* STATIC, not automatic, and this is not a style choice.
	 *
	 * sizeof(struct gw_core_ctx) is 2588 bytes against a
	 * CONFIG_MAIN_STACK_SIZE of 4096. As an automatic it claimed 63% of this
	 * thread's entire stack before a single callee got a frame, and
	 * do_solve() -- which adds 344 bytes of its own locals and sits at the
	 * bottom of the deepest call chain in the firmware
	 * (uwb_gateway_run -> apos_gw_step -> step_range -> do_solve ->
	 * apos_geom_solve -> apos_geom_refine -> cost) -- runs from exactly this
	 * stack. Xtensa's windowed ABI spills register windows on deep chains on
	 * top of every declared frame, so the real high-water mark is higher
	 * than the sum of those frames, and neither CONFIG_STACK_SENTINEL nor
	 * hardware stack protection is enabled in the production image: an
	 * overflow here is silent and corrupts whatever lies below.
	 *
	 * It was ~236 bytes when the anchor survey was written and reviewed --
	 * seats[] was indexed by CFP slot, so GW_N_CFP (11) of them. The
	 * seat/schedule split for 100-tag capacity made it seats[GW_MAX_SEATS]
	 * (128), growing this one automatic by ~2352 bytes. Nothing flagged it:
	 * gw_core is host-tested where the stack is megabytes, the survey had
	 * never been run on hardware, and the two changes were on different
	 * branches.
	 *
	 * Static is correct rather than merely bigger-is-safer:
	 * uwb_gateway_run() is called once from main() and never returns, so
	 * there is exactly one instance for the life of the process either way.
	 * Nothing else may take this address -- see apos_gw_result()'s note on
	 * why unsynchronised reads of gateway-loop state are safe only from
	 * strictly lower-priority threads. */
	static struct gw_core_ctx ctx;

	gw_core_init(&ctx);
	ccp_master_init();
	apos_gw_init();

	LOG_INF("{\"status\":\"gateway\",\"x\":%.2f,\"y\":%.2f,"
		"\"superframe_ms\":200,\"slots\":%u,\"seats\":%u,"
		"\"airtime_budget\":%u}",
		(double)cfg->x, (double)cfg->y, GW_N_CFP,
		(unsigned int)GW_MAX_SEATS, (unsigned int)GW_SCHED_CAPACITY);

	uint64_t beacon_tx_ts = tx_beacon(&ctx, false, 0);

	if (beacon_tx_ts == 0) {
		LOG_ERR("first beacon failed to transmit — cannot start");
		return;
	}

	/* Armed only now: before this point the radio bring-up legitimately
	 * takes longer than CRUMB_STALL_MS and would report a stall that is
	 * really just a slow boot. */
	k_timer_start(&gw_stall_timer, K_MSEC(CRUMB_STALL_MS),
		      K_MSEC(CRUMB_STALL_MS));

	/* Liveness of the OUTER loop, one line per superframe. If these stop
	 * while the board is still powered, the loop is trapped inside the inner
	 * for(;;), and the stall watchdog below reports which clock is at fault.
	 * Debug image only -- five lines a second is not a production log, and
	 * under CONFIG_LOG_MODE_DEFERRED it costs only the enqueue. */
	while (1) {
		uint32_t next_beacon = (uint32_t)((beacon_tx_ts +
			((uint64_t)T_SUPERFRAME_UUS * UUS_TO_DWT_TIME)) >> 8);

		LOG_DBG("{\"gw_sf\":{\"systime\":%u,\"next_beacon\":%u,"
			"\"fc\":%u,\"apos_busy\":%d}}",
			dwt_readsystimestamphi32(), next_beacon,
			ctx.frame_counter, (int)apos_gw_busy());

		int64_t loop_entry_ms = k_uptime_get();
		uint32_t loop_entry_systime = dwt_readsystimestamphi32();

		for (;;) {
			CRUMB(GW_CRUMB_LOOP_TOP);

			uint32_t now = dwt_readsystimestamphi32();
			int32_t to_beacon = (int32_t)(next_beacon - now);

			/* Stall watchdog, BEFORE the break test it exists to
			 * catch failing. */
			if (k_uptime_get() - loop_entry_ms >
			    (int64_t)LOOP_STALL_MS) {
				LOG_ERR("{\"gw_fault\":{\"why\":\"inner "
					"loop has not reached the beacon in "
					"%u ms\",\"systime\":%u,"
					"\"systime_at_entry\":%u,"
					"\"radio_clock_moved\":%d,"
					"\"next_beacon\":%u,"
					"\"to_beacon\":%d,"
					"\"apos_busy\":%d}}",
					LOOP_STALL_MS, now, loop_entry_systime,
					(int)(now != loop_entry_systime),
					next_beacon, (int)to_beacon,
					(int)apos_gw_busy());
				/* Sleep, not k_yield(): yielding only reaches
				 * threads at this priority or above, and every
				 * thread that could print the line just
				 * enqueued -- or accept a console command to
				 * ask about it -- is BELOW this one. This
				 * millisecond is the whole reason DEFERRED
				 * logging can report a scheduler-starving
				 * fault at all, and it is why debug.conf does
				 * NOT need immediate mode. */
				loop_entry_ms = k_uptime_get();
				loop_entry_systime = now;
				k_msleep(1);
				continue;
			}

			if (to_beacon <= (int32_t)UUS_TO_HI32(BEACON_ARM_MARGIN_UUS)) {
				break;
			}

			/* Advance any running survey by at most one frame.
			 *
			 * Deliberately at the TOP of the loop body, not after
			 * dispatch(): the two `continue`s below (no RX event
			 * within the window, and a runt frame) would otherwise
			 * skip the step entirely, and an RX timeout is the
			 * NORMAL outcome on a quiet network. Since the first
			 * SURVEY_BEGIN must go out before any anchor has
			 * anything to reply to, a step reachable only after a
			 * successful RX would never start an enumeration at
			 * all.
			 *
			 * to_beacon was just read, so no recomputation is
			 * needed before the step; it is re-read afterwards
			 * because the step may have transmitted, and arming
			 * the RX window from a stale figure would push its
			 * expiry past the beacon. */
			if (apos_gw_busy()) {
				int32_t reserve = (int32_t)UUS_TO_HI32(
					BEACON_ARM_MARGIN_UUS +
					APOS_GW_STEP_BUDGET_UUS);

				if (to_beacon > reserve) {
					uint32_t span = (uint32_t)to_beacon -
						UUS_TO_HI32(BEACON_ARM_MARGIN_UUS);
					uint32_t avail_uus = (uint32_t)(
						((uint64_t)span << 8) / UUS_TO_DWT_TIME);

					CRUMB(GW_CRUMB_APOS_STEP);
					apos_gw_step(avail_uus, &gw_seq);

					now = dwt_readsystimestamphi32();
					to_beacon = (int32_t)(next_beacon - now);
					if (to_beacon <=
					    (int32_t)UUS_TO_HI32(BEACON_ARM_MARGIN_UUS)) {
						break;
					}
				}
			}

			/* Expire the RX window BEACON_ARM_MARGIN_UUS before the
			 * beacon, so the delayed-TX setup has a guaranteed
			 * window. Without the subtraction the timeout fires at
			 * exactly the beacon instant and the delayed TX fails
			 * every time. */
			uint32_t span_hi32 =
				(uint32_t)to_beacon - UUS_TO_HI32(BEACON_ARM_MARGIN_UUS);
			uint32_t rx_to_uus =
				(uint32_t)(((uint64_t)span_hi32 << 8) / UUS_TO_DWT_TIME);

			CRUMB(GW_CRUMB_RX_ARM);
			dwt_setpreambledetecttimeout(0);
			dwt_setrxtimeout(rx_to_uus);
			dwt_setrxaftertxdelay(0);
			dwt_rxenable(DWT_START_RX_IMMEDIATE);

			/* Bounded, not K_FOREVER. DWT_INT_RX includes RXFTO so a
			 * timeout normally arrives, but a MAC loop that can wedge
			 * takes the whole network down, not one range. One
			 * superframe of slack past the window is ample. */
			CRUMB(GW_CRUMB_RX_WAIT);
			if (k_sem_take(&rx_sem, K_MSEC(400)) != 0) {
				LOG_WRN("no RX event within the window — re-arming");
				dwt_forcetrxoff();
				continue;
			}

			/* rx_status is captured by the callbacks for symmetry with
			 * uwb_slave.c but is not read here: the gateway needs no
			 * CIR, since it does not answer ranging polls. */
			uint16_t flen = rx_len;

			rx_pending = false;

			if (flen <= FCS_LEN || flen > RX_BUF_LEN) {
				continue;
			}

			CRUMB(GW_CRUMB_RX_READ);
			dwt_readrxdata(rx_buf, flen, 0);

			uint64_t rx_ts = uwb_get_rx_timestamp_u64();

			CRUMB(GW_CRUMB_DISPATCH);
			dispatch(&ctx, rx_buf, (uint16_t)(flen - FCS_LEN), rx_ts);
		}

		CRUMB(GW_CRUMB_SF_TICK);
		gw_core_superframe_tick(&ctx);

		CRUMB(GW_CRUMB_TX_BEACON);
		uint64_t ts = tx_beacon(&ctx, true, next_beacon);

		/* Only on a CONFIRMED beacon. ts == 0 means the beacon did not
		 * go out, and a CCP scheduled from a beacon that never
		 * transmitted would be scheduled from a time that does not
		 * exist. Bounded: one delayed TX, one bounded TXFRS wait. */
		if (ts != 0u) {
			CRUMB(GW_CRUMB_CCP_TX);
			ccp_master_after_beacon(ts, &gw_seq);
		}

		/* On a miss, re-base on the current time rather than compounding
		 * the error into every following superframe. */
		beacon_tx_ts = ts ? ts
				  : (((uint64_t)dwt_readsystimestamphi32()) << 8);
	}
}
