/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SLAVE mode: an SS-TWR responder that also observes the gateway beacon.
 *
 * Interrupt-driven rather than polled. br101 runs dwt_isr() from the system
 * workqueue (platform/dw3000_hw.c), so the callbacks below are in thread
 * context -- but they still do the minimum and hand off, because the main
 * thread is ours to block and the workqueue is shared. A polling loop was
 * rejected outright: Zephyr's main thread is priority 0 and the shell thread
 * is 14, so a loop that never yields would starve the console.
 */

#include "uwb_modes.h"

#include "anchor_respond.h"
#include "apos_node.h"
#include "beacon_guard.h"
#include "blink_rx.h"
#include "ccp_slave.h"
#include "uwb_debug.h"
#include "uwb_dwtime.h"
#include "uwb_irq_wait.h"
#include "uwb_frame_802_15_4z.h"
#include "uwb_mac.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#ifdef CONFIG_ANCLA_RANGING_DEBUG
#include <zephyr/shell/shell.h>
#endif

#include <deca_device_api.h>

#include <string.h>

/* ANCLA_LOG_LEVEL, not LOG_LEVEL_INF: observe_beacon()'s per-beacon line is a
 * LOG_DBG that an INF cap makes unreachable from a .conf overlay, and the
 * beacon-guard lock state it prints is an input to every suppression decision
 * in anchor_respond.c. Still LOG_LEVEL_INF in production -- see uwb_debug.h. */
LOG_MODULE_REGISTER(uwb_slave, ANCLA_LOG_LEVEL);

/* Longest frame the contract defines is a 37-byte beacon; +FCS, rounded up. */
#define RX_BUF_LEN 64

static K_SEM_DEFINE(rx_sem, 0, 1);

/* Written by the RX callbacks, read by the main thread. rx_pending is the
 * guard that makes this safe: br101's IRQ-drain loop (platform/dw3000_hw.c)
 * calls dwt_isr() repeatedly while the IRQ line is held, and DWT_INT_RX
 * includes DWT_INT_CIADONE_BIT_MASK, whose completion can trail the RXFCG
 * pass -- so a second cb_rx_ok/cb_rx_fail can fire (or a datalength==0 case
 * can dispatch cb_rx_fail) before the main thread has consumed the previous
 * event's rx_status/rx_len via k_sem_take. Without the guard that second
 * write corrupts the still-unread values, and the second k_sem_give is
 * silently swallowed by the limit-1 semaphore -- a real event lost or
 * corrupted. rx_pending makes each callback a no-op once an event is already
 * waiting for the main thread; the main thread clears it right after
 * k_sem_take, before touching rx_status/rx_len, so the next callback is free
 * to capture again. A plain flag suffices here: these callbacks run in
 * cooperative thread context on the system workqueue, not a preempting ISR. */
static volatile uint32_t rx_status;
static volatile uint16_t rx_len;
static volatile bool rx_pending;

static uint8_t rx_buf[RX_BUF_LEN];
static uint8_t frame_seq_nb;

static struct beacon_guard bguard;

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

static void rx_arm(void)
{
	dwt_setpreambledetecttimeout(0);
	dwt_setrxtimeout(0);
	dwt_setrxaftertxdelay(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/* Read the Ipatov CIR metrics, but only if CIA had finished for this frame.
 *
 * dwt_isr() clears SYS_STATUS_ALL_RX_GOOD -- which includes CIADONE -- before
 * invoking cb_rx_ok, so the register cannot be polled here; the pre-clear
 * snapshot in cb_data->status is the only evidence available. Reporting zero is
 * safe: the tag ranks anchors by these metrics, it does not accept or reject a
 * range on them. */
#ifdef CONFIG_ANCLA_RANGING_DEBUG
/* Side channel from read_cir() for the `cia` shell command: whether the last
 * call actually read diagnostics, and the first-path amplitude it saw (F1),
 * which read_cir()'s own outputs do not carry. */
static bool dbg_diag_valid;
static uint32_t dbg_last_f1;
#endif

static void read_cir(uint32_t status, int32_t *cir_power, uint16_t *cir_quality)
{
#ifdef CONFIG_ANCLA_RANGING_DEBUG
	dbg_diag_valid = false;
#endif
	if (!(status & DWT_INT_CIADONE_BIT_MASK)) {
		*cir_power = 0;
		*cir_quality = 0;
		return;
	}

	dwt_rxdiag_t diag;

	memset(&diag, 0, sizeof(diag));
	dwt_readdiagnostics(&diag);
	/* Ipatov (preamble) metrics — STS is off in this PHY. */
	*cir_power = (int32_t)diag.ipatovPower;
	*cir_quality = diag.ipatovAccumCount;
#ifdef CONFIG_ANCLA_RANGING_DEBUG
	dbg_last_f1 = diag.ipatovF1;
	dbg_diag_valid = true;
#endif
}

static void observe_beacon(const uint8_t *buf, uint16_t len,
			   const uwb_config_t *cfg, uint64_t rx_ts)
{
	if (!uwb_frame_is_beacon(buf, len)) {
		return;
	}

	uint8_t proto_ver = 0;
	uint32_t frame_counter = 0;
	uint16_t slot_map[UWB_FRAME_N_CFP];
	uint8_t n_slots = 0;

	int rc = uwb_frame_parse_beacon(buf, len, &proto_ver, &frame_counter,
					slot_map, &n_slots);
	if (rc) {
		LOG_WRN("malformed beacon (%d)", rc);
		return;
	}
	/* slot_map cannot have overflowed above: parse_beacon derives n_slots as
	 * (len - 15) / 2, and uwb_frame_is_valid() already capped len at
	 * UWB_FRAME_MAX_LEN (37) == 15 + 2 * UWB_FRAME_N_CFP, so n_slots <= 11.
	 * This check exists to catch that identity being broken by a future
	 * change to either constant, not to protect the write. */
	if (n_slots > UWB_FRAME_N_CFP) {
		LOG_ERR("beacon parsed %u slots, buffer holds %u — "
			"UWB_FRAME_MAX_LEN and UWB_FRAME_N_CFP disagree",
			n_slots, UWB_FRAME_N_CFP);
		return;
	}

	int slot = uwb_frame_beacon_find_addr(slot_map, n_slots,
					      uwb_config_short_addr(cfg));

	/* Re-anchor on the beacon's own RX timestamp, in the same hi32 units
	 * every scheduled TX is expressed in -- no clock conversion, so no
	 * second clock to drift against. */
	beacon_guard_beacon(&bguard, (uint32_t)(rx_ts >> 8));

	/* Recorded and logged only. Gating RX windows on beacon timing needs a
	 * real gateway to develop against — that is spec D. LOG_DBG, not
	 * LOG_INF: this fires on every beacon (roughly every 195 ms) regardless
	 * of ranging activity, and at LOG_LEVEL_INF the call compiles out
	 * entirely — link-budget timing on the DISCOVERY/WAVE responders (see
	 * disc_schedule.h, anchor_respond.c) matters more than always-on beacon
	 * visibility. Bump this module's LOG_MODULE_REGISTER level to
	 * LOG_LEVEL_DBG to re-enable for debugging. */
	LOG_DBG("BEACON ver=%u counter=%u slots=%u our_slot=%d locked=%d",
		proto_ver, frame_counter, n_slots, slot,
		beacon_guard_locked(&bguard));
}

#ifdef CONFIG_ANCLA_RANGING_DEBUG

/* Type byte offset in every frame this network uses -- the module frames and
 * the legacy WAVE/VEWA pair alike (OFF_TYPE in uwb_frame_802_15_4z.c). Counted
 * here rather than inside each responder so one place sees every frame that
 * reached the loop, including the ones every responder declines. */
#define DBG_OFF_TYPE 9

/* ---- Receive instrumentation, read from the `cia` shell command ----------
 *
 * There is deliberately NO periodic log line here. An earlier 1 Hz heartbeat
 * ran an extra SPI read and a log call on the loop that has to re-arm RX within
 * a few hundred microseconds of each beacon, and it cost the CCP behind every
 * fifth beacon (the heartbeat fell on a beacon boundary). Everything below is
 * counted in memory at the point of the event and printed only when asked,
 * from the shell thread.
 *
 * Questions this answers, in order of how decisive they are:
 *   - is the CCP link starved of SNR, or saturated? (CIA accumulation count,
 *     channel power and first-path amplitude per frame class)
 *   - does a CCP arrive when RX is re-armed early enough? (probability of a CCP
 *     after each beacon, bucketed by how long the re-arm took)
 *   - how do receive errors break down? (PHR / Reed-Solomon / FCS / SFD timeout)
 */
enum dbg_class { DBG_BEACON, DBG_CCP, DBG_OTHER, DBG_N_CLASS };

struct dbg_cia {
	uint32_t n;      /* frames with diagnostics */
	uint32_t no_cia; /* frames whose CIADONE was not set */
	uint32_t acc_min, acc_max;
	uint32_t pow_min, pow_max;
	uint32_t f1_min, f1_max;
	uint64_t acc_sum, pow_sum, f1_sum;
};

/* Re-arm latency buckets, beacon RMARKER to RX re-armed. 50 us wide, from 300 us. */
#define DBG_BUCKETS 16
#define DBG_BUCKET_BASE_NS 300000u
#define DBG_BUCKET_STEP_NS 50000u

struct dbg_data {
	uint32_t rx_ok;
	uint32_t rx_err;
	uint32_t wave;   /* 0xE0 legacy poll, addressed to any anchor */
	uint32_t disc;   /* 0xE2 DISCOVERY broadcast */
	uint32_t beacon; /* 0xE5 */
	uint32_t apos;   /* 0xEB survey traffic */
	uint32_t ccp;    /* 0xEF clock calibration packet */
	uint32_t other;
	uint32_t err_phe;    /* PHR error */
	uint32_t err_fsl;    /* Reed-Solomon / sync loss */
	uint32_t err_fce;    /* FCS error */
	uint32_t err_sto;    /* SFD timeout */
	uint32_t err_cia;    /* CIA error */
	uint32_t err_other;
	struct dbg_cia cia[DBG_N_CLASS];
	uint32_t rearm_n, rearm_min, rearm_max;
	uint64_t rearm_sum;
	uint32_t bucket_beacons[DBG_BUCKETS];
	uint32_t bucket_ccp[DBG_BUCKETS];
	uint32_t ccp_off_n, ccp_off_min, ccp_off_max;
	uint64_t ccp_off_sum;
};

static struct dbg_data dbg;
static struct dbg_data dbg_snap;

/* Bucket of the latest beacon while its CCP has not yet been seen; -1 once
 * consumed or when no beacon is outstanding. Not part of dbg so a `cia reset`
 * does not lose the pairing in flight. */
static int dbg_pending_bucket = -1;
static uint64_t dbg_beacon_ts;

static void dbg_reset(void)
{
	memset(&dbg, 0, sizeof(dbg));
	for (int i = 0; i < DBG_N_CLASS; i++) {
		dbg.cia[i].acc_min = UINT32_MAX;
		dbg.cia[i].pow_min = UINT32_MAX;
		dbg.cia[i].f1_min = UINT32_MAX;
	}
	dbg.rearm_min = UINT32_MAX;
	dbg.ccp_off_min = UINT32_MAX;
}

/* hi32 tick = 256 DTU = 4.006 ns. */
static uint32_t dbg_ticks_to_ns(uint32_t ticks)
{
	return (uint32_t)(((uint64_t)ticks * 4006u) / 1000u);
}

static void dbg_count(const uint8_t *buf, uint16_t plen, uint64_t rx_ts)
{
	dbg.rx_ok++;
	if (plen <= DBG_OFF_TYPE) {
		dbg.other++;
		return;
	}
	switch (buf[DBG_OFF_TYPE]) {
	case 0xE0: dbg.wave++;   break;
	case 0xE2: dbg.disc++;   break;
	case 0xE5:
		dbg.beacon++;
		dbg_beacon_ts = rx_ts;
		break;
	case 0xEB: dbg.apos++;   break;
	case 0xEF:
		dbg.ccp++;
		if (dbg_pending_bucket >= 0) {
			uint32_t off = dbg_ticks_to_ns(
				(uint32_t)(((rx_ts - dbg_beacon_ts) &
					    0xFFFFFFFFFFULL) >> 8));

			dbg.bucket_ccp[dbg_pending_bucket]++;
			dbg_pending_bucket = -1;
			dbg.ccp_off_n++;
			dbg.ccp_off_sum += off;
			if (off < dbg.ccp_off_min) {
				dbg.ccp_off_min = off;
			}
			if (off > dbg.ccp_off_max) {
				dbg.ccp_off_max = off;
			}
		}
		break;
	default:
		dbg.other++;
		/* Type and length of anything the responders will not claim. A
		 * DISCOVERY that is well-formed enough to carry 0xE2 but too
		 * short for UWB_FRAME_LEN_DISC counts as disc above and yet
		 * produces no {"disc":...} verdict line, so comparing the two is
		 * what separates a malformed frame from a refused one. */
		LOG_DBG("{\"rx_other\":{\"type\":\"0x%02X\",\"plen\":%u,"
			"\"src\":\"0x%04X\"}}",
			buf[DBG_OFF_TYPE], plen,
			uwb_frame_get_src_addr(buf));
		break;
	}
}

static void dbg_on_error(uint32_t status)
{
	dbg.rx_err++;
	if (status & DWT_INT_RXPHE_BIT_MASK) {
		dbg.err_phe++;
	} else if (status & DWT_INT_RXFSL_BIT_MASK) {
		dbg.err_fsl++;
	} else if (status & DWT_INT_RXFCE_BIT_MASK) {
		dbg.err_fce++;
	} else if (status & DWT_INT_RXSTO_BIT_MASK) {
		dbg.err_sto++;
	} else if (status & DWT_INT_CIAERR_BIT_MASK) {
		dbg.err_cia++;
	} else {
		dbg.err_other++;
	}
}

/* Called right after read_cir(): folds that frame's diagnostics into its
 * class. Uses the side channel read_cir() leaves behind. */
static void dbg_cia_record(enum dbg_class cls, int32_t cir_power,
			   uint16_t cir_quality)
{
	struct dbg_cia *c = &dbg.cia[cls];

	if (!dbg_diag_valid) {
		c->no_cia++;
		return;
	}

	uint32_t acc = cir_quality;
	uint32_t pwr = (uint32_t)cir_power;
	uint32_t f1 = dbg_last_f1;

	c->n++;
	c->acc_sum += acc;
	c->pow_sum += pwr;
	c->f1_sum += f1;
	if (acc < c->acc_min) { c->acc_min = acc; }
	if (acc > c->acc_max) { c->acc_max = acc; }
	if (pwr < c->pow_min) { c->pow_min = pwr; }
	if (pwr > c->pow_max) { c->pow_max = pwr; }
	if (f1 < c->f1_min) { c->f1_min = f1; }
	if (f1 > c->f1_max) { c->f1_max = f1; }
}

/* Called right after a beacon's rx_arm(): `rearm_ticks` is the hi32 distance
 * from the beacon's RMARKER to the moment RX was armed again. */
static void dbg_beacon_rearmed(uint32_t rearm_ticks)
{
	uint32_t ns = dbg_ticks_to_ns(rearm_ticks);
	uint32_t b = 0;

	if (ns > DBG_BUCKET_BASE_NS) {
		b = (ns - DBG_BUCKET_BASE_NS) / DBG_BUCKET_STEP_NS;
		if (b >= DBG_BUCKETS) {
			b = DBG_BUCKETS - 1;
		}
	}
	dbg.rearm_n++;
	dbg.rearm_sum += ns;
	if (ns < dbg.rearm_min) { dbg.rearm_min = ns; }
	if (ns > dbg.rearm_max) { dbg.rearm_max = ns; }
	dbg.bucket_beacons[b]++;
	dbg_pending_bucket = (int)b;
}

static uint32_t dbg_mean(uint64_t sum, uint32_t n)
{
	return n ? (uint32_t)(sum / n) : 0u;
}

static uint32_t dbg_min(uint32_t v, uint32_t n)
{
	return n ? v : 0u;
}

static void print_cia(const struct shell *sh, const char *name,
		      const struct dbg_cia *c)
{
	shell_print(sh,
		    "{\"cia\":{\"class\":\"%s\",\"n\":%u,\"no_cia\":%u,"
		    "\"accum\":[%u,%u,%u],\"power\":[%u,%u,%u],"
		    "\"f1\":[%u,%u,%u]}}",
		    name, c->n, c->no_cia,
		    dbg_min(c->acc_min, c->n), dbg_mean(c->acc_sum, c->n), c->acc_max,
		    dbg_min(c->pow_min, c->n), dbg_mean(c->pow_sum, c->n), c->pow_max,
		    dbg_min(c->f1_min, c->n), dbg_mean(c->f1_sum, c->n), c->f1_max);
}

/* cia stats: everything counted since boot or the last `cia reset`, as JSON.
 * accum / power / f1 are [min, mean, max]. ccp_vs_rearm rows are
 * [bucket lower edge in us, beacons whose re-arm landed there, of those how
 * many were followed by a CCP]. */
static int cmd_cia_stats(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* The main thread is preemptible on a SLAVE and writes these, so the
	 * snapshot is taken with the scheduler locked. */
	k_sched_lock();
	dbg_snap = dbg;
	k_sched_unlock();

	const struct dbg_data *d = &dbg_snap;

	shell_print(sh,
		    "{\"rx\":{\"ok\":%u,\"err\":%u,\"beacon\":%u,\"ccp\":%u,"
		    "\"wave\":%u,\"disc\":%u,\"apos\":%u,\"other\":%u,"
		    "\"err_phr\":%u,\"err_rs\":%u,\"err_fcs\":%u,"
		    "\"err_sfd_to\":%u,\"err_cia\":%u,\"err_other\":%u}}",
		    d->rx_ok, d->rx_err, d->beacon, d->ccp, d->wave, d->disc,
		    d->apos, d->other, d->err_phe, d->err_fsl, d->err_fce,
		    d->err_sto, d->err_cia, d->err_other);
	print_cia(sh, "beacon", &d->cia[DBG_BEACON]);
	print_cia(sh, "ccp", &d->cia[DBG_CCP]);
	print_cia(sh, "other", &d->cia[DBG_OTHER]);
	shell_print(sh,
		    "{\"timing\":{\"rearm_ns\":{\"n\":%u,\"min\":%u,\"mean\":%u,"
		    "\"max\":%u},\"ccp_off_ns\":{\"n\":%u,\"min\":%u,\"mean\":%u,"
		    "\"max\":%u}}}",
		    d->rearm_n, dbg_min(d->rearm_min, d->rearm_n),
		    dbg_mean(d->rearm_sum, d->rearm_n), d->rearm_max,
		    d->ccp_off_n, dbg_min(d->ccp_off_min, d->ccp_off_n),
		    dbg_mean(d->ccp_off_sum, d->ccp_off_n), d->ccp_off_max);

	shell_fprintf(sh, SHELL_NORMAL, "{\"ccp_vs_rearm\":[");
	for (int i = 0; i < DBG_BUCKETS; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "%s[%u,%u,%u]", i ? "," : "",
			      (DBG_BUCKET_BASE_NS + (uint32_t)i * DBG_BUCKET_STEP_NS) /
				      1000u,
			      d->bucket_beacons[i], d->bucket_ccp[i]);
	}
	shell_fprintf(sh, SHELL_NORMAL, "]}\n");
	return 0;
}

static int cmd_cia_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	k_sched_lock();
	dbg_reset();
	k_sched_unlock();
	shell_print(sh, "cia counters cleared");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cia,
	SHELL_CMD_ARG(stats, NULL,
		      "stats -- receive counters and per-class CIA diagnostics "
		      "(accum count, channel power, first-path F1), error "
		      "breakdown, beacon re-arm latency and the CCP-after-beacon "
		      "table, as JSON. Debug image only",
		      cmd_cia_stats, 1, 0),
	SHELL_CMD_ARG(reset, NULL, "reset -- clear the counters", cmd_cia_reset,
		      1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(cia, &sub_cia, "RX diagnostics (debug image)", NULL);

#endif /* CONFIG_ANCLA_RANGING_DEBUG */

void uwb_slave_run(const uwb_config_t *cfg)
{
	/* Snapshot at mode-entry: anchor_shell.c's "anchor id/ant/pos" commands
	 * mutate the live uwb_config_get() singleton that cfg points at, and
	 * this loop runs indefinitely on the main thread with no synchronisation
	 * against the shell thread. Operating on a stable copy restores the
	 * documented "changes take effect only on reboot" contract -- notably
	 * for ant_delay_tx, which resp_tx_ts arithmetic must keep matching the
	 * antenna delay actually programmed into the radio at boot.
	 *
	 * Kept as a named MUTABLE object as well as the const view:
	 * apos_node_on_rx() writes a surveyed position straight into it, so
	 * `apos apply` takes effect without a reboot, while every other consumer
	 * keeps the const view and the reboot contract above for id and antenna
	 * delay. */
	static uwb_config_t cfg_snapshot;

	cfg_snapshot = *cfg;
	cfg = &cfg_snapshot;

	static dwt_callbacks_s cbs;

	cbs.cbRxOk = cb_rx_ok;
	cbs.cbRxTo = cb_rx_fail;
	cbs.cbRxErr = cb_rx_fail;
	dwt_setcallbacks(&cbs);

	/* DWT_INT_RX is the RX-only mask. TXFRS must stay out of it: tx_delayed()
	 * polls for transmit completion, and an ISR that cleared the bit first
	 * would hang that poll. */
	dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT);

	LOG_INF("{\"status\":\"listening\",\"mode\":\"slave\",\"id\":%u,"
		"\"short_addr\":\"0x%04X\"}",
		cfg->anchor_id, uwb_config_short_addr(cfg));

	apos_node_init();
	blink_rx_init(cfg);
	ccp_slave_init();
	/* B14: install this anchor's configured flight-time compensation
	 * (see sync_model.h's sync_model_set_tau()) before the loop below
	 * ever feeds it a CCP. `sync tau` shell commands re-apply this live
	 * afterward -- FR-5's feed-forward, deliberately NOT gated behind the
	 * snapshot-at-mode-entry / reboot-to-apply contract cfg_snapshot
	 * otherwise enforces for ant_delay and friends. */
	ccp_slave_set_tau(uwb_config_sync_tau_effective(cfg));

	beacon_guard_init(&bguard, UUS_TO_HI32(T_SUPERFRAME_UUS),
			  UUS_TO_HI32(BEACON_GUARD_UUS),
			  UUS_TO_HI32(BEACON_OCCUPANCY_UUS));

	rx_arm();

#ifdef CONFIG_ANCLA_RANGING_DEBUG
	dbg_reset();
#endif

	while (1) {
		/* No periodic work on this loop, in any image: see the note on the
		 * receive instrumentation above for why a 1 Hz heartbeat cost the
		 * CCP behind every fifth beacon. */
		(void)uwb_irq_wait(&rx_sem, K_FOREVER);

		uint32_t status = rx_status;
		uint16_t flen = rx_len;
		rx_pending = false;

		/* flen counts the FCS. Anything that cannot hold one is not a
		 * frame we can reason about. cb_rx_fail() reports flen 0, so this
		 * is also where an RX error or timeout lands. */
		if (flen <= FCS_LEN || flen > RX_BUF_LEN) {
#ifdef CONFIG_ANCLA_RANGING_DEBUG
			dbg_on_error(status);
#endif
			rx_arm();
			continue;
		}

		int32_t cir_power = 0;
		uint16_t cir_quality = 0;

		dwt_readrxdata(rx_buf, flen, 0);
		uint64_t rx_ts = uwb_get_rx_timestamp_u64();

		/* Payload length: every consumer below derives field counts from
		 * it, and two extra bytes corrupt them silently. */
		uint16_t plen = (uint16_t)(flen - FCS_LEN);

#ifdef CONFIG_ANCLA_RANGING_DEBUG
		dbg_count(rx_buf, plen, rx_ts);
#endif

		/* A beacon is followed IMMEDIATELY by the CCP (ccp_master.c
		 * transmits it with DWT_START_TX_IMMEDIATE right after the
		 * beacon's TXFRS). At PLEN_256 that CCP's preamble is only
		 * ~262 us long and starts ~150-230 us after the beacon ends,
		 * so this receiver has to be listening again within a few
		 * hundred microseconds. At PLEN_1024 the preamble lasted
		 * ~1050 us and the CIR read below (133-156 us of SPI) plus
		 * the rest of the chain fit inside it; at 256 they do not.
		 *
		 * So a beacon takes a short path: data and timestamp are
		 * already read, RX is re-armed NOW, and only then is the
		 * beacon processed. rx_buf is written solely by this thread
		 * (dwt_readrxdata above), so observing it after re-arming is
		 * safe. read_cir() is skipped: nothing on this path uses the
		 * beacon's CIR. */
		if (uwb_frame_is_beacon(rx_buf, plen)) {
			rx_arm();
#ifdef CONFIG_ANCLA_RANGING_DEBUG
			/* Beacon RMARKER to RX re-armed. The CCP preamble must
			 * not have started before this. The CIA read that
			 * follows comes AFTER the re-arm and finishes well
			 * before the CCP frame ends (CIA only runs at a
			 * frame's end), so it cannot corrupt that frame's own
			 * diagnostics. */
			dbg_beacon_rearmed(dwt_readsystimestamphi32() -
					   (uint32_t)(rx_ts >> 8));
			read_cir(status, &cir_power, &cir_quality);
			dbg_cia_record(DBG_BEACON, cir_power, cir_quality);
#endif
			observe_beacon(rx_buf, plen, cfg, rx_ts);
			continue;
		}

		read_cir(status, &cir_power, &cir_quality);
#ifdef CONFIG_ANCLA_RANGING_DEBUG
		dbg_cia_record((plen > DBG_OFF_TYPE && rx_buf[DBG_OFF_TYPE] == 0xEF)
				       ? DBG_CCP
				       : DBG_OTHER,
			       cir_power, cir_quality);
#endif

		/* First in the chain, and cheap to reject: ccp_frame_is_ccp()
		 * is a type-and-length test. A CCP arrives once per superframe
		 * and is for nobody else, so there is no reason to walk it past
		 * the survey and ranging responders. */
		if (ccp_slave_on_rx(rx_buf, plen, rx_ts)) {
			rx_arm();
			continue;
		}

		/* A BLINK is nobody else's either, and its timestamp IS the
		 * datum: it is handled before the responder chain so no branch
		 * of that chain can spend time between reading the timestamp
		 * and converting it. It transmits nothing, so it does not
		 * interact with the beacon guard. Deliberately AFTER the CCP:
		 * the CCP is what makes a BLINK convertible at all, so if both
		 * ever land in one turn of the loop, the frame that sustains
		 * the clock model goes first. */
		if (blink_rx_on_rx(rx_buf, plen, rx_ts, cir_quality)) {
			rx_arm();
			continue;
		}

		/* Offered to each in turn; each ignores what is not its
		 * own. APOS goes first and short-circuits: a survey
		 * frame is never also a ranging poll or a beacon, and a
		 * RANGE_CMD blocks for a whole batch, so there is no
		 * point offering it to anyone else afterwards. */
		if (!apos_node_on_rx(rx_buf, plen, &cfg_snapshot,
				     &frame_seq_nb, &bguard)) {
			anchor_respond_wave_poll(rx_buf, plen, rx_ts, cfg,
						 &frame_seq_nb, &bguard,
						 apos_node_window_open());
			anchor_respond_discovery(rx_buf, plen, rx_ts, cfg,
						 &frame_seq_nb, cir_power,
						 cir_quality, &bguard);
			observe_beacon(rx_buf, plen, cfg, rx_ts);
		}

		rx_arm();
	}
}
