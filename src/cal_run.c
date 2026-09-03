/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal_run.h. The RX path mirrors uwb_slave.c deliberately -- it is the
 * production responder, and the whole point of calibrating in this image is
 * that the exchange being measured is the one the tag actually uses.
 */

#include "cal_run.h"

#include "anchor_respond.h"
#include "ss_initiator.h"
#include "cal_math.h"
#include "cal_solve.h"
#include "uwb_dwtime.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <errno.h>

LOG_MODULE_REGISTER(cal_run, LOG_LEVEL_INF);

#define RX_BUF_LEN 64

static K_SEM_DEFINE(rx_sem, 0, 1);
static K_SEM_DEFINE(req_sem, 0, 1);
static K_SEM_DEFINE(done_sem, 0, 1);

/* Handed between the shell thread and the main loop, one batch at a time. The
 * semaphore pair is the handshake: the shell writes g_req before giving
 * req_sem and reads g_res only after taking done_sem, so the two never touch
 * either struct at once. g_busy rejects a second command while one is running
 * rather than letting it corrupt the first. */
static struct cal_request g_req;
static struct cal_result g_res;
static bool g_busy;

/* See uwb_slave.c:38-51 for why rx_pending is required. */
static volatile uint32_t rx_status;
static volatile uint16_t rx_len;
static volatile bool rx_pending;

static uint8_t rx_buf[RX_BUF_LEN];
static uint8_t frame_seq_nb;

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

int cal_run_execute(const struct cal_request *req, struct cal_result *out)
{
	if (g_busy) {
		return -EBUSY;
	}
	g_busy = true;
	g_req = *req;

	/* Clear any stale token left by a prior batch that timed out below but
	 * is still running in the background: without this, this command's
	 * k_sem_take() could return instantly on that stale give and hand back
	 * the PREVIOUS (unrelated) batch's result as if it were this one's. */
	k_sem_reset(&done_sem);
	k_sem_give(&req_sem);

	/* Generous: a fully failing 128-sample batch is 128 * (10 + 25) ms of
	 * timeouts plus the inter-sample sleep, about 6 s. */
	if (k_sem_take(&done_sem, K_SECONDS(30)) != 0) {
		g_busy = false;
		return -ETIMEDOUT;
	}

	*out = g_res;
	g_busy = false;
	return g_res.status;
}

/* Static, not on the stack: CONFIG_MAIN_STACK_SIZE is 4096 and this is 512 B,
 * which is a large fraction of it. */
static int32_t samples[CAL_MAX_SAMPLES];

static void run_batch(const struct cal_request *req, struct cal_result *res,
		      uwb_config_t *cfg)
{
	uint32_t valid = 0;
	uint32_t attempts = CAL_MAX_SAMPLES;
	int64_t lvl_sum = 0;

	if (req->link && req->attempts > 0u) {
		attempts = req->attempts;
	}

	ss_initiator_enter();

	for (uint32_t i = 0; i < attempts; i++) {
		int16_t rssi_q8 = SS_INITIATOR_RSSI_INVALID;
		int32_t mm;

		if (req->link) {
			mm = ss_initiator_range_ex(req->peer_wire_id, &rssi_q8);
		} else {
			mm = ss_initiator_range(req->peer_wire_id);
		}

		if (mm != INT32_MIN) {
			/* samples[] holds CAL_MAX_SAMPLES. In link mode a
			 * caller may ask for more ATTEMPTS than that -- the
			 * point at long range is a bigger denominator, not a
			 * bigger sample set -- so keep counting valid
			 * exchanges past the array and store only what fits.
			 * The statistics are then over the first
			 * CAL_MAX_SAMPLES, which is stated in the report. */
			if (valid < CAL_MAX_SAMPLES) {
				samples[valid] = mm;
			}
			valid++;

			/* q8.8 dBm -> dBm x10, rounded to nearest. The
			 * driver's own value; see ss_initiator.h for why this
			 * is not computed here. */
			int32_t lvl = ((int32_t)rssi_q8 * 10 +
				       ((rssi_q8 < 0) ? -128 : 128)) / 256;

			if (rssi_q8 != SS_INITIATOR_RSSI_INVALID) {
				if (res->rx_level_n == 0u) {
					res->rx_level_min_x10 = lvl;
					res->rx_level_max_x10 = lvl;
				} else {
					if (lvl < res->rx_level_min_x10)
						res->rx_level_min_x10 = lvl;
					if (lvl > res->rx_level_max_x10)
						res->rx_level_max_x10 = lvl;
				}
				lvl_sum += lvl;
				res->rx_level_n++;
			}
		}

		/* Yields to the shell (priority 14) so the console stays
		 * responsive through the batch, and decorrelates consecutive
		 * exchanges slightly. */
		k_sleep(K_MSEC(2));
	}

	ss_initiator_leave();

	res->attempted = attempts;
	res->valid = valid;

	ss_initiator_diag(NULL, &res->f_tx_start, &res->f_tx_done,
			  &res->f_rx_to_err, &res->f_len, &res->f_hdr,
			  &res->f_layout);

	if (req->link) {
		size_t n = (valid < CAL_MAX_SAMPLES) ? valid : CAL_MAX_SAMPLES;
		struct cal_link_stats st;

		if (res->rx_level_n > 0u) {
			res->rx_level_mean_x10 =
				(int32_t)(lvl_sum / (int64_t)res->rx_level_n);
		}

		/* NO -ENODATA floor here, deliberately. That gate protects a
		 * CALIBRATION from being computed over a handful of lucky
		 * frames; in link mode a low success rate is not a failed
		 * measurement, it IS the measurement. A batch with zero valid
		 * exchanges still reports attempted/valid and the failure
		 * breakdown, which at range is the whole answer. */
		if (n > 0u && cal_link_stats_compute(samples, n, &st)) {
			res->mean_mm = st.mean_mm;
			res->sd_mm   = st.sd_mm;
			res->min_mm  = st.min_mm;
			res->max_mm  = st.max_mm;
		}
		res->status = 0;
		return;
	}

	/* Enough samples to mean anything. Below this the peer is not really
	 * answering -- wrong PHY, wrong peer id, or out of range -- and a mean
	 * over a handful of lucky frames would look like a calibration. */
	if (valid < CAL_MAX_SAMPLES / 4U) {
		res->status = -ENODATA;
		return;
	}

	if (!cal_filtered_mean(samples, valid, &res->mean_mm, &res->kept)) {
		res->status = -EIO;
		return;
	}

	res->error_mm = res->mean_mm - res->ref_mm;

	if (!req->persist) {
		res->status = 0;
		return;
	}

	res->old_tx = cfg->ant_delay_tx;

	int rc = cal_solve_tx_delay(res->mean_mm, res->ref_mm,
				    cfg->ant_delay_tx, cfg->ant_delay_rx,
				    &res->new_tx);

	if (rc) {
		res->status = rc;
		return;
	}

	/* Applied hot, and the snapshot and the register move together -- see
	 * the comment on cfg_live in cal_run(). Persistence is the shell's job:
	 * it owns the uwb_store call, this loop owns the radio. */
	cfg->ant_delay_tx = res->new_tx;
	dwt_settxantennadelay(res->new_tx);

	res->status = 0;
}

void cal_run(const uwb_config_t *cfg)
{
	/* Mutable, unlike uwb_slave_run()'s const snapshot: a `cal ref` applies
	 * its solved delay hot, and this copy and the radio's TX antenna delay
	 * register must move together. anchor_respond.c:141 adds
	 * cfg->ant_delay_tx in software to build the reported resp_tx_ts while
	 * the radio applies the register to the frame it actually sends; if the
	 * two disagree this board silently reports a turnaround it did not
	 * perform. cal_run owns this copy outright -- the shell thread never
	 * touches it. */
	static uwb_config_t cfg_live;

	cfg_live = *cfg;

	static dwt_callbacks_s cbs;

	cbs.cbRxOk = cb_rx_ok;
	cbs.cbRxTo = cb_rx_fail;
	cbs.cbRxErr = cb_rx_fail;
	dwt_setcallbacks(&cbs);

	/* RX only, same as the production responder: TXFRS must stay out of the
	 * mask or the responder's tx_delayed() completion poll would hang. */
	dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT);

	LOG_INF("{\"status\":\"listening\",\"mode\":\"cal\",\"id\":%u,"
		"\"short_addr\":\"0x%04X\",\"ant_tx\":%u,\"ant_rx\":%u}",
		cfg_live.anchor_id, uwb_config_short_addr(&cfg_live),
		cfg_live.ant_delay_tx, cfg_live.ant_delay_rx);

	struct k_poll_event events[2] = {
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&rx_sem, 0),
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&req_sem, 0),
	};

	rx_arm();

	while (1) {
		k_poll(events, 2, K_FOREVER);

		if (events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
			k_sem_take(&req_sem, K_NO_WAIT);

			struct cal_result res = {0};

			res.ref_mm = g_req.ref_mm;
			run_batch(&g_req, &res, &cfg_live);
			g_res = res;

			rx_arm();
			k_sem_give(&done_sem);
		}

		if (events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
			k_sem_take(&rx_sem, K_NO_WAIT);

			uint16_t flen = rx_len;

			rx_pending = false;

			if (flen > FCS_LEN && flen <= RX_BUF_LEN) {
				dwt_readrxdata(rx_buf, flen, 0);

				uint64_t rx_ts = uwb_get_rx_timestamp_u64();
				uint16_t plen = (uint16_t)(flen - FCS_LEN);

				/* DIAGNOSTIC: this board's idle responder duty
				 * had never been exercised against another
				 * live ANCLA cal image before this debugging
				 * session (only against the DWM3001CDK
				 * reference, which does not filter on id).
				 * Show what actually arrived, and what this
				 * board expects as its own wire id, so a
				 * mismatch or a "nothing ever arrives" case
				 * is visible directly instead of guessed at. */
				LOG_HEXDUMP_INF(rx_buf, plen,
						"cal diag: RX frame while idle");
				LOG_INF("cal diag: plen=%u our_wire_id=0x%02X "
					"frame_id_byte=0x%02X (idx10)",
					plen,
					(unsigned int)(uwb_config_short_addr(&cfg_live) & 0xFFu),
					(plen > 10U) ? rx_buf[10] : 0xFFU);

				/* No beacon guard: there is no gateway on air
				 * during calibration, and
				 * beacon_guard_tx_allowed() returns true while
				 * unlocked in any case (beacon_guard.c:37). */
				anchor_respond_wave_poll(rx_buf, plen, rx_ts,
							 &cfg_live,
							 &frame_seq_nb, NULL,
							 true);
			}

			rx_arm();
		}

		events[0].state = K_POLL_STATE_NOT_READY;
		events[1].state = K_POLL_STATE_NOT_READY;
	}
}
