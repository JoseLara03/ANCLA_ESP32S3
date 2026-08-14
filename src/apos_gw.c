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

/* Set by apos_gw_start_run(), cleared by apos_gw_start_enum(): whether the ENUM
 * phase now running should fall through into RANGE when it completes. */
static bool is_run;

/* RANGE phase cursor. `pair_idx` walks every ORDERED pair in a fixed row-major
 * order: for from = 0..n-1, for to = 0..n-1, skipping from == to. A single
 * index rather than two counters, so the phase state is one number to reason
 * about and meas_done/meas_total are trivially derived. */
static uint16_t pair_idx;
static uint16_t pair_total;
static uint8_t pair_retries;
static uint8_t tx_fails;
static bool awaiting_rsp;
static uint16_t await_from_addr;
static uint16_t await_peer_addr;

/* Result judgement, filled by do_solve()/judge_result(). */
static bool accepted;
static int16_t solve_redundancy; /* see apos_gw_result_redundancy() */
static uint16_t solve_n_edges;
static bool solve_unverified;

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
	is_run = false;
	pair_idx = 0;
	pair_total = 0;
	pair_retries = 0;
	tx_fails = 0;
	awaiting_rsp = false;
	accepted = false;
	solve_redundancy = 0;
	solve_n_edges = 0;
	solve_unverified = false;
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
	out->meas_done = pair_idx;
	out->meas_total = pair_total;
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
	is_run = false;

	/* Clear the run cursor too, so a bare `apos enum` after an aborted run
	 * cannot inherit stale RANGE state. */
	pair_idx = 0;
	pair_total = 0;
	pair_retries = 0;
	tx_fails = 0;
	awaiting_rsp = false;
	accepted = false;
	solve_unverified = false;
	solve_redundancy = 0;
	solve_n_edges = 0;

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

/* ---- RANGE phase ---- */

/* Decompose an ordered-pair index into (from, to), skipping the diagonal. */
static void pair_of(uint16_t idx, uint8_t n, uint8_t *from, uint8_t *to)
{
	uint16_t per_row = (uint16_t)(n - 1u);

	*from = (uint8_t)(idx / per_row);

	uint8_t col = (uint8_t)(idx % per_row);

	*to = (col >= *from) ? (uint8_t)(col + 1u) : col;
}

void apos_gw_set_zoff(float dz)
{
	zoff_m = dz;
}

const struct apos_result *apos_gw_result(void)
{
	return &res;
}

bool apos_gw_accepted(void)
{
	return accepted;
}

bool apos_gw_result_unverified(void)
{
	return solve_unverified;
}

int apos_gw_result_redundancy(void)
{
	return solve_redundancy;
}

int apos_gw_start_run(void)
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}
	if (!apos_gw_gauge_set()) {
		return -EINVAL;
	}

	apos_table_init(&tbl);
	memset(&res, 0, sizeof(res));
	have_result = false;
	accepted = false;
	solve_unverified = false;
	solve_redundancy = 0;
	solve_n_edges = 0;
	session = new_session();
	enum_round = 0;
	next_action_ms = 0;
	pair_idx = 0;
	pair_total = 0;
	pair_retries = 0;
	tx_fails = 0;
	awaiting_rsp = false;
	is_run = true;

	/* Starts in ENUM: a run always re-enumerates rather than trusting a
	 * table from an earlier `apos enum`, because a board may have been
	 * rebooted, re-addressed or swapped in between. The gauge survives that
	 * because it is stored as addresses, not indices. */
	phase = APOS_GW_ENUM;

	LOG_INF("{\"apos\":\"run\",\"session\":%u}", session);
	return 0;
}

/* Resolve the four gauge addresses to node indices in the freshly enumerated
 * table. Returns 0, or -ENOENT with the offending address logged. */
static int resolve_gauge(struct apos_gauge *g)
{
	uint8_t *out[4] = {&g->origin, &g->xaxis, &g->plane, &g->up};

	for (int k = 0; k < 4; k++) {
		int idx = apos_table_find_addr(&tbl, gauge_addr[k]);

		if (idx < 0) {
			LOG_ERR("{\"apos_error\":\"gauge address 0x%04X did not "
				"answer enumeration\"}", gauge_addr[k]);
			return -ENOENT;
		}
		*out[k] = (uint8_t)idx;
	}
	return 0;
}

static void judge_result(void)
{
	bool planar = res.planarity_m * 1000.0f <
		      (float)APOS_ACCEPT_PLANARITY_MM;

	accepted = res.n_placed == res.n_nodes &&
		   res.n_ambiguous == 0u &&
		   res.rms_m * 1000.0f < (float)APOS_ACCEPT_RMS_MM &&
		   res.worst_edge_m <= APOS_ACCEPT_WORST_FACTOR * res.rms_m +
				       (float)APOS_ACCEPT_RMS_MM / 1000.0f;

	LOG_INF("{\"apos_solve\":{\"nodes\":%u,\"placed\":%u,\"ambiguous\":%u,"
		"\"rms_mm\":%d,\"worst_mm\":%d,\"worst_pair\":[%u,%u],"
		"\"planarity_mm\":%d,\"iters\":%u,\"spare_edges\":%d,"
		"\"rms_meaningful\":%u,\"accepted\":%u}}",
		res.n_nodes, res.n_placed, res.n_ambiguous,
		(int)(res.rms_m * 1000.0f), (int)(res.worst_edge_m * 1000.0f),
		res.worst_i, res.worst_j,
		(int)(res.planarity_m * 1000.0f), res.iterations,
		solve_redundancy, solve_unverified ? 0u : 1u,
		accepted ? 1u : 0u);

	for (uint8_t k = 0; k < res.n_nodes; k++) {
		LOG_INF("{\"apos_node\":{\"idx\":%u,\"addr\":\"0x%04X\","
			"\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"state\":%u,"
			"\"resid_mm\":%d}}",
			k, tbl.peer[k].short_addr, (double)res.node[k].x,
			(double)res.node[k].y, (double)res.node[k].z,
			res.node[k].state,
			(int)(res.node[k].residual_m * 1000.0f));
	}

	/* The degenerate-redundancy case, said out loud. `accepted` is left
	 * exactly as the thresholds computed it -- weakening or inverting a
	 * threshold here would hide the condition rather than show it -- but an
	 * operator reading `"accepted":1` off the console has to be told that
	 * on this mesh the number it was computed from could not have come out
	 * any other way. See apos_geom.h's note on apos_geom_refine(). */
	if (solve_unverified) {
		LOG_WRN("{\"apos_warn\":\"rms_mm and worst_mm are NOT a quality "
			"check on this array: %u usable edge(s) against %d free "
			"parameters (3N-6) leaves %d spare. With no spare edge "
			"the fit reproduces ANY set of ranges exactly, so "
			"rms_mm is ~0 however bad the ranging was. "
			"\\\"accepted\\\":%u here means only that nothing "
			"contradicted the ranges — it does NOT mean they are "
			"correct. Check the solved node-to-node distances "
			"against a tape measure before `apos apply`, or add a "
			"fifth anchor so the mesh has a spare edge.\"}",
			solve_n_edges, 3 * (int)res.n_placed - 6,
			solve_redundancy,
			accepted ? 1u : 0u);
	}

	if (planar && res.n_placed >= 4u) {
		LOG_WRN("{\"apos_warn\":\"array is near-coplanar (%d mm) — x and "
			"y are good but the solved z values are not "
			"survey-quality\"}",
			(int)(res.planarity_m * 1000.0f));
	}
	if (res.n_ambiguous) {
		LOG_WRN("{\"apos_warn\":\"%u node(s) reflection-ambiguous — each "
			"needs a fourth measured edge or a repositioned "
			"anchor\"}", res.n_ambiguous);
	}
}

static void do_solve(void)
{
	struct apos_gauge g;
	struct apos_edge edges[APOS_MAX_EDGES];

	if (resolve_gauge(&g) != 0) {
		phase = APOS_GW_IDLE;
		return;
	}

	uint16_t n_edges = apos_table_symmetrise(&tbl, edges, APOS_MAX_EDGES,
						(uint8_t)APOS_MIN_N_OK);
	uint16_t missing = apos_table_missing_pairs(&tbl,
						   (uint8_t)APOS_MIN_N_OK);

	/* Logged, never silent: a hole changes what the fit rests on, and
	 * "covered everything" is exactly the wrong impression to leave. */
	LOG_INF("{\"apos_edges\":{\"usable\":%u,\"missing_pairs\":%u}}",
		n_edges, missing);

	int rc = apos_geom_solve(edges, n_edges, tbl.n_peers, &g, &res);

	if (rc == -ENODATA) {
		LOG_ERR("{\"apos_error\":\"the gauge anchors are not mutually "
			"ranged — move them into line of sight of each other "
			"and re-run\"}");
		phase = APOS_GW_IDLE;
		return;
	}
	if (rc) {
		LOG_ERR("{\"apos_error\":\"solve failed (errno %d)\"}", rc);
		phase = APOS_GW_IDLE;
		return;
	}

	if (zoff_m != 0.0f) {
		apos_geom_zoff(&res, zoff_m);
	}

	/* Redundancy of the fit that just ran: the gauge fixes 6 of the 3N
	 * degrees of freedom, so n_edges must EXCEED 3*n_placed - 6 before
	 * rms_m can disagree with anything. n_edges is an upper bound on the
	 * edges the fit actually used (edges touching an unplaced node are
	 * dropped), which makes this an optimistic estimate of the spare count
	 * -- and being optimistic about redundancy is the safe direction only
	 * for the <= 0 test, which is why the flag is set with >=-style
	 * conservatism below. */
	solve_n_edges = n_edges;
	solve_redundancy = (int16_t)((int)n_edges -
				     (3 * (int)res.n_placed - 6));
	solve_unverified = solve_redundancy <= 0;

	have_result = true;
	judge_result();
	phase = APOS_GW_IDLE;
}

static void on_range_rsp(const uint8_t *buf, uint16_t plen)
{
	uint16_t sess = 0, peer = 0, sd = 0;
	int32_t mean = 0;
	uint8_t n_ok = 0;

	if (apos_frame_parse_range_rsp(buf, plen, &sess, &peer, &mean, &sd,
				       &n_ok) != 0) {
		return;
	}
	if (sess != session || !awaiting_rsp) {
		return;
	}
	/* Both endpoints must match what we asked for. Without the peer check a
	 * late reply from the PREVIOUS pair would be recorded against this one,
	 * which is a wrong edge rather than a missing one -- far worse. */
	if (apos_frame_src(buf) != await_from_addr || peer != await_peer_addr) {
		return;
	}

	int from = apos_table_find_addr(&tbl, await_from_addr);
	int to = apos_table_find_addr(&tbl, await_peer_addr);

	if (n_ok < APOS_MIN_N_OK) {
		LOG_WRN("{\"apos_thin\":{\"from\":\"0x%04X\",\"to\":\"0x%04X\","
			"\"n_ok\":%u}}", await_from_addr, await_peer_addr, n_ok);
	} else if (from >= 0 && to >= 0) {
		apos_table_add_meas(&tbl, (uint8_t)from, (uint8_t)to, mean, sd,
				    n_ok);
	}

	awaiting_rsp = false;
	pair_retries = 0;
	tx_fails = 0;
	pair_idx++;
	next_action_ms = 0; /* next pair on the very next step */
}

/* Give up on the pair at the cursor and move on. A hole is a reported outcome,
 * not a failure of the run: the fit works around it and the operator gets to
 * see exactly which pair could not range. */
static void retire_pair(const char *why)
{
	LOG_WRN("{\"apos_hole\":{\"from\":\"0x%04X\",\"to\":\"0x%04X\","
		"\"why\":\"%s\"}}", await_from_addr, await_peer_addr, why);
	awaiting_rsp = false;
	pair_retries = 0;
	tx_fails = 0;
	pair_idx++;
}

static void step_range(uint32_t avail_uus, uint8_t *seq)
{
	int64_t now = k_uptime_get();

	if (awaiting_rsp) {
		if (now < next_action_ms) {
			return;
		}
		/* Timed out. */
		if (pair_retries < APOS_GW_RANGE_RETRIES) {
			pair_retries++;
			awaiting_rsp = false;
			LOG_WRN("{\"apos_retry\":{\"from\":\"0x%04X\","
				"\"to\":\"0x%04X\"}}", await_from_addr,
				await_peer_addr);
		} else {
			retire_pair("no RANGE_RSP");
		}
		return;
	}

	if (pair_idx >= pair_total) {
		/* The one step that transmits nothing and cannot be bounded in
		 * microseconds. It runs only with a full solve budget in hand;
		 * otherwise it waits for the next superframe, which costs
		 * 200 ms of latency once per run and protects the beacon. */
		if (avail_uus < APOS_GW_SOLVE_BUDGET_UUS) {
			return;
		}
		do_solve();
		return;
	}

	uint8_t from, to;

	pair_of(pair_idx, tbl.n_peers, &from, &to);
	await_from_addr = tbl.peer[from].short_addr;
	await_peer_addr = tbl.peer[to].short_addr;

	int n = apos_frame_range_cmd_build(tx_buf, sizeof(tx_buf),
					  UWB_ADDR_GATEWAY_RESERVED,
					  await_from_addr, session,
					  await_peer_addr,
					  (uint8_t)APOS_GW_N_EXCHANGES);

	if (n < 0) {
		LOG_ERR("RANGE_CMD build failed (%d)", n);
		phase = APOS_GW_IDLE;
		return;
	}
	if (!tx_now((uint16_t)n, seq)) {
		/* The command never left. Retry on the next step rather than
		 * counting it against this pair's retry budget -- but only up
		 * to APOS_GW_TX_FAIL_LIMIT times, or a wedged radio would park
		 * the survey on this pair forever with pair_idx never
		 * advancing and nothing on the console saying why. */
		if (++tx_fails >= APOS_GW_TX_FAIL_LIMIT) {
			retire_pair("RANGE_CMD would not transmit");
		}
		return;
	}
	tx_fails = 0;

	awaiting_rsp = true;
	next_action_ms = now + APOS_GW_RANGE_TIMEOUT_MS;
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
	case APOS_SUB_RANGE_RSP:
		on_range_rsp(buf, plen);
		break;
	default:
		/* SETPOS_ACK arrives in Task 12. */
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

		/* An explicit flag, NOT apos_gw_gauge_set(). The gauge is
		 * sticky module state: an operator who sets the gauge and then
		 * runs a bare `apos enum` to re-read the addresses would,
		 * gating on the gauge, silently get a full multi-minute
		 * ranging run they never asked for. Whether this is a run is a
		 * property of which command started it. */
		if (!is_run) {
			phase = APOS_GW_IDLE;
			return;
		}
		if (tbl.n_peers < APOS_MIN_NODES) {
			LOG_ERR("{\"apos_error\":\"%u anchor(s) answered; a 3D "
				"gauge needs at least %u\"}",
				tbl.n_peers, APOS_MIN_NODES);
			phase = APOS_GW_IDLE;
			return;
		}

		pair_total = (uint16_t)tbl.n_peers *
			     (uint16_t)(tbl.n_peers - 1u);
		pair_idx = 0;
		pair_retries = 0;
		tx_fails = 0;
		awaiting_rsp = false;
		next_action_ms = 0;
		phase = APOS_GW_RANGE;
		LOG_INF("{\"apos\":\"ranging\",\"ordered_pairs\":%u}",
			pair_total);
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
	case APOS_GW_RANGE:
		step_range(avail_uus, seq);
		break;
	default:
		/* APOS_GW_APPLY is Task 12's to implement. Unreachable until it
		 * sets that phase --
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
