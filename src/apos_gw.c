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

/* APPLY phase cursor. */
static uint8_t apply_idx;
static uint8_t apply_retries;
static uint8_t applied_ok;
static uint8_t applied_fail;
static uint8_t applied_skip;
static bool apply_ending;   /* all nodes done; SURVEY_END still to send */
static bool apply_persisted;

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
	apply_idx = 0;
	apply_retries = 0;
	applied_ok = 0;
	applied_fail = 0;
	applied_skip = 0;
	apply_ending = false;
	apply_persisted = false;
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
	out->applied_ok = applied_ok;
	out->applied_fail = applied_fail;
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

	/* Phase, not just session: the session is unchanged across a whole run,
	 * so a straggler ENUM_RSP arriving during RANGE would otherwise be
	 * accepted. That appends a peer and bumps tbl.n_peers while pair_total
	 * stays frozen at the old N -- pair_of() then decomposes with the new
	 * per_row against the old total, revisiting some ordered pairs, never
	 * commanding others, and dropping the new node into the solve with no
	 * edges at all. It would also let the -EADDRINUSE branch below abort a
	 * run in progress with a message about enumeration, which by then is not
	 * even true. Late enumeration replies belong to ENUM or to nothing. */
	if (phase != APOS_GW_ENUM) {
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

/* Returns a pointer into live module storage, unsynchronised, while the gateway
 * loop may be rewriting it in do_solve(). That is safe TODAY for one reason
 * only: the gateway loop runs at K_PRIO_COOP(0), so no lower-priority thread
 * -- the shell included -- can be scheduled at any instant between do_solve()'s
 * first and last write to `res`. A reader therefore never observes a half-built
 * result; it sees the previous one or the next one.
 *
 * That assumption breaks the moment a caller is NOT strictly lower priority than
 * the gateway loop: another cooperative thread at priority 0, anything
 * preemptible above it, or an ISR. Any such reader needs a copy taken under a
 * lock, or `res` double-buffered with the published pointer swapped at the end
 * of do_solve(). Task 12's `apos apply` reads this from the gateway loop itself,
 * which is fine; a future MQTT or web reader would not be. */
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
		/* Deliberately TWO messages, not one. Deferred logging formats
		 * into a shared CONFIG_LOG_BUFFER_SIZE pool (1024 B by default,
		 * and prj.conf does not raise it), and a single ~550-byte
		 * message is both the most droppable in this file under a burst
		 * and the one an operator least tolerates losing. Two ~280-byte
		 * messages fit far more comfortably, and if the second is ever
		 * dropped the first still carries the finding. */
		LOG_WRN("{\"apos_warn\":\"rms_mm and worst_mm are NOT a quality "
			"check on this array: %u usable edge(s) against %d free "
			"parameters (3N-6) leaves %d spare. With no spare edge "
			"the fit reproduces ANY set of ranges exactly, so "
			"rms_mm is ~0 however bad the ranging was.\"}",
			solve_n_edges, 3 * (int)res.n_placed - 6,
			solve_redundancy);
		LOG_WRN("{\"apos_warn\":\"\\\"accepted\\\":%u above therefore "
			"means only that nothing contradicted the ranges — it "
			"does NOT mean they are correct. Check the solved "
			"node-to-node distances against a tape measure before "
			"`apos apply`, or add a fifth anchor so the mesh has a "
			"spare edge.\"}", accepted ? 1u : 0u);
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
	 * dropped), so this is an OPTIMISTIC estimate of the spare count: the
	 * fit may have had fewer spare edges than this says, never more. That is
	 * why the flag below triggers on <= 0 rather than < 0 -- an optimistic
	 * count of exactly zero spare edges is already the degenerate case, and
	 * anything this overestimates only moves further into it. */
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
		 * microseconds. APOS_GW_SOLVE_BUDGET_UUS is sized so only the
		 * first step after a beacon can satisfy this test, giving the
		 * solve most of a superframe to run in. Waiting for that costs
		 * up to 200 ms of latency once per run, and protects the
		 * beacon. */
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

/* ---- APPLY phase ---- */

int apos_gw_start_apply(bool force)
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}
	if (!have_result) {
		return -ENODATA;
	}
	if (!accepted && !force) {
		return -EPERM;
	}

	apply_idx = 0;
	apply_retries = 0;
	applied_ok = 0;
	applied_fail = 0;
	applied_skip = 0;
	apply_ending = false;
	apply_persisted = false;
	awaiting_rsp = false;
	tx_fails = 0;
	next_action_ms = 0;
	phase = APOS_GW_APPLY;

	LOG_INF("{\"apos\":\"applying\",\"nodes\":%u,\"forced\":%u}",
		res.n_nodes, force ? 1u : 0u);

	/* Repeated HERE, not only at solve time. Apply is the point of no
	 * return -- it rewrites every anchor's NVS and this gateway's survey --
	 * and the operator typing it may have read `"accepted":1` minutes ago.
	 * `force` deliberately does not gate this: the condition is a property
	 * of the mesh, not a decision, and it is loudest precisely when the
	 * result looks best. */
	if (solve_unverified) {
		LOG_WRN("{\"apos_warn\":\"COMMITTING AN UNVERIFIED SURVEY: %u "
			"usable edge(s) against %d free parameters (3N-6), "
			"leaving %d spare edge(s), so rms_mm was ~0 however bad "
			"the ranging was.\"}", solve_n_edges,
			3 * (int)res.n_placed - 6, solve_redundancy);
		LOG_WRN("{\"apos_warn\":\"on this array \\\"accepted\\\" means "
			"only that nothing contradicted the ranges — it does "
			"NOT mean they are correct. These coordinates are about "
			"to be written to every anchor's NVS.\"}");
	}
	return 0;
}

/* Persist the survey locally, with the gauge origin first. Returns true only if
 * the survey is actually on flash.
 *
 * node[0] must be the origin: apos_store.h documents that the geographic
 * reference belongs to it, and Task 13's rewrite of pos_json.c -- which
 * replaces today's stub anchors payload with the real survey -- will read
 * node[0] as the reference the platform places the map against. The solver's
 * node order is the order anchors answered enumeration in, which is arbitrary,
 * so the origin is moved to the front here rather than being assumed. */
static bool persist_survey(void)
{
	struct apos_survey s;
	struct apos_gauge g;

	memset(&s, 0, sizeof(s));

	if (resolve_gauge(&g) != 0) {
		LOG_ERR("{\"apos_error\":\"cannot persist — gauge no longer "
			"resolves\"}");
		return false;
	}

	/* The ordering contract above is only satisfiable if the origin is in
	 * the output at all. An unplaced origin should be impossible -- the
	 * gauge is placed by construction -- but writing node[0] as some other
	 * anchor while pos_json still reads it as the geographic reference is a
	 * silent wrong-coordinate failure, which is the exact class of bug this
	 * feature exists to remove. Refuse instead. */
	if (res.node[g.origin].state == APOS_NODE_UNPLACED) {
		LOG_ERR("{\"apos_error\":\"cannot persist — the gauge origin "
			"0x%04X was not placed by the solve\"}",
			tbl.peer[g.origin].short_addr);
		return false;
	}

	uint8_t w = 0;

	/* Origin first, then everything else in solver order. */
	for (uint8_t pass = 0; pass < 2; pass++) {
		for (uint8_t k = 0; k < res.n_nodes && w < APOS_MAX_NODES; k++) {
			bool is_origin = (k == g.origin);

			if ((pass == 0) != is_origin) {
				continue;
			}
			if (res.node[k].state == APOS_NODE_UNPLACED) {
				continue;
			}
			memcpy(s.node[w].eui, tbl.peer[k].eui, APOS_EUI_LEN);
			s.node[w].short_addr = tbl.peer[k].short_addr;
			s.node[w].x = res.node[k].x;
			s.node[w].y = res.node[k].y;
			s.node[w].z = res.node[k].z;
			w++;
		}
	}

	s.n_nodes = w;
	s.valid = w > 0u;

	int rc = apos_store_save(&s);

	if (rc) {
		LOG_ERR("{\"apos_error\":\"survey applied to the anchors but NOT "
			"persisted on the gateway (errno %d) — the anchors "
			"topic will fall back to the stub after a reboot\"}",
			rc);
		return false;
	}

	LOG_INF("{\"apos_store\":\"survey saved\",\"nodes\":%u}", w);
	return true;
}

static void on_setpos_ack(const uint8_t *buf, uint16_t plen)
{
	uint16_t sess = 0;
	float x = 0.0f, y = 0.0f, z = 0.0f;
	bool ok = false;

	if (apos_frame_parse_setpos_ack(buf, plen, &sess, &x, &y, &z, &ok)
	    != 0) {
		return;
	}
	if (sess != session || !awaiting_rsp) {
		return;
	}
	/* Phase, not just session and awaiting_rsp: the session is unchanged
	 * across a whole run, and RANGE also sets awaiting_rsp. A SETPOS_ACK
	 * arriving then -- a duplicate from a previous apply against the same
	 * session -- would otherwise advance the RANGE cursor past a pair that
	 * was never measured. Same reasoning as on_enum_rsp(). */
	if (phase != APOS_GW_APPLY) {
		return;
	}
	if (apos_frame_src(buf) != await_from_addr) {
		return;
	}

	if (ok) {
		applied_ok++;
		LOG_INF("{\"apos_applied\":{\"addr\":\"0x%04X\",\"x\":%.3f,"
			"\"y\":%.3f,\"z\":%.3f}}", await_from_addr,
			(double)x, (double)y, (double)z);
	} else {
		/* The anchor applied it but could not persist it. Counted as a
		 * failure: it will silently revert on its next reboot, which is
		 * exactly the kind of half-applied state worth being loud
		 * about. */
		applied_fail++;
		LOG_ERR("{\"apos_error\":\"0x%04X applied but could not persist "
			"— it will revert on reboot\"}", await_from_addr);
	}

	awaiting_rsp = false;
	apply_retries = 0;
	tx_fails = 0;
	apply_idx++;
	next_action_ms = 0;
}

static void step_apply(uint32_t avail_uus, uint8_t *seq)
{
	int64_t now = k_uptime_get();

	if (awaiting_rsp) {
		if (now < next_action_ms) {
			return;
		}
		if (apply_retries < APOS_GW_APPLY_RETRIES) {
			apply_retries++;
			awaiting_rsp = false;
			LOG_WRN("{\"apos_retry\":{\"setpos\":\"0x%04X\","
				"\"attempt\":%u}}", await_from_addr,
				apply_retries);
		} else {
			applied_fail++;
			LOG_ERR("{\"apos_error\":\"0x%04X never acknowledged "
				"SETPOS after %u attempts — it is still on its "
				"OLD coordinates\"}", await_from_addr,
				APOS_GW_APPLY_RETRIES + 1u);
			awaiting_rsp = false;
			apply_retries = 0;
			tx_fails = 0;
			apply_idx++;
		}
		return;
	}

	if (apply_ending) {
		int n = apos_frame_survey_end_build(tx_buf, sizeof(tx_buf),
						  UWB_ADDR_GATEWAY_RESERVED,
						  session);

		if (n < 0) {
			LOG_ERR("SURVEY_END build failed (%d)", n);
		} else if (!tx_now((uint16_t)n, seq)) {
			/* Not retried: the windows the run opened close on
			 * their own after APOS_NODE_REFRESH_S with nothing
			 * refreshing them, so this costs a minute of an open
			 * window, not a stuck deployment. Retrying instead
			 * would keep the survey busy on a wedged radio. */
			LOG_WRN("{\"apos_warn\":\"SURVEY_END did not transmit — "
				"the anchors' survey windows will lapse on "
				"their own instead\"}");
		}

		/* "persisted" is part of the summary, not left to the error
		 * logged inside persist_survey(): the summary is the last line
		 * an operator reads off a scrolling monitor, and anchors that
		 * all took their coordinates while the GATEWAY has no survey is
		 * the exact split this feature exists to remove -- after a
		 * reboot the anchors topic falls back to the stub and the
		 * platform map disagrees with the solver. */
		LOG_INF("{\"apos_apply_done\":{\"ok\":%u,\"failed\":%u,"
			"\"skipped\":%u,\"nodes\":%u,\"persisted\":%u}}",
			applied_ok, applied_fail, applied_skip, res.n_nodes,
			apply_persisted ? 1u : 0u);
		if (applied_fail || applied_skip) {
			LOG_ERR("{\"apos_error\":\"%u anchor(s) did NOT take the "
				"new coordinates — the deployment is "
				"inconsistent. Re-run `apos apply`.\"}",
				(unsigned int)(applied_fail + applied_skip));
		}
		if (!apply_persisted) {
			LOG_ERR("{\"apos_error\":\"the anchors were updated but "
				"the gateway did NOT persist the survey — it "
				"will publish the STUB anchors payload after a "
				"reboot and the map will disagree with the "
				"solver. Re-run `apos apply`.\"}");
		}
		apply_ending = false;
		phase = APOS_GW_IDLE;
		return;
	}

	/* Skip unplaced nodes: there is nothing to push, and pushing (0, 0, 0)
	 * would be exactly the silent-wrong-coordinate failure this whole
	 * design exists to remove. Counted, so the summary above cannot report
	 * a clean `failed:0` for an apply that only covered part of the
	 * array. */
	while (apply_idx < res.n_nodes &&
	       res.node[apply_idx].state == APOS_NODE_UNPLACED) {
		LOG_WRN("{\"apos_skip\":{\"addr\":\"0x%04X\",\"reason\":"
			"\"unplaced\"}}", tbl.peer[apply_idx].short_addr);
		applied_skip++;
		apply_idx++;
	}

	if (apply_idx >= res.n_nodes) {
		/* Persist before SURVEY_END, so a gateway that dies between the
		 * two still has the survey on flash.
		 *
		 * Gated on the SOLVE budget, not the step budget, for the same
		 * reason the solve is -- and the reason is stronger here,
		 * because it is not about thread priority at all. This is a
		 * WROOM-1-N8R2 executing XIP from the same flash apos_store
		 * writes to: any write or erase disables the instruction cache
		 * and stalls ALL code execution for its duration. K_PRIO_COOP(0)
		 * does not protect the gateway loop from that and nothing else
		 * can either. A settings append is sub-millisecond of
		 * programming, but an NVS sector rotation erases 4 kB at ~25-45
		 * ms -- 5-9x BEACON_ARM_MARGIN_UUS. The beacon then arms late,
		 * its delayed TX times out waiting for TXFRS, beacon_tx_ts is
		 * left unadvanced, and every slave's beacon_guard starts
		 * suppressing against a schedule that has shifted underneath it.
		 * That is a network-wide timing fault, not a dropped frame.
		 *
		 * APOS_GW_SOLVE_BUDGET_UUS (150000, ~154 ms) is reused rather
		 * than a new constant: it is already sized so that only the
		 * first step after a beacon can satisfy it, which is exactly the
		 * ~150 ms of clear air this write wants. That covers a typical
		 * GC erase with room to spare; it does NOT cover a pathological
		 * one, and no in-loop gate can. The fully correct fix is to stop
		 * writing flash from this thread at all -- hand the persist to a
		 * preemptible worker and poll for its completion, the same
		 * remedy apos_gw.h names for the solve. */
		if (avail_uus < APOS_GW_SOLVE_BUDGET_UUS) {
			return;
		}
		apply_persisted = persist_survey();
		apply_ending = true;
		return;
	}

	await_from_addr = tbl.peer[apply_idx].short_addr;

	int n = apos_frame_setpos_build(tx_buf, sizeof(tx_buf),
				       UWB_ADDR_GATEWAY_RESERVED,
				       await_from_addr, session,
				       res.node[apply_idx].x,
				       res.node[apply_idx].y,
				       res.node[apply_idx].z);

	if (n < 0) {
		LOG_ERR("SETPOS build failed (%d)", n);
		phase = APOS_GW_IDLE;
		return;
	}
	if (!tx_now((uint16_t)n, seq)) {
		/* Bounded exactly as step_range()'s RANGE_CMD is: a TX that
		 * never starts is not this anchor's fault and does not consume
		 * its retry budget, but a wedged radio must not park APPLY on
		 * one anchor forever with apply_idx never advancing and nothing
		 * on the console saying why. */
		if (++tx_fails >= APOS_GW_TX_FAIL_LIMIT) {
			applied_fail++;
			LOG_ERR("{\"apos_error\":\"SETPOS for 0x%04X would not "
				"transmit — it is still on its OLD "
				"coordinates\"}", await_from_addr);
			tx_fails = 0;
			apply_retries = 0;
			apply_idx++;
		}
		return;
	}
	tx_fails = 0;

	awaiting_rsp = true;
	next_action_ms = now + APOS_GW_APPLY_TIMEOUT_MS;
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
	case APOS_SUB_SETPOS_ACK:
		on_setpos_ack(buf, plen);
		break;
	default:
		/* The remaining subtypes are gateway-to-anchor commands; a
		 * board echoing one back is not this module's business. */
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
	case APOS_GW_APPLY:
		step_apply(avail_uus, seq);
		break;
	default:
		/* Every value of enum apos_gw_phase now has a handler, so this
		 * is unreachable through the normal paths -- but `phase` is a
		 * uint8_t, not the enum, so the compiler does not prove that,
		 * and the arm is kept as the guard it was written to be:
		 * a phase with no handler means apos_gw_busy() stays true
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
