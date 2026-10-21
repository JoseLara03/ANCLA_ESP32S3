/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_gw.h.
 */

#include "apos_gw.h"

#include "apos_addrmap.h"
#include "apos_frame.h"
#include "apos_node.h"    /* APOS_MIN_N_OK, APOS_ENUM_* for the gap derivation */
#include "apos_stats.h"
#include "apos_store.h"
#include "gw_selftau.h"   /* GW_SELFTAU_STEP_BUDGET_UUS */
#include "node_eui.h"
#include "ss_initiator.h"
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
static int32_t gauge_addr[4]; /* gauge_addr[3] (up) == -1 means 2D mode */

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
static bool apply_persisted;

/* Result judgement, filled by do_solve()/judge_result(). */
static bool accepted;
static int16_t solve_redundancy; /* see apos_gw_result_redundancy() */
static uint16_t solve_n_edges;
static bool solve_unverified;

/* Ranging-quality numbers derived from the raw directed measurements, not from
 * the fit. On a 3D gauge over the real four-anchor array the fit's rms_m is
 * vacuous (see apos_gw_result_unverified()) -- a rigid 4-node framework stays
 * isostatic regardless of how good its edges are -- so these are the only
 * quality signals that actually carry information about the RANGING there.
 * A 2D (3-anchor) gauge run over the same four anchors is NOT that case: it
 * has one spare edge (2*4-3 = 5 free parameters against 6 edges), so rms_m
 * becomes a real, if weak, signal for the first time. Either way these two
 * numbers say nothing about whether the GEOMETRY is over-determined. */
static int32_t max_recip_mm;
static uint16_t max_sd_mm;
/* ---- B14 static survey ---------------------------------------------------
 *
 * The gateway is a node of a 3D survey. gw_idx is its index in `tbl` (-1 when
 * the survey does not include it: a 2D survey, or one not yet through
 * enumeration). Everything below is touched only from the gateway loop except
 * where a comment says otherwise. */
static int8_t gw_idx = -1;
static uint8_t gw_eui[APOS_EUI_LEN];
static bool gw_eui_ready;

/* Gateway -> anchor ranging, run locally. One pair at a time. */
static int32_t local_s[APOS_STATS_MAX];
static uint8_t local_n_try;
static uint8_t local_n_ok;

/* Judgement of the last 3D solve. */
static struct apos_gauge_quality gq;
static bool gq_valid;
static bool gauge_degenerate;
static struct sync_link links[SYNC_LINK_MAX];
static uint8_t n_links;
static bool links_verified;

/* Apply bookkeeping beyond what the SETPOS loop already has. */
enum apply_stage {
	AP_MARK    = 0, /* persist the apply-in-progress marker, hold fixes */
	AP_REOPEN  = 1, /* one-shot SURVEY_BEGIN so every window is open */
	AP_SETPOS  = 2,
	AP_SETTAU  = 3,
	AP_COMMIT  = 4, /* persist survey + links, release the hold */
	AP_END     = 5, /* SURVEY_END and summary */
};
static uint8_t apply_stage;
static bool apply_with_links;     /* this apply installs links (3D + gateway) */
static uint32_t apply_generation; /* the generation this apply installs */
static uint32_t applied_generation; /* last one fully installed */
static uint8_t tau_ok;

/* Fix-publication hold. See apos_gw_publish_hold(). volatile: written on the
 * gateway loop, read by tdoa_gw on the same loop, but also by the shell. */
static volatile bool hold;
static const char *hold_reason = "none";

static uint8_t tx_buf[APOS_LEN_MAX];

/* ---- `apos assign` (spec 2026-10-21 D7) ---------------------------------- */

_Static_assert(APOS_GW_ASSIGN_CONVERGE_MS < APOS_GW_WINDOW_S * 1000u,
	       "assign convergence must finish inside the survey window");

enum assign_stage {
	AS_ENUM     = 0, /* APOS_GW_ENUM_ROUNDS of SURVEY_BEGIN, then plan */
	AS_SEND     = 1, /* one SETADDR per ASSIGN, retried on a missing ACK */
	AS_CONVERGE = 2, /* re-enumerate until every board shows its new address */
	AS_PERSIST  = 3, /* budget-gated write of the map (+ survey addresses) */
};
static uint8_t assign_stage;
static uint8_t plan_n;
static struct apos_plan_peer plan_peer[APOS_MAX_NODES];
static struct apos_plan_out plan_out[APOS_MAX_NODES];
static bool plan_acked[APOS_MAX_NODES];
static bool plan_nacked[APOS_MAX_NODES];  /* refused, or never transmitted */
static struct apos_addrmap forget_map;     /* queued by apos_gw_forget() */
static uint8_t assign_idx;
static uint8_t assign_failed;
static uint8_t assign_acked;
static int64_t converge_deadline_ms;

void apos_gw_init(void)
{
	apos_table_init(&tbl);
	memset(&res, 0, sizeof(res));
	memset(gauge_addr, 0, sizeof(gauge_addr));
	gauge_addr[3] = -1; /* up unset means 2D, not "address 0x0000" */
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
	apply_persisted = false;
	max_recip_mm = 0;
	max_sd_mm = 0;

	gw_idx = -1;
	local_n_try = 0;
	local_n_ok = 0;
	gq_valid = false;
	gauge_degenerate = false;
	n_links = 0;
	links_verified = false;
	apply_stage = AP_MARK;
	apply_with_links = false;
	apply_generation = 0;
	tau_ok = 0;
	applied_generation = apos_store_links()->generation;
	assign_stage = AS_ENUM;
	plan_n = 0;

	/* This board's own identity, from the same source apos_node.c hashes
	 * for its stagger (node_eui owns the read and the zero padding). */
	memcpy(gw_eui, node_eui(), sizeof(gw_eui));
	gw_eui_ready = node_eui_valid();
	if (!gw_eui_ready) {
		LOG_ERR("{\"apos_error\":\"no valid EUI — this gateway cannot be "
			"a node of its own survey\"}");
	}

	/* An apply that was started and never finished leaves the anchors holding
	 * a mix of generations. The marker was persisted BEFORE the first anchor
	 * was touched, so finding it here -- or finding one that cannot be read --
	 * means fixes must stay suspended until an apply completes. */
	hold = false;
	hold_reason = "none";
	if (apos_store_apply_pending()) {
		hold = true;
		hold_reason = "interrupted-apply";
		LOG_ERR("{\"apos_error\":\"an `apos apply` (generation %u) was "
			"started and never finished — the anchors may hold a mix "
			"of survey generations. Position fixes are SUSPENDED. "
			"Re-run `apos run` and `apos apply`.\"}",
			(unsigned int)apos_store_pending_generation());
	}
}

bool apos_gw_publish_hold(void)
{
	return hold;
}

const char *apos_gw_hold_reason(void)
{
	return hold_reason;
}

const uint8_t *apos_gw_eui(void)
{
	return gw_eui_ready ? gw_eui : NULL;
}

bool apos_gw_survey_has_gateway(void)
{
	return gw_idx >= 0;
}

uint8_t apos_gw_links(const struct sync_link **out)
{
	if (out) {
		*out = links;
	}
	return n_links;
}

bool apos_gw_gauge_quality(struct apos_gauge_quality *out)
{
	if (!gq_valid) {
		return false;
	}
	if (out) {
		*out = gq;
	}
	return true;
}

bool apos_gw_gauge_degenerate(void)
{
	return gauge_degenerate;
}

bool apos_gw_links_verified(void)
{
	return links_verified;
}

uint32_t apos_gw_generation(void)
{
	return applied_generation;
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
	return gauge_addr[0] != 0 && gauge_addr[1] != 0 && gauge_addr[2] != 0;
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
	out->gateway_node = gw_idx >= 0;
	out->hold = hold;
}

int apos_gw_set_gauge(uint16_t origin, uint16_t xaxis, uint16_t plane,
		      int32_t up)
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}

	const int32_t a[3] = {origin, xaxis, plane};

	for (int i = 0; i < 3; i++) {
		if (a[i] == 0 || a[i] == (int32_t)UWB_ADDR_GATEWAY_RESERVED) {
			return -EINVAL;
		}
		for (int j = i + 1; j < 3; j++) {
			if (a[i] == a[j]) {
				return -EINVAL;
			}
		}
	}
	if (up != -1) {
		if (up == 0 || up == (int32_t)UWB_ADDR_GATEWAY_RESERVED) {
			return -EINVAL;
		}
		for (int i = 0; i < 3; i++) {
			if (up == a[i]) {
				return -EINVAL;
			}
		}
	}

	gauge_addr[0] = a[0];
	gauge_addr[1] = a[1];
	gauge_addr[2] = a[2];
	gauge_addr[3] = up;
	return 0;
}

enum apos_geom_dim apos_gw_gauge_dim(void)
{
	return (gauge_addr[3] == -1) ? APOS_GEOM_2D : APOS_GEOM_3D;
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

/* Everything a fresh enumeration or run must forget about a previous one, so a
 * bare `apos enum` after an aborted run cannot inherit a stale gateway node,
 * link set or gauge verdict. */
static void reset_static_state(void)
{
	gw_idx = -1;
	local_n_try = 0;
	local_n_ok = 0;
	gq_valid = false;
	gauge_degenerate = false;
	n_links = 0;
	links_verified = false;
	tau_ok = 0;
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
	reset_static_state();

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
	 * edges at all. Late enumeration replies belong to ENUM, or to the
	 * enumerating stages of ASSIGN (its first rounds and the convergence
	 * re-enumeration, which is how a re-addressed board is seen to come
	 * back at its new address), or to nothing. */
	bool assign_listening = (phase == APOS_GW_ASSIGN &&
				 (assign_stage == AS_ENUM ||
				  assign_stage == AS_CONVERGE));

	if (phase != APOS_GW_ENUM && !assign_listening) {
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

	/* Two boards on one address (the same `anchor id`, or both UNASSIGNED)
	 * are both kept, keyed by EUI, and flagged addr_conflict. A run refuses
	 * on that when enumeration closes; `apos assign` re-addresses them. */
	int idx = apos_table_add_peer(&tbl, eui, addr, pv, x, y, z);

	if (idx == -ENOSPC) {
		LOG_WRN("more than %u anchors answered — ignoring 0x%04X",
			APOS_MAX_NODES, addr);
		return;
	}
	if (idx < 0) {
		return;
	}

	char hex[NODE_EUI_HEX_LEN + 1u];

	node_eui_fmt(eui, hex);
	LOG_INF("{\"apos_peer\":{\"idx\":%d,\"id\":%d,\"addr\":\"0x%04X\","
		"\"eui\":\"%s\",\"conflict\":%u,\"pos_valid\":%u,"
		"\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
		idx, (addr == UWB_ANCHOR_ADDR_UNASSIGNED)
		     ? -1 : (int)(addr - UWB_ANCHOR_ADDR_BASE),
		addr, hex, tbl.peer[idx].addr_conflict ? 1u : 0u,
		pv ? 1u : 0u, (double)x, (double)y, (double)z);
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

void apos_gw_result_quality(int32_t *out_recip_mm, uint16_t *out_sd_mm)
{
	if (out_recip_mm) {
		*out_recip_mm = max_recip_mm;
	}
	if (out_sd_mm) {
		*out_sd_mm = max_sd_mm;
	}
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
	reset_static_state();
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

/* Resolve the gauge addresses -- origin/xaxis/plane always, plus up when the
 * gauge is 3D -- to node indices in the freshly enumerated table. Returns 0,
 * or -ENOENT with the offending address logged. */
static int resolve_gauge(struct apos_gauge *g)
{
	uint8_t *out3[3] = {&g->origin, &g->xaxis, &g->plane};

	for (int k = 0; k < 3; k++) {
		int idx = apos_table_find_addr(&tbl, (uint16_t)gauge_addr[k]);

		if (idx < 0) {
			LOG_ERR("{\"apos_error\":\"gauge address 0x%04X did "
				"not answer enumeration\"}",
				(uint16_t)gauge_addr[k]);
			return -ENOENT;
		}
		*out3[k] = (uint8_t)idx;
	}

	if (gauge_addr[3] == -1) {
		g->dim = APOS_GEOM_2D;
		/* Never read by apos_geom_gauge_valid()/apos_geom_seed() in
		 * 2D mode, but must still be a valid index (< n_nodes) since
		 * struct apos_gauge carries no separate "up is absent" bit at
		 * this layer -- see apos_geom.h. g->origin is always valid. */
		g->up = g->origin;
		return 0;
	}

	int idx = apos_table_find_addr(&tbl, (uint16_t)gauge_addr[3]);

	if (idx < 0) {
		LOG_ERR("{\"apos_error\":\"gauge address 0x%04X did not "
			"answer enumeration\"}", (uint16_t)gauge_addr[3]);
		return -ENOENT;
	}
	g->up = (uint8_t)idx;
	g->dim = APOS_GEOM_3D;
	return 0;
}

static void judge_result(void)
{
	bool planar = res.planarity_m * 1000.0f <
		      (float)APOS_ACCEPT_PLANARITY_MM;
	bool gauge_thin = res.dim == APOS_GEOM_2D &&
			  res.gauge_collinearity_ratio <
				  APOS_ACCEPT_GAUGE_COLLINEARITY;

	accepted = res.n_placed == res.n_nodes &&
		   res.n_ambiguous == 0u &&
		   res.rms_m * 1000.0f < (float)APOS_ACCEPT_RMS_MM &&
		   res.worst_edge_m <= APOS_ACCEPT_WORST_FACTOR * res.rms_m +
				       (float)APOS_ACCEPT_RMS_MM / 1000.0f;

	/* The static install adds two conditions. A degenerate gauge is judged
	 * here too so `accepted` never reads 1 for it, but it is ALSO a hard
	 * refusal that `force` cannot override (apos_gw_start_apply()). Links
	 * that are unchecked or outside the gate fail acceptance and CAN be
	 * forced, like every other acceptance threshold. */
	if (res.dim == APOS_GEOM_3D && gw_idx >= 0) {
		accepted = accepted && !gauge_degenerate && links_verified;
	}

	LOG_INF("{\"apos_solve\":{\"nodes\":%u,\"placed\":%u,\"ambiguous\":%u,"
		"\"rms_mm\":%d,\"worst_mm\":%d,\"worst_pair\":[%u,%u],"
		"\"planarity_mm\":%d,\"gauge_collinearity_permille\":%d,"
		"\"iters\":%u,\"spare_edges\":%d,"
		"\"rms_meaningful\":%u,\"max_reciprocal_mm\":%d,"
		"\"max_sd_mm\":%u,\"accepted\":%u}}",
		res.n_nodes, res.n_placed, res.n_ambiguous,
		(int)(res.rms_m * 1000.0f), (int)(res.worst_edge_m * 1000.0f),
		res.worst_i, res.worst_j,
		(int)(res.planarity_m * 1000.0f),
		(int)(res.gauge_collinearity_ratio * 1000.0f), res.iterations,
		solve_redundancy, solve_unverified ? 0u : 1u,
		max_recip_mm, max_sd_mm, accepted ? 1u : 0u);

	for (uint8_t k = 0; k < res.n_nodes; k++) {
		LOG_INF("{\"apos_node\":{\"idx\":%u,\"addr\":\"0x%04X\","
			"\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"state\":%u,"
			"\"resid_mm\":%d}}",
			k, tbl.peer[k].short_addr, (double)res.node[k].x,
			(double)res.node[k].y, (double)res.node[k].z,
			res.node[k].state,
			(int)(res.node[k].residual_m * 1000.0f));
	}
	if (res.dim == APOS_GEOM_3D && gw_idx >= 0) {
		if (gq_valid) {
			LOG_INF("{\"apos_gauge\":{\"up_height_mm\":%d,"
				"\"height_floor_mm\":%d,\"collinearity_permille\":%d,"
				"\"edge_rms_mm\":%d,\"coplanar\":%u,\"collinear\":%u}}",
				(int)(gq.up_height_m * 1000.0f),
				(int)(gq.height_floor_m * 1000.0f),
				(int)(gq.collinearity * 1000.0f),
				(int)(gq.edge_rms_m * 1000.0f),
				gq.coplanar ? 1u : 0u, gq.collinear ? 1u : 0u);
		}
		for (uint8_t k = 0; k < n_links; k++) {
			LOG_INF("{\"apos_link\":{\"child\":\"0x%04X\",\"tau_dtu\":%d,"
				"\"geom_mm\":%d,\"meas_mm\":%d,\"err_mm\":%d,"
				"\"checked\":%u,\"within_gate\":%u,\"valid\":%u}}",
				links[k].child_addr, (int)links[k].tau_dtu,
				(int)links[k].geom_mm, (int)links[k].meas_mm,
				(int)links[k].err_mm, links[k].checked ? 1u : 0u,
				links[k].within_gate ? 1u : 0u,
				links[k].valid ? 1u : 0u);
		}
		if (gauge_degenerate) {
			LOG_ERR("{\"apos_error\":\"the four gauge anchors do not span a "
				"3D frame (up %d mm off the origin/xaxis/plane plane, "
				"noise floor %d mm; collinearity %d/1000). This candidate "
				"is REFUSED and `apos apply force` will not install it: "
				"its z would be noise. Mount `origin`, `xaxis` and `plane` "
				"in one plane and `up` clearly off it, then re-run.\"}",
				gq_valid ? (int)(gq.up_height_m * 1000.0f) : 0,
				gq_valid ? (int)(gq.height_floor_m * 1000.0f) : 0,
				gq_valid ? (int)(gq.collinearity * 1000.0f) : 0);
		}
		if (!links_verified) {
			LOG_WRN("{\"apos_warn\":\"%u of %u gateway links are verified "
				"(measured directly and within %d mm of the geometry) "
				"— `apos apply` will refuse unless forced\"}",
				sync_link_count_good(links, n_links), n_links,
				(int)SYNC_LINK_GATE_MM);
		}
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
			"parameters (%s) leaves %d spare. With no spare edge "
			"the fit reproduces ANY set of ranges exactly, so "
			"rms_mm is ~0 however bad the ranging was.\"}",
			solve_n_edges,
			apos_geom_free_params(res.dim, res.n_placed),
			(res.dim == APOS_GEOM_2D) ? "2N-3" : "3N-6",
			solve_redundancy);
		LOG_WRN("{\"apos_warn\":\"\\\"accepted\\\":%u above therefore "
			"means only that nothing contradicted the ranges — it "
			"does NOT mean they are correct. Read max_reciprocal_mm "
			"(%d) and max_sd_mm (%u) instead: those describe the "
			"RANGING, not the fit. Then confirm the solved "
			"node-to-node distances against a tape measure before "
			"`apos apply`.\"}", accepted ? 1u : 0u, max_recip_mm,
			max_sd_mm);
	}

	if (res.dim == APOS_GEOM_3D && planar && res.n_placed >= 4u) {
		LOG_WRN("{\"apos_warn\":\"array is near-coplanar (%d mm) — x and "
			"y are good but the solved z values are not "
			"survey-quality\"}",
			(int)(res.planarity_m * 1000.0f));
	}
	/* The 2D analogue of the coplanarity warning above: a near-collinear
	 * origin/xaxis/plane triangle carries almost no information about
	 * which way +y points, so the solved y of EVERY placed node is
	 * noise-dominated -- and unlike the 3D case, nothing else here catches
	 * it: rms_mm can read ~0 on this same mesh regardless (see
	 * solve_unverified above), so this is the only warning that exists for
	 * this specific failure mode. */
	if (gauge_thin) {
		LOG_WRN("{\"apos_warn\":\"2D gauge triangle is near-collinear "
			"(collinearity ratio %d/1000) — the solved +y direction "
			"is noise-dominated; move the `plane` anchor further off "
			"the origin-xaxis line and re-run\"}",
			(int)(res.gauge_collinearity_ratio * 1000.0f));
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

	/* Computed from the raw directed measurements, before the fit gets a
	 * say. On this array the fit cannot judge the ranging, so these two are
	 * the operator's real quality numbers. */
	apos_table_quality(&tbl, (uint8_t)APOS_MIN_N_OK, &max_recip_mm,
			   &max_sd_mm);

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
	 * degrees of freedom in 3D (3 of the 2N in 2D), so n_edges must EXCEED
	 * apos_geom_free_params(res.dim, res.n_placed) before rms_m can
	 * disagree with anything. n_edges is an upper bound on the
	 * edges the fit actually used (edges touching an unplaced node are
	 * dropped), so this is an OPTIMISTIC estimate of the spare count: the
	 * fit may have had fewer spare edges than this says, never more. That is
	 * why the flag below triggers on <= 0 rather than < 0 -- an optimistic
	 * count of exactly zero spare edges is already the degenerate case, and
	 * anything this overestimates only moves further into it. */
	solve_n_edges = n_edges;
	solve_redundancy = (int16_t)((int)n_edges -
				     apos_geom_free_params(res.dim,
							   res.n_placed));
	solve_unverified = solve_redundancy <= 0;

	/* The static-install checks, 3D with the gateway as a node only. */
	gq_valid = false;
	gauge_degenerate = false;
	n_links = 0;
	links_verified = false;
	if (res.dim == APOS_GEOM_3D && gw_idx >= 0) {
		float err = (float)max_sd_mm / 1000.0f;

		if (err < APOS_GAUGE_RANGE_ERR_FLOOR_M) {
			err = APOS_GAUGE_RANGE_ERR_FLOOR_M;
		}
		if (apos_geom_gauge_conditioning(&res, &g, err, &gq) == 0) {
			gq_valid = true;
			gauge_degenerate = gq.collinear || gq.coplanar;
		} else {
			/* A gauge node the solve could not place: that is not a
			 * "maybe"; the frame does not exist. */
			gauge_degenerate = true;
		}

		int nl = sync_link_derive(&tbl, &res, (uint8_t)gw_idx,
					  (uint8_t)APOS_MIN_N_OK, links,
					  SYNC_LINK_MAX);

		if (nl > 0) {
			n_links = (uint8_t)nl;
			/* Every anchor must have a link, checked against a direct
			 * measurement and inside the gate. */
			links_verified = (n_links == (uint8_t)(tbl.n_peers - 1u)) &&
					 (sync_link_count_good(links, n_links) ==
					  n_links);
		}
	}

	have_result = true;
	judge_result();
	phase = APOS_GW_IDLE;
}

/* One finished pair, from either path: a RANGE_RSP off the air (anchor ->
 * anchor) or the gateway's own exchanges (gateway -> anchor). The acceptance
 * rules are shared on purpose -- a thin, non-positive or accepted pair means
 * the same thing whoever measured it. Uses await_from_addr/await_peer_addr. */
static void record_pair(int32_t mean, uint16_t sd, uint8_t n_ok)
{
	int from = apos_table_find_addr(&tbl, await_from_addr);
	int to = apos_table_find_addr(&tbl, await_peer_addr);

	if (n_ok < APOS_MIN_N_OK) {
		LOG_WRN("{\"apos_thin\":{\"from\":\"0x%04X\",\"to\":\"0x%04X\","
			"\"n_ok\":%u}}", await_from_addr, await_peer_addr, n_ok);
	} else if (mean <= 0) {
		/* A physically impossible distance, and reachable for real: the
		 * antenna delays are still on the factory seed
		 * (UWB_ANT_DELAY_DEFAULT) and on a ~1 m inter-anchor array a
		 * mis-trimmed delay comfortably exceeds the true range, so the
		 * reported mean comes back negative. Rejected HERE rather than
		 * being let into the table: symmetrise() would hand it on as a
		 * negative edge, which on a gauge edge surfaces as -ENODATA and
		 * gets reported as "the gauge anchors are not mutually ranged"
		 * -- sending the operator to chase a line-of-sight problem that
		 * does not exist -- and on any other edge is squared inside
		 * trilaterate3() and silently produces garbage geometry that the
		 * (vacuous) rms check happily accepts. A hole with an accurate
		 * reason beats a bad edge with a misleading one. */
		LOG_WRN("{\"apos_bad_range\":{\"from\":\"0x%04X\",\"to\":"
			"\"0x%04X\",\"mean_mm\":%d,\"why\":\"non-positive "
			"distance — almost certainly uncalibrated antenna "
			"delay; see docs/antenna-delay-calibration.md\"}}",
			await_from_addr, await_peer_addr, mean);
	} else if (from >= 0 && to >= 0) {
		apos_table_add_meas(&tbl, (uint8_t)from, (uint8_t)to, mean, sd,
				    n_ok);
		/* One line per accepted pair. Until this existed the ranging
		 * phase logged ONLY failures, so a run in progress and a run
		 * that had died were indistinguishable from the console: 12
		 * pairs at up to APOS_GW_RANGE_TIMEOUT_MS * (1 +
		 * APOS_GW_RANGE_RETRIES) each is over a minute of silence.
		 * That is what sent the first hardware investigation of this
		 * phase looking for a hang that had not happened yet. */
		LOG_INF("{\"apos_pair\":{\"i\":%u,\"of\":%u,\"from\":"
			"\"0x%04X\",\"to\":\"0x%04X\",\"mean_mm\":%d,"
			"\"sd_mm\":%u,\"n_ok\":%u}}",
			(unsigned int)pair_idx + 1u,
			(unsigned int)pair_total, await_from_addr,
			await_peer_addr, mean, sd, n_ok);
	}
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
	/* Phase, not just session and awaiting_rsp: the session is unchanged
	 * across a whole run and APPLY also sets awaiting_rsp. apos_node.c
	 * deliberately RETRANSMITS a RANGE_RSP once when the first TX fails, so
	 * duplicates are engineered into this protocol rather than exceptional.
	 * A straggler arriving during APPLY whose (src, peer) happens to match a
	 * stale await_*_addr would otherwise clear awaiting_rsp and advance
	 * pair_idx -- which during APPLY means a spurious duplicate SETPOS and a
	 * corrupted meas_done. Same reasoning as on_enum_rsp()/on_setpos_ack(). */
	if (phase != APOS_GW_RANGE) {
		return;
	}
	/* Both endpoints must match what we asked for. Without the peer check a
	 * late reply from the PREVIOUS pair would be recorded against this one,
	 * which is a wrong edge rather than a missing one -- far worse. */
	if (apos_frame_src(buf) != await_from_addr || peer != await_peer_addr) {
		return;
	}

	record_pair(mean, sd, n_ok);

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

/* Convert a span of the DW3220's hi32 timestamp into uus, the way
 * uwb_gateway.c's loop does. */
static uint32_t hi32_span_to_uus(uint32_t span_hi32)
{
	return (uint32_t)(((uint64_t)span_hi32 << 8) / UUS_TO_DWT_TIME);
}

/* The gateway ranging the anchor at the cursor itself: up to
 * APOS_GW_LOCAL_BURST SS-TWR exchanges per call, each only if one worst-case
 * exchange (GW_SELFTAU_STEP_BUDGET_UUS) still fits before the beacon must be
 * armed, with a yield between them so the shell and log threads are not
 * starved by the cooperative loop.
 *
 * Bounded three ways -- the burst cap, the time left, and
 * APOS_GW_LOCAL_ATTEMPTS per pair -- so no combination of a silent anchor and
 * a busy superframe can hold the beacon or park the survey on one pair. An
 * anchor that never answers simply finishes the pair with n_ok below
 * APOS_MIN_N_OK, which record_pair() reports as a hole by address. */
static void step_local_range(uint32_t avail_uus)
{
	if (avail_uus < GW_SELFTAU_STEP_BUDGET_UUS) {
		return; /* wait for a roomier step; costs latency only */
	}

	uint32_t t0 = dwt_readsystimestamphi32();
	uint8_t wire_id = (uint8_t)(await_peer_addr & 0xFFu);

	for (uint8_t b = 0; b < APOS_GW_LOCAL_BURST &&
			    local_n_try < APOS_GW_LOCAL_ATTEMPTS; b++) {
		uint32_t used = hi32_span_to_uus(dwt_readsystimestamphi32() - t0);

		if (avail_uus < used + GW_SELFTAU_STEP_BUDGET_UUS) {
			break;
		}

		ss_initiator_enter();
		int32_t mm = ss_initiator_range(wire_id);

		ss_initiator_leave();

		local_n_try++;
		if (mm != INT32_MIN && local_n_ok < APOS_STATS_MAX) {
			local_s[local_n_ok++] = mm;
		}
		k_sleep(K_MSEC(1));
	}

	if (local_n_try < APOS_GW_LOCAL_ATTEMPTS) {
		return; /* more next step */
	}

	int32_t mean = 0;
	uint16_t sd = 0;
	uint8_t kept = apos_batch_stats(local_s, local_n_ok, &mean, &sd);

	LOG_INF("{\"apos_local_range\":{\"peer\":\"0x%04X\",\"tried\":%u,"
		"\"ok\":%u,\"kept\":%u,\"mean_mm\":%d,\"sd_mm\":%u}}",
		await_peer_addr, local_n_try, local_n_ok, kept, mean, sd);

	/* n_ok is the KEPT count, as on the anchor side: the min_n_ok threshold
	 * is about how many samples the mean rests on. */
	record_pair(mean, sd, kept);

	local_n_try = 0;
	local_n_ok = 0;
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

	/* anchor -> gateway is NOT measured (see apos_gw.h, "DELIBERATE
	 * DEVIATION"). Skipped silently and counted as covered: the pair is
	 * still measured in the other direction, which is the only one this
	 * design takes, so it is not a hole. */
	if (gw_idx >= 0 && to == (uint8_t)gw_idx) {
		pair_idx++;
		return;
	}

	await_from_addr = tbl.peer[from].short_addr;
	await_peer_addr = tbl.peer[to].short_addr;

	/* gateway -> anchor: this board ranges the anchor itself, no frame. */
	if (gw_idx >= 0 && from == (uint8_t)gw_idx) {
		step_local_range(avail_uus);
		return;
	}

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

	apply_with_links = (res.dim == APOS_GEOM_3D) && (gw_idx >= 0);

	/* A degenerate gauge is refused HARD, before `force` is even looked at.
	 * `force` overrides acceptance THRESHOLDS; a coplanar or collinear gauge
	 * is not a threshold, it is a coordinate frame whose z and mirror side are
	 * noise, and installing it would write that into every anchor and into the
	 * stored survey. */
	if (apply_with_links && gauge_degenerate) {
		return -EDOM;
	}
	if (!accepted && !force) {
		return -EPERM;
	}

	if (apply_with_links) {
		/* Even forced, every anchor needs a link that could be COMPUTED.
		 * "Unverified" (no direct measurement, or outside the gate) is
		 * what force overrides; "no tau at all" is not something an
		 * operator can accept. */
		if (n_links != (uint8_t)(tbl.n_peers - 1u)) {
			return -ENODATA;
		}
		for (uint8_t k = 0; k < n_links; k++) {
			if (!links[k].valid) {
				return -ENODATA;
			}
		}

		uint32_t g = apos_store_links()->generation;

		if (apos_store_pending_generation() > g) {
			g = apos_store_pending_generation();
		}
		apply_generation = g + 1u;
	}

	apply_idx = 0;
	apply_retries = 0;
	applied_ok = 0;
	applied_fail = 0;
	applied_skip = 0;
	tau_ok = 0;
	apply_persisted = false;
	apply_stage = apply_with_links ? AP_MARK : AP_REOPEN;
	awaiting_rsp = false;
	tx_fails = 0;
	next_action_ms = 0;
	phase = APOS_GW_APPLY;

	LOG_INF("{\"apos\":\"applying\",\"nodes\":%u,\"forced\":%u,"
		"\"links\":%u,\"generation\":%u}",
		res.n_nodes, force ? 1u : 0u, apply_with_links ? 1u : 0u,
		(unsigned int)apply_generation);

	/* Repeated HERE, not only at solve time. Apply is the point of no
	 * return -- it rewrites every anchor's NVS and this gateway's survey --
	 * and the operator typing it may have read `"accepted":1` minutes ago.
	 * `force` deliberately does not gate this: the condition is a property
	 * of the mesh, not a decision, and it is loudest precisely when the
	 * result looks best. */
	if (solve_unverified) {
		LOG_WRN("{\"apos_warn\":\"COMMITTING AN UNVERIFIED SURVEY: %u "
			"usable edge(s) against %d free parameters (%s), "
			"leaving %d spare edge(s), so rms_mm was ~0 however bad "
			"the ranging was.\"}", solve_n_edges,
			apos_geom_free_params(res.dim, res.n_placed),
			(res.dim == APOS_GEOM_2D) ? "2N-3" : "3N-6",
			solve_redundancy);
		LOG_WRN("{\"apos_warn\":\"on this array \\\"accepted\\\" means "
			"only that nothing contradicted the ranges — it does "
			"NOT mean they are correct. Read max_reciprocal_mm (%d) "
			"and max_sd_mm (%u), and confirm the solved "
			"node-to-node distances against a tape measure. These "
			"coordinates are about to be written to every anchor's "
			"NVS.\"}", max_recip_mm, max_sd_mm);
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
 * so the origin is moved to the front here rather than being assumed.
 *
 * The gateway, when it is a node, is written like any other (short address
 * 0x0000, its real EUI). It is the origin of nothing -- the gauge is
 * anchors-only -- so it never lands in node[0]. */
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
	s.dim = res.dim;
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

/* The links of this apply, in the persisted form, stamped with the generation
 * being installed. */
static int persist_links(void)
{
	/* static: the gateway loop runs on the 4096-byte main stack, whose peak is
	 * already 1748 B inside the solve, and this is ~200 B on top of a settings
	 * write. Called from one place, never reentrantly. */
	static struct apos_links l;

	memset(&l, 0, sizeof(l));
	l.generation = apply_generation;
	l.n = n_links;
	for (uint8_t k = 0; k < n_links; k++) {
		memcpy(l.rec[k].child_eui, links[k].child_eui, APOS_EUI_LEN);
		memcpy(l.rec[k].parent_eui, links[k].parent_eui, APOS_EUI_LEN);
		l.rec[k].tau_dtu = links[k].tau_dtu;
		l.rec[k].valid = links[k].valid;
		l.rec[k].source = links[k].source;
	}
	return apos_store_save_links(&l);
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
	 * was never measured. Same reasoning as on_enum_rsp(). The stage matters
	 * for the same reason inside APPLY: a late SETPOS_ACK must not be taken
	 * for the answer to a SETTAU. */
	if (phase != APOS_GW_APPLY || apply_stage != AP_SETPOS) {
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

/* The tau the SETTAU in flight carries, to check the ACK echoes it. */
static int32_t await_tau;

static void on_settau_ack(const uint8_t *buf, uint16_t plen)
{
	uint16_t sess = 0;
	uint32_t gen = 0;
	int32_t tau = 0;
	uint8_t status = 0xFFu;

	if (apos_frame_parse_settau_ack(buf, plen, &sess, &gen, &tau, &status)
	    != 0) {
		return;
	}
	if (sess != session || !awaiting_rsp) {
		return;
	}
	if (phase != APOS_GW_APPLY || apply_stage != AP_SETTAU) {
		return;
	}
	if (apos_frame_src(buf) != await_from_addr) {
		return;
	}

	/* The ACK reports what the anchor actually HOLDS, so a generation or tau
	 * that differs from what was sent is a failure even with status OK: it
	 * means a stale duplicate or an anchor that stored something else. Not
	 * retried -- a retry would only restate the same command to a board that
	 * answered clearly. */
	if (status == APOS_SETTAU_OK && gen == apply_generation &&
	    tau == await_tau) {
		tau_ok++;
		LOG_INF("{\"apos_settau_ok\":{\"addr\":\"0x%04X\",\"tau_dtu\":%d,"
			"\"gen\":%u}}", await_from_addr, (int)tau,
			(unsigned int)gen);
	} else {
		applied_fail++;
		LOG_ERR("{\"apos_error\":\"0x%04X answered SETTAU with status %u, "
			"generation %u, tau %d (sent generation %u, tau %d) — it "
			"did NOT take the link\"}", await_from_addr, status,
			(unsigned int)gen, (int)tau, (unsigned int)apply_generation,
			(int)await_tau);
	}

	awaiting_rsp = false;
	apply_retries = 0;
	tx_fails = 0;
	apply_idx++;
	next_action_ms = 0;
}

/* The link whose child is node `idx` of the table, or NULL. By EUI. */
static const struct sync_link *link_for_node(uint8_t idx)
{
	for (uint8_t k = 0; k < n_links; k++) {
		if (memcmp(links[k].child_eui, tbl.peer[idx].eui,
			   APOS_EUI_LEN) == 0) {
			return &links[k];
		}
	}
	return NULL;
}

/* Advance a node cursor past nodes this stage has nothing to send to: the
 * gateway itself (it is not commanded over the air), and -- for SETPOS only --
 * unplaced nodes, which are COUNTED so the summary cannot report a clean
 * `failed:0` for an apply that only covered part of the array. Pushing
 * (0, 0, 0) to an unplaced node would be exactly the silent-wrong-coordinate
 * failure this whole design exists to remove. */
static void skip_nodes(bool count_unplaced)
{
	while (apply_idx < res.n_nodes) {
		if (gw_idx >= 0 && apply_idx == (uint8_t)gw_idx) {
			apply_idx++;
			continue;
		}
		if (res.node[apply_idx].state == APOS_NODE_UNPLACED) {
			if (count_unplaced) {
				LOG_WRN("{\"apos_skip\":{\"addr\":\"0x%04X\","
					"\"reason\":\"unplaced\"}}",
					tbl.peer[apply_idx].short_addr);
				applied_skip++;
			}
			apply_idx++;
			continue;
		}
		break;
	}
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
			LOG_WRN("{\"apos_retry\":{\"%s\":\"0x%04X\","
				"\"attempt\":%u}}",
				(apply_stage == AP_SETTAU) ? "settau" : "setpos",
				await_from_addr, apply_retries);
		} else {
			applied_fail++;
			if (apply_stage == AP_SETTAU) {
				LOG_ERR("{\"apos_error\":\"0x%04X never acknowledged "
					"SETTAU after %u attempts — incompatible "
					"firmware (it predates the static link) or out "
					"of range. It is still on its OLD link.\"}",
					await_from_addr, APOS_GW_APPLY_RETRIES + 1u);
			} else {
				LOG_ERR("{\"apos_error\":\"0x%04X never acknowledged "
					"SETPOS after %u attempts — it is still on its "
					"OLD coordinates\"}", await_from_addr,
					APOS_GW_APPLY_RETRIES + 1u);
			}
			awaiting_rsp = false;
			apply_retries = 0;
			tx_fails = 0;
			apply_idx++;
		}
		return;
	}

	/* One-shot, FIRST of a links apply: record that an apply is under way
	 * BEFORE any anchor is touched, and suspend fixes. If the gateway dies
	 * from here until the commit, the marker is on flash and the next boot
	 * knows the anchors may hold a mix of generations.
	 *
	 * Gated on the SOLVE budget for the same reason every flash write on this
	 * loop is: a settings write can stall ALL execution for an NVS sector
	 * erase (see step_apply's commit below). If the marker cannot be written,
	 * NOTHING has changed yet and the apply is abandoned cleanly -- the one
	 * failure here that costs the operator nothing. */
	if (apply_stage == AP_MARK) {
		if (avail_uus < APOS_GW_SOLVE_BUDGET_UUS) {
			return;
		}
		int rc = apos_store_set_pending(apply_generation);

		if (rc) {
			LOG_ERR("{\"apos_error\":\"cannot record the apply-in-progress "
				"marker (errno %d) — apply ABANDONED before touching "
				"any anchor\"}", rc);
			phase = APOS_GW_IDLE;
			return;
		}
		hold = true;
		hold_reason = "apply-in-progress";
		apply_stage = AP_REOPEN;
		return;
	}

	/* One-shot: re-open every anchor's survey window before the first
	 * SETPOS goes out.
	 *
	 * Without this the whole apply fails, on every anchor, for a reason that
	 * looks like a dead radio. A node's window is refreshed only by an
	 * in-session SETPOS or RANGE_CMD addressed to it, and a node's LAST
	 * RANGE_CMD is in its own row of the row-major pair walk -- node 0's row
	 * is the first three of twelve pairs. Nothing between the end of RANGE
	 * and the start of APPLY refreshes anything, and the operator is
	 * deliberately told (docs/anchor-auto-positioning.md §5) to tape-measure
	 * the solved distances before applying, which takes minutes against a
	 * 60 s APOS_NODE_REFRESH_S. handle_setpos() would then refuse every
	 * SETPOS (its session check compares against a window_session that has
	 * been lazily zeroed by an expiring window) and apply would end
	 * `failed:4`.
	 *
	 * Sent with the SAME session, so handle_survey_begin() re-opens on it
	 * and every later SETPOS matches. It costs one extra frame and one
	 * settle interval per apply.
	 *
	 * One frame per step is respected: this arm transmits and returns. The
	 * staggered ENUM_RSPs it provokes are discarded by on_enum_rsp()'s
	 * phase guard -- phase is APOS_GW_APPLY throughout -- so they cannot
	 * append peers to the table the apply is walking. */
	if (apply_stage == AP_REOPEN) {
		apply_stage = AP_SETPOS;
		apply_idx = 0;
		send_survey_begin(seq);
		/* Enough for the worst-case enumeration stagger
		 * (APOS_ENUM_WINDOW_MS, 768 ms) to drain
		 * before the first SETPOS competes with it on the air. Not
		 * retried on TX failure: a SETPOS refused for a closed window is
		 * already reported per anchor, loudly, by the retry path above. */
		next_action_ms = now + APOS_GW_APPLY_SETTLE_MS;
		return;
	}

	/* Settle interval after the re-broadcast above. Every other path sets
	 * next_action_ms to 0 or is gated by the awaiting_rsp arm, so this is a
	 * no-op outside that window. */
	if (now < next_action_ms) {
		return;
	}

	if (apply_stage == AP_END) {
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
			"\"skipped\":%u,\"nodes\":%u,\"persisted\":%u,"
			"\"links\":%u,\"tau_ok\":%u,\"generation\":%u,"
			"\"hold\":%u}}",
			applied_ok, applied_fail, applied_skip, res.n_nodes,
			apply_persisted ? 1u : 0u, apply_with_links ? 1u : 0u,
			tau_ok, (unsigned int)(apply_with_links ? apply_generation : 0u),
			hold ? 1u : 0u);
		if (applied_fail || applied_skip) {
			LOG_ERR("{\"apos_error\":\"%u anchor(s) did NOT take the "
				"new coordinates%s — the deployment is "
				"inconsistent. Re-run `apos apply`.\"}",
				(unsigned int)(applied_fail + applied_skip),
				apply_with_links ? " or link" : "");
		}
		if (!apply_persisted) {
			LOG_ERR("{\"apos_error\":\"the gateway did NOT persist the "
				"survey%s — %s Re-run `apos apply`.\"}",
				apply_with_links ? " and links" : "",
				apply_with_links
				? "position fixes stay SUSPENDED and the previous "
				  "survey stays active."
				: "it will publish the STUB anchors payload after "
				  "a reboot and the map will disagree with the "
				  "solver.");
		}
		apply_stage = AP_MARK;
		phase = APOS_GW_IDLE;
		return;
	}

	if (apply_stage == AP_SETPOS) {
		skip_nodes(true);
		if (apply_idx >= res.n_nodes) {
			apply_idx = 0;
			apply_stage = apply_with_links ? AP_SETTAU : AP_COMMIT;
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
		return;
	}

	if (apply_stage == AP_SETTAU) {
		skip_nodes(false);
		if (apply_idx >= res.n_nodes) {
			apply_stage = AP_COMMIT;
			return;
		}

		const struct sync_link *lk = link_for_node(apply_idx);

		if (!lk || !lk->valid) {
			/* Cannot happen after start_apply()'s checks, but a link
			 * that is missing here must stop the apply from reporting
			 * clean rather than silently skip an anchor. */
			applied_fail++;
			LOG_ERR("{\"apos_error\":\"no valid link for 0x%04X — it "
				"keeps its OLD link\"}",
				tbl.peer[apply_idx].short_addr);
			apply_idx++;
			return;
		}

		await_from_addr = tbl.peer[apply_idx].short_addr;
		await_tau = lk->tau_dtu;

		int n = apos_frame_settau_build(tx_buf, sizeof(tx_buf),
						UWB_ADDR_GATEWAY_RESERVED,
						await_from_addr, session,
						apply_generation, lk->tau_dtu,
						lk->parent_eui);

		if (n < 0) {
			LOG_ERR("SETTAU build failed (%d)", n);
			phase = APOS_GW_IDLE;
			return;
		}
		if (!tx_now((uint16_t)n, seq)) {
			if (++tx_fails >= APOS_GW_TX_FAIL_LIMIT) {
				applied_fail++;
				LOG_ERR("{\"apos_error\":\"SETTAU for 0x%04X would not "
					"transmit — it keeps its OLD link\"}",
					await_from_addr);
				tx_fails = 0;
				apply_retries = 0;
				apply_idx++;
			}
			return;
		}
		tx_fails = 0;

		awaiting_rsp = true;
		next_action_ms = now + APOS_GW_APPLY_TIMEOUT_MS;
		return;
	}

	/* AP_COMMIT.
	 *
	 * Persist before SURVEY_END, so a gateway that dies between the two
	 * still has the survey on flash.
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
	 * remedy apos_gw.h names for the solve.
	 *
	 * A links apply does up to three writes here (survey, links, marker
	 * clear) in the one gated step, in an order where a cut at any point
	 * leaves the marker set. */
	if (avail_uus < APOS_GW_SOLVE_BUDGET_UUS) {
		return;
	}

	if (!apply_with_links) {
		/* The historical anchors-only flow: persists regardless of
		 * individual anchor failures, which the summary reports. */
		apply_persisted = persist_survey();
		apply_stage = AP_END;
		return;
	}

	if (applied_fail == 0u && applied_skip == 0u) {
		apply_persisted = persist_survey();
		if (apply_persisted) {
			int rc = persist_links();

			if (rc) {
				LOG_ERR("{\"apos_error\":\"links NOT persisted "
					"(errno %d)\"}", rc);
				apply_persisted = false;
			}
		}
		if (apply_persisted) {
			int rc = apos_store_clear_pending();

			if (rc) {
				LOG_ERR("{\"apos_error\":\"apply marker NOT "
					"cleared (errno %d) — fixes stay suspended "
					"until the next successful apply\"}", rc);
				apply_persisted = false;
			}
		}
		if (apply_persisted) {
			applied_generation = apply_generation;
			hold = false;
			hold_reason = "none";
			LOG_INF("{\"apos\":\"installed\",\"generation\":%u,"
				"\"links\":%u}", (unsigned int)apply_generation,
				n_links);
		} else {
			hold = true;
			hold_reason = "apply-incomplete";
		}
	} else {
		/* Some anchor did not take it. The gateway keeps its PREVIOUS
		 * survey and links, the marker stays set, and fixes stay
		 * suspended: the anchors that DID take it now run a generation
		 * the gateway does not know about. Re-running `apos apply`
		 * restates the same commands idempotently. */
		apply_persisted = false;
		hold = true;
		hold_reason = "apply-incomplete";
	}
	apply_stage = AP_END;
}

/* ---- `apos assign` ---- */

int apos_gw_start_assign(void)
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}
	if (apos_store_addrmap_rejected()) {
		/* Writing a new map would overwrite a record this firmware cannot
		 * read -- possibly one a newer build wrote. That is the operator's
		 * call (`apos addr clear`), not a side effect of assign. */
		return -EIO;
	}

	apos_table_init(&tbl);
	have_result = false;
	session = new_session();
	enum_round = 0;
	next_action_ms = 0;
	is_run = false;
	pair_idx = 0;
	pair_total = 0;
	awaiting_rsp = false;
	reset_static_state();

	assign_stage = AS_ENUM;
	plan_n = 0;
	assign_idx = 0;
	assign_failed = 0;
	assign_acked = 0;
	memset(plan_acked, 0, sizeof(plan_acked));
	memset(plan_nacked, 0, sizeof(plan_nacked));
	phase = APOS_GW_ASSIGN;

	LOG_INF("{\"apos\":\"assigning\",\"session\":%u}", session);
	return 0;
}

/* Queued, not written here: a settings write stalls ALL execution for an NVS
 * sector erase (XIP), so it runs from the gateway loop under the same budget
 * gate as every other apos flash write -- see step_forget(). */
int apos_gw_forget(const uint8_t eui[APOS_EUI_LEN])
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}
	if (eui == NULL) {
		memset(&forget_map, 0, sizeof(forget_map));
	} else {
		forget_map = *apos_store_addrmap();
		if (apos_addrmap_forget(&forget_map, eui) != 0) {
			return -ENOENT;
		}
	}
	phase = APOS_GW_FORGET;
	return 0;
}

static void step_forget(uint32_t avail_uus)
{
	if (avail_uus < APOS_GW_SOLVE_BUDGET_UUS) {
		return;
	}

	int rc = apos_store_save_addrmap(&forget_map);

	if (rc) {
		LOG_ERR("{\"apos_error\":\"address map NOT updated (errno %d)\"}", rc);
	} else {
		LOG_INF("{\"apos_addrmap\":{\"n\":%u,\"saved\":1}}", forget_map.n);
	}
	phase = APOS_GW_IDLE;
}

static void log_plan_entry(uint8_t i)
{
	static const char *const what[] = {"keep", "adopt", "assign", "nospace"};
	char hex[NODE_EUI_HEX_LEN + 1u];

	node_eui_fmt(plan_peer[i].eui, hex);
	LOG_INF("{\"apos_assign_plan\":{\"eui\":\"%s\",\"from\":\"0x%04X\","
		"\"to\":\"0x%04X\",\"action\":\"%s\"}}", hex, plan_peer[i].addr,
		plan_out[i].addr,
		(plan_out[i].action < 4u) ? what[plan_out[i].action] : "?");
}

/* Build the plan from the freshly enumerated table. Returns the number of
 * SETADDRs it needs, or a negative errno (already logged). */
static int make_plan(void)
{
	uint8_t n_assign = 0;

	plan_n = 0;
	for (uint8_t k = 0; k < tbl.n_peers; k++) {
		if (tbl.peer[k].short_addr == UWB_ADDR_GATEWAY_RESERVED) {
			continue;   /* never in an assign table, but never planned */
		}
		memcpy(plan_peer[plan_n].eui, tbl.peer[k].eui, APOS_EUI_LEN);
		plan_peer[plan_n].addr = tbl.peer[k].short_addr;
		plan_n++;
	}
	if (plan_n == 0u) {
		LOG_ERR("{\"apos_error\":\"no anchor answered enumeration — nothing "
			"was assigned\"}");
		return -ENODEV;
	}

	int rc = apos_addr_plan(apos_store_addrmap(), plan_peer, plan_n,
				UWB_ANCHOR_ADDR_BASE,
				UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS,
				plan_out, &n_assign);

	if (rc == -EINVAL) {
		LOG_ERR("{\"apos_error\":\"assign plan refused its input (two "
			"boards with one EUI, or an all-zero EUI)\"}");
		return rc;
	}
	for (uint8_t i = 0; i < plan_n; i++) {
		log_plan_entry(i);
	}
	if (rc == -ENOSPC) {
		LOG_ERR("{\"apos_error\":\"not every anchor fits: %u address(es) "
			"and %u map slot(s). Nothing was changed. Free a stale "
			"reservation with `apos forget <eui>`.\"}",
			(unsigned int)UWB_MAX_ANCHORS,
			(unsigned int)APOS_ADDRMAP_MAX);
		return rc;
	}
	return n_assign;
}

/* Whether planned board `i` now answers enumeration from its target address. */
static bool seen_at_target(uint8_t i)
{
	int k = apos_table_find_eui(&tbl, plan_peer[i].eui);

	return k >= 0 && tbl.peer[k].short_addr == plan_out[i].addr;
}

/* An ASSIGN still worth waiting for: not refused by the board. A missing ACK is
 * NOT a refusal -- the board reboots 500 ms after its ACK leaves, so an ACK lost
 * on air is followed by retries that hit a rebooting board with its survey
 * window closed. Whether it moved is settled by re-enumeration instead. */
static bool assign_pending(uint8_t i)
{
	return plan_out[i].action == APOS_PLAN_ASSIGN && !plan_nacked[i];
}

/* Close the assign: re-plan from what the table shows NOW. An empty plan (all
 * KEEP) means every board answers from the address the map gives it. */
static void finish_assign(void)
{
	uint8_t pending = 0;
	bool replanned;
	struct apos_plan_out now_out[APOS_MAX_NODES];

	for (uint8_t i = 0; i < plan_n; i++) {
		int k = apos_table_find_eui(&tbl, plan_peer[i].eui);

		if (k >= 0) {
			plan_peer[i].addr = tbl.peer[k].short_addr;
		}
	}
	replanned = apos_addr_plan(apos_store_addrmap(), plan_peer, plan_n,
				   UWB_ANCHOR_ADDR_BASE,
				   UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS,
				   now_out, &pending) == 0;
	pending = 0;
	for (uint8_t i = 0; i < plan_n && replanned; i++) {
		if (now_out[i].action != APOS_PLAN_KEEP) {
			char hex[NODE_EUI_HEX_LEN + 1u];

			node_eui_fmt(plan_peer[i].eui, hex);
			LOG_ERR("{\"apos_assign_unconverged\":{\"eui\":\"%s\","
				"\"at\":\"0x%04X\",\"want\":\"0x%04X\"}}", hex,
				plan_peer[i].addr, now_out[i].addr);
			pending++;
		}
	}

	bool ok = replanned && pending == 0u && assign_failed == 0u;

	LOG_INF("{\"apos_assign_done\":{\"anchors\":%u,\"acked\":%u,"
		"\"failed\":%u,\"converged\":%u}}", plan_n, assign_acked,
		assign_failed, ok ? 1u : 0u);
	if (!ok) {
		LOG_ERR("{\"apos_error\":\"`apos assign` did NOT converge — see "
			"the lines above. Re-run it once the listed boards answer.\"}");
	}
	phase = APOS_GW_IDLE;
}

static void on_setaddr_ack(const uint8_t *buf, uint16_t plen)
{
	uint16_t sess = 0, addr = 0;
	uint8_t eui[APOS_EUI_LEN];
	uint8_t status = 0xFFu;

	if (apos_frame_parse_setaddr_ack(buf, plen, &sess, eui, &addr, &status)
	    != 0) {
		return;
	}
	if (sess != session || !awaiting_rsp || phase != APOS_GW_ASSIGN ||
	    assign_stage != AS_SEND || assign_idx >= plan_n) {
		return;
	}
	/* Paired by EUI, never by source address: the ACK leaves from the
	 * board's OLD address, which may be shared or UNASSIGNED. */
	if (memcmp(eui, plan_peer[assign_idx].eui, APOS_EUI_LEN) != 0) {
		return;
	}

	char hex[NODE_EUI_HEX_LEN + 1u];

	node_eui_fmt(eui, hex);
	if (status == APOS_SETADDR_OK && addr == plan_out[assign_idx].addr) {
		plan_acked[assign_idx] = true;
		assign_acked++;
		LOG_INF("{\"apos_setaddr_ok\":{\"eui\":\"%s\",\"addr\":\"0x%04X\"}}",
			hex, addr);
	} else {
		/* An explicit refusal: the board answered and did not move. */
		plan_nacked[assign_idx] = true;
		assign_failed++;
		LOG_ERR("{\"apos_error\":\"%s answered SETADDR with status %u, "
			"address 0x%04X (sent 0x%04X) — it did NOT take it\"}", hex,
			status, addr, plan_out[assign_idx].addr);
	}
	awaiting_rsp = false;
	apply_retries = 0;
	tx_fails = 0;
	assign_idx++;
	next_action_ms = 0;
}

/* Rewrite the stored survey's short addresses for boards that just moved, so
 * the retained anchors payload (named by address, schema frozen with the
 * platform) and every address-keyed consumer follow the boards. Matched by EUI.
 * Returns false (logged) when the result would name two nodes by one address
 * -- a surveyed board that was absent today and holds no reservation -- in
 * which case nothing is changed and a re-survey is the fix. */
static bool survey_follow_moves(struct apos_survey *sv, bool *changed)
{
	*changed = false;
	*sv = *apos_store_get();
	if (!sv->valid) {
		return true;
	}
	for (uint8_t i = 0; i < plan_n; i++) {
		if (plan_out[i].action != APOS_PLAN_ASSIGN ||
		    !(plan_acked[i] || seen_at_target(i))) {
			continue;
		}
		for (uint8_t n = 0; n < sv->n_nodes; n++) {
			if (memcmp(sv->node[n].eui, plan_peer[i].eui,
				   APOS_EUI_LEN) == 0 &&
			    sv->node[n].short_addr != plan_out[i].addr) {
				sv->node[n].short_addr = plan_out[i].addr;
				*changed = true;
			}
		}
	}
	for (uint8_t a = 0; a < sv->n_nodes; a++) {
		for (uint8_t b = (uint8_t)(a + 1u); b < sv->n_nodes; b++) {
			if (sv->node[a].short_addr == sv->node[b].short_addr) {
				LOG_ERR("{\"apos_error\":\"after the assign the stored "
					"survey would hold two nodes at 0x%04X — left "
					"unchanged; re-run `apos run` + `apos apply`\"}",
					sv->node[a].short_addr);
				*changed = false;
				return false;
			}
		}
	}
	return true;
}

static void step_assign(uint32_t avail_uus, uint8_t *seq)
{
	int64_t now = k_uptime_get();

	if (assign_stage == AS_ENUM) {
		if (now < next_action_ms) {
			return;
		}
		if (enum_round < APOS_GW_ENUM_ROUNDS) {
			send_survey_begin(seq);
			enum_round++;
			next_action_ms = now + APOS_GW_ENUM_SETTLE_MS;
			return;
		}
		LOG_INF("{\"apos_enum_done\":{\"peers\":%u,\"needs_assign\":%u}}",
			tbl.n_peers, apos_table_needs_assign(&tbl) ? 1u : 0u);

		int n_assign = make_plan();

		if (n_assign < 0) {
			phase = APOS_GW_IDLE;
			return;
		}
		assign_idx = 0;
		apply_retries = 0;
		tx_fails = 0;
		awaiting_rsp = false;
		next_action_ms = 0;
		assign_stage = (n_assign > 0) ? AS_SEND : AS_PERSIST;
		return;
	}

	if (assign_stage == AS_SEND) {
		if (awaiting_rsp) {
			if (now < next_action_ms) {
				return;
			}
			if (apply_retries < APOS_GW_APPLY_RETRIES) {
				apply_retries++;
				awaiting_rsp = false;
			} else {
				/* No ACK at all. Not a failure yet: see
				 * assign_pending(). Convergence decides. */
				char hex[NODE_EUI_HEX_LEN + 1u];

				node_eui_fmt(plan_peer[assign_idx].eui, hex);
				LOG_WRN("{\"apos_warn\":\"%s never acknowledged SETADDR "
					"after %u attempts — waiting to see whether it "
					"moved anyway\"}", hex,
					APOS_GW_APPLY_RETRIES + 1u);
				awaiting_rsp = false;
				apply_retries = 0;
				tx_fails = 0;
				assign_idx++;
			}
			return;
		}
		while (assign_idx < plan_n &&
		       plan_out[assign_idx].action != APOS_PLAN_ASSIGN) {
			assign_idx++;
		}
		if (assign_idx >= plan_n) {
			bool wait = false;

			for (uint8_t i = 0; i < plan_n; i++) {
				wait = wait || assign_pending(i);
			}
			converge_deadline_ms = now + APOS_GW_ASSIGN_CONVERGE_MS;
			next_action_ms = 0;
			assign_stage = wait ? AS_CONVERGE : AS_PERSIST;
			return;
		}

		int n = apos_frame_setaddr_build(tx_buf, sizeof(tx_buf),
						 UWB_ADDR_GATEWAY_RESERVED,
						 APOS_ADDR_BCAST, session,
						 plan_peer[assign_idx].eui,
						 plan_out[assign_idx].addr);

		if (n < 0) {
			LOG_ERR("SETADDR build failed (%d)", n);
			phase = APOS_GW_IDLE;
			return;
		}
		if (!tx_now((uint16_t)n, seq)) {
			/* Bounded like SETPOS: a TX that never starts is not the
			 * board's fault, but a wedged radio must not park the
			 * assign forever. Nothing reached it, so it cannot have
			 * moved: a failure, not a pending. */
			if (++tx_fails >= APOS_GW_TX_FAIL_LIMIT) {
				plan_nacked[assign_idx] = true;
				assign_failed++;
				tx_fails = 0;
				apply_retries = 0;
				assign_idx++;
			}
			return;
		}
		tx_fails = 0;
		awaiting_rsp = true;
		next_action_ms = now + APOS_GW_APPLY_TIMEOUT_MS;
		return;
	}

	if (assign_stage == AS_CONVERGE) {
		/* Done once every board that did not refuse answers enumeration
		 * from its new address. The reboot closed its survey window;
		 * each SURVEY_BEGIN below reopens it on the same session. */
		bool all = true;

		for (uint8_t i = 0; i < plan_n && all; i++) {
			if (assign_pending(i) && !seen_at_target(i)) {
				all = false;
			}
		}
		if (all || now >= converge_deadline_ms) {
			assign_stage = AS_PERSIST;
			return;
		}
		if (now < next_action_ms) {
			return;
		}
		send_survey_begin(seq);
		next_action_ms = now + APOS_GW_ENUM_SETTLE_MS;
		return;
	}

	/* AS_PERSIST. Only what is settled goes in: ADOPTs (no frame needed),
	 * and ASSIGNs the board either acknowledged or was SEEN at afterwards.
	 * A board with neither is reported and re-planned next time, not
	 * reserved on a guess. */
	struct apos_addrmap m = *apos_store_addrmap();
	bool map_changed = false;
	bool moved = false;
	uint8_t newly_failed = 0;

	for (uint8_t i = 0; i < plan_n; i++) {
		bool take = plan_out[i].action == APOS_PLAN_ADOPT;

		if (assign_pending(i)) {
			if (plan_acked[i] || seen_at_target(i)) {
				take = true;
				moved = true;
			} else {
				char hex[NODE_EUI_HEX_LEN + 1u];

				node_eui_fmt(plan_peer[i].eui, hex);
				LOG_ERR("{\"apos_error\":\"%s neither acknowledged "
					"SETADDR nor appeared at 0x%04X — firmware older "
					"than address assignment, or out of range\"}",
					hex, plan_out[i].addr);
				newly_failed++;
			}
		}
		if (!take) {
			continue;
		}
		int k = apos_addrmap_find_eui(&m, plan_peer[i].eui);

		if (k >= 0 && m.e[k].addr == plan_out[i].addr) {
			continue;
		}
		if (apos_addrmap_put(&m, plan_peer[i].eui, plan_out[i].addr) != 0) {
			/* The plan was built against this same map, so this cannot
			 * fail; if it does, persist nothing. */
			LOG_ERR("{\"apos_error\":\"address map update refused — "
				"nothing persisted\"}");
			phase = APOS_GW_IDLE;
			return;
		}
		map_changed = true;
	}

	struct apos_survey sv;
	bool survey_changed = false;

	(void)survey_follow_moves(&sv, &survey_changed);

	if (map_changed || survey_changed) {
		/* Flash writes on the beacon-owning loop: gated like every other
		 * one (see AP_COMMIT in step_apply()). Up to two in this step. */
		if (avail_uus < APOS_GW_SOLVE_BUDGET_UUS) {
			return;
		}
		if (map_changed) {
			int rc = apos_store_save_addrmap(&m);

			if (rc) {
				LOG_ERR("{\"apos_error\":\"address map NOT persisted "
					"(errno %d) — the anchors that took a new "
					"address will be re-planned next time\"}", rc);
				phase = APOS_GW_IDLE;
				return;
			}
		}
		if (survey_changed) {
			int rc = apos_store_save(&sv);

			if (rc) {
				LOG_ERR("{\"apos_error\":\"survey addresses NOT updated "
					"(errno %d) — re-run `apos run` + `apos apply`\"}",
					rc);
			}
		}
	}
	assign_failed += newly_failed;

	if (moved && apos_gw_gauge_set()) {
		/* The gauge is a set of ADDRESSES; boards just changed theirs, so
		 * it may now name different boxes. */
		memset(gauge_addr, 0, sizeof(gauge_addr));
		gauge_addr[3] = -1;
		LOG_WRN("{\"apos_warn\":\"addresses changed — the gauge was "
			"cleared; set it again with `apos gauge`\"}");
	}
	finish_assign();
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
	case APOS_SUB_SETTAU_ACK:
		on_settau_ack(buf, plen);
		break;
	case APOS_SUB_SETADDR_ACK:
		on_setaddr_ack(buf, plen);
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
		LOG_INF("{\"apos_enum_done\":{\"peers\":%u,\"needs_assign\":%u}}",
			tbl.n_peers, apos_table_needs_assign(&tbl) ? 1u : 0u);

		/* Unassigned or duplicate addresses: nothing below can address
		 * those boards. A run never re-addresses them itself (the gauge
		 * is a set of addresses and would silently follow the wrong
		 * board), so it stops and says which command fixes it. */
		if (apos_table_needs_assign(&tbl)) {
			for (uint8_t k = 0; k < tbl.n_peers; k++) {
				if (tbl.peer[k].addr_conflict) {
					char hex[NODE_EUI_HEX_LEN + 1u];

					node_eui_fmt(tbl.peer[k].eui, hex);
					LOG_WRN("{\"apos_needs_assign\":{\"eui\":"
						"\"%s\",\"addr\":\"0x%04X\"}}", hex,
						tbl.peer[k].short_addr);
				}
			}
			if (is_run) {
				LOG_ERR("{\"apos_error\":\"some anchors are "
					"unassigned or share an address — run "
					"`apos assign` first, then set the gauge "
					"again\"}");
			}
			phase = APOS_GW_IDLE;
			return;
		}

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
		uint8_t min_nodes = (apos_gw_gauge_dim() == APOS_GEOM_2D)
					    ? APOS_MIN_NODES_2D
					    : APOS_MIN_NODES_3D;

		if (tbl.n_peers < min_nodes) {
			LOG_ERR("{\"apos_error\":\"%u anchor(s) answered; a "
				"%s gauge needs at least %u\"}",
				tbl.n_peers,
				(min_nodes == APOS_MIN_NODES_2D) ? "2D" : "3D",
				min_nodes);
			phase = APOS_GW_IDLE;
			return;
		}

		/* A 3D survey is the static-install flow: the gateway joins the
		 * table as one more node, with its real EUI, exactly once and
		 * locally -- it is never enumerated over the air and never
		 * commands itself. A 2D survey is the historical anchors-only
		 * flow and is left exactly as it was. The gauge itself stays
		 * anchors-only: apos_gw_set_gauge() refuses 0x0000. */
		if (min_nodes == APOS_MIN_NODES_3D) {
			if (!gw_eui_ready) {
				LOG_ERR("{\"apos_error\":\"this gateway has no EUI-64, "
					"so it cannot be a node of its own survey\"}");
				phase = APOS_GW_IDLE;
				return;
			}
			int gi = apos_table_add_peer(&tbl, gw_eui,
						     UWB_ADDR_GATEWAY_RESERVED,
						     false, 0.0f, 0.0f, 0.0f);

			if (gi == -ENOSPC) {
				LOG_ERR("{\"apos_error\":\"%u anchors answered and the "
					"survey holds at most %u nodes, so the gateway "
					"cannot be added — it would otherwise be "
					"silently left out\"}", tbl.n_peers,
					APOS_MAX_NODES);
				phase = APOS_GW_IDLE;
				return;
			}
			if (gi < 0) {
				LOG_ERR("{\"apos_error\":\"could not add the gateway to "
					"the survey (errno %d)\"}", gi);
				phase = APOS_GW_IDLE;
				return;
			}
			gw_idx = (int8_t)gi;
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
	next_action_ms = now + APOS_GW_ENUM_SETTLE_MS;
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
	case APOS_GW_ASSIGN:
		step_assign(avail_uus, seq);
		break;
	case APOS_GW_FORGET:
		step_forget(avail_uus);
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

int apos_gw_trigger_from_mqtt(const char *payload, size_t len)
{
	/* Bounded copy so strstr() cannot run off a payload the broker did not
	 * NUL-terminate. 64 bytes is far more than the accepted documents need
	 * and anything longer is not one of them. */
	char tmp[64];

	if (!payload || len == 0) {
		return -EINVAL;
	}
	if (len >= sizeof(tmp)) {
		LOG_WRN("survey trigger payload too long (%u) — ignored",
			(unsigned int)len);
		return -EINVAL;
	}
	memcpy(tmp, payload, len);
	tmp[len] = '\0';

	if (strstr(tmp, "\"run\"") != NULL) {
		LOG_INF("{\"apos\":\"triggered by MQTT\",\"cmd\":\"run\"}");
		return apos_gw_start_run();
	}
	if (strstr(tmp, "\"apply\"") != NULL) {
		LOG_INF("{\"apos\":\"triggered by MQTT\",\"cmd\":\"apply\"}");
		return apos_gw_start_apply(false);
	}

	LOG_WRN("unrecognised survey trigger: %s", tmp);
	return -EINVAL;
}
