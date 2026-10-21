/*
 * Host test of src/apos_gw.c -- the gateway's survey step machine -- against a
 * FAKE WORLD: a fake radio that captures every frame the gateway transmits, a
 * set of fake anchors that answer it (or deliberately do not), a fake NVS with
 * injectable failures, and a fake clock the test drives. The module under test
 * is the real file, unmodified; only the Zephyr and DW3000 edges are shimmed
 * (tests/apos_gw/shim/).
 *
 * This exists because the static-install apply is a DISTRIBUTED sequence --
 * marker, SETPOS to every anchor, SETTAU to every anchor, then a three-write
 * commit -- and the properties that matter are about its ORDERING and its
 * failure paths, not about arithmetic: no fix may be published as valid while
 * the anchors can hold a mix of generations, a lost or duplicated ACK must not
 * advance or skip a node, and a failure must leave the previous survey active
 * with the marker still set. None of that is reachable from a pure-function
 * test, and a hardware campaign finds it only by luck.
 *
 *   gcc -Wall -Wextra -Itests/apos_gw/shim -Itests/mac_budget/shim \
 *       -Itests/ccp_frame/shim -Isrc -o tests/apos_gw/test_apos_gw.exe \
 *       tests/apos_gw/test_apos_gw.c src/apos_gw.c src/apos_frame.c \
 *       src/apos_geom.c src/apos_table.c src/apos_stats.c src/apos_links.c \
 *       src/sync_link.c src/uwb_config.c src/node_eui.c -lm
 */

#include "apos_gw.h"

#include "apos_addrmap.h"
#include "apos_frame.h"
#include "apos_links.h"
#include "apos_store.h"
#include "ss_initiator.h"
#include "uwb_config.h"
#include "uwb_dwtime.h"

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail;

#define CHECK(cond) do { \
	if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* ---- fake kernel, logging, platform ------------------------------------- */

static int64_t now_ms;
static int log_count[4];
static bool verbose;

int64_t k_uptime_get(void) { return now_ms; }
int32_t k_sleep(k_timeout_t t) { now_ms += t.ms; return 0; }

void shim_log(int level, const char *fmt, ...)
{
	log_count[level]++;
	if (verbose) {
		va_list ap;

		va_start(ap, fmt);
		printf("  [%c] ", "DIWE"[level]);
		vprintf(fmt, ap);
		printf("\n");
		va_end(ap);
	}
}

ssize_t hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	for (size_t i = 0; i < length; i++) {
		buffer[i] = (uint8_t)(0xA0 + i);
	}
	return (ssize_t)length;
}

static uint32_t rand_ctr = 0x1234;
uint32_t sys_rand32_get(void) { return rand_ctr += 7919; }

/* ---- fake radio ---------------------------------------------------------- */

#define TXQ 128
static uint8_t txq[TXQ][64];
static uint16_t txq_len[TXQ];
static int txq_head, txq_tail;
static uint8_t cur_buf[64];
static uint16_t cur_len;
static uint32_t hi32;

void dwt_forcetrxoff(void) {}
void dwt_writesysstatuslo(uint32_t mask) { (void)mask; }
int dwt_writetxdata(uint16_t len, uint8_t *buf, uint16_t off)
{
	(void)off;
	memcpy(cur_buf, buf, len);
	cur_len = len;
	return 0;
}
void dwt_writetxfctrl(uint16_t len, uint16_t off, int ranging)
{
	(void)len; (void)off; (void)ranging;
}
int dwt_starttx(int mode)
{
	(void)mode;
	memcpy(txq[txq_tail % TXQ], cur_buf, cur_len);
	txq_len[txq_tail % TXQ] = cur_len;
	txq_tail++;
	return DWT_SUCCESS;
}
uint32_t dwt_readsystimestamphi32(void) { return hi32 += 1000u; }
bool uwb_wait_for_sysstatus_lo(uint32_t m, uint32_t t) { (void)m; (void)t; return true; }

/* ---- fake NVS ------------------------------------------------------------ */

static struct apos_survey store_survey;
static struct apos_links store_links;
static bool pending_set;
static bool pending_rejected;
static uint32_t pending_gen;
static int survey_saves, links_saves;
static bool fail_pending_write, fail_survey_write, fail_links_write;

const struct apos_survey *apos_store_get(void) { return &store_survey; }
int apos_store_save(const struct apos_survey *s)
{
	if (fail_survey_write) {
		return -EIO;
	}
	store_survey = *s;
	survey_saves++;
	return 0;
}
const struct apos_links *apos_store_links(void) { return &store_links; }
bool apos_store_links_rejected(void) { return false; }
int apos_store_save_links(const struct apos_links *l)
{
	if (fail_links_write) {
		return -EIO;
	}
	store_links = *l;
	links_saves++;
	return 0;
}
bool apos_store_apply_pending(void) { return pending_rejected || pending_set; }
uint32_t apos_store_pending_generation(void) { return pending_gen; }
int apos_store_set_pending(uint32_t g)
{
	if (fail_pending_write) {
		return -EIO;
	}
	pending_set = true;
	pending_gen = g;
	return 0;
}
int apos_store_clear_pending(void)
{
	pending_set = false;
	pending_rejected = false;
	return 0;
}

static struct apos_addrmap store_map;
static bool map_rejected;
static bool fail_map_write;
static int map_saves;

const struct apos_addrmap *apos_store_addrmap(void) { return &store_map; }
bool apos_store_addrmap_rejected(void) { return map_rejected; }
int apos_store_save_addrmap(const struct apos_addrmap *m)
{
	if (fail_map_write) {
		return -EIO;
	}
	store_map = *m;
	map_rejected = false;
	map_saves++;
	return 0;
}

/* ---- fake anchors -------------------------------------------------------- */

#define NA 8
struct fake_anchor {
	uint16_t addr;
	uint8_t  eui[8];
	float    pos[3];
	bool     present;
	bool     range_silent;     /* gateway cannot range it */
	int      drop_settau_acks; /* swallow this many SETTAU ACKs */
	bool     never_acks_settau;
	bool     never_acks_setpos;
	uint8_t  settau_status;
	bool     ack_stale_gen;   /* ACK echoes an OLD generation */
	/* what it ends up holding */
	bool     has_tau;
	int32_t  tau;
	uint32_t gen;
	int      n_settau_rx;
	int      n_setpos_rx;
	/* address assignment */
	bool     old_fw;            /* predates SETADDR: drops it silently */
	int      drop_setaddr_acks; /* swallow this many ACKs (and do NOT reboot) */
	bool     fail_persist_id;   /* NVS write of the new id fails */
	int64_t  reboot_ms;         /* how long a reboot keeps it off the air */
	int64_t  offline_until;     /* deaf (rebooting) until this time */
	int      n_setaddr_rx;
	int      n_reboots;
};
static struct fake_anchor A[NA];
static float GW[3];

/* world flags */
static bool dup_acks;
static bool inject_stale_setpos_ack;
static bool inject_stale_settau_ack;
static int  setpos_frames, settau_frames, to_gateway_frames, range_cmds_at_gateway;
static bool hold_during_setpos, hold_during_settau;
static bool pending_before_first_setpos;
static bool saw_setpos;
static int  setaddr_frames;
static bool inject_foreign_setaddr_ack;

static uint32_t noise_ctr = 99;
static int32_t noise_mm(void)
{
	noise_ctr = noise_ctr * 1664525u + 1013904223u;
	return (int32_t)((noise_ctr >> 16) % 17u) - 8; /* +-8 mm */
}

static float dist(const float a[3], const float b[3])
{
	float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];

	return sqrtf(dx * dx + dy * dy + dz * dz);
}

int32_t ss_initiator_range(uint8_t wire_id)
{
	for (int i = 0; i < NA; i++) {
		if (A[i].present && (uint8_t)(A[i].addr & 0xFFu) == wire_id) {
			if (A[i].range_silent) {
				return INT32_MIN;
			}
			return (int32_t)lroundf(dist(GW, A[i].pos) * 1000.0f) +
			       noise_mm();
		}
	}
	return INT32_MIN;
}
void ss_initiator_enter(void) {}
void ss_initiator_leave(void) {}

static struct fake_anchor *by_addr(uint16_t addr)
{
	for (int i = 0; i < NA; i++) {
		if (A[i].present && A[i].addr == addr) {
			return &A[i];
		}
	}
	return NULL;
}

static uint8_t rx_seq;

static void deliver(uint8_t *buf, int n)
{
	apos_frame_set_seq(buf, rx_seq++);
	apos_gw_on_rx(buf, (uint16_t)n);
}

static void handle_frame(const uint8_t *f, uint16_t len)
{
	uint8_t buf[64];

	if (!apos_frame_is_apos(f, len)) {
		return;
	}
	if (apos_frame_dest(f) == UWB_ADDR_GATEWAY_RESERVED &&
	    apos_frame_subtype(f) != APOS_SUB_SURVEY_BEGIN) {
		to_gateway_frames++;
	}

	switch (apos_frame_subtype(f)) {
	case APOS_SUB_SURVEY_BEGIN: {
		uint16_t sess, win;

		apos_frame_parse_survey_begin(f, len, &sess, &win);
		for (int i = 0; i < NA; i++) {
			if (!A[i].present || now_ms < A[i].offline_until) {
				continue;
			}
			int n = apos_frame_enum_rsp_build(buf, sizeof(buf), A[i].addr, 0,
							  sess, A[i].eui, false, 0, 0, 0);
			deliver(buf, n);
		}
		break;
	}
	case APOS_SUB_RANGE_CMD: {
		uint16_t sess, peer;
		uint8_t nx;
		struct fake_anchor *from = by_addr(apos_frame_dest(f));
		struct fake_anchor *to;

		apos_frame_parse_range_cmd(f, len, &sess, &peer, &nx);
		if (peer == UWB_ADDR_GATEWAY_RESERVED) {
			range_cmds_at_gateway++;
		}
		to = by_addr(peer);
		if (!from || !to) {
			break;
		}
		int n = apos_frame_range_rsp_build(buf, sizeof(buf), from->addr, 0, sess,
						   peer,
						   (int32_t)lroundf(dist(from->pos, to->pos) * 1000.0f) + noise_mm(),
						   10, 40);
		deliver(buf, n);
		break;
	}
	case APOS_SUB_SETPOS: {
		uint16_t sess;
		float x, y, z;
		struct fake_anchor *a = by_addr(apos_frame_dest(f));

		apos_frame_parse_setpos(f, len, &sess, &x, &y, &z);
		setpos_frames++;
		if (!saw_setpos) {
			saw_setpos = true;
			pending_before_first_setpos = pending_set;
		}
		hold_during_setpos = apos_gw_publish_hold();
		if (!a) {
			break;
		}
		a->n_setpos_rx++;
		if (a->never_acks_setpos) {
			break;
		}
		if (inject_stale_settau_ack) {
			/* Delivered BEFORE the genuine SETPOS_ACK. */
			int m = apos_frame_settau_ack_build(buf, sizeof(buf), a->addr, 0,
							    sess, 0, 0, APOS_SETTAU_OK);
			deliver(buf, m);
		}
		int n = apos_frame_setpos_ack_build(buf, sizeof(buf), a->addr, 0, sess,
						    x, y, z, true);
		deliver(buf, n);
		if (dup_acks) {
			deliver(buf, n);
		}
		break;
	}
	case APOS_SUB_SETTAU: {
		uint16_t sess;
		uint8_t ver, parent[8];
		uint32_t gen;
		int32_t tau;
		struct fake_anchor *a = by_addr(apos_frame_dest(f));

		apos_frame_parse_settau(f, len, &sess, &ver, &gen, &tau, parent);
		settau_frames++;
		hold_during_settau = apos_gw_publish_hold();
		if (!a) {
			break;
		}
		a->n_settau_rx++;
		if (inject_stale_setpos_ack) {
			/* A SETPOS_ACK straggler arriving while SETTAU is in flight
			 * must not be taken for the answer to it -- delivered even to an
			 * anchor whose REAL answer never comes. */
			int m = apos_frame_setpos_ack_build(buf, sizeof(buf), a->addr, 0,
							    sess, 0, 0, 0, true);
			deliver(buf, m);
		}
		if (a->never_acks_settau) {
			break;
		}
		if (a->settau_status == APOS_SETTAU_OK) {
			a->has_tau = true;
			a->tau = tau;
			a->gen = gen;
		}
		if (a->drop_settau_acks > 0) {
			a->drop_settau_acks--;
			break;
		}
		int n = apos_frame_settau_ack_build(buf, sizeof(buf), a->addr, 0, sess,
						    a->ack_stale_gen ? gen - 1u : a->gen,
						    a->tau, a->settau_status);
		deliver(buf, n);
		if (dup_acks) {
			deliver(buf, n);
		}
		break;
	}
	case APOS_SUB_SETADDR: {
		uint16_t sess, addr;
		uint8_t target[8];

		CHECK(apos_frame_dest(f) == APOS_ADDR_BCAST);
		apos_frame_parse_setaddr(f, len, &sess, target, &addr);
		setaddr_frames++;
		if (inject_foreign_setaddr_ack) {
			/* An ACK carrying the right address and status but ANOTHER
			 * board's EUI, delivered first: it must not count. */
			uint8_t other[8];

			memset(other, 0x77, sizeof(other));
			int m = apos_frame_setaddr_ack_build(buf, sizeof(buf), 0x00FF, 0,
							     sess, other, addr,
							     APOS_SETADDR_OK);
			deliver(buf, m);
		}
		for (int i = 0; i < NA; i++) {
			struct fake_anchor *a = &A[i];

			if (!a->present || now_ms < a->offline_until || a->old_fw) {
				continue;
			}
			/* The SAME decision function the real anchor calls. */
			enum apos_setaddr_verdict v = apos_setaddr_check(
				a->eui, a->addr, target, addr, UWB_ANCHOR_ADDR_BASE,
				UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS);

			if (v == APOS_SETADDR_IGNORE) {
				continue;
			}
			a->n_setaddr_rx++;

			uint8_t st = APOS_SETADDR_OK;
			uint16_t held = a->addr;
			bool reboot = false;

			if (v == APOS_SETADDR_V_BAD) {
				st = APOS_SETADDR_BAD_ADDR;
			} else if (v == APOS_SETADDR_V_CHANGE) {
				if (a->fail_persist_id) {
					st = APOS_SETADDR_PERSIST_ERR;
				} else {
					held = addr;
					reboot = true;
				}
			}
			if (a->drop_setaddr_acks > 0) {
				/* ACK lost on air: the real anchor does not reboot. */
				a->drop_setaddr_acks--;
				continue;
			}
			int n = apos_frame_setaddr_ack_build(buf, sizeof(buf), a->addr, 0,
							     sess, a->eui, held, st);
			deliver(buf, n);
			if (reboot) {
				a->addr = held;
				a->offline_until = now_ms + a->reboot_ms;
				a->n_reboots++;
			}
		}
		break;
	}
	default:
		break;
	}
}

static void drain_tx(void)
{
	while (txq_head != txq_tail) {
		uint8_t f[64];
		uint16_t n = txq_len[txq_head % TXQ];

		memcpy(f, txq[txq_head % TXQ], n);
		txq_head++;
		handle_frame(f, n);
	}
}

static uint8_t gw_seq;

static void pump_avail(int max_steps, uint32_t avail)
{
	for (int i = 0; i < max_steps && apos_gw_busy(); i++) {
		now_ms += 60;
		apos_gw_step(avail, &gw_seq);
		drain_tx();
	}
}

static void pump(int max_steps)
{
	pump_avail(max_steps, 195000u);
}

/* ---- world setup --------------------------------------------------------- */

static void world(int n_anchors, bool coplanar)
{
	static const float P[4][3] = {
		{0.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f},
		{0.0f, 3.0f, 0.0f}, {3.0f, 3.0f, 1.2f},
	};

	memset(A, 0, sizeof(A));
	for (int i = 0; i < n_anchors && i < NA; i++) {
		A[i].present = true;
		A[i].addr = (uint16_t)(UWB_ANCHOR_ADDR_BASE + i);
		memset(A[i].eui, 0x10 + i, 8);
		if (i < 4) {
			memcpy(A[i].pos, P[i], sizeof(A[i].pos));
		} else {
			A[i].pos[0] = 1.0f * (float)i;
			A[i].pos[1] = 2.0f;
			A[i].pos[2] = 0.5f;
		}
		if (coplanar && i < 4) {
			A[i].pos[2] = 0.0f;
		}
	}
	GW[0] = 1.2f; GW[1] = 1.4f; GW[2] = -1.3f;

	dup_acks = false;
	inject_stale_setpos_ack = false;
	inject_stale_settau_ack = false;
	setpos_frames = settau_frames = to_gateway_frames = 0;
	range_cmds_at_gateway = 0;
	hold_during_setpos = hold_during_settau = false;
	pending_before_first_setpos = false;
	saw_setpos = false;
	txq_head = txq_tail = 0;
	memset(log_count, 0, sizeof(log_count));

	memset(&store_survey, 0, sizeof(store_survey));
	memset(&store_links, 0, sizeof(store_links));
	pending_set = pending_rejected = false;
	pending_gen = 0;
	survey_saves = links_saves = 0;
	fail_pending_write = fail_survey_write = fail_links_write = false;

	memset(&store_map, 0, sizeof(store_map));
	map_rejected = fail_map_write = false;
	map_saves = 0;
	setaddr_frames = 0;
	inject_foreign_setaddr_ack = false;
	for (int i = 0; i < NA; i++) {
		A[i].reboot_ms = 5000;   /* the operator's "good" boot */
	}
}

static void start(bool three_d)
{
	apos_gw_init();
	CHECK(apos_gw_set_gauge(1, 2, 3, three_d ? 4 : -1) == 0);
}

static void run_survey(void)
{
	CHECK(apos_gw_start_run() == 0);
	pump(20000);
	CHECK(!apos_gw_busy());
}

static struct apos_gw_status status(void)
{
	struct apos_gw_status st;

	apos_gw_get_status(&st);
	return st;
}

/* ---- scenarios ----------------------------------------------------------- */

static void test_happy_path_installs_everything_in_order(void)
{
	world(4, false);
	start(true);
	run_survey();

	struct apos_gw_status st = status();

	CHECK(st.have_result && st.gateway_node);
	CHECK(st.n_peers == 5);
	CHECK(apos_gw_accepted());
	CHECK(!apos_gw_gauge_degenerate());
	CHECK(apos_gw_links_verified());
	CHECK(!apos_gw_result_unverified());      /* 10 edges vs 9 free: one spare */
	CHECK(apos_gw_result_redundancy() == 1);
	CHECK(!apos_gw_publish_hold());           /* running a survey does not hold */

	const struct sync_link *lk;
	uint8_t nl = apos_gw_links(&lk);

	CHECK(nl == 4);
	for (int i = 0; i < nl; i++) {
		CHECK(lk[i].valid && lk[i].checked && lk[i].within_gate);
		CHECK(lk[i].tau_dtu > 100 && lk[i].tau_dtu < 2000);
	}

	/* The gateway is NOT commanded and NOT ranged by anyone over the air. */
	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	CHECK(!apos_gw_busy());

	CHECK(setpos_frames == 4);                /* never one to the gateway */
	CHECK(range_cmds_at_gateway == 0);        /* nobody is told to poll the gateway */
	CHECK(settau_frames == 4);
	CHECK(pending_before_first_setpos);       /* marker is on flash FIRST */
	CHECK(hold_during_setpos && hold_during_settau);
	CHECK(!apos_gw_publish_hold());           /* released only at the end */
	CHECK(!apos_store_apply_pending());
	CHECK(strcmp(apos_gw_hold_reason(), "none") == 0);

	CHECK(survey_saves == 1 && links_saves == 1);
	CHECK(store_survey.n_nodes == 5 && store_survey.dim == APOS_GEOM_3D);
	CHECK(store_survey.node[0].short_addr == 1); /* gauge origin first */
	int gw_nodes = 0;
	for (int i = 0; i < store_survey.n_nodes; i++) {
		if (store_survey.node[i].short_addr == UWB_ADDR_GATEWAY_RESERVED) {
			gw_nodes++;
			CHECK(memcmp(store_survey.node[i].eui, apos_gw_eui(), 8) == 0);
		}
	}
	CHECK(gw_nodes == 1);

	CHECK(store_links.n == 4 && store_links.generation == 1);
	CHECK(apos_gw_generation() == 1);
	for (int i = 0; i < 4; i++) {
		const struct apos_link_rec *r = apos_links_find(&store_links, A[i].eui);

		CHECK(r != NULL && r->valid);
		CHECK(A[i].has_tau && A[i].gen == 1);
		CHECK(r && r->tau_dtu == A[i].tau);        /* what it HOLDS == what is stored */
		CHECK(r && memcmp(r->parent_eui, apos_gw_eui(), 8) == 0);
	}
}

static void test_one_anchor_never_acks_leaves_old_survey_and_holds_fixes(void)
{
	world(4, false);
	start(true);
	run_survey();
	A[3].never_acks_settau = true;

	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	CHECK(!apos_gw_busy());

	/* Three anchors took generation 1, one did not. The gateway must NOT
	 * adopt a survey the anchors only partly run. */
	CHECK(A[0].gen == 1 && A[1].gen == 1 && A[2].gen == 1 && !A[3].has_tau);
	CHECK(survey_saves == 0 && links_saves == 0);
	CHECK(apos_gw_generation() == 0);
	CHECK(apos_gw_publish_hold());
	CHECK(strcmp(apos_gw_hold_reason(), "apply-incomplete") == 0);
	CHECK(apos_store_apply_pending());            /* survives a reboot */
	CHECK(log_count[3] > 0);                      /* and it was reported */
	CHECK(A[3].n_settau_rx == 1 + (int)APOS_GW_APPLY_RETRIES);

	/* The anchor comes back; re-applying restates everything idempotently,
	 * and only THEN does the hold lift. */
	A[3].never_acks_settau = false;
	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	CHECK(!apos_gw_publish_hold());
	CHECK(!apos_store_apply_pending());
	CHECK(survey_saves == 1 && links_saves == 1);
	/* Generation is monotonic across the retry, never reused: the earlier
	 * attempt already stamped three anchors with 1. */
	CHECK(apos_gw_generation() == 2 && store_links.generation == 2);
	for (int i = 0; i < 4; i++) {
		CHECK(A[i].gen == 2);
	}
}

static void test_reboot_between_marker_and_commit_stays_held(void)
{
	world(4, false);
	start(true);
	run_survey();
	A[1].never_acks_settau = true;
	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	CHECK(apos_store_apply_pending());

	/* "Reboot": RAM state is gone, NVS (the fake) is what remains. */
	apos_gw_init();
	CHECK(apos_gw_publish_hold());
	CHECK(strcmp(apos_gw_hold_reason(), "interrupted-apply") == 0);
	CHECK(apos_gw_generation() == 0);
	CHECK(store_survey.n_nodes == 0);             /* old survey intact */
}

static void test_unreadable_marker_counts_as_interrupted(void)
{
	world(4, false);
	pending_rejected = true;
	apos_gw_init();
	CHECK(apos_gw_publish_hold());
	CHECK(strcmp(apos_gw_hold_reason(), "interrupted-apply") == 0);
	CHECK(log_count[3] > 0);
}

static void test_lost_ack_is_retried_and_succeeds(void)
{
	world(4, false);
	start(true);
	run_survey();
	A[2].drop_settau_acks = 1;

	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	CHECK(!apos_gw_publish_hold());
	CHECK(A[2].n_settau_rx == 2);                 /* asked twice, applied once */
	CHECK(A[2].gen == 1);
	CHECK(survey_saves == 1 && links_saves == 1);
}

static void test_duplicate_acks_neither_skip_nor_double_count(void)
{
	world(4, false);
	start(true);
	run_survey();
	dup_acks = true;

	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	CHECK(setpos_frames == 4 && settau_frames == 4);
	for (int i = 0; i < 4; i++) {
		CHECK(A[i].n_setpos_rx == 1 && A[i].n_settau_rx == 1);
	}
	CHECK(!apos_gw_publish_hold());
	CHECK(store_links.n == 4);
}

static void test_stale_setpos_ack_is_not_the_settau_answer(void)
{
	world(4, false);
	start(true);
	run_survey();
	inject_stale_setpos_ack = true;
	A[1].never_acks_settau = true; /* its REAL answer never comes */

	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	/* If the stale SETPOS_ACKs had been accepted as SETTAU answers, anchor 1
	 * would have been counted as done and the apply would have committed. */
	CHECK(A[1].n_settau_rx == 1 + (int)APOS_GW_APPLY_RETRIES);
	CHECK(apos_gw_publish_hold());
	CHECK(survey_saves == 0);
}

/* The converse straggler: a SETTAU_ACK that arrives while the SETPOS stage is
 * still running (a duplicate from an earlier apply of the same session, say)
 * must not be taken for the answer to the SETPOS in flight. */
static void test_stale_settau_ack_is_not_the_setpos_answer(void)
{
	world(4, false);
	start(true);
	run_survey();
	inject_stale_settau_ack = true;

	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	/* Taken for an answer, its generation/tau (0/0) would have been counted as
	 * a failed SETTAU and the apply would not have committed. */
	CHECK(!apos_gw_publish_hold());
	CHECK(survey_saves == 1 && links_saves == 1);
	CHECK(setpos_frames == 4 && settau_frames == 4);
}

static void test_wrong_generation_or_status_in_ack_is_a_failure(void)
{
	world(4, false);
	start(true);
	run_survey();
	A[2].settau_status = APOS_SETTAU_PERSIST_ERR; /* applied in RAM, NVS failed */

	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	CHECK(apos_gw_publish_hold());
	CHECK(survey_saves == 0 && links_saves == 0);
	CHECK(A[2].n_settau_rx == 1);                 /* a clear answer is not retried */
	CHECK(log_count[3] > 0);
}

/* An ACK that echoes a DIFFERENT generation than the one sent is a stale
 * duplicate or an anchor that stored something else: not a success, whatever
 * its status byte says. */
static void test_ack_for_the_wrong_generation_is_not_accepted(void)
{
	world(4, false);
	start(true);
	run_survey();
	A[0].ack_stale_gen = true;

	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	CHECK(apos_gw_publish_hold());
	CHECK(survey_saves == 0 && links_saves == 0);
	CHECK(A[0].n_settau_rx == 1);
}

static void test_marker_write_failure_abandons_before_touching_anything(void)
{
	world(4, false);
	start(true);
	run_survey();
	fail_pending_write = true;

	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	CHECK(!apos_gw_busy());
	CHECK(setpos_frames == 0 && settau_frames == 0);
	CHECK(!apos_gw_publish_hold());               /* nothing changed, nothing held */
	CHECK(survey_saves == 0);
	CHECK(log_count[3] > 0);
}

static void test_commit_write_failures_keep_the_hold(void)
{
	world(4, false);
	start(true);
	run_survey();
	fail_links_write = true;

	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	/* Every anchor took it, but the gateway could not record the links: the
	 * marker must stay set and fixes stay suspended. */
	CHECK(A[0].gen == 1 && A[3].gen == 1);
	CHECK(apos_gw_publish_hold());
	CHECK(apos_store_apply_pending());
	CHECK(strcmp(apos_gw_hold_reason(), "apply-incomplete") == 0);
	CHECK(apos_gw_generation() == 0);
}

static void test_coplanar_gauge_is_refused_even_with_force(void)
{
	world(4, true);
	start(true);
	CHECK(apos_gw_start_run() == 0);
	pump(20000);

	struct apos_gw_status st = status();

	if (st.have_result) {
		CHECK(apos_gw_gauge_degenerate());
		CHECK(!apos_gw_accepted());
		CHECK(apos_gw_start_apply(true) == -EDOM);   /* force does not help */
		CHECK(apos_gw_start_apply(false) == -EDOM);
	} else {
		/* The solver itself refused a flat gauge: also a refusal. */
		CHECK(apos_gw_start_apply(true) == -ENODATA);
	}
	CHECK(setpos_frames == 0 && settau_frames == 0);
	CHECK(!apos_gw_publish_hold());
	CHECK(survey_saves == 0);
}

static void test_unverified_link_fails_acceptance_but_force_can_apply(void)
{
	world(4, false);
	start(true);
	A[2].range_silent = true; /* the gateway cannot range this anchor */
	run_survey();

	struct apos_gw_status st = status();

	CHECK(st.have_result);
	CHECK(!apos_gw_links_verified());
	CHECK(!apos_gw_accepted());
	CHECK(apos_gw_start_apply(false) == -EPERM);
	CHECK(!apos_gw_busy() && setpos_frames == 0);
}

static void test_missing_link_cannot_be_forced(void)
{
	/* Anchor 3 is not enumerated at all, so only three anchors answer: a 3D
	 * gauge needs four, and the run refuses before it measures anything. */
	world(3, false);
	start(true);
	CHECK(apos_gw_start_run() == 0);
	pump(20000);
	CHECK(!status().have_result);
	CHECK(apos_gw_start_apply(true) == -ENODATA);
	CHECK(setpos_frames == 0);
}

static void test_survey_is_refused_when_the_gateway_has_no_room(void)
{
	world(8, false);
	start(true);
	CHECK(apos_gw_start_run() == 0);
	pump(20000);
	/* 8 anchors fill the table; the gateway cannot be added, and the survey
	 * must say so instead of silently leaving it out. */
	CHECK(!status().have_result);
	CHECK(!status().gateway_node);
	CHECK(log_count[3] > 0);
}

static void test_2d_flow_is_the_historical_anchors_only_flow(void)
{
	world(4, true);
	/* 2D: origin/xaxis/plane only, 4th anchor rides along. */
	apos_gw_init();
	CHECK(apos_gw_set_gauge(1, 2, 3, -1) == 0);
	run_survey();

	struct apos_gw_status st = status();

	CHECK(st.have_result && !st.gateway_node && st.n_peers == 4);
	const struct sync_link *lk;

	CHECK(apos_gw_links(&lk) == 0);

	CHECK(apos_gw_start_apply(true) == 0);
	pump(20000);
	CHECK(setpos_frames == 4 && settau_frames == 0);
	CHECK(!apos_gw_publish_hold() && !hold_during_setpos);
	CHECK(!pending_before_first_setpos);          /* no marker in the old flow */
	CHECK(survey_saves == 1 && links_saves == 0);
	CHECK(store_survey.n_nodes == 4 && store_survey.dim == APOS_GEOM_2D);
	for (int i = 0; i < store_survey.n_nodes; i++) {
		CHECK(store_survey.node[i].short_addr != UWB_ADDR_GATEWAY_RESERVED);
	}
}

static void test_gateway_cannot_be_a_gauge_reference(void)
{
	world(4, false);
	apos_gw_init();
	CHECK(apos_gw_set_gauge(0, 2, 3, 4) == -EINVAL);   /* origin = gateway */
	CHECK(apos_gw_set_gauge(1, 0, 3, 4) == -EINVAL);
	CHECK(apos_gw_set_gauge(1, 2, 0, 4) == -EINVAL);
	CHECK(apos_gw_set_gauge(1, 2, 3, 0) == -EINVAL);   /* up = gateway */
	CHECK(apos_gw_set_gauge(1, 2, 2, 4) == -EINVAL);   /* repeated */
	CHECK(apos_gw_set_gauge(1, 2, 3, 3) == -EINVAL);
}

static void test_enumeration_order_does_not_change_identities(void)
{
	world(4, false);
	/* Reverse who answers first: addresses and EUIs stay with the boards. */
	struct fake_anchor t = A[0];

	A[0] = A[3];
	A[3] = t;
	start(true);
	run_survey();
	CHECK(apos_gw_accepted());
	CHECK(apos_gw_start_apply(false) == 0);
	pump(20000);
	CHECK(!apos_gw_publish_hold());
	for (int i = 0; i < 4; i++) {
		const struct apos_link_rec *r = apos_links_find(&store_links, A[i].eui);

		CHECK(r && r->tau_dtu == A[i].tau);
	}
	CHECK(store_survey.node[0].short_addr == 1);
}

/* ---- `apos assign` (spec 2026-10-21) ------------------------------------ */

static void make_unassigned(int n)
{
	for (int i = 0; i < n; i++) {
		A[i].addr = UWB_ANCHOR_ADDR_UNASSIGNED;
	}
}

static void run_assign(void)
{
	CHECK(apos_gw_start_assign() == 0);
	pump(20000);
	CHECK(!apos_gw_busy());
}

static bool map_has(int i, uint16_t addr)
{
	int k = apos_addrmap_find_eui(&store_map, A[i].eui);

	return k >= 0 && store_map.e[k].addr == addr;
}

static bool addresses_distinct_and_valid(int n)
{
	for (int i = 0; i < n; i++) {
		if (A[i].addr < UWB_ANCHOR_ADDR_BASE ||
		    A[i].addr >= UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS) {
			return false;
		}
		for (int j = 0; j < i; j++) {
			if (A[i].addr == A[j].addr) {
				return false;
			}
		}
	}
	return true;
}

/* H1 in host form. */
static void test_assign_gives_factory_fresh_boards_distinct_addresses(void)
{
	world(4, false);
	make_unassigned(4);
	apos_gw_init();
	run_assign();

	CHECK(addresses_distinct_and_valid(4));
	CHECK(map_saves == 1);
	CHECK(store_map.n == 4);
	for (int i = 0; i < 4; i++) {
		CHECK(map_has(i, A[i].addr));
		CHECK(A[i].n_reboots == 1);
	}
	/* EUI order: A[0] has the lowest EUI and gets the lowest address. */
	CHECK(A[0].addr == 1 && A[3].addr == 4);
	CHECK(setaddr_frames == 4);
	CHECK(log_count[3] == 0);

	/* A second assign over the converged array changes nothing. */
	setaddr_frames = 0;
	run_assign();
	CHECK(setaddr_frames == 0 && map_saves == 1);
	CHECK(log_count[3] == 0);
}

/* H3 in host form: zero frames, zero reboots, the map simply learns them. */
static void test_assign_adopts_a_hand_numbered_fleet_without_frames(void)
{
	world(4, false);
	apos_gw_init();
	run_assign();

	CHECK(setaddr_frames == 0);
	CHECK(map_saves == 1 && store_map.n == 4);
	for (int i = 0; i < 4; i++) {
		CHECK(A[i].n_reboots == 0);
		CHECK(map_has(i, (uint16_t)(UWB_ANCHOR_ADDR_BASE + i)));
	}
	CHECK(log_count[3] == 0);
}

static void test_assign_resolves_a_duplicate_lowest_eui_keeps_it(void)
{
	world(4, false);
	A[1].addr = 1;   /* same `anchor id` as A[0], which has the lower EUI */
	apos_gw_init();
	run_assign();

	CHECK(A[0].addr == 1 && A[0].n_reboots == 0);
	CHECK(A[1].addr == 2 && A[1].n_reboots == 1);   /* the only free one */
	CHECK(addresses_distinct_and_valid(4));
	CHECK(setaddr_frames == 1);
	CHECK(log_count[3] == 0);
}

static void test_assign_lost_ack_is_retried(void)
{
	world(4, false);
	make_unassigned(4);
	A[2].drop_setaddr_acks = 1;
	apos_gw_init();
	run_assign();

	CHECK(A[2].n_setaddr_rx == 2);   /* asked twice, rebooted once */
	CHECK(A[2].n_reboots == 1);
	CHECK(addresses_distinct_and_valid(4));
	CHECK(map_has(2, A[2].addr));
	CHECK(log_count[3] == 0);
}

/* H7 in host form. */
static void test_assign_old_firmware_is_reported_and_not_reserved(void)
{
	world(4, false);
	make_unassigned(4);
	A[3].old_fw = true;
	apos_gw_init();
	run_assign();

	CHECK(log_count[3] > 0);                         /* reported */
	CHECK(apos_addrmap_find_eui(&store_map, A[3].eui) < 0);   /* not reserved */
	CHECK(A[3].addr == UWB_ANCHOR_ADDR_UNASSIGNED);
	/* The others took theirs and are in the map. */
	for (int i = 0; i < 3; i++) {
		CHECK(map_has(i, A[i].addr));
	}
	/* Asked exactly 1 + retries times: no second SEND cycle. */
	CHECK(setaddr_frames == 3 + 1 + (int)APOS_GW_APPLY_RETRIES);
}

static void test_assign_foreign_ack_is_not_counted(void)
{
	world(4, false);
	make_unassigned(1);
	A[0].old_fw = true;
	inject_foreign_setaddr_ack = true;
	apos_gw_init();
	run_assign();

	CHECK(log_count[3] > 0);
	CHECK(apos_addrmap_find_eui(&store_map, A[0].eui) < 0);
}

static void test_assign_map_write_failure_aborts(void)
{
	world(4, false);
	make_unassigned(2);
	fail_map_write = true;
	apos_gw_init();
	run_assign();

	CHECK(map_saves == 0);
	CHECK(log_count[3] > 0);
}

/* The map write is a flash write on the beacon-owning loop: it must wait for a
 * step with the whole solve budget in hand, however long that takes. */
static void test_assign_map_write_waits_for_budget(void)
{
	world(4, false);
	make_unassigned(2);
	apos_gw_init();
	CHECK(apos_gw_start_assign() == 0);
	pump_avail(2000, APOS_GW_STEP_BUDGET_UUS);       /* enough to transmit */
	CHECK(apos_gw_busy());
	CHECK(map_saves == 0);
	pump(20000);
	CHECK(!apos_gw_busy());
	CHECK(map_saves == 1);
}

static void test_assign_slow_boot_inside_window_converges(void)
{
	world(4, false);
	make_unassigned(4);
	for (int i = 0; i < 4; i++) {
		A[i].reboot_ms = 22000;   /* the hung-WiFi-driver boot */
	}
	apos_gw_init();
	run_assign();

	CHECK(addresses_distinct_and_valid(4));
	CHECK(log_count[3] == 0);
}

static void test_assign_board_that_never_returns_fails_after_deadline(void)
{
	world(4, false);
	make_unassigned(4);
	A[1].reboot_ms = 10 * 60 * 1000;
	apos_gw_init();

	int64_t t0 = now_ms;

	run_assign();
	CHECK(now_ms - t0 >= (int64_t)APOS_GW_ASSIGN_CONVERGE_MS);
	CHECK(log_count[3] > 0);
	/* Persisted anyway: it ACKed and holds the address, it just did not
	 * come back in time to prove it. */
	CHECK(map_has(1, A[1].addr));
}

static void test_assign_clears_the_gauge_when_addresses_move(void)
{
	world(4, false);
	A[1].addr = 1;
	apos_gw_init();
	CHECK(apos_gw_set_gauge(1, 3, 4, -1) == 0);
	run_assign();
	CHECK(!apos_gw_gauge_set());

	/* No address moves: the gauge survives. */
	CHECK(apos_gw_set_gauge(1, 3, 4, -1) == 0);
	run_assign();
	CHECK(apos_gw_gauge_set());
}

static void test_run_refuses_with_unassigned_boards(void)
{
	world(4, false);
	A[2].addr = UWB_ANCHOR_ADDR_UNASSIGNED;
	start(false);
	CHECK(apos_gw_start_run() == 0);
	pump(20000);
	CHECK(!apos_gw_busy());
	CHECK(!status().have_result);
	CHECK(status().meas_total == 0);   /* stopped before ranging anything */
	CHECK(log_count[3] > 0);
	CHECK(setaddr_frames == 0);   /* a run never re-addresses by itself */
}

static void test_assign_refuses_over_an_unreadable_map(void)
{
	world(4, false);
	map_rejected = true;
	apos_gw_init();
	CHECK(apos_gw_start_assign() == -EIO);
	CHECK(apos_gw_forget(NULL) == 0);   /* the explicit clear */
	CHECK(!map_rejected);
	CHECK(apos_gw_start_assign() == 0);
	pump(20000);
}

int main(void)
{
	verbose = getenv("APOS_GW_TEST_VERBOSE") != NULL;

	test_happy_path_installs_everything_in_order();
	test_one_anchor_never_acks_leaves_old_survey_and_holds_fixes();
	test_reboot_between_marker_and_commit_stays_held();
	test_unreadable_marker_counts_as_interrupted();
	test_lost_ack_is_retried_and_succeeds();
	test_duplicate_acks_neither_skip_nor_double_count();
	test_stale_setpos_ack_is_not_the_settau_answer();
	test_stale_settau_ack_is_not_the_setpos_answer();
	test_wrong_generation_or_status_in_ack_is_a_failure();
	test_ack_for_the_wrong_generation_is_not_accepted();
	test_marker_write_failure_abandons_before_touching_anything();
	test_commit_write_failures_keep_the_hold();
	test_coplanar_gauge_is_refused_even_with_force();
	test_unverified_link_fails_acceptance_but_force_can_apply();
	test_missing_link_cannot_be_forced();
	test_survey_is_refused_when_the_gateway_has_no_room();
	test_2d_flow_is_the_historical_anchors_only_flow();
	test_gateway_cannot_be_a_gauge_reference();
	test_enumeration_order_does_not_change_identities();
	test_assign_gives_factory_fresh_boards_distinct_addresses();
	test_assign_adopts_a_hand_numbered_fleet_without_frames();
	test_assign_resolves_a_duplicate_lowest_eui_keeps_it();
	test_assign_lost_ack_is_retried();
	test_assign_old_firmware_is_reported_and_not_reserved();
	test_assign_foreign_ack_is_not_counted();
	test_assign_map_write_failure_aborts();
	test_assign_map_write_waits_for_budget();
	test_assign_slow_boot_inside_window_converges();
	test_assign_board_that_never_returns_fails_after_deadline();
	test_assign_clears_the_gauge_when_addresses_move();
	test_run_refuses_with_unassigned_boards();
	test_assign_refuses_over_an_unreadable_map();

	if (g_fail) {
		printf("%d CHECK(s) FAILED\n", g_fail);
		return 1;
	}
	printf("apos_gw: ALL TESTS PASSED\n");
	return 0;
}
