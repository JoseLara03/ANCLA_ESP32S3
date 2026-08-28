/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tdoa_gw.h"

#include "apos_store.h"
#include "net_uplink.h"
#include "pos_json.h"
#include "pos_sink.h"
#include "pos_solver.h"
#include "tag_id.h"
#include "tdoa_collect.h"
#include "tdoa_dtu.h"
#include "tdoa_solve.h"
#include "uwb_config.h"
#include "uwb_debug.h"
#include "uwb_frame_802_15_4z.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(tdoa_gw, ANCLA_LOG_LEVEL);

/* Per-tag memory: the seed for the next solve, the last known battery, and the
 * plausibility reference for the jump gate. Same order of magnitude as the
 * collector's slot count on purpose -- a tag with an open group is exactly a
 * tag that will want a seed. */
#define TDOA_GW_SEED_SLOTS   16u

/* A seed older than this is not evidence about where the tag is now. At 5 Hz a
 * live tag refreshes it every 200 ms, so 1 s is five missed blinks. */
#define TDOA_GW_SEED_AGE_MS  1000

/* Largest jump accepted against a seed younger than TDOA_GW_SEED_AGE_MS.
 * 10 m in <= 1 s is 36 km/h: impossible for a person carrying a tag, and it is
 * the ONLY guard against the mirror-branch failure tdoa_solve.h warns about --
 * a tag outside the anchor hull can converge on the reflected solution and
 * still report valid = true, and its residual will not say so. Deliberately
 * generous: this rejects the branch flip (which lands metres away), not
 * ordinary noise. */
#define TDOA_GW_MAX_JUMP_M   10.0f

struct tag_memo {
	uint16_t tag_addr;
	float    x, y;
	/* last_ms is touched by every observation and drives LRU eviction;
	 * pos_ms is touched only when x/y are written and is what the seed's
	 * freshness is judged on. Two fields deliberately: an observation
	 * carrying only a battery reading must not make a stale POSITION look
	 * fresh, which is exactly what one shared timestamp would do. */
	uint32_t last_ms;
	uint32_t pos_ms;
	uint8_t  batt_soc;
	bool     valid;
	bool     has_pos;   /* x/y are a real previous fix, not zero-init */
};

/* All module state static -- the gateway loop runs on the 4096-byte main stack,
 * where `main` already peaks at 1748 B during apos's do_solve(), and a 2588-byte
 * automatic in THIS thread once overflowed that stack in total silence (no log
 * line, no fatal dump, not even a k_timer ISR report; see CLAUDE.md). struct
 * tdoa_collect alone measures 1792 B, so as an automatic here it would sit at
 * ~3540/4096 before tdoa_solve()'s own frame, and Xtensa's windowed ABI spills
 * register windows on top of every declared frame. Static is also simply
 * correct: there is exactly one collector for the life of the process. */
static struct tdoa_collect collect;
static struct tag_memo     memo[TDOA_GW_SEED_SLOTS];

static uint32_t n_obs_in;
static uint32_t n_dup;
static uint32_t n_shed;
static uint32_t n_fix_out;
static uint32_t n_no_anchor;
static uint32_t n_implausible;
static uint32_t n_solve_fail;
static uint32_t n_jump;

/* Warn ONCE PER BOOT for each of these, and let the counters carry the rest.
 * All three conditions are per-observation or per-fix and all three persist for
 * as long as their cause does, so an unconditional LOG_WRN is 4 x 5 x tags
 * records a second enqueued from the K_PRIO_COOP(0) beacon loop, forever. The
 * enqueue itself is cheap under deferred logging, but CONFIG_LOG_MODE_OVERFLOW
 * then OVERWRITES older records -- including the very lines an operator would
 * be reading to work out why the gateway is unsurveyed. Same instinct as the
 * apos/ccp rate-limited summaries; `blink stats` is where the magnitudes live.
 *
 * THE GAP, stated rather than left to be discovered: once per BOOT, not once
 * per episode. A 4-anchor deployment that DEGRADES to 3 anchors mid-session
 * warns once and then stays quiet even though the condition is new, and a
 * survey applied while the gateway runs silences nothing that already fired.
 * Accepted: the counters (`blink stats`) are the live signal, and a warning
 * that can re-arm needs an episode notion this module has no reason to own. */
static bool warned_blind_residual;
static bool warned_no_anchor;
static bool warned_shed;
static bool warned_implausible;
static bool warned_solve_fail;
static bool warned_jump;

void tdoa_gw_init(void)
{
	tdoa_collect_init(&collect);
	memset(memo, 0, sizeof(memo));
	n_obs_in      = 0u;
	n_dup         = 0u;
	n_shed        = 0u;
	n_fix_out     = 0u;
	n_no_anchor   = 0u;
	n_implausible = 0u;
	n_solve_fail  = 0u;
	n_jump        = 0u;
	warned_blind_residual = false;
	warned_no_anchor = false;
	warned_shed = false;
	warned_implausible = false;
	warned_solve_fail = false;
	warned_jump = false;
}

/* Where anchor `anchor_id` is, from the applied survey.
 *
 * The survey is keyed by short address, and an anchor's short address is
 * UWB_ANCHOR_ADDR_BASE + anchor_id (the MAC contract reserves 0x0000 for the
 * gateway, which is why the console's 0-based id cannot go on the wire). An
 * unsurveyed gateway returns false for every anchor and therefore publishes no
 * fixes at all -- which is correct and is the same refusal
 * anchor_respond_wave_poll() makes: coordinates that were never measured
 * produce a confidently meaningless position. */
static bool anchor_xyz(uint8_t anchor_id, float *x, float *y, float *z)
{
	const struct apos_survey *s = apos_store_get();
	uint16_t want;
	uint8_t i;

	if (s == NULL || !s->valid || anchor_id >= UWB_MAX_ANCHORS) {
		return false;
	}

	want = (uint16_t)(UWB_ANCHOR_ADDR_BASE + anchor_id);

	for (i = 0u; i < s->n_nodes; i++) {
		if (s->node[i].short_addr == want) {
			*x = s->node[i].x;
			*y = s->node[i].y;
			*z = s->node[i].z;
			return true;
		}
	}
	return false;
}

static struct tag_memo *memo_find(uint16_t tag_addr)
{
	uint8_t i;

	for (i = 0u; i < TDOA_GW_SEED_SLOTS; i++) {
		if (memo[i].valid && memo[i].tag_addr == tag_addr) {
			return &memo[i];
		}
	}
	return NULL;
}

/* An entry for `tag_addr`: the existing one, a free slot, or the oldest one.
 * Never NULL.
 *
 * Evicting the OLDEST is right HERE and would be wrong in tdoa_collect -- do
 * not "unify" the two. This is a per-tag cache where every entry is equally
 * complete and age is the only signal of whether it still describes a live tag;
 * the collector's slots hold PARTIAL groups, where age correlates with being
 * closer to COMPLETE, so it evicts by completeness first and never evicts a
 * releasable group while a less-complete one exists (see its header's
 * slot-exhaustion note). Same words, opposite conclusion, different data. */
static struct tag_memo *memo_claim(uint16_t tag_addr, uint32_t now_ms)
{
	struct tag_memo *oldest = &memo[0];
	uint8_t i;

	for (i = 0u; i < TDOA_GW_SEED_SLOTS; i++) {
		if (memo[i].valid && memo[i].tag_addr == tag_addr) {
			memo[i].last_ms = now_ms;
			return &memo[i];
		}
		if (!memo[i].valid) {
			memo[i].valid    = true;
			memo[i].has_pos  = false;
			memo[i].tag_addr = tag_addr;
			memo[i].last_ms  = now_ms;
			memo[i].pos_ms   = now_ms;
			memo[i].batt_soc = UWB_FRAME_POS_SOC_UNKNOWN;
			return &memo[i];
		}
		/* Signed difference: now_ms wraps freely. */
		if ((int32_t)(memo[i].last_ms - oldest->last_ms) < 0) {
			oldest = &memo[i];
		}
	}

	memset(oldest, 0, sizeof(*oldest));
	oldest->valid    = true;
	oldest->tag_addr = tag_addr;
	oldest->last_ms  = now_ms;
	oldest->pos_ms   = now_ms;
	oldest->batt_soc = UWB_FRAME_POS_SOC_UNKNOWN;
	return oldest;
}

/* Why tdoa_collect_add() would refuse this observation, decided BEFORE the call
 * so the two causes can be counted apart. Reads only public struct fields and
 * costs one bounded scan of TDOA_COLLECT_SLOTS; it changes nothing.
 *
 * Necessary because tdoa_collect_add() returns a bare bool for two conditions
 * that call for opposite actions: a duplicate is harmless, while load shedding
 * means this module is not draining fast enough. Discarding that distinction
 * would leave a shedding gateway looking exactly like anchors that went quiet.
 * Returns true when the refusal (if any) is load shedding. */
static bool refusal_is_shedding(const struct tdoa_obs *o)
{
	bool all_releasable = true;
	uint8_t i;

	for (i = 0u; i < TDOA_COLLECT_SLOTS; i++) {
		const struct tdoa_group *g = &collect.slot[i];

		if (!g->used) {
			/* A free slot exists, so it cannot have been
			 * shedding. */
			return false;
		}
		if (g->tag_addr == o->tag_addr &&
		    g->blink_seq == o->blink_seq) {
			/* The group exists, so the refusal was the duplicate
			 * anchor bit -- never shedding. */
			return false;
		}
		if (g->n < TDOA_MIN_ANCHORS) {
			all_releasable = false;
		}
	}
	return all_releasable;
}

/* One observation from the uplink into the collector. Returns false only when
 * the queue was empty, so the caller can stop early. */
static bool ingest_one(uint32_t now_ms)
{
	struct pos_blink_obs obs;
	struct tdoa_obs t;
	struct tag_memo *mm;
	float z = 0.0f;

	if (!net_uplink_get_obs(&obs)) {
		return false;
	}

	if (!anchor_xyz(obs.anchor_id, &t.meas.x, &t.meas.y, &z)) {
		n_no_anchor++;
		if (!warned_no_anchor) {
			warned_no_anchor = true;
			LOG_WRN("observation from anchor %u: not in the applied "
				"survey - dropped. Warned ONCE; `blink stats` "
				"carries the count. On an unsurveyed gateway "
				"EVERY observation lands here",
				obs.anchor_id);
		}
		return true;
	}

	/* dz follows struct pos_meas's convention: anchor z minus TAG z, and
	 * the tag's z is unmeasured, so it is assumed 0 -- exactly the
	 * assumption the tag's own solver already makes with the coordinates
	 * a WAVE response carries. A survey run in APOS_GEOM_2D mode pins
	 * every z at 0 anyway, which is the deployment this project has. */
	t.meas.dz    = z;
	t.meas.t_dtu = obs.t_dtu;
	t.tag_addr   = obs.tag_addr;
	t.blink_seq  = obs.blink_seq;
	t.anchor_id  = obs.anchor_id;

	/* The battery reading rides on the observation, not on the group: the
	 * collector carries only struct tdoa_meas, so it is remembered here
	 * per tag and read back when the fix is built. */
	mm = memo_claim(obs.tag_addr, now_ms);
	mm->batt_soc = obs.batt_soc;

	if (tdoa_collect_add(&collect, &t, now_ms)) {
		n_obs_in++;
	} else if (refusal_is_shedding(&t)) {
		/* Every slot already held a releasable group, so the collector
		 * refused this observation rather than destroying a fix that
		 * already exists. Counted separately because a gateway shedding
		 * load this way otherwise looks identical to anchors that
		 * stopped publishing. */
		n_shed++;
		if (!warned_shed) {
			warned_shed = true;
			LOG_WRN("observation from anchor %u shed: all %u "
				"collector slots hold releasable groups - "
				"tdoa_gw_step() is not draining fast enough. "
				"Warned ONCE; `blink stats` carries the count",
				obs.anchor_id,
				(unsigned int)TDOA_COLLECT_SLOTS);
		}
	} else {
		n_dup++;
	}
	return true;
}

/* One ready group into one published fix. Returns false when no group was
 * ready, so the caller can stop early. */
static bool solve_one(const struct gw_core_ctx *ctx, uint32_t now_ms)
{
	struct tdoa_meas m[POS_MAX_ANCHORS];
	struct pos_result res;
	struct pos_fix fix;
	struct tag_memo *mm;
	uint8_t eui[UWB_FRAME_EUI_LEN];
	const float *seed = NULL;
	float seed_xy[2];
	size_t n = 0u;
	uint16_t tag_addr = 0u;

	if (!tdoa_collect_take_ready(&collect, now_ms, m, &n, &tag_addr)) {
		return false;
	}

	/* Absolute 40-bit timestamps in, signed differences out. Must happen
	 * before the solver sees them and before the plausibility test. */
	tdoa_dtu_rebase(m, n);

	if (!tdoa_dtu_plausible(m, n)) {
		n_implausible++;
		if (!warned_implausible) {
			warned_implausible = true;
			LOG_WRN("blink from 0x%04X: implausible spread - "
				"dropped (broken sync, or a wrap this rebase "
				"did not fix). Warned ONCE; `blink stats` "
				"carries the count", tag_addr);
		}
		return true;
	}

	mm = memo_find(tag_addr);
	if (mm != NULL && mm->has_pos) {
		int32_t age = (int32_t)(now_ms - mm->pos_ms);

		if (age >= 0 && age < TDOA_GW_SEED_AGE_MS) {
			seed_xy[0] = mm->x;
			seed_xy[1] = mm->y;
			seed = seed_xy;
		}
	}

	if (!tdoa_solve(m, n, seed, &res) || !res.valid) {
		n_solve_fail++;
		if (!warned_solve_fail) {
			warned_solve_fail = true;
			LOG_WRN("blink from 0x%04X: solve failed over %u "
				"anchors. Warned ONCE; `blink stats` carries "
				"the count", tag_addr, (unsigned int)n);
		}
		return true;
	}

	/* The mirror-branch gate. tdoa_solve()'s header says outright that a
	 * tag outside the anchor hull can converge on the reflected solution
	 * and still report valid, and that its residual is too weak to tell --
	 * zero by construction at n == 3. A recent previous fix is the
	 * corroboration that header asks for. */
	if (seed != NULL) {
		float dx = res.x - seed_xy[0];
		float dy = res.y - seed_xy[1];

		if (sqrtf(dx * dx + dy * dy) > TDOA_GW_MAX_JUMP_M) {
			n_jump++;
			if (!warned_jump) {
				warned_jump = true;
				LOG_WRN("blink from 0x%04X: fix (%.2f, %.2f) "
					"jumps more than %.1f m from the last "
					"one - dropped. Warned ONCE; `blink "
					"stats` carries the count",
					tag_addr, (double)res.x, (double)res.y,
					(double)TDOA_GW_MAX_JUMP_M);
			}
			return true;
		}
	}

	mm = memo_claim(tag_addr, now_ms);
	mm->x       = res.x;
	mm->y       = res.y;
	mm->pos_ms  = now_ms;
	mm->has_pos = true;

	fix.src_addr   = tag_addr;
	fix.x          = res.x;
	fix.y          = res.y;
	fix.n_anchors  = res.n_used;
	fix.batt_soc   = mm->batt_soc;

	/* tdoa_solve.h's caller contract, honoured here: at TDOA_MIN_ANCHORS the
	 * system is exactly determined, so residual_m is numerically zero
	 * however wrong the timestamps were. Zeroed explicitly rather than
	 * forwarded, so nothing downstream can read a near-zero float off
	 * pos_sink's console line as evidence of a good fit. From n_used == 4
	 * there is one spare equation and it carries (weak) information, so it
	 * is forwarded. */
	if (res.n_used <= TDOA_MIN_ANCHORS) {
		fix.residual_m = 0.0f;
		if (!warned_blind_residual) {
			warned_blind_residual = true;
			LOG_WRN("TDoA fixes are being solved over %u anchors: "
				"`residual` on the console line is ZERO BY "
				"CONSTRUCTION at this anchor count and is NOT "
				"a quality signal - a fourth surveyed anchor "
				"is what makes it one",
				(unsigned int)res.n_used);
		}
	} else {
		fix.residual_m = res.residual_m;
	}

	/* Identical Tid derivation to the 0xEA path in uwb_gateway.c, including
	 * the fallback and its cost: a straggler whose seat expired gets
	 * tag_id = src_addr and shows on the platform as a one-record phantom
	 * device. Documented and accepted there; unchanged here on purpose --
	 * two different derivations of Tid would be a worse bug than the
	 * phantom. */
	/* The one LOG_WRN in this module left UNRATE-LIMITED, deliberately: it
	 * is the same line, for the same event, that the 0xEA path in
	 * uwb_gateway.c already logs on every straggler, and it has no counter
	 * of its own -- rate-limiting it here would make the two paths disagree
	 * and leave the event invisible. It is also self-limiting: a tag with no
	 * seat is a tag whose lease expired, not a steady state. */
	if (gw_core_find_eui(ctx, tag_addr, eui)) {
		fix.tag_id = tag_id_from_eui(eui, UWB_FRAME_EUI_LEN);
	} else {
		LOG_WRN("TDoA fix from 0x%04X: no live seat, Tid falls back to "
			"short address", tag_addr);
		fix.tag_id = tag_addr;
	}

	pos_sink_publish(&fix);
	n_fix_out++;
	return true;
}

void tdoa_gw_step(const struct gw_core_ctx *ctx, uint32_t now_ms)
{
	unsigned int i;

	for (i = 0u; i < TDOA_GW_INGEST_MAX; i++) {
		if (!ingest_one(now_ms)) {
			break;
		}
	}

	for (i = 0u; i < TDOA_GW_SOLVE_MAX; i++) {
		if (!solve_one(ctx, now_ms)) {
			break;
		}
	}
}

void tdoa_gw_stats(uint32_t *n_obs, uint32_t *n_reject, uint32_t *n_fix,
		   uint32_t *no_anchor, uint32_t *implausible,
		   uint32_t *solve_fail, uint32_t *jump)
{
	if (n_obs != NULL)       { *n_obs = n_obs_in; }
	if (n_reject != NULL)    { *n_reject = n_dup + n_shed; }
	if (n_fix != NULL)       { *n_fix = n_fix_out; }
	if (no_anchor != NULL)   { *no_anchor = n_no_anchor; }
	if (implausible != NULL) { *implausible = n_implausible; }
	if (solve_fail != NULL)  { *solve_fail = n_solve_fail; }
	if (jump != NULL)        { *jump = n_jump; }
}

void tdoa_gw_reject_detail(uint32_t *dup, uint32_t *shed)
{
	if (dup != NULL)  { *dup = n_dup; }
	if (shed != NULL) { *shed = n_shed; }
}
