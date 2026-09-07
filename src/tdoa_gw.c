/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tdoa_gw.h"

#include "apos_store.h"
#include "blink_frame.h"   /* BLINK_FLAG_MOVING */
#include "net_uplink.h"
#include "pos_abg.h"
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

	/* Reference anchor's ABSOLUTE 40-bit t_dtu from this tag's last
	 * accepted group, captured before tdoa_dtu_rebase() -- the instant of
	 * that fix, in hardware, at 15.65 ps. It is what the out-of-order
	 * check in solve_one() measures against, and it is the clock any
	 * per-tag filter on this path takes its dt from (the EKF did until
	 * 2026-09-06; pos_abg does now -- see solve_one() and
	 * docs/superpowers/specs/2026-09-06-abg-position-filter-design.md).
	 * `has_ref_t` distinguishes "no previous group yet" (fresh memo slot)
	 * from a genuine dt anomaly; without it a freshly claimed slot would
	 * read as an anomaly rather than a cold start. */
	int64_t  last_ref_t_dtu;
	bool     has_ref_t;

	/* The filter (spec §4.1). x/y/pos_ms/has_pos above keep their meaning:
	 * the last PUBLISHED position, which is what seeds tdoa_solve() and
	 * jump-gates it; they follow the filter's output at publish time. */
	struct pos_abg abg;
	/* Latched from the last observation's BLINK_FLAG_MOVING in
	 * ingest_one(). Every observation of one blink carries the same flags
	 * byte (one frame, several anchors), so the last to arrive is as good
	 * as any. Its ABSENCE (proto-4 anchor) reads as still -- see the
	 * `still` counter. */
	bool     tag_moving;
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

/* One shared filter config for every tag -- there is no per-tag tuning.
 * Written only by the shell setters below (k_sched_lock()-fenced) and read
 * by solve_one(). abg_lambda is what the last set_lambda() was given, kept
 * only so `blink abg` can print it back. */
static struct pos_abg_cfg abg_cfg;
static float abg_lambda;

static uint32_t n_obs_in;
static uint32_t n_dup;
static uint32_t n_shed;
static uint32_t n_fix_out;
static uint32_t n_no_anchor;
static uint32_t n_implausible;
static uint32_t n_solve_fail;
static uint32_t n_jump;

/* Groups discarded because they were released out of order -- see
 * TDOA_DT_REORDER_MAX_MS. Not a subset of any other counter: such a group
 * is never solved, so `tdoa.fixes` does not count it, and it is not a
 * solve_fail either. */
static uint32_t n_reorder;
static bool warned_reorder;

static uint32_t n_abg_seeded;
static uint32_t n_abg_dt_reseed;
static uint32_t n_abg_filtered;
static uint32_t n_abg_gate_rejected;
static uint32_t n_abg_reseed;
static uint32_t n_abg_still;
static bool warned_gate;
static bool warned_dt_reseed;

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
	pos_abg_cfg_defaults(&abg_cfg);
	abg_lambda = 0.02f;
	n_obs_in      = 0u;
	n_dup         = 0u;
	n_shed        = 0u;
	n_fix_out     = 0u;
	n_no_anchor   = 0u;
	n_implausible = 0u;
	n_solve_fail  = 0u;
	n_jump        = 0u;
	n_reorder     = 0u;
	warned_reorder = false;
	warned_blind_residual = false;
	warned_no_anchor = false;
	warned_shed = false;
	warned_implausible = false;
	warned_solve_fail = false;
	warned_jump = false;
	n_abg_seeded        = 0u;
	n_abg_dt_reseed     = 0u;
	n_abg_filtered      = 0u;
	n_abg_gate_rejected = 0u;
	n_abg_reseed        = 0u;
	n_abg_still         = 0u;
	warned_gate         = false;
	warned_dt_reseed    = false;
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
			memo[i].batt_soc = UWB_FRAME_POS_SOC_CONNECTED;
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
	oldest->batt_soc = UWB_FRAME_POS_SOC_CONNECTED;
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

	/* dz follows struct pos_meas's convention: anchor z minus TAG z.
	 *
	 * The tag's z is still unmeasured -- nothing on the wire carries it --
	 * but it is no longer ASSUMED to be zero: apos_store's tag_z_m is the
	 * site's answer, set once with `apos tagz`. It defaults to 0.0, which
	 * is exactly the old behaviour, so an unconfigured gateway reports the
	 * same numbers it did before.
	 *
	 * Do not "simplify" this away on the grounds that a 2D survey pins
	 * every anchor at z = 0 and a common dz cancels in a range difference.
	 * It does not cancel: sqrt(rho^2 + dz^2) is nonlinear in rho, so a
	 * uniform dz compresses the differences and solving with dz = 0
	 * against a real 1.4 m separation pulls the reported positions INWARD,
	 * by 0.23 m at 1 m from centre on this project's 2.5 m array. The
	 * numbers and the full statement are on tag_z_m in apos_store.h. */
	t.meas.dz    = z - apos_store_get()->tag_z_m;

	/* The publishing anchor's own measured timestamp jitter, DTU -> metres
	 * of path. 0 means the anchor sent none (too few CCP residuals yet, or
	 * firmware older than proto 5), forwarded as 0.0f, which every consumer
	 * must read as UNKNOWN rather than as zero uncertainty. Since the EKF's
	 * removal (2026-09-06) nothing on this path consumes it -- tdoa_solve()
	 * is unweighted by design -- but it is on the wire regardless, and
	 * carrying it costs nothing (see struct tdoa_meas). */
	t.meas.sigma_m = (float)obs.sigma_dtu * TDOA_M_PER_DTU;
	t.meas.t_dtu = obs.t_dtu;
	t.tag_addr   = obs.tag_addr;
	t.blink_seq  = obs.blink_seq;
	t.anchor_id  = obs.anchor_id;

	/* The battery reading rides on the observation, not on the group: the
	 * collector carries only struct tdoa_meas, so it is remembered here
	 * per tag and read back when the fix is built. */
	mm = memo_claim(obs.tag_addr, now_ms);
	mm->batt_soc = obs.batt_soc;
	/* Anything that rides on the OBSERVATION rather than on the geometry
	 * is remembered per tag here and read back when the fix is built --
	 * the collector carries only struct tdoa_meas. */
	mm->tag_moving = (obs.flags & BLINK_FLAG_MOVING) != 0u;

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

/* Signed 40-bit difference, same discipline as sync_model.c's sdelta40() and
 * tdoa_dtu.c's rebase -- this project's "every timestamp comparison is a
 * signed difference" rule, applied at the DW3220's own 40-bit modulo. Local
 * copy: same precedent as sync_model.c and ccp_slave.c, which each keep
 * their own rather than sharing a header for a four-line function. */
static int64_t sdelta40(uint64_t a, uint64_t b)
{
	uint64_t d = (a - b) & 0xFFFFFFFFFFULL;

	if (d & (1ULL << 39)) {
		return (int64_t)d - (int64_t)(1ULL << 40);
	}
	return (int64_t)d;
}

/* Seconds per DW3220 device time unit: 1 / (499.2 MHz x 128). Matches
 * sync_model.h's "1 DTU = 15.65 ps" (SYNC_DTU_PER_NS = 64, rounded); kept as
 * its own local constant rather than pulling in sync_model.h for one
 * conversion factor this module has no other use for. */
#define TDOA_GW_S_PER_DTU  1.56498e-11f

/* One fresh tdoa_solve() against the group, seeded from the last PUBLISHED
 * position when that is recent, plus the mirror-branch jump gate against that
 * same position. Returns false (n_solve_fail or n_jump already counted and
 * warned) on a solve failure or a rejected jump; true, with *out filled in,
 * on success. Touches nothing in `mm`: the caller decides what to publish. */
static bool resolve_one(const struct tag_memo *mm, const struct tdoa_meas *m,
			size_t n, uint16_t tag_addr, uint32_t now_ms,
			struct pos_result *out)
{
	float seed_xy[2];
	const float *seed = NULL;

	if (mm->has_pos) {
		int32_t age = (int32_t)(now_ms - mm->pos_ms);

		if (age >= 0 && age < TDOA_GW_SEED_AGE_MS) {
			seed_xy[0] = mm->x;
			seed_xy[1] = mm->y;
			seed = seed_xy;
		}
	}

	if (!tdoa_solve(m, n, seed, out) || !out->valid) {
		n_solve_fail++;
		if (!warned_solve_fail) {
			warned_solve_fail = true;
			LOG_WRN("blink from 0x%04X: solve failed over %u "
				"anchors. Warned ONCE; `blink stats` carries "
				"the count", tag_addr, (unsigned int)n);
		}
		return false;
	}

	/* The mirror-branch gate. tdoa_solve()'s header says outright that a
	 * tag outside the anchor hull can converge on the reflected solution
	 * and still report valid, and that its residual is too weak to tell --
	 * zero by construction at n == 3. A recent previous fix is the
	 * corroboration that header asks for. */
	if (seed != NULL) {
		float dx = out->x - seed_xy[0];
		float dy = out->y - seed_xy[1];

		if (sqrtf(dx * dx + dy * dy) > TDOA_GW_MAX_JUMP_M) {
			n_jump++;
			if (!warned_jump) {
				warned_jump = true;
				LOG_WRN("blink from 0x%04X: fix (%.2f, %.2f) "
					"jumps more than %.1f m from the last "
					"one - dropped. Warned ONCE; `blink "
					"stats` carries the count",
					tag_addr, (double)out->x,
					(double)out->y,
					(double)TDOA_GW_MAX_JUMP_M);
			}
			return false;
		}
	}

	return true;
}

/*
 * REMOVED 2026-09-06: the per-tag constant-velocity EKF (pos_ekf, added
 * 2026-09-02 and extended 2026-09-03 with per-anchor weighting, the tag's
 * MOVING bit as its ZUPT gate, and a temporary trace ring). Every fix this
 * function publishes is now the RAW tdoa_solve() result again, exactly as it
 * was before 2026-09-02.
 *
 * Why: measured on hardware, the EKF's tightly-coupled range-difference
 * update did not give a better-looking track than the raw solve. With only a
 * 3-axis accelerometer on the tag (no gyroscope) its motion model has nothing
 * to steer it, and on this site's thin 3-anchor geometry a single accepted
 * equation per cycle was enough to keep its divergence recovery disarmed
 * while it ran away (2026-09-03, 17 m outside a 2.4 m array). The decision
 * (2026-09-06) is to smooth the solved POSITION instead, with an
 * alpha-beta-gamma filter whose only job is to damp the jumps that bad
 * anchor coordinates or a poorly synchronised CCP put into consecutive fixes:
 * docs/superpowers/specs/2026-09-06-abg-position-filter-design.md and its
 * plan. That filter (pos_abg, src/pos_abg.h) is now wired in below -- see
 * solve_one()'s seed/step/get calls.
 *
 * What survived the removal, and why: the per-tag dt reference
 * (`last_ref_t_dtu`) and the out-of-order discard below, because publishing
 * a group that describes an EARLIER instant than the last published fix
 * makes the trace jump backwards whether or not a filter is running, and
 * because the filter to come needs exactly that reference (the 2026-09-03
 * rewind defect is documented on TDOA_DT_REORDER_MAX_MS and must not be
 * re-learned).
 */

/* One ready group into one published fix. Returns false when no group was
 * ready, so the caller can stop early. */
static bool solve_one(const struct gw_core_ctx *ctx, uint32_t now_ms)
{
	struct tdoa_meas m[POS_MAX_ANCHORS];
	struct pos_result res;
	struct pos_fix fix;
	struct tag_memo *mm;
	uint8_t eui[UWB_FRAME_EUI_LEN];
	int64_t fix_t_dtu;
	size_t n = 0u;
	uint16_t tag_addr = 0u;

	if (!tdoa_collect_take_ready(&collect, now_ms, m, &n, &tag_addr)) {
		return false;
	}

	/* The instant of this fix: the reference anchor's ABSOLUTE 40-bit
	 * t_dtu, captured BEFORE tdoa_dtu_rebase() turns m[] into signed
	 * differences. The out-of-order check below measures against it. */
	fix_t_dtu = m[0].t_dtu;

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

	mm = memo_claim(tag_addr, now_ms);

	/* dt classification. dt_ok means "predict through it"; anything else
	 * that is not dropped outright reseeds. All from the DTU clock -- never
	 * now_ms, which is quantized to the superframe and would fabricate
	 * velocity. */
	bool  dt_ok = false;
	float dt_s  = 0.0f;

	if (mm->has_ref_t) {
		int64_t raw_dt = sdelta40((uint64_t)fix_t_dtu,
					 (uint64_t)mm->last_ref_t_dtu);

		dt_s = (float)raw_dt * TDOA_GW_S_PER_DTU;

		/* A small NEGATIVE dt is a group released out of order -- it
		 * describes an instant EARLIER than this tag's last fix.
		 * Discard it whole: do not publish it (the trace would
		 * step backwards in time), do not step the filter, and do NOT
		 * advance last_ref_t_dtu. Letting it advance rewinds the
		 * reference, so the next group's dt spans time already
		 * integrated -- measured 2026-09-03 as three groups in one
		 * cycle predicting 1.2 s for 200 ms of real elapsed time.
		 *
		 * A dt under POS_ABG_DT_MIN_S is the same case from the other
		 * side: no two distinct blinks of one tag are that close, so it
		 * is a duplicate group of one blink, and beta/dt would blow up
		 * on it. Same treatment, same counter.
		 *
		 * A LARGE negative dt is a forward gap that aliased through
		 * sdelta40()'s sign boundary: the group is genuinely new, the
		 * reference must advance, and the filter reseeds rather than
		 * predicting. TDOA_DT_REORDER_MAX_MS carries the derivation of
		 * where the two populations separate. */
		if ((raw_dt <= 0 &&
		     -dt_s <= ((float)TDOA_DT_REORDER_MAX_MS / 1000.0f)) ||
		    (raw_dt > 0 && dt_s < POS_ABG_DT_MIN_S)) {
			n_reorder++;
			if (!warned_reorder) {
				warned_reorder = true;
				LOG_WRN("blink from 0x%04X released out of "
					"order (dt %d ms): group discarded, "
					"time reference held. Warned ONCE; "
					"`blink stats` carries the count",
					tag_addr, (int)(dt_s * 1000.0f));
			}
			return true;
		}
		dt_ok = (raw_dt > 0) &&
			(dt_s <= ((float)TDOA_DT_MAX_MS / 1000.0f));
	}
	mm->last_ref_t_dtu = fix_t_dtu;
	mm->has_ref_t      = true;

	/* NOTE: if resolve_one() fails below, the clock reference above has
	 * already advanced but mm->abg (the filter) has not -- the next
	 * successful cycle's dt will therefore span MORE real time than the
	 * filter actually predicts through, an under-propagation. This is the
	 * documented tradeoff in spec §5's edge table ("filter untouched,
	 * clock reference advanced"); it can produce a spurious gate_rejected
	 * or reseed shortly after a solve_fail/jump on a fast-moving tag.
	 * Not a bug -- there is no filter state to correctly advance to
	 * without a solved position to predict against -- but worth knowing
	 * before chasing an unexplained gate_rejected as its own defect. */
	if (!resolve_one(mm, m, n, tag_addr, now_ms, &res)) {
		/* Already counted and warned (solve_fail or jump). Nothing is
		 * published for this group and the filter is not stepped: a
		 * republished previous fix would be the stale-republish bug of
		 * 2026-09-02 all over again. */
		return true;
	}

	/* The filter (spec §4.2). Every path that publishes goes through
	 * pos_abg_get() below; every path that does not returns here. */
	if (!pos_abg_get(&mm->abg, NULL, NULL, NULL, NULL)) {
		pos_abg_seed(&mm->abg, res.x, res.y);
		n_abg_seeded++;
	} else if (!dt_ok) {
		pos_abg_seed(&mm->abg, res.x, res.y);
		n_abg_dt_reseed++;
		if (!warned_dt_reseed) {
			warned_dt_reseed = true;
			LOG_WRN("blink from 0x%04X: dt %d ms outside "
				"[%d..%d] ms, filter reseeded on the fresh "
				"solve. Warned ONCE; `blink stats` carries "
				"the count. A high count means slow-tier "
				"tags or marginal coverage, not a bug",
				tag_addr, (int)(dt_s * 1000.0f),
				(int)(POS_ABG_DT_MIN_S * 1000.0f),
				(int)TDOA_DT_MAX_MS);
		}
	} else {
		bool still = !mm->tag_moving;

		switch (pos_abg_step(&mm->abg, &abg_cfg, dt_s, res.x, res.y,
				     still)) {
		case POS_ABG_ACCEPTED:
			n_abg_filtered++;
			if (still) {
				n_abg_still++;
			}
			break;
		case POS_ABG_RESEEDED:
			n_abg_reseed++;
			break;
		case POS_ABG_REJECTED:
			n_abg_gate_rejected++;
			if (!warned_gate) {
				warned_gate = true;
				LOG_WRN("blink from 0x%04X: solve (%.2f, "
					"%.2f) is more than %.1f m from the "
					"filter's prediction - not published. "
					"Warned ONCE; `blink stats` carries "
					"the count", tag_addr, (double)res.x,
					(double)res.y,
					(double)abg_cfg.gate_m);
			}
			return true;
		case POS_ABG_BAD_INPUT:
		default:
			/* dt_ok guarantees dt > 0 and the solve guarantees a
			 * finite (x, y), so this is unreachable; counted as a
			 * solve failure rather than silently published. */
			n_solve_fail++;
			return true;
		}
	}

	if (!pos_abg_get(&mm->abg, &fix.x, &fix.y, NULL, NULL)) {
		/* Defensive only: every branch above leaves the filter seeded. */
		return true;
	}

	fix.src_addr   = tag_addr;
	fix.n_anchors  = (uint8_t)n;
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
			LOG_WRN("TDoA fixes are being solved over %u "
				"anchors: `residual` on the console "
				"line is ZERO BY CONSTRUCTION at this "
				"anchor count and is NOT a quality "
				"signal - a fourth surveyed anchor is "
				"what makes it one",
				(unsigned int)res.n_used);
		}
	} else {
		fix.residual_m = res.residual_m;
	}

	/* The seed/jump-gate memo follows the position actually PUBLISHED --
	 * the filter's, not the raw solve's. */
	mm->x       = fix.x;
	mm->y       = fix.y;
	mm->pos_ms  = now_ms;
	mm->has_pos = true;

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
	const struct apos_survey *s = apos_store_get();

	/* The collector's early-release threshold is a DEPLOYMENT fact (how
	 * many anchors the survey actually placed), not the compile-time
	 * POS_MAX_ANCHORS ceiling -- see tdoa_collect_set_expected()'s header.
	 * Re-set every step rather than once at init: it is one field write
	 * behind an already-read pointer, and it means a re-survey applied
	 * while the gateway is running (anchor count changed, no reboot) takes
	 * effect on the very next step instead of needing one. Skipped while
	 * unsurveyed, matching ingest_one(), which already drops every
	 * observation in that state via anchor_xyz() -- there is no anchor
	 * count to set expectations from yet. */
	if (s != NULL && s->valid) {
		tdoa_collect_set_expected(&collect, s->n_nodes);
	}

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

uint32_t tdoa_gw_reorder_count(void)
{
	return n_reorder;
}

void tdoa_gw_abg_stats(uint32_t *n_seeded, uint32_t *n_dt_reseed,
		       uint32_t *n_filtered, uint32_t *n_gate_rejected,
		       uint32_t *n_reseed, uint32_t *n_still)
{
	if (n_seeded != NULL)        { *n_seeded = n_abg_seeded; }
	if (n_dt_reseed != NULL)     { *n_dt_reseed = n_abg_dt_reseed; }
	if (n_filtered != NULL)      { *n_filtered = n_abg_filtered; }
	if (n_gate_rejected != NULL) { *n_gate_rejected = n_abg_gate_rejected; }
	if (n_reseed != NULL)        { *n_reseed = n_abg_reseed; }
	if (n_still != NULL)         { *n_still = n_abg_still; }
}

const struct pos_abg_cfg *tdoa_gw_abg_cfg(void)
{
	return &abg_cfg;
}

float tdoa_gw_abg_lambda(void)
{
	return abg_lambda;
}

/* The shell (preemptible) writes; the K_PRIO_COOP(0) loop reads the fields
 * one by one and CAN preempt the shell between two stores, so a three-field
 * lambda change is fenced -- same fence, same reason as
 * ccp_slave_residual_reset(). Single-field setters get the same fence for
 * uniformity; it costs nothing. */
void tdoa_gw_abg_set_lambda(float lambda)
{
	struct pos_abg_cfg tmp = abg_cfg;

	pos_abg_cfg_from_index(&tmp, lambda);
	k_sched_lock();
	abg_cfg.alpha = tmp.alpha;
	abg_cfg.beta  = tmp.beta;
	abg_cfg.gamma = tmp.gamma;
	abg_lambda    = lambda;
	k_sched_unlock();
}

void tdoa_gw_abg_set_gamma(float gamma)
{
	k_sched_lock();
	abg_cfg.gamma = gamma;
	k_sched_unlock();
}

void tdoa_gw_abg_set_gate(float gate_m)
{
	k_sched_lock();
	abg_cfg.gate_m = gate_m;
	k_sched_unlock();
}

void tdoa_gw_abg_set_alpha_still(float alpha_still)
{
	k_sched_lock();
	abg_cfg.alpha_still = alpha_still;
	k_sched_unlock();
}

void tdoa_gw_abg_set_reset_after(uint8_t n)
{
	k_sched_lock();
	abg_cfg.reset_after = n;
	k_sched_unlock();
}
