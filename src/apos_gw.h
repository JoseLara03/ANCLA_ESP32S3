/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Gateway orchestration for the anchor survey: enumerate, command every pair to
 * range, solve the geometry, push the answer back.
 *
 * A STEP MACHINE, not a thread and not a blocking routine. The gateway loop runs
 * at K_PRIO_COOP(0) and the beacon is the whole network's time base, so nothing
 * on that path may hold it for longer than BEACON_ARM_MARGIN_UUS. apos_gw_step()
 * emits at most ONE frame and returns; a survey therefore unfolds over many
 * superframes and the beacon cadence is never disturbed. Survey timeouts are
 * measured with k_uptime_get() deltas observed across steps, so a step that gets
 * skipped costs latency and never correctness.
 *
 * The shell never transmits. `apos run` sets state and returns; this module does
 * the work from the gateway loop and logs the result as JSON when it completes.
 * Two threads on the DW3220's SPI bus at once would corrupt both.
 */

#ifndef APOS_GW_H
#define APOS_GW_H

#include "apos_geom.h"
#include "apos_node.h"  /* APOS_ENUM_WINDOW_MS -- the settle interval below is
			 * derived from it rather than repeated as a literal */
#include "apos_table.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum apos_gw_phase {
	APOS_GW_IDLE  = 0,
	APOS_GW_ENUM  = 1,
	APOS_GW_RANGE = 2,
	APOS_GW_APPLY = 3,
};

/* How long a survey window anchors are told to hold open.
 *
 * At 32 anchors this can no longer cover a whole run: even with candidate-pair
 * filtering the ranging phase is minutes, and its true worst case -- every pair
 * timing out at APOS_GW_RANGE_TIMEOUT_MS twice -- is over an hour, which no
 * fixed window could cover. A window is therefore no longer expected to outlive
 * the run; APOS_GW_WINDOW_REFRESH_MS below is what keeps it alive.
 *
 * 120 s is kept as the per-broadcast lifetime because it is the SAFETY bound:
 * it is how long an anchor stays willing to initiate polls after the gateway
 * goes quiet. Lengthening it to span a whole run would weaken exactly the
 * property the window exists for. */
#define APOS_GW_WINDOW_S 120u

/* SURVEY_BEGIN is broadcast this many times during ENUM and the replies are
 * unioned (apos_table_add_peer() is idempotent on EUI and unions heard_ids).
 *
 * Repeating the broadcast only helps because apos_node.c salts its stagger hash
 * with the round counter, so each round is an INDEPENDENT slot draw: two
 * anchors that collide in one round are unlikely to collide in the next, and
 * the union still enumerates both. With an unsalted hash the same pair would
 * collide in every round and the extra broadcasts would buy nothing at all --
 * see enum_slot() in apos_node.c.
 *
 * EIGHT rounds, not the three that served four anchors. Two independent reasons,
 * and the second is the binding one:
 *
 *   - Enumeration coverage. With APOS_ENUM_SLOTS (64) and 32 anchors a given
 *     anchor collides with probability 1 - (63/64)^31 = 38.6 % per round, so
 *     0.386^8 = 4.9e-4 -- about 0.016 anchors expected missing from a 32-anchor
 *     array. Three rounds would leave 0.386^3 = 5.8 %, i.e. ~1.8 anchors
 *     missing on a typical run. See apos_node.h for the full derivation.
 *   - Adjacency coverage, which needs MORE rounds than enumeration does. A
 *     board sleeps through its own stagger slot with the receiver off, so it can
 *     only hear peers whose slot fell later, and its reply carries what it heard
 *     in EARLIER rounds -- round 0 reports nothing and the final round's
 *     observations are never reported at all. Eight rounds therefore yield seven
 *     rounds of usable evidence for apos_table_is_candidate(): per pair,
 *     0.386^7 = 1.3e-3 of never being observed, so under one pair of a full
 *     496-pair mesh (far fewer on a real sparse site) is missed. A missed pair
 *     costs one edge, is counted in the apos_edges log line and is caught by
 *     apos_geom_rigidity() downstream -- it is not silent.
 *
 * Eight rounds at APOS_GW_ENUM_SETTLE_MS is ~6.6 s of enumeration, once per
 * run, against a ranging phase measured in minutes. */
#define APOS_GW_ENUM_ROUNDS  8u

/* Long enough for a whole enumeration reply window to drain before the next
 * frame competes with it on the air: APOS_ENUM_WINDOW_MS (768 ms) plus the last
 * slot's reply airtime and TX path.
 *
 * DERIVED, not a literal. The value it has to clear is
 * APOS_ENUM_SLOTS * APOS_ENUM_SLOT_MS in apos_node.h, and an earlier version of
 * this header carried the product as a hand-copied number in two places -- both
 * of which silently became wrong the moment the slot count moved.
 *
 * One constant serves all three call sites, because all three are the same
 * question: the gap between ENUM rounds, the settle after APPLY's one-shot
 * re-broadcast, and the settle after a mid-RANGE window refresh. */
#define APOS_GW_ENUM_SETTLE_MS (APOS_ENUM_WINDOW_MS + 60u)

/* How often the gateway re-broadcasts SURVEY_BEGIN during the RANGE phase to
 * keep every anchor's survey window alive.
 *
 * NOT optional at 32 anchors, and the failure it prevents is silent. A node's
 * window is refreshed only by an in-session frame ADDRESSED TO IT
 * (window_refresh() in apos_node.c), and the only such frame during ranging is
 * a RANGE_CMD naming it as the INITIATOR -- all of which sit in one contiguous
 * run of the pair walk. After its own run finishes, a node is never addressed
 * again until APPLY, yet it must keep ANSWERING the SS-TWR polls that later
 * initiators aim at it, and on a cold deployment
 * anchor_respond_wave_poll() will only do that while the window is open. With
 * 12 ordered pairs the whole run fitted inside one window and this never
 * mattered; with hundreds it does, and the symptom would be every late pair
 * coming back as a hole with the radio looking dead.
 *
 * 30 s, comfortably inside APOS_NODE_REFRESH_S (60 s) and APOS_GW_WINDOW_S
 * (120 s), so a single dropped or suppressed broadcast cannot lapse a window.
 * Each refresh costs one frame plus APOS_GW_ENUM_SETTLE_MS of paused ranging --
 * ~2.7 % overhead -- because the broadcast provokes a full staggered ENUM_RSP
 * storm, which on_enum_rsp()'s `phase != APOS_GW_ENUM` guard discards but which
 * must still be off the air before the next RANGE_CMD. */
#define APOS_GW_WINDOW_REFRESH_MS 30000u

/* Worst-case cost of one apos_gw_step() transmission. The gateway loop refuses
 * to enter a step unless BEACON_ARM_MARGIN_UUS + this much time remains before
 * the beacon, so this number must cover the step's WORST case, not its typical
 * one: under-reserving lets a step overrun into the beacon's arming window, and
 * a delayed beacon TX that misses its slot costs every node in the network its
 * time base.
 *
 * Derivation, in UUS (1 uus = 512/499.2 MHz ~= 1.0256 us, so uus = us/1.0256):
 *   - the bounded TXFRS wait, apos_gw.c's TX_COMPLETE_TIMEOUT_MS = 8 ms
 *     -> 8000 / 1.0256 ~= 7800 uus. This dominates, and it is a real bound
 *        rather than a typical: a TX that never completes burns all of it.
 *   - airtime of the largest APOS frame this module transmits. That is
 *     APOS_LEN_SETPOS (25 bytes) + FCS_LEN once Task 12 lands, not today's
 *     15-byte SURVEY_BEGIN -- sized for the largest so the constant does not
 *     need revisiting. At PLEN_1024 the preamble alone is ~1.05 ms and 27
 *     bytes at 850 kbps with 4z overhead is ~0.4 ms: ~1.45 ms -> ~1420 uus,
 *     rounded to 1500. Deliberately NOT APOS_LEN_MAX: that is ENUM_RSP (38
 *     bytes since the neighbour bitmap was added), which only ANCHORS
 *     transmit -- this budget covers what the GATEWAY puts on the air.
 *   - 700 uus of margin for the SPI register writes around the TX and for
 *     rounding.
 * 7800 + 1500 + 700 = 10000 uus (~10.3 ms), against a 200 ms superframe.
 *
 * An earlier value of 3000 claimed to cover "one frame plus the bounded TXFRS
 * wait" while reserving less than half of the TXFRS bound alone. Skipping more
 * steps is pure latency by design (every survey deadline is absolute
 * wall-clock); a missed beacon is a real fault. Re-derive if
 * TX_COMPLETE_TIMEOUT_MS or the largest APOS frame changes. */
#define APOS_GW_STEP_BUDGET_UUS 10000u

/* Reserved separately for the ONE step that does not transmit: the solve.
 *
 * apos_geom_solve() is a Levenberg-Marquardt fit, up to APOS_LM_MAX_ITER (200)
 * iterations, each rebuilding an n x n normal-equation matrix over every usable
 * edge -- n = 3N-6 free parameters -- and running a dense Gaussian elimination
 * with partial pivoting on it. It is bounded, but bounded in ITERATIONS, not in
 * microseconds, and its bound is nowhere near APOS_GW_STEP_BUDGET_UUS: on a
 * four-anchor array (n = 6, an 18x18 matrix in the old 8-node sizing) a rough
 * count is ~15k float operations plus ~5 kB of matrix traffic per iteration,
 * which at 240 MHz through the instruction cache is tens of microseconds per
 * iteration at best and 200 of them is comfortably past BEACON_ARM_MARGIN_UUS
 * (5000 uus). Running it in a step sized for one transmission would mean a
 * beacon armed late -- and the beacon is the whole network's time base.
 *
 * AT 32 NODES THAT ARITHMETIC NO LONGER FITS IN THIS BUDGET, AND THE BUDGET WAS
 * NEVER A HARD BOUND ANYWAY. The elimination is O(n^3), so n = 90 (a full
 * 32-anchor 3D survey) is ~3400x the 6-parameter case per iteration, and the
 * Jacobian assembly grows with the edge count on top of that. If a solve at
 * n = 6 really is "tens of microseconds" per iteration, then at n = 90 a single
 * iteration is already into the tens of milliseconds and 200 of them is
 * seconds -- far past the ~195000 uus of clear air deferring to the top of a
 * superframe can buy. Nothing about that is caught by this gate: the gate only
 * decides WHEN the solve starts, never how long it runs, so a 32-node solve
 * that overruns arms the beacon late no matter what this number says.
 *
 * Left as-is rather than raised, because raising it cannot help -- there is no
 * value that fits a multi-second computation into a 200 ms superframe. The real
 * fix is the one already named below: get the solve off this thread. Treat that
 * as the FIRST thing to do if a bench run at more than a handful of anchors
 * shows a late beacon, and do not read the numbers below as covering it.
 *
 * It cannot be split across steps (apos_geom_refine() is one call and keeps its
 * working matrices in function-local static storage), so instead the solve step
 * simply refuses to start unless this much time remains before the beacon must
 * be armed.
 *
 * 150000 uus (~154 ms) is chosen so that the ONLY moment in a superframe that
 * can satisfy it is the first step after a beacon. avail_uus peaks at
 * T_SUPERFRAME_UUS - BEACON_ARM_MARGIN_UUS ~= 195000 and falls monotonically
 * through the superframe, so a threshold this high cannot be met mid-superframe
 * -- and mid-superframe is reachable, because a step runs whenever the RX
 * servicing above it finishes early, not only at the top of the loop. A smaller
 * threshold would therefore guarantee only ITSELF as headroom, not the ~195000
 * uus this reasoning wants; sizing it near the superframe is what makes
 * "the solve gets a whole superframe to run in" true of the code rather than of
 * the typical case. The cost is nil: the solve happens once per run, and the
 * loop offers a qualifying step every 200 ms.
 *
 * THIS IS AN ESTIMATE, not a measurement -- neither this value nor the solve it
 * guards has ever been timed on hardware. The solve now REPORTS its own wall
 * clock: `solve_ms` in the `{"apos_solve":...}` line is the measurement this
 * comment could not make, so a bench run that shows a late beacon at the instant
 * that line is logged has the duration that caused it on the same line. Do NOT
 * lower APOS_LM_MAX_ITER, which would change what is reported.
 * The only genuinely HARD bound is to stop running the solve on this thread at
 * all -- hand it to a preemptible worker that touches no SPI and let the
 * gateway loop poll for its completion. That is the real fix if this ever hurts;
 * it is deliberately out of scope here, because deferring to the top of a
 * superframe removes the risk for every plausible solve duration. */
#define APOS_GW_SOLVE_BUDGET_UUS 150000u

/* Largest peer count the solve above is believed to fit inside
 * APOS_GW_SOLVE_BUDGET_UUS. Past it, do_solve() logs a warning naming the size
 * BEFORE it starts solving, and the `apos_solve` line reports the measured
 * `solve_ms` after. Together those are the only runtime signal that exists for
 * the blocking risk the comment above describes.
 *
 * A ROUND NUMBER, and honest about what it is: NO mesh of ANY size has been
 * timed on hardware yet, so this is not a measurement either. Eight is chosen
 * because it is the largest array this feature's hardware step covers (an
 * 8-anchor survey checked against a tape measure), and because the same O(n^3)
 * scaling used above puts an 8-node solve two orders of magnitude inside the
 * budget: n = 3*8-6 = 18 free parameters against n = 90 at 32 nodes is 1/125 of
 * the per-iteration cost, so even APOS_LM_MAX_ITER iterations of it land in the
 * tens of milliseconds against ~195 ms of clear air.
 *
 * Move it when someone MEASURES a larger array -- a `solve_ms` comfortably
 * inside the budget from a real mesh of that size -- not when a solve merely
 * looks fine. */
#define APOS_GW_SOLVE_TIMED_NODES 8u

/* Exchanges per commanded pair. The gateway owns this tradeoff, which is why it
 * is a RANGE_CMD field and not a constant on the anchor. 40 at ~5 ms is the
 * ~200 ms batch apos_node's beacon-staleness budget is sized around, and 40
 * samples make the reported sd a usable quality signal rather than noise. */
#define APOS_GW_N_EXCHANGES 40u

/* How long to wait for a RANGE_RSP. The batch itself is ~200 ms plus the
 * anchor's own beacon-guard suppressions, so this is generous by design: a
 * spurious timeout costs a retry and a wrong measurement costs the geometry. */
#define APOS_GW_RANGE_TIMEOUT_MS 3000u

/* One retry per ordered pair. A pair that fails twice is reported as a hole
 * rather than retried indefinitely -- the fit works around holes, and an
 * operator needs the run to finish so they can see WHICH pair failed. */
#define APOS_GW_RANGE_RETRIES 1u

/* Consecutive failures to get a RANGE_CMD off the air before the pair is given
 * up on. A TX that never starts is not the pair's fault, so it does not consume
 * the retry budget above -- but it must not be retried forever either: the
 * radio can wedge, and a step that re-attempts an impossible transmission every
 * superframe with nothing advancing pair_idx is a survey that never finishes
 * and never reports why. */
#define APOS_GW_TX_FAIL_LIMIT 10u

/* A SETPOS_ACK comes straight back -- no ranging batch in between -- so this is
 * tight compared with APOS_GW_RANGE_TIMEOUT_MS. */
#define APOS_GW_APPLY_TIMEOUT_MS 1500u

/* Three attempts per anchor. Unlike a failed range, a failed SETPOS cannot be
 * shrugged off as a hole: an anchor left on its old coordinates while its peers
 * move to new ones is a silently inconsistent deployment, so this retries hard
 * and then reports the anchor by address. */
#define APOS_GW_APPLY_RETRIES 3u

/* Acceptance thresholds. Anchored on the antenna-delay cross-check, which
 * accepts |error| < 30 mm per pair: a fit over many such edges should land
 * inside 50 mm RMS, and anything much worse means a bad edge or a wrong gauge
 * rather than accumulated noise.
 *
 * These are the numbers most likely to need adjusting after the first real
 * bench run. They are thresholds on a REPORTED result, so raising one never
 * changes what was measured -- only whether `apos apply` will proceed.
 *
 * THEY ARE ALSO ONLY MEANINGFUL WHERE THE FRAMEWORK IS RIGID AND HAS SPARE
 * EDGES. See apos_gw_result_unverified() below and the long note in
 * apos_geom.h: with n_edges <= 3N-6 the fit reproduces any input exactly and
 * rms_m comes back zero however bad the ranges were, and a flexible mesh can
 * report a small rms_m for a shape it never determined. A four-anchor array
 * solved in 3D is always that case; a sparse 32-anchor mesh usually is not,
 * which is what makes these thresholds a real acceptance test at scale. */
#define APOS_ACCEPT_RMS_MM       50u
#define APOS_ACCEPT_WORST_FACTOR 3.0f
/* Below this, the array is too close to coplanar for the solved z values to
 * mean anything. Does NOT block acceptance on its own -- x and y are still
 * good -- but it is reported, and the operator is told z is not survey-quality. */
#define APOS_ACCEPT_PLANARITY_MM 100u
/* Below this ratio, a 2D gauge's origin/xaxis/plane triangle is too close to
 * a straight line for the solved +y direction to mean anything -- the 2D
 * analogue of APOS_ACCEPT_PLANARITY_MM above, and does NOT block acceptance
 * on its own either. See apos_geom.h's gauge_collinearity_ratio note: a
 * `plane` anchor only 50 mm off a 1 m origin-xaxis baseline (ratio 0.05) was
 * enough for an injected 30 mm range error elsewhere in the mesh to move the
 * solved y by 5x, with rms_mm staying at 0. */
#define APOS_ACCEPT_GAUGE_COLLINEARITY 0.10f

struct apos_gw_status {
	uint8_t  phase;        /* enum apos_gw_phase */
	uint16_t session;
	uint8_t  n_peers;
	uint16_t meas_done;    /* ordered pairs attempted so far */
	uint16_t meas_total;   /* ordered pairs in this run */
	uint8_t  applied_ok;
	uint8_t  applied_fail;
	bool     have_result;
};

void apos_gw_init(void);

/* Consume one received frame. Ignores anything that is not an APOS frame from a
 * known peer for the current session. */
void apos_gw_on_rx(const uint8_t *buf, uint16_t plen);

/* Advance the survey by at most one transmission. avail_uus is how much time is
 * left before the beacon must be armed; the step does nothing if that is less
 * than APOS_GW_STEP_BUDGET_UUS. seq is the gateway's shared frame sequence
 * counter, so survey frames stay in the same numbering as beacons and grants --
 * which is what makes a sniffer capture readable. */
void apos_gw_step(uint32_t avail_uus, uint8_t *seq);

bool apos_gw_busy(void);
void apos_gw_get_status(struct apos_gw_status *out);

/* The live table, for `apos enum` to print. Never NULL. */
const struct apos_table *apos_gw_table(void);

/* Begin an enumeration-only pass. Returns 0, or -EBUSY if a survey is running. */
int apos_gw_start_enum(void);

/* Record the gauge as SHORT ADDRESSES, not node indices: indices are an artefact
 * of the order anchors happened to answer enumeration in, and would silently
 * point at different boards after a re-enumeration. Addresses are resolved to
 * indices at solve time.
 *
 * up == -1 selects 2D mode (no up designation, no reflection to resolve).
 * -1 is a dedicated sentinel, not 0: 0x0000 is UWB_ADDR_GATEWAY_RESERVED and
 * must stay rejected as an address on its own terms, distinct from "not
 * given".
 *
 * Returns 0, -EINVAL if origin/xaxis/plane are not distinct (or up, when
 * given, is not additionally distinct from them), or -EBUSY while a survey
 * runs. Does NOT require the addresses to be enumerated yet -- an operator
 * may legitimately set the gauge from a site sketch before powering the
 * array. */
int apos_gw_set_gauge(uint16_t origin, uint16_t xaxis, uint16_t plane,
		      int32_t up);

bool apos_gw_gauge_set(void);

/* The dimensionality implied by the currently-stored gauge. Only ever
 * meaningful once apos_gw_gauge_set() is true -- see apos_gw.c's
 * step_enum() for the one call site, which is only reached after that
 * precondition already holds. */
enum apos_geom_dim apos_gw_gauge_dim(void);

/* Begin a full run: re-enumerate, range every ordered pair, solve, report.
 * Persists NOTHING. Returns 0, -EBUSY if a survey runs, or -EINVAL if the gauge
 * has not been set. */
int apos_gw_start_run(void);

/* The last solved result. Never NULL; check apos_gw_get_status()->have_result. */
const struct apos_result *apos_gw_result(void);

/* Whether the last result met every threshold in this header. */
bool apos_gw_accepted(void);

/* True when the last result's rms_m / worst_edge_m CANNOT be read as a quality
 * check on the ranging.
 *
 * Exactly the negation of apos_geom_rigidity()'s `redundant` over the edge list
 * the solve ran on, so it is true for any of THREE distinct reasons. The
 * `apos_rigidity` JSON line (and `apos show`) carries all three, and the
 * warning that follows a run names whichever one fired:
 *
 *   - the framework is DISCONNECTED (rigidity.n_components > 1) -- two groups of
 *     anchors that cannot hear each other have no measured relationship, so
 *     their relative placement is invented whatever rms_m says;
 *   - some node has FEWER THAN d EDGES (rigidity.degree_ok false) -- it is not
 *     pinned at all, and the fit will still place it somewhere;
 *   - the mesh has NO SPARE EDGE (rigidity.spare_edges <= 0) -- with usable
 *     edges <= 2N-3 in 2D or 3N-6 in 3D, LM re-embeds whatever distances it was
 *     handed exactly and both numbers come back at (or near) zero regardless of
 *     how bad the ranging was.
 *
 * When it is true, apos_gw_accepted() means only "nothing contradicted the
 * ranges", never "the ranges are good".
 *
 * THE SIZE OF THE DEPLOYMENT DECIDES WHICH REGIME YOU ARE IN. A four-anchor
 * array solved in 3D is a full mesh of exactly 6 edges against exactly 6 free
 * parameters -- isostatic, no spare equation, so this flag is true on every such
 * run and no threshold can change that. The same four anchors solved in 2D have
 * one spare edge (2*4-3 = 5 against 6), and a sparse-but-redundant 32-anchor
 * mesh normally has dozens, so at scale this flag is FALSE on a healthy run and
 * rms_m becomes a real acceptance signal for the first time. That is the whole
 * point of scaling the survey; it is no longer "not a corner case, it is every
 * case".
 *
 * What it is NOT is a rigidity proof. apos_geom_rigidity()'s three conditions
 * are necessary, not sufficient -- a hinge between two densely-meshed clusters
 * passes all of them -- so a false here means "nothing detectable contradicts
 * the geometry either", which is still weaker than a measurement. Read
 * apos_gw_result_quality() (reciprocal disagreement and per-pair sd) for the
 * RANGING, and confirm the solved node-to-node distances against a tape measure
 * before committing a survey the site depends on. That advice does not expire
 * with the anchor count.
 *
 * See the long notes on apos_geom_rigidity() and apos_geom_refine(). */
bool apos_gw_result_unverified(void);

/* The rigidity verdict behind the last solve, over the symmetrised edge list
 * and all enumerated peers. Never NULL; meaningful only when have_result.
 *
 * Computed over ALL enumerated nodes, not only the placed ones, so it answers
 * "is the framework this array can measure rigid" rather than "did this
 * particular fit have spare equations". The two coincide exactly when the result
 * is accepted, since apos_gw_accepted() already requires every node placed and
 * none ambiguous; where they differ -- some node unplaced -- the whole-mesh
 * answer is the conservative one, because an unplaced node either leaves the
 * graph disconnected or leaves it short of degree d. */
const struct apos_rigidity *apos_gw_result_rigidity(void);

/* Ranging-quality maxima behind the last solve, straight from the directed
 * measurements rather than from the fit: the largest |d(A->B) - d(B->A)| over
 * pairs measured both ways (-1 if there is no such pair), and the largest
 * per-measurement sd. Either pointer may be NULL. Meaningful only when
 * have_result. See apos_table_quality(). */
void apos_gw_result_quality(int32_t *max_recip_mm, uint16_t *max_sd_mm);

/* Spare edges in the last solve: usable_edges - (2N-3 in 2D, 3N-6 in 3D).
 * Shorthand for apos_gw_result_rigidity()->spare_edges. <= 0 is one of the three
 * unverified reasons above -- but not the only one, so do not read a positive
 * value here as "verified" on its own. Meaningful only when have_result. */
int apos_gw_result_redundancy(void);

/* Push the last solved result to every anchor, persist it locally, and close the
 * survey window.
 *
 * Refuses a result that failed acceptance unless force is true. Returns 0,
 * -EBUSY if a survey runs, -ENODATA if there is no result to apply, or
 * -EPERM if the result failed acceptance and force was not given.
 *
 * `force` overrides ACCEPTANCE only. It does not, and must not, suppress the
 * unverified-mesh warning: that condition is a property of the array's edge
 * count, not an operator decision, and it is loudest exactly where the result
 * looks best. See apos_gw_result_unverified(). */
int apos_gw_start_apply(bool force);

/* Shift z on every subsequent solve, moving z = 0 off the plane through the
 * three gauge anchors and onto the floor. Applied at solve time, so changing it
 * requires a re-run rather than silently rewriting a reported result. */
void apos_gw_set_zoff(float dz);

/* Parse and act on a survey trigger document.
 *
 * UNWIRED: net_uplink.c does not subscribe to anything, so nothing calls this
 * yet. It exists complete and tested-by-inspection so that wiring a subscribe
 * path later is a transport change only, with no protocol decisions left open.
 *
 * Accepts a minimal document, matched by substring rather than parsed with a
 * JSON library -- this project links no JSON parser and one command word does
 * not justify adding one:
 *   {"cmd":"run"}     -> apos_gw_start_run()
 *   {"cmd":"apply"}   -> apos_gw_start_apply(false)
 *
 * Deliberately NOT accepted: "apply force" and any form of setting the gauge.
 * The gauge is a physical claim about which box is where and a forced apply
 * overrides a failed acceptance check -- neither should be assertable by a
 * broker message. Both stay console-only.
 *
 * Returns 0, -EINVAL on an unrecognised document, or whatever the underlying
 * start function returned. */
int apos_gw_trigger_from_mqtt(const char *payload, size_t len);

#endif /* APOS_GW_H */
