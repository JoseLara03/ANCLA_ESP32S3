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

/* How long a survey window anchors are told to hold open. Generously longer
 * than the ranging phase for the largest supported deployment: 56 ordered pairs
 * at ~300 ms each is under 20 s, and every APOS frame refreshes the window
 * anyway (APOS_NODE_REFRESH_S). */
#define APOS_GW_WINDOW_S 120u

/* SURVEY_BEGIN is broadcast this many times, spaced this far apart, and the
 * replies are unioned. One round would be enough if no two EUIs hashed to the
 * same stagger slot; three makes a slot collision cost a retry instead of a
 * missing anchor. The gap must exceed the worst-case stagger
 * (APOS_ENUM_SLOTS * APOS_ENUM_SLOT_MS = 8 * 30 = 240 ms) plus reply airtime;
 * 400 ms clears 240 ms with ~160 ms to spare, which is ample for a ~1.5 ms
 * ENUM_RSP. */
#define APOS_GW_ENUM_ROUNDS  3u
#define APOS_GW_ENUM_GAP_MS  400u

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
 *     rounded to 1500.
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
 * iterations, each rebuilding an 18x18 normal-equation matrix over every usable
 * edge and running a dense Gaussian elimination with partial pivoting on it. It
 * is bounded, but bounded in ITERATIONS, not in microseconds, and its bound is
 * nowhere near APOS_GW_STEP_BUDGET_UUS: a rough count is ~15k float operations
 * plus ~5 kB of matrix traffic per iteration, which at 240 MHz through the
 * instruction cache is tens of microseconds per iteration at best and 200 of
 * them is comfortably past BEACON_ARM_MARGIN_UUS (5000 uus). Running it in a
 * step sized for one transmission would mean a beacon armed late -- and the
 * beacon is the whole network's time base.
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
 * guards has ever been timed on hardware. If a bench run ever shows a late
 * beacon at the instant `{"apos_solve":...}` is logged, measure the solve
 * first. Do NOT lower APOS_LM_MAX_ITER, which would change what is reported.
 * The only genuinely HARD bound is to stop running the solve on this thread at
 * all -- hand it to a preemptible worker that touches no SPI and let the
 * gateway loop poll for its completion. That is the real fix if this ever hurts;
 * it is deliberately out of scope here, because deferring to the top of a
 * superframe removes the risk for every plausible solve duration. */
#define APOS_GW_SOLVE_BUDGET_UUS 150000u

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
 * THEY ARE ALSO ONLY MEANINGFUL WHERE THE MESH HAS SPARE EDGES. See
 * apos_gw_result_unverified() below and the long note in apos_geom.h: with
 * n_edges <= 3N-6 the fit reproduces any input exactly and rms_m comes back
 * zero however bad the ranges were, so passing these thresholds proves nothing.
 * The real deployment (four ranging slaves, full mesh) is exactly that case. */
#define APOS_ACCEPT_RMS_MM       50u
#define APOS_ACCEPT_WORST_FACTOR 3.0f
/* Below this, the array is too close to coplanar for the solved z values to
 * mean anything. Does NOT block acceptance on its own -- x and y are still
 * good -- but it is reported, and the operator is told z is not survey-quality. */
#define APOS_ACCEPT_PLANARITY_MM 100u

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
 * Returns 0, -EINVAL if the four are not distinct, or -EBUSY while a survey
 * runs. Does NOT require the addresses to be enumerated yet -- an operator may
 * legitimately set the gauge from a site sketch before powering the array. */
int apos_gw_set_gauge(uint16_t origin, uint16_t xaxis, uint16_t plane,
		      uint16_t up);

bool apos_gw_gauge_set(void);

/* Begin a full run: re-enumerate, range every ordered pair, solve, report.
 * Persists NOTHING. Returns 0, -EBUSY if a survey runs, or -EINVAL if the gauge
 * has not been set. */
int apos_gw_start_run(void);

/* The last solved result. Never NULL; check apos_gw_get_status()->have_result. */
const struct apos_result *apos_gw_result(void);

/* Whether the last result met every threshold in this header. */
bool apos_gw_accepted(void);

/* True when the last result's rms_m / worst_edge_m CANNOT be read as a quality
 * check, because the mesh had no spare edges: usable edges <= 3 * n_placed - 6.
 * In that regime LM re-embeds whatever distances it was handed exactly and both
 * numbers come back at (or near) zero regardless of how bad the ranging was, so
 * apos_gw_accepted() means only "nothing contradicted the ranges", never "the
 * ranges are good".
 *
 * This is not a corner case: the deployment is four ranging slaves, and a
 * four-node full mesh has exactly 6 edges against exactly 6 free parameters.
 * Anything that reports acceptance to an operator must report this alongside
 * it. See the long note on apos_geom_refine(). */
bool apos_gw_result_unverified(void);

/* Spare edges in the last solve: usable_edges - (3 * n_placed - 6). <= 0 is the
 * unverified regime above. Meaningful only when have_result. */
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

#endif /* APOS_GW_H */
