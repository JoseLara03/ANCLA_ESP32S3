/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Anchor side of the auto-positioning survey. Offered every received frame by
 * the SLAVE loop; ignores anything that is not an APOS frame.
 *
 * This board never decides its own role, never holds a deployment list, and
 * never initiates anything. It answers enumeration, runs exactly the ranging
 * batch it is told to run, and stores exactly the coordinates it is given. The
 * gateway holds all the state -- which is what makes an `anchor id` swap
 * harmless.
 *
 * The survey window is the safety boundary. SURVEY_BEGIN opens it for a bounded
 * number of seconds; a later in-session SETPOS refreshes it; SURVEY_END closes
 * it early. Outside the window this module transmits nothing and this board
 * will not act as an initiator, which is what keeps a production anchor from
 * colliding with tag ranging traffic.
 */

#ifndef APOS_NODE_H
#define APOS_NODE_H

#include "beacon_guard.h"
#include "uwb_config.h"
#include "uwb_mac.h"

#include <stdbool.h>
#include <stdint.h>

/* Enumeration reply stagger. SURVEY_BEGIN is a broadcast, so without a stagger
 * every anchor answers at once and the gateway hears a collision.
 *
 * The slot is chosen by hashing this board's EUI-64, NOT by anchor_id: assuming
 * anything about ids is the coupling this design exists to remove, and a
 * duplicate id must remain DETECTABLE rather than being silently folded into
 * one reply. EUI-64 is unique by manufacture, so the slot needs no
 * configuration.
 *
 * The hash is salted with the SURVEY_BEGIN round counter and the session
 * (apos_node.c's enum_slot()), so it is deliberately NOT stable across rounds or
 * across runs. Uniqueness of the EUI does not make the SLOT unique -- 8 slots
 * and 4 anchors collide with probability ~59 % -- and an unsalted hash would
 * repeat that same collision in every round of every run, which is a permanently
 * missing anchor rather than a retryable one.
 *
 * 8 slots x 30 ms bounds the worst-case reply delay at 210 ms, which is about
 * one superframe -- short enough for the SLAVE loop to simply sleep through it. */
#define APOS_ENUM_SLOTS   8u
#define APOS_ENUM_SLOT_MS 30u

/* Seconds a window is extended by on each in-session APOS frame. Long enough
 * that a slow ranging phase never lets the window lapse mid-survey, short
 * enough that a gateway which stops talking releases this board promptly. */
#define APOS_NODE_REFRESH_S 60u

/* Hard cap on one commanded batch, independent of what the RANGE_CMD asks for.
 * The gateway owns the tradeoff and sends 40, but this board must not be
 * talkable into an unbounded transmit run by a malformed or hostile command --
 * that is the failure mode the survey window exists to prevent, and a count
 * cap is the second half of it.
 *
 * This bound alone is NOT what keeps the batch inside beacon_guard's lock
 * budget -- APOS_RANGE_BATCH_DEADLINE_MS below is. Sized only on the SUCCESS
 * case (~5 ms/exchange, ~320 ms for 64), this constant is far too loose for
 * the FAILURE case: ss_initiator_range()'s own worst case is
 * TX_DONE_TIMEOUT_MS (10) + RX_DONE_TIMEOUT_MS (25) = 35 ms, plus the 2 ms
 * inter-exchange sleep = 37 ms/exchange. A batch against an unreachable peer
 * -- a routine event, since the survey commands every pair including
 * out-of-range ones -- would run 64 x 37 ms =~ 2.4 s if this count cap were
 * the only bound, three times beacon_guard's ~800 ms lock budget
 * (BEACON_GUARD_MAX_MISSES superframes). The wall-clock deadline is what
 * actually protects the guard; this constant only bounds the DATA a
 * successful batch collects (Task 11's gateway asks for 40, so the reported
 * sd stays a useful quality signal). */
#define APOS_MAX_EXCHANGES 64u

/* Wall-clock deadline for one RANGE_CMD batch, independent of APOS_MAX_EXCHANGES:
 * the loop breaks out once elapsed time exceeds this, whatever the exchange
 * count reached. This is what actually bounds the batch against
 * beacon_guard's lock budget in the failure case above -- the count cap alone
 * does not, since a batch of all-timeout exchanges is ~37 ms each, not ~5 ms.
 *
 * Derived from the same lock budget the guard uses to judge its own
 * prediction stale: BEACON_GUARD_MAX_MISSES * T_SUPERFRAME_UUS is 4 * 195000
 * UUS = 780000 UUS =~ 800 ms (T_SUPERFRAME_UUS is exactly 200.0 ms, see
 * uwb_mac.h). 700 ms leaves an 80 ms margin inside that 800 ms budget for the
 * RANGE_RSP build/send (and its retry, see apos_node.c) that follows the
 * batch before the guard's next real read. A deadline break is not a
 * failure: whatever exchanges completed are still good data, and n_ok already
 * reports that count honestly. */
#define APOS_RANGE_BATCH_DEADLINE_MS \
	((BEACON_GUARD_MAX_MISSES * T_SUPERFRAME_UUS) / 1000u - 80u)

/* Below this many successful exchanges the batch is reported with its real
 * n_ok and the gateway discards it (apos_table_symmetrise's min_n_ok). Reported
 * rather than suppressed: "the pair cannot range" and "the command never
 * arrived" must not collapse into the same silence at the gateway. */
#define APOS_MIN_N_OK 10u

/* Clear to the closed-window state. Call once before the SLAVE loop starts. */
void apos_node_init(void);

/* True while a gateway-opened survey window is live.
 *
 * Read by anchor_respond_wave_poll() via uwb_slave.c: during a survey every
 * anchor is still unpositioned yet must answer its peers' polls, so this is
 * what lets the poll responder refuse when unpositioned without breaking a
 * survey of a cold deployment. Also gated on by the RANGE_CMD path.
 *
 * Evaluates the deadline on each call, so an expired window closes itself with
 * no tick or timer. */
bool apos_node_window_open(void);

/* This board's EUI-64. Never NULL; 8 bytes.
 *
 * Before apos_node_init() has run (or if hwinfo failed), this is all zero --
 * indistinguishable from a genuine all-zero id. Callers that care about the
 * difference must track readiness themselves; this module does not expose
 * one. */
const uint8_t *apos_node_eui(void);

/* Offer one received frame. Returns true if it was an APOS frame handled here,
 * so the caller can skip the remaining responders.
 *
 * plen MUST exclude the 2-byte FCS -- callers pass flen - FCS_LEN, exactly as
 * every other frame consumer in this project does. The apos_frame parsers
 * require an EXACT length match per subtype, so a caller that forgets this
 * (the project's classic mistake) gets a board that receives every survey
 * frame and silently answers none -- watch for the "parse failed" LOG_WRN
 * lines in apos_node.c, which name the subtype and the plen actually seen.
 *
 * cfg must be the SLAVE loop's own mutable snapshot: a SETPOS is applied to it
 * immediately, not deferred to the next reboot. Unlike ant_delay_tx there is no
 * radio register to keep in step with it, and an operator who has just run
 * `apos apply` expects the anchor to report its new coordinates at once.
 *
 * May block for up to APOS_ENUM_SLOTS * APOS_ENUM_SLOT_MS on an ENUM_RSP and,
 * once Task 7 lands, for the length of one ranging batch on a RANGE_CMD. Both
 * are bounded and both happen only during commissioning. */
bool apos_node_on_rx(const uint8_t *buf, uint16_t plen, uwb_config_t *cfg,
		     uint8_t *seq, struct beacon_guard *bg);

#endif /* APOS_NODE_H */
