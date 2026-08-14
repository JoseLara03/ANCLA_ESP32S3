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
 * number of seconds; any later APOS frame refreshes it; SURVEY_END closes it
 * early. Outside the window this module transmits nothing and this board will
 * not act as an initiator, which is what keeps a production anchor from
 * colliding with tag ranging traffic.
 */

#ifndef APOS_NODE_H
#define APOS_NODE_H

#include "beacon_guard.h"
#include "uwb_config.h"

#include <stdbool.h>
#include <stdint.h>

/* Enumeration reply stagger. SURVEY_BEGIN is a broadcast, so without a stagger
 * every anchor answers at once and the gateway hears a collision.
 *
 * The slot is chosen by hashing this board's EUI-64, NOT by anchor_id: assuming
 * anything about ids is the coupling this design exists to remove, and a
 * duplicate id must remain DETECTABLE rather than being silently folded into
 * one reply. EUI-64 is unique by manufacture, so the stagger is collision-free
 * with no configuration and is stable across reboots.
 *
 * 8 slots x 30 ms bounds the worst-case reply delay at 210 ms, which is about
 * one superframe -- short enough for the SLAVE loop to simply sleep through it. */
#define APOS_ENUM_SLOTS   8u
#define APOS_ENUM_SLOT_MS 30u

/* Seconds a window is extended by on each in-session APOS frame. Long enough
 * that a slow ranging phase never lets the window lapse mid-survey, short
 * enough that a gateway which stops talking releases this board promptly. */
#define APOS_NODE_REFRESH_S 60u

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

/* This board's EUI-64. Never NULL; 8 bytes. */
const uint8_t *apos_node_eui(void);

/* Offer one received frame. Returns true if it was an APOS frame handled here,
 * so the caller can skip the remaining responders.
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
