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
 * (APOS_ENUM_SLOTS * APOS_ENUM_SLOT_MS = 210 ms) plus reply airtime. */
#define APOS_GW_ENUM_ROUNDS  3u
#define APOS_GW_ENUM_GAP_MS  400u

/* Rough cost of one apos_gw_step() transmission, used to decide whether there
 * is room before the next beacon. One frame plus the bounded TXFRS wait. */
#define APOS_GW_STEP_BUDGET_UUS 3000u

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

#endif /* APOS_GW_H */
