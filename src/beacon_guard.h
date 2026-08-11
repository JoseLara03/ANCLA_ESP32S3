/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Beacon-collision avoidance for the ranging responders.
 *
 * An anchor schedules its response at a fixed delay from the poll that
 * triggered it and, without this, never checks where that delay lands. Two
 * cases put a response on top of the gateway's beacon:
 *
 *   - DISCOVERY, the dominant one. It is broadcast by tags that have not
 *     joined yet, so it is not confined to a CFP slot and can arrive anywhere
 *     in the superframe -- and disc_resp_delay_uus(3) puts the response many
 *     milliseconds later with no relation to the frame structure at all.
 *   - The tail of the last CFP slot, where the superframe's ~5.5 ms of slack
 *     is shorter than the response turnaround.
 *
 * A hit corrupts a broadcast every node in the network depends on, which costs
 * far more than the single range that caused it. The gateway protects itself
 * with its beacon arm margin; this is the slaves' mirror of that.
 *
 * Pure C -- no Zephyr, no radio headers -- so it is host-testable. Times are
 * opaque 32-bit ticks; callers supply DW3000 hi32 units (256 DTU ~= 4.006 ns)
 * via UUS_TO_HI32 in uwb_dwtime.h.
 */

#ifndef BEACON_GUARD_H
#define BEACON_GUARD_H

#include <stdbool.h>
#include <stdint.h>

/* Consecutive predicted beacons the guard may roll past without seeing one
 * before it declares the prediction stale.
 *
 * 4 superframes is ~800 ms, over which two independent +/-20 ppm crystals
 * drift ~32 us relative -- still far inside a 500 us guard, so a prediction
 * this old is still trustworthy. Beyond it, suppressing against a guess would
 * block legitimate transmits at arbitrary times, which is worse than the
 * collisions suppression exists to prevent. */
#define BEACON_GUARD_MAX_MISSES 4

struct beacon_guard {
	uint32_t next_beacon; /* predicted start of the next beacon */
	uint32_t period;      /* superframe period */
	uint32_t guard;       /* pad on each side of the beacon */
	uint32_t occupancy;   /* beacon airtime */
	uint8_t  misses;      /* predicted beacons rolled past unseen */
	bool     locked;      /* false until the first beacon, and after a
			       * stale-prediction drop */
};

/* Configure and clear to the unlocked state. Suppression does nothing until
 * the first beacon_guard_beacon(). */
void beacon_guard_init(struct beacon_guard *g, uint32_t period, uint32_t guard,
		       uint32_t occupancy);

/* Record a received beacon at beacon_at, re-anchoring the prediction and
 * clearing the miss count. Re-anchoring every beacon is why no drift model is
 * needed: error accumulates for one superframe, not indefinitely. */
void beacon_guard_beacon(struct beacon_guard *g, uint32_t beacon_at);

/* True if a transmission at tx_at is clear of the beacon window.
 *
 * Mutating: rolls the prediction forward past tx_at, counting each predicted
 * beacon it passes as a miss, and drops the lock past BEACON_GUARD_MAX_MISSES.
 * Always true while unlocked. */
bool beacon_guard_tx_allowed(struct beacon_guard *g, uint32_t tx_at);

/* Whether the prediction is currently trusted. Diagnostics and tests. */
bool beacon_guard_locked(const struct beacon_guard *g);

#endif /* BEACON_GUARD_H */
