/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "beacon_guard.h"

/* Signed-difference comparison. hi32 wraps every ~17.2 s, so "is a after b"
 * is only meaningful as the sign of the difference, never as a > b. Correct
 * for any interval under ~8.6 s; every span here is one superframe (200 ms). */
static inline bool after_or_eq(uint32_t a, uint32_t b)
{
	return (int32_t)(a - b) >= 0;
}

void beacon_guard_init(struct beacon_guard *g, uint32_t period, uint32_t guard,
		       uint32_t occupancy)
{
	g->next_beacon = 0;
	g->period = period;
	g->guard = guard;
	g->occupancy = occupancy;
	g->misses = 0;
	g->locked = false;
}

void beacon_guard_beacon(struct beacon_guard *g, uint32_t beacon_at)
{
	g->next_beacon = beacon_at + g->period;
	g->misses = 0;
	g->locked = true;
}

bool beacon_guard_tx_allowed(struct beacon_guard *g, uint32_t tx_at)
{
	if (!g->locked) {
		return true;
	}

	/* Advance to the superframe tx_at actually falls in. Each beacon we
	 * step over is one we predicted but never received. */
	while ((int32_t)(tx_at - (g->next_beacon + g->occupancy + g->guard)) > 0) {
		if (g->misses >= BEACON_GUARD_MAX_MISSES) {
			g->locked = false;
			return true;
		}
		g->misses++;
		g->next_beacon += g->period;
	}

	/* Forbidden: [next_beacon - guard, next_beacon + occupancy + guard]. */
	if (after_or_eq(tx_at, g->next_beacon - g->guard)) {
		return false;
	}
	return true;
}

bool beacon_guard_locked(const struct beacon_guard *g)
{
	return g->locked;
}

uint32_t beacon_guard_next(const struct beacon_guard *g)
{
	return g->next_beacon;
}

uint8_t beacon_guard_misses(const struct beacon_guard *g)
{
	return g->misses;
}
