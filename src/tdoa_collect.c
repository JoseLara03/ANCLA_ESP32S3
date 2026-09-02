/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See tdoa_collect.h for the design rationale.
 */

#include "tdoa_collect.h"
#include <string.h>

void tdoa_collect_init(struct tdoa_collect *c)
{
	memset(c, 0, sizeof(*c));
	c->expected = POS_MAX_ANCHORS;
}

void tdoa_collect_set_expected(struct tdoa_collect *c, uint8_t n)
{
	if (n < TDOA_MIN_ANCHORS) n = TDOA_MIN_ANCHORS;
	if (n > POS_MAX_ANCHORS) n = POS_MAX_ANCHORS;
	c->expected = n;
}

/* Signed age of a group at `now_ms`, correct across a wrapping ms counter for
 * any interval under ~2^31 ms. Same discipline as gw_core.c's frame_counter
 * and beacon_guard.c's hi32. */
static int32_t age_ms(uint32_t now_ms, uint32_t first_ms)
{
	return (int32_t)(now_ms - first_ms);
}

static struct tdoa_group *find_group(struct tdoa_collect *c,
				     uint16_t tag_addr, uint8_t blink_seq)
{
	for (unsigned int i = 0; i < TDOA_COLLECT_SLOTS; i++) {
		struct tdoa_group *g = &c->slot[i];

		if (g->used && g->tag_addr == tag_addr &&
		    g->blink_seq == blink_seq)
			return g;
	}
	return NULL;
}

static struct tdoa_group *find_free(struct tdoa_collect *c)
{
	for (unsigned int i = 0; i < TDOA_COLLECT_SLOTS; i++)
		if (!c->slot[i].used) return &c->slot[i];
	return NULL;
}

/* The least-complete group, ties broken by oldest first_ms -- the eviction
 * victim on slot exhaustion. Never returns a group that is already
 * RELEASABLE (n >= TDOA_MIN_ANCHORS) while a less-complete one exists,
 * because that would destroy an already-resolvable fix to make room for a
 * blink that may never complete. Returns NULL when every slot already holds
 * a releasable group -- see tdoa_collect.h's slot-exhaustion note for what
 * the caller does with that. Only called when every slot is in use. */
static struct tdoa_group *find_eviction_victim(struct tdoa_collect *c,
					       uint32_t now_ms)
{
	struct tdoa_group *victim = NULL;
	int32_t victim_age = 0;

	for (unsigned int i = 0; i < TDOA_COLLECT_SLOTS; i++) {
		struct tdoa_group *g = &c->slot[i];

		if (g->n >= TDOA_MIN_ANCHORS) continue;   /* protected: releasable */

		int32_t a = age_ms(now_ms, g->first_ms);

		if (victim == NULL || g->n < victim->n ||
		    (g->n == victim->n && a > victim_age)) {
			victim = g;
			victim_age = a;
		}
	}
	return victim;
}

bool tdoa_collect_add(struct tdoa_collect *c, const struct tdoa_obs *o,
		      uint32_t now_ms)
{
	if (o->anchor_id >= POS_MAX_ANCHORS) return false;

	uint32_t bit = 1u << o->anchor_id;
	struct tdoa_group *g = find_group(c, o->tag_addr, o->blink_seq);

	if (g == NULL) {
		g = find_free(c);
		if (g == NULL) {
			g = find_eviction_victim(c, now_ms);
			/* Every slot already holds a releasable group: reject rather
			 * than destroy an already-resolvable fix. */
			if (g == NULL) return false;
		}

		g->tag_addr    = o->tag_addr;
		g->blink_seq   = o->blink_seq;
		g->n           = 0;
		g->anchor_bits = 0;
		g->first_ms    = now_ms;
		g->used        = true;
	}

	if (g->anchor_bits & bit) return false;   /* duplicate anchor, ignored */

	g->meas[g->n] = o->meas;
	g->n++;
	g->anchor_bits |= bit;
	return true;
}

bool tdoa_collect_take_ready(struct tdoa_collect *c, uint32_t now_ms,
			     struct tdoa_meas *out, size_t *n_out,
			     uint16_t *tag_addr_out)
{
	bool have_ready = false;
	struct tdoa_group *ready = NULL;

	for (unsigned int i = 0; i < TDOA_COLLECT_SLOTS; i++) {
		struct tdoa_group *g = &c->slot[i];

		if (!g->used) continue;

		if (g->n >= c->expected) {
			if (!have_ready) { ready = g; have_ready = true; }
			continue;
		}

		if (age_ms(now_ms, g->first_ms) >= (int32_t)TDOA_COLLECT_WINDOW_MS) {
			if (g->n >= TDOA_MIN_ANCHORS) {
				if (!have_ready) { ready = g; have_ready = true; }
			} else {
				g->used = false;   /* below minimum: discard */
			}
		}
	}

	if (!have_ready) return false;

	memcpy(out, ready->meas, ready->n * sizeof(*out));
	*n_out = ready->n;
	*tag_addr_out = ready->tag_addr;
	ready->used = false;
	return true;
}
