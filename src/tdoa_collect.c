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

/* Oldest by first_ms, measured as the largest signed age against `now_ms`.
 * Only called when every slot is in use, so it always finds one. */
static struct tdoa_group *find_oldest(struct tdoa_collect *c, uint32_t now_ms)
{
	struct tdoa_group *oldest = &c->slot[0];
	int32_t oldest_age = age_ms(now_ms, oldest->first_ms);

	for (unsigned int i = 1; i < TDOA_COLLECT_SLOTS; i++) {
		int32_t a = age_ms(now_ms, c->slot[i].first_ms);

		if (a > oldest_age) {
			oldest = &c->slot[i];
			oldest_age = a;
		}
	}
	return oldest;
}

bool tdoa_collect_add(struct tdoa_collect *c, const struct tdoa_obs *o,
		      uint32_t now_ms)
{
	if (o->anchor_id >= POS_MAX_ANCHORS) return false;

	uint32_t bit = 1u << o->anchor_id;
	struct tdoa_group *g = find_group(c, o->tag_addr, o->blink_seq);

	if (g == NULL) {
		g = find_free(c);
		if (g == NULL)
			g = find_oldest(c, now_ms);   /* slot exhaustion: evict oldest */

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

		if (g->n >= POS_MAX_ANCHORS) {
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
