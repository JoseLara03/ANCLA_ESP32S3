/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Seat table, leases, address pool and the per-superframe slot schedule.
 * See gw_core.h for why seats and slots are separate things.
 */

#include "gw_core.h"
#include <string.h>

/* Superframes between participations, indexed by normalized tier. */
static const uint32_t tier_period[GW_TIER_COUNT] = {
    [GW_TIER_IDLE] = 25u,
    [GW_TIER_SLOW] = 5u,
    [GW_TIER_FAST] = 1u,
};

uint8_t gw_core_normalize_tier(uint8_t tier)
{
    return (tier < GW_TIER_COUNT) ? tier : (uint8_t)GW_TIER_IDLE;
}

uint32_t gw_core_tier_period(uint8_t tier)
{
    return tier_period[gw_core_normalize_tier(tier)];
}

uint32_t gw_core_tier_cost(uint8_t tier)
{
    /* Whole number by construction: GW_SCHED_WINDOW_SF is the IDLE period and
     * every other period divides it. */
    return GW_SCHED_WINDOW_SF / gw_core_tier_period(tier);
}

/* ---- Seat lookup -------------------------------------------------------- */

static int find_seat_by_addr(const struct gw_core_ctx *c, uint16_t addr)
{
    if (addr == 0) return -1;
    for (unsigned int i = 0; i < GW_MAX_SEATS; i++)
        if (c->seats[i].short_addr == addr) return (int)i;
    return -1;
}

static int find_seat_by_eui(const struct gw_core_ctx *c, const uint8_t eui[UWB_FRAME_EUI_LEN])
{
    for (unsigned int i = 0; i < GW_MAX_SEATS; i++)
        if (c->seats[i].short_addr != 0 &&
            memcmp(c->seats[i].eui, eui, UWB_FRAME_EUI_LEN) == 0) return (int)i;
    return -1;
}

uint16_t gw_core_seats_used(const struct gw_core_ctx *c)
{
    uint16_t n = 0;

    for (unsigned int i = 0; i < GW_MAX_SEATS; i++)
        if (c->seats[i].short_addr != 0) n++;
    return n;
}

/* Recomputed from the table on every call rather than carried as a running
 * total. 128 iterations is nothing, and an incremental counter would have to be
 * adjusted correctly at five separate sites (join, re-join, tier change,
 * release, lease expiry) -- one missed adjustment and the gateway either
 * refuses tags forever or oversubscribes the CFP, both of which look like a
 * radio fault from outside. */
uint32_t gw_core_cost_used(const struct gw_core_ctx *c)
{
    uint32_t cost = 0;

    for (unsigned int i = 0; i < GW_MAX_SEATS; i++)
        if (c->seats[i].short_addr != 0)
            cost += gw_core_tier_cost(c->seats[i].tier);
    return cost;
}

/* Highest tier at or below `want` whose cost fits in `budget`, or -1 if not
 * even IDLE fits. Walks down from the request, so a tag asking for FAST in a
 * busy cell gets SLOW rather than a refusal. */
static int best_tier_within(uint8_t want, uint32_t budget)
{
    for (int t = (int)gw_core_normalize_tier(want); t >= 0; t--) {
        if (gw_core_tier_cost((uint8_t)t) <= budget) return t;
    }
    return -1;
}

/* Slot-superframes this seat may claim, excluding whatever it already costs --
 * so a re-join or keepalive at an unchanged tier is always admissible, and an
 * upgrade is charged only the difference. `own` is 0 for a seat that does not
 * exist yet. */
static uint32_t free_budget_for(const struct gw_core_ctx *c, uint32_t own)
{
    uint32_t used = gw_core_cost_used(c) - own;

    return (used < GW_SCHED_CAPACITY) ? GW_SCHED_CAPACITY - used : 0u;
}

/* The tier to actually grant. Prefers the strict ladder; falls back to IDLE
 * under the bounded overshoot of GW_SCHED_OVERSUB_SF so a saturated cell
 * degrades a tag instead of making it invisible. Returns -1 only when even
 * that is unavailable. */
static int grant_tier(const struct gw_core_ctx *c, uint8_t want, uint32_t own)
{
    int t = best_tier_within(want, free_budget_for(c, own));

    if (t >= 0) return t;

    if (GW_SCHED_OVERSUB_SF > 0u) {
        uint32_t used = gw_core_cost_used(c) - own;

        if (used + gw_core_tier_cost(GW_TIER_IDLE) <=
            GW_SCHED_CAPACITY + GW_SCHED_OVERSUB_SF) {
            return (int)GW_TIER_IDLE;
        }
    }
    return -1;
}

/* ---- Scheduling --------------------------------------------------------- */

/* Fill c->sched for the current frame_counter.
 *
 * Earliest-deadline-first over the live seats: a seat is eligible once
 * frame_counter has reached its next_due, and the most overdue eligible seats
 * take the GW_N_CFP slots. Serving a seat pushes its next_due out by its tier
 * period; a seat that misses out keeps its stale next_due and so becomes MORE
 * overdue, which is what makes the scheduler starvation-free without any
 * explicit fairness bookkeeping.
 *
 * Starvation-freedom depends on admission control, not on this function:
 * grant_tier() is what keeps the sum of demands at or below GW_N_CFP slots per
 * superframe on average, give or take the bounded GW_SCHED_OVERSUB_SF. Remove
 * that and this loop will happily let 11 FAST seats crowd out every IDLE seat
 * forever. The overshoot is safe here precisely because it is bounded and
 * IDLE-only: a few slot-superframes of excess make the most-overdue seats a
 * little later, which is the graceful direction, whereas an unbounded excess
 * would silently halve everyone's rate.
 *
 * Cost is GW_N_CFP * GW_MAX_SEATS comparisons = 11 * 128, on a K_PRIO_COOP(0)
 * loop once per 200 ms superframe. Selection is by repeated max rather than a
 * sort because the array must not be reordered -- the index IS the seat id.
 */
static void reschedule(struct gw_core_ctx *c)
{
    int chosen[GW_N_CFP];
    unsigned int n_chosen = 0;

    for (unsigned int slot = 0; slot < GW_N_CFP; slot++) {
        int      best      = -1;
        int32_t  best_late = 0;

        for (unsigned int i = 0; i < GW_MAX_SEATS; i++) {
            if (c->seats[i].short_addr == 0) continue;

            /* Already given a slot this superframe. n_chosen <= GW_N_CFP, so
             * this scan is at most 11 comparisons. */
            bool taken = false;
            for (unsigned int k = 0; k < n_chosen; k++)
                if (chosen[k] == (int)i) { taken = true; break; }
            if (taken) continue;

            /* Signed difference: correct across the frame counter's 2^32 wrap
             * for any interval under ~2^31 superframes, which is 13 millennia
             * at 200 ms. A plain unsigned compare would, on the one superframe
             * where the counter wraps, mark every seat as not-yet-due and emit
             * an entirely empty slot map. */
            int32_t late = (int32_t)(c->frame_counter - c->seats[i].next_due);

            if (late < 0) continue;                  /* not due yet */
            if (best < 0 || late > best_late) {
                best      = (int)i;
                best_late = late;
            }
        }

        if (best < 0) break;                         /* nothing else is due */

        chosen[n_chosen++] = best;
        c->sched[slot] = c->seats[best].short_addr;
        c->seats[best].next_due =
            c->frame_counter + gw_core_tier_period(c->seats[best].tier);
    }

    for (unsigned int slot = n_chosen; slot < GW_N_CFP; slot++)
        c->sched[slot] = UWB_FRAME_ADDR_BCAST;
}

/* ---- Address pool ------------------------------------------------------- */

static uint16_t alloc_short_addr(struct gw_core_ctx *c)
{
    /* Monotonic pool from GW_TAG_ADDR_BASE, skipping any address held by a
     * live seat. The guard bound is GW_MAX_SEATS (not GW_N_CFP as it was while
     * the table was slot-indexed): with GW_MAX_SEATS live seats, that many
     * collisions is the worst case before a free value must appear.
     *
     * DO NOT "improve" this into prompt address reuse. The long wrap --
     * 0x0100..0xFFFD is 65278 values, so a freed address is not handed out
     * again for that many joins -- is an accidental but load-bearing
     * QUARANTINE, and it is what protects the Tid fallback path:
     *
     * uwb_gateway.c's POS dispatch is deliberately not gated on seat state, so
     * a fix can arrive just after its sender's lease expired. It then calls
     * gw_core_find_eui(), which returns false, and the fix is published with
     * tag_id = src_addr -- an uninformative one-record phantom, documented and
     * accepted (see CLAUDE.md's Tid entry).
     *
     * Reuse an address promptly and that failure changes character. Tag A's
     * seat expires, tag B joins and is given A's old address, then a straggler
     * POS frame from A arrives: find_eui() now succeeds and returns B's EUI, so
     * A's position is attributed to B -- a real, wrong, live device. That is
     * the one path by which the fallback can be silently wrong about a real tag
     * rather than merely uninformative, and the monotonic pool is what keeps it
     * at the ~1.5e-5 coincidence bound instead of making it routine.
     *
     * The Tid fix (hashing the EUI) does not help here: the whole point of this
     * path is that the EUI looked up is the WRONG tag's. */
    for (unsigned int guard = 0; guard <= GW_MAX_SEATS; guard++) {
        uint16_t a = c->next_short_addr++;

        if (c->next_short_addr >= 0xFFFEu) c->next_short_addr = GW_TAG_ADDR_BASE;
        if (find_seat_by_addr(c, a) < 0) return a;
    }
    return GW_TAG_ADDR_BASE; /* unreachable with < GW_MAX_SEATS live seats */
}

/* ---- Lifecycle ---------------------------------------------------------- */

void gw_core_init(struct gw_core_ctx *c)
{
    memset(c, 0, sizeof(*c));
    c->next_short_addr = GW_TAG_ADDR_BASE;

    /* An empty map before the first tick, so a beacon built at frame 0 is
     * schema-valid rather than full of zeroes. */
    for (unsigned int slot = 0; slot < GW_N_CFP; slot++)
        c->sched[slot] = UWB_FRAME_ADDR_BCAST;
}

void gw_core_build_slotmap(const struct gw_core_ctx *c, uint16_t out[GW_N_CFP])
{
    memcpy(out, c->sched, sizeof(c->sched));
}

bool gw_core_find_eui(const struct gw_core_ctx *c, uint16_t short_addr,
                      uint8_t eui_out[UWB_FRAME_EUI_LEN])
{
    int idx = find_seat_by_addr(c, short_addr);

    if (idx < 0) {
        return false;
    }
    memcpy(eui_out, c->seats[idx].eui, UWB_FRAME_EUI_LEN);
    return true;
}

bool gw_core_join(struct gw_core_ctx *c, const uint8_t eui[UWB_FRAME_EUI_LEN],
                  uint8_t req_tier, struct gw_grant *out)
{
    int idx = find_seat_by_eui(c, eui);              /* idempotent re-join */
    bool fresh = (idx < 0);

    if (fresh) {
        for (unsigned int i = 0; i < GW_MAX_SEATS; i++)
            if (c->seats[i].short_addr == 0) { idx = (int)i; break; }
        if (idx < 0) return false;                   /* no seat left */
    }

    uint32_t own  = fresh ? 0u : gw_core_tier_cost(c->seats[idx].tier);
    int      tier = grant_tier(c, req_tier, own);

    if (tier < 0) {
        /* A fresh tag that cannot even be served at IDLE is refused outright.
         * An existing seat keeps what it already had rather than being evicted
         * by its own keepalive. */
        if (fresh) return false;
        tier = (int)c->seats[idx].tier;
    }

    if (fresh) {
        c->seats[idx].short_addr = alloc_short_addr(c);
        memcpy(c->seats[idx].eui, eui, UWB_FRAME_EUI_LEN);
        /* Due immediately, so the tag appears in the very next slot map rather
         * than waiting out a full tier period after being granted. */
        c->seats[idx].next_due = c->frame_counter;
    }
    c->seats[idx].tier            = (uint8_t)tier;
    c->seats[idx].lease_remaining = GW_LEASE_SF;

    out->short_addr = c->seats[idx].short_addr;
    out->seat_id    = (uint8_t)idx;
    out->tier       = (uint8_t)tier;
    out->lease      = GW_LEASE_SF;
    return true;
}

void gw_core_keepalive(struct gw_core_ctx *c, uint16_t short_addr,
                       uint8_t req_tier, struct gw_grant *out)
{
    int idx = find_seat_by_addr(c, short_addr);

    if (idx < 0) return;

    uint32_t own  = gw_core_tier_cost(c->seats[idx].tier);
    int      tier = grant_tier(c, req_tier, own);

    /* Never evict a seat on its own keepalive: if nothing fits, it keeps the
     * tier it already had. `own` was excluded from the budget above, so this
     * branch is only reachable for a genuine UPGRADE that does not fit. */
    if (tier < 0) tier = (int)c->seats[idx].tier;

    c->seats[idx].tier            = (uint8_t)tier;
    c->seats[idx].lease_remaining = GW_LEASE_SF;

    if (out) {
        out->short_addr = c->seats[idx].short_addr;
        out->seat_id    = (uint8_t)idx;
        out->tier       = (uint8_t)tier;
        out->lease      = GW_LEASE_SF;
    }
}

void gw_core_release(struct gw_core_ctx *c, uint16_t short_addr)
{
    int idx = find_seat_by_addr(c, short_addr);

    if (idx < 0) return;
    memset(&c->seats[idx], 0, sizeof(c->seats[idx]));
}

void gw_core_superframe_tick(struct gw_core_ctx *c)
{
    c->frame_counter++;
    for (unsigned int i = 0; i < GW_MAX_SEATS; i++) {
        if (c->seats[i].short_addr == 0) continue;
        if (c->seats[i].lease_remaining > 0) c->seats[i].lease_remaining--;
        if (c->seats[i].lease_remaining == 0)
            memset(&c->seats[i], 0, sizeof(c->seats[i]));
    }

    /* After reclamation, so a slot is never handed to a seat that expired this
     * very superframe. */
    reschedule(c);
}
