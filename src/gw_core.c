/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CFP seat management for the gateway: a [GW_CYCLE_C][GW_N_CFP] table of
 * seats, leases aged one superframe at a time across every phase, and a
 * monotonic short-address pool for joining tags. See gw_core.h for what
 * "phase" means here, and for the two invariants the allocator below
 * maintains (one CFP slot per tag across all its phases; those phases evenly
 * spread). Ported from the nRF5 gateway (fw-cre Src/gw_core.c); it was already
 * pure C with no radio dependency, which is why it carries the whole host-test
 * burden for the MAC's state machine.
 *
 * The allocator is deliberately non-preemptive: it claims only cells that are
 * free or already the requesting tag's, so no request of any tier can shrink
 * any other tag. The visible cost of that choice is that phases go to whoever
 * asks first -- an early FAST tag can hold GW_PHASES_MAX_FAST while a later
 * one is left with one, and enough of them can fill the table until a new JOIN
 * is refused. (That ceiling is 4 rather than 8 partly to bound this very
 * effect; see GW_PHASES_MAX_FAST in gw_core.h for the capacity arithmetic.)
 * That refusal is loud (uwb_gateway.c logs it) and self-correcting as movers
 * demote to SLOW/IDLE and their KEEPALIVEs hand the excess back, which is why
 * there is no fairness or reservation rule here. Adding one would mean taking
 * phases off a seated tag, and the silent-seat-loss failure that creates is a
 * worse outcome than an unfair-but-logged distribution.
 */

#include "gw_core.h"
#include <string.h>

void gw_core_init(struct gw_core_ctx *c)
{
    memset(c, 0, sizeof(*c));
    c->next_short_addr = GW_TAG_ADDR_BASE;
}

void gw_core_build_slotmap(const struct gw_core_ctx *c, uint8_t phase,
                           uint16_t out[GW_N_CFP])
{
    if (phase >= GW_CYCLE_C) {
        /* See the header comment: an out-of-range phase is a caller bug.
         * The only caller (uwb_gateway.c's tx_beacon()) derives phase from
         * ctx->frame_counter % GW_CYCLE_C, which is always in range, so this
         * is a backstop against indexing outside seats[][], not an expected
         * path. */
        for (int i = 0; i < GW_N_CFP; i++) out[i] = UWB_FRAME_ADDR_BCAST;
        return;
    }
    for (int i = 0; i < GW_N_CFP; i++) {
        out[i] = c->seats[phase][i].short_addr ? c->seats[phase][i].short_addr
                                                : UWB_FRAME_ADDR_BCAST;
    }
}

/* Locate the FIRST cell holding `addr`, searching every phase. A tag may hold
 * several cells now, so this answers only "is this address seated, and where
 * is one of its cells" -- enough for its three callers (a liveness test in
 * alloc_short_addr(), reading the shared EUI in gw_core_find_eui() and
 * gw_core_keepalive()), and NOT enough for anything that must act on the whole
 * tag: use tag_footprint() or free_all_cells() for that. Returns false (and
 * leaves *phase_out and *slot_out untouched) if no live seat holds it -- addr
 * == 0 never matches, since 0 means "free" in struct gw_seat. Bounded at
 * GW_CYCLE_C * GW_N_CFP (GW_MAX_SEATS) iterations worst case, same class of
 * bound as every other loop on this gateway-loop-reachable path. */
static bool find_seat_by_addr(const struct gw_core_ctx *c, uint16_t addr,
                               int *phase_out, int *slot_out)
{
    if (addr == 0) return false;
    for (int p = 0; p < GW_CYCLE_C; p++) {
        for (int s = 0; s < GW_N_CFP; s++) {
            if (c->seats[p][s].short_addr == addr) {
                *phase_out = p;
                *slot_out = s;
                return true;
            }
        }
    }
    return false;
}

/* Same search, keyed by EUI instead of short address -- used for idempotent
 * re-JOIN (a tag that missed its GRANT and retries must land on its existing
 * seat, not consume a second one). Only live seats (short_addr != 0) are
 * considered, so a just-freed seat's stale EUI bytes can never match. Like
 * find_seat_by_addr() this reports one cell of possibly many; the re-JOIN path
 * uses it purely to recover the tag's short address, then works from that. */
static bool find_seat_by_eui(const struct gw_core_ctx *c,
                              const uint8_t eui[UWB_FRAME_EUI_LEN],
                              int *phase_out, int *slot_out)
{
    for (int p = 0; p < GW_CYCLE_C; p++) {
        for (int s = 0; s < GW_N_CFP; s++) {
            if (c->seats[p][s].short_addr != 0 &&
                memcmp(c->seats[p][s].eui, eui, UWB_FRAME_EUI_LEN) == 0) {
                *phase_out = p;
                *slot_out = s;
                return true;
            }
        }
    }
    return false;
}

static uint16_t alloc_short_addr(struct gw_core_ctx *c)
{
    /* Monotonic pool from GW_TAG_ADDR_BASE; skip any address held by a live
     * seat anywhere in the table. At most GW_MAX_SEATS seats are live, so a
     * free value is found within that many guard iterations. */
    for (int guard = 0; guard <= GW_MAX_SEATS; guard++) {
        uint16_t a = c->next_short_addr++;
        int p, s;

        if (c->next_short_addr >= 0xFFFEu) c->next_short_addr = GW_TAG_ADDR_BASE;
        if (!find_seat_by_addr(c, a, &p, &s)) return a;
    }
    return GW_TAG_ADDR_BASE; /* unreachable with < GW_MAX_SEATS live seats */
}

/* ---- multi-phase bookkeeping -------------------------------------------
 *
 * One tag owns one cell per phase it holds, all carrying its short address.
 * The three helpers below are the whole of that bookkeeping: read a tag's
 * footprint, free it, and test whether a candidate phase set is claimable.
 */

/* Phase bitmask of every cell holding `addr`, plus the CFP slot they share.
 * Returns the number of cells found; 0 leaves *mask_out 0 and *slot_out -1.
 * The slot reported is the one belonging to the tag's LOWEST phase, which
 * under invariant 1 (gw_core.h) is every one of its cells' slot -- reading it
 * from the lowest phase rather than asserting the invariant means a violation
 * would misreport a slot instead of stepping outside seats[][]. */
static uint8_t tag_footprint(const struct gw_core_ctx *c, uint16_t addr,
                             uint16_t *mask_out, int *slot_out)
{
    uint8_t n = 0;

    *mask_out = 0;
    *slot_out = -1;
    if (addr == 0) return 0;

    for (int p = 0; p < GW_CYCLE_C; p++) {
        for (int s = 0; s < GW_N_CFP; s++) {
            if (c->seats[p][s].short_addr != addr) continue;
            *mask_out |= (uint16_t)(1u << p);
            if (*slot_out < 0) *slot_out = s;
            n++;
        }
    }
    return n;
}

/* Free every cell holding `addr`. Both gw_core_release() and the re-allocation
 * path need this: clearing only the first cell find_seat_by_addr() returns
 * would strand the rest holding a short address granted to nobody, still
 * published in their phases' beacon slot maps and still aging down to their own
 * expiry -- an orphan seat with no way to release it. addr == 0 is refused
 * because 0 is the "free" marker: a scan for it would match every empty cell. */
static void free_all_cells(struct gw_core_ctx *c, uint16_t addr)
{
    if (addr == 0) return;
    for (int p = 0; p < GW_CYCLE_C; p++) {
        for (int s = 0; s < GW_N_CFP; s++) {
            if (c->seats[p][s].short_addr == addr) {
                memset(&c->seats[p][s], 0, sizeof(c->seats[p][s]));
            }
        }
    }
}

/* Refresh lease_remaining to GW_LEASE_SF on every cell holding `addr`,
 * touching nothing else -- not tier, not the phase set, not the slot. Shared
 * by gw_core_keepalive()'s "regrant is unreachable" fallback (which also
 * refreshes tier, since a KEEPALIVE carries one) and gw_core_pos_seen() below
 * (which must NOT touch tier -- POS carries no tier field at all). addr == 0
 * is refused, same "0 means free" convention as free_all_cells() and
 * tag_footprint(). Bounded at GW_MAX_SEATS iterations, same class of bound as
 * every other loop on this gateway-loop-reachable path. */
static void refresh_all_leases(struct gw_core_ctx *c, uint16_t addr)
{
    if (addr == 0) return;
    for (int p = 0; p < GW_CYCLE_C; p++) {
        for (int s = 0; s < GW_N_CFP; s++) {
            if (c->seats[p][s].short_addr == addr) {
                c->seats[p][s].lease_remaining = GW_LEASE_SF;
            }
        }
    }
}

/* Is the evenly spread set { base, base+stride, ..., base+(n-1)*stride } at
 * CFP slot `slot` claimable by `addr`? A cell qualifies only when it is free
 * or ALREADY THIS TAG'S.
 *
 * That single predicate is the entire "under contention, a FAST tag never
 * takes the last phase of a stationary tag" rule -- there is no tier
 * comparison anywhere in this file, because no tier may evict any other at
 * all. A tag reduced to zero phases would have lost its seat without ever
 * passing through gw_core_release(): the beacon would just stop naming it,
 * its own KEEPALIVEs would be answered as GW_KEEPALIVE_UNKNOWN, and nothing
 * would have logged a thing.
 *
 * Passing addr == 0 (a tag with no seat yet) makes "or mine" collapse into
 * "free", which is exactly right for a first JOIN: nothing in the table
 * belongs to it.
 *
 * No modulo is needed: `base` is always < stride and stride * n ==
 * GW_CYCLE_C, so the largest member is (stride-1) + (n-1)*stride ==
 * GW_CYCLE_C - 1. */
static bool candidate_ok(const struct gw_core_ctx *c, int base, int stride,
                         int n, int slot, uint16_t addr)
{
    for (int k = 0; k < n; k++) {
        uint16_t held = c->seats[base + k * stride][slot].short_addr;

        if (held != 0 && held != addr) return false;
    }
    return true;
}

/* Lowest phase in a mask, or -1 for an empty mask. */
static int lowest_phase(uint16_t mask)
{
    for (int p = 0; p < GW_CYCLE_C; p++) {
        if (mask & (uint16_t)(1u << p)) return p;
    }
    return -1;
}

/* Ceiling on the phases a tier may hold. An unrecognised tier byte -- the tag
 * enum only defines 0..2, and this value arrives straight off the air -- is
 * treated as IDLE. That is the direction a malformed frame cannot exploit: the
 * opposite default (the tag's own "unknown reads back FAST", which is the safe
 * direction for a tag deciding how hard to listen) would let one bad byte
 * claim a FAST tag's whole allocation on a shared table. */
static int tier_max_phases(uint8_t tier)
{
    switch (tier) {
    case GW_TIER_FAST: return GW_PHASES_MAX_FAST;
    case GW_TIER_SLOW: return GW_PHASES_MAX_SLOW;
    case GW_TIER_IDLE: return GW_PHASES_MAX_IDLE;
    default:           return GW_PHASES_MAX_IDLE;
    }
}

/* Find an `n`-phase evenly spread candidate for `addr`.
 *
 * Search ORDER only ever affects churn -- any candidate passing candidate_ok()
 * is equally legal -- and the order below exists to make a steady-state
 * KEEPALIVE a no-op. A tag re-granted the same count must land back exactly
 * where it was, not get shuffled to a numerically lower base every lease
 * period: each move changes which superframes it ranges in, and it only
 * discovers that from the next beacon it happens to hear.
 *
 * Pass 1 (skipped when the tag holds nothing) keeps the tag's own CFP slot and
 * walks the bases starting from its current one, so both "nothing changed" and
 * "grow/shrink around where I already am" resolve without moving the slot
 * index. Pass 2 is exhaustive and base-major; with n == 1 stride becomes
 * GW_CYCLE_C, so `b` there IS the phase and pass 2 degenerates to exactly the
 * phase-major fill Task 12 documented and tests/gw_core pins -- phase 0's
 * GW_N_CFP slots before any of phase 1's.
 *
 * Cost is the same for every n, which is what keeps this safe to call from the
 * K_PRIO_COOP(0) gateway loop: pass 2 tests (GW_CYCLE_C / n) * GW_N_CFP
 * candidates of n cells each, i.e. exactly GW_MAX_SEATS cell reads however the
 * cycle is sliced, and pass 1 adds at most GW_CYCLE_C more. */
static bool find_candidate(const struct gw_core_ctx *c, uint16_t addr, int n,
                           int pref_base, int pref_slot,
                           int *base_out, int *slot_out)
{
    int stride = GW_CYCLE_C / n;

    if (pref_slot >= 0 && pref_base >= 0) {
        for (int i = 0; i < stride; i++) {
            int b = (pref_base % stride + i) % stride;

            if (candidate_ok(c, b, stride, n, pref_slot, addr)) {
                *base_out = b;
                *slot_out = pref_slot;
                return true;
            }
        }
    }
    for (int b = 0; b < stride; b++) {
        for (int s = 0; s < GW_N_CFP; s++) {
            if (candidate_ok(c, b, stride, n, s, addr)) {
                *base_out = b;
                *slot_out = s;
                return true;
            }
        }
    }
    return false;
}

/* Walk DOWN from `max_n` by halving until a candidate of that size is
 * available -- 4 -> 2 -> 1 at today's FAST ceiling, and generically any
 * power-of-two rung up to GW_CYCLE_C. The tier's ceiling is attempted first
 * and the fallback is driven purely by what is unclaimed, so no threshold has
 * to be invented and no tag is ever displaced to satisfy a bigger request.
 * Nothing here is capped at GW_PHASES_MAX_FAST: raising that constant is all
 * it takes to use a higher rung, and tests/gw_core keeps n == 8 covered
 * meanwhile (see gw_core_keepalive_max_for_test()).
 *
 * The ladder ALWAYS succeeds for a tag that already holds a cell: at n == 1
 * stride is GW_CYCLE_C, pass 1 of find_candidate() tries (pref_base,
 * pref_slot) first, and that cell is the tag's own so candidate_ok() accepts
 * it with no free cell needed anywhere. That is the structural reason a
 * re-allocation cannot strand a tag at zero phases -- it is not a threshold
 * that could be mistuned.
 *
 * The two skips make the halving safe if GW_CYCLE_C is ever changed to
 * something a rung does not divide, which would otherwise produce a set that
 * is not evenly spread (invariant 2). */
static bool alloc_phases(const struct gw_core_ctx *c, uint16_t addr, int max_n,
                         int pref_base, int pref_slot,
                         int *base_out, int *slot_out, int *n_out)
{
    for (int n = max_n; n >= 1; n /= 2) {
        if (n > GW_CYCLE_C) continue;
        if ((GW_CYCLE_C % n) != 0) continue;
        if (find_candidate(c, addr, n, pref_base, pref_slot, base_out, slot_out)) {
            *n_out = n;
            return true;
        }
    }
    return false;
}

/* Write `addr` into every cell of the winning candidate and return the phase
 * mask it produced. Every cell gets its own copy of the EUI and its own lease:
 * gw_core_superframe_tick() ages each cell independently, and they stay in
 * step only because they are all set to GW_LEASE_SF here, in the same call. */
static uint16_t claim(struct gw_core_ctx *c, uint16_t addr,
                      const uint8_t eui[UWB_FRAME_EUI_LEN], int base, int n,
                      int slot, uint8_t tier)
{
    int stride = GW_CYCLE_C / n;
    uint16_t mask = 0;

    for (int k = 0; k < n; k++) {
        struct gw_seat *seat = &c->seats[base + k * stride][slot];

        seat->short_addr      = addr;
        memcpy(seat->eui, eui, UWB_FRAME_EUI_LEN);
        seat->tier            = tier;
        seat->lease_remaining = GW_LEASE_SF;
        mask |= (uint16_t)(1u << (base + k * stride));
    }
    return mask;
}

/* Re-place `addr` on the phase set `max_n` and the table's occupancy now
 * allow, and refresh its lease. Returns false, having touched nothing, only
 * when no candidate of any size exists -- which for a tag that already holds a
 * cell cannot happen (see alloc_phases()), so in practice only a first JOIN
 * into a full table.
 *
 * The order is load-bearing. alloc_phases() runs BEFORE anything is freed, so
 * a failure leaves the tag exactly as it was; and the free then happens BEFORE
 * the claim, which is what makes candidate_ok()'s "or mine" safe to act on --
 * every cell in the winning set was free or this tag's own, so once the tag's
 * own are cleared they are all free. Freeing all of them (not just the ones
 * outside the winning set) is also what returns a tier DROP's excess cells to
 * the pool instead of leaving them held. */
static bool regrant(struct gw_core_ctx *c, uint16_t addr,
                    const uint8_t eui[UWB_FRAME_EUI_LEN], uint8_t tier,
                    int max_n, struct gw_grant *out, bool *changed_out)
{
    uint16_t old_mask;
    int old_slot;
    int base, slot, n;

    (void)tag_footprint(c, addr, &old_mask, &old_slot);

    if (!alloc_phases(c, addr, max_n, lowest_phase(old_mask), old_slot,
                      &base, &slot, &n)) {
        return false;
    }

    free_all_cells(c, addr);
    uint16_t mask = claim(c, addr, eui, base, n, slot, tier);

    if (changed_out) {
        *changed_out = (mask != old_mask) || (slot != old_slot);
    }
    if (out) {
        out->short_addr = addr;
        out->phase      = (uint8_t)base;
        out->slot_index = (uint8_t)slot;
        out->phase_mask = mask;
        out->tier       = tier;
        out->lease      = GW_LEASE_SF;
    }
    return true;
}

bool gw_core_find_eui(const struct gw_core_ctx *c, uint16_t short_addr,
                      uint8_t eui_out[UWB_FRAME_EUI_LEN])
{
    int phase, slot;

    if (!find_seat_by_addr(c, short_addr, &phase, &slot)) {
        return false;
    }
    memcpy(eui_out, c->seats[phase][slot].eui, UWB_FRAME_EUI_LEN);
    return true;
}

bool gw_core_join(struct gw_core_ctx *c, const uint8_t eui[UWB_FRAME_EUI_LEN],
                  uint8_t req_tier, struct gw_grant *out)
{
    int phase, slot;

    if (find_seat_by_eui(c, eui, &phase, &slot)) {
        /* Re-JOIN: this EUI already holds a seat, so the tag missed its GRANT
         * and retried. Re-grant it the count it already has -- refreshing the
         * lease and the recorded tier, and re-reporting its phase set so the
         * replacement GRANT describes reality -- rather than re-running the
         * tier ladder. Growing here would let a tag that keeps losing GRANTs
         * ratchet itself up phases it has never actually used, and shrinking
         * here would punish it for a lost frame; gw_core_keepalive() is the
         * one place tiering is applied. */
        uint16_t addr = c->seats[phase][slot].short_addr;
        uint16_t held_mask;   /* only the cell COUNT is wanted here; regrant() */
        int held_slot;        /* re-reads the footprint to find the tag itself */
        uint8_t held = tag_footprint(c, addr, &held_mask, &held_slot);

        return regrant(c, addr, eui, req_tier, (int)held, out, NULL);
    }

    /* A tag with no seat gets exactly ONE phase whatever tier it asked for:
     * it has done nothing but broadcast a JOIN, and gw_core_keepalive() hands
     * out the phases its tier earns once it proves it is staying. Placement is
     * left to find_candidate()'s exhaustive pass, which for n == 1 fills
     * phase-major -- all of phase 0's GW_N_CFP slots, then phase 1's -- so
     * every one of the GW_MAX_SEATS cells is reachable rather than just phase
     * 0's row.
     *
     * addr 0 here is not a placeholder to fix up later: it is what tells
     * candidate_ok() that nothing in the table already belongs to this tag.
     * The address is drawn only once a cell is known to exist, so a refused
     * JOIN does not burn a value out of the monotonic pool. */
    int base, n;

    if (!alloc_phases(c, 0, 1, -1, -1, &base, &slot, &n)) {
        return false;                                  /* network full */
    }

    uint16_t addr = alloc_short_addr(c);
    uint16_t mask = claim(c, addr, eui, base, n, slot, req_tier);

    out->short_addr = addr;
    out->phase      = (uint8_t)base;
    out->slot_index = (uint8_t)slot;
    out->phase_mask = mask;
    out->tier       = req_tier;
    out->lease      = GW_LEASE_SF;
    return true;
}

/* The whole of gw_core_keepalive(), with the phase ceiling supplied by the
 * caller instead of looked up from the tier. Both public entry points below
 * are one line each over this, so the tier-driven production path and the
 * explicit-ceiling test path cannot drift apart. */
static enum gw_keepalive_result keepalive_with_max(struct gw_core_ctx *c,
                                                   uint16_t short_addr,
                                                   uint8_t req_tier, int max_n,
                                                   struct gw_grant *out)
{
    int phase, slot;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    bool changed = false;

    if (!find_seat_by_addr(c, short_addr, &phase, &slot)) {
        return GW_KEEPALIVE_UNKNOWN;
    }
    /* Copied out before regrant() frees the tag's cells -- the EUI lives only
     * in the seats, and every cell of the winning set needs its own copy. */
    memcpy(eui, c->seats[phase][slot].eui, UWB_FRAME_EUI_LEN);

    if (!regrant(c, short_addr, eui, req_tier, max_n, out, &changed)) {
        /* Unreachable: alloc_phases()'s n == 1 rung re-claims a cell this tag
         * already holds and so cannot fail once find_seat_by_addr() has
         * succeeded. Kept because the alternative on a future refactor that
         * broke that property is a tag whose lease quietly runs out while it
         * is still sending KEEPALIVEs -- so refresh in place instead, leaving
         * the phase set alone. */
        for (int p = 0; p < GW_CYCLE_C; p++) {
            for (int s = 0; s < GW_N_CFP; s++) {
                if (c->seats[p][s].short_addr == short_addr) {
                    c->seats[p][s].tier = req_tier;
                }
            }
        }
        refresh_all_leases(c, short_addr);
        return GW_KEEPALIVE_SAME;
    }
    return changed ? GW_KEEPALIVE_REPHASED : GW_KEEPALIVE_SAME;
}

enum gw_keepalive_result gw_core_keepalive(struct gw_core_ctx *c,
                                           uint16_t short_addr, uint8_t req_tier,
                                           struct gw_grant *out)
{
    /* The tier ceiling is the ONLY thing this adds over the worker above, and
     * it is the whole of tier policy: everything else -- the seat lookup, the
     * EUI copy, the regrant and its unreachable fallback -- is shared. */
    return keepalive_with_max(c, short_addr, req_tier,
                              tier_max_phases(req_tier), out);
}

/* See gw_core.h: a test seam, not for production use. It exists so the
 * allocator's n == 8 rung stays covered now that GW_PHASES_MAX_FAST is 4 and
 * no tier's ceiling reaches it. */
enum gw_keepalive_result gw_core_keepalive_max_for_test(struct gw_core_ctx *c,
                                                        uint16_t short_addr,
                                                        uint8_t req_tier,
                                                        int max_phases,
                                                        struct gw_grant *out)
{
    return keepalive_with_max(c, short_addr, req_tier, max_phases, out);
}

/* POS carries no tier field at all (uwb_frame_pos_build()/_parse_pos() --
 * src_addr, x, y, residual_m, n_anchors, batt_soc, nothing else), so a POS
 * frame can only ever be a "this address is still alive" signal, never a
 * re-grant. Refreshes lease_remaining on every cell `short_addr` holds and
 * changes nothing else -- not tier, not the phase set, not the slot. A no-op,
 * not an error, when no live seat holds that address (a straggler POS after
 * lease expiry, or any other unknown address): it never resurrects or creates
 * a seat, matching uwb_gateway.c's existing rule that POS is not gated on
 * seat state. */
void gw_core_pos_seen(struct gw_core_ctx *c, uint16_t short_addr)
{
    refresh_all_leases(c, short_addr);
}

void gw_core_release(struct gw_core_ctx *c, uint16_t short_addr)
{
    /* Every cell, not the first one found -- see the header comment. An
     * unknown address is a bounded no-op scan, not an error. */
    free_all_cells(c, short_addr);
}

void gw_core_superframe_tick(struct gw_core_ctx *c)
{
    c->frame_counter++;
    /* Every phase ages every call, not just whichever phase is "current" this
     * superframe -- a tag holding one phase and a tag holding four must both
     * reach lease_remaining == 0 after exactly GW_LEASE_SF calls. That is also
     * why a multi-phase tag expires as ONE unit with no bookkeeping here: all
     * of its cells were set to GW_LEASE_SF in the same claim() call and each
     * decrements once per call, so they hit zero together and are reclaimed on
     * the same tick. Aging only the current phase would make a
     * one-phase tag's lease last GW_CYCLE_C times longer in wall-clock terms
     * than intended. GW_CYCLE_C * GW_N_CFP (GW_MAX_SEATS) iterations, bounded
     * and cheap -- this runs once per 200 ms superframe on the gateway's
     * K_PRIO_COOP(0) loop. */
    for (int p = 0; p < GW_CYCLE_C; p++) {
        for (int s = 0; s < GW_N_CFP; s++) {
            struct gw_seat *seat = &c->seats[p][s];

            if (seat->short_addr == 0) continue;
            if (seat->lease_remaining > 0) seat->lease_remaining--;
            if (seat->lease_remaining == 0) memset(seat, 0, sizeof(*seat));
        }
    }
}
