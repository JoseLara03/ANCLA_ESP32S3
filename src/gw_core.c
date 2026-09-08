/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CFP seat management for the gateway: a [GW_CYCLE_C][GW_N_CFP] table of
 * seats, leases aged one superframe at a time across every phase, and a
 * monotonic short-address pool for joining tags. See gw_core.h for what
 * "phase" means here. Ported from the nRF5 gateway (fw-cre Src/gw_core.c);
 * it was already pure C with no radio dependency, which is why it carries the
 * whole host-test burden for the MAC's state machine.
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

/* Locate the seat holding `addr`, searching every phase. Returns false (and
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
 * considered, so a just-freed seat's stale EUI bytes can never match. */
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

    if (!find_seat_by_eui(c, eui, &phase, &slot)) {   /* not a re-join */
        /* INTERIM single-phase policy (Task 12 scope): a JOIN claims exactly
         * one (phase, slot) cell, found by scanning phase-major -- fill phase
         * 0's GW_N_CFP slots first, then phase 1's, and so on. This is
         * deliberately the simplest policy that still uses the whole
         * [GW_CYCLE_C][GW_N_CFP] table (so all GW_MAX_SEATS seats are
         * reachable, not just phase 0's), not an attempt at real allocation.
         * Task 13 replaces this with tier-based multi-phase allocation
         * (IDLE/SLOW/FAST -> 1/2/4/8 phases spread evenly around the cycle);
         * until that lands, req_tier is stored on the seat but does not
         * change how many phases a tag owns -- always one. */
        phase = -1;
        for (int p = 0; p < GW_CYCLE_C && phase < 0; p++) {
            for (int s = 0; s < GW_N_CFP; s++) {
                if (c->seats[p][s].short_addr == 0) {
                    phase = p;
                    slot = s;
                    break;
                }
            }
        }
        if (phase < 0) return false;                  /* network full */
        c->seats[phase][slot].short_addr = alloc_short_addr(c);
        memcpy(c->seats[phase][slot].eui, eui, UWB_FRAME_EUI_LEN);
    }
    c->seats[phase][slot].tier            = req_tier;
    c->seats[phase][slot].lease_remaining = GW_LEASE_SF;

    out->short_addr = c->seats[phase][slot].short_addr;
    out->phase      = (uint8_t)phase;
    out->slot_index = (uint8_t)slot;
    out->tier       = req_tier;
    out->lease      = GW_LEASE_SF;
    return true;
}

void gw_core_keepalive(struct gw_core_ctx *c, uint16_t short_addr, uint8_t req_tier)
{
    int phase, slot;

    if (!find_seat_by_addr(c, short_addr, &phase, &slot)) return;
    c->seats[phase][slot].tier            = req_tier;
    c->seats[phase][slot].lease_remaining = GW_LEASE_SF;
}

void gw_core_release(struct gw_core_ctx *c, uint16_t short_addr)
{
    int phase, slot;

    if (!find_seat_by_addr(c, short_addr, &phase, &slot)) return;
    memset(&c->seats[phase][slot], 0, sizeof(c->seats[phase][slot]));
}

void gw_core_superframe_tick(struct gw_core_ctx *c)
{
    c->frame_counter++;
    /* Every phase ages every call, not just whichever phase is "current" this
     * superframe -- a tag holding one phase and a tag holding four (once
     * Task 13 makes that possible) must both reach lease_remaining == 0 after
     * exactly GW_LEASE_SF calls. Aging only the current phase would make a
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
