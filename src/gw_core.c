/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CFP seat management for the gateway: a 12-entry table keyed by slot index,
 * leases aged one superframe at a time, and a monotonic short-address pool for
 * joining tags. Ported unchanged from the nRF5 gateway (fw-cre Src/gw_core.c);
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

void gw_core_build_slotmap(const struct gw_core_ctx *c, uint16_t out[GW_N_CFP])
{
    for (int i = 0; i < GW_N_CFP; i++) {
        out[i] = c->seats[i].short_addr ? c->seats[i].short_addr
                                        : UWB_FRAME_ADDR_BCAST;
    }
}

static int find_seat_by_addr(const struct gw_core_ctx *c, uint16_t addr)
{
    if (addr == 0) return -1;
    for (int i = 0; i < GW_N_CFP; i++)
        if (c->seats[i].short_addr == addr) return i;
    return -1;
}

static int find_seat_by_eui(const struct gw_core_ctx *c, const uint8_t eui[UWB_FRAME_EUI_LEN])
{
    for (int i = 0; i < GW_N_CFP; i++)
        if (c->seats[i].short_addr != 0 &&
            memcmp(c->seats[i].eui, eui, UWB_FRAME_EUI_LEN) == 0) return i;
    return -1;
}

static uint16_t alloc_short_addr(struct gw_core_ctx *c)
{
    /* Monotonic pool from GW_TAG_ADDR_BASE; skip any address held by a live
     * seat. At most GW_N_CFP seats are live, so a free value is found quickly. */
    for (int guard = 0; guard <= GW_N_CFP; guard++) {
        uint16_t a = c->next_short_addr++;
        if (c->next_short_addr >= 0xFFFEu) c->next_short_addr = GW_TAG_ADDR_BASE;
        if (find_seat_by_addr(c, a) < 0) return a;
    }
    return GW_TAG_ADDR_BASE; /* unreachable with < GW_N_CFP live seats */
}

bool gw_core_join(struct gw_core_ctx *c, const uint8_t eui[UWB_FRAME_EUI_LEN],
                  uint8_t req_tier, struct gw_grant *out)
{
    int idx = find_seat_by_eui(c, eui);              /* idempotent re-join */
    if (idx < 0) {
        for (int i = 0; i < GW_N_CFP; i++)
            if (c->seats[i].short_addr == 0) { idx = i; break; }
        if (idx < 0) return false;                   /* network full */
        c->seats[idx].short_addr = alloc_short_addr(c);
        memcpy(c->seats[idx].eui, eui, UWB_FRAME_EUI_LEN);
    }
    c->seats[idx].tier            = req_tier;
    c->seats[idx].lease_remaining = GW_LEASE_SF;

    out->short_addr = c->seats[idx].short_addr;
    out->slot_index = (uint8_t)idx;
    out->tier       = req_tier;
    out->lease      = GW_LEASE_SF;
    return true;
}

void gw_core_keepalive(struct gw_core_ctx *c, uint16_t short_addr, uint8_t req_tier)
{
    int idx = find_seat_by_addr(c, short_addr);
    if (idx < 0) return;
    c->seats[idx].tier            = req_tier;
    c->seats[idx].lease_remaining = GW_LEASE_SF;
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
    for (int i = 0; i < GW_N_CFP; i++) {
        if (c->seats[i].short_addr == 0) continue;
        if (c->seats[i].lease_remaining > 0) c->seats[i].lease_remaining--;
        if (c->seats[i].lease_remaining == 0)
            memset(&c->seats[i], 0, sizeof(c->seats[i]));
    }
}
