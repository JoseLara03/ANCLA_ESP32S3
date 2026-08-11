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

#ifndef GW_CORE_H
#define GW_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "uwb_frame_802_15_4z.h"   /* UWB_FRAME_N_CFP / _EUI_LEN / _ADDR_BCAST */

#define GW_N_CFP          UWB_FRAME_N_CFP   /* 12 ranging slots */
#define GW_LEASE_SF       50u               /* lease length, superframes */
#define GW_TAG_ADDR_BASE  0x0100u           /* tag short-addr pool base */

struct gw_seat {
    uint16_t short_addr;                 /* 0 = free */
    uint8_t  eui[UWB_FRAME_EUI_LEN];
    uint8_t  tier;                       /* last requested tier */
    uint16_t lease_remaining;            /* superframes until reclaim */
};

struct gw_core_ctx {
    struct gw_seat seats[GW_N_CFP];      /* index = CFP slot */
    uint32_t frame_counter;
    uint16_t next_short_addr;            /* monotonic pool */
};

struct gw_grant {
    uint16_t short_addr;
    uint8_t  slot_index;
    uint8_t  tier;
    uint16_t lease;
};

void gw_core_init(struct gw_core_ctx *c);
bool gw_core_join(struct gw_core_ctx *c, const uint8_t eui[UWB_FRAME_EUI_LEN],
                  uint8_t req_tier, struct gw_grant *out);
void gw_core_keepalive(struct gw_core_ctx *c, uint16_t short_addr, uint8_t req_tier);
void gw_core_release(struct gw_core_ctx *c, uint16_t short_addr);
void gw_core_superframe_tick(struct gw_core_ctx *c);
void gw_core_build_slotmap(const struct gw_core_ctx *c, uint16_t out[GW_N_CFP]);

#endif /* GW_CORE_H */
