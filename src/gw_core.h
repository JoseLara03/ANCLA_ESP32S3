/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CFP seat management for the gateway: an 11-entry table keyed by slot index,
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

#define GW_N_CFP          UWB_FRAME_N_CFP   /* 11 ranging slots */
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

/* The tag identity published to the customer platform: the LOW 32 BITS of the
 * EUI-64, reading the byte array big-endian (eui[0] most significant, the same
 * order `apos enum` prints an EUI in), i.e. eui[4..7].
 *
 * Why not the short address, which is what this used to be: a short address is
 * a MAC lease, not an identity. gw_core_superframe_tick() wipes a seat when its
 * lease expires, and the next JOIN from the same tag misses find_seat_by_eui()
 * and draws a FRESH address from the monotonic pool -- so a tag that drops out
 * for 10 s comes back as a different device to the platform, and its old record
 * stops updating. The EUI-64 is burned into the tag (nRF52 FICR->DEVICEID) and
 * never changes.
 *
 * Why not the whole 64 bits: the platform's JSON layer is not guaranteed to
 * parse integers above 2^53 exactly, and a silently rounded Tid is worse than a
 * narrower one -- two tags could round onto the same value. 32 bits is exact in
 * any JSON parser and in an int32/uint32 database column. Both FICR->DEVICEID
 * words are random, so the low half carries full entropy; the collision risk is
 * birthday-on-2^32, which is negligible for a site's worth of tags.
 *
 * Returns 0 for a NULL eui. A genuine EUI whose low 32 bits are zero is
 * possible in principle (p = 2^-32) and would be indistinguishable here, which
 * is why callers carry a separate validity flag rather than testing for 0. */
uint32_t gw_core_tag_id(const uint8_t eui[UWB_FRAME_EUI_LEN]);

/* The EUI-64 of the live seat holding `short_addr`, or NULL if no seat holds it.
 * The gateway's POS path uses this to turn the address in a 0xEA frame back
 * into the stable identity above. Points into `c`; valid until the seat is
 * reused or expires. */
const uint8_t *gw_core_eui_by_addr(const struct gw_core_ctx *c, uint16_t short_addr);

void gw_core_init(struct gw_core_ctx *c);
bool gw_core_join(struct gw_core_ctx *c, const uint8_t eui[UWB_FRAME_EUI_LEN],
                  uint8_t req_tier, struct gw_grant *out);
void gw_core_keepalive(struct gw_core_ctx *c, uint16_t short_addr, uint8_t req_tier);
void gw_core_release(struct gw_core_ctx *c, uint16_t short_addr);
void gw_core_superframe_tick(struct gw_core_ctx *c);
void gw_core_build_slotmap(const struct gw_core_ctx *c, uint16_t out[GW_N_CFP]);

#endif /* GW_CORE_H */
