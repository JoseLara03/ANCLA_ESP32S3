/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CFP seat management for the gateway: a [GW_CYCLE_C][GW_N_CFP] table of
 * seats -- one row per TDMA "phase", GW_N_CFP CFP slots per phase -- leases
 * aged one superframe at a time for every phase, and a monotonic short-address
 * pool for joining tags. The single-phase table this started from was ported
 * unchanged from the nRF5 gateway (fw-cre Src/gw_core.c); the phase dimension
 * is new (network-scaling-v3 Task 12) and has no nRF5 ancestor. Still pure C
 * with no radio dependency, which is why it carries the whole host-test burden
 * for the MAC's state machine.
 *
 * "Phase" here means a tag's assigned slice of the GW_CYCLE_C-superframe
 * repeating cycle, not anything to do with the apos_* anchor-survey subsystem
 * (unrelated code elsewhere in this repo that also uses phase-shaped words in
 * its own domain). A seat's (phase, slot) pair is which superframe-within-the-
 * cycle and which CFP slot within it a tag ranges in. Today (Task 12) every
 * JOIN is granted exactly one phase -- see gw_core_join()'s comment. Task 13
 * (not this one) makes that tier-dependent: IDLE/SLOW/FAST tags get
 * 1/2/4/8 phases spread evenly around the cycle.
 */

#ifndef GW_CORE_H
#define GW_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "uwb_frame_802_15_4z.h"   /* UWB_FRAME_N_CFP / _EUI_LEN / _ADDR_BCAST */

#define GW_N_CFP          UWB_FRAME_N_CFP   /* ranging (CFP) slots per phase --
                                              * 11 today, becomes 14 once a
                                              * separate, deferred task grows
                                              * UWB_FRAME_N_CFP to match a
                                              * hardware slot-occupancy
                                              * measurement. Never assume 11
                                              * or 14 here; always read this
                                              * macro. */
#define GW_CYCLE_C        16                /* phase-cycle length: distinct
                                              * per-superframe "phases" a seat
                                              * can be scheduled into, so a
                                              * tag can eventually hold several
                                              * and range faster than once per
                                              * full 16-superframe cycle. Also
                                              * the modulus a later task's
                                              * ANNOUNCE rotation must stay
                                              * coprime with (gcd(A, C) == 1) --
                                              * unrelated to seat allocation,
                                              * noted here only so this isn't
                                              * shrunk without checking that
                                              * task if/when it lands.
                                              *
                                              * Deliberately no 'u' suffix,
                                              * unlike GW_LEASE_SF/
                                              * GW_TAG_ADDR_BASE below: this is
                                              * compared against plain `int`
                                              * loop counters all over
                                              * gw_core.c (same convention
                                              * GW_N_CFP already uses), and an
                                              * unsigned constant there would
                                              * make every one of those a
                                              * signed/unsigned comparison
                                              * warning under -Wextra. */
#define GW_LEASE_SF       50u               /* lease length, superframes */
#define GW_TAG_ADDR_BASE  0x0100u           /* tag short-addr pool base */

/* Total seat capacity across every phase. ALWAYS derive from this product --
 * never hardcode 176 (today's GW_N_CFP == 11) or 224 (the value once a
 * deferred task grows GW_N_CFP to 14): that other task changes GW_N_CFP alone
 * and this macro must track it automatically. */
#define GW_MAX_SEATS      (GW_CYCLE_C * GW_N_CFP)

struct gw_seat {
    uint16_t short_addr;                 /* 0 = free */
    uint8_t  eui[UWB_FRAME_EUI_LEN];
    uint8_t  tier;                       /* last requested tier */
    uint16_t lease_remaining;            /* superframes until reclaim */
};

struct gw_core_ctx {
    struct gw_seat seats[GW_CYCLE_C][GW_N_CFP];  /* [phase][CFP slot] */
    uint32_t frame_counter;
    uint16_t next_short_addr;                    /* monotonic pool */
};

struct gw_grant {
    uint16_t short_addr;
    uint8_t  phase;       /* which of the GW_CYCLE_C phases this seat lives in */
    uint8_t  slot_index;  /* CFP slot index within that phase */
    uint8_t  tier;
    uint16_t lease;
};

void gw_core_init(struct gw_core_ctx *c);
bool gw_core_join(struct gw_core_ctx *c, const uint8_t eui[UWB_FRAME_EUI_LEN],
                  uint8_t req_tier, struct gw_grant *out);
void gw_core_keepalive(struct gw_core_ctx *c, uint16_t short_addr, uint8_t req_tier);
void gw_core_release(struct gw_core_ctx *c, uint16_t short_addr);
void gw_core_superframe_tick(struct gw_core_ctx *c);

/* Publish the slot map for ONE phase (0..GW_CYCLE_C-1) -- the row of seats a
 * tag scheduled into that phase sees this superframe. A phase outside that
 * range is a caller bug, not a data condition worth propagating: it is
 * reported back as all-broadcast rather than indexing outside seats[][],
 * since the caller runs on the gateway's K_PRIO_COOP(0) loop where an
 * out-of-bounds array access is a far worse outcome than a wrong beacon. */
void gw_core_build_slotmap(const struct gw_core_ctx *c, uint8_t phase,
                           uint16_t out[GW_N_CFP]);

/* Look up the EUI of whichever seat currently holds `short_addr`. Returns
 * false (and leaves eui_out untouched) if no live seat holds that address --
 * this can legitimately happen for a POS frame that arrives just after its
 * sender's lease expired (see uwb_gateway.c's dispatch(), which does not
 * gate POS on seat state). Callers must have a fallback for false, not
 * treat it as an error. */
bool gw_core_find_eui(const struct gw_core_ctx *c, uint16_t short_addr,
                      uint8_t eui_out[UWB_FRAME_EUI_LEN]);

#endif /* GW_CORE_H */
