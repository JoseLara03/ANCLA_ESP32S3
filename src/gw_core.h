/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CFP seat management for the gateway: a seat table, leases aged one superframe
 * at a time, a short-address pool for joining tags, and the per-superframe
 * schedule that decides which seats transmit.
 *
 * Originally ported unchanged from the nRF5 gateway (fw-cre Src/gw_core.c),
 * where the seat table was indexed BY CFP SLOT -- 11 slots, therefore 11 tags,
 * with gw_core_join() returning false for the 12th. That is the shape this
 * module no longer has; see "seat versus schedule" below.
 *
 * Still pure C with no radio dependency, which is why it carries the whole
 * host-test burden for the MAC's state machine.
 *
 * ---- Seat versus schedule ------------------------------------------------
 *
 * The old table conflated two things that have different lifetimes:
 *
 *   - A SEAT is an identity: a short address, an EUI, a tier and a lease. It
 *     belongs to one tag and survives superframes in which that tag does not
 *     transmit. There are GW_MAX_SEATS of them.
 *   - A SLOT is airtime: one of the GW_N_CFP ranging windows in THIS
 *     superframe. It is handed out fresh every superframe.
 *
 * Because they were the same array index, a tag held its slot 100 % of the time
 * whatever its tier -- so an IDLE tag that ranges once every 25 superframes
 * still idled a slot for the other 24, and the MAC contract's claim that low
 * tiers "hand airtime back" was never true. Tiers saved tag battery and no
 * network capacity at all.
 *
 * Splitting them is what makes 100 tags fit in 11 slots. The wire format does
 * NOT change: the beacon still carries GW_N_CFP short addresses. What changes
 * is the meaning -- the slot map is now "who transmits this superframe", not
 * "who owns what". A tag absent from the map must therefore sleep rather than
 * conclude its seat was reclaimed; that is the tag-side half of the change and
 * the reason for the proto_ver bump. See
 * docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md section 4.1.
 */

#ifndef GW_CORE_H
#define GW_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "uwb_frame_802_15_4z.h"   /* UWB_FRAME_N_CFP / _EUI_LEN / _ADDR_BCAST */

#define GW_N_CFP          UWB_FRAME_N_CFP   /* ranging slots per superframe */
#define GW_LEASE_SF       50u               /* lease length, superframes */
#define GW_TAG_ADDR_BASE  0x0100u           /* tag short-addr pool base */

/* Logical seats. Deliberately much larger than GW_N_CFP -- that inequality is
 * the whole point of this module now.
 *
 * 128 covers the 100-tag product target with headroom, at 128 * 20 bytes = 2.5
 * kB of gateway RAM, which is nothing on this part. It is NOT a capacity
 * promise on its own: GW_SCHED_CAPACITY below is the binding limit for any tag
 * mix that is not entirely IDLE. */
#define GW_MAX_SEATS      128u

/* ---- Rate tiers ---------------------------------------------------------
 *
 * Superframes between participations, per MAC contract section 5.1. Indexed by
 * the tier value the tag requests, which is the tag's uwb_net.h enum:
 * 0 = IDLE, 1 = SLOW, 2 = FAST. Keep these three in step with that enum -- they
 * are a wire-visible contract, not a local tuning choice.
 *
 *   FAST  every superframe      5 Hz
 *   SLOW  every 5th             1 Hz
 *   IDLE  every 25th            0.2 Hz
 */
#define GW_TIER_IDLE      0u
#define GW_TIER_SLOW      1u
#define GW_TIER_FAST      2u
#define GW_TIER_COUNT     3u

/* The rate-accounting window. Equal to the IDLE tier's period, so every tier's
 * cost is a whole number of slot-superframes and the arithmetic below needs no
 * fractions. */
#define GW_SCHED_WINDOW_SF  25u

/* Slot-superframes available per window, and what one seat of each tier costs.
 * This is the same accounting mac_budget.h uses, deliberately: capacity is
 * counted in slot-superframes so tiers are directly comparable.
 *
 *   GW_N_CFP * GW_SCHED_WINDOW_SF = 11 * 25 = 275 available
 *   FAST 25, SLOW 5, IDLE 1
 *
 * So 11 FAST movers exactly saturate the cell, or 275 IDLE tags would -- except
 * GW_MAX_SEATS caps the seat count at 128 first. */
#define GW_SCHED_CAPACITY (GW_N_CFP * GW_SCHED_WINDOW_SF)

/* Bounded oversubscription, available ONLY to the bottom rung of the tier
 * ladder (IDLE).
 *
 * Why it exists: GW_N_CFP FAST seats cost 11 * 25 = 275, which is exactly
 * GW_SCHED_CAPACITY. Under strict admission control the next tag is then
 * refused outright -- even asking for IDLE, because not one slot-superframe
 * remains. That is arithmetically right and a bad product: eleven people
 * walking, and the twelfth tag cannot associate at all, so it is INVISIBLE to
 * the platform rather than merely slow.
 *
 * Allowing a bounded overshoot for IDLE only is the trade. IDLE is 0.2 Hz
 * presence telemetry, and the EDF scheduler degrades gracefully rather than
 * breaking: an oversubscribed seat is simply served a little late, and the
 * lateness ordering means the delay lands on whoever has waited longest. At
 * GW_N_CFP the worst case is 286/275 = 4 % over, i.e. an IDLE grant arriving
 * up to about one superframe late. Nothing about the FAST or SLOW guarantees
 * is touched: only IDLE may overshoot, so a rate-guaranteed tier can never be
 * admitted into capacity that does not exist.
 *
 * If a deployment would rather refuse a tag than let presence reports slip,
 * set this to 0 -- the behaviour reverts to strict admission control and the
 * twelfth tag is refused. That is a product decision, not a tuning knob. */
#define GW_SCHED_OVERSUB_SF  GW_N_CFP

/* Cost of one seat at `tier`, in slot-superframes per window. An unknown tier
 * is charged as IDLE, matching gw_core_normalize_tier(). */
uint32_t gw_core_tier_cost(uint8_t tier);

/* Superframes between participations for `tier`. Unknown tiers read as IDLE. */
uint32_t gw_core_tier_period(uint8_t tier);

/* Fold any byte off the wire into a tier this gateway implements. An unknown
 * value becomes IDLE -- the CHEAPEST tier, never the fastest, so a corrupt or
 * future-versioned request cannot be used to claim airtime. */
uint8_t gw_core_normalize_tier(uint8_t tier);

struct gw_seat {
    uint16_t short_addr;                 /* 0 = free */
    uint8_t  eui[UWB_FRAME_EUI_LEN];
    uint8_t  tier;                       /* GRANTED tier, already normalized */
    uint16_t lease_remaining;            /* superframes until reclaim */
    /* Frame counter at which this seat is next entitled to a slot. Compared
     * with signed difference arithmetic, so it is safe across the counter's
     * 2^32 wrap -- the same discipline beacon_guard.c uses for hi32. */
    uint32_t next_due;
};

struct gw_core_ctx {
    struct gw_seat seats[GW_MAX_SEATS];   /* index = seat id, NOT slot */
    uint32_t frame_counter;
    uint16_t next_short_addr;             /* monotonic pool */
    /* The slot map for the CURRENT superframe, recomputed by
     * gw_core_superframe_tick(). Held as state rather than computed inside
     * gw_core_build_slotmap() so that (a) build_slotmap stays a const read and
     * its production call site in uwb_gateway.c did not have to change, and
     * (b) it is structurally impossible to advance the schedule twice for one
     * superframe by calling the getter twice -- which would silently halve
     * every tag's update rate. */
    uint16_t sched[GW_N_CFP];
};

struct gw_grant {
    uint16_t short_addr;
    /* The tag's SEAT id, not a CFP slot index. Occupies the same GRANT byte as
     * before (it still fits: GW_MAX_SEATS is 128), and the tag echoes it in
     * KEEPALIVE for sanity. Which slot the tag transmits in is read from the
     * beacon's slot map every superframe instead. */
    uint8_t  seat_id;
    uint8_t  tier;                       /* GRANTED tier; may be below requested */
    uint16_t lease;
};

void gw_core_init(struct gw_core_ctx *c);

/* Admit a tag, or refresh it if its EUI already holds a seat.
 *
 * Grants the highest tier at or below `req_tier` that fits the remaining
 * GW_SCHED_CAPACITY, which the GRANT frame reports back -- the contract already
 * allows a granted tier to differ from the requested one (section 5.1), so
 * degrading is protocol-legal and strictly better than refusing.
 *
 * Returns false only when the tag cannot be admitted at all: no free seat, or
 * not even one IDLE seat's worth of capacity left. */
bool gw_core_join(struct gw_core_ctx *c, const uint8_t eui[UWB_FRAME_EUI_LEN],
                  uint8_t req_tier, struct gw_grant *out);

/* Refresh a lease and re-tier. A tier UPGRADE is admission-controlled exactly
 * as at join, so a fleet that all starts moving at once degrades gracefully
 * instead of oversubscribing the CFP. Writes the granted tier back through
 * `out` when it is non-NULL, so the caller can tell the tag what it actually
 * got. */
void gw_core_keepalive(struct gw_core_ctx *c, uint16_t short_addr,
                       uint8_t req_tier, struct gw_grant *out);

void gw_core_release(struct gw_core_ctx *c, uint16_t short_addr);

/* Age every lease, advance the frame counter, and recompute this superframe's
 * slot map. Call exactly once per superframe. */
void gw_core_superframe_tick(struct gw_core_ctx *c);

/* Copy out the current superframe's slot map. Pure read -- see `sched`. */
void gw_core_build_slotmap(const struct gw_core_ctx *c, uint16_t out[GW_N_CFP]);

/* Look up the EUI of whichever seat currently holds `short_addr`. Returns
 * false (and leaves eui_out untouched) if no live seat holds that address --
 * this can legitimately happen for a POS frame that arrives just after its
 * sender's lease expired (see uwb_gateway.c's dispatch(), which does not
 * gate POS on seat state). Callers must have a fallback for false, not
 * treat it as an error. */
bool gw_core_find_eui(const struct gw_core_ctx *c, uint16_t short_addr,
                      uint8_t eui_out[UWB_FRAME_EUI_LEN]);

/* ---- Introspection, for the shell and the host tests ------------------- */

/* Live seats, and the slot-superframes they consume of GW_SCHED_CAPACITY. */
uint16_t gw_core_seats_used(const struct gw_core_ctx *c);
uint32_t gw_core_cost_used(const struct gw_core_ctx *c);

#endif /* GW_CORE_H */
