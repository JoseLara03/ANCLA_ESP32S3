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
 * cycle and which CFP slot within it a tag ranges in.
 *
 * A tag holds ONE seat per phase it owns, so a FAST tag occupies up to
 * GW_PHASES_MAX_FAST cells that all carry the same short_addr and the same
 * EUI. Two invariants follow, and everything in gw_core.c depends on them:
 *
 *   1. All of one tag's cells sit in the SAME CFP slot index, differing only
 *      in phase. That is what lets a whole grant be described by one slot
 *      number plus a 16-bit phase mask -- the shape a separate, deferred task
 *      puts on the wire by growing the GRANT frame to carry `phase_mask`.
 *   2. A tag's cells are spread EVENLY: { base, base + C/n, base + 2C/n, ... }
 *      for n phases, C == GW_CYCLE_C. Four consecutive phases would give a
 *      mover four fixes inside 0.8 s and then 2.4 s of silence, which is worse
 *      for its EKF than an even 1.25 Hz.
 *
 * How a tag learns where to range today: NOT from the GRANT. It looks its own
 * short address up in the slot map of every beacon it receives
 * (uwb_frame_beacon_find_addr(), called from
 * tag_testting/src/uwb_net_runner.c:795-805, which sets ev.in_map/ev.map_slot
 * from the result) and ranges in the CFP slot that names it. That much is
 * real, and it is how a tag picks up its SLOT without being re-GRANTed.
 *
 * What does NOT follow -- and an earlier version of this comment asserted that
 * it did, wrongly and in the dangerous direction -- is that publishing a tag in
 * n phases is by itself enough to make it range n times per cycle. A beacon
 * carries ONE phase's row (uwb_gateway.c's tx_beacon(): frame_counter %
 * GW_CYCLE_C), and the current tag firmware treats its absence from a beacon
 * it actually RECEIVED as a reclaimed lease: in both UWB_ST_DISCOVER
 * (tag_testting/src/uwb_net.c:282-284) and UWB_ST_RANGING (the same file,
 * :320-322), `if (!ev->in_map)` goes straight to UWB_ST_SCAN on the FIRST such
 * beacon, with no miss tolerance -- UWB_NET_MISS_MAX guards beacons that never
 * ARRIVED (UWB_EV_BEACON_MISS), not beacons that arrived without the tag in
 * them. A FAST tag hears every one of them: tier_defaults in
 * tag_testting/src/uwb_net.c:17-21 gives FAST listen_skip == 1.
 *
 * So a tag holding n < GW_CYCLE_C phases is absent from GW_CYCLE_C - n beacons
 * per cycle and loses its seat on the first of those. Two things are therefore
 * genuine FUNCTIONAL prerequisites -- not the power optimisation this comment
 * used to call them -- before such a grant can be exercised against real tag
 * firmware:
 *
 *   a. the deferred GRANT `phase_mask` (invariant 1 above), so a tag is told
 *      which phases are its own; and
 *   b. a tag-side change deriving listen_skip and the in_map test FROM that
 *      mask -- checking only the beacons of the phases it owns, instead of
 *      reading absence from any beacon at all as lease loss.
 *
 * Scope this honestly: the hazard is a property of publishing one phase per
 * beacon, which arrived with the phase table itself, NOT of multi-phase
 * allocation. It already applies to the single-phase grants that predate the
 * tier ladder below, and holding MORE phases makes a tag absent from FEWER
 * beacons, not more. Nothing here is unsafe to land -- the allocator and its
 * host tests are self-contained gateway-side bookkeeping -- but until (a) and
 * (b) both exist, no phase grant smaller than GW_CYCLE_C should be believed on
 * air, and a multi-phase one in particular will bounce a tag to SCAN rather
 * than merely cost it power.
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

/* Rate tiers. These are WIRE values, not a private enum: `req_tier` reaches
 * gw_core_join()/gw_core_keepalive() straight out of byte 18 of a JOIN and
 * byte 10 of a KEEPALIVE, written by the tag from its own
 *   typedef enum { UWB_TIER_IDLE = 0, UWB_TIER_SLOW = 1, UWB_TIER_FAST = 2 }
 * (tag_testting/src/uwb_net.h). Renumbering either side re-tiers every tag in
 * the field with nothing reporting a fault: a FAST mover read as IDLE gets one
 * phase and ranges at 0.31 Hz, which looks like a weak-signal problem rather
 * than a MAC one. Keep these three equal to the tag's enum. */
#define GW_TIER_IDLE      0u
#define GW_TIER_SLOW      1u
#define GW_TIER_FAST      2u

/* Phases a tag of each tier may hold. These are CEILINGS, not fixed counts:
 * the allocator starts at the tier's ceiling and halves (4 -> 2 -> 1) until it
 * finds that many evenly spread phases actually free at one CFP slot, so a
 * shortfall is measured by what is genuinely unclaimed and never by evicting
 * anyone. Halving keeps invariant 2 above exact: every rung divides
 * GW_CYCLE_C. An unrecognised tier byte is treated as IDLE, the direction that
 * cannot let a malformed frame claim the maximum.
 *
 * GW_PHASES_MAX_FAST is 4 -- the plan's "FAST 4". Its parenthetical "(8 when
 * the budget allows)" is DELIBERATELY NOT IMPLEMENTED: deciding when a budget
 * allows it needs system-wide occupancy sensing that does not exist here, and
 * a conservative default is cheap while an early-filled table is not. The
 * arithmetic behind that choice: GW_MAX_SEATS is GW_CYCLE_C * GW_N_CFP == 176
 * today (224 only once a deferred task grows GW_N_CFP to 14), while 20
 * simultaneous FAST tags at 8 phases plus 80 stationary ones at 1 comes to
 * 240 -- overcommitted even against the eventual 224. And exhaustion does not
 * gracefully degrade the mover that caused it: this allocator never evicts, so
 * the cost lands on some unrelated stationary tag whose JOIN is refused
 * outright (zero fixes), which is strictly worse than the smaller allocation
 * the mover would have been fine with. 4 phases is 1.25 Hz at a 200 ms
 * superframe, which the design treats as adequate; 8 is 2.5 Hz, a bonus.
 * Raising this once real occupancy data justifies it is a one-line change.
 *
 * The MECHANISM stays generically capable of any power-of-two n up to
 * GW_CYCLE_C: nothing in alloc_phases()/find_candidate()/claim() is capped at
 * 4, and tests/gw_core still pins n == 8 through
 * gw_core_keepalive_max_for_test() below, so a later raise does not land on
 * untested code. */
#define GW_PHASES_MAX_IDLE  1
#define GW_PHASES_MAX_SLOW  2
#define GW_PHASES_MAX_FAST  4

_Static_assert(GW_CYCLE_C <= 16,
               "struct gw_grant.phase_mask is 16 bits: GW_CYCLE_C cannot exceed 16");
_Static_assert(GW_PHASES_MAX_FAST <= GW_CYCLE_C,
               "a tag cannot hold more phases than the cycle has");

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
    uint8_t  phase;       /* BASE phase of the grant: the lowest phase in
                           * phase_mask, i.e. the `base` of
                           * base, base + C/n, base + 2C/n, ... A one-phase
                           * grant makes this simply "the" phase, which is what
                           * it meant before multi-phase grants existed. */
    uint8_t  slot_index;  /* the ONE CFP slot used in EVERY phase this grant
                           * covers (invariant 1 in the file comment) */
    uint16_t phase_mask;  /* bit i set == this tag holds phase i. Always
                           * gw_phase_count() bits, always evenly spaced.
                           * A separate, deferred task puts exactly this field
                           * on the wire in the GRANT frame; until then it is
                           * for logging and for the beacon slot maps, which
                           * already carry the same information implicitly. */
    uint8_t  tier;
    uint16_t lease;
};

/* Phases in a grant. Kept as a derivation of phase_mask rather than a second
 * struct field, so the two can never disagree. Bounded at GW_CYCLE_C
 * iterations (Kernighan clear-lowest-bit), and deliberately not a compiler
 * popcount builtin: this header is consumed by both the Xtensa build and the
 * plain-gcc host tests. */
static inline uint8_t gw_phase_count(uint16_t phase_mask)
{
    uint8_t n = 0;

    while (phase_mask) {
        phase_mask &= (uint16_t)(phase_mask - 1u);
        n++;
    }
    return n;
}

/* What a KEEPALIVE actually did. The distinction is not cosmetic: a KEEPALIVE
 * arrives every few superframes from every seated tag, so logging each one at
 * LOG_INF would bury the console at any real tag count -- while a re-phase
 * genuinely changes which superframes a tag ranges in and must be visible. It
 * is also the signal the deferred GRANT-phase_mask task needs to decide when
 * to re-GRANT a tag rather than wait for it to re-JOIN. */
enum gw_keepalive_result {
    GW_KEEPALIVE_UNKNOWN = 0,  /* no live seat holds that short address */
    GW_KEEPALIVE_SAME,         /* lease refreshed; phase set unchanged */
    GW_KEEPALIVE_REPHASED,     /* lease refreshed AND the phase set moved */
};

void gw_core_init(struct gw_core_ctx *c);

/* Seat a tag. A tag not already seated is granted exactly ONE phase whatever
 * tier it asked for -- a tag that has done nothing but broadcast a JOIN has
 * not yet proved it will stay, and the phases its tier earns are handed out by
 * gw_core_keepalive() below once it does. A JOIN from an EUI that already
 * holds a seat is a retry (its GRANT was lost): it refreshes the lease and the
 * recorded tier and re-reports the phase set the tag already has, growing and
 * shrinking nothing. Returns false only when no free cell exists anywhere. */
bool gw_core_join(struct gw_core_ctx *c, const uint8_t eui[UWB_FRAME_EUI_LEN],
                  uint8_t req_tier, struct gw_grant *out);

/* Refresh a lease and re-allocate the tag's phases to match `req_tier`. This
 * is where tiering actually happens: growing a mover up to the ceiling its
 * tier allows, and -- just as importantly -- shrinking a tag that has gone
 * still, which FREES its excess cells back to the pool rather than merely
 * forgetting them. `out` may be NULL; it is filled only on SAME/REPHASED. */
enum gw_keepalive_result gw_core_keepalive(struct gw_core_ctx *c,
                                           uint16_t short_addr, uint8_t req_tier,
                                           struct gw_grant *out);

/* TEST SEAM. Identical to gw_core_keepalive() in every respect -- the same
 * ladder, the same ownership predicate, the same non-stranding guarantee --
 * except that the phase ceiling is passed in rather than looked up from the
 * tier. PRODUCTION CODE MUST NOT CALL THIS: gw_core_keepalive() is what makes
 * the GW_PHASES_MAX_* ceilings the single place tier policy lives, and a
 * caller here could hand a tag more phases than its tier allows.
 *
 * It exists because GW_PHASES_MAX_FAST is 4, so no tier's ceiling reaches the
 * ladder's n == 8 rung any more, while the allocator is deliberately still
 * generic in n (see GW_PHASES_MAX_FAST above). Without this seam, lowering
 * that ceiling would have silently retired the only coverage of n == 8 and a
 * later raise back to 8 would be a change to untested code. tests/gw_core is
 * the only caller; grep for it before adding another. */
enum gw_keepalive_result gw_core_keepalive_max_for_test(struct gw_core_ctx *c,
                                                        uint16_t short_addr,
                                                        uint8_t req_tier,
                                                        int max_phases,
                                                        struct gw_grant *out);

/* Free EVERY cell this tag holds, across all phases -- not just the first one
 * found. A multi-phase tag whose other cells survived a release would keep
 * appearing in those phases' beacon slot maps under an address granted to
 * nobody, until each aged out on its own lease. */
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

/* Look up the EUI of whichever seat currently holds `short_addr`. Reporting
 * the FIRST matching cell is still correct now that one tag can hold several:
 * every cell of a tag carries a copy of the same EUI (see claim() in
 * gw_core.c), so which one answers cannot change the result. Returns
 * false (and leaves eui_out untouched) if no live seat holds that address --
 * this can legitimately happen for a POS frame that arrives just after its
 * sender's lease expired (see uwb_gateway.c's dispatch(), which does not
 * gate POS on seat state). Callers must have a fallback for false, not
 * treat it as an error. */
bool gw_core_find_eui(const struct gw_core_ctx *c, uint16_t short_addr,
                      uint8_t eui_out[UWB_FRAME_EUI_LEN]);

#endif /* GW_CORE_H */
