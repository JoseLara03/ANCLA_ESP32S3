#!/usr/bin/env python3
"""
Standalone capacity estimator: how many tags this MAC can track simultaneously
at a given update rate, under EACH of the two ranging modes this project
supports -- TWR (the original CAP/seat MAC) and BLINK/TDoA (the slotted mode,
Task 4B).

This is a PURE-PYTHON REIMPLEMENTATION of the airtime/capacity model in
src/mac_budget.{c,h}, src/blink_sched.{c,h} and the tier-cost constants in
src/gw_core.h. It is NOT wired to those C files and is NOT host-tested
against them -- it is a hand-written copy, in the same risk class CLAUDE.md
already flags for src/cal_math.c, src/uwb_frame_802_15_4z.c and
src/pos_solver.c: if mac_budget.c's formula changes, or GW_MAX_SEATS /
GW_SCHED_WINDOW_SF / the tier table in gw_core.h changes, THIS FILE must be
updated by hand or it will quietly report a stale capacity number. There is
no build-time or test-time check tying the two together.

---- The central asymmetry this tool exists to show --------------------------

TWR (src/gw_core.h): capacity is AIRTIME-SHARED. There are only GW_N_CFP
ranging slots per 200 ms superframe, and every tag -- whatever its rate tier
-- competes for a slot each time it is due. Capacity is counted in
"slot-superframes" over a GW_SCHED_WINDOW_SF-superframe accounting window
(gw_core_tier_cost()/mac_capacity_slot_sf()/mac_demand_slot_sf()): a tag
updating faster occupies a slot more often, so raising the target rate
SHRINKS how many tags fit. This is why "tags @ 1 Hz" and "tags @ 5 Hz" are
different numbers for TWR (5x apart, since 5 Hz is FAST and 1 Hz is SLOW,
i.e. 5x the tier cost per src/gw_core.c's tier_period table).

BLINK/TDoA (src/gw_core.h's Task 4 comment on tx_beacon(), src/blink_sched.c):
every admitted tag gets a PERMANENTLY DEDICATED slot,
blink_sched_slot_index(seat_id) = seat_id (the identity). The gateway does
not re-share that slot with anyone else, at any rate: tx_beacon() in BLINK
mode "stops reading [gw_core's tier/cost] output and writes sched[]
reserved/zero instead" (CLAUDE.md, TDoA migration Task 4). So BLINK capacity
is purely min(GW_MAX_SEATS, BLINK_N_SLOTS), and it is the SAME number
whatever Hz you ask for -- update rate is a tag-side sleep/TX-cadence choice
that never touches slot admission. This tool prints that explicitly rather
than silently reproducing the same integer twice with no explanation.

---- Usage --------------------------------------------------------------

    python tools/tag_density.py                    # today's frozen params, 1 Hz
    python tools/tag_density.py --rate-hz 5
    python tools/tag_density.py --rate-hz 0.2
    python tools/tag_density.py --bitrate 6800000 --plen 256 --pac 8 \\
                                 --n-cfp auto      # explore a future PHY

Every PHY/MAC constant defaults to today's frozen contract (uwb_phy.h,
uwb_mac.h, gw_core.h, blink_sched.h) and can be overridden, in case a future
hardware revision changes the PHY -- see --help.
"""

import argparse
import math
import sys

# ---------------------------------------------------------------------------
# ---- PHY-derived primitives, mirroring src/mac_budget.h's macros ----------
# ---------------------------------------------------------------------------
#
# All arithmetic is done in integer picoseconds, then truncated to nanoseconds
# with a single floor division -- exactly the discipline mac_budget.h uses
# ("Everything is INTEGER arithmetic in picoseconds ... so the macros can
# appear in a BUILD_ASSERT"). Truncating, not rounding, matches
# MAC_PS_TO_NS()'s uint32_t cast of ps/1000.

MAC_PREAMBLE_SYM_PS = 1017630  # src/mac_budget.h MAC_PREAMBLE_SYM_PS (64 MHz PRF)
MAC_UUS_PS = 1025641           # src/mac_budget.h MAC_UUS_PS (512 / 499.2 MHz)
MAC_PHR_BITS = 19              # src/mac_budget.h MAC_PHR_BITS (DWT_PHRRATE_STD)
MAC_FCS_BYTES = 2              # src/mac_budget.h MAC_FCS_BYTES


def mac_bits_ps(bits, bps):
    """mac_budget.h MAC_BITS_PS()."""
    return bits * 1_000_000_000_000 // bps


def mac_shr_ps(plen_sym, sfd_sym):
    """mac_budget.h MAC_SHR_PS(). The RMARKER sits at the END of the SHR."""
    return (plen_sym + sfd_sym) * MAC_PREAMBLE_SYM_PS


def mac_frame_ps(plen_sym, sfd_sym, payload_bytes, bps):
    """mac_budget.h MAC_FRAME_PS(). `payload_bytes` EXCLUDES the FCS."""
    return (
        mac_shr_ps(plen_sym, sfd_sym)
        + mac_bits_ps(MAC_PHR_BITS, bps)
        + mac_bits_ps((payload_bytes + MAC_FCS_BYTES) * 8, bps)
    )


def mac_sstwr_exchange_ps(plen_sym, sfd_sym, resp_bytes, bps, turn_uus):
    """mac_budget.h MAC_SSTWR_EXCHANGE_PS().

    Deliberately NOT poll_frame + turnaround + resp_frame -- see the long
    comment on this macro in src/mac_budget.h and the "SS-TWR exchange"
    hard-won fact in CLAUDE.md: that naive form double-counts BOTH SHRs
    because POLL_RX_TO_RESP_TX_DLY_UUS is measured RMARKER-to-RMARKER and
    each RMARKER already sits at the END of its own SHR. The correct span is

        poll SHR  +  turnaround  +  (response PHR + payload + FCS)

    with the poll's own payload and the response's SHR both falling INSIDE
    the turnaround window. Getting this wrong inflated a 4-anchor slot from
    a real 13.3 ms to a phantom 18.1 ms in an early draft of the design spec
    -- reproducing that bug here would be exactly the kind of silent-drift
    risk this docstring warns about.
    """
    return (
        mac_shr_ps(plen_sym, sfd_sym)
        + turn_uus * MAC_UUS_PS
        + mac_bits_ps(MAC_PHR_BITS, bps)
        + mac_bits_ps((resp_bytes + MAC_FCS_BYTES) * 8, bps)
    )


def ps_to_ns(ps):
    return ps // 1000


def uus_to_ns(uus):
    """mac_budget.h MAC_UUS_TO_NS()."""
    return uus * MAC_UUS_PS // 1000


class MacPhy:
    """Mirrors struct mac_phy (src/mac_budget.h)."""

    def __init__(self, plen_sym, sfd_sym, pac_sym, bitrate_bps):
        self.plen_sym = plen_sym
        self.sfd_sym = sfd_sym
        self.pac_sym = pac_sym
        self.bitrate_bps = bitrate_bps


def mac_shr_ns(phy):
    return ps_to_ns(mac_shr_ps(phy.plen_sym, phy.sfd_sym))


def mac_frame_ns(phy, payload_bytes):
    """Mirrors mac_budget.c mac_frame_ns()."""
    return ps_to_ns(mac_frame_ps(phy.plen_sym, phy.sfd_sym, payload_bytes, phy.bitrate_bps))


def mac_sstwr_exchange_ns(phy, resp_bytes, turnaround_uus):
    """Mirrors mac_budget.c mac_sstwr_exchange_ns()."""
    return ps_to_ns(
        mac_sstwr_exchange_ps(phy.plen_sym, phy.sfd_sym, resp_bytes, phy.bitrate_bps, turnaround_uus)
    )


class MacCell:
    """Mirrors struct mac_cell (src/mac_budget.h): the superframe overhead a
    cell pays before any ranging or blink slot fits.

        [BEACON][g][ CAP: n_cap minislots ][g][ CFP: slots ][g]
    """

    def __init__(self, superframe_ns, beacon_ns, guard_ns, minislot_ns, n_cap):
        self.superframe_ns = superframe_ns
        self.beacon_ns = beacon_ns
        self.guard_ns = guard_ns
        self.minislot_ns = minislot_ns
        self.n_cap = n_cap


def mac_cell_usable_ns(cell):
    """Mirrors mac_budget.c mac_cell_usable_ns(). Three guards are charged:
    after the beacon, before the CAP, and after the CFP."""
    overhead = cell.beacon_ns + 3 * cell.guard_ns + cell.n_cap * cell.minislot_ns
    if overhead >= cell.superframe_ns:
        return 0
    return cell.superframe_ns - overhead


def mac_cell_max_slots(cell, slot_ns):
    """Mirrors mac_budget.c mac_cell_max_slots(). Truncating division is
    deliberate: a partial slot is not a slot."""
    if slot_ns == 0:
        return 0
    return mac_cell_usable_ns(cell) // slot_ns


def mac_capacity_slot_sf(n_slots, window_sf):
    """Mirrors mac_budget.c mac_capacity_slot_sf()."""
    return n_slots * window_sf


def mac_demand_slot_sf(n_tags, rate_div, window_sf):
    """Mirrors mac_budget.c mac_demand_slot_sf(). Participations one tag
    makes in the window, rounded UP."""
    if rate_div == 0:
        rate_div = 1
    per_tag = (window_sf + rate_div - 1) // rate_div
    return n_tags * per_tag


# ---------------------------------------------------------------------------
# ---- BLINK slot model, mirroring src/blink_sched.{c,h} --------------------
# ---------------------------------------------------------------------------

def blink_sched_slot_ns(phy, blink_frame_len, blink_slot_guard_uus):
    """Mirrors blink_sched.c blink_sched_slot_ns()."""
    return mac_frame_ns(phy, blink_frame_len) + uus_to_ns(blink_slot_guard_uus)


def blink_sched_n_slots(cell, phy, blink_frame_len, blink_slot_guard_uus):
    """Mirrors blink_sched.c blink_sched_n_slots(): a thin wrapper over
    mac_cell_max_slots(cell, blink_sched_slot_ns())."""
    return mac_cell_max_slots(cell, blink_sched_slot_ns(phy, blink_frame_len, blink_slot_guard_uus))


# ---------------------------------------------------------------------------
# ---- Defaults: today's frozen contract -------------------------------------
# ---------------------------------------------------------------------------
# uwb_phy.h: channel 5, PLEN_1024, PAC32, code 9, 850 kbps, SFD_IEEE_4Z,
# STS/PDoA off.
DEFAULT_PLEN_SYM = 1024
DEFAULT_SFD_SYM = 8          # 4z 8-symbol SFD
DEFAULT_PAC_SYM = 32
DEFAULT_BITRATE_BPS = 850_000
DEFAULT_CHANNEL = 5
DEFAULT_PREAMBLE_CODE = 9

# src/uwb_mac.h
DEFAULT_T_SUPERFRAME_UUS = 195_000     # 200.0 ms
DEFAULT_T_GUARD_UUS = 488              # 0.5 ms superframe-partition guard,
                                        # charged three times (mac_cell_usable_ns)
DEFAULT_BEACON_OCCUPANCY_UUS = 1500

# src/uwb_frame_802_15_4z.h
DEFAULT_N_CFP = 11                     # UWB_FRAME_N_CFP, a frozen WIRE literal
DEFAULT_N_CAP = 4                      # UWB_FRAME_N_CAP
DEFAULT_LEN_KEEPALIVE = 12             # UWB_FRAME_LEN_KEEPALIVE
DEFAULT_LEN_RESP = 20                  # UWB_FRAME_LEN_RESP
DEFAULT_MINISLOT_OVERHEAD_NS = 100_000  # the "+100000u" in UWB_MAC_CFP_USABLE_NS

# anchor_respond.c / mac_budget test suite
DEFAULT_TURNAROUND_UUS = 2000          # POLL_RX_TO_RESP_TX_DLY_UUS
DEFAULT_N_ANCHORS = 4                  # UWB_FRAME_MAX_ANCHORS: one CFP slot must
                                        # hold a full sweep against every anchor

# src/blink_frame.h
DEFAULT_BLINK_FRAME_LEN = 14           # BLINK_FRAME_LEN

# src/blink_sched.h
DEFAULT_BLINK_SLOT_GUARD_UUS = 100     # BLINK_SLOT_GUARD_UUS (measured, Task 2)

# src/gw_core.h
DEFAULT_GW_MAX_SEATS = 128
DEFAULT_GW_SCHED_WINDOW_SF = 25        # = the IDLE tier's period


def beacon_len_bytes(n_cfp):
    """UWB_FRAME_LEN_BEACON = 15 + 2 * UWB_FRAME_N_CFP."""
    return 15 + 2 * n_cfp


def build_cell(phy, args, n_cfp):
    beacon_bytes = beacon_len_bytes(n_cfp)
    return MacCell(
        superframe_ns=uus_to_ns(args.superframe_uus),
        beacon_ns=mac_frame_ns(phy, beacon_bytes),
        guard_ns=uus_to_ns(args.guard_uus),
        minislot_ns=mac_frame_ns(phy, args.keepalive_bytes) + args.minislot_overhead_ns,
        n_cap=args.n_cap,
    )


def superframe_hz(superframe_uus):
    superframe_ns = uus_to_ns(superframe_uus)
    return 1_000_000_000.0 / superframe_ns


def rate_div_for(rate_hz, superframe_uus, window_sf):
    """Superframes-per-participation for an arbitrary target rate, generalising
    gw_core.h's fixed FAST=1/SLOW=5/IDLE=25 table to any --rate-hz. Rounded to
    the nearest whole superframe count (a tag can only be scheduled on whole
    superframe boundaries) and clamped to >= 1."""
    sf_hz = superframe_hz(superframe_uus)
    if rate_hz <= 0:
        raise ValueError("--rate-hz must be > 0")
    div = round(sf_hz / rate_hz)
    return max(1, min(div, window_sf))


def twr_capacity(args, phy):
    """TWR (CAP/seat MAC): airtime-shared. Capacity in slot-superframes over
    GW_SCHED_WINDOW_SF, per src/gw_core.h's tier-cost accounting
    (gw_core_tier_cost() / mac_capacity_slot_sf() / mac_demand_slot_sf()).

    n_cfp is either the shipped wire literal (UWB_FRAME_N_CFP, default 11) or,
    with --n-cfp auto, the number of ranging slots the PHY/airtime budget
    would actually afford for the given PHY (UWB_MAC_CFP_SLOTS_FEASIBLE) --
    useful for exploring a future PHY, since the shipped literal cannot move
    on its own (it is frozen by proto_ver, see uwb_frame_802_15_4z.h).
    """
    if args.n_cfp == "auto":
        cell_for_feasibility = build_cell(phy, args, DEFAULT_N_CFP)
        slot_ns = args.n_anchors * mac_sstwr_exchange_ns(phy, args.resp_bytes, args.turnaround_uus)
        n_cfp = mac_cell_max_slots(cell_for_feasibility, slot_ns)
        feasible_note = (
            f"(auto: {n_cfp} CFP slots fit the airtime budget at this PHY; "
            f"the shipped wire literal is {DEFAULT_N_CFP})"
        )
    else:
        n_cfp = int(args.n_cfp)
        feasible_note = None

    capacity_sf = mac_capacity_slot_sf(n_cfp, args.window_sf)
    rate_div = rate_div_for(args.rate_hz, args.superframe_uus, args.window_sf)
    per_tag_sf = math.ceil(args.window_sf / rate_div)
    max_tags = capacity_sf // per_tag_sf

    return {
        "n_cfp": n_cfp,
        "feasible_note": feasible_note,
        "capacity_slot_sf": capacity_sf,
        "rate_div": rate_div,
        "per_tag_slot_sf": per_tag_sf,
        "max_tags": max_tags,
    }


def blink_capacity(args, phy):
    """BLINK/TDoA: dedicated-slot MAC. Capacity is min(GW_MAX_SEATS,
    BLINK_N_SLOTS), independent of rate -- see the module docstring."""
    cell = build_cell(phy, args, args.n_cfp_for_cell)
    n_slots = blink_sched_n_slots(cell, phy, args.blink_frame_len, args.blink_slot_guard_uus)
    max_tags = min(args.max_seats, n_slots)
    return {
        "blink_n_slots": n_slots,
        "max_seats": args.max_seats,
        "max_tags": max_tags,
        "bound": "GW_MAX_SEATS" if args.max_seats <= n_slots else "BLINK_N_SLOTS",
    }


def parse_args(argv):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )

    p.add_argument("--rate-hz", type=float, default=1.0,
                    help="target per-tag update rate, Hz (default: 1.0)")

    phy = p.add_argument_group("PHY (defaults: today's frozen uwb_phy.h contract)")
    phy.add_argument("--plen", type=int, default=DEFAULT_PLEN_SYM, dest="plen_sym",
                      help="TX preamble length, symbols (default: %(default)s, DWT_PLEN_1024)")
    phy.add_argument("--sfd", type=int, default=DEFAULT_SFD_SYM, dest="sfd_sym",
                      help="SFD length, symbols (default: %(default)s, 4z 8-symbol SFD)")
    phy.add_argument("--pac", type=int, default=DEFAULT_PAC_SYM, dest="pac_sym",
                      help="RX preamble acquisition chunk, symbols (default: %(default)s, DWT_PAC32)")
    phy.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE_BPS, dest="bitrate_bps",
                      help="data rate, bps (default: %(default)s = 850k; use 6800000 for 6.8M)")
    phy.add_argument("--channel", type=int, default=DEFAULT_CHANNEL,
                      help="UWB channel, cosmetic only -- not an input to the airtime "
                           "model (default: %(default)s)")
    phy.add_argument("--preamble-code", type=int, default=DEFAULT_PREAMBLE_CODE,
                      help="preamble code, cosmetic only -- not an input to the airtime "
                           "model (default: %(default)s)")

    mac = p.add_argument_group("MAC / superframe (defaults: today's uwb_mac.h, gw_core.h)")
    mac.add_argument("--superframe-uus", type=int, default=DEFAULT_T_SUPERFRAME_UUS,
                      help="T_SUPERFRAME_UUS (default: %(default)s = 200.0 ms)")
    mac.add_argument("--guard-uus", type=int, default=DEFAULT_T_GUARD_UUS,
                      help="T_GUARD_UUS, charged 3x per superframe (default: %(default)s)")
    mac.add_argument("--n-cfp", default=str(DEFAULT_N_CFP), dest="n_cfp",
                      help="TWR ranging slots per superframe (GW_N_CFP / UWB_FRAME_N_CFP). "
                           "Pass 'auto' to derive it from the airtime budget instead of the "
                           "shipped wire literal (default: %(default)s)")
    mac.add_argument("--n-cap", type=int, default=DEFAULT_N_CAP,
                      help="CAP Aloha minislots per superframe (default: %(default)s)")
    mac.add_argument("--keepalive-bytes", type=int, default=DEFAULT_LEN_KEEPALIVE,
                      help="UWB_FRAME_LEN_KEEPALIVE, sizes the CAP minislot (default: %(default)s)")
    mac.add_argument("--minislot-overhead-ns", type=int, default=DEFAULT_MINISLOT_OVERHEAD_NS,
                      help="fixed per-CAP-minislot overhead beyond frame airtime (default: %(default)s)")
    mac.add_argument("--resp-bytes", type=int, default=DEFAULT_LEN_RESP,
                      help="UWB_FRAME_LEN_RESP, the SS-TWR response payload (default: %(default)s)")
    mac.add_argument("--turnaround-uus", type=int, default=DEFAULT_TURNAROUND_UUS,
                      help="POLL_RX_TO_RESP_TX_DLY_UUS (default: %(default)s)")
    mac.add_argument("--n-anchors", type=int, default=DEFAULT_N_ANCHORS,
                      help="anchors ranged per TWR CFP slot / --n-cfp=auto sizing "
                           "(default: %(default)s, UWB_FRAME_MAX_ANCHORS)")
    mac.add_argument("--window-sf", type=int, default=DEFAULT_GW_SCHED_WINDOW_SF,
                      dest="window_sf",
                      help="GW_SCHED_WINDOW_SF, the rate-accounting window (default: %(default)s)")

    blink = p.add_argument_group("BLINK/TDoA (defaults: today's blink_sched.h, gw_core.h)")
    blink.add_argument("--blink-frame-len", type=int, default=DEFAULT_BLINK_FRAME_LEN,
                        help="BLINK_FRAME_LEN (default: %(default)s)")
    blink.add_argument("--blink-slot-guard-uus", type=int, default=DEFAULT_BLINK_SLOT_GUARD_UUS,
                        help="BLINK_SLOT_GUARD_UUS (default: %(default)s, measured Task 2 "
                             "2026-09-01: gateway TX-arm jitter 64us + tag crystal drift 8us, "
                             "rounded up)")
    blink.add_argument("--max-seats", type=int, default=DEFAULT_GW_MAX_SEATS, dest="max_seats",
                        help="GW_MAX_SEATS, the seat table's fixed array size (default: %(default)s)")

    args = p.parse_args(argv)

    if args.n_cfp != "auto":
        try:
            int(args.n_cfp)
        except ValueError:
            p.error("--n-cfp must be an integer or 'auto'")

    # BLINK mode's cell shares the same beacon/CAP overhead as TWR mode: only
    # the CFP is repartitioned into blink slots (design doc section 1.1,
    # "the CFP is repartitioned rather than extended"). Beacon length depends
    # on n_cfp (UWB_FRAME_LEN_BEACON = 15 + 2*n_cfp), which is a TWR-mode
    # concept -- BLINK mode still transmits the same wire-format beacon, just
    # with sched[] reserved/zero, so use the shipped literal for sizing it
    # rather than an --n-cfp=auto exploration value.
    args.n_cfp_for_cell = DEFAULT_N_CFP if args.n_cfp == "auto" else int(args.n_cfp)

    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)

    phy = MacPhy(args.plen_sym, args.sfd_sym, args.pac_sym, args.bitrate_bps)

    twr = twr_capacity(args, phy)
    blink = blink_capacity(args, phy)

    print(f"tag_density: capacity estimate at {args.rate_hz:g} Hz update rate")
    print(f"  PHY: PLEN={phy.plen_sym} SFD={phy.sfd_sym} PAC={phy.pac_sym} "
          f"bitrate={phy.bitrate_bps} bps  (ch{args.channel}, code {args.preamble_code}, "
          f"cosmetic-only for airtime)")
    print(f"  superframe: {args.superframe_uus} UUS = {uus_to_ns(args.superframe_uus)/1e6:.3f} ms")
    print()

    print("---- TWR (CAP/seat MAC, src/gw_core.h) ----")
    print(f"  GW_N_CFP (ranging slots/superframe): {twr['n_cfp']}"
          + (f"  {twr['feasible_note']}" if twr['feasible_note'] else ""))
    print(f"  GW_SCHED_WINDOW_SF: {args.window_sf}")
    print(f"  capacity = n_cfp * window_sf = {twr['n_cfp']} * {args.window_sf} "
          f"= {twr['capacity_slot_sf']} slot-superframes")
    print(f"  at {args.rate_hz:g} Hz: rate_div (superframes/participation) = {twr['rate_div']}, "
          f"cost/tag = ceil(window_sf/rate_div) = {twr['per_tag_slot_sf']} slot-superframes")
    print(f"  => max tags @ {args.rate_hz:g} Hz = {twr['capacity_slot_sf']} / {twr['per_tag_slot_sf']} "
          f"= {twr['max_tags']}")
    print()

    print("---- BLINK/TDoA (dedicated-slot MAC, src/blink_sched.h + gw_core.h Task 4) ----")
    print(f"  BLINK_N_SLOTS (airtime-derived): {blink['blink_n_slots']}")
    print(f"  GW_MAX_SEATS (seat table size):  {blink['max_seats']}")
    print(f"  => max tags @ ANY rate = min(GW_MAX_SEATS, BLINK_N_SLOTS) "
          f"= min({blink['max_seats']}, {blink['blink_n_slots']}) = {blink['max_tags']}"
          f"  [bound: {blink['bound']}]")
    print()

    print("---- Why these differ ----")
    print("  TWR shares GW_N_CFP airtime slots across every tag due for a ranging")
    print("  exchange this superframe; a tag polled more often (higher Hz) occupies a")
    print("  slot more often, so raising the rate SHRINKS how many tags fit at once.")
    print("  BLINK gives every admitted tag its OWN permanent slot")
    print("  (blink_sched_slot_index(seat_id) == seat_id, an identity mapping) and the")
    print("  gateway's tier/cost accounting is bypassed entirely in BLINK mode -- so its")
    print("  capacity is a pure seat-table/airtime bound with NO Hz term at all.")
    if abs(args.rate_hz - 5.0) > 1e-9:
        twr_at_5 = twr_capacity(
            argparse.Namespace(**{**vars(args), "rate_hz": 5.0}), phy
        )
        print(f"  (check: at 5 Hz instead of {args.rate_hz:g} Hz, TWR's number moves to "
              f"{twr_at_5['max_tags']}; BLINK's stays {blink['max_tags']}.)")
    print()

    print("---- Sanity check against known figures (default params only) ----")
    is_default = (
        args.plen_sym == DEFAULT_PLEN_SYM and args.sfd_sym == DEFAULT_SFD_SYM
        and args.pac_sym == DEFAULT_PAC_SYM and args.bitrate_bps == DEFAULT_BITRATE_BPS
        and args.n_cfp == str(DEFAULT_N_CFP) and args.window_sf == DEFAULT_GW_SCHED_WINDOW_SF
        and args.max_seats == DEFAULT_GW_MAX_SEATS
        and args.blink_slot_guard_uus == DEFAULT_BLINK_SLOT_GUARD_UUS
    )
    if is_default:
        ok_blink = blink["blink_n_slots"] >= 140 and blink["blink_n_slots"] <= 148
        ok_blink_cap = blink["max_tags"] == args.max_seats
        print(f"  BLINK_N_SLOTS in [140,148] (tests/blink_sched/ pins ~144): "
              f"{blink['blink_n_slots']}  {'OK' if ok_blink else 'MISMATCH'}")
        print(f"  BLINK capacity == GW_MAX_SEATS (128, since 144 > 128): "
              f"{blink['max_tags']}  {'OK' if ok_blink_cap else 'MISMATCH'}")
        if abs(args.rate_hz - 1.0) < 1e-9:
            ok_twr = twr["max_tags"] == 55
            print(f"  TWR @ 1 Hz == 275/5 == 55 (hand check from gw_core.h): "
                  f"{twr['max_tags']}  {'OK' if ok_twr else 'MISMATCH'}")
    else:
        print("  (skipped: non-default parameters given)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
