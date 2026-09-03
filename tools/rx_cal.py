#!/usr/bin/env python3
"""Fit per-anchor RX antenna-delay offsets from a raw BLINK-observation capture.

WHY THIS EXISTS
---------------
SS-TWR -- which is what `cal ref` and the anchor survey both use -- can only
ever observe the SUM ant_tx + ant_rx. It constrains the TX/RX SPLIT not at
all. TDoA is the opposite: every anchor only ever RECEIVES, nothing cancels,
and the observable carries the DIFFERENCE in RX-delay error between anchors
at 4.69 mm per DTU. So a fleet that passed `cal ref` and a `cal peer`
cross-check can still be metres wrong in TDoA, and no TWR measurement of any
kind can detect it. See CLAUDE.md's 2026-08-28 laser-campaign entry, which
records exactly this gap and measured ~207 units of per-board spread in the
quantity TWR *can* see.

THE OBSERVABLE
--------------
The DW3220 SUBTRACTS the configured ant_rx from every RX timestamp, so an
anchor k reports

    tau_k = T_emit + r_k / c - delta_k + noise

where delta_k is its RX delay in DTU. T_emit -- and therefore the
transmitting tag's own TX delay -- is common to every receiver of one blink,
so it cancels in a range difference:

    (tau_k - tau_0) * MM_PER_DTU - (r_k - r_0) = -(delta_k - delta_0) * MM_PER_DTU

The left side is measurable from the capture plus the surveyed geometry and a
tape-measured tag position. The right side is a CONSTANT -- independent of
where the tag is. That is what makes this fit separable per anchor (a plain
mean, no matrix) and, more importantly, what makes it FALSIFIABLE: estimate
b_k independently at several tag positions and the estimates must AGREE. If
they drift with position, the error is geometry (or tag height), not RX
delay, and applying this fit would bake a bias in rather than remove one.

Only DIFFERENCES are observable, which is not a limitation: a common RX
offset shifts every anchor's timestamp equally and is invisible to TDoA. The
reference anchor (lowest id present, matching tdoa_collect's own choice) is
therefore pinned as the gauge and its delays are left alone.

APPLYING THE RESULT
-------------------
Move the SPLIT, hold the SUM:

    ant_rx += b_k      ant_tx -= b_k

SS-TWR observes only the sum (CLAUDE.md's antenna-delay derivation: both TX
and RX are half a tick of range per unit on that path), so TWR ranging, the
`apos` survey and the existing `cal ref` calibration all come out unchanged.
Only the TDoA-visible split moves. This is why the campaign is safe to run on
an already-calibrated fleet.

INPUT
-----
A capture of the raw observation topic, e.g.

    mosquitto_sub -h HOST -t "uwb/anchor/blink/#" -v > cap.txt

Both bare-JSON lines and mosquitto_sub -v lines ("topic {json}") are
accepted, as are gateway console lines with the JSON embedded.
"""

import argparse
import json
import math
import re
import sys
from collections import defaultdict

# One DW3220 device time unit of PATH length. This is the full 4.69 mm, not
# the 2.35 mm/unit that CLAUDE.md's SS-TWR entry quotes: there, half of each
# unit lands in the round-trip average, whereas a TDoA timestamp is one-way
# and the whole unit shows up in the range difference.
MM_PER_DTU = 4.6917214

DTU_WRAP = 1 << 40

_JSON = re.compile(r'\{[^{}]*"a"\s*:[^{}]*\}')


def load(paths):
    """Yield observation dicts from one or more capture files."""
    for path in paths:
        with open(path, "r", errors="replace") as fh:
            for line in fh:
                for m in _JSON.finditer(line):
                    try:
                        o = json.loads(m.group(0))
                    except ValueError:
                        continue
                    if "a" in o and "t" in o and "s" in o and "ts" in o:
                        yield o


def sdelta40(a, b):
    """Signed 40-bit difference, same discipline as the firmware's sdelta40().

    Raw t_dtu wraps every ~17.2 s, and two anchors' stamps of the SAME blink
    can straddle that boundary. Without this the difference reads ~2^40 DTU
    (about 5.16 million km of path) and the group is silently garbage.
    """
    d = (int(a) - int(b)) % DTU_WRAP
    if d >= DTU_WRAP // 2:
        d -= DTU_WRAP
    return d


def group(obs, min_anchors):
    """Group observations by (tag, blink_seq).

    blink_seq wraps at 256, but the firmware's own note applies here too: a
    group closes far faster than a seq value can recur. Guarded anyway by
    rejecting a group whose internal spread is implausible (see fit_spot).
    """
    groups = defaultdict(dict)
    for o in obs:
        # A duplicate report from the same anchor for the same blink is an
        # MQTT redelivery; keep the first, exactly as tdoa_collect rejects
        # the second rather than folding it in twice.
        groups[(int(o["t"]), int(o["s"]))].setdefault(int(o["a"]), o)
    return [(k, v) for k, v in groups.items() if len(v) >= min_anchors]


def parse_anchors(specs):
    out = {}
    for s in specs:
        m = re.match(r"^\s*(\d+)\s*:\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*$", s)
        if not m:
            sys.exit("bad --anchor %r; want id:x,y,z" % s)
        out[int(m.group(1))] = (float(m.group(2)), float(m.group(3)),
                                float(m.group(4)))
    return out


def anchors_from_log(path):
    """Pull the surveyed geometry out of a capture's own apos_peer lines.

    Preferred over typing coordinates by hand: it cannot disagree with the
    survey the gateway was actually solving against, which is the exact class
    of mistake the anchor-auto-positioning branch exists to remove.
    """
    out = {}
    pat = re.compile(r'"apos_peer"\s*:\s*(\{.*?\})\s*\}')
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            m = pat.search(line)
            if not m:
                continue
            try:
                d = json.loads(m.group(1))
            except ValueError:
                continue
            if "id" in d and d.get("pos_valid"):
                out[int(d["id"])] = (float(d["x"]), float(d["y"]),
                                     float(d.get("z", 0.0)))
    return out


def stats(v):
    n = len(v)
    mean = sum(v) / n
    sd = math.sqrt(sum((x - mean) ** 2 for x in v) / n) if n > 1 else 0.0
    s = sorted(v)
    return n, mean, sd, s[n // 2]


def fit_spot(gs, anchors, tag, tag_z, ref_id, max_spread_dtu):
    """Per-anchor b_k estimates (DTU) for one tag position."""
    per = defaultdict(list)
    dropped = 0

    def rng(aid):
        ax, ay, az = anchors[aid]
        dz = az - tag_z
        return math.sqrt((tag[0] - ax) ** 2 + (tag[1] - ay) ** 2 + dz * dz)

    for (_tag_addr, _seq), by_anchor in gs:
        if ref_id not in by_anchor:
            continue
        if any(a not in anchors for a in by_anchor):
            continue

        t0 = int(by_anchor[ref_id]["ts"])
        deltas = {a: sdelta40(o["ts"], t0) for a, o in by_anchor.items()}

        # A group whose stamps span more than a plausible path difference is
        # not one blink -- a (tag, seq) collision, or a stamp from an anchor
        # whose clock model had just re-baselined. Same role as the
        # firmware's tdoa_dtu_plausible().
        if max(abs(d) for d in deltas.values()) > max_spread_dtu:
            dropped += 1
            continue

        r0 = rng(ref_id)
        for a, d in deltas.items():
            if a == ref_id:
                continue
            resid_mm = d * MM_PER_DTU - (rng(a) - r0) * 1000.0
            per[a].append(-resid_mm / MM_PER_DTU)

    return per, dropped


def main():
    ap = argparse.ArgumentParser(
        description="Fit per-anchor RX antenna-delay offsets from BLINK captures.")
    ap.add_argument("--spot", action="append", required=True, metavar="X,Y:FILE",
                    help="a tape-measured tag position and its capture file. "
                         "Repeat -- at least 3 spots, or the fit cannot be "
                         "checked for consistency and is not evidence.")
    ap.add_argument("--anchor", action="append", default=[], metavar="ID:X,Y,Z",
                    help="anchor coordinates in metres (survey frame).")
    ap.add_argument("--anchors-from", metavar="FILE",
                    help="instead of --anchor, read the geometry from a "
                         "gateway log's own apos_peer lines.")
    ap.add_argument("--tag-z", type=float, default=0.0,
                    help="tag plane z in the survey frame, NEGATIVE when tags "
                         "sit below ceiling anchors. Same quantity as the "
                         "firmware's `apos tagz`. Default 0.0.")
    ap.add_argument("--ref", type=int, default=None,
                    help="reference anchor id (gauge). Default: lowest id "
                         "present, matching tdoa_collect's own choice.")
    ap.add_argument("--min-anchors", type=int, default=3)
    ap.add_argument("--max-spread-m", type=float, default=153.7,
                    help="reject a group whose internal path spread exceeds "
                         "this. Default matches TDOA_DTU_MAX_SPREAD.")
    ap.add_argument("--current", action="append", default=[],
                    metavar="ID:TX,RX",
                    help="each anchor's live ant_tx,ant_rx (from `anchor "
                         "show`). Given, the report prints the exact `anchor "
                         "ant` command to run.")
    args = ap.parse_args()

    spots = []
    for s in args.spot:
        m = re.match(r"^\s*([-\d.]+)\s*,\s*([-\d.]+)\s*:\s*(.+?)\s*$", s)
        if not m:
            sys.exit("bad --spot %r; want X,Y:FILE" % s)
        spots.append(((float(m.group(1)), float(m.group(2))), m.group(3)))

    anchors = parse_anchors(args.anchor)
    if args.anchors_from:
        anchors.update(anchors_from_log(args.anchors_from))
    if not anchors:
        for _p, f in spots:
            anchors.update(anchors_from_log(f))
    if len(anchors) < 3:
        sys.exit("need at least 3 anchor positions; got %d. Pass --anchor or "
                 "--anchors-from." % len(anchors))

    ref = args.ref if args.ref is not None else min(anchors)
    max_spread_dtu = args.max_spread_m * 1000.0 / MM_PER_DTU

    cur = {}
    for s in args.current:
        m = re.match(r"^\s*(\d+)\s*:\s*(\d+)\s*,\s*(\d+)\s*$", s)
        if not m:
            sys.exit("bad --current %r; want ID:TX,RX" % s)
        cur[int(m.group(1))] = (int(m.group(2)), int(m.group(3)))

    print("anchors (survey frame, metres):")
    for a in sorted(anchors):
        mark = "   <- reference (gauge, delays untouched)" if a == ref else ""
        print("  id %d  (%.3f, %.3f, %.3f)%s" % ((a,) + anchors[a] + (mark,)))
    print("tag plane z: %.3f m" % args.tag_z)
    print()

    per_spot = {}
    for pos, path in spots:
        gs = group(load([path]), args.min_anchors)
        if not gs:
            print("SPOT (%.3f, %.3f): no usable groups in %s" % (pos + (path,)))
            continue
        per, dropped = fit_spot(gs, anchors, pos, args.tag_z, ref,
                                max_spread_dtu)
        per_spot[pos] = per
        print("SPOT (%.3f, %.3f)  %s" % (pos + (path,)))
        print("  %d groups (>=%d anchors), %d dropped for implausible spread"
              % (len(gs), args.min_anchors, dropped))
        for a in sorted(per):
            n, mean, sd, med = stats(per[a])
            print("    anchor %d: b = %+8.1f DTU (%+8.1f mm)  sd %6.1f DTU  "
                  "median %+8.1f  n=%d"
                  % (a, mean, mean * MM_PER_DTU, sd, med, n))
        print()

    if len(per_spot) < 2:
        print("Only %d spot(s) fitted. The whole point of several spots is "
              "that a genuine RX-delay offset is IDENTICAL at every one -- "
              "with fewer than 2 there is nothing to check the fit against, "
              "so this is not yet evidence. Do not apply it."
              % len(per_spot))
        return 1

    print("=" * 70)
    print("CONSISTENCY -- an RX-delay offset must be the SAME at every spot.")
    print("Spread across spots is the falsification test: a spread comparable")
    print("to the offset itself means the residual is geometry or tag height,")
    print("not RX delay, and applying this fit would ADD a bias.")
    print("=" * 70)

    all_ids = set()
    for p in per_spot.values():
        all_ids |= set(p)

    combined = {}
    ok = True
    for a in sorted(all_ids):
        means = [sum(p[a]) / len(p[a]) for p in per_spot.values() if a in p]
        if len(means) < 2:
            print("  anchor %d: only %d spot(s) reported it -- skipped, "
                  "nothing to cross-check." % (a, len(means)))
            ok = False
            continue
        m = sum(means) / len(means)
        spread = max(means) - min(means)
        combined[a] = m
        good = spread < 0.5 * max(abs(m), 1.0)
        if not good:
            ok = False
        print("  anchor %d: mean %+8.1f DTU (%+7.1f mm)  spread across spots "
              "%6.1f DTU (%6.1f mm)  -> %s"
              % (a, m, m * MM_PER_DTU, spread, spread * MM_PER_DTU,
                 "consistent" if good else "INCONSISTENT"))

    print()
    if not ok:
        print("At least one anchor is INCONSISTENT across spots. Do NOT apply.")
        print("Check, in this order: the tape-measured spot coordinates, the")
        print("survey geometry (`apos show` against a tape), and --tag-z.")
        print("A wrong tag height in particular biases every spot by a")
        print("DIFFERENT amount, because sqrt(rho^2 + dz^2) is nonlinear -- so")
        print("it shows up here as exactly this kind of spread.")
        return 1

    print("APPLY -- move the SPLIT, hold the SUM, so SS-TWR/apos/`cal ref`")
    print("stay exactly as calibrated. Reference anchor %d is the gauge and" % ref)
    print("is deliberately left alone (a common RX offset is invisible to TDoA).")
    print()
    for a in sorted(combined):
        d = int(round(combined[a]))
        if a in cur:
            tx, rx = cur[a]
            print("  anchor id %d:  anchor ant %d %d      # tx %d%+d, rx %d%+d"
                  % (a, tx - d, rx + d, tx, -d, rx, d))
        else:
            print("  anchor id %d:  ant_rx %+d, ant_tx %+d   (pass "
                  "--current %d:TX,RX for the exact command)"
                  % (a, d, -d, a))
    print()
    print("Then `kernel reboot cold` on each -- uwb_radio.c applies the delays")
    print("at boot only. Re-run a capture afterwards: the refitted b should")
    print("collapse toward zero. That re-run is the acceptance test; this")
    print("report is not.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
