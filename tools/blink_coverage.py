#!/usr/bin/env python3
"""Per-anchor BLINK reception coverage from a raw observation capture.

The range instrument for the TDoA receive path -- tag -> anchor, which is the
UNAMPLIFIED direction (this board has a PA and no LNA) and therefore the one
that sets how far TDoA works. See docs/range-test.md.

Reading `blink stats` on three consoles in a field is impractical; the broker
sees every anchor at once and each observation carries the anchor id:

    mosquitto_sub -h HOST -t "uwb/anchor/blink/#" -v > range_30m.txt
    python tools/blink_coverage.py range_30m.txt

WHAT THIS MEASURES, AND WHAT IT DOES NOT
----------------------------------------
Coverage is computed against the UNION of blinks any anchor reported, since
nothing in the capture knows how many the tag actually emitted. So an anchor
at 100 % coverage means "heard everything anybody heard", not "heard
everything sent" -- if every anchor misses the same blink it is invisible
here. `blink_seq` is a per-tag counter, so a gap in it DOES bound the total:
the fleet-wide miss rate is reported separately from that.

An anchor that HEARS a blink but has lost CCP sync discards it and publishes
nothing (blink_rx.c). This capture therefore measures `stamped`, not `rx`. If
coverage collapses at distance, read `blink stats` on that anchor once:
`no_sync == rx` means the CCP link died rather than the blink link, and the
two need opposite fixes.
"""

import argparse
import json
import re
import sys
from collections import defaultdict

_JSON = re.compile(r'\{[^{}]*"a"\s*:[^{}]*\}')


def load(paths):
    for path in paths:
        with open(path, "r", errors="replace") as fh:
            for line in fh:
                for m in _JSON.finditer(line):
                    try:
                        o = json.loads(m.group(0))
                    except ValueError:
                        continue
                    if "a" in o and "t" in o and "s" in o:
                        yield o


class SeqUnwrap:
    """Per-tag blink_seq unwrapper.

    blink_seq is one byte, so a capture longer than 256 blinks -- 5 minutes at
    2 Hz is ~600 -- reuses every value. Keying groups on the raw seq would
    then silently MERGE blinks that are minutes apart, under-reporting the
    distinct-blink count and inflating every coverage figure computed from it.
    Found while validating this tool against synthetic data, where 300 blinks
    over 256 values collapsed to 249.

    Standard unwrap against a running maximum: a step is taken as forward
    while it is under half the range and backward otherwise, which is exactly
    right for observations of the SAME blink arriving interleaved from
    different anchors (a few counts either side of the max) and for the
    ordinary forward progression. It cannot represent a genuine silence of
    more than 128 blinks -- 64 s at 2 Hz -- which reads as a backward step
    instead; that undercounts emissions and therefore only ever makes the
    fleet-coverage figure look BETTER than it is. Stated rather than hidden,
    because a range test is exactly where long silences happen.
    """

    def __init__(self):
        self.hi = None

    def __call__(self, s):
        if self.hi is None:
            self.hi = s
            return s
        d = (s - (self.hi % 256)) % 256
        if d > 128:
            d -= 256
        v = self.hi + d
        if v > self.hi:
            self.hi = v
        return v


def main():
    ap = argparse.ArgumentParser(
        description="Per-anchor BLINK reception coverage from an MQTT capture.")
    ap.add_argument("capture", nargs="+")
    ap.add_argument("--tag", type=lambda v: int(v, 0), default=None,
                    help="restrict to one tag short address (e.g. 0x0100). "
                         "Default: report each tag separately.")
    args = ap.parse_args()

    # (tag, seq) -> set of anchor ids that reported it
    seen = defaultdict(set)
    quality = defaultdict(list)
    unwrap = defaultdict(SeqUnwrap)

    for o in load(args.capture):
        tag, aid = int(o["t"]), int(o["a"])
        if args.tag is not None and tag != args.tag:
            continue
        seq = unwrap[tag](int(o["s"]))
        seen[(tag, seq)].add(aid)
        if "q" in o:
            quality[(tag, aid)].append(int(o["q"]))

    tags = sorted({t for t, _ in seen})
    if not tags:
        print("no observations found")
        return 1

    for tag in tags:
        blinks = [(s, a) for (t, s), a in seen.items() if t == tag]
        total = len(blinks)
        # The unwrapped seq span bounds how many the tag EMITTED, which the
        # union of receptions cannot: a blink NO anchor heard leaves a hole in
        # the seq numbering and nothing else.
        seqs = [s for s, _a in blinks]
        emitted = max(seqs) - min(seqs) + 1

        anchors = sorted({a for _s, aset in blinks for a in aset})
        print("tag 0x%04X" % tag)
        print("  %d distinct blinks reported by somebody; seq span implies "
              ">= %d emitted" % (total, emitted))
        if emitted > 0:
            print("  fleet-wide: %d of %d heard by at least one anchor "
                  "(%.1f%%) -- blinks NO anchor heard are invisible to a "
                  "coverage figure and only this line bounds them"
                  % (total, emitted, 100.0 * total / emitted))

        print("  %-8s %8s %8s   %s" % ("anchor", "heard", "coverage",
                                       "median q (Ipatov accum)"))
        for a in anchors:
            n = sum(1 for _s, aset in blinks if a in aset)
            q = quality.get((tag, a), [])
            qs = "%d (n=%d)" % (sorted(q)[len(q) // 2], len(q)) if q else "-"
            print("  %-8d %8d %7.1f%%   %s" % (a, n, 100.0 * n / total, qs))

        # How many anchors heard each blink -- what actually decides whether a
        # fix is possible at all (TDOA_MIN_ANCHORS is 3).
        hist = defaultdict(int)
        for _s, aset in blinks:
            hist[len(aset)] += 1
        print("  anchors per blink:", ", ".join(
            "%d->%d" % (k, hist[k]) for k in sorted(hist)))
        solvable = sum(v for k, v in hist.items() if k >= 3)
        print("  %d of %d blinks reached >= 3 anchors (%.1f%%) -- below "
              "TDOA_MIN_ANCHORS nothing is solvable"
              % (solvable, total, 100.0 * solvable / total))
        print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
