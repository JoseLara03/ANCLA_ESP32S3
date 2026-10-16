"""Measure per-anchor timestamp offsets from ONE capture with a tag at a known position.

Anchors are unreachable once mounted, so a static timestamp error (a wrong `tau`, an RX
delay) cannot be fixed with `sync tau set`. The edge server applies it instead
(OFFSETS_DTU). This tool produces that value.

    python -I edge/tools/calibrate.py cap.csv --ranges 0=3.279,1=1.261,2=1.680,3=3.669

cap.csv is a capture of the local broker (columns local_ms, topic, payload) that contains
`uwb/anchor/blink/<zone>` messages, e.g. made with the probe used in this repo's sessions.

Inputs
  --ranges  laser SLANT distances from the tag antenna to each anchor antenna, metres.
            Preferred: it needs no survey geometry, so survey error cannot leak in.
  --ref     the anchor whose offset is defined as 0 (default: smallest median sigma). Only
            DIFFERENCES between anchors are observable, so a common constant is meaningless.

Output: OFFSETS_DTU JSON, plus a stability table. An anchor whose offset drifts by more than
~100 mm across the capture, or whose published sigma is saturated, is reported UNSTABLE: its
mean is not a calibration, it is a symptom (a sync model that is still being corrupted).
"""
import argparse
import csv
import json
import statistics
import sys

W = 1 << 40
MM_PER_DTU = 4.6917214


def sdelta(a, b):
    d = (a - b) % W
    return d - W if d >= W // 2 else d


def load_groups(path):
    groups, open_ = [], {}
    sig = {}
    with open(path, newline="") as f:
        rd = csv.reader(f)
        next(rd)
        t0 = None
        for r in rd:
            if "anchor/blink" not in r[1]:
                continue
            t = float(r[0])
            t0 = t if t0 is None else t0
            j = json.loads(r[2])
            sig.setdefault(j["a"], []).append(j.get("e", 0))
            g = open_.get(j["s"])
            if g is None or t - g["ta"] > 600 or j["a"] in g["v"]:
                g = {"t": t - t0, "ta": t, "v": {}}
                open_[j["s"]] = g
                groups.append(g)
            g["v"][j["a"]] = int(j["ts"])
    return groups, sig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("--ranges", required=True, help="id=metres,... e.g. 0=3.279,1=1.261")
    ap.add_argument("--ref", type=int, default=None)
    ap.add_argument("--drift-mm", type=float, default=100.0)
    a = ap.parse_args()

    d = {int(k): float(v) for k, v in (p.split("=") for p in a.ranges.split(","))}
    groups, sig = load_groups(a.capture)
    ids = sorted(d)
    full = [g for g in groups if all(i in g["v"] for i in ids)]
    if len(full) < 100:
        sys.exit(f"only {len(full)} complete groups; need at least 100")

    med_sigma = {i: statistics.median(sig[i]) for i in ids}
    ref = a.ref if a.ref is not None else min(ids, key=lambda i: med_sigma[i])
    span_s = max(g["t"] for g in full) / 1000.0
    n_bins = max(2, int(span_s // 60))
    print(f"{len(full)} complete groups over {span_s:.0f} s; reference anchor = {ref}; "
          f"median sigma per anchor: { {i: int(v) for i, v in med_sigma.items()} }")

    offsets, report = {}, []
    for k in ids:
        if k == ref:
            offsets[k] = 0
            continue
        b_mm = [(sdelta(g["v"][k], g["v"][ref]) * MM_PER_DTU / 1000.0 - (d[k] - d[ref])) * 1000.0
                for g in full]
        med = statistics.median(b_mm)
        bins = []
        for i in range(n_bins):
            lo, hi = i * span_s * 1000 / n_bins, (i + 1) * span_s * 1000 / n_bins
            vals = [b for g, b in zip(full, b_mm) if lo <= g["t"] < hi]
            if vals:
                bins.append(statistics.median(vals))
        drift = max(bins) - min(bins)
        offsets[k] = round(med / MM_PER_DTU)
        report.append((k, med, med / MM_PER_DTU, statistics.pstdev(b_mm), drift,
                       drift > a.drift_mm or med_sigma[k] > 1000))

    print("\nanchor  offset_mm  offset_DTU  sd_mm  drift_mm(median per bin)  status")
    for k, mm, dtu, sd, dr, bad in report:
        print(f"  {k}     {mm:+8.0f}  {dtu:+9.1f}  {sd:6.0f}  {dr:8.0f}                 "
              f"{'UNSTABLE - do not trust' if bad else 'ok'}")
    print("\nOFFSETS_DTU=" + json.dumps({str(k): v for k, v in offsets.items() if v}))
    if any(r[5] for r in report):
        print("\nWARNING: at least one anchor is UNSTABLE. Its offset above is not a calibration; "
              "the server excludes anchors with a saturated sigma instead (SIGMA_MAX_DTU).")


if __name__ == "__main__":
    main()
