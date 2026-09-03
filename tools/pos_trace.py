#!/usr/bin/env python3
"""
Reduce a GATEWAY console capture to the numbers the TDoA position tasks turn
on. Written for Task 1 (stationary-tag baseline) and Task 7 (the filter
runaway) of docs/superpowers/plans/2026-09-03-tdoa-accuracy-filter-part2.md,
after the same analysis was done by hand twice and re-derived differently
each time.

Reads two kinds of line, both already in the production image:

    <inf> pos_sink: {"tag":"0x0100","tid":693116308,"x":1.16,"y":0.69,...}
    <inf> apos_gw:  {"apos_peer":{...,"x":1.752,"y":0.920,"z":0.000}}

The anchor geometry is taken from the capture's own `apos enum` output rather
than typed on the command line -- a hull compared against hand-entered
coordinates measures the typing, not the deployment. Pass --anchor to
override when the capture has no enumeration in it (e.g. a run that did not
re-enumerate after a reboot).

    python pos_trace.py CAPTURE.txt
    python pos_trace.py CAPTURE.txt --tid 693116308
    python pos_trace.py CAPTURE.txt --anchor 0,0 --anchor 1.752,0.920 \
                                    --anchor 2.385,0
    python pos_trace.py CAPTURE.txt --csv trace.csv

WHAT THIS IS NOT: it does not know where the tag actually was. Every
dispersion figure it prints is spread about the trace's OWN mean, i.e.
PRECISION. Turning any of it into accuracy needs a surveyed ground-truth
point and a tape measure, which is a separate and still-unstarted piece of
work (CLAUDE.md, TDoA section).

The console is ANSI-coloured and carries shell prompt redraws, so lines are
stripped before parsing. Timestamps come from Zephyr's own log clock, which
is a millisecond counter -- fine for cadence and gaps, and NOT the same clock
the filter's dt comes from (that one is the reference anchor's 40-bit DTU;
see tdoa_gw.c). A gap seen here is a gap in PUBLISHED fixes, which is what
matters for a consumer, but it is not by itself proof of what the filter did.
"""

import argparse
import io
import math
import re
import sys

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")
PROMPT_RE = re.compile(r"uwb:~\$ ")

FIX_RE = re.compile(
    r"\[(\d+):(\d+):([0-9.]+)[,0-9]*\].*pos_sink:.*"
    r'"tag":"0x([0-9A-Fa-f]+)".*?"tid":(\d+).*?"x":(-?[0-9.]+),"y":(-?[0-9.]+)'
)
PEER_RE = re.compile(
    r'"apos_peer":\{[^}]*?"addr":"0x([0-9A-Fa-f]+)"[^}]*?'
    r'"x":(-?[0-9.]+),"y":(-?[0-9.]+)'
)
STATS_RE = re.compile(r'\{"(blink|tdoa|tdoa_ekf)":\{(.+?)\}\}')


def clean(path):
    out = []
    with io.open(path, encoding="utf-8", errors="replace") as fh:
        for ln in fh:
            out.append(PROMPT_RE.sub("", ANSI_RE.sub("", ln)).rstrip("\n"))
    return out


BOOT_RE = re.compile(r"Booting Zephyr OS")
TS_RE = re.compile(r"\[(\d+):(\d+):([0-9.]+)")


def parse(lines):
    fixes = []
    anchors = {}
    stats = []
    boot = 0
    for idx, ln in enumerate(lines):
        if BOOT_RE.search(ln):
            boot += 1
            continue
        m = FIX_RE.search(ln)
        if m:
            t = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))
            fixes.append(
                dict(t=t, boot=boot, idx=idx, addr=m.group(4), tid=int(m.group(5)),
                     x=float(m.group(6)), y=float(m.group(7)))
            )
            continue
        m = PEER_RE.search(ln)
        if m:
            # Enumeration repeats across rounds; last writer wins, and every
            # round reports the same coordinates for a given address.
            anchors[m.group(1)] = (float(m.group(2)), float(m.group(3)))
            continue
        m = STATS_RE.search(ln)
        if m:
            kv = {}
            for part in m.group(2).split(","):
                if ":" not in part:
                    continue
                k, v = part.split(":", 1)
                kv[k.strip().strip('"')] = v.strip().strip('"')
            # `blink stats` prints from the SHELL, so its lines carry no
            # [hh:mm:ss] log prefix. The file line index is what orders it
            # against the fixes; a timestamp is simply not available.
            stats.append((m.group(1), kv, boot, idx))
    return fixes, anchors, stats


def pct(sorted_vals, q):
    if not sorted_vals:
        return float("nan")
    return sorted_vals[min(len(sorted_vals) - 1,
                           max(0, int(q * (len(sorted_vals) - 1))))]


def hull_metrics(pts, anchors):
    """Distance each point lies OUTSIDE the anchor polygon, 0 when inside.

    For 3 anchors this is the triangle; for more it is the convex hull. The
    polygon is what the survey placed, so "outside" is a statement about the
    deployment and not about a threshold anyone chose.
    """
    if len(anchors) < 3:
        return None

    A = convex_hull(anchors)

    def seg_dist(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx +
                                                   (p[1] - a[1]) * dy) / L2))
        return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))

    def inside(p):
        sign = None
        for i in range(len(A)):
            a, b = A[i], A[(i + 1) % len(A)]
            cr = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
            if abs(cr) < 1e-12:
                continue
            s = cr > 0
            if sign is None:
                sign = s
            elif s != sign:
                return False
        return True

    out = []
    for p in pts:
        if inside(p):
            out.append(0.0)
        else:
            out.append(min(seg_dist(p, A[i], A[(i + 1) % len(A)])
                           for i in range(len(A))))
    return out


def convex_hull(pts):
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts

    def half(ps):
        st = []
        for p in ps:
            while len(st) >= 2:
                a, b = st[-2], st[-1]
                if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) <= 0:
                    st.pop()
                else:
                    break
            st.append(p)
        return st

    return half(pts)[:-1] + half(list(reversed(pts)))[:-1]


def report_tag(tid, rows, anchors, runaway_m):
    print()
    print("=" * 68)
    print("tag Tid=%d  (short addr seen: %s)"
          % (tid, ",".join(sorted({r["addr"] for r in rows}))))
    print("=" * 68)
    print("fixes %d   span %.1f s" % (len(rows), rows[-1]["t"] - rows[0]["t"]))
    if len(rows) < 4:
        print("  too few fixes to say anything")
        return

    # ---- cadence -------------------------------------------------------
    dts = [rows[i]["t"] - rows[i - 1]["t"] for i in range(1, len(rows))]
    ds = sorted(dts)
    gaps = [d for d in dts if d > 1.0]
    print()
    print("cadence of PUBLISHED fixes")
    print("  dt  median %.3f s   p90 %.3f   max %.3f"
          % (pct(ds, 0.5), pct(ds, 0.9), ds[-1]))
    print("  gaps > 1 s: %d" % len(gaps))
    print("  NOTE: TDOA_DT_MAX_MS gates the FILTER's dt, which comes off the")
    print("        reference anchor's DTU clock, not this log clock. A gap")
    print("        here bounds what a consumer saw, not what the filter did.")

    # ---- dispersion about the trace's own mean -- PRECISION -------------
    mx = sum(r["x"] for r in rows) / len(rows)
    my = sum(r["y"] for r in rows) / len(rows)
    dev = [math.hypot(r["x"] - mx, r["y"] - my) for r in rows]
    rms = math.sqrt(sum(d * d for d in dev) / len(dev))
    sdx = math.sqrt(sum((r["x"] - mx) ** 2 for r in rows) / len(rows))
    sdy = math.sqrt(sum((r["y"] - my) ** 2 for r in rows) / len(rows))
    dsort = sorted(dev)
    print()
    print("dispersion about the trace's OWN mean  (PRECISION, not accuracy)")
    print("  mean (%.3f, %.3f)" % (mx, my))
    print("  RMS %.3f m    p50 %.3f   p95 %.3f   max %.3f"
          % (rms, pct(dsort, 0.5), pct(dsort, 0.95), dsort[-1]))
    print("  std x %.3f m   std y %.3f m" % (sdx, sdy))
    if sdx > 0 and sdy / sdx > 1.8:
        print("  y is %.1fx noisier than x -- consistent with a THIN anchor"
              % (sdy / sdx))
        print("  geometry, where y is the weakly observable axis.")
    elif sdy > 0 and sdx / sdy > 1.8:
        print("  x is %.1fx noisier than y." % (sdx / sdy))

    # ---- consecutive-step size ----------------------------------------
    steps = []
    n_same_ms = 0
    for i in range(1, len(rows)):
        dt = rows[i]["t"] - rows[i - 1]["t"]
        d = math.hypot(rows[i]["x"] - rows[i - 1]["x"],
                       rows[i]["y"] - rows[i - 1]["y"])
        if dt <= 0.0:
            # Two fixes stamped in the same millisecond by the LOG clock.
            # Dividing by that dt yields hundreds of m/s, which looks like a
            # measurement and is an artifact of log-clock quantisation -- so
            # the pair is counted and excluded from the speed statistics
            # rather than left to inflate the max.
            n_same_ms += 1
            steps.append((d, None))
            continue
        steps.append((d, d / dt))
    sd = sorted(s[0] for s in steps)
    sv = sorted(s[1] for s in steps if s[1] is not None)
    print()
    print("consecutive step")
    print("  distance  p50 %.3f m   p95 %.3f   max %.3f"
          % (pct(sd, 0.5), pct(sd, 0.95), sd[-1]))
    print("  implied speed  p50 %.2f m/s   p95 %.2f   max %.2f"
          % (pct(sv, 0.5), pct(sv, 0.95), sv[-1]))
    if n_same_ms:
        print("  (%d consecutive pair(s) shared a log millisecond and are "
              "excluded" % n_same_ms)
        print("   from the speed figures -- log-clock quantisation, not a "
              "measurement)")

    # ---- hull ----------------------------------------------------------
    outs = hull_metrics([(r["x"], r["y"]) for r in rows], list(anchors.values()))
    if outs is None:
        print()
        print("no anchor geometry in the capture -- pass --anchor to get the")
        print("inside/outside figures (they are the Task 7 acceptance number)")
        return

    n_in = sum(1 for v in outs if v == 0.0)
    osort = sorted(outs)
    print()
    print("position against the SURVEYED anchor polygon")
    print("  inside: %d of %d (%.1f%%)" % (n_in, len(outs),
                                           100.0 * n_in / len(outs)))
    print("  distance outside  p50 %.2f m   p90 %.2f   max %.2f"
          % (pct(osort, 0.5), pct(osort, 0.9), osort[-1]))
    for thr in (1.0, 3.0, 5.0):
        k = sum(1 for v in outs if v > thr)
        print("  more than %.0f m outside: %d (%.1f%%)"
              % (thr, k, 100.0 * k / len(outs)))

    # ---- runaway episodes ---------------------------------------------
    runs = []
    cur = []
    for r, o in zip(rows, outs):
        if o > runaway_m:
            cur.append((r, o))
        else:
            if cur:
                runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    runs.sort(key=len, reverse=True)
    print()
    print("runaway episodes  (consecutive fixes more than %.1f m outside)"
          % runaway_m)
    if not runs:
        print("  none")
    for ep in runs[:3]:
        worst = max(ep, key=lambda z: z[1])
        print("  t=%.1f..%.1f  (%.1f s, %d fixes)  from (%.2f,%.2f) "
              "to (%.2f,%.2f), %.2f m out"
              % (ep[0][0]["t"], ep[-1][0]["t"],
                 ep[-1][0]["t"] - ep[0][0]["t"], len(ep),
                 ep[0][0]["x"], ep[0][0]["y"],
                 worst[0]["x"], worst[0]["y"], worst[1]))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("--tid", type=int, action="append",
                    help="only this Tid; repeatable. Default: every tag seen.")
    ap.add_argument("--anchor", action="append", default=[],
                    help="x,y of an anchor, repeatable. Overrides the "
                         "capture's own apos enum output.")
    ap.add_argument("--runaway", type=float, default=1.0,
                    help="metres outside the polygon that counts as a runaway "
                         "fix (default 1.0)")
    ap.add_argument("--csv", help="write the parsed trace here")
    args = ap.parse_args()

    lines = clean(args.capture)
    fixes, anchors, stats = parse(lines)

    if args.anchor:
        anchors = {}
        for i, a in enumerate(args.anchor):
            x, y = a.split(",")
            anchors["arg%d" % i] = (float(x), float(y))

    print("parsed %d fix lines, %d anchors, %d stats lines"
          % (len(fixes), len(anchors), len(stats)))
    if anchors:
        print("anchor geometry:")
        for k in sorted(anchors):
            print("  %-6s (%.3f, %.3f)" % (k, anchors[k][0], anchors[k][1]))
        pts = list(anchors.values())
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        R = max(math.hypot(p[0] - cx, p[1] - cy) for p in pts)
        print("  centroid (%.3f, %.3f)   max anchor-centroid radius %.3f m"
              % (cx, cy, R))

    if stats:
        print()
        print("last stats line of each kind:")
        seen = {}
        for kind, kv, _b, _t in stats:
            seen[kind] = kv
        for kind in ("blink", "tdoa", "tdoa_ekf"):
            if kind in seen:
                print("  %-9s %s" % (kind, seen[kind]))
        e = seen.get("tdoa_ekf")
        t = seen.get("tdoa")
        if e and t:
            try:
                s, f, r = (int(e["seeded"]), int(e["filtered"]),
                           int(e["reseed"]))
                fx = int(t["fixes"])
                print("  fixes(%d) vs seeded+filtered+reseed(%d): %s"
                      % (fx, s + f + r,
                         "balances" if fx == s + f + r
                         else "MISMATCH -- stale republish, see tdoa_gw.c"))
                if f and r == 0:
                    print("  reseed is 0 over %d filtered cycles -- the "
                          "divergence" % f)
                    print("  recovery never fired. See Task 7.")
            except (KeyError, ValueError):
                pass

    if not fixes:
        print()
        print("no pos_sink lines found -- is this a GATEWAY capture?")
        return 1

    # ---- console loss: the trap this check exists to stop -----------------
    #
    # tdoa.fixes counts what the gateway PRODUCED; the pos_sink lines are what
    # the console DELIVERED. The USB-JTAG console and the terminal program both
    # sit outside Zephyr's log accounting, so records can vanish with no
    # "--- N messages dropped ---" line anywhere. When that happens every
    # cadence figure above turns into a statement about the console, and the
    # gaps it reports are not gaps in the fix stream at all.
    #
    # Measured on COM15_2026_09_03.12.26.58.574: 1424 fixes produced, 291
    # logged (20.4%), with the shortfall concentrated in the tail -- which
    # reads exactly like a tag dropping to a slow tier and is nothing of the
    # kind. Worth an explicit check because that misreading was one sentence
    # away from being written down as a finding.
    #
    # TWO ways this comparison lies if made naively, both found on real
    # captures 2026-09-03:
    #
    #   - A REBOOT resets the counters while the log keeps accumulating
    #     lines, so a multi-boot capture reports more lines than fixes.
    #   - `blink stats` run MID-RUN snapshots a counter that later fixes are
    #     not in, with the same effect.
    #
    # So the comparison is scoped to the counter's own boot and to fixes at or
    # before its timestamp. A capture whose stats line is not the last thing
    # in it simply gets no delivery figure, which is better than a wrong one.
    prod = None
    prod_boot = None
    prod_idx = None
    for kind, kv, b, i in stats:
        if kind == "tdoa" and "fixes" in kv:
            try:
                prod, prod_boot, prod_idx = int(kv["fixes"]), b, i
            except ValueError:
                pass
    frac = 100.0
    if prod:
        boots = sorted({r["boot"] for r in fixes})
        if len(boots) > 1:
            print()
            print("capture spans %d boot(s) with fixes in them (%s). Counters"
                  % (len(boots), ",".join(str(b) for b in boots)))
            print("reset on every boot, so the delivery check below is scoped")
            print("to boot %d, the one the last `blink stats` came from."
                  % prod_boot)
        scoped = [r for r in fixes
                  if r["boot"] == prod_boot and r["idx"] < prod_idx]
        after = len([r for r in fixes
                     if r["boot"] == prod_boot and r["idx"] > prod_idx])
        frac = 100.0 * len(scoped) / prod
        print()
        print("console delivery: %d of %d fixes the gateway counted (%.1f%%)"
              % (len(scoped), prod, frac))
        if after:
            print("  (%d further fix line(s) came AFTER that `blink stats`, so"
                  % after)
            print("   the counter does not cover them and they are excluded.")
            print("   Run `blink stats` LAST for this figure to cover the run.)")
        if frac < 90.0:
            print("  *** THE CONSOLE DROPPED RECORDS. Zephyr reports no drop")
            print("  *** here because the loss is below its accounting (USB")
            print("  *** console / terminal). Treat every cadence and gap")
            print("  *** figure below as a property of the CONSOLE, not of")
            print("  *** the fix stream. Dispersion and hull figures survive")
            print("  *** (they are per-fix), but they are a SAMPLE, and the")
            print("  *** sampling is not uniform in time.")
            print("  *** For real cadence use the gateway's own counters:")
            print("  *** ingested / anchors = blinks, and dt_invalid.")

    if args.csv:
        with io.open(args.csv, "w", encoding="utf-8", newline="") as fh:
            fh.write("t,tid,addr,x,y\n")
            for r in fixes:
                fh.write("%.3f,%d,%s,%.3f,%.3f\n"
                         % (r["t"], r["tid"], r["addr"], r["x"], r["y"]))
        print("wrote %s" % args.csv)

    tids = sorted({r["tid"] for r in fixes})
    if args.tid:
        tids = [t for t in tids if t in args.tid]
    for tid in tids:
        report_tag(tid, [r for r in fixes if r["tid"] == tid],
                   anchors, args.runaway)
    return 0


if __name__ == "__main__":
    sys.exit(main())
