"""Analyze a mqtt_capture.py CSV: per-tag fix intervals, duplicates, jumps,
and per-blink-seq anchor coverage / timestamp spread."""
import csv
import json
import sys
from collections import defaultdict

rows = list(csv.DictReader(open(sys.argv[1])))
pos = defaultdict(list)
blink = defaultdict(list)

for r in rows:
    t = float(r["local_ms"])
    if "position" in r["topic"]:
        d = json.loads(r["payload"])
        pos[d["Tid"]].append((t, d["x"], d["y"]))
    elif "blink" in r["topic"]:
        d = json.loads(r["payload"])
        blink[d["t"]].append((t, d["s"], d["a"], int(d["ts"])))

DTU_S = 1.56498e-11

print("== POSITIONS ==")
for tid, fix in pos.items():
    print(f"--- Tid {tid}: {len(fix)} fixes")
    pt = None
    for i, (t, x, y) in enumerate(fix):
        dt = t - pt[0] if pt else 0
        dist = ((x - pt[1]) ** 2 + (y - pt[2]) ** 2) ** 0.5 if pt else 0
        dup = " DUP" if pt and x == pt[1] and y == pt[2] else ""
        print(f"{i:3d} t={t:14.0f} dt={dt:7.1f} x={x:9.3f} y={y:9.3f} "
              f"step={dist:6.3f}{dup}")
        pt = (t, x, y)

print("\n== BLINKS per tag ==")
for tag, bl in blink.items():
    byseq = defaultdict(list)
    for t, s, a, ts in bl:
        byseq[s].append((t, a, ts))
    seqs = sorted(byseq)
    print(f"tag 0x{tag:04X}: {len(bl)} obs, {len(seqs)} distinct seqs")
    prev_ts = None
    for s in seqs:
        obs = byseq[s]
        ts0 = obs[0][2]
        dts = (ts0 - prev_ts) * DTU_S * 1000 if prev_ts else 0
        prev_ts = ts0
        anchors = sorted(a for _, a, _ in obs)
        spread_ms = (max(o[2] for o in obs) - min(o[2] for o in obs)) * DTU_S * 1000
        print(f"  seq={s:4d} blinkdt={dts:8.1f}ms anchors={anchors} "
              f"ts_spread={spread_ms:6.3f}ms")
