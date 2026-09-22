#!/usr/bin/env python3
"""Compute B14 flight-time compensation (tau) per anchor from a survey.

WHY THIS EXISTS
----------------
The CCP converts every anchor's timestamps into the sync master's (the
gateway's) TRANSMIT time base, but the model that does the converting never
subtracted the master-to-anchor flight time back off -- see CLAUDE.md's B14
entry. Every anchor ends up EARLY by `d_Mi / c`, and since the CCP master
(the gateway) is not one of the anchors ranged by the survey, nothing on the
anchor side can measure this itself in Fase A. It has to be computed from a
known geometry and pushed to each anchor with `sync tau set <dtu>`.

    tau_i_dtu = round(distance(gateway, anchor_i) * DTU_PER_METRE)

Positive tau means the anchor is FARTHER from the gateway than the survey's
own reference node would need to be for tau=0 to be exactly right for
everyone -- there is no such node in general, this is just the anchor's own
one-way flight time. See sync_model.h's sync_model_set_tau() for what the
firmware does with the result and why the sign is "the anchor is ahead of
the master's declared time by this much, so add it back".

INPUT
-----
A capture of `apos show`'s console output (copy-paste the terminal, or
redirect a monitor log to a file) plus the gateway's own position in the SAME
survey frame, since the gateway is deliberately not one of the surveyed
nodes (it is MAC-only and ranges no one -- CLAUDE.md's "gateway is not in the
survey" entry). Only lines shaped like

    {"addr":"0x0001","x":1.263,"y":0.000,"z":0.000}

are read; everything else in the capture (the enumerated-peers block, the
phase/session line, ref lat/lon, ...) is ignored, same tolerant-line-scanning
approach as tools/rx_cal.py.

The gateway position is NOT in that capture -- there is nothing in `apos
show` that could report it, since the gateway holds no survey entry of its
own. Measure it (tape or a second, throwaway `apos`-style survey point) and
pass it with --gateway.

OUTPUT
------
One `sync tau set <dtu>` line per surveyed anchor, ready to paste into that
anchor's console, plus the same in JSON for scripting.

USAGE
-----
    python tools/sync_tau_from_survey.py --gateway 1.19,0.46,0.0 apos_show.txt
"""

import argparse
import json
import re
import sys

# Path length per DW3220 device time unit -- the same constant tools/rx_cal.py
# uses, for the same reason: 1 DTU = 1 / (499.2 MHz * 128) s, times c, in mm.
MM_PER_DTU = 4.6917214
DTU_PER_METRE = 1000.0 / MM_PER_DTU

_NODE = re.compile(
    r'"addr"\s*:\s*"(0x[0-9A-Fa-f]+)"\s*,\s*"x"\s*:\s*([-\d.]+)\s*,\s*'
    r'"y"\s*:\s*([-\d.]+)\s*,\s*"z"\s*:\s*([-\d.]+)'
)


def parse_nodes(text):
    """[(addr_int, x, y, z), ...] for every `apos show` node line found."""
    nodes = []
    for m in _NODE.finditer(text):
        addr = int(m.group(1), 16)
        x, y, z = (float(m.group(i)) for i in (2, 3, 4))
        nodes.append((addr, x, y, z))
    return nodes


def tau_dtu(gateway_xyz, anchor_xyz):
    gx, gy, gz = gateway_xyz
    ax, ay, az = anchor_xyz
    dist_m = ((ax - gx) ** 2 + (ay - gy) ** 2 + (az - gz) ** 2) ** 0.5
    return round(dist_m * DTU_PER_METRE), dist_m


def parse_xyz(s):
    parts = s.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f'"{s}" is not x,y,z (three comma-separated numbers)')
    try:
        return tuple(float(p) for p in parts)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from e


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", nargs="?", type=argparse.FileType("r"),
                     default=sys.stdin,
                     help="apos show capture (default: stdin)")
    ap.add_argument("--gateway", required=True, type=parse_xyz,
                     help="gateway position in the survey frame, x,y,z metres")
    ap.add_argument("--json", action="store_true",
                     help="emit JSON instead of console commands")
    args = ap.parse_args()

    text = args.capture.read()
    nodes = parse_nodes(text)
    if not nodes:
        print("no `apos show` node lines found in the capture -- expected "
              'lines shaped like {"addr":"0x0001","x":...,"y":...,"z":...}',
              file=sys.stderr)
        return 1

    results = []
    for addr, x, y, z in nodes:
        dtu, dist_m = tau_dtu(args.gateway, (x, y, z))
        results.append({"addr": f"0x{addr:04X}", "anchor_id": addr - 1,
                         "distance_m": round(dist_m, 3), "tau_dtu": dtu})

    if args.json:
        print(json.dumps(results, indent=2))
    else:
        for r in results:
            print(f'# anchor {r["addr"]} (id {r["anchor_id"]}), '
                  f'{r["distance_m"]} m from gateway')
            print(f'sync tau set {r["tau_dtu"]}')
    return 0


if __name__ == "__main__":
    sys.exit(main())
