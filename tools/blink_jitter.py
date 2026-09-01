#!/usr/bin/env python3
"""
Task 2 helper: extract BLINK-vs-beacon RMARKER jitter from a simple_rx.c
sniffer capture (fw-cre's Src/examples/ex_02a_simple_rx, modified to print
each frame's DW3000 RX timestamp as "TS=<10 hex digits>").

Method: for a chosen tag's short address, take every BLINK (func 0xF0) frame
and subtract the sniffer's own timestamp of the beacon (func 0xE5, src
0x0000) immediately preceding it in the log. Both timestamps come off the
same DW3000 clock, so the difference is the tag's real RMARKER offset from
the beacon -- the quantity BLINK_SLOT_GUARD_UUS is provisional against.

Only relies on the sniffer's *own* clock: no PC/UART timing involved, and no
40-bit wrap risk since one superframe (200 ms) is far under the ~17.2 s wrap
period, so plain hex subtraction is safe as long as the beacon actually
precedes its blink in the same superframe.

Usage:
    python blink_jitter.py CAPTURE.txt --tag 01 01
    python blink_jitter.py CAPTURE.txt --tag 0101 --csv out.csv

--tag takes the two source-address bytes in the order the sniffer prints
them (lo, hi) -- e.g. a tag printed as "... FF FF 01 01 F0 ..." is --tag 01 01
or --tag 0101 (both forms accepted).
"""

import argparse
import re
import statistics
import sys

DTU_NS = 0.01565  # 1 DTU tick = 15.65 ps
FUNC_BEACON = 0xE5
FUNC_BLINK = 0xF0
MAX_SUPERFRAME_TICKS = int(0.2 / (DTU_NS * 1e-9))  # ~200 ms in DTU ticks, sanity bound

LINE_RE = re.compile(
    r"RX\[(\d+)\]\s*TS=([0-9A-Fa-f]+)\s*RSSI=[-0-9.]+\s*dBm:\s*(.+)"
)


def parse_tag_arg(tag_arg):
    """Accept ['01', '01'] or ['0101'] -> (0x01, 0x01)."""
    joined = "".join(tag_arg).strip()
    if len(joined) != 4:
        raise ValueError(f"--tag must be 4 hex digits (2 bytes), got '{joined}'")
    lo = int(joined[0:2], 16)
    hi = int(joined[2:4], 16)
    return (lo, hi)


def parse_bytes(byte_str):
    """Return list[int] from whitespace-separated 2-hex-digit tokens.
    Returns None if any token is not a clean 2-hex-digit byte (garbled
    line) -- caller should skip the line rather than guess."""
    out = []
    for tok in byte_str.split():
        if len(tok) != 2:
            return None
        try:
            out.append(int(tok, 16))
        except ValueError:
            return None
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile", help="sniffer capture text file")
    ap.add_argument("--tag", nargs="+", required=True, help="tag src addr bytes, lo hi (e.g. --tag 01 01 or --tag 0101)")
    ap.add_argument("--csv", help="optional path to dump per-sample deltas as CSV")
    args = ap.parse_args()

    tag_lo, tag_hi = parse_tag_arg(args.tag)

    last_beacon_ts = None
    deltas = []          # accepted (blink_seq_byte, delta_ticks)
    discarded = 0
    skipped_garbled = 0
    n_beacons = 0
    n_blinks_seen = 0

    with open(args.logfile, "r", encoding="utf-8", errors="replace") as f:
        for lineno, line in enumerate(f, 1):
            m = LINE_RE.search(line)
            if not m:
                continue
            rx_len, ts_hex, byte_str = m.groups()
            frame = parse_bytes(byte_str)
            if frame is None or len(frame) < 10:
                skipped_garbled += 1
                continue

            dest = (frame[5], frame[6]) if len(frame) > 6 else None
            src = (frame[7], frame[8]) if len(frame) > 8 else None
            func = frame[9]

            if func == FUNC_BEACON and src == (0x00, 0x00):
                last_beacon_ts = int(ts_hex, 16)
                n_beacons += 1
                continue

            if func == FUNC_BLINK:
                n_blinks_seen += 1
                if src != (tag_lo, tag_hi):
                    continue
                if last_beacon_ts is None:
                    discarded += 1
                    continue
                blink_ts = int(ts_hex, 16)
                delta = blink_ts - last_beacon_ts
                if delta <= 0 or delta > MAX_SUPERFRAME_TICKS:
                    # beacon before this blink was likely garbled/missed;
                    # this delta is against a stale beacon from an earlier
                    # superframe -- discard rather than report garbage.
                    discarded += 1
                    continue
                blink_seq_byte = frame[10] if len(frame) > 10 else None
                deltas.append((blink_seq_byte, delta))

    print(f"file: {args.logfile}")
    print(f"tag src addr: {tag_lo:02X} {tag_hi:02X}")
    print(f"beacons seen: {n_beacons}, blinks seen (any tag): {n_blinks_seen}, "
          f"garbled lines skipped: {skipped_garbled}")
    print(f"accepted samples: {len(deltas)}, discarded (no/stale beacon or "
          f"out-of-range): {discarded}")

    if not deltas:
        print("No usable samples for this tag. Check --tag bytes and that "
              "this tag's BLINKs appear in the capture.")
        sys.exit(1)

    ticks = [d for _, d in deltas]
    mean_ticks = statistics.mean(ticks)
    stdev_ticks = statistics.pstdev(ticks) if len(ticks) > 1 else 0.0
    min_ticks = min(ticks)
    max_ticks = max(ticks)
    spread_ticks = max_ticks - min_ticks

    def ns(t):
        return t * DTU_NS

    print()
    print(f"offset from beacon RMARKER (n={len(ticks)}):")
    print(f"  mean   : {mean_ticks:,.1f} ticks = {ns(mean_ticks):,.3f} ns "
          f"({ns(mean_ticks)/1e6:.6f} ms)")
    print(f"  stdev  : {stdev_ticks:,.1f} ticks = {ns(stdev_ticks):,.3f} ns")
    print(f"  min    : {min_ticks:,d} ticks = {ns(min_ticks):,.3f} ns")
    print(f"  max    : {max_ticks:,d} ticks = {ns(max_ticks):,.3f} ns")
    print(f"  spread : {spread_ticks:,d} ticks = {ns(spread_ticks):,.3f} ns  "
          f"<-- this is the arm-jitter figure for BLINK_SLOT_GUARD_UUS")

    if len(ticks) < 200:
        print()
        print(f"NOTE: only {len(ticks)} samples -- the plan asks for >=200 "
              f"before this feeds BLINK_SLOT_GUARD_UUS. Treat this as a "
              f"preview, not the final number.")

    if args.csv:
        with open(args.csv, "w", encoding="utf-8") as out:
            out.write("blink_seq_byte,delta_ticks,delta_ns\n")
            for seq_byte, d in deltas:
                seq_str = f"0x{seq_byte:02X}" if seq_byte is not None else ""
                out.write(f"{seq_str},{d},{ns(d):.3f}\n")
        print(f"\nper-sample deltas written to {args.csv}")


if __name__ == "__main__":
    main()
