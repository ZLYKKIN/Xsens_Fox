#!/usr/bin/env python3
"""Validate left/right glove finger world directions are mirror-symmetric when
the user holds K-pose-like spread (start of session, before first movement).
Reads [ergo-raw] entries from fox_mocap.log.
Usage: python tools/verify_glove_mirror.py logs/fox_mocap.log
"""
import re
import sys
import math

TICK_RE = re.compile(r"\[ergo-raw\] tick=(\d+)")
ENTRY_RE = re.compile(r"\[entry\s+(\d)\] id=(\d+)\s+\((Right|Left)\s+Glove\)")
FINGER_RE = re.compile(
    r"(thumb|index|middle|ring|pinky)\s+spread=(-?[0-9.]+)°\s+MCP=(-?[0-9.]+)°\s+"
    r"PIP=(-?[0-9.]+)°\s+DIP=(-?[0-9.]+)°"
)


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    snapshots = []
    cur = None
    cur_hand = None
    with open(sys.argv[1], "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            mt = TICK_RE.search(line)
            if mt:
                if cur:
                    snapshots.append(cur)
                cur = {"tick": int(mt.group(1)), "left": {}, "right": {}}
                cur_hand = None
                continue
            me = ENTRY_RE.search(line)
            if me and cur:
                cur_hand = me.group(3).lower()
                continue
            mf = FINGER_RE.search(line)
            if mf and cur and cur_hand:
                name = mf.group(1)
                spread = float(mf.group(2))
                mcp = float(mf.group(3))
                pip = float(mf.group(4))
                dip = float(mf.group(5))
                cur[cur_hand][name] = (spread, mcp, pip, dip)
            if len(snapshots) > 10:
                break
        if cur:
            snapshots.append(cur)
    if not snapshots:
        print("No glove ergonomic snapshots found.")
        return
    snap = snapshots[0]
    print(f"Inspecting first snapshot tick={snap['tick']}")
    if not snap["left"] or not snap["right"]:
        print("Missing one hand's data; cannot compare.")
        return
    print(f"{'finger':<8} {'L(spread,MCP,PIP,DIP)':<28} {'R(spread,MCP,PIP,DIP)':<28} "
          f"{'spread_mirror_ok':<18}")
    fingers = ["thumb", "index", "middle", "ring", "pinky"]
    mirror_ok = 0
    mirror_total = 0
    for f in fingers:
        L = snap["left"].get(f)
        R = snap["right"].get(f)
        if not L or not R:
            continue
        mirror_total += 1
        spread_ok = abs(L[0] + R[0]) < 25.0
        if spread_ok:
            mirror_ok += 1
        Ltxt = f"({L[0]:>6.1f},{L[1]:>5.1f},{L[2]:>5.1f},{L[3]:>5.1f})"
        Rtxt = f"({R[0]:>6.1f},{R[1]:>5.1f},{R[2]:>5.1f},{R[3]:>5.1f})"
        print(f"{f:<8} {Ltxt:<28} {Rtxt:<28} {'YES' if spread_ok else 'NO':<18}")
    pct = 100.0 * mirror_ok / max(1, mirror_total)
    print(f"Mirror symmetry: {mirror_ok}/{mirror_total} ({pct:.0f}%)")
    if pct < 60.0:
        print("WARN: gloves may not be in symmetric pose at session start.")


if __name__ == "__main__":
    main()
