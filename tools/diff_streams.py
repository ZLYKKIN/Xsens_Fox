#!/usr/bin/env python3
"""Cross-check the MVN MXTP pose stream Fox Mocap emits against what the Blender
add-on actually receives, segment by segment.

Two logs are compared:

  * Fox Mocap stdout captured with ``-test`` — ``[STREAM SNAPSHOT]`` blocks with
    ``wire[ID] Name pos=(x,y,z) q=(w,x,y,z)`` lines (and ``wireF[ID] ...`` for
    glove fingers).  This is the *sent* wire value.
  * The Blender add-on's ``alllog.txt`` — ``[BLENDER SNAPSHOT]`` blocks with
    ``seg[idx] name quat=(w,x,y,z) pos=(x,y,z)`` lines, where ``idx == ID - 1``.
    This is the *received* wire value (logged raw, before pose.py conversion).

The two values for the same segment ID must be identical up to log precision.
A mismatch pinpoints a transport/conformance bug — wrong segment ordering, a
swapped quaternion component (w/x/y/z order), endianness, or dropped segments —
rather than a downstream retargeting issue.

Usage:
    python3 tools/diff_streams.py fox_stream.log blender_alllog.txt
    python3 tools/diff_streams.py fox.log alllog.txt --tol-deg 0.5 --tol-pos 0.01
    python3 tools/diff_streams.py --self-test

Exit code is non-zero if any segment exceeds tolerance (handy for CI / scripts).
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from typing import Dict, List, Optional, Tuple

# A parsed snapshot: {segment_id (1-based): (pos(x,y,z), quat(w,x,y,z))}.
Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
Snapshot = Dict[int, Tuple[Vec3, Quat]]

_NUM = r"([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)"

# Fox Mocap stdout (main.cpp pushFrame / pushFrameWithGloves).
_FOX_HEADER = re.compile(r"\[STREAM SNAPSHOT\].*?sample=(\d+)")
_FOX_BODY = re.compile(
    r"wire\[\s*(\d+)\]\s+\S+\s+"
    r"pos=\(\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*\)\s+"
    r"q=\(\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*\)"
)
_FOX_FINGER = re.compile(
    r"wireF\[\s*(\d+)\].*?"
    r"pos=\(\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*\).*?"
    r"q=\(\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*\)"
)

# Blender add-on alllog.txt (file_logger.log_snapshot).
_BL_HEADER = re.compile(r"\[BLENDER SNAPSHOT\]\s+sample=(\d+)")
_BL_SEG = re.compile(
    r"seg\[\s*(\d+)\]\s+\S+\s+"
    r"quat=\(\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*\)\s+"
    r"pos=\(\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*\)"
)


def _quat_angle_deg(a: Quat, b: Quat) -> float:
    """Geodesic angle between two quaternions, sign-insensitive (q and -q are
    the same rotation).  Both are normalised first because the logged values are
    rounded to 3-4 decimals and are therefore not exactly unit-length."""
    na = math.sqrt(sum(x * x for x in a)) or 1.0
    nb = math.sqrt(sum(x * x for x in b)) or 1.0
    dot = sum((x / na) * (y / nb) for x, y in zip(a, b))
    dot = max(-1.0, min(1.0, abs(dot)))
    return math.degrees(2.0 * math.acos(dot))


def _pos_dist(a: Vec3, b: Vec3) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def parse_fox(text: str) -> List[Tuple[int, Snapshot]]:
    """Parse Fox Mocap -test stdout into [(sample, snapshot), ...]."""
    snaps: List[Tuple[int, Snapshot]] = []
    cur: Optional[Snapshot] = None
    sample = -1
    for line in text.splitlines():
        h = _FOX_HEADER.search(line)
        if h:
            if cur:
                snaps.append((sample, cur))
            sample = int(h.group(1))
            cur = {}
            continue
        if cur is None:
            continue
        m = _FOX_BODY.search(line)
        if m:
            seg = int(m.group(1))
            pos = (float(m.group(2)), float(m.group(3)), float(m.group(4)))
            quat = (float(m.group(5)), float(m.group(6)), float(m.group(7)), float(m.group(8)))
            cur[seg] = (pos, quat)
            continue
        f = _FOX_FINGER.search(line)
        if f:
            seg = int(f.group(1))
            pos = (float(f.group(2)), float(f.group(3)), float(f.group(4)))
            quat = (float(f.group(5)), float(f.group(6)), float(f.group(7)), float(f.group(8)))
            cur[seg] = (pos, quat)
    if cur:
        snaps.append((sample, cur))
    return snaps


def parse_blender(text: str) -> List[Tuple[int, Snapshot]]:
    """Parse Blender alllog.txt into [(sample, snapshot), ...].  Segment idx is
    0-based in the log, so the stored key is idx + 1 to match the wire ID."""
    snaps: List[Tuple[int, Snapshot]] = []
    cur: Optional[Snapshot] = None
    sample = -1
    for line in text.splitlines():
        h = _BL_HEADER.search(line)
        if h:
            if cur:
                snaps.append((sample, cur))
            sample = int(h.group(1))
            cur = {}
            continue
        if cur is None:
            continue
        m = _BL_SEG.search(line)
        if m:
            seg = int(m.group(1)) + 1  # idx -> 1-based wire ID
            quat = (float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)))
            pos = (float(m.group(6)), float(m.group(7)), float(m.group(8)))
            cur[seg] = (pos, quat)
    if cur:
        snaps.append((sample, cur))
    return snaps


def diff_snapshots(fox: Snapshot, blender: Snapshot,
                   tol_deg: float, tol_pos: float) -> Tuple[List[str], int]:
    """Return (report_lines, num_failures)."""
    lines: List[str] = []
    fails = 0
    seg_ids = sorted(set(fox) | set(blender))
    lines.append(f"{'segID':>5}  {'Δquat(deg)':>11}  {'Δpos(m)':>9}  status")
    lines.append("-" * 44)
    for sid in seg_ids:
        if sid not in fox:
            lines.append(f"{sid:>5}  {'—':>11}  {'—':>9}  MISSING in Fox stream")
            fails += 1
            continue
        if sid not in blender:
            lines.append(f"{sid:>5}  {'—':>11}  {'—':>9}  MISSING in Blender log")
            fails += 1
            continue
        fpos, fq = fox[sid]
        bpos, bq = blender[sid]
        dq = _quat_angle_deg(fq, bq)
        dp = _pos_dist(fpos, bpos)
        ok = dq <= tol_deg and dp <= tol_pos
        if not ok:
            fails += 1
        lines.append(f"{sid:>5}  {dq:>11.4f}  {dp:>9.4f}  {'ok' if ok else 'MISMATCH'}")
    return lines, fails


def _pick_last(snaps: List[Tuple[int, Snapshot]], label: str) -> Snapshot:
    if not snaps:
        sys.exit(f"error: no [{label}] snapshots found in input")
    return snaps[-1][1]


def run_diff(fox_path: str, blender_path: str, tol_deg: float, tol_pos: float) -> int:
    with open(fox_path, "r", encoding="utf-8", errors="ignore") as fh:
        fox_snaps = parse_fox(fh.read())
    with open(blender_path, "r", encoding="utf-8", errors="ignore") as fh:
        bl_snaps = parse_blender(fh.read())

    fox = _pick_last(fox_snaps, "STREAM SNAPSHOT")
    blender = _pick_last(bl_snaps, "BLENDER SNAPSHOT")

    print(f"Fox stream:   {len(fox_snaps)} snapshot(s), last has {len(fox)} segment(s)")
    print(f"Blender log:  {len(bl_snaps)} snapshot(s), last has {len(blender)} segment(s)")
    print(f"Tolerance:    {tol_deg} deg / {tol_pos} m\n")

    lines, fails = diff_snapshots(fox, blender, tol_deg, tol_pos)
    print("\n".join(lines))
    print()
    if fails:
        print(f"RESULT: FAIL — {fails} segment(s) outside tolerance.")
        return 1
    print("RESULT: PASS — every segment matches within tolerance.")
    return 0


def _self_test() -> int:
    """Validate the parser + diff logic on synthetic logs (no Blender needed)."""
    fox_log = (
        "========== [STREAM SNAPSHOT] body-only  sample=10  -> 127.0.0.1:9763  ==========\n"
        "  wire[ 1] Pelvis         pos=(  0.0000,  0.0000,  0.0000) q=(  1.0000,  0.0000,  0.0000,  0.0000) |delta|=  0.0000deg\n"
        "  wire[ 2] L5             pos=(  0.0000,  0.0000,  0.1000) q=(  0.7071,  0.0000,  0.0000,  0.7071) |delta|= 90.0000deg\n"
        "  wireF[24] Lcarpus(slot 0) pos=(0,0,0) q=(1.0000,0.0000,0.0000,0.0000)  [follows hand]\n"
        "============================================================\n"
    )
    blender_log = (
        "[schema] version=3\n"
        "========== [BLENDER SNAPSHOT] sample=0  t=1.00s ==========\n"
        "  seg[ 0] pelvis         quat=(+1.000,+0.000,+0.000,+0.000) pos=(+0.000,+0.000,+0.000)\n"
        "  seg[ 1] l5             quat=(+0.707,+0.000,+0.000,+0.707) pos=(+0.000,+0.000,+0.100)\n"
        "============================================================\n"
    )
    fox = parse_fox(fox_log)
    bl = parse_blender(blender_log)
    assert len(fox) == 1 and len(bl) == 1, "snapshot count"
    fsnap, bsnap = fox[0][1], bl[0][1]
    assert set(fsnap) == {1, 2, 24}, f"fox seg ids {set(fsnap)}"
    assert set(bsnap) == {1, 2}, f"blender seg ids {set(bsnap)}"
    # Matching segments (1, 2) must pass; the -q sign must not matter.
    bsnap[2] = (bsnap[2][0], tuple(-c for c in bsnap[2][1]))  # flip hemisphere
    lines, fails = diff_snapshots(fsnap, bsnap, tol_deg=0.5, tol_pos=0.01)
    assert fails == 1, f"expected 1 failure (seg 24 missing in Blender), got {fails}"
    # A genuine quaternion mismatch must be caught.
    bad = dict(bsnap)
    bad[1] = (bad[1][0], (0.0, 1.0, 0.0, 0.0))  # 180deg off
    _, fails2 = diff_snapshots({1: fsnap[1]}, {1: bad[1]}, tol_deg=0.5, tol_pos=0.01)
    assert fails2 == 1, "expected the 180deg mismatch to fail"
    print("self-test: OK (parsing, segId mapping, hemisphere-insensitive diff, mismatch detection)")
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fox_log", nargs="?", help="Fox Mocap -test stdout capture")
    ap.add_argument("blender_log", nargs="?", help="Blender add-on alllog.txt")
    ap.add_argument("--tol-deg", type=float, default=1.0,
                    help="max per-segment quaternion angle delta in degrees (default 1.0)")
    ap.add_argument("--tol-pos", type=float, default=0.01,
                    help="max per-segment position delta in metres (default 0.01)")
    ap.add_argument("--self-test", action="store_true", help="run built-in self-test and exit")
    args = ap.parse_args(argv)

    if args.self_test:
        return _self_test()
    if not args.fox_log or not args.blender_log:
        ap.error("fox_log and blender_log are required (or use --self-test)")
    return run_diff(args.fox_log, args.blender_log, args.tol_deg, args.tol_pos)


if __name__ == "__main__":
    raise SystemExit(main())
