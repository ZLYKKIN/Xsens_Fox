#!/usr/bin/env python3
"""Pair [send] frames in fox_mocap.log with [recv] frames in blender_session.log.
Reports per-segment max |delta quat| (degrees) and |delta pos| (meters).
Usage: python tools/verify_streaming.py logs/fox_mocap.log logs/blender_session.log
"""
import re
import sys
import math
from collections import defaultdict

SEND_RE = re.compile(
    r"\[send\] t=([0-9.]+) sample=(\d+)(?: wire=(\d+))? seg=(\d+) "
    r"quat=(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+) "
    r"pos=(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+)"
)
RECV_RE = re.compile(
    r"\[recv\] t=([0-9.]+) sample=(\d+) seg=(\d+) "
    r"quat=(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+) "
    r"pos=(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+)"
)

SEG_NAMES = [
    "Pelvis", "L5", "L3", "T12", "T8", "Neck", "Head",
    "RShoulder", "RUpperArm", "RForearm", "RHand",
    "LShoulder", "LUpperArm", "LForearm", "LHand",
    "RUpperLeg", "RLowerLeg", "RFoot", "RToe",
    "LUpperLeg", "LLowerLeg", "LFoot", "LToe",
]


def quat_angle_deg(q):
    w = max(-1.0, min(1.0, abs(q[0])))
    return 2.0 * math.degrees(math.acos(w))


def quat_diff_deg(a, b):
    d = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]
    d = min(1.0, max(-1.0, abs(d)))
    return 2.0 * math.degrees(math.acos(d))


def parse_send(path):
    out = defaultdict(dict)
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = SEND_RE.match(line)
            if not m:
                continue
            sample = int(m.group(2))
            wire = int(m.group(3)) if m.group(3) else sample
            seg = int(m.group(4))
            q = tuple(float(m.group(i)) for i in (5, 6, 7, 8))
            p = tuple(float(m.group(i)) for i in (9, 10, 11))
            out[wire][seg] = (q, p, sample)
    return out


def parse_recv(path):
    out = defaultdict(dict)
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = RECV_RE.match(line)
            if not m:
                continue
            sample = int(m.group(2))
            seg = int(m.group(3))
            q = tuple(float(m.group(i)) for i in (4, 5, 6, 7))
            p = tuple(float(m.group(i)) for i in (8, 9, 10))
            out[sample][seg] = (q, p)
    return out


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    sends = parse_send(sys.argv[1])
    recvs = parse_recv(sys.argv[2])

    common = sorted(set(sends.keys()) & set(recvs.keys()))
    print(f"send frames: {len(sends)}  recv frames: {len(recvs)}  paired: {len(common)}")
    if not common:
        print("No matching samples; cannot verify.")
        return

    max_dq = [0.0] * 23
    max_dp = [0.0] * 23
    seg_seen = [0] * 23
    for s in common:
        sframe = sends[s]
        rframe = recvs[s]
        for seg in range(23):
            if seg not in sframe or seg not in rframe:
                continue
            sq, sp, _ = sframe[seg]
            rq, rp = rframe[seg]
            dq = quat_diff_deg(sq, rq)
            dp = math.sqrt(sum((sp[i] - rp[i]) ** 2 for i in range(3)))
            if dq > max_dq[seg]:
                max_dq[seg] = dq
            if dp > max_dp[seg]:
                max_dp[seg] = dp
            seg_seen[seg] += 1

    print(f"{'seg':<3} {'name':<12} {'n':>6} {'max Δquat°':>11} {'max Δpos m':>11}")
    for seg in range(23):
        name = SEG_NAMES[seg]
        print(f"{seg:<3} {name:<12} {seg_seen[seg]:>6} "
              f"{max_dq[seg]:>11.3f} {max_dp[seg]:>11.5f}")
    worst = max(max_dq)
    worst_p = max(max_dp)
    if worst < 0.01 and worst_p < 0.001:
        print("PASS: per-frame fidelity within tolerance.")
    else:
        print(f"WARN: max Δquat={worst:.3f}° max Δpos={worst_p:.4f} m")


if __name__ == "__main__":
    main()
