#!/usr/bin/env python3
"""Verify our software's world frame against logged T-pose accelerometer samples.
Reads the [calib] T-pose snapshot from fox_mocap.log and checks that gravity
points to -Z in our world frame (Z-up convention) for every sensored segment.
Usage: python tools/verify_axes.py logs/fox_mocap.log
"""
import re
import sys
import math

QUAT_RE = re.compile(
    r"quat\s*=\s*\(\s*(-?[0-9.]+)\s*,\s*(-?[0-9.]+)\s*,\s*(-?[0-9.]+)\s*,\s*(-?[0-9.]+)\s*\)"
)
ACC_RE = re.compile(
    r"acc\s*=\s*\(\s*(-?[0-9.]+)\s*,\s*(-?[0-9.]+)\s*,\s*(-?[0-9.]+)\s*\)\s*\|\s*(-?[0-9.]+)g"
)
SEG_RE = re.compile(r"seg\[\s*\d+\]\s+(pelvis|t8|head|r_shoulder|r_upper_arm|r_forearm|r_hand|"
                    r"l_shoulder|l_upper_arm|l_forearm|l_hand|"
                    r"r_upper_leg|r_lower_leg|r_foot|"
                    r"l_upper_leg|l_lower_leg|l_foot)\b.*live")


def quat_rotate(q, v):
    w, x, y, z = q
    qv = (x, y, z)
    t = (
        2.0 * (qv[1] * v[2] - qv[2] * v[1]),
        2.0 * (qv[2] * v[0] - qv[0] * v[2]),
        2.0 * (qv[0] * v[1] - qv[1] * v[0]),
    )
    return (
        v[0] + w * t[0] + (qv[1] * t[2] - qv[2] * t[1]),
        v[1] + w * t[1] + (qv[2] * t[0] - qv[0] * t[2]),
        v[2] + w * t[2] + (qv[0] * t[1] - qv[1] * t[0]),
    )


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    in_snapshot = False
    cur_seg = None
    cur_acc = None
    rows = []
    with open(sys.argv[1], "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if "[FUSED SNAPSHOT]" in line:
                in_snapshot = True
                continue
            if line.startswith("===") or line.startswith("---"):
                continue
            if not in_snapshot:
                continue
            m = SEG_RE.search(line)
            if m:
                cur_seg = m.group(1)
                cur_acc = None
                ma = ACC_RE.search(line)
                if ma:
                    cur_acc = (float(ma.group(1)), float(ma.group(2)), float(ma.group(3)))
                continue
            mq = QUAT_RE.search(line)
            if mq and cur_seg and cur_acc is not None:
                cur_quat = tuple(float(mq.group(i)) for i in (1, 2, 3, 4))
                rows.append((cur_seg, cur_quat, cur_acc))
                cur_seg = None
                cur_acc = None
            if len(rows) >= 17:
                break
    if not rows:
        print("No T-pose samples found in log.")
        return
    print(f"Found {len(rows)} T-pose samples. World gravity reconstructed per segment:")
    print(f"{'segment':<14} {'q_w':>7} {'q_x':>7} {'q_y':>7} {'q_z':>7} "
          f"{'gW_x':>7} {'gW_y':>7} {'gW_z':>7} {'|gW|':>6}")
    all_zup = True
    for seg, q, a in rows:
        gW = quat_rotate(q, a)
        mag = math.sqrt(gW[0]**2 + gW[1]**2 + gW[2]**2)
        zish = (abs(gW[2]) > 0.7 * mag) and (gW[2] < 0)
        if not zish:
            all_zup = False
        print(f"{seg:<14} {q[0]:>7.3f} {q[1]:>7.3f} {q[2]:>7.3f} {q[3]:>7.3f} "
              f"{gW[0]:>7.3f} {gW[1]:>7.3f} {gW[2]:>7.3f} {mag:>6.3f}")
    if all_zup:
        print("PASS: gravity points to world -Z for every segment (Z-up confirmed).")
    else:
        print("WARN: at least one segment's world gravity is not aligned with -Z.")


if __name__ == "__main__":
    main()
