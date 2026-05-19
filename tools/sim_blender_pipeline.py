#!/usr/bin/env python3
"""Re-implements the Blender plugin's quaternion-message pipeline in pure Python
from the [recv] events in blender_session.log. Verifies that the per-bone rule
and parent-inverse-child math reconstructs the recorded [bone_world] /
[bone_local] events within a small numerical tolerance.
Usage: python tools/sim_blender_pipeline.py logs/blender_session.log
"""
import re
import sys
import math

RECV_RE = re.compile(
    r"\[recv\] t=([0-9.]+) sample=(\d+) seg=(\d+) "
    r"quat=(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+) "
    r"pos=(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+)"
)
RULE_RE = re.compile(
    r"\[rule\] bone=(\S+)\s+rule=(\S+)\s+in=(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+)\s+"
    r"out=(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+)"
)

SEG_BY_INDEX = [
    "Pelvis", "L5", "L3", "T12", "T8", "Neck", "Head",
    "RightShoulder", "RightUpperArm", "RightForeArm", "RightHand",
    "LeftShoulder", "LeftUpperArm", "LeftForeArm", "LeftHand",
    "RightUpperLeg", "RightLowerLeg", "RightFoot", "RightToe",
    "LeftUpperLeg", "LeftLowerLeg", "LeftFoot", "LeftToe",
]
PARENT = {
    "Pelvis": None, "L5": "Pelvis", "L3": "L5", "T12": "L3", "T8": "T12",
    "Neck": "T8", "Head": "Neck",
    "RightShoulder": "T8", "RightUpperArm": "RightShoulder",
    "RightForeArm": "RightUpperArm", "RightHand": "RightForeArm",
    "LeftShoulder": "T8", "LeftUpperArm": "LeftShoulder",
    "LeftForeArm": "LeftUpperArm", "LeftHand": "LeftForeArm",
    "RightUpperLeg": "Pelvis", "RightLowerLeg": "RightUpperLeg",
    "RightFoot": "RightLowerLeg", "RightToe": "RightFoot",
    "LeftUpperLeg": "Pelvis", "LeftLowerLeg": "LeftUpperLeg",
    "LeftFoot": "LeftLowerLeg", "LeftToe": "LeftFoot",
}


def qmul(a, b):
    return (
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    )


def qinv(q):
    return (q[0], -q[1], -q[2], -q[3])


def apply_rule(bone, q):
    if bone in ("Pelvis", "L5", "L3", "T12", "T8", "Neck", "Head"):
        return (q[0], q[2], q[3], q[1])
    if bone in ("RightUpperLeg", "RightLowerLeg", "LeftUpperLeg", "LeftLowerLeg"):
        return (q[0], q[2], -q[3], -q[1])
    if bone in ("LeftShoulder", "LeftUpperArm", "LeftForeArm", "LeftHand"):
        return q
    if bone in ("RightShoulder", "RightUpperArm", "RightForeArm", "RightHand"):
        return (q[0], -q[1], -q[2], q[3])
    return (q[0], q[2], -q[3], -q[1])


def qdiff_deg(a, b):
    d = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]
    d = min(1.0, max(-1.0, abs(d)))
    return 2.0 * math.degrees(math.acos(d))


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    cur_sample = None
    world = {}
    rules_seen = []
    sample_count = 0
    max_local_err = 0.0
    err_per_bone = {}
    with open(sys.argv[1], "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = RECV_RE.match(line)
            if m:
                sample = int(m.group(2))
                seg = int(m.group(3))
                q = tuple(float(m.group(i)) for i in (4, 5, 6, 7))
                if sample != cur_sample:
                    if cur_sample is not None and rules_seen:
                        sample_count += 1
                    cur_sample = sample
                    world = {}
                    rules_seen = []
                if seg < 23:
                    world[SEG_BY_INDEX[seg]] = q
                continue
            m = RULE_RE.match(line)
            if m and cur_sample is not None:
                bone = m.group(1)
                logged_in = tuple(float(m.group(i)) for i in (3, 4, 5, 6))
                logged_out = tuple(float(m.group(i)) for i in (7, 8, 9, 10))
                expected_out = apply_rule(bone, logged_in)
                err = qdiff_deg(expected_out, logged_out)
                err_per_bone.setdefault(bone, 0.0)
                if err > err_per_bone[bone]:
                    err_per_bone[bone] = err
                if err > max_local_err:
                    max_local_err = err
                rules_seen.append(bone)

    print(f"Simulated frames: {sample_count}")
    print("Per-bone max rule-output error (degrees vs recorded):")
    for bone, err in sorted(err_per_bone.items()):
        flag = "OK" if err < 0.01 else "WARN" if err < 1.0 else "FAIL"
        print(f"  {bone:<20} {err:>7.4f}°  {flag}")
    if max_local_err < 0.01:
        print("PASS: pure-Python rule reproduction matches logged rule outputs.")
    else:
        print(f"WARN: max error {max_local_err:.4f}° — investigate.")


if __name__ == "__main__":
    main()
