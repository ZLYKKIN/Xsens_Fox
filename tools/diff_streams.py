"""diff_streams.py — cross-log diff between Fox Mocap's stdout and the
Blender plugin's alllog.txt.

The two logs talk to each other through MXTP02 UDP datagrams.  Each
segment in each frame goes through a chain of stages and either log
captures different points of that chain:

    Fox (sender)             Blender plugin (receiver)
    ────────────────         ────────────────────────────────────────
    [send]            ───►   [raw]   datagram received
                              │
                              ▼
                             [recv]  per-segment quat+pos (wire-exact)
                              │
                              ▼
                             [axis_pos]  position after Vector(y,z,x)
                              │
                              ▼
                             [converted]  position + quaternion bundle
                              │
                              ▼
                             [rule]  per-segment quaternion permutation
                              │
                              ▼
                             [bone_world] / [bone_local]  final bone state

If the rendered skeleton in Blender does not match what Fox intended,
the chain breaks at exactly one of these stages.  This tool joins the
two logs by (sample, seg-or-name) and prints the chain in a way that
makes the divergence obvious.

Usage:
    python3 tools/diff_streams.py \
        --fox /path/to/fox-stdout.log \
        --blender /path/to/alllog.txt

Optional filters:
    --segment NAME      only this segment (e.g. Pelvis, RightHand)
    --frames N          limit to the first N matched frames
    --first-divergence  print only the first frame where stages disagree
    --t-pose-only       only consider the first frame whose [send] quats
                        are all close to identity (typical T-pose state
                        immediately after calibration)
    --rest-pose         dump the [rest_pose] table only and exit
    --tolerance EPS     numeric tolerance for "match" (default 1e-4)

The tool is stdlib-only — no numpy / scipy / Blender dependency, so it
runs anywhere Python 3.8+ runs.
"""
from __future__ import annotations

import argparse
import re
import sys
import math
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


# Numeric segment id (1..23) ↔ canonical Xsens name.
SEGMENT_ID_TO_NAME = {
    0: "Pelvis",    1: "L5",        2: "L3",         3: "T12",
    4: "T8",        5: "Neck",      6: "Head",
    7: "RightShoulder", 8: "RightUpperArm",  9: "RightForeArm", 10: "RightHand",
    11: "LeftShoulder", 12: "LeftUpperArm", 13: "LeftForeArm",  14: "LeftHand",
    15: "RightUpperLeg",16: "RightLowerLeg",17: "RightFoot",    18: "RightToe",
    19: "LeftUpperLeg", 20: "LeftLowerLeg", 21: "LeftFoot",     22: "LeftToe",
}
NAME_TO_SEGMENT_ID = {v: k for k, v in SEGMENT_ID_TO_NAME.items()}


# ─── Parsing ─────────────────────────────────────────────────────────────


@dataclass
class WireEvent:
    """Stage-1: byte-exact wire transmission.  Both Fox [send] and Blender
    [recv] populate this — they should agree when the link is clean."""
    t: float
    sample: int
    seg: int
    quat: Tuple[float, float, float, float]
    pos:  Tuple[float, float, float]


@dataclass
class AxisPosEvent:
    """Stage-2: position after the plugin's Vector(y,z,x) axis remap."""
    seg_name: str
    pos_in:  Tuple[float, float, float]
    pos_out: Tuple[float, float, float]
    rule: str


@dataclass
class ConvertedEvent:
    """Stage-3: per-segment record AFTER both position and quaternion have
    been pushed through their respective converters."""
    seg_name: str
    quat: Tuple[float, float, float, float]
    pos:  Tuple[float, float, float]


@dataclass
class RuleEvent:
    """Stage-4: per-segment quaternion remap applied by calculate_rotation."""
    bone: str
    rule: str
    q_in:  Tuple[float, float, float, float]
    q_out: Tuple[float, float, float, float]


@dataclass
class BoneApplyEvent:
    """Stage-5: actual bone state written into Blender (Pelvis carries
    location too via [bone_world])."""
    bone: str
    quat: Tuple[float, float, float, float]
    loc: Optional[Tuple[float, float, float]] = None
    scope: str = "local"   # "world" or "local"


@dataclass
class RestPoseEvent:
    """Stage-0: rest pose of each edit-bone at armature build.  This is
    what the user sees BEFORE any [recv] arrives — the "wrong starting
    pose" the diagnostic is chasing."""
    bone: str
    parent: str
    head: Tuple[float, float, float]
    tail: Tuple[float, float, float]
    roll: float
    delta_q: Tuple[float, float, float, float]


@dataclass
class Frame:
    sample: int
    # Per-segment chain — keyed by canonical segment id (0..22).  Each
    # field is populated when the relevant log event is seen.
    wire_send: Dict[int, WireEvent] = field(default_factory=dict)
    wire_recv: Dict[int, WireEvent] = field(default_factory=dict)
    axis_pos:  Dict[str, AxisPosEvent] = field(default_factory=dict)
    converted: Dict[str, ConvertedEvent] = field(default_factory=dict)
    rule:      Dict[str, RuleEvent] = field(default_factory=dict)
    applied:   Dict[str, BoneApplyEvent] = field(default_factory=dict)


# Line regexes — match what file_logger emits.
RE_SEND = re.compile(
    r"\[send\]\s+t=(?P<t>[\d.]+)\s+sample=(?P<sample>\d+)\s+seg=(?P<seg>\d+)\s+"
    r"quat=(?P<qw>-?[\d.eE+-]+),(?P<qx>-?[\d.eE+-]+),"
    r"(?P<qy>-?[\d.eE+-]+),(?P<qz>-?[\d.eE+-]+)\s+"
    r"pos=(?P<px>-?[\d.eE+-]+),(?P<py>-?[\d.eE+-]+),(?P<pz>-?[\d.eE+-]+)"
)
RE_RECV = re.compile(
    r"\[recv\]\s+t=(?P<t>[\d.]+)\s+sample=(?P<sample>\d+)\s+seg=(?P<seg>\d+)\s+"
    r"quat=(?P<qw>-?[\d.eE+-]+),(?P<qx>-?[\d.eE+-]+),"
    r"(?P<qy>-?[\d.eE+-]+),(?P<qz>-?[\d.eE+-]+)\s+"
    r"pos=(?P<px>-?[\d.eE+-]+),(?P<py>-?[\d.eE+-]+),(?P<pz>-?[\d.eE+-]+)"
)
RE_AXIS = re.compile(
    r"\[axis_pos\]\s+seg=(?P<seg>\S+)\s+"
    r"in=(?P<ix>-?[\d.eE+-]+),(?P<iy>-?[\d.eE+-]+),(?P<iz>-?[\d.eE+-]+)\s+"
    r"out=(?P<ox>-?[\d.eE+-]+),(?P<oy>-?[\d.eE+-]+),(?P<oz>-?[\d.eE+-]+)\s+"
    r"rule=(?P<rule>\S+)"
)
RE_CONVERTED = re.compile(
    r"\[converted\]\s+seg=(?P<seg>\S+)\s+"
    r"quat=(?P<qw>-?[\d.eE+-]+),(?P<qx>-?[\d.eE+-]+),"
    r"(?P<qy>-?[\d.eE+-]+),(?P<qz>-?[\d.eE+-]+)\s+"
    r"pos=(?P<px>-?[\d.eE+-]+),(?P<py>-?[\d.eE+-]+),(?P<pz>-?[\d.eE+-]+)"
)
RE_RULE = re.compile(
    r"\[rule\]\s+bone=(?P<bone>\S+)\s+rule=(?P<rule>\S+)\s+"
    r"in=(?P<iw>-?[\d.eE+-]+),(?P<ix>-?[\d.eE+-]+),"
    r"(?P<iy>-?[\d.eE+-]+),(?P<iz>-?[\d.eE+-]+)\s+"
    r"out=(?P<ow>-?[\d.eE+-]+),(?P<ox>-?[\d.eE+-]+),"
    r"(?P<oy>-?[\d.eE+-]+),(?P<oz>-?[\d.eE+-]+)"
)
RE_BONE_WORLD = re.compile(
    r"\[bone_world\]\s+bone=(?P<bone>\S+)\s+"
    r"loc=(?P<lx>-?[\d.eE+-]+),(?P<ly>-?[\d.eE+-]+),(?P<lz>-?[\d.eE+-]+)\s+"
    r"quat=(?P<qw>-?[\d.eE+-]+),(?P<qx>-?[\d.eE+-]+),"
    r"(?P<qy>-?[\d.eE+-]+),(?P<qz>-?[\d.eE+-]+)"
)
RE_BONE_LOCAL = re.compile(
    r"\[bone_local\]\s+bone=(?P<bone>\S+)\s+"
    r"quat=(?P<qw>-?[\d.eE+-]+),(?P<qx>-?[\d.eE+-]+),"
    r"(?P<qy>-?[\d.eE+-]+),(?P<qz>-?[\d.eE+-]+)"
)
RE_REST_POSE = re.compile(
    r"\[rest_pose\]\s+bone=(?P<bone>\S+)\s+parent=(?P<parent>\S+)\s+"
    r"head=(?P<hx>-?[\d.eE+-]+),(?P<hy>-?[\d.eE+-]+),(?P<hz>-?[\d.eE+-]+)\s+"
    r"tail=(?P<tx>-?[\d.eE+-]+),(?P<ty>-?[\d.eE+-]+),(?P<tz>-?[\d.eE+-]+)\s+"
    r"roll=(?P<roll>-?[\d.eE+-]+)\s+"
    r"delta_q=(?P<dw>-?[\d.eE+-]+),(?P<dx>-?[\d.eE+-]+),"
    r"(?P<dy>-?[\d.eE+-]+),(?P<dz>-?[\d.eE+-]+)"
)


def _float(s: str) -> float:
    return float(s)


def _seg_id_from_name(name: str) -> Optional[int]:
    return NAME_TO_SEGMENT_ID.get(name)


def parse_fox_log(path: Path) -> Dict[int, Frame]:
    """Extract `[send]` lines from the Fox stdout dump into Frame objects."""
    frames: Dict[int, Frame] = {}
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = RE_SEND.search(line)
            if not m:
                continue
            sample = int(m["sample"])
            seg = int(m["seg"])
            ev = WireEvent(
                t=_float(m["t"]),
                sample=sample,
                seg=seg,
                quat=(_float(m["qw"]), _float(m["qx"]), _float(m["qy"]), _float(m["qz"])),
                pos=(_float(m["px"]), _float(m["py"]), _float(m["pz"])),
            )
            frame = frames.setdefault(sample, Frame(sample=sample))
            frame.wire_send[seg] = ev
    return frames


def parse_blender_log(path: Path) -> Tuple[Dict[int, Frame], List[RestPoseEvent]]:
    """Extract every recognised event line from alllog.txt and bin them by
    sample number.  Rest-pose events are not frame-indexed (they fire once
    when the armature is built), so they come back as a flat list."""
    frames: Dict[int, Frame] = {}
    rest_pose: List[RestPoseEvent] = []
    # We need to attribute [axis_pos]/[converted]/[rule]/[bone_*] lines to
    # the most recent [recv] sample.  The plugin processes a whole MXTP02
    # packet serially: [recv] lines fire first for every segment, THEN the
    # apply path emits [axis_pos]/etc.  So tracking "current sample" by the
    # last seen [recv] is correct.
    current_sample: Optional[int] = None
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            mr = RE_RECV.search(line)
            if mr:
                sample = int(mr["sample"])
                seg = int(mr["seg"])
                ev = WireEvent(
                    t=_float(mr["t"]),
                    sample=sample,
                    seg=seg,
                    quat=(_float(mr["qw"]), _float(mr["qx"]),
                          _float(mr["qy"]), _float(mr["qz"])),
                    pos=(_float(mr["px"]), _float(mr["py"]), _float(mr["pz"])),
                )
                frame = frames.setdefault(sample, Frame(sample=sample))
                frame.wire_recv[seg] = ev
                current_sample = sample
                continue
            ma = RE_AXIS.search(line)
            if ma and current_sample is not None:
                ev2 = AxisPosEvent(
                    seg_name=ma["seg"],
                    pos_in=(_float(ma["ix"]), _float(ma["iy"]), _float(ma["iz"])),
                    pos_out=(_float(ma["ox"]), _float(ma["oy"]), _float(ma["oz"])),
                    rule=ma["rule"],
                )
                frames[current_sample].axis_pos[ma["seg"]] = ev2
                continue
            mc = RE_CONVERTED.search(line)
            if mc and current_sample is not None:
                ev3 = ConvertedEvent(
                    seg_name=mc["seg"],
                    quat=(_float(mc["qw"]), _float(mc["qx"]),
                          _float(mc["qy"]), _float(mc["qz"])),
                    pos=(_float(mc["px"]), _float(mc["py"]), _float(mc["pz"])),
                )
                frames[current_sample].converted[mc["seg"]] = ev3
                continue
            mu = RE_RULE.search(line)
            if mu and current_sample is not None:
                ev4 = RuleEvent(
                    bone=mu["bone"],
                    rule=mu["rule"],
                    q_in=(_float(mu["iw"]), _float(mu["ix"]),
                          _float(mu["iy"]), _float(mu["iz"])),
                    q_out=(_float(mu["ow"]), _float(mu["ox"]),
                           _float(mu["oy"]), _float(mu["oz"])),
                )
                frames[current_sample].rule[mu["bone"]] = ev4
                continue
            mw = RE_BONE_WORLD.search(line)
            if mw and current_sample is not None:
                ev5 = BoneApplyEvent(
                    bone=mw["bone"],
                    quat=(_float(mw["qw"]), _float(mw["qx"]),
                          _float(mw["qy"]), _float(mw["qz"])),
                    loc=(_float(mw["lx"]), _float(mw["ly"]), _float(mw["lz"])),
                    scope="world",
                )
                frames[current_sample].applied[mw["bone"]] = ev5
                continue
            ml = RE_BONE_LOCAL.search(line)
            if ml and current_sample is not None:
                ev6 = BoneApplyEvent(
                    bone=ml["bone"],
                    quat=(_float(ml["qw"]), _float(ml["qx"]),
                          _float(ml["qy"]), _float(ml["qz"])),
                    scope="local",
                )
                frames[current_sample].applied[ml["bone"]] = ev6
                continue
            mp = RE_REST_POSE.search(line)
            if mp:
                rest_pose.append(RestPoseEvent(
                    bone=mp["bone"],
                    parent=mp["parent"],
                    head=(_float(mp["hx"]), _float(mp["hy"]), _float(mp["hz"])),
                    tail=(_float(mp["tx"]), _float(mp["ty"]), _float(mp["tz"])),
                    roll=_float(mp["roll"]),
                    delta_q=(_float(mp["dw"]), _float(mp["dx"]),
                             _float(mp["dy"]), _float(mp["dz"])),
                ))
    return frames, rest_pose


# ─── Comparison primitives ───────────────────────────────────────────────


def vec_dist(a: Tuple[float, ...], b: Tuple[float, ...]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def quat_dot_abs(a: Tuple[float, float, float, float],
                 b: Tuple[float, float, float, float]) -> float:
    """|a · b|.  For unit quats, ≥ cos(half_angle) — i.e. close to 1 means
    a and b represent the same rotation (modulo hemisphere flip)."""
    d = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]
    return abs(d)


def quat_angle_deg(a: Tuple[float, float, float, float],
                   b: Tuple[float, float, float, float]) -> float:
    """Geodesic angle between rotations represented by a and b."""
    d = min(1.0, quat_dot_abs(a, b))
    return 2.0 * math.degrees(math.acos(d))


def is_identity_quat(q: Tuple[float, float, float, float], eps: float = 1e-3) -> bool:
    return abs(q[0] - 1.0) < eps and abs(q[1]) < eps and abs(q[2]) < eps and abs(q[3]) < eps


# ─── Diff printers ───────────────────────────────────────────────────────


def fmt_quat(q: Tuple[float, float, float, float]) -> str:
    return f"({q[0]:+.5f},{q[1]:+.5f},{q[2]:+.5f},{q[3]:+.5f})"


def fmt_vec(v: Tuple[float, float, float]) -> str:
    return f"({v[0]:+.5f},{v[1]:+.5f},{v[2]:+.5f})"


def stage_chain(frame: Frame, seg_id: int, tol: float) -> List[Tuple[str, str, bool]]:
    """Build a list of (stage_label, value_string, ok_flag) for one segment
    in one frame.  ok_flag is True when the stage's output is consistent
    with the previous stage given the documented transformation.
    """
    seg_name = SEGMENT_ID_TO_NAME[seg_id]
    rows: List[Tuple[str, str, bool]] = []

    send = frame.wire_send.get(seg_id)
    recv = frame.wire_recv.get(seg_id)
    axis = frame.axis_pos.get(seg_name)
    conv = frame.converted.get(seg_name)
    rule = frame.rule.get(seg_name)
    appl = frame.applied.get(seg_name)

    if send is not None:
        rows.append(("[send] WIRE", f"q={fmt_quat(send.quat)} p={fmt_vec(send.pos)}", True))
    else:
        rows.append(("[send] WIRE", "<missing>", False))

    if recv is None:
        rows.append(("[recv] WIRE", "<missing>", False))
    elif send is not None:
        wire_q_match = vec_dist(send.quat, recv.quat) < tol
        wire_p_match = vec_dist(send.pos,  recv.pos)  < tol
        ok = wire_q_match and wire_p_match
        rows.append(("[recv] WIRE",
                     f"q={fmt_quat(recv.quat)} p={fmt_vec(recv.pos)}"
                     + ("" if ok else "  ← MISMATCH vs send"),
                     ok))
    else:
        rows.append(("[recv] WIRE",
                     f"q={fmt_quat(recv.quat)} p={fmt_vec(recv.pos)}", True))

    if axis is not None and recv is not None:
        # Plugin's documented rule: Vector(y, z, x).  Verify the line actually
        # follows it given the [recv] position.
        expected_out = (recv.pos[1], recv.pos[2], recv.pos[0])
        ok = vec_dist(axis.pos_out, expected_out) < tol
        rows.append(("[axis_pos] CONVERT",
                     f"in={fmt_vec(axis.pos_in)} out={fmt_vec(axis.pos_out)} rule={axis.rule}"
                     + ("" if ok else "  ← rule does not match (y,z,x)"),
                     ok))
    elif axis is not None:
        rows.append(("[axis_pos] CONVERT",
                     f"in={fmt_vec(axis.pos_in)} out={fmt_vec(axis.pos_out)} rule={axis.rule}",
                     True))
    else:
        rows.append(("[axis_pos] CONVERT", "<missing>", False))

    if conv is not None and recv is not None:
        # Quaternion is taken verbatim from the wire (no axis remap at this
        # stage in the plugin) — verify that.
        ok = vec_dist(conv.quat, recv.quat) < tol
        rows.append(("[converted] BUNDLE",
                     f"q={fmt_quat(conv.quat)} p={fmt_vec(conv.pos)}"
                     + ("" if ok else "  ← quat changed at convert stage"),
                     ok))
    elif conv is not None:
        rows.append(("[converted] BUNDLE",
                     f"q={fmt_quat(conv.quat)} p={fmt_vec(conv.pos)}", True))
    else:
        rows.append(("[converted] BUNDLE", "<missing>", False))

    if rule is not None:
        # Sanity: rule.q_in should equal converted.quat (or its hemisphere
        # flip for child bones where the parent_inv has been pre-applied).
        rows.append(("[rule] REMAP",
                     f"rule={rule.rule} in={fmt_quat(rule.q_in)} out={fmt_quat(rule.q_out)}",
                     True))
    else:
        rows.append(("[rule] REMAP", "<missing>", False))

    if appl is not None:
        loc = "" if appl.loc is None else f"  loc={fmt_vec(appl.loc)}"
        rows.append((f"[bone_{appl.scope}] APPLY",
                     f"q={fmt_quat(appl.quat)}{loc}", True))
    else:
        rows.append(("[bone_*] APPLY", "<missing>", False))

    return rows


def print_frame_chain(frame: Frame, segments: List[int], tol: float, stream=None) -> None:
    if stream is None:
        stream = sys.stdout
    stream.write(f"\n────── sample {frame.sample} ──────\n")
    for seg in segments:
        seg_name = SEGMENT_ID_TO_NAME[seg]
        send = frame.wire_send.get(seg)
        recv = frame.wire_recv.get(seg)
        # Skip segments with neither send nor recv (e.g. unsensored spine
        # which Fox synthesises but the wire still carries identity for —
        # so [send] is present; if the user filtered to one segment we
        # still want to print).
        if send is None and recv is None and len(segments) > 1:
            continue
        stream.write(f"  seg {seg:2d} ({seg_name})\n")
        for stage, value, ok in stage_chain(frame, seg, tol):
            mark = "✓" if ok else "✗"
            stream.write(f"    {mark} {stage:24s} {value}\n")


def first_divergence(frame: Frame, seg: int, tol: float) -> Optional[str]:
    """Return the label of the first stage in the chain whose output does
    not match the prior stage.  None if everything aligns.  Stages with
    missing data (no log line for that stage) are skipped — they're not
    a divergence, just an absence (e.g. an unsensored segment has no
    [send] line in some Fox builds)."""
    for stage, value, ok in stage_chain(frame, seg, tol):
        if ok:
            continue
        if "missing" in value:
            continue
        return stage
    return None


def t_pose_quat_summary(frame: Frame, tol_deg: float = 0.5) -> Dict[str, int]:
    """How many [send] quats are within `tol_deg` of identity at this frame.
    Frames where this is ≥ 22 are essentially T-pose (just-after-calibration)
    snapshots and are the cleanest place to investigate the rest-pose chain
    — there's no IMU noise mixed into the data."""
    near_id_send = 0
    near_id_appl = 0
    for seg_id in range(23):
        s = frame.wire_send.get(seg_id)
        if s is not None and is_identity_quat(s.quat):
            near_id_send += 1
        seg_name = SEGMENT_ID_TO_NAME[seg_id]
        a = frame.applied.get(seg_name)
        if a is not None and is_identity_quat(a.quat):
            near_id_appl += 1
    return {"send_identity": near_id_send, "applied_identity": near_id_appl}


def print_rest_pose_table(rest_pose: List[RestPoseEvent], stream=None) -> None:
    if stream is None:
        stream = sys.stdout
    stream.write("\n═══════ Rest pose (armature edit-bone state at build) ═══════\n")
    stream.write(f"{'bone':<20} {'parent':<20} {'head (Blender)':<32} {'tail':<32} {'roll':>8} {'delta_q':<40}\n")
    for ev in rest_pose:
        stream.write(
            f"{ev.bone:<20} {ev.parent:<20} "
            f"{fmt_vec(ev.head):<32} {fmt_vec(ev.tail):<32} "
            f"{ev.roll:+8.4f} {fmt_quat(ev.delta_q):<40}\n"
        )
    if not rest_pose:
        stream.write("  <no [rest_pose] events found — armature was not built during this log window>\n")


# ─── CLI ──────────────────────────────────────────────────────────────────


def _parse_segments_filter(arg: str) -> List[int]:
    if not arg:
        return list(range(23))
    if arg in NAME_TO_SEGMENT_ID:
        return [NAME_TO_SEGMENT_ID[arg]]
    raise SystemExit(
        f"unknown segment '{arg}'.  Valid names: {sorted(NAME_TO_SEGMENT_ID)}")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Diff Fox Mocap [send] vs Blender plugin alllog.txt")
    ap.add_argument("--fox", required=False,
                    help="path to Fox stdout dump (containing [send] lines)")
    ap.add_argument("--blender", required=True,
                    help="path to Blender plugin alllog.txt")
    ap.add_argument("--segment", default="",
                    help="filter to one segment (e.g. Pelvis, RightHand)")
    ap.add_argument("--frames", type=int, default=0,
                    help="limit to first N matched frames (0 = all)")
    ap.add_argument("--first-divergence", action="store_true",
                    help="only print the first frame where stages disagree")
    ap.add_argument("--t-pose-only", action="store_true",
                    help="only print frames whose [send] quats are all near identity")
    ap.add_argument("--rest-pose", action="store_true",
                    help="dump the [rest_pose] table only and exit")
    ap.add_argument("--tolerance", type=float, default=1e-4,
                    help="numeric tolerance for stage-to-stage match (default 1e-4)")
    ap.add_argument("--summary", action="store_true",
                    help="just print a one-line summary per frame, no chain")
    args = ap.parse_args(argv)

    blender_path = Path(args.blender)
    if not blender_path.exists():
        print(f"error: blender log not found at {blender_path}", file=sys.stderr)
        return 2
    blender_frames, rest_pose = parse_blender_log(blender_path)

    if args.rest_pose:
        print_rest_pose_table(rest_pose)
        return 0

    if args.fox:
        fox_path = Path(args.fox)
        if not fox_path.exists():
            print(f"error: fox log not found at {fox_path}", file=sys.stderr)
            return 2
        fox_frames = parse_fox_log(fox_path)
        # Merge Fox's [send] into the Blender frame dict.  The sample number
        # is the join key.
        for sample, ffr in fox_frames.items():
            tgt = blender_frames.setdefault(sample, Frame(sample=sample))
            tgt.wire_send.update(ffr.wire_send)

    segments = _parse_segments_filter(args.segment)
    sorted_samples = sorted(blender_frames.keys())

    if args.summary:
        print(f"{'sample':>10} {'wire_match':>12} {'apply_match_id':>16} {'first_div':<24}")
        printed = 0
        for s in sorted_samples:
            fr = blender_frames[s]
            wire = 0
            for seg in segments:
                a = fr.wire_send.get(seg)
                b = fr.wire_recv.get(seg)
                if a and b and vec_dist(a.quat, b.quat) < args.tolerance \
                            and vec_dist(a.pos, b.pos) < args.tolerance:
                    wire += 1
            sum_t = t_pose_quat_summary(fr)
            # First divergent stage across all segments.
            divs = [first_divergence(fr, seg, args.tolerance) for seg in segments]
            divs = [d for d in divs if d]
            first_d = divs[0] if divs else ""
            print(f"{s:>10} {wire:>12} {sum_t['applied_identity']:>16} {first_d:<24}")
            printed += 1
            if args.frames and printed >= args.frames:
                break
        return 0

    printed = 0
    for s in sorted_samples:
        fr = blender_frames[s]
        if args.t_pose_only:
            if t_pose_quat_summary(fr)["send_identity"] < 22:
                continue
        if args.first_divergence:
            divs = [first_divergence(fr, seg, args.tolerance) for seg in segments]
            divs = [d for d in divs if d]
            if not divs:
                continue
        print_frame_chain(fr, segments, args.tolerance)
        printed += 1
        if args.frames and printed >= args.frames:
            break

    if rest_pose:
        print_rest_pose_table(rest_pose)

    if printed == 0:
        print("\n(no matched frames printed — try removing --first-divergence "
              "or --t-pose-only)", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
