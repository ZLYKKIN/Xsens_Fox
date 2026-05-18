"""Self-test for tools/diff_streams.py.

The diff tool is used to cross-check Fox Mocap's `[send]` lines against
the Blender plugin's `alllog.txt`.  If the tool itself is broken, the
user can't trust any "find the bug" conclusions drawn from its output.

This file builds two minimal synthetic logs (one clean, one with
intentional wire and convert-rule mismatches) and asserts that the tool
correctly classifies each case.
"""
import io
import os
import sys
import tempfile
from pathlib import Path


HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools"))

import diff_streams as DS   # noqa: E402


# ───────────── fixtures ─────────────


CLEAN_FOX = """\
[send] t=1700000000.000000 sample=42 seg=0 quat=1.000000,0.000000,0.000000,0.000000 pos=0.000000,0.000000,0.000000
[send] t=1700000000.000001 sample=42 seg=4 quat=1.000000,0.000000,0.000000,0.000000 pos=0.000000,0.000000,0.000000
[send] t=1700000000.000002 sample=42 seg=10 quat=1.000000,0.000000,0.000000,0.000000 pos=0.000000,-0.600000,0.000000
[send] t=1700000000.011111 sample=43 seg=0 quat=0.998000,0.063000,0.000000,0.000000 pos=0.000000,0.000000,0.000000
"""

CLEAN_BLENDER = """\
[boot] alllog opened
[schema] version=2
[rest_pose] bone=Pelvis parent=- head=0.000000,0.000000,0.960000 tail=0.000000,0.000000,1.010000 roll=0.000000 delta_q=0.707107,0.707107,0.000000,0.000000
[rest_pose] bone=RightHand parent=RightForeArm head=-0.600000,0.000000,1.400000 tail=-0.650000,0.000000,1.400000 roll=0.000000 delta_q=0.500000,0.500000,0.500000,0.500000
[recv] t=1700000000.001001 sample=42 seg=0 quat=1.000000,0.000000,0.000000,0.000000 pos=0.000000,0.000000,0.000000
[recv] t=1700000000.001002 sample=42 seg=4 quat=1.000000,0.000000,0.000000,0.000000 pos=0.000000,0.000000,0.000000
[recv] t=1700000000.001003 sample=42 seg=10 quat=1.000000,0.000000,0.000000,0.000000 pos=0.000000,-0.600000,0.000000
[axis_pos] seg=Pelvis in=0.000000,0.000000,0.000000 out=0.000000,0.000000,0.000000 rule=Vector(y,z,x)
[axis_pos] seg=RightHand in=0.000000,-0.600000,0.000000 out=-0.600000,0.000000,0.000000 rule=Vector(y,z,x)
[converted] seg=Pelvis quat=1.000000,0.000000,0.000000,0.000000 pos=0.000000,0.000000,0.000000
[converted] seg=RightHand quat=1.000000,0.000000,0.000000,0.000000 pos=-0.600000,0.000000,0.000000
[rule] bone=Pelvis rule=spine_wxyz->w,y,z,x in=1.000000,0.000000,0.000000,0.000000 out=1.000000,0.000000,0.000000,0.000000
[bone_world] bone=Pelvis loc=0.000000,0.000000,0.000000 quat=1.000000,0.000000,0.000000,0.000000
[rule] bone=RightHand rule=rarm_wxyz->w,-x,-y,z in=1.000000,0.000000,0.000000,0.000000 out=1.000000,0.000000,0.000000,0.000000
[bone_local] bone=RightHand quat=1.000000,0.000000,0.000000,0.000000
[recv] t=1700000000.012001 sample=43 seg=0 quat=0.998000,0.063000,0.000000,0.000000 pos=0.000000,0.000000,0.000000
[converted] seg=Pelvis quat=0.998000,0.063000,0.000000,0.000000 pos=0.000000,0.000000,0.000000
[rule] bone=Pelvis rule=spine_wxyz->w,y,z,x in=0.998000,0.063000,0.000000,0.000000 out=0.998000,0.000000,0.000000,0.063000
[bone_world] bone=Pelvis loc=0.000000,0.010000,0.000000 quat=0.998000,0.000000,0.000000,0.063000
"""

# Wire mismatch: recv quat differs from send quat for sample 42 seg 0
WIRE_MISMATCH_BLENDER = """\
[boot] alllog opened
[recv] t=1700000000.001001 sample=42 seg=0 quat=0.500000,0.500000,0.000000,0.000000 pos=0.000000,0.000000,0.000000
[axis_pos] seg=Pelvis in=0.000000,0.000000,0.000000 out=0.000000,0.000000,0.000000 rule=Vector(y,z,x)
[converted] seg=Pelvis quat=0.500000,0.500000,0.000000,0.000000 pos=0.000000,0.000000,0.000000
[rule] bone=Pelvis rule=spine_wxyz->w,y,z,x in=0.500000,0.500000,0.000000,0.000000 out=0.500000,0.000000,0.000000,0.500000
[bone_world] bone=Pelvis loc=0.000000,0.000000,0.000000 quat=0.500000,0.000000,0.000000,0.500000
"""

# Axis rule violation: [axis_pos] out doesn't match Vector(y, z, x) of recv.pos
RULE_MISMATCH_BLENDER = """\
[boot] alllog opened
[recv] t=1700000000.001003 sample=42 seg=10 quat=1.000000,0.000000,0.000000,0.000000 pos=0.000000,-0.600000,0.000000
[axis_pos] seg=RightHand in=0.000000,-0.600000,0.000000 out=0.999999,0.000000,0.000000 rule=Vector(y,z,x)
[converted] seg=RightHand quat=1.000000,0.000000,0.000000,0.000000 pos=0.999999,0.000000,0.000000
"""


def _tmpfile(content: str) -> Path:
    tmp = tempfile.NamedTemporaryFile(
        delete=False, mode="w", encoding="utf-8", suffix=".log")
    tmp.write(content)
    tmp.close()
    return Path(tmp.name)


# ───────────── tests ─────────────


def test_parser_picks_up_send_lines():
    fox = _tmpfile(CLEAN_FOX)
    frames = DS.parse_fox_log(fox)
    assert 42 in frames and 43 in frames
    assert 0 in frames[42].wire_send
    assert frames[42].wire_send[0].quat == (1.0, 0.0, 0.0, 0.0)
    assert frames[42].wire_send[10].pos == (0.0, -0.6, 0.0)


def test_parser_picks_up_all_blender_events():
    blender = _tmpfile(CLEAN_BLENDER)
    frames, rest_pose = DS.parse_blender_log(blender)
    # Rest pose: two bones.
    assert {ev.bone for ev in rest_pose} == {"Pelvis", "RightHand"}
    # Sample 42 has all chain stages present.
    f42 = frames[42]
    assert 0 in f42.wire_recv and 4 in f42.wire_recv and 10 in f42.wire_recv
    assert "Pelvis" in f42.axis_pos
    assert "Pelvis" in f42.converted
    assert "Pelvis" in f42.rule
    assert "Pelvis" in f42.applied
    assert f42.applied["Pelvis"].scope == "world"
    assert f42.applied["RightHand"].scope == "local"


def test_clean_log_reports_no_divergence():
    fox = _tmpfile(CLEAN_FOX)
    blender = _tmpfile(CLEAN_BLENDER)
    frames, _ = DS.parse_blender_log(blender)
    fox_frames = DS.parse_fox_log(fox)
    for s, ffr in fox_frames.items():
        frames.setdefault(s, DS.Frame(sample=s)).wire_send.update(ffr.wire_send)
    # Sample 42 segments 0, 4, 10 should have NO divergence anywhere.
    for seg in (0, 4, 10):
        assert DS.first_divergence(frames[42], seg, tol=1e-4) is None, \
            f"clean log should not flag seg {seg}"


def test_wire_mismatch_detected_at_recv_stage():
    fox = _tmpfile(CLEAN_FOX)
    blender = _tmpfile(WIRE_MISMATCH_BLENDER)
    frames, _ = DS.parse_blender_log(blender)
    fox_frames = DS.parse_fox_log(fox)
    for s, ffr in fox_frames.items():
        frames.setdefault(s, DS.Frame(sample=s)).wire_send.update(ffr.wire_send)
    div = DS.first_divergence(frames[42], seg=0, tol=1e-4)
    assert div == "[recv] WIRE", f"expected wire-stage flag, got {div!r}"


def test_axis_rule_mismatch_detected():
    fox = _tmpfile(CLEAN_FOX)
    blender = _tmpfile(RULE_MISMATCH_BLENDER)
    frames, _ = DS.parse_blender_log(blender)
    fox_frames = DS.parse_fox_log(fox)
    for s, ffr in fox_frames.items():
        frames.setdefault(s, DS.Frame(sample=s)).wire_send.update(ffr.wire_send)
    # Wire is fine (we used send=clean, recv=same).  But axis_pos out
    # doesn't follow Vector(y,z,x) of recv.pos, so the axis stage fires.
    div = DS.first_divergence(frames[42], seg=10, tol=1e-4)
    assert div == "[axis_pos] CONVERT", f"expected axis-stage flag, got {div!r}"


def test_t_pose_summary_counts_identity_send_quats():
    fox = _tmpfile(CLEAN_FOX)
    blender = _tmpfile(CLEAN_BLENDER)
    frames, _ = DS.parse_blender_log(blender)
    fox_frames = DS.parse_fox_log(fox)
    for s, ffr in fox_frames.items():
        frames.setdefault(s, DS.Frame(sample=s)).wire_send.update(ffr.wire_send)
    # Sample 42: all 3 [send] quats are identity → count = 3.
    summary = DS.t_pose_quat_summary(frames[42])
    assert summary["send_identity"] == 3, summary
    # Sample 43: only seg 0 [send] arrived AND it's a 7° tilt (not identity).
    summary = DS.t_pose_quat_summary(frames[43])
    assert summary["send_identity"] == 0


def test_cli_summary_runs_clean():
    """End-to-end smoke: main() with --summary on clean logs prints rows
    and exits 0."""
    fox = _tmpfile(CLEAN_FOX)
    blender = _tmpfile(CLEAN_BLENDER)
    # Redirect stdout so the print doesn't fight pytest's capture.
    captured = io.StringIO()
    old_stdout = sys.stdout
    sys.stdout = captured
    try:
        rc = DS.main(["--fox", str(fox), "--blender", str(blender), "--summary"])
    finally:
        sys.stdout = old_stdout
    assert rc == 0
    text = captured.getvalue()
    assert "sample" in text
    assert "42" in text and "43" in text


def test_cli_rest_pose_runs_clean():
    blender = _tmpfile(CLEAN_BLENDER)
    captured = io.StringIO()
    old_stdout = sys.stdout
    sys.stdout = captured
    try:
        rc = DS.main(["--blender", str(blender), "--rest-pose"])
    finally:
        sys.stdout = old_stdout
    assert rc == 0
    assert "Pelvis" in captured.getvalue()
    assert "RightHand" in captured.getvalue()


if __name__ == "__main__":
    test_parser_picks_up_send_lines()
    test_parser_picks_up_all_blender_events()
    test_clean_log_reports_no_divergence()
    test_wire_mismatch_detected_at_recv_stage()
    test_axis_rule_mismatch_detected()
    test_t_pose_summary_counts_identity_send_quats()
    test_cli_summary_runs_clean()
    test_cli_rest_pose_runs_clean()
    print("test_diff_streams: PASS")
