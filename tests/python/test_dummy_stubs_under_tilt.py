"""§A test: scapular and hip dummy stubs in SkeletonXsens::addDummySegments
must follow the FULL pelvis/T8 orientation, not just its yaw projection.

Mirrors the math in scr/main.cpp:443-470.  Under side-lean or combined
tilt+yaw the current yaw_only_quat-based implementation drops the
tilt component, displacing the scapular/hip stub by tens of degrees.
The fix composes the full oriented quaternion of T8/Pelvis with the
existing Euler factor instead.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qrot, axangle, euler_xyz, PI


def yaw_only(q):
    """Mirror of scr/main.cpp:603 yaw_only_quat."""
    w, x, y, z = q
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    n2 = w*w + z*z
    if n2 < 1e-24:
        return np.array([1.0, 0.0, 0.0, 0.0])
    n = 1.0 / np.sqrt(n2)
    return np.array([w*n, 0.0, 0.0, z*n])


# Default T-pose angle for pelvis / T8 (both = Rot_y(-π/2) in tpose).
DEF_ANG_PELVIS = euler_xyz(0, -PI/2, 0)
DEF_ANG_T8     = euler_xyz(0, -PI/2, 0)

# Euler factor that the existing code applies on top of yaw — taken
# verbatim from scr/main.cpp:453-456.
SCAP_R_EULER = euler_xyz(0, -PI/2, -PI/2)
SCAP_L_EULER = euler_xyz(0, -PI/2,  PI/2)
HIP_R_EULER  = euler_xyz(0,    0,  -PI/2)
HIP_L_EULER  = euler_xyz(0,    0,   PI/2)


def stub_dir_current(raw, def_ang, euler_factor):
    """Reproduce the current yaw_only_quat-based stub direction."""
    oriented = qnorm(qmul(raw, def_ang))
    return qrot(qnorm(qmul(yaw_only(oriented), euler_factor)),
                np.array([1.0, 0.0, 0.0]))


def stub_dir_fixed(raw, def_ang, euler_factor):
    """Proposed full-orientation composition — keeps tilt in the stub."""
    oriented = qnorm(qmul(raw, def_ang))
    return qrot(qnorm(qmul(oriented, euler_factor)),
                np.array([1.0, 0.0, 0.0]))


def ang_deg(a, b):
    d = float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)),
                      -1.0, 1.0))
    return float(np.degrees(np.arccos(d)))


def _all_stubs(raw_pelvis, raw_t8, fn):
    return [
        fn(raw_pelvis, DEF_ANG_PELVIS, HIP_R_EULER),
        fn(raw_pelvis, DEF_ANG_PELVIS, HIP_L_EULER),
        fn(raw_t8,     DEF_ANG_T8,     SCAP_R_EULER),
        fn(raw_t8,     DEF_ANG_T8,     SCAP_L_EULER),
    ]


def test_tpose_match():
    """At T-pose (identity raw), current and fixed must agree."""
    id_q = np.array([1.0, 0.0, 0.0, 0.0])
    cur = _all_stubs(id_q, id_q, stub_dir_current)
    fix = _all_stubs(id_q, id_q, stub_dir_fixed)
    for i, (c, f) in enumerate(zip(cur, fix)):
        assert ang_deg(c, f) < 0.5, f"stub {i}: T-pose mismatch {c} vs {f}"


def test_pure_yaw_match():
    """Pure yaw — yaw extraction is exact, so current==fixed."""
    y = axangle([0, 0, 1], np.deg2rad(90))
    cur = _all_stubs(y, y, stub_dir_current)
    fix = _all_stubs(y, y, stub_dir_fixed)
    for i, (c, f) in enumerate(zip(cur, fix)):
        assert ang_deg(c, f) < 0.5, f"stub {i}: pure-yaw mismatch {c} vs {f}"


def test_forward_lean_match():
    """Forward bend only — current code happens to be correct here."""
    b = axangle([0, 1, 0], np.deg2rad(-60))
    cur = _all_stubs(b, b, stub_dir_current)
    fix = _all_stubs(b, b, stub_dir_fixed)
    for i, (c, f) in enumerate(zip(cur, fix)):
        assert ang_deg(c, f) < 0.5, f"stub {i}: fwd-lean mismatch {c} vs {f}"


def test_side_lean_fixed_only():
    """Side-lean reveals the bug.  After the §A fix the gap is ≤0.5°.

    Test the FIXED path (which is what shipped code becomes).  The current
    path is exercised by test_side_lean_current_is_wrong below for
    documentation purposes.
    """
    s = axangle([1, 0, 0], np.deg2rad(-30))
    # Ideal world-frame stub direction = oriented * (0,-1,0) for R,
    # or (0,+1,0) for L.  This is exactly what the fixed code returns.
    fix = _all_stubs(s, s, stub_dir_fixed)
    oriented_t8     = qnorm(qmul(s, DEF_ANG_T8))
    oriented_pelvis = qnorm(qmul(s, DEF_ANG_PELVIS))
    ideal = [
        qrot(oriented_pelvis, np.array([0.0, -1.0, 0.0])),
        qrot(oriented_pelvis, np.array([0.0,  1.0, 0.0])),
        qrot(oriented_t8,     np.array([0.0, -1.0, 0.0])),
        qrot(oriented_t8,     np.array([0.0,  1.0, 0.0])),
    ]
    for i, (f, want) in enumerate(zip(fix, ideal)):
        gap = ang_deg(f, want)
        assert gap < 0.5, f"stub {i}: fix gap {gap:.2f}° (want < 0.5°)"


def test_side_lean_current_is_wrong():
    """Regression marker: prove the *current* implementation fails."""
    s = axangle([1, 0, 0], np.deg2rad(-30))
    cur = _all_stubs(s, s, stub_dir_current)
    oriented_t8     = qnorm(qmul(s, DEF_ANG_T8))
    oriented_pelvis = qnorm(qmul(s, DEF_ANG_PELVIS))
    ideal = [
        qrot(oriented_pelvis, np.array([0.0, -1.0, 0.0])),
        qrot(oriented_pelvis, np.array([0.0,  1.0, 0.0])),
        qrot(oriented_t8,     np.array([0.0, -1.0, 0.0])),
        qrot(oriented_t8,     np.array([0.0,  1.0, 0.0])),
    ]
    worst = max(ang_deg(c, w) for c, w in zip(cur, ideal))
    assert worst > 30.0, f"expected current code to deviate >30°, got {worst:.1f}°"


def test_combined_tilt_yaw_fixed_only():
    """Combined forward+side+yaw: only the fixed path should be near 0°."""
    raw = qnorm(qmul(
        axangle([0, 0, 1], np.deg2rad(90)),
        qnorm(qmul(axangle([0, 1, 0], np.deg2rad(-30)),
                   axangle([1, 0, 0], np.deg2rad(-30))))))
    fix = _all_stubs(raw, raw, stub_dir_fixed)
    oriented_t8     = qnorm(qmul(raw, DEF_ANG_T8))
    oriented_pelvis = qnorm(qmul(raw, DEF_ANG_PELVIS))
    ideal = [
        qrot(oriented_pelvis, np.array([0.0, -1.0, 0.0])),
        qrot(oriented_pelvis, np.array([0.0,  1.0, 0.0])),
        qrot(oriented_t8,     np.array([0.0, -1.0, 0.0])),
        qrot(oriented_t8,     np.array([0.0,  1.0, 0.0])),
    ]
    for i, (f, want) in enumerate(zip(fix, ideal)):
        gap = ang_deg(f, want)
        assert gap < 0.5, f"stub {i}: fix gap {gap:.2f}° (want < 0.5°)"


if __name__ == "__main__":
    test_tpose_match()
    test_pure_yaw_match()
    test_forward_lean_match()
    test_side_lean_fixed_only()
    test_side_lean_current_is_wrong()
    test_combined_tilt_yaw_fixed_only()
    print("test_dummy_stubs_under_tilt: PASS")
