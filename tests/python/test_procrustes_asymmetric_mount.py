"""S12: scr/main.cpp:5657 `procrustesPair` averages or copies sensor s2s
between matching R/L pairs.  The 60° upper guard (line 5687) suppresses
fusion when the deviation looks like a real mounting asymmetry, but
allows fusion in the 3°–60° band.

A 17-sensor body suit will inevitably have non-perfectly-symmetric
strap rotations.  Field experience puts realistic asymmetry at 5°–20°
(one shoulder strap tighter than the other, one thigh strap rotated
slightly).  These offsets fall in the fusion band and get averaged
50/50, washing out the per-side accuracy.

The user-reported "right thigh bends sideways when I lift my knee"
matches this signature: the right-leg mounting has a small lateral
rotation that the calibrator should preserve, but instead it's
averaged with the left side which is mounted differently.

This test mirrors the procrustesPair fusion math, exercises the
band edges, and documents the current behaviour.  If anyone tightens
the 60° threshold (or moves the lower bound), the assertions update.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qinv, axangle, mirror_y_quat, slerp


CPP_DEV_LOW = 3.0    # below this, the pair already matches → skip
CPP_DEV_HIGH = 60.0  # above this, real asymmetry → skip


def quat_angle_deg(q):
    w = min(1.0, abs(q[0]))
    return 2.0 * np.degrees(np.arccos(w))


def mirror_y_dev_deg(qR, qL):
    """Mirror of scr/main.cpp:890 mirrorYDeviationDeg."""
    mY = mirror_y_quat(qL)
    dot = float(np.dot(qR, mY))
    if dot < 0.0:
        dot = -dot
    dot = min(1.0, dot)
    return float(2.0 * np.degrees(np.arccos(dot)))


def parallel_dev_deg(qR, qL):
    dot = float(np.dot(qR, qL))
    if dot < 0:
        dot = -dot
    dot = min(1.0, dot)
    return float(2.0 * np.degrees(np.arccos(dot)))


def procrustes_pair_fuse(qR, qL, resR, resL, dev_low=CPP_DEV_LOW,
                          dev_high=CPP_DEV_HIGH):
    """Mirror of scr/main.cpp:5657-5755 procrustesPair (mirror branch).
    Returns (qR_out, qL_out, action) where action is 'skip-aligned',
    'skip-asymmetric', or 'averaged'."""
    dev = mirror_y_dev_deg(qR, qL)
    if dev <= dev_low:
        return qR.copy(), qL.copy(), "skip-aligned"
    if dev >= dev_high:
        return qR.copy(), qL.copy(), "skip-asymmetric"
    # Both-modes-equal branch: weighted slerp average.
    counterpartL = mirror_y_quat(qL)
    dot = float(np.dot(qR, counterpartL))
    if dot < 0:
        counterpartL = -counterpartL
    wR = min(0.70, max(0.05, 1.0 - resR / 30.0))
    wL = min(0.70, max(0.05, 1.0 - resL / 30.0))
    wTot = wR + wL
    tR = wL / wTot if wTot > 1e-6 else 0.5
    qAvg = slerp(qR, counterpartL, tR)
    qAvgForL = mirror_y_quat(qAvg)
    return qAvg, qAvgForL, "averaged"


def yaw_only(q):
    """Twist component about Z."""
    w, z = q[0], q[3]
    if w < 0:
        w, z = -w, -z
    n2 = w * w + z * z
    if n2 < 1e-12:
        return np.array([1., 0, 0, 0])
    n = 1.0 / np.sqrt(n2)
    return np.array([w * n, 0, 0, z * n])


def yaw_deg(q):
    half = float(np.arctan2(abs(q[3]), abs(q[0])))
    return 2.0 * np.degrees(half) * (1 if q[3] >= 0 else -1)


def test_small_band_averaging_halves_asymmetry():
    """A 10° real R-side mount offset + 0° L-side, both TRIAD with similar
    residuals.  After procrustesPair fusion, R's offset is averaged with
    L's (mirrored back), halving it to ≈5°.  This is the bug surface."""
    qR = axangle([0, 0, 1], np.deg2rad(10))   # 10° real R offset
    qL = np.array([1., 0, 0, 0])               # 0° L mount
    yaw_R_before = yaw_deg(qR)
    qR_out, _, action = procrustes_pair_fuse(qR, qL, resR=5.0, resL=5.0)
    yaw_R_after = yaw_deg(qR_out)
    assert action == "averaged", f"expected averaged, got {action}"
    assert abs(yaw_R_after) < abs(yaw_R_before), \
        f"averaging didn't move R yaw closer to 0: {yaw_R_before}° → {yaw_R_after}°"


def test_large_asymmetry_preserved():
    """A 65° real R-side offset exceeds the 60° guard → procrustesPair
    leaves both sides untouched."""
    qR = axangle([0, 0, 1], np.deg2rad(65))
    qL = np.array([1., 0, 0, 0])
    qR_out, qL_out, action = procrustes_pair_fuse(qR, qL, resR=5.0, resL=5.0)
    assert action == "skip-asymmetric"
    assert np.allclose(qR_out, qR)
    assert np.allclose(qL_out, qL)


def test_tightened_guard_preserves_realistic_asymmetry():
    """Proposal: tighten dev_high from 60° to 12° so 15° real
    asymmetries (typical 17-sensor suit mounting variance) are
    preserved, not averaged.  This is the C++ fix candidate."""
    qR = axangle([0, 0, 1], np.deg2rad(15))
    qL = np.array([1., 0, 0, 0])
    yaw_R_before = yaw_deg(qR)
    qR_out, _, action = procrustes_pair_fuse(qR, qL, resR=5.0, resL=5.0,
                                              dev_high=12.0)
    yaw_R_after = yaw_deg(qR_out)
    assert action == "skip-asymmetric", \
        f"with dev_high=12° a 15° asymmetry should be preserved, got {action}"
    assert abs(yaw_R_after - yaw_R_before) < 0.5


def test_aligned_pair_skipped_below_low_threshold():
    """A 2° mirror-dev pair is already aligned → no fusion needed."""
    qR = axangle([0, 0, 1], np.deg2rad(2))
    qL = mirror_y_quat(axangle([0, 0, 1], np.deg2rad(2)))
    _, _, action = procrustes_pair_fuse(qR, qL, resR=3.0, resL=3.0)
    assert action == "skip-aligned"


def test_cpp_uses_12deg_band_after_S12():
    """After S12 the C++ procrustesPair uses [3°, 12°] band — small drift
    (< 3°) is left alone, real-but-small asymmetry (3°–12°) is averaged
    for cleaner symmetry, larger offsets (≥ 12°) are preserved as real
    asymmetric mounting.  Both pose passes (K and N) must agree on the
    band — otherwise one pass overrides the other and the fix loses
    effect."""
    import re
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    # Both procrustes passes must use the 12° upper bound.
    count_12 = len(re.findall(r"dev >= 12\.0", src))
    assert count_12 >= 2, \
        f"expected >= 2 dev >= 12.0 sites (K-pass + N-pass), got {count_12}"
    # The legacy 60° must be gone (or only in a comment) — sanity check.
    code_lines_with_60 = [
        line for line in src.splitlines()
        if "dev >= 60.0" in line and not line.strip().startswith("//")
    ]
    assert not code_lines_with_60, \
        f"legacy 60° guard still active: {code_lines_with_60}"


if __name__ == "__main__":
    test_small_band_averaging_halves_asymmetry()
    test_large_asymmetry_preserved()
    test_tightened_guard_preserves_realistic_asymmetry()
    test_aligned_pair_skipped_below_low_threshold()
    test_cpp_uses_12deg_band_after_S12()
    print("test_procrustes_asymmetric_mount: PASS")
