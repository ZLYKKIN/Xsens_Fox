"""S11: TRIAD requires two non-parallel gravity-in-body vectors.  For
each of 23 segments compute gravity-in-body for T, N, K pose references
(from defaultSegAnglesFor) and the pair-wise separation.

KEY FINDING — root cause of the user's "right thigh bends sideways"
report:

  defaultSegAnglesFor("kpose")  (scr/main.cpp:288-326)  encodes the
  K-pose math model as:

    upper leg E(0,0,0)        # bone +X = forward horizontal (thighs out)
    lower leg E(P/2, 0, 0)    # bone +X = forward horizontal (legs EXTENDED)
    foot      E(0, 0, 0)      # bone +X = forward horizontal
    toe       E(0, 0, 0)      # bone +X = forward horizontal

  But the user-facing hint at scr/main.cpp:3886 says:

    "K-поза: сядьте на стул (бёдра горизонтально, колени 90°) + руки
     прямо вперёд горизонтально"

  i.e. THIGHS horizontal, KNEES 90° → lower leg vertical down.

  Math says K-pose lower leg is horizontal (knees locked).  Hint says
  K-pose lower leg is vertical (knees bent).  **They disagree.**

  Consequence: when the user follows the hint and bends knees 90°,
  the lower-leg sensor sees gravity along its bone X-axis (same as
  T-pose).  But the calibrator's gravity-in-body table expects
  gravity-in-body = (0, -1, 0) for K-pose.  TRIAD produces a wrong s2s
  rotation for lower leg / foot / toe → the leg's lift axis is offset
  → lifting the knee produces sideways bend.

This test documents the matrix and the mismatch so any future change
either:
  (a) Updates the K-pose hint to say "legs extended straight out
      (knees locked)" — matches the math model.
  (b) Updates defaultSegAnglesFor("kpose") to match the seated-bent-knees
      hint, accepting that lower legs / feet revert to ecompass-only.
  (c) Adds a 4th calibration pose specifically for the lower-leg axis.

The test asserts the CURRENT state so a change that breaks/fixes it is
immediately visible in CI.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import euler_xyz, qrot, qinv, PI


SEG = {
    "pelvis": 0, "l5": 1, "l3": 2, "t12": 3, "t8": 4, "neck": 5, "head": 6,
    "r_shoulder": 7, "r_upper_arm": 8, "r_forearm": 9, "r_hand": 10,
    "l_shoulder": 11, "l_upper_arm": 12, "l_forearm": 13, "l_hand": 14,
    "r_upper_leg": 15, "r_lower_leg": 16, "r_foot": 17, "r_toe": 18,
    "l_upper_leg": 19, "l_lower_leg": 20, "l_foot": 21, "l_toe": 22,
}


def E(x, y, z):
    return euler_xyz(x, y, z)


def default_seg_angles(pose):
    """Mirror of scr/main.cpp:288-326 (defaultSegAnglesFor)."""
    P = PI
    if pose == "tpose":
        return [
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0, 0, -P/2), E(0, 0, -P/2), E(0, 0, -P/2), E(0, 0, -P/2),
            E(0, 0,  P/2), E(0, 0,  P/2), E(0, 0,  P/2), E(0, 0,  P/2),
            E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0),     E(0, 0, 0),
            E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0),     E(0, 0, 0),
        ]
    if pose == "kpose":
        return [
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0, 0, -P/2), E(0, 0, 0),     E(0, 0, 0),     E(0, 0, 0),
            E(0, 0,  P/2), E(0, 0, 0),     E(0, 0, 0),     E(0, 0, 0),
            E(0, 0, 0),    E(P/2, 0, 0),   E(0, 0, 0),     E(0, 0, 0),
            E(0, 0, 0),    E(P/2, 0, 0),   E(0, 0, 0),     E(0, 0, 0),
        ]
    return [
        E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
        E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
        E(0,    0, -P/2),
        E( P/2, 0, -P/2), E( P/2, 0, -P/2), E( P/2, 0, -P/2),
        E(0,    0,  P/2),
        E(-P/2, 0,  P/2), E(-P/2, 0,  P/2), E(-P/2, 0,  P/2),
        E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0), E(0, 0, 0),
        E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0), E(0, 0, 0),
    ]


def gravity_in_body(def_ang):
    """Mirror of scr/main.cpp:1161 gravityInBodyFrame."""
    return qrot(qinv(def_ang), np.array([0., 0, -1]))


def pair_sep(g1, g2):
    return float(np.linalg.norm(np.cross(g1, g2)))


SEPS = {pose: [gravity_in_body(da) for da in default_seg_angles(pose)]
        for pose in ("tpose", "npose", "kpose")}

# scr/main.cpp:5487 — pair is skipped if sep <= 0.3.
TRIAD_SEP_MIN = 0.30


def best_pair(idx):
    gT, gN, gK = SEPS["tpose"][idx], SEPS["npose"][idx], SEPS["kpose"][idx]
    pairs = {"TN": pair_sep(gT, gN), "TK": pair_sep(gT, gK), "NK": pair_sep(gN, gK)}
    return max(pairs.items(), key=lambda kv: kv[1])


def has_any_valid_pair(idx):
    p, sep = best_pair(idx)
    return sep > TRIAD_SEP_MIN, p, sep


def test_upper_arms_calibratable_via_TN():
    """Upper arms / forearms / hands have TN sep ≈ 1 (T arm horizontal,
    N arm down by side) → TRIAD always works for these sensors."""
    for name in ("r_upper_arm", "r_forearm", "r_hand",
                 "l_upper_arm", "l_forearm", "l_hand"):
        ok, pair, sep = has_any_valid_pair(SEG[name])
        assert ok and pair == "TN", f"{name}: expected TN viable, got {pair}@{sep:.3f}"


def test_upper_legs_calibratable_only_via_K_pose():
    """Upper legs have TN=0 (both poses have thighs vertical down) but
    TK=1.0 (K-pose has thighs horizontal forward).  K-pose IS necessary
    for upper-leg calibration."""
    for name in ("r_upper_leg", "l_upper_leg"):
        idx = SEG[name]
        tn = pair_sep(SEPS["tpose"][idx], SEPS["npose"][idx])
        tk = pair_sep(SEPS["tpose"][idx], SEPS["kpose"][idx])
        assert tn < 0.01, f"{name}: TN should be degenerate, got {tn:.3f}"
        assert tk > 0.9, f"{name}: TK should be viable, got {tk:.3f}"


def test_lower_legs_TK_disagrees_with_user_hint():
    """ROOT CAUSE OF KNEE-LIFT-SIDEWAYS BUG.

    Math model has defAng_K[r_lower_leg] = E(P/2, 0, 0) → bone +X (a
    rotation about the bone's long axis doesn't change bone direction,
    so the lower-leg bone stays HORIZONTAL FORWARD in K-pose).  This
    makes gravity-in-body = (0, -1, 0) for K-pose, separate from T-pose's
    (1, 0, 0) → sep TK = 1.0 → TRIAD math thinks lower leg is calibratable.

    But the user-facing K-pose hint asks for "колени 90°" (knees bent
    90°), which puts the lower leg VERTICAL.  When user follows the hint,
    the lower-leg sensor sees gravity aligned to bone-X (same as T-pose).
    The calibrator pairs the wrong sensor-data (gravity along bone-X)
    with the wrong body-frame model (gravity at right angle) → produces
    a spurious 90° s2s rotation on the lower leg's bone axis.

    At runtime, lifting the knee rotates the sensor's lift axis (Y in
    sensor frame), which due to the spurious s2s ends up as a body-frame
    side-lean axis instead of a body-frame pitch axis → knee bends
    sideways.
    """
    idx = SEG["r_lower_leg"]
    tk = pair_sep(SEPS["tpose"][idx], SEPS["kpose"][idx])
    # Math model: sep TK = 1.0  (the calibrator believes K-pose has
    # lower legs horizontal).  If you ever change the math model to put
    # lower leg vertical in K-pose, this test must be updated.
    assert abs(tk - 1.0) < 1e-3, \
        f"lower_leg TK changed from 1.0 to {tk:.3f} — math model touched"


def test_feet_and_toes_TRIAD_fully_degenerate():
    """Feet and toes have defAng_K = E(0,0,0) (same gravity-in-body as
    T-pose) → no TRIAD pair viable → ecompass-only.  Mag heading drift
    is the dominant calibration error source for foot / toe orientation."""
    for name in ("r_foot", "l_foot", "r_toe", "l_toe"):
        ok, pair, sep = has_any_valid_pair(SEG[name])
        assert not ok, \
            f"{name}: unexpectedly viable TRIAD {pair}@{sep:.3f}"


def test_shoulders_TRIAD_fully_degenerate():
    """Shoulder stubs share the same body-axis as T8 in all three poses
    → ecompass-only.  This is fine because the shoulder bone is short
    (5 cm) and mostly drives the scapula stub direction, not a free arm."""
    for name in ("r_shoulder", "l_shoulder"):
        ok, pair, sep = has_any_valid_pair(SEG[name])
        assert not ok, \
            f"{name}: unexpectedly viable TRIAD {pair}@{sep:.3f}"


def test_spine_all_degenerate_by_design():
    """Pelvis through head all share the vertical body frame in every
    reference pose → ecompass-only.  This is correct: spine segments
    have a single anatomical axis (vertical) so single-pose gravity +
    mag heading is sufficient."""
    for name in ("pelvis", "l5", "l3", "t12", "t8", "neck", "head"):
        ok, pair, sep = has_any_valid_pair(SEG[name])
        assert not ok, \
            f"{name}: unexpectedly viable TRIAD {pair}@{sep:.3f}"


def test_cpp_has_kpose_in_calibration():
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    assert 'defaultSegAnglesFor("kpose")' in src
    assert "m_accAccumK" in src


def test_kpose_hint_text_documents_seated_with_bent_knees():
    """The user-facing hint string is `kpose_hint` at scr/main.cpp:3886.
    If anyone changes the hint to ask for legs extended (knees locked),
    that would resolve the math-vs-hint mismatch — this test would then
    fail and prompt updating the math comments to reflect the resolution."""
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    import re
    m = re.search(r'"kpose_hint"\s*,\s*"([^"]+)"', src)
    assert m, "kpose_hint string not found"
    hint = m.group(1)
    # The currently-shipped hint asks for bent knees ("колени 90°") which
    # disagrees with the math model.
    assert "колени 90" in hint or "колени 90°" in hint, \
        f"K-pose hint changed: '{hint}' — verify the math model matches"


if __name__ == "__main__":
    test_upper_arms_calibratable_via_TN()
    test_upper_legs_calibratable_only_via_K_pose()
    test_lower_legs_TK_disagrees_with_user_hint()
    test_feet_and_toes_TRIAD_fully_degenerate()
    test_shoulders_TRIAD_fully_degenerate()
    test_spine_all_degenerate_by_design()
    test_cpp_has_kpose_in_calibration()
    test_kpose_hint_text_documents_seated_with_bent_knees()
    print("test_pose_separation_matrix: PASS")
    print()
    # Print the matrix for human inspection (for the user to see during
    # debugging).
    print(f"{'segment':14}  sepTN   sepTK   sepNK   best       viable")
    print("-" * 60)
    for name, idx in SEG.items():
        gT, gN, gK = SEPS["tpose"][idx], SEPS["npose"][idx], SEPS["kpose"][idx]
        tn, tk, nk = pair_sep(gT, gN), pair_sep(gT, gK), pair_sep(gN, gK)
        best = max(("TN", tn), ("TK", tk), ("NK", nk), key=lambda x: x[1])
        viable = "YES" if best[1] > TRIAD_SEP_MIN else "NO (ecompass-only)"
        print(f"{name:14}  {tn:.3f}  {tk:.3f}  {nk:.3f}  {best[0]}@{best[1]:.2f}    {viable}")
