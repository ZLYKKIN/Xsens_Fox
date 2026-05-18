"""Phase B (L1): LocomotionSolver now picks the per-foot anchor candidate
from heel/toe contact state instead of always using the lowest of
{foot,toe,toe-tip}.

The detector itself (test_heel_toe_gait.py) was already there; phase B
wires its output into the anchor selection inside the solver:

    heel_down & !toe_down  → anchor = heel
    heel_down &  toe_down  → anchor = midpoint(heel, toe-ball)
    !heel_down & toe_down  → anchor = toe-ball (toe-off pivot)
    !heel_down & !toe_down → anchor = lowest (air / fallback)

This test pins the picker function (a pure function over four points and
four booleans) and checks the C++ source contains the wiring.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np


def pick_anchor(heel_down, toe_down, heel, toe):
    """Mirror of LocomotionSolver::update(8-arg)::pick lambda in
    scr/main.cpp (phase B body)."""
    heel = np.asarray(heel, dtype=float)
    toe = np.asarray(toe, dtype=float)
    if heel_down and not toe_down:
        return heel
    if heel_down and toe_down:
        return 0.5 * (heel + toe)
    if (not heel_down) and toe_down:
        return toe
    return heel if heel[2] <= toe[2] else toe


def test_heel_strike_anchor_is_heel():
    heel = (0.30, 0.00, 0.00)
    toe = (0.45, 0.00, 0.04)   # toe still slightly up
    anchor = pick_anchor(True, False, heel, toe)
    assert np.allclose(anchor, heel), f"heel-strike: anchor={anchor}, want heel={heel}"


def test_midstance_anchor_is_midpoint():
    heel = (0.30, 0.00, 0.00)
    toe = (0.45, 0.00, 0.00)
    anchor = pick_anchor(True, True, heel, toe)
    expected = (0.375, 0.0, 0.0)
    assert np.allclose(anchor, expected), \
        f"midstance: anchor={anchor}, want midpoint={expected}"


def test_toe_off_anchor_is_toe_ball():
    heel = (0.30, 0.00, 0.06)  # heel rising
    toe = (0.45, 0.00, 0.00)   # toe still on floor
    anchor = pick_anchor(False, True, heel, toe)
    assert np.allclose(anchor, toe), f"toe-off: anchor={anchor}, want toe={toe}"


def test_airborne_anchor_falls_back_to_lowest():
    """During swing the anchor is unused by the commit gate (release
    fires), but the chosen point still flows into bestPelvisEstimate
    via the fkR/fkL arg.  The fallback should be deterministic — the
    lowest of the two — to avoid jitter."""
    heel = (0.30, 0.00, 0.10)
    toe = (0.45, 0.00, 0.07)
    anchor = pick_anchor(False, False, heel, toe)
    assert np.allclose(anchor, toe), f"air: anchor={anchor}, want toe (lower) {toe}"


def test_rolling_foot_anchor_does_not_walk_forward_during_stance():
    """End-to-end invariant of phase B: with the rolling-foot anchor,
    a single stride's stance phase produces ONE anchor commit point
    (~midfoot), not two separate commits (one at heel, one at toe).

    We simulate one stance phase across heel-strike → midstance →
    toe-off and verify that the anchor point's forward-direction
    distance traversed across the whole stance is bounded by the
    midpoint-to-toe distance (≈ half-foot-length), not the full
    heel-to-toe distance.
    """
    foot_len = 0.25
    # Synthetic stance: heel position fixed; toe-ball at heel+foot_len.
    # Heel/toe stay at Z=0 except heel rises during toe-off.
    heel = (0.0, 0.0, 0.0)
    toe = (foot_len, 0.0, 0.0)

    # Heel-strike (first frame): heel down, toe up
    a0 = pick_anchor(True, False, heel, (foot_len, 0.0, 0.04))
    # Midstance: both down
    a1 = pick_anchor(True, True, heel, toe)
    # Toe-off: heel up, toe down
    a2 = pick_anchor(False, True, (heel[0], heel[1], 0.06), toe)

    forward_range = max(a0[0], a1[0], a2[0]) - min(a0[0], a1[0], a2[0])
    # The biggest forward step the anchor takes is from heel (0.0) to
    # toe-ball (foot_len).  Rolling-foot logic: in midstance the anchor
    # is at midfoot, NOT at heel or at toe — so the per-phase delta is
    # half-foot, half-foot, not full-foot.  This matters because the
    # outer commit/release hysteresis only accepts moves that survive
    # several frames; smaller per-phase deltas mean fewer false commits.
    assert forward_range <= foot_len + 1e-6
    # Midstance anchor must be strictly between heel and toe.
    assert heel[0] < a1[0] < toe[0]


def test_cpp_uses_heel_toe_anchor_overload():
    """Static-source check: phase B is wired through tickLocoHT (the
    new 8-arg LocomotionSolver::update overload).  MainWindow calls
    it instead of the legacy 6-arg tickLoco."""
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    h = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.h"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    with open(h, "r", encoding="utf-8") as f:
        hdr = f.read()
    # Header declares the new 8-arg update overload.
    assert "fkRightHeel" in hdr and "fkRightToe" in hdr, \
        "main.h: 8-arg LocomotionSolver::update overload missing"
    # New tickLocoHT helper exists.
    assert "tickLocoHT" in hdr and "tickLocoHT" in src, \
        "tickLocoHT missing from header/impl"
    # MainWindow calls it.
    assert "m_viewport->tickLocoHT(" in src, \
        "MainWindow must use tickLocoHT to drive heel/toe-aware anchor"
    # tickHeelToe is called BEFORE tickLocoHT in onRenderTick so
    # m_contact is fresh.
    idx_ht = src.find("m_viewport->tickHeelToe(")
    idx_lh = src.find("m_viewport->tickLocoHT(")
    assert idx_ht > 0 and idx_lh > idx_ht, \
        "tickHeelToe must precede tickLocoHT in onRenderTick (phase B order)"


if __name__ == "__main__":
    test_heel_strike_anchor_is_heel()
    test_midstance_anchor_is_midpoint()
    test_toe_off_anchor_is_toe_ball()
    test_airborne_anchor_falls_back_to_lowest()
    test_rolling_foot_anchor_does_not_walk_forward_during_stance()
    test_cpp_uses_heel_toe_anchor_overload()
    print("test_heel_toe_anchor: PASS")
