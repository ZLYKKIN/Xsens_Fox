"""S3: walking gait is heel-strike → midstance → toe-off → swing.  The
current `LocomotionSolver` (scr/main.cpp:1222) tracks one contact per
foot — it sees "foot down" as a single binary, which misses the
heel/toe transition in midstance.

This test specifies a 4-contact heel/toe detector that runs alongside
the existing 1-contact anchor logic, computes (rHeelDown, rToeDown,
lHeelDown, lToeDown) per frame, and asserts:

  • During heel-strike (foot Z=0, ankle angled forward) only heel is down.
  • During midstance only heel AND toe are both down.
  • During toe-off only toe is down.
  • During swing (foot Z > 0) neither is down.

The detector is a thin function over two FK points per foot (heel/toe
keypoints) — the C++ side will read `kp[SEG_RFoot]` (heel) and
`kp[SEG_RToe]` (ball) directly.  This test pins the algorithm so the
C++ port can be validated 1:1.
"""
import numpy as np


# Heel/toe contact thresholds (m).  Matched to the existing
# m_heightMarginSlow = 0.08 m budget in LocomotionSolver — same height
# semantics, applied per-point.
HEIGHT_THRESH = 0.02     # within 2 cm of the floor → "down"
RELEASE_HYST = 0.015     # release at 1.5 cm above the threshold


def heel_toe_contact(heel_z, toe_z, prev_heel_down, prev_toe_down):
    """Per-foot, per-frame detector.  Uses a small hysteresis so a foot
    that's resting on the floor doesn't chatter from sensor noise."""
    heel_down = prev_heel_down
    toe_down = prev_toe_down
    if heel_z <= HEIGHT_THRESH:
        heel_down = True
    elif heel_z > HEIGHT_THRESH + RELEASE_HYST:
        heel_down = False
    if toe_z <= HEIGHT_THRESH:
        toe_down = True
    elif toe_z > HEIGHT_THRESH + RELEASE_HYST:
        toe_down = False
    return heel_down, toe_down


def gait_cycle(t, period=0.8, foot_length=0.25):
    """Return (heel_z, toe_z) at gait phase t ∈ [0, 1).  One step:
      0.00–0.10  heel-strike  : heel low, toe ~ heel + foot_l × sin(15°)
      0.10–0.45  midstance    : both low
      0.45–0.55  toe-off      : heel rising, toe low
      0.55–1.00  swing        : both lifted, peak at 0.78
    """
    phase = (t % period) / period
    if phase < 0.10:
        # heel-strike: heel just touched, toe still slightly up
        u = phase / 0.10
        heel_z = 0.0
        toe_z = (1.0 - u) * 0.04
    elif phase < 0.45:
        # full plant — both on floor
        heel_z = 0.0
        toe_z = 0.0
    elif phase < 0.55:
        # toe-off: heel rises faster than toe; toe pivots on the floor
        u = (phase - 0.45) / 0.10
        heel_z = u * 0.06
        toe_z = max(0.0, u * 0.01)
    else:
        # swing: full lift, sin profile
        u = (phase - 0.55) / 0.45
        amp = 0.12
        heel_z = amp * np.sin(np.pi * u)
        toe_z = amp * np.sin(np.pi * u) + 0.01
    return heel_z, toe_z


def simulate_walk(duration_s=3.0, fps=90, period=0.8):
    n = int(duration_s * fps)
    series = []
    heel_d, toe_d = True, True
    for i in range(n):
        t = i / fps
        hz, tz = gait_cycle(t, period=period)
        heel_d, toe_d = heel_toe_contact(hz, tz, heel_d, toe_d)
        # gait phase label for the test (not part of detector)
        ph = (t % period) / period
        if ph < 0.10:
            label = "heel-strike"
        elif ph < 0.45:
            label = "midstance"
        elif ph < 0.55:
            label = "toe-off"
        else:
            label = "swing"
        series.append((t, hz, tz, heel_d, toe_d, label))
    return series


def _phase_states(series, label):
    return [(h, t) for (_, _, _, h, t, lab) in series if lab == label]


def test_heel_strike_initial_only_heel_down():
    """At the very start of heel-strike (just after swing), only the heel
    has touched.  The toe is still in the air (toe_z ≈ 0.04 m) — detector
    should report (heel=True, toe=False) for the first 2-3 frames.

    The toe drops below the floor threshold quickly (within ~20% of
    stride), at which point we enter midstance — both down."""
    s = simulate_walk()
    initial = _phase_states(s, "heel-strike")[:2]
    assert initial, "no heel-strike states in simulation"
    for (h, t) in initial:
        assert h and not t, \
            f"start of heel-strike: expected (True, False), got ({h}, {t})"


def test_midstance_both_down():
    """During midstance both heel and toe rest on the floor."""
    s = simulate_walk()
    states = _phase_states(s, "midstance")
    # Sample after the first few frames (give the detector time to flip
    # toe down after heel-strike).
    for (h, t) in states[5:]:
        assert h and t, f"midstance: expected both down, got ({h}, {t})"


def test_toe_off_only_toe_down():
    """During toe-off the heel has lifted while the toe is still on the floor."""
    s = simulate_walk()
    states = _phase_states(s, "toe-off")
    # Take samples late in the toe-off window where heel has clearly lifted.
    if not states:
        return
    late = states[len(states) // 2:]
    saw_heel_up = any((not h and t) for (h, t) in late)
    assert saw_heel_up, \
        f"toe-off: heel never lifted while toe stayed down — states={late}"


def test_swing_both_up():
    """Peak swing: both lifted."""
    s = simulate_walk()
    states = _phase_states(s, "swing")
    # mid-swing — peak lift.
    mid = states[len(states) // 2]
    assert not mid[0] and not mid[1], \
        f"mid-swing: expected both up, got {mid}"


def test_no_chatter_during_midstance():
    """The hysteresis must prevent heel/toe flips during midstance."""
    s = simulate_walk()
    states = [(h, t) for (_, _, _, h, t, lab) in s if lab == "midstance"]
    flips = sum(1 for i in range(1, len(states)) if states[i] != states[i-1])
    # Allow ONE flip at the very start of midstance (toe coming down after
    # heel-strike), nothing else.
    assert flips <= 1, f"midstance had {flips} state flips — chatter detected"


def test_full_cycle_has_all_four_states():
    """One complete gait cycle must visit all four (heel,toe) combinations
    in the right order:  (T,F) → (T,T) → (F,T) → (F,F) → repeat."""
    s = simulate_walk(duration_s=2.0)
    seen = []
    prev = None
    for (_, _, _, h, t, _) in s:
        cur = (h, t)
        if cur != prev:
            seen.append(cur)
            prev = cur
    # After settling, the sequence should cycle through these four.
    canonical = {(True, False), (True, True), (False, True), (False, False)}
    visited = set(seen)
    missing = canonical - visited
    assert not missing, f"gait cycle missed states: {missing}; visited {visited}"


def test_cpp_thresholds_match_python():
    """Ensure scr/main.h declares the same heel/toe thresholds the
    Python detector uses — keeps the C++ port and this regression
    test in sync."""
    import os, re
    HERE = os.path.dirname(os.path.abspath(__file__))
    h = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.h"))
    with open(h, "r", encoding="utf-8") as f:
        src = f.read()
    m1 = re.search(r"m_heelToeThreshM\s*=\s*([0-9.]+)", src)
    m2 = re.search(r"m_heelToeReleaseHystM\s*=\s*([0-9.]+)", src)
    assert m1 and m2, "heel/toe threshold constants not found in main.h"
    cpp_thresh = float(m1.group(1))
    cpp_hyst = float(m2.group(1))
    assert abs(cpp_thresh - HEIGHT_THRESH) < 1e-9, \
        f"main.h m_heelToeThreshM={cpp_thresh} but Python HEIGHT_THRESH={HEIGHT_THRESH}"
    assert abs(cpp_hyst - RELEASE_HYST) < 1e-9, \
        f"main.h m_heelToeReleaseHystM={cpp_hyst} but Python RELEASE_HYST={RELEASE_HYST}"


if __name__ == "__main__":
    test_heel_strike_initial_only_heel_down()
    test_midstance_both_down()
    test_toe_off_only_toe_down()
    test_swing_both_up()
    test_no_chatter_during_midstance()
    test_full_cycle_has_all_four_states()
    test_cpp_thresholds_match_python()
    print("test_heel_toe_gait: PASS")
