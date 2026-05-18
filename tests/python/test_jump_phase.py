"""Phase C (L2): airborne-phase guard in LocomotionSolver.

Without a pelvis-translation sensor we can't track jumps ballistically,
but we can stop the solver from *incorrectly* re-anchoring during a jump.
The guard:
  • increments `m_airTicks` while BOTH feet are released (not committed),
  • flips `m_airborne` true at ≥ 4 ticks (~44 ms @ 90 Hz),
  • resets to 0 the moment any foot commits again,
  • suppresses ZUPT accumulation and the pose-aware Z drift-kill while
    airborne (these would otherwise force-anchor the pelvis to floor).

This file pins those state transitions on a Python mock and verifies
the C++ wiring is present.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


AIR_TICKS_THRESH = 4


def air_state_step(prev_air_ticks, committed_r, committed_l):
    """Mirror of phase-C airTicks increment in scr/main.cpp::LocomotionSolver::update."""
    if (not committed_r) and (not committed_l):
        new_ticks = min(prev_air_ticks + 1, 4096)
    else:
        new_ticks = 0
    airborne = new_ticks >= AIR_TICKS_THRESH
    return new_ticks, airborne


def test_airborne_fires_after_threshold_frames():
    """Both feet released for ≥ 4 frames → airborne goes True on the 4th."""
    ticks = 0
    seen_airborne = []
    for f in range(8):
        ticks, ab = air_state_step(ticks, False, False)
        seen_airborne.append(ab)
    # First 3 frames: not yet airborne; 4th onward: airborne.
    assert seen_airborne[:3] == [False, False, False], \
        f"airborne fired too early: {seen_airborne[:3]}"
    assert all(seen_airborne[3:]), f"airborne not sustained: {seen_airborne[3:]}"


def test_airborne_resets_on_commit():
    """Any commit during airborne resets m_airTicks → airborne flips off."""
    ticks = 0
    for _ in range(10):
        ticks, _ = air_state_step(ticks, False, False)
    assert ticks >= AIR_TICKS_THRESH
    # Now right foot commits — airborne must clear immediately.
    ticks, ab = air_state_step(ticks, True, False)
    assert ticks == 0 and ab is False, \
        f"airborne didn't reset on commit: ticks={ticks} airborne={ab}"


def test_walking_swing_does_not_falsely_trigger_airborne():
    """During normal walking, at any frame at least one foot is
    committed (alternating support).  The airborne guard must NEVER
    fire on this pattern."""
    ticks = 0
    saw_airborne = False
    # 60 frames: alternating support — right committed for 5 frames,
    # then left committed for 5 frames, repeating.  No frame has both
    # released, so airTicks stays 0.
    for f in range(60):
        phase = (f // 5) % 2
        committed_r = (phase == 0)
        committed_l = (phase == 1)
        ticks, ab = air_state_step(ticks, committed_r, committed_l)
        if ab:
            saw_airborne = True
    assert not saw_airborne, "walking should never trigger airborne (always one foot committed)"


def test_double_support_release_then_recommit_short_blip_no_airborne():
    """Brief 1-frame double release (within hysteresis) shouldn't flip
    airborne — too short to reach the 4-frame threshold."""
    ticks = 0
    ticks, ab1 = air_state_step(ticks, False, False)
    ticks, ab2 = air_state_step(ticks, True, False)
    assert not ab1 and not ab2, "1-frame blip should not flip airborne"


def test_real_jump_profile_lands_airborne():
    """Simulate a realistic 200 ms jump (≈18 frames @ 90 Hz) where
    both feet are released throughout."""
    ticks = 0
    airborne_during = []
    for _ in range(18):
        ticks, ab = air_state_step(ticks, False, False)
        airborne_during.append(ab)
    # First 3 frames: not airborne; 4th onward: airborne.
    assert sum(airborne_during) >= 14, \
        f"jump: too few airborne frames {sum(airborne_during)} / 18"
    # Landing — both committed again.
    ticks, ab = air_state_step(ticks, True, True)
    assert not ab, "landing must clear airborne"


def test_cpp_wires_airborne_into_zupt_and_z_drift_kill():
    """Static-source check:
      1. m_airTicks / m_airborne defined in main.h
      2. ZUPT line gates on `!m_airborne`
      3. Z drift-kill `if (m_pose == PoseStand …)` is also gated on `!m_airborne`
      4. reset() zeroes m_airTicks/m_airborne
    """
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    h = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.h"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    with open(h, "r", encoding="utf-8") as f:
        hdr = f.read()
    assert "m_airTicks" in hdr and "m_airborne" in hdr, \
        "main.h: m_airTicks / m_airborne missing"
    assert "isAirborne()" in hdr, "main.h: isAirborne() accessor missing"
    # ZUPT gate must mention !m_airborne (or equivalent).
    assert "!m_airborne" in src, \
        "ZUPT / drift-kill must check !m_airborne (phase C guard)"
    # The Z drift-kill block must contain the airborne gate now.
    idx = src.find("Pose-aware Z drift-kill")
    assert idx > 0, "Z drift-kill block not located"
    window = src[idx:idx + 1500]
    assert "!m_airborne" in window, \
        "Z drift-kill block must gate on !m_airborne"
    # reset() resets m_airTicks.
    reset_idx = src.find("void LocomotionSolver::reset()")
    assert reset_idx > 0, "LocomotionSolver::reset not found"
    reset_window = src[reset_idx:reset_idx + 2000]
    assert "m_airTicks" in reset_window and "m_airborne" in reset_window, \
        "reset() must zero m_airTicks / m_airborne"


if __name__ == "__main__":
    test_airborne_fires_after_threshold_frames()
    test_airborne_resets_on_commit()
    test_walking_swing_does_not_falsely_trigger_airborne()
    test_double_support_release_then_recommit_short_blip_no_airborne()
    test_real_jump_profile_lands_airborne()
    test_cpp_wires_airborne_into_zupt_and_z_drift_kill()
    print("test_jump_phase: PASS")
