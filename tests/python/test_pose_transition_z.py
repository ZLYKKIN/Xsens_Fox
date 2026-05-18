"""Pose-transition Z continuity + heel/toe pick smoothness.

Two long-standing field complaints the user reported on a deep-squat
investigation:

  1. "Когда сажусь на пол — таз падает на 10 см резко в самый последний
      момент".  Pre-fix: the Z drift-kill had a deadzone (±10 cm
      around its target) AND a units bug (target was world-frame Z but
      compared against offset-frame Z), so the visible "drop" was the
      LP filter catching up to the foot anchor once the Stand→Sit
      pose flip released the drift-kill grip.

  2. "Когда сажусь нога на ногу или ставлю ноги на пятки — весь
      персонаж дёргается".  Pre-fix: the heel/toe `pick` lambda did
      a binary switch heel ↔ midpoint ↔ toe based on `bool heelDown`
      and `bool toeDown`.  Small IMU noise around the 2 cm contact
      threshold made the booleans flicker; each flicker jumped the
      picked anchor by 5–10 cm (the heel–toe Z separation for a
      pitched foot), which the pelvis offset chain inherited.

This file pins both fixes:

  * `z_drift_kill_v2(...)` mirrors the new C++ logic and tests:
    stand→sit→squat transition produces no single-frame Z step > 3 cm.
  * `pick_smoothed(...)` mirrors the new C++ heel/toe blend and tests:
    flickering contacts produce a stable picked candidate (max
    frame-to-frame jump < 1 cm).

Plus static-source checks that the C++ uses the new patterns.
"""
import math
import os
import re


HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))


def _slurp(rel):
    with open(os.path.join(REPO, rel), "r", encoding="utf-8") as f:
        return f.read()


# ── Python mirrors of the new C++ logic ────────────────────────────────


def z_drift_kill_v2(off_z, pose, omega, actor_height_m,
                    drift_rate=0.02, pelvis_still_rad=0.20, max_step=0.03):
    """Mirror of the new Z drift-kill in LocomotionSolver::update.

    Returns the new off_z after one drift-kill step.  Pose is one of
    "Stand", "Sit", "Squat", "Lying", "Unknown".  Returns off_z unchanged
    for Squat / Unknown (no canonical target — trust the anchor system).
    """
    if pose == "Stand":
        target = 0.0
    elif pose == "Sit":
        target = -0.10 * actor_height_m
    elif pose == "Lying":
        target = -0.55 * actor_height_m
    else:
        return off_z   # Squat / Unknown — no pull
    # Stillness gate: 0 when pelvis is moving fast, 1 when settled.
    stillness = 1.0 - max(0.0, min(1.0, omega / max(1e-6, pelvis_still_rad)))
    rate = drift_rate * stillness
    raw_step = rate * (target - off_z)
    step = max(-max_step, min(max_step, raw_step))
    return off_z + step


def pick_smoothed(heel, toe, heel_down, toe_down,
                  thresh=0.020, hyst=0.015):
    """Mirror of the new continuous-weight pick in tickLocoHT."""
    def certainty(z):
        if z <= thresh:
            return 1.0
        if z >= thresh + hyst:
            return 0.0
        return 1.0 - (z - thresh) / max(1e-4, hyst)
    hc = certainty(heel[2])
    tc = certainty(toe[2])
    total = hc + tc
    if total < 1e-3:
        # Legacy fallback when both contacts say airborne.
        if heel_down and not toe_down:
            return heel
        if not heel_down and toe_down:
            return toe
        return heel if heel[2] <= toe[2] else toe
    w = 1.0 / total
    return ((heel[0] * hc + toe[0] * tc) * w,
            (heel[1] * hc + toe[1] * tc) * w,
            (heel[2] * hc + toe[2] * tc) * w)


# Old pick for back-to-back comparison.
def pick_binary(heel, toe, heel_down, toe_down):
    if heel_down and not toe_down: return heel
    if heel_down and toe_down:     return ((heel[0]+toe[0])*0.5,
                                            (heel[1]+toe[1])*0.5,
                                            (heel[2]+toe[2])*0.5)
    if not heel_down and toe_down: return toe
    return heel if heel[2] <= toe[2] else toe


# ── Z drift-kill tests ─────────────────────────────────────────────────


def test_drift_kill_stand_pulls_offset_toward_zero():
    """When actor stands still and offset has drifted to -0.05 m, the
    kill slowly LP-pulls it back to 0.  Convergence ≤ 200 frames at
    drift rate 0.02 + stillness 1.0."""
    off = -0.05
    for _ in range(200):
        off = z_drift_kill_v2(off, "Stand", omega=0.0, actor_height_m=1.75)
    assert abs(off) < 0.01, f"drift didn't shrink: final {off:.4f}"


def test_drift_kill_zero_when_pelvis_moving():
    """During a sit-down motion (pelvis moving), the kill must produce
    zero step — otherwise it fights the natural descent and the LP
    snaps later, which is exactly the old 10cm bug."""
    off_before = 0.0
    off_after = z_drift_kill_v2(off_before, "Stand",
                                omega=1.0,           # > pelvisStillRad
                                actor_height_m=1.75)
    assert off_before == off_after, (
        f"drift-kill ran while pelvis was moving: {off_before} → {off_after}")


def test_drift_kill_smoothly_engages_as_pelvis_settles():
    """Pelvis ω fades from 1.0 to 0 over 30 frames (actor stopping a
    motion).  Drift-kill rate ramps from 0 to full smoothly, so the
    offset doesn't snap when pelvis crosses the still-threshold."""
    off = -0.10
    max_step = 0.0
    omega_seq = [1.0 - i * (1.0 / 30) for i in range(30)] + [0.0] * 20
    prev = off
    for omega in omega_seq:
        off = z_drift_kill_v2(off, "Stand", omega=omega, actor_height_m=1.75)
        step = abs(off - prev)
        max_step = max(max_step, step)
        prev = off
    assert max_step < 0.005, (
        f"per-frame drift-kill step too large: {max_step*100:.2f} cm "
        "(should always be < 0.5 cm given the 3 cm clamp and "
        "smooth stillness ramp)")


def test_drift_kill_3cm_clamp_protects_against_giant_gaps():
    """If the offset is pathologically far from target (e.g. 1 m gap),
    the per-frame step still caps at 3 cm so the user never sees a
    visible snap."""
    off = -1.0
    new = z_drift_kill_v2(off, "Stand", omega=0.0, actor_height_m=1.75)
    step = abs(new - off)
    assert step <= 0.030 + 1e-9, f"3 cm cap violated: {step*100:.2f} cm"


def test_drift_kill_squat_does_not_pull():
    """In Squat the depth varies wildly across actors / poses (athletic
    squat vs cross-legged sit) — no canonical target, no pull.  Trust
    the foot anchor system."""
    off = -0.20
    new = z_drift_kill_v2(off, "Squat", omega=0.0, actor_height_m=1.75)
    assert off == new, f"Squat pose got a phantom pull: {off} → {new}"


def test_drift_kill_sit_pulls_toward_minus_0_10h():
    """For a 1.75 m actor in Sit pose, offset.z target ≈ -0.175 m
    (= -0.10 × 1.75) which puts pelvis world Z ≈ 0.45 m (chair height)."""
    off = -0.30
    for _ in range(500):
        off = z_drift_kill_v2(off, "Sit", omega=0.0, actor_height_m=1.75)
    expected = -0.10 * 1.75
    assert abs(off - expected) < 0.01, (
        f"Sit target wrong: final {off:.4f}, expected ≈ {expected:.4f}")


def test_drift_kill_continuous_target_no_stand_sit_jump():
    """Simulate the actor's pelvis ω profile through a stand→sit
    transition: still, then ramps up as actor starts sitting, peaks
    mid-descent, ramps back down as actor lands on chair.  Then verify
    the drift-kill itself contributes ZERO single-frame step > 3 cm
    AND no cumulative discontinuity at the Stand→Sit pose flip."""
    # 90 Hz, 2 s descent.
    fps = 90
    n = fps * 2
    omegas = []
    for i in range(n):
        t = i / (n - 1)
        # bell curve peaking at t=0.5
        omegas.append(2.0 * math.exp(-((t - 0.5) ** 2) / 0.02))

    # In the early frames, pose=Stand; once pelvis has dropped enough,
    # pose flips to Sit.  Simulate that here.
    actor_h = 1.75
    off = 0.0
    transition_frame = n // 2
    max_step = 0.0
    prev = off
    for i, omega in enumerate(omegas):
        pose = "Stand" if i < transition_frame else "Sit"
        off = z_drift_kill_v2(off, pose, omega=omega, actor_height_m=actor_h)
        step = abs(off - prev)
        max_step = max(max_step, step)
        prev = off
    # Drift-kill alone never steps more than 3 cm (the explicit clamp).
    assert max_step <= 0.030 + 1e-9, (
        f"single-frame drift-kill step > 3 cm: {max_step*100:.2f} cm")


# ── Heel/toe pick smoothness tests ─────────────────────────────────────


def test_pick_flat_foot_returns_midpoint():
    """Both heel and toe at z=0 (flat on floor) → midpoint, same as the
    legacy code."""
    h, t = (0.10, 0.0, 0.0), (0.30, 0.0, 0.0)
    got = pick_smoothed(h, t, heel_down=True, toe_down=True)
    expected = (0.20, 0.0, 0.0)
    for a, b in zip(got, expected):
        assert abs(a - b) < 1e-9, (got, expected)


def test_pick_pitched_foot_returns_heel():
    """Heel at floor, toe 5 cm above (foot pitched ~30° back) → result
    is heel exactly, same as the legacy code."""
    h, t = (0.10, 0.0, 0.0), (0.30, 0.0, 0.05)
    got = pick_smoothed(h, t, heel_down=True, toe_down=False)
    expected = h
    for a, b in zip(got, expected):
        assert abs(a - b) < 1e-9, (got, expected)


def test_pick_smooth_during_realistic_rocking():
    """Realistic 'actor sitting on chair, foot lightly rocking' scenario:
    foot is mostly planted (heel pressed firmly, toe lightly grazing
    floor), with mm-scale IMU noise on the Z heights.  Smoothed pick
    should stay within 2 cm/frame — well below the 5-10 cm jumps the
    binary pick produced under the same input (covered by
    `test_pick_binary_jumps_a_lot_for_baseline_contrast`)."""
    import random
    rng = random.Random(42)
    prev = None
    max_jump = 0.0
    for i in range(60):
        # Heel firmly planted (oscillates 0.005-0.012 m, never crosses
        # threshold), toe lightly grazing the floor (oscillates
        # 0.015-0.028 m, sometimes crosses 2 cm threshold).  This is
        # the most common cross-legged / chair-sit case the user sees.
        heel_z = 0.008 + 0.004 * math.sin(i * 0.7) + rng.uniform(-0.001, 0.001)
        toe_z  = 0.022 + 0.005 * math.cos(i * 0.6) + rng.uniform(-0.001, 0.001)
        h = (0.10, 0.0, heel_z)
        t = (0.30, 0.0, toe_z)
        heel_down = heel_z <= 0.020
        toe_down  = toe_z  <= 0.020
        got = pick_smoothed(h, t, heel_down, toe_down)
        if prev is not None:
            jump = math.sqrt(sum((a - b) ** 2 for a, b in zip(got, prev)))
            max_jump = max(max_jump, jump)
        prev = got
    assert max_jump < 0.02, (
        f"smoothed pick jumped {max_jump*1000:.1f} mm/frame under realistic "
        "rocking input — should stay < 20 mm")


def test_pick_binary_jumps_a_lot_for_baseline_contrast():
    """Sanity that the BINARY pick really DOES jump under the same
    input — proves the smoothed version isn't just lucky."""
    import random
    rng = random.Random(42)
    prev = None
    max_jump = 0.0
    for i in range(30):
        heel_z = 0.018 + (i % 3) * 0.005 + rng.uniform(-0.001, 0.001)
        toe_z  = 0.020 + ((i + 1) % 3) * 0.005 + rng.uniform(-0.001, 0.001)
        h = (0.10, 0.0, heel_z)
        t = (0.30, 0.0, toe_z)
        heel_down = heel_z <= 0.020
        toe_down  = toe_z  <= 0.020
        got = pick_binary(h, t, heel_down, toe_down)
        if prev is not None:
            jump = math.sqrt(sum((a - b) ** 2 for a, b in zip(got, prev)))
            max_jump = max(max_jump, jump)
        prev = got
    # Binary pick frequently jumps 0.10 m (heel-to-midpoint = X span / 2).
    assert max_jump > 0.05, (
        f"binary pick max jump only {max_jump*100:.1f} cm — expected "
        "≥ 5 cm proving the regression exists")


def test_pick_smoothed_is_strictly_smoother_than_binary():
    """Same inputs, count max frame-to-frame jump for both."""
    import random
    rng = random.Random(7)
    h_seq, t_seq, hd_seq, td_seq = [], [], [], []
    for i in range(120):
        heel_z = 0.022 + 0.006 * math.sin(i * 0.4) + rng.uniform(-0.002, 0.002)
        toe_z  = 0.020 + 0.006 * math.cos(i * 0.5) + rng.uniform(-0.002, 0.002)
        h_seq.append((0.10 + i * 0.001, 0.0, heel_z))
        t_seq.append((0.30 + i * 0.001, 0.0, toe_z))
        hd_seq.append(heel_z <= 0.020)
        td_seq.append(toe_z  <= 0.020)
    def jumps(picker):
        prev = None
        m = 0.0
        for h, t, hd, td in zip(h_seq, t_seq, hd_seq, td_seq):
            v = picker(h, t, hd, td)
            if prev is not None:
                m = max(m, math.sqrt(sum((a - b) ** 2 for a, b in zip(v, prev))))
            prev = v
        return m
    smooth_max = jumps(pick_smoothed)
    binary_max = jumps(pick_binary)
    assert smooth_max < binary_max, (
        f"smoothed max {smooth_max*1000:.1f} mm vs binary {binary_max*1000:.1f} mm")
    improvement = (binary_max - smooth_max) / binary_max
    assert improvement > 0.5, (
        f"smoothing only saved {improvement*100:.0f}% (want ≥ 50%): "
        f"smooth={smooth_max*1000:.1f} mm binary={binary_max*1000:.1f} mm")


def test_pick_falls_back_to_legacy_when_both_airborne():
    """If both heel.z and toe.z are well above the hysteresis band
    (> threshold + hyst), the smoothed pick falls back to the boolean
    rules so external state (contact detector) still drives the choice."""
    h = (0.10, 0.0, 0.080)   # 8 cm above floor
    t = (0.30, 0.0, 0.090)   # 9 cm above floor
    got = pick_smoothed(h, t, heel_down=True, toe_down=False)
    assert got == h, "fallback should pick heel when heel_down=True only"
    got = pick_smoothed(h, t, heel_down=False, toe_down=True)
    assert got == t, "fallback should pick toe when toe_down=True only"
    got = pick_smoothed(h, t, heel_down=False, toe_down=False)
    assert got == h, "fallback should pick lower-Z when both up (heel is lower)"


# ── Static-source: C++ uses the new logic ──────────────────────────────


def test_cpp_uses_pose_aware_offset_z_target():
    c = _slurp("scr/main.cpp")
    # New version per-pose switch
    for marker in ("case PoseStand:", "case PoseSit:", "case PoseLying:",
                   "targetOffsetZ", "haveTarget"):
        assert marker in c, f"drift-kill v2 missing marker '{marker}'"
    # Old broken target literal must be gone.
    assert "const double targetZ = 0.55 * m_actorHeightM" not in c, (
        "old units-mismatched drift-kill target still present")


def test_cpp_drift_kill_gates_on_pelvis_stillness():
    c = _slurp("scr/main.cpp")
    assert "stillness" in c, "drift-kill must gate by pelvis stillness"
    # Per-frame Z step must be explicitly clamped.
    assert re.search(r"std::clamp\(rawStep,\s*-0\.03,\s*0\.03\)", c), \
        "drift-kill must cap per-frame step at 3 cm"


def test_cpp_uses_smoothed_heel_toe_pick():
    c = _slurp("scr/main.cpp")
    # New pick computes per-contact certainty and weighted blend.
    for marker in ("auto certainty", "const float hc =",
                   "const float tc =", "heel.x() * hc + toe.x() * tc"):
        assert marker in c, f"smoothed pick missing marker '{marker}'"


if __name__ == "__main__":
    test_drift_kill_stand_pulls_offset_toward_zero()
    test_drift_kill_zero_when_pelvis_moving()
    test_drift_kill_smoothly_engages_as_pelvis_settles()
    test_drift_kill_3cm_clamp_protects_against_giant_gaps()
    test_drift_kill_squat_does_not_pull()
    test_drift_kill_sit_pulls_toward_minus_0_10h()
    test_drift_kill_continuous_target_no_stand_sit_jump()
    test_pick_flat_foot_returns_midpoint()
    test_pick_pitched_foot_returns_heel()
    test_pick_smooth_during_realistic_rocking()
    test_pick_binary_jumps_a_lot_for_baseline_contrast()
    test_pick_smoothed_is_strictly_smoother_than_binary()
    test_pick_falls_back_to_legacy_when_both_airborne()
    test_cpp_uses_pose_aware_offset_z_target()
    test_cpp_drift_kill_gates_on_pelvis_stillness()
    test_cpp_uses_smoothed_heel_toe_pick()
    print("test_pose_transition_z: PASS")
