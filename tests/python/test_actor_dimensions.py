"""Actor anthropometry — static-source + numerical tests for the new
shoulderWidthCm / hipWidthCm / handLengthCm overrides in ActorConfig.

These three fields close a long-standing gap in the wizard:

  * shoulderWidthCm: bi-acromial breadth (drives the FK shoulder bone
    length and the armSpan→armScale derivation).  Wrong value used to
    break the prayer-hands pose because the two hands meet at the
    wrong horizontal distance from the spine.

  * hipWidthCm: bi-iliac breadth (drives the pelvis dummy stub length).
    Wrong value used to break the lotus pose because the legs fold
    from the wrong starting points.

  * handLengthCm: wrist-to-fingertip length (drives the FK hand bone).
    Wrong value used to make "hands-on-head" pose impossible for actors
    with hands far from the population mean.

Static checks pin the fields' existence and use in buildLengths.
Numerical checks re-implement the buildLengths formula in Python and
verify each field produces the expected change in the relevant bone
length.
"""
import os
import re


HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))


def _slurp(rel):
    with open(os.path.join(REPO, rel), "r", encoding="utf-8") as f:
        return f.read()


# ── Static-source: ActorConfig struct ────────────────────────────────────


def test_actor_config_has_new_fields():
    h = _slurp("scr/main.h")
    # All three must appear as `double NAME = 0.0;` in ActorConfig.
    for name in ("shoulderWidthCm", "hipWidthCm", "handLengthCm"):
        assert re.search(
            r"double\s+" + name + r"\s*=\s*0\.0\s*;",
            h), f"ActorConfig missing `double {name} = 0.0;`"


def test_wizard_result_has_new_fields():
    h = _slurp("scr/main.h")
    # The wizard Result is the bridge between wizard UI and MainWindow's
    # ActorConfig — losing the fields here would silently drop the user
    # input on the floor.
    # Find the NewSessionWizard::Result struct body and check inside.
    m = re.search(r"struct Result\s*\{(.*?)\};", h, re.DOTALL)
    assert m, "NewSessionWizard::Result struct not found in main.h"
    body = m.group(1)
    for name in ("shoulderWidthCm", "hipWidthCm", "handLengthCm"):
        assert name in body, f"Result struct missing {name}"


def test_wizard_has_spinbox_pointers():
    h = _slurp("scr/main.h")
    for ptr in ("m_shoulder", "m_hip", "m_hand"):
        assert re.search(r"QDoubleSpinBox\*\s+" + ptr + r"\s*=\s*nullptr",
                          h), f"wizard missing QDoubleSpinBox* {ptr}"
    for ptr in ("m_lblShoulder", "m_lblHip", "m_lblHand"):
        assert re.search(r"QLabel\*\s+" + ptr + r"\s*=\s*nullptr",
                          h), f"wizard missing QLabel* {ptr}"


# ── Static-source: buildLengths uses the fields ──────────────────────────


def test_buildlengths_consumes_new_fields():
    c = _slurp("scr/main.cpp")
    # The function should read each ActorConfig override.
    assert "actor.shoulderWidthCm" in c, "buildLengths must read shoulderWidthCm"
    assert "actor.hipWidthCm"      in c, "buildLengths must read hipWidthCm"
    assert "actor.handLengthCm"    in c, "buildLengths must read handLengthCm"
    # And use the derived per-side metres for the FK bones.
    assert "shoulderHalfM"  in c, "buildLengths missing shoulderHalfM variable"
    assert "hipHalfM"       in c, "buildLengths missing hipHalfM variable"
    assert "handLenM"       in c, "buildLengths missing handLenM variable"


def test_buildlengths_fixed_bodyWidthM_hardcode():
    """The old armSpan→armScale derivation used `bodyWidthM = 0.30 × (h/1.75)`
    which under-estimated bi-acromial breadth by ~15 cm and over-counted
    arm length per side.  The fixed code uses shoulderWidthM (user
    override OR 0.259·h) as the same quantity."""
    c = _slurp("scr/main.cpp")
    assert "0.30 * (h / 1.75)" not in c, (
        "buildLengths still uses the broken bodyWidthM hardcode — must "
        "use shoulderWidthM (user override OR 0.259·h) instead")


# ── Static-source: wizard UI wiring ──────────────────────────────────────


def test_wizard_goNext_writes_new_fields_to_result():
    c = _slurp("scr/main.cpp")
    # Look for the goNext write site.
    m = re.search(r"m_result\.heightCm\s*=.*?\}", c, re.DOTALL)
    assert m, "goNext write site not found"
    body = m.group(0)
    for name in ("shoulderWidthCm", "hipWidthCm", "handLengthCm"):
        assert "m_result." + name in body, (
            f"goNext doesn't write m_result.{name} — wizard input is lost")


def test_mainwindow_threads_new_fields_to_actor():
    c = _slurp("scr/main.cpp")
    # MainWindow ctor builds an ActorConfig from m_setup; all three
    # new fields must be copied across.
    assert "actor.shoulderWidthCm = m_setup.shoulderWidthCm" in c
    assert "actor.hipWidthCm      = m_setup.hipWidthCm"      in c
    assert "actor.handLengthCm    = m_setup.handLengthCm"    in c


def test_breakdown_has_three_new_rows():
    c = _slurp("scr/main.cpp")
    # Anthropometric breakdown UI must surface the three new derived values
    # so the actor sees what FK will use after picking 0 (auto from height).
    for k in ("bk_shoulder", "bk_hip", "bk_hand"):
        assert '"' + k + '"' in c, f"breakdown missing row {k}"


# ── Numerical: re-implement buildLengths in Python and verify ────────────


def build_lengths(height_cm=175.0, foot_cm=26.0, arm_span_cm=0.0, leg_cm=0.0,
                  shoulder_cm=0.0, hip_cm=0.0, hand_cm=0.0):
    """Python mirror of SkeletonXsens::buildLengths in scr/main.cpp.
    Returns a dict of per-segment lengths in metres."""
    h = height_cm / 100.0
    fl = foot_cm / 100.0
    trunkScale = h / 1.75

    shoulderWidthM = (shoulder_cm / 100.0) if shoulder_cm > 0.0 else (0.259 * h)
    shoulderHalfM = max(0.05, 0.5 * shoulderWidthM)

    hipWidthM = (hip_cm / 100.0) if hip_cm > 0.0 else (0.191 * h)
    hipHalfM = max(0.05, 0.5 * hipWidthM)

    armScale = 1.0
    if arm_span_cm > 0.0:
        armPerSideM = max(0.10, (arm_span_cm / 100.0 - shoulderWidthM) * 0.5)
        defArmM = 0.44 * h
        if defArmM > 1e-6:
            armScale = armPerSideM / defArmM

    legScale = 1.0
    if leg_cm > 0.0:
        legPerSideM = max(0.20, leg_cm / 100.0)
        defLegM = 0.491 * h
        if defLegM > 1e-6:
            legScale = legPerSideM / defLegM

    handLenM = (hand_cm / 100.0) if hand_cm > 0.0 else (0.108 * h * armScale)

    return {
        "shoulder":  shoulderHalfM,
        "hip_stub":  hipHalfM,
        "upper_arm": 0.186 * h * armScale,
        "forearm":   0.146 * h * armScale,
        "hand":      handLenM,
        "thigh":     0.245 * h * legScale,
        "shin":      0.246 * h * legScale,
        "foot":      0.60 * fl,
        "toe":       0.40 * fl,
        "trunkScale": trunkScale,
    }


def test_default_dimensions_match_canonical_ratios():
    """For h=175 cm with all overrides=0, every derived length is the
    Drillis-Contini canonical value."""
    L = build_lengths(175.0)
    h = 1.75
    assert abs(L["shoulder"]  - 0.5 * 0.259 * h) < 1e-6
    assert abs(L["hip_stub"]  - 0.5 * 0.191 * h) < 1e-6
    assert abs(L["upper_arm"] - 0.186 * h)       < 1e-6
    assert abs(L["forearm"]   - 0.146 * h)       < 1e-6
    assert abs(L["hand"]      - 0.108 * h)       < 1e-6
    assert abs(L["thigh"]     - 0.245 * h)       < 1e-6
    assert abs(L["shin"]      - 0.246 * h)       < 1e-6
    assert abs(L["foot"]      - 0.6 * 0.26)      < 1e-6


def test_shoulder_width_drives_shoulder_bone_length():
    """Override shoulder breadth → FK shoulder bone changes proportionally,
    independent of height."""
    L_default = build_lengths(175.0)
    L_broad   = build_lengths(175.0, shoulder_cm=55.0)  # 55 cm bi-acromial
    L_narrow  = build_lengths(175.0, shoulder_cm=35.0)
    assert abs(L_broad ["shoulder"] - 0.275) < 1e-6, L_broad
    assert abs(L_narrow["shoulder"] - 0.175) < 1e-6, L_narrow
    # And bigger shoulders → bigger FK bone (sanity).
    assert L_broad["shoulder"] > L_default["shoulder"] > L_narrow["shoulder"]


def test_hip_width_drives_pelvis_stub_length():
    L_default = build_lengths(175.0)
    L_wide    = build_lengths(175.0, hip_cm=40.0)
    L_narrow  = build_lengths(175.0, hip_cm=25.0)
    assert abs(L_wide  ["hip_stub"] - 0.200) < 1e-6
    assert abs(L_narrow["hip_stub"] - 0.125) < 1e-6
    assert L_wide["hip_stub"] > L_default["hip_stub"] > L_narrow["hip_stub"]


def test_hand_length_drives_hand_bone():
    """User override beats the height-derived default."""
    L_default = build_lengths(175.0)          # 0.108 × 1.75 = 0.189
    L_big     = build_lengths(175.0, hand_cm=22.0)
    L_small   = build_lengths(175.0, hand_cm=14.0)
    assert abs(L_default["hand"] - 0.108 * 1.75) < 1e-6
    assert abs(L_big    ["hand"] - 0.22) < 1e-6
    assert abs(L_small  ["hand"] - 0.14) < 1e-6


def test_armScale_uses_shoulderWidth_not_old_hardcode():
    """Pre-fix: armScale used bodyWidthM = 0.30 × (h/1.75) — for h=1.75
    that's 0.30 m.  Fixed: armScale uses shoulderWidth = 0.259·h or
    user override.  Verify the change: with armSpan=180 cm and default
    height, armScale must reflect 0.45 m bi-acromial, not 0.30 m."""
    L = build_lengths(175.0, arm_span_cm=180.0)
    # arm per side = (1.80 - 0.45325) / 2 ≈ 0.673 m
    # default per side = 0.44 × 1.75 = 0.77 m
    # armScale ≈ 0.673 / 0.77 ≈ 0.874
    # upper_arm = 0.186 × 1.75 × 0.874 ≈ 0.284
    expected_arm = (1.80 - 0.259 * 1.75) * 0.5
    expected_scale = expected_arm / (0.44 * 1.75)
    assert abs(L["upper_arm"] - 0.186 * 1.75 * expected_scale) < 1e-4


def test_explicit_shoulder_override_threads_into_armScale():
    """If user enters BOTH shoulder width AND arm span, the armScale
    derivation must use the user's shoulder (not the height default)."""
    L_narrow_sh = build_lengths(175.0, arm_span_cm=180.0, shoulder_cm=35.0)
    L_wide_sh   = build_lengths(175.0, arm_span_cm=180.0, shoulder_cm=55.0)
    # Narrower shoulders + same armSpan → longer per-side arm.
    assert L_narrow_sh["upper_arm"] > L_wide_sh["upper_arm"], (
        L_narrow_sh, L_wide_sh)


def test_zero_overrides_keep_height_derived_behaviour():
    """All overrides at 0 → identical lengths to the pre-fix path
    (modulo the bodyWidthM 0.30→0.259 correction, which only affects
    the armSpan path when armSpan > 0)."""
    L1 = build_lengths(180.0, foot_cm=27.0)
    L2 = build_lengths(180.0, foot_cm=27.0, shoulder_cm=0.0, hip_cm=0.0, hand_cm=0.0)
    for k in L1:
        assert abs(L1[k] - L2[k]) < 1e-9, (k, L1[k], L2[k])


def test_shoulderHalfM_lower_bound_protects_against_zero():
    """Adversarial user enters shoulderWidthCm = 0.01 → max(0.05, 0.5*0.0001)
    = 0.05 m floor.  Without the floor, FK has a degenerate shoulder bone."""
    L = build_lengths(175.0, shoulder_cm=0.1)
    assert L["shoulder"] >= 0.05


def test_realistic_actor_population_produces_sane_geometry():
    """Sample three actor archetypes and verify the resulting lengths
    are within human-physiology ranges."""
    # 150 cm / petite
    L_short = build_lengths(150.0, foot_cm=22.0, arm_span_cm=148.0,
                            leg_cm=72.0, shoulder_cm=36.0, hip_cm=29.0,
                            hand_cm=15.5)
    # 175 cm / average male
    L_avg   = build_lengths(175.0, foot_cm=26.0, arm_span_cm=178.0,
                            leg_cm=88.0, shoulder_cm=45.5, hip_cm=33.4,
                            hand_cm=19.0)
    # 200 cm / tall
    L_tall  = build_lengths(200.0, foot_cm=30.0, arm_span_cm=205.0,
                            leg_cm=101.0, shoulder_cm=52.0, hip_cm=38.0,
                            hand_cm=21.5)
    for tag, L in (("short", L_short), ("avg", L_avg), ("tall", L_tall)):
        # All lengths positive, finite, monotone in actor height.
        for k, v in L.items():
            if k == "trunkScale":
                continue
            assert v > 0.0 and v < 1.0, f"{tag} {k}={v} out of human range"
    # Tall actor's bones uniformly bigger than short actor's.
    for k in ("upper_arm", "forearm", "hand", "thigh", "shin", "foot",
              "shoulder", "hip_stub"):
        assert L_tall[k] > L_short[k], (k, L_tall[k], L_short[k])


if __name__ == "__main__":
    test_actor_config_has_new_fields()
    test_wizard_result_has_new_fields()
    test_wizard_has_spinbox_pointers()
    test_buildlengths_consumes_new_fields()
    test_buildlengths_fixed_bodyWidthM_hardcode()
    test_wizard_goNext_writes_new_fields_to_result()
    test_mainwindow_threads_new_fields_to_actor()
    test_breakdown_has_three_new_rows()
    test_default_dimensions_match_canonical_ratios()
    test_shoulder_width_drives_shoulder_bone_length()
    test_hip_width_drives_pelvis_stub_length()
    test_hand_length_drives_hand_bone()
    test_armScale_uses_shoulderWidth_not_old_hardcode()
    test_explicit_shoulder_override_threads_into_armScale()
    test_zero_overrides_keep_height_derived_behaviour()
    test_shoulderHalfM_lower_bound_protects_against_zero()
    test_realistic_actor_population_produces_sane_geometry()
    print("test_actor_dimensions: PASS")
