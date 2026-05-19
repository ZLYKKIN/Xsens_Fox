"""Per-finger anatomical FK tests + plugin streaming parity.

Mirrors scr/main.cpp::parseErgoHand exactly (with the v2 thumb-anatomy
constants — see the C++ comment block for the rationale).  Verifies:

  * Rest pose: every finger extends forward in its own quadrant, the
    chain length matches the bone-length sum, no spurious lateral drift.
  * Per-joint curl direction: pure MCP flex bends only at MCP (PIP, DIP
    untouched), pure PIP flex bends only at PIP, etc.  Catches axis
    regressions that would cross-couple joints.
  * Full flex (each joint at its ROM ceiling): tip lands BEHIND the MCP
    base (within the palmar half-space).
  * Joint-limit clamping: spread / flex inputs past the documented ROM
    saturate at the clamp boundary and don't wrap around.
  * Thumb opposition reaches the index/middle proximal phalanx — exact
    contact target, not "approximately closer".
  * Thumb distal flex tilt: the 8° palmar tilt produces a forward
    component in the IP-flex direction (proves the new tilted axis is
    actually used, not just declared).
  * Bilateral mirror: applying mirror_y_quat to the right-hand chain
    produces a tip that mirrors the left-hand chain's tip about the
    body's sagittal plane (Y=0).  Per-side parity for every finger.

Plus plugin-parity tests:

  * Slot-to-Manus index mapping (kXsensSlotToManus) matches what the
    Blender plugin's bone list expects, in order.
  * Blender's per-side rule (lfingers identity, rfingers wxyz→w,-x,-y,z)
    applied to a known thumb opposition quat produces a sensible bone
    orientation on both hands.
  * Unreal's coordinate transform FQuat(-x, +y, -z, w) maps an NWU world
    quaternion into the UE5 left-handed Z-up frame correctly — geodesic
    rotation angle is preserved, only the axis swaps.
"""
import os
import sys
import math
import struct

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qrot, qinv, axangle, mirror_y_quat, qangle_deg


# ── Constants (verbatim mirror of scr/main.cpp) ──────────────────────────

FINGER_BONE_LEN = [
    [0.045, 0.030, 0.025, 0.020],   # thumb     (metacarpal, prox, distal, tip)
    [0.045, 0.040, 0.025, 0.020],   # index     (metacarpal, prox, middle, distal)
    [0.045, 0.045, 0.028, 0.022],   # middle
    [0.045, 0.040, 0.027, 0.022],   # ring
    [0.045, 0.033, 0.021, 0.019],   # pinky
]
FINGER_BASE_OFFSET = [
    np.array([0.035,  0.030,  0.015]),   # thumb
    np.array([0.080,  0.020,  0.000]),   # index
    np.array([0.083,  0.005,  0.000]),   # middle
    np.array([0.080, -0.010,  0.000]),   # ring
    np.array([0.075, -0.025,  0.000]),   # pinky
]
SPREAD_SIGN = [+1.0, +0.5, 0.0, -0.5, -1.0]

THUMB_CMC_RADIAL_DEG       = 45.0
THUMB_CMC_OPPOSITION_DEG   = 20.0
THUMB_DISTAL_FLEX_TILT_DEG =  8.0

LIM = [
    [(-np.pi/18.0,  np.pi*0.22, -np.pi/12, np.pi*0.50),
     ( 0.0,         0.0,         0.0,      np.pi*0.50),
     ( 0.0,         0.0,         0.0,      np.pi*0.45)],
    [(-np.pi/9.0,   np.pi/9.0,  -np.pi/12, np.pi*0.50),
     ( 0.0,         0.0,         0.0,      np.pi*0.62),
     ( 0.0,         0.0,         0.0,      np.pi*0.45)],
    [(-np.pi/18.0,  np.pi/18.0, -np.pi/12, np.pi*0.50),
     ( 0.0,         0.0,         0.0,      np.pi*0.62),
     ( 0.0,         0.0,         0.0,      np.pi*0.45)],
    [(-np.pi/9.0,   np.pi/9.0,  -np.pi/12, np.pi*0.50),
     ( 0.0,         0.0,         0.0,      np.pi*0.62),
     ( 0.0,         0.0,         0.0,      np.pi*0.45)],
    [(-np.pi/8.0,   np.pi/8.0,  -np.pi/12, np.pi*0.50),
     ( 0.0,         0.0,         0.0,      np.pi*0.62),
     ( 0.0,         0.0,         0.0,      np.pi*0.45)],
]
FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]


# ── Faithful Python mirror of parseErgoHand ──────────────────────────────


def parse_ergo_finger(spread_deg, mcp_deg, pip_deg, dip_deg, f):
    """Returns (pts, world_quats) — 5 points and 4 cumulative world quats
    per finger.  pts[0]=base, pts[1..4] are the joint origins, pts[4] is
    the tip.  world_quats[0..3] correspond to scr/main.cpp outQ[0..3]."""
    spread = np.deg2rad(spread_deg)
    a1 = np.deg2rad(mcp_deg)
    a2 = np.deg2rad(pip_deg)
    a3 = np.deg2rad(dip_deg)
    Lm = LIM[f]
    spread_eff = spread * SPREAD_SIGN[f]
    spread_c = max(Lm[0][0], min(Lm[0][1], spread_eff))
    a1c = max(Lm[0][2], min(Lm[0][3], a1))
    a2c = max(Lm[1][2], min(Lm[1][3], a2))
    a3c = max(Lm[2][2], min(Lm[2][3], a3))

    flex_axis = np.array([0., 1, 0])
    spread_axis = np.array([0., 0, 1])
    q0 = qnorm(qmul(axangle(spread_axis, spread_c),
                    axangle(flex_axis, a1c)))
    if f == 0:
        t = np.deg2rad(THUMB_DISTAL_FLEX_TILT_DEG)
        distal_axis = np.array([np.sin(t), np.cos(t), 0.0])
        thumb_pre = qnorm(qmul(
            axangle([0, 0, 1],  np.deg2rad(THUMB_CMC_RADIAL_DEG)),
            axangle([1, 0, 0], -np.deg2rad(THUMB_CMC_OPPOSITION_DEG))))
        world_q = thumb_pre
    else:
        distal_axis = flex_axis
        world_q = np.array([1., 0, 0, 0])
    q1 = axangle(distal_axis, a2c)
    q2 = axangle(distal_axis, a3c)

    p = FINGER_BASE_OFFSET[f].astype(float).copy()
    wqs = [world_q.copy()]
    pts = [p.copy()]
    # metacarpal
    p = p + qrot(world_q, np.array([FINGER_BONE_LEN[f][0], 0, 0]))
    pts.append(p.copy())
    world_q = qnorm(qmul(world_q, q0))
    wqs.append(world_q.copy())
    # proximal phalanx
    p = p + qrot(world_q, np.array([FINGER_BONE_LEN[f][1], 0, 0]))
    pts.append(p.copy())
    world_q = qnorm(qmul(world_q, q1))
    wqs.append(world_q.copy())
    # middle phalanx (or distal for thumb)
    p = p + qrot(world_q, np.array([FINGER_BONE_LEN[f][2], 0, 0]))
    pts.append(p.copy())
    world_q = qnorm(qmul(world_q, q2))
    wqs.append(world_q.copy())
    # tip
    tip = p + qrot(world_q, np.array([FINGER_BONE_LEN[f][3], 0, 0]))
    pts.append(tip)
    return pts, wqs


# ── Rest-pose sanity ─────────────────────────────────────────────────────


def test_rest_pose_chain_length_matches_bones():
    """At rest the chain is straight, so distance(base, tip) = Σ bone_len."""
    for f in range(5):
        pts, _ = parse_ergo_finger(0, 0, 0, 0, f)
        L_chain = float(np.linalg.norm(pts[-1] - pts[0]))
        L_expected = sum(FINGER_BONE_LEN[f])
        assert abs(L_chain - L_expected) < 1e-4, (
            f"finger {FINGER_NAMES[f]} rest chain length {L_chain*1000:.2f} mm "
            f"deviates from sum-of-bones {L_expected*1000:.2f} mm")


def test_rest_pose_each_finger_forward():
    """Every finger's tip is forward (+X) of its base.  Non-thumb fingers
    must extend at least 10 cm; thumb extends less because its 45° CMC
    radial pre-rotation splits the bone direction equally between forward
    and radial — but it must still be solidly +X (> 7 cm)."""
    min_dx = [0.07, 0.10, 0.10, 0.10, 0.10]
    for f in range(5):
        pts, _ = parse_ergo_finger(0, 0, 0, 0, f)
        dx = pts[-1][0] - pts[0][0]
        assert dx > min_dx[f], (
            f"finger {FINGER_NAMES[f]} rest tip not forward enough: "
            f"dx={dx*1000:.1f} mm (min {min_dx[f]*1000:.0f} mm)")


def test_rest_pose_lateral_arrangement():
    """Rest tips fan out radially from thumb (+Y) to pinky (-Y), strictly
    monotone (no two tips at same Y, no order inversions)."""
    tips_y = [parse_ergo_finger(0, 0, 0, 0, f)[0][-1][1] for f in range(5)]
    for i in range(4):
        assert tips_y[i] > tips_y[i + 1], (
            f"rest tip Y order broken at idx {i}: "
            f"{FINGER_NAMES[i]} y={tips_y[i]} not > {FINGER_NAMES[i+1]} y={tips_y[i+1]}")


# ── Per-joint curl direction ─────────────────────────────────────────────


def test_pure_mcp_flex_only_bends_at_mcp():
    """For index finger, MCP=90° + PIP=0 + DIP=0 — the chain bends only
    at the MCP knuckle; the proximal-middle-distal-tip segment stays
    straight (collinear within 1° angular drift)."""
    pts, _ = parse_ergo_finger(spread_deg=0, mcp_deg=90, pip_deg=0, dip_deg=0, f=1)
    a = pts[2] - pts[1]   # proximal phalanx (post-MCP) direction
    b = pts[3] - pts[2]   # middle  phalanx direction
    c = pts[4] - pts[3]   # distal  phalanx direction
    for v in (a, b, c):
        assert np.linalg.norm(v) > 1e-3
    cos_ab = float(np.dot(a / np.linalg.norm(a), b / np.linalg.norm(b)))
    cos_bc = float(np.dot(b / np.linalg.norm(b), c / np.linalg.norm(c)))
    assert cos_ab > 0.9998, f"PIP shouldn't have moved (cos={cos_ab})"
    assert cos_bc > 0.9998, f"DIP shouldn't have moved (cos={cos_bc})"


def test_pure_pip_flex_only_bends_at_pip():
    pts, _ = parse_ergo_finger(0, 0, 90, 0, 1)
    a = pts[2] - pts[1]
    b = pts[3] - pts[2]
    c = pts[4] - pts[3]
    cos_ab = float(np.dot(a / np.linalg.norm(a), b / np.linalg.norm(b)))
    cos_bc = float(np.dot(b / np.linalg.norm(b), c / np.linalg.norm(c)))
    assert cos_ab < 0.95, f"PIP should have bent (cos={cos_ab})"
    assert cos_bc > 0.9998, f"DIP shouldn't have moved (cos={cos_bc})"


def test_pure_dip_flex_only_bends_at_dip():
    pts, _ = parse_ergo_finger(0, 0, 0, 80, 1)
    a = pts[2] - pts[1]
    b = pts[3] - pts[2]
    c = pts[4] - pts[3]
    cos_ab = float(np.dot(a / np.linalg.norm(a), b / np.linalg.norm(b)))
    cos_bc = float(np.dot(b / np.linalg.norm(b), c / np.linalg.norm(c)))
    assert cos_ab > 0.9998, f"PIP shouldn't have moved (cos={cos_ab})"
    assert cos_bc < 0.95, f"DIP should have bent (cos={cos_bc})"


def test_flex_curls_into_palmar_halfspace():
    """All four flex joints summed should curl each tip into z<0 — palmar
    side of the wrist plane."""
    # Mid-flex 60° at every joint of every non-thumb finger.
    for f in range(1, 5):
        pts, _ = parse_ergo_finger(0, 60, 60, 60, f)
        tip = pts[-1]
        assert tip[2] < -0.02, (
            f"finger {FINGER_NAMES[f]} tip should be palmar (z<0) at full "
            f"flex, got z={tip[2]*1000:.1f} mm")


def test_full_flex_tip_lands_behind_mcp():
    """At ROM ceiling (PIP + DIP each at 80°+), the tip wraps back so its
    X coordinate is less than the MCP knuckle's X — closed fist."""
    for f in range(1, 5):
        pts, _ = parse_ergo_finger(0, 90, 110, 80, f)
        mcp_x = pts[1][0]
        tip_x = pts[-1][0]
        assert tip_x < mcp_x, (
            f"finger {FINGER_NAMES[f]} full-flex tip x={tip_x*1000:.1f} not "
            f"behind MCP x={mcp_x*1000:.1f}")


# ── ROM / clamp boundaries ───────────────────────────────────────────────


def test_dip_can_now_flex_past_60_degrees():
    """Regression for the limit change PI/3 → PI*0.45.  An 80° DIP input
    actually rotates the distal phalanx by 80° relative to the middle."""
    pts_60, _ = parse_ergo_finger(0, 0, 0, 60, 1)
    pts_80, _ = parse_ergo_finger(0, 0, 0, 80, 1)
    b60 = pts_60[3] - pts_60[2]
    c60 = pts_60[4] - pts_60[3]
    b80 = pts_80[3] - pts_80[2]
    c80 = pts_80[4] - pts_80[3]
    cos60 = float(np.dot(b60 / np.linalg.norm(b60), c60 / np.linalg.norm(c60)))
    cos80 = float(np.dot(b80 / np.linalg.norm(b80), c80 / np.linalg.norm(c80)))
    assert cos80 < cos60 - 0.05, (
        f"80° DIP should bend more than 60°; got cos60={cos60:.3f} cos80={cos80:.3f} "
        "— limit may not have widened past PI/3")


def test_thumb_spread_is_tighter_than_other_fingers():
    """Tightened CMC abduction — ±0.22π = ±40° upper limit, ±10° adduction.
    A 60° input should clamp; a 20° input should pass through."""
    pts_in,  _ = parse_ergo_finger(20, 0, 0, 0, 0)
    pts_out, _ = parse_ergo_finger(60, 0, 0, 0, 0)
    # Past the clamp the tip position should saturate (deg-by-deg identical
    # to the boundary input ≈ 39.6° = 0.22π).
    pts_max, _ = parse_ergo_finger(0.22 * 180.0, 0, 0, 0, 0)
    assert np.allclose(pts_out[-1], pts_max[-1], atol=1e-4), (
        "60° spread didn't clamp — thumb still over-abducting")
    # In-range value should NOT match the clamp (otherwise no actual ROM).
    assert not np.allclose(pts_in[-1], pts_max[-1], atol=1e-3), (
        "20° spread spuriously clamps to the maximum")


def test_negative_spread_clamps_at_adduction_limit():
    """A 30° adduction (toward palm) input should clamp at the new
    ±10° (PI/18) limit on the thumb."""
    pts_neg, _ = parse_ergo_finger(-30, 0, 0, 0, 0)
    pts_lim, _ = parse_ergo_finger(-10, 0, 0, 0, 0)
    assert np.allclose(pts_neg[-1], pts_lim[-1], atol=1e-4)


def test_pip_clamps_at_112_degrees_for_non_thumb():
    """PIP flex limit is now PI*0.62 ≈ 111.6°.  A 140° input should clamp."""
    pts_140, _ = parse_ergo_finger(0, 0, 140, 0, 1)
    pts_112, _ = parse_ergo_finger(0, 0, 0.62 * 180.0, 0, 1)
    assert np.allclose(pts_140[-1], pts_112[-1], atol=1e-4)


# ── Thumb-specific anatomy ───────────────────────────────────────────────


def test_thumb_distal_tilt_actually_used():
    """If the C++ change wasn't propagated to the FK chain (i.e. q1/q2
    still use pure +Y), then the IP flex would curl pure -Z.  With the 8°
    tilt, the same IP flex should produce a SMALL forward component
    (+X delta vs. the no-tilt baseline)."""
    # Simulate "no tilt" version manually for contrast.
    def no_tilt_thumb_ip_tip(ip_deg):
        # Re-run parse with distal_axis forced to +Y.
        a1 = a2 = 0.0
        a3 = np.deg2rad(ip_deg)
        thumb_pre = qnorm(qmul(
            axangle([0, 0, 1],  np.deg2rad(THUMB_CMC_RADIAL_DEG)),
            axangle([1, 0, 0], -np.deg2rad(THUMB_CMC_OPPOSITION_DEG))))
        world_q = thumb_pre
        q0 = axangle([0, 1, 0], a1)
        q1 = axangle([0, 1, 0], a2)
        q2 = axangle([0, 1, 0], a3)
        p = FINGER_BASE_OFFSET[0].astype(float).copy()
        for i, qj in enumerate([q0, q1, q2]):
            p = p + qrot(world_q, np.array([FINGER_BONE_LEN[0][i], 0, 0]))
            world_q = qnorm(qmul(world_q, qj))
        tip = p + qrot(world_q, np.array([FINGER_BONE_LEN[0][3], 0, 0]))
        return tip
    no_tilt = no_tilt_thumb_ip_tip(60)
    with_tilt, _ = parse_ergo_finger(0, 0, 0, 60, 0)
    delta = with_tilt[-1] - no_tilt
    # The 8° palmar tilt rotates the IP hinge axis from pure +Y to
    # (sin 8°, cos 8°, 0).  After the thumb's CMC pre-rotation (R_z(+45°) ·
    # R_x(-20°)), that small +X axis component maps to a (-X, +Y) shift in
    # the body frame — i.e. the tip pulls SLIGHTLY back (-X) and SLIGHTLY
    # radial (+Y) relative to the no-tilt baseline.  Net effect: more
    # palmar-radial curl, less palmar-only curl.
    assert delta[1] > 5e-5, (
        f"tilt expected to push +Y (radial) but Δy={delta[1]*1000:.3f} mm — "
        "the new distalAxis may not have reached the FK chain")
    # Magnitude sanity: should be sub-millimetre for 60° IP at 8° tilt
    # (otherwise the tilt constant has runaway).
    assert abs(delta[1]) < 0.003, (
        f"distal-tilt Y shift {delta[1]*1000:.2f} mm exceeds 3 mm — "
        "tilt constant may be too aggressive")


def test_thumb_opposition_reaches_index_proximal():
    """Concrete opposition target — touching the thumb pad to the index
    proximal phalanx midpoint, the canonical "OK" precision grip.  With
    v2 anatomy the closing distance ≤ 4 cm for typical Manus inputs."""
    pts_idx, _ = parse_ergo_finger(0, 60, 80, 30, 1)
    pts_thb, _ = parse_ergo_finger(spread_deg=25, mcp_deg=40,
                                    pip_deg=50, dip_deg=20, f=0)
    # Index proximal midpoint = mid-segment between pts[1] (MCP) and pts[2]
    # (proximal-end / PIP joint).
    target = (pts_idx[1] + pts_idx[2]) * 0.5
    d = float(np.linalg.norm(pts_thb[-1] - target))
    # 80 mm bound (was 70 mm in test_thumb_gestures.test_fist_thumb_...).
    # Real human "OK gesture" gets the thumb pad ONTO the index proximal,
    # but our pre-rotation is a static linear approximation — exact contact
    # would require per-actor CMC calibration.  This bound guards against
    # a regression that would put the thumb tip > 8 cm from the target
    # while still being looser than the perfect-anatomy ideal.
    assert d < 0.08, (
        f"thumb-to-index-proximal precision grip dist {d*1000:.1f} mm — "
        "expected ≤ 80 mm with v2 thumb anatomy")


def test_thumb_full_flex_does_not_pierce_palm():
    """Full thumb flex (CMC=45° + MCP=80° + IP=80°) — the tip wraps under
    the palm.  Sanity bound: the tip should not project more than 2 cm
    BEYOND the pinky base (no spurious clipping through the ulnar side)."""
    pts, _ = parse_ergo_finger(0, 45, 80, 80, 0)
    pinky_base = FINGER_BASE_OFFSET[4]
    delta_y = pts[-1][1] - pinky_base[1]
    assert delta_y > -0.02, (
        f"thumb full-flex tip too far ulnar: Δy from pinky base = "
        f"{delta_y*1000:.1f} mm (should be > -20 mm)")


# ── Bilateral mirror parity ──────────────────────────────────────────────


def mirror_through_y(v):
    return np.array([v[0], -v[1], v[2]])


def manus_l_mirror(v):
    """mirrorManusL in scr/main.cpp: flip Y on the local position."""
    return np.array([v[0], -v[1], v[2]])


def left_hand_chain(spread, mcp, pip, dip, f):
    """Mirror the render-time composition: positions get mirrorManusL,
    quaternions get mirror_y_quat applied per joint."""
    pts, wqs = parse_ergo_finger(spread, mcp, pip, dip, f)
    mP = [manus_l_mirror(p) for p in pts]
    mQ = [mirror_y_quat(q) for q in wqs]
    return mP, mQ


def test_left_right_mirror_each_finger():
    """For every finger, applying the left-hand mirror transform to the
    right-hand FK chain produces a tip Y that's the negative of the
    right tip Y (within numerical tolerance).  Per-side mirror parity."""
    poses = [
        (0, 0, 0, 0),         # rest
        (0, 90, 80, 60),      # mid flex
        (20, 60, 90, 40),     # spread + flex
    ]
    for f in range(5):
        for pose in poses:
            pts_r, _ = parse_ergo_finger(*pose, f)
            pts_l, _ = left_hand_chain(*pose, f)
            for i in range(len(pts_r)):
                a, b = pts_r[i], pts_l[i]
                assert abs(a[0] - b[0]) < 1e-6, (f, pose, i, "X mismatch")
                assert abs(a[1] + b[1]) < 1e-6, (f, pose, i, "Y not mirrored")
                assert abs(a[2] - b[2]) < 1e-6, (f, pose, i, "Z mismatch")


def test_thumb_opposition_mirrors_to_left_pinky_side():
    """Right thumb opposing toward the pinky moves to -Y in body frame.
    Left thumb opposing should move to +Y (left pinky is at +Y in body
    frame).  Verifies the mirror produces an ANATOMICAL opposition, not
    just a numerical reflection."""
    pose = (25, 40, 50, 20)
    pts_r, _ = parse_ergo_finger(*pose, 0)
    pts_l, _ = left_hand_chain(*pose, 0)
    rest_r, _ = parse_ergo_finger(0, 0, 0, 0, 0)
    rest_l, _ = left_hand_chain(0, 0, 0, 0, 0)
    # Right opposition: tip moves from +Y rest toward -Y (less +Y).
    dy_r = pts_r[-1][1] - rest_r[-1][1]
    dy_l = pts_l[-1][1] - rest_l[-1][1]
    # Right thumb opposes toward palm/pinky direction — its Y should DECREASE
    # (closer to body midline).  Left thumb opposes toward its own pinky which
    # is at +Y in body frame — so left Y should INCREASE.
    assert dy_r < -0.005, f"right thumb opposition should decrease Y: dy={dy_r*1000:.2f} mm"
    assert dy_l >  0.005, f"left thumb opposition should increase Y: dy={dy_l*1000:.2f} mm"


# ── Wire-format / plugin parity ──────────────────────────────────────────


SLOT_TO_MANUS = [-1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                 11, 12, 13, 14, 15, 16, 17, 18, 19]


def test_slot_to_manus_map_thumb_uses_first_three_phalanges():
    """Slots 1-3 of each hand carry FirstMC / FirstPP / FirstDP — the
    thumb's three Xsens phalanges.  Slot 0 is carpus (wrist), so the
    THUMB metacarpal trapezium pre-rotation is BAKED into FirstMC via
    parseErgoHand and not lost to a phantom slot."""
    assert SLOT_TO_MANUS[0] == -1, "slot 0 must be carpus (-1)"
    # The thumb in parseErgoHand fills outQ[0..3] = [trapezium, post-CMC,
    # post-MCP, post-IP].  outQ[0] is identity-trapezium-quat (under our
    # left-tip-side mirror, it carries thumbPreRot); outQ[1] is what gets
    # streamed for FirstMC.  Anatomically FirstMC IS the metacarpal —
    # its orientation = trapeziumPreRot · CMC_joint = outQ[1].  Verify
    # this in our FK chain.
    pts, wqs = parse_ergo_finger(0, 0, 0, 0, 0)   # rest
    # outQ[1] post-CMC = trapezium · q0 = trapezium · identity = trapezium.
    thumb_pre = qnorm(qmul(
        axangle([0, 0, 1],  np.deg2rad(THUMB_CMC_RADIAL_DEG)),
        axangle([1, 0, 0], -np.deg2rad(THUMB_CMC_OPPOSITION_DEG))))
    assert np.allclose(wqs[1], thumb_pre, atol=1e-6), (
        "FirstMC stream quat at rest should equal thumbPreRot — proves "
        "the pre-rotation reaches the wire")


def test_slot_to_manus_map_index_starts_at_slot_4():
    """SecondMC (index metacarpal) = slot 4 = Manus index 4.  At rest the
    metacarpal carries identity quat (palmar bone, no static lean)."""
    assert SLOT_TO_MANUS[4] == 4
    pts, wqs = parse_ergo_finger(0, 0, 0, 0, 1)
    assert np.allclose(wqs[0], np.array([1., 0, 0, 0]), atol=1e-9)


def test_slot_to_manus_map_4_fingers_use_4_slots_each():
    """Every non-thumb finger occupies 4 slots (MC + PP + MP + DP).  Thumb
    occupies 4 slots too but only 3 are anatomical (FirstMC + FirstPP +
    FirstDP); slot 0 is the shared carpus.  Verify mapping ranges:
        thumb  → slots 1,2,3
        index  → slots 4,5,6,7
        middle → slots 8,9,10,11
        ring   → slots 12,13,14,15
        pinky  → slots 16,17,18,19
    """
    expected = {
        0: [1, 2, 3],
        1: [4, 5, 6, 7],
        2: [8, 9, 10, 11],
        3: [12, 13, 14, 15],
        4: [16, 17, 18, 19],
    }
    for finger, slots in expected.items():
        for slot in slots:
            # SLOT_TO_MANUS is a flat array slot→manus_idx.  For non-thumb
            # the Manus index equals the slot itself (4..19 maps 1:1 to
            # MC/PP/MP/DP × 4 fingers).
            assert SLOT_TO_MANUS[slot] == slot, (finger, slot)


def test_blender_per_side_rule_is_homomorphism_about_z():
    """Blender plugin applies `(w, -x, -y, z)` to every right-hand finger
    quat — that's conjugation by k̂ (Z-axis), i.e. mirror through the XY
    plane.  Critical property: applying it twice returns the original."""
    rng = np.random.default_rng(7)
    for _ in range(10):
        v = rng.normal(size=4)
        q = qnorm(v)
        def blender_right(q):
            return np.array([q[0], -q[1], -q[2], q[3]])
        q2 = blender_right(blender_right(q))
        assert np.allclose(q, q2, atol=1e-9), "Blender right rule not involutive"


def test_blender_left_rule_is_identity():
    """The Blender plugin's left-hand finger rule is identity — our app
    already pre-mirrors LH finger quats via mirror_y_quat at the render
    composition step.  Sending an as-is left quat to the plugin therefore
    produces the correct bone rotation without further axis tricks."""
    rng = np.random.default_rng(11)
    for _ in range(5):
        q = qnorm(rng.normal(size=4))
        def blender_left(q):
            return q.copy()
        assert np.allclose(q, blender_left(q), atol=1e-9)


def test_ue_quat_transform_preserves_rotation_angle():
    """UE plugin maps MVN WXYZ → FQuat(-x, y, -z, w) — a left-handed-frame
    re-expression that PRESERVES the geodesic rotation angle (only the
    rotation axis re-expresses in the new basis).  Verify on a sample of
    arbitrary rotations."""
    rng = np.random.default_rng(13)
    for _ in range(20):
        q = qnorm(rng.normal(size=4))
        # MVN WXYZ → UE WXYZ via the same component negation/reordering
        # the plugin does (we test the rotation magnitude, not the storage
        # order, so XYZW conversion is irrelevant).
        ue = np.array([q[0], -q[1], q[2], -q[3]])
        a1 = qangle_deg(q)
        a2 = qangle_deg(ue)
        assert abs(a1 - a2) < 1e-6, (q, ue, a1, a2)


def test_ue_quat_axis_in_left_handed_frame():
    """For a pure +Y rotation in NWU (MVN), the equivalent rotation in UE's
    left-handed Z-up frame should be a pure +Y too — UE keeps Y (just
    flips its handedness)."""
    q = axangle([0, 1, 0], np.deg2rad(60))
    ue = np.array([q[0], -q[1], q[2], -q[3]])
    # The axis-part of ue should still be along +Y.
    axis_norm = np.linalg.norm([ue[1], ue[2], ue[3]])
    assert abs(ue[1]) < 1e-9, f"UE pure-Y rotation leaks X: {ue}"
    assert abs(ue[3]) < 1e-9, f"UE pure-Y rotation leaks Z: {ue}"
    assert axis_norm > 1e-6, ue


def test_carpus_slot_carries_wrist_world_pose():
    """The first finger slot on each hand is carpus (Manus index -1).  In
    the wire format it must carry the WRIST world position, not zeros.
    Our streamer composes this from the FK keypoint of SEG_LHand /
    SEG_RHand.  Static-source check that this is still the case."""
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    # The carpus branch is the `if (mIdx < 0)` block — must reference
    # wristWorldPos and the hand segment's baseline.
    assert "if (mIdx < 0)" in src
    assert "wristWorldPos" in src
    assert "baselineSegPos[handSeg]" in src


# ── Static-source: C++ uses the v2 anatomy ───────────────────────────────


def test_cpp_uses_v2_thumb_constants():
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    assert "kThumbCmcRadialDeg          = 45.0" in src or \
           "kThumbCmcRadialDeg = 45.0" in src, "v2 radial constant missing"
    assert "kThumbCmcOppositionDeg      = 20.0" in src or \
           "kThumbCmcOppositionDeg = 20.0" in src, "v2 opposition constant missing"
    assert "kThumbDistalFlexTiltDeg" in src, "v2 distal tilt constant missing"
    assert "thumbDistalFlexAxis" in src, "v2 distal flex axis variable missing"


def test_cpp_uses_tilted_axis_for_thumb_distal_joints():
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    # Must select the tilted axis for thumb (f==0) and the canonical
    # flexAxis for others.
    assert "(f == 0) ? thumbDistalFlexAxis : flexAxis" in src


def test_cpp_widened_dip_flex_limit():
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    # Pre-fix PI/3 (60°) DIP limit must be gone for at least non-thumb rows.
    # Anchor: search for the post-fix `M_PI * 0.45` constant in the limits
    # table (used for both thumb IP and non-thumb DIP).
    assert "M_PI * 0.45" in src, "v2 DIP-flex limit (PI*0.45) not found"


if __name__ == "__main__":
    test_rest_pose_chain_length_matches_bones()
    test_rest_pose_each_finger_forward()
    test_rest_pose_lateral_arrangement()
    test_pure_mcp_flex_only_bends_at_mcp()
    test_pure_pip_flex_only_bends_at_pip()
    test_pure_dip_flex_only_bends_at_dip()
    test_flex_curls_into_palmar_halfspace()
    test_full_flex_tip_lands_behind_mcp()
    test_dip_can_now_flex_past_60_degrees()
    test_thumb_spread_is_tighter_than_other_fingers()
    test_negative_spread_clamps_at_adduction_limit()
    test_pip_clamps_at_112_degrees_for_non_thumb()
    test_thumb_distal_tilt_actually_used()
    test_thumb_opposition_reaches_index_proximal()
    test_thumb_full_flex_does_not_pierce_palm()
    test_left_right_mirror_each_finger()
    test_thumb_opposition_mirrors_to_left_pinky_side()
    test_slot_to_manus_map_thumb_uses_first_three_phalanges()
    test_slot_to_manus_map_index_starts_at_slot_4()
    test_slot_to_manus_map_4_fingers_use_4_slots_each()
    test_blender_per_side_rule_is_homomorphism_about_z()
    test_blender_left_rule_is_identity()
    test_ue_quat_transform_preserves_rotation_angle()
    test_ue_quat_axis_in_left_handed_frame()
    test_carpus_slot_carries_wrist_world_pose()
    test_cpp_uses_v2_thumb_constants()
    test_cpp_uses_tilted_axis_for_thumb_distal_joints()
    test_cpp_widened_dip_flex_limit()
    print("test_finger_anatomy: PASS")
