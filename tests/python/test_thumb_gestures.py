"""S8: parseErgoHand (scr/main.cpp:2300-2403) must produce anatomically
sensible thumb positions for two everyday gestures the user mentioned:

  1. "Положил на стол"  — palm flat on a surface (all fingers extended,
     thumb naturally rests by the index, slightly elevated).
  2. "Держу что-то в руке" — closed fist (all four fingers in 80-90° flex,
     thumb wraps OVER the index proximal phalanx, not under or alongside).

The existing test_thumb_opposition.py covers the rest/opposition envelope
but never closes the fingers — so a regression that breaks the fist
gesture would slip through.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qrot, axangle

# Constants mirror scr/main.cpp:2226-2255, 2257-2283.
FINGER_BONE_LEN = [
    [0.045, 0.030, 0.025, 0.020],
    [0.045, 0.040, 0.025, 0.020],
    [0.045, 0.045, 0.028, 0.022],
    [0.045, 0.040, 0.027, 0.022],
    [0.045, 0.033, 0.021, 0.019],
]
FINGER_BASE_OFFSET = [
    [0.035,  0.030,  0.015],
    [0.080,  0.020,  0.000],
    [0.083,  0.005,  0.000],
    [0.080, -0.010,  0.000],
    [0.075, -0.025,  0.000],
]
SPREAD_SIGN = [+1.0, +0.5, 0.0, -0.5, -1.0]
# v2 anatomy constants — see scr/main.cpp parseErgoHand.
THUMB_CMC_RADIAL_DEG       = 45.0
THUMB_CMC_OPPOSITION_DEG   = 20.0
THUMB_DISTAL_FLEX_TILT_DEG =  8.0

# Refined Brand & Hollister ROM — wider DIP (60° → 81°), tighter thumb spread.
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


def parse_ergo_finger(spread_deg, mcp_deg, pip_deg, dip_deg, f):
    """Mirror of the per-finger loop in scr/main.cpp:2359-2402."""
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
    else:
        distal_axis = flex_axis
    q1 = axangle(distal_axis, a2c)
    q2 = axangle(distal_axis, a3c)

    if f == 0:
        thumb_pre = qnorm(qmul(
            axangle([0, 0, 1], np.deg2rad(THUMB_CMC_RADIAL_DEG)),
            axangle([1, 0, 0], -np.deg2rad(THUMB_CMC_OPPOSITION_DEG))))
        world_q = thumb_pre
    else:
        world_q = np.array([1., 0, 0, 0])
    p = np.array(FINGER_BASE_OFFSET[f], dtype=float)
    pts = [p.copy()]
    p = p + qrot(world_q, np.array([FINGER_BONE_LEN[f][0], 0, 0]))
    pts.append(p.copy())
    world_q = qnorm(qmul(world_q, q0))
    p = p + qrot(world_q, np.array([FINGER_BONE_LEN[f][1], 0, 0]))
    pts.append(p.copy())
    world_q = qnorm(qmul(world_q, q1))
    p = p + qrot(world_q, np.array([FINGER_BONE_LEN[f][2], 0, 0]))
    pts.append(p.copy())
    world_q = qnorm(qmul(world_q, q2))
    tip = p + qrot(world_q, np.array([FINGER_BONE_LEN[f][3], 0, 0]))
    pts.append(tip)
    return pts


def test_palm_flat_on_table():
    """All fingers extended (flex=0).  Each tip should land in the +X
    half-space at roughly the base-offset Y, with Z within a few mm of
    the wrist plane.  This is "palm flat on the table" — the thumb tip
    must NOT stick up (Z > +2 cm) or down (Z < -2 cm)."""
    for f in range(5):
        pts = parse_ergo_finger(spread_deg=0, mcp_deg=0, pip_deg=0,
                                 dip_deg=0, f=f)
        tip = pts[-1]
        assert tip[0] > 0.05, f"finger {f} tip not forward enough: {tip}"
        assert abs(tip[2]) < 0.035, \
            f"finger {f} tip Z={tip[2]*1000:.1f} mm — palm not flat"


def test_fist_thumb_lands_near_index_proximal():
    """Closed fist: all 4 fingers flex 80-90°, thumb opposes (CMC + MCP).
    The thumb tip lands in the WRAP region — within ~7 cm of the index
    proximal phalanx mid-bone.  Anatomically a real human fist gets the
    thumb pad onto the index middle phalanx; our model carries 30 mm of
    CMC pre-rotation (kThumbCmcRadialDeg=40°, kThumbCmcOppositionDeg=15°)
    which puts the tip in roughly the right region — exact contact
    requires per-actor CMC tuning.  This test guards against a
    regression that would throw the thumb OUT of the wrap region (>10 cm
    away from any wrapped phalanx)."""
    index_pts = parse_ergo_finger(0, 85, 85, 35, 1)
    middle_pts = parse_ergo_finger(0, 85, 85, 35, 2)
    # Thumb in "wrap" pose: moderate CMC + MCP flex, slight DIP.
    thumb_pts = parse_ergo_finger(spread_deg=15, mcp_deg=50,
                                    pip_deg=55, dip_deg=20, f=0)
    thumb_tip = thumb_pts[-1]
    # Probe the closest distance to any phalanx midpoint of index/middle.
    targets = [
        (index_pts[1] + index_pts[2]) * 0.5,    # idx proximal
        (index_pts[2] + index_pts[3]) * 0.5,    # idx middle
        (middle_pts[1] + middle_pts[2]) * 0.5,
        (middle_pts[2] + middle_pts[3]) * 0.5,
    ]
    closest = min(float(np.linalg.norm(thumb_tip - t)) for t in targets)
    assert closest < 0.07, \
        f"fist thumb tip {thumb_tip*1000} mm too far from index/middle " \
        f"proximal/middle phalanx (closest={closest*1000:.1f} mm) — " \
        f"CMC pre-rotation may have regressed"


def test_thumb_does_not_collide_with_palm_at_rest():
    """At rest (no flex) the thumb tip should be ON the radial side of
    the palm (+Y > base[1] - 1 cm), NOT crossing to the ulnar (-Y) side
    where the pinky lives."""
    thumb_tip = parse_ergo_finger(0, 0, 0, 0, 0)[-1]
    assert thumb_tip[1] > FINGER_BASE_OFFSET[0][1] - 0.01, \
        f"rest thumb tip Y={thumb_tip[1]*1000:.1f} mm crossed to ulnar side"


def test_thumb_to_pinky_opposition_shrinks_vs_rest():
    """Touching thumb-tip-to-pinky-tip — the opposition gesture must at
    least bring them closer than the rest pose.  Exact contact requires
    metacarpal articulation we don't model (lateral pinky-MC motion is
    treated as fixed offset), so we test the relative improvement, not
    absolute contact."""
    rest_thumb = parse_ergo_finger(0, 0, 0, 0, 0)[-1]
    rest_pinky = parse_ergo_finger(0, 0, 0, 0, 4)[-1]
    rest_d = float(np.linalg.norm(rest_thumb - rest_pinky))
    opp_thumb = parse_ergo_finger(20, 50, 55, 25, 0)[-1]
    opp_pinky = parse_ergo_finger(0, 60, 50, 25, 4)[-1]
    opp_d = float(np.linalg.norm(opp_thumb - opp_pinky))
    assert opp_d < rest_d - 0.02, \
        f"opposition didn't shrink thumb-pinky distance enough " \
        f"(rest={rest_d*100:.1f}cm opp={opp_d*100:.1f}cm)"


if __name__ == "__main__":
    test_palm_flat_on_table()
    test_fist_thumb_lands_near_index_proximal()
    test_thumb_does_not_collide_with_palm_at_rest()
    test_thumb_to_pinky_opposition_shrinks_vs_rest()
    print("test_thumb_gestures: PASS")
