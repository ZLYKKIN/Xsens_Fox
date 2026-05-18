"""S5: scr/main.cpp applies TWO wrist-related operations on the same
frame:

  1. `coupleWristForearm` (MainWindow::onRenderTick:9483) — siphons 20%
     of the hand-local twist into the forearm so the visual forearm
     rotates with pronation/supination.

  2. `constrain_wrist_twist` (MocapViewport::updatePose:7253) — clamps
     hand local flex/lat-dev/twist to anatomical limits.

The order in the rendered frame is: (1) couple in onRenderTick → push
into viewport.updatePose → (2) constrain inside updatePose.  The
critical question: does the constraint *re-clamp* the same twist that
the couple already moved, double-counting and shrinking the legitimate
twist below what the user produced?

This test mirrors both operations in Python and asserts that:
  • A within-limits twist survives both passes (no double clamping).
  • An over-the-limit twist is correctly clamped exactly once.
  • A pure flex (no twist) passes through couple unchanged because
    coupleWristForearm only touches the twist component.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qinv, axangle, swing_twist


MAX_FLEX = np.pi * 0.5
MAX_DEV = np.pi / 6.0
TWIST_W = 1.0
COUPLE_FRACTION = 0.20   # from coupleWristForearm:9494


def couple_wrist_forearm(q_hand_world, q_forearm_world):
    """Mirror of scr/main.cpp:9483-9505.  Returns (new_hand_world,
    new_forearm_world).  20% of the hand's local twist moves into the
    forearm; the hand-local relative orientation is rebalanced so the
    visible hand orientation is unchanged."""
    local_hand = qnorm(qmul(qinv(q_forearm_world), q_hand_world))
    _, h_twist = swing_twist(local_hand, np.array([1., 0, 0]))
    tx = max(-1.0, min(1.0, h_twist[1]))
    tw = max(0.0, min(1.0, h_twist[0]))
    twist_half = float(np.arctan2(tx, tw))
    fa_additional = twist_half * 2.0 * COUPLE_FRACTION
    fa_twist_add = axangle([1, 0, 0], fa_additional)
    fa_new = qnorm(qmul(q_forearm_world, fa_twist_add))
    fa_twist_inv = qinv(fa_twist_add)
    local_rebal = qnorm(qmul(fa_twist_inv, local_hand))
    h_new = qnorm(qmul(fa_new, local_rebal))
    return h_new, fa_new


def constrain_wrist_twist(q_hand_world, q_forearm_world,
                           max_flex=MAX_FLEX, max_dev=MAX_DEV,
                           twist_weight=TWIST_W):
    """Mirror of scr/main.cpp:731-774."""
    q_fa_inv = qinv(q_forearm_world)
    q_local = qnorm(qmul(q_fa_inv, q_hand_world))
    if q_local[0] < 0:
        q_local = -q_local
    swing, twist = swing_twist(q_local, np.array([1., 0, 0]))
    twist_half = float(np.arctan2(twist[1], twist[0]))
    twist_ang = 2.0 * twist_half * twist_weight
    twist_out = np.array([np.cos(twist_ang * 0.5),
                           np.sin(twist_ang * 0.5), 0, 0])
    if swing[0] < 0:
        swing = -swing
    sxy = float(np.sqrt(swing[2]**2 + swing[3]**2))
    swing_ang = 2.0 * float(np.arctan2(sxy, swing[0]))
    ay = swing[2] / sxy if sxy > 1e-9 else 0.0
    az = swing[3] / sxy if sxy > 1e-9 else 0.0
    flex = swing_ang * ay
    dev = swing_ang * az
    flex_c = max(-max_flex, min(max_flex, flex))
    dev_c = max(-max_dev, min(max_dev, dev))
    new_ang = float(np.sqrt(flex_c**2 + dev_c**2))
    if new_ang > 1e-9:
        half = new_ang * 0.5
        s = np.sin(half) / new_ang
        swing_out = np.array([np.cos(half), 0, flex_c * s, dev_c * s])
    else:
        swing_out = np.array([1., 0, 0, 0])
    q_local_out = qnorm(qmul(swing_out, twist_out))
    return qnorm(qmul(q_forearm_world, q_local_out))


def pipeline(q_hand_world, q_forearm_world):
    """Full per-frame order: couple → constrain.  Returns final hand world."""
    h1, fa1 = couple_wrist_forearm(q_hand_world, q_forearm_world)
    h2 = constrain_wrist_twist(h1, fa1)
    return h2, fa1


def twist_deg(q_hand_world, q_forearm_world):
    """Recover the residual hand-local twist in degrees."""
    local = qnorm(qmul(qinv(q_forearm_world), q_hand_world))
    if local[0] < 0:
        local = -local
    _, twist = swing_twist(local, np.array([1., 0, 0]))
    half = float(np.arctan2(twist[1], abs(twist[0])))
    return 2.0 * np.degrees(half)


def world_hand_twist_deg(q_hand_world):
    """Twist of the hand WORLD orientation about world +X (assuming
    forearm aligned to +X — only valid in this test's setup)."""
    _, t = swing_twist(q_hand_world, np.array([1., 0, 0]))
    half = float(np.arctan2(t[1], abs(t[0])))
    return 2.0 * np.degrees(half)


def test_in_limits_twist_visible_unchanged_in_world():
    """A 30° wrist twist (within all limits) is split by couple as 6°
    forearm + 24° hand-local, then the constraint passes it through
    unchanged at twist_weight=1.0.  The *visible* (world-frame) hand
    rotation must remain 30°, otherwise the user feels the wrist
    snapped back.  coupleWristForearm's rebalance is what guarantees
    this — without the rebalance the world hand would land at 6° + 24°
    × world_x = 30° only by coincidence of axis-alignment."""
    forearm = np.array([1., 0, 0, 0])
    hand = axangle([1, 0, 0], np.deg2rad(30))
    h_out, fa_out = pipeline(hand, forearm)
    visible_twist = abs(world_hand_twist_deg(h_out))
    assert abs(visible_twist - 30) < 2, \
        f"visible 30° wrist twist became {visible_twist:.1f}° in world — " \
        f"couple+constrain double-applied or rebalance broken"


def test_over_limit_flex_clamped_once():
    """120° forward flex must clamp to 90° (MAX_FLEX) and not get
    re-clamped to something less by a second pass.  The couple stage
    doesn't touch flex, only twist."""
    forearm = np.array([1., 0, 0, 0])
    hand = axangle([0, 1, 0], np.deg2rad(120))
    h_out, fa_out = pipeline(hand, forearm)
    local = qnorm(qmul(qinv(fa_out), h_out))
    swing, _ = swing_twist(local, np.array([1., 0, 0]))
    sxy = float(np.sqrt(swing[2]**2 + swing[3]**2))
    flex_ang = 2.0 * float(np.arctan2(sxy, abs(swing[0])))
    assert abs(flex_ang - MAX_FLEX) < np.deg2rad(2), \
        f"flex clamped to {np.degrees(flex_ang):.1f}°, expected ~90°"


def test_pure_flex_no_twist_drift():
    """30° pure forward flex (zero twist input).  Couple should not
    introduce any twist since the hand-local twist is zero; constrain
    is a no-op.  The output flex stays 30°, twist stays 0."""
    forearm = np.array([1., 0, 0, 0])
    hand = axangle([0, 1, 0], np.deg2rad(30))
    h_out, fa_out = pipeline(hand, forearm)
    out_twist = abs(twist_deg(h_out, fa_out))
    assert out_twist < 1.0, \
        f"pure flex leaked into twist ({out_twist:.2f}°) — coupling is " \
        f"unstable on non-twist inputs"


def test_couple_then_constrain_equivalent_to_constrain_then_couple():
    """If both orderings give the same final visible hand orientation
    (within tolerance), the current pipeline order is fine.  This is a
    sanity check that the operations are approximately commutative for
    within-limit values; if they aren't, S5 needs a deeper investigation."""
    forearm = np.array([1., 0, 0, 0])
    hand = qnorm(qmul(axangle([0, 1, 0], np.deg2rad(25)),
                       axangle([1, 0, 0], np.deg2rad(20))))
    # Path A: couple → constrain (current C++ order).
    a_h, a_fa = pipeline(hand, forearm)
    # Path B: constrain → couple.
    h1 = constrain_wrist_twist(hand, forearm)
    b_h, b_fa = couple_wrist_forearm(h1, forearm)
    # Compare hand world orientations.
    diff = float(np.linalg.norm(a_h - b_h))
    diff = min(diff, float(np.linalg.norm(a_h + b_h)))
    assert diff < 0.02, \
        f"order-dependence in wrist pipeline: |Δq| = {diff:.3f}"


if __name__ == "__main__":
    test_in_limits_twist_visible_unchanged_in_world()
    test_over_limit_flex_clamped_once()
    test_pure_flex_no_twist_drift()
    test_couple_then_constrain_equivalent_to_constrain_then_couple()
    print("test_wrist_constraint_order: PASS")
