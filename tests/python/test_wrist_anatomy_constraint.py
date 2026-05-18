"""§B test: constrain_wrist_twist (scr/main.cpp:721-764) must keep the
hand within physiological flex / lateral deviation / twist limits.

Mirrors the C++ math in Python and feeds extreme synthetic poses.  The
constraint is computed in the forearm-local frame via swing-twist
decomposition along forearm +X.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qrot, qinv, axangle


def swing_twist(q, axis):
    """Mirror of scr/main.cpp:204 swingTwistDecompose."""
    w, x, y, z = q
    dot = x*axis[0] + y*axis[1] + z*axis[2]
    twist = np.array([w, dot*axis[0], dot*axis[1], dot*axis[2]])
    n2 = float(np.dot(twist, twist))
    if n2 < 1e-24:
        return np.array([1.0, 0, 0, 0]), q.copy()
    twist = twist / np.sqrt(n2)
    if twist[0] < 0:
        twist = -twist
    swing = qnorm(qmul(q, np.array([twist[0], -twist[1], -twist[2], -twist[3]])))
    return swing, twist


def constrain_wrist_twist(q_hand_world, q_forearm_world,
                          max_flex_rad, max_lat_dev_rad, twist_weight):
    """Mirror of scr/main.cpp:721-764."""
    qFAinv = np.array([q_forearm_world[0], -q_forearm_world[1],
                       -q_forearm_world[2], -q_forearm_world[3]])
    qFAinv = qFAinv / float(np.dot(q_forearm_world, q_forearm_world))
    qLocal = qnorm(qmul(qFAinv, q_hand_world))
    if qLocal[0] < 0:
        qLocal = -qLocal
    swing, twist = swing_twist(qLocal, np.array([1.0, 0.0, 0.0]))

    twist_half = float(np.arctan2(twist[1], twist[0]))
    twist_ang = 2.0 * twist_half * twist_weight
    twist_out = np.array([np.cos(twist_ang * 0.5),
                          np.sin(twist_ang * 0.5), 0.0, 0.0])

    if swing[0] < 0:
        swing = -swing
    sxy = float(np.sqrt(swing[2]**2 + swing[3]**2))
    swing_ang = 2.0 * float(np.arctan2(sxy, swing[0]))
    ay = swing[2] / sxy if sxy > 1e-9 else 0.0
    az = swing[3] / sxy if sxy > 1e-9 else 0.0
    flex_ang = swing_ang * ay
    dev_ang  = swing_ang * az
    flex_c = max(-max_flex_rad,    min( max_flex_rad,    flex_ang))
    dev_c  = max(-max_lat_dev_rad, min( max_lat_dev_rad, dev_ang))
    new_ang = float(np.sqrt(flex_c**2 + dev_c**2))
    if new_ang > 1e-9:
        half = new_ang * 0.5
        s = float(np.sin(half) / new_ang)
        swing_out = np.array([np.cos(half), 0.0, flex_c*s, dev_c*s])
    else:
        swing_out = np.array([1.0, 0, 0, 0])
    qLocalOut = qnorm(qmul(swing_out, twist_out))
    return qnorm(qmul(q_forearm_world, qLocalOut))


def local_after_constraint(q_hand, q_forearm, max_flex, max_dev, tw):
    out = constrain_wrist_twist(q_hand, q_forearm, max_flex, max_dev, tw)
    qFAinv = np.array([q_forearm[0], -q_forearm[1], -q_forearm[2], -q_forearm[3]])
    qFAinv = qFAinv / float(np.dot(q_forearm, q_forearm))
    return qnorm(qmul(qFAinv, out))


# Default cfg from scr/main.h:113-118.
MAX_FLEX = np.pi * 0.5
MAX_DEV  = np.pi / 6.0
TWIST_W  = 1.0


def test_extreme_flex_clamped():
    """Hand flexed 120° forward — must be clamped to ≤90° (maxFlexRad)."""
    forearm = np.array([1.0, 0, 0, 0])
    hand    = axangle([0, 1, 0], np.deg2rad(120))
    local   = local_after_constraint(hand, forearm, MAX_FLEX, MAX_DEV, TWIST_W)
    swing, _ = swing_twist(local, np.array([1.0, 0.0, 0.0]))
    sxy = float(np.sqrt(swing[2]**2 + swing[3]**2))
    swing_ang = 2.0 * float(np.arctan2(sxy, abs(swing[0])))
    ay = swing[2] / sxy if sxy > 1e-9 else 0.0
    flex = swing_ang * ay
    assert abs(flex) <= MAX_FLEX + 1e-6, f"flex {np.degrees(flex):.1f}° exceeds limit"


def test_extreme_lateral_clamped():
    """Hand laterally deviated 45° — must be clamped to ≤30°."""
    forearm = np.array([1.0, 0, 0, 0])
    hand    = axangle([0, 0, 1], np.deg2rad(45))
    local   = local_after_constraint(hand, forearm, MAX_FLEX, MAX_DEV, TWIST_W)
    swing, _ = swing_twist(local, np.array([1.0, 0.0, 0.0]))
    sxy = float(np.sqrt(swing[2]**2 + swing[3]**2))
    swing_ang = 2.0 * float(np.arctan2(sxy, abs(swing[0])))
    az = swing[3] / sxy if sxy > 1e-9 else 0.0
    dev = swing_ang * az
    assert abs(dev) <= MAX_DEV + 1e-6, f"dev {np.degrees(dev):.1f}° exceeds limit"


def test_identity_passthrough():
    """No relative rotation — output equals forearm orientation, hand-local
    quaternion stays identity."""
    forearm = axangle([0, 1, 0], np.deg2rad(35))  # arbitrary forearm yaw
    hand    = forearm
    local   = local_after_constraint(hand, forearm, MAX_FLEX, MAX_DEV, TWIST_W)
    # local must be identity (or near-identity).
    if local[0] < 0:
        local = -local
    assert local[0] > 1.0 - 1e-6, f"identity passthrough broken: {local}"


def test_twist_scaling():
    """With twist_weight = 0.5, a 90° twist along forearm X should come out
    as 45° twist."""
    forearm = np.array([1.0, 0, 0, 0])
    hand    = axangle([1, 0, 0], np.deg2rad(90))
    local   = local_after_constraint(hand, forearm, MAX_FLEX, MAX_DEV, 0.5)
    _, twist = swing_twist(local, np.array([1.0, 0.0, 0.0]))
    twist_ang = 2.0 * float(np.arctan2(twist[1], abs(twist[0])))
    assert abs(abs(twist_ang) - np.deg2rad(45)) < np.deg2rad(2), \
        f"twist_weight=0.5 should give 45°, got {np.degrees(twist_ang):.1f}°"


def test_within_limits_unchanged():
    """Hand flexed 30° (within 90° limit) and twisted 20° — output should
    match input up to the swing-twist round-trip."""
    forearm = np.array([1.0, 0, 0, 0])
    hand    = qnorm(qmul(axangle([1, 0, 0], np.deg2rad(20)),
                         axangle([0, 1, 0], np.deg2rad(30))))
    local   = local_after_constraint(hand, forearm, MAX_FLEX, MAX_DEV, TWIST_W)
    swing, twist = swing_twist(local, np.array([1.0, 0.0, 0.0]))
    sxy = float(np.sqrt(swing[2]**2 + swing[3]**2))
    flex = 2.0 * float(np.arctan2(sxy, abs(swing[0])))
    twist_ang = 2.0 * abs(float(np.arctan2(twist[1], abs(twist[0]))))
    assert abs(flex - np.deg2rad(30)) < np.deg2rad(2), \
        f"30° flex passthrough broken: {np.degrees(flex):.1f}°"
    assert abs(twist_ang - np.deg2rad(20)) < np.deg2rad(2), \
        f"20° twist passthrough broken: {np.degrees(twist_ang):.1f}°"


if __name__ == "__main__":
    test_identity_passthrough()
    test_extreme_flex_clamped()
    test_extreme_lateral_clamped()
    test_twist_scaling()
    test_within_limits_unchanged()
    print("test_wrist_anatomy_constraint: PASS")
