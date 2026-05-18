"""S13: palm orientation relative to forearm.  The 17th orange sensor
is mounted on the back of the Manus glove (per user: "последний
оранжевый датчик мы крепим прямо на перчатки").  Two simultaneous data
sources:

  • SEG_RHand / SEG_LHand: world rotation of the hand from the orange
    sensor (after calibration via cand · refInv).  This drives the
    wrist's world frame.
  • Manus ergonomics stream: per-finger flex/spread in the *Manus-local*
    frame (+X fingers forward, +Y thumb side, +Z dorsal).

The composition done at scr/main.cpp:9596-9623 is:

    qWristWorld     = quat_mult(qHandBody, defAngFor(SEG_RHand))
    qFingerWorldR   = quat_mult(qWristWorld, qFingerManusLocal)
    qFingerWorldL   = quat_mult(qWristWorld, mirror_y_quat(qFingerManusLocal))

This test verifies the math invariant: with the hand at T-pose
identity (no rotation from calibration) AND fingers at rest (identity),
the forearm-to-wrist-to-finger world direction is a continuous
straight line.  No "palm folds over wrong" artefacts.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qinv, qrot, axangle, mirror_y_quat, euler_xyz, PI


def def_ang_for(seg_name):
    """Mirror of scr/main.cpp:288 defaultSegAnglesFor('tpose') for the
    segments this test touches."""
    P = PI
    if seg_name == "r_forearm":
        return euler_xyz(0, 0, -P / 2)
    if seg_name == "r_hand":
        return euler_xyz(0, 0, -P / 2)
    if seg_name == "l_forearm":
        return euler_xyz(0, 0, P / 2)
    if seg_name == "l_hand":
        return euler_xyz(0, 0, P / 2)
    return np.array([1., 0, 0, 0])


def t_pose_arm_direction(side):
    """In T-pose the right arm points along world -Y (actor's right
    side), the left arm along world +Y."""
    return np.array([0., -1, 0]) if side == "r" else np.array([0., 1, 0])


def test_rest_palm_continues_forearm_direction_right():
    """T-pose right hand at identity cand → forearm and hand bone
    vectors both point along world -Y.  Hand orientation is the same
    as forearm orientation."""
    cand = np.array([1., 0, 0, 0])
    q_hand_world = qmul(cand, def_ang_for("r_hand"))
    q_fa_world = qmul(cand, def_ang_for("r_forearm"))
    bone_hand = qrot(q_hand_world, np.array([1., 0, 0]))
    bone_fa = qrot(q_fa_world, np.array([1., 0, 0]))
    assert float(np.dot(bone_hand, bone_fa)) > 0.999, \
        f"hand and forearm not aligned at rest: dot={float(np.dot(bone_hand, bone_fa))}"
    expected = t_pose_arm_direction("r")
    assert float(np.dot(bone_hand, expected)) > 0.999


def test_rest_palm_continues_forearm_direction_left():
    """Mirror for left side."""
    cand = np.array([1., 0, 0, 0])
    q_hand_world = qmul(cand, def_ang_for("l_hand"))
    q_fa_world = qmul(cand, def_ang_for("l_forearm"))
    bone_hand = qrot(q_hand_world, np.array([1., 0, 0]))
    bone_fa = qrot(q_fa_world, np.array([1., 0, 0]))
    assert float(np.dot(bone_hand, bone_fa)) > 0.999
    expected = t_pose_arm_direction("l")
    assert float(np.dot(bone_hand, expected)) > 0.999


def test_wrist_twist_does_not_flip_palm_normal():
    """A 30° pronation twist of the right wrist (hand-local +X rotation)
    must NOT flip the dorsal-up palm direction by more than 30°.  The
    palm normal stays roughly perpendicular to the hand bone after a
    wrist twist."""
    cand_twist = axangle([1, 0, 0], np.deg2rad(30))
    q_hand_world = qmul(cand_twist, def_ang_for("r_hand"))
    # Body-hand local +Z = dorsal up.  After the twist, that local +Z
    # should land within ~30° of its T-pose direction.
    palm_normal_world = qrot(q_hand_world, np.array([0., 0, 1]))
    rest_q = def_ang_for("r_hand")
    rest_palm_world = qrot(rest_q, np.array([0., 0, 1]))
    cos_ang = float(np.dot(palm_normal_world, rest_palm_world))
    cos_ang = max(-1.0, min(1.0, cos_ang))
    ang_deg = float(np.degrees(np.arccos(cos_ang)))
    assert ang_deg < 31, \
        f"30° wrist twist rotated palm normal by {ang_deg:.1f}°"


def test_manus_thumb_side_axis_matches_body_hand():
    """In Manus-local: +Y is the thumb side.  In body-hand-local
    (T-pose right hand): +Y in the body frame must also be the thumb
    side (radial direction from middle finger toward thumb).

    For the right hand in T-pose: body +Y → world ??.  T-pose right
    hand defAng = E(0,0,-P/2) rotates body +Y to world +X (forward).
    Anatomical thumb-side of a right hand held palm-down in T-pose
    points forward — ✓.  This test pins the convention."""
    q_hand_world = qmul(np.array([1., 0, 0, 0]), def_ang_for("r_hand"))
    body_y_in_world = qrot(q_hand_world, np.array([0., 1, 0]))
    expected = np.array([1., 0, 0])    # +X world = forward
    assert float(np.dot(body_y_in_world, expected)) > 0.999, \
        f"R-hand body +Y not forward in T-pose: {body_y_in_world}"


def test_manus_y_flip_for_left_hand_makes_thumb_side_consistent():
    """For the left hand, Manus reports the same +Y = thumb side
    convention.  But L-hand body +Y in T-pose points BACKWARD (the
    left thumb is forward, which is body's -Y direction).  Therefore
    finger data must be Y-mirrored before composition.  Verify the
    composition produces the expected world thumb-side for the L
    hand."""
    finger_in_manus_local = axangle([0, 1, 0], np.deg2rad(45))  # 45° flex
    finger_in_left_body = mirror_y_quat(finger_in_manus_local)
    q_lhand_world = qmul(np.array([1., 0, 0, 0]), def_ang_for("l_hand"))
    finger_world = qmul(q_lhand_world, finger_in_left_body)
    # The flex rotates the bone forward/down anatomically — verify the
    # resulting bone direction is in the +Y world half (since L hand
    # extends in +Y world direction in T-pose).
    bone = qrot(finger_world, np.array([1., 0, 0]))
    assert bone[1] > 0.5, \
        f"L-hand finger flex didn't land in +Y world half: {bone}"


def test_cpp_passes_calibrated_qhand_to_finger_composition():
    """Static-source check: scr/main.cpp:9596+ must compose finger world
    rotations via qStream[SEG_RHand] (which is the cand · refInv result
    that includes the orange sensor's calibrated s2s).  If anyone
    changes the composition to use raw quaternion or skip defAngFor,
    fingers will drift relative to the wrist."""
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    # Both lines must reference qStream[SEG_RHand]/[SEG_LHand] AND
    # defAngFor.  The pattern is straight after the gloves enabled check.
    assert "qStream[SEG_RHand]" in src
    assert "qStream[SEG_LHand]" in src
    assert "defAngFor(SEG_RHand)" in src
    assert "defAngFor(SEG_LHand)" in src


if __name__ == "__main__":
    test_rest_palm_continues_forearm_direction_right()
    test_rest_palm_continues_forearm_direction_left()
    test_wrist_twist_does_not_flip_palm_normal()
    test_manus_thumb_side_axis_matches_body_hand()
    test_manus_y_flip_for_left_hand_makes_thumb_side_consistent()
    test_cpp_passes_calibrated_qhand_to_finger_composition()
    print("test_palm_with_glove_sensor: PASS")
