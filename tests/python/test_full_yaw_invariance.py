"""S6: When the actor yaws (rotates about world Z) as a whole, every
joint should produce the SAME body-relative orientation that it produced
before the rotation.  In other words, rotating the input quaternions by
Q_yaw(θ) and the world by Q_yaw(θ) is equivalent to running the pipeline
unchanged (everything moves together).

Pipeline stages exercised:
  • `cand = raw · refInv` (the FK input frame)
  • `constrain_shoulder_cone` (anatomical, body-relative)
  • spine smoothstep slerp (interpolation in world frame)
  • `coupleScapHumeral` (body-frame boost via similarity through pelvis)
  • `coupleWristForearm` (forearm twist coupling, hand local frame)

This test catches the failure mode where one of the helpers still uses
world-axis hardcoded vectors that don't rotate with the body — which
would manifest as bones bending "the wrong way" when the actor turns.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qinv, qrot, axangle, swing_twist, slerp


def smoothstep(x):
    return x * x * (3.0 - 2.0 * x)


def constrain_shoulder_cone(q_seg, q_pelvis, is_right):
    """Mirror of scr/main.cpp:652-729."""
    bone_world = qrot(q_seg, np.array([1., 0, 0]))
    pelvis_frame = qrot(qinv(q_pelvis), bone_world)
    bone_l = float(np.linalg.norm(pelvis_frame))
    if bone_l < 1e-6:
        return q_seg
    n = pelvis_frame / bone_l
    upN, latN, backN = float(n[0]), float(n[1]), float(n[2])
    across = (latN > 0.20) if is_right else (latN < -0.20)
    behind = backN > 0.60
    horiz = abs(upN) < 0.55
    if not (across and behind and horiz):
        return q_seg
    K_BACK_MAX = 0.30
    target = np.array([upN, latN, min(backN, K_BACK_MAX)])
    t_len = float(np.linalg.norm(target))
    if t_len < 1e-6:
        return q_seg
    target = (target / t_len) * bone_l
    target_world = qrot(q_pelvis, target)
    from_v = bone_world / np.linalg.norm(bone_world)
    to_v = target_world / np.linalg.norm(target_world)
    d = float(np.dot(from_v, to_v))
    if d > 0.9999:
        return q_seg
    if d < -0.9999:
        axis = np.cross(np.array([0, 0, 1.]), from_v)
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(np.array([1, 0, 0.]), from_v)
        axis = axis / max(1e-9, np.linalg.norm(axis))
        q_correct = np.array([0.0, axis[0], axis[1], axis[2]])
    else:
        axis = np.cross(from_v, to_v)
        s = np.sqrt((1.0 + d) * 2.0)
        q_correct = qnorm(np.array([0.5 * s, axis[0] / s, axis[1] / s, axis[2] / s]))
    violation = max(0.0, backN - K_BACK_MAX)
    violation = min(violation, 0.60)
    t = violation / 0.60
    strength = 0.30 * t * t * t
    q_partial = slerp(np.array([1., 0, 0, 0]), q_correct, strength)
    return qnorm(qmul(q_partial, q_seg))


def couple_scap_humeral(q_pelvis_world, q_sh_world, q_ua_world, is_right):
    """Mirror of scr/main.cpp:9456-9479 (the in-frame coupling)."""
    arm_dir = qrot(q_ua_world, np.array([1., 0, 0]))
    up_z = max(-1.0, min(1.0, float(arm_dir[2])))
    activate = 0.30
    if up_z < activate:
        return q_sh_world
    normalised = (up_z - activate) / (1.0 - activate)
    scap_ang = normalised * 0.30
    signed = -scap_ang if is_right else scap_ang
    scap_body = axangle([1, 0, 0], signed)
    # Convert body-frame rotation to world via similarity through pelvis.
    scap_world = qnorm(qmul(qmul(q_pelvis_world, scap_body), qinv(q_pelvis_world)))
    return qnorm(qmul(scap_world, q_sh_world))


def couple_wrist_forearm(q_h_world, q_fa_world):
    """Mirror of scr/main.cpp:9483-9505."""
    local_hand = qnorm(qmul(qinv(q_fa_world), q_h_world))
    _, h_tw = swing_twist(local_hand, np.array([1., 0, 0]))
    tx = max(-1.0, min(1.0, h_tw[1]))
    tw = max(0.0, min(1.0, h_tw[0]))
    half = float(np.arctan2(tx, tw))
    fa_add = half * 2.0 * 0.20
    fa_twist_add = axangle([1, 0, 0], fa_add)
    fa_new = qnorm(qmul(q_fa_world, fa_twist_add))
    local_rebal = qnorm(qmul(qinv(fa_twist_add), local_hand))
    h_new = qnorm(qmul(fa_new, local_rebal))
    return h_new, fa_new


def make_test_pose():
    """Return a synthetic 23-segment 'cand' array (post raw · refInv) that
    represents a slightly-non-trivial body pose: arms raised 30°, torso
    leaning forward 10°, head turned 5°.  All numbers chosen to exercise
    each pipeline stage."""
    P = np.pi
    q = [np.array([1., 0, 0, 0]) for _ in range(23)]
    # Pelvis: yaw 0 in baseline test.
    q[0] = np.array([1., 0, 0, 0])
    # T8: leaned forward 10° about body +Y (pitch).
    q[4] = axangle([0, 1, 0], np.deg2rad(10))
    # Head: yawed 5° about world Z.
    q[6] = axangle([0, 0, 1], np.deg2rad(5))
    # Right upper arm: raised 30° about body axis (mostly vertical lift).
    q[8] = axangle([0, -1, 0], np.deg2rad(30))
    # Left upper arm: mirror.
    q[12] = axangle([0, -1, 0], np.deg2rad(30))
    # Forearms: small flex.
    q[9] = axangle([0, 1, 0], np.deg2rad(20))
    q[13] = axangle([0, 1, 0], np.deg2rad(20))
    # Hands: 15° twist.
    q[10] = axangle([1, 0, 0], np.deg2rad(15))
    q[14] = axangle([1, 0, 0], np.deg2rad(-15))
    return q


def run_pipeline(cand):
    """Apply the body-relative stages to `cand` and return the resulting
    23-segment array (post couple + constraint + spine slerp).

    Pelvis world is taken directly from cand[0] (in this simulation we
    treat 'cand' as world-frame body quats because the test rotates
    everything together)."""
    q = [c.copy() for c in cand]
    q_pelvis = q[0]
    # Shoulder cone (S2 stage in pipeline).
    q[8] = constrain_shoulder_cone(q[8], q_pelvis, True)
    q[12] = constrain_shoulder_cone(q[12], q_pelvis, False)
    # Spine smoothstep slerp.
    q[1] = slerp(q[0], q[4], smoothstep(0.22))
    q[2] = slerp(q[0], q[4], smoothstep(0.50))
    q[3] = slerp(q[0], q[4], smoothstep(0.78))
    q[5] = slerp(q[4], q[6], smoothstep(0.62))
    # Couple scap-humeral (operates on hand world via similarity).
    q[7] = couple_scap_humeral(q_pelvis, q[7], q[8], True)
    q[11] = couple_scap_humeral(q_pelvis, q[11], q[12], False)
    # Couple wrist-forearm.
    q[10], q[9] = couple_wrist_forearm(q[10], q[9])
    q[14], q[13] = couple_wrist_forearm(q[14], q[13])
    return q


def rotate_pose_by_world_yaw(cand, theta):
    """Multiply every body quat on the LEFT by Q_yaw(θ) — this represents
    the actor turning by θ about world Z."""
    qy = axangle([0, 0, 1], theta)
    return [qnorm(qmul(qy, c)) for c in cand]


def body_relative_equal(out_base, out_rot, theta, tol_deg=0.5):
    """Compare `out_rot` (pipeline output for yawed input) against
    `out_base` rotated by Q_yaw(θ).  If yaw-invariant, every segment
    should match within tol_deg."""
    qy = axangle([0, 0, 1], theta)
    qy_inv = qinv(qy)
    max_err_deg = 0.0
    worst = -1
    for i in range(23):
        expected = qnorm(qmul(qy, out_base[i]))
        actual = out_rot[i]
        delta = qnorm(qmul(actual, qinv(expected)))
        if delta[0] < 0:
            delta = -delta
        ang = 2.0 * np.degrees(np.arccos(min(1.0, abs(delta[0]))))
        if ang > max_err_deg:
            max_err_deg = ang
            worst = i
    return max_err_deg, worst


def test_yaw_invariance_45deg():
    cand = make_test_pose()
    base = run_pipeline(cand)
    rotated = run_pipeline(rotate_pose_by_world_yaw(cand, np.deg2rad(45)))
    err, worst = body_relative_equal(base, rotated, np.deg2rad(45))
    assert err < 0.5, \
        f"45° yaw: max body-relative error {err:.3f}° on segment {worst}"


def test_yaw_invariance_90deg():
    cand = make_test_pose()
    base = run_pipeline(cand)
    rotated = run_pipeline(rotate_pose_by_world_yaw(cand, np.deg2rad(90)))
    err, worst = body_relative_equal(base, rotated, np.deg2rad(90))
    assert err < 0.5, \
        f"90° yaw: max body-relative error {err:.3f}° on segment {worst}"


def test_yaw_invariance_180deg():
    cand = make_test_pose()
    base = run_pipeline(cand)
    rotated = run_pipeline(rotate_pose_by_world_yaw(cand, np.deg2rad(180)))
    err, worst = body_relative_equal(base, rotated, np.deg2rad(180))
    assert err < 0.5, \
        f"180° yaw: max body-relative error {err:.3f}° on segment {worst}"


def test_yaw_invariance_neg135deg():
    """Negative-direction yaw — covers the alternate branch of the
    yaw-quaternion hemisphere fix in yaw_only_quat."""
    cand = make_test_pose()
    base = run_pipeline(cand)
    rotated = run_pipeline(rotate_pose_by_world_yaw(cand, np.deg2rad(-135)))
    err, worst = body_relative_equal(base, rotated, np.deg2rad(-135))
    assert err < 0.5, \
        f"-135° yaw: max body-relative error {err:.3f}° on segment {worst}"


if __name__ == "__main__":
    test_yaw_invariance_45deg()
    test_yaw_invariance_90deg()
    test_yaw_invariance_180deg()
    test_yaw_invariance_neg135deg()
    print("test_full_yaw_invariance: PASS")
