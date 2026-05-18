"""F8 test: shoulder cone constraint must NOT fire on natural reaches
(side-back, holster reach, backhand) — only on the truly-behind-the-back
arm pose that crosses to the opposite shoulder side."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qrot, axangle, qmul, qinv, euler_xyz, PI


def constrain_activates(boneWorld, qPelvis, isRight):
    """Mirror of scr/main.cpp:constrain_shoulder_cone — the three-flag
    gate (post-F8 inequality)."""
    # pelvis-local = qPelvis.inv() * boneWorld
    pelvisFrame = qrot(np.array([qPelvis[0], -qPelvis[1], -qPelvis[2], -qPelvis[3]]), boneWorld)
    boneL = float(np.linalg.norm(pelvisFrame))
    if boneL < 1e-6:
        return False
    n = pelvisFrame / boneL
    upN, latN, backN = n[0], n[1], n[2]
    # F8: across-midline is OPPOSITE side, not own side.
    acrossMid = (latN > 0.20) if isRight else (latN < -0.20)
    clearlyBehind = backN > 0.60
    inHorizBand = abs(upN) < 0.55
    return acrossMid and clearlyBehind and inHorizBand


def pelvis_world_identity():
    """In Fox's NWU + defAng_pelvis = Rot_y(-π/2) convention, an identity
    raw pelvis means oriented[Pelvis] = defAng[Pelvis] = Rot_y(-π/2)."""
    return euler_xyz(0, -PI / 2, 0)


def make_arm_dir(world_dir):
    """Return a quaternion that rotates body-local +X to `world_dir`."""
    src = np.array([1.0, 0.0, 0.0])
    d = np.array(world_dir, dtype=float)
    d = d / np.linalg.norm(d)
    dot = float(np.clip(np.dot(src, d), -1, 1))
    if dot > 0.9999:
        return np.array([1., 0, 0, 0])
    if dot < -0.9999:
        return axangle([0, 0, 1], np.pi)
    axis = np.cross(src, d)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(dot)
    return axangle(axis.tolist(), angle)


def test_tpose_no_clamp():
    """R-arm in T-pose (pointing world -Y) is NOT across the midline."""
    qPel = pelvis_world_identity()
    qArm = make_arm_dir([0, -1, 0])
    boneW = qrot(qArm, np.array([1.0, 0.0, 0.0]))
    assert not constrain_activates(boneW, qPel, True)


def test_back_swing_own_side_no_clamp():
    """R-arm reaching back on its own side — no longer triggers (F8 fix)."""
    qPel = pelvis_world_identity()
    qArm = make_arm_dir([-1, -1, 0])  # back-right (own side)
    boneW = qrot(qArm, np.array([1.0, 0.0, 0.0]))
    assert not constrain_activates(boneW, qPel, True), \
        "R arm reaching back on own side should NOT trigger after F8"


def test_crossing_midline_behind_does_clamp():
    """R-arm reaching across body to actor's left AND back is the legit
    weird-pose case where we want soft clamping."""
    qPel = pelvis_world_identity()
    # Direction: left (+Y_world) + back (-X_world), but with up component
    # small enough to land in horizontal band.
    qArm = make_arm_dir([-0.7, 0.5, 0.0])
    boneW = qrot(qArm, np.array([1.0, 0.0, 0.0]))
    # pelvisFrame = inv(Rot_y(-π/2)) · world = Rot_y(+π/2) · world
    # Rot_y(+π/2): x→-z, z→x ; so (-0.7, 0.5, 0)  →  (0, 0.5, 0.7)
    # latN=+0.5 (>0.2 ✓ for R), backN=+0.7 (>0.6 ✓), upN=0 (|0|<0.55 ✓)
    assert constrain_activates(boneW, qPel, True), \
        "R arm crossing left + behind should trigger after F8"


def test_l_arm_symmetric():
    """L-arm: crossing to actor's RIGHT (-Y_world) + back should clamp."""
    qPel = pelvis_world_identity()
    qArm = make_arm_dir([-0.7, -0.5, 0.0])
    boneW = qrot(qArm, np.array([1.0, 0.0, 0.0]))
    assert constrain_activates(boneW, qPel, False), \
        "L arm crossing right + behind should trigger after F8"


if __name__ == "__main__":
    test_tpose_no_clamp()
    test_back_swing_own_side_no_clamp()
    test_crossing_midline_behind_does_clamp()
    test_l_arm_symmetric()
    print("test_shoulder_cone: PASS")
