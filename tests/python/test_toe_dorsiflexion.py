"""Phase D: q[SEG_*Toe] gets +25° dorsiflexion around foot-local Y when
heel is up and toe-ball is down (toe-off phase of gait).  All other
phases: q_toe = q_foot.

This pins the lookup table in MainWindow::onRenderTick around scr/main.cpp:9572.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qrot, axangle


DORSI_DEG = 25.0


def toe_orient(q_foot, heel_down, toe_down):
    """Mirror of scr/main.cpp phase-D logic.  Sign: foot body frame is
    X=forward, Y=lateral, Z=up; rotation by +θ around +Y tilts +X
    toward -Z (toe down = plantarflexion).  Dorsiflexion (toe UP) is
    therefore -25° around +Y."""
    if toe_down and not heel_down:
        dorsi = axangle([0, 1, 0], np.deg2rad(-DORSI_DEG))
        return qnorm(qmul(q_foot, dorsi))
    return q_foot


def test_midstance_toe_is_parallel_to_foot():
    """Both heel and toe down — no dorsi correction."""
    qf = axangle([0, 1, 0], np.deg2rad(5))  # small pitch
    qt = toe_orient(qf, heel_down=True, toe_down=True)
    assert np.allclose(qt, qf), f"midstance toe must equal foot, got delta {qt - qf}"


def test_heel_strike_toe_is_parallel_to_foot():
    """Heel down, toe up — also no dorsi (toe still parallel)."""
    qf = axangle([0, 1, 0], np.deg2rad(-3))
    qt = toe_orient(qf, heel_down=True, toe_down=False)
    assert np.allclose(qt, qf), "heel-strike toe must equal foot"


def test_toe_off_applies_dorsi():
    """Heel up, toe down — toe rotates -25 deg around foot-local Y so the
    toe-tip lifts UP (dorsiflexion).  With +Y rotation tilting +X toward
    -Z, a -25° rotation tilts +X toward +Z."""
    qf = np.array([1., 0, 0, 0])  # identity foot
    qt = toe_orient(qf, heel_down=False, toe_down=True)
    bone_dir = qrot(qt, np.array([1.0, 0, 0]))
    expected_x = float(np.cos(np.deg2rad(DORSI_DEG)))
    expected_z = float(np.sin(np.deg2rad(DORSI_DEG)))  # +Z = up
    assert abs(bone_dir[0] - expected_x) < 1e-6, \
        f"toe-off X component {bone_dir[0]}, want {expected_x}"
    assert abs(bone_dir[2] - expected_z) < 1e-6, \
        f"toe-off Z component (must be UP, +sin25) {bone_dir[2]}, want {expected_z}"


def test_airborne_toe_is_parallel_to_foot():
    """Both heel and toe up (swing) — fallback q_toe = q_foot."""
    qf = axangle([1, 0, 0], np.deg2rad(15))
    qt = toe_orient(qf, heel_down=False, toe_down=False)
    assert np.allclose(qt, qf), "swing toe must equal foot"


def test_dorsi_is_in_foot_local_frame_not_world():
    """When the foot is yawed, the dorsi correction must follow the foot
    (body-local Y), not stay world-aligned.  Foot yawed 90° around +Z
    sends body +X to world +Y; the toe-bone after a body-frame dorsi
    must still be in the (world +Y, world +Z) half-plane with a positive
    Z component (toe lifting UP)."""
    qf = axangle([0, 0, 1], np.deg2rad(90))  # foot yawed 90 deg
    qt = toe_orient(qf, heel_down=False, toe_down=True)
    bone_dir = qrot(qt, np.array([1.0, 0, 0]))
    assert abs(bone_dir[1] - np.cos(np.deg2rad(DORSI_DEG))) < 1e-5, \
        f"yawed-foot dorsi: Y component {bone_dir[1]}"
    assert abs(bone_dir[2] - np.sin(np.deg2rad(DORSI_DEG))) < 1e-5, \
        f"yawed-foot dorsi: Z component {bone_dir[2]}"
    assert abs(bone_dir[0]) < 1e-5, \
        f"yawed-foot dorsi: leaked into world-X by {bone_dir[0]}"


def test_cpp_uses_phase_d_dorsiflexion():
    """Static-source check: scr/main.cpp:onRenderTick must contain the
    25 deg dorsi axisAngleQuat around body-Y, gated on (!heelDown &&
    toeDown).  Tracking this fixes the table in case future refactors
    accidentally drop the toe-off phase."""
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    idx = src.find("Phase D: toe dorsiflexion during toe-off")
    assert idx > 0, "phase D comment block missing — was the implementation reverted?"
    window = src[idx:idx + 2000]
    assert "qDegreesToRadians(-25.0)" in window, \
        "phase D: -25 deg dorsi (toe UP) missing from onRenderTick"
    assert "axisAngleQuat(QVector3D(0, 1, 0)" in window, \
        "phase D: dorsi must rotate around body-local Y"
    assert "cs.rToeDown && !cs.rHeelDown" in window, \
        "phase D: right toe gate must be (toeDown && !heelDown)"
    assert "cs.lToeDown && !cs.lHeelDown" in window, \
        "phase D: left toe gate must be (toeDown && !heelDown)"


if __name__ == "__main__":
    test_midstance_toe_is_parallel_to_foot()
    test_heel_strike_toe_is_parallel_to_foot()
    test_toe_off_applies_dorsi()
    test_airborne_toe_is_parallel_to_foot()
    test_dorsi_is_in_foot_local_frame_not_world()
    test_cpp_uses_phase_d_dorsiflexion()
    print("test_toe_dorsiflexion: PASS")
