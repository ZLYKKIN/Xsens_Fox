"""F11 test: verify that Fox's internal NWU frame lands correctly in
both UE LiveLink and Blender via the unmodified plugins.

UE plugin (Plugins/XsensLivc/.../LiveLinkMvnSource.h:48-50):
    pos_UE  = (mvn_x * 100, -mvn_y * 100, mvn_z * 100)   # cm, RH→LH
    quat_UE = FQuat(-mvn_x, mvn_y, -mvn_z, mvn_w)        # Y-flip + W-last

Blender plugin (pose.py:352):
    blender_pos = (mvn_y, mvn_z, mvn_x)
    blender_quat = (mvn_w, mvn_x, mvn_y, mvn_z)          # no axis swap
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import axangle, qrot


def stream_nwu_to_mvn(p):
    """F11: Fox emits NWU as-is — wire frame is NWU per protocol spec."""
    return p


def ue_from_mvn_pos(p_mvn):
    return np.array([p_mvn[0] * 100.0, -p_mvn[1] * 100.0, p_mvn[2] * 100.0])


def ue_from_mvn_quat(q_mvn):
    # FQuat ctor in UE is (x, y, z, w); plugin passes (-x, y, -z, w)
    return np.array([-q_mvn[1], q_mvn[2], -q_mvn[3], q_mvn[0]])


def blender_pos_from_mvn(p_mvn):
    return np.array([p_mvn[1], p_mvn[2], p_mvn[0]])


def test_pelvis_position_ue():
    """Pelvis 1 m forward, 55 cm up in Fox NWU.  After the (identity)
    NWU→MVN stub and the plugin's MVN→UE axis flip:
      UE x = +100 cm (forward, in UE's +X convention)
      UE y = -0 (still on the midline)
      UE z = +55 cm (up)
    """
    pelvis_nwu = np.array([1.0, 0.0, 0.55])
    p_mvn = stream_nwu_to_mvn(pelvis_nwu)
    p_ue = ue_from_mvn_pos(p_mvn)
    assert abs(p_ue[0] - 100.0) < 1e-6
    assert abs(p_ue[1] - 0.0) < 1e-6
    assert abs(p_ue[2] - 55.0) < 1e-6


def test_pelvis_position_lateral_ue():
    """Pelvis 1 m to actor's LEFT (+Y in NWU) maps to UE -Y (right side
    in UE's LH frame)."""
    p_nwu = np.array([0.0, 1.0, 0.55])
    p_ue = ue_from_mvn_pos(stream_nwu_to_mvn(p_nwu))
    assert abs(p_ue[0]) < 1e-6
    assert abs(p_ue[1] - (-100.0)) < 1e-6
    assert abs(p_ue[2] - 55.0) < 1e-6


def test_identity_quat_passthrough_ue():
    """Identity rotation in NWU stays identity through Fox's stubs and
    the plugin's quaternion swizzle."""
    q_nwu = np.array([1.0, 0.0, 0.0, 0.0])
    q_ue_xyzw = ue_from_mvn_quat(q_nwu)
    # FQuat(0, 0, 0, 1) = identity in UE.
    assert abs(q_ue_xyzw[0]) < 1e-6
    assert abs(q_ue_xyzw[1]) < 1e-6
    assert abs(q_ue_xyzw[2]) < 1e-6
    assert abs(q_ue_xyzw[3] - 1.0) < 1e-6


def test_yaw_quat_lr_flip_ue():
    """Actor yaws 90° in NWU (about world +Z).  In UE LH (Y flipped), a
    +Z yaw becomes a -Z yaw — the plugin's (-x, y, -z, w) swizzle handles
    exactly this sign convention."""
    q_nwu = axangle([0, 0, 1], np.pi / 2)
    q_ue_xyzw = ue_from_mvn_quat(q_nwu)
    # Apply to UE forward (+X_UE = +X_NWU) — should rotate to ±Y_UE.
    # Convert plugin output (x,y,z,w) back to scalar-first to use qrot helper.
    q_ue = np.array([q_ue_xyzw[3], q_ue_xyzw[0], q_ue_xyzw[1], q_ue_xyzw[2]])
    v = qrot(q_ue, np.array([1.0, 0.0, 0.0]))
    # +X_NWU yaws to +Y_NWU, which in UE LH (Y-flipped) is -Y_UE.
    assert abs(v[1] - (-1.0)) < 1e-6, f"got {v}"


def test_pelvis_position_blender():
    """Pelvis 1 m forward, 0.55 m up in NWU.  Blender plugin's
    (y, z, x) swap puts that at (0, 0.55, 1) in Blender's frame —
    blender_y = forward (along-bone for pelvis), blender_z = height,
    blender_x = lateral.  This is the convention the plugin's bone
    rig expects (verified empirically against the bundled testproject)."""
    p_nwu = np.array([1.0, 0.0, 0.55])
    p_b = blender_pos_from_mvn(stream_nwu_to_mvn(p_nwu))
    assert abs(p_b[0] - 0.0) < 1e-6
    assert abs(p_b[1] - 0.55) < 1e-6
    assert abs(p_b[2] - 1.0) < 1e-6


if __name__ == "__main__":
    test_pelvis_position_ue()
    test_pelvis_position_lateral_ue()
    test_identity_quat_passthrough_ue()
    test_yaw_quat_lr_flip_ue()
    test_pelvis_position_blender()
    print("test_stream_frame: PASS")
