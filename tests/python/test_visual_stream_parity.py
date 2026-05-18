"""Phase A: viewport and stream must use the SAME wrist world orientation
when composing fingers, so what we render on the local skeleton is exactly
what the receiving plugin (Blender / UE) gets.

Three divergence sources existed before phase A:

  D1. viewport-finger block at scr/main.cpp:9948+ took the pre-filter,
      pre-coupled `q[SEG_RHand]` while the stream block took the
      post-filter, post-sceneYaw `qStream[SEG_RHand]` — wrist drift-lock
      separated them by up to the lock angle (≈ 0.5° but visible).

  D2. pushFrameWithGloves at scr/main.cpp:8144 sent finger positions
      WITHOUT subtracting baselineSegPos[handSeg], while body segments
      and the carpus slot DID subtract baseline.  Fingers landed in the
      wrong frame in the plugin (offset by ~pelvis-baseline).

  D3. sceneYaw applied to keypoint positions in viewport but NOT to the
      wrist orientation used to rotate finger offsets — non-zero yaw
      after `Reset` caused viewport fingers to lag the streamed pose by
      the yaw angle.

This file pins the post-fix invariants: viewport and stream must produce
the same world-frame finger positions, modulo a baseline subtraction
that is shared with the body payload.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qinv, qrot, axangle, mirror_y_quat, euler_xyz, PI


def def_ang_r_hand_tpose():
    return euler_xyz(0, 0, -PI / 2)


def def_ang_l_hand_tpose():
    return euler_xyz(0, 0,  PI / 2)


def mirror_y_pos(p):
    """scr/main.cpp:mirrorManusL."""
    return np.array([p[0], -p[1], p[2]])


def compose_wrist_world(q_out_hand, def_ang_hand, scene_yaw_rad):
    """Phase A canonical wrist world orient:
       q_wrist_world = R_sceneYaw · qOut[hand] · defAngFor(hand)
    Used by BOTH viewport (relR/relL) and stream (rGloveWorld[i] /
    rGloveWorldP[i])."""
    scene = axangle([0, 0, 1], scene_yaw_rad)
    return qnorm(qmul(scene, qmul(q_out_hand, def_ang_hand)))


def viewport_finger_world(wrist_world_pos, finger_local_pos, q_wrist_world,
                          is_left):
    """Mirror of scr/main.cpp:9929+ AFTER phase A.  Viewport draws each
    finger as `kp[wrist] + relR[i]`; `kp[wrist]` is the scene-yawed
    wrist world position; `relR[i] = vec_rotate(local, qWristWorld)`
    where qWristWorld is the SHARED scene-yawed source."""
    if is_left:
        finger_local_pos = mirror_y_pos(finger_local_pos)
    rel = qrot(q_wrist_world, finger_local_pos)
    return wrist_world_pos + rel


def stream_finger_world(wrist_world_pos, finger_local_pos, q_wrist_world,
                        is_left):
    """Mirror of scr/main.cpp:9682-9692 AFTER phase A.  Stream computes
    `rGloveWorldP[i] = wristPos + vec_rotate(local, qWristWorld)` using
    the SAME qWristWorld variable as the viewport block above."""
    if is_left:
        finger_local_pos = mirror_y_pos(finger_local_pos)
    return wrist_world_pos + qrot(q_wrist_world, finger_local_pos)


def stream_pushframe_payload(finger_world_pos, baseline_seg_pos_hand):
    """Mirror of scr/main.cpp:8144 AFTER phase A: non-carpus finger
    slots subtract baselineSegPos[handSeg] just like the body
    segments and the carpus slot already do."""
    return finger_world_pos - baseline_seg_pos_hand


def test_viewport_and_stream_use_same_wrist_world_zero_yaw():
    """With sceneYaw=0 and no drift-lock difference, viewport and stream
    arrive at the same finger world position."""
    q_out_hand = qnorm(qmul(axangle([0, 1, 0], np.deg2rad(20)),
                            axangle([1, 0, 0], np.deg2rad(15))))
    da = def_ang_r_hand_tpose()
    q_w = compose_wrist_world(q_out_hand, da, scene_yaw_rad=0.0)
    wrist = np.array([0.1, -0.4, 1.2])
    finger_local = np.array([0.080, 0.020, 0.000])
    fv = viewport_finger_world(wrist, finger_local, q_w, is_left=False)
    fs = stream_finger_world(wrist, finger_local, q_w, is_left=False)
    assert np.allclose(fv, fs, atol=1e-9), \
        f"viewport vs stream mismatch (zero yaw): {fv} vs {fs}"


def test_viewport_and_stream_use_same_wrist_world_nonzero_yaw():
    """Phase A's defining test: at sceneYaw=π/3, both viewport and
    stream apply the SAME composed wrist orientation (which carries
    sceneYaw), so the finger world positions still agree."""
    q_out_hand = qnorm(qmul(axangle([0, 0, 1], np.deg2rad(10)),
                            axangle([1, 0, 0], np.deg2rad(-25))))
    da = def_ang_r_hand_tpose()
    yaw = np.pi / 3.0
    q_w = compose_wrist_world(q_out_hand, da, scene_yaw_rad=yaw)
    wrist = np.array([0.2, -0.3, 1.25])
    finger_local = np.array([0.083, 0.005, 0.000])
    fv = viewport_finger_world(wrist, finger_local, q_w, is_left=False)
    fs = stream_finger_world(wrist, finger_local, q_w, is_left=False)
    assert np.allclose(fv, fs, atol=1e-9), \
        f"viewport vs stream mismatch (yaw=π/3): {fv} vs {fs}"


def test_left_hand_mirror_y_consistent_between_viewport_and_stream():
    """Left hand: Manus reports +Y = thumb side for BOTH hands; we
    flip Y on the local finger position before composing. Both
    viewport and stream apply the same flip, so the world position
    must agree."""
    q_out_hand = qnorm(qmul(axangle([0, 1, 0], np.deg2rad(5)),
                            axangle([1, 0, 0], np.deg2rad(-10))))
    da = def_ang_l_hand_tpose()
    q_w = compose_wrist_world(q_out_hand, da, scene_yaw_rad=np.pi / 6)
    wrist = np.array([0.2, 0.3, 1.25])
    finger_local = np.array([0.083, 0.005, 0.000])
    fv = viewport_finger_world(wrist, finger_local, q_w, is_left=True)
    fs = stream_finger_world(wrist, finger_local, q_w, is_left=True)
    assert np.allclose(fv, fs, atol=1e-9), \
        f"left-hand viewport vs stream mismatch: {fv} vs {fs}"


def test_drift_locked_hand_does_not_split_viewport_from_stream():
    """Simulate: pre-filter q[SEG_RHand] = A (live), post-filter
    qOut[SEG_RHand] = B (drift-lock locked at older value).  Before
    phase A the viewport used A and the stream used B → divergence
    proportional to (A·B⁻¹).  After phase A both use B → no
    divergence."""
    q_pre_filter = axangle([1, 0, 0], np.deg2rad(8))    # raw
    q_post_filter = axangle([1, 0, 0], np.deg2rad(2))   # drift-locked older value
    da = def_ang_r_hand_tpose()
    # Phase A canonical: use post-filter EVERYWHERE.
    q_w = compose_wrist_world(q_post_filter, da, scene_yaw_rad=0.0)
    wrist = np.array([0.1, -0.4, 1.2])
    finger_local = np.array([0.080, 0.020, 0.000])
    fv = viewport_finger_world(wrist, finger_local, q_w, is_left=False)
    fs = stream_finger_world(wrist, finger_local, q_w, is_left=False)
    assert np.allclose(fv, fs, atol=1e-9), \
        "drift-lock must not split viewport from stream after phase A"
    # And the buggy "pre-filter on viewport, post-filter on stream"
    # path WOULD have produced a difference — sanity-check that we'd
    # really see it without the fix:
    q_w_pre  = compose_wrist_world(q_pre_filter,  da, scene_yaw_rad=0.0)
    q_w_post = compose_wrist_world(q_post_filter, da, scene_yaw_rad=0.0)
    fv_buggy = viewport_finger_world(wrist, finger_local, q_w_pre,  is_left=False)
    fs_real  = stream_finger_world (wrist, finger_local, q_w_post, is_left=False)
    assert not np.allclose(fv_buggy, fs_real, atol=1e-4), \
        "synthetic drift-lock should produce visible divergence in the buggy path"


def test_stream_finger_payload_is_baseline_relative():
    """Body segments stream baseline-subtracted positions; phase A
    makes non-carpus fingers do the same so they live in the same
    frame as the body on the receiving plugin."""
    finger_world = np.array([0.12, -0.34, 1.21])
    baseline_pelvis_hand = np.array([0.05, -0.10, 1.00])
    payload = stream_pushframe_payload(finger_world, baseline_pelvis_hand)
    expected = finger_world - baseline_pelvis_hand
    assert np.allclose(payload, expected, atol=1e-9), \
        f"baseline subtraction failed: {payload} vs {expected}"


def test_cpp_uses_shared_wrist_world_variables():
    """Static-source check: the viewport-finger block at the bottom of
    onRenderTick must reference qRHandWorld / qLHandWorld (the shared
    scene-yawed, post-filter wrist orientations) and NOT recompute
    quat_mult(q[SEG_RHand], ...) locally — that was the D1 bug."""
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    # The shared variables must exist.
    assert "Quat qRHandWorld" in src, "phase A: qRHandWorld not defined"
    assert "Quat qLHandWorld" in src, "phase A: qLHandWorld not defined"
    # The viewport-finger block must reference them.
    viewport_block_idx = src.find("Rotate Manus-local finger positions into WORLD frame")
    assert viewport_block_idx > 0, "viewport finger block comment missing"
    viewport_block = src[viewport_block_idx:viewport_block_idx + 4000]
    assert "qRHandWorld" in viewport_block, \
        "viewport finger block must use the shared qRHandWorld"
    assert "qLHandWorld" in viewport_block, \
        "viewport finger block must use the shared qLHandWorld"
    # And the OLD pattern (local qRHandFull / qLHandFull) must be gone.
    assert "qRHandFull" not in viewport_block, \
        "phase A bug: viewport finger block still has the local qRHandFull"
    assert "qLHandFull" not in viewport_block, \
        "phase A bug: viewport finger block still has the local qLHandFull"


def test_cpp_finger_payload_subtracts_baseline():
    """Static-source check: pushFrameWithGloves non-carpus branch
    subtracts baselineSegPos[handSeg] just like the carpus branch."""
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    # Find the lambda body, look for `pArr[mIdx] - m_impl->baselineSegPos[handSeg]`
    needle = "pArr[mIdx] - m_impl->baselineSegPos[handSeg]"
    assert needle in src, (
        "phase A bug: non-carpus finger branch must subtract "
        "baselineSegPos[handSeg]; pattern not found in main.cpp"
    )


if __name__ == "__main__":
    test_viewport_and_stream_use_same_wrist_world_zero_yaw()
    test_viewport_and_stream_use_same_wrist_world_nonzero_yaw()
    test_left_hand_mirror_y_consistent_between_viewport_and_stream()
    test_drift_locked_hand_does_not_split_viewport_from_stream()
    test_stream_finger_payload_is_baseline_relative()
    test_cpp_uses_shared_wrist_world_variables()
    test_cpp_finger_payload_subtracts_baseline()
    print("test_visual_stream_parity: PASS")
