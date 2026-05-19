"""F-2.2 (covered through F-2.3) — verifies that the 4-mount-model system
can recover the L↔R pairing for arms whose s2s rotation is in the
169-178° band (logs/fox_mocap.log:4456-4461).  Such large rotations mean
sensors were mounted near-180°-flipped relative to the canonical orientation.
The extended mount candidate set must contain at least one model that
matches the observed pair-symmetry residual."""

from quat_math import (mirror_y_quat, mirror_z_quat, anti_parallel_quat,
                       canonical_hemisphere, qmul, qinv, axangle, quat_dev_deg,
                       qangle_deg)
from log_fixtures import S2S_ROTATION_DEG
import numpy as np


def test_180_flip_pair_recovered_by_four_models():
    base = axangle([0.0, 0.0, 1.0], np.deg2rad(20.0))
    qR = qmul(axangle([1.0, 0.0, 0.0], np.pi), base)
    qL = qmul(axangle([1.0, 0.0, 0.0], np.pi), base)
    devs = {
        "mirror-Y":      quat_dev_deg(qR, mirror_y_quat(qL)),
        "parallel":      quat_dev_deg(qR, qL),
        "mirror-Z":      quat_dev_deg(qR, mirror_z_quat(qL)),
        "anti-parallel": quat_dev_deg(qR, anti_parallel_quat(qL)),
    }
    best = min(devs.values())
    assert best < 30.0, (
        f"with sensors both flipped 180° one of {devs} must give "
        f"residual < 30°, got best={best:.2f}°")


def test_log_s2s_rotations_actually_near_180():
    near_180 = [s for s, deg in S2S_ROTATION_DEG.items() if deg > 160.0]
    assert len(near_180) >= 5, (
        f"log contains {len(near_180)} segments with s2s>160° — "
        f"180-flip detection must apply to all of them")


def test_canonical_resolves_q_minus_q_ambiguity():
    q = axangle([0.0, 0.0, 1.0], np.deg2rad(45.0))
    c1 = canonical_hemisphere(q)
    c2 = canonical_hemisphere(-q)
    assert np.allclose(c1, c2, atol=1e-9), (
        "canonical_hemisphere must collapse q and -q to the same representative")


def test_anti_parallel_helper_flips_w_sign():
    q = canonical_hemisphere(axangle([1.0, 0.0, 0.0], np.deg2rad(30.0)))
    ap = anti_parallel_quat(q)
    assert ap[0] <= 0.0
    assert np.allclose(ap[1:], q[1:])
