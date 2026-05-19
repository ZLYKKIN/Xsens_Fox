"""F-3.1 — feeds pathological measurements into the FoxKf python mirror
to verify the NaN/Inf guard auto-resets state and downstream output
stays finite.  Drift bound after 100 normal steps post-injection comes
from the FUSED-SNAPSHOT gyr magnitudes in logs/fox_mocap.log:898 (≤2°/s
typical) — so 100 frames of clean data at 90Hz should leave residual
orientation < 3°."""

import numpy as np
from fox_kf_mirror import FoxKf, FoxKfSettings


def _normal_step(kf, dt=1/90.0):
    kf.predict(np.array([0.0, 0.0, 0.0], dtype=np.float32), dt)
    kf.update_acc(np.array([0.0, 0.0, -1.0], dtype=np.float32))


def test_no_nan_after_huge_acc_injection():
    kf = FoxKf(); kf.initialise(FoxKfSettings())
    for _ in range(50):
        _normal_step(kf)
    kf.update_acc(np.array([1e10, 1e10, 1e10], dtype=np.float32))
    for _ in range(100):
        _normal_step(kf)
    q = kf.orient()
    assert np.all(np.isfinite(q)), f"NaN in orient after injection: {q}"


def test_no_nan_after_huge_gyro_injection():
    kf = FoxKf(); kf.initialise(FoxKfSettings())
    for _ in range(50):
        _normal_step(kf)
    kf.predict(np.array([1e10, 0.0, 0.0], dtype=np.float32), 1.0)
    for _ in range(100):
        _normal_step(kf)
    q = kf.orient()
    assert np.all(np.isfinite(q))


def test_recovery_orientation_close_to_identity():
    kf = FoxKf(); kf.initialise(FoxKfSettings())
    for _ in range(100):
        _normal_step(kf)
    kf.update_acc(np.array([1e9, 0.0, 0.0], dtype=np.float32))
    for _ in range(300):
        _normal_step(kf)
    q = kf.orient()
    assert np.all(np.isfinite(q))
    assert abs(q[0]) > 0.5, f"after recovery q.w should approach 1.0, got {q}"


def test_zero_dt_is_noop():
    kf = FoxKf(); kf.initialise(FoxKfSettings())
    kf.predict(np.array([0.1, 0.0, 0.0], dtype=np.float32), 0.01)
    q1 = kf.orient().copy()
    kf.predict(np.array([0.1, 0.0, 0.0], dtype=np.float32), 0.0)
    q2 = kf.orient().copy()
    assert np.allclose(q1, q2), "predict with dt=0 must not change state"
