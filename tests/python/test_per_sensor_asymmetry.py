"""§E test: symYawS2S_K (scr/main.cpp:5713-5762) must NOT mask real
per-sensor mounting asymmetry below its guard threshold.

We feed synthetic TRIAD-recovered s2s quats whose R-shoulder is mounted
+8° rotated about Z relative to L-shoulder (a realistic mounting offset
when the suit straps are tightened asymmetrically), then apply the same
symYawS2S_K averaging the C++ code does and verify how much of the
genuine asymmetry survives.

If the existing guard (0.95) allows the average — i.e. the 8° offset
ends up averaged out — the test asserts a known failure mode: the guard
must be tightened so genuine asymmetry is preserved per sensor.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, axangle, qrot


def yaw_only(q):
    w, x, y, z = q
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    n2 = w*w + z*z
    if n2 < 1e-24:
        return np.array([1.0, 0, 0, 0])
    n = 1.0 / np.sqrt(n2)
    return np.array([w*n, 0.0, 0.0, z*n])


def sym_yaw_s2s_pair(qR, qL, guard):
    """Mirror of scr/main.cpp:5713-5752 symYawS2S_K.  Returns (qR_new, qL_new)
    or (qR, qL) unchanged if the guard rejects the average."""
    yawR = yaw_only(qR)
    yawL = yaw_only(qL)
    yawLmir = np.array([yawL[0], 0.0, 0.0, -yawL[3]])
    d = yawR[0] * yawLmir[0] + yawR[3] * yawLmir[3]
    if abs(d) < guard:
        return qR.copy(), qL.copy()
    # tiltR = qR * yawR.inv()  (yaw-only is its own quasi-inverse via conj)
    yawR_inv = np.array([yawR[0], -yawR[1], -yawR[2], -yawR[3]])
    yawL_inv = np.array([yawL[0], -yawL[1], -yawL[2], -yawL[3]])
    tiltR = qnorm(qmul(qR, yawR_inv))
    tiltL = qnorm(qmul(qL, yawL_inv))
    sgn = -1.0 if d < 0 else 1.0
    yaw_avg = np.array([0.5 * (yawR[0] + sgn * yawLmir[0]),
                        0.0, 0.0,
                        0.5 * (yawR[3] + sgn * yawLmir[3])])
    yn2 = yaw_avg[0]**2 + yaw_avg[3]**2
    if yn2 < 1e-24:
        return qR.copy(), qL.copy()
    yaw_avg = yaw_avg / np.sqrt(yn2)
    yaw_avg_mir = np.array([yaw_avg[0], 0.0, 0.0, -yaw_avg[3]])
    return qnorm(qmul(yaw_avg, tiltR)), qnorm(qmul(yaw_avg_mir, tiltL))


def yaw_deg(q):
    w, x, y, z = q
    return float(np.degrees(2.0 * np.arctan2(abs(z), abs(w)))) * (1 if z >= 0 else -1)


def test_guard_095_masks_8deg_asymmetry():
    """Default guard 0.95 corresponds to about 18° cosine.  An 8° real
    asymmetry survives the guard and gets averaged.
    """
    # R-shoulder mounted +8° about Z relative to ideal; L-shoulder ideal.
    qR = axangle([0, 0, 1], np.deg2rad(8))
    qL = np.array([1.0, 0, 0, 0])
    qR_new, qL_new = sym_yaw_s2s_pair(qR, qL, guard=0.95)
    # After averaging, R is pulled toward 4° (half of 8).
    yaw_r_before = yaw_deg(qR)
    yaw_r_after  = yaw_deg(qR_new)
    assert abs(yaw_r_after - yaw_r_before) > 2.0, \
        f"current guard 0.95 should mask 8° asymmetry " \
        f"(before={yaw_r_before:.1f}° after={yaw_r_after:.1f}°)"


def test_tighter_guard_preserves_8deg_asymmetry():
    """The C++ guard is a dot-product of yaw quats; quaternions use the
    half-angle, so guard d ≥ cos(yaw_diff/2).  To preserve 8° real
    asymmetry the guard must be ≥ cos(4°) = 0.9976.  We pick 0.998 for
    a small safety margin (blocks > ~7.2° asymmetry, still averages
    smaller drift)."""
    qR = axangle([0, 0, 1], np.deg2rad(8))
    qL = np.array([1.0, 0, 0, 0])
    qR_new, qL_new = sym_yaw_s2s_pair(qR, qL, guard=0.998)
    yaw_r_before = yaw_deg(qR)
    yaw_r_after  = yaw_deg(qR_new)
    assert abs(yaw_r_after - yaw_r_before) < 0.5, \
        f"tightened guard 0.998 should preserve 8° asymmetry " \
        f"(before={yaw_r_before:.1f}° after={yaw_r_after:.1f}°)"


def test_tighter_guard_still_corrects_small_drift():
    """A small symmetric mounting drift (R = +1°, L = -1°, mirror-symmetric)
    must still be averaged by the 0.998 guard — d = cos(0°) = 1.0."""
    qR = axangle([0, 0, 1], np.deg2rad(1))
    qL = axangle([0, 0, 1], np.deg2rad(-1))     # mirror — yaw_L_mir = +1°
    qR_new, _ = sym_yaw_s2s_pair(qR, qL, guard=0.998)
    # Mirror-symmetric input: d = 1.0, averaging fires.  The averaged
    # yaw_avg equals yawR exactly (it's the mean of two identical yaws
    # post-mirror), so qR_new should be very close to qR.
    assert np.linalg.norm(qR_new - qR) < 1e-6, \
        "mirror-symmetric pair should be preserved (averaging is the no-op)"


def test_off_axis_tilt_unaffected():
    """The averaging only touches yaw — a pure pitch/roll component
    should pass through untouched on both sides regardless of guard."""
    qR = qnorm(qmul(axangle([0, 0, 1], np.deg2rad(2)),
                    axangle([1, 0, 0], np.deg2rad(5))))
    qL = qnorm(qmul(axangle([0, 0, 1], np.deg2rad(-2)),
                    axangle([1, 0, 0], np.deg2rad(5))))
    qR_new, qL_new = sym_yaw_s2s_pair(qR, qL, guard=0.998)
    # Recover tilt = q * yaw.inv() on both sides; tilts must match input
    # within the yaw-decomposition numerical accuracy (≈0.5° drift is the
    # expected slop because swing-twist decomposition along Z is not exact
    # when the quat has a non-trivial X-component).
    yawR_inv_in  = np.array([yaw_only(qR)[0], 0, 0, -yaw_only(qR)[3]])
    tilt_in      = qnorm(qmul(qR, yawR_inv_in))
    yawR_inv_out = np.array([yaw_only(qR_new)[0], 0, 0, -yaw_only(qR_new)[3]])
    tilt_out     = qnorm(qmul(qR_new, yawR_inv_out))
    drift_q = float(min(np.linalg.norm(tilt_in - tilt_out),
                        np.linalg.norm(tilt_in + tilt_out)))
    drift_deg = float(np.degrees(2.0 * drift_q))   # small-angle: ‖Δq‖ ≈ Δθ
    assert drift_deg < 1.0, \
        f"tilt drift through symYawS2S = {drift_deg:.3f}° (should be < 1°)"


if __name__ == "__main__":
    test_guard_095_masks_8deg_asymmetry()
    test_tighter_guard_preserves_8deg_asymmetry()
    test_tighter_guard_still_corrects_small_drift()
    print("test_per_sensor_asymmetry: PASS")
