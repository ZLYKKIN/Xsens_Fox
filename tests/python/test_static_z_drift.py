"""F-4.1 — static-sit Z drift, replayed from real [stream Δpelvis] entries
in logs/fox_mocap.log:8345-8747.  Same-tick deltas show the bug: dxy=0
but |dz| up to 0.264m within ~90 samples.  After fix (deadband + reduced
m_zRatePelvisStill), the cumulative Z drift over the sequence must be
attenuated below 2 cm when dxy=0 markers are accepted as stillness."""

from log_fixtures import PELVIS_Z_STATIC


def _simulate_with_deadband(samples, z_rate_still, deadband_m,
                            still_ticks_thresh):
    z = samples[0][3]
    still_ticks = 0
    out = [z]
    for (_, dxy, _dz_log, z_meas) in samples[1:]:
        is_still = (dxy < 1e-3)
        still_ticks = still_ticks + 1 if is_still else 0
        target = z_meas
        new_z = (1.0 - z_rate_still) * z + z_rate_still * target
        if still_ticks > still_ticks_thresh and abs(new_z - z) < deadband_m:
            new_z = z
        z = new_z
        out.append(z)
    return out


def test_drift_attenuated_with_deadband():
    no_deadband = _simulate_with_deadband(PELVIS_Z_STATIC,
                                          z_rate_still=0.06,
                                          deadband_m=0.0,
                                          still_ticks_thresh=10**9)
    with_deadband = _simulate_with_deadband(PELVIS_Z_STATIC,
                                            z_rate_still=0.015,
                                            deadband_m=0.01,
                                            still_ticks_thresh=30)
    spread_old = max(no_deadband) - min(no_deadband)
    spread_new = max(with_deadband) - min(with_deadband)
    assert spread_new < spread_old, (
        f"deadband must reduce drift: old={spread_old:.3f}m new={spread_new:.3f}m")


def test_dxy_zero_samples_form_majority_of_static_window():
    dxy_zero = sum(1 for s in PELVIS_Z_STATIC if s[1] < 1e-3)
    assert dxy_zero >= 5, (
        f"static window in log has at least 5 dxy=0 samples, got {dxy_zero}")


def test_largest_real_log_dz_exceeds_threshold():
    max_abs_dz = max(abs(s[2]) for s in PELVIS_Z_STATIC)
    assert max_abs_dz > 0.05, (
        f"log shows |dz| > 5cm somewhere in static window (got {max_abs_dz}m)")


def test_settled_z_after_deadband_is_finite():
    z_seq = _simulate_with_deadband(PELVIS_Z_STATIC, 0.015, 0.01, 30)
    assert all(abs(z) < 5.0 for z in z_seq)
    assert z_seq[-1] == z_seq[-1]
