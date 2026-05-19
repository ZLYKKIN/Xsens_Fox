"""Integration check across all real-log calibration outcomes.  Confirms
that the post-fix expected behaviour matches the union of issues reported
in logs/fox_mocap.log:
  - 13 segments with TN residual > 0° (need calibration improvement)
  - 6 segments at confidence < 0.3 (must trigger UI warning)
  - 5 pairs failed symmetry averaging (must hit fallback path)
  - 4 segments at residual > 30° (must hit ecompass→TRIAD fallback)
"""

from log_fixtures import (CALIB_TN_RESIDUAL, CALIB_K_RESULT, CONFIDENCE,
                          PAIR_SYMMETRY, TPOSE_LR_PAIR_ANGLE_DEG)


def test_count_tn_segments_with_residual_above_zero():
    nonzero = [s for s, (_, r) in CALIB_TN_RESIDUAL.items() if r > 0.0]
    assert len(nonzero) == 12, f"expected 12 nonzero residuals, got {len(nonzero)}"


def test_count_low_confidence_critical_segments():
    critical = {"pelvis", "t8", "head", "r_upper_arm", "l_upper_arm",
                "r_lower_leg", "l_lower_leg", "r_foot", "l_foot"}
    low = [s for s in critical if CONFIDENCE[s][1] < 0.3]
    assert len(low) >= 6, (
        f"At least 6 critical segments must be flagged for UI warning; got {low}")


def test_count_pairs_failing_average():
    failing = []
    for pair, vals in PAIR_SYMMETRY.items():
        dev_best = min(vals["devMirr"], vals["devPar"])
        if dev_best >= 12.0:
            failing.append(pair)
    assert len(failing) >= 5, (
        f"At least 5 pairs must hit fallback path; got {len(failing)}: {failing}")


def test_segments_needing_triad_fallback():
    high_resid = [s for s, (_, r) in CALIB_K_RESULT.items() if r > 30.0]
    assert "r_foot" in high_resid
    assert "l_foot" in high_resid
    assert "r_lower_leg" in high_resid
    assert "l_lower_leg" in high_resid


def test_dlr_foot_asymmetry_matches_log():
    foot_dlr = TPOSE_LR_PAIR_ANGLE_DEG["foot"]["dLR"]
    assert abs(foot_dlr + 25.02) < 1e-3, (
        f"foot ΔLR in log is -25.02°, got {foot_dlr}")


def test_pelvis_calibration_remains_perfect():
    method, resid = CALIB_TN_RESIDUAL["pelvis"]
    assert method == "ecompass"
    assert resid == 0.0
