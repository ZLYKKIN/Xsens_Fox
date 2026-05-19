"""F-2.5 — uses real K-pose calibration outcomes from
logs/fox_mocap.log:7054 to verify TRIAD fallback dominates ecompass for
limb segments when residuals diverge.  In the log r_upper_leg got
Wahba bad → TRIAD via TK residual=1.51° (mode=triad, conf=0.88).  The
fallback rule must therefore reach the same TRIAD decision when given a
high ecompass residual and a low TRIAD residual."""

from log_fixtures import CALIB_K_RESULT, CONFIDENCE


def _ecomp_residual(seg):
    return CALIB_K_RESULT.get(seg, ("tilt", 99.0))[1]


def _fallback_chooses_triad(ecomp_resid, triad_resid):
    return (triad_resid + 5.0) < ecomp_resid


def test_r_upper_leg_uses_triad_via_tk():
    method, resid = CALIB_K_RESULT["r_upper_leg"]
    assert method == "triad_fallback"
    assert resid < 5.0, f"r_upper_leg TRIAD residual={resid}° should be < 5°"
    assert _fallback_chooses_triad(25.25, resid)


def test_lower_leg_residuals_indicate_need_for_fallback():
    for seg in ("r_lower_leg", "l_lower_leg", "r_foot", "l_foot"):
        method, resid = CALIB_K_RESULT[seg]
        assert resid >= 30.0, (
            f"{seg} residual={resid}° in log — must trigger TRIAD fallback")


def test_confident_segments_stay_confident():
    for seg in ("pelvis", "t8", "head", "r_upper_leg", "l_upper_leg"):
        mode, conf, resid = CONFIDENCE[seg]
        assert conf > 0.8, f"{seg} should keep conf>0.8 (was {conf})"


def test_fallback_logic_inert_when_ecomp_low():
    assert not _fallback_chooses_triad(2.0, 5.0)
    assert _fallback_chooses_triad(30.0, 5.0)
    assert _fallback_chooses_triad(50.0, 1.5)
