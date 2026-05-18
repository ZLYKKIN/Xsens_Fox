"""Static-source contract for the T-N-K → FoxKf prior wiring.

Before this work-stream the FoxKf integration had a dead path: the wizard
computed per-sensor `calibReference` quats and `gyrBias` estimates, but the
receiver re-initialised the filter from (identity, zero) on the next
predict.  The 2-5 s of acc/mag/ZUPT updates after calibration finished
were wasted on re-convergence the wizard had already done.

This file pins the integration shape so future refactors don't quietly
revert to the dead path:

  1. `MocapReceiver::setKfPriors` is declared in main.h and defined in
     main.cpp.
  2. The K-pose calibration finaliser calls `m_rx->setKfPriors(...)`
     after installing the s2s alignment.
  3. `MocapReceiver::setKfPriors` calls `kf.setPrior(...)` per sensor
     (the actual FoxKf entry-point).
  4. `SuitPose` carries per-segment `orientStdDeg` from the filter.
  5. The receiver fills `staging.orientStdDeg[targetSeg]` after each
     filter update.
  6. `MocapViewport::updatePose` has a 3-arg overload that takes
     `orientStdDeg` and the drift-lock uses it as a confidence gate.
  7. `MainWindow::onRenderTick` passes `f.orientStdDeg` to updatePose.
  8. The upstream `gyr - gyr_bias` subtraction is gone — bias is now
     entirely FoxKf's responsibility via setKfPriors + online ZUPT.
"""
import os, re


HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))


def _slurp(rel):
    with open(os.path.join(REPO, rel), "r", encoding="utf-8") as f:
        return f.read()


def test_setkfpriors_declared_and_defined():
    h = _slurp("scr/main.h")
    c = _slurp("scr/main.cpp")
    assert "void setKfPriors(" in h, \
        "main.h must declare MocapReceiver::setKfPriors"
    assert "void MocapReceiver::setKfPriors(" in c, \
        "main.cpp must define MocapReceiver::setKfPriors"


def test_setkfpriors_called_from_k_finaliser():
    """The K-pose finaliser (CalibPhase::CaptureK branch in onCaptureTick)
    must call setKfPriors right after the s2s alignment so the filter
    starts from the calibration state."""
    c = _slurp("scr/main.cpp")
    assert "m_rx->setKfPriors(" in c, \
        "no setKfPriors call site — calibration prior is not flowing"
    # The call must come AFTER the K-pose setS2sAlignment.
    k_marker = "// Install the FoxKf prior NOW"
    assert k_marker in c, \
        ("K-pose finaliser should have the FoxKf-prior install block — "
         "missing the explicit comment that pins the install point")


def test_setkfpriors_calls_kf_setprior_per_sensor():
    """The receiver-side setKfPriors implementation must actually call
    FoxKf::setPrior — the integration is otherwise still a no-op."""
    c = _slurp("scr/main.cpp")
    impl_start = c.find("void MocapReceiver::setKfPriors(")
    assert impl_start >= 0, "MocapReceiver::setKfPriors definition not found"
    # Look ahead enough to cover the body.
    body = c[impl_start: impl_start + 2400]
    assert ".setPrior(" in body, (
        "MocapReceiver::setKfPriors body must call FoxKf::setPrior per sensor")
    assert "kDegToRad" in body or "M_PI" in body, (
        "setKfPriors should convert gyrBias from deg/s to rad/s before "
        "feeding the filter (FoxKf consumes rad/s)")


def test_suitpose_has_orient_std_deg():
    h = _slurp("scr/main.h")
    assert "orientStdDeg" in h, "SuitPose missing orientStdDeg array"
    assert re.search(
        r"std::array<float,\s*kXsensSegmentCount>\s*orientStdDeg",
        h), "SuitPose::orientStdDeg must be std::array<float,23>"


def test_receiver_fills_orient_std_deg():
    c = _slurp("scr/main.cpp")
    assert "staging.orientStdDeg[" in c, (
        "receiver must publish per-sensor FoxKf orientStdDeg into the "
        "shared SuitPose frame")
    assert "orientStdDeg()" in c, (
        "receiver must read FoxKf::orientStdDeg() — otherwise the field "
        "stays at default 0")


def test_viewport_updatepose_takes_stddeg():
    h = _slurp("scr/main.h")
    # New overload with the orientStdDeg parameter.
    sig_re = re.compile(
        r"updatePose\(.*?std::array<float,\s*kXsensSegmentCount>&\s+orientStdDeg",
        re.DOTALL)
    assert sig_re.search(h), \
        "MocapViewport::updatePose must have a 3-arg overload with orientStdDeg"


def test_render_tick_passes_stddeg_to_viewport():
    c = _slurp("scr/main.cpp")
    assert "f.orientStdDeg" in c, \
        ("MainWindow::onRenderTick must pass f.orientStdDeg into the "
         "viewport (otherwise the confidence gate sees zeros for every "
         "sensor and stays fully open)")


def test_drift_lock_consults_stddeg():
    """The drift-lock confidence gate inside MocapViewport::updatePose
    must read the orientStdDeg array (not just the heuristic angVel)."""
    c = _slurp("scr/main.cpp")
    # The threshold constant + the OR-by-zero short-circuit are the
    # distinguishing markers.
    assert "kStdLockThreshDeg" in c, \
        "drift-lock missing the FoxKf confidence threshold constant"
    assert "orientStdDeg[i]" in c, \
        "drift-lock must read per-segment orientStdDeg, not just |omega|"


def test_no_double_bias_subtraction():
    """The upstream `gyr - gyr_bias` subtraction must be gone — bias is
    handled by FoxKf, seeded via setKfPriors."""
    c = _slurp("scr/main.cpp")
    # The exact text of the dead path.
    bad_patterns = [
        "gyrForFilter = gyrForFilter - I.gyrBias[targetSeg]",
        "if (I.gyrBiasActive",
    ]
    for pat in bad_patterns:
        assert pat not in c, (
            f"upstream bias-subtraction path '{pat}' must be removed — "
            "double-counts FoxKf's online bias estimate")


if __name__ == "__main__":
    test_setkfpriors_declared_and_defined()
    test_setkfpriors_called_from_k_finaliser()
    test_setkfpriors_calls_kf_setprior_per_sensor()
    test_suitpose_has_orient_std_deg()
    test_receiver_fills_orient_std_deg()
    test_viewport_updatepose_takes_stddeg()
    test_render_tick_passes_stddeg_to_viewport()
    test_drift_lock_consults_stddeg()
    test_no_double_bias_subtraction()
    print("test_kf_prior_wiring: PASS")
