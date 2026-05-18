"""Static-source contract for the FoxKf migration.

This test fixes the integration shape of the codebase so future refactors
cannot silently revive the old xio Fusion path or introduce a parallel
filter implementation alongside FoxKf.

Checks:
  • scr/fusion/ contains ONLY FoxKf — no FusionAhrs / FusionBias /
    FusionCompass / FusionMath / FusionRemap leftovers.
  • CMakeLists.txt lists FoxKf.cpp; no FusionAhrs.c etc.
  • main.cpp #include lists FoxKf.h and NOT fusion/Fusion.h.
  • The receiver loop uses kf.predict / updateAcc / updateMag / updateZupt
    and does NOT call any FusionAhrs* / FusionBias* symbol.
  • The receiver Impl struct holds `kf[]` and `kfReady[]` arrays — the
    old `fusion[]`, `bias[]`, `ahrsCfg[]`, `fusionReady[]`, `biasReady[]`
    arrays are gone.
"""
import os, re


HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))


def test_fusion_dir_is_only_foxkf():
    fusion_dir = os.path.join(REPO, "scr", "fusion")
    files = sorted(os.listdir(fusion_dir))
    expected = ["FoxKf.cpp", "FoxKf.h"]
    assert files == expected, f"scr/fusion/ should only contain FoxKf; got {files}"


def test_cmake_includes_foxkf_not_xio():
    cm = os.path.join(REPO, "CMakeLists.txt")
    with open(cm, "r", encoding="utf-8") as f:
        src = f.read()
    assert "FoxKf.cpp" in src, "CMakeLists.txt must reference FoxKf.cpp"
    for old in ["FusionAhrs.c", "FusionBias.c", "FusionCompass.c"]:
        assert old not in src, f"CMakeLists.txt still references retired {old}"


def test_main_cpp_imports_foxkf_only():
    main_cpp = os.path.join(REPO, "scr", "main.cpp")
    with open(main_cpp, "r", encoding="utf-8") as f:
        src = f.read()
    assert '#include "fusion/FoxKf.h"' in src, \
        "main.cpp must #include fusion/FoxKf.h"
    assert '#include "fusion/Fusion.h"' not in src, \
        "main.cpp still imports the retired xio Fusion aggregator header"


def test_receiver_loop_uses_kf_methods():
    main_cpp = os.path.join(REPO, "scr", "main.cpp")
    with open(main_cpp, "r", encoding="utf-8") as f:
        src = f.read()
    # All FoxKf entry points must be called somewhere in the source.
    for method in ["kf.predict(", "kf.updateAcc(", "kf.updateMag(",
                    "kf.updateZupt(", "kf.initialise("]:
        assert method in src, f"FoxKf entry point {method} not wired"
    # And no xio Fusion symbols.
    for old in ["FusionAhrsUpdate", "FusionAhrsInitialise",
                "FusionBiasUpdate", "FusionBiasInitialise",
                "fusionAhrsDefaultSettings", "fusionBiasDefaultSettings",
                "FusionConventionNwu"]:
        assert old not in src, \
            f"main.cpp still references retired symbol {old}"


def test_impl_struct_has_kf_arrays_only():
    main_cpp = os.path.join(REPO, "scr", "main.cpp")
    with open(main_cpp, "r", encoding="utf-8") as f:
        src = f.read()
    # New per-sensor arrays.
    assert re.search(r"std::array<FoxKf,\s*kXsensSegmentCount>\s*kf\{}", src) or \
           "std::array<FoxKf, kXsensSegmentCount> kf{};" in src, \
        "Impl struct missing kf[] array"
    assert "kfReady" in src, "Impl struct missing kfReady[] array"
    # Old per-sensor arrays must be gone (struct field declarations).
    bad_decls = [
        r"std::array<FusionAhrs,\s*kXsensSegmentCount>",
        r"std::array<FusionBias,\s*kXsensSegmentCount>",
        r"std::array<FusionAhrsSettings,\s*kXsensSegmentCount>",
    ]
    for pat in bad_decls:
        assert re.search(pat, src) is None, \
            f"Impl struct still declares retired array matching /{pat}/"


def test_setprior_present_in_foxkf():
    """FoxKf must expose a setPrior method — that's the integration hook
    for the T-N-K calibration to inject the per-sensor reference orient
    plus the calibration-derived gyro bias estimate."""
    fkh = os.path.join(REPO, "scr", "fusion", "FoxKf.h")
    with open(fkh, "r", encoding="utf-8") as f:
        h = f.read()
    assert "setPrior(" in h, "FoxKf.h missing setPrior() for calibration integration"


def test_no_dead_filter_api():
    """The dead public surface (set on never-called paths, or backed by
    state nothing reads) must not creep back: see plan phase 4."""
    main_cpp = os.path.join(REPO, "scr", "main.cpp")
    main_h   = os.path.join(REPO, "scr", "main.h")
    fkh      = os.path.join(REPO, "scr", "fusion", "FoxKf.h")
    fkc      = os.path.join(REPO, "scr", "fusion", "FoxKf.cpp")
    blobs = {p: open(p, "r", encoding="utf-8").read()
             for p in [main_cpp, main_h, fkh, fkc]}

    # FoxKf diagnostic-only members that nothing called.
    for sig in ["restart(", "covarianceSnapshot(", "setCovariance("]:
        for p in [fkh, fkc]:
            assert sig not in blobs[p], (
                f"{os.path.basename(p)} still declares dead FoxKf method '{sig}'")

    # Receiver-side: getters with no callers and the dead per-sensor gain.
    for sig in ["snapshotGyroAvg(", "liveGyrSensor(",
                "setSegmentGain(", "setGyroBias(",
                "segGainActive", "gyrBiasActive"]:
        for p in [main_h, main_cpp]:
            assert sig not in blobs[p], (
                f"{os.path.basename(p)} still references retired '{sig}'")


def test_extern_c_wrapper_gone():
    """xio Fusion was C; FoxKf is a C++ class in namespace fox.  The old
    `extern \"C\" { #include \"fusion/FoxKf.h\" }` wrapper made no sense
    semantically and shouldn't survive the migration."""
    main_cpp = os.path.join(REPO, "scr", "main.cpp")
    with open(main_cpp, "r", encoding="utf-8") as f:
        src = f.read()
    # Match the smallest substring that proves the wrapper exists.
    bad = 'extern "C" {\n#include "fusion/FoxKf.h"'
    assert bad not in src, ("main.cpp still wraps FoxKf.h in extern \"C\" "
                            "— FoxKf is C++, drop the wrapper")
    assert '#include "fusion/FoxKf.h"' in src, (
        "FoxKf.h must still be #include'd in main.cpp (just without extern C)")


def test_eigen_used_in_foxkf():
    """The CMake build links Eigen3::Eigen; the project promises in
    FoxKf.cpp's header comment that the EKF math goes through Eigen
    fixed-size types (Joseph-form covariance update etc).  Verify the
    header is actually pulled — otherwise the linkage is dead-weight."""
    fkc = os.path.join(REPO, "scr", "fusion", "FoxKf.cpp")
    with open(fkc, "r", encoding="utf-8") as f:
        src = f.read()
    assert ("#include <Eigen/Dense>" in src
            or "#include <Eigen/Core>" in src), (
        "FoxKf.cpp must include an Eigen header — CMake links Eigen3::Eigen "
        "but it's dead-weight if the .cpp doesn't use it")
    # Joseph form: P ← (I-KH) P (I-KH)ᵀ + K R Kᵀ.  The transpose() pattern
    # on the (I - K*H) term is what distinguishes it from the naive form.
    assert "K * K.transpose()" in src or "K*K.transpose()" in src, (
        "FoxKf.cpp covariance update should use Joseph form for numerical "
        "stability (look for `K * K.transpose()` in the update path)")


def test_identity_nwu_wrappers_gone():
    """conjugateNwuToMvn / rotateNwuToMvn were identity wrappers kept as
    a "future protocol variant" edit-point — pure dead-future-proofing.
    The internal NWU frame matches the MVN wire format exactly."""
    main_cpp = os.path.join(REPO, "scr", "main.cpp")
    with open(main_cpp, "r", encoding="utf-8") as f:
        src = f.read()
    for sig in ["conjugateNwuToMvn", "rotateNwuToMvn"]:
        assert sig not in src, (
            f"identity wrapper '{sig}' must be removed (NWU == wire frame)")


if __name__ == "__main__":
    test_fusion_dir_is_only_foxkf()
    test_cmake_includes_foxkf_not_xio()
    test_main_cpp_imports_foxkf_only()
    test_receiver_loop_uses_kf_methods()
    test_impl_struct_has_kf_arrays_only()
    test_setprior_present_in_foxkf()
    test_no_dead_filter_api()
    test_extern_c_wrapper_gone()
    test_eigen_used_in_foxkf()
    test_identity_nwu_wrappers_gone()
    print("test_fox_kf_static_source: PASS")
