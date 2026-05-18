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


if __name__ == "__main__":
    test_fusion_dir_is_only_foxkf()
    test_cmake_includes_foxkf_not_xio()
    test_main_cpp_imports_foxkf_only()
    test_receiver_loop_uses_kf_methods()
    test_impl_struct_has_kf_arrays_only()
    test_setprior_present_in_foxkf()
    print("test_fox_kf_static_source: PASS")
