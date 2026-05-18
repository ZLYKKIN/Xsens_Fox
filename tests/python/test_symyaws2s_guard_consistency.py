"""S1: scr/main.cpp has two `symYawS2S` lambdas — one runs after K-pose
refinement (line ~5735), one after N-pose refinement (line ~6315).  They
share the same algorithm but historically had different `guard` constants.

The K-pose lambda was tightened to 0.998/0.99 (test_per_sensor_asymmetry
proves this preserves a real 8° per-sensor mounting asymmetry).  The
N-pose lambda was missed and still carries the old 0.95/0.7 — which would
silently average away the same asymmetry during N-pose calibration,
nullifying the K-pose preservation a few seconds later in the wizard.

This test guards against the inconsistency by reading the C++ source and
asserting both lambdas use identical guard constants.
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
CPP = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))


def _extract_guards(text):
    """Find every occurrence of `bothTriad ? <X> : <Y>` inside a function
    whose name starts with `symYawS2S`.  Returns a list of (triadGuard, ecompGuard)
    pairs in source order."""
    # Find the bodies of every symYawS2S* lambda.  Each lambda is defined as
    # `auto symYawS2S_X = [&](...) { ... };` — capture from open brace to
    # the matching close.  We use a coarse non-overlapping span via the
    # next `};` after the auto.
    pat = re.compile(r"auto\s+(symYawS2S\w*)\s*=\s*\[[^\]]*\][^{]*\{(.+?)\n\s*\};",
                     re.DOTALL)
    guard_pat = re.compile(
        r"bothTriad\s*\?\s*([0-9.]+)\s*:\s*([0-9.]+)"
    )
    out = []
    for name, body in pat.findall(text):
        m = guard_pat.search(body)
        if m:
            out.append((name, float(m.group(1)), float(m.group(2))))
    return out


def test_symYawS2S_guards_consistent_across_lambdas():
    """Every symYawS2S* lambda must use identical guard constants — the
    same per-sensor asymmetry should be preserved/averaged identically in
    every refinement pass."""
    with open(CPP, "r", encoding="utf-8") as f:
        text = f.read()
    guards = _extract_guards(text)
    assert len(guards) >= 2, \
        f"expected at least 2 symYawS2S* lambdas, found {len(guards)}: {guards}"

    first_name, first_tr, first_ec = guards[0]
    for name, tr, ec in guards[1:]:
        assert tr == first_tr, (
            f"{name}: triad guard {tr} != {first_name}'s {first_tr} — "
            f"per-sensor asymmetry would be averaged differently between "
            f"calibration passes")
        assert ec == first_ec, (
            f"{name}: ecompass guard {ec} != {first_name}'s {first_ec}")


def test_symYawS2S_guards_preserve_8deg_asymmetry():
    """Per test_per_sensor_asymmetry's analysis: guard must be ≥ cos(4°)
    ≈ 0.9976 to preserve an 8° real per-sensor mounting offset.  Assert
    the active value is at least that tight."""
    with open(CPP, "r", encoding="utf-8") as f:
        text = f.read()
    guards = _extract_guards(text)
    assert guards, "no symYawS2S* lambdas found"
    for name, tr, ec in guards:
        assert tr >= 0.9976, (
            f"{name}: triad guard {tr} < 0.9976 (cos(4°)) — would mask "
            f"the 8° per-sensor mounting asymmetry the suit actually has")


if __name__ == "__main__":
    test_symYawS2S_guards_consistent_across_lambdas()
    test_symYawS2S_guards_preserve_8deg_asymmetry()
    print("test_symyaws2s_guard_consistency: PASS")
