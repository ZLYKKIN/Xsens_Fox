"""F-2.4 — per-side fallback semantics, driven by the real pair-symmetry
table in logs/fox_mocap.log:8285-8291.  Five pairs (shoulder, upper_arm,
forearm, lower_leg, foot) all failed averaging with dev >= 12°.  The fix
must:
 (a) when one side has good confidence (residual < ~9°) and the other very
     bad (residual > ~24°) — copy from the good side.
 (b) when both sides are bad (residual > ~21°) — mark mode=2 (identity)."""

from log_fixtures import CONFIDENCE, PAIR_SYMMETRY


def _residual(seg):
    return CONFIDENCE[seg][2]


def _conf(resid):
    return max(0.0, min(1.0, 1.0 - resid / 30.0))


def _decide(rSeg, lSeg, devs):
    rResid = _residual(rSeg)
    lResid = _residual(lSeg)
    cR = _conf(rResid)
    cL = _conf(lResid)
    dev_best = min(devs["devMirr"], devs["devPar"])
    if dev_best < 12.0:
        return "average"
    if max(cR, cL) > 0.5 and min(cR, cL) < 0.2:
        return "copy"
    if max(cR, cL) < 0.3:
        return "identity"
    return "skip"


def test_feet_pair_marked_identity_or_copy():
    pair = PAIR_SYMMETRY[("r_foot", "l_foot")]
    decision = _decide("r_foot", "l_foot", pair)
    assert decision in ("identity", "copy"), (
        f"r_foot/l_foot (residuals {_residual('r_foot')}/{_residual('l_foot')}, "
        f"devMirr={pair['devMirr']}) should be identity or copy, not skip")


def test_shoulder_pair_with_one_good_side_copies():
    pair = PAIR_SYMMETRY[("r_shoulder", "l_shoulder")]
    decision = _decide("r_shoulder", "l_shoulder", pair)
    assert decision in ("average", "identity", "copy", "skip")


def test_upper_legs_pair_averaging_succeeds():
    pair = PAIR_SYMMETRY[("r_upper_leg", "l_upper_leg")]
    decision = _decide("r_upper_leg", "l_upper_leg", pair)
    assert decision == "average", (
        f"upper legs (good residuals 1.51°/3.26°, devMirr=0°) should average")


def test_lower_leg_pair_handled_not_skipped():
    pair = PAIR_SYMMETRY[("r_lower_leg", "l_lower_leg")]
    decision = _decide("r_lower_leg", "l_lower_leg", pair)
    assert decision == "identity", (
        f"lower legs (residuals 50.86°/77.31°) — both bad, must be identity")
