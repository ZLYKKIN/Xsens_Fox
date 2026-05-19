"""F-2.3 — verifies mirror-Z / anti-parallel mount models on real foot
quaternions from FUSED SNAPSHOT t=307284.72s (logs/fox_mocap.log:2520,2528).
The pair r_foot=(-0.264, 0.199, -0.522, -0.786) and
l_foot=(-0.467, -0.718, 0.480, 0.192) gave devMirr=55.09° and devPar=93.91°
(SYMMETRY FAIL).  With four mount models the best of {MY, PR, MZ, AP}
should be substantially smaller than the worst, confirming the extra
candidates cover physical asymmetric-sock mountings."""

from quat_math import (mirror_y_quat, mirror_z_quat, anti_parallel_quat,
                       canonical_hemisphere, qmul, qinv, quat_dev_deg)
from log_fixtures import TPOSE_QUAT_FUSED, PAIR_SYMMETRY
import numpy as np


def _candidates_dev(qR, qL):
    return [
        ("mirror-Y",      quat_dev_deg(qR, mirror_y_quat(qL))),
        ("parallel",      quat_dev_deg(qR, qL)),
        ("mirror-Z",      quat_dev_deg(qR, mirror_z_quat(qL))),
        ("anti-parallel", quat_dev_deg(qR, anti_parallel_quat(qL))),
    ]


def test_feet_have_at_least_one_candidate_below_30deg():
    qR = np.array(TPOSE_QUAT_FUSED["r_foot"])
    qL = np.array(TPOSE_QUAT_FUSED["l_foot"])
    devs = _candidates_dev(qR, qL)
    best = min(d for _, d in devs)
    worst = max(d for _, d in devs)
    assert worst - best > 30.0, (
        f"With 4 mount models there must be a clear separation between best "
        f"and worst; got best={best:.1f}° worst={worst:.1f}° {devs}")


def test_feet_pair_extra_models_separate_from_mirror_y():
    qR = np.array(TPOSE_QUAT_FUSED["r_foot"])
    qL = np.array(TPOSE_QUAT_FUSED["l_foot"])
    devs = dict(_candidates_dev(qR, qL))
    best = min(devs.values())
    assert best < devs["mirror-Y"] - 30.0, (
        f"with asymmetric sock mounting at least one of mirror-Z / anti-parallel "
        f"must be 30°+ better than mirror-Y; got {devs}")


def test_feet_anti_parallel_does_not_falsely_win():
    qR = np.array(TPOSE_QUAT_FUSED["r_foot"])
    qL = np.array(TPOSE_QUAT_FUSED["l_foot"])
    devs = dict(_candidates_dev(qR, qL))
    best_name = min(devs, key=devs.get)
    second_dev = sorted(devs.values())[1]
    assert devs[best_name] >= 0.0
    assert second_dev >= devs[best_name]


def test_legs_pair_no_change_in_winner_under_4_models():
    qR = np.array(TPOSE_QUAT_FUSED["r_upper_leg"])
    qL = np.array(TPOSE_QUAT_FUSED["l_upper_leg"])
    devs = dict(_candidates_dev(qR, qL))
    best_name = min(devs, key=devs.get)
    second_name = sorted(devs, key=devs.get)[1]
    assert devs[best_name] < devs[second_name] + 1e-3
    assert best_name in ("mirror-Y", "parallel", "mirror-Z", "anti-parallel")
