"""F19 test (regression): mirror-Y symmetric raw input -> mirror-Y
symmetric FK output.  This guards against any FK change that would break
left/right symmetry under mirrored arm/leg motion."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qrot, qinv, axangle, mirror_y_quat, euler_xyz, PI


def def_ang_tpose():
    E = euler_xyz
    return ([E(0, -PI/2, 0)] * 7
            + [E(0, 0, -PI/2)] * 4
            + [E(0, 0,  PI/2)] * 4
            + [E(0,  PI/2, 0)] * 2 + [E(0, 0, 0)] * 2
            + [E(0,  PI/2, 0)] * 2 + [E(0, 0, 0)] * 2)


def test_mirror_y_symmetry():
    defT = def_ang_tpose()
    i_R, i_L = 8, 12   # R/L upper arm

    test_cases = [
        ("arms 90° forward",
         axangle([0, 0, 1], +np.pi/2),
         axangle([0, 0, 1], -np.pi/2)),
        ("arms 45° forward",
         axangle([0, 0, 1], +np.pi/4),
         axangle([0, 0, 1], -np.pi/4)),
        ("compound (45° up + 90° rot)",
         qmul(axangle([1, 0, 0], -np.pi/4), axangle([0, 0, 1], +np.pi/2)),
         qmul(axangle([1, 0, 0],  np.pi/4), axangle([0, 0, 1], -np.pi/2))),
    ]
    refR, refL = defT[i_R], defT[i_L]

    for name, dR, dL in test_cases:
        raw_R = qnorm(qmul(dR, refR))
        raw_L = qnorm(qmul(dL, refL))
        candR = qnorm(qmul(raw_R, qinv(refR)))
        candL = qnorm(qmul(raw_L, qinv(refL)))
        worldR = qnorm(qmul(candR, defT[i_R]))
        worldL = qnorm(qmul(candL, defT[i_L]))
        dirR = qrot(worldR, np.array([1., 0, 0]))
        dirL = qrot(worldL, np.array([1., 0, 0]))
        mirr_dirR = np.array([dirR[0], -dirR[1], dirR[2]])
        err = float(np.linalg.norm(dirL - mirr_dirR))
        assert err < 1e-10, f"{name}: mirror symmetry broken, err={err}"


if __name__ == "__main__":
    test_mirror_y_symmetry()
    print("test_symmetry: PASS")
