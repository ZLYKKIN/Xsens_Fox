"""F18 test (regression): global yaw rotation of the actor must propagate
exactly into each segment's world orientation.  Uses the cand = raw * refInv
pipeline from MainWindow::onRenderTick."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qmul, qnorm, qrot, qinv, axangle, euler_xyz, PI

# Same default angles as test_default_angles
def def_ang_tpose():
    E = euler_xyz
    return ([E(0, -PI/2, 0)] * 7
            + [E(0, 0, -PI/2)] * 4
            + [E(0, 0,  PI/2)] * 4
            + [E(0,  PI/2, 0)] * 2 + [E(0, 0, 0)] * 2
            + [E(0,  PI/2, 0)] * 2 + [E(0, 0, 0)] * 2)


T_POSE_DIRS = {
    0: (0, 0, 1), 1: (0, 0, 1), 2: (0, 0, 1), 3: (0, 0, 1),
    4: (0, 0, 1), 5: (0, 0, 1), 6: (0, 0, 1),
    7: (0, -1, 0), 8: (0, -1, 0), 9: (0, -1, 0), 10: (0, -1, 0),
    11: (0, 1, 0), 12: (0, 1, 0), 13: (0, 1, 0), 14: (0, 1, 0),
    15: (0, 0, -1), 16: (0, 0, -1), 17: (1, 0, 0), 18: (1, 0, 0),
    19: (0, 0, -1), 20: (0, 0, -1), 21: (1, 0, 0), 22: (1, 0, 0),
}


def test_global_yaw_propagates():
    """When the actor yaws 90° globally, every bone should rotate 90° too."""
    defT = def_ang_tpose()
    refWorld = defT          # idealised calibration
    yaw = axangle([0, 0, 1], np.pi / 2)

    for i in range(23):
        raw_after_yaw = qnorm(qmul(yaw, refWorld[i]))
        cand = qnorm(qmul(raw_after_yaw, qinv(refWorld[i])))
        world_q = qnorm(qmul(cand, defT[i]))
        d = qrot(world_q, np.array([1.0, 0.0, 0.0]))
        expected = qrot(yaw, np.array(T_POSE_DIRS[i], dtype=float))
        err = np.degrees(np.arccos(float(np.clip(np.dot(d, expected), -1, 1))))
        assert err < 0.5, f"seg {i}: err {err:.3f}° (got {d}, expected {expected})"


if __name__ == "__main__":
    test_global_yaw_propagates()
    print("test_global_yaw: PASS")
