"""F18 test (regression): each of the 23 segment default angles, applied
to identity raw input, must produce the anatomically expected bone direction
in T-pose. Failure here would mean the FK reference frame is broken."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from quat_math import qrot, euler_xyz, PI


def def_ang_tpose():
    E = euler_xyz
    return ([E(0, -PI/2, 0)] * 7
            + [E(0, 0, -PI/2)] * 4
            + [E(0, 0,  PI/2)] * 4
            + [E(0,  PI/2, 0)] * 2 + [E(0, 0, 0)] * 2
            + [E(0,  PI/2, 0)] * 2 + [E(0, 0, 0)] * 2)


NAMES = ["pelvis", "L5", "L3", "T12", "T8", "Neck", "Head",
         "RShoulder", "RUpperArm", "RForearm", "RHand",
         "LShoulder", "LUpperArm", "LForearm", "LHand",
         "RUpperLeg", "RLowerLeg", "RFoot", "RToe",
         "LUpperLeg", "LLowerLeg", "LFoot", "LToe"]

EXPECTED_T = {
    0: (0, 0, 1), 1: (0, 0, 1), 2: (0, 0, 1), 3: (0, 0, 1),
    4: (0, 0, 1), 5: (0, 0, 1), 6: (0, 0, 1),
    7: (0, -1, 0), 8: (0, -1, 0), 9: (0, -1, 0), 10: (0, -1, 0),
    11: (0, 1, 0), 12: (0, 1, 0), 13: (0, 1, 0), 14: (0, 1, 0),
    15: (0, 0, -1), 16: (0, 0, -1), 17: (1, 0, 0), 18: (1, 0, 0),
    19: (0, 0, -1), 20: (0, 0, -1), 21: (1, 0, 0), 22: (1, 0, 0),
}


def test_default_t_pose():
    defT = def_ang_tpose()
    bad = []
    for i in range(23):
        d = qrot(defT[i], np.array([1.0, 0.0, 0.0]))
        e = np.array(EXPECTED_T[i], dtype=float)
        err = np.degrees(np.arccos(float(np.clip(np.dot(d, e), -1, 1))))
        if err > 0.5:
            bad.append((i, NAMES[i], d.tolist(), e.tolist(), err))
    assert not bad, f"T-pose default angle errors: {bad}"


if __name__ == "__main__":
    test_default_t_pose()
    print("test_default_angles: PASS")
