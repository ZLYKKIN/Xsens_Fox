"""F-3.3 — adaptive mag-disable.  In logs/fox_mocap.log lower_legs and
feet sit at residual=50-85° with mode=ecomp (lines 7111-7116).  The new
FoxKf has to auto-disable mag updates when innovation stays large, and
re-enable when the field is clean again.  We feed a synthetic body with
ground-truth identity orientation and check residual via gravity-only
sensor fusion stays stable when mag is noisy."""

import numpy as np
from fox_kf_mirror import FoxKf, FoxKfSettings


def _normal_acc(kf):
    kf.update_acc(np.array([0.0, 0.0, -1.0], dtype=np.float32))


def test_orientation_stays_close_with_clean_mag():
    kf = FoxKf(); kf.initialise(FoxKfSettings())
    cosD = np.cos(1.047); sinD = np.sin(1.047)
    for _ in range(500):
        kf.predict(np.array([0.0, 0.0, 0.0], dtype=np.float32), 1/90.0)
        _normal_acc(kf)
        kf.update_mag(np.array([cosD, 0.0, -sinD], dtype=np.float32))
    q = kf.orient()
    ang_deg = 2.0 * np.degrees(np.arccos(min(1.0, abs(q[0]))))
    assert ang_deg < 5.0, f"clean-mag run drift {ang_deg:.2f}° > 5°"


def test_orientation_stays_close_with_corrupted_mag():
    kf = FoxKf(); kf.initialise(FoxKfSettings())
    bad = np.array([0.6, 0.7, -0.4], dtype=np.float32)
    for _ in range(500):
        kf.predict(np.array([0.0, 0.0, 0.0], dtype=np.float32), 1/90.0)
        _normal_acc(kf)
        kf.update_mag(bad)
    q = kf.orient()
    ang_deg = 2.0 * np.degrees(np.arccos(min(1.0, abs(q[0]))))
    assert ang_deg < 15.0, (
        f"corrupted-mag drift {ang_deg:.2f}° must stay <15° with mag-disable")
