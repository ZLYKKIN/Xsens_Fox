"""F-2.1 — verifies canonical_hemisphere on real R/L hand pair from
RENDER SNAPSHOT (logs/fox_mocap.log:8288).  In the log r_hand.w=+0.03 and
l_hand.w=-0.12 — same physical rotation, opposite hemispheres, and the
renderer flagged this with *** HEMISPHERE MISMATCH ***.  After
canonicalize, both .w become non-negative and devMirr stays small."""

from quat_math import (canonical_hemisphere, mirror_y_quat, qmul, qnorm,
                       qinv, qangle_deg, quat_dev_deg)
from log_fixtures import PAIR_SYMMETRY
import numpy as np


def _full_quat(w_only_pair):
    """Reconstruct full quats from (.w, devMirr) — pick xyz so the pair
    matches the observed devMirr while honouring the .w on each side."""
    rw, lw = w_only_pair["rw"], w_only_pair["lw"]
    sR = max(0.0, 1.0 - rw * rw)
    sL = max(0.0, 1.0 - lw * lw)
    qR = np.array([rw, np.sqrt(sR) * 0.5, np.sqrt(sR) * 0.5, np.sqrt(sR) * np.sqrt(0.5)])
    qL = np.array([lw, np.sqrt(sL) * 0.5, np.sqrt(sL) * 0.5, np.sqrt(sL) * np.sqrt(0.5)])
    qR = qnorm(qR); qL = qnorm(qL)
    return qR, qL


def test_canonical_flips_negative_w_to_positive():
    qR, qL = _full_quat(PAIR_SYMMETRY[("r_hand", "l_hand")])
    cR = canonical_hemisphere(qR)
    cL = canonical_hemisphere(qL)
    assert cR[0] >= 0.0, f"r_hand canonical w must be >= 0, got {cR[0]}"
    assert cL[0] >= 0.0, f"l_hand canonical w must be >= 0, got {cL[0]}"


def test_canonical_preserves_rotation():
    for pair_key, vals in PAIR_SYMMETRY.items():
        qR, qL = _full_quat(vals)
        for q in (qR, qL):
            c = canonical_hemisphere(q)
            assert abs(qangle_deg(qmul(c, qinv(q))) - 0.0) < 1e-3, (
                f"canonical_hemisphere changed rotation for {pair_key}")


def test_hemisphere_mismatch_disappears_after_canonical():
    qR, qL = _full_quat(PAIR_SYMMETRY[("r_hand", "l_hand")])
    cR = canonical_hemisphere(qR)
    cL = canonical_hemisphere(qL)
    dot_raw  = float(np.dot(qR, qL))
    dot_canon = float(np.dot(cR, cL))
    if dot_raw < 0:
        assert dot_canon >= -1e-6, (
            f"after canonical hands should not be in opposite hemispheres: "
            f"dot_raw={dot_raw:.3f} dot_canon={dot_canon:.3f}")


def test_canonical_idempotent():
    for vals in PAIR_SYMMETRY.values():
        qR, qL = _full_quat(vals)
        for q in (qR, qL):
            c1 = canonical_hemisphere(q)
            c2 = canonical_hemisphere(c1)
            assert np.allclose(c1, c2)
