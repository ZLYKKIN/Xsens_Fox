"""F-6.1 — checks that every quat emitted to plugins has w >= 0.  Uses
real shoulder pair from logs/fox_mocap.log:8285 where R.w=-0.501 and
L.w=-0.837 — both negative.  After canonicalize at the LiveStreamSender
boundary, downstream receivers must see only positive w."""

from quat_math import canonical_hemisphere, qmul, qinv, qnorm, qangle_deg
from log_fixtures import PAIR_SYMMETRY
import numpy as np


def _pair_to_quats(p):
    rw, lw = p["rw"], p["lw"]
    sR = max(0.0, 1.0 - rw * rw)
    sL = max(0.0, 1.0 - lw * lw)
    qR = qnorm(np.array([rw,  np.sqrt(sR)*0.5, np.sqrt(sR)*0.5, np.sqrt(sR)*np.sqrt(0.5)]))
    qL = qnorm(np.array([lw, -np.sqrt(sL)*0.5, np.sqrt(sL)*0.5, np.sqrt(sL)*np.sqrt(0.5)]))
    return qR, qL


def _emit(q_seg, q_baseline):
    q = qnorm(qmul(q_seg, qinv(q_baseline)))
    return canonical_hemisphere(q)


def test_negative_w_pair_emits_positive():
    qR, qL = _pair_to_quats(PAIR_SYMMETRY[("r_shoulder", "l_shoulder")])
    baseline = np.array([1.0, 0.0, 0.0, 0.0])
    sentR = _emit(qR, baseline)
    sentL = _emit(qL, baseline)
    assert sentR[0] >= 0.0, f"r_shoulder emitted w={sentR[0]}, must be >=0"
    assert sentL[0] >= 0.0, f"l_shoulder emitted w={sentL[0]}, must be >=0"


def test_no_sign_flip_in_small_angle_stream():
    base = np.array([1.0, 0.0, 0.0, 0.0])
    seq = []
    for i in range(30):
        ang = np.deg2rad(0.1 * i)
        x = np.sin(ang * 0.5)
        w = np.cos(ang * 0.5)
        q = qnorm(np.array([w, x, 0.0, 0.0]))
        seq.append(_emit(q, base))
    for a, b in zip(seq, seq[1:]):
        assert float(np.dot(a, b)) >= -1e-6, (
            "stream of small-angle increments should never flip sign after canonical")


def test_canonical_preserves_rotation_after_emit():
    base = np.array([1.0, 0.0, 0.0, 0.0])
    for vals in PAIR_SYMMETRY.values():
        qR, qL = _pair_to_quats(vals)
        for q in (qR, qL):
            raw   = qnorm(qmul(q, qinv(base)))
            canon = canonical_hemisphere(raw)
            delta = qmul(canon, qinv(raw))
            ang = qangle_deg(delta)
            assert ang < 1e-3 or abs(ang - 360.0) < 1e-3, ang
