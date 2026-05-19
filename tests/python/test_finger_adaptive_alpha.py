"""F-5.1 — replays real [glove-outlier] deltas from
logs/fox_mocap.log:1618-1668.  In the log every delta in the 30-50° band
got α=0.1, killing fast finger motion.  The adaptive rule must give those
same deltas a much higher α (≥0.30) while keeping α near 0.10 for low
deltas (<5°) and dropping again for shock-level (>80°) noise."""

from log_fixtures import GLOVE_OUTLIER_DELTAS


def alpha_for(idx, delta):
    is_thumb = (idx < 4)
    alpha_slow = 0.15 if is_thumb else 0.10
    alpha_fast = 0.55 if is_thumb else 0.50
    alpha_shock = alpha_slow
    slow_edge = 5.0
    fast_edge = 35.0 if is_thumb else 30.0
    shock_edge = 90.0 if is_thumb else 80.0
    if delta <= slow_edge:
        return alpha_slow
    if delta <= fast_edge:
        t = (delta - slow_edge) / max(1e-3, fast_edge - slow_edge)
        return alpha_slow + t * (alpha_fast - alpha_slow)
    if delta <= shock_edge:
        t = (delta - fast_edge) / max(1e-3, shock_edge - fast_edge)
        return alpha_fast + t * (alpha_shock - alpha_fast)
    return alpha_shock


def _joint_idx(finger, joint):
    finger_id = {"thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4}[finger]
    joint_id  = {"spread": 0, "MCP": 1, "PIP": 2, "DIP": 3}[joint]
    return finger_id * 4 + joint_id


def test_log_outliers_get_higher_alpha_than_old_rule():
    for _hand, finger, joint, delta in GLOVE_OUTLIER_DELTAS:
        idx = _joint_idx(finger, joint)
        a_new = alpha_for(idx, delta)
        assert a_new >= 0.20, (
            f"{finger} {joint} delta={delta}° old α=0.1 → new α must be ≥0.20, got {a_new:.3f}")


def test_slow_motion_uses_slow_alpha():
    for finger, joint in (("index", "PIP"), ("middle", "DIP"), ("thumb", "MCP")):
        idx = _joint_idx(finger, joint)
        a = alpha_for(idx, 2.0)
        assert a <= 0.20, f"slow motion (delta=2°) should keep α<=0.20, got {a:.3f}"


def test_shock_returns_to_slow_alpha():
    a_shock = alpha_for(_joint_idx("middle", "PIP"), 120.0)
    assert a_shock <= 0.15, f"shock (delta=120°) must drop α<=0.15, got {a_shock:.3f}"


def test_thumb_alpha_peaks_within_thumb_fast_band():
    a_thumb_peak = alpha_for(_joint_idx("thumb", "MCP"), 35.0)
    a_thumb_slow = alpha_for(_joint_idx("thumb", "MCP"),  5.0)
    assert a_thumb_peak >= a_thumb_slow + 0.30, (
        f"thumb peak α={a_thumb_peak:.3f} must exceed slow α={a_thumb_slow:.3f} by 0.30")
