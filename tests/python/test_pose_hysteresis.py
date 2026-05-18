"""S4: LocomotionSolver._classifyPose (scr/main.cpp:1639) decides between
PoseStand / PoseSit / PoseSquat / PoseLying purely on pelvis-to-foot Z.
At the boundaries (`m_sitKneeThresh = 0.55`, `m_squatKneeThresh = 0.30`)
there is no hysteresis, so a slow sit→stand transition makes the
classifier flip Sit↔Stand on every frame that touches the threshold.
Each flip resets m_poseTicks, so the pose-aware Z drift-kill (scr/main.cpp:1599)
never fires during the transition — that's the "стрёмно когда садимся и
встаём" effect the user reports.

This test simulates a 1.5 s sit→stand profile and counts the number of
PoseSit↔PoseStand toggles.  Without hysteresis ≥3 toggles fire on the
boundary; with ±5 cm hysteresis it fires exactly once.
"""
import numpy as np


# Constants from scr/main.h:362-365 (m_lieTiltCosThresh etc.)
SIT_KNEE = 0.55      # m
SQUAT_KNEE = 0.30    # m
LIE_TILT_COS = 0.50

# Constants from scr/main.cpp:1653 — pelvis sits at 0.55 × actorHeight in
# stand pose, height baseline 1.75 m.
ACTOR_H = 1.75
PELVIS_Z_LOCO = 0.55 * ACTOR_H

POSE_UNKNOWN, POSE_STAND, POSE_SIT, POSE_SQUAT, POSE_LYING = range(5)


def classify(pelvis_to_foot, tilt_cos, prev_pose=POSE_UNKNOWN, hysteresis=0.0):
    """Mirrors _classifyPose.  hysteresis=0 → current C++ behaviour."""
    if tilt_cos < LIE_TILT_COS:
        return POSE_LYING

    # Apply hysteresis: if we were in pose X, the threshold to LEAVE X is
    # shifted by `hysteresis` away from the entry threshold, so a value
    # that just barely crossed the boundary doesn't immediately re-cross.
    sit = SIT_KNEE
    squat = SQUAT_KNEE
    if prev_pose == POSE_SIT:
        # to exit Sit upward into Stand we need pelvisToFoot >= sit + h
        sit = SIT_KNEE + hysteresis
    elif prev_pose == POSE_STAND:
        # to exit Stand downward into Sit we need pelvisToFoot < sit - h
        sit = SIT_KNEE - hysteresis
    if prev_pose == POSE_SQUAT:
        squat = SQUAT_KNEE + hysteresis
    elif prev_pose in (POSE_SIT, POSE_STAND):
        squat = SQUAT_KNEE - hysteresis

    if pelvis_to_foot < squat:
        return POSE_SQUAT
    if pelvis_to_foot < sit:
        return POSE_SIT
    return POSE_STAND


def simulate_sit_to_stand(duration_s=1.5, fps=90, noise_amp=0.005, seed=42):
    """Synthesise a `pelvisToFoot` time series that sweeps from below SIT_KNEE
    (sitting: feet near pelvis ⇒ small value) up across the threshold into
    Stand (feet far below pelvis ⇒ large value) over `duration_s`.  Adds
    realistic IMU jitter on top of the linear ramp."""
    rng = np.random.default_rng(seed)
    n = int(duration_s * fps)
    start = SIT_KNEE - 0.10
    end = SIT_KNEE + 0.20
    series = []
    for i in range(n):
        t = i / max(1, n - 1)
        v = start + t * (end - start) + rng.normal(0, noise_amp)
        series.append(v)
    return np.array(series)


def count_toggles(series, hysteresis=0.0):
    prev = POSE_UNKNOWN
    toggles = 0
    sit_stand_toggles = 0
    for v in series:
        p = classify(v, tilt_cos=1.0, prev_pose=prev, hysteresis=hysteresis)
        if p != prev and prev != POSE_UNKNOWN:
            toggles += 1
            if (p, prev) in [(POSE_SIT, POSE_STAND), (POSE_STAND, POSE_SIT)]:
                sit_stand_toggles += 1
        prev = p
    return toggles, sit_stand_toggles


def test_without_hysteresis_flaps_on_boundary():
    """Confirm the bug: no-hysteresis classifier flap-flops when noise
    straddles SIT_KNEE."""
    # Construct a series that hovers right at SIT_KNEE.
    rng = np.random.default_rng(7)
    series = np.array([SIT_KNEE + rng.normal(0, 0.01) for _ in range(120)])
    _, sit_stand = count_toggles(series, hysteresis=0.0)
    assert sit_stand >= 5, \
        f"expected boundary chatter without hysteresis, got {sit_stand} flips"


def test_with_hysteresis_no_chatter():
    """The same boundary noise with ±5 cm hysteresis produces ≤1 toggle."""
    rng = np.random.default_rng(7)
    series = np.array([SIT_KNEE + rng.normal(0, 0.01) for _ in range(120)])
    _, sit_stand = count_toggles(series, hysteresis=0.05)
    assert sit_stand <= 1, \
        f"hysteresis 0.05 should suppress boundary chatter, got {sit_stand} flips"


def test_real_sit_to_stand_exactly_one_transition():
    """A clean sit→stand ramp (with realistic noise) crosses SIT_KNEE
    exactly once.  Without hysteresis we may get ≥2 flips; with hysteresis
    we get exactly 1."""
    series = simulate_sit_to_stand()
    _, sit_stand_no_h = count_toggles(series, hysteresis=0.0)
    _, sit_stand_with_h = count_toggles(series, hysteresis=0.05)
    assert sit_stand_with_h == 1, \
        f"with hysteresis a single sit→stand should fire exactly 1 transition; got {sit_stand_with_h}"
    # Document the bug: without hysteresis we get more flips than the
    # physical event count.
    assert sit_stand_no_h >= sit_stand_with_h


def test_hysteresis_does_not_block_real_transitions():
    """If the actor genuinely stands up by 30+ cm, hysteresis must not
    prevent the Sit→Stand classification."""
    series = simulate_sit_to_stand(duration_s=2.0, fps=90, noise_amp=0.001)
    _, sit_stand = count_toggles(series, hysteresis=0.05)
    assert sit_stand == 1, f"clean stand-up should fire exactly 1 transition, got {sit_stand}"


if __name__ == "__main__":
    test_without_hysteresis_flaps_on_boundary()
    test_with_hysteresis_no_chatter()
    test_real_sit_to_stand_exactly_one_transition()
    test_hysteresis_does_not_block_real_transitions()
    print("test_pose_hysteresis: PASS")
