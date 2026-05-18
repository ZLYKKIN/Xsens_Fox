"""F1+F5 test: the locomotion solver's confidence/still detection should
not commit during the swing phase of normal walking.

Mirrors the m_stillRad, m_confCommit/Release, m_confRise/FallRate maths
from scr/main.cpp:LocomotionSolver::update."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

# Constants matching scr/main.h (after F1/F5 fixes).
STILL_RAD = 0.30        # was 3.00
HEIGHT_MARGIN_SLOW = 0.08
CONF_COMMIT = 0.55      # was 0.35
CONF_RELEASE = 0.30     # was 0.25
CONF_RISE = 0.50
CONF_FALL = 0.25


def sStill(omega):
    s = (STILL_RAD - omega) / STILL_RAD
    return max(0.0, min(1.0, s))


def sLow(fk_z, fkMinZ):
    dz = fk_z - fkMinZ
    s = 1.0 - dz / max(HEIGHT_MARGIN_SLOW, 1e-3)
    return max(0.0, min(1.0, s))


def smooth(prev, raw):
    r = CONF_RISE if raw > prev else CONF_FALL
    return (1 - r) * prev + r * raw


def simulate_walk(duration_s=6.0, fps=90):
    """Synthetic 1.5 Hz cadence single-foot motion: stance @ low ω, swing
    @ peak 6 rad/s.  Returns the commit timeline."""
    n = int(duration_s * fps)
    omega = np.zeros(n)
    fk_z = np.zeros(n)
    cadence = 1.5
    stance_frac = 0.55
    for i in range(n):
        t = i / fps
        phase = (t * cadence) % 1.0
        if phase < stance_frac:
            omega[i] = 0.05 + 0.05 * np.random.rand()      # standing-still
            fk_z[i] = 0.0
        else:
            sp = (phase - stance_frac) / (1.0 - stance_frac)
            omega[i] = 6.0 * np.sin(np.pi * sp)             # 0 -> 6 -> 0
            fk_z[i] = 0.12 * np.sin(np.pi * sp)             # foot lifts 12 cm
    return omega, fk_z


def test_no_chatter_when_standing():
    """10 s of foot held still — commit fires once, never chatters."""
    np.random.seed(0)
    omega = np.full(900, 0.03)
    fk_z = np.zeros(900)
    conf = 0.0
    committed = False
    flips = 0
    for i in range(len(omega)):
        raw = sStill(omega[i]) * sLow(fk_z[i], 0.0)
        conf = smooth(conf, raw)
        if not committed and conf >= CONF_COMMIT:
            committed = True; flips += 1
        elif committed and conf < CONF_RELEASE:
            committed = False; flips += 1
    assert committed, "should be committed at end of stand-still"
    assert flips <= 1, f"chatter: {flips} commit/release flips during stand-still"


def test_walking_releases_during_swing():
    """During normal walking, the foot must release commit at peak swing."""
    np.random.seed(1)
    omega, fk_z = simulate_walk(duration_s=4.0, fps=90)
    conf = 0.0
    committed = False
    committed_swing = 0
    for i in range(len(omega)):
        raw = sStill(omega[i]) * sLow(fk_z[i], 0.0)
        conf = smooth(conf, raw)
        if not committed and conf >= CONF_COMMIT:
            committed = True
        elif committed and conf < CONF_RELEASE:
            committed = False
        # Count frames where committed=True while foot is actively swinging.
        if committed and omega[i] > 3.0:
            committed_swing += 1
    # Allow up to a handful of frames lag (release is filtered) but not
    # a continuous "committed throughout swing" state.
    assert committed_swing < 10, \
        f"committed during {committed_swing} swing frames — F1 threshold still too loose"


if __name__ == "__main__":
    test_no_chatter_when_standing()
    test_walking_releases_during_swing()
    print("test_locomotion_thresholds: PASS")
