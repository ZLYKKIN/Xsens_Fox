"""Python mirror of scr/fusion/FoxKf.cpp (Multiplicative EKF).

Used by the regression tests to verify the C++ implementation mathematically.
Identical state/jacobian/update equations; pure numpy.  All settings names
mirror FoxKfSettings 1:1.

State:
    q   : 4-quat (w, x, y, z), world ← body, Hamilton
    b_g : 3-gyro bias estimate, rad/s
    P   : 6x6 covariance of δx = [δθ (rotation vec), δb_g]
"""
import numpy as np


class FoxKfSettings:
    def __init__(self):
        self.gyroNoiseStd     = 0.005
        self.gyroBiasRwStd    = 1.0e-5
        self.accNoiseStd      = 0.05
        self.magNoiseStd      = 0.10
        self.accRejectG       = 0.30
        self.magRejectUnit    = 0.40
        self.magDipRad        = 1.047
        self.zuptOmegaThresh  = 0.05
        self.zuptAccThresh    = 0.03
        self.zuptHoldFrames   = 30
        self.initOrientStdDeg = 5.0
        self.initBiasStd      = 0.5
        self.magDisableResidualDeg = 25.0
        self.magDisableHoldFrames  = 30
        self.magReenableResidual   = 0.15
        self.biasAnchorRate        = 0.01
        self.biasAnchorFrames      = 5400
        self.dthetaSanityRad       = np.pi


def quat_mul(a, b):
    return np.array([
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    ])


def quat_norm(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1., 0, 0, 0])
    return q / n


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_rot(q, v):
    qv = np.array([q[1], q[2], q[3]])
    t = 2.0 * np.cross(qv, v)
    return v + q[0] * t + np.cross(qv, t)


def quat_rot_inv(q, v):
    return quat_rot(quat_conj(q), v)


def skew(v):
    return np.array([
        [   0, -v[2],  v[1]],
        [v[2],     0, -v[0]],
        [-v[1], v[0],     0]
    ])


def exp_so3(w):
    a = float(np.linalg.norm(w))
    if a < 1e-8:
        return np.array([1.0, 0.5*w[0], 0.5*w[1], 0.5*w[2]])
    h = 0.5 * a
    c = np.cos(h)
    s = np.sin(h) / a
    return np.array([c, w[0]*s, w[1]*s, w[2]*s])


def _is_finite_arr(a):
    return bool(np.all(np.isfinite(np.asarray(a))))


class FoxKf:
    def __init__(self, settings=None):
        self.initialise(settings)

    def initialise(self, settings=None):
        self.s = settings if settings is not None else FoxKfSettings()
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.b = np.zeros(3)
        a = np.deg2rad(self.s.initOrientStdDeg)
        self.P = np.zeros((6, 6))
        for i in range(3):
            self.P[i, i] = a * a
            self.P[3+i, 3+i] = self.s.initBiasStd ** 2
        self.last_gyr_corr = np.zeros(3)
        self.last_acc = np.zeros(3)
        self.still_ticks = 0
        self.stationary = False
        self.mag_resid_lp = 0.0
        self.mag_disable_ticks = 0
        self.mag_disabled = False
        self.b_snap = np.zeros(3)
        self.frames_since_zupt = 0
        self.auto_reset_count = 0

    def set_prior(self, q_world_body, bias_init,
                  orient_std_deg=None, bias_std=None):
        q_new = quat_norm(np.asarray(q_world_body, dtype=float))
        b_new = np.asarray(bias_init, dtype=float).copy()
        if not _is_finite_arr(q_new): q_new = np.array([1.0, 0.0, 0.0, 0.0])
        if not _is_finite_arr(b_new): b_new = np.zeros(3)
        self.q = q_new
        self.b = b_new
        a_deg = orient_std_deg if orient_std_deg is not None else self.s.initOrientStdDeg
        b_std = bias_std if bias_std is not None else self.s.initBiasStd
        a = np.deg2rad(a_deg)
        self.P = np.zeros((6, 6))
        for i in range(3):
            self.P[i, i] = a * a
            self.P[3+i, 3+i] = b_std * b_std
        self.b_snap = self.b.copy()
        self.frames_since_zupt = 0
        self.mag_resid_lp = 0.0
        self.mag_disable_ticks = 0
        self.mag_disabled = False

    def _maybe_auto_reset(self):
        if (not _is_finite_arr(self.q)) or (not _is_finite_arr(self.b)) or (not _is_finite_arr(self.P)):
            settings = self.s
            self.initialise(settings)
            self.auto_reset_count += 1

    def predict(self, gyr, dt):
        if dt <= 0:
            return
        dt = min(dt, 0.5)
        gyr = np.asarray(gyr, dtype=float)
        if not _is_finite_arr(gyr):
            return
        w = gyr - self.b
        self.last_gyr_corr = w.copy()
        wdt = w * dt
        self.q = quat_norm(quat_mul(self.q, exp_so3(wdt)))

        F = np.zeros((6, 6))
        F[:3, :3] = np.eye(3) - skew(wdt)
        F[:3, 3:] = -np.eye(3) * dt
        F[3:, 3:] = np.eye(3)
        Q = np.zeros((6, 6))
        qg = self.s.gyroNoiseStd ** 2 * dt * dt
        qb = self.s.gyroBiasRwStd ** 2 * dt
        for i in range(3):
            Q[i, i] = qg
            Q[3+i, 3+i] = qb
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

        self.frames_since_zupt += 1
        if self.frames_since_zupt > self.s.biasAnchorFrames:
            k = min(1.0, self.s.biasAnchorRate * dt)
            self.b = (1.0 - k) * self.b + k * self.b_snap

        wmag = float(np.linalg.norm(w))
        amag = float(np.linalg.norm(self.last_acc))
        still_now = (wmag < self.s.zuptOmegaThresh) and \
                    (abs(amag - 1.0) < self.s.zuptAccThresh)
        if still_now:
            self.still_ticks = min(self.still_ticks + 1, 100000)
        else:
            self.still_ticks = 0
        self.stationary = self.still_ticks >= self.s.zuptHoldFrames

        self._maybe_auto_reset()

    def _update3(self, innov, H, R_diag):
        S = H @ self.P @ H.T + R_diag * np.eye(3)
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.P @ H.T @ Sinv
        dx = K @ innov
        d_theta = dx[:3]
        if float(np.linalg.norm(d_theta)) > self.s.dthetaSanityRad:
            return
        d_bias = dx[3:]
        self.q = quat_norm(quat_mul(exp_so3(d_theta), self.q))
        self.b = self.b + d_bias
        I6 = np.eye(6)
        self.P = (I6 - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)

    def update_acc(self, acc_unit_g):
        a = np.asarray(acc_unit_g, dtype=float)
        if not _is_finite_arr(a):
            return
        self.last_acc = a.copy()
        mag = float(np.linalg.norm(a))
        err = abs(mag - 1.0)
        if err > self.s.accRejectG * 2.0:
            return
        rScale = 1.0 + 9.0 * min(1.0, err / max(1e-6, self.s.accRejectG))
        if err > self.s.accRejectG:
            rScale *= 1.0 + (err - self.s.accRejectG) * 20.0
        rA = self.s.accNoiseStd ** 2 * rScale
        grav_up = np.array([0.0, 0.0, -1.0])
        g_body = quat_rot_inv(self.q, grav_up)
        innov = a - g_body
        H = np.zeros((3, 6))
        H[:3, :3] = skew(g_body)
        self._update3(innov, H, rA)
        self._maybe_auto_reset()

    def update_mag(self, mag_unit):
        v = np.asarray(mag_unit, dtype=float)
        if not _is_finite_arr(v):
            return
        mag = float(np.linalg.norm(v))
        err = abs(mag - 1.0)
        if err > self.s.magRejectUnit * 2.0:
            return
        cD = np.cos(self.s.magDipRad)
        sD = np.sin(self.s.magDipRad)
        m_ref = np.array([cD, 0.0, -sD])
        m_body = quat_rot_inv(self.q, m_ref)
        innov = v - m_body
        innov_norm = float(np.linalg.norm(innov))

        thresh = np.sin(np.deg2rad(self.s.magDisableResidualDeg))
        if self.mag_disabled:
            self.mag_resid_lp = 0.95 * self.mag_resid_lp + 0.05 * innov_norm
            if self.mag_resid_lp < self.s.magReenableResidual:
                self.mag_disabled = False
                self.mag_disable_ticks = 0
            return
        self.mag_resid_lp = 0.9 * self.mag_resid_lp + 0.1 * innov_norm
        if self.mag_resid_lp > thresh:
            self.mag_disable_ticks += 1
            if self.mag_disable_ticks > self.s.magDisableHoldFrames:
                self.mag_disabled = True
                return
        else:
            self.mag_disable_ticks = max(0, self.mag_disable_ticks - 1)

        rScale = 1.0 + 9.0 * min(1.0, err / max(1e-6, self.s.magRejectUnit))
        rM = self.s.magNoiseStd ** 2 * rScale
        H = np.zeros((3, 6))
        H[:3, :3] = skew(m_body)
        self._update3(innov, H, rM)
        self._maybe_auto_reset()

    def update_zupt(self):
        innov = -self.last_gyr_corr.copy()
        H = np.zeros((3, 6))
        H[:3, 3:] = -np.eye(3)
        r = self.s.gyroNoiseStd ** 2 * 0.1
        S = H @ self.P @ H.T + r * np.eye(3)
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.P @ H.T @ Sinv
        dx = K @ innov
        d_theta = dx[:3]
        if float(np.linalg.norm(d_theta)) > self.s.dthetaSanityRad:
            return
        self.q = quat_norm(quat_mul(exp_so3(d_theta), self.q))
        self.b = self.b + dx[3:]
        I6 = np.eye(6)
        self.P = (I6 - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)

        self.b_snap = self.b.copy()
        self.frames_since_zupt = 0
        self._maybe_auto_reset()

    def orient(self):
        return self.q.copy()

    def gyro_bias(self):
        return self.b.copy()

    def orient_std_deg(self):
        var = self.P[0, 0] + self.P[1, 1] + self.P[2, 2]
        return float(np.sqrt(max(0.0, var))) * 180.0 / np.pi

    def bias_std(self):
        var = (self.P[3, 3] + self.P[4, 4] + self.P[5, 5]) / 3.0
        return float(np.sqrt(max(0.0, var)))
