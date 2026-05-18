// ============================================================================
//  FoxKf — implementation of the unified per-sensor MEKF.
//  See FoxKf.h for the architecture overview.
// ============================================================================

#include "FoxKf.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace fox {

// ---------- Tiny dense-linalg helpers (no external dependencies) ------------
//
// 6×6 covariance is row-major: index(i,j) = 6*i + j.  We hand-roll the few
// operations the EKF needs rather than pulling Eigen, because:
//   • the dimensions are fixed (6×6 / 6×3 / 3×3),
//   • the receiver thread does this at 100-240 Hz × 17 sensors and we want
//     deterministic perf,
//   • no third-party headers to vendor for Windows MSVC build.

static inline int idx66(int i, int j) { return 6 * i + j; }

static inline void zero66(float* M) { std::memset(M, 0, 36 * sizeof(float)); }

static inline void identity66(float* M) {
    zero66(M);
    for (int i = 0; i < 6; ++i) M[idx66(i, i)] = 1.0f;
}

// C = A·B  where A is 6×6, B is 6×6, C is 6×6.
static void mul66x66(const float* A, const float* B, float* C) {
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 6; ++k) s += A[idx66(i, k)] * B[idx66(k, j)];
            C[idx66(i, j)] = s;
        }
}

// C = A·Bᵀ.  Both 6×6.
static void mul66x66t(const float* A, const float* B, float* C) {
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 6; ++k) s += A[idx66(i, k)] * B[idx66(j, k)];
            C[idx66(i, j)] = s;
        }
}

// C = A + B.  Both 6×6.  (Kept as a utility for the linalg toolbox; the
// EKF path uses in-place += in the Q-add step.)
[[maybe_unused]] static void add66(const float* A, const float* B, float* C) {
    for (int i = 0; i < 36; ++i) C[i] = A[i] + B[i];
}

// Symmetrize: M = 0.5·(M + Mᵀ).  Numerical hygiene each frame.
static void symmetrize66(float* M) {
    for (int i = 0; i < 6; ++i)
        for (int j = i + 1; j < 6; ++j) {
            float s = 0.5f * (M[idx66(i, j)] + M[idx66(j, i)]);
            M[idx66(i, j)] = s;
            M[idx66(j, i)] = s;
        }
}

// y = M·x  where M is 6×3, x is 3-vec, y is 6-vec.  Used in K·innov.
static void mul63x3(const float* M, const float* x, float* y) {
    for (int i = 0; i < 6; ++i) {
        float s = 0.0f;
        for (int k = 0; k < 3; ++k) s += M[3 * i + k] * x[k];
        y[i] = s;
    }
}

// C = A·B  where A is 6×6, B is 6×3, C is 6×3.  (Kept as a utility; current
// EKF path computes PHᵀ and KH inline.)
[[maybe_unused]] static void mul66x63(const float* A, const float* B, float* C) {
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 6; ++k) s += A[idx66(i, k)] * B[3 * k + j];
            C[3 * i + j] = s;
        }
}

// 3×3 inversion via adjugate (we only invert the 3×3 innovation cov).
// Returns false on near-singular.
static bool inv33(const float* A, float* I) {
    const float a = A[0], b = A[1], c = A[2];
    const float d = A[3], e = A[4], f = A[5];
    const float g = A[6], h = A[7], k = A[8];
    const float det = a * (e * k - f * h)
                    - b * (d * k - f * g)
                    + c * (d * h - e * g);
    if (std::fabs(det) < 1e-12f) return false;
    const float invDet = 1.0f / det;
    I[0] =  (e * k - f * h) * invDet;
    I[1] = -(b * k - c * h) * invDet;
    I[2] =  (b * f - c * e) * invDet;
    I[3] = -(d * k - f * g) * invDet;
    I[4] =  (a * k - c * g) * invDet;
    I[5] = -(a * f - c * d) * invDet;
    I[6] =  (d * h - e * g) * invDet;
    I[7] = -(a * h - b * g) * invDet;
    I[8] =  (a * e - b * d) * invDet;
    return true;
}

// 3×3 skew-symmetric matrix [v]_× (row-major 9-element).
static void skew3(const Vec3& v, float* M) {
    M[0] = 0.0f;   M[1] = -v[2]; M[2] =  v[1];
    M[3] =  v[2];  M[4] = 0.0f;  M[5] = -v[0];
    M[6] = -v[1];  M[7] =  v[0]; M[8] = 0.0f;
}

// Rotate vec v by quat q: r = q ⊗ v ⊗ q⁻¹.  (q in Hamilton WXYZ.)
static Vec3 quatRot(const Quat4& q, const Vec3& v) {
    const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    const float tx = 2.0f * (qy * v[2] - qz * v[1]);
    const float ty = 2.0f * (qz * v[0] - qx * v[2]);
    const float tz = 2.0f * (qx * v[1] - qy * v[0]);
    Vec3 r;
    r[0] = v[0] + qw * tx + (qy * tz - qz * ty);
    r[1] = v[1] + qw * ty + (qz * tx - qx * tz);
    r[2] = v[2] + qw * tz + (qx * ty - qy * tx);
    return r;
}

// Rotate vec v by inverse(q): r = qᵀ ⊗ v ⊗ q  (q in Hamilton WXYZ).
// Equivalent to quatRot with q.conj.
static Vec3 quatRotInv(const Quat4& q, const Vec3& v) {
    Quat4 qc{q[0], -q[1], -q[2], -q[3]};
    return quatRot(qc, v);
}

// Hamilton product r = a ⊗ b.
static Quat4 quatMul(const Quat4& a, const Quat4& b) {
    return Quat4{
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    };
}

static Quat4 quatNormalize(const Quat4& q) {
    const float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (n2 < 1e-20f) return Quat4{1.0f, 0.0f, 0.0f, 0.0f};
    const float n = std::sqrt(n2);
    return Quat4{q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

// Exponential map: rotation vector (3-vec, axis·angle) → quaternion.
static Quat4 expSO3(const Vec3& w) {
    const float a = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    if (a < 1e-8f) {
        // First-order: cos≈1, sin/a ≈ 1/2.
        return Quat4{1.0f, 0.5f * w[0], 0.5f * w[1], 0.5f * w[2]};
    }
    const float h = 0.5f * a;
    const float c = std::cos(h);
    const float s = std::sin(h) / a;
    return Quat4{c, w[0] * s, w[1] * s, w[2] * s};
}

// ---------- FoxKf ------------------------------------------------------------

FoxKf::FoxKf() {
    initialise();
}

void FoxKf::initialise(const FoxKfSettings& s) {
    m_set = s;
    m_q = Quat4{1.0f, 0.0f, 0.0f, 0.0f};
    m_b = Vec3{0.0f, 0.0f, 0.0f};
    // Initial covariance: large on orient (we know nothing), modest on bias.
    identity66(m_P);
    const float aRad = m_set.initOrientStdDeg * 3.14159265358979f / 180.0f;
    const float a2 = aRad * aRad;
    const float b2 = m_set.initBiasStd * m_set.initBiasStd;
    for (int i = 0; i < 3; ++i) m_P[idx66(i, i)] = a2;
    for (int i = 3; i < 6; ++i) m_P[idx66(i, i)] = b2;
    m_still = false;
    m_stillTicks = 0;
    m_lastGyrCorrected = Vec3{0.0f, 0.0f, 0.0f};
    m_lastAcc          = Vec3{0.0f, 0.0f, 0.0f};
}

void FoxKf::setPrior(const Quat4& qWorldBody, const Vec3& biasInit,
                     float orientStdDeg, float biasStd) {
    m_q = quatNormalize(qWorldBody);
    m_b = biasInit;
    const float aDeg = (orientStdDeg < 0.0f) ? m_set.initOrientStdDeg : orientStdDeg;
    const float bStd = (biasStd      < 0.0f) ? m_set.initBiasStd      : biasStd;
    const float aRad = aDeg * 3.14159265358979f / 180.0f;
    zero66(m_P);
    for (int i = 0; i < 3; ++i) m_P[idx66(i, i)] = aRad * aRad;
    for (int i = 3; i < 6; ++i) m_P[idx66(i, i)] = bStd * bStd;
}

void FoxKf::restart() {
    const float aRad = m_set.initOrientStdDeg * 3.14159265358979f / 180.0f;
    const float b2 = m_set.initBiasStd * m_set.initBiasStd;
    zero66(m_P);
    for (int i = 0; i < 3; ++i) m_P[idx66(i, i)] = aRad * aRad;
    for (int i = 3; i < 6; ++i) m_P[idx66(i, i)] = b2;
    m_still = false;
    m_stillTicks = 0;
}

void FoxKf::predict(const Vec3& gyrRadPerSec, float dt) {
    if (dt <= 0.0f) return;
    if (dt > 0.5f)  dt = 0.5f;          // clamp pathological gaps

    // Corrected angular rate.
    Vec3 w{gyrRadPerSec[0] - m_b[0],
           gyrRadPerSec[1] - m_b[1],
           gyrRadPerSec[2] - m_b[2]};
    m_lastGyrCorrected = w;

    // Quaternion integration (1st-order via exp).
    Vec3 wdt{w[0] * dt, w[1] * dt, w[2] * dt};
    m_q = quatNormalize(quatMul(m_q, expSO3(wdt)));

    // Linearised state-transition  F = [[I - [wdt]_×, -I·dt], [0, I]] (6×6).
    float F[36];
    identity66(F);
    float Sk[9];
    skew3(wdt, Sk);
    // F[0:3,0:3] = I - skew(wdt)
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            F[idx66(i, j)] = ((i == j) ? 1.0f : 0.0f) - Sk[3 * i + j];
    // F[0:3,3:6] = -I · dt
    for (int i = 0; i < 3; ++i) F[idx66(i, 3 + i)] = -dt;
    // F[3:6,0:3] = 0   (already)
    // F[3:6,3:6] = I   (already)

    // Process noise Q (6×6 diag): gyro noise feeds δθ, bias RW feeds δb.
    const float qg = m_set.gyroNoiseStd * m_set.gyroNoiseStd * dt * dt;
    const float qb = m_set.gyroBiasRwStd * m_set.gyroBiasRwStd * dt;

    // P ← F P Fᵀ + Q
    float FP[36], Pnew[36];
    mul66x66(F, m_P, FP);
    mul66x66t(FP, F, Pnew);
    for (int i = 0; i < 3; ++i) Pnew[idx66(i, i)]         += qg;
    for (int i = 0; i < 3; ++i) Pnew[idx66(3 + i, 3 + i)] += qb;
    std::memcpy(m_P, Pnew, sizeof(m_P));
    symmetrize66(m_P);

    // Stationarity gate: small corrected gyro + small |a-1g| (set in updateAcc).
    const float wmag = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    const float aMag = std::sqrt(m_lastAcc[0] * m_lastAcc[0]
                                + m_lastAcc[1] * m_lastAcc[1]
                                + m_lastAcc[2] * m_lastAcc[2]);
    const bool stillNow = (wmag < m_set.zuptOmegaThresh)
                        && (std::fabs(aMag - 1.0f) < m_set.zuptAccThresh);
    if (stillNow) m_stillTicks = std::min(m_stillTicks + 1, 100000);
    else          m_stillTicks = 0;
    m_still = (m_stillTicks >= m_set.zuptHoldFrames);
}

void FoxKf::updateAcc(const Vec3& accUnitG) {
    m_lastAcc = accUnitG;
    const float mag = std::sqrt(accUnitG[0] * accUnitG[0]
                              + accUnitG[1] * accUnitG[1]
                              + accUnitG[2] * accUnitG[2]);
    const float err = std::fabs(mag - 1.0f);
    if (err > m_set.accRejectG * 2.0f) return;  // hard skip on huge shock

    // Inflate R adaptively on borderline cases (XKF3i pattern: track but
    // distrust).  Multiplier scales linearly from 1× at err=0 to 10× at
    // err=accRejectG, then more aggressively beyond.
    float rScale = 1.0f + 9.0f * std::min(1.0f, err / std::max(1e-6f, m_set.accRejectG));
    if (err > m_set.accRejectG) rScale *= 1.0f + (err - m_set.accRejectG) * 20.0f;
    const float rA = m_set.accNoiseStd * m_set.accNoiseStd * rScale;

    // Predicted gravity in body frame: ĝ_body = R(q)ᵀ · [0,0,-1]
    // (XKF convention: accelerometer at rest reads +1g UP in body when body
    // +Z is up; if our convention differs, the per-segment s2s rotation
    // applied before the filter is identity for that mapping.  See receiver
    // path: gyrForFilter / accForFilter already through s2s_inv).
    Vec3 gravUp{0.0f, 0.0f, -1.0f};
    Vec3 gBody = quatRotInv(m_q, gravUp);

    // Innovation z - h(x)
    Vec3 innov{accUnitG[0] - gBody[0],
               accUnitG[1] - gBody[1],
               accUnitG[2] - gBody[2]};

    // Measurement Jacobian H = [ [gBody]_× , 0_{3×3} ] (3×6).
    float H[18];           // row-major 3×6
    float Sk[9];
    skew3(gBody, Sk);
    std::memset(H, 0, sizeof(H));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) H[6 * i + j] = Sk[3 * i + j];
    // H[*,3..5] = 0  (acc doesn't depend on bias)

    // Innovation cov S = H P Hᵀ + R  (3×3).
    // First, HP = H · P (3×6).
    float HP[18];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 6; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 6; ++k) s += H[6 * i + k] * m_P[idx66(k, j)];
            HP[6 * i + j] = s;
        }
    float S33[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 6; ++k) s += HP[6 * i + k] * H[6 * j + k];
            S33[3 * i + j] = s;
            if (i == j) S33[3 * i + j] += rA;
        }
    float Sinv[9];
    if (!inv33(S33, Sinv)) return;       // ill-conditioned, skip

    // Kalman gain K = P Hᵀ S⁻¹  (6×3).
    float PHt[18];                        // 6×3
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 6; ++k) s += m_P[idx66(i, k)] * H[6 * j + k];
            PHt[3 * i + j] = s;
        }
    float K63[18];                        // 6×3
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += PHt[3 * i + k] * Sinv[3 * k + j];
            K63[3 * i + j] = s;
        }

    // δx = K · innov
    float dx[6];
    mul63x3(K63, innov.data(), dx);
    // Apply correction
    Vec3 dTheta{dx[0], dx[1], dx[2]};
    Vec3 dBias{dx[3], dx[4], dx[5]};
    m_q = quatNormalize(quatMul(expSO3(dTheta), m_q));
    m_b[0] += dBias[0]; m_b[1] += dBias[1]; m_b[2] += dBias[2];

    // P ← (I - K H) P  (Joseph form would be more stable but ours is fine
    // when measurements arrive at sane SNR; we symmetrise each frame).
    float KH[36];
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += K63[3 * i + k] * H[6 * k + j];
            KH[idx66(i, j)] = s;
        }
    float ImKH[36];
    identity66(ImKH);
    for (int i = 0; i < 36; ++i) ImKH[i] -= KH[i];
    float Pnew[36];
    mul66x66(ImKH, m_P, Pnew);
    std::memcpy(m_P, Pnew, sizeof(m_P));
    symmetrize66(m_P);
}

void FoxKf::updateMag(const Vec3& magUnit) {
    const float mag = std::sqrt(magUnit[0] * magUnit[0]
                              + magUnit[1] * magUnit[1]
                              + magUnit[2] * magUnit[2]);
    const float err = std::fabs(mag - 1.0f);
    if (err > m_set.magRejectUnit * 2.0f) return;
    float rScale = 1.0f + 9.0f * std::min(1.0f, err / std::max(1e-6f, m_set.magRejectUnit));
    const float rM = m_set.magNoiseStd * m_set.magNoiseStd * rScale;

    // Magnetic reference in world frame: along horizontal north with dip
    // angle pointing down.  Convention NWU: +X = north, +Z = up.
    // Reference unit vector = (cosD, 0, -sinD)  (dip points downward).
    const float cD = std::cos(m_set.magDipRad);
    const float sD = std::sin(m_set.magDipRad);
    Vec3 mRefWorld{cD, 0.0f, -sD};
    Vec3 mBody = quatRotInv(m_q, mRefWorld);

    Vec3 innov{magUnit[0] - mBody[0],
               magUnit[1] - mBody[1],
               magUnit[2] - mBody[2]};

    float H[18];
    float Sk[9];
    skew3(mBody, Sk);
    std::memset(H, 0, sizeof(H));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) H[6 * i + j] = Sk[3 * i + j];

    float HP[18];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 6; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 6; ++k) s += H[6 * i + k] * m_P[idx66(k, j)];
            HP[6 * i + j] = s;
        }
    float S33[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 6; ++k) s += HP[6 * i + k] * H[6 * j + k];
            S33[3 * i + j] = s;
            if (i == j) S33[3 * i + j] += rM;
        }
    float Sinv[9];
    if (!inv33(S33, Sinv)) return;
    float PHt[18];
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 6; ++k) s += m_P[idx66(i, k)] * H[6 * j + k];
            PHt[3 * i + j] = s;
        }
    float K63[18];
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += PHt[3 * i + k] * Sinv[3 * k + j];
            K63[3 * i + j] = s;
        }
    float dx[6];
    mul63x3(K63, innov.data(), dx);
    Vec3 dTheta{dx[0], dx[1], dx[2]};
    m_q = quatNormalize(quatMul(expSO3(dTheta), m_q));
    m_b[0] += dx[3]; m_b[1] += dx[4]; m_b[2] += dx[5];

    float KH[36];
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += K63[3 * i + k] * H[6 * k + j];
            KH[idx66(i, j)] = s;
        }
    float ImKH[36];
    identity66(ImKH);
    for (int i = 0; i < 36; ++i) ImKH[i] -= KH[i];
    float Pnew[36];
    mul66x66(ImKH, m_P, Pnew);
    std::memcpy(m_P, Pnew, sizeof(m_P));
    symmetrize66(m_P);
}

void FoxKf::updateZupt() {
    // ZUPT measurement: corrected gyro should be 0.  Measurement Jacobian
    // H = [0_{3×3} , -I_3] (gyro = ω - b ⇒ d(gyro)/d(δb) = -I).
    Vec3 innov{-m_lastGyrCorrected[0],
               -m_lastGyrCorrected[1],
               -m_lastGyrCorrected[2]};
    const float r = m_set.gyroNoiseStd * m_set.gyroNoiseStd * 0.1f;  // tight measurement noise

    // S = P[3:6,3:6] + r·I
    float S33[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            S33[3 * i + j] = m_P[idx66(3 + i, 3 + j)];
            if (i == j) S33[3 * i + j] += r;
        }
    float Sinv[9];
    if (!inv33(S33, Sinv)) return;
    // K = P[:,3:6] · S⁻¹  (6×3).
    float PHt[18];
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 3; ++j) {
            PHt[3 * i + j] = -m_P[idx66(i, 3 + j)];   // H = [0, -I], so PHᵀ = -P[:,3:6]
        }
    float K63[18];
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += PHt[3 * i + k] * Sinv[3 * k + j];
            K63[3 * i + j] = s;
        }
    float dx[6];
    mul63x3(K63, innov.data(), dx);
    Vec3 dTheta{dx[0], dx[1], dx[2]};
    m_q = quatNormalize(quatMul(expSO3(dTheta), m_q));
    m_b[0] += dx[3]; m_b[1] += dx[4]; m_b[2] += dx[5];

    // P ← (I - K H) P  where H = [0, -I] ⇒ KH = [-K_θ block | -K_b block]
    // populated only in columns 3..5.  We construct KH then I-KH.
    float KH[36];
    std::memset(KH, 0, sizeof(KH));
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 3; ++j) KH[idx66(i, 3 + j)] = -K63[3 * i + j];
    float ImKH[36];
    identity66(ImKH);
    for (int i = 0; i < 36; ++i) ImKH[i] -= KH[i];
    float Pnew[36];
    mul66x66(ImKH, m_P, Pnew);
    std::memcpy(m_P, Pnew, sizeof(m_P));
    symmetrize66(m_P);
}

float FoxKf::orientStdDeg() const {
    const float varSum = m_P[idx66(0, 0)] + m_P[idx66(1, 1)] + m_P[idx66(2, 2)];
    return std::sqrt(std::max(0.0f, varSum)) * 180.0f / 3.14159265358979f;
}

float FoxKf::biasStd() const {
    const float varSum = m_P[idx66(3, 3)] + m_P[idx66(4, 4)] + m_P[idx66(5, 5)];
    return std::sqrt(std::max(0.0f, varSum / 3.0f));
}

void FoxKf::covarianceSnapshot(float P_out[36]) const {
    std::memcpy(P_out, m_P, sizeof(m_P));
}

void FoxKf::setCovariance(const float P_in[36]) {
    std::memcpy(m_P, P_in, sizeof(m_P));
}

}  // namespace fox
