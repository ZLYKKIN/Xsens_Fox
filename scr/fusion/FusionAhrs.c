/**
 * @file FusionAhrs.c
 * @brief Spec-compliant single-sensor orientation EKF (FOX_KFA §43).
 *
 * All formulas and numeric constants in this file come from the FOX_KFA
 * specification (reverse-engineered from fox_definitions.xsb, XOR-101
 * decrypted).  Every block cites the spec section it implements.
 *
 *  Section map (within this file):
 *      §43.1  state + initial covariance
 *      §43.2  time prediction (quaternion exp-map, bias random walk)
 *      §43.3  process-noise Q diagonal
 *      §43.4A accelerometer measurement update
 *      §43.4B magnetometer measurement update (+ §51.3 3-gate)
 *      §43.4C ZRU (zero-rotation update)
 *      §43.5  Joseph-form covariance update
 *      §43.6  low-pass acceleration (LPA, τ=10s)
 *      §43.7  acceleration-divergence monitor (fAccBoost adaptive R_acc)
 *      §43.8  adaptive gyro-bias σ from mag residual
 *      §43.9  adaptive m0-learning τ_M0 (Fast 30 / Mid 120 / Slow 300 s)
 *      §43.10 hardware noise model (FOX_IMU_w2/x2/x3)
 *      §43.12 stillness detector (movement-redef threshold 0.3°/s, 5 s hold)
 *      §43.14 scenario flags — body / gloveHuman (doMag=B1, doProjMag=B1, doZru=B1)
 *      §50    Gauss-Markov skin-artifact post-filter (τ=0.15 s)
 *      §51.1  reference magnetic field m0 (norm 1, dip 78° / −67.328°)
 *      §51.3  three-gate magnetometer rejection (6° / 3.5° / 3 %)
 *
 *  Error state ordering (15 DOF):  [δθx δθy δθz, δb_gx ..z, δb_ax ..z,
 *                                   δm_x ..z, δv_x ..z]
 *  P is row-major 15×15 float.  K is row-major 15×3 float.
 */

#include "FusionAhrs.h"
#include <math.h>
#include <string.h>
#include <float.h>

// ============================================================================
//  §43 — spec constants (body / gloveHuman scenario, §43.14 overrides applied)
// ============================================================================

// §43.4A — Earth gravity used inside the filter (spec FOX_FE.gravity).
#define KFA_GRAVITY_MS2          9.812687f

// §43.10 — IMU noise model: FOX_IMU_w2 / x2.  Body Pack V2 ships this chip.
#define KFA_SIGMA_ACC_MS2        0.05f         // m/s² (acc measurement SD)
#define KFA_SIGMA_GYR_DEG_S      0.20f         // °/s  (gyro measurement SD)
#define KFA_SIGMA_MAG_NORM       0.028f        // normalised (FOX_IMU_w2 ndCoef × √BW)

// §43.1 — initial standard deviations for P₀
#define KFA_INIT_SD_ORIENT_DEG   3.0f
#define KFA_INIT_SD_GYRBIAS_DEG  0.4f          // body (gloveHuman); gloveBase = 0.3
#define KFA_INIT_SD_ACCBIAS_MS2  0.10f
#define KFA_INIT_SD_MAG_NORM     0.20f
#define KFA_INIT_SD_VEL_MS       2.0f

// §43.3 — process-noise SDs
#define KFA_S_QV_ACC_LP          0.04f         // m/s² (low-pass vel + acc-bias)
#define KFA_S_QV_MAG_RW          0.01f         // magnetic random walk
#define KFA_GYR_BIAS_MIN_DEG     0.005f        // §43.8 floor
#define KFA_GYR_BIAS_MAX_DEG     0.07f         // §43.8 ceiling
#define KFA_MAG_RES_THRESH       0.03f         // §43.8 magResThresholdGyrBiasDeg

// §43.7 — accDivMon (body uses HighBoost = 3 per §43.14 gloveHuman)
#define KFA_LPA_TAU_S            10.0f
#define KFA_ACCDIV_THRESH_LOW    0.5f          // m/s² — below ⇒ low dyn
#define KFA_ACCDIV_THRESH_HIGH   3.0f          // m/s² (body; glove = 2)
#define KFA_ACCDIV_VEL_THRESH    2.0f          // m/s — |v_lp| threshold
#define KFA_ACCDIV_HIGH_HOLD_S   0.5f          // seconds above HighThresh to disable
#define KFA_FACCBOOST_MAX        1000.0f
#define KFA_FACCBOOST_RAMP_UP_S  5.0f
#define KFA_FACCBOOST_RAMP_DN_S  60.0f
#define KFA_VEL_TAU_S            2.0f          // §43.2 tauVel for v_lp

// §51.1 / §51.3 — magnetic model + 3-gate thresholds.  Mirror of
// foxbody::kMagnet (not link-visible from a C file).
#define KFA_MAG_NORM_REF         1.0f
#define KFA_MAG_NORM_GATE        0.03f         // §51.3 normDiffFromModelMax
#define KFA_MAG_DIP_GATE_DEG     3.5f          // §51.3 dipDiffFromModelMaxDeg
#define KFA_MAG_ANG_GATE_DEG     6.0f          // §51.3 angleDiffFromModelMaxDeg

// §43.9 — adaptive τ_M0 (low / medium / high dynamics)
#define KFA_TAU_M0_FAST_S        30.0f
#define KFA_TAU_M0_MED_S         120.0f
#define KFA_TAU_M0_SLOW_S        300.0f

// §43.4C — ZRU
#define KFA_ZRU_VARIANCE         9.0f          // (fZruThreshold)² = 3² (rad/s)²
#define KFA_ZRU_MIN_SAMPLES      15
#define KFA_ZRU_UPDATE_RATE      2             // apply update every N frames once still

// §43.12 — stillness detector
#define KFA_STILLNESS_OMEGA_DEG  0.3f          // movementRedefThresholdDeg
#define KFA_STILLNESS_HOLD_S     5.0f
#define KFA_STILLNESS_ACC_BAND_MS2 0.5f        // ||a| - g| < 0.5

// §50 — skin-artifact GM post-filter
#define KFA_SKIN_TAU_S           0.15f

// ============================================================================
//  Small linear algebra (15×15) — hand-rolled, no allocation
// ============================================================================
#define N15 15

static inline float DegToRad(float d) { return d * 0.017453292519943295f; }
static inline float RadToDeg(float r) { return r * 57.29577951308232f; }

// Symmetrise: P ← (P + Pᵀ) / 2  — applied after every Joseph update.
// Also floors negative diagonals to zero (numerical hygiene).
static void Symm15(float *P) {
    for (int i = 0; i < N15; ++i)
        for (int j = i + 1; j < N15; ++j) {
            const float avg = 0.5f * (P[i * N15 + j] + P[j * N15 + i]);
            P[i * N15 + j] = avg;
            P[j * N15 + i] = avg;
        }
    for (int i = 0; i < N15; ++i)
        if (P[i * N15 + i] < 0.0f) P[i * N15 + i] = 0.0f;
}

// Joseph form for a 3-row measurement (spec §43.5):
//   P ← (I − K·H) · P · (I − K·H)ᵀ + K·R·Kᵀ
//
// Inputs (all row-major):
//   P : 15×15  (in/out)
//   H : 3×15
//   K : 15×3
//   Rdiag : 3  (R is assumed diagonal — spec §43.3, §43.4)
//
// Stack scratch ≈ 2 × 15·15 · 4 B = 1.8 KB.
static void Joseph_3x15(float *P,
                        const float *H,
                        const float *K,
                        const float Rdiag[3]) {
    float M[N15 * N15];
    memset(M, 0, sizeof(M));
    for (int i = 0; i < N15; ++i) M[i * N15 + i] = 1.0f;
    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < N15; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += K[i * 3 + k] * H[k * N15 + j];
            M[i * N15 + j] -= s;
        }

    float T[N15 * N15];
    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < N15; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N15; ++k) s += M[i * N15 + k] * P[k * N15 + j];
            T[i * N15 + j] = s;
        }

    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < N15; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N15; ++k) s += T[i * N15 + k] * M[j * N15 + k];
            P[i * N15 + j] = s;
        }

    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < N15; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += K[i * 3 + k] * Rdiag[k] * K[j * 3 + k];
            P[i * N15 + j] += s;
        }

    Symm15(P);
}

// 3×3 symmetric matrix inverse via cofactors.  Returns 0 on singular.
static int InvSym3(const float M[9], float Inv[9]) {
    const float a = M[0], b = M[1], c = M[2];
    const float d = M[4], e = M[5];
    const float f = M[8];
    const float det = a * (d * f - e * e) - b * (b * f - e * c) + c * (b * e - d * c);
    if (fabsf(det) < 1e-12f) return 0;
    const float inv = 1.0f / det;
    Inv[0] = inv * (d * f - e * e);
    Inv[1] = inv * (c * e - b * f);
    Inv[2] = inv * (b * e - c * d);
    Inv[3] = Inv[1];
    Inv[4] = inv * (a * f - c * c);
    Inv[5] = inv * (b * c - a * e);
    Inv[6] = Inv[2];
    Inv[7] = Inv[5];
    Inv[8] = inv * (a * d - b * b);
    return 1;
}

// ============================================================================
//  Quaternion helpers — exp-map, world rotation, slerp
// ============================================================================

// Spec §5.1 — exp-map: rotation vector (rad) → unit quaternion.
static FusionQuaternion ExpQuat(FusionVector phi) {
    const float n = sqrtf(phi.axis.x * phi.axis.x +
                          phi.axis.y * phi.axis.y +
                          phi.axis.z * phi.axis.z);
    FusionQuaternion q;
    if (n < 1e-9f) {
        q.element.w = 1.0f - 0.125f * n * n;
        const float s = 0.5f * (1.0f - n * n / 24.0f);
        q.element.x = s * phi.axis.x;
        q.element.y = s * phi.axis.y;
        q.element.z = s * phi.axis.z;
    } else {
        const float h = 0.5f * n;
        const float s = sinf(h) / n;
        q.element.w = cosf(h);
        q.element.x = s * phi.axis.x;
        q.element.y = s * phi.axis.y;
        q.element.z = s * phi.axis.z;
    }
    return q;
}

// v_world = q · v_sensor · q*   (spec §6.1)
static FusionVector RotByQ(FusionQuaternion q, FusionVector v) {
#define Q q.element
    const float xx = Q.x * Q.x, yy = Q.y * Q.y, zz = Q.z * Q.z;
    const float wx = Q.w * Q.x, wy = Q.w * Q.y, wz = Q.w * Q.z;
    const float xy = Q.x * Q.y, xz = Q.x * Q.z, yz = Q.y * Q.z;
    FusionVector r = {{
        (1 - 2 * (yy + zz)) * v.axis.x + 2 * (xy - wz)       * v.axis.y + 2 * (xz + wy)       * v.axis.z,
        2 * (xy + wz)       * v.axis.x + (1 - 2 * (xx + zz)) * v.axis.y + 2 * (yz - wx)       * v.axis.z,
        2 * (xz - wy)       * v.axis.x + 2 * (yz + wx)       * v.axis.y + (1 - 2 * (xx + yy)) * v.axis.z,
    }};
#undef Q
    return r;
}

static FusionVector RotByQInv(FusionQuaternion q, FusionVector v) {
    q.element.x = -q.element.x;
    q.element.y = -q.element.y;
    q.element.z = -q.element.z;
    return RotByQ(q, v);
}

// Slerp for the §50 skin post-filter.
static FusionQuaternion Slerp(FusionQuaternion a, FusionQuaternion b, float t) {
    float dot = a.element.w * b.element.w + a.element.x * b.element.x +
                a.element.y * b.element.y + a.element.z * b.element.z;
    if (dot < 0.0f) {
        b.element.w = -b.element.w;
        b.element.x = -b.element.x;
        b.element.y = -b.element.y;
        b.element.z = -b.element.z;
        dot = -dot;
    }
    if (dot > 0.9995f) {
        FusionQuaternion r = {{
            a.element.w + t * (b.element.w - a.element.w),
            a.element.x + t * (b.element.x - a.element.x),
            a.element.y + t * (b.element.y - a.element.y),
            a.element.z + t * (b.element.z - a.element.z),
        }};
        return FusionQuaternionNormalise(r);
    }
    const float theta_0 = acosf(dot);
    const float theta   = theta_0 * t;
    const float sin_t0  = sinf(theta_0);
    const float s_a     = sinf(theta_0 - theta) / sin_t0;
    const float s_b     = sinf(theta)           / sin_t0;
    FusionQuaternion r = {{
        s_a * a.element.w + s_b * b.element.w,
        s_a * a.element.x + s_b * b.element.x,
        s_a * a.element.y + s_b * b.element.y,
        s_a * a.element.z + s_b * b.element.z,
    }};
    return r;
}

// [v]× as a 3×3 row-major buffer
static void Skew3(FusionVector v, float M[9]) {
    M[0] = 0;             M[1] = -v.axis.z;    M[2] =  v.axis.y;
    M[3] = v.axis.z;      M[4] = 0;            M[5] = -v.axis.x;
    M[6] = -v.axis.y;     M[7] =  v.axis.x;    M[8] = 0;
}

// Extract a 3×3 sub-block from a row-major 15×15 matrix.
static void Sub3x3(const float *P15, int rowStart, int colStart, float out9[9]) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out9[i * 3 + j] = P15[(rowStart + i) * N15 + (colStart + j)];
}

// Apply an error-state correction δx (15 elements) to the nominal state.
static void ApplyDeltaX(FusionAhrs *ahrs, const float dx[15]) {
    // Orientation: q ← q ⊗ exp_quat(δθ)
    FusionVector dth = { .axis = { dx[0], dx[1], dx[2] } };
    ahrs->q = FusionQuaternionNormalise(FusionQuaternionProduct(ahrs->q, ExpQuat(dth)));
    // Biases / m0 / v_lp: additive
    ahrs->b_g.axis.x += dx[3];  ahrs->b_g.axis.y += dx[4];  ahrs->b_g.axis.z += dx[5];
    ahrs->b_a.axis.x += dx[6];  ahrs->b_a.axis.y += dx[7];  ahrs->b_a.axis.z += dx[8];
    ahrs->m0.axis.x  += dx[9];  ahrs->m0.axis.y  += dx[10]; ahrs->m0.axis.z  += dx[11];
    ahrs->v_lp.axis.x+= dx[12]; ahrs->v_lp.axis.y+= dx[13]; ahrs->v_lp.axis.z+= dx[14];
}

// ============================================================================
//  §43.14 — defaults (body / gloveHuman scenario)
// ============================================================================
const FusionAhrsSettings fusionAhrsDefaultSettings = {
    .convention             = FusionConventionNwu,
    .sampleRateHz           = 240.0f,
    .magDipModelDeg         = 78.0f,       // §51.6 e_dip_mag for body
    .magDeclinationDeg      = 0.0f,
    .magNormReferenceLocal  = 0.0f,        // 0 → use body baseline (1.0)
};

// ============================================================================
//  Initialisation / Restart / SetSettings
// ============================================================================
static void RebuildM0(FusionAhrs *ahrs) {
    // §51.1 — m0 = (cos(I)·cos(D), -cos(I)·sin(D), -sin(I)).  Note the sign:
    // dip > 0 means the field points downward in NWU (Z = up), so m0.z < 0.
    const float dip = DegToRad(ahrs->settings.magDipModelDeg);
    const float dec = DegToRad(ahrs->settings.magDeclinationDeg);
    const float cd  = cosf(dip);
    ahrs->m0.axis.x =  cd * cosf(dec);
    ahrs->m0.axis.y = -cd * sinf(dec);
    ahrs->m0.axis.z = -sinf(dip);
}

void FusionAhrsRestart(FusionAhrs *ahrs) {
    ahrs->q   = FUSION_QUATERNION_IDENTITY;
    ahrs->b_g = FUSION_VECTOR_ZERO;
    ahrs->b_a = FUSION_VECTOR_ZERO;
    RebuildM0(ahrs);
    ahrs->m0_avg = ahrs->m0;
    ahrs->m0_avg_ready = false;
    ahrs->v_lp = FUSION_VECTOR_ZERO;

    const float sO = DegToRad(KFA_INIT_SD_ORIENT_DEG);
    const float sG = DegToRad(KFA_INIT_SD_GYRBIAS_DEG);
    const float sA = KFA_INIT_SD_ACCBIAS_MS2;
    const float sM = KFA_INIT_SD_MAG_NORM;
    const float sV = KFA_INIT_SD_VEL_MS;
    memset(ahrs->P, 0, sizeof(ahrs->P));
    for (int i = 0; i < 3; ++i) ahrs->P[(0  + i) * N15 + (0  + i)] = sO * sO;
    for (int i = 0; i < 3; ++i) ahrs->P[(3  + i) * N15 + (3  + i)] = sG * sG;
    for (int i = 0; i < 3; ++i) ahrs->P[(6  + i) * N15 + (6  + i)] = sA * sA;
    for (int i = 0; i < 3; ++i) ahrs->P[(9  + i) * N15 + (9  + i)] = sM * sM;
    for (int i = 0; i < 3; ++i) ahrs->P[(12 + i) * N15 + (12 + i)] = sV * sV;

    ahrs->a_lp = FUSION_VECTOR_ZERO;
    ahrs->a_lp_ready = false;
    ahrs->fAccBoost = 1.0f;
    ahrs->dAccHighTime = 0.0f;
    ahrs->sBg = DegToRad(KFA_GYR_BIAS_MIN_DEG);
    ahrs->tauM0 = KFA_TAU_M0_MED_S;
    ahrs->stillnessTime = 0.0f;
    ahrs->zruSampleCount = 0;
    ahrs->zruFrameCounter = 0;
    ahrs->qSkin = FUSION_QUATERNION_IDENTITY;
    ahrs->qSkinReady = false;
    ahrs->sigmaAcc = KFA_SIGMA_ACC_MS2;
    ahrs->sigmaGyr = DegToRad(KFA_SIGMA_GYR_DEG_S);
    ahrs->sigmaMag = KFA_SIGMA_MAG_NORM;
    ahrs->dAcc = 0.0f;
    ahrs->magResidualNorm = 0.0f;
    ahrs->magGateOpen = false;
    ahrs->accUsedThisFrame = false;
    ahrs->zruActiveThisFrame = false;
}

void FusionAhrsSetSettings(FusionAhrs *ahrs, const FusionAhrsSettings *settings) {
    ahrs->settings = *settings;
    if (ahrs->settings.sampleRateHz <= 0.0f) ahrs->settings.sampleRateHz = 240.0f;
    RebuildM0(ahrs);
}

void FusionAhrsInitialise(FusionAhrs *ahrs) {
    ahrs->settings = fusionAhrsDefaultSettings;
    FusionAhrsRestart(ahrs);
}

// ============================================================================
//  §43.2 / §43.3 — time + covariance prediction
// ============================================================================
static void Predict(FusionAhrs *ahrs, FusionVector gyroDegS, float dt) {
    FusionVector omega = {{
        DegToRad(gyroDegS.axis.x) - ahrs->b_g.axis.x,
        DegToRad(gyroDegS.axis.y) - ahrs->b_g.axis.y,
        DegToRad(gyroDegS.axis.z) - ahrs->b_g.axis.z,
    }};

    // Nominal quaternion: exp-map integration
    FusionVector phi = { .axis = { omega.axis.x * dt, omega.axis.y * dt, omega.axis.z * dt } };
    ahrs->q = FusionQuaternionNormalise(FusionQuaternionProduct(ahrs->q, ExpQuat(phi)));

    // Error-state transition Φ (15×15), first-order:
    //   Φ_θθ   = I − [ω]× · dt
    //   Φ_θ,b_g = −dt · I       (orientation error grows by bias error)
    //   all other blocks: Φ = I (random-walk states)
    float Phi[N15 * N15];
    memset(Phi, 0, sizeof(Phi));
    for (int i = 0; i < N15; ++i) Phi[i * N15 + i] = 1.0f;
    float W[9]; Skew3(omega, W);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Phi[i * N15 + j] -= W[i * 3 + j] * dt;
    for (int i = 0; i < 3; ++i) Phi[i * N15 + (3 + i)] = -dt;

    // P ← Φ · P · Φᵀ
    float T[N15 * N15];
    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < N15; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N15; ++k) s += Phi[i * N15 + k] * ahrs->P[k * N15 + j];
            T[i * N15 + j] = s;
        }
    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < N15; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N15; ++k) s += T[i * N15 + k] * Phi[j * N15 + k];
            ahrs->P[i * N15 + j] = s;
        }

    // P += Q · dt  (spec §43.3 — diagonal)
    const float qO = ahrs->sigmaGyr * ahrs->sigmaGyr;
    const float qG = ahrs->sBg * ahrs->sBg;
    const float qA = KFA_S_QV_ACC_LP * KFA_S_QV_ACC_LP;
    const float qM = KFA_S_QV_MAG_RW * KFA_S_QV_MAG_RW;
    const float qV = KFA_S_QV_ACC_LP * KFA_S_QV_ACC_LP;
    for (int i = 0; i < 3; ++i) ahrs->P[(0  + i) * N15 + (0  + i)] += qO * dt * dt;
    for (int i = 0; i < 3; ++i) ahrs->P[(3  + i) * N15 + (3  + i)] += qG * dt;
    for (int i = 0; i < 3; ++i) ahrs->P[(6  + i) * N15 + (6  + i)] += qA * dt;
    for (int i = 0; i < 3; ++i) ahrs->P[(9  + i) * N15 + (9  + i)] += qM * dt;
    for (int i = 0; i < 3; ++i) ahrs->P[(12 + i) * N15 + (12 + i)] += qV * dt;

    Symm15(ahrs->P);
}

// ============================================================================
//  §43.6 — LPA filter  /  §43.7 — accDivMon (adaptive fAccBoost)
// ============================================================================
static void UpdateLPAAndDivMon(FusionAhrs *ahrs, FusionVector aSensorMs2, float dt) {
    FusionVector aCorr = {{
        aSensorMs2.axis.x - ahrs->b_a.axis.x,
        aSensorMs2.axis.y - ahrs->b_a.axis.y,
        aSensorMs2.axis.z - ahrs->b_a.axis.z,
    }};
    const float b = expf(-dt / KFA_LPA_TAU_S);
    if (!ahrs->a_lp_ready) {
        ahrs->a_lp = aCorr;
        ahrs->a_lp_ready = true;
    } else {
        ahrs->a_lp.axis.x = b * ahrs->a_lp.axis.x + (1 - b) * aCorr.axis.x;
        ahrs->a_lp.axis.y = b * ahrs->a_lp.axis.y + (1 - b) * aCorr.axis.y;
        ahrs->a_lp.axis.z = b * ahrs->a_lp.axis.z + (1 - b) * aCorr.axis.z;
    }

    // dAcc in the sensor frame: |a_corr - R⁻¹·(0,0,-g)|
    // Sensor measures specific force; at rest in NWU it reports +g along
    // the Z axis (anti-gravity).  The expected sensor-frame reading is
    // therefore R⁻¹·(0,0,+g), not (0,0,-g).
    const FusionVector gWorld  = { .axis = { 0.0f, 0.0f, +KFA_GRAVITY_MS2 } };
    const FusionVector gSensor = RotByQInv(ahrs->q, gWorld);
    const FusionVector resid = {{
        aCorr.axis.x - gSensor.axis.x,
        aCorr.axis.y - gSensor.axis.y,
        aCorr.axis.z - gSensor.axis.z,
    }};
    ahrs->dAcc = sqrtf(resid.axis.x * resid.axis.x +
                       resid.axis.y * resid.axis.y +
                       resid.axis.z * resid.axis.z);

    // World-frame v_lp — used as the dynamics gate for fAccBoost / τ_M0.
    // Linear acceleration = (specific force in world) − (specific-force
    // gravity in world).  In NWU with the specific-force convention, the
    // specific-force gravity reads (0,0,+g); subtract it from aWorld.z so
    // v_lp stays zero on a stationary sensor.
    const float bv = expf(-dt / KFA_VEL_TAU_S);
    const FusionVector aWorld = RotByQ(ahrs->q, aCorr);
    ahrs->v_lp.axis.x = bv * ahrs->v_lp.axis.x + (1 - bv) * aWorld.axis.x;
    ahrs->v_lp.axis.y = bv * ahrs->v_lp.axis.y + (1 - bv) * aWorld.axis.y;
    ahrs->v_lp.axis.z = bv * ahrs->v_lp.axis.z + (1 - bv) * (aWorld.axis.z - KFA_GRAVITY_MS2);
    const float vMag = sqrtf(ahrs->v_lp.axis.x * ahrs->v_lp.axis.x +
                             ahrs->v_lp.axis.y * ahrs->v_lp.axis.y +
                             ahrs->v_lp.axis.z * ahrs->v_lp.axis.z);

    const bool lowDyn = (ahrs->dAcc < KFA_ACCDIV_THRESH_LOW) && (vMag < KFA_ACCDIV_VEL_THRESH);
    if (lowDyn) {
        const float k = 1.0f - expf(-dt / KFA_FACCBOOST_RAMP_UP_S);
        ahrs->fAccBoost += (KFA_FACCBOOST_MAX - ahrs->fAccBoost) * k;
    } else {
        const float k = 1.0f - expf(-dt / KFA_FACCBOOST_RAMP_DN_S);
        ahrs->fAccBoost += (1.0f - ahrs->fAccBoost) * k;
    }

    if (ahrs->dAcc > KFA_ACCDIV_THRESH_HIGH) ahrs->dAccHighTime += dt;
    else                                     ahrs->dAccHighTime  = 0.0f;

    if      (lowDyn)                                            ahrs->tauM0 = KFA_TAU_M0_FAST_S;
    else if (ahrs->dAccHighTime > KFA_ACCDIV_HIGH_HOLD_S)       ahrs->tauM0 = KFA_TAU_M0_SLOW_S;
    else                                                        ahrs->tauM0 = KFA_TAU_M0_MED_S;
}

// ============================================================================
//  §43.4A — Accelerometer measurement update
//
//  Measurement model:  h_acc(x) = R⁻¹(q) · (0, 0, -g)   (gravity in sensor frame)
//  Innovation:         r = (a - b_a) - h_acc(x)
//  Jacobian H (3×15):
//      H_θ   =  [h_acc]×                       (skew of predicted gravity)
//      H_b_a = -I                              (acc-bias subtracts from meas)
//      other = 0
//  R_eff = (σ_acc² / fAccBoost) · I
//
//  Disabled when accDivMon flags the high-dynamics holdoff.
// ============================================================================
static void ApplyAccUpdate(FusionAhrs *ahrs, FusionVector aSensorMs2) {
    if (ahrs->dAccHighTime > KFA_ACCDIV_HIGH_HOLD_S) {
        ahrs->accUsedThisFrame = false;
        return;
    }

    // Sensor measures specific force; at rest in NWU it reports +g along
    // the Z axis (anti-gravity).  The expected sensor-frame reading is
    // therefore R⁻¹·(0,0,+g), not (0,0,-g).
    const FusionVector gWorld  = { .axis = { 0.0f, 0.0f, +KFA_GRAVITY_MS2 } };
    const FusionVector hAcc    = RotByQInv(ahrs->q, gWorld);
    const FusionVector aCorr   = {{
        aSensorMs2.axis.x - ahrs->b_a.axis.x,
        aSensorMs2.axis.y - ahrs->b_a.axis.y,
        aSensorMs2.axis.z - ahrs->b_a.axis.z,
    }};
    const float r[3] = {
        aCorr.axis.x - hAcc.axis.x,
        aCorr.axis.y - hAcc.axis.y,
        aCorr.axis.z - hAcc.axis.z,
    };

    // H = ∂h/∂x for h(x) = R⁻¹(q)·g_specific + b_a  (measurement model:
    // sensor reports specific force + acc bias).  Differentiating w.r.t.
    // δθ (right-perturbation of q) gives ∂R⁻¹·g/∂δθ = +[h_acc]× (sensor-
    // frame Jacobian; positive sign per the spec §43.4A linearisation).
    // ∂h/∂δb_a = +I — bias adds positively to the predicted reading,
    // so the Kalman gain corrects b_a in the same direction as r.
    float H[3 * N15];
    memset(H, 0, sizeof(H));
    float Hth[9]; Skew3(hAcc, Hth);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) H[i * N15 + (0 + j)] = Hth[i * 3 + j];
    for (int i = 0; i < 3; ++i)     H[i * N15 + (6 + i)] = +1.0f;

    // R = σ²/fAccBoost
    const float Rs = (ahrs->sigmaAcc * ahrs->sigmaAcc) / ahrs->fAccBoost;
    const float Rdiag[3] = { Rs, Rs, Rs };

    // S = H·P·Hᵀ + R   (3×3, symmetric)
    float HP[3 * N15];   // H · P  → 3×15
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < N15; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N15; ++k) s += H[i * N15 + k] * ahrs->P[k * N15 + j];
            HP[i * N15 + j] = s;
        }
    float S[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N15; ++k) s += HP[i * N15 + k] * H[j * N15 + k];
            S[i * 3 + j] = s;
        }
    S[0] += Rs;  S[4] += Rs;  S[8] += Rs;

    float Sinv[9];
    if (!InvSym3(S, Sinv)) { ahrs->accUsedThisFrame = false; return; }

    // K = P · Hᵀ · S⁻¹   (15×3)
    float PHt[N15 * 3];
    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N15; ++k) s += ahrs->P[i * N15 + k] * H[j * N15 + k];
            PHt[i * 3 + j] = s;
        }
    float K[N15 * 3];
    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += PHt[i * 3 + k] * Sinv[k * 3 + j];
            K[i * 3 + j] = s;
        }

    // δx = K · r
    float dx[N15];
    for (int i = 0; i < N15; ++i)
        dx[i] = K[i * 3 + 0] * r[0] + K[i * 3 + 1] * r[1] + K[i * 3 + 2] * r[2];
    ApplyDeltaX(ahrs, dx);

    Joseph_3x15(ahrs->P, H, K, Rdiag);
    ahrs->accUsedThisFrame = true;
}

// ============================================================================
//  §43.4B / §51.3 — Magnetometer update + 3-gate rejection
// ============================================================================
static bool MagGate(FusionAhrs *ahrs, FusionVector m) {
    const float mNorm = sqrtf(m.axis.x * m.axis.x + m.axis.y * m.axis.y + m.axis.z * m.axis.z);
    if (mNorm < 1e-6f) return false;

    // Gate 1 — norm  (§51.3 normDiffFromModelMax = 0.03)
    // Reference norm is segment-specific (§51.6 — pelvis 1.0, head 1.3,
    // hands 1.35, feet 1.22).  Falls back to the body baseline (1.0) when
    // the caller leaves magNormReferenceLocal at zero.
    const float refNorm = (ahrs->settings.magNormReferenceLocal > 1e-3f)
                              ? ahrs->settings.magNormReferenceLocal
                              : KFA_MAG_NORM_REF;
    const float normErr = fabsf(mNorm - refNorm) / refNorm;
    if (normErr > KFA_MAG_NORM_GATE) return false;

    // Gate 2 — dip.  Use the specific-force gravity convention (+Z up in
    // NWU): sensor-frame "up" = R⁻¹·(0,0,+1).  Dip = arcsin(-m̂·ĝ_up)
    // because positive dip means the field tilts DOWN.
    const FusionVector gUp = RotByQInv(ahrs->q, (FusionVector){{0, 0, +1}});
    const float gDotMUnit = (gUp.axis.x * m.axis.x +
                             gUp.axis.y * m.axis.y +
                             gUp.axis.z * m.axis.z) / mNorm;       // m̂ · ĝ_up
    const float dipMeasDeg = RadToDeg(asinf(fmaxf(-1.0f, fminf(1.0f, -gDotMUnit))));
    if (fabsf(dipMeasDeg - ahrs->settings.magDipModelDeg) > KFA_MAG_DIP_GATE_DEG) return false;

    // Gate 3 — declination angle on the horizontal plane.
    // m_perp = m − (m·ĝ_up) · ĝ_up  (§43.14 doProjectMagOnHoriPlane=B1).
    const float gDotM = gUp.axis.x * m.axis.x + gUp.axis.y * m.axis.y + gUp.axis.z * m.axis.z;
    const FusionVector mHoriz = {{
        m.axis.x - gDotM * gUp.axis.x,
        m.axis.y - gDotM * gUp.axis.y,
        m.axis.z - gDotM * gUp.axis.z,
    }};
    const FusionVector mWorld = RotByQ(ahrs->q, mHoriz);
    // NWU: X = North → declination measured east-of-north as atan2(-Y, X)
    const float angDeg = RadToDeg(atan2f(-mWorld.axis.y, mWorld.axis.x));
    if (fabsf(angDeg - ahrs->settings.magDeclinationDeg) > KFA_MAG_ANG_GATE_DEG) return false;
    return true;
}

static void ApplyMagUpdate(FusionAhrs *ahrs, FusionVector m, float dt) {
    if (m.axis.x == 0.0f && m.axis.y == 0.0f && m.axis.z == 0.0f) {
        ahrs->magGateOpen = false;
        return;
    }
    if (!MagGate(ahrs, m)) {
        ahrs->magGateOpen = false;
        return;
    }
    ahrs->magGateOpen = true;

    // Innovation: r = m - h_mag,  h_mag = q⁻¹ · m0 · q
    const FusionVector hMag = RotByQInv(ahrs->q, ahrs->m0);
    const float r[3] = {
        m.axis.x - hMag.axis.x,
        m.axis.y - hMag.axis.y,
        m.axis.z - hMag.axis.z,
    };
    ahrs->magResidualNorm = sqrtf(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);

    // H: ∂h_mag/∂δθ = [h_mag]×;  ∂h_mag/∂δm = q⁻¹ (rotation matrix Rᵀ)
    float H[3 * N15];
    memset(H, 0, sizeof(H));
    float Hth[9]; Skew3(hMag, Hth);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) H[i * N15 + (0 + j)] = Hth[i * 3 + j];
    // Rᵀ (sensor ← world): take each world basis vector, rotate to sensor frame.
    const FusionVector ex = RotByQInv(ahrs->q, (FusionVector){{1, 0, 0}});
    const FusionVector ey = RotByQInv(ahrs->q, (FusionVector){{0, 1, 0}});
    const FusionVector ez = RotByQInv(ahrs->q, (FusionVector){{0, 0, 1}});
    H[0 * N15 + 9 + 0] = ex.axis.x;  H[0 * N15 + 9 + 1] = ey.axis.x;  H[0 * N15 + 9 + 2] = ez.axis.x;
    H[1 * N15 + 9 + 0] = ex.axis.y;  H[1 * N15 + 9 + 1] = ey.axis.y;  H[1 * N15 + 9 + 2] = ez.axis.y;
    H[2 * N15 + 9 + 0] = ex.axis.z;  H[2 * N15 + 9 + 1] = ey.axis.z;  H[2 * N15 + 9 + 2] = ez.axis.z;

    const float Rs = ahrs->sigmaMag * ahrs->sigmaMag;
    const float Rdiag[3] = { Rs, Rs, Rs };

    float HP[3 * N15];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < N15; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N15; ++k) s += H[i * N15 + k] * ahrs->P[k * N15 + j];
            HP[i * N15 + j] = s;
        }
    float S[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N15; ++k) s += HP[i * N15 + k] * H[j * N15 + k];
            S[i * 3 + j] = s;
        }
    S[0] += Rs;  S[4] += Rs;  S[8] += Rs;

    float Sinv[9];
    if (!InvSym3(S, Sinv)) return;

    float PHt[N15 * 3];
    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N15; ++k) s += ahrs->P[i * N15 + k] * H[j * N15 + k];
            PHt[i * 3 + j] = s;
        }
    float K[N15 * 3];
    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += PHt[i * 3 + k] * Sinv[k * 3 + j];
            K[i * 3 + j] = s;
        }

    float dx[N15];
    for (int i = 0; i < N15; ++i)
        dx[i] = K[i * 3 + 0] * r[0] + K[i * 3 + 1] * r[1] + K[i * 3 + 2] * r[2];
    ApplyDeltaX(ahrs, dx);
    Joseph_3x15(ahrs->P, H, K, Rdiag);

    // §43.8 — adaptive σ_bg based on current mag residual
    const float ratio = ahrs->magResidualNorm / KFA_MAG_RES_THRESH;
    const float clamped = ratio > 1.0f ? 1.0f : ratio;
    const float f = 0.15f + 0.85f * clamped * clamped;
    const float sBgMax = DegToRad(KFA_GYR_BIAS_MAX_DEG);
    const float sBgMin = DegToRad(KFA_GYR_BIAS_MIN_DEG);
    const float sBg = sBgMax * f;
    ahrs->sBg = (sBg > sBgMin) ? sBg : sBgMin;

    // §43.9 — m0 random-walk learning (slow IIR towards R(q)·m_meas).
    // Only when we're confident enough to have passed the 3-gate above.
    const FusionVector mWorld = RotByQ(ahrs->q, m);
    const float a = expf(-dt / ahrs->tauM0);
    if (!ahrs->m0_avg_ready) {
        ahrs->m0_avg = mWorld;
        ahrs->m0_avg_ready = true;
    } else {
        ahrs->m0_avg.axis.x = a * ahrs->m0_avg.axis.x + (1 - a) * mWorld.axis.x;
        ahrs->m0_avg.axis.y = a * ahrs->m0_avg.axis.y + (1 - a) * mWorld.axis.y;
        ahrs->m0_avg.axis.z = a * ahrs->m0_avg.axis.z + (1 - a) * mWorld.axis.z;
    }
    // Renormalise to keep |m0| ≈ 1 (m0 represents a unit direction, not magnitude).
    const float mAvgNorm = sqrtf(ahrs->m0_avg.axis.x * ahrs->m0_avg.axis.x +
                                 ahrs->m0_avg.axis.y * ahrs->m0_avg.axis.y +
                                 ahrs->m0_avg.axis.z * ahrs->m0_avg.axis.z);
    if (mAvgNorm > 1e-6f) {
        const float inv = 1.0f / mAvgNorm;
        ahrs->m0.axis.x = ahrs->m0_avg.axis.x * inv;
        ahrs->m0.axis.y = ahrs->m0_avg.axis.y * inv;
        ahrs->m0.axis.z = ahrs->m0_avg.axis.z * inv;
    }
}

// ============================================================================
//  §43.4C / §43.12 — Stillness detector + ZRU
// ============================================================================
static void ApplyZruIfStill(FusionAhrs *ahrs, FusionVector gyroDegS, FusionVector aSensorMs2, float dt) {
    ahrs->zruActiveThisFrame = false;

    const float omegaDeg = sqrtf(
        (gyroDegS.axis.x - RadToDeg(ahrs->b_g.axis.x)) * (gyroDegS.axis.x - RadToDeg(ahrs->b_g.axis.x)) +
        (gyroDegS.axis.y - RadToDeg(ahrs->b_g.axis.y)) * (gyroDegS.axis.y - RadToDeg(ahrs->b_g.axis.y)) +
        (gyroDegS.axis.z - RadToDeg(ahrs->b_g.axis.z)) * (gyroDegS.axis.z - RadToDeg(ahrs->b_g.axis.z))
    );
    const float aNorm = sqrtf(aSensorMs2.axis.x * aSensorMs2.axis.x +
                              aSensorMs2.axis.y * aSensorMs2.axis.y +
                              aSensorMs2.axis.z * aSensorMs2.axis.z);
    const bool stillOmega = (omegaDeg < KFA_STILLNESS_OMEGA_DEG);
    const bool stillAcc   = (fabsf(aNorm - KFA_GRAVITY_MS2) < KFA_STILLNESS_ACC_BAND_MS2);

    if (stillOmega && stillAcc) {
        ahrs->stillnessTime += dt;
        ahrs->zruSampleCount = (ahrs->zruSampleCount < KFA_ZRU_MIN_SAMPLES * 2)
                               ? ahrs->zruSampleCount + 1 : ahrs->zruSampleCount;
    } else {
        ahrs->stillnessTime = 0.0f;
        ahrs->zruSampleCount = 0;
        return;
    }

    if (ahrs->stillnessTime < KFA_STILLNESS_HOLD_S) return;
    if (ahrs->zruSampleCount < KFA_ZRU_MIN_SAMPLES) return;

    // Rate-limit ZRU application
    if (++ahrs->zruFrameCounter < KFA_ZRU_UPDATE_RATE) return;
    ahrs->zruFrameCounter = 0;

    // Pseudo-measurement z = 0;  h_zru(x) = ω - b_g (radians/s).
    // r = z - h = -(ω - b_g) = b_g - ω.
    const FusionVector omegaRad = {{
        DegToRad(gyroDegS.axis.x),
        DegToRad(gyroDegS.axis.y),
        DegToRad(gyroDegS.axis.z),
    }};
    const float r[3] = {
        ahrs->b_g.axis.x - omegaRad.axis.x,
        ahrs->b_g.axis.y - omegaRad.axis.y,
        ahrs->b_g.axis.z - omegaRad.axis.z,
    };

    // H: ∂h_zru / ∂δb_g = I  (only b_g enters this measurement)
    float H[3 * N15];
    memset(H, 0, sizeof(H));
    for (int i = 0; i < 3; ++i) H[i * N15 + (3 + i)] = 1.0f;

    const float Rdiag[3] = { KFA_ZRU_VARIANCE, KFA_ZRU_VARIANCE, KFA_ZRU_VARIANCE };

    // S = H·P·Hᵀ + R = P[b_g,b_g] + R·I  (since H picks the 3×3 bias block)
    float S[9];
    Sub3x3(ahrs->P, 3, 3, S);
    S[0] += KFA_ZRU_VARIANCE;  S[4] += KFA_ZRU_VARIANCE;  S[8] += KFA_ZRU_VARIANCE;

    float Sinv[9];
    if (!InvSym3(S, Sinv)) return;

    // K = P · Hᵀ · S⁻¹.  Hᵀ picks columns 3..5 of P → P[:, 3..5].
    float PHt[N15 * 3];
    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < 3; ++j)
            PHt[i * 3 + j] = ahrs->P[i * N15 + (3 + j)];
    float K[N15 * 3];
    for (int i = 0; i < N15; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += PHt[i * 3 + k] * Sinv[k * 3 + j];
            K[i * 3 + j] = s;
        }

    float dx[N15];
    for (int i = 0; i < N15; ++i)
        dx[i] = K[i * 3 + 0] * r[0] + K[i * 3 + 1] * r[1] + K[i * 3 + 2] * r[2];
    ApplyDeltaX(ahrs, dx);
    Joseph_3x15(ahrs->P, H, K, Rdiag);

    ahrs->zruActiveThisFrame = true;
}

// ============================================================================
//  §50 — Gauss-Markov skin-artifact post-filter
//  q_skin(k+1) = slerp(q_skin(k), q_fresh, α),  α = 1 − exp(−dt/τ),  τ = 0.15 s.
//  First frame seeds the state.
// ============================================================================
static void ApplySkinPostFilter(FusionAhrs *ahrs, float dt) {
    if (!ahrs->qSkinReady) {
        ahrs->qSkin = ahrs->q;
        ahrs->qSkinReady = true;
        return;
    }
    const float alpha = 1.0f - expf(-dt / KFA_SKIN_TAU_S);
    ahrs->qSkin = Slerp(ahrs->qSkin, ahrs->q, alpha);
    ahrs->qSkin = FusionQuaternionNormalise(ahrs->qSkin);
}

// ============================================================================
//  Public API — one frame
// ============================================================================
void FusionAhrsUpdate(FusionAhrs *ahrs,
                      FusionVector gyroscope,            // °/s, sensor frame
                      FusionVector accelerometer,         // g (multiples of 9.81 m/s²)
                      FusionVector magnetometer,          // normalised, sensor frame
                      float dt) {
    if (dt <= 0.0f || dt > 1.0f) return;

    // a is in g; convert to m/s².
    const FusionVector aMs2 = {{
        accelerometer.axis.x * KFA_GRAVITY_MS2,
        accelerometer.axis.y * KFA_GRAVITY_MS2,
        accelerometer.axis.z * KFA_GRAVITY_MS2,
    }};

    // 1. §43.2 / §43.3 — prediction
    Predict(ahrs, gyroscope, dt);

    // 2. §43.6 / §43.7 — LPA + accDivMon (must run before acc update — sets R-scale)
    UpdateLPAAndDivMon(ahrs, aMs2, dt);

    // 3. §43.4A — accelerometer update (gravity-driven tilt correction)
    ApplyAccUpdate(ahrs, aMs2);

    // 4. §43.4B + §51.3 — magnetometer update (+ adaptive σ_bg in §43.8, m0 in §43.9)
    ApplyMagUpdate(ahrs, magnetometer, dt);

    // 5. §43.4C — ZRU when stillness detector fires
    ApplyZruIfStill(ahrs, gyroscope, aMs2, dt);

    // 6. §50 — Gauss-Markov skin-artifact post-filter
    ApplySkinPostFilter(ahrs, dt);
}

void FusionAhrsUpdateNoMagnetometer(FusionAhrs *ahrs,
                                    FusionVector gyroscope,
                                    FusionVector accelerometer,
                                    float dt) {
    FusionAhrsUpdate(ahrs, gyroscope, accelerometer, FUSION_VECTOR_ZERO, dt);
}

// ============================================================================
//  Getters
// ============================================================================
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *ahrs) {
    return ahrs->qSkinReady ? ahrs->qSkin : ahrs->q;
}

void FusionAhrsSetQuaternion(FusionAhrs *ahrs, FusionQuaternion q) {
    ahrs->q = FusionQuaternionNormalise(q);
    ahrs->qSkinReady = false;
}

FusionVector FusionAhrsGetGravity(const FusionAhrs *ahrs) {
    // Sensor-frame gravity unit vector — specific-force convention, so at
    // rest in upright pose this returns (0,0,+1).  Matches the same sign
    // chosen for the acc update model above.
    const FusionVector gWorld = { .axis = { 0.0f, 0.0f, +1.0f } };
    return RotByQInv(ahrs->q, gWorld);
}

FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs *ahrs) {
    // a_linear (sensor frame, g units) = a_lp / g  −  gravity_sensor
    const FusionVector g = FusionAhrsGetGravity(ahrs);
    const FusionVector r = {{
        ahrs->a_lp.axis.x / KFA_GRAVITY_MS2 - g.axis.x,
        ahrs->a_lp.axis.y / KFA_GRAVITY_MS2 - g.axis.y,
        ahrs->a_lp.axis.z / KFA_GRAVITY_MS2 - g.axis.z,
    }};
    return r;
}

FusionVector FusionAhrsGetEarthAcceleration(const FusionAhrs *ahrs) {
    return RotByQ(ahrs->q, FusionAhrsGetLinearAcceleration(ahrs));
}

FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *ahrs) {
    FusionAhrsInternalStates s;
    s.accelerationError = ahrs->dAcc;
    s.accelerometerIgnored = !ahrs->accUsedThisFrame;
    s.accelerationRecoveryTrigger = (ahrs->fAccBoost - 1.0f) / (KFA_FACCBOOST_MAX - 1.0f);
    s.magneticError = RadToDeg(ahrs->magResidualNorm);
    s.magnetometerIgnored = !ahrs->magGateOpen;
    s.magneticRecoveryTrigger = 0.0f;
    return s;
}

FusionAhrsFlags FusionAhrsGetFlags(const FusionAhrs *ahrs) {
    FusionAhrsFlags f;
    f.startup = false;
    f.angularRateRecovery = false;
    f.accelerationRecovery = !ahrs->accUsedThisFrame;
    f.magneticRecovery = !ahrs->magGateOpen;
    return f;
}

void FusionAhrsSetHeading(FusionAhrs *ahrs, float headingDeg) {
    // Replace yaw component of ahrs->q with headingDeg.  ZYX Euler yaw
    // extraction per spec §4.1.
    const float Qw = ahrs->q.element.w, Qx = ahrs->q.element.x;
    const float Qy = ahrs->q.element.y, Qz = ahrs->q.element.z;
    const float yaw = atan2f(Qw * Qz + Qx * Qy, 0.5f - Qy * Qy - Qz * Qz);
    const float half = 0.5f * (yaw - DegToRad(headingDeg));
    const FusionQuaternion rot = {{ cosf(half), 0.0f, 0.0f, -sinf(half) }};
    ahrs->q = FusionQuaternionNormalise(FusionQuaternionProduct(rot, ahrs->q));
    ahrs->qSkinReady = false;
}
