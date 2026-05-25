
#include "FusionAhrs.h"
#include <math.h>
#include <string.h>
#include <float.h>

#define KFA_GRAVITY_MS2          9.812687f

// formules.txt §43 (стр. 3394): FoxKF — error-state EKF ориентации одного сенсора.
// Константы ниже сверены с эталоном и совпадают: §43.10 (стр. 3681) s_acc~0.05, s_gyr~0.2°,
// s_mag~0.028 (FOX_IMU_w2); §43.1 (стр. 3414) init SD 3.0°/0.4°/0.10/0.20/2.0;
// §43.3 (стр. 3461) sQvAccLowPass=0.04, sQvMagRandomWalk=0.01; §43.8 (стр. 3600) границы
// биаса гиро 0.005°/0.07°, magResThresh=0.03, времена 0.6/3.0 с. Адаптация sBg — см. ниже §43.8.
#define KFA_SIGMA_ACC_MS2        0.05f
#define KFA_SIGMA_GYR_DEG_S      0.20f
#define KFA_SIGMA_MAG_NORM       0.028f

#define KFA_INIT_SD_ORIENT_DEG   3.0f
#define KFA_INIT_SD_GYRBIAS_DEG  0.4f
#define KFA_INIT_SD_ACCBIAS_MS2  0.10f
#define KFA_INIT_SD_MAG_NORM     0.20f
#define KFA_INIT_SD_VEL_MS       2.0f

#define KFA_S_QV_ACC_LP          0.04f
#define KFA_S_QV_MAG_RW          0.01f
#define KFA_GYR_BIAS_MIN_DEG     0.005f
#define KFA_GYR_BIAS_MAX_DEG     0.07f
#define KFA_MAG_RES_THRESH       0.03f
#define KFA_MAG_RES_TIME_UP_S    0.6f
#define KFA_MAG_RES_TIME_DOWN_S  3.0f
#define KFA_REDEF_CLOSED_THR_S   5.0f
#define KFA_REDEF_RAMP_S         2.0f
#define KFA_REDEF_SIGMA_DOWNSCALE 0.10f

#define KFA_LPA_TAU_S            10.0f
#define KFA_ACCDIV_THRESH_LOW    0.5f
#define KFA_ACCDIV_THRESH_HIGH   3.0f
#define KFA_ACCDIV_VEL_THRESH    2.0f
#define KFA_ACCDIV_HIGH_HOLD_S   0.5f
#define KFA_FACCBOOST_MAX        1000.0f
#define KFA_FACCBOOST_RAMP_UP_S  5.0f
#define KFA_FACCBOOST_RAMP_DN_S  60.0f
#define KFA_VEL_TAU_S            2.0f

#define KFA_MAG_NORM_REF         1.0f
#define KFA_MAG_NORM_GATE        0.03f
#define KFA_MAG_DIP_GATE_DEG     3.5f
#define KFA_MAG_ANG_GATE_DEG     6.0f

#define KFA_TAU_M0_FAST_S        30.0f
#define KFA_TAU_M0_MED_S         120.0f
#define KFA_TAU_M0_SLOW_S        300.0f

#define KFA_ZRU_VARIANCE         9.0f
#define KFA_ZRU_MIN_SAMPLES      15
#define KFA_ZRU_UPDATE_RATE      2

#define KFA_STILLNESS_OMEGA_DEG  0.3f
#define KFA_STILLNESS_HOLD_S     5.0f
#define KFA_STILLNESS_ACC_BAND_MS2 0.5f

#define KFA_SKIN_TAU_S           0.15f

#define KFA_P_DIAG_MIN           1.0e-12f
#define KFA_P_DIAG_MAX           1.0e+6f

#define N15 17
#define IDX_MAGNORM 15
#define IDX_SKINPHI 16

static inline float DegToRad(float d) { return d * 0.017453292519943295f; }
static inline float RadToDeg(float r) { return r * 57.29577951308232f; }

static void Symm15(float *P) {
    for (int i = 0; i < N15; ++i)
        for (int j = i + 1; j < N15; ++j) {
            const float avg = 0.5f * (P[i * N15 + j] + P[j * N15 + i]);
            P[i * N15 + j] = avg;
            P[j * N15 + i] = avg;
        }
    for (int i = 0; i < N15; ++i) {
        float d = P[i * N15 + i];
        if (!(d > KFA_P_DIAG_MIN)) d = KFA_P_DIAG_MIN;
        else if (d > KFA_P_DIAG_MAX) d = KFA_P_DIAG_MAX;
        P[i * N15 + i] = d;
    }
}

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

static void Skew3(FusionVector v, float M[9]) {
    M[0] = 0;             M[1] = -v.axis.z;    M[2] =  v.axis.y;
    M[3] = v.axis.z;      M[4] = 0;            M[5] = -v.axis.x;
    M[6] = -v.axis.y;     M[7] =  v.axis.x;    M[8] = 0;
}

static void Sub3x3(const float *P15, int rowStart, int colStart, float out9[9]) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out9[i * 3 + j] = P15[(rowStart + i) * N15 + (colStart + j)];
}

static void ApplyDeltaX(FusionAhrs *ahrs, const float dx[N15]) {

    FusionVector dth = { .axis = { dx[0], dx[1], dx[2] } };
    ahrs->q = FusionQuaternionNormalise(FusionQuaternionProduct(ahrs->q, ExpQuat(dth)));

    ahrs->b_g.axis.x += dx[3];  ahrs->b_g.axis.y += dx[4];  ahrs->b_g.axis.z += dx[5];
    ahrs->b_a.axis.x += dx[6];  ahrs->b_a.axis.y += dx[7];  ahrs->b_a.axis.z += dx[8];
    ahrs->m0.axis.x  += dx[9];  ahrs->m0.axis.y  += dx[10]; ahrs->m0.axis.z  += dx[11];
    ahrs->v_lp.axis.x+= dx[12]; ahrs->v_lp.axis.y+= dx[13]; ahrs->v_lp.axis.z+= dx[14];
    ahrs->magNormBias   += dx[IDX_MAGNORM];
    ahrs->skinPhiScalar += dx[IDX_SKINPHI];
}

const FusionAhrsSettings fusionAhrsDefaultSettings = {
    .convention             = FusionConventionNwu,
    .sampleRateHz           = 240.0f,
    .magDipModelDeg         = 78.0f,
    .magDeclinationDeg      = 0.0f,
    .magNormReferenceLocal  = 0.0f,
    .magDipGateRelax        = 0.0f,
    .magAngGateRelax        = 0.0f,
    .magNormGateRelax       = 0.0f,
};

static void RebuildM0(FusionAhrs *ahrs) {

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

    const float sN = 0.05f;
    const float sS = 3.0f * 0.017453292519943295f;
    ahrs->P[IDX_MAGNORM * N15 + IDX_MAGNORM] = sN * sN;
    ahrs->P[IDX_SKINPHI * N15 + IDX_SKINPHI] = sS * sS;
    ahrs->magNormBias   = 0.0f;
    ahrs->skinPhiScalar = 0.0f;

    ahrs->a_lp = FUSION_VECTOR_ZERO;
    ahrs->a_lp_ready = false;
    ahrs->a_lin_body  = FUSION_VECTOR_ZERO;
    ahrs->a_lin_world = FUSION_VECTOR_ZERO;
    ahrs->a_lin_ready = false;
    ahrs->fAccBoost = 1.0f;
    ahrs->dAccHighTime = 0.0f;
    ahrs->sBg = DegToRad(KFA_GYR_BIAS_MIN_DEG);
    ahrs->tauM0 = KFA_TAU_M0_MED_S;
    ahrs->magClearStreakSec = 0.0f;
    ahrs->magClosedStreakSec = 0.0f;
    ahrs->magRedefBoostTimer = 0.0f;
    ahrs->magWasClosed = false;
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

void FusionAhrsSetNoise(FusionAhrs *ahrs,
                        float sigmaAccMs2,
                        float sigmaGyrDegS,
                        float sigmaMagNorm)
{
    if (sigmaAccMs2  > 0.0f) ahrs->sigmaAcc = sigmaAccMs2;
    if (sigmaGyrDegS > 0.0f) ahrs->sigmaGyr = DegToRad(sigmaGyrDegS);
    if (sigmaMagNorm > 0.0f) ahrs->sigmaMag = sigmaMagNorm;
}

void FusionAhrsSetSampleRate(FusionAhrs *ahrs, float sampleRateHz)
{
    if (sampleRateHz > 1.0f) ahrs->settings.sampleRateHz = sampleRateHz;
}

static void Predict(FusionAhrs *ahrs, FusionVector gyroDegS, float dt) {
    FusionVector omega = {{
        DegToRad(gyroDegS.axis.x) - ahrs->b_g.axis.x,
        DegToRad(gyroDegS.axis.y) - ahrs->b_g.axis.y,
        DegToRad(gyroDegS.axis.z) - ahrs->b_g.axis.z,
    }};

    FusionVector phi = { .axis = { omega.axis.x * dt, omega.axis.y * dt, omega.axis.z * dt } };
    ahrs->q = FusionQuaternionNormalise(FusionQuaternionProduct(ahrs->q, ExpQuat(phi)));

    float Phi[N15 * N15];
    memset(Phi, 0, sizeof(Phi));
    for (int i = 0; i < N15; ++i) Phi[i * N15 + i] = 1.0f;
    float W[9]; Skew3(omega, W);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Phi[i * N15 + j] -= W[i * 3 + j] * dt;
    for (int i = 0; i < 3; ++i) Phi[i * N15 + (3 + i)] = -dt;

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

    const float fs = (ahrs->settings.sampleRateHz > 1.0f)
                      ? ahrs->settings.sampleRateHz : 240.0f;
    const float n_gyr_psd = ahrs->sigmaGyr / sqrtf(fs);
    const float qO = n_gyr_psd * n_gyr_psd;
    const float qG = ahrs->sBg * ahrs->sBg;
    const float qA = KFA_S_QV_ACC_LP * KFA_S_QV_ACC_LP;
    const float qM = KFA_S_QV_MAG_RW * KFA_S_QV_MAG_RW;
    const float qV = KFA_S_QV_ACC_LP * KFA_S_QV_ACC_LP;
    for (int i = 0; i < 3; ++i) ahrs->P[(0  + i) * N15 + (0  + i)] += qO * dt;
    for (int i = 0; i < 3; ++i) ahrs->P[(3  + i) * N15 + (3  + i)] += qG * dt;
    for (int i = 0; i < 3; ++i) ahrs->P[(6  + i) * N15 + (6  + i)] += qA * dt;
    for (int i = 0; i < 3; ++i) ahrs->P[(9  + i) * N15 + (9  + i)] += qM * dt;
    for (int i = 0; i < 3; ++i) ahrs->P[(12 + i) * N15 + (12 + i)] += qV * dt;

    const float qN = 1.0e-4f * 1.0e-4f;
    ahrs->P[IDX_MAGNORM * N15 + IDX_MAGNORM] += qN * dt;

    const float skinSigma = 3.0f * 0.017453292519943295f;
    const float skinTau   = KFA_SKIN_TAU_S;
    const float skinDecay = expf(-2.0f * dt / skinTau);
    ahrs->P[IDX_SKINPHI * N15 + IDX_SKINPHI] *= skinDecay;
    ahrs->skinPhiScalar  *= expf(-dt / skinTau);
    const float qS = (skinSigma * skinSigma) * (1.0f - skinDecay);
    ahrs->P[IDX_SKINPHI * N15 + IDX_SKINPHI] += qS;

    Symm15(ahrs->P);
}

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

    ahrs->a_lin_body  = resid;
    ahrs->a_lin_world = RotByQ(ahrs->q, resid);
    ahrs->a_lin_ready = true;

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

static void ApplyAccUpdate(FusionAhrs *ahrs, FusionVector aSensorMs2) {
    if (ahrs->dAccHighTime > KFA_ACCDIV_HIGH_HOLD_S) {
        ahrs->accUsedThisFrame = false;
        return;
    }

    const FusionVector gWorld  = { .axis = { 0.0f, 0.0f, +KFA_GRAVITY_MS2 } };
    FusionVector hAcc    = RotByQInv(ahrs->q, gWorld);
    const float skinAng = ahrs->skinPhiScalar;
    if (fabsf(skinAng) > 1e-6f) {
        const float c = cosf(skinAng);
        const float s = sinf(skinAng);
        const float gx = hAcc.axis.x;
        const float gy = hAcc.axis.y;
        const float invXy = 1.0f / sqrtf(gx * gx + gy * gy + 1e-12f);
        const float ax = -gy * invXy;
        const float ay = +gx * invXy;
        const float az = 0.0f;
        const FusionVector hRot = {{
            (c + ax * ax * (1.0f - c))             * hAcc.axis.x
              + (ax * ay * (1.0f - c) - az * s)    * hAcc.axis.y
              + (ax * az * (1.0f - c) + ay * s)    * hAcc.axis.z,
            (ay * ax * (1.0f - c) + az * s)        * hAcc.axis.x
              + (c + ay * ay * (1.0f - c))         * hAcc.axis.y
              + (ay * az * (1.0f - c) - ax * s)    * hAcc.axis.z,
            (az * ax * (1.0f - c) - ay * s)        * hAcc.axis.x
              + (az * ay * (1.0f - c) + ax * s)    * hAcc.axis.y
              + (c + az * az * (1.0f - c))         * hAcc.axis.z,
        }};
        hAcc = hRot;
    }
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

    float H[3 * N15];
    memset(H, 0, sizeof(H));
    float Hth[9]; Skew3(hAcc, Hth);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) H[i * N15 + (0 + j)] = Hth[i * 3 + j];
    for (int i = 0; i < 3; ++i)     H[i * N15 + (6 + i)] = +1.0f;

    const float rNorm = sqrtf(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    if (rNorm > 1e-6f) {
        const float invN = 1.0f / rNorm;
        H[0 * N15 + IDX_SKINPHI] = -r[0] * invN;
        H[1 * N15 + IDX_SKINPHI] = -r[1] * invN;
        H[2 * N15 + IDX_SKINPHI] = -r[2] * invN;
    }

    const float Rs = (ahrs->sigmaAcc * ahrs->sigmaAcc) / ahrs->fAccBoost;
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
    if (!InvSym3(S, Sinv)) { ahrs->accUsedThisFrame = false; return; }

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
    ahrs->accUsedThisFrame = true;
}

static bool MagGate(FusionAhrs *ahrs, FusionVector m) {
    const float mNorm = sqrtf(m.axis.x * m.axis.x + m.axis.y * m.axis.y + m.axis.z * m.axis.z);
    if (mNorm < 1e-6f) return false;

    const float dipRelax  = (ahrs->settings.magDipGateRelax  > 0.0f)
                                ? ahrs->settings.magDipGateRelax  : 1.0f;
    const float angRelax  = (ahrs->settings.magAngGateRelax  > 0.0f)
                                ? ahrs->settings.magAngGateRelax  : 1.0f;
    const float normRelax = (ahrs->settings.magNormGateRelax > 0.0f)
                                ? ahrs->settings.magNormGateRelax : 1.0f;

    const float refNorm = (ahrs->settings.magNormReferenceLocal > 1e-3f)
                              ? ahrs->settings.magNormReferenceLocal
                              : KFA_MAG_NORM_REF;
    const float normErr = fabsf(mNorm - refNorm) / refNorm;
    if (normErr > KFA_MAG_NORM_GATE * normRelax) return false;

    const FusionVector gUp = RotByQInv(ahrs->q, (FusionVector){{0, 0, +1}});
    const float gDotMUnit = (gUp.axis.x * m.axis.x +
                             gUp.axis.y * m.axis.y +
                             gUp.axis.z * m.axis.z) / mNorm;
    const float dipMeasDeg = RadToDeg(asinf(fmaxf(-1.0f, fminf(1.0f, -gDotMUnit))));
    if (fabsf(dipMeasDeg - ahrs->settings.magDipModelDeg)
            > KFA_MAG_DIP_GATE_DEG * dipRelax) return false;

    const float gDotM = gUp.axis.x * m.axis.x + gUp.axis.y * m.axis.y + gUp.axis.z * m.axis.z;
    const FusionVector mHoriz = {{
        m.axis.x - gDotM * gUp.axis.x,
        m.axis.y - gDotM * gUp.axis.y,
        m.axis.z - gDotM * gUp.axis.z,
    }};
    const FusionVector mWorld = RotByQ(ahrs->q, mHoriz);

    const float angDeg = RadToDeg(atan2f(-mWorld.axis.y, mWorld.axis.x));
    if (fabsf(angDeg - ahrs->settings.magDeclinationDeg)
            > KFA_MAG_ANG_GATE_DEG * angRelax) return false;
    return true;
}

static void ApplyMagUpdate(FusionAhrs *ahrs, FusionVector m, float dt) {

    if (m.axis.x == 0.0f && m.axis.y == 0.0f && m.axis.z == 0.0f) {
        ahrs->magGateOpen = false;
        ahrs->magClearStreakSec = 0.0f;
        ahrs->magClosedStreakSec += dt;
        ahrs->magWasClosed = true;
        return;
    }
    if (!MagGate(ahrs, m)) {
        ahrs->magGateOpen = false;
        ahrs->magClearStreakSec = 0.0f;
        ahrs->magClosedStreakSec += dt;
        ahrs->magWasClosed = true;
        return;
    }

    ahrs->magClearStreakSec += dt;
    if (ahrs->magClearStreakSec < KFA_MAG_RES_TIME_UP_S) {
        ahrs->magGateOpen = false;

        return;
    }

    if (ahrs->magWasClosed &&
        ahrs->magClosedStreakSec >= KFA_REDEF_CLOSED_THR_S) {
        ahrs->magRedefBoostTimer = KFA_REDEF_RAMP_S;
    }
    ahrs->magClosedStreakSec = 0.0f;
    ahrs->magWasClosed = false;
    ahrs->magGateOpen = true;

    const float magNormScale = 1.0f + ahrs->magNormBias;
    FusionVector hMag = RotByQInv(ahrs->q, ahrs->m0);
    hMag.axis.x *= magNormScale;
    hMag.axis.y *= magNormScale;
    hMag.axis.z *= magNormScale;
    const float r[3] = {
        m.axis.x - hMag.axis.x,
        m.axis.y - hMag.axis.y,
        m.axis.z - hMag.axis.z,
    };
    ahrs->magResidualNorm = sqrtf(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);

    float H[3 * N15];
    memset(H, 0, sizeof(H));
    float Hth[9]; Skew3(hMag, Hth);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) H[i * N15 + (0 + j)] = Hth[i * 3 + j];

    const FusionVector ex = RotByQInv(ahrs->q, (FusionVector){{1, 0, 0}});
    const FusionVector ey = RotByQInv(ahrs->q, (FusionVector){{0, 1, 0}});
    const FusionVector ez = RotByQInv(ahrs->q, (FusionVector){{0, 0, 1}});
    H[0 * N15 + 9 + 0] = ex.axis.x;  H[0 * N15 + 9 + 1] = ey.axis.x;  H[0 * N15 + 9 + 2] = ez.axis.x;
    H[1 * N15 + 9 + 0] = ex.axis.y;  H[1 * N15 + 9 + 1] = ey.axis.y;  H[1 * N15 + 9 + 2] = ez.axis.y;
    H[2 * N15 + 9 + 0] = ex.axis.z;  H[2 * N15 + 9 + 1] = ey.axis.z;  H[2 * N15 + 9 + 2] = ez.axis.z;

    H[0 * N15 + IDX_MAGNORM] = hMag.axis.x;
    H[1 * N15 + IDX_MAGNORM] = hMag.axis.y;
    H[2 * N15 + IDX_MAGNORM] = hMag.axis.z;

    float sigmaMagEff = ahrs->sigmaMag;
    if (ahrs->magRedefBoostTimer > 0.0f) {
        const float frac = ahrs->magRedefBoostTimer / KFA_REDEF_RAMP_S;

        const float boostMul = KFA_REDEF_SIGMA_DOWNSCALE
                             + (1.0f - KFA_REDEF_SIGMA_DOWNSCALE) * (1.0f - frac);
        sigmaMagEff = ahrs->sigmaMag * boostMul;
        ahrs->magRedefBoostTimer -= dt;
        if (ahrs->magRedefBoostTimer < 0.0f) ahrs->magRedefBoostTimer = 0.0f;
    }
    const float Rs = sigmaMagEff * sigmaMagEff;
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

    // formules.txt §43.8 (стр. 3600): адаптивный s_bg от магнит-резидуала.
    // f = 0.15 + 0.85·min((r_mag/0.03)²,1); s_bg = max(min, max·f). Совпадает точно.
    const float ratio = ahrs->magResidualNorm / KFA_MAG_RES_THRESH;
    const float clamped = ratio > 1.0f ? 1.0f : ratio;
    const float f = 0.15f + 0.85f * clamped * clamped;
    const float sBgMax = DegToRad(KFA_GYR_BIAS_MAX_DEG);
    const float sBgMin = DegToRad(KFA_GYR_BIAS_MIN_DEG);
    const float sBg = sBgMax * f;
    ahrs->sBg = (sBg > sBgMin) ? sBg : sBgMin;

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

    if (++ahrs->zruFrameCounter < KFA_ZRU_UPDATE_RATE) return;
    ahrs->zruFrameCounter = 0;

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

    float H[3 * N15];
    memset(H, 0, sizeof(H));
    for (int i = 0; i < 3; ++i) H[i * N15 + (3 + i)] = 1.0f;

    const float Rdiag[3] = { KFA_ZRU_VARIANCE, KFA_ZRU_VARIANCE, KFA_ZRU_VARIANCE };

    float S[9];
    Sub3x3(ahrs->P, 3, 3, S);
    S[0] += KFA_ZRU_VARIANCE;  S[4] += KFA_ZRU_VARIANCE;  S[8] += KFA_ZRU_VARIANCE;

    float Sinv[9];
    if (!InvSym3(S, Sinv)) return;

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

void FusionAhrsUpdate(FusionAhrs *ahrs,
                      FusionVector gyroscope,
                      FusionVector accelerometer,
                      FusionVector magnetometer,
                      float dt) {
    if (dt <= 0.0f || dt > 1.0f) return;

    const FusionVector aMs2 = {{
        accelerometer.axis.x * KFA_GRAVITY_MS2,
        accelerometer.axis.y * KFA_GRAVITY_MS2,
        accelerometer.axis.z * KFA_GRAVITY_MS2,
    }};

    Predict(ahrs, gyroscope, dt);

    UpdateLPAAndDivMon(ahrs, aMs2, dt);

    ApplyAccUpdate(ahrs, aMs2);

    ApplyMagUpdate(ahrs, magnetometer, dt);

    ApplyZruIfStill(ahrs, gyroscope, aMs2, dt);

    ApplySkinPostFilter(ahrs, dt);
}

void FusionAhrsUpdateNoMagnetometer(FusionAhrs *ahrs,
                                    FusionVector gyroscope,
                                    FusionVector accelerometer,
                                    float dt) {
    FusionAhrsUpdate(ahrs, gyroscope, accelerometer, FUSION_VECTOR_ZERO, dt);
}

FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *ahrs) {
    return ahrs->qSkinReady ? ahrs->qSkin : ahrs->q;
}

void FusionAhrsSetQuaternion(FusionAhrs *ahrs, FusionQuaternion q) {
    ahrs->q = FusionQuaternionNormalise(q);
    ahrs->qSkinReady = false;
}

FusionVector FusionAhrsGetGravity(const FusionAhrs *ahrs) {

    const FusionVector gWorld = { .axis = { 0.0f, 0.0f, +1.0f } };
    return RotByQInv(ahrs->q, gWorld);
}

FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs *ahrs) {

    if (!ahrs->a_lin_ready) return FUSION_VECTOR_ZERO;
    const FusionVector r = {{
        ahrs->a_lin_body.axis.x / KFA_GRAVITY_MS2,
        ahrs->a_lin_body.axis.y / KFA_GRAVITY_MS2,
        ahrs->a_lin_body.axis.z / KFA_GRAVITY_MS2,
    }};
    return r;
}

FusionVector FusionAhrsGetEarthAcceleration(const FusionAhrs *ahrs) {
    if (!ahrs->a_lin_ready) return FUSION_VECTOR_ZERO;
    const FusionVector r = {{
        ahrs->a_lin_world.axis.x / KFA_GRAVITY_MS2,
        ahrs->a_lin_world.axis.y / KFA_GRAVITY_MS2,
        ahrs->a_lin_world.axis.z / KFA_GRAVITY_MS2,
    }};
    return r;
}

FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *ahrs) {
    FusionAhrsInternalStates s;
    s.accelerationError = ahrs->dAcc;
    s.accelerometerIgnored = !ahrs->accUsedThisFrame;
    s.accelerationRecoveryTrigger = (ahrs->fAccBoost - 1.0f) / (KFA_FACCBOOST_MAX - 1.0f);
    s.magneticError = RadToDeg(ahrs->magResidualNorm);
    s.magnetometerIgnored = !ahrs->magGateOpen;
    s.magneticRecoveryTrigger = 0.0f;
    s.magNormBias = ahrs->magNormBias;
    s.skinPhiDeg  = RadToDeg(ahrs->skinPhiScalar);
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

    const float Qw = ahrs->q.element.w, Qx = ahrs->q.element.x;
    const float Qy = ahrs->q.element.y, Qz = ahrs->q.element.z;
    const float yaw = atan2f(Qw * Qz + Qx * Qy, 0.5f - Qy * Qy - Qz * Qz);
    const float half = 0.5f * (yaw - DegToRad(headingDeg));
    const FusionQuaternion rot = {{ cosf(half), 0.0f, 0.0f, -sinf(half) }};
    ahrs->q = FusionQuaternionNormalise(FusionQuaternionProduct(rot, ahrs->q));
    ahrs->qSkinReady = false;
}
