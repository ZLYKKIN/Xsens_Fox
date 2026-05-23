/**
 * @file FusionAhrs.h
 * @brief Spec-compliant single-sensor orientation EKF (FOX_KFA §43).
 *
 * This is a full extended Kalman filter — not Madgwick — that fuses gyro,
 * accelerometer and magnetometer into the sensor-to-world quaternion
 * q_S→G per §43.13.  Reverse-engineered from fox_definitions.xsb and
 * implemented exclusively from the formulas / coefficients in §43, §50,
 * §51 of the FOX_KFA specification.  Numbers in the .c file all cite
 * their § section.
 *
 * Nominal state (16 doubles):
 *     q_S→G        4   sensor→world quaternion, |q|=1
 *     b_g          3   gyro bias  (sensor frame, rad/s)
 *     b_a          3   accel bias (sensor frame, m/s²)
 *     m0           3   magnetic-field model, world frame, normalised
 *     v_lp         3   low-pass linear velocity, world frame, m/s
 *
 * Error state (15 DOF):  [δθ, δb_g, δb_a, δm, δv]  with δθ a small-angle
 * tangent vector applied as q_new = q ⊗ exp_quat(δθ).
 *
 * Public API (preserved from xio so main.cpp keeps compiling):
 *     FusionAhrsInitialise / Restart / SetSettings
 *     FusionAhrsUpdate(g, a, m, dt)
 *     FusionAhrsUpdateNoMagnetometer(g, a, dt)
 *     FusionAhrsGetQuaternion       — returns skin-filtered (§50) output
 *     FusionAhrsGetLinearAcceleration / GetEarthAcceleration
 *     fusionAhrsDefaultSettings
 *
 * Everything that was «xio Madgwick» — gain, accelerationRejection in
 * degrees, magneticRejection in degrees, recoveryTriggerPeriod — is
 * gone.  Spec parameterises through noise covariances Q/R which are
 * built from the IMU model (§43.10) and the body scenario (§43.14
 * gloveHuman overrides — body = doMagnetometerUpdate=B1,
 * doProjectMagOnHoriPlane=B1, accDivMonThresholdHighBoost=3, doZru=B1).
 */

#ifndef FUSION_AHRS_H
#define FUSION_AHRS_H

#include "FusionConvention.h"
#include "FusionMath.h"
#include <stdbool.h>

// ----------------------------------------------------------------------------
//  Settings — small, immutable after init.  Numbers come from spec, not from
//  the user; sampleRateHz / magDipModelDeg / magDeclinationDeg are the only
//  three knobs because they depend on the deployment (suit rate, geography).
// ----------------------------------------------------------------------------
typedef struct {
    FusionConvention convention;        // Spec §25.2 — NWU only; ENU/NED accepted but treated as NWU
    float            sampleRateHz;      // 60 (Awinda) or 240 (Link); used to scale Q
    float            magDipModelDeg;    // §51.1 m0defDipAngleRad — +78° body / −67.328° generic
    float            magDeclinationDeg; // §51.1 m0defDeclinationAngleRad — 0° default
    // Spec §51.6 — per-segment expected magnetic-field norm.  Body is the
    // reference at 1.0; pelvis ≈ 1.0, head ≈ 1.3 (skull metal),
    // hands ≈ 1.35 (glove electronics), feet ≈ 1.22 (sole metal).  When
    // 0 (default) the legacy single-norm gate at 1.0 is used.
    float            magNormReferenceLocal;
    // Spec §51.3 + §51.6 — per-segment gate-relaxation multipliers (≥ 1).
    // Multiply into KFA_MAG_DIP_GATE_DEG and KFA_MAG_ANG_GATE_DEG so the
    // strict body thresholds (3.5° dip, 6° angle) can be loosened on
    // sensors that read a distorted field — head (4×), arms (5×), feet
    // (2×) etc.  Defaults of 0 mean "no relaxation" → strict baseline.
    float            magDipGateRelax;
    float            magAngGateRelax;
    // Spec §51.3 normDiffFromModelMax × per-seg multiplier (≥ 1) for the
    // norm gate.  Default 0 → fall back to KFA_MAG_NORM_GATE = 0.03.
    float            magNormGateRelax;
} FusionAhrsSettings;

// ----------------------------------------------------------------------------
//  Filter state.  All fields are private; main.cpp keeps it by value in
//  Impl::fusion[seg] and only ever touches it through the API below.
// ----------------------------------------------------------------------------
typedef struct {
    // §43.1 nominal state
    FusionQuaternion q;                 // q_S→G
    FusionVector     b_g;               // gyro bias  (sensor frame, rad/s)
    FusionVector     b_a;               // accel bias (sensor frame, m/s²)
    FusionVector     m0;                // magnetic model, world frame, normalised
    FusionVector     v_lp;              // world-frame low-pass velocity, m/s

    // §43.5 error-state covariance (row-major 15×15)
    float P[15 * 15];

    // §43.6 LPA filter (low-pass accelerometer, sensor frame, m/s²)
    FusionVector a_lp;
    bool         a_lp_ready;

    // §43.7 accDivMon adaptive R_acc — fAccBoost ∈ [1, 1000]
    float fAccBoost;
    float dAccHighTime;                 // accumulated seconds above threshold

    // §43.8 adaptive gyro-bias process-noise SD (radians)
    float sBg;

    // §43.9 adaptive m0-learning τ + running average (world frame)
    float        tauM0;
    FusionVector m0_avg;
    bool         m0_avg_ready;
    // §51.5 — magnetic gate up-hysteresis.  After a failure the gate stays
    // closed until the 3-condition check has passed continuously for
    // KFA_MAG_RES_TIME_UP_S = 0.6 s.  This suppresses brief field glitches
    // that would otherwise leak through the per-frame gate.
    float        magClearStreakSec;

    // §43.12 stillness detector + ZRU rate-limit
    float stillnessTime;
    int   zruSampleCount;
    int   zruFrameCounter;              // applies update every updateRateZru frames

    // §50 Gauss-Markov skin-artifact post-filter (output smoothing)
    FusionQuaternion qSkin;
    bool             qSkinReady;

    // §43.10 chosen IMU noise model (m/s² / rad/s / normalised)
    float sigmaAcc;
    float sigmaGyr;
    float sigmaMag;

    // Diagnostics (read-only from main.cpp via getters)
    float dAcc;                         // |a - b_a - R·g|, m/s²
    float magResidualNorm;              // |m - m_pred|, normalised
    bool  magGateOpen;
    bool  accUsedThisFrame;
    bool  zruActiveThisFrame;

    // Settings copy
    FusionAhrsSettings settings;
} FusionAhrs;

// ----------------------------------------------------------------------------
//  Diagnostic snapshots (returned by getter; same shape as the xio API so
//  the rare main.cpp consumer that touches them keeps compiling).
// ----------------------------------------------------------------------------
typedef struct {
    float accelerationError;            // m/s² — |a - b_a - R·g|
    bool  accelerometerIgnored;
    float accelerationRecoveryTrigger;  // 0..1 — fAccBoost normalised
    float magneticError;                // degrees — atan2 of the mag residual
    bool  magnetometerIgnored;
    float magneticRecoveryTrigger;      // unused in EKF, always 0
} FusionAhrsInternalStates;

typedef struct {
    bool startup;                       // always false in EKF (no Madgwick ramp)
    bool angularRateRecovery;           // true on first frame
    bool accelerationRecovery;          // mirror of accUsedThisFrame inversion
    bool magneticRecovery;              // mirror of magGateOpen inversion
} FusionAhrsFlags;

// ----------------------------------------------------------------------------
//  Defaults — spec §43.14 gloveHuman scenario applied (body, Link 240 Hz,
//  NWU, 78° dip).  Override per session by passing your own FusionAhrsSettings.
// ----------------------------------------------------------------------------
extern const FusionAhrsSettings fusionAhrsDefaultSettings;

// ----------------------------------------------------------------------------
//  Public API.  Same names as xio; semantics changed (EKF, not Madgwick).
// ----------------------------------------------------------------------------
void FusionAhrsInitialise(FusionAhrs *ahrs);
void FusionAhrsRestart(FusionAhrs *ahrs);
void FusionAhrsSetSettings(FusionAhrs *ahrs, const FusionAhrsSettings *settings);

void FusionAhrsUpdate(FusionAhrs *ahrs,
                      FusionVector gyroscope,            // deg/s
                      FusionVector accelerometer,         // g (multiply of 9.81 m/s²)
                      FusionVector magnetometer,          // normalised
                      float dt);
void FusionAhrsUpdateNoMagnetometer(FusionAhrs *ahrs,
                                    FusionVector gyroscope,
                                    FusionVector accelerometer,
                                    float dt);

FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *ahrs);    // skin-filtered §50
void             FusionAhrsSetQuaternion(FusionAhrs *ahrs, FusionQuaternion q);
FusionVector     FusionAhrsGetGravity(const FusionAhrs *ahrs);
FusionVector     FusionAhrsGetLinearAcceleration(const FusionAhrs *ahrs);
FusionVector     FusionAhrsGetEarthAcceleration(const FusionAhrs *ahrs);
FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *ahrs);
FusionAhrsFlags          FusionAhrsGetFlags(const FusionAhrs *ahrs);
void                     FusionAhrsSetHeading(FusionAhrs *ahrs, float headingDeg);

#endif
