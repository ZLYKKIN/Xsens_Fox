// ============================================================================
//  FoxKf — single unified XKFA-class Multiplicative Extended Kalman Filter
//  for one Awinda inertial tracker.
//
//  Architecture (per Movella XKF3i white paper + standard MEKF references):
//      State  x  = [ q (4-quat, world←body)  ;  b_g (3-gyro bias) ]
//      Error  δx = [ δθ (3-rotation vector)  ;  δb_g (3) ]
//      Cov    P  = 6×6 of δx
//
//      Predict: q ← q ⊗ Exp((ω - b_g)·dt) ;  P ← F P Fᵀ + Q
//      UpdateAcc: measurement ≈ R(q)ᵀ·[0,0,-1] g  (gravity in body frame)
//      UpdateMag: measurement ≈ R(q)ᵀ·[cosD,0,-sinD] (north + dip in body frame)
//      UpdateZupt: measurement (ω - b_g) ≈ 0 when stationary detected
//
//  Replaces (xio Fusion Madgwick + FusionBias static detector) with a single
//  filter that also exposes orientation uncertainty (orientStdDeg) consumed
//  by the per-segment drift-lock in MocapViewport::updatePose as a confidence
//  gate, instead of re-deriving it heuristically from angular velocity.
//
//  Design constraints:
//    * No Qt dependencies — pure C++ + Eigen for portability and standalone
//      unit-testing.  The receiver thread converts QVector3D ↔ std::array
//      at the boundary; internals are Eigen fixed-size types.
//    * All matrix algebra uses Eigen::Matrix<float,N,M> fixed-size types,
//      stack-allocated, no heap.  Covariance update is in Joseph form
//      (numerically stable under finite-precision arithmetic).
//    * All inputs in SI units after the existing calibration pipeline:
//      gyroscope in rad/s (caller converts from deg/s at the boundary),
//      accelerometer in g (post acc_magn normalisation), magnetometer
//      unit-norm after mag_magn + soft-iron correction.
// ============================================================================

#pragma once

#include <array>

namespace fox {

using Vec3  = std::array<float, 3>;
using Quat4 = std::array<float, 4>;       // (w, x, y, z) Hamilton

struct FoxKfSettings {
    float gyroNoiseStd       = 0.005f;     // rad/√s
    float gyroBiasRwStd      = 1.0e-5f;    // rad/s/√s
    float accNoiseStd        = 0.05f;      // g  (≈ 0.5 m/s²)
    float magNoiseStd        = 0.10f;      // unit-norm-mag
    float accRejectG         = 0.30f;      // skip update if |a|-1 > this (g)
    float magRejectUnit      = 0.40f;      // skip update if |m|-1 > this
    float magDipRad          = 1.047f;     // 60° default
    float zuptOmegaThresh    = 0.05f;      // rad/s
    float zuptAccThresh      = 0.03f;      // g
    int   zuptHoldFrames     = 30;         // ≈ 0.33 s at 90 Hz
    float initOrientStdDeg   = 5.0f;
    float initBiasStd        = 0.5f;       // rad/s — very loose until ZUPT
    float magDisableResidualDeg = 25.0f;
    int   magDisableHoldFrames  = 30;
    float magReenableResidual   = 0.15f;
    // Bias-anchor: when no ZUPT has fired for `biasAnchorFrames` ticks,
    // gently pull the live bias estimate back toward the last ZUPT snapshot
    // at `biasAnchorRate` per frame.  Previously 5400 frames (≈ 22 s at
    // 240 Hz) was too long: active limbs that rarely satisfy the strict
    // ZUPT thresholds never recover before their bias rails at
    // ±maxBiasRadPerSec, which then integrates into uncorrectable yaw drift.
    // 1800 frames (≈ 7.5 s) + rate 0.02 lets bias return to the anchor
    // within a few seconds of mild motion without disturbing well-anchored
    // sensors (rate is a fraction × dt, so a settled bias stays settled).
    float biasAnchorRate        = 0.02f;
    int   biasAnchorFrames      = 1800;
    float dthetaSanityRad       = 3.14159265f;
    float maxBiasRadPerSec      = 0.0873f;
    // Soft-stillness thresholds: a 4× looser version of the strict ZUPT
    // gate, used by isSoftStationary() as a hint that the segment is
    // approximately at rest even if it hasn't satisfied the tight 0.05
    // rad/s + 0.03 g thresholds.  Consumers (skeletal-closure / yaw-anchor
    // second layer in main.cpp) use this to gently re-align segments
    // without freezing them.
    float softOmegaThresh       = 0.20f;
    float softAccThresh         = 0.10f;
    int   softHoldFrames        = 15;
};

class FoxKf {
public:
    FoxKf();
    void initialise(const FoxKfSettings& s = FoxKfSettings{});
    const FoxKfSettings& settings() const { return m_set; }

    void setPrior(const Quat4& qWorldBody,
                  const Vec3&  biasInit,
                  float orientStdDeg = -1.0f,    // -1 ⇒ use settings.initOrientStdDeg
                  float biasStd      = -1.0f);   // -1 ⇒ use settings.initBiasStd

    void predict(const Vec3& gyrRadPerSec, float dt);

    void updateAcc(const Vec3& accUnitG);

    void updateMag(const Vec3& magUnit);

    void updateZupt();

    Quat4 orient()       const { return m_q; }
    Vec3  gyroBias()     const { return m_b; }
    float orientStdDeg() const;
    float biasStd()      const;
    bool  isStationary() const { return m_still; }
    bool  isSoftStationary() const { return m_softStill; }
    bool  magDisabled()  const { return m_magDisabled; }
    float magResidLp()   const { return m_magResidLp; }
    int   autoResetCount() const { return m_autoResetCount; }

private:
    FoxKfSettings m_set;
    Quat4 m_q{1.0f, 0.0f, 0.0f, 0.0f};       // world ← body quaternion
    Vec3  m_b{0.0f, 0.0f, 0.0f};             // gyro bias estimate (rad/s)
    float m_P[36]{};                          // 6×6 covariance — opaque
    bool  m_still           = false;
    int   m_stillTicks      = 0;
    bool  m_softStill       = false;
    int   m_softStillTicks  = 0;
    Vec3  m_lastGyrCorrected{0.0f, 0.0f, 0.0f};
    Vec3  m_lastAcc         {0.0f, 0.0f, 0.0f};
    float m_magResidLp      = 0.0f;
    int   m_magDisableTicks = 0;
    bool  m_magDisabled     = false;
    Vec3  m_bSnap           {0.0f, 0.0f, 0.0f};
    int   m_framesSinceZupt = 0;
    int   m_autoResetCount  = 0;
};

}  // namespace fox
