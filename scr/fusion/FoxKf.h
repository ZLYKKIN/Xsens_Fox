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

// 3-vector & 4-quat aliases used at the FoxKf boundary.  The receiver
// converts between these and QVector3D / Quat outside the filter.
using Vec3  = std::array<float, 3>;
using Quat4 = std::array<float, 4>;       // (w, x, y, z) Hamilton

struct FoxKfSettings {
    // Process noise.  Source: Awinda BNO055/MTw datasheet typical
    // gyro spectral density 0.005-0.010 rad/√s, gyro RW (bias instability)
    // ~1e-5 rad/s/√s at the 100 Hz rate.  Conservative defaults below.
    float gyroNoiseStd       = 0.005f;     // rad/√s
    float gyroBiasRwStd      = 1.0e-5f;    // rad/s/√s
    // Measurement noise.
    float accNoiseStd        = 0.05f;      // g  (≈ 0.5 m/s²)
    float magNoiseStd        = 0.10f;      // unit-norm-mag
    // Rejection gates.  Adaptive inflation keeps the filter open on shocks
    // (acc) and structural mag disturbances rather than freezing (xio Fusion's
    // approach was a hard cutout via recoveryTriggerPeriod; XKF3i instead
    // tracks disturbance and slowly converges).
    float accRejectG         = 0.30f;      // skip update if |a|-1 > this (g)
    float magRejectUnit      = 0.40f;      // skip update if |m|-1 > this
    // Magnetic dip — average for European/Russian latitudes ~ 60° down.
    // Caller MAY override per-region; the filter uses cos(dip)/sin(dip)
    // to compute the body-frame magnetic reference.
    float magDipRad          = 1.047f;     // 60° default
    // Stationarity gate (ZUPT).  Detect when (ω - b_g) and (a - 1g) are
    // both small for `zuptHoldFrames`.
    float zuptOmegaThresh    = 0.05f;      // rad/s
    float zuptAccThresh      = 0.03f;      // g
    int   zuptHoldFrames     = 30;         // ≈ 0.33 s at 90 Hz
    // Initial covariance scale when a prior is set.
    float initOrientStdDeg   = 5.0f;
    float initBiasStd        = 0.5f;       // rad/s — very loose until ZUPT
};

class FoxKf {
public:
    FoxKf();
    void initialise(const FoxKfSettings& s = FoxKfSettings{});
    const FoxKfSettings& settings() const { return m_set; }

    // Reset orientation and bias to a known prior and inflate covariance.
    // Called once per sensor after T-N-K calibration finishes (the wizard
    // produces a world-frame reference quat and a gyro-bias estimate; the
    // filter takes those as its starting point and refines online).
    void setPrior(const Quat4& qWorldBody,
                  const Vec3&  biasInit,
                  float orientStdDeg = -1.0f,    // -1 ⇒ use settings.initOrientStdDeg
                  float biasStd      = -1.0f);   // -1 ⇒ use settings.initBiasStd

    // Predict: integrate gyro for dt seconds, propagate covariance.
    // gyrRadPerSec is in the SENSOR / segment body frame after the
    // calibration s2s rotation (the caller's existing pipeline already
    // applies inv(s2s) before invoking the filter).
    void predict(const Vec3& gyrRadPerSec, float dt);

    // Update with accelerometer.  Expected ≈ unit-norm direction of gravity
    // in body frame.  Filter ignores the update when |accUnitG| is far from
    // 1.0 (impact / linear acc).  Pass post-calibration acc / |acc_magn|.
    void updateAcc(const Vec3& accUnitG);

    // Update with magnetometer.  Expected ≈ unit-norm magnetic field in body
    // frame.  Filter rejects when the norm is far from 1 (structural
    // disturbance / hard iron).  Pass post-calibration mag / |mag_magn|
    // already corrected for soft-iron.
    void updateMag(const Vec3& magUnit);

    // Update when actor is detected stationary: forces (ω - b_g) ≈ 0.
    // Caller-driven gate (e.g. through LocomotionSolver) is fine too;
    // the filter also tracks its own gate via m_stillTicks below.  We
    // expose this as an explicit method so external knowledge (foot
    // contact / sit pose) can pin ZUPT firmly.
    void updateZupt();

    Quat4 orient()       const { return m_q; }
    Vec3  gyroBias()     const { return m_b; }
    // Orientation 1-sigma uncertainty in degrees (sqrt of the trace of the
    // δθ-block of P, converted to deg).  Used by MocapViewport's drift-lock
    // as a confidence gate so a still-converging filter is never frozen.
    float orientStdDeg() const;
    float biasStd()      const;
    bool  isStationary() const { return m_still; }

private:
    FoxKfSettings m_set;
    Quat4 m_q{1.0f, 0.0f, 0.0f, 0.0f};       // world ← body quaternion
    Vec3  m_b{0.0f, 0.0f, 0.0f};             // gyro bias estimate (rad/s)
    float m_P[36]{};                          // 6×6 covariance, row-major
                                              // (kept as flat float[36] so
                                              // the header has no Eigen
                                              // dependency — the .cpp
                                              // re-views it as Eigen::Matrix
                                              // for all math).
    bool  m_still           = false;
    int   m_stillTicks      = 0;
    Vec3  m_lastGyrCorrected{0.0f, 0.0f, 0.0f};
    Vec3  m_lastAcc         {0.0f, 0.0f, 0.0f};
};

}  // namespace fox
