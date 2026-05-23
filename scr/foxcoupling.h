// Fox Mocap — biomechanical post-FK coupling (spec §40 / §45–§48).
//
// Implements the seven analytic coupling laws that turn the raw per-sensor
// orientation chain into a biomechanically correct pose.  All functions
// operate in place on the 23-segment world-frame orientation array; the
// numeric coefficients live in foxbody.h (kCSpine, kCArms, kCKnees,
// kCAnkles, kCToes, kCPelvis, kSpineNeck, kScapHumThetaLow/HighDeg,
// kKneeScrewMaxDeg, kAnkleDorsiLimitRad, kToeRockerLow/HighRad) and are
// the spec values verbatim.
//
// Call order (post-FK, before WLS hand-off / FK-position pass):
//     applyPelvisTilt(orient);
//     applySpineRhythm(orient);
//     applyNeckRhythm(orient);
//     applyScapuloHumeral(orient);
//     applyKneeScrewHome(orient);
//     applyAnkleCoupling(orient);
//     const auto tw = computeToeRockerWeights(orient);
//
// Each function is idempotent on its own outputs (re-applying the same law
// to an already coupled pose is a no-op), so the order above can be
// re-invoked safely if the solver shifts the parent orientations.
#pragma once

#include "foxbody.h"
#include "foxmath.h"

#include <array>

namespace fox::coupling {

// Spine rhythm — redistribute the Pelvis → T8 relative rotation across the
// six intermediate spinal joints using kCSpine[0..3] fractions.  Each
// daughter (L5, L3, T12) is placed on the great-circle arc between the
// Pelvis and T8 orientations at the partial-sum fraction; T8 itself is
// unchanged (it's a measured IMU).
void applySpineRhythm(std::array<Quat, fox::body::kSegmentCount>& orient);

// Neck / head rhythm — redistribute T8 → Head onto Neck using
// kCSpine[5..6] (the cervical pair).  Head and T8 are measured; Neck is
// the only unsensored intermediary.
void applyNeckRhythm(std::array<Quat, fox::body::kSegmentCount>& orient);

// Pelvic tilt transmission — kCPelvis[0] = 0.35 of the pelvis sagittal
// tilt is forwarded into L5/S1 (which is normally the stiffest lumbar
// joint).  Runs *before* applySpineRhythm so the spine rhythm picks up
// the slightly-larger working angle on the L5 segment.
void applyPelvisTilt(std::array<Quat, fox::body::kSegmentCount>& orient);

// Scapulo-humeral ratio — the scapula (Shoulder segment) follows the
// humerus (UpperArm segment) by a piecewise-linear fraction
// c_eff(theta_humerus) ∈ [kCArms[0]=0.95, kCArms[2]=0.99], piecewise-
// linear in [kScapHumThetaLowDeg, kScapHumThetaHighDeg] = [60°, 90°].
// Applied independently to the right and left arm chains.
void applyScapuloHumeral(std::array<Quat, fox::body::kSegmentCount>& orient);

// Knee screw-home — at deep flexion the tibia (LowerLeg) rotates outward
// about its long axis by theta_screw = kCKnees[1]·(1-cos θ_knee)·15°.
// Sign is mirrored on the left leg (external rotation is +Z right,
// −Z left in the anatomical convention).
void applyKneeScrewHome(std::array<Quat, fox::body::kSegmentCount>& orient);

// Ankle coupling — clamp plantar-/dorsi-flexion to [-kAnkleDorsiLimitRad,
// kCAnkles[1]=π/6] and apply the eversion coupling
// θ_ev_corr = c_ankles[0]·θ_ev + c_ankles[2]·sin(θ_pf).
void applyAnkleCoupling(std::array<Quat, fox::body::kSegmentCount>& orient);

// Per-foot heel ↔ toe contact weight pair derived from the toe-segment
// extension angle.  Used by ContactDetector to bias probability between
// heel (pHeel) and ball (pBall) points during the foot-roll phase of
// the gait.
struct ToeWeights {
    double w_heel_R, w_toe_R;
    double w_heel_L, w_toe_L;
};
ToeWeights computeToeRockerWeights(
    const std::array<Quat, fox::body::kSegmentCount>& orient);

// Per-frame diagnostic snapshot — populated by the coupling functions
// when -test -gloves is active (`g_testFlag` && `g_glovesFlag`).  Used
// by main.cpp's pose_solver::dumpFrameDiag to print the applied c_eff /
// θ values; reading does not require any solver state.
struct Diagnostics {
    // Spine rhythm — final fractions applied (sum of kCSpine[0..3]/Σ).
    double spineFracL5  = 0.0;
    double spineFracL3  = 0.0;
    double spineFracT12 = 0.0;
    double spineFullDeg = 0.0;          // ‖phi‖ in degrees, Pelvis→T8

    // Scapulo-humeral.
    double scapThetaRDeg = 0.0;         // humerus elevation, R
    double scapThetaLDeg = 0.0;         // humerus elevation, L
    double scapCEffR     = 0.0;         // piecewise-linear coefficient, R
    double scapCEffL     = 0.0;         // piecewise-linear coefficient, L

    // Knee screw-home.
    double kneeFlexRDeg  = 0.0;
    double kneeFlexLDeg  = 0.0;
    double kneeScrewRDeg = 0.0;
    double kneeScrewLDeg = 0.0;

    // Ankle.
    double anklePfRDeg   = 0.0;         // signed; clamp into [-45°, +30°]
    double anklePfLDeg   = 0.0;
    bool   ankleClampedR = false;
    bool   ankleClampedL = false;

    // Toe rocker.
    double toeRDeg = 0.0;
    double toeLDeg = 0.0;
    ToeWeights toeWeights{1.0, 0.0, 1.0, 0.0};
};
Diagnostics& diagnostics();     // thread_local snapshot

}  // namespace fox::coupling
