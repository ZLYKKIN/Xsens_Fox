// Fox Mocap — biomechanical joint-coupling post-filter (spec §40 / §45–§48).
//
// In the original FOX_KFA engine, the joint-coupling weights c_spine / c_arms /
// c_knees / c_ankles / c_toes enter the weighted-LSM solver as Jacobian rows.
// In this reduced-scope build (Level 1 of the migration plan, no WLS solver)
// we apply the coupling deterministically as a redistribution of the parent→
// child rotation across an anatomical chain, using log-map weighted blending:
//
//     φ_total = log( q_parent ⊗ conj(q_child) )            (spec §5.2, §11.3)
//     w_j     = c_j / Σ c                                  (normalised weights)
//     q_j     = exp( w_j · φ_total )                       (spec §5.1, §40)
//
// This is mathematically equivalent to the WLS minimum when all hint weights
// are equal (gradient zero ↔ weight-proportional partition of the total
// rotation).  It is NOT a Kalman-style update — there is no covariance
// tracking — but it captures the anatomical realism the c_* coefficients
// encode (spine rhythm, scapulo-humeral ratio, knee screw-home, ankle/toe
// rocker) without requiring the sparse linear-algebra stack.
//
// Conventions (spec §25.2):
//   X = forward, Y = left, Z = up; quaternions WXYZ scalar-first;
//   all functions operate on segment-world quaternions (oriented[i] in the
//   existing SkeletonXsens pipeline).
#pragma once

#include "foxbody.h"
#include "foxmath.h"

#include <array>

namespace fox::coupling {

// Spec §45 — spine rhythm.  Inputs are world-frame Pelvis and T8
// orientations.  Output is the redistributed world-frame orientations for
// the four intermediate spinal segments L5, L3, T12 and T8 itself (T8 is
// reconstructed from the chain composition and may differ slightly from
// the input qT8 when the redistribution doesn't sum to the total — which
// is why we normalise weights to keep that error at zero).
//
// out[0..3] = world orientations of L5, L3, T12, T8 (in that order).
// The neck and head (joints jT1C7 / jC1Head) keep their input orientations
// — they're modelled separately as «free» (weight 0.9 each), and the
// spine block redistributes only the trunk total.
void applySpineRhythm(const Quat& qPelvisWorld,
                      const Quat& qT8World,
                      std::array<Quat, 4>& outL5_L3_T12_T8);

// Spec §46 — scapulo-humeral ratio.  When the humerus (UpperArm) is
// elevated relative to T8, the scapula (Shoulder) follows along with
// coefficient kCArms.  Inputs: T8 and UpperArm in world frame plus the raw
// Shoulder orientation (which we override).  Output: corrected Shoulder.
//
// The mirror happens automatically: feeding R-UpperArm/R-Shoulder yields
// right-side coupling; feeding L-UpperArm/L-Shoulder yields left-side.
Quat applyScapuloHumeral(const Quat& qT8World,
                         const Quat& qShoulderRawWorld,
                         const Quat& qUpperArmWorld);

// Spec §47 — knee «screw-home» mechanism.  When the knee approaches full
// extension (flexion < ~20°), the tibia obligatorily rotates externally
// (right leg) / internally (left leg) by a small coupled angle (~5°).
// Input: thigh and shank in world frame.  Output: corrected shank.
Quat applyKneeScrew(const Quat& qUpperLegWorld,
                    const Quat& qLowerLegRawWorld,
                    bool leftSide);

// Spec §48 — ankle plantar-flexion limit + toe rocker.  Applies the kCAnkles
// limit (30° plantar) as a soft clamp and propagates a small toe lift when
// the foot is plantar-flexed (kCToes[5] = sin5° threshold).
//
// Inputs: shank, foot, toe (raw) in world frame; outputs: corrected foot,
// corrected toe.
void applyAnkleToe(const Quat& qLowerLegWorld,
                   const Quat& qFootRawWorld,
                   const Quat& qToeRawWorld,
                   Quat& qFootOut,
                   Quat& qToeOut);

}  // namespace fox::coupling
