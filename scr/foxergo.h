// Fox Mocap — ergonomic joint-angle decomposition.
//
// Implements the jointAnglesErgo output described by the FOX_KFA spec
// (section 30):  for each of 22 body joints, compute the anatomical
// flexion / abduction / rotation triple in degrees, applying the per-joint
// sign convention encoded in fox::body::kErgoHandler (spec §30.4).
//
// Two outputs co-exist on every frame:
//   • engineering angles — Euler-ZYX of the relative quaternion parent⊗conj(child)
//     (see SkeletonXsens::computeKeypoints / existing engineering UDP stream);
//   • ergonomic angles — decomposed via fox::body::kErgoHandler[j], with
//     per-joint axis-sign flips so the numbers are clinically meaningful.
//
// World-axis convention follows spec §25.2:
//   X = forward (anterior), Y = left, Z = up (right-handed).
// Anatomical naming of the resulting triple per spec §30.3:
//   axis X  →  abduction (+) / adduction (−)     [coronal plane]
//   axis Y  →  flexion   (+) / extension (−)     [sagittal plane]
//   axis Z  →  internal (+) / external (−)       [transverse plane]
#pragma once

#include "foxbody.h"
#include "foxmath.h"

#include <array>

namespace fox::ergo {

// Anatomical angle triple (degrees).  Field order matches spec §30.3:
//   abduction (X), flexion (Y), rotation (Z).
struct JointAngles {
    double abductionDeg;
    double flexionDeg;
    double rotationDeg;
};

// Compute the relative joint quaternion q_joint = q_parent ⊗ conj(q_child),
// then dispatch to one of five extractors selected by fox::body::kErgoHandler:
//   type 0 — axial (midline)        — no L/R sign flip
//   type 1 — right-side limb        — engineering signs unchanged
//   type 2 — left-side  limb        — mirrors X and Z signs
//   type 3 — right foot specialised — variant-B Euler + foot-axis fix
//   type 4 — left  foot specialised — variant-B Euler with L mirror
//
// `qParentWorld` and `qChildWorld` are the world-frame segment orientations
// (the same quantity SkeletonXsens stores in `oriented[i]`).  No allocations.
JointAngles jointAnglesErgo(int jointIdx, const Quat& qParentWorld, const Quat& qChildWorld);

// Convenience: compute the full vector for all 22 body joints.  Input is the
// 23-segment world-orientation array; output is in joint-index order
// (matches fox::body::kJoints[i]).  This is the structured replacement for
// the existing quaternion-only UDP/CSV stream.
std::array<JointAngles, fox::body::kJointCount>
jointAnglesErgoAll(const std::array<Quat, fox::body::kSegmentCount>& segWorld);

}  // namespace fox::ergo
