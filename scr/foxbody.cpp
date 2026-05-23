// Fox Mocap — biomechanical model implementation.
//
// Concrete data here is reproduced verbatim from the reverse-engineered
// fox_definitions.xsb (Xsens FOX_KFA Motion Engine spec, section 24 and
// appendix E).  The four-decimal precision is exactly what the spec ships.
#include "foxbody.h"

namespace fox::body {

namespace {

// Convenience: build a unit quaternion from an angle θ (radians) around an
// axis aligned with one of (X, Y, Z) — same convention as foxmath::euler_to_quat
// for a single-axis rotation (cos(θ/2), sin(θ/2)·axis).  Used to encode the
// small posture quaternions from the .xsa reference files.
constexpr double cos_half(double th) { return std::cos(0.5 * th); }
constexpr double sin_half(double th) { return std::sin(0.5 * th); }

Quat axisX(double thRad) { return Quat(cos_half(thRad), sin_half(thRad), 0, 0); }
Quat axisY(double thRad) { return Quat(cos_half(thRad), 0, sin_half(thRad), 0); }
Quat axisZ(double thRad) { return Quat(cos_half(thRad), 0, 0, sin_half(thRad)); }

constexpr double kDeg = 0.017453292519943295;   // π/180

// ---------------------------------------------------------------------------
//  Spec §24.1 — Legacy T-pose spine + legs (shared with §24.2 N-pose).
//  Numbers are the verbatim 4-decimal kratny taken from the .xsa file:
//    seg1 Pelvis = (0.9984697627340179, 0, 0.05530038793601835, 0)  → 6.34° +Y
//    seg3 L3     = (0.9987077, 0, −0.05082252,                    0)  → 5.83° −Y
//    seg4 T12    = (0.9987051, 0, −0.05087403,                    0)  → 5.83° −Y
//    seg6 Neck   = (0.9939512, 0,  0.1098229,                     0)  → 12.61° +Y
//    seg7 Head   = (0.9995685, 0,  0.02937369,                    0)  → 3.37° +Y
//    legs: seg16 RUpperLeg (…, 0.02401766,…) → 2.75° +Y;
//          seg17 RLowerLeg (…, 0.04063974,…) → 4.66° +Y
//  All other spine/leg segments in legacy = identity.
// ---------------------------------------------------------------------------
constexpr Quat kIdent = Quat(1, 0, 0, 0);

inline Quat legacySpine(int seg) {
    switch (seg) {
        case 0:  return Quat(0.9984697627340179, 0, 0.05530038793601835, 0);   // Pelvis
        case 1:  return kIdent;                                                // L5
        case 2:  return Quat(0.9987077007098614, 0, -0.05082252003612363, 0);  // L3
        case 3:  return Quat(0.9987050781810652, 0, -0.05087402888854364, 0);  // T12
        case 4:  return kIdent;                                                // T8
        case 5:  return Quat(0.9939511715005132, 0,  0.1098228968510563,  0);  // Neck
        case 6:  return Quat(0.999568500071736,  0,  0.02937369000210807, 0);  // Head
        default: return kIdent;
    }
}

inline Quat legacyLeg(int seg) {
    // §24.1: same for legacy/male/female.  Identical for left/right (16,20 and 17,21).
    switch (seg) {
        case 15:  // RUpperLeg
        case 19:  // LUpperLeg
            return Quat(0.9997115343780182, 0, 0.02401766082591783, 0);  // 2.75° +Y
        case 16:  // RLowerLeg
        case 20:  // LLowerLeg
            return Quat(0.999173864575052,  0, 0.04063973855914903, 0);  // 4.66° +Y
        case 17:  // RFoot
        case 18:  // RToe
        case 21:  // LFoot
        case 22:  // LToe
            return kIdent;
        default:
            return kIdent;
    }
}

// ---------------------------------------------------------------------------
//  Spec §24.3 — Male spine offsets (different from legacy: 9°, 8.94°, 4.6°,
//  5.07°, 4.3° around ±Y; head & neck = identity).
// ---------------------------------------------------------------------------
inline Quat maleSpine(int seg) {
    switch (seg) {
        case 0:  return Quat(0.99691733, 0,  0.0784591,   0);   // Pelvis 9.00° +Y
        case 1:  return Quat(0.99695624, 0,  0.0779632,   0);   // L5     8.94° +Y
        case 2:  return Quat(0.99919194, 0, -0.04019283,  0);   // L3     4.61° −Y
        case 3:  return Quat(0.99902228, 0, -0.04420961,  0);   // T12    5.07° −Y
        case 4:  return Quat(0.99929604, 0,  0.03751577,  0);   // T8     4.30° +Y
        case 5:  return kIdent;                                  // Neck identity
        case 6:  return kIdent;                                  // Head identity
        default: return kIdent;
    }
}

// ---------------------------------------------------------------------------
//  Spec §24.4 — Female spine offsets (greater anterior tilt: 12°, 11.25°,
//  5.80°, 6.38°, 4.45°; neck & head identity).
// ---------------------------------------------------------------------------
inline Quat femaleSpine(int seg) {
    switch (seg) {
        case 0:  return Quat(0.9945219,  0,  0.10452846,  0);   // Pelvis 12.00° +Y
        case 1:  return Quat(0.99518216, 0,  0.09804319,  0);   // L5     11.25° +Y
        case 2:  return Quat(0.99872068, 0, -0.05056679,  0);   // L3     5.80° −Y
        case 3:  return Quat(0.99845209, 0, -0.05561849,  0);   // T12    6.38° −Y
        case 4:  return Quat(0.99924607, 0,  0.03882382,  0);   // T8     4.45° +Y
        case 5:  return kIdent;
        case 6:  return kIdent;
        default: return kIdent;
    }
}

// ---------------------------------------------------------------------------
//  Spec §24.1 — Legacy T-pose: arms are identity (extended laterally).
//  Spec §24.2 — Legacy N-pose: arms are rotated ±90° around X with a small
//  ±10° shoulder offset.  Used by both legacy and male/female (§24.2 says
//  «руки в N-позе — те же ±10°/±90°»).
// ---------------------------------------------------------------------------
inline Quat armT(int seg) {
    // All arm segments in T-pose are identity (q_α = 1,0,0,0) per spec §24.1.
    (void)seg;
    return kIdent;
}

inline Quat armN(int seg) {
    // Spec §24.2:
    //   seg8  RShoulder = (cos 5°,  sin 5°, 0, 0)  → 10° around +X
    //   seg9..11 RUpperArm/Forearm/Hand = (cos 45°,  sin 45°, 0, 0)  → 90° around +X
    //   seg12 LShoulder = (cos 5°, −sin 5°, 0, 0)  → 10° around −X
    //   seg13..15 LUpperArm/Forearm/Hand = (cos 45°, −sin 45°, 0, 0) → 90° around −X
    switch (seg) {
        case 7:  return Quat(kCos5,    kSin5,   0, 0);   // RShoulder
        case 8:                                          // RUpperArm
        case 9:                                          // RForearm
        case 10: return Quat(kSqrt2H,  kSqrt2H, 0, 0);   // RHand
        case 11: return Quat(kCos5,   -kSin5,   0, 0);   // LShoulder
        case 12:                                         // LUpperArm
        case 13:                                         // LForearm
        case 14: return Quat(kSqrt2H, -kSqrt2H, 0, 0);   // LHand
        default: return kIdent;
    }
}

// Male T-pose has a subtle forearm pronation ±Z (spec §24.3):
//   seg10 RForeArm = (0.9997969880959272, 0, 0,  0.02014900976009601)  → 2.31° +Z
//   seg14 LForeArm = same with −Z
inline Quat maleArmTOverride(int seg) {
    if (seg == 9)  return Quat(0.9997969880959272, 0, 0,  0.02014900976009601);
    if (seg == 13) return Quat(0.9997969880959272, 0, 0, -0.02014900976009601);
    return armT(seg);
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
//  Public API.
// ---------------------------------------------------------------------------
Quat referenceQuat(int seg, Pose pose, Gender gender) {
    if (seg < 0 || seg >= kSegmentCount) return kIdent;

    // Spine segments 0..6 — by gender.
    if (seg <= 6) {
        switch (gender) {
            case GenderMale:   return maleSpine(seg);
            case GenderFemale: return femaleSpine(seg);
            case GenderLegacy:
            default:           return legacySpine(seg);
        }
    }
    // Arm segments 7..14 — by pose (with male T-pose pronation override).
    if (seg >= 7 && seg <= 14) {
        if (pose == PoseT) {
            return (gender == GenderMale) ? maleArmTOverride(seg) : armT(seg);
        }
        return armN(seg);  // N-pose arms are identical across genders
    }
    // Leg segments 15..22 — identical across pose/gender (spec §24.1 line «ноги одинаковы в T и N»).
    return legacyLeg(seg);
}

}  // namespace fox::body
