// Fox Mocap — ergonomic joint-angle implementation (spec §30).
#include "foxergo.h"

namespace fox::ergo {

using fox::Matrix3;
using fox::Euler3;

namespace {

constexpr double kRad2Deg = 57.29577951308232;   // spec Appendix A

// Engineering Euler-ZYX from the spec §4.1 quaternion formula.  Result in
// radians; matches fox_types_engine.dll exactly (see spec §34.3 for the sign
// convention — pitch carries the −180/π factor).
inline Euler3 eulerZYX(const Quat& q)
{
    Euler3 e;
    // roll(X) = atan2(2(w·x + y·z), 1 − 2(x² + y²))
    e.e0 = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    // pitch(Y) = asin(clamp(2(w·y − z·x)))  — spec §4.1 sign is asin(2(zx−wy))·(−1)
    e.e1 = fox::clamp_asin(2.0 * (q.w * q.y - q.z * q.x));
    // yaw(Z) = atan2(2(w·z + x·y), 1 − 2(y² + z²))
    e.e2 = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return e;
}

// Type 0 — axial / midline.  No left/right sign flip; the angle vector goes
// straight through with anatomical-axis remapping per spec §25.2:
//     abduction = roll(X), flexion = pitch(Y), rotation = yaw(Z).
JointAngles handlerAxial(const Quat& qRel)
{
    const Euler3 e = eulerZYX(qRel);
    return { e.e0 * kRad2Deg, e.e1 * kRad2Deg, e.e2 * kRad2Deg };
}

// Type 1 — right-side limb (engineering signs unchanged for the right side).
JointAngles handlerRight(const Quat& qRel)
{
    const Euler3 e = eulerZYX(qRel);
    return { e.e0 * kRad2Deg, e.e1 * kRad2Deg, e.e2 * kRad2Deg };
}

// Type 2 — left-side limb.  Spec §30.4 implements the L/R mirror as separate
// handlers with abduction (X) and rotation (Z) sign-flipped so left-hip
// abduction is positive in the same direction as right-hip abduction
// (clinical convention).  Flexion (Y) keeps its sign.
JointAngles handlerLeft(const Quat& qRel)
{
    const Euler3 e = eulerZYX(qRel);
    return { -e.e0 * kRad2Deg,  e.e1 * kRad2Deg, -e.e2 * kRad2Deg };
}

// Types 3 / 4 — foot-specialised.  Spec §30.4 notes a different Euler order
// (variant B from §4.3) so the ankle's tilt component sits on the middle
// axis.  We extract via R(q) → matrix-Euler-B and remap to (abd, flex, rot).
JointAngles handlerFoot(const Quat& qRel, bool leftSide)
{
    const Matrix3 R = fox::quat_to_matrix(qRel);
    const Euler3 e  = fox::matrix_to_euler_B(R);   // (atan2(-m20,m22), asin(m21), atan2(-m01,m11))
    // §28.5 anatomical mapping for ankle:
    //   variant-B middle axis (asin(m21))  →  flexion (Y, dorsi/plantar)
    //   variant-B first  axis              →  rotation (Z, axial)
    //   variant-B third  axis              →  abduction (X, eversion/inversion)
    double abd = e.e2 * kRad2Deg;
    double flx = e.e1 * kRad2Deg;
    double rot = e.e0 * kRad2Deg;
    if (leftSide) { abd = -abd; rot = -rot; }
    return { abd, flx, rot };
}

}  // anonymous namespace

JointAngles jointAnglesErgo(int jointIdx, const Quat& qParentWorld, const Quat& qChildWorld)
{
    // Relative quaternion parent → child: q_rel = q_parent ⊗ conj(q_child).
    // (Spec §11.3 [2.3] — gives the rotation that takes the child frame into
    // the parent frame, which is the natural definition of a joint angle.)
    const Quat qRel = fox::quat_mult(qParentWorld, qChildWorld.conj()).normalized();

    JointAngles a;
    switch (fox::body::ergoTypeOf(jointIdx)) {
        case 0:  a = handlerAxial(qRel); break;
        case 1:  a = handlerRight(qRel); break;
        case 2:  a = handlerLeft(qRel); break;
        case 3:  a = handlerFoot(qRel, /*leftSide=*/false); break;
        case 4:  a = handlerFoot(qRel, /*leftSide=*/true); break;
        default: a = handlerAxial(qRel); break;
    }

    // Spec §14 / §37 — clamp to per-joint anatomical range of motion.
    if (jointIdx >= 0 && jointIdx < fox::body::kJointCount) {
        const auto& rom = fox::body::kJointRom[jointIdx];
        if (a.abductionDeg < rom.abdMin) a.abductionDeg = rom.abdMin;
        if (a.abductionDeg > rom.abdMax) a.abductionDeg = rom.abdMax;
        if (a.flexionDeg   < rom.flxMin) a.flexionDeg   = rom.flxMin;
        if (a.flexionDeg   > rom.flxMax) a.flexionDeg   = rom.flxMax;
        if (a.rotationDeg  < rom.rotMin) a.rotationDeg  = rom.rotMin;
        if (a.rotationDeg  > rom.rotMax) a.rotationDeg  = rom.rotMax;
    }
    return a;
}

std::array<JointAngles, fox::body::kJointCount>
jointAnglesErgoAll(const std::array<Quat, fox::body::kSegmentCount>& segWorld)
{
    std::array<JointAngles, fox::body::kJointCount> out{};
    for (int j = 0; j < fox::body::kJointCount; ++j) {
        const auto& jd = fox::body::kJoints[j];
        out[j] = jointAnglesErgo(j, segWorld[jd.parent], segWorld[jd.child]);
    }
    return out;
}

}  // namespace fox::ergo
