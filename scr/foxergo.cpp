
#include "foxergo.h"

namespace fox::ergo {

using fox::Matrix3;
using fox::Euler3;

namespace {

constexpr double kRad2Deg = 57.29577951308232;

// formules.txt §114.4 (стр. 385): ZYX roll/pitch/yaw из кватerniona (точное
// совпадение со спекой). Знаки L/R-зеркала — в handlerLeft/handlerFoot (§30).
inline Euler3 eulerZYX(const Quat& q)
{
    Euler3 e;

    e.e0 = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y));

    e.e1 = fox::clamp_asin(2.0 * (q.w * q.y - q.z * q.x));

    e.e2 = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return e;
}

JointAngles handlerAxial(const Quat& qRel)
{
    const Euler3 e = eulerZYX(qRel);
    return { e.e0 * kRad2Deg, e.e1 * kRad2Deg, e.e2 * kRad2Deg };
}

JointAngles handlerRight(const Quat& qRel)
{
    const Euler3 e = eulerZYX(qRel);
    return { e.e0 * kRad2Deg, e.e1 * kRad2Deg, e.e2 * kRad2Deg };
}

JointAngles handlerLeft(const Quat& qRel)
{
    const Euler3 e = eulerZYX(qRel);
    return { -e.e0 * kRad2Deg,  e.e1 * kRad2Deg, -e.e2 * kRad2Deg };
}

JointAngles handlerFoot(const Quat& qRel, bool leftSide)
{
    const Matrix3 R = fox::quat_to_matrix(qRel);
    const Euler3 e  = fox::matrix_to_euler_B(R);

    double abd = e.e2 * kRad2Deg;
    double flx = e.e1 * kRad2Deg;
    double rot = e.e0 * kRad2Deg;
    if (leftSide) { abd = -abd; rot = -rot; }
    return { abd, flx, rot };
}

}

JointAngles jointAnglesErgo(int jointIdx, const Quat& qParentWorld, const Quat& qChildWorld)
{

    const Quat qRel = fox::quat_mult(qParentWorld, qChildWorld.conj()).normalized();

    JointAngles a;
    switch (fox::body::ergoTypeOf(jointIdx)) {
        case 0:  a = handlerAxial(qRel); break;
        case 1:  a = handlerRight(qRel); break;
        case 2:  a = handlerLeft(qRel); break;
        case 3:  a = handlerFoot(qRel, false); break;
        case 4:  a = handlerFoot(qRel, true); break;
        default: a = handlerAxial(qRel); break;
    }

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

}
