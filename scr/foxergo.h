
#pragma once

#include "foxbody.h"
#include "foxmath.h"

#include <array>

namespace fox::ergo {

struct JointAngles {
    double abductionDeg;
    double flexionDeg;
    double rotationDeg;
};

JointAngles jointAnglesErgo(int jointIdx, const Quat& qParentWorld, const Quat& qChildWorld);

std::array<JointAngles, fox::body::kJointCount>
jointAnglesErgoAll(const std::array<Quat, fox::body::kSegmentCount>& segWorld);

}
