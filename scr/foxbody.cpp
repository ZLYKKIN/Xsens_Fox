
#include "foxbody.h"

#include <cstring>
#include <string>

namespace fox::body {

namespace {

inline double cos_half(double th) { return std::cos(0.5 * th); }
inline double sin_half(double th) { return std::sin(0.5 * th); }

Quat axisX(double thRad) { return Quat(cos_half(thRad), sin_half(thRad), 0, 0); }
Quat axisY(double thRad) { return Quat(cos_half(thRad), 0, sin_half(thRad), 0); }
Quat axisZ(double thRad) { return Quat(cos_half(thRad), 0, 0, sin_half(thRad)); }

constexpr double kDeg = 0.017453292519943295;

constexpr Quat kIdent = Quat(1, 0, 0, 0);

inline Quat legacySpine(int seg) {
    switch (seg) {
        case 0:  return Quat(0.9984697627340179, 0, 0.05530038793601835, 0);
        case 1:  return kIdent;
        case 2:  return Quat(0.9987077007098614, 0, -0.05082252003612363, 0);
        case 3:  return Quat(0.9987050781810652, 0, -0.05087402888854364, 0);
        case 4:  return kIdent;
        case 5:  return Quat(0.9939511715005132, 0,  0.1098228968510563,  0);
        case 6:  return Quat(0.999568500071736,  0,  0.02937369000210807, 0);
        default: return kIdent;
    }
}

inline Quat legacyLeg(int seg) {

    switch (seg) {
        case 15:
        case 19:
            return Quat(0.9997115343780182, 0, 0.02401766082591783, 0);
        case 16:
        case 20:
            return Quat(0.999173864575052,  0, 0.04063973855914903, 0);
        case 17:
        case 18:
        case 21:
        case 22:
            return kIdent;
        default:
            return kIdent;
    }
}

inline Quat maleSpine(int seg) {
    switch (seg) {
        case 0:  return Quat(0.99691733, 0,  0.0784591,   0);
        case 1:  return Quat(0.99695624, 0,  0.0779632,   0);
        case 2:  return Quat(0.99919194, 0, -0.04019283,  0);
        case 3:  return Quat(0.99902228, 0, -0.04420961,  0);
        case 4:  return Quat(0.99929604, 0,  0.03751577,  0);
        case 5:  return kIdent;
        case 6:  return kIdent;
        default: return kIdent;
    }
}

inline Quat femaleSpine(int seg) {
    switch (seg) {
        case 0:  return Quat(0.9945219,  0,  0.10452846,  0);
        case 1:  return Quat(0.99518216, 0,  0.09804319,  0);
        case 2:  return Quat(0.99872068, 0, -0.05056679,  0);
        case 3:  return Quat(0.99845209, 0, -0.05561849,  0);
        case 4:  return Quat(0.99924607, 0,  0.03882382,  0);
        case 5:  return kIdent;
        case 6:  return kIdent;
        default: return kIdent;
    }
}

inline Quat armT(int seg) {

    (void)seg;
    return kIdent;
}

inline Quat armN(int seg) {

    switch (seg) {
        case 7:  return Quat(kCos5,    kSin5,   0, 0);
        case 8:
        case 9:
        case 10: return Quat(kSqrt2H,  kSqrt2H, 0, 0);
        case 11: return Quat(kCos5,   -kSin5,   0, 0);
        case 12:
        case 13:
        case 14: return Quat(kSqrt2H, -kSqrt2H, 0, 0);
        default: return kIdent;
    }
}

inline Quat maleArmTOverride(int seg) {
    if (seg == 9)  return Quat(0.9997969880959272, 0, 0,  0.02014900976009601);
    if (seg == 13) return Quat(0.9997969880959272, 0, 0, -0.02014900976009601);
    return armT(seg);
}

}

Quat referenceQuat(int seg, Pose pose, Gender gender) {
    if (seg < 0 || seg >= kSegmentCount) return kIdent;

    if (seg <= 6) {
        switch (gender) {
            case GenderMale:   return maleSpine(seg);
            case GenderFemale: return femaleSpine(seg);
            case GenderLegacy:
            default:           return legacySpine(seg);
        }
    }

    if (seg >= 7 && seg <= 14) {
        if (pose == PoseT) {
            return (gender == GenderMale) ? maleArmTOverride(seg) : armT(seg);
        }
        return armN(seg);
    }

    return legacyLeg(seg);
}

}
