
#include "foxbody.h"

#include <cstring>
#include <string>
#include <utility>

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

namespace {

bool eat(const std::string& s, std::size_t& i, const char* token) {
    const std::size_t n = std::strlen(token);
    if (s.compare(i, n, token) != 0) return false;
    i += n;
    return true;
}

SpcEpoch parseEpoch(const std::string& s, std::size_t& i) {
    if (eat(s, i, "calibration"))   return SpcEpoch::Calibration;
    if (eat(s, i, "leftArmRaise"))  return SpcEpoch::LeftArmRaise;
    if (eat(s, i, "rightArmRaise")) return SpcEpoch::RightArmRaise;
    if (eat(s, i, "leftLegRaise"))  return SpcEpoch::LeftLegRaise;
    if (eat(s, i, "rightLegRaise")) return SpcEpoch::RightLegRaise;
    return SpcEpoch::Calibration;
}
SpcSignal parseSignal(const std::string& s, std::size_t& i) {
    if (eat(s, i, "Acc")) return SpcSignal::Acc;
    if (eat(s, i, "Gyr")) return SpcSignal::Gyr;
    return SpcSignal::Acc;
}
SpcAxis parseAxis(const std::string& s, std::size_t& i) {
    if (eat(s, i, "Normxyz")) return SpcAxis::Normxyz;
    if (eat(s, i, "xAbs"))    return SpcAxis::XAbs;
    if (eat(s, i, "yAbs"))    return SpcAxis::YAbs;
    if (eat(s, i, "zAbs"))    return SpcAxis::ZAbs;
    if (eat(s, i, "x"))       return SpcAxis::X;
    if (eat(s, i, "y"))       return SpcAxis::Y;
    if (eat(s, i, "z"))       return SpcAxis::Z;
    return SpcAxis::Normxyz;
}
SpcBand parseBand(const std::string& s, std::size_t& i) {
    if (eat(s, i, "freqBand0.5To4.0"))   return SpcBand::Band0p5To4;
    if (eat(s, i, "freqBand4.5To10.0"))  return SpcBand::Band4p5To10;
    if (eat(s, i, "freqBand10.0To-1.0")) return SpcBand::Band10ToNyq;
    return SpcBand::None;
}
SpcStat parseStat(const std::string& s, std::size_t& i) {

    if (eat(s, i, "sameAxisInterSensorCorrAbsMax")) return SpcStat::SameAxisInterSensorCorrAbsMax;
    if (eat(s, i, "sameAxisInterSensorCorrAbsSum")) return SpcStat::SameAxisInterSensorCorrAbsSum;
    if (eat(s, i, "sameAxisInterSensorCorrMax"))    return SpcStat::SameAxisInterSensorCorrMax;
    if (eat(s, i, "sameAxisInterSensorCorrSum"))    return SpcStat::SameAxisInterSensorCorrSum;
    if (eat(s, i, "sameSensorInterAxisCorrAbsMax")) return SpcStat::SameSensorInterAxisCorrAbsMax;
    if (eat(s, i, "sameSensorInterAxisCorrAbsSum")) return SpcStat::SameSensorInterAxisCorrAbsSum;
    if (eat(s, i, "sameSensorInterAxisCorrMax"))    return SpcStat::SameSensorInterAxisCorrMax;
    if (eat(s, i, "sameSensorInterAxisCorrSum"))    return SpcStat::SameSensorInterAxisCorrSum;
    if (eat(s, i, "kurtosis"))                       return SpcStat::Kurtosis;
    if (eat(s, i, "maxIdx"))                         return SpcStat::MaxIdx;
    if (eat(s, i, "mean"))                           return SpcStat::Mean;
    if (eat(s, i, "rms"))                            return SpcStat::Rms;
    if (eat(s, i, "skew"))                           return SpcStat::Skew;
    if (eat(s, i, "std"))                            return SpcStat::Std;
    if (eat(s, i, "sum"))                            return SpcStat::Sum;
    if (eat(s, i, "var"))                            return SpcStat::Var;
    if (eat(s, i, "max"))                            return SpcStat::Max;
    return SpcStat::Mean;
}

SpcFeatureSpec parseFeatureName(const char* name) {
    const std::string s(name);
    std::size_t i = 0;
    SpcFeatureSpec out;
    out.epoch  = parseEpoch(s, i);
    out.signal = parseSignal(s, i);
    if (i < s.size() && s[i] == '_') ++i;
    out.axis   = parseAxis(s, i);
    if (i < s.size() && s[i] == '_') ++i;
    out.band   = parseBand(s, i);
    out.stat   = parseStat(s, i);
    return out;
}

std::pair<float, float> deriveRange(const SpcFeatureSpec& f) {
    const bool isCorr = f.stat == SpcStat::SameAxisInterSensorCorrMax
                     || f.stat == SpcStat::SameAxisInterSensorCorrAbsMax
                     || f.stat == SpcStat::SameAxisInterSensorCorrSum
                     || f.stat == SpcStat::SameAxisInterSensorCorrAbsSum
                     || f.stat == SpcStat::SameSensorInterAxisCorrMax
                     || f.stat == SpcStat::SameSensorInterAxisCorrAbsMax
                     || f.stat == SpcStat::SameSensorInterAxisCorrSum
                     || f.stat == SpcStat::SameSensorInterAxisCorrAbsSum;
    if (isCorr) {

        const bool isSum = f.stat == SpcStat::SameAxisInterSensorCorrSum
                        || f.stat == SpcStat::SameAxisInterSensorCorrAbsSum;
        const bool isAbsSum  = f.stat == SpcStat::SameAxisInterSensorCorrAbsSum;
        const bool isInterAxisSum = f.stat == SpcStat::SameSensorInterAxisCorrSum
                                 || f.stat == SpcStat::SameSensorInterAxisCorrAbsSum;
        const bool isAbsMax = f.stat == SpcStat::SameAxisInterSensorCorrAbsMax
                           || f.stat == SpcStat::SameSensorInterAxisCorrAbsMax;
        if (isSum)        return isAbsSum ? std::pair{0.0f, 16.0f} : std::pair{-16.0f, 16.0f};
        if (isInterAxisSum) return std::pair{-2.0f, 2.0f};
        return isAbsMax ? std::pair{0.0f, 1.0f} : std::pair{-1.0f, 1.0f};
    }
    if (f.stat == SpcStat::MaxIdx) {

        return {0.0f, 1.0f};
    }
    if (f.stat == SpcStat::Skew) {
        return {-3.0f, 3.0f};
    }
    if (f.stat == SpcStat::Kurtosis) {
        return {-3.0f, 30.0f};
    }

    const bool isGyr     = (f.signal == SpcSignal::Gyr);
    const bool isNorm    = (f.axis   == SpcAxis::Normxyz);
    const bool isAbs     = (f.axis   == SpcAxis::XAbs || f.axis == SpcAxis::YAbs
                                                     || f.axis == SpcAxis::ZAbs);
    const bool isSigned  = !isNorm && !isAbs;
    const bool isFreq    = (f.band != SpcBand::None);

    constexpr float kWin = 300.0f;
    float scalarMax = 0.0f;
    if (isGyr) {
        scalarMax = isFreq ? 25.0f : 15.0f;
    } else {
        scalarMax = isFreq ? 50.0f : 40.0f;
        if (f.stat == SpcStat::Max && isNorm) scalarMax = 80.0f;
    }
    float lo = isSigned ? -scalarMax : 0.0f;
    float hi = scalarMax;
    if (f.stat == SpcStat::Sum) {
        lo *= kWin; hi *= kWin;
    }
    if (f.stat == SpcStat::Std || f.stat == SpcStat::Var) {
        lo = 0.0f;
        hi = (f.stat == SpcStat::Var) ? scalarMax * scalarMax : scalarMax;
    }
    return {lo, hi};
}

std::array<SpcFeatureSpec, kSpcFeatureCount> buildSpecs() {
    std::array<SpcFeatureSpec, kSpcFeatureCount> out{};
    for (int i = 0; i < kSpcFeatureCount; ++i) {
        out[i] = parseFeatureName(::fox::body::kFeatureNames[i]);
    }
    return out;
}
std::array<float, kSpcFeatureCount> buildRangeLo() {
    std::array<float, kSpcFeatureCount> out{};
    for (int i = 0; i < kSpcFeatureCount; ++i) {
        out[i] = deriveRange(::fox::body::kFeatureSpecs[i]).first;
    }
    return out;
}
std::array<float, kSpcFeatureCount> buildRangeHi() {
    std::array<float, kSpcFeatureCount> out{};
    for (int i = 0; i < kSpcFeatureCount; ++i) {
        out[i] = deriveRange(::fox::body::kFeatureSpecs[i]).second;
    }
    return out;
}

}

const std::array<const char*, kSpcFeatureCount> kFeatureNames = {{
    "calibrationAcc_Normxyz_freqBand4.5To10.0mean",
    "calibrationAcc_Normxyz_freqBand4.5To10.0sum",
    "calibrationAcc_Normxyz_mean",
    "calibrationAcc_Normxyz_rms",
    "calibrationAcc_Normxyz_sameAxisInterSensorCorrAbsMax",
    "calibrationAcc_Normxyz_sameAxisInterSensorCorrAbsSum",
    "calibrationAcc_Normxyz_sameAxisInterSensorCorrMax",
    "calibrationAcc_Normxyz_sameAxisInterSensorCorrSum",
    "calibrationAcc_Normxyz_sum",
    "calibrationAcc_x_sameAxisInterSensorCorrAbsMax",
    "calibrationAcc_x_sameAxisInterSensorCorrAbsSum",
    "calibrationAcc_x_sameAxisInterSensorCorrMax",
    "calibrationAcc_x_sameSensorInterAxisCorrMax",
    "calibrationAcc_x_sameSensorInterAxisCorrSum",
    "calibrationAcc_xAbs_freqBand4.5To10.0mean",
    "calibrationAcc_xAbs_freqBand4.5To10.0sum",
    "calibrationAcc_xAbs_mean",
    "calibrationAcc_xAbs_skew",
    "calibrationAcc_xAbs_sum",
    "calibrationAcc_y_freqBand10.0To-1.0kurtosis",
    "calibrationAcc_y_sameAxisInterSensorCorrAbsMax",
    "calibrationAcc_yAbs_mean",
    "calibrationAcc_yAbs_sum",
    "calibrationAcc_z_freqBand10.0To-1.0max",
    "calibrationAcc_z_mean",
    "calibrationAcc_z_sameAxisInterSensorCorrAbsMax",
    "calibrationAcc_z_sameAxisInterSensorCorrMax",
    "calibrationAcc_z_sameAxisInterSensorCorrSum",
    "calibrationAcc_z_sum",
    "calibrationAcc_zAbs_freqBand0.5To4.0maxIdx",
    "calibrationAcc_zAbs_freqBand4.5To10.0mean",
    "calibrationAcc_zAbs_freqBand4.5To10.0sum",
    "calibrationGyr_Normxyz_freqBand0.5To4.0max",
    "calibrationGyr_Normxyz_freqBand0.5To4.0maxIdx",
    "calibrationGyr_Normxyz_freqBand0.5To4.0mean",
    "calibrationGyr_Normxyz_freqBand0.5To4.0rms",
    "calibrationGyr_Normxyz_freqBand0.5To4.0std",
    "calibrationGyr_Normxyz_freqBand0.5To4.0sum",
    "calibrationGyr_Normxyz_freqBand10.0To-1.0maxIdx",
    "calibrationGyr_Normxyz_mean",
    "calibrationGyr_Normxyz_rms",
    "calibrationGyr_Normxyz_sameAxisInterSensorCorrAbsMax",
    "calibrationGyr_Normxyz_sameAxisInterSensorCorrAbsSum",
    "calibrationGyr_Normxyz_sameAxisInterSensorCorrMax",
    "calibrationGyr_Normxyz_sameAxisInterSensorCorrSum",
    "calibrationGyr_Normxyz_sum",
    "calibrationGyr_x_freqBand0.5To4.0max",
    "calibrationGyr_x_sameAxisInterSensorCorrAbsSum",
    "calibrationGyr_x_sameAxisInterSensorCorrSum",
    "calibrationGyr_x_sameSensorInterAxisCorrAbsMax",
    "calibrationGyr_x_sameSensorInterAxisCorrSum",
    "calibrationGyr_y_freqBand0.5To4.0max",
    "calibrationGyr_y_freqBand4.5To10.0rms",
    "calibrationGyr_y_max",
    "calibrationGyr_y_rms",
    "calibrationGyr_y_sameAxisInterSensorCorrAbsMax",
    "calibrationGyr_y_sameSensorInterAxisCorrAbsMax",
    "calibrationGyr_y_std",
    "calibrationGyr_yAbs_freqBand10.0To-1.0rms",
    "calibrationGyr_yAbs_freqBand10.0To-1.0std",
    "calibrationGyr_yAbs_freqBand4.5To10.0mean",
    "calibrationGyr_yAbs_freqBand4.5To10.0rms",
    "calibrationGyr_yAbs_freqBand4.5To10.0std",
    "calibrationGyr_yAbs_freqBand4.5To10.0sum",
    "calibrationGyr_yAbs_max",
    "calibrationGyr_yAbs_mean",
    "calibrationGyr_yAbs_rms",
    "calibrationGyr_yAbs_sum",
    "calibrationGyr_z_sameAxisInterSensorCorrAbsMax",
    "calibrationGyr_z_sameAxisInterSensorCorrAbsSum",
    "calibrationGyr_z_sameAxisInterSensorCorrMax",
    "calibrationGyr_z_sameAxisInterSensorCorrSum",
    "calibrationGyr_z_sameSensorInterAxisCorrMax",
    "calibrationGyr_z_std",
    "calibrationGyr_zAbs_freqBand0.5To4.0skew",
    "calibrationGyr_zAbs_freqBand4.5To10.0rms",
    "calibrationGyr_zAbs_freqBand4.5To10.0std",
    "calibrationGyr_zAbs_mean",
    "calibrationGyr_zAbs_sum",
    "leftArmRaiseAcc_Normxyz_freqBand0.5To4.0mean",
    "leftArmRaiseAcc_Normxyz_freqBand0.5To4.0rms",
    "leftArmRaiseAcc_Normxyz_freqBand0.5To4.0sum",
    "leftArmRaiseAcc_Normxyz_freqBand0.5To4.0var",
    "leftArmRaiseAcc_Normxyz_freqBand10.0To-1.0kurtosis",
    "leftArmRaiseAcc_Normxyz_freqBand10.0To-1.0max",
    "leftArmRaiseAcc_Normxyz_freqBand10.0To-1.0skew",
    "leftArmRaiseAcc_Normxyz_freqBand4.5To10.0rms",
    "leftArmRaiseAcc_Normxyz_freqBand4.5To10.0var",
    "leftArmRaiseAcc_Normxyz_mean",
    "leftArmRaiseAcc_Normxyz_rms",
    "leftArmRaiseAcc_Normxyz_sameAxisInterSensorCorrAbsSum",
    "leftArmRaiseAcc_Normxyz_sameAxisInterSensorCorrMax",
    "leftArmRaiseAcc_Normxyz_std",
    "leftArmRaiseAcc_Normxyz_sum",
    "leftArmRaiseAcc_Normxyz_var",
    "leftArmRaiseAcc_x_max",
    "leftArmRaiseAcc_x_rms",
    "leftArmRaiseAcc_x_sameAxisInterSensorCorrAbsMax",
    "leftArmRaiseAcc_x_sameAxisInterSensorCorrAbsSum",
    "leftArmRaiseAcc_x_sameAxisInterSensorCorrMax",
    "leftArmRaiseAcc_x_sameSensorInterAxisCorrAbsMax",
    "leftArmRaiseAcc_x_sameSensorInterAxisCorrAbsSum",
    "leftArmRaiseAcc_x_std",
    "leftArmRaiseAcc_x_var",
    "leftArmRaiseAcc_xAbs_freqBand0.5To4.0mean",
    "leftArmRaiseAcc_xAbs_freqBand0.5To4.0rms",
    "leftArmRaiseAcc_xAbs_freqBand0.5To4.0sum",
    "leftArmRaiseAcc_xAbs_freqBand4.5To10.0max",
    "leftArmRaiseAcc_xAbs_freqBand4.5To10.0std",
    "leftArmRaiseAcc_xAbs_mean",
    "leftArmRaiseAcc_xAbs_rms",
    "leftArmRaiseAcc_xAbs_std",
    "leftArmRaiseAcc_xAbs_sum",
    "leftArmRaiseAcc_xAbs_var",
    "leftArmRaiseAcc_y_max",
    "leftArmRaiseAcc_y_sameSensorInterAxisCorrSum",
    "leftArmRaiseAcc_y_var",
    "leftArmRaiseAcc_yAbs_mean",
    "leftArmRaiseAcc_yAbs_sum",
    "leftArmRaiseAcc_z_freqBand10.0To-1.0kurtosis",
    "leftArmRaiseAcc_z_mean",
    "leftArmRaiseAcc_z_sameAxisInterSensorCorrSum",
    "leftArmRaiseAcc_z_sum",
    "leftArmRaiseAcc_z_var",
    "leftArmRaiseAcc_zAbs_maxIdx",
    "leftArmRaiseGyr_Normxyz_maxIdx",
    "leftArmRaiseGyr_Normxyz_mean",
    "leftArmRaiseGyr_Normxyz_rms",
    "leftArmRaiseGyr_Normxyz_sameAxisInterSensorCorrAbsMax",
    "leftArmRaiseGyr_Normxyz_sameAxisInterSensorCorrAbsSum",
    "leftArmRaiseGyr_Normxyz_sameAxisInterSensorCorrSum",
    "leftArmRaiseGyr_Normxyz_sum",
    "leftArmRaiseGyr_x_freqBand10.0To-1.0skew",
    "leftArmRaiseGyr_x_maxIdx",
    "leftArmRaiseGyr_x_sameAxisInterSensorCorrAbsMax",
    "leftArmRaiseGyr_x_sameAxisInterSensorCorrSum",
    "leftArmRaiseGyr_x_sameSensorInterAxisCorrMax",
    "leftArmRaiseGyr_xAbs_maxIdx",
    "leftArmRaiseGyr_xAbs_mean",
    "leftArmRaiseGyr_xAbs_std",
    "leftArmRaiseGyr_xAbs_sum",
    "leftArmRaiseGyr_y_freqBand10.0To-1.0max",
    "leftArmRaiseGyr_y_freqBand4.5To10.0maxIdx",
    "leftArmRaiseGyr_y_freqBand4.5To10.0var",
    "leftArmRaiseGyr_y_max",
    "leftArmRaiseGyr_y_sameAxisInterSensorCorrAbsMax",
    "leftArmRaiseGyr_y_sameSensorInterAxisCorrAbsMax",
    "leftArmRaiseGyr_y_std",
    "leftArmRaiseGyr_yAbs_max",
    "leftArmRaiseGyr_yAbs_mean",
    "leftArmRaiseGyr_yAbs_sum",
    "leftArmRaiseGyr_z_freqBand0.5To4.0mean",
    "leftArmRaiseGyr_z_freqBand10.0To-1.0maxIdx",
    "leftArmRaiseGyr_z_max",
    "leftArmRaiseGyr_z_maxIdx",
    "leftLegRaiseAcc_Normxyz_mean",
    "leftLegRaiseAcc_Normxyz_rms",
    "leftLegRaiseAcc_Normxyz_sameAxisInterSensorCorrAbsSum",
    "leftLegRaiseAcc_Normxyz_sameAxisInterSensorCorrSum",
    "leftLegRaiseAcc_Normxyz_std",
    "leftLegRaiseAcc_Normxyz_sum",
    "leftLegRaiseAcc_x_sameAxisInterSensorCorrAbsMax",
    "leftLegRaiseAcc_x_sameAxisInterSensorCorrMax",
    "leftLegRaiseAcc_x_std",
    "leftLegRaiseAcc_xAbs_freqBand10.0To-1.0kurtosis",
    "leftLegRaiseAcc_xAbs_mean",
    "leftLegRaiseAcc_xAbs_std",
    "leftLegRaiseAcc_xAbs_sum",
    "leftLegRaiseAcc_xAbs_var",
    "leftLegRaiseAcc_y_rms",
    "leftLegRaiseAcc_y_sameAxisInterSensorCorrAbsMax",
    "leftLegRaiseAcc_y_std",
    "leftLegRaiseAcc_yAbs_mean",
    "leftLegRaiseAcc_yAbs_rms",
    "leftLegRaiseAcc_yAbs_sum",
    "leftLegRaiseAcc_z_freqBand10.0To-1.0max",
    "leftLegRaiseAcc_zAbs_freqBand10.0To-1.0rms",
    "leftLegRaiseAcc_zAbs_var",
    "leftLegRaiseGyr_Normxyz_mean",
    "leftLegRaiseGyr_Normxyz_rms",
    "leftLegRaiseGyr_Normxyz_sameAxisInterSensorCorrAbsMax",
    "leftLegRaiseGyr_Normxyz_sameAxisInterSensorCorrMax",
    "leftLegRaiseGyr_Normxyz_std",
    "leftLegRaiseGyr_Normxyz_sum",
    "leftLegRaiseGyr_Normxyz_var",
    "leftLegRaiseGyr_x_sameSensorInterAxisCorrMax",
    "leftLegRaiseGyr_y_freqBand4.5To10.0rms",
    "leftLegRaiseGyr_y_freqBand4.5To10.0std",
    "leftLegRaiseGyr_y_sameAxisInterSensorCorrAbsSum",
    "leftLegRaiseGyr_yAbs_freqBand10.0To-1.0skew",
    "leftLegRaiseGyr_yAbs_max",
    "leftLegRaiseGyr_z_freqBand10.0To-1.0max",
    "leftLegRaiseGyr_z_maxIdx",
    "leftLegRaiseGyr_z_sameAxisInterSensorCorrAbsSum",
    "leftLegRaiseGyr_z_sameAxisInterSensorCorrMax",
    "leftLegRaiseGyr_zAbs_freqBand10.0To-1.0skew",
    "rightArmRaiseAcc_Normxyz_freqBand0.5To4.0mean",
    "rightArmRaiseAcc_Normxyz_freqBand0.5To4.0sum",
    "rightArmRaiseAcc_Normxyz_freqBand0.5To4.0var",
    "rightArmRaiseAcc_Normxyz_freqBand10.0To-1.0kurtosis",
    "rightArmRaiseAcc_Normxyz_freqBand10.0To-1.0max",
    "rightArmRaiseAcc_Normxyz_freqBand10.0To-1.0maxIdx",
    "rightArmRaiseAcc_Normxyz_freqBand10.0To-1.0rms",
    "rightArmRaiseAcc_Normxyz_freqBand10.0To-1.0std",
    "rightArmRaiseAcc_Normxyz_max",
    "rightArmRaiseAcc_Normxyz_mean",
    "rightArmRaiseAcc_Normxyz_rms",
    "rightArmRaiseAcc_Normxyz_sameAxisInterSensorCorrAbsMax",
    "rightArmRaiseAcc_Normxyz_sameAxisInterSensorCorrMax",
    "rightArmRaiseAcc_Normxyz_std",
    "rightArmRaiseAcc_Normxyz_sum",
    "rightArmRaiseAcc_Normxyz_var",
    "rightArmRaiseAcc_x_freqBand10.0To-1.0kurtosis",
    "rightArmRaiseAcc_x_max",
    "rightArmRaiseAcc_x_sameAxisInterSensorCorrAbsMax",
    "rightArmRaiseAcc_x_sameAxisInterSensorCorrAbsSum",
    "rightArmRaiseAcc_x_sameAxisInterSensorCorrMax",
    "rightArmRaiseAcc_x_sameAxisInterSensorCorrSum",
    "rightArmRaiseAcc_x_sameSensorInterAxisCorrAbsMax",
    "rightArmRaiseAcc_x_sameSensorInterAxisCorrAbsSum",
    "rightArmRaiseAcc_x_std",
    "rightArmRaiseAcc_x_var",
    "rightArmRaiseAcc_xAbs_freqBand10.0To-1.0kurtosis",
    "rightArmRaiseAcc_xAbs_mean",
    "rightArmRaiseAcc_xAbs_sum",
    "rightArmRaiseAcc_xAbs_var",
    "rightArmRaiseAcc_y_freqBand10.0To-1.0kurtosis",
    "rightArmRaiseAcc_y_rms",
    "rightArmRaiseAcc_y_sameSensorInterAxisCorrMax",
    "rightArmRaiseAcc_y_var",
    "rightArmRaiseAcc_yAbs_maxIdx",
    "rightArmRaiseAcc_yAbs_mean",
    "rightArmRaiseAcc_yAbs_rms",
    "rightArmRaiseAcc_yAbs_sum",
    "rightArmRaiseAcc_z_mean",
    "rightArmRaiseAcc_z_std",
    "rightArmRaiseAcc_z_sum",
    "rightArmRaiseAcc_z_var",
    "rightArmRaiseAcc_zAbs_freqBand10.0To-1.0skew",
    "rightArmRaiseAcc_zAbs_std",
    "rightArmRaiseGyr_Normxyz_freqBand10.0To-1.0kurtosis",
    "rightArmRaiseGyr_Normxyz_max",
    "rightArmRaiseGyr_Normxyz_mean",
    "rightArmRaiseGyr_Normxyz_rms",
    "rightArmRaiseGyr_Normxyz_sameAxisInterSensorCorrAbsMax",
    "rightArmRaiseGyr_Normxyz_sameAxisInterSensorCorrMax",
    "rightArmRaiseGyr_Normxyz_sameAxisInterSensorCorrSum",
    "rightArmRaiseGyr_Normxyz_skew",
    "rightArmRaiseGyr_Normxyz_std",
    "rightArmRaiseGyr_Normxyz_sum",
    "rightArmRaiseGyr_Normxyz_var",
    "rightArmRaiseGyr_x_max",
    "rightArmRaiseGyr_x_sameAxisInterSensorCorrAbsSum",
    "rightArmRaiseGyr_x_sameSensorInterAxisCorrMax",
    "rightArmRaiseGyr_x_std",
    "rightArmRaiseGyr_xAbs_mean",
    "rightArmRaiseGyr_xAbs_sum",
    "rightArmRaiseGyr_y_max",
    "rightArmRaiseGyr_y_sameSensorInterAxisCorrMax",
    "rightArmRaiseGyr_yAbs_freqBand4.5To10.0max",
    "rightArmRaiseGyr_yAbs_max",
    "rightArmRaiseGyr_yAbs_mean",
    "rightArmRaiseGyr_yAbs_sum",
    "rightArmRaiseGyr_z_sameAxisInterSensorCorrSum",
    "rightArmRaiseGyr_zAbs_mean",
    "rightArmRaiseGyr_zAbs_sum",
    "rightLegRaiseAcc_Normxyz_freqBand0.5To4.0mean",
    "rightLegRaiseAcc_Normxyz_freqBand0.5To4.0sum",
    "rightLegRaiseAcc_Normxyz_sameAxisInterSensorCorrAbsSum",
    "rightLegRaiseAcc_Normxyz_sameAxisInterSensorCorrSum",
    "rightLegRaiseAcc_x_freqBand10.0To-1.0maxIdx",
    "rightLegRaiseAcc_x_sameAxisInterSensorCorrAbsSum",
    "rightLegRaiseAcc_x_sameAxisInterSensorCorrMax",
    "rightLegRaiseAcc_xAbs_freqBand0.5To4.0std",
    "rightLegRaiseAcc_xAbs_mean",
    "rightLegRaiseAcc_xAbs_std",
    "rightLegRaiseAcc_xAbs_sum",
    "rightLegRaiseAcc_xAbs_var",
    "rightLegRaiseAcc_y_rms",
    "rightLegRaiseAcc_y_sameAxisInterSensorCorrAbsMax",
    "rightLegRaiseAcc_yAbs_max",
    "rightLegRaiseAcc_yAbs_mean",
    "rightLegRaiseAcc_yAbs_rms",
    "rightLegRaiseAcc_yAbs_sum",
    "rightLegRaiseAcc_z_mean",
    "rightLegRaiseAcc_z_sameAxisInterSensorCorrAbsMax",
    "rightLegRaiseAcc_z_sum",
    "rightLegRaiseAcc_z_var",
    "rightLegRaiseAcc_zAbs_freqBand0.5To4.0skew",
    "rightLegRaiseAcc_zAbs_freqBand0.5To4.0var",
    "rightLegRaiseAcc_zAbs_var",
    "rightLegRaiseGyr_Normxyz_mean",
    "rightLegRaiseGyr_Normxyz_rms",
    "rightLegRaiseGyr_Normxyz_sameAxisInterSensorCorrAbsMax",
    "rightLegRaiseGyr_Normxyz_sameAxisInterSensorCorrMax",
    "rightLegRaiseGyr_Normxyz_std",
    "rightLegRaiseGyr_Normxyz_sum",
    "rightLegRaiseGyr_Normxyz_var",
    "rightLegRaiseGyr_x_sameSensorInterAxisCorrSum",
    "rightLegRaiseGyr_xAbs_std",
    "rightLegRaiseGyr_y_freqBand10.0To-1.0max",
    "rightLegRaiseGyr_y_sameAxisInterSensorCorrMax",
    "rightLegRaiseGyr_y_sameSensorInterAxisCorrAbsMax",
    "rightLegRaiseGyr_y_sameSensorInterAxisCorrSum",
    "rightLegRaiseGyr_yAbs_freqBand10.0To-1.0kurtosis",
    "rightLegRaiseGyr_yAbs_max",
    "rightLegRaiseGyr_yAbs_mean",
    "rightLegRaiseGyr_yAbs_std",
    "rightLegRaiseGyr_yAbs_sum",
    "rightLegRaiseGyr_z_freqBand10.0To-1.0max",
    "rightLegRaiseGyr_z_max",
    "rightLegRaiseGyr_z_maxIdx",
    "rightLegRaiseGyr_z_sameAxisInterSensorCorrSum",
    "rightLegRaiseGyr_zAbs_mean",
    "rightLegRaiseGyr_zAbs_sum",
}};

const std::array<SpcFeatureSpec, kSpcFeatureCount> kFeatureSpecs = buildSpecs();
const std::array<float,          kSpcFeatureCount> kFeatureMin   = buildRangeLo();
const std::array<float,          kSpcFeatureCount> kFeatureMax   = buildRangeHi();

}
