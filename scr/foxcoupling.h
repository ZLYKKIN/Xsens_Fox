
#pragma once

#include "foxbody.h"
#include "foxmath.h"

#include <array>

namespace fox::coupling {

void applySpineRhythm(std::array<Quat, fox::body::kSegmentCount>& orient);

void applyNeckRhythm(std::array<Quat, fox::body::kSegmentCount>& orient);

void applyPelvisTilt(std::array<Quat, fox::body::kSegmentCount>& orient);

void applyScapuloHumeral(std::array<Quat, fox::body::kSegmentCount>& orient);

void applyKneeScrewHome(std::array<Quat, fox::body::kSegmentCount>& orient);

void applyAnkleCoupling(std::array<Quat, fox::body::kSegmentCount>& orient);

struct ToeWeights {
    double w_heel_R, w_toe_R;
    double w_heel_L, w_toe_L;
};
ToeWeights computeToeRockerWeights(
    const std::array<Quat, fox::body::kSegmentCount>& orient);

struct Diagnostics {

    double spineFracL5  = 0.0;
    double spineFracL3  = 0.0;
    double spineFracT12 = 0.0;
    double spineFullDeg = 0.0;

    double scapThetaRDeg = 0.0;
    double scapThetaLDeg = 0.0;
    double scapCEffR     = 0.0;
    double scapCEffL     = 0.0;

    double kneeFlexRDeg  = 0.0;
    double kneeFlexLDeg  = 0.0;
    double kneeScrewRDeg = 0.0;
    double kneeScrewLDeg = 0.0;

    double anklePfRDeg   = 0.0;
    double anklePfLDeg   = 0.0;
    bool   ankleClampedR = false;
    bool   ankleClampedL = false;

    double toeRDeg = 0.0;
    double toeLDeg = 0.0;
    ToeWeights toeWeights{1.0, 0.0, 1.0, 0.0};
};
Diagnostics& diagnostics();

}
