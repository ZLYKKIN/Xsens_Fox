
#ifndef FUSION_MODEL_H
#define FUSION_MODEL_H

#include "FusionMath.h"

static inline FusionVector FusionModelInertial(const FusionVector uncalibrated, const FusionMatrix misalignment, const FusionVector sensitivity, const FusionVector offset) {
    return FusionMatrixMultiply(misalignment, FusionVectorHadamard(FusionVectorSubtract(uncalibrated, offset), sensitivity));
}

static inline FusionVector FusionModelMagnetic(const FusionVector uncalibrated, const FusionMatrix softIronMatrix, const FusionVector hardIronOffset) {
    return FusionMatrixMultiply(softIronMatrix, FusionVectorSubtract(uncalibrated, hardIronOffset));
}

#endif
