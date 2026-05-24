
#ifndef FUSION_REMAP_H
#define FUSION_REMAP_H

#include "FusionMath.h"

typedef enum {
    FusionRemapAlignmentPXPYPZ,
    FusionRemapAlignmentPXPZNY,
    FusionRemapAlignmentPXNZPY,
    FusionRemapAlignmentPXNYNZ,
    FusionRemapAlignmentPYPXNZ,
    FusionRemapAlignmentPYPZPX,
    FusionRemapAlignmentPYNZNX,
    FusionRemapAlignmentPYNXPZ,
    FusionRemapAlignmentPZPXPY,
    FusionRemapAlignmentPZPYNX,
    FusionRemapAlignmentPZNYPX,
    FusionRemapAlignmentPZNXNY,
    FusionRemapAlignmentNZPXNY,
    FusionRemapAlignmentNZPYPX,
    FusionRemapAlignmentNZNYNX,
    FusionRemapAlignmentNZNXPY,
    FusionRemapAlignmentNYPXPZ,
    FusionRemapAlignmentNYPZNX,
    FusionRemapAlignmentNYNZPX,
    FusionRemapAlignmentNYNXNZ,
    FusionRemapAlignmentNXPYNZ,
    FusionRemapAlignmentNXPZPY,
    FusionRemapAlignmentNXNZNY,
    FusionRemapAlignmentNXNYPZ,
} FusionRemapAlignment;

static inline FusionVector FusionRemap(const FusionVector sensor, const FusionRemapAlignment alignment) {
    FusionVector result;
    switch (alignment) {
        case FusionRemapAlignmentPXPYPZ:
            break;
        case FusionRemapAlignmentPXPZNY:
            result.axis.x = +sensor.axis.x;
            result.axis.y = +sensor.axis.z;
            result.axis.z = -sensor.axis.y;
            return result;
        case FusionRemapAlignmentPXNZPY:
            result.axis.x = +sensor.axis.x;
            result.axis.y = -sensor.axis.z;
            result.axis.z = +sensor.axis.y;
            return result;
        case FusionRemapAlignmentPXNYNZ:
            result.axis.x = +sensor.axis.x;
            result.axis.y = -sensor.axis.y;
            result.axis.z = -sensor.axis.z;
            return result;
        case FusionRemapAlignmentPYPXNZ:
            result.axis.x = +sensor.axis.y;
            result.axis.y = +sensor.axis.x;
            result.axis.z = -sensor.axis.z;
            return result;
        case FusionRemapAlignmentPYPZPX:
            result.axis.x = +sensor.axis.y;
            result.axis.y = +sensor.axis.z;
            result.axis.z = +sensor.axis.x;
            return result;
        case FusionRemapAlignmentPYNZNX:
            result.axis.x = +sensor.axis.y;
            result.axis.y = -sensor.axis.z;
            result.axis.z = -sensor.axis.x;
            return result;
        case FusionRemapAlignmentPYNXPZ:
            result.axis.x = +sensor.axis.y;
            result.axis.y = -sensor.axis.x;
            result.axis.z = +sensor.axis.z;
            return result;
        case FusionRemapAlignmentPZPXPY:
            result.axis.x = +sensor.axis.z;
            result.axis.y = +sensor.axis.x;
            result.axis.z = +sensor.axis.y;
            return result;
        case FusionRemapAlignmentPZPYNX:
            result.axis.x = +sensor.axis.z;
            result.axis.y = +sensor.axis.y;
            result.axis.z = -sensor.axis.x;
            return result;
        case FusionRemapAlignmentPZNYPX:
            result.axis.x = +sensor.axis.z;
            result.axis.y = -sensor.axis.y;
            result.axis.z = +sensor.axis.x;
            return result;
        case FusionRemapAlignmentPZNXNY:
            result.axis.x = +sensor.axis.z;
            result.axis.y = -sensor.axis.x;
            result.axis.z = -sensor.axis.y;
            return result;
        case FusionRemapAlignmentNZPXNY:
            result.axis.x = -sensor.axis.z;
            result.axis.y = +sensor.axis.x;
            result.axis.z = -sensor.axis.y;
            return result;
        case FusionRemapAlignmentNZPYPX:
            result.axis.x = -sensor.axis.z;
            result.axis.y = +sensor.axis.y;
            result.axis.z = +sensor.axis.x;
            return result;
        case FusionRemapAlignmentNZNYNX:
            result.axis.x = -sensor.axis.z;
            result.axis.y = -sensor.axis.y;
            result.axis.z = -sensor.axis.x;
            return result;
        case FusionRemapAlignmentNZNXPY:
            result.axis.x = -sensor.axis.z;
            result.axis.y = -sensor.axis.x;
            result.axis.z = +sensor.axis.y;
            return result;
        case FusionRemapAlignmentNYPXPZ:
            result.axis.x = -sensor.axis.y;
            result.axis.y = +sensor.axis.x;
            result.axis.z = +sensor.axis.z;
            return result;
        case FusionRemapAlignmentNYPZNX:
            result.axis.x = -sensor.axis.y;
            result.axis.y = +sensor.axis.z;
            result.axis.z = -sensor.axis.x;
            return result;
        case FusionRemapAlignmentNYNZPX:
            result.axis.x = -sensor.axis.y;
            result.axis.y = -sensor.axis.z;
            result.axis.z = +sensor.axis.x;
            return result;
        case FusionRemapAlignmentNYNXNZ:
            result.axis.x = -sensor.axis.y;
            result.axis.y = -sensor.axis.x;
            result.axis.z = -sensor.axis.z;
            return result;
        case FusionRemapAlignmentNXPYNZ:
            result.axis.x = -sensor.axis.x;
            result.axis.y = +sensor.axis.y;
            result.axis.z = -sensor.axis.z;
            return result;
        case FusionRemapAlignmentNXPZPY:
            result.axis.x = -sensor.axis.x;
            result.axis.y = +sensor.axis.z;
            result.axis.z = +sensor.axis.y;
            return result;
        case FusionRemapAlignmentNXNZNY:
            result.axis.x = -sensor.axis.x;
            result.axis.y = -sensor.axis.z;
            result.axis.z = -sensor.axis.y;
            return result;
        case FusionRemapAlignmentNXNYPZ:
            result.axis.x = -sensor.axis.x;
            result.axis.y = -sensor.axis.y;
            result.axis.z = +sensor.axis.z;
            return result;
    }
    return sensor;
}

#endif
