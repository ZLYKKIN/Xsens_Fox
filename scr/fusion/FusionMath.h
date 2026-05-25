
#ifndef FUSION_MATH_H
#define FUSION_MATH_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef union {
    float array[3];

    struct {
        float x;
        float y;
        float z;
    } axis;
} FusionVector;

typedef union {
    float array[4];

    struct {
        float w;
        float x;
        float y;
        float z;
    } element;
} FusionQuaternion;

typedef union {
    float array[9];

    struct {
        float xx;
        float xy;
        float xz;
        float yx;
        float yy;
        float yz;
        float zx;
        float zy;
        float zz;
    } element;
} FusionMatrix;

typedef union {
    float array[3];

    struct {
        float roll;
        float pitch;
        float yaw;
    } angle;
} FusionEuler;

#define FUSION_VECTOR_ZERO ((FusionVector){ .array = {0.0f, 0.0f, 0.0f} })

#define FUSION_VECTOR_ONES ((FusionVector){ .array = {1.0f, 1.0f, 1.0f} })

#define FUSION_QUATERNION_IDENTITY ((FusionQuaternion){ .array = {1.0f, 0.0f, 0.0f, 0.0f} })

#define FUSION_MATRIX_IDENTITY ((FusionMatrix){ .array = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f} })

#define FUSION_EULER_ZERO ((FusionEuler){ .array = {0.0f, 0.0f, 0.0f} })

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// §38 переводы градусы<->радианы (formules.txt)
static inline float FusionDegreesToRadians(const float degrees) {
    return degrees * ((float) M_PI / 180.0f);
}

static inline float FusionRadiansToDegrees(const float radians) {
    return radians * (180.0f / (float) M_PI);
}

// §115/§1280 безопасный asin с зажимом аргумента в [-1,1] (formules.txt)
static inline float FusionArcSin(const float value) {
    if (value <= -1.0f) {
        return (float) M_PI / -2.0f;
    }
    if (value >= 1.0f) {
        return (float) M_PI / 2.0f;
    }
    return asinf(value);
}

#ifndef FUSION_USE_NORMAL_SQRT

// §181/§1283 быстрый обратный квадратный корень (bit-hack + 1 итерация Ньютона-Рафсона) (formules.txt)
static inline float FusionFastInverseSqrt(const float x) {
    typedef union {
        float f;
        int32_t i;
    } Union32;

    Union32 union32 = {.f = x};
    union32.i = 0x5F1F1412 - (union32.i >> 1);
    return union32.f * (1.69000231f - 0.714158168f * x * union32.f * union32.f);
}

#endif

static inline bool FusionVectorIsZero(const FusionVector v) {
    return (v.axis.x == 0.0f) && (v.axis.y == 0.0f) && (v.axis.z == 0.0f);
}

static inline FusionVector FusionVectorAdd(const FusionVector a, const FusionVector b) {
    const FusionVector result = {
        .axis = {
            .x = a.axis.x + b.axis.x,
            .y = a.axis.y + b.axis.y,
            .z = a.axis.z + b.axis.z,
        }
    };
    return result;
}

static inline FusionVector FusionVectorSubtract(const FusionVector a, const FusionVector b) {
    const FusionVector result = {
        .axis = {
            .x = a.axis.x - b.axis.x,
            .y = a.axis.y - b.axis.y,
            .z = a.axis.z - b.axis.z,
        }
    };
    return result;
}

static inline FusionVector FusionVectorScale(const FusionVector v, const float s) {
    const FusionVector result = {
        .axis = {
            .x = v.axis.x * s,
            .y = v.axis.y * s,
            .z = v.axis.z * s,
        }
    };
    return result;
}

static inline float FusionVectorSum(const FusionVector v) {
    return v.axis.x + v.axis.y + v.axis.z;
}

static inline FusionVector FusionVectorHadamard(const FusionVector a, const FusionVector b) {
    const FusionVector result = {
        .axis = {
            .x = a.axis.x * b.axis.x,
            .y = a.axis.y * b.axis.y,
            .z = a.axis.z * b.axis.z,
        }
    };
    return result;
}

static inline FusionVector FusionVectorCross(const FusionVector a, const FusionVector b) {
    const FusionVector result = {
        .axis = {
            .x = a.axis.y * b.axis.z - a.axis.z * b.axis.y,
            .y = a.axis.z * b.axis.x - a.axis.x * b.axis.z,
            .z = a.axis.x * b.axis.y - a.axis.y * b.axis.x,
        }
    };
    return result;
}

static inline float FusionVectorDot(const FusionVector a, const FusionVector b) {
    return FusionVectorSum(FusionVectorHadamard(a, b));
}

static inline float FusionVectorNormSquared(const FusionVector v) {
    return FusionVectorSum(FusionVectorHadamard(v, v));
}

static inline float FusionVectorNorm(const FusionVector v) {
    return sqrtf(FusionVectorNormSquared(v));
}

static inline FusionVector FusionVectorNormalise(const FusionVector v) {
#ifdef FUSION_USE_NORMAL_SQRT
    return FusionVectorScale(v, 1.0f / FusionVectorNorm(v));
#else
    return FusionVectorScale(v, FusionFastInverseSqrt(FusionVectorNormSquared(v)));
#endif
}

static inline FusionQuaternion FusionQuaternionAdd(const FusionQuaternion a, const FusionQuaternion b) {
    const FusionQuaternion result = {
        .element = {
            .w = a.element.w + b.element.w,
            .x = a.element.x + b.element.x,
            .y = a.element.y + b.element.y,
            .z = a.element.z + b.element.z,
        }
    };
    return result;
}

static inline FusionQuaternion FusionQuaternionScale(const FusionQuaternion q, const float s) {
    const FusionQuaternion result = {
        .element = {
            .w = q.element.w * s,
            .x = q.element.x * s,
            .y = q.element.y * s,
            .z = q.element.z * s,
        }
    };
    return result;
}

static inline float FusionQuaternionSum(const FusionQuaternion q) {
    return q.element.w + q.element.x + q.element.y + q.element.z;
}

static inline FusionQuaternion FusionQuaternionHadamard(const FusionQuaternion a, const FusionQuaternion b) {
    const FusionQuaternion result = {
        .element = {
            .w = a.element.w * b.element.w,
            .x = a.element.x * b.element.x,
            .y = a.element.y * b.element.y,
            .z = a.element.z * b.element.z,
        }
    };
    return result;
}

static inline FusionQuaternion FusionQuaternionProduct(const FusionQuaternion a, const FusionQuaternion b) {
#define A a.element
#define B b.element
    const FusionQuaternion result = {
        .element = {
            .w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z,
            .x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y,
            .y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x,
            .z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w,
        }
    };
#undef A
#undef B
    return result;
}

static inline FusionQuaternion FusionQuaternionVectorProduct(const FusionQuaternion q, const FusionVector v) {
#define Q q.element
#define V v.axis
    const FusionQuaternion result = {
        .element = {
            .w = -Q.x * V.x - Q.y * V.y - Q.z * V.z,
            .x = Q.w * V.x + Q.y * V.z - Q.z * V.y,
            .y = Q.w * V.y - Q.x * V.z + Q.z * V.x,
            .z = Q.w * V.z + Q.x * V.y - Q.y * V.x,
        }
    };
#undef Q
#undef V
    return result;
}

static inline float FusionQuaternionNormSquared(const FusionQuaternion q) {
    return FusionQuaternionSum(FusionQuaternionHadamard(q, q));
}

static inline float FusionQuaternionNorm(const FusionQuaternion q) {
    return sqrtf(FusionQuaternionNormSquared(q));
}

static inline FusionQuaternion FusionQuaternionNormalise(const FusionQuaternion q) {
#ifdef FUSION_USE_NORMAL_SQRT
    return FusionQuaternionScale(q, 1.0f / FusionQuaternionNorm(q));
#else
    return FusionQuaternionScale(q, FusionFastInverseSqrt(FusionQuaternionNormSquared(q)));
#endif
}

static inline FusionVector FusionMatrixMultiply(const FusionMatrix m, const FusionVector v) {
#define M m.element
#define V v.axis
    const FusionVector result = {
        .axis = {
            .x = M.xx * V.x + M.xy * V.y + M.xz * V.z,
            .y = M.yx * V.x + M.yy * V.y + M.yz * V.z,
            .z = M.zx * V.x + M.zy * V.y + M.zz * V.z,
        }
    };
#undef M
#undef V
    return result;
}

static inline FusionMatrix FusionQuaternionToMatrix(const FusionQuaternion q) {
#define Q q.element
    const float twoQw = 2.0f * Q.w;
    const float twoQx = 2.0f * Q.x;
    const float twoQy = 2.0f * Q.y;
    const float twoQz = 2.0f * Q.z;
    const FusionMatrix matrix = {
        .element = {
            .xx = twoQw * Q.w - 1.0f + twoQx * Q.x,
            .xy = twoQx * Q.y - twoQw * Q.z,
            .xz = twoQx * Q.z + twoQw * Q.y,
            .yx = twoQx * Q.y + twoQw * Q.z,
            .yy = twoQw * Q.w - 1.0f + twoQy * Q.y,
            .yz = twoQy * Q.z - twoQw * Q.x,
            .zx = twoQx * Q.z - twoQw * Q.y,
            .zy = twoQy * Q.z + twoQw * Q.x,
            .zz = twoQw * Q.w - 1.0f + twoQz * Q.z,
        }
    };
#undef Q
    return matrix;
}

static inline FusionEuler FusionQuaternionToEuler(const FusionQuaternion q) {
#define Q q.element
    const FusionEuler euler = {
        .angle = {
            .roll = FusionRadiansToDegrees(atan2f(Q.y * Q.z + Q.w * Q.x, Q.w * Q.w + Q.z * Q.z - 0.5f)),
            .pitch = FusionRadiansToDegrees(FusionArcSin(2.0f * (Q.w * Q.y - Q.x * Q.z))),
            .yaw = FusionRadiansToDegrees(atan2f(Q.x * Q.y + Q.w * Q.z, Q.w * Q.w + Q.x * Q.x - 0.5f)),
        }
    };
#undef Q
    return euler;
}

#endif
