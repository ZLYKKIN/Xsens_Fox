// Fox Mocap — pure rotation math (WXYZ quaternions, scalar-first).
//
// Extracted verbatim from main.cpp/main.h so it can be unit-tested in isolation
// (it links against only QtGui's QVector3D, with no Xsens/Manus/Qt-Widgets
// dependency).  All conventions match hipose/rotations.py and the MVN MXTP wire
// contract: quat_mult = Hamilton product, vec_rotate(v,q) = q·[0,v]·q⁻¹, world
// frame = NWU (X-fwd, Y-left, Z-up, right-handed).
#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <QtGui/QVector3D>

#include <cmath>

namespace fox {

// ============================================================================
//  Quaternion (WXYZ, scalar first)
// ============================================================================

struct Quat {
    double w, x, y, z;

    constexpr Quat() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    constexpr Quat(double w_, double x_, double y_, double z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    double norm() const { return std::sqrt(w*w + x*x + y*y + z*z); }

    // True when every component is finite (no NaN / ±Inf).
    bool isFinite() const {
        return std::isfinite(w) && std::isfinite(x)
            && std::isfinite(y) && std::isfinite(z);
    }

    // Unit quaternion.  Returns identity for a degenerate (near-zero) OR a
    // non-finite quaternion.  NOTE: the previous guard `n < 1e-12` is FALSE
    // when n is NaN, so a NaN/Inf quaternion used to pass through unchanged and
    // propagate into the wire stream (Blender mathutils / UE FQuat then choke).
    // Testing norm-squared finiteness closes that hole at the single source
    // every rotation flows through.  For finite, non-degenerate input the
    // result is identical to before.
    Quat normalized() const {
        const double n2 = w*w + x*x + y*y + z*z;
        if (!std::isfinite(n2) || n2 < 1e-24) return Quat(1, 0, 0, 0);
        const double n = std::sqrt(n2);
        return Quat(w/n, x/n, y/n, z/n);
    }

    Quat conj() const { return Quat(w, -x, -y, -z); }

    Quat inv() const {
        const double n2 = w*w + x*x + y*y + z*z;
        if (!std::isfinite(n2) || n2 < 1e-12) return Quat(1, 0, 0, 0);
        return Quat(w/n2, -x/n2, -y/n2, -z/n2);
    }
};

// ============================================================================
//  Free functions
// ============================================================================

// Hamilton product (scipy Rotation composition).
Quat quat_mult(const Quat& a, const Quat& b);

// Rotate vector v by quaternion q  (v' = q · [0,v] · q⁻¹).
QVector3D vec_rotate(const QVector3D& v, const Quat& q);

// Decompose q into a swing about a plane and a twist about axisU.
void swingTwistDecompose(const Quat& q, const QVector3D& axisU,
                         Quat& outSwing, Quat& outTwist);

// Euler angles → quaternion.  seq is a 3-char upper-case code ("XYZ", "YXZ", …)
// of intrinsic rotations (matches scipy's uppercase convention).
Quat euler_to_quat(double a, double b, double c, const char* seq);

// Drop everything but the yaw (rotation about world +Z); identity if undefined.
Quat yaw_only_quat(const Quat& q);

// Shortest-path spherical interpolation, t in [0,1].  NaN-safe.
Quat slerp_quat(const Quat& a, const Quat& b, double t);

// Rotation magnitude of q in degrees (0..360).
double quat_angle_deg(const Quat& q);

// Mirror a rotation across the body XZ-plane (Y-flip).  j·q·j⁻¹ = (w,-x,y,-z);
// a homomorphism, so it composes correctly with quat_mult.
inline Quat mirror_y_quat(const Quat& q) {
    return Quat(q.w, -q.x, q.y, -q.z);
}

// Return `q` expressed in the same hemisphere as `prev`.  A quaternion and its
// negation encode the *same* rotation, so this never changes what a frame
// represents — but it stops the sign from flipping between consecutive frames,
// which would otherwise make a receiver that interpolates (UE LiveLink /
// Blender) SLERP the long way around and visibly pop ("terminator-line" jitter).
inline Quat hemisphereContinuous(const Quat& q, const Quat& prev) {
    const double dot = q.w*prev.w + q.x*prev.x + q.y*prev.y + q.z*prev.z;
    return (dot < 0.0) ? Quat(-q.w, -q.x, -q.y, -q.z) : q;
}

}  // namespace fox
