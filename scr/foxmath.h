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
#include <vector>

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

// ============================================================================
//  Spec primitives — exact formulas from the FOX_KFA Motion Engine reference.
// ============================================================================

// Spec §15.2 — «safe» asin/acos with built-in argument clamp to [-1, +1].
// Round-off in earlier products can push the argument just past the domain;
// without the clamp asin/acos return NaN.  Both forms are inline because they
// land in tight inner loops (SLERP, Euler decomposition, ω-from-Δq).
inline double clamp_asin(double x) {
    if (x <= -1.0) return -M_PI_2;
    if (x >=  1.0) return  M_PI_2;
    return std::asin(x);
}
inline double clamp_acos(double x) {
    if (x <= -1.0) return  M_PI;
    if (x >=  1.0) return  0.0;
    return std::acos(x);
}

// Spec §5.1 — exp-map: rotation vector φ = θ·n  →  unit quaternion.
// q.w = cos(θ/2), q.xyz = sin(θ/2)·n with the small-angle limit handled.
Quat quat_exp_rotvec(double phix, double phiy, double phiz);

// Spec §5.2 — log-map: unit quaternion → rotation vector φ = θ·n.
// θ = 2·atan2(‖xyz‖, w); n = xyz / ‖xyz‖.  Returns (0,0,0) for q ≈ identity.
QVector3D quat_log(const Quat& q);

// Spec §3.1 — quaternion → 3×3 rotation matrix.  Row-major: [r0,r1,r2,r3,...,r8].
// Uses the Kayley form on the diagonal (w²±x²±y²±z²) which is correct even
// for slightly non-unit q.
struct Matrix3 { double m[9]; };
Matrix3 quat_to_matrix(const Quat& q);

// Spec §4.3 — matrix → Euler-XYZ (variant A) and Euler-YXZ-like (variant B),
// used by jointAnglesErgo for per-joint anatomical decomposition.
// Both return roll/pitch/yaw in RADIANS; callers multiply by 180/π if needed.
struct Euler3 { double e0, e1, e2; };
Euler3 matrix_to_euler_A(const Matrix3& R);  // (atan2(m21,m11), asin(-m01), atan2(m02,m00))
Euler3 matrix_to_euler_B(const Matrix3& R);  // (atan2(-m20,m22), asin(m21), atan2(-m01,m11))

// Spec §12.2 — angular velocity from a relative quaternion Δq over Δt.
//     v = Δq.xyz,  ‖v‖ = sin(θ/2);
//     ω = (2·asin(‖v‖) / (‖v‖·Δt)) · v          (‖v‖ ≠ 0)
//     ω = (2 / Δt) · v                          (small-angle limit)
// This is the *exact* formula extracted from fox_types_engine.dll.
QVector3D angular_velocity_from_quat(const Quat& dq, double dtSec);

// Spec §3.2 — Shepperd's algorithm: matrix → quaternion via the largest
// diagonal candidate (numerically stable when one component is near zero).
// Result is canonicalised to w ≥ 0 and normalised.
Quat matrix_to_quat_sheppard(const Matrix3& R);

// Spec §174.2 — Shepard / Markley 4-D eigenvector quaternion average.
//   M = Σ q_k q_kᵀ  (4×4 symmetric, accumulated over `samples`);
//   q_avg = principal eigenvector of M, canonicalised to w ≥ 0.
// Stable across hemispheres and outperforms the naïve sum/normalise mean for
// quaternion spreads beyond a few degrees.  Returns identity if `samples`
// is empty.
Quat quat_avg_markley(const std::vector<Quat>& samples);

// Spec §174.2 helper — Jacobi eigendecomposition of a 4×4 symmetric matrix.
// On return: A's diagonal holds eigenvalues, U holds the eigenvectors in
// columns.  Used by quat_avg_markley; exposed because the same matrix
// shape comes up in other quaternion-averaging contexts.
void jacobiSym4(double A[4][4], double U[4][4]);

// Spec §40 — fractional power of a unit quaternion via log/exp:
//     q^t = exp( t · log(q) ).
// For t = w_j / Σw, this distributes a single «total» rotation across
// several joints in proportion to their coupling weights (spine rhythm,
// scapulo-humeral ratio, knee screw, ankle/toe rocker).  Identity for t = 0.
Quat quat_pow(const Quat& q, double t);

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
