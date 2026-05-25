
#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <QtGui/QVector3D>

#include <cmath>
#include <vector>

namespace fox {

struct Quat {
    double w, x, y, z;

    constexpr Quat() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    constexpr Quat(double w_, double x_, double y_, double z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    double norm() const { return std::sqrt(w*w + x*x + y*y + z*z); }

    bool isFinite() const {
        return std::isfinite(w) && std::isfinite(x)
            && std::isfinite(y) && std::isfinite(z);
    }

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
        return Quat(w/n2, -x/n2, -y/n2, -z/n2).normalized();
    }
};

Quat quat_mult(const Quat& a, const Quat& b);

QVector3D vec_rotate(const QVector3D& v, const Quat& q);

void swingTwistDecompose(const Quat& q, const QVector3D& axisU,
                         Quat& outSwing, Quat& outTwist);

Quat euler_to_quat(double a, double b, double c, const char* seq);

Quat yaw_only_quat(const Quat& q);

Quat slerp_quat(const Quat& a, const Quat& b, double t);

double quat_angle_deg(const Quat& q);

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

Quat quat_exp_rotvec(double phix, double phiy, double phiz);

QVector3D quat_log(const Quat& q);

struct Matrix3 { double m[9]; };
Matrix3 quat_to_matrix(const Quat& q);

struct Euler3 { double e0, e1, e2; };
Euler3 matrix_to_euler_A(const Matrix3& R);
Euler3 matrix_to_euler_B(const Matrix3& R);

QVector3D angular_velocity_from_quat(const Quat& dq, double dtSec);

Quat matrix_to_quat_sheppard(const Matrix3& R);

Quat quat_avg_markley(const std::vector<Quat>& samples);

void jacobiSym4(double A[4][4], double U[4][4]);

Quat quat_pow(const Quat& q, double t);

// formules §4040: сагиттальное L/R зеркало вращения (keep y, negate x,z).
inline Quat mirror_y_quat(const Quat& q) {
    return Quat(q.w, -q.x, q.y, -q.z);
}

constexpr double kSolverC1 = 272332.63;
constexpr double kSolverC2 = 40680634.23;
inline double solverRationalRatio(double x, double b) {
    const double radicand = kSolverC2 - kSolverC1 * b * b;
    if (radicand <= 0.0) return 1.0;
    const double s   = std::sqrt(radicand);
    const double den = x * s + (kSolverC2 - kSolverC1);
    if (std::abs(den) < 1e-12) return 1.0;
    return 1.0 + kSolverC1 / den;
}

inline double solverDirection(double x, double b, double d) {
    const double ratio = solverRationalRatio(x, b);
    const double denom = std::sqrt(b * b + d * d * ratio * ratio);
    return (denom < 1e-18) ? 0.0 : b / denom;
}

inline Quat hemisphereContinuous(const Quat& q, const Quat& prev) {
    const double dot = q.w*prev.w + q.x*prev.x + q.y*prev.y + q.z*prev.z;
    return (dot < 0.0) ? Quat(-q.w, -q.x, -q.y, -q.z) : q;
}

}
