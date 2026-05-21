// Unit tests for scr/foxmath.{h,cpp} — the pure rotation math that drives the
// whole pipeline.  Conventions verified against hipose/rotations.py and the MVN
// MXTP wire contract (WXYZ, Hamilton product, NWU right-handed world frame).
#include "foxmath.h"
#include "foxtest.h"

#include <limits>

using fox::Quat;

namespace {

constexpr double PI  = 3.14159265358979323846;
constexpr double EPS = 1e-9;

// Compare two quaternions as rotations (q and -q are the same rotation).
bool sameRotation(const Quat& a, const Quat& b, double eps = 1e-6)
{
    const double s = (a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z) < 0 ? -1.0 : 1.0;
    return std::fabs(a.w - s*b.w) < eps && std::fabs(a.x - s*b.x) < eps
        && std::fabs(a.y - s*b.y) < eps && std::fabs(a.z - s*b.z) < eps;
}

Quat qx(double a) { return fox::euler_to_quat(a, 0, 0, "XYZ"); }
Quat qy(double a) { return fox::euler_to_quat(0, a, 0, "XYZ"); }
Quat qz(double a) { return fox::euler_to_quat(0, 0, a, "XYZ"); }

void test_quat_mult()
{
    const Quat I;
    const Quat a(0.5, 0.5, 0.5, 0.5);                 // a valid unit quat
    CHECK(sameRotation(fox::quat_mult(a, I), a));
    CHECK(sameRotation(fox::quat_mult(I, a), a));

    // Qz(90°) ∘ Qz(90°) == Qz(180°)
    CHECK(sameRotation(fox::quat_mult(qz(PI/2), qz(PI/2)), qz(PI)));

    // Non-commutative: Qx ∘ Qy != Qy ∘ Qx
    CHECK(!sameRotation(fox::quat_mult(qx(PI/2), qy(PI/2)),
                        fox::quat_mult(qy(PI/2), qx(PI/2))));

    // Associativity
    const Quat b = qx(0.3), c = qy(0.7), d = qz(1.1);
    CHECK(sameRotation(fox::quat_mult(fox::quat_mult(b, c), d),
                       fox::quat_mult(b, fox::quat_mult(c, d))));
}

void test_normalize_inv_finite()
{
    CHECK(sameRotation(Quat(2, 0, 0, 0).normalized(), Quat(1, 0, 0, 0)));
    CHECK_NEAR(qz(0.9).normalized().norm(), 1.0, EPS);

    // Degenerate → identity
    CHECK(sameRotation(Quat(0, 0, 0, 0).normalized(), Quat(1, 0, 0, 0)));

    // *** The hardening under test: non-finite must collapse to identity. ***
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK(sameRotation(Quat(nan, 0, 0, 0).normalized(), Quat(1, 0, 0, 0)));
    CHECK(sameRotation(Quat(1, inf, 0, 0).normalized(), Quat(1, 0, 0, 0)));
    CHECK(sameRotation(Quat(0, 0, nan, -inf).normalized(), Quat(1, 0, 0, 0)));
    CHECK(sameRotation(Quat(nan, 0, 0, 0).inv(), Quat(1, 0, 0, 0)));
    CHECK(sameRotation(Quat(inf, inf, inf, inf).inv(), Quat(1, 0, 0, 0)));

    CHECK(Quat(1, 0, 0, 0).isFinite());
    CHECK(!Quat(nan, 0, 0, 0).isFinite());
    CHECK(!Quat(0, inf, 0, 0).isFinite());

    // q · q⁻¹ == identity for a unit quat
    const Quat q = qz(PI/2);
    CHECK(sameRotation(fox::quat_mult(q, q.inv()), Quat(1, 0, 0, 0)));
}

void test_vec_rotate()
{
    // +Z by +90° takes +X → +Y (right-handed)
    QVector3D r = fox::vec_rotate(QVector3D(1, 0, 0), qz(PI/2));
    CHECK_NEAR(r.x(), 0.0, 1e-6); CHECK_NEAR(r.y(), 1.0, 1e-6); CHECK_NEAR(r.z(), 0.0, 1e-6);

    // +Y by +90° takes +X → -Z
    r = fox::vec_rotate(QVector3D(1, 0, 0), qy(PI/2));
    CHECK_NEAR(r.x(), 0.0, 1e-6); CHECK_NEAR(r.y(), 0.0, 1e-6); CHECK_NEAR(r.z(), -1.0, 1e-6);

    // Identity leaves the vector unchanged
    r = fox::vec_rotate(QVector3D(3, -2, 5), Quat(1, 0, 0, 0));
    CHECK_NEAR(r.x(), 3.0, 1e-6); CHECK_NEAR(r.y(), -2.0, 1e-6); CHECK_NEAR(r.z(), 5.0, 1e-6);

    // Round-trip: rotate then inverse-rotate restores the vector
    const Quat q = fox::quat_mult(qz(0.6), qx(-0.4));
    const QVector3D v(0.3f, 1.7f, -0.9f);
    const QVector3D back = fox::vec_rotate(fox::vec_rotate(v, q), q.inv());
    CHECK_NEAR(back.x(), v.x(), 1e-5);
    CHECK_NEAR(back.y(), v.y(), 1e-5);
    CHECK_NEAR(back.z(), v.z(), 1e-5);
}

void test_euler_to_quat()
{
    // Single-axis sequences equal the bare axis rotation
    CHECK(sameRotation(fox::euler_to_quat(PI/2, 0, 0, "XYZ"), qx(PI/2)));
    CHECK(sameRotation(fox::euler_to_quat(0, 0, PI/2, "XYZ"), qz(PI/2)));

    // Intrinsic XYZ == Qx(a) ∘ Qy(b) ∘ Qz(c)
    const double a = 0.4, b = -0.8, c = 1.2;
    const Quat expect = fox::quat_mult(fox::quat_mult(qx(a), qy(b)), qz(c));
    CHECK(sameRotation(fox::euler_to_quat(a, b, c, "XYZ"), expect));

    // Output is always unit length
    CHECK_NEAR(fox::euler_to_quat(a, b, c, "ZYX").norm(), 1.0, EPS);
}

void test_swing_twist()
{
    const QVector3D axis(1, 0, 0);
    const Quat q = fox::quat_mult(qx(0.7), qy(0.5));   // twist about X + swing
    Quat swing, twist;
    fox::swingTwistDecompose(q, axis, swing, twist);

    // Reconstruction: q == swing ∘ twist
    CHECK(sameRotation(fox::quat_mult(swing, twist), q));
    // Twist is purely about the X axis (no Y/Z vector component)
    CHECK_NEAR(twist.y, 0.0, 1e-9);
    CHECK_NEAR(twist.z, 0.0, 1e-9);

    // Pure twist about the axis → swing ≈ identity
    fox::swingTwistDecompose(qx(0.9), axis, swing, twist);
    CHECK(sameRotation(swing, Quat(1, 0, 0, 0)));
    CHECK(sameRotation(twist, qx(0.9)));

    // Pure swing (rotation orthogonal to the axis) → twist ≈ identity
    fox::swingTwistDecompose(qy(0.6), axis, swing, twist);
    CHECK(sameRotation(twist, Quat(1, 0, 0, 0)));
    CHECK(sameRotation(swing, qy(0.6)));
}

void test_yaw_only()
{
    // A pure-yaw quat is returned unchanged
    CHECK(sameRotation(fox::yaw_only_quat(qz(0.7)), qz(0.7)));
    // Pure roll/pitch carries no yaw → identity
    CHECK(sameRotation(fox::yaw_only_quat(qx(0.5)), Quat(1, 0, 0, 0)));
    CHECK(sameRotation(fox::yaw_only_quat(qy(0.5)), Quat(1, 0, 0, 0)));
    // Result is always unit
    CHECK_NEAR(fox::yaw_only_quat(qz(2.0)).norm(), 1.0, EPS);
}

void test_slerp()
{
    const Quat a = Quat(1, 0, 0, 0), b = qz(PI/2);
    CHECK(sameRotation(fox::slerp_quat(a, b, 0.0), a));
    CHECK(sameRotation(fox::slerp_quat(a, b, 1.0), b));
    CHECK_NEAR(fox::quat_angle_deg(fox::slerp_quat(a, b, 0.5)), 45.0, 1e-4);

    // Opposite-hemisphere endpoint still interpolates the short way
    const Quat bNeg(-b.w, -b.x, -b.y, -b.z);
    CHECK(sameRotation(fox::slerp_quat(a, bNeg, 1.0), b));
    CHECK_NEAR(fox::slerp_quat(a, b, 0.3).norm(), 1.0, 1e-6);
}

void test_quat_angle_deg()
{
    CHECK_NEAR(fox::quat_angle_deg(Quat(1, 0, 0, 0)), 0.0, 1e-9);
    CHECK_NEAR(fox::quat_angle_deg(qz(PI/2)), 90.0, 1e-4);
    CHECK_NEAR(fox::quat_angle_deg(qx(PI)), 180.0, 1e-4);
}

void test_mirror_y()
{
    // mirror is a homomorphism: mirror(a∘b) == mirror(a)∘mirror(b)
    const Quat a = qz(0.5), b = qx(0.9);
    CHECK(sameRotation(fox::mirror_y_quat(fox::quat_mult(a, b)),
                       fox::quat_mult(fox::mirror_y_quat(a), fox::mirror_y_quat(b))));
    // A Z-rotation mirrored across the XZ-plane reverses sign
    CHECK(sameRotation(fox::mirror_y_quat(qz(0.7)), qz(-0.7)));
}

void test_hemisphere_continuous()
{
    const Quat prev(1, 0, 0, 0);
    const Quat q = qz(PI/2);
    const Quat qNeg(-q.w, -q.x, -q.y, -q.z);

    // Same hemisphere → unchanged
    Quat out = fox::hemisphereContinuous(q, prev);
    CHECK(out.w == q.w && out.z == q.z);
    // Opposite hemisphere → flipped back (still the same rotation as q)
    out = fox::hemisphereContinuous(qNeg, prev);
    CHECK(sameRotation(out, q));
    CHECK(out.w*prev.w + out.x*prev.x + out.y*prev.y + out.z*prev.z >= 0.0);
}

}  // namespace

int main()
{
    RUN(test_quat_mult);
    RUN(test_normalize_inv_finite);
    RUN(test_vec_rotate);
    RUN(test_euler_to_quat);
    RUN(test_swing_twist);
    RUN(test_yaw_only);
    RUN(test_slerp);
    RUN(test_quat_angle_deg);
    RUN(test_mirror_y);
    RUN(test_hemisphere_continuous);
    return fox_report("foxmath");
}
