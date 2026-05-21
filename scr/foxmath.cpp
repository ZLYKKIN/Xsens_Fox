// Fox Mocap — pure rotation math implementations.  Moved verbatim from main.cpp
// (no logic change) so the same code that drives the live pipeline is what the
// unit tests exercise.
#include "foxmath.h"

namespace fox {

Quat quat_mult(const Quat& a, const Quat& b)
{
    // Hamilton product: matches scipy.spatial.transform.Rotation composition.
    return Quat(
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w);
}

QVector3D vec_rotate(const QVector3D& v, const Quat& q)
{
    // Efficient formulation: v' = v + 2w(qv × v) + 2(qv × (qv × v))
    const QVector3D qv(float(q.x), float(q.y), float(q.z));
    const QVector3D t = 2.0f * QVector3D::crossProduct(qv, v);
    return v + float(q.w) * t + QVector3D::crossProduct(qv, t);
}

void swingTwistDecompose(const Quat& q, const QVector3D& axisU,
                         Quat& outSwing, Quat& outTwist)
{
    const double dot = q.x * double(axisU.x()) + q.y * double(axisU.y()) + q.z * double(axisU.z());
    Quat twist(q.w, dot * double(axisU.x()), dot * double(axisU.y()), dot * double(axisU.z()));
    const double n2 = twist.w * twist.w + twist.x * twist.x + twist.y * twist.y + twist.z * twist.z;
    if (n2 < 1e-12) {
        outTwist = Quat(1, 0, 0, 0);
        outSwing = q;
        return;
    }
    const double inv = 1.0 / std::sqrt(n2);
    twist.w *= inv; twist.x *= inv; twist.y *= inv; twist.z *= inv;
    if (twist.w < 0) {
        twist.w = -twist.w; twist.x = -twist.x; twist.y = -twist.y; twist.z = -twist.z;
    }
    outTwist = twist;
    outSwing = quat_mult(q, twist.inv()).normalized();
}

static Quat axis_quat(char axis, double ang)
{
    const double h = ang * 0.5;
    const double c = std::cos(h);
    const double s = std::sin(h);
    switch (axis) {
        case 'X': case 'x': return Quat(c, s, 0, 0);
        case 'Y': case 'y': return Quat(c, 0, s, 0);
        case 'Z': case 'z': return Quat(c, 0, 0, s);
    }
    return Quat(1, 0, 0, 0);
}

Quat euler_to_quat(double a, double b, double c, const char* seq)
{
    // Intrinsic rotations (uppercase scipy convention).
    // "XYZ" means: first rotate by a about local X, then b about new Y,
    //              then c about newest Z.  As a Hamilton product this is
    //              Qx(a) * Qy(b) * Qz(c).
    const Quat qa = axis_quat(seq[0], a);
    const Quat qb = axis_quat(seq[1], b);
    const Quat qc = axis_quat(seq[2], c);
    return quat_mult(quat_mult(qa, qb), qc).normalized();
}

Quat yaw_only_quat(const Quat& q)
{
    double w = q.w, z = q.z;
    // Hemisphere fix — берём ветвь с положительным w для непрерывности.
    if (w < 0.0) { w = -w; z = -z; }
    const double n2 = w*w + z*z;
    if (n2 < 1e-12) {
        // Оба компонента нулевые: yaw неопределён, возвращаем identity.
        return Quat(1, 0, 0, 0);
    }
    const double n = 1.0 / std::sqrt(n2);
    return Quat(w * n, 0.0, 0.0, z * n);
}

Quat slerp_quat(const Quat& a, const Quat& b, double t)
{
    double dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    Quat b2 = b;
    if (dot < 0.0) {
        b2 = Quat(-b.w, -b.x, -b.y, -b.z);
        dot = -dot;
    }
    if (dot > 0.9995) {
        Quat r(a.w + t*(b2.w - a.w), a.x + t*(b2.x - a.x),
               a.y + t*(b2.y - a.y), a.z + t*(b2.z - a.z));
        return r.normalized();
    }
    const double theta0 = std::acos(dot);
    const double theta  = theta0 * t;
    const double sinT0  = std::sin(theta0);
    const double s0 = std::sin(theta0 - theta) / sinT0;
    const double s1 = std::sin(theta) / sinT0;
    return Quat(s0*a.w + s1*b2.w, s0*a.x + s1*b2.x,
                s0*a.y + s1*b2.y, s0*a.z + s1*b2.z).normalized();
}

double quat_angle_deg(const Quat& q)
{
    const double w = std::abs(q.w) > 1.0 ? 1.0 : std::abs(q.w);
    return 2.0 * std::acos(w) * 180.0 / 3.14159265358979323846;
}

}  // namespace fox
