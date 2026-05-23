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
    // Spec §8.1 — small-angle branch is gated on sin(Ω), not on dot itself.
    // sinT0 < 6.1e-6 implies Ω is small enough that sin(Ω)/Ω ≈ 1 and the
    // SLERP weights degenerate to plain lerp.  Computing it once up front
    // (rather than the looser dot > 0.9995 shortcut) matches the engine.
    const double theta0 = clamp_acos(dot);
    const double sinT0  = std::sin(theta0);
    if (sinT0 < 6.1e-6) {
        Quat r(a.w + t*(b2.w - a.w), a.x + t*(b2.x - a.x),
               a.y + t*(b2.y - a.y), a.z + t*(b2.z - a.z));
        return r.normalized();
    }
    const double theta  = theta0 * t;
    const double s0 = std::sin(theta0 - theta) / sinT0;
    const double s1 = std::sin(theta) / sinT0;
    return Quat(s0*a.w + s1*b2.w, s0*a.x + s1*b2.x,
                s0*a.y + s1*b2.y, s0*a.z + s1*b2.z).normalized();
}

double quat_angle_deg(const Quat& q)
{
    const double w = std::abs(q.w) > 1.0 ? 1.0 : std::abs(q.w);
    return 2.0 * std::acos(w) * 180.0 / M_PI;
}

// ============================================================================
//  Spec primitives — see header for spec section references.
// ============================================================================

Quat quat_exp_rotvec(double phix, double phiy, double phiz)
{
    // Spec §5.1: θ = ‖φ‖; q.xyz = (φ/θ)·sin(θ/2); q.w = cos(θ/2).  Limit at
    // θ→0 falls out of the Taylor series sin(θ/2)/θ ≈ 1/2, so the «small-angle»
    // branch is just the limit value 0.5 with no division.
    const double th2 = phix*phix + phiy*phiy + phiz*phiz;
    if (th2 < 1e-24) return Quat(1, 0, 0, 0);
    const double th = std::sqrt(th2);
    const double half = 0.5 * th;
    const double s = std::sin(half) / th;   // = sin(θ/2) / θ
    return Quat(std::cos(half), s * phix, s * phiy, s * phiz);
}

QVector3D quat_log(const Quat& q)
{
    // Spec §5.2: θ = 2·atan2(‖v‖, w), n = v/‖v‖, φ = θ·n.  For q close to
    // identity ‖v‖ ≈ 0 and the limit is φ = 2·v (Taylor: atan2(x,1) ≈ x).
    const double vn2 = q.x*q.x + q.y*q.y + q.z*q.z;
    if (vn2 < 1e-24) {
        // Small-angle: φ = 2·v (sign of w handled by upstream hemisphere fix).
        const double s = (q.w >= 0.0) ? 2.0 : -2.0;
        return QVector3D(float(s*q.x), float(s*q.y), float(s*q.z));
    }
    const double vn = std::sqrt(vn2);
    const double th = 2.0 * std::atan2(vn, q.w);
    const double k  = th / vn;
    return QVector3D(float(k*q.x), float(k*q.y), float(k*q.z));
}

Matrix3 quat_to_matrix(const Quat& q)
{
    // Spec §3.1 / fox_types_engine §34.6 — full Kayley form on the diagonal.
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    const double ww=w*w, xx=x*x, yy=y*y, zz=z*z;
    const double xy=x*y, xz=x*z, yz=y*z;
    const double wx=w*x, wy=w*y, wz=w*z;
    Matrix3 R;
    R.m[0] = ww + xx - yy - zz;   R.m[1] = 2.0 * (xy - wz);     R.m[2] = 2.0 * (xz + wy);
    R.m[3] = 2.0 * (xy + wz);     R.m[4] = ww - xx + yy - zz;   R.m[5] = 2.0 * (yz - wx);
    R.m[6] = 2.0 * (xz - wy);     R.m[7] = 2.0 * (yz + wx);     R.m[8] = ww - xx - yy + zz;
    return R;
}

Euler3 matrix_to_euler_A(const Matrix3& R)
{
    // Spec §4.3 variant A: middle = asin(-m01), first = atan2(m21,m11), second = atan2(m02,m00).
    Euler3 e;
    e.e0 = std::atan2( R.m[7], R.m[4] );        // atan2(m21, m11)
    e.e1 = clamp_asin( -R.m[1] );               // asin(-m01)
    e.e2 = std::atan2( R.m[2], R.m[0] );        // atan2(m02, m00)
    return e;
}

Euler3 matrix_to_euler_B(const Matrix3& R)
{
    // Spec §4.3 variant B: middle = asin(m21), first = atan2(-m20,m22), second = atan2(-m01,m11).
    Euler3 e;
    e.e0 = std::atan2( -R.m[6], R.m[8] );       // atan2(-m20, m22)
    e.e1 = clamp_asin( R.m[7] );                // asin(m21)
    e.e2 = std::atan2( -R.m[1], R.m[4] );       // atan2(-m01, m11)
    return e;
}

Quat matrix_to_quat_sheppard(const Matrix3& R)
{
    // Spec §3.2 — Shepperd's algorithm with the largest-trace pivot.  The
    // four candidates t0..t3 equal 4·w², 4·x², 4·y², 4·z² respectively;
    // picking the largest keeps the divisor 2·√(t_k) far from zero, which
    // is what the naive form (just w² from the trace) gets wrong when w ≈ 0.
    const double m00 = R.m[0], m01 = R.m[1], m02 = R.m[2];
    const double m10 = R.m[3], m11 = R.m[4], m12 = R.m[5];
    const double m20 = R.m[6], m21 = R.m[7], m22 = R.m[8];

    const double t0 = 1.0 + m00 + m11 + m22;   // 4w²
    const double t1 = 1.0 + m00 - m11 - m22;   // 4x²
    const double t2 = 1.0 - m00 + m11 - m22;   // 4y²
    const double t3 = 1.0 - m00 - m11 + m22;   // 4z²

    Quat q;
    if (t0 >= t1 && t0 >= t2 && t0 >= t3) {
        const double s = 0.5 / std::sqrt(std::max(t0, 1e-24));
        q.w = 0.25 / s;
        q.x = (m21 - m12) * s;
        q.y = (m02 - m20) * s;
        q.z = (m10 - m01) * s;
    } else if (t1 >= t2 && t1 >= t3) {
        const double s = 0.5 / std::sqrt(std::max(t1, 1e-24));
        q.w = (m21 - m12) * s;
        q.x = 0.25 / s;
        q.y = (m01 + m10) * s;
        q.z = (m02 + m20) * s;
    } else if (t2 >= t3) {
        const double s = 0.5 / std::sqrt(std::max(t2, 1e-24));
        q.w = (m02 - m20) * s;
        q.x = (m01 + m10) * s;
        q.y = 0.25 / s;
        q.z = (m12 + m21) * s;
    } else {
        const double s = 0.5 / std::sqrt(std::max(t3, 1e-24));
        q.w = (m10 - m01) * s;
        q.x = (m02 + m20) * s;
        q.y = (m12 + m21) * s;
        q.z = 0.25 / s;
    }
    if (q.w < 0.0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
    return q.normalized();
}

Quat quat_pow(const Quat& q, double t)
{
    // Spec §40 — q^t = exp(t · log(q)).  Identity is the unique fixed point
    // (any t leaves it unchanged); for non-identity q the small-angle path
    // in quat_log/quat_exp_rotvec handles the limit.
    const QVector3D phi = quat_log(q);
    return quat_exp_rotvec(t * double(phi.x()), t * double(phi.y()), t * double(phi.z()));
}

QVector3D angular_velocity_from_quat(const Quat& dq, double dtSec)
{
    // Spec §12.2: ω = (2·asin(‖v‖) / (‖v‖·Δt)) · v,  v = (dq.x, dq.y, dq.z).
    // Hemisphere canonicalisation [1.3]: q and −q describe the same rotation,
    // but only the w≥0 representative gives the shortest-path half-angle
    // through asin (which is monotone on [0, π/2]).  Without it, rotations
    // close to ±π flip sign and the velocity vector reverses.
    if (dtSec <= 0.0) return QVector3D(0, 0, 0);
    double x = dq.x, y = dq.y, z = dq.z;
    if (dq.w < 0.0) { x = -x; y = -y; z = -z; }
    const double vn2 = x*x + y*y + z*z;
    if (vn2 < 1e-24) {
        const double k = 2.0 / dtSec;
        return QVector3D(float(k*x), float(k*y), float(k*z));
    }
    const double vn = std::sqrt(vn2);
    const double k  = 2.0 * clamp_asin(vn) / (vn * dtSec);
    return QVector3D(float(k*x), float(k*y), float(k*z));
}

// Spec §174.2 — Jacobi eigendecomposition of 4×4 symmetric.  Cyclic
// sweeps over off-diagonals; converges in ~6–8 sweeps for the matrix
// sizes we feed it.  Modifies A in place (eigenvalues on diagonal);
// U receives the eigenvector columns.
void jacobiSym4(double A[4][4], double U[4][4])
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            U[i][j] = (i == j) ? 1.0 : 0.0;

    for (int sweep = 0; sweep < 60; ++sweep) {
        int p = 0, q = 1;
        double maxOff = std::abs(A[0][1]);
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                if (std::abs(A[i][j]) > maxOff) {
                    maxOff = std::abs(A[i][j]); p = i; q = j;
                }
            }
        }
        if (maxOff < 1e-14) break;

        const double app = A[p][p];
        const double aqq = A[q][q];
        const double apq = A[p][q];
        double t;
        if (std::abs(apq) < 1e-300) { t = 0.0; }
        else {
            const double theta = (aqq - app) / (2.0 * apq);
            t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta*theta + 1.0));
        }
        const double c = 1.0 / std::sqrt(t*t + 1.0);
        const double s = t * c;

        A[p][p] = app - t * apq;
        A[q][q] = aqq + t * apq;
        A[p][q] = A[q][p] = 0.0;
        for (int k = 0; k < 4; ++k) {
            if (k == p || k == q) continue;
            const double akp = A[k][p];
            const double akq = A[k][q];
            A[k][p] = A[p][k] = c * akp - s * akq;
            A[k][q] = A[q][k] = s * akp + c * akq;
        }
        for (int k = 0; k < 4; ++k) {
            const double ukp = U[k][p];
            const double ukq = U[k][q];
            U[k][p] = c * ukp - s * ukq;
            U[k][q] = s * ukp + c * ukq;
        }
    }
}

// Spec §174.2 — Markley/Shepard 4-D eigenvector quaternion average.
Quat quat_avg_markley(const std::vector<Quat>& samples)
{
    if (samples.empty()) return Quat(1, 0, 0, 0);
    double M[4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
    for (const Quat& q : samples) {
        const double v[4] = { q.w, q.x, q.y, q.z };
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                M[i][j] += v[i] * v[j];
    }
    double U[4][4];
    jacobiSym4(M, U);
    int kmax = 0;
    for (int k = 1; k < 4; ++k)
        if (M[k][k] > M[kmax][kmax]) kmax = k;
    Quat q(U[0][kmax], U[1][kmax], U[2][kmax], U[3][kmax]);
    if (q.w < 0.0) q = Quat(-q.w, -q.x, -q.y, -q.z);
    return q.normalized();
}

}  // namespace fox
