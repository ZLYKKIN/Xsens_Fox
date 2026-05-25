
#include "foxmath.h"

namespace fox {

// §2.1/§112 произведение Гамильтона q = a (x) b (композиция поворотов) (formules.txt)
Quat quat_mult(const Quat& a, const Quat& b)
{

    return Quat(
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w);
}

// §6/§113 поворот вектора кватернионом v' = v + 2w(qv x v) + 2(qv x (qv x v)) (formules.txt)
QVector3D vec_rotate(const QVector3D& v, const Quat& q)
{

    const QVector3D qv(float(q.x), float(q.y), float(q.z));
    const QVector3D t = 2.0f * QVector3D::crossProduct(qv, v);
    return v + float(q.w) * t + QVector3D::crossProduct(qv, t);
}

// §2137 swing-twist разложение: выделение вращения вокруг анатомической оси (formules.txt)
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

// §1/§124 кватернион поворота вокруг одной оси: q=(cos(ang/2), n*sin(ang/2)) (formules.txt)
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

// §124 углы Эйлера -> кватернион (композиция трёх осевых поворотов по seq) (formules.txt)
Quat euler_to_quat(double a, double b, double c, const char* seq)
{

    const Quat qa = axis_quat(seq[0], a);
    const Quat qb = axis_quat(seq[1], b);
    const Quat qc = axis_quat(seq[2], c);
    return quat_mult(quat_mult(qa, qb), qc).normalized();
}

// §2137 извлечение только рыскания (twist вокруг мировой оси Z) (formules.txt)
Quat yaw_only_quat(const Quat& q)
{
    Quat swing, twist;
    swingTwistDecompose(q, QVector3D(0.0f, 0.0f, 1.0f), swing, twist);
    double w = twist.w, z = twist.z;
    if (w < 0.0) { w = -w; z = -z; }
    const double n2 = w * w + z * z;
    if (n2 < 1e-12) return Quat(1, 0, 0, 0);
    const double n = 1.0 / std::sqrt(n2);
    return Quat(w * n, 0.0, 0.0, z * n);
}

// §8/§121/§2098 SLERP; приведение к ближнему полушарию (dot<0 -> инверсия b) (formules.txt)
Quat slerp_quat(const Quat& a, const Quat& b, double t)
{
    double dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    Quat b2 = b;
    if (dot < 0.0) {
        b2 = Quat(-b.w, -b.x, -b.y, -b.z);
        dot = -dot;
    }

    const double theta0 = clamp_acos(dot);
    const double sinT0  = std::sin(theta0);
    if (sinT0 < 6.1e-6) {  // §38 порог малого угла SLERP (1e-6..1e-5) -> линейная ветвь (formules.txt)
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

// §5/§2096 угол поворота кватерниона в градусах: theta = 2*acos(|w|)*180/pi (formules.txt)
double quat_angle_deg(const Quat& q)
{
    const double w = std::abs(q.w) > 1.0 ? 1.0 : std::abs(q.w);
    return 2.0 * std::acos(w) * 180.0 / M_PI;
}

// §5/§1276 экспонента: вектор поворота (rotvec) -> кватернион; малый угол th^2<1e-24 -> I (formules.txt)
Quat quat_exp_rotvec(double phix, double phiy, double phiz)
{

    const double th2 = phix*phix + phiy*phiy + phiz*phiz;
    if (th2 < 1e-24) return Quat(1, 0, 0, 0);
    const double th = std::sqrt(th2);
    const double half = 0.5 * th;
    const double s = std::sin(half) / th;
    return Quat(std::cos(half), s * phix, s * phiy, s * phiz);
}

// §5/§1277 логарифм: кватернион -> вектор поворота th = 2*atan2(|v|,w) (formules.txt)
QVector3D quat_log(const Quat& qin)
{
    const Quat q = (qin.w < 0.0)
        ? Quat(-qin.w, -qin.x, -qin.y, -qin.z) : qin;
    const double vn2 = q.x*q.x + q.y*q.y + q.z*q.z;
    if (vn2 < 1e-24) {

        const double s = (q.w >= 0.0) ? 2.0 : -2.0;
        return QVector3D(float(s*q.x), float(s*q.y), float(s*q.z));
    }
    const double vn = std::sqrt(vn2);
    const double th = 2.0 * std::atan2(vn, q.w);
    const double k  = th / vn;
    return QVector3D(float(k*q.x), float(k*q.y), float(k*q.z));
}

// §3/§2093 кватернион -> матрица поворота R(q) 3x3 (formules.txt)
Matrix3 quat_to_matrix(const Quat& q)
{

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

// §4/§114 матрица -> углы Эйлера (раскладка A; зажим asin через clamp_asin §115) (formules.txt)
Euler3 matrix_to_euler_A(const Matrix3& R)
{

    Euler3 e;
    e.e0 = std::atan2( R.m[7], R.m[4] );
    e.e1 = clamp_asin( -R.m[1] );
    e.e2 = std::atan2( R.m[2], R.m[0] );
    return e;
}

// §4/§114 матрица -> углы Эйлера (раскладка B, иной осевой порядок; зажим asin §115) (formules.txt)
Euler3 matrix_to_euler_B(const Matrix3& R)
{

    Euler3 e;
    e.e0 = std::atan2( -R.m[6], R.m[8] );
    e.e1 = clamp_asin( R.m[7] );
    e.e2 = std::atan2( -R.m[1], R.m[4] );
    return e;
}

// §123 Шеппард: матрица -> кватернион (выбор макс. из 4 ветвей t0..t3 для устойчивости) (formules.txt)
Quat matrix_to_quat_sheppard(const Matrix3& R)
{

    const double m00 = R.m[0], m01 = R.m[1], m02 = R.m[2];
    const double m10 = R.m[3], m11 = R.m[4], m12 = R.m[5];
    const double m20 = R.m[6], m21 = R.m[7], m22 = R.m[8];

    const double t0 = 1.0 + m00 + m11 + m22;
    const double t1 = 1.0 + m00 - m11 - m22;
    const double t2 = 1.0 - m00 + m11 - m22;
    const double t3 = 1.0 - m00 - m11 + m22;

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

// §2098 степень кватерниона q^t = exp(t*log(q)) (для SQUAD/интерполяции) (formules.txt)
Quat quat_pow(const Quat& q, double t)
{

    const QVector3D phi = quat_log(q);
    return quat_exp_rotvec(t * double(phi.x()), t * double(phi.y()), t * double(phi.z()));
}

// §1842 угловая скорость из дельта-кватерниона: omega = 2*asin(|v|)/(|v|*dt) * v (formules.txt)
QVector3D angular_velocity_from_quat(const Quat& dq, double dtSec)
{

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

// §1824 Якоби-вращения для симметричной 4x4 (собств. разложение матрицы M в методе Markley) (formules.txt)
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

// §1824 усреднение кватернионов (Markley): собств. вектор макс. собств. значения M=Σ q*q^T (formules.txt)
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

}
