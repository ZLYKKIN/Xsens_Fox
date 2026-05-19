// ============================================================================
//  FoxKf — Eigen-backed Multiplicative EKF for one Awinda tracker.
//  See FoxKf.h for the architecture overview and state/measurement equations.
//
//  All linalg goes through Eigen's fixed-size types (no heap, no Dynamic).
//  Covariance update is in Joseph form
//      P ← (I - K H) P (I - K H)ᵀ + K R Kᵀ
//  which is numerically stable under finite-precision arithmetic — it keeps
//  P symmetric positive-semidefinite for ANY K (not just the optimal one),
//  whereas the naive (I-KH)P form can drift to indefiniteness after enough
//  updates and silently break the filter.
// ============================================================================

#include "FoxKf.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace fox {

namespace {

using Mat6  = Eigen::Matrix<float, 6, 6>;
using Mat36 = Eigen::Matrix<float, 3, 6>;
using Mat63 = Eigen::Matrix<float, 6, 3>;
using Mat3  = Eigen::Matrix<float, 3, 3>;
using Vec6  = Eigen::Matrix<float, 6, 1>;
using V3    = Eigen::Vector3f;
using Qf    = Eigen::Quaternionf;

constexpr float kPiF = 3.14159265358979323846f;

V3   toV3 (const Vec3&  v) { return V3(v[0], v[1], v[2]); }
Vec3 toVec3(const V3&   v) { return Vec3{v.x(), v.y(), v.z()}; }
Qf   toQf (const Quat4& q) { return Qf(q[0], q[1], q[2], q[3]); }
Quat4 toQuat4(const Qf&  q) {
    Qf qn = q.normalized();
    return Quat4{qn.w(), qn.x(), qn.y(), qn.z()};
}

Mat3 skew(const V3& v) {
    Mat3 M;
    M <<    0.f, -v.z(),  v.y(),
          v.z(),    0.f, -v.x(),
         -v.y(),  v.x(),    0.f;
    return M;
}

Qf expSO3(const V3& w) {
    const float a = w.norm();
    if (a < 1e-8f) {
        return Qf(1.0f, 0.5f * w.x(), 0.5f * w.y(), 0.5f * w.z()).normalized();
    }
    const float h = 0.5f * a;
    const float s = std::sin(h) / a;
    return Qf(std::cos(h), w.x() * s, w.y() * s, w.z() * s).normalized();
}

V3 rotateInv(const Qf& q, const V3& v) {
    return q.conjugate() * v;
}

Eigen::Map<Mat6,       Eigen::Unaligned, Eigen::Stride<6, 1>>
covView(float* P) {
    return Eigen::Map<Mat6, Eigen::Unaligned, Eigen::Stride<6, 1>>(P);
}
Eigen::Map<const Mat6, Eigen::Unaligned, Eigen::Stride<6, 1>>
covView(const float* P) {
    return Eigen::Map<const Mat6, Eigen::Unaligned, Eigen::Stride<6, 1>>(P);
}

}  // namespace

FoxKf::FoxKf() { initialise(); }

void FoxKf::initialise(const FoxKfSettings& s) {
    m_set = s;
    m_q = Quat4{1.0f, 0.0f, 0.0f, 0.0f};
    m_b = Vec3{0.0f, 0.0f, 0.0f};

    auto P = covView(m_P);
    P.setZero();
    const float aRad = m_set.initOrientStdDeg * kPiF / 180.0f;
    const float a2 = aRad * aRad;
    const float b2 = m_set.initBiasStd * m_set.initBiasStd;
    P.diagonal().head<3>().setConstant(a2);
    P.diagonal().tail<3>().setConstant(b2);

    m_still = false;
    m_stillTicks = 0;
    m_lastGyrCorrected = Vec3{0.0f, 0.0f, 0.0f};
    m_lastAcc          = Vec3{0.0f, 0.0f, 0.0f};
}

void FoxKf::setPrior(const Quat4& qWorldBody, const Vec3& biasInit,
                     float orientStdDeg, float biasStd) {
    m_q = toQuat4(toQf(qWorldBody));
    m_b = biasInit;
    const float aDeg = (orientStdDeg < 0.0f) ? m_set.initOrientStdDeg : orientStdDeg;
    const float bStd = (biasStd      < 0.0f) ? m_set.initBiasStd      : biasStd;
    const float aRad = aDeg * kPiF / 180.0f;

    auto P = covView(m_P);
    P.setZero();
    P.diagonal().head<3>().setConstant(aRad * aRad);
    P.diagonal().tail<3>().setConstant(bStd * bStd);
}

void FoxKf::predict(const Vec3& gyrRadPerSec, float dt) {
    if (dt <= 0.0f) return;
    if (dt > 0.5f) dt = 0.5f;          // clamp pathological gaps

    const V3 w = toV3(gyrRadPerSec) - toV3(m_b);
    m_lastGyrCorrected = toVec3(w);

    const V3 wdt = w * dt;
    Qf q = toQf(m_q) * expSO3(wdt);
    m_q = toQuat4(q);

    Mat6 F = Mat6::Identity();
    F.block<3, 3>(0, 0) = Mat3::Identity() - skew(wdt);
    F.block<3, 3>(0, 3) = -Mat3::Identity() * dt;

    const float qg = m_set.gyroNoiseStd  * m_set.gyroNoiseStd  * dt * dt;
    const float qb = m_set.gyroBiasRwStd * m_set.gyroBiasRwStd * dt;

    auto P = covView(m_P);
    P = F * P * F.transpose();
    P.diagonal().head<3>().array() += qg;
    P.diagonal().tail<3>().array() += qb;
    P = 0.5f * (P + P.transpose().eval());

    const float wmag = w.norm();
    const float aMag = toV3(m_lastAcc).norm();
    const bool stillNow = (wmag < m_set.zuptOmegaThresh)
                       && (std::fabs(aMag - 1.0f) < m_set.zuptAccThresh);
    m_stillTicks = stillNow ? std::min(m_stillTicks + 1, 100000) : 0;
    m_still = (m_stillTicks >= m_set.zuptHoldFrames);
}

void FoxKf::updateAcc(const Vec3& accUnitG) {
    m_lastAcc = accUnitG;
    const V3 a = toV3(accUnitG);
    const float mag = a.norm();
    const float err = std::fabs(mag - 1.0f);
    if (err > m_set.accRejectG * 2.0f) return;           // hard skip on huge shock

    float rScale = 1.0f + 9.0f * std::min(1.0f, err / std::max(1e-6f, m_set.accRejectG));
    if (err > m_set.accRejectG) rScale *= 1.0f + (err - m_set.accRejectG) * 20.0f;
    const float rA = m_set.accNoiseStd * m_set.accNoiseStd * rScale;

    const V3 gravUp(0.0f, 0.0f, -1.0f);
    const V3 gBody = rotateInv(toQf(m_q), gravUp);
    const V3 innov = a - gBody;

    Mat36 H = Mat36::Zero();
    H.block<3, 3>(0, 0) = skew(gBody);

    auto P = covView(m_P);
    const Mat3 S = H * P * H.transpose() + rA * Mat3::Identity();
    Eigen::FullPivLU<Mat3> lu(S);
    if (!lu.isInvertible()) return;
    const Mat3 Sinv = lu.inverse();
    const Mat63 K = P * H.transpose() * Sinv;

    const Vec6 dx = K * innov;
    const V3 dTheta = dx.head<3>();
    const V3 dBias  = dx.tail<3>();
    m_q = toQuat4(expSO3(dTheta) * toQf(m_q));
    m_b = Vec3{m_b[0] + dBias.x(), m_b[1] + dBias.y(), m_b[2] + dBias.z()};

    const Mat6 IKH = Mat6::Identity() - K * H;
    P = IKH * P * IKH.transpose() + rA * (K * K.transpose());
    P = 0.5f * (P + P.transpose().eval());
}

void FoxKf::updateMag(const Vec3& magUnit) {
    const V3 m = toV3(magUnit);
    const float mag = m.norm();
    const float err = std::fabs(mag - 1.0f);
    if (err > m_set.magRejectUnit * 2.0f) return;

    float rScale = 1.0f + 9.0f * std::min(1.0f, err / std::max(1e-6f, m_set.magRejectUnit));
    const float rM = m_set.magNoiseStd * m_set.magNoiseStd * rScale;

    const float cD = std::cos(m_set.magDipRad);
    const float sD = std::sin(m_set.magDipRad);
    const V3 mRefWorld(cD, 0.0f, -sD);
    const V3 mBody = rotateInv(toQf(m_q), mRefWorld);
    const V3 innov = m - mBody;

    Mat36 H = Mat36::Zero();
    H.block<3, 3>(0, 0) = skew(mBody);

    auto P = covView(m_P);
    const Mat3 S = H * P * H.transpose() + rM * Mat3::Identity();
    Eigen::FullPivLU<Mat3> lu(S);
    if (!lu.isInvertible()) return;
    const Mat3 Sinv = lu.inverse();
    const Mat63 K = P * H.transpose() * Sinv;

    const Vec6 dx = K * innov;
    const V3 dTheta = dx.head<3>();
    const V3 dBias  = dx.tail<3>();
    m_q = toQuat4(expSO3(dTheta) * toQf(m_q));
    m_b = Vec3{m_b[0] + dBias.x(), m_b[1] + dBias.y(), m_b[2] + dBias.z()};

    const Mat6 IKH = Mat6::Identity() - K * H;
    P = IKH * P * IKH.transpose() + rM * (K * K.transpose());
    P = 0.5f * (P + P.transpose().eval());
}

void FoxKf::updateZupt() {
    const V3 innov = -toV3(m_lastGyrCorrected);
    const float r = m_set.gyroNoiseStd * m_set.gyroNoiseStd * 0.1f;  // tight measurement noise

    Mat36 H = Mat36::Zero();
    H.block<3, 3>(0, 3) = -Mat3::Identity();

    auto P = covView(m_P);
    const Mat3 S = H * P * H.transpose() + r * Mat3::Identity();
    Eigen::FullPivLU<Mat3> lu(S);
    if (!lu.isInvertible()) return;
    const Mat3 Sinv = lu.inverse();
    const Mat63 K = P * H.transpose() * Sinv;

    const Vec6 dx = K * innov;
    m_q = toQuat4(expSO3(dx.head<3>()) * toQf(m_q));
    m_b = Vec3{m_b[0] + dx[3], m_b[1] + dx[4], m_b[2] + dx[5]};

    const Mat6 IKH = Mat6::Identity() - K * H;
    P = IKH * P * IKH.transpose() + r * (K * K.transpose());
    P = 0.5f * (P + P.transpose().eval());
}

float FoxKf::orientStdDeg() const {
    const auto P = covView(m_P);
    const float varSum = P(0, 0) + P(1, 1) + P(2, 2);
    return std::sqrt(std::max(0.0f, varSum)) * 180.0f / kPiF;
}

float FoxKf::biasStd() const {
    const auto P = covView(m_P);
    const float varSum = P(3, 3) + P(4, 4) + P(5, 5);
    return std::sqrt(std::max(0.0f, varSum / 3.0f));
}

}  // namespace fox
