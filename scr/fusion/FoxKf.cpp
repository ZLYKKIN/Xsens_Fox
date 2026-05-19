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

bool isFiniteQuat(const Quat4& q) {
    return std::isfinite(q[0]) && std::isfinite(q[1])
        && std::isfinite(q[2]) && std::isfinite(q[3]);
}
bool isFiniteVec(const Vec3& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
bool isFiniteCov(const float* P) {
    for (int i = 0; i < 36; ++i) if (!std::isfinite(P[i])) return false;
    return true;
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
    m_magResidLp      = 0.0f;
    m_magDisableTicks = 0;
    m_magDisabled     = false;
    m_bSnap           = Vec3{0.0f, 0.0f, 0.0f};
    m_framesSinceZupt = 0;
}

void FoxKf::setPrior(const Quat4& qWorldBody, const Vec3& biasInit,
                     float orientStdDeg, float biasStd) {
    Quat4 qNew = toQuat4(toQf(qWorldBody));
    Vec3  bNew = biasInit;
    if (!isFiniteQuat(qNew)) qNew = Quat4{1.0f, 0.0f, 0.0f, 0.0f};
    if (!isFiniteVec(bNew))  bNew = Vec3{0.0f, 0.0f, 0.0f};
    const float biasLim = m_set.maxBiasRadPerSec;
    bNew[0] = bNew[0] > biasLim ? biasLim : (bNew[0] < -biasLim ? -biasLim : bNew[0]);
    bNew[1] = bNew[1] > biasLim ? biasLim : (bNew[1] < -biasLim ? -biasLim : bNew[1]);
    bNew[2] = bNew[2] > biasLim ? biasLim : (bNew[2] < -biasLim ? -biasLim : bNew[2]);
    m_q = qNew;
    m_b = bNew;
    const float aDeg = (orientStdDeg < 0.0f) ? m_set.initOrientStdDeg : orientStdDeg;
    const float bStd = (biasStd      < 0.0f) ? m_set.initBiasStd      : biasStd;
    const float aRad = aDeg * kPiF / 180.0f;

    auto P = covView(m_P);
    P.setZero();
    P.diagonal().head<3>().setConstant(aRad * aRad);
    P.diagonal().tail<3>().setConstant(bStd * bStd);

    m_bSnap = m_b;
    m_framesSinceZupt = 0;
    m_magResidLp = 0.0f;
    m_magDisableTicks = 0;
    m_magDisabled = false;
}

void FoxKf::predict(const Vec3& gyrRadPerSec, float dt) {
    if (dt <= 0.0f) return;
    if (dt > 0.5f) dt = 0.5f;

    if (!isFiniteVec(gyrRadPerSec)) return;

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

    if (m_framesSinceZupt < 2000000000) ++m_framesSinceZupt;
    if (m_framesSinceZupt > m_set.biasAnchorFrames) {
        const float k = std::min(1.0f, m_set.biasAnchorRate * dt);
        m_b[0] = (1.0f - k) * m_b[0] + k * m_bSnap[0];
        m_b[1] = (1.0f - k) * m_b[1] + k * m_bSnap[1];
        m_b[2] = (1.0f - k) * m_b[2] + k * m_bSnap[2];
    }

    const float wmag = w.norm();
    const float aMag = toV3(m_lastAcc).norm();
    const bool stillNow = (wmag < m_set.zuptOmegaThresh)
                       && (std::fabs(aMag - 1.0f) < m_set.zuptAccThresh);
    m_stillTicks = stillNow ? std::min(m_stillTicks + 1, 100000) : 0;
    m_still = (m_stillTicks >= m_set.zuptHoldFrames);

    if (!isFiniteQuat(m_q) || !isFiniteVec(m_b) || !isFiniteCov(m_P)) {
        const FoxKfSettings s = m_set;
        initialise(s);
        ++m_autoResetCount;
    }
}

static inline float clampBiasRadPerSec(float v, float lim) {
    return v > lim ? lim : (v < -lim ? -lim : v);
}

void FoxKf::updateAcc(const Vec3& accUnitG) {
    if (!isFiniteVec(accUnitG)) return;
    m_lastAcc = accUnitG;
    const V3 a = toV3(accUnitG);
    const float mag = a.norm();
    const float err = std::fabs(mag - 1.0f);
    if (err > m_set.accRejectG * 2.0f) return;

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
    if (dTheta.norm() > m_set.dthetaSanityRad) return;
    m_q = toQuat4(expSO3(dTheta) * toQf(m_q));
    const float biasLim = m_set.maxBiasRadPerSec;
    m_b = Vec3{
        clampBiasRadPerSec(m_b[0] + dBias.x(), biasLim),
        clampBiasRadPerSec(m_b[1] + dBias.y(), biasLim),
        clampBiasRadPerSec(m_b[2] + dBias.z(), biasLim)
    };

    const Mat6 IKH = Mat6::Identity() - K * H;
    P = IKH * P * IKH.transpose() + rA * (K * K.transpose());
    P = 0.5f * (P + P.transpose().eval());

    if (!isFiniteQuat(m_q) || !isFiniteVec(m_b) || !isFiniteCov(m_P)) {
        const FoxKfSettings s = m_set;
        initialise(s);
        ++m_autoResetCount;
    }
}

void FoxKf::updateMag(const Vec3& magUnit) {
    if (!isFiniteVec(magUnit)) return;
    const V3 m = toV3(magUnit);
    const float mag = m.norm();
    const float err = std::fabs(mag - 1.0f);
    if (err > m_set.magRejectUnit * 2.0f) return;

    const float cD = std::cos(m_set.magDipRad);
    const float sD = std::sin(m_set.magDipRad);
    const V3 mRefWorld(cD, 0.0f, -sD);
    const V3 mBody = rotateInv(toQf(m_q), mRefWorld);
    const V3 innov = m - mBody;
    const float innovNorm = innov.norm();

    const float thresh = std::sin(m_set.magDisableResidualDeg * kPiF / 180.0f);
    if (m_magDisabled) {
        m_magResidLp = 0.95f * m_magResidLp + 0.05f * innovNorm;
        if (m_magResidLp < m_set.magReenableResidual) {
            m_magDisabled = false;
            m_magDisableTicks = 0;
        }
        return;
    }
    m_magResidLp = 0.9f * m_magResidLp + 0.1f * innovNorm;
    if (m_magResidLp > thresh) {
        ++m_magDisableTicks;
        if (m_magDisableTicks > m_set.magDisableHoldFrames) {
            m_magDisabled = true;
            return;
        }
    } else {
        m_magDisableTicks = std::max(0, m_magDisableTicks - 1);
    }

    float rScale = 1.0f + 9.0f * std::min(1.0f, err / std::max(1e-6f, m_set.magRejectUnit));
    const float rM = m_set.magNoiseStd * m_set.magNoiseStd * rScale;

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
    if (dTheta.norm() > m_set.dthetaSanityRad) return;
    m_q = toQuat4(expSO3(dTheta) * toQf(m_q));
    const float biasLim = m_set.maxBiasRadPerSec;
    m_b = Vec3{
        clampBiasRadPerSec(m_b[0] + dBias.x(), biasLim),
        clampBiasRadPerSec(m_b[1] + dBias.y(), biasLim),
        clampBiasRadPerSec(m_b[2] + dBias.z(), biasLim)
    };

    const Mat6 IKH = Mat6::Identity() - K * H;
    P = IKH * P * IKH.transpose() + rM * (K * K.transpose());
    P = 0.5f * (P + P.transpose().eval());

    if (!isFiniteQuat(m_q) || !isFiniteVec(m_b) || !isFiniteCov(m_P)) {
        const FoxKfSettings s = m_set;
        initialise(s);
        ++m_autoResetCount;
    }
}

void FoxKf::updateZupt() {
    const V3 innov = -toV3(m_lastGyrCorrected);
    const float r = m_set.gyroNoiseStd * m_set.gyroNoiseStd * 0.1f;

    Mat36 H = Mat36::Zero();
    H.block<3, 3>(0, 3) = -Mat3::Identity();

    auto P = covView(m_P);
    const Mat3 S = H * P * H.transpose() + r * Mat3::Identity();
    Eigen::FullPivLU<Mat3> lu(S);
    if (!lu.isInvertible()) return;
    const Mat3 Sinv = lu.inverse();
    const Mat63 K = P * H.transpose() * Sinv;

    const Vec6 dx = K * innov;
    const V3 dTheta = dx.head<3>();
    if (dTheta.norm() > m_set.dthetaSanityRad) return;
    m_q = toQuat4(expSO3(dTheta) * toQf(m_q));
    const float biasLim = m_set.maxBiasRadPerSec;
    m_b = Vec3{
        clampBiasRadPerSec(m_b[0] + dx[3], biasLim),
        clampBiasRadPerSec(m_b[1] + dx[4], biasLim),
        clampBiasRadPerSec(m_b[2] + dx[5], biasLim)
    };

    const Mat6 IKH = Mat6::Identity() - K * H;
    P = IKH * P * IKH.transpose() + r * (K * K.transpose());
    P = 0.5f * (P + P.transpose().eval());

    m_bSnap = m_b;
    m_framesSinceZupt = 0;

    if (!isFiniteQuat(m_q) || !isFiniteVec(m_b) || !isFiniteCov(m_P)) {
        const FoxKfSettings s = m_set;
        initialise(s);
        ++m_autoResetCount;
    }
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
