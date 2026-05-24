
#include "foxcoupling.h"

#include <algorithm>
#include <cmath>

namespace fox::coupling {

namespace {

constexpr int kSEG_Pelvis    = 0;
constexpr int kSEG_L5        = 1;
constexpr int kSEG_L3        = 2;
constexpr int kSEG_T12       = 3;
constexpr int kSEG_T8        = 4;
constexpr int kSEG_Neck      = 5;
constexpr int kSEG_Head      = 6;
constexpr int kSEG_RShoulder = 7;
constexpr int kSEG_RUpperArm = 8;
constexpr int kSEG_LShoulder = 11;
constexpr int kSEG_LUpperArm = 12;
constexpr int kSEG_RUpperLeg = 15;
constexpr int kSEG_RLowerLeg = 16;
constexpr int kSEG_RFoot     = 17;
constexpr int kSEG_RToe      = 18;
constexpr int kSEG_LUpperLeg = 19;
constexpr int kSEG_LLowerLeg = 20;
constexpr int kSEG_LFoot     = 21;
constexpr int kSEG_LToe      = 22;

constexpr double kR2D = fox::body::constants::kRad2Deg;
constexpr double kD2R = fox::body::constants::kDeg2Rad;

inline Quat canon(const Quat& q) {
    if (q.w < 0.0) return Quat(-q.w, -q.x, -q.y, -q.z);
    return q;
}

inline Quat arcAt(const Quat& qa, const Quat& qb, double t) {
    const Quat qRel = canon(quat_mult(qb, qa.conj()).normalized());
    const QVector3D phi = quat_log(qRel);
    const Quat dq = quat_exp_rotvec(t * double(phi.x()),
                                    t * double(phi.y()),
                                    t * double(phi.z()));
    return quat_mult(dq, qa).normalized();
}

}

Diagnostics& diagnostics() {
    thread_local Diagnostics d;
    return d;
}

void applySpineRhythm(std::array<Quat, fox::body::kSegmentCount>& orient)
{

    namespace fb = fox::body;
    const Quat qPel = orient[kSEG_Pelvis];
    const Quat qT8  = orient[kSEG_T8];

    const double sumC = fb::kCSpine[0] + fb::kCSpine[1] +
                        fb::kCSpine[2] + fb::kCSpine[3];
    if (sumC <= 1e-9) return;

    const double tL5  =  fb::kCSpine[0]                                       / sumC;
    const double tL3  = (fb::kCSpine[0] + fb::kCSpine[1])                     / sumC;
    const double tT12 = (fb::kCSpine[0] + fb::kCSpine[1] + fb::kCSpine[2])    / sumC;

    const Quat qRel = canon(quat_mult(qT8, qPel.conj()).normalized());
    const QVector3D phi = quat_log(qRel);
    const double cAxial = fb::kCSpine[4];

    auto positionAt = [&](double t) {
        const Quat dq = quat_exp_rotvec(t * double(phi.x()),
                                        t * double(phi.y()),
                                        t * cAxial * double(phi.z()));
        return quat_mult(dq, qPel).normalized();
    };
    orient[kSEG_L5]  = positionAt(tL5);
    orient[kSEG_L3]  = positionAt(tL3);
    orient[kSEG_T12] = positionAt(tT12);

    Diagnostics& d = diagnostics();
    d.spineFracL5  = tL5;
    d.spineFracL3  = tL3;
    d.spineFracT12 = tT12;
    d.spineFullDeg = std::sqrt(double(phi.x() * phi.x() + phi.y() * phi.y()
                                      + phi.z() * phi.z())) * kR2D;
}

void applyNeckRhythm(std::array<Quat, fox::body::kSegmentCount>& orient)
{
    namespace fb = fox::body;
    const Quat qT8   = orient[kSEG_T8];
    const Quat qHead = orient[kSEG_Head];

    const double sumC = fb::kCSpine[5] + fb::kCSpine[6];
    if (sumC <= 1e-9) return;
    const double tNeck = fb::kCSpine[5] / sumC;
    orient[kSEG_Neck] = arcAt(qT8, qHead, tNeck);

    if (std::abs(fb::kSpineNeck.cNeck - 1.0) > 1e-6) {
        const Quat qRel = canon(quat_mult(orient[kSEG_Neck], qT8.conj()).normalized());
        const QVector3D phi = quat_log(qRel);
        const Quat dq = quat_exp_rotvec(double(phi.x()),
                                        double(phi.y()),
                                        fb::kSpineNeck.cNeck * double(phi.z()));
        orient[kSEG_Neck] = quat_mult(dq, qT8).normalized();
    }
}

void applyPelvisTilt(std::array<Quat, fox::body::kSegmentCount>& orient)
{
    namespace fb = fox::body;
    const double frac  = fb::kCPelvis[0];
    const double scale = fb::kCPelvis[1];
    if (std::abs(frac) <= 1e-6) return;

    const Quat qPel = canon(orient[kSEG_Pelvis]);
    const QVector3D phiPel = quat_log(qPel);
    const double tiltDeg = std::abs(double(phiPel.y())) * kR2D;
    const double ramp    = (scale > 1e-6) ? std::min(1.0, tiltDeg / scale) : 1.0;
    const double frac_eff = frac * ramp;
    if (std::abs(frac_eff) <= 1e-9) return;
    const Quat dq = quat_exp_rotvec(0.0, frac_eff * double(phiPel.y()), 0.0);
    orient[kSEG_L5] = quat_mult(dq, orient[kSEG_L5]).normalized();
}

namespace {

double scapCEffAxis(double thetaAxisDeg, double cArmLow)
{
    namespace fb = fox::body;
    const double low  = fb::kScapHumThetaLowDeg;
    const double high = fb::kScapHumThetaHighDeg;
    const double a = std::abs(thetaAxisDeg);
    if (a <= low)  return cArmLow;
    if (a >= high) return fb::kCArms[2];
    const double t = (a - low) / (high - low);
    return cArmLow + t * (fb::kCArms[2] - cArmLow);
}

void applyOneScap(std::array<Quat, fox::body::kSegmentCount>& orient,
                  int shoulderSeg, int upperArmSeg, bool diagIsR)
{
    namespace fb = fox::body;

    const Quat qT8       = orient[kSEG_T8];
    const Quat qUpperArm = orient[upperArmSeg];

    const Quat qRelHumT8 = canon(quat_mult(qUpperArm, qT8.conj()).normalized());
    const QVector3D phi  = quat_log(qRelHumT8);
    const double cEffX   = scapCEffAxis(double(phi.x()) * kR2D, fb::kCArms[0]);
    const double cEffY   = scapCEffAxis(double(phi.y()) * kR2D, fb::kCArms[1]);
    const double thetaH  = std::sqrt(double(phi.x() * phi.x() + phi.y() * phi.y()
                                            + phi.z() * phi.z())) * kR2D;

    const Quat dqScapInT8 = quat_exp_rotvec(cEffX * double(phi.x()),
                                            cEffY * double(phi.y()),
                                            0.0);
    orient[shoulderSeg] = quat_mult(dqScapInT8, qT8).normalized();

    Diagnostics& d = diagnostics();

    const double cEffAvg = 0.5 * (cEffX + cEffY);
    if (diagIsR) { d.scapThetaRDeg = thetaH; d.scapCEffR = cEffAvg; }
    else         { d.scapThetaLDeg = thetaH; d.scapCEffL = cEffAvg; }
}

}

void applyScapuloHumeral(std::array<Quat, fox::body::kSegmentCount>& orient)
{
    applyOneScap(orient, kSEG_RShoulder, kSEG_RUpperArm, true);
    applyOneScap(orient, kSEG_LShoulder, kSEG_LUpperArm, false);
}

namespace {

void applyOneKnee(std::array<Quat, fox::body::kSegmentCount>& orient,
                  int upperLegSeg, int lowerLegSeg, bool isRight)
{
    namespace fb = fox::body;
    const Quat qU = orient[upperLegSeg];
    const Quat qL = orient[lowerLegSeg];
    const Quat qRel = canon(quat_mult(qL, qU.conj()).normalized());
    const QVector3D phi = quat_log(qRel);

    const double thetaKneeRad = std::abs(double(phi.y()));
    const double thetaScrewRad =
        fb::kCKnees[1] * (1.0 - std::cos(thetaKneeRad))
        * fb::kKneeScrewMaxDeg * kD2R;
    const double signedScrew = isRight ? thetaScrewRad : -thetaScrewRad;

    if (std::abs(signedScrew) < 1e-7) {
        Diagnostics& d = diagnostics();
        if (isRight) { d.kneeFlexRDeg = thetaKneeRad * kR2D; d.kneeScrewRDeg = 0.0; }
        else         { d.kneeFlexLDeg = thetaKneeRad * kR2D; d.kneeScrewLDeg = 0.0; }
        return;
    }

    const Quat dq(std::cos(0.5 * signedScrew), 0.0, 0.0, std::sin(0.5 * signedScrew));
    orient[lowerLegSeg] = quat_mult(orient[lowerLegSeg], dq).normalized();

    Diagnostics& d = diagnostics();
    if (isRight) {
        d.kneeFlexRDeg  = thetaKneeRad * kR2D;
        d.kneeScrewRDeg = signedScrew  * kR2D;
    } else {
        d.kneeFlexLDeg  = thetaKneeRad * kR2D;
        d.kneeScrewLDeg = signedScrew  * kR2D;
    }
}

}

void applyKneeScrewHome(std::array<Quat, fox::body::kSegmentCount>& orient)
{
    applyOneKnee(orient, kSEG_RUpperLeg, kSEG_RLowerLeg, true);
    applyOneKnee(orient, kSEG_LUpperLeg, kSEG_LLowerLeg, false);
}

namespace {

void applyOneAnkle(std::array<Quat, fox::body::kSegmentCount>& orient,
                   int lowerLegSeg, int footSeg, bool isRight)
{
    namespace fb = fox::body;
    const Quat qLL = orient[lowerLegSeg];
    const Quat qF  = orient[footSeg];

    const Quat qRel = canon(quat_mult(qF, qLL.conj()).normalized());
    const fox::Matrix3 R = fox::quat_to_matrix(qRel);
    const fox::Euler3 e  = fox::matrix_to_euler_B(R);

    double thetaPf = e.e1;
    double thetaEv = e.e2;
    const double thetaAx = e.e0;

    const double pfMax = fb::kCAnkles[1];
    const double pfMin = -fb::kAnkleDorsiLimitRad;
    const bool clamped = (thetaPf > pfMax) || (thetaPf < pfMin);
    if (clamped) thetaPf = std::clamp(thetaPf, pfMin, pfMax);

    thetaEv = fb::kCAnkles[0] * thetaEv + fb::kCAnkles[2] * std::sin(thetaPf);

    const double cP = std::cos(0.5 * thetaPf), sP = std::sin(0.5 * thetaPf);
    const double cE = std::cos(0.5 * thetaEv), sE = std::sin(0.5 * thetaEv);
    const double cA = std::cos(0.5 * thetaAx), sA = std::sin(0.5 * thetaAx);

    const Quat qZ(cA, 0.0, 0.0, sA);
    const Quat qX(cE, sE, 0.0, 0.0);
    const Quat qY(cP, 0.0, sP, 0.0);
    const Quat qRelNew = quat_mult(quat_mult(qZ, qX), qY).normalized();
    orient[footSeg] = quat_mult(qRelNew, qLL).normalized();

    Diagnostics& d = diagnostics();
    if (isRight) { d.anklePfRDeg = thetaPf * kR2D; d.ankleClampedR = clamped; }
    else         { d.anklePfLDeg = thetaPf * kR2D; d.ankleClampedL = clamped; }
}

}

void applyAnkleCoupling(std::array<Quat, fox::body::kSegmentCount>& orient)
{
    applyOneAnkle(orient, kSEG_RLowerLeg, kSEG_RFoot, true);
    applyOneAnkle(orient, kSEG_LLowerLeg, kSEG_LFoot, false);
}

ToeWeights computeToeRockerWeights(
    const std::array<Quat, fox::body::kSegmentCount>& orient)
{
    namespace fb = fox::body;
    auto toeAngleRad = [&](int footSeg, int toeSeg) -> double {
        const Quat qF = orient[footSeg];
        const Quat qT = orient[toeSeg];
        const Quat qRel = canon(quat_mult(qT, qF.conj()).normalized());
        const QVector3D phi = quat_log(qRel);

        return double(phi.y());
    };
    auto weights = [&](double thetaT) -> std::pair<double, double> {
        const double lo = fb::kToeRockerLowRad;
        const double hi = fb::kToeRockerHighRad;
        const double absT = std::abs(thetaT);
        if      (absT <= lo) return { 1.0, 0.0 };
        else if (absT >= hi) return { 0.0, 1.0 };
        const double t = (absT - lo) / (hi - lo);
        return { 1.0 - t, t };
    };

    const double thetaR = toeAngleRad(kSEG_RFoot, kSEG_RToe);
    const double thetaL = toeAngleRad(kSEG_LFoot, kSEG_LToe);
    auto wR = weights(thetaR);
    auto wL = weights(thetaL);

    Diagnostics& d = diagnostics();
    d.toeRDeg = thetaR * kR2D;
    d.toeLDeg = thetaL * kR2D;
    d.toeWeights = { wR.first, wR.second, wL.first, wL.second };

    return { wR.first, wR.second, wL.first, wL.second };
}

}
