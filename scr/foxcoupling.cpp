// Fox Mocap — biomechanical post-FK coupling implementation (spec §40 / §45–§48).
//
// Every law here is the closed-form deterministic version of what the
// FOX_FE weighted-MNK solver applies as Jacobian rows in §44.3 В.  The
// numeric coefficients all come from foxbody.h (see header for the list);
// the algorithms are §45.2, §46.2, §47.2, §48.1, §48.2 verbatim.
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

// Hemisphere-canonicalise a quaternion (w ≥ 0) — required before
// quat_log / fractional power.  Spec §1.3.
inline Quat canon(const Quat& q) {
    if (q.w < 0.0) return Quat(-q.w, -q.x, -q.y, -q.z);
    return q;
}

// Place `q` on the great-circle arc from `qa` to `qb` at fraction `t`.
// Equivalent to `qa ⊗ exp(t · log(qa⁻¹ ⊗ qb))` and well-defined for
// the small-angle limit (foxmath handles both branches).
inline Quat arcAt(const Quat& qa, const Quat& qb, double t) {
    const Quat qRel = canon(quat_mult(qb, qa.conj()).normalized());
    const QVector3D phi = quat_log(qRel);
    const Quat dq = quat_exp_rotvec(t * double(phi.x()),
                                    t * double(phi.y()),
                                    t * double(phi.z()));
    return quat_mult(dq, qa).normalized();
}

// thread-local diagnostic snapshot (header type Diagnostics).
}  // namespace

Diagnostics& diagnostics() {
    thread_local Diagnostics d;
    return d;
}

// ----------------------------------------------------------------------------
//  Spec §45 — spine rhythm: Pelvis → T8 redistributed across L5 / L3 / T12.
//  Pelvis and T8 are measured IMUs (kSensorPresent); L5, L3, T12 are not.
// ----------------------------------------------------------------------------
void applySpineRhythm(std::array<Quat, fox::body::kSegmentCount>& orient)
{
    // Spec §45.5 + §45.1 — per-joint angular fractions of the Pelvis→T8
    // rotation, with c_spine[4] = 0.35 attenuating the lumbar AXIAL TWIST.
    //
    //     theta_full   = log( qT8 ⊗ conj(qPelvis) )
    //     theta_j (X,Y) = (c_spine[j] / W_sum) · theta_full.(X,Y)
    //     theta_j (Z)   = (c_spine[j] / W_sum) · c_axial · theta_full.Z
    //     q_j           = exp(theta_j) ⊗ qPelvis
    //
    // c_axial = c_spine[4] = 0.35 is the lumbar-axial-rotation stiffness
    // (spec §45.1 "особый коэффициент, вероятно твист/осевая ротация";
    // matches kSpineNeck.cNeck = 0.35 used on the cervical chain).  This
    // is what stops a yawed-T8 IMU from dragging the lumbar segments with
    // it 1:1; the spine in vivo resists axial torque at the L5/L3/T12 joints.
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
    const double cAxial = fb::kCSpine[4];           // 0.35 (spec §45.1)

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

// ----------------------------------------------------------------------------
//  Spec §45 + §45.1 spineNeck — Neck placed on the T8→Head arc at
//  kCSpine[5] / (kCSpine[5] + kCSpine[6]).  Numerically both are 0.9,
//  giving Neck exactly half-way; kSpineNeck.cNeck = 0.35 modulates the
//  axial twist (Z component) separately.
// ----------------------------------------------------------------------------
void applyNeckRhythm(std::array<Quat, fox::body::kSegmentCount>& orient)
{
    namespace fb = fox::body;
    const Quat qT8   = orient[kSEG_T8];
    const Quat qHead = orient[kSEG_Head];

    const double sumC = fb::kCSpine[5] + fb::kCSpine[6];
    if (sumC <= 1e-9) return;
    const double tNeck = fb::kCSpine[5] / sumC;        // 0.5 for {0.9, 0.9}
    orient[kSEG_Neck] = arcAt(qT8, qHead, tNeck);

    // Axial-twist attenuation per kSpineNeck.cNeck = 0.35 — the cervical
    // joint is stiff in axial rotation, so the Z component of the Neck
    // tilt receives only 35 % of the great-circle solution.
    if (std::abs(fb::kSpineNeck.cNeck - 1.0) > 1e-6) {
        const Quat qRel = canon(quat_mult(orient[kSEG_Neck], qT8.conj()).normalized());
        const QVector3D phi = quat_log(qRel);
        const Quat dq = quat_exp_rotvec(double(phi.x()),
                                        double(phi.y()),
                                        fb::kSpineNeck.cNeck * double(phi.z()));
        orient[kSEG_Neck] = quat_mult(dq, qT8).normalized();
    }
}

// ----------------------------------------------------------------------------
//  Spec §45.3 — pelvic tilt transmission: c_pelvis[0] = 0.35 fraction of
//  the pelvis sagittal tilt is forwarded into L5.  Operates in the
//  pelvis-relative frame to avoid double-counting yaw.
// ----------------------------------------------------------------------------
void applyPelvisTilt(std::array<Quat, fox::body::kSegmentCount>& orient)
{
    namespace fb = fox::body;
    const double frac = fb::kCPelvis[0];        // = 0.35
    if (std::abs(frac) <= 1e-6) return;

    // Pelvis tilt = log(qPelvis) → its Y component is the sagittal tilt
    // angle (about the body anterior-posterior axis).  We add fraction *
    // tilt to L5 along the Y axis only.
    const Quat qPel = canon(orient[kSEG_Pelvis]);
    const QVector3D phiPel = quat_log(qPel);
    const Quat dq = quat_exp_rotvec(0.0, frac * double(phiPel.y()), 0.0);
    orient[kSEG_L5] = quat_mult(dq, orient[kSEG_L5]).normalized();
}

// ----------------------------------------------------------------------------
//  Spec §46 — scapulo-humeral rhythm.  c_eff is piecewise-linear in
//  humerus elevation, applied to the abduction (X) and flexion (Y)
//  components of the shoulder→upper-arm relative orientation.
// ----------------------------------------------------------------------------
namespace {

double scapCEff(double thetaHumerusDeg)
{
    namespace fb = fox::body;
    const double low  = fb::kScapHumThetaLowDeg;     // 60
    const double high = fb::kScapHumThetaHighDeg;    // 90
    if (thetaHumerusDeg <= low)  return fb::kCArms[0];        // 0.95
    if (thetaHumerusDeg >= high) return fb::kCArms[2];        // 0.99
    const double t = (thetaHumerusDeg - low) / (high - low);
    return fb::kCArms[0] + t * (fb::kCArms[2] - fb::kCArms[0]);
}

void applyOneScap(std::array<Quat, fox::body::kSegmentCount>& orient,
                  int shoulderSeg, int upperArmSeg, bool diagIsR)
{
    // Spec §46.2 — scapulo-humeral rhythm.
    //
    //     theta_humerus_total = log( qUpperArm ⊗ conj(qT8) )      ← T8 reference
    //     theta_scapula       = c_eff(theta_humerus_total) · theta_humerus_total
    //     qScapula            = exp(theta_scapula) ⊗ qT8           ← absolute
    //
    // The spec phrases the rhythm in the T8 (sternum) frame, NOT in the
    // gleno-humeral relative.  At physiological elevation c_eff is 0.95–0.99,
    // so the scapula picks up the bulk of the elevation and the gleno-humeral
    // joint (humerus relative to scapula) only contributes the residual ~5 %.
    // The UpperArm world orientation is the direct IMU measurement and is
    // preserved.  Only the (X, Y) elevation components couple — axial humerus
    // twist (Z) stays with the humerus, not the scapula (clinical convention).
    const Quat qT8       = orient[kSEG_T8];
    const Quat qUpperArm = orient[upperArmSeg];

    const Quat qRelHumT8 = canon(quat_mult(qUpperArm, qT8.conj()).normalized());
    const QVector3D phi  = quat_log(qRelHumT8);
    const double thetaH  = std::sqrt(double(phi.x() * phi.x() + phi.y() * phi.y()
                                            + phi.z() * phi.z())) * kR2D;
    const double cEff    = scapCEff(thetaH);

    // Scapula in T8 frame = c_eff × humerus elevation (X, Y); zero axial twist.
    const Quat dqScapInT8 = quat_exp_rotvec(cEff * double(phi.x()),
                                            cEff * double(phi.y()),
                                            0.0);
    orient[shoulderSeg] = quat_mult(dqScapInT8, qT8).normalized();
    // UpperArm world orientation unchanged — direct IMU measurement is the
    // ground truth; the scapula is the constrained quantity.
    // (orient[upperArmSeg] left alone)

    Diagnostics& d = diagnostics();
    if (diagIsR) { d.scapThetaRDeg = thetaH; d.scapCEffR = cEff; }
    else         { d.scapThetaLDeg = thetaH; d.scapCEffL = cEff; }
}

}  // namespace

void applyScapuloHumeral(std::array<Quat, fox::body::kSegmentCount>& orient)
{
    applyOneScap(orient, kSEG_RShoulder, kSEG_RUpperArm, /*diagIsR=*/true);
    applyOneScap(orient, kSEG_LShoulder, kSEG_LUpperArm, /*diagIsR=*/false);
}

// ----------------------------------------------------------------------------
//  Spec §47.2 — knee screw-home: tibia rotates outward about its long
//  axis as a function of flexion angle.  Sign mirrored on the left leg.
// ----------------------------------------------------------------------------
namespace {

void applyOneKnee(std::array<Quat, fox::body::kSegmentCount>& orient,
                  int upperLegSeg, int lowerLegSeg, bool isRight)
{
    namespace fb = fox::body;
    const Quat qU = orient[upperLegSeg];
    const Quat qL = orient[lowerLegSeg];
    const Quat qRel = canon(quat_mult(qL, qU.conj()).normalized());
    const QVector3D phi = quat_log(qRel);

    // Flexion magnitude = |phi.y| (Y is the bending axis at the knee).
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

    // Add the screw rotation about the local Z axis of the lower-leg.
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

}  // namespace

void applyKneeScrewHome(std::array<Quat, fox::body::kSegmentCount>& orient)
{
    applyOneKnee(orient, kSEG_RUpperLeg, kSEG_RLowerLeg, /*isRight=*/true);
    applyOneKnee(orient, kSEG_LUpperLeg, kSEG_LLowerLeg, /*isRight=*/false);
}

// ----------------------------------------------------------------------------
//  Spec §48.1 — ankle plantar/dorsi-flexion clamp + eversion coupling.
//  Uses matrix-Euler variant B (spec §4.3) so the middle axis carries the
//  flexion component, matching the foxergo handlerFoot decomposition.
// ----------------------------------------------------------------------------
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
    //  e.e1 = asin(m21)              → flexion (Y axis, plantar/dorsi)
    //  e.e2 = atan2(-m01, m11)       → eversion (X axis)
    //  e.e0 = atan2(-m20, m22)       → axial   (Z axis)
    double thetaPf = e.e1;            // signed: + plantar, − dorsi
    double thetaEv = e.e2;
    const double thetaAx = e.e0;

    const double pfMax = fb::kCAnkles[1];                   //  +30°  (π/6)
    const double pfMin = -fb::kAnkleDorsiLimitRad;          //  -45°  (-π/4)
    const bool clamped = (thetaPf > pfMax) || (thetaPf < pfMin);
    if (clamped) thetaPf = std::clamp(thetaPf, pfMin, pfMax);

    // Eversion coupling: θ_ev' = c_ankles[0]·θ_ev + c_ankles[2]·sin(θ_pf).
    thetaEv = fb::kCAnkles[0] * thetaEv + fb::kCAnkles[2] * std::sin(thetaPf);

    // Recompose via Euler-B inverse: build R from (axial, flex, ev) and
    // convert back to a quaternion through the standard X-Y-Z chain.
    // Variant-B order is intrinsic Y-X-Z (middle = X-asin), so reconstruct
    // a quaternion that decomposes back to (axial, flex, ev) when matrix-
    // euler-B is re-applied.  Equivalent to ZXY intrinsic = ZYX with
    // permuted axes.
    const double cP = std::cos(0.5 * thetaPf), sP = std::sin(0.5 * thetaPf);
    const double cE = std::cos(0.5 * thetaEv), sE = std::sin(0.5 * thetaEv);
    const double cA = std::cos(0.5 * thetaAx), sA = std::sin(0.5 * thetaAx);
    // Compose: q = qZ(axial) ⊗ qX(eversion) ⊗ qY(flex)
    const Quat qZ(cA, 0.0, 0.0, sA);
    const Quat qX(cE, sE, 0.0, 0.0);
    const Quat qY(cP, 0.0, sP, 0.0);
    const Quat qRelNew = quat_mult(quat_mult(qZ, qX), qY).normalized();
    orient[footSeg] = quat_mult(qRelNew, qLL).normalized();

    Diagnostics& d = diagnostics();
    if (isRight) { d.anklePfRDeg = thetaPf * kR2D; d.ankleClampedR = clamped; }
    else         { d.anklePfLDeg = thetaPf * kR2D; d.ankleClampedL = clamped; }
}

}  // namespace

void applyAnkleCoupling(std::array<Quat, fox::body::kSegmentCount>& orient)
{
    applyOneAnkle(orient, kSEG_RLowerLeg, kSEG_RFoot, /*isRight=*/true);
    applyOneAnkle(orient, kSEG_LLowerLeg, kSEG_LFoot, /*isRight=*/false);
}

// ----------------------------------------------------------------------------
//  Spec §48.2 — toe-rocker switching.  Returns the heel/toe contact
//  probability weights derived from the Foot → Toe relative extension
//  angle.  The numeric thresholds (5°, 30°) come from kCToes[5] = sin5°
//  and kToeRockerHighRad.
// ----------------------------------------------------------------------------
ToeWeights computeToeRockerWeights(
    const std::array<Quat, fox::body::kSegmentCount>& orient)
{
    namespace fb = fox::body;
    auto toeAngleRad = [&](int footSeg, int toeSeg) -> double {
        const Quat qF = orient[footSeg];
        const Quat qT = orient[toeSeg];
        const Quat qRel = canon(quat_mult(qT, qF.conj()).normalized());
        const QVector3D phi = quat_log(qRel);
        // Toe MTP flexion is about the Y axis (lateral); positive when
        // the toe lifts.  We use signed phi.y so the gait detection knows
        // the direction (lifting vs curling under).
        return double(phi.y());
    };
    auto weights = [&](double thetaT) -> std::pair<double, double> {
        const double lo = fb::kToeRockerLowRad;       // 5° = 0.0872 rad
        const double hi = fb::kToeRockerHighRad;      // 30° = 0.5236 rad
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

}  // namespace fox::coupling
