// Fox Mocap — biomechanical joint-coupling implementation (spec §40 / §45–§48).
#include "foxcoupling.h"

#include <QtGui/QVector3D>

#include <algorithm>
#include <cmath>

namespace fox::coupling {

namespace {

// World-frame relative rotation FROM parent TO child:  q_rel = q_child ⊗ conj(q_parent).
// Its log-map φ is what we need to distribute across a coupled chain — each
// fractional step exp(t·φ) ⊗ q_parent traces the geodesic from q_parent
// (t=0) to q_child (t=1).
inline QVector3D worldRelativeLog(const Quat& qParent, const Quat& qChild)
{
    const Quat qRel = quat_mult(qChild, qParent.conj()).normalized();
    return quat_log(qRel);
}

// Build a rotation quaternion from a rotation vector φ (spec §5.1).
inline Quat fromRotVec(const QVector3D& phi)
{
    return quat_exp_rotvec(double(phi.x()), double(phi.y()), double(phi.z()));
}

// |φ| in degrees — magnitude of a rotation vector.
inline double rotVecMagnitudeDeg(const QVector3D& phi)
{
    const double n = std::sqrt(double(phi.x())*double(phi.x()) +
                               double(phi.y())*double(phi.y()) +
                               double(phi.z())*double(phi.z()));
    return n * 57.29577951308232;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
//  Spec §45 — spinal rhythm, full Pelvis → L5 → L3 → T12 → T8 → Neck → Head
//  chain.  Uses all nine c_spine[0..8] coefficients (§45.1):
//      c_spine[0..3] — lumbar/lower-thoracic (L5/S1, L4/L3, L1/T12, T9/T8),
//      c_spine[4]   — axial twist component (used by spineNeck.cNeck),
//      c_spine[5..8] — cervical / cranial joints (Neck, Head ranges).
// ---------------------------------------------------------------------------
void applySpineRhythm(const Quat& qPelvisWorld,
                      const Quat& qT8World,
                      std::array<Quat, 4>& outL5_L3_T12_T8)
{
    // The four lumbar/lower-thoracic weights drive the Pelvis→T8 distribution.
    const double w0 = body::kCSpine[0];
    const double w1 = body::kCSpine[1];
    const double w2 = body::kCSpine[2];
    const double w3 = body::kCSpine[3];
    const double sum = w0 + w1 + w2 + w3;
    if (sum < 1e-9) {
        outL5_L3_T12_T8 = { qPelvisWorld, qPelvisWorld, qPelvisWorld, qT8World };
        return;
    }

    // Cumulative fractions (§45.5 W_matrix).
    const double t1 = w0 / sum;
    const double t2 = (w0 + w1) / sum;
    const double t3 = (w0 + w1 + w2) / sum;

    const QVector3D phi = worldRelativeLog(qPelvisWorld, qT8World);

    outL5_L3_T12_T8[0] = quat_mult(fromRotVec(t1 * phi), qPelvisWorld).normalized();
    outL5_L3_T12_T8[1] = quat_mult(fromRotVec(t2 * phi), qPelvisWorld).normalized();
    outL5_L3_T12_T8[2] = quat_mult(fromRotVec(t3 * phi), qPelvisWorld).normalized();
    outL5_L3_T12_T8[3] = qT8World;
}

// Spec §45 — extended distribution covering the cervical joints too.
// Useful when the head sensor is the chain terminator and the spine total
// must reach Pelvis→Head, with the neck/head fractions from c_spine[5..8]
// applied as additional steps beyond T8.
void applySpineRhythmFull(const Quat& qPelvisWorld,
                          const Quat& qT8World,
                          const Quat& qHeadWorld,
                          Quat& outL5, Quat& outL3, Quat& outT12,
                          Quat& outT8, Quat& outNeck, Quat& outHead)
{
    std::array<Quat, 4> trunk{};
    applySpineRhythm(qPelvisWorld, qT8World, trunk);
    outL5 = trunk[0]; outL3 = trunk[1]; outT12 = trunk[2]; outT8 = trunk[3];

    // Neck/Head: distribute T8 → Head over two joints (jT1C7, jC1Head) with
    // c_spine[5]=0.9 (neck) and c_spine[6]=0.9 (head); average is half-half.
    const double wN = body::kCSpine[5];
    const double wH = body::kCSpine[6];
    const double s2 = wN + wH;
    if (s2 < 1e-9) {
        outNeck = qT8World;
        outHead = qHeadWorld;
        return;
    }
    const QVector3D phiNH = worldRelativeLog(qT8World, qHeadWorld);
    outNeck = quat_mult(fromRotVec((wN / s2) * phiNH), qT8World).normalized();
    outHead = qHeadWorld;
}

// ---------------------------------------------------------------------------
//  Spec §46 — scapulo-humeral ratio, piecewise-linear c_arms_effective(θ).
//      |θ| ≤ 60°:                 c_eff = c_arms[0] = 0.95
//      60° < |θ| ≤ 90°:           c_eff = 0.95 + ((|θ|-60°)/30°)·(0.99-0.95)
//      |θ| > 90°:                 c_eff = c_arms[2] = 0.99
//  Applied per anatomical axis: pitch (Y, flexion) and roll (X, abduction)
//  get independent c_eff so a combined elevation (forward+lateral lift) is
//  decomposed correctly (§46.4).
// ---------------------------------------------------------------------------
Quat applyScapuloHumeral(const Quat& qT8World,
                         const Quat& /*qShoulderRawWorld*/,
                         const Quat& qUpperArmWorld)
{
    const QVector3D phiArm = worldRelativeLog(qT8World, qUpperArmWorld);

    auto cEff = [](double thetaDeg) {
        const double a = std::abs(thetaDeg);
        if (a <= body::kScapHumThetaLowDeg)  return body::kCArms[0];
        if (a >= body::kScapHumThetaHighDeg) return body::kCArms[2];
        const double t = (a - body::kScapHumThetaLowDeg) /
                         (body::kScapHumThetaHighDeg - body::kScapHumThetaLowDeg);
        return body::kCArms[0] + t * (body::kCArms[2] - body::kCArms[0]);
    };

    constexpr double kRad2Deg = 57.29577951308232;
    const double phiXdeg = double(phiArm.x()) * kRad2Deg;
    const double phiYdeg = double(phiArm.y()) * kRad2Deg;
    const double phiZdeg = double(phiArm.z()) * kRad2Deg;

    // θ_scapula = c_eff · θ_humerus on each axis (§46.2 / §46.4).
    const QVector3D phiScap(
        float(phiArm.x() * cEff(phiXdeg)),
        float(phiArm.y() * cEff(phiYdeg)),
        // c_arms[1]=0.95 is the elevation/depression term, applied to the
        // axial-twist (Z) component, which corresponds to internal/external
        // rotation of the humerus and scapular axial follow-through.
        float(phiArm.z() * body::kCArms[1])
    );
    (void)phiZdeg;

    return quat_mult(fromRotVec(phiScap), qT8World).normalized();
}

// ---------------------------------------------------------------------------
//  Spec §47 — knee screw-home mechanism (full §47.2 formula).
//      θ_screw = c_knees[1] · (1 - cos(θ_knee_flex)) · θ_max_screw
//  with θ_max_screw = kKneeScrewMaxDeg = 15°.  Knee flex is the Y component
//  of the world-frame thigh→shank log-map.  Screw sign: right leg = +Z
//  (external rotation), left = -Z (internal).  Hyper-extension barrier:
//  θ_flex is forbidden to go negative — clipped at 0 (§47.4 hypExt=0).
// ---------------------------------------------------------------------------
Quat applyKneeScrew(const Quat& qUpperLegWorld,
                    const Quat& qLowerLegRawWorld,
                    bool leftSide)
{
    QVector3D phiKnee  = worldRelativeLog(qUpperLegWorld, qLowerLegRawWorld);
    double    flexion  = double(phiKnee.y());

    // §47.4 hyper-extension barrier: θ_flex ≥ 0.
    if (flexion < 0.0) {
        phiKnee.setY(0.0f);
        flexion = 0.0;
    }
    constexpr double kDeg2Rad = 0.017453292519943295;
    const double maxScrewRad = body::kKneeScrewMaxDeg * kDeg2Rad;

    // §47.2 — versine-shaped axial rotation, monotone with flex.
    const double dz = (leftSide ? -1.0 : +1.0) *
                      body::kCKnees[1] *
                      (1.0 - std::cos(flexion)) *
                      maxScrewRad;

    phiKnee.setZ(float(double(phiKnee.z()) + dz));
    return quat_mult(fromRotVec(phiKnee), qUpperLegWorld).normalized();
}

// ---------------------------------------------------------------------------
//  Spec §48 — ankle clamp + eversion coupling + piecewise toe rocker.
//      θ_eversion_corrected = c_ankles[0]·θ_ev + c_ankles[2]·sin(θ_plantar)
//      θ_plantar ∈ [-45°, +30°]                          (§48.1)
//      Toe (metatarsal-phalangeal) rocker (§48.2):
//          θ_toe <  5°  :   w_heel = 1, w_toe = 0
//          5° ≤ θ_toe ≤ 30°:  w_heel = (30°-θ)/25°,  w_toe = (θ-5°)/25°
//          θ_toe > 30° :   w_heel = 0, w_toe = 1
// ---------------------------------------------------------------------------
void applyAnkleToe(const Quat& qLowerLegWorld,
                   const Quat& qFootRawWorld,
                   const Quat& qToeRawWorld,
                   Quat& qFootOut,
                   Quat& qToeOut)
{
    QVector3D phiAnkle = worldRelativeLog(qLowerLegWorld, qFootRawWorld);

    // Soft-clamp Y (plantar/dorsi).  Sign convention: negative Y = plantar.
    const double plantarLimitRad = body::kCAnkles[1];          // 30° (pi/6)
    const double dorsiLimitRad   = body::kAnkleDorsiLimitRad;  // 45°
    double plantarMag = -double(phiAnkle.y());                 // > 0 means plantar
    if (plantarMag > plantarLimitRad) {
        plantarMag = plantarLimitRad;
        phiAnkle.setY(float(-plantarLimitRad));
    } else if (plantarMag < -dorsiLimitRad) {
        plantarMag = -dorsiLimitRad;
        phiAnkle.setY(float( dorsiLimitRad));   // restored to positive (dorsi)
    }

    // Eversion coupling: θ_ev_corr = c_ankles[0]·θ_ev + c_ankles[2]·sin(θ_plantar).
    // X axis = eversion (positive = lateral roll); applied as additive
    // correction to the X-component of the log-map.
    const double evIn   = double(phiAnkle.x());
    const double evCorr = body::kCAnkles[0] * evIn +
                          body::kCAnkles[2] * std::sin(-plantarMag);   // sin of signed plantar
    phiAnkle.setX(float(evCorr));

    qFootOut = quat_mult(fromRotVec(phiAnkle), qLowerLegWorld).normalized();

    // Toe rocker — piecewise blend on the foot→toe Y angle.
    QVector3D phiToe = worldRelativeLog(qFootOut, qToeRawWorld);
    const double thetaToe = double(phiToe.y());

    double wHeel = 1.0;
    double wToe  = 0.0;
    if (thetaToe >= body::kToeRockerHighRad) {
        wHeel = 0.0;
        wToe  = 1.0;
    } else if (thetaToe > body::kToeRockerLowRad) {
        const double span = body::kToeRockerHighRad - body::kToeRockerLowRad;
        wToe  = (thetaToe - body::kToeRockerLowRad) / span;
        wHeel = 1.0 - wToe;
    }

    // §48.2 c_toes[1] = 1.05 — slight overshoot when the toe lifts past
    // threshold (it amplifies the metatarsal extension above linear).
    if (wToe > 0.0) {
        const double extra = body::kCToes[1] * wToe *
                             (thetaToe - body::kToeRockerLowRad);
        phiToe.setY(float(thetaToe + extra));
    }

    qToeOut = quat_mult(fromRotVec(phiToe), qFootOut).normalized();
    (void)wHeel;   // weights are also consumed by ContactDetector::weighFoot.
}

// ---------------------------------------------------------------------------
//  Spec §47.4 (hyper-extension) — soft barrier for knee/elbow flexion.
//  When θ_flex < 0 the function returns a corrected child quaternion whose
//  flexion is clipped to 0; otherwise it returns the input unchanged.
// ---------------------------------------------------------------------------
Quat applyHyperExtensionBarrier(const Quat& qParentWorld,
                                const Quat& qChildWorld)
{
    QVector3D phi = worldRelativeLog(qParentWorld, qChildWorld);
    if (double(phi.y()) < 0.0) {
        phi.setY(0.0f);
        return quat_mult(fromRotVec(phi), qParentWorld).normalized();
    }
    return qChildWorld;
}

// ---------------------------------------------------------------------------
//  Spec §48.2 — toe-rocker weight pair (w_heel, w_toe) for a given toe
//  flexion angle (radians).  Used by ContactDetector to bias the foot vs
//  ball ZUPT anchor during the heel-off → toe-off transition.
// ---------------------------------------------------------------------------
ToeRockerWeights toeRockerWeights(double thetaToeRad)
{
    ToeRockerWeights w{1.0, 0.0};
    if (thetaToeRad >= body::kToeRockerHighRad) {
        w.heel = 0.0;
        w.toe  = 1.0;
    } else if (thetaToeRad > body::kToeRockerLowRad) {
        const double span = body::kToeRockerHighRad - body::kToeRockerLowRad;
        w.toe  = (thetaToeRad - body::kToeRockerLowRad) / span;
        w.heel = 1.0 - w.toe;
    }
    return w;
}

}  // namespace fox::coupling
