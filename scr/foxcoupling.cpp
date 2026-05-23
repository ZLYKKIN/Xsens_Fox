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

// Body-frame relative rotation (spec §2.3 / §11.3 convention):
//     q_joint = conj(q_child) ⊗ q_parent — used by foxergo.cpp when extracting
//     anatomical angles.  We use the *world*-frame form above for redistribution.
//
// Build a rotation quaternion from a rotation vector φ (spec §5.1).
inline Quat fromRotVec(const QVector3D& phi)
{
    return quat_exp_rotvec(double(phi.x()), double(phi.y()), double(phi.z()));
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
//  Spec §45 — spine rhythm.
// ---------------------------------------------------------------------------
void applySpineRhythm(const Quat& qPelvisWorld,
                      const Quat& qT8World,
                      std::array<Quat, 4>& outL5_L3_T12_T8)
{
    // World-frame rotation FROM pelvis TO T8.  Spec §40 c_spine[0..3] are
    // the relative weights of the four lumbar/lower-thoracic joints; we
    // normalise by their sum so the geodesic from qPelvis to qT8 is
    // sampled at fractional t = w0, w0+w1, w0+w1+w2, w0+w1+w2+w3 = 1.
    const double sum = body::kCSpine[0] + body::kCSpine[1] + body::kCSpine[2] + body::kCSpine[3];
    if (sum < 1e-9) {
        // Defensive: zero weights would collapse the spine.
        outL5_L3_T12_T8 = { qPelvisWorld, qPelvisWorld, qPelvisWorld, qT8World };
        return;
    }
    const double w0 = body::kCSpine[0] / sum;
    const double w1 = body::kCSpine[1] / sum;
    const double w2 = body::kCSpine[2] / sum;

    const QVector3D phi = worldRelativeLog(qPelvisWorld, qT8World);

    // q(t) = exp(t·φ) ⊗ qPelvis traces the world-frame geodesic; at t=1 we
    // recover qT8 exactly (within normalisation).
    const QVector3D phi_L5  = w0           * phi;
    const QVector3D phi_L3  = (w0+w1)      * phi;
    const QVector3D phi_T12 = (w0+w1+w2)   * phi;

    outL5_L3_T12_T8[0] = quat_mult(fromRotVec(phi_L5),  qPelvisWorld).normalized();
    outL5_L3_T12_T8[1] = quat_mult(fromRotVec(phi_L3),  qPelvisWorld).normalized();
    outL5_L3_T12_T8[2] = quat_mult(fromRotVec(phi_T12), qPelvisWorld).normalized();
    outL5_L3_T12_T8[3] = qT8World;   // chain terminus = sensored T8 itself
}

// ---------------------------------------------------------------------------
//  Spec §46 — scapulo-humeral ratio.
// ---------------------------------------------------------------------------
Quat applyScapuloHumeral(const Quat& qT8World,
                         const Quat& qShoulderRawWorld,
                         const Quat& qUpperArmWorld)
{
    // Spec §40 c_arms = [0.95, 0.95, 0.99] — scapula follows the humerus
    // with ~95–99% gain.  We blend the raw shoulder kinematics (from its
    // own IMU) with a modelled fraction of the upper-arm motion, in log-
    // map space, then re-anchor to T8.
    const QVector3D phiArm = worldRelativeLog(qT8World, qUpperArmWorld);
    const QVector3D phiRaw = worldRelativeLog(qT8World, qShoulderRawWorld);

    constexpr double cMean = (0.95 + 0.95 + 0.99) / 3.0;   // ≈ 0.9633
    const QVector3D phiOut = 0.5 * phiRaw + 0.5 * (cMean * phiArm);

    return quat_mult(fromRotVec(phiOut), qT8World).normalized();
}

// ---------------------------------------------------------------------------
//  Spec §47 — knee screw-home mechanism.
// ---------------------------------------------------------------------------
Quat applyKneeScrew(const Quat& qUpperLegWorld,
                    const Quat& qLowerLegRawWorld,
                    bool leftSide)
{
    // Relative thigh→shank rotation in world frame.  Knee flexion is the
    // Y component; the screw-home effect imposes a small Z-axis (axial)
    // rotation when |flexion| < 20° (linear ramp to ~5° at full extension).
    // c_knees[0] = 0.9 attenuates the terminal magnitude.
    const QVector3D phiKnee  = worldRelativeLog(qUpperLegWorld, qLowerLegRawWorld);
    const double    flexion  = double(phiKnee.y());

    constexpr double kFullExtensionThresholdRad = 20.0 * 0.017453292519943295;  // 20°
    constexpr double kScrewTerminalRad          = 5.0  * 0.017453292519943295;  // 5°

    if (std::abs(flexion) >= kFullExtensionThresholdRad) {
        return qLowerLegRawWorld;       // outside screw-home range — no coupling
    }
    const double ramp = 1.0 - std::abs(flexion) / kFullExtensionThresholdRad;
    const double dz   = (leftSide ? +1.0 : -1.0) * body::kCKnees[0] * kScrewTerminalRad * ramp;

    QVector3D phiOut = phiKnee;
    phiOut.setZ(float(double(phiKnee.z()) + dz));
    return quat_mult(fromRotVec(phiOut), qUpperLegWorld).normalized();
}

// ---------------------------------------------------------------------------
//  Spec §48 — ankle plantar limit + toe rocker.
// ---------------------------------------------------------------------------
void applyAnkleToe(const Quat& qLowerLegWorld,
                   const Quat& qFootRawWorld,
                   const Quat& qToeRawWorld,
                   Quat& qFootOut,
                   Quat& qToeOut)
{
    // Ankle dorsi/plantar = Y axis of the shank→foot relative rotation.
    // Soft-clamp to body::kCAnkles[1] = 30° plantar (negative Y in our convention).
    QVector3D phiAnkle = worldRelativeLog(qLowerLegWorld, qFootRawWorld);
    const double plantarLimitRad = body::kCAnkles[1];   // 0.523599 rad = 30°

    if (double(phiAnkle.y()) < -plantarLimitRad) {
        phiAnkle.setY(float(-plantarLimitRad));
    }
    qFootOut = quat_mult(fromRotVec(phiAnkle), qLowerLegWorld).normalized();

    // Toe rocker: when the foot is plantar-flexed beyond sin(5°) (≈ body::kCToes[5]),
    // the toe lifts proportionally with gain body::kCToes[1] = 1.05.
    QVector3D phiToe = worldRelativeLog(qFootOut, qToeRawWorld);
    const double plantar = -double(phiAnkle.y());   // positive when plantar-flexed

    if (plantar > body::kCToes[5]) {
        const double extra = body::kCToes[1] * (plantar - body::kCToes[5]);
        phiToe.setY(float(double(phiToe.y()) + extra));
    }
    qToeOut = quat_mult(fromRotVec(phiToe), qFootOut).normalized();
}

}  // namespace fox::coupling
