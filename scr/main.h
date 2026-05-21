// Fox Mocap — MVN-style app for Xsens Link + optional gloves.
// Full C++ port of HumanInertialPose-main (23-segment Xsens skeleton,
// T/N-pose reference angles, dummy-segment trick, forward kinematics,
// static sensor-to-segment calibration).  Connection layer mirrors the
// XESNSE pose-source abstraction (MVN MXTP02/25 over UDP :9763).

#pragma once

// MSVC does not define M_PI in <cmath> unless this is requested.
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <QtCore/QByteArray>
#include <QtCore/QMutex>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtGui/QFont>
#include <QtGui/QMatrix4x4>
#include <QtGui/QVector2D>
#include <QtGui/QVector3D>
#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>

#include <array>
#include <atomic>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "foxmath.h"   // Quat + pure rotation math (extracted, unit-tested)

// ============================================================================
//  Constants
// ============================================================================

namespace fox {

constexpr int     kXsensSegmentCount  = 23;
constexpr int     kXsensKeypointCount = 28;    // 23 joints + 5 end-points
constexpr int     kXsensSegmentCountWithDummies = 27;
constexpr int     kFingerSegmentsHand = 20;
constexpr double  kRenderFps          = 90.0;
constexpr double  kStaleSeconds       = 2.0;
constexpr int     kCalibrationSamples = 500;    // fixed-count average; valid at any suit rate
constexpr int     kCountdownSeconds   = 6;    // 6 s prep / Madgwick warm-up
                                              // (5 s convergence at 240 Hz × β=0.35
                                              //  comfortably flattens all 17 filters)

// Suit family — drives the whole-system update rate ("hertz").  Picked at
// startup (wizard, or forced to Link by -test).  Xsens Link streams at
// 240 Hz; a full 17-tracker Awinda suit tops out at 60 Hz.
enum class SuitType { Awinda, Link };
constexpr double nativeRateHz(SuitType s) { return s == SuitType::Link ? 240.0 : 60.0; }

// Rate-compensation helpers.  The locomotion solver and various timers were
// hand-tuned for a fixed 90 Hz tick; these convert those magic numbers so the
// behaviour is preserved at any processing rate (60 / 90 / 240).  Both are
// exact no-ops at 90 Hz.
//   ticksFor       — a duration expressed as a tick count at rateHz.
//   rateAdjustAlpha — a first-order LP/blend coefficient a0 (tuned at dt0=1/90)
//                     re-expressed for a step of dt via its time constant tau.
inline int ticksFor(double seconds, double rateHz) {
    const long n = std::lround(seconds * rateHz);
    return n < 1 ? 1 : static_cast<int>(n);
}
inline double rateAdjustAlpha(double a0, double dt, double dt0 = 1.0 / 90.0) {
    if (a0 <= 0.0) return 0.0;
    if (a0 >= 1.0) return 1.0;
    const double tau = -dt0 / std::log(1.0 - a0);
    return 1.0 - std::exp(-dt / tau);
}

// Xsens segment indices (matches MXTP segId - 1, same as hipose xsens_segment_names)
enum Seg : int {
    SEG_Pelvis        = 0,  SEG_L5        = 1,  SEG_L3            = 2,
    SEG_T12           = 3,  SEG_T8        = 4,  SEG_Neck          = 5,
    SEG_Head          = 6,  SEG_RShoulder = 7,  SEG_RUpperArm     = 8,
    SEG_RForearm      = 9,  SEG_RHand     = 10, SEG_LShoulder     = 11,
    SEG_LUpperArm     = 12, SEG_LForearm  = 13, SEG_LHand         = 14,
    SEG_RUpperLeg     = 15, SEG_RLowerLeg = 16, SEG_RFoot         = 17,
    SEG_RToe          = 18, SEG_LUpperLeg = 19, SEG_LLowerLeg     = 20,
    SEG_LFoot         = 21, SEG_LToe      = 22,
};

extern const char* kSegmentNames[kXsensSegmentCount];

// ============================================================================
//  Quaternion math (WXYZ, scalar first) — mirrors hipose/rotations.py
//
//  The Quat struct and the rotation primitives (quat_mult / vec_rotate /
//  euler_to_quat / swingTwistDecompose / slerp_quat / yaw_only_quat /
//  mirror_y_quat / hemisphereContinuous) now live in foxmath.h (included
//  above) so the exact code the live pipeline runs is unit-tested in isolation.
// ============================================================================

struct WristAnatomicalCfg {
    double maxFlexRad   = M_PI * 0.5;
    double maxLatDevRad = M_PI / 6.0;
    double twistWeight  = 1.0;
    bool   enabled      = true;
};

struct FingerJointLimit {
    double spreadMin;
    double spreadMax;
    double flexMin;
    double flexMax;
};

// ============================================================================
//  SuitPose — the latest per-frame suit data shared between threads.
// ============================================================================

struct SuitPose {
    quint64 sampleCounter = 0;
    double  recvTime      = 0.0;                   // monotonic seconds

    // Only the 17 trackers actually on the suit write orientations; spine
    // (L5, L3, T12), Neck and toes have no physical sensor.  The defaults
    // below keep them as identity quats so the skeleton's T/N-pose angles
    // still drive them correctly inside forward kinematics.
    std::array<Quat, kXsensSegmentCount> quat;     // W,X,Y,Z, world-frame
    std::array<bool, kXsensSegmentCount> segValid{};   // per-segment: have we seen it?

    // Most-recent per-sensor raw calibrated IMU channels (after SI-unit
    // reconstruction) — used by the calibration dialog to build the full
    // imu-calibration set exactly like hipose's apply_imu_calibration:
    // acc_magn, gyr_bias, mag_magn and s2s_offset.
    std::array<QVector3D, kXsensSegmentCount> accSensor{};   // g
    std::array<QVector3D, kXsensSegmentCount> gyrSensor{};   // deg/s
    std::array<QVector3D, kXsensSegmentCount> magSensor{};   // calibrated units

    QVector3D pelvisPos{0, 0, 0};                   // root position in meters

    // Last known per-segment packet timestamp (monotonic seconds). Used by
    // SensorIndicatorsPanel to detect stale sensors (>2s without update).
    std::array<double, kXsensSegmentCount> segLastT{};
    // Body-pack battery 0..100. -1 if unknown.
    int batteryLevel = -1;

    bool                                      hasGloves = false;
    std::array<Quat,      kFingerSegmentsHand> rightGloveQ{};
    std::array<Quat,      kFingerSegmentsHand> leftGloveQ{};
    std::array<QVector3D, kFingerSegmentsHand> rightGloveP{};   // positions, meters
    std::array<QVector3D, kFingerSegmentsHand> leftGloveP{};

    SuitPose() { for (auto& q : quat) q = Quat(1, 0, 0, 0); }
};

// Finger connectivity — five chains of 4 bones each.  Bone chain[k] attaches
// from the tip of chain[k-1] (or from the wrist for k = 0).  The table below
// uses indices into the raw 20-entry per-hand array.
constexpr int kFingerChainCount    = 5;
constexpr int kFingerChainLen      = 4;
extern const int kFingerChains[kFingerChainCount][kFingerChainLen];
extern const char* kFingerChainNames[kFingerChainCount];

// ============================================================================
//  SkeletonXsens — 23 segments + 4 dummy stubs, FK identical to hipose
// ============================================================================

struct ActorConfig {
    double heightCm        = 175.0;
    double footLengthCm    = 26.0;
    double armSpanCm       = 0.0;
    double legLengthCm     = 0.0;
    // FIX issue 5: дополнительные анатомические размеры.  0 = вычислить
    // из роста по дефолтной пропорции (h/1.75 × baseline).
    double hipWidthCm      = 0.0;
    double shoulderWidthCm = 0.0;
    double trunkLengthCm   = 0.0;
    bool   useGloves       = false;
};

// ============================================================================
//  Locomotion — foot-contact detector + foot-lock IK + drift limiter
//
//  Architecture mirrors MVN / Xsens Engine (XME) as observed in the
//  ghidra decomp (xme64.dll):
//    * `XmeControl_setContacts` (tag 0xb / 0x11f / 0x111) shows MVN's
//      biomech solver consumes a (rightDown, leftDown) pair produced by
//      an external detector → we replicate the detector here.
//    * `XmeControl_setTimestampedAiding`, `clearAiding`, `customAiding`
//      show drift correction is a loose (Kalman-style) aiding blend of
//      external observations, NOT raw acceleration integration → we do
//      the same for foot-ground position constraints (Z always re-pinned
//      to 0 on re-anchor, XY frozen between strides).
//    * `XmeControl_setFingerTrackingData` takes a (side, q_array[20],
//      timestamp?) tuple — we keep the same 20-segment-per-hand layout
//      and fuse as  world_q = wrist_q * finger_q_local.  See the docs
//      in the finger section.
// ============================================================================

struct ContactState {
    bool   rightDown  = false;
    bool   leftDown   = false;
    double rightAngV  = 0.0;        // rad/s (smoothed) — for debug/UI
    double leftAngV   = 0.0;
};

// Single-source diagnostic snapshot of the LocomotionSolver's internal state,
// filled by LocomotionSolver::diag() from the exact members the solver computed
// this frame (never recomputed in the logger).  Read by the -test [LOCO] dump,
// the per-frame [pulse] line, and the [evt:*] transition detectors.
struct LocoDiag {
    int       pose          = 0;    // LocomotionSolver::PoseKind
    int       poseTicks     = 0;
    int       support       = 2;    // LocomotionSolver::Side (RIGHT/LEFT/BOTH)
    double    confR         = 0.0,  confL = 0.0;
    bool      committedR    = false, committedL = false;
    double    rAngV         = 0.0,  lAngV = 0.0;       // foot |ω| LP, rad/s
    double    pelvisAngV    = 0.0,  pelvisYawAngV = 0.0;
    double    footPitchZR   = 0.0,  footPitchZL = 0.0; // sin(pitch): >0 heel-down, <0 toe-down
    double    contactBlendR = 0.0,  contactBlendL = 0.0;
    double    heelLiftConfR = 0.0,  heelLiftConfL = 0.0;
    bool      heelLiftR     = false, heelLiftL = false;
    double    pelvisZVel    = 0.0;  // m/s (LP), jump/squat/land detector
    int       airborneTicks = 0,    landedTicks = 0, zuptTicks = 0;
    double    tiltCos       = 1.0;  // pelvis tilt cosine (lie detector)
    QVector3D offset        {0, 0, 0};
    QVector3D anchorR       {0, 0, 0}, anchorL {0, 0, 0};

    // Decision internals — WHY the offset / anchors moved this frame.  These are
    // the update() locals that drive commit/release, the dual-anchor blend and
    // the per-frame step cap; logging them is what lets a captured jump /
    // acrobatics / glitch be diagnosed after the fact (the smoothed confR/L and
    // final offset above only show the result, not the cause).
    double    rawCR         = 0.0,  rawCL        = 0.0; // pre-smoothing contact confidence
    double    fkxyStableWR  = 0.0,  fkxyStableWL = 0.0; // planted-despite-rotating weight
    double    fkxyRangeR    = 0.0,  fkxyRangeL   = 0.0; // FK-XY spread over window (m)
    double    yawFreezeW    = 0.0,  pelvisRotKill = 0.0;
    double    rollingWR     = 0.0,  rollingWL    = 0.0; // toe-roll hysteresis weight
    double    imbalance     = 0.0;                      // |effR-effL|/total (R/L anchor skew)
    double    effR          = 0.0,  effL         = 0.0; // effective dual-anchor weights
    double    maxStepXY     = 0.0;                      // per-frame XY step cap (m)
    bool      stepClampedXY = false;                    // the step cap actually clamped
    bool      feetLifted    = false, ballistic   = false, driftAir = false; // airborne gates
};

// Human-readable name of a LocomotionSolver::PoseKind value (for the log).
inline const char* locoPoseName(int pose) {
    switch (pose) {
        case 1: return "Stand";
        case 2: return "Sit";
        case 3: return "Squat";
        case 4: return "Lying";
        case 5: return "Airborne";
        default: return "Unknown";
    }
}

class LocomotionSolver {
public:
    void reset();

    // Feed one frame of data.  Foot keypoints are FK world positions
    // BEFORE applying this solver's offset.  Returns the translation to
    // add to every keypoint so the skeleton walks in the scene.
    //
    // v2 signature: now takes the pelvis quaternion too.  The solver needs
    // it to detect "pelvis is actually holding still" — the master gate
    // for freezing the offset against isolated-limb motion (scenario A) and
    // preventing feet-up-on-chair from dropping the avatar (scenario C).
    //
    // FIX (heel/toe contact discrimination): сигнатура расширена тремя
    // FK-точками стопы (heel/ball/tip), а не одной "lowest"-точкой.
    // Solver сам по footPitchZ выбирает activeContact из этих точек:
    // heel-contact (pitch>+15°) → heel; toe-contact (pitch<-15°) → ball;
    // в середине → lowest3.  Без этой правки anchor для стояния на
    // мыске фиксирует heel-keypoint на полу — лодыжка проваливается
    // на ~13 см.
    QVector3D update(const Quat& rightFootQuat,
                     const Quat& leftFootQuat,
                     const Quat& pelvisQuat,
                     const QVector3D& fkRightHeel,
                     const QVector3D& fkRightBall,
                     const QVector3D& fkRightTip,
                     const QVector3D& fkLeftHeel,
                     const QVector3D& fkLeftBall,
                     const QVector3D& fkLeftTip,
                     double tSeconds);

    ContactState contact() const { return m_contact; }
    enum Side { RIGHT = 0, LEFT = 1, BOTH = 2 };
    Side currentSupport() const  { return m_support; }
    QVector3D anchor()    const  { return m_anchor; }
    void setVerbose(bool v) { m_verbose = v; }

private:
    bool m_verbose = false;
    // --- Foot-contact detector --------------------------------------------
    Quat   m_prevRQ;
    Quat   m_prevLQ;
    double m_lastT        = 0.0;
    bool   m_haveLast     = false;
    double m_rAngV        = 0.0;    // low-passed angular-velocity magnitude
    double m_lAngV        = 0.0;
    int    m_rPlantTicks  = 0;
    int    m_lPlantTicks  = 0;
    int    m_rLiftTicks   = 0;
    int    m_lLiftTicks   = 0;
    ContactState m_contact{};

    // --- Foot-lock anchor (position aiding constraint) --------------------
    bool      m_initialised = false;
    Side      m_support     = RIGHT;
    QVector3D m_anchor      {0, 0, 0};

    double m_stillRad     = 3.00;
    double m_heightMargin = 0.03;
    int    m_latchTicks   = 3;
    double m_switchMargin = 0.04;
    double m_heightMarginSlow = 0.08;

    // --- Per-instance state that used to live as function-local statics ---
    QVector3D m_offsetLast   {0, 0, 0};
    // Pelvis world offset as of the last foot commit.  Carries the accumulated
    // travel across the swing→plant transition: when no single foot is locked,
    // re-anchoring off the LP-smoothed m_offsetLast (which lags / is pulled
    // toward origin) discarded each step's forward progress ("walks in place").
    QVector3D m_offsetCommitted {0, 0, 0};
    bool      m_offsetReady  = false;

    // =====================================================================
        //               v3 STATE — weighted dual-anchor + ZUPT
        // =====================================================================
        // Pelvis angular-velocity tracker (used by pelvis-stillness gate and
        // ZUPT).
        Quat      m_prevPelvisQ      {1, 0, 0, 0};
        double    m_pelvisAngV        = 0.0;
        double    m_pelvisYawAngV     = 0.0;
        bool      m_yawFrozenPrev     = false;

        // FK-XY history ring buffers (planted-despite-rotating criterion).
        // The window spans ~111 ms; its length in samples scales with the
        // processing rate (10 @ 90 Hz, 7 @ 60 Hz, 27 @ 240 Hz).  The arrays are
        // sized for the highest rate; m_fkxyWindow is the active length.
        static constexpr int kFKXYWindowMax = 32;
        int       m_fkxyWindow        = 10;     // set by setProcRate (≤ Max)
        std::array<QVector2D, kFKXYWindowMax> m_rFKXY {};
        std::array<QVector2D, kFKXYWindowMax> m_lFKXY {};
        int       m_fkxyHead          = 0;
        int       m_fkxyCount         = 0;

        // v3: per-foot contact confidence, smoothed; rising edge → commit,
        // falling edge → release with hysteresis.
        double    m_confR             = 0.0;
        double    m_confL             = 0.0;
        bool      m_committedR        = false;
        bool      m_committedL        = false;
        QVector3D m_anchorR           {0, 0, 0};
        QVector3D m_anchorL           {0, 0, 0};
        int       m_recentCommitTicks = 0;

        // v3: pose classification state (set each frame in update()).
        // PoseAirborne добавлен в FIX (airborne phase): override через
        // pelvisZVel + feetLifted detection в update().
        enum PoseKind { PoseUnknown = 0, PoseStand = 1, PoseSit = 2,
                        PoseSquat = 3, PoseLying = 4, PoseAirborne = 5 };
        PoseKind  m_pose              = PoseUnknown;
        int       m_poseTicks         = 0;

        // v3: ZUPT ticks counter.  When pelvis + both feet still for
        // >= m_zuptTicksThresh frames → offset fully frozen.
        int       m_zuptTicks         = 0;

        // FIX issue 9 (sit-snap 10 cm падение): мягкий переход Z вместо
        // мгновенного setZ(0).  При переходе stand→sit таз идёт вниз,
        // но пока стопы не реально низко, мы НЕ форсируем Z на пол —
        // блёндим за ~10 кадров.
        int       m_lowZTicksR        = 0;
        int       m_lowZTicksL        = 0;
        int       m_zSnapBlendTicks   = 0;     // оставшиеся кадры бленда

        // FIX issue 10 (toe-roll глобальный сдвиг): пока стопа быстро
        // вращается без перемещения XY, замораживаем commit/release и
        // удерживаем conf в полосе вокруг порогов.
        bool      m_confRFrozenForRoll = false;
        bool      m_confLFrozenForRoll = false;
        double    m_confRFrozenValue   = 0.0;
        double    m_confLFrozenValue   = 0.0;

        // FIX (heel/toe contact discrimination): sin(pitch) каждой стопы,
        // LP α=0.30.  vec_rotate((1,0,0), qFoot).z = 2*(qx*qz - qw*qy).
        // > 0  → ball выше heel (носок поднят, опора на пятке)
        // < 0  → ball ниже heel (пятка поднята, опора на мыске).
        // defAngFor(SEG_RFoot/LFoot) = identity в tpose/npose, поэтому
        // qR/qL уже совпадает с world quat стопы.
        double    m_footPitchZR        = 0.0;
        double    m_footPitchZL        = 0.0;
        // Сглаженный contact blend: +1 = heel-contact, -1 = toe-contact, 0 = neutral.
        // LP α=0.20 (~55ms response).  Используется для re-snap anchor при
        // смене contact-point (Phase 3).
        double    m_contactBlendR      = 0.0;
        double    m_contactBlendL      = 0.0;

        // FIX (squat heel-lift): smoothstep confidence что пятка поднята
        // именно в позе Squat/Sit.  LP α=0.10 (333ms), heel-lift в приседе
        // обычно держится секундами.  m_heelLiftR/L bool — пороговое значение.
        double    m_heelLiftConfR      = 0.0;
        double    m_heelLiftConfL      = 0.0;
        bool      m_heelLiftR          = false;
        bool      m_heelLiftL          = false;
        // Foot length (метры), нужен для Z-snap при heel-lifted squat.
        double    m_footLengthM        = 0.26;

        // FIX (airborne phase): отдельная PoseAirborne фаза.
        // m_pelvisZVel оценивается из delta-offset.z по dt, LP α=0.30.
        // Активация: feetLifted + pelvisZVel > 0.50 m/s (ballistic),
        //  либо feetLifted + !committed обе + zVel > 0.15 (drift fallback).
        // Стабилизирована >= 5 кадров (55ms) → pose = PoseAirborne.
        // Выход → m_landedTicks=12 (133ms re-anchor ramp).
        double    m_pelvisZVel         = 0.0;
        double    m_pelvisZPrev        = 0.0;
        bool      m_havePelvisZPrev    = false;
        int       m_airborneTicks      = 0;
        // Last pelvis-tilt cosine from _classifyPose() (stored for -test [LOCO]).
        double    m_lastTiltCos        = 1.0;
        int       m_landedTicks        = 0;

        // Rate-derived tick durations (set by setProcRate; defaults = @90 Hz).
        double    m_procRateHz          = 90.0;
        int       m_airborneStableTicks = 5;    // ~0.055 s airborne stabilisation
        int       m_landedRampTicks     = 12;   // ~0.133 s landing re-anchor ramp
        int       m_commitFadeTicks     = 12;   // ~0.133 s post-commit step-cap fade

        // --- tunables ---------------------------------------------------------
        // FIX «walks in place / 5-10 cm jumps»: тюнинг параметров локомоции.
        //
        // Старые значения требовали слишком высокой confidence для commit
        // (0.70) и медленно гасили стрижок при движении одной ноги (0.18),
        // из-за чего во время быстрой ходьбы анкор-офсеты не успевали
        // обновиться → персонаж стоял на месте.  Ниже — оптимизация под
        // нормальный темп ходьбы (1-2 m/s, длина шага 0.4-0.7 m).
        double    m_fkxyStableRange   = 0.04;   // было 0.03 — допускаем
                                                // 4-cm jitter в planted FK
        double    m_pelvisStillRad    = 0.20;   // было 0.12 — реже триггерим
                                                // pelvis-stillness gate во время ходьбы
        // Confidence hysteresis / LP rates
        double    m_confCommit        = 0.35;   // было 0.70 — commit быстрее
        double    m_confRelease       = 0.25;   // было 0.30 — release быстрее тоже
        double    m_confRiseRate      = 0.50;   // было 0.30 — быстрый rise
        double    m_confFallRate      = 0.25;   // было 0.10 — быстрый fall (не заклинивать)
        // Offset blend rates
        double    m_offsetRatePrimary = 0.40;   // было 0.18 — primary anchor catches up быстро
        double    m_offsetRateDouble  = 0.25;   // было 0.10 — double-support тоже бодрее
        double    m_zRatePelvisMoving = 0.40;   // pose transitions
        double    m_zRatePelvisStill  = 0.06;   // было 0.04 — pose held
        // Pose-aware Z drift kill
        double    m_zDriveRate        = 0.02;
        int       m_poseStableTicks   = 45;     // 0.5 s @ 90 Hz
        // ZUPT
        int       m_zuptTicksThresh   = 60;     // было 45 — ZUPT даёт срабатывать
                                                // только когда actor правда стоит
        // Pose thresholds
        double    m_lieTiltCosThresh  = 0.50;   // cos(60°) — pelvis tilt
        double    m_squatKneeThresh   = 0.30;   // m — |pelvis-to-foot Z|
        double    m_sitKneeThresh     = 0.55;   // m — pelvis-to-foot Z for sit

        // FIX issue 9: soft Z transition tunables
        int       m_lowZTicksRequired = 6;       // ~67 ms @ 90 Hz: foot must
                                                 // be near floor this long
                                                 // before we trust a sit-floor
        double    m_lowZBandM         = 0.04;    // 4 cm от fkMinZ → "low"
        int       m_zSnapBlendFrames  = 10;      // ~110 ms taper до Z=0

        // FIX issue 10: toe-roll detector + conf hysteresis
        double    m_rollAngVThresh    = 2.0;     // rad/s — стопа быстро крутится
        double    m_rollXYRangeMax    = 0.03;    // m — но XY почти не двигается
        double    m_confHystBand      = 0.05;    // полоса вокруг commit/release
        // Actor height — must be set by owner after construction via
        // setActorHeight() before the first update() call.
        double    m_actorHeightM      = 1.75;

        // --- Diagnostic-only captures of update() locals (-test [LOCO]) -------
        // Written each frame for the log; never read back into the solver math.
        double m_dbgRawCR = 0.0,        m_dbgRawCL = 0.0;
        double m_dbgFkxyStableWR = 0.0, m_dbgFkxyStableWL = 0.0;
        double m_dbgFkxyRangeR = 0.0,   m_dbgFkxyRangeL = 0.0;
        double m_dbgYawFreezeW = 0.0,   m_dbgPelvisRotKill = 0.0;
        double m_dbgRollingWR = 0.0,    m_dbgRollingWL = 0.0;
        double m_dbgImbalance = 0.0,    m_dbgEffR = 0.0, m_dbgEffL = 0.0;
        double m_dbgMaxStepXY = 0.0;
        bool   m_dbgStepClampedXY = false;
        bool   m_dbgFeetLifted = false, m_dbgBallistic = false, m_dbgDriftAir = false;
        bool   m_dbgConfigLogged = false;   // one-shot [LOCO CONFIG] guard

        // === v3 additions END ===

      public:
        void setActorHeight(double h) { m_actorHeightM = std::max(0.5, h); }
        // FIX (heel-lift Z-snap): foot length задаётся owner'ом из
        // ActorConfig.footLengthCm.  Используется в Phase 3 commit-блоке
        // для расчёта world.z = bone_foot * sin(pitch) при heel-lifted squat.
        void setFootLength(double m) { m_footLengthM = std::max(0.10, m); }

        // Single-source diagnostic readout — copies the exact internal state
        // the solver computed this frame so the -test log never recomputes a
        // locomotion formula.  Cheap (plain copies); safe to call once/frame.
        LocoDiag diag() const {
            LocoDiag d;
            d.pose          = int(m_pose);
            d.poseTicks     = m_poseTicks;
            d.support       = int(m_support);
            d.confR         = m_confR;          d.confL = m_confL;
            d.committedR    = m_committedR;     d.committedL = m_committedL;
            d.rAngV         = m_rAngV;          d.lAngV = m_lAngV;
            d.pelvisAngV    = m_pelvisAngV;     d.pelvisYawAngV = m_pelvisYawAngV;
            d.footPitchZR   = m_footPitchZR;    d.footPitchZL = m_footPitchZL;
            d.contactBlendR = m_contactBlendR;  d.contactBlendL = m_contactBlendL;
            d.heelLiftConfR = m_heelLiftConfR;  d.heelLiftConfL = m_heelLiftConfL;
            d.heelLiftR     = m_heelLiftR;      d.heelLiftL = m_heelLiftL;
            d.pelvisZVel    = m_pelvisZVel;
            d.airborneTicks = m_airborneTicks;  d.landedTicks = m_landedTicks;
            d.zuptTicks     = m_zuptTicks;
            d.tiltCos       = m_lastTiltCos;
            d.offset        = m_offsetLast;
            d.anchorR       = m_anchorR;        d.anchorL = m_anchorL;
            d.rawCR         = m_dbgRawCR;       d.rawCL = m_dbgRawCL;
            d.fkxyStableWR  = m_dbgFkxyStableWR; d.fkxyStableWL = m_dbgFkxyStableWL;
            d.fkxyRangeR    = m_dbgFkxyRangeR;  d.fkxyRangeL = m_dbgFkxyRangeL;
            d.yawFreezeW    = m_dbgYawFreezeW;  d.pelvisRotKill = m_dbgPelvisRotKill;
            d.rollingWR     = m_dbgRollingWR;   d.rollingWL = m_dbgRollingWL;
            d.imbalance     = m_dbgImbalance;
            d.effR          = m_dbgEffR;        d.effL = m_dbgEffL;
            d.maxStepXY     = m_dbgMaxStepXY;   d.stepClampedXY = m_dbgStepClampedXY;
            d.feetLifted    = m_dbgFeetLifted;  d.ballistic = m_dbgBallistic;
            d.driftAir      = m_dbgDriftAir;
            return d;
        }

        // Bind the processing rate (Hz) so every @90 Hz-tuned tick count and
        // first-order blend coefficient is re-derived for the current suit
        // cadence (Link 240 / Awinda 60).  Exact no-op at 90 Hz.  Must be
        // called before the first update(); survives reset().
        void setProcRate(double hz) {
            m_procRateHz = (hz > 1.0) ? hz : 90.0;
            const double dt = 1.0 / m_procRateHz;
            m_poseStableTicks     = ticksFor(0.500, m_procRateHz);
            m_zuptTicksThresh     = ticksFor(0.667, m_procRateHz);
            m_lowZTicksRequired   = ticksFor(0.067, m_procRateHz);
            m_zSnapBlendFrames    = ticksFor(0.111, m_procRateHz);
            m_landedRampTicks     = ticksFor(0.133, m_procRateHz);
            m_commitFadeTicks     = ticksFor(0.133, m_procRateHz);
            m_airborneStableTicks = ticksFor(0.055, m_procRateHz);
            const int w           = ticksFor(0.111, m_procRateHz);
            m_fkxyWindow          = (w > kFKXYWindowMax) ? kFKXYWindowMax : w;
            m_confRiseRate        = rateAdjustAlpha(0.50, dt);
            m_confFallRate        = rateAdjustAlpha(0.25, dt);
            m_offsetRatePrimary   = rateAdjustAlpha(0.40, dt);
            m_offsetRateDouble    = rateAdjustAlpha(0.25, dt);
            m_zRatePelvisMoving   = rateAdjustAlpha(0.40, dt);
            m_zRatePelvisStill    = rateAdjustAlpha(0.06, dt);
            m_zDriveRate          = rateAdjustAlpha(0.02, dt);
        }

      private:
        PoseKind _classifyPose(const Quat& qPelvis,
                               const QVector3D& fkR,
                               const QVector3D& fkL,
                               double& outTiltCos) const;
};

// Single-source FK diagnostic capture (-test).  computeKeypoints() fills this
// when a non-null pointer is passed, so the log shows the EXACT intermediate
// values the real skeleton used — no parallel recomputation.  Stage order:
//   oriented = quat_mult(raw, defAng) (+ shoulder-cone)  ->  global (with the 4
//   dummy stubs)  ->  boneVec = vec_rotate([len,0,0], global)  ->  kp (chain).
struct FkDiag {
    std::array<Quat,      kXsensSegmentCount>           oriented{};  // raw*defAng (+cone)
    std::array<Quat,      kXsensSegmentCountWithDummies> global{};   // after dummy stubs
    std::array<QVector3D, kXsensSegmentCountWithDummies> boneVec{};  // local→world bone vector
    std::array<float,     kXsensSegmentCountWithDummies> len{};      // bone lengths used (m)
    std::array<QVector3D, kXsensKeypointCount>          kp{};        // final keypoints (world, m)
    QVector3D rootPos{0, 0, 0};
};

class SkeletonXsens {
public:
    // pose = "tpose" or "npose"
    SkeletonXsens(const ActorConfig& actor, const std::string& pose);

    // Forward kinematics.  segmentOrients has 23 entries (WXYZ quats), the
    // result has kXsensKeypointCount (28) 3-D points in meters.  When diag is
    // non-null it is filled with every intermediate (single-source -test log).
    std::array<QVector3D, kXsensKeypointCount>
    computeKeypoints(const std::array<Quat, kXsensSegmentCount>& segOrients,
                     const QVector3D& rootPos,
                     FkDiag* diag = nullptr) const;

    // Adds 4 dummy segments (right/left scapular, right/left pelvis) so the
    // kinematic chain has 27 entries total.
    std::array<Quat, kXsensSegmentCountWithDummies>
    addDummySegments(const std::array<Quat, kXsensSegmentCount>& segs) const;

    // Bone topology (per-segment start/end keypoint indices).
    const std::array<int, kXsensSegmentCountWithDummies>& startPts() const { return m_start; }
    const std::array<int, kXsensSegmentCountWithDummies>& endPts()   const { return m_end; }
    const std::array<float, kXsensSegmentCountWithDummies>& lengths() const { return m_len; }
    const std::array<Quat,  kXsensSegmentCount>& defaultSegAngles() const { return m_defAng; }

    const std::string& poseKind() const { return m_pose; }

    // v4: public access to default-angles table for external finger FK.
    // onRenderTick uses this to compose full hand world-orientation as
    //   q_hand_world = cand[SEG_RHand/LHand] * defAngFor(SEG_RHand/LHand).
    Quat defAngFor(int seg) const {
        return (seg >= 0 && seg < kXsensSegmentCount)
                   ? m_defAng[seg] : Quat(1, 0, 0, 0);
    }

private:
    std::string m_pose;
    // 27 entries: 7 spine  +  5 right-arm (w/ scap stub)  +  5 left-arm  +
    //             5 right-leg (w/ pelvis stub)  +  5 left-leg
    std::array<int,   kXsensSegmentCountWithDummies> m_start{};
    std::array<int,   kXsensSegmentCountWithDummies> m_end{};
    std::array<float, kXsensSegmentCountWithDummies> m_len{};
    // 23 entries: the canonical default angles that align passed-in quats
    // with the T/N-pose reference.
    std::array<Quat,  kXsensSegmentCount> m_defAng{};

    void buildTopology();
    void buildDefaultAngles();
    void buildLengths(const ActorConfig& actor);
};

// Free-function accessor used by the double-pose calibrator — returns the
// SkeletonXsens default_seg_angles table for "tpose" or "npose" without
// instantiating a full SkeletonXsens object.
std::array<Quat, kXsensSegmentCount>
defaultSegAnglesFor(const std::string& pose);

// ============================================================================
//  MXTP parser (MVN streaming, big-endian).  Same wire format as my Python
//  port — see XESNSE network_monitor.cpp for the state-machine inspiration.
// ============================================================================

// ============================================================================
//  XDA wire types — copies of the opaque structs used by Xsens Device API.
//  Layouts are verified against XESNSE's static_asserts.
// ============================================================================

struct XsDeviceIdBlob {
    quint64  deviceId       = 0;
    char     productCode[24]{};
    quint32  productVariant = 0;
    quint16  hardwareVersion= 0;
    quint16  subDevice      = 0;
};
struct XsQuaternionBlob { double w = 1.0, x = 0.0, y = 0.0, z = 0.0; };
struct XsTimeStampBlob  { qint64 msTime = 0; };
struct XsDataPacketBlob {
    void*           d = nullptr;
    XsDeviceIdBlob  deviceId{};
    XsTimeStampBlob toa{};
    qint64          packetId = 0;
    XsTimeStampBlob etos{};
};
static_assert(sizeof(XsDataPacketBlob) == 72,
              "XsDataPacket layout must be 72 bytes (matches XDA ABI)");

// XsVector — 3-element (or larger) double vector used by the XDA getters
// for calibrated acc / gyr / mag data.  data[0..size-1] is the payload.
struct XsVectorBlob {
    double* data  = nullptr;
    size_t  size  = 0;
    size_t  flags = 0;
};

// ============================================================================
//  Connection state — mirrors XESNSE's ConnectionState enum
// ============================================================================

enum class ConnStatus {
    NotInitialized,     // before anything happens
    NoDriver,           // xsensdeviceapi64.dll missing or cannot load
    Scanning,           // XsScanner_scanPorts in progress
    NoDevice,           // scan finished, no Xsens hardware present
    Connecting,         // port opened, enumerating trackers
    Streaming,          // data is flowing from the suit
    Stale,              // had data but it stopped for > kStaleSeconds
    Failed,             // hard error (see statusDetail)
};

const char* connStatusName(ConnStatus s);

// ============================================================================
//  XDA direct-connection thread.  Loads xsensdeviceapi64.dll and
//  xstypes64.dll from the `dll/` folder next to the exe, scans Xsens ports,
//  opens the first suit found, switches it to measurement mode and polls
//  orientation quaternions per segment.  Same signals as the previous
//  receiver so the rest of the app (MainWindow, calibration, viewport)
//  wires up unchanged.
// ============================================================================

class MocapReceiver : public QThread {
    Q_OBJECT
public:
    explicit MocapReceiver(bool testMode, QObject* parent=nullptr);
    ~MocapReceiver() override;

    void stop();

    // Stop any previous attempt (if running) and kick off a fresh XDA scan.
    // Safe to call any time — used by the wizard's "Connect suit" button.
    void restart();

    // Bring up Manus gloves: loads ManusSDK.dll, resolves the CoreSdk exports,
    // connects to a running ManusCore host and registers the ergonomics /
    // skeleton / system stream callbacks.  Returns true once CoreSdk reports a
    // live connection.  Idempotent — safe to call repeatedly (the DLL load and
    // handshake are guarded).
    bool connectGloves();
    bool glovesReady()     const;       // physical glove(s) visible to Core
    bool glovesCoreReady() const;       // ManusCore responded to handshake
    bool glovesDllLoaded() const;       // ManusSDK.dll was found and loaded

    // Force all Madgwick filters to re-initialise from the next acc+mag
    // sample.  Called at the start of calibration countdown so the 3-second
    // prepare window doubles as a fusion warm-up.
    void resetFusion();

    // Connection transport preference.  ComPort = scan USB serial ports (Awinda
    // dongle / Link over USB).  Network = XsScanner_enumerateNetworkDevices
    // (Link / Awinda discovered over WiFi or Ethernet).  The PC must already be
    // on the same network as the suit — XDA does device discovery, not the
    // Wi-Fi association itself (that is done at the OS level).
    enum class Transport { ComPort, Network };
    void setTransport(Transport t);

    // Expected native update rate (Hz) implied by the chosen suit (Link 240 /
    // Awinda 60).  Seeds freqHz before the device is queried so the IMU/AHRS
    // math is correct from the first packet; the live XsDevice_updateRate()
    // result is reconciled against it (a warning is logged on mismatch).
    void setExpectedRate(double hz);

    // Install sensor-to-segment alignment quaternions (one per tracker).
    // The receiver rotates each sensor's acc / gyr / mag by inv(s2s[i])
    // before handing them to the fusion filter, exactly like hipose's
    // apply_imu_calibration(rotation=s2s_offset, inv=True) step.
    void setS2sAlignment(const std::array<Quat, kXsensSegmentCount>& s2s);
    void resetS2sAlignment();
    Quat getS2s(int idx) const;

    // Per-sensor magnetometer magnitude (hipose: mag_magn).  Each sensor
    // reads a slightly different raw magnetic-field strength, so we scale
    // each one by its own mean |mag| from the calibration pose before
    // feeding it into the fusion filter — the same normalisation
    // apply_imu_calibration() does in Python.
    void setMagNormalisation(const std::array<double, kXsensSegmentCount>& magMagn);

    // Per-sensor accelerometer magnitude (hipose: acc_magn) and gyro DC
    // bias (hipose: gyr_bias).  Applied as `acc = acc / acc_magn` and
    // `gyr = gyr - gyr_bias` before sensor-to-segment rotation —
    // exactly matches hipose's apply_imu_calibration() pre-processing.
    void setAccNormalisation(const std::array<double, kXsensSegmentCount>& accMagn);
    void setGyroBias(const std::array<QVector3D, kXsensSegmentCount>& gyrBias);

    void setMagSoftIron(const std::array<std::array<double, 9>, kXsensSegmentCount>& mat,
                        const std::array<QVector3D, kXsensSegmentCount>& offset);

    void setSegmentGain(const std::array<float, kXsensSegmentCount>& gain);

    QVector3D snapshotGyroAvg(int idx, int samples) const;
    QVector3D liveGyrSensor(int idx) const;

    // Thread-safe copy of the last suit frame.
    SuitPose snapshot() const;

    // Current connection state + a short human-readable detail (e.g. the
    // error text from the XDA scanner or the number of trackers discovered).
    ConnStatus  status()       const;
    QString     statusDetail() const;
    int         activeSensors() const;            // 0..17 trackers streaming
    bool        isStreaming()   const { return status() == ConnStatus::Streaming; }
    bool        hasGloves()     const;

signals:
    void poseReceived();
    void gloveStatusChanged(bool connected);
    void statusChanged(int status, const QString& detail);
    void fpsUpdated(double hz);

protected:
    void run() override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

// ============================================================================
//  Localisation — runtime language switch (ru / en) with a signal.
// ============================================================================

class Lang : public QObject {
    Q_OBJECT
public:
    static Lang& instance();
    enum Code { RU, EN };

    Code        current() const { return m_code; }
    void        setLanguage(Code c);
    static QString t(const char* key);        // lookup for the current lang

signals:
    void changed();                           // widgets re-translate on this

private:
    Lang()  = default;
    Code m_code = Code::RU;
};

// ============================================================================
//  NewSessionWizard — single QDialog, five pages via QStackedWidget.
//  Welcome → Mode → Dimensions → Calibration → Ready.  All pages re-translate
//  on language change.  Calibration is gated on the suit being Streaming.
// ============================================================================

class NewSessionWizard : public QDialog {
    Q_OBJECT
public:
    struct Result {
        bool     useGloves = false;
        SuitType suit      = SuitType::Awinda;   // drives system update rate
        double heightCm = 175.0;
        double footLengthCm = 26.0;
        double armSpanCm = 0.0;
        double legLengthCm = 0.0;
        // FIX issue 5: новые поля размеров.  0 = compute from height.
        double hipWidthCm      = 0.0;
        double shoulderWidthCm = 0.0;
        double trunkLengthCm   = 0.0;
        std::string poseKind = "tpose";
        std::array<Quat, kXsensSegmentCount> calibReference{};
        std::array<Quat, kXsensSegmentCount> tposeReference{};
        QVector3D                            tposePelvisPos{};
        bool                                 tposeCaptured = false;
        // FIX (gloves polish): finger ergonomics averaged during T-pose.
        // Каждый float — угол в градусах: layout =
        // [thumb spread, thumb MCP, thumb PIP, thumb DIP, index spread,
        //  index MCP, ..., pinky DIP].  Используется в parseErgoHand как
        // "neutral" — runtime раздаёт effective = raw - baseline, что
        // компенсирует встроенный bias glove'а под конкретного актёра.
        // Zero-init = baseline отсутствует = старое поведение.
        std::array<float, 20> fingerBaselineR{};
        std::array<float, 20> fingerBaselineL{};
        bool                  fingerBaselineCaptured = false;
    };

    NewSessionWizard(MocapReceiver* rx, bool testMode, QWidget* parent=nullptr);
    Result result() const { return m_result; }

    // Pre-select the "Suit with gloves" radio button.  Wired from main()
    // when the operator launches with --gloves so the wizard opens in the
    // right mode without an extra click.
    void preselectGloves(bool on);

    // Pre-select the suit family (Link / Awinda).  Wired from main() so a CLI
    // override (-test ⇒ Link) opens the wizard on the right suit and rate.
    void preselectSuit(SuitType suit);

private slots:
    void goNext();
    void goBack();
    void onLanguageChanged(int idx);
    void onConnectSuit();
    void onConnectGloves();
    void onModeChanged();
    void onCalibrationBegin();
    void onCountdownTick();
    void onCaptureTick();
    void onStatusTick();

private:
    MocapReceiver* m_rx   = nullptr;
    bool           m_test = false;
    Result         m_result;

    class QStackedWidget* m_pages = nullptr;
    int            m_pageIdx      = 0;

    // Page 1 (welcome)
    class QLabel*    m_welcomeImg = nullptr;
    class QLabel*    m_welcomeHeading = nullptr;
    class QLabel*    m_welcomeSub = nullptr;
    class QComboBox* m_langCombo  = nullptr;
    class QPushButton* m_btnStart = nullptr;

    // Page 2 (mode)
    class QLabel*        m_modeTitle = nullptr;
    class QRadioButton*  m_rbSuit   = nullptr;
    class QRadioButton*  m_rbSuitG  = nullptr;
    class QPushButton*   m_btnConnectSuit   = nullptr;
    class QPushButton*   m_btnConnectGloves = nullptr;
    class QLabel*        m_suitDot    = nullptr;
    class QLabel*        m_suitText   = nullptr;
    class QLabel*        m_gloveDot   = nullptr;
    class QLabel*        m_gloveText  = nullptr;
    class QLabel*        m_modeHint   = nullptr;
    class QComboBox*     m_cbxSuit      = nullptr;   // Xsens Link 240 / Awinda 60
    class QComboBox*     m_cbxTransport = nullptr;   // COM / WiFi
    class QLabel*        m_wifiHint   = nullptr;     // WiFi: "join same network" hint

    // Page 3 (dims)
    class QLabel*          m_dimsTitle = nullptr;
    class QLabel*          m_lblHeight = nullptr;
    class QLabel*          m_lblFoot   = nullptr;
    class QLabel*          m_lblArm    = nullptr;   // FIX: размах рук перенесён сюда
    class QLabel*          m_lblLeg    = nullptr;   // FIX: длина ноги перенесена сюда
    // FIX issue 5: новые опциональные поля (0 = вычислить из роста).
    class QLabel*          m_lblHip    = nullptr;
    class QLabel*          m_lblShoulder = nullptr;
    class QLabel*          m_lblTrunk  = nullptr;
    class QDoubleSpinBox*  m_height = nullptr;
    class QDoubleSpinBox*  m_foot   = nullptr;
    class QDoubleSpinBox*  m_arm    = nullptr;      // FIX: arm span (опц., 0 = по росту)
    class QDoubleSpinBox*  m_leg    = nullptr;      // FIX: leg length (опц., 0 = по росту)
    class QDoubleSpinBox*  m_hip      = nullptr;    // FIX issue 5: hip width
    class QDoubleSpinBox*  m_shoulder = nullptr;    // FIX issue 5: shoulder width
    class QDoubleSpinBox*  m_trunk    = nullptr;    // FIX issue 5: trunk length
    class QLabel*          m_dimsHint  = nullptr;

    // Page 4 (calibration)
    // Single-pose T/N selection is gone — we always run a T→N sequence.
    class QLabel*        m_calibTitle = nullptr;
    class QLabel*        m_poseImage  = nullptr;
    class QLabel*        m_poseHint   = nullptr;
    class QLabel*        m_connBadge  = nullptr;     // "suit: connected" /(green) "disconnected" (red)
    class QLabel*        m_countLabel = nullptr;
    class QLabel*        m_stillLabel = nullptr;
    class QProgressBar*  m_countdownBar = nullptr;
    class QProgressBar*  m_readyBar     = nullptr;
    class QPushButton*   m_btnCalibBegin = nullptr;
    class QLabel*        m_calibStatus   = nullptr;

    QTimer m_countTimer;
    QTimer m_captureTimer;
    QTimer m_statusTimer;                              // live connection watcher
    int    m_countTicksLeft = 0;
    int    m_goodSamples    = 0;
    qint64 m_lastSampleCtr  = -1;
    std::vector<std::array<Quat, kXsensSegmentCount>> m_samples;
    std::array<Quat, kXsensSegmentCount> m_prevSnap{};
    bool   m_havePrev = false;
    bool   m_calibComplete = false;
    int    m_settleGen = 0;   // bumped to invalidate pending settle / singleShot callbacks
    bool   m_suitBtnCooldown = false;  // debounce window after a Connect-suit click

    // Double-pose calibration state machine.
    //
    //  Idle
    //   → (user presses Start) → PrepT (3 s countdown, show T-pose image)
    //   → CaptureT (accumulate for kCalibrationSamples)
    //   → PrepN (3 s countdown, switch image to N-pose)
    //   → CaptureN (accumulate again)
    //   → Settle (push s2s into receiver, wait for FusionAhrs to converge)
    //   → Done (goNext())
    enum class CalibPhase { Idle, PrepT, CaptureT, SettleT, PrepN, CaptureN, Settle, PrepK, CaptureK, Done };
    CalibPhase m_phase = CalibPhase::Idle;

    // Per-pose accumulators (acc_magn, gyr_bias, mag_magn) — T and N.
    std::array<QVector3D, kXsensSegmentCount> m_accAccumT{};
    std::array<QVector3D, kXsensSegmentCount> m_gyrAccumT{};
    std::array<QVector3D, kXsensSegmentCount> m_magAccumT{};
    std::array<double,    kXsensSegmentCount> m_accMagAccumT{};
    std::array<int,       kXsensSegmentCount> m_accumCountT{};

    // FIX (gloves polish): finger ergonomics accumulators (T-pose only).
    // На каждом good-frame копится сумма Manus EMA-smoothed degrees,
    // а в момент CaptureT-завершения делится на m_fingerAccumCount
    // → записывается в m_result.fingerBaselineR/L.
    std::array<double, 20> m_fingerAccumR{};
    std::array<double, 20> m_fingerAccumL{};
    int                    m_fingerAccumCount = 0;

    std::array<QVector3D, kXsensSegmentCount> m_accAccumN{};
    std::array<QVector3D, kXsensSegmentCount> m_gyrAccumN{};
    std::array<QVector3D, kXsensSegmentCount> m_magAccumN{};
    std::array<double,    kXsensSegmentCount> m_accMagAccumN{};
    std::array<int,       kXsensSegmentCount> m_accumCountN{};

    std::array<QVector3D, kXsensSegmentCount> m_accAccumK{};
    std::array<QVector3D, kXsensSegmentCount> m_gyrAccumK{};
    std::array<QVector3D, kXsensSegmentCount> m_magAccumK{};
    std::array<double,    kXsensSegmentCount> m_accMagAccumK{};
    std::array<int,       kXsensSegmentCount> m_accumCountK{};

    std::array<QVector3D, kXsensSegmentCount> m_gyrSqAccumT{};
    std::array<QVector3D, kXsensSegmentCount> m_gyrSqAccumN{};
    std::array<QVector3D, kXsensSegmentCount> m_gyrSqAccumK{};
    std::array<std::array<double, 6>, kXsensSegmentCount> m_magOuterAccumT{};
    std::array<std::array<double, 6>, kXsensSegmentCount> m_magOuterAccumN{};
    std::array<std::array<double, 6>, kXsensSegmentCount> m_magOuterAccumK{};

    std::array<QVector3D, kXsensSegmentCount>* m_gyrSqAccum    = nullptr;
    std::array<std::array<double, 6>, kXsensSegmentCount>* m_magOuterAccum = nullptr;

    std::array<QVector3D, kXsensSegmentCount>* m_accAccum    = nullptr;
    std::array<QVector3D, kXsensSegmentCount>* m_gyrAccum    = nullptr;
    std::array<QVector3D, kXsensSegmentCount>* m_magAccum    = nullptr;
    std::array<double,    kXsensSegmentCount>* m_accMagAccum = nullptr;
    std::array<int,       kXsensSegmentCount>* m_accumCount  = nullptr;

    class QLabel* m_calibQuality = nullptr;

    // Page 5 (ready)
    class QLabel*        m_readyTitle = nullptr;
    class QLabel*        m_readySummary = nullptr;
    class QPushButton*   m_btnFinish = nullptr;

    // Nav
    class QPushButton* m_btnBack = nullptr;
    class QPushButton* m_btnNext = nullptr;

    void buildPages();
    void retranslate();
    void updateNavButtons();
    void refreshPoseImage();
    void setBadge(QLabel* lab, const QString& txt, bool green);

    // True while a calibration run is in progress (any phase except Idle/Done),
    // INCLUDING the timer-less "settle" phases. Gates navigation and tells us
    // when a lost connection must abort the run.
    bool calibBusy() const {
        return m_phase != CalibPhase::Idle && m_phase != CalibPhase::Done;
    }
    // Cancel an in-progress calibration: invalidate pending settle callbacks,
    // stop timers, reset progress bars and accumulated samples back to Idle.
    void abortCalibration();

protected:
    void closeEvent(QCloseEvent*) override;
};

// ============================================================================
//  SensorIndicatorsPanel — replaces the old control/status side-panel.
//  Shows current session mode (read-only), a grid of 17 suit-tracker live
//  indicators, finger-chain indicators (when gloves are active) and a
//  pause/resume control.
// ============================================================================

class SensorIndicatorsPanel : public QWidget {
    Q_OBJECT
public:
    SensorIndicatorsPanel(bool useGloves, QWidget* parent=nullptr);

    void setMode(bool useGloves);
    void setSuitLive(bool live, const QString& detail);
    void setFps(double hz);
    void setSessionRunning(bool running);

    // Called each render tick with the latest snapshot — lights up per-tracker
    // indicators whose segValid[i] is true within a fresh window.
    void updateFromPose(const SuitPose& f);

    // External sync of freeze state (updates button text/check without signal).
    void setFreezeState(bool frozen);

    void setActorDefaults(const ActorConfig& a);

signals:
    void resetClicked();              // Reset: feet-on-floor, scene origin
    void freezeToggled(bool frozen);  // one toggle
    void actorChanged(ActorConfig actor);

private:
    bool           m_useGloves = false;
    bool           m_running   = false;
    bool           m_frozen    = false;

    QLabel*        m_lblMode      = nullptr;
    QLabel*        m_lblSuit      = nullptr;
    QLabel*        m_lblFps       = nullptr;
    QLabel*        m_lblBattery   = nullptr;
    QLabel*        m_lblSession   = nullptr;
    QPushButton*   m_btnReset   = nullptr;
    QPushButton*   m_btnFreeze  = nullptr;

    // 17 tracker indicators (the physical sensors).  Indices follow
    // segmentFromLocationId → SkeletonXsens segment index.
    struct TrackerCell { QLabel* dot = nullptr; QLabel* name = nullptr; };
    std::array<TrackerCell, kXsensSegmentCount> m_trackers{};

    // 10 finger chain indicators (5 per hand).
    struct FingerCell { QLabel* dot = nullptr; QLabel* name = nullptr; };
    std::array<FingerCell, 10> m_fingers{};
    QWidget*   m_fingersBox = nullptr;

    QWidget*           m_bodyBox    = nullptr;
    QDoubleSpinBox*    m_bodyHeight = nullptr;
    QDoubleSpinBox*    m_bodyFoot   = nullptr;
    QDoubleSpinBox*    m_bodyArm    = nullptr;
    QDoubleSpinBox*    m_bodyLeg    = nullptr;
    QTimer*            m_bodyDebounce = nullptr;
    bool               m_bodySuppress = false;
    QLabel*            m_bodyHeader = nullptr;
    QLabel*            m_bodySub    = nullptr;
    QLabel*            m_bodyLblH   = nullptr;
    QLabel*            m_bodyLblF   = nullptr;
    QLabel*            m_bodyLblA   = nullptr;
    QLabel*            m_bodyLblL   = nullptr;

    void retranslate();
    void setDot(QLabel* dot, bool live);
};

// ============================================================================
//  3-D viewport — QOpenGLWidget that renders the skeleton using GL_LINES
// ============================================================================

class MocapViewport : public QOpenGLWidget {
    Q_OBJECT
public:
    MocapViewport(const ActorConfig& actor, const std::string& pose,
                  QWidget* parent=nullptr);

    // Thread-safe: called from Qt timer on the GUI thread.
    void updatePose(const std::array<Quat, kXsensSegmentCount>& orientations,
                    const QVector3D& rootPos);

    // Updates hand visualisation (positions in world frame, meters).
    void updateHands(bool haveGloves,
                     const std::array<QVector3D, kFingerSegmentsHand>& right,
                     const std::array<QVector3D, kFingerSegmentsHand>& left);

    // Rebuild the skeleton when the pose preset changes.
    void setPose(const std::string& pose);

    void setActor(const ActorConfig& actor);
    ActorConfig actor() const { return m_actor; }
    void setLocoVerbose(bool v) { m_loco.setVerbose(v); }
    // Bind the processing rate so the locomotion solver re-derives its
    // @90 Hz-tuned timings for the active suit cadence (Link 240 / Awinda 60),
    // and cap the GL repaint at the display rate (never above ~90 Hz).
    void setProcRate(double hz) {
        m_loco.setProcRate(hz);
        const double cap = (hz > 90.0) ? 90.0 : (hz > 1.0 ? hz : 90.0);
        m_paintMinIntervalSec = 1.0 / cap;
    }

    // Reset: capture current pelvis world XY, apply offset so pelvis goes to
    // scene origin (0,0), feet rest on floor (Z from FK, no forcing).
    void resetSceneOrigin();
    // Freeze toggle: ON keeps skeleton at scene origin while user moves IRL.
    // OFF resumes natural scene-space tracking.
    void setFreezeXY(bool frozen);
    bool isFrozen() const { return m_freezeXY; }

    void setWristCfg(bool right, const WristAnatomicalCfg& cfg) {
        if (right) m_wristCfgR = cfg; else m_wristCfgL = cfg;
    }
    WristAnatomicalCfg wristCfg(bool right) const {
        return right ? m_wristCfgR : m_wristCfgL;
    }

    // FIX (gloves polish): pin wrist drift-correction anchor to the
    // hand-vs-forearm rotation captured during T-pose calibration.
    // Без T-pose anchor anchor берётся от первого "lock" момента в
    // runtime — может оказаться в кривой позе.  С anchor — кисть всегда
    // стремится "продолжать" forearm в T-pose геометрии (ладонь вниз).
    void setTposeHandAnchor(const Quat& forearmR_T, const Quat& handR_T,
                            const Quat& forearmL_T, const Quat& handL_T);

    // FIX (T-pose foot direction reference): pin foot-yaw anchor from
    // T-pose calibration.  В T-pose обе стопы смотрят вперёд (+X в
    // pelvis-yaw-frame).  Сохраняем foot orientation rel-to-pelvis-yaw
    // как ground truth — после этого foot-yaw drift correction работает
    // не относительно lowerLeg (который меняется при flex'е колена), а
    // относительно стабильного pelvis-yaw frame.
    void setTposeFootAnchor(const Quat& pelvis_T,
                            const Quat& rFoot_T, const Quat& lFoot_T);

    // FIX (cross-legged direction protection): hints set externally by
    // MainWindow::onRenderTick.  При cross-legged drift correction
    // attenuates по crossConf чтобы не разворачивать стопу неправильно.
    void setCrossLeggedHints(bool r, bool l, double cR, double cL) {
        m_crossLeggedR = r; m_crossLeggedL = l;
        m_crossLeggedConfR = cR; m_crossLeggedConfL = cL;
    }

    QSize sizeHint() const override { return QSize(800, 600); }

    const std::array<Quat, kXsensSegmentCount>& filteredOrient() const { return m_orient; }

    // --- Per-segment drift-lock diagnostics (for the -test RENDER SNAPSHOT) ---
    // These expose the REAL filter state so the log can distinguish a held
    // (drift-locked) segment from one tracking live motion, and show how much
    // accumulated drift the anchor is correcting.
    bool   segLocked(int i)      const { return m_locked[i]; }
    bool   segAnchorValid(int i) const { return m_anchorValid[i]; }
    double segAngVelLP(int i)    const { return m_angVelLP[i]; }      // smoothed |ω| deg/s
    double segAngAcc(int i)      const { return m_dbgAngAcc[i]; }     // |Δω|/dt deg/s² (lock gate)
    double segStillTicks(int i)  const { return m_stillTicks[i]; }    // seconds held still
    Quat   segDriftLocal(int i)  const { return m_driftLocal[i]; }    // accumulated drift

    // Last on-screen pelvis position (post-locomotion + scene shift). Used by
    // the live streamer so the receiver gets the same dynamic pelvis as the
    // viewport — without this pelvis is static and the rig "doesn't move".
    QVector3D lastRenderedPelvis() const { return m_lastRenderedPelvis; }

    QVector3D tickLoco(const std::array<Quat, kXsensSegmentCount>& q,
                       const QVector3D& fkRHeel,
                       const QVector3D& fkRBall,
                       const QVector3D& fkRTip,
                       const QVector3D& fkLHeel,
                       const QVector3D& fkLBall,
                       const QVector3D& fkLTip,
                       double tSec);
    QVector3D lastLocoOffset() const { return m_lastLocoOffset; }

    QVector3D sceneShift() const { return m_sceneShift; }
    float sceneYaw() const { return m_sceneYaw; }
    QVector3D freezeAnchor() const { return m_freezeAnchor; }

    // Single-source operator-view diagnostics for the -test [VIEW] dump: the
    // exact keypoints the operator sees this frame (after loco offset + floor
    // clamp + sceneYaw + sceneShift + freeze), the floor-clamp lift applied,
    // and the live locomotion state.  Captured in drawSkeleton(); never
    // recomputed in the logger.
    const std::array<QVector3D, kXsensKeypointCount>& lastRenderedKeypoints() const { return m_lastRenderedKp; }
    float    lastFloorClamp() const { return m_lastFloorClamp; }
    LocoDiag locoDiag()       const { return m_loco.diag(); }

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent (QMouseEvent*) override;
    void wheelEvent     (QWheelEvent*) override;

private:
    ActorConfig                m_actor;
    std::unique_ptr<SkeletonXsens> m_skel;

    std::array<Quat, kXsensSegmentCount> m_orient{};
    QVector3D                  m_root{0, 0, 0};

    bool                       m_haveGloves = false;
    std::array<QVector3D, kFingerSegmentsHand> m_rightHand{};
    std::array<QVector3D, kFingerSegmentsHand> m_leftHand{};

    // Locomotion solver — runs after FK, provides scene-frame walking
    // with MVN-style foot-contact + drift protection.
    LocomotionSolver m_loco;

    // Scene reset / freeze (does NOT touch motion pipeline — pure view-side).
    QVector3D m_sceneShift{0, 0, 0};  // applied to every kp AFTER loco
    float     m_sceneYaw   = 0.0f;    // rad, rotation about Z (reset pins yaw)
    bool      m_freezeXY   = false;
    QVector3D m_freezeAnchor{0, 0, 0};  // pelvis world XY at freeze moment
    QVector3D m_lastRenderedPelvis{0, 0, 0};  // cached for resetSceneOrigin()
    QVector3D m_lastLocoOffset{0, 0, 0};
    // -test [VIEW]: the keypoints the operator literally sees this frame and the
    // floor-clamp lift applied, captured in drawSkeleton() (single source).
    std::array<QVector3D, kXsensKeypointCount> m_lastRenderedKp{};
    float     m_lastFloorClamp = 0.0f;

    // Per-segment drift-lock: if a bone's angular velocity has been <0.8 deg/s
    // for >0.5 s, replace its quat with the locked value. Drift shows up as
    // a low, steady angular velocity (0.05-0.3 deg/s, nearly constant axis);
    // real motion has higher peak + accelerations. We also track angular
    // acceleration — if it's near zero (drift sig) even when speed is non-
    // zero, we lock. Real motion has angular-accel spikes that release lock.
    std::array<Quat, kXsensSegmentCount>   m_lockQuat{};      // held output
    std::array<bool, kXsensSegmentCount>   m_locked{};
    std::array<double, kXsensSegmentCount> m_angVelPrev{};    // last |ω|
    std::array<double, kXsensSegmentCount> m_angVelLP{};      // smoothed |ω|
    std::array<double, kXsensSegmentCount> m_dbgAngAcc{};     // |Δω|/dt deg/s² (-test lock gate)
    std::array<double, kXsensSegmentCount> m_stillTicks{};    // consecutive
    std::array<Quat, kXsensSegmentCount>   m_prevQ{};
    std::array<Quat, kXsensSegmentCount>   m_outPrevQ{};
    std::array<double, kXsensSegmentCount> m_unlockBlend{};
    // FIX (terminator smoothing): симметричный lock-in blend.  Раньше
    // m_locked[i]=true моментально подставлял m_lockQuat[i] в filtered[i],
    // что создавало visible 0.5°-1° step при входе в lock.  С этим blend
    // 7-кадровый ramp (~77ms @ 90Hz) делает переход незаметным.
    std::array<double, kXsensSegmentCount> m_lockBlend{};
    std::array<Quat, kXsensSegmentCount>   m_anchorLocal{};
    std::array<bool, kXsensSegmentCount>   m_anchorValid{};
    std::array<Quat, kXsensSegmentCount>   m_driftLocal{};
    // FIX (gloves polish): когда true, m_anchorLocal[SEG_RHand/LHand] было
    // зафиксировано из T-pose calibration и больше не обновляется по
    // lock-моментам — кисть всегда "тянется" к T-pose геометрии.
    bool                                    m_tposeHandAnchorValid = false;

    // FIX (T-pose foot direction reference): foot-quat в pelvis-yaw frame
    // во время T-pose calibration.  Используется в foot-yaw drift correction
    // как stable reference (вместо lowerLeg, который меняется при flex'е).
    Quat                                    m_tposeFootRefPelR{1, 0, 0, 0};
    Quat                                    m_tposeFootRefPelL{1, 0, 0, 0};
    bool                                    m_tposeFootAnchorValid = false;

    // FIX (cross-legged): hints set externally per-frame.
    bool                                    m_crossLeggedR = false;
    bool                                    m_crossLeggedL = false;
    double                                  m_crossLeggedConfR = 0.0;
    double                                  m_crossLeggedConfL = 0.0;
    // FIX issue 11/7: время непрерывной "тишины" сегмента (allCalm).
    // Когда сегмент 5+ секунд спокоен — применяем дополнительную damped
    // twist коррекцию (для wrist в issue 11) или yaw-коррекцию (для foot
    // в issue 7), чтобы убрать накопленный gyro дрейф.
    std::array<double, kXsensSegmentCount> m_calmSeconds{};
    bool                                    m_havePrevQ = false;
    double                                  m_lastRenderT = 0.0;

    // GL repaint throttle.  The solver/record/stream tick runs at the full suit
    // rate in MainWindow::onRenderTick; we only redraw at the display rate.
    double                                  m_lastPaintSec = 0.0;
    double                                  m_paintMinIntervalSec = 1.0 / 90.0;

    WristAnatomicalCfg m_wristCfgR{};
    WristAnatomicalCfg m_wristCfgL{};

    // camera (orbit).  pitch > 0 = camera above the target looking down.
    float m_yaw   = 35.0f;
    float m_pitch = 12.0f;
    float m_dist  = 3.2f;
    QPoint m_lastMouse;

    void drawFloor();
    void drawReferenceFrame();
    void drawSkeleton();
};

// ============================================================================
//  Recording + Live-stream support
// ============================================================================

// Single frame of recorded data.  We store everything we'd need to write a
// BVH or FBX file later — segment quaternions (already calibration-offset
// applied by MainWindow::onRenderTick), pelvis position, wall-clock t.
struct RecordedFrame {
    double  t         = 0.0;           // seconds since record start
    std::array<Quat,      kXsensSegmentCount> segQuat{};
    QVector3D                                  pelvisPos{0, 0, 0};
    // Optional hand data — kept so finger authors don't lose gloves.
    bool    hasGloves = false;
    std::array<Quat, kFingerSegmentsHand> rightGloveQ{};
    std::array<Quat, kFingerSegmentsHand> leftGloveQ{};
};

enum class RecordFormat   { BVH, FBX };
enum class RecordQuality  { Normal, HdPostProcessing };

struct RecordSettings {
    RecordFormat  format  = RecordFormat::BVH;
    RecordQuality quality = RecordQuality::Normal;
    int           fps     = 30;        // 24 / 30 / 60
};

// Small HUD shown over the viewport while recording — frame count, elapsed
// time, and a Stop button.  Repainted every render tick.
class RecordHud : public QWidget {
    Q_OBJECT
public:
    explicit RecordHud(QWidget* parent = nullptr);
    void updateStats(qint64 frames, double elapsedSec);
    void setFormatLabel(const QString& text);

signals:
    void stopClicked();

private:
    class QLabel*      m_lblFormat = nullptr;
    class QLabel*      m_lblFrames = nullptr;
    class QLabel*      m_lblTime   = nullptr;
    class QPushButton* m_btnStop   = nullptr;
};

// Record wizard (BVH/FBX → Normal/HD → 24/30/60 → Start).  Opened from the
// "Запись" tab.  On accept() the chosen RecordSettings lives in result().
class RecordWizard : public QDialog {
    Q_OBJECT
public:
    explicit RecordWizard(SuitType suit = SuitType::Awinda, QWidget* parent = nullptr);
    RecordSettings result() const { return m_result; }

private slots:
    void goNext();
    void goBack();

private:
    SuitType m_suit = SuitType::Awinda;   // gates the high-rate (120/240) fps options
    class QStackedWidget* m_pages = nullptr;
    int   m_pageIdx = 0;

    class QComboBox* m_format  = nullptr;
    class QComboBox* m_quality = nullptr;
    class QComboBox* m_fps     = nullptr;

    class QPushButton* m_btnNext   = nullptr;
    class QPushButton* m_btnBack   = nullptr;
    class QPushButton* m_btnStart  = nullptr;
    class QPushButton* m_btnCancel = nullptr;

    RecordSettings m_result{};

    void buildPages();
    void updateNav();
};

// Live-stream wizard.  Lets the operator pick a target plugin and a port,
// then start sending live pose data.  Concrete transport lives in the
// sender class — wired after the plugin protocol research lands.
// Live-stream target.  BOTH targets use the same MVN MXTP wire protocol AND
// the same wire coordinate frame: the MVN default **Z-Up, right-handed**,
// which is identical to our internal NWU (X-fwd, Y-left, Z-up).  We therefore
// emit one identical stream to either — no per-target axis conversion (the UE
// LiveLink plugin does its own Z-up-RH→UE-LH Y-flip; the Blender add-on remaps
// from the same Z-up global frame).  The enum only selects UI defaults such as
// the subject name / port, not the coordinate math.
enum class LiveTarget { BlenderMVN, UnrealLiveLink };

struct LiveSettings {
    LiveTarget    target    = LiveTarget::BlenderMVN;
    QString       host      = "127.0.0.1";
    int           port      = 9763;           // MVN default; overridden in UI
    bool          useGloves = false;
    int           fps       = 60;             // 24 / 30 / 60 — UI throttle
    // FIX (stream polish): gloves frame = 24 header + 63 seg × 32 = 2040 байт,
    // > 1500 MTU.  На loopback ядро делает IP fragmentation, на LAN risk потери.
    // Если true — body и fingers идут в два отдельных UDP datagram'а через
    // MXTP dgCounter splitting (bit 7 = last).  Оставлено в false до verify
    // что Blender plugin handles multi-datagram reassembly.
    bool          splitGloveDatagrams = false;
    // FIX (stream polish): однократный hex-dump первого MXTP02 фрейма в
    // stdout — для byte-уровня проверки против MVN протокол spec.
    // Используется в -test режиме для diff против известного хорошего
    // MVN Animate output.
    bool          debugDumpFirstFrame = false;
    // -test verbose stream logging: periodic [STREAM SNAPSHOT] of the EXACT
    // per-segment wire values (segId, position, quaternion delta) that leave
    // the socket, so the log proves what the plugins actually receive.
    bool          verboseLog = false;
    // T-pose origin position (meters) for each of 23 Xsens body segments.
    // Plugin (LiveLinkMvnSource) кладёт это в FTransform.Scale3D и потом
    // ULiveLinkMvnRetargetAsset делит unrealLength/xsensLength для масштаба
    // позиции pelvis. Если оставить нули — pelvis улетает на ~47x.
    std::array<QVector3D, kXsensSegmentCount> tposeOriginM{};
    // Default T-pose segment angles per skeleton.defAngFor(i).  Сохраняется
    // как метаданные на стороне отправителя; в wire не уходит — плагины
    // строят свои rest poses из MXTP13 origins + bone hierarchy.
    std::array<Quat, kXsensSegmentCount> defAngT{};
};

class LiveStreamWizard : public QDialog {
    Q_OBJECT
public:
    explicit LiveStreamWizard(SuitType suit = SuitType::Awinda, QWidget* parent = nullptr);
    LiveSettings result() const { return m_result; }

private:
    SuitType             m_suit   = SuitType::Awinda;  // gates 120/240 fps options
    class QComboBox*     m_target = nullptr;
    class QComboBox*     m_host   = nullptr;
    class QSpinBox*      m_port   = nullptr;
    class QComboBox*     m_fps    = nullptr;        // 24/30/60 (+120/240 for Link)
    class QPushButton*   m_btnStart = nullptr;
    class QPushButton*   m_btnCancel = nullptr;
    LiveSettings         m_result{};
};

// Live-stream sender.  Consumes per-frame skeleton state from MainWindow
// and writes plugin-native packets on a UDP socket.  Implementation
// wrappers differ per target (MVN MXTP vs. XsensLivc wire format).
class LiveStreamSender : public QObject {
    Q_OBJECT
public:
    explicit LiveStreamSender(QObject* parent = nullptr);
    ~LiveStreamSender() override;

    bool start(const LiveSettings& cfg, QString* err = nullptr);
    void stop();
    bool isRunning() const { return m_running; }

    void recalibrate();

    void setTposeBaseline(const std::array<Quat, kXsensSegmentCount>& tposeQ,
                          const QVector3D& tposePelvis);

    void pushFrame(quint32 sample,
                   const std::array<Quat, kXsensSegmentCount>& segQuat,
                   const QVector3D& pelvisPos);

    // Glove variant — sends body segments then right+left hand fingers
    // (20 each).  Plugins auto-detect gloves via the header's
    // fingerSegmentCount field (Blender MVN plugin: segmentCount ≥ 32 ⇒
    // gloves present). Safe to call every tick; if gloves data is all-
    // zero the plugin gracefully ignores finger segments.
    void pushFrameWithGloves(quint32 sample,
        const std::array<Quat, kXsensSegmentCount>& segQuat,
        const QVector3D& pelvisPos,
        const std::array<Quat, kFingerSegmentsHand>& rightGloveQ,
        const std::array<QVector3D, kFingerSegmentsHand>& rightGloveP,
        const std::array<Quat, kFingerSegmentsHand>& leftGloveQ,
        const std::array<QVector3D, kFingerSegmentsHand>& leftGloveP);

    const LiveSettings& settings() const { return m_cfg; }

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
    LiveSettings m_cfg{};
    bool         m_running = false;
};

// ============================================================================
//  MainWindow — wires everything together + black/orange theme.
// ============================================================================

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    // Takes the already-started receiver and the wizard result (calibration
    // reference + actor dimensions + hardware mode).
    MainWindow(MocapReceiver* rx,
               const NewSessionWizard::Result& wizardResult,
               bool testMode);
    ~MainWindow() override;

    void setWristConstraintEnabled(bool enabled);

private slots:
    void onConnStatusChanged(int status, const QString& detail);
    void onGloveStatus(bool up);
    void onFps(double hz);
    void onRenderTick();
    void onPauseSession();
    void onResumeSession();
    void onOpenLiveWizard();
    void onOpenRecordWizard();
    void onRecordStop();

private:
    NewSessionWizard::Result m_setup;
    bool            m_test = false;

    MocapReceiver*  m_rx       = nullptr;
    MocapViewport*  m_viewport = nullptr;
    QTimer          m_renderTimer;
    SensorIndicatorsPanel* m_panel = nullptr;

    bool            m_sessionRunning = true;

    // Suit-driven processing rate (Hz): the whole solve / record / stream loop
    // ticks at this rate (Link → 240, Awinda → 60).  GL repaint is throttled
    // separately inside the viewport so we never draw faster than the display.
    double          m_procRateHz   = 90.0;

    // v4: local skeleton instance — used by onRenderTick to compose the
    // full world orientation of each wrist via defAngFor() before FK-ing
    // the Manus-local finger positions.  Kept separate from the viewport's
    // own m_skel so the render-thread math never touches the GL state.
    std::unique_ptr<SkeletonXsens> m_skel;

    // Recording state.
    bool                       m_recording   = false;
    bool                       m_finishing   = false;  // inside finishRecording() modal loops
    bool                       m_takePending = false;  // a recorded take is held unsaved
    RecordSettings             m_recCfg{};
    std::vector<RecordedFrame> m_recBuffer;
    bool                       m_recOverflowWarned = false;  // warn once if take outgrows reserve
    bool                       m_recHardCapped     = false;  // buffer frozen at the ~60 min hard cap
    qint64                     m_recStartMs = 0;
    qint64                     m_streamStartMs = 0;
    // HUD overlay in top-left of viewport showing REC/STREAM mode + seconds.
    QLabel*                    m_modeHud = nullptr;
    QTimer*                    m_modeHudTimer = nullptr;
    qint64                     m_recLastSample = -1;
    RecordHud*                 m_hud = nullptr;

    // Live streaming.
    LiveStreamSender*          m_streamer = nullptr;

    void logTest(const std::string& msg) const;
    void startRecording(const RecordSettings& cfg);
    void finishRecording();
    void layoutHud();

protected:
    void closeEvent(QCloseEvent*) override;
    void resizeEvent(QResizeEvent*) override;
};

// ============================================================================
//  Black / orange Qt stylesheet
// ============================================================================

extern const char* kStyleSheet;

// ============================================================================
//  Entry-point helpers
// ============================================================================

struct CliArgs {
    bool test   = false;
    bool gloves = false;                     // --gloves : enable hand skeleton in -test
    bool wristConstraint = false;
    SuitType suit = SuitType::Awinda;        // -test forces Link; --awinda/--link override
};
CliArgs parseCli(int argc, char** argv);

// Logger used in -test mode (flushes immediately).
void testLog(const std::string& msg, bool enabled);

}  // namespace fox
