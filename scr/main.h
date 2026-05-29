
#pragma once

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
#include <QtWidgets/QCheckBox>
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

#include "foxmath.h"
#include "foxbody.h"
#include "foxergo.h"

namespace fox {

constexpr int     kXsensSegmentCount  = 23;
constexpr int     kXsensKeypointCount = 28;
constexpr int     kXsensSegmentCountWithDummies = 27;
constexpr int     kFingerSegmentsHand = 20;
constexpr double  kRenderFps          = 90.0;
constexpr double  kStaleSeconds       = 2.0;
constexpr int     kCalibrationSamples = 720;
constexpr int     kCountdownSeconds   = 3;

enum class SuitType { Awinda, Link };
// §2035/§2088 нативная частота костюма: Link 240 Гц, Awinda 60 Гц (полнотельная, 17 сенсоров).
//   Это дефолт/фолбэк/подсказка UI — в работе частота берётся из реального железа
//   (deviceUpdateRate, main.cpp ~5745) и пробрасывается в обработку (см. SuitPose::nativeHz). (formules.txt)
constexpr double nativeRateHz(SuitType s) { return s == SuitType::Link ? 240.0 : 60.0; }

inline int ticksFor(double seconds, double rateHz) {
    const long n = std::lround(seconds * rateHz);
    return n < 1 ? 1 : static_cast<int>(n);
}
// dt0=1/90 — ЭТАЛОННАЯ частота, при которой заданы коэффициенты a0 (НЕ частота костюма):
//   функция сохраняет постоянную времени tau при любом реальном dt. Менять dt0 нельзя — сместит тюнинг.
inline double rateAdjustAlpha(double a0, double dt, double dt0 = 1.0 / 90.0) {
    if (a0 <= 0.0) return 0.0;
    if (a0 >= 1.0) return 1.0;
    const double tau = -dt0 / std::log(1.0 - a0);
    return 1.0 - std::exp(-dt / tau);
}

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

struct WristAnatomicalCfg {

    // §XIX/§5 кисть YXZ: предел сгибания запястья ±80° (1.3963 рад), локтевое/лучевое
    //   отклонение ±30° (0.5236 рад) (formules.txt стр.645, 1530)
    double maxFlexRad   = 1.3962634015954636;
    double maxLatDevRad = 0.5235987755982988;
    double twistWeight  = 1.0;
    bool   enabled      = true;
};

struct FingerJointLimit {
    double spreadMin;
    double spreadMax;
    double flexMin;
    double flexMax;
};

struct SuitPose {
    quint64 sampleCounter = 0;
    double  recvTime      = 0.0;
    // §2035 разрешённая нативная частота железа (deviceUpdateRate, иначе ожидаемая по типу костюма):
    //   Awinda 60 Гц / Link 240 Гц / прочие модели 100/120 — обработка подстраивается под неё (formules.txt)
    double  nativeHz      = 0.0;

    std::array<Quat, kXsensSegmentCount> quat;
    std::array<bool, kXsensSegmentCount> segValid{};

    std::array<QVector3D, kXsensSegmentCount> accSensor{};
    std::array<QVector3D, kXsensSegmentCount> gyrSensor{};
    std::array<QVector3D, kXsensSegmentCount> magSensor{};

    std::array<QVector3D, kXsensSegmentCount> linAccBody{};
    std::array<QVector3D, kXsensSegmentCount> linVelWorld{};

    QVector3D pelvisPos{0, 0, 0};

    std::array<double, kXsensSegmentCount> segLastT{};

    int batteryLevel = -1;

    bool                                      hasGloves = false;
    std::array<Quat,      kFingerSegmentsHand> rightGloveQ{};
    std::array<Quat,      kFingerSegmentsHand> leftGloveQ{};
    std::array<QVector3D, kFingerSegmentsHand> rightGloveP{};
    std::array<QVector3D, kFingerSegmentsHand> leftGloveP{};

    SuitPose() { for (auto& q : quat) q = Quat(1, 0, 0, 0); }
};

constexpr int kFingerChainCount    = 5;
constexpr int kFingerChainLen      = 4;
extern const int kFingerChains[kFingerChainCount][kFingerChainLen];
extern const char* kFingerChainNames[kFingerChainCount];

struct ActorConfig {
    double heightCm        = 175.0;
    double footLengthCm    = 26.0;
    double armSpanCm       = 0.0;
    double legLengthCm     = 0.0;

    double hipWidthCm      = 0.0;
    double shoulderWidthCm = 0.0;
    double trunkLengthCm   = 0.0;

    double upperArmCm      = 0.0;
    double forearmCm       = 0.0;
    double handCm          = 0.0;
    double thighCm         = 0.0;
    double shankCm         = 0.0;

    fox::body::Gender gender = fox::body::GenderMale;

    bool   useGloves       = false;
};

struct ContactState {
    bool   rightDown  = false;
    bool   leftDown   = false;
    double rightAngV  = 0.0;
    double leftAngV   = 0.0;
};

struct LocoDiag {
    int       pose          = 0;
    int       poseTicks     = 0;
    int       support       = 2;
    double    confR         = 0.0,  confL = 0.0;
    bool      committedR    = false, committedL = false;
    double    rAngV         = 0.0,  lAngV = 0.0;
    double    pelvisAngV    = 0.0,  pelvisYawAngV = 0.0;
    double    footPitchZR   = 0.0,  footPitchZL = 0.0;
    double    contactBlendR = 0.0,  contactBlendL = 0.0;
    double    heelLiftConfR = 0.0,  heelLiftConfL = 0.0;
    bool      heelLiftR     = false, heelLiftL = false;
    double    pelvisZVel    = 0.0;
    int       airborneTicks = 0,    landedTicks = 0, zuptTicks = 0;
    double    tiltCos       = 1.0;
    QVector3D offset        {0, 0, 0};
    QVector3D anchorR       {0, 0, 0}, anchorL {0, 0, 0};

    double    rawCR         = 0.0,  rawCL        = 0.0;
    double    fkxyStableWR  = 0.0,  fkxyStableWL = 0.0;
    double    fkxyRangeR    = 0.0,  fkxyRangeL   = 0.0;
    double    yawFreezeW    = 0.0,  pelvisRotKill = 0.0;
    double    rollingWR     = 0.0,  rollingWL    = 0.0;
    double    imbalance     = 0.0;
    double    effR          = 0.0,  effL         = 0.0;
    double    maxStepXY     = 0.0;
    bool      stepClampedXY = false;
    bool      feetLifted    = false, ballistic   = false, driftAir = false;
};

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

    Quat   m_prevRQ;
    Quat   m_prevLQ;
    double m_lastT        = 0.0;
    bool   m_haveLast     = false;
    double m_rAngV        = 0.0;
    double m_lAngV        = 0.0;
    int    m_rPlantTicks  = 0;
    int    m_lPlantTicks  = 0;
    int    m_rLiftTicks   = 0;
    int    m_lLiftTicks   = 0;
    ContactState m_contact{};

    bool      m_initialised = false;
    Side      m_support     = RIGHT;
    QVector3D m_anchor      {0, 0, 0};

    // НАСТРОЙКИ ДЕТЕКТОРА КОНТАКТА/ПОЗЫ (этот блок и блок m_fkxyStableRange.. ниже):
    //   engine heuristics — эмпирически подобранный конечный автомат foot-lock/позы для рендера.
    //   Тематически родственны §XIV (контакт с полом, ZUPT) и §XXIII (активности: стоя/сидя/
    //   присед/лёжа/полёт), но конкретные пороги/скорости/тики НЕ взяты 1:1 из formules.txt.
    double m_stillRad     = 3.00;
    double m_heightMargin = 0.03;
    int    m_latchTicks   = 3;
    double m_switchMargin = 0.04;
    double m_heightMarginSlow = 0.08;

    QVector3D m_offsetLast   {0, 0, 0};

    bool      m_offsetReady  = false;

        Quat      m_prevPelvisQ      {1, 0, 0, 0};
        double    m_pelvisAngV        = 0.0;
        double    m_pelvisYawAngV     = 0.0;
        bool      m_yawFrozenPrev     = false;

        static constexpr int kFKXYWindowMax = 32;
        int       m_fkxyWindow        = 10;
        std::array<QVector2D, kFKXYWindowMax> m_rFKXY {};
        std::array<QVector2D, kFKXYWindowMax> m_lFKXY {};
        int       m_fkxyHead          = 0;
        int       m_fkxyCount         = 0;

        double    m_confR             = 0.0;
        double    m_confL             = 0.0;
        bool      m_committedR        = false;
        bool      m_committedL        = false;
        QVector3D m_anchorR           {0, 0, 0};
        QVector3D m_anchorL           {0, 0, 0};
        int       m_recentCommitTicks = 0;

        enum PoseKind { PoseUnknown = 0, PoseStand = 1, PoseSit = 2,
                        PoseSquat = 3, PoseLying = 4, PoseAirborne = 5 };
        PoseKind  m_pose              = PoseUnknown;
        PoseKind  m_posePrev          = PoseUnknown;
        int       m_poseTicks         = 0;

        int       m_zuptTicks         = 0;

        int       m_lowZTicksR        = 0;
        int       m_lowZTicksL        = 0;
        int       m_zSnapBlendTicks   = 0;

        bool      m_confRFrozenForRoll = false;
        bool      m_confLFrozenForRoll = false;
        double    m_confRFrozenValue   = 0.0;
        double    m_confLFrozenValue   = 0.0;

        double    m_footPitchZR        = 0.0;
        double    m_footPitchZL        = 0.0;

        double    m_contactBlendR      = 0.0;
        double    m_contactBlendL      = 0.0;

        double    m_heelLiftConfR      = 0.0;
        double    m_heelLiftConfL      = 0.0;
        bool      m_heelLiftR          = false;
        bool      m_heelLiftL          = false;

        // §57 длина стопы по умолчанию 0.26 м (≈ footRatio 0.152 × рост 1.75; уточняется
        //   калибровкой footSize §2006). Используется в геометрии контакта стопы (formules.txt)
        double    m_footLengthM        = 0.26;

        double    m_pelvisZVel         = 0.0;
        double    m_pelvisZPrev        = 0.0;
        bool      m_havePelvisZPrev    = false;
        int       m_airborneTicks      = 0;

        double    m_lastTiltCos        = 1.0;
        int       m_landedTicks        = 0;

        double    m_procRateHz          = 60.0;   // дефолт = частота костюма по умолчанию (Awinda 60 Гц); перезаписывается setProcRate реальной частотой
        int       m_airborneStableTicks = 5;
        int       m_landedRampTicks     = 12;
        int       m_commitFadeTicks     = 12;

        double    m_fkxyStableRange   = 0.04;

        double    m_pelvisStillRad    = 0.20;

        // §loco-clamp макс. горизонтальная скорость таза (м/с): физический потолок
        //   XY-смещения корня за кадр (maxStepXY = m_pelvisVMaxXY·dt). Щедрый —
        //   не режет реальную ходьбу/бег, ловит только аномальные скачки/dt-спайки.
        double    m_pelvisVMaxXY      = 3.0;

        double    m_confCommit        = 0.35;
        double    m_confRelease       = 0.25;
        double    m_confRiseRate      = 0.50;
        double    m_confFallRate      = 0.25;

        double    m_offsetRatePrimary = 0.40;
        double    m_offsetRateDouble  = 0.25;
        double    m_zRatePelvisMoving = 0.40;
        double    m_zRatePelvisStill  = 0.06;

        double    m_zDriveRate        = 0.02;
        int       m_poseStableTicks   = 45;

        int       m_zuptTicksThresh   = 60;

        double    m_lieTiltCosThresh  = 0.50;
        double    m_squatKneeThresh   = 0.30;
        double    m_sitKneeThresh     = 0.55;

        int       m_lowZTicksRequired = 6;

        double    m_lowZBandM         = 0.04;
        int       m_zSnapBlendFrames  = 10;

        double    m_rollAngVThresh    = 2.0;
        double    m_rollXYRangeMax    = 0.03;
        double    m_confHystBand      = 0.05;

        double    m_actorHeightM      = 1.75;

        double m_dbgRawCR = 0.0,        m_dbgRawCL = 0.0;
        double m_dbgFkxyStableWR = 0.0, m_dbgFkxyStableWL = 0.0;
        double m_dbgFkxyRangeR = 0.0,   m_dbgFkxyRangeL = 0.0;
        double m_dbgYawFreezeW = 0.0,   m_dbgPelvisRotKill = 0.0;
        double m_dbgRollingWR = 0.0,    m_dbgRollingWL = 0.0;
        double m_dbgImbalance = 0.0,    m_dbgEffR = 0.0, m_dbgEffL = 0.0;
        double m_dbgMaxStepXY = 0.0;
        bool   m_dbgStepClampedXY = false;
        bool   m_dbgFeetLifted = false, m_dbgBallistic = false, m_dbgDriftAir = false;
        bool   m_dbgConfigLogged = false;

      public:
        void setActorHeight(double h) { m_actorHeightM = std::max(0.5, h); }

        void setFootLength(double m) { m_footLengthM = std::max(0.10, m); }

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

        void setProcRate(double hz) {
            m_procRateHz = (hz > 1.0) ? hz : 60.0;
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

struct FkDiag {
    std::array<Quat,      kXsensSegmentCount>           oriented{};
    std::array<Quat,      kXsensSegmentCountWithDummies> global{};
    std::array<QVector3D, kXsensSegmentCountWithDummies> boneVec{};
    std::array<float,     kXsensSegmentCountWithDummies> len{};
    std::array<QVector3D, kXsensKeypointCount>          kp{};
    QVector3D rootPos{0, 0, 0};

    std::array<Quat, kXsensSegmentCount>                couplingOut{};

    std::array<fox::ergo::JointAngles, fox::body::kJointCount> ergo{};
};

class SkeletonXsens {
public:

    SkeletonXsens(const ActorConfig& actor, const std::string& pose);

    std::array<QVector3D, kXsensKeypointCount>
    computeKeypoints(const std::array<Quat, kXsensSegmentCount>& segOrients,
                     const QVector3D& rootPos,
                     FkDiag* diag = nullptr) const;

    std::array<Quat, kXsensSegmentCountWithDummies>
    addDummySegments(const std::array<Quat, kXsensSegmentCount>& segs) const;

    const std::array<int, kXsensSegmentCountWithDummies>& startPts() const { return m_start; }
    const std::array<int, kXsensSegmentCountWithDummies>& endPts()   const { return m_end; }
    const std::array<float, kXsensSegmentCountWithDummies>& lengths() const { return m_len; }

    static std::array<double, 5> defaultLimbCm(fox::body::Gender gender, double heightCm);

    const std::array<QVector3D, kXsensSegmentCountWithDummies>& localOffsets() const { return m_localOffset; }
    const std::array<Quat,  kXsensSegmentCount>& defaultSegAngles() const { return m_defAng; }

    const std::string& poseKind() const { return m_pose; }

    Quat defAngFor(int seg) const {
        return (seg >= 0 && seg < kXsensSegmentCount)
                   ? m_defAng[seg] : Quat(1, 0, 0, 0);
    }

    void setAccLPBodyHint(const std::array<QVector3D, kXsensSegmentCount>& acc) const {
        m_accLPBodyHint = acc;
        m_accLPBodyValid = true;
    }

    QVector3D sensorWorldPos(int seg) const {
        if (seg < 0 || seg >= kXsensSegmentCount) return QVector3D(0, 0, 0);
        return m_haveLastSensorPos ? m_lastSensorPos[seg] : QVector3D(0, 0, 0);
    }

private:
    std::string m_pose;

    std::array<int,   kXsensSegmentCountWithDummies> m_start{};
    std::array<int,   kXsensSegmentCountWithDummies> m_end{};
    std::array<float, kXsensSegmentCountWithDummies> m_len{};

    std::array<QVector3D, kXsensSegmentCountWithDummies> m_localOffset{};

    std::array<Quat,  kXsensSegmentCount> m_defAng{};

    mutable std::array<QVector3D, kXsensSegmentCount> m_lastSegCenter{};
    mutable bool                                      m_haveLastSegCenter = false;
    mutable std::array<QVector3D, kXsensSegmentCount> m_accLPBodyHint{};
    mutable bool                                      m_accLPBodyValid = false;

    mutable std::array<QVector3D, kXsensSegmentCount> m_lastSensorPos{};
    mutable bool                                      m_haveLastSensorPos = false;

    void buildTopology();
    void buildDefaultAngles();
    void buildLengths(const ActorConfig& actor);
};

std::array<Quat, kXsensSegmentCount>
defaultSegAnglesFor(const std::string& pose);

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

struct XsVectorBlob {
    double* data  = nullptr;
    size_t  size  = 0;
    size_t  flags = 0;
};

enum class ConnStatus {
    NotInitialized,
    NoDriver,
    Scanning,
    NoDevice,
    Connecting,
    Streaming,
    Stale,
    Failed,
};

const char* connStatusName(ConnStatus s);

class MocapReceiver : public QThread {
    Q_OBJECT
public:
    explicit MocapReceiver(bool testMode, QObject* parent=nullptr);
    ~MocapReceiver() override;

    void stop();

    void restart();

    bool connectGloves();
    bool glovesReady()     const;
    bool glovesCoreReady() const;
    bool glovesDllLoaded() const;

    void resetFusion();

    enum class Transport { ComPort, Network };
    void setTransport(Transport t);

    void setExpectedRate(double hz);
    double expectedRate() const;

    void setS2sAlignment(const std::array<Quat, kXsensSegmentCount>& s2s);
    void resetS2sAlignment();
    Quat getS2s(int idx) const;

    void setMagNormalisation(const std::array<double, kXsensSegmentCount>& magMagn);

    void setAccNormalisation(const std::array<double, kXsensSegmentCount>& accMagn);
    void setGyroBias(const std::array<QVector3D, kXsensSegmentCount>& gyrBias);

    void setMagSoftIron(const std::array<std::array<double, 9>, kXsensSegmentCount>& mat,
                        const std::array<QVector3D, kXsensSegmentCount>& offset);

    void setSegmentGain(const std::array<float, kXsensSegmentCount>& gain);

    void setMagneticDeclinationDeg(double deg);
    void setMagneticInclinationDeg(double deg);
    double magneticDeclinationDeg() const;
    double magneticInclinationDeg() const;

    bool saveCalibration(const QString& path) const;
    bool loadCalibration(const QString& path);

    QVector3D snapshotGyroAvg(int idx, int samples) const;
    QVector3D liveGyrSensor(int idx) const;

    SuitPose snapshot() const;

    ConnStatus  status()       const;
    QString     statusDetail() const;
    int         activeSensors() const;
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

class Lang : public QObject {
    Q_OBJECT
public:
    static Lang& instance();
    enum Code { RU, EN };

    Code        current() const { return m_code; }
    void        setLanguage(Code c);
    static QString t(const char* key);

signals:
    void changed();

private:
    Lang()  = default;
    Code m_code = Code::RU;
};

class NewSessionWizard : public QDialog {
    Q_OBJECT
public:
    struct Result {
        bool     useGloves = false;
        SuitType suit      = SuitType::Awinda;
        fox::body::Gender gender = fox::body::GenderMale;
        double heightCm = 175.0;
        double footLengthCm = 26.0;
        double armSpanCm = 0.0;
        double legLengthCm = 0.0;

        double upperArmCm = 0.0;
        double forearmCm  = 0.0;
        double handCm     = 0.0;
        double thighCm    = 0.0;
        double shankCm    = 0.0;

        double hipWidthCm      = 0.0;
        double shoulderWidthCm = 0.0;
        double trunkLengthCm   = 0.0;
        std::string poseKind = "tpose";
        std::array<Quat, kXsensSegmentCount> calibReference{};
        std::array<Quat, kXsensSegmentCount> tposeReference{};
        QVector3D                            tposePelvisPos{};
        bool                                 tposeCaptured = false;

        std::array<float, 20> fingerBaselineR{};
        std::array<float, 20> fingerBaselineL{};
        bool                  fingerBaselineCaptured = false;
    };

    NewSessionWizard(MocapReceiver* rx, bool testMode, QWidget* parent=nullptr);
    Result result() const { return m_result; }

    void preselectGloves(bool on);

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

    class QLabel*    m_welcomeImg = nullptr;
    class QLabel*    m_welcomeHeading = nullptr;
    class QLabel*    m_welcomeSub = nullptr;
    class QComboBox* m_langCombo  = nullptr;
    class QPushButton* m_btnStart = nullptr;

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
    class QComboBox*     m_cbxSuit      = nullptr;
    class QComboBox*     m_cbxTransport = nullptr;
    class QLabel*        m_wifiHint   = nullptr;

    class QLabel*          m_dimsTitle = nullptr;
    class QLabel*          m_lblHeight = nullptr;
    class QLabel*          m_lblFoot   = nullptr;

    class QLabel*          m_lblHip    = nullptr;
    class QLabel*          m_lblShoulder = nullptr;
    class QLabel*          m_lblTrunk  = nullptr;
    class QDoubleSpinBox*  m_height = nullptr;
    class QDoubleSpinBox*  m_foot   = nullptr;
    class QDoubleSpinBox*  m_hip      = nullptr;
    class QDoubleSpinBox*  m_shoulder = nullptr;
    class QDoubleSpinBox*  m_trunk    = nullptr;
    class QComboBox*       m_gender   = nullptr;
    class QLabel*          m_lblGender = nullptr;
    class QDoubleSpinBox*  m_upperArm = nullptr;
    class QDoubleSpinBox*  m_forearm  = nullptr;
    class QDoubleSpinBox*  m_hand     = nullptr;
    class QDoubleSpinBox*  m_thigh    = nullptr;
    class QDoubleSpinBox*  m_shank    = nullptr;
    class QLabel*          m_lblUpperArm = nullptr;
    class QLabel*          m_lblForearm  = nullptr;
    class QLabel*          m_lblHand     = nullptr;
    class QLabel*          m_lblThigh    = nullptr;
    class QLabel*          m_lblShank    = nullptr;
    class QLabel*          m_dimsHint  = nullptr;

    class QLabel*        m_calibTitle = nullptr;
    class QLabel*        m_poseImage  = nullptr;
    class QLabel*        m_poseHint   = nullptr;
    class QLabel*        m_connBadge  = nullptr;
    class QLabel*        m_countLabel = nullptr;
    class QLabel*        m_stillLabel = nullptr;
    class QProgressBar*  m_countdownBar = nullptr;
    class QProgressBar*  m_readyBar     = nullptr;
    class QPushButton*   m_btnCalibBegin = nullptr;
    class QLabel*        m_calibStatus   = nullptr;

    QTimer m_countTimer;
    QTimer m_captureTimer;
    QTimer m_statusTimer;
    int    m_countTicksLeft = 0;
    int    m_goodSamples    = 0;
    qint64 m_lastSampleCtr  = -1;
    std::vector<std::array<Quat, kXsensSegmentCount>> m_samples;
    std::array<Quat, kXsensSegmentCount> m_prevSnap{};
    bool   m_havePrev = false;
    bool   m_calibComplete = false;
    int    m_settleGen = 0;
    bool   m_suitBtnCooldown = false;

    qint64 m_phaseStartMs = 0;

    enum class CalibPhase { Idle, PrepT, CaptureT, SettleT, PrepN, CaptureN, Settle,
                            Done };
    CalibPhase m_phase = CalibPhase::Idle;

    std::array<QVector3D, kXsensSegmentCount> m_accAccumT{};
    std::array<QVector3D, kXsensSegmentCount> m_gyrAccumT{};
    std::array<QVector3D, kXsensSegmentCount> m_magAccumT{};
    std::array<double,    kXsensSegmentCount> m_accMagAccumT{};
    std::array<double,    kXsensSegmentCount> m_magMagAccumT{};
    std::array<int,       kXsensSegmentCount> m_accumCountT{};

    std::array<double, 20> m_fingerAccumR{};
    std::array<double, 20> m_fingerAccumL{};
    int                    m_fingerAccumCount = 0;

    std::array<QVector3D, kXsensSegmentCount> m_accAccumN{};
    std::array<QVector3D, kXsensSegmentCount> m_gyrAccumN{};
    std::array<QVector3D, kXsensSegmentCount> m_magAccumN{};
    std::array<double,    kXsensSegmentCount> m_accMagAccumN{};
    std::array<double,    kXsensSegmentCount> m_magMagAccumN{};
    std::array<int,       kXsensSegmentCount> m_accumCountN{};

    std::array<QVector3D, kXsensSegmentCount> m_gyrSqAccumT{};
    std::array<QVector3D, kXsensSegmentCount> m_gyrSqAccumN{};
    std::array<std::array<double, 6>, kXsensSegmentCount> m_magOuterAccumT{};
    std::array<std::array<double, 6>, kXsensSegmentCount> m_magOuterAccumN{};

    std::array<QVector3D, kXsensSegmentCount>* m_gyrSqAccum    = nullptr;
    std::array<std::array<double, 6>, kXsensSegmentCount>* m_magOuterAccum = nullptr;

    std::array<QVector3D, kXsensSegmentCount>* m_accAccum    = nullptr;
    std::array<QVector3D, kXsensSegmentCount>* m_gyrAccum    = nullptr;
    std::array<QVector3D, kXsensSegmentCount>* m_magAccum    = nullptr;
    std::array<double,    kXsensSegmentCount>* m_accMagAccum = nullptr;
    std::array<double,    kXsensSegmentCount>* m_magMagAccum = nullptr;
    std::array<int,       kXsensSegmentCount>* m_accumCount  = nullptr;

    class QLabel* m_calibQuality = nullptr;

    class QLabel*        m_readyTitle = nullptr;
    class QLabel*        m_readySummary = nullptr;
    class QPushButton*   m_btnFinish = nullptr;

    class QPushButton* m_btnBack = nullptr;
    class QPushButton* m_btnNext = nullptr;

    void buildPages();
    void retranslate();
    void updateNavButtons();
    void refreshPoseImage();
    void setBadge(QLabel* lab, const QString& txt, bool green);

    void logCalibPhaseTransition(const char* tag);

    bool calibBusy() const {
        return m_phase != CalibPhase::Idle && m_phase != CalibPhase::Done;
    }

    bool isNPosePhase() const noexcept {
        return m_phase == CalibPhase::PrepN
            || m_phase == CalibPhase::CaptureN;
    }

    void abortCalibration();

protected:
    void closeEvent(QCloseEvent*) override;
};

class SensorIndicatorsPanel : public QWidget {
    Q_OBJECT
public:
    SensorIndicatorsPanel(bool useGloves, QWidget* parent=nullptr);

    void setMode(bool useGloves);
    void setSuitLive(bool live, const QString& detail);
    void setFps(double hz);
    void setSessionRunning(bool running);

    void updateFromPose(const SuitPose& f);

    void setFreezeState(bool frozen);

    void clearLiveDots();

    void setActorDefaults(const ActorConfig& a);

signals:
    void resetClicked();
    void freezeToggled(bool frozen);
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

    struct TrackerCell { QLabel* dot = nullptr; QLabel* name = nullptr; };
    std::array<TrackerCell, kXsensSegmentCount> m_trackers{};

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

class MocapViewport : public QOpenGLWidget {
    Q_OBJECT
public:
    MocapViewport(const ActorConfig& actor, const std::string& pose,
                  QWidget* parent=nullptr);

    void updatePose(const std::array<Quat, kXsensSegmentCount>& orientations,
                    const QVector3D& rootPos);

    void updateHands(bool haveGloves,
                     const std::array<QVector3D, kFingerSegmentsHand>& right,
                     const std::array<QVector3D, kFingerSegmentsHand>& left);

    void setPose(const std::string& pose);

    void setActor(const ActorConfig& actor);
    ActorConfig actor() const { return m_actor; }
    void setLocoVerbose(bool v) { m_loco.setVerbose(v); }
    void setCondVerbose(bool v) { m_condVerbose = v; }

    void setProcRate(double hz) {
        m_loco.setProcRate(hz);
        // kRenderFps здесь = потолок частоты ОТРИСОВКИ, НЕ частота сенсора: рисуем не быстрее
        //   рендер-цели даже на Link 240 Гц (солвер при этом обрабатывает все кадры). Не путать с багом Awinda.
        const double cap = (hz > kRenderFps) ? kRenderFps : (hz > 1.0 ? hz : kRenderFps);
        m_paintMinIntervalSec = 1.0 / cap;
        m_nomDt = 1.0 / ((hz > 1.0) ? hz : 60.0);
    }

    void resetSceneOrigin();

    void setFreezeXY(bool frozen);
    bool isFrozen() const { return m_freezeXY; }

    void setWristCfg(bool right, const WristAnatomicalCfg& cfg) {
        if (right) m_wristCfgR = cfg; else m_wristCfgL = cfg;
    }
    WristAnatomicalCfg wristCfg(bool right) const {
        return right ? m_wristCfgR : m_wristCfgL;
    }

    void setTposeHandAnchor(const Quat& forearmR_T, const Quat& handR_T,
                            const Quat& forearmL_T, const Quat& handL_T);

    void setTposeFootAnchor(const Quat& pelvis_T,
                            const Quat& rFoot_T, const Quat& lFoot_T);

    void setCrossLeggedHints(bool r, bool l, double cR, double cL) {
        m_crossLeggedR = r; m_crossLeggedL = l;
        m_crossLeggedConfR = cR; m_crossLeggedConfL = cL;
    }

    QSize sizeHint() const override { return QSize(800, 600); }

    const std::array<Quat, kXsensSegmentCount>& filteredOrient() const { return m_orient; }

    bool   segLocked(int i)      const { return m_locked[i]; }
    bool   segAnchorValid(int i) const { return m_anchorValid[i]; }
    double segAngVelLP(int i)    const { return m_angVelLP[i]; }
    double segAngAcc(int i)      const { return m_dbgAngAcc[i]; }
    double segStillTicks(int i)  const { return m_stillTicks[i]; }
    Quat   segDriftLocal(int i)  const { return m_driftLocal[i]; }

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
    // §35 пин корня по XY в МИРОВОЙ системе при включённой заморозке — общий для вьюпорта/стрима/записи (formules.txt)
    QVector3D lockPelvisIfFrozen(const QVector3D& worldPelvis) const {
        if (!m_freezeXY) return worldPelvis;
        return QVector3D(m_freezeAnchor.x(), m_freezeAnchor.y(), worldPelvis.z());
    }

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

    LocomotionSolver m_loco;

    QVector3D m_sceneShift{0, 0, 0};
    float     m_sceneYaw   = 0.0f;
    bool      m_freezeXY   = false;
    QVector3D m_freezeAnchor{0, 0, 0};
    QVector3D m_lastRenderedPelvis{0, 0, 0};
    QVector3D m_lastWorldPelvis{0, 0, 0};   // §35 таз в мире (до камеры/заморозки) — якорь заморозки
    QVector3D m_lastLocoOffset{0, 0, 0};

    std::array<QVector3D, kXsensKeypointCount> m_lastRenderedKp{};
    float     m_lastFloorClamp = 0.0f;

    std::array<Quat, kXsensSegmentCount>   m_lockQuat{};
    std::array<bool, kXsensSegmentCount>   m_locked{};
    std::array<double, kXsensSegmentCount> m_angVelPrev{};
    std::array<double, kXsensSegmentCount> m_angVelLP{};
    std::array<double, kXsensSegmentCount> m_dbgAngAcc{};
    std::array<double, kXsensSegmentCount> m_stillTicks{};
    std::array<Quat, kXsensSegmentCount>   m_prevQ{};
    std::array<Quat, kXsensSegmentCount>   m_outPrevQ{};

    std::array<Quat, kXsensSegmentCount>   m_condPrev{};
    bool                                    m_haveCond    = false;
    double                                  m_nomDt       = 1.0 / 60.0;   // дефолт частоты обработки (Awinda 60 Гц); перезаписывается setProcRate
    bool                                    m_condVerbose = false;
    std::array<double, kXsensSegmentCount> m_unlockBlend{};

    std::array<double, kXsensSegmentCount> m_lockBlend{};
    std::array<Quat, kXsensSegmentCount>   m_anchorLocal{};
    std::array<bool, kXsensSegmentCount>   m_anchorValid{};
    std::array<Quat, kXsensSegmentCount>   m_driftLocal{};

    bool                                    m_tposeHandAnchorValid = false;

    Quat                                    m_tposeFootRefPelR{1, 0, 0, 0};
    Quat                                    m_tposeFootRefPelL{1, 0, 0, 0};
    bool                                    m_tposeFootAnchorValid = false;

    bool                                    m_crossLeggedR = false;
    bool                                    m_crossLeggedL = false;
    double                                  m_crossLeggedConfR = 0.0;
    double                                  m_crossLeggedConfL = 0.0;

    std::array<double, kXsensSegmentCount> m_calmSeconds{};
    bool                                    m_havePrevQ = false;
    double                                  m_lastRenderT = 0.0;

    double                                  m_lastPaintSec = 0.0;
    double                                  m_paintMinIntervalSec = 1.0 / kRenderFps;   // потолок ОТРИСОВКИ (рендер), не частота сенсора

    WristAnatomicalCfg m_wristCfgR{};
    WristAnatomicalCfg m_wristCfgL{};

    float m_yaw   = 180.0f;
    float m_pitch = 12.0f;
    float m_dist  = 3.2f;
    QPoint m_lastMouse;

    void drawFloor();
    void drawReferenceFrame();
    void drawSkeleton();
};

struct RecordedFrame {
    double  t         = 0.0;
    std::array<Quat,      kXsensSegmentCount> segQuat{};
    QVector3D                                  pelvisPos{0, 0, 0};

    bool    hasGloves = false;
    std::array<Quat, kFingerSegmentsHand> rightGloveQ{};
    std::array<Quat, kFingerSegmentsHand> leftGloveQ{};
};

enum class RecordFormat   { BVH, FBX };
enum class RecordQuality  { Normal, HdPostProcessing };

struct RecordSettings {
    RecordFormat  format  = RecordFormat::BVH;
    RecordQuality quality = RecordQuality::Normal;
    int           fps     = 30;
};

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

class RecordWizard : public QDialog {
    Q_OBJECT
public:
    explicit RecordWizard(SuitType suit = SuitType::Awinda, QWidget* parent = nullptr);
    RecordSettings result() const { return m_result; }

private slots:
    void goNext();
    void goBack();

private:
    SuitType m_suit = SuitType::Awinda;
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

enum class LiveTarget { BlenderMVN, UnrealLiveLink };

struct LiveSettings {
    LiveTarget    target    = LiveTarget::BlenderMVN;
    QString       host      = "127.0.0.1";
    int           port      = 9763;
    bool          useGloves = false;
    int           fps       = 60;

    bool          splitGloveDatagrams = false;

    bool          debugDumpFirstFrame = false;

    bool          verboseLog = false;

    std::array<QVector3D, kXsensSegmentCount> tposeOriginM{};

    std::array<Quat, kXsensSegmentCount> defAngT{};
};

class LiveStreamWizard : public QDialog {
    Q_OBJECT
public:
    explicit LiveStreamWizard(SuitType suit = SuitType::Awinda, QWidget* parent = nullptr);
    LiveSettings result() const { return m_result; }

private:
    SuitType             m_suit   = SuitType::Awinda;
    class QComboBox*     m_target = nullptr;
    class QComboBox*     m_host   = nullptr;
    class QSpinBox*      m_port   = nullptr;
    class QComboBox*     m_fps    = nullptr;
    class QPushButton*   m_btnStart = nullptr;
    class QPushButton*   m_btnCancel = nullptr;
    LiveSettings         m_result{};
};

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

struct JointOffsets {
    std::array<QVector3D, kXsensSegmentCount> deg{};

    bool isZero() const {
        for (const auto& v : deg)
            if (v.x() != 0.0f || v.y() != 0.0f || v.z() != 0.0f) return false;
        return true;
    }
    void clear() { for (auto& v : deg) v = QVector3D(0, 0, 0); }

    static QString filePath();
    bool load(const QString& path);
    bool save(const QString& path) const;
};

class JointOffsetsDialog : public QDialog {
    Q_OBJECT
public:
    explicit JointOffsetsDialog(JointOffsets* offsets, QWidget* parent = nullptr);

private:
    void buildUi();
    void syncControlsFromModel();

    struct AxisCtl { class QSlider* slider = nullptr; class QDoubleSpinBox* spin = nullptr; };

    JointOffsets* m_offsets = nullptr;
    std::array<std::array<AxisCtl, 3>, kXsensSegmentCount> m_ctl{};
    bool         m_syncing = false;
    class QLabel* m_status = nullptr;
};

class MainWindow : public QMainWindow {
    Q_OBJECT
public:

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
    void onOpenJointSettings();

public:
    void setGlovesMode(bool on) { m_gloves = on; }

private:
    NewSessionWizard::Result m_setup;
    bool            m_test = false;
    bool            m_gloves = false;

    MocapReceiver*  m_rx       = nullptr;
    MocapViewport*  m_viewport = nullptr;
    QTimer          m_renderTimer;
    SensorIndicatorsPanel* m_panel = nullptr;

    bool            m_sessionRunning = true;

    double          m_procRateHz   = 60.0;   // дефолт = частота костюма по умолчанию (Awinda 60 Гц); перезаписывается реальной частотой железа

    std::unique_ptr<SkeletonXsens> m_skel;

    bool                       m_recording   = false;
    bool                       m_finishing   = false;
    bool                       m_takePending = false;
    RecordSettings             m_recCfg{};
    std::vector<RecordedFrame> m_recBuffer;
    bool                       m_recOverflowWarned = false;
    bool                       m_recHardCapped     = false;
    qint64                     m_recStartMs = 0;
    qint64                     m_streamStartMs = 0;

    QLabel*                    m_modeHud = nullptr;
    QTimer*                    m_modeHudTimer = nullptr;
    qint64                     m_recLastSample = -1;
    RecordHud*                 m_hud = nullptr;

    LiveStreamSender*          m_streamer = nullptr;

    JointOffsets               m_jointOffsets;
    JointOffsetsDialog*        m_jointDlg = nullptr;

    void logTest(const std::string& msg) const;
    void startRecording(const RecordSettings& cfg);
    void finishRecording();
    void layoutHud();

protected:
    void closeEvent(QCloseEvent*) override;
    void resizeEvent(QResizeEvent*) override;
};

extern const char* kStyleSheet;

struct CliArgs {
    bool test   = false;
    bool gloves = false;
    bool wristConstraint = false;
    SuitType suit = SuitType::Awinda;
    QString language;
};
CliArgs parseCli(int argc, char** argv);

// --- Thread-safe diagnostic logging (test mode only) -----------------------
// Test output is produced from several threads at once (network worker, Manus
// SDK callback, GUI/render, main). Every write funnels through one mutex so
// lines cannot interleave/corrupt, and each is stamped with a program-relative
// timestamp so the log can be correlated with what the operator was doing
// (T-pose, N-pose, the moment a limb moved, etc.).
double logUptimeSec();                     // seconds since the first log call
void   logLine(const std::string& msg);    // one timestamped line, emitted atomically
void   logBlock(const std::string& block);  // multi-line block, emitted atomically

void testLog(const std::string& msg, bool enabled);

}
