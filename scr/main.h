// Fox Mocap — MVN-compatible motion-capture app for Xsens Link suits
// and optional Manus gloves.  Implements a 23-segment Xsens skeleton,
// T / N / K-pose reference-angle calibration, forward kinematics with
// dummy-segment helpers, and static sensor-to-segment alignment.
// Network layer accepts MVN MXTP02 / MXTP25 datagrams on UDP :9763.

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

// ============================================================================
// ============================================================================

namespace fox {

constexpr int     kXsensSegmentCount  = 23;
constexpr int     kXsensKeypointCount = 28;    // 23 joints + 5 end-points
constexpr int     kXsensSegmentCountWithDummies = 27;
constexpr int     kFingerSegmentsHand = 20;
constexpr double  kRenderFps          = 90.0;
constexpr double  kStaleSeconds       = 2.0;
constexpr int     kCalibrationSamples = 500;    // ~5 s @ 100 Hz
constexpr int     kCountdownSeconds   = 6;    // 6 s prep / Madgwick warm-up

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
// ============================================================================

struct Quat {
    double w, x, y, z;

    constexpr Quat() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    constexpr Quat(double w_, double x_, double y_, double z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    double norm() const { return std::sqrt(w*w + x*x + y*y + z*z); }

    Quat normalized() const {
        const double n = norm();
        if (n < 1e-12) return Quat(1, 0, 0, 0);
        return Quat(w/n, x/n, y/n, z/n);
    }

    Quat conj() const { return Quat(w, -x, -y, -z); }
    Quat inv()  const { Quat q = conj(); double n2 = w*w+x*x+y*y+z*z;
                        if (n2 < 1e-12) return Quat();
                        return Quat(q.w/n2, q.x/n2, q.y/n2, q.z/n2); }
};

Quat quat_mult(const Quat& a, const Quat& b);
QVector3D vec_rotate(const QVector3D& v, const Quat& q);
Quat euler_to_quat(double a, double b, double c, const char* seq);

void swingTwistDecompose(const Quat& q, const QVector3D& axisU,
                         Quat& outSwing, Quat& outTwist);

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

extern const FingerJointLimit kFingerLimits[5][3];

// ============================================================================
// ============================================================================

struct SuitPose {
    quint64 sampleCounter = 0;
    double  recvTime      = 0.0;                   // monotonic seconds

    std::array<Quat, kXsensSegmentCount> quat;     // W,X,Y,Z, world-frame
    std::array<bool, kXsensSegmentCount> segValid{};   // per-segment: have we seen it?

    std::array<QVector3D, kXsensSegmentCount> accSensor{};   // g
    std::array<QVector3D, kXsensSegmentCount> gyrSensor{};   // deg/s
    std::array<QVector3D, kXsensSegmentCount> magSensor{};   // calibrated units

    QVector3D pelvisPos{0, 0, 0};                   // root position in meters

    std::array<float, kXsensSegmentCount> orientStdDeg{};

    std::array<double, kXsensSegmentCount> segLastT{};
    int batteryLevel = -1;

    bool                                      hasGloves = false;
    std::array<Quat,      kFingerSegmentsHand> rightGloveQ{};
    std::array<Quat,      kFingerSegmentsHand> leftGloveQ{};
    std::array<QVector3D, kFingerSegmentsHand> rightGloveP{};   // positions, meters
    std::array<QVector3D, kFingerSegmentsHand> leftGloveP{};

    SuitPose() { for (auto& q : quat) q = Quat(1, 0, 0, 0); }
};

constexpr int kFingerChainCount    = 5;
constexpr int kFingerChainLen      = 4;
extern const int kFingerChains[kFingerChainCount][kFingerChainLen];
extern const char* kFingerChainNames[kFingerChainCount];

// ============================================================================
// ============================================================================

struct ActorConfig {
    double heightCm      = 175.0;
    double footLengthCm  = 26.0;
    double armSpanCm     = 0.0;
    double legLengthCm   = 0.0;
    double shoulderWidthCm = 0.0;
    double hipWidthCm      = 0.0;
    double handLengthCm    = 0.0;
    bool   useGloves       = false;
};

// ============================================================================
// ============================================================================

struct ContactState {
    bool   rightDown  = false;
    bool   leftDown   = false;
    double rightAngV  = 0.0;        // rad/s (smoothed) — for debug/UI
    double leftAngV   = 0.0;
    bool   rHeelDown  = false;
    bool   rToeDown   = false;
    bool   lHeelDown  = false;
    bool   lToeDown   = false;
};

class LocomotionSolver {
public:
    void reset();

    QVector3D update(const Quat& rightFootQuat,
                     const Quat& leftFootQuat,
                     const Quat& pelvisQuat,
                     const QVector3D& fkRightFoot,
                     const QVector3D& fkLeftFoot,
                     double tSeconds);

    QVector3D update(const Quat& rightFootQuat,
                     const Quat& leftFootQuat,
                     const Quat& pelvisQuat,
                     const QVector3D& fkRightHeel,
                     const QVector3D& fkRightToe,
                     const QVector3D& fkLeftHeel,
                     const QVector3D& fkLeftToe,
                     double tSeconds);

    void updateHeelToeContacts(const QVector3D& fkRHeel,
                               const QVector3D& fkRToe,
                               const QVector3D& fkLHeel,
                               const QVector3D& fkLToe);

    ContactState contact() const { return m_contact; }
    enum Side { RIGHT = 0, LEFT = 1, BOTH = 2 };
    Side currentSupport() const  { return m_support; }
    QVector3D anchor()    const  { return m_anchor; }
    void setVerbose(bool v) { m_verbose = v; }
    bool isAirborne() const { return m_airborne; }
    int  airTicks()   const { return m_airTicks; }

private:
    bool m_verbose = false;
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

    bool      m_initialised = false;
    Side      m_support     = RIGHT;
    QVector3D m_anchor      {0, 0, 0};

    double m_stillRad     = 0.30;
    double m_heightMargin = 0.03;
    int    m_latchTicks   = 3;
    double m_switchMargin = 0.04;
    double m_heightMarginSlow = 0.08;

    QVector3D m_offsetLast   {0, 0, 0};
    QVector3D m_offsetPrev   {0, 0, 0};
    bool      m_offsetReady  = false;

    // =====================================================================
        // =====================================================================
        Quat      m_prevPelvisQ      {1, 0, 0, 0};
        double    m_pelvisAngV        = 0.0;
        double    m_pelvisYawAngV     = 0.0;
        bool      m_yawFrozenPrev     = false;
        bool      m_yawFrozenPrevR    = false;
        bool      m_yawFrozenPrevL    = false;
        int       m_pelvisStillTicks  = 0;

        static constexpr int kFKXYWindow = 10;
        std::array<QVector2D, kFKXYWindow> m_rFKXY {};
        std::array<QVector2D, kFKXYWindow> m_lFKXY {};
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
                        PoseSquat = 3, PoseLying = 4 };
        PoseKind  m_pose              = PoseUnknown;
        int       m_poseTicks         = 0;

        int       m_zuptTicks         = 0;

        int       m_airTicks          = 0;
        bool      m_airborne          = false;
        int       m_airTicksThresh    = 4;

        double    m_fkxyStableRange   = 0.025;
        double    m_pelvisStillRad    = 0.20;   // было 0.12 — реже триггерим
        int       m_pelvisStillTicksThresh = 30;
        double    m_confCommit        = 0.55;
        double    m_confRelease       = 0.30;
        double    m_confRiseRate      = 0.50;   // было 0.30 — быстрый rise
        double    m_confFallRate      = 0.25;   // было 0.10 — быстрый fall (не заклинивать)
        double    m_offsetRatePrimary = 0.40;   // было 0.18 — primary anchor catches up быстро
        double    m_offsetRateDouble  = 0.25;   // было 0.10 — double-support тоже бодрее
        double    m_zRatePelvisMoving = 0.40;   // pose transitions
        double    m_zRatePelvisStill  = 0.015;
        double    m_zDriveRate        = 0.02;
        int       m_poseStableTicks   = 45;     // 0.5 s @ 90 Hz
        int       m_zuptTicksThresh   = 30;
        double    m_zStillDeadbandM   = 0.01;
        double    m_lieTiltCosThresh  = 0.50;   // cos(60°) — pelvis tilt
        double    m_squatKneeThresh   = 0.30;   // m — |pelvis-to-foot Z|
        double    m_sitKneeThresh     = 0.55;   // m — pelvis-to-foot Z for sit
        double    m_actorHeightM      = 1.75;

      public:
        void setActorHeight(double h) { m_actorHeightM = std::max(0.5, h); }

      private:
        PoseKind _classifyPose(const Quat& qPelvis,
                               const QVector3D& fkR,
                               const QVector3D& fkL,
                               PoseKind prevPose,
                               double& outTiltCos) const;
        double m_poseHysteresisM = 0.05;

        double m_heelToeThreshM       = 0.02;   // height "down" gate (m)
        double m_heelToeReleaseHystM  = 0.015;  // release adds +1.5 cm
};

class SkeletonXsens {
public:
    SkeletonXsens(const ActorConfig& actor, const std::string& pose);

    std::array<QVector3D, kXsensKeypointCount>
    computeKeypoints(const std::array<Quat, kXsensSegmentCount>& segOrients,
                     const QVector3D& rootPos) const;

    std::array<Quat, kXsensSegmentCountWithDummies>
    addDummySegments(const std::array<Quat, kXsensSegmentCount>& segs) const;

    const std::array<int, kXsensSegmentCountWithDummies>& startPts() const { return m_start; }
    const std::array<int, kXsensSegmentCountWithDummies>& endPts()   const { return m_end; }
    const std::array<float, kXsensSegmentCountWithDummies>& lengths() const { return m_len; }
    const std::array<Quat,  kXsensSegmentCount>& defaultSegAngles() const { return m_defAng; }

    const std::string& poseKind() const { return m_pose; }

    Quat defAngFor(int seg) const {
        return (seg >= 0 && seg < kXsensSegmentCount)
                   ? m_defAng[seg] : Quat(1, 0, 0, 0);
    }

private:
    std::string m_pose;
    std::array<int,   kXsensSegmentCountWithDummies> m_start{};
    std::array<int,   kXsensSegmentCountWithDummies> m_end{};
    std::array<float, kXsensSegmentCountWithDummies> m_len{};
    std::array<Quat,  kXsensSegmentCount> m_defAng{};

    void buildTopology();
    void buildDefaultAngles();
    void buildLengths(const ActorConfig& actor);
};

std::array<Quat, kXsensSegmentCount>
defaultSegAnglesFor(const std::string& pose);

// ============================================================================
// ============================================================================

// ============================================================================
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

struct XsVectorBlob {
    double* data  = nullptr;
    size_t  size  = 0;
    size_t  flags = 0;
};

// ============================================================================
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
// ============================================================================

class MocapReceiver : public QThread {
    Q_OBJECT
public:
    explicit MocapReceiver(bool testMode, QObject* parent=nullptr);
    ~MocapReceiver() override;

    void stop();

    void restart();

    bool connectGloves();
    bool glovesReady()     const;       // physical glove(s) visible to Core
    bool glovesCoreReady() const;       // ManusCore responded to handshake
    bool glovesDllLoaded() const;       // ManusSDK.dll was found and loaded

    void resetFusion();

    enum class Transport { ComPort, Network };
    void setTransport(Transport t);
    void setWifiCredentials(const QString& ssid, const QString& password);

    void setS2sAlignment(const std::array<Quat, kXsensSegmentCount>& s2s);
    void resetS2sAlignment();
    Quat getS2s(int idx) const;

    void setMagNormalisation(const std::array<double, kXsensSegmentCount>& magMagn);

    void setAccNormalisation(const std::array<double, kXsensSegmentCount>& accMagn);

    void setMagSoftIron(const std::array<std::array<double, 9>, kXsensSegmentCount>& mat,
                        const std::array<QVector3D, kXsensSegmentCount>& offset);

    void setKfPriors(const std::array<Quat, kXsensSegmentCount>& refQuat,
                     const std::array<QVector3D, kXsensSegmentCount>& gyrBiasDegSec);

    // Variant that accepts per-segment orientation uncertainty (degrees).
    // Used after calibration when different segments have different
    // confidence (e.g. spinal sensors with ecompass residual 0° vs. leg
    // sensors fall-back to TRIAD with residual 5-15°).  A tight 2° prior
    // on a low-confidence segment forces the filter to trust a possibly
    // wrong calibration and stalls acc/mag corrections; a looser prior
    // (~10-15°) lets the filter adapt over the first few seconds while
    // still preserving the priors on the well-conditioned spine.
    void setKfPriors(const std::array<Quat, kXsensSegmentCount>& refQuat,
                     const std::array<QVector3D, kXsensSegmentCount>& gyrBiasDegSec,
                     const std::array<float, kXsensSegmentCount>& orientStdDeg);

    SuitPose snapshot() const;

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
// ============================================================================

class NewSessionWizard : public QDialog {
    Q_OBJECT
public:
    struct Result {
        bool   useGloves = false;
        double heightCm = 175.0;
        double footLengthCm = 26.0;
        double armSpanCm = 0.0;
        double legLengthCm = 0.0;
        double shoulderWidthCm = 0.0;
        double hipWidthCm      = 0.0;
        double handLengthCm    = 0.0;
        std::string poseKind = "tpose";
        std::array<Quat, kXsensSegmentCount> calibReference{};
        std::array<Quat, kXsensSegmentCount> tposeReference{};
        QVector3D                            tposePelvisPos{};
        bool                                 tposeCaptured = false;
    };

    NewSessionWizard(MocapReceiver* rx, bool testMode, QWidget* parent=nullptr);
    Result result() const { return m_result; }

    void preselectGloves(bool on);

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
    class QComboBox*     m_cbxTransport = nullptr;   // COM / WiFi
    class QLineEdit*     m_edSsid     = nullptr;
    class QLineEdit*     m_edPassword = nullptr;
    class QWidget*       m_wifiRow    = nullptr;

    class QLabel*          m_dimsTitle = nullptr;
    class QLabel*          m_lblHeight = nullptr;
    class QLabel*          m_lblFoot   = nullptr;
    class QLabel*          m_lblArm    = nullptr;   // FIX: размах рук перенесён сюда
    class QLabel*          m_lblLeg    = nullptr;   // FIX: длина ноги перенесена сюда
    class QLabel*          m_lblShoulder = nullptr; // bi-acromial breadth
    class QLabel*          m_lblHip      = nullptr; // bi-iliac breadth
    class QLabel*          m_lblHand     = nullptr; // wrist→fingertip
    class QDoubleSpinBox*  m_height = nullptr;
    class QDoubleSpinBox*  m_foot   = nullptr;
    class QDoubleSpinBox*  m_arm    = nullptr;      // FIX: arm span (опц., 0 = по росту)
    class QDoubleSpinBox*  m_leg    = nullptr;      // FIX: leg length (опц., 0 = по росту)
    class QDoubleSpinBox*  m_shoulder = nullptr;    // 0 = derive (0.259 × h)
    class QDoubleSpinBox*  m_hip      = nullptr;    // 0 = derive (0.191 × h)
    class QDoubleSpinBox*  m_hand     = nullptr;    // 0 = derive (0.108 × h × armScale)
    class QLabel*          m_dimsHint  = nullptr;

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
    bool   m_stillState = true;
    int    m_stillStreak = 0;
    int    m_moveStreak  = 0;
    bool   m_calibComplete = false;
    std::array<bool, kXsensSegmentCount> m_asymmetricMount{};

    enum class CalibPhase { Idle, PrepT, CaptureT, SettleT, PrepN, CaptureN, Settle, PrepK, CaptureK, Done };
    CalibPhase m_phase = CalibPhase::Idle;

    std::array<QVector3D, kXsensSegmentCount> m_accAccumT{};
    std::array<QVector3D, kXsensSegmentCount> m_gyrAccumT{};
    std::array<QVector3D, kXsensSegmentCount> m_magAccumT{};
    std::array<double,    kXsensSegmentCount> m_accMagAccumT{};
    std::array<int,       kXsensSegmentCount> m_accumCountT{};

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

protected:
    void closeEvent(QCloseEvent*) override;
};

// ============================================================================
// ============================================================================

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

    void setActorDefaults(const ActorConfig& a);

signals:
    void pauseClicked();              // legacy (kept compat)
    void resumeClicked();
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
    QPushButton*   m_btnPauseResume = nullptr;  // legacy, kept compat
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

// ============================================================================
// ============================================================================

class MocapViewport : public QOpenGLWidget {
    Q_OBJECT
public:
    MocapViewport(const ActorConfig& actor, const std::string& pose,
                  QWidget* parent=nullptr);

    void updatePose(const std::array<Quat, kXsensSegmentCount>& orientations,
                    const QVector3D& rootPos);
    void updatePose(const std::array<Quat, kXsensSegmentCount>& orientations,
                    const QVector3D& rootPos,
                    const std::array<float, kXsensSegmentCount>& orientStdDeg);

    void updateHands(bool haveGloves,
                     const std::array<QVector3D, kFingerSegmentsHand>& right,
                     const std::array<QVector3D, kFingerSegmentsHand>& left);

    void setPose(const std::string& pose);

    void setActor(const ActorConfig& actor);
    ActorConfig actor() const { return m_actor; }
    void setLocoVerbose(bool v) { m_loco.setVerbose(v); }

    void resetSceneOrigin();
    void setFreezeXY(bool frozen);
    bool isFrozen() const { return m_freezeXY; }

    void setWristCfg(bool right, const WristAnatomicalCfg& cfg) {
        if (right) m_wristCfgR = cfg; else m_wristCfgL = cfg;
    }
    WristAnatomicalCfg wristCfg(bool right) const {
        return right ? m_wristCfgR : m_wristCfgL;
    }

    QSize sizeHint() const override { return QSize(800, 600); }

    const std::array<Quat, kXsensSegmentCount>& filteredOrient() const { return m_orient; }

    QVector3D lastRenderedPelvis() const { return m_lastRenderedPelvis; }

    QVector3D tickLoco(const std::array<Quat, kXsensSegmentCount>& q,
                       const QVector3D& fkRFoot,
                       const QVector3D& fkLFoot,
                       double tSec);
    QVector3D tickLocoHT(const std::array<Quat, kXsensSegmentCount>& q,
                         const QVector3D& fkRHeel,
                         const QVector3D& fkRToe,
                         const QVector3D& fkLHeel,
                         const QVector3D& fkLToe,
                         double tSec);
    void tickHeelToe(const QVector3D& fkRHeel,
                     const QVector3D& fkRToe,
                     const QVector3D& fkLHeel,
                     const QVector3D& fkLToe) {
        m_loco.updateHeelToeContacts(fkRHeel, fkRToe, fkLHeel, fkLToe);
    }
    ContactState contactState() const { return m_loco.contact(); }
    QVector3D lastLocoOffset() const { return m_lastLocoOffset; }

    QVector3D sceneShift() const { return m_sceneShift; }
    float sceneYaw() const { return m_sceneYaw; }
    QVector3D freezeAnchor() const { return m_freezeAnchor; }

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

    QVector3D m_sceneShift{0, 0, 0};  // applied to every kp AFTER loco
    float     m_sceneYaw   = 0.0f;    // rad, rotation about Z (reset pins yaw)
    bool      m_freezeXY   = false;
    QVector3D m_freezeAnchor{0, 0, 0};  // pelvis world XY at freeze moment
    QVector3D m_lastRenderedPelvis{0, 0, 0};  // cached for resetSceneOrigin()
    QVector3D m_lastLocoOffset{0, 0, 0};

    std::array<Quat, kXsensSegmentCount>   m_lockQuat{};      // held output
    std::array<bool, kXsensSegmentCount>   m_locked{};
    std::array<double, kXsensSegmentCount> m_angVelPrev{};    // last |ω|
    std::array<double, kXsensSegmentCount> m_angVelLP{};      // smoothed |ω|
    std::array<double, kXsensSegmentCount> m_stillTicks{};    // consecutive
    std::array<Quat, kXsensSegmentCount>   m_prevQ{};
    std::array<Quat, kXsensSegmentCount>   m_outPrevQ{};
    std::array<double, kXsensSegmentCount> m_unlockBlend{};
    std::array<Quat, kXsensSegmentCount>   m_anchorLocal{};
    std::array<bool, kXsensSegmentCount>   m_anchorValid{};
    std::array<Quat, kXsensSegmentCount>   m_driftLocal{};
    bool                                    m_havePrevQ = false;
    double                                  m_lastRenderT = 0.0;

    WristAnatomicalCfg m_wristCfgR{};
    WristAnatomicalCfg m_wristCfgL{};

    float m_yaw   = 35.0f;
    float m_pitch = 12.0f;
    float m_dist  = 3.2f;
    QPoint m_lastMouse;

    void drawFloor();
    void drawReferenceFrame();
    void drawSkeleton();
};

// ============================================================================
// ============================================================================

struct RecordedFrame {
    double  t         = 0.0;           // seconds since record start
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
    int           fps     = 30;        // 24 / 30 / 60
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
    explicit RecordWizard(QWidget* parent = nullptr);
    RecordSettings result() const { return m_result; }

private slots:
    void goNext();
    void goBack();

private:
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

enum class LiveTarget { BlenderMVN, XsensLivc };

struct LiveSettings {
    LiveTarget    target    = LiveTarget::BlenderMVN;
    QString       host      = "127.0.0.1";
    int           port      = 9763;           // MVN default; overridden in UI
    bool          useGloves = false;
    int           fps       = 60;             // 24 / 30 / 60 — UI throttle
    std::array<QVector3D, kXsensSegmentCount> tposeOriginM{};
    std::array<Quat, kXsensSegmentCount> defAngT{};
};

class LiveStreamWizard : public QDialog {
    Q_OBJECT
public:
    explicit LiveStreamWizard(QWidget* parent = nullptr);
    LiveSettings result() const { return m_result; }

private:
    class QComboBox*     m_target = nullptr;
    class QComboBox*     m_host   = nullptr;
    class QSpinBox*      m_port   = nullptr;
    class QComboBox*     m_fps    = nullptr;        // new: 24/30/60
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
                   const std::array<QVector3D, kXsensSegmentCount>& segPos,
                   const QVector3D& pelvisPos);

    void pushFrameWithGloves(quint32 sample,
        const std::array<Quat, kXsensSegmentCount>& segQuat,
        const std::array<QVector3D, kXsensSegmentCount>& segPos,
        const QVector3D& pelvisPos,
        const std::array<Quat, kFingerSegmentsHand>& rightGloveQ,
        const std::array<QVector3D, kFingerSegmentsHand>& rightGloveP,
        const std::array<Quat, kFingerSegmentsHand>& leftGloveQ,
        const std::array<QVector3D, kFingerSegmentsHand>& leftGloveP,
        const QVector3D& wristPosR,
        const QVector3D& wristPosL);

    const LiveSettings& settings() const { return m_cfg; }

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
    LiveSettings m_cfg{};
    bool         m_running = false;
};

// ============================================================================
// ============================================================================

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
    void onBuildCoordinates();
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

    std::unique_ptr<SkeletonXsens> m_skel;

    bool                       m_recording = false;
    RecordSettings             m_recCfg{};
    std::vector<RecordedFrame> m_recBuffer;
    qint64                     m_recStartMs = 0;
    qint64                     m_streamStartMs = 0;
    QLabel*                    m_modeHud = nullptr;
    QTimer*                    m_modeHudTimer = nullptr;
    qint64                     m_recLastSample = -1;
    RecordHud*                 m_hud = nullptr;

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
// ============================================================================

extern const char* kStyleSheet;

// ============================================================================
// ============================================================================

struct CliArgs {
    bool test   = false;
    bool gloves = false;                     // --gloves : enable hand skeleton in -test
    bool wristConstraint = false;
};
CliArgs parseCli(int argc, char** argv);

void testLog(const std::string& msg, bool enabled);

extern bool g_testStreamLog;

}  // namespace fox
