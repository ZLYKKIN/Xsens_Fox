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
//  Constants
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
                                              // (5 s convergence at 240 Hz × β=0.35
                                              //  comfortably flattens all 17 filters)

// Xsens segment indices (matches MXTP segId - 1).
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
//  Quaternion math (WXYZ, scalar first) — scipy / Hamilton convention
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

// Hamilton product (scipy Rotation composition).
Quat quat_mult(const Quat& a, const Quat& b);
// Rotate vector v by quaternion q  (v' = q * [0,v] * q^-1)
QVector3D vec_rotate(const QVector3D& v, const Quat& q);
// Convert Euler angles to quaternion. seq is a 3-char upper-case code like
// "XYZ" or "YXZ"  (intrinsic rotations, matches scipy 'XYZ' uppercase).
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
    // imu-calibration set: acc_magn, gyr_bias, mag_magn and s2s_offset.
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
//  SkeletonXsens — 23 segments + 4 dummy stubs, forward kinematics
// ============================================================================

struct ActorConfig {
    double heightCm      = 175.0;
    double footLengthCm  = 26.0;
    double armSpanCm     = 0.0;
    double legLengthCm   = 0.0;
    bool   useGloves     = false;
};

// ============================================================================
//  Locomotion — foot-contact detector + foot-lock IK + drift limiter
//
//  Design follows the well-known structure of inertial-mocap biomech
//  solvers:
//    * a (rightDown, leftDown) contact pair drives foot-lock IK rather
//      than raw acceleration double-integration;
//    * drift correction is a loose, Kalman-style aiding blend of
//      external observations (foot anchor: Z re-pinned to 0 on re-anchor,
//      XY frozen between strides);
//    * finger tracking uses a 20-segment-per-hand local-quaternion array
//      fused into world space as world_q = wrist_q * finger_q_local.
// ============================================================================

struct ContactState {
    bool   rightDown  = false;
    bool   leftDown   = false;
    double rightAngV  = 0.0;        // rad/s (smoothed) — for debug/UI
    double leftAngV   = 0.0;
};

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
    QVector3D update(const Quat& rightFootQuat,
                     const Quat& leftFootQuat,
                     const Quat& pelvisQuat,
                     const QVector3D& fkRightFoot,
                     const QVector3D& fkLeftFoot,
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

    // F1: 3.00 rad/s (= 172°/s) was too lenient — sStill stayed positive
    // through the swing phase of normal walking, causing premature anchor
    // commits and the "walks in place / 5-10 cm jumps" behaviour the
    // comment block below already complained about.  0.30 rad/s (= 17°/s)
    // sits comfortably above breathing tremor and standing micro-sway,
    // below all walking gait kinematics.
    double m_stillRad     = 0.30;
    double m_heightMargin = 0.03;
    int    m_latchTicks   = 3;
    double m_switchMargin = 0.04;
    double m_heightMarginSlow = 0.08;

    // --- Per-instance state that used to live as function-local statics ---
    QVector3D m_stillFootR   {0, 0, 0};
    QVector3D m_stillFootL   {0, 0, 0};
    int       m_stillTicksR  = 0;
    int       m_stillTicksL  = 0;
    QVector3D m_offsetLast   {0, 0, 0};
    // F2: previous frame's offset, used to predict pelvis world XY when
    // both feet are released (between strides) so anchor commits use a
    // current-frame estimate instead of a stale m_offsetLast.
    QVector3D m_offsetPrev   {0, 0, 0};
    bool      m_offsetReady  = false;
    bool      m_floorEmaValid= false;
    float     m_floorEma     = 0.0f;
    double    m_floorEmaRate = 0.05;

    // =====================================================================
        //               v3 STATE — weighted dual-anchor + ZUPT
        // =====================================================================
        // Pelvis angular-velocity tracker (used by pelvis-stillness gate and
        // ZUPT).
        Quat      m_prevPelvisQ      {1, 0, 0, 0};
        double    m_pelvisAngV        = 0.0;
        double    m_pelvisYawAngV     = 0.0;
        // F4: yaw-freeze is now per-foot.  A pirouette has one foot
        // pivoting (which should stay anchored) and the other swinging
        // (which should re-commit at touchdown).  Old single-flag freeze
        // dropped both feet's stability tracking whenever the pelvis spun,
        // causing the character to spiral.
        bool      m_yawFrozenPrev     = false;
        bool      m_yawFrozenPrevR    = false;
        bool      m_yawFrozenPrevL    = false;
        int       m_pelvisStillTicks  = 0;

        // FK-XY history ring buffers (planted-despite-rotating criterion).
        // 10 samples ≈ 111 ms at 90 Hz.
        static constexpr int kFKXYWindow = 10;
        std::array<QVector2D, kFKXYWindow> m_rFKXY {};
        std::array<QVector2D, kFKXYWindow> m_lFKXY {};
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
        enum PoseKind { PoseUnknown = 0, PoseStand = 1, PoseSit = 2,
                        PoseSquat = 3, PoseLying = 4 };
        PoseKind  m_pose              = PoseUnknown;
        int       m_poseTicks         = 0;

        // v3: ZUPT ticks counter.  When pelvis + both feet still for
        // >= m_zuptTicksThresh frames → offset fully frozen.
        int       m_zuptTicks         = 0;

        // --- tunables ---------------------------------------------------------
        // FIX «walks in place / 5-10 cm jumps»: тюнинг параметров локомоции.
        //
        // Старые значения требовали слишком высокой confidence для commit
        // (0.70) и медленно гасили стрижок при движении одной ноги (0.18),
        // из-за чего во время быстрой ходьбы анкор-офсеты не успевали
        // обновиться → персонаж стоял на месте.  Ниже — оптимизация под
        // нормальный темп ходьбы (1-2 m/s, длина шага 0.4-0.7 m).
        // F1: with m_stillRad now realistic (0.30 rad/s), FK-XY jitter
        // dominates — tighten the stability window from 4 cm to 2.5 cm so
        // the FK-XY criterion can carry more weight.
        double    m_fkxyStableRange   = 0.025;
        double    m_pelvisStillRad    = 0.20;   // было 0.12 — реже триггерим
                                                // pelvis-stillness gate во время ходьбы
        int       m_pelvisStillTicksThresh = 30;
        // F5: confidence hysteresis — widen from 0.35/0.25 (margin 0.10) to
        // 0.55/0.30 (margin 0.25).  With m_stillRad=3.0 the old narrow
        // hysteresis flipped a few times per second during stance; the new
        // values prevent that chatter while still committing within a few
        // frames of foot plant once the foot is genuinely still.
        double    m_confCommit        = 0.55;
        double    m_confRelease       = 0.30;
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
        // Actor height — must be set by owner after construction via
        // setActorHeight() before the first update() call.
        double    m_actorHeightM      = 1.75;

        // === v3 additions END ===

        // v3 no longer uses these v2 members, but they stay as data to keep
        // the class layout and v2-era calls to m_loco.anchor() compilable:
        //   m_floorEmaValid, m_floorEma, m_floorEmaRate — dead state
        //   m_stillFootR/L, m_stillTicksR/L            — dead, replaced by confR/L

      public:
        void setActorHeight(double h) { m_actorHeightM = std::max(0.5, h); }

      private:
        PoseKind _classifyPose(const Quat& qPelvis,
                               const QVector3D& fkR,
                               const QVector3D& fkL,
                               double& outTiltCos) const;
};

class SkeletonXsens {
public:
    // pose = "tpose" or "npose"
    SkeletonXsens(const ActorConfig& actor, const std::string& pose);

    // Forward kinematics.  segmentOrients has 23 entries (WXYZ quats), the
    // result has kXsensKeypointCount (28) 3-D points in meters.
    std::array<QVector3D, kXsensKeypointCount>
    computeKeypoints(const std::array<Quat, kXsensSegmentCount>& segOrients,
                     const QVector3D& rootPos) const;

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
//  MXTP parser (MVN streaming, big-endian).  Implements the public MVN
//  network-streamer wire format documented in the MVN User Manual.
// ============================================================================

// ============================================================================
//  XDA wire types — copies of the opaque structs used by Xsens Device API.
//  Layouts are verified via static_assert against the runtime ABI.
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
//  Connection state — standard enum used by the suit-receiver thread.
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

    // Try to bring up Manus gloves (loads manus.dll + CoreSdk_Initialize).
    // Returns true if initialisation succeeded.  Currently a soft-stub: the
    // full Manus integration is a separate layer; this just verifies the
    // SDK is present so the UI can tell the user.
    bool connectGloves();
    bool glovesReady()     const;       // physical glove(s) visible to Core
    bool glovesCoreReady() const;       // ManusCore responded to handshake
    bool glovesDllLoaded() const;       // ManusSDK.dll was found and loaded

    // Force all Madgwick filters to re-initialise from the next acc+mag
    // sample.  Called at the start of calibration countdown so the 3-second
    // prepare window doubles as a fusion warm-up.
    void resetFusion();

    // Connection transport preference. COM = scan USB serial ports (Awinda
    // dongle / MT-Link). Network = use XsScanner_enumerateNetworkDevices
    // (WiFi Awinda station / Body Pack V2 on ethernet).
    enum class Transport { ComPort, Network };
    void setTransport(Transport t);
    // User-supplied WiFi credentials. Purely informational for now — actual
    // Windows WLAN handshake is outside the XDA surface. The app stores them
    // so future WiFi-auto-connect can reuse them without re-asking.
    void setWifiCredentials(const QString& ssid, const QString& password);

    // Install sensor-to-segment alignment quaternions (one per tracker).
    // The receiver rotates each sensor's acc / gyr / mag by inv(s2s[i])
    // before handing them to the fusion filter (standard sensor-to-body
    // alignment, equivalent to applying the inverse s2s offset to raw IMU
    // samples before AHRS).
    void setS2sAlignment(const std::array<Quat, kXsensSegmentCount>& s2s);
    void resetS2sAlignment();
    Quat getS2s(int idx) const;

    // Per-sensor magnetometer magnitude (mag_magn).  Each sensor reads a
    // slightly different raw magnetic-field strength, so we scale each one
    // by its own mean |mag| from the calibration pose before feeding it
    // into the fusion filter.
    void setMagNormalisation(const std::array<double, kXsensSegmentCount>& magMagn);

    // Per-sensor accelerometer magnitude (acc_magn) and gyro DC bias
    // (gyr_bias).  Applied as `acc = acc / acc_magn` and
    // `gyr = gyr - gyr_bias` before sensor-to-segment rotation; standard
    // IMU calibration pre-processing.
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
        bool   useGloves = false;
        double heightCm = 175.0;
        double footLengthCm = 26.0;
        double armSpanCm = 0.0;
        double legLengthCm = 0.0;
        std::string poseKind = "tpose";
        std::array<Quat, kXsensSegmentCount> calibReference{};
        std::array<Quat, kXsensSegmentCount> tposeReference{};
        QVector3D                            tposePelvisPos{};
        bool                                 tposeCaptured = false;
    };

    NewSessionWizard(MocapReceiver* rx, bool testMode, QWidget* parent=nullptr);
    Result result() const { return m_result; }

    // Pre-select the "Suit with gloves" radio button.  Wired from main()
    // when the operator launches with --gloves so the wizard opens in the
    // right mode without an extra click.
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
    class QComboBox*     m_cbxTransport = nullptr;   // COM / WiFi
    class QLineEdit*     m_edSsid     = nullptr;
    class QLineEdit*     m_edPassword = nullptr;
    class QWidget*       m_wifiRow    = nullptr;

    // Page 3 (dims)
    class QLabel*          m_dimsTitle = nullptr;
    class QLabel*          m_lblHeight = nullptr;
    class QLabel*          m_lblFoot   = nullptr;
    class QLabel*          m_lblArm    = nullptr;   // FIX: размах рук перенесён сюда
    class QLabel*          m_lblLeg    = nullptr;   // FIX: длина ноги перенесена сюда
    class QDoubleSpinBox*  m_height = nullptr;
    class QDoubleSpinBox*  m_foot   = nullptr;
    class QDoubleSpinBox*  m_arm    = nullptr;      // FIX: arm span (опц., 0 = по росту)
    class QDoubleSpinBox*  m_leg    = nullptr;      // FIX: leg length (опц., 0 = по росту)
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

    QSize sizeHint() const override { return QSize(800, 600); }

    const std::array<Quat, kXsensSegmentCount>& filteredOrient() const { return m_orient; }

    // Last on-screen pelvis position (post-locomotion + scene shift). Used by
    // the live streamer so the receiver gets the same dynamic pelvis as the
    // viewport — without this pelvis is static and the rig "doesn't move".
    QVector3D lastRenderedPelvis() const { return m_lastRenderedPelvis; }

    QVector3D tickLoco(const std::array<Quat, kXsensSegmentCount>& q,
                       const QVector3D& fkRFoot,
                       const QVector3D& fkLFoot,
                       double tSec);
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

// Live-stream wizard.  Lets the operator pick a target plugin and a port,
// then start sending live pose data.  Concrete transport lives in the
// sender class — wired after the plugin protocol research lands.
enum class LiveTarget { BlenderMVN, XsensLivc };

struct LiveSettings {
    LiveTarget    target    = LiveTarget::BlenderMVN;
    QString       host      = "127.0.0.1";
    int           port      = 9763;           // MVN default; overridden in UI
    bool          useGloves = false;
    int           fps       = 60;             // 24 / 30 / 60 — UI throttle
    // T-pose origin position (meters) for each of 23 Xsens body segments.
    // Plugin (LiveLinkMvnSource) кладёт это в FTransform.Scale3D и потом
    // ULiveLinkMvnRetargetAsset делит unrealLength/xsensLength для масштаба
    // позиции pelvis. Если оставить нули — pelvis улетает на ~47x.
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

    // The position-aware overload sends real world-frame FK positions for
    // every body segment (MXTP02 carries per-segment absolute positions per
    // the protocol spec — sending zeros breaks UE retargeting and Blender
    // prop empties).  Pass kp from SkeletonXsens::computeKeypoints, offset
    // by locomotion / scene shift.
    void pushFrame(quint32 sample,
                   const std::array<Quat, kXsensSegmentCount>& segQuat,
                   const std::array<QVector3D, kXsensSegmentCount>& segPos,
                   const QVector3D& pelvisPos);

    // Glove variant — sends body segments then right+left hand fingers
    // (20 each).  Plugins auto-detect gloves via the header's
    // fingerSegmentCount field (Blender MVN plugin: segmentCount ≥ 32 ⇒
    // gloves present). Safe to call every tick; if gloves data is all-
    // zero the plugin gracefully ignores finger segments.
    //
    // wristPosR/L are the world-frame positions of the right/left wrist
    // joints (kp[SEG_RHand]/kp[SEG_LHand] from FK) — used to populate the
    // carpus slot (Manus index -1) with real positions instead of zeros.
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

    // v4: local skeleton instance — used by onRenderTick to compose the
    // full world orientation of each wrist via defAngFor() before FK-ing
    // the Manus-local finger positions.  Kept separate from the viewport's
    // own m_skel so the render-thread math never touches the GL state.
    std::unique_ptr<SkeletonXsens> m_skel;

    // Recording state.
    bool                       m_recording = false;
    RecordSettings             m_recCfg{};
    std::vector<RecordedFrame> m_recBuffer;
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
};
CliArgs parseCli(int argc, char** argv);

// Logger used in -test mode (flushes immediately).
void testLog(const std::string& msg, bool enabled);

// Dual-logging switch (defined in main.cpp). main() sets this to true when
// -test is on CLI; LiveStreamSender::pushFrame dumps each segment we send
// to stdout in the same format the Blender plug-in's file-logger emits.
extern bool g_testStreamLog;

}  // namespace fox
