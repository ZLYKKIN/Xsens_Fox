// Fox Mocap — implementation.  See main.h for the component overview.
//
// All skeleton math is ported 1:1 from HumanInertialPose-main:
//   * compute_skeleton_kpts_from_seg_orient   (base_skeleton.py)
//   * SkeletonXsens.default_seg_angles        (skeletons/xsens.py)
//   * add_dummy_segments                      (skeletons/xsens.py)
//   * quat_mult / quat_diff / vec_rotate /
//     convert_euler_to_quat                   (rotations.py)

#include "main.h"
#include "foxwire.h"   // MVN MXTP byte encoders (extracted, unit-tested)

#include <QtCore/QCommandLineOption>
#include <QtCore/QCommandLineParser>
#include <QtCore/QDebug>
#include <QtCore/QElapsedTimer>
#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtGui/QMouseEvent>
#include <QtGui/QOpenGLContext>
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QWheelEvent>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QStyle>
#include <QtWidgets/QTabBar>
#include <QtWidgets/QMenu>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSlider>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QFrame>
#include <QtWidgets/QProgressDialog>
#include <QtCore/QFile>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkInterface>
#include <QtEndian>
#include <QtCore/QDateTime>
#include <QtCore/QLocale>
#include <QtGui/QPixmap>
#include <QtGui/QPainter>
#include <QtGui/QPainterPath>
#include <QtGui/QPen>
#include <QtGui/QIcon>

#include <QtGui/QSurfaceFormat>

// xio Fusion — production AHRS from Sebastian Madgwick himself.
// Provides: startup ramp, gyro bias correction, acceleration rejection,
// magnetic rejection + recovery, all in NWU convention natively.
extern "C" {
#include "fusion/Fusion.h"
}

// Legacy immediate-mode GL symbols (glBegin/glVertex3f/…) live in the system
// OpenGL library on Windows.  We include the header explicitly because Qt's
// QOpenGLFunctions does not expose them.
#ifdef _WIN32
#  include <windows.h>
#  include <io.h>
#  include <fcntl.h>
#  include <dwmapi.h>
#endif
#include <GL/gl.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace fox {

// ---------------------------------------------------------------------------
// Win11 dark title bar + procedural fox icon. Enables immersive dark mode and
// sets accent colors so the caption/border/text match the app dark palette.
// ---------------------------------------------------------------------------
void applyDarkTitleBar(QWidget* w)
{
#ifdef _WIN32
    if (!w) return;
    HWND hwnd = reinterpret_cast<HWND>(w->winId());
    if (!hwnd) return;
    BOOL dark = TRUE;
    HRESULT hr = ::DwmSetWindowAttribute(hwnd, 20, &dark, sizeof(dark));
    if (FAILED(hr)) ::DwmSetWindowAttribute(hwnd, 19, &dark, sizeof(dark));
    COLORREF caption = 0x000E0E0E;
    COLORREF border  = 0x00242424;
    COLORREF text    = 0x00EAEAEA;
    ::DwmSetWindowAttribute(hwnd, 35, &caption, sizeof(caption));
    ::DwmSetWindowAttribute(hwnd, 34, &border,  sizeof(border));
    ::DwmSetWindowAttribute(hwnd, 36, &text,    sizeof(text));
#else
    (void)w;
#endif
}

QIcon makeFoxAppIcon()
{
    auto draw = [](int S) {
        QPixmap pm(S, S);
        pm.fill(Qt::transparent);
        QPainter p(&pm);
        p.setRenderHint(QPainter::Antialiasing, true);
        const qreal s = qreal(S);
        QColor fur   (0xF5, 0x7C, 0x2A);
        QColor furLt (0xFF, 0xA9, 0x55);
        QColor dark  (0x14, 0x14, 0x14);
        QColor white (0xF8, 0xF8, 0xF8);
        auto P = [s](qreal fx, qreal fy) { return QPointF(fx * s, fy * s); };
        QPolygonF earL; earL << P(0.14, 0.18) << P(0.30, 0.04) << P(0.38, 0.30);
        QPolygonF earR; earR << P(0.86, 0.18) << P(0.70, 0.04) << P(0.62, 0.30);
        p.setPen(QPen(dark, qMax(1.0, s / 32.0))); p.setBrush(fur);
        p.drawPolygon(earL); p.drawPolygon(earR);
        p.setBrush(QColor(0x80, 0x30, 0x10));
        QPolygonF earLi; earLi << P(0.20, 0.18) << P(0.30, 0.10) << P(0.33, 0.24);
        QPolygonF earRi; earRi << P(0.80, 0.18) << P(0.70, 0.10) << P(0.67, 0.24);
        p.setPen(Qt::NoPen); p.drawPolygon(earLi); p.drawPolygon(earRi);
        p.setPen(QPen(dark, qMax(1.0, s / 28.0)));
        QLinearGradient grad(P(0.5, 0.15), P(0.5, 0.95));
        grad.setColorAt(0.0, furLt); grad.setColorAt(1.0, fur);
        p.setBrush(grad);
        QPainterPath face;
        face.moveTo(P(0.15, 0.38));
        face.cubicTo(P(0.15, 0.65), P(0.30, 0.93), P(0.50, 0.96));
        face.cubicTo(P(0.70, 0.93), P(0.85, 0.65), P(0.85, 0.38));
        face.cubicTo(P(0.80, 0.28), P(0.60, 0.30), P(0.50, 0.32));
        face.cubicTo(P(0.40, 0.30), P(0.20, 0.28), P(0.15, 0.38));
        face.closeSubpath(); p.drawPath(face);
        p.setPen(Qt::NoPen); p.setBrush(white);
        QPainterPath cheek;
        cheek.moveTo(P(0.30, 0.60));
        cheek.cubicTo(P(0.35, 0.82), P(0.50, 0.90), P(0.50, 0.88));
        cheek.cubicTo(P(0.50, 0.90), P(0.65, 0.82), P(0.70, 0.60));
        cheek.cubicTo(P(0.60, 0.58), P(0.40, 0.58), P(0.30, 0.60));
        p.drawPath(cheek);
        p.setBrush(dark);
        p.drawEllipse(P(0.34, 0.52), s * 0.055, s * 0.070);
        p.drawEllipse(P(0.66, 0.52), s * 0.055, s * 0.070);
        QPolygonF nose; nose << P(0.44, 0.69) << P(0.56, 0.69) << P(0.50, 0.76);
        p.drawPolygon(nose);
        p.end();
        return pm;
    };
    QIcon icon;
    for (int sz : {16, 24, 32, 48, 64, 128, 256}) icon.addPixmap(draw(sz));
    return icon;
}

// ============================================================================
//  Segment names (for console/test output)
// ============================================================================

const char* kSegmentNames[kXsensSegmentCount] = {
    "pelvis", "l5", "l3", "t12", "t8", "neck", "head",
    "r_shoulder", "r_upper_arm", "r_forearm", "r_hand",
    "l_shoulder", "l_upper_arm", "l_forearm", "l_hand",
    "r_upper_leg", "r_lower_leg", "r_foot", "r_toe",
    "l_upper_leg", "l_lower_leg", "l_foot", "l_toe",
};

// Five finger chains, raw 20-entry layout (matches XESNSE finger_layout.hpp).
// Indices point into rightGlove[] / leftGlove[].  Each chain is drawn as a
// poly-line starting at the wrist (hand segment) and walking bone-by-bone,
// so the hand visually continues the forearm.
const int kFingerChains[kFingerChainCount][kFingerChainLen] = {
    { 0,  1,  2,  3 },   // thumb       (proximal → distal → extra tip)
    { 4,  5,  6,  7 },   // index
    { 8,  9, 10, 11 },   // middle
    { 12, 13, 14, 15 },  // ring
    { 16, 17, 18, 19 },  // pinky
};
const char* kFingerChainNames[kFingerChainCount] = {
    "thumb", "index", "middle", "ring", "pinky",
};

// ============================================================================
//  Quaternion math (WXYZ, scalar-first) now lives in foxmath.h / foxmath.cpp
//  — Quat, quat_mult, vec_rotate, swingTwistDecompose, euler_to_quat,
//  yaw_only_quat, slerp_quat, quat_angle_deg, mirror_y_quat and
//  hemisphereContinuous — extracted verbatim so the exact code the live
//  pipeline runs is unit-tested in isolation.  Included via main.h.
// ============================================================================

// ============================================================================
//  SkeletonXsens — topology, lengths, default angles, FK.
// ============================================================================

SkeletonXsens::SkeletonXsens(const ActorConfig& actor, const std::string& pose)
    : m_pose(pose)
{
    buildTopology();
    buildDefaultAngles();
    buildLengths(actor);
}

void SkeletonXsens::buildTopology()
{
    // Copy of the tables in hipose/skeleton/skeletons/xsens.py:
    //   seg_start_pts / seg_end_pts  (27 entries including 4 dummy stubs).
    //            | pelvis->head(7) |   right arm(5)  |    left arm(5)   |    right leg(5)  |   left leg(5)
    static const int S[kXsensSegmentCountWithDummies] = {
        0, 1, 2, 3, 4, 5, 6,    5, 7, 8, 9, 10,    5, 11, 12, 13, 14,
        0, 15, 16, 17, 18,      0, 19, 20, 21, 22,
    };
    static const int E[kXsensSegmentCountWithDummies] = {
        1, 2, 3, 4, 5, 6, 23,   7, 8, 9, 10, 24,   11, 12, 13, 14, 25,
        15, 16, 17, 18, 26,     19, 20, 21, 22, 27,
    };
    std::copy(std::begin(S), std::end(S), m_start.begin());
    std::copy(std::begin(E), std::end(E), m_end.begin());
}

// Free-function view of SkeletonXsens.default_seg_angles for a given pose.
// Used by the double-pose calibrator to compute gravity_in_body for both
// T and N references without instantiating a full skeleton.
std::array<Quat, kXsensSegmentCount>
defaultSegAnglesFor(const std::string& pose)
{
    const double P = M_PI;
    auto E = [](double x, double y, double z) {
        return euler_to_quat(x, y, z, "XYZ");
    };
    std::array<Quat, kXsensSegmentCount> out{};
    if (pose == "tpose") {
        out = {
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0, 0, -P/2), E(0, 0, -P/2), E(0, 0, -P/2), E(0, 0, -P/2),
            E(0, 0,  P/2), E(0, 0,  P/2), E(0, 0,  P/2), E(0, 0,  P/2),
            E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0),     E(0, 0, 0),
            E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0),     E(0, 0, 0),
        };
    } else if (pose == "kpose") {
        out = {
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0, 0, -P/2), E(0, 0, 0),     E(0, 0, 0),     E(0, 0, 0),
            E(0, 0,  P/2), E(0, 0, 0),     E(0, 0, 0),     E(0, 0, 0),
            E(0, 0, 0),    E(0, P/2, 0),   E(0, 0, 0),     E(0, 0, 0),
            E(0, 0, 0),    E(0, P/2, 0),   E(0, 0, 0),     E(0, 0, 0),
        };
    } else {
        out = {
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0,    0, -P/2),
            E( P/2, 0, -P/2), E( P/2, 0, -P/2), E( P/2, 0, -P/2),
            E(0,    0,  P/2),
            E(-P/2, 0,  P/2), E(-P/2, 0,  P/2), E(-P/2, 0,  P/2),
            E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0), E(0, 0, 0),
            E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0), E(0, 0, 0),
        };
    }
    return out;
}

void SkeletonXsens::buildDefaultAngles()
{
    // Copied from SkeletonXsens.__init__ in xsens.py.
    // Both presets use seq="XYZ".  23 entries (no dummies — dummy angles
    // are derived from T8 and Pelvis at FK time in addDummySegments).
    const double P = M_PI;

    auto E = [](double x, double y, double z) {
        return euler_to_quat(x, y, z, "XYZ");
    };

    if (m_pose == "tpose") {
        m_defAng = {
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            // right arm
            E(0, 0, -P/2), E(0, 0, -P/2), E(0, 0, -P/2), E(0, 0, -P/2),
            // left arm
            E(0, 0,  P/2), E(0, 0,  P/2), E(0, 0,  P/2), E(0, 0,  P/2),
            // right leg
            E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0),     E(0, 0, 0),
            // left leg
            E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0),     E(0, 0, 0),
        };
    } else {
        // npose
        m_defAng = {
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            E(0, -P/2, 0), E(0, -P/2, 0), E(0, -P/2, 0),
            // right arm — shoulder remains along -Z, then arm bends down (N-pose)
            E(0,    0, -P/2),
            E( P/2, 0, -P/2), E( P/2, 0, -P/2), E( P/2, 0, -P/2),
            // left arm
            E(0,    0,  P/2),
            E(-P/2, 0,  P/2), E(-P/2, 0,  P/2), E(-P/2, 0,  P/2),
            // right leg
            E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0), E(0, 0, 0),
            // left leg
            E(0,  P/2, 0), E(0,  P/2, 0), E(0, 0, 0), E(0, 0, 0),
        };
    }
}

void SkeletonXsens::buildLengths(const ActorConfig& actor)
{
    // 27 entries in segment order (start->end).
    //
    // Two corrections on top of the first pass:
    //   1. "shoulder" segment was 0.259·h/2 ≈ 22 cm — that was the
    //       bi-acromial half-width (T8 → acromion lateral distance).  In
    //       SkeletonXsens it sits INSIDE the arm chain (scapula→shoulder
    //       joint→upper arm start), and hipose's default is 0.10 m — we
    //       now scale that default by (h / 1.75).
    //   2. "foot" & "toe": foot was 0.039·h (≈ 6.8 cm, nonsense) and the
    //       whole user-supplied foot length went to the TOE bone.  Real
    //       anatomy splits heel-to-ball ≈ 60 % of foot length, ball-to-tip
    //       ≈ 40 %.  Using the user's foot-length directly now.
    //   3. Spine and small stubs are now scaled by trunk ratio (h/1.75)
    //       so tall/short actors don't inherit the hipose h=1.75 defaults.
    const double h  = actor.heightCm      / 100.0;
    const double fl = actor.footLengthCm  / 100.0;
    const double trunkScale = h / 1.75;   // spine & stubs scale with height

    double armScale = 1.0;
    if (actor.armSpanCm > 0.0) {
        const double bodyWidthM = 0.30 * (h / 1.75);
        const double armPerSideM = std::max(0.10,
            (actor.armSpanCm / 100.0 - bodyWidthM) * 0.5);
        const double defArmM = 0.44 * h;
        armScale = (defArmM > 1e-6) ? (armPerSideM / defArmM) : 1.0;
    }
    double legScale = 1.0;
    if (actor.legLengthCm > 0.0) {
        const double legPerSideM = std::max(0.20, actor.legLengthCm / 100.0);
        const double defLegM = 0.491 * h;
        legScale = (defLegM > 1e-6) ? (legPerSideM / defLegM) : 1.0;
    }
    // FIX issue 5: hip width / shoulder width / trunk length.
    // hip stub L[17]/L[22] = pelvis → hip joint (полширины таза).
    // scapular stub L[7]/L[16] = T8 → плечевой сустав (полширины плеч).
    // trunk segments L[0..5] = pelvis→neck (6 vertebrae).
    const double pelvisHalfM = (actor.hipWidthCm > 0.0)
        ? std::max(0.05, actor.hipWidthCm / 200.0)
        : 0.10 * trunkScale;
    const double scapHalfM = (actor.shoulderWidthCm > 0.0)
        ? std::max(0.05, actor.shoulderWidthCm / 200.0)
        : 0.05 * trunkScale;
    // Дефолтная сумма длин spine + head = 0.55h (по hipose).  Если actor
    // ввёл свою длину туловища, нормируем 6 сегментов spine по этому
    // значению (head→top-of-head не масштабируется — это часть черепа).
    const double trunkSegScale = (actor.trunkLengthCm > 0.0)
        ? std::max(0.30, actor.trunkLengthCm / 100.0) / (0.55 * h)
        : trunkScale;

    const std::array<float, kXsensSegmentCountWithDummies> L = {
        // ----- spine + head -----
        float(0.10 * trunkSegScale),  // pelvis → L5
        float(0.10 * trunkSegScale),  // L5 → L3
        float(0.10 * trunkSegScale),  // L3 → T12
        float(0.15 * trunkSegScale),  // T12 → T8
        float(0.10 * trunkSegScale),  // T8 → neck
        float(0.05 * trunkSegScale),  // neck → head
        float(0.130 * h),             // head → top-of-head (vertex)
        // ----- right arm -----
        float(scapHalfM),             // T8 → R-scapular stub
        float(0.10 * trunkScale),
        float(0.186 * h * armScale),
        float(0.146 * h * armScale),
        float(0.108 * h * armScale),
        // ----- left arm (mirror) -----
        float(scapHalfM),             // T8 → L-scapular stub
        float(0.10 * trunkScale),
        float(0.186 * h * armScale),
        float(0.146 * h * armScale),
        float(0.108 * h * armScale),
        // ----- right leg -----
        float(pelvisHalfM),           // pelvis stub  (pelvis → hip joint)
        float(0.245 * h * legScale),  // upper leg    (hip    → knee)
        float(0.246 * h * legScale),  // lower leg    (knee   → ankle)
        float(0.60  * fl),            // foot (heel → ball, ~60 % of foot length)
        float(0.40  * fl),            // toe  (ball → tip,  ~40 % of foot length)
        // ----- left leg (mirror) -----
        float(pelvisHalfM),
        float(0.245 * h * legScale),
        float(0.246 * h * legScale),
        float(0.60  * fl),
        float(0.40  * fl),
    };
    m_len = L;
}

std::array<Quat, kXsensSegmentCountWithDummies>
SkeletonXsens::addDummySegments(const std::array<Quat, kXsensSegmentCount>& s) const
{
    // 4 dummy-сегмента, которые превращают 23-цепочку в 27-цепочку FK.
    // Смещения тазовых и лопаточных "заглушек" теперь все по ±π/2 —
    // анатомически верные 90° отступы от оси позвоночника, без 100°
    // костыля который вводил асимметрию +10° в каждой руке.
    const double P = M_PI;
    const Quat t8yaw     = yaw_only_quat(s[SEG_T8]);
    const Quat pelvisYaw = yaw_only_quat(s[SEG_Pelvis]);
    const Quat rScap = quat_mult(t8yaw,     euler_to_quat(0, -P/2, -P/2, "XYZ"));
    const Quat lScap = quat_mult(t8yaw,     euler_to_quat(0, -P/2,  P/2, "XYZ"));
    const Quat rHip  = quat_mult(pelvisYaw, euler_to_quat(0,  0,   -P/2, "XYZ"));
    const Quat lHip  = quat_mult(pelvisYaw, euler_to_quat(0,  0,    P/2, "XYZ"));

    std::array<Quat, kXsensSegmentCountWithDummies> out{};
    int k = 0;
    for (int i = 0; i < 7; ++i) out[k++] = s[i];              // pelvis..head
    out[k++] = rScap;
    for (int i = 7;  i < 11; ++i) out[k++] = s[i];             // right arm
    out[k++] = lScap;
    for (int i = 11; i < 15; ++i) out[k++] = s[i];             // left arm
    out[k++] = rHip;
    for (int i = 15; i < 19; ++i) out[k++] = s[i];             // right leg
    out[k++] = lHip;
    for (int i = 19; i < 23; ++i) out[k++] = s[i];             // left leg
    return out;
}

// Forward declarations — the helpers themselves live near LocomotionSolver.
static Quat constrain_shoulder_cone(const Quat& q_seg, const Quat& q_pelvis,
                                     bool isRight);
static Quat constrain_wrist_twist(const Quat& q_hand_world,
                                  const Quat& q_forearm_world,
                                  const Quat& q_anchor_local,
                                  double maxFlexRad, double maxLatDevRad,
                                  double twistWeight, bool isRight);

std::array<QVector3D, kXsensKeypointCount>
SkeletonXsens::computeKeypoints(const std::array<Quat, kXsensSegmentCount>& raw,
                                const QVector3D& rootPos,
                                FkDiag* diag) const
{
    // Step 1: apply default segment angles  (quat_mult(raw, m_defAng)).
    // Guard against denormalised or NaN input quaternions produced by
    // upstream slerp blends — any non-unit quat would silently scale the
    // bone-vector rotation and make the skeleton collapse.
    std::array<Quat, kXsensSegmentCount> oriented{};
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        Quat safeRaw = raw[i];
        const double n2 = safeRaw.w*safeRaw.w + safeRaw.x*safeRaw.x
                        + safeRaw.y*safeRaw.y + safeRaw.z*safeRaw.z;
        if (n2 < 1e-10 || !std::isfinite(n2))
            safeRaw = Quat(1, 0, 0, 0);
        else if (std::abs(n2 - 1.0) > 1e-6)
            safeRaw = safeRaw.normalized();
        oriented[i] = quat_mult(safeRaw, m_defAng[i]);
    }

    // Step 1b: anatomical soft-constraint on the shoulder joints only.
    // Keeps a glitched mag yaw from throwing an arm "behind the back".
    const Quat& q_pelvis_world = oriented[SEG_Pelvis];
    oriented[SEG_RUpperArm] = constrain_shoulder_cone(
        oriented[SEG_RUpperArm], q_pelvis_world, /*isRight=*/true);
    oriented[SEG_LUpperArm] = constrain_shoulder_cone(
        oriented[SEG_LUpperArm], q_pelvis_world, /*isRight=*/false);

    // Step 2: expand to 27 (add dummy stubs).
    const auto global = addDummySegments(oriented);

    // Step 3: rotate each segment's [L, 0, 0] bone vector into world.
    std::array<QVector3D, kXsensSegmentCountWithDummies> boneVec{};
    for (int s = 0; s < kXsensSegmentCountWithDummies; ++s) {
        const QVector3D local(m_len[s], 0.0f, 0.0f);
        boneVec[s] = vec_rotate(local, global[s]);
    }

    // Step 4: walk kinematic chain from root (pelvis, index 0).
    std::array<QVector3D, kXsensKeypointCount> kp{};
    std::array<bool,      kXsensKeypointCount> seen{};
    kp[0]   = rootPos;
    seen[0] = true;
    for (int s = 0; s < kXsensSegmentCountWithDummies; ++s) {
        const int a = m_start[s];
        const int b = m_end[s];
        if (!seen[a]) continue;                     // start not yet positioned
        kp[b]  = kp[a] + boneVec[s];
        seen[b] = true;
    }
    // -test single-source capture: every FK intermediate exactly as used here.
    if (diag) {
        diag->oriented = oriented;
        diag->global   = global;
        diag->boneVec  = boneVec;
        diag->len      = m_len;
        diag->kp       = kp;
        diag->rootPos  = rootPos;
    }
    return kp;
}

// ============================================================================
//  Calibration AHRS-init helper — analytic ecompass orientation consumed by
//  the double/triple-pose sensor-to-segment solver.  The live per-segment
//  attitude filter is the xio Fusion library (see MocapReceiver::run); the
//  hand-rolled Madgwick filter that used to live here was unused and removed.
// ============================================================================

// ecompass analytical init — from acc (gravity) + mag (horizontal north).
// Returns an NED quaternion identical to ahrs.common.orientation.ecompass
// with frame="NED", representation="quaternion".
static Quat ecompassNED(const QVector3D& a, const QVector3D& m)
{
    // Build rotation matrix whose rows are sensor-axes expressed in NED.
    QVector3D down = -a.normalized();                 // gravity points up in sensor
    // Degenerate guard: when the magnetic vector is (nearly) parallel to
    // gravity, cross(down, m) collapses to ~0 and the basis is undefined.
    // Fall back to any axis perpendicular to `down` so the matrix stays
    // orthonormal and finite — yaw is then arbitrary, but the caller's
    // residual check rejects such a pose anyway.
    QVector3D east = QVector3D::crossProduct(down, m);
    if (east.lengthSquared() < 1e-12f) {
        east = QVector3D::crossProduct(down, QVector3D(1, 0, 0));
        if (east.lengthSquared() < 1e-12f)
            east = QVector3D::crossProduct(down, QVector3D(0, 1, 0));
    }
    east.normalize();
    QVector3D north = QVector3D::crossProduct(east, down).normalized();
    // R: rows = north, east, down  (NED) — world basis in sensor frame.
    const double m00 = north.x(), m01 = north.y(), m02 = north.z();
    const double m10 = east.x(),  m11 = east.y(),  m12 = east.z();
    const double m20 = down.x(),  m21 = down.y(),  m22 = down.z();
    const double tr = m00 + m11 + m22;
    Quat q;
    if (tr > 0.0) {
        const double s = 0.5 / std::sqrt(tr + 1.0);
        q.w = 0.25 / s;
        q.x = (m21 - m12) * s;
        q.y = (m02 - m20) * s;
        q.z = (m10 - m01) * s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
        q.w = (m21 - m12) / s;
        q.x = 0.25 * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25 * s;
        q.z = (m12 + m21) / s;
    } else {
        const double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25 * s;
    }
    return q.normalized();
}

// ============================================================================
//  LocomotionSolver — direct port of the contract observed in the
//  xme64.dll ghidra decomp (see header for references).
// ============================================================================

// ---------- [A]/[B] yaw_only_quat + slerp_quat now live in foxmath.{h,cpp} ----
// Gimbal-lock note (still relevant to the FK below): the composition
// oriented[Pelvis] = raw · Rot_y(-π/2) keeps the pelvis near -90° pitch, where a
// Tait-Bryan Z-Y-X atan2 yaw extraction would collapse to 0.  yaw_only_quat's
// swing-twist split about world-Z sidesteps that singularity and is what drives
// the shoulder cone.

// ---------- [C] shoulder cone soft-constraint ----------
static Quat constrain_shoulder_cone(const Quat& q_seg, const Quat& q_pelvis,
                                    bool isRight)
{
    // Направление сегмента-кости в мире.
    const QVector3D boneWorld = vec_rotate(QVector3D(1.0f, 0.0f, 0.0f), q_seg);

    // Выражаем кость в системе координат таза:
    //   pelvis local +X = вверх (позвоночник),
    //   pelvis local +Y = влево для актёра,
    //   pelvis local +Z = назад для актёра.
    const QVector3D pelvisFrame = vec_rotate(boneWorld, q_pelvis.inv());
    const float boneL = pelvisFrame.length();
    if (boneL < 1e-6f) return q_seg;

    const QVector3D n = pelvisFrame / boneL;
    const float upN   = n.x();
    const float latN  = n.y();
    const float backN = n.z();

    // (1) ЧЕРЕЗ СРЕДНЮЮ ЛИНИЮ — рука не просто "своя сторона", а уходит
    //     ЗА свою плечевую ось.  Правая рука должна быть latN < -0.20
    //     (порог выше чем прежние -0.08 → не ловим замах, guard, reach).
    const bool acrossMid = isRight ? (latN < -0.20f) : (latN >  0.20f);

    // (2) ЯВНО ЗА СПИНУ — минимум 0.40 вместо 0.30.
    const bool clearlyBehind = backN > 0.40f;

    // (3) ТОЛЬКО В ГЛЕНО-ГУМЕРАЛЬНОЙ ГОРИЗОНТАЛИ — |upN| < 0.55.
    //     Исключает overhead-замах и удары сверху вниз из зоны клампа.
    const bool inHorizBand = std::abs(upN) < 0.55f;

    if (!(acrossMid && clearlyBehind && inHorizBand))
        return q_seg;

    // Цель: завернуть кость обратно к плечевой плоскости, сохраняя длину.
    constexpr float kBackMax = 0.30f;
    QVector3D targetPelvis(upN, latN, std::min(backN, kBackMax));
    const float tLen = targetPelvis.length();
    if (tLen < 1e-6f) return q_seg;
    targetPelvis = (targetPelvis / tLen) * boneL;

    const QVector3D targetWorld = vec_rotate(targetPelvis, q_pelvis);
    const QVector3D from = boneWorld.normalized();
    const QVector3D to   = targetWorld.normalized();

    const float d = QVector3D::dotProduct(from, to);
    if (d > 0.9999f) return q_seg;

    Quat qCorrect;
    if (d < -0.9999f) {
        QVector3D axis = QVector3D::crossProduct(QVector3D(0, 0, 1), from);
        if (axis.lengthSquared() < 1e-6f)
            axis = QVector3D::crossProduct(QVector3D(1, 0, 0), from);
        axis.normalize();
        qCorrect = Quat(0.0, axis.x(), axis.y(), axis.z());
    } else {
        const QVector3D axis = QVector3D::crossProduct(from, to);
        const float s = std::sqrt((1.0f + d) * 2.0f);
        qCorrect = Quat(0.5 * s, axis.x() / s, axis.y() / s, axis.z() / s)
                       .normalized();
    }

    // Кубический ramp: производная 0 на границе зоны, максимум 0.30.
    float violation = std::max(0.0f, backN - kBackMax);
    violation = std::min(violation, 0.60f);
    const float t        = violation / 0.60f;
    const float strength = 0.30f * t * t * t;

    const Quat qPartial = slerp_quat(Quat(1, 0, 0, 0), qCorrect, strength);
    return quat_mult(qPartial, q_seg).normalized();
}

static Quat constrain_wrist_twist(const Quat& q_hand_world,
                                  const Quat& q_forearm_world,
                                  const Quat& q_anchor_local,
                                  double maxFlexRad, double maxLatDevRad,
                                  double twistWeight, bool isRight)
{
    (void)isRight;
    // Hand-in-forearm-local frame.
    const Quat qFAinv = q_forearm_world.inv();
    Quat qLocal = quat_mult(qFAinv, q_hand_world).normalized();

    // FIX (gloves polish): swing/twist decompose производим относительно
    // T-pose anchor, а не identity.  Без этого "нулевая поза" соответствует
    // identity в hand-local frame, но анатомически T-поза ладонью вниз —
    // НЕ identity (там defAngFor задаёт Rz(±π/2)).  Поэтому clamp
    // flex/lat-dev раньше работал относительно неправильного нуля.
    // Теперь clamp применяется к "отклонению от T-pose позы".
    Quat qLocalRel = quat_mult(q_anchor_local.inv(), qLocal).normalized();
    if (qLocalRel.w < 0) {
        qLocalRel.w = -qLocalRel.w; qLocalRel.x = -qLocalRel.x;
        qLocalRel.y = -qLocalRel.y; qLocalRel.z = -qLocalRel.z;
    }

    Quat swing, twist;
    swingTwistDecompose(qLocalRel, QVector3D(1.0f, 0.0f, 0.0f), swing, twist);

    double twistHalf = std::atan2(twist.x, twist.w);
    double twistAng = 2.0 * twistHalf * twistWeight;
    Quat twistOut(std::cos(twistAng * 0.5),
                  std::sin(twistAng * 0.5), 0.0, 0.0);

    if (swing.w < 0) {
        swing.w = -swing.w; swing.x = -swing.x;
        swing.y = -swing.y; swing.z = -swing.z;
    }
    const double sxy = std::sqrt(swing.y * swing.y + swing.z * swing.z);
    const double swingAng = 2.0 * std::atan2(sxy, swing.w);
    const double ay = (sxy > 1e-9) ? swing.y / sxy : 0.0;
    const double az = (sxy > 1e-9) ? swing.z / sxy : 0.0;
    const double flexAng = swingAng * ay;
    const double devAng  = swingAng * az;
    const double flexC = std::clamp(flexAng, -maxFlexRad, maxFlexRad);
    const double devC  = std::clamp(devAng,  -maxLatDevRad, maxLatDevRad);
    const double newAng = std::sqrt(flexC * flexC + devC * devC);
    Quat swingOut(1.0, 0.0, 0.0, 0.0);
    if (newAng > 1e-9) {
        const double half = newAng * 0.5;
        const double s = std::sin(half) / newAng;
        swingOut = Quat(std::cos(half), 0.0, flexC * s, devC * s);
    }

    // Recompose: anchor * swing-clamped * twist-clamped — затем вернуть
    // в world через forearm.
    const Quat qLocalRelOut = quat_mult(swingOut, twistOut).normalized();
    const Quat qLocalOut = quat_mult(q_anchor_local, qLocalRelOut).normalized();
    return quat_mult(q_forearm_world, qLocalOut).normalized();
}

// ---------- [D] quat_angle_deg now lives in foxmath.{h,cpp} ----------

// ---------- Matrix → quaternion helper ----------
// Pass 3x3 rotation matrix by rows m00,m01,m02,m10,m11,m12,m20,m21,m22.
static Quat matToQuat(double m00, double m01, double m02,
                      double m10, double m11, double m12,
                      double m20, double m21, double m22)
{
    const double tr = m00 + m11 + m22;
    Quat q;
    if (tr > 0.0) {
        const double s = 0.5 / std::sqrt(tr + 1.0);
        q.w = 0.25 / s;
        q.x = (m21 - m12) * s;
        q.y = (m02 - m20) * s;
        q.z = (m10 - m01) * s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
        q.w = (m21 - m12) / s;
        q.x = 0.25 * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25 * s;
        q.z = (m12 + m21) / s;
    } else {
        const double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25 * s;
    }
    return q.normalized();
}

// ---------- TRIAD (double-pose) sensor-to-body solver ----------
//
// Given two linearly-independent vector observations made in BOTH the body
// frame AND the sensor frame, return the rotation R_sensor_to_body such
// that R * v_sensor ≈ v_body for both pairs.
//
// In our pipeline: v1 = gravity in T-pose, v2 = gravity in N-pose.  Both
// are unit vectors (normalised).  For segments where the two body-frame
// gravity vectors are NEARLY parallel (torso, legs, feet — body is upright
// in both poses) TRIAD degenerates; the caller must fall back to the
// ecompass single-pose estimate.
//
// Returned quaternion rotates sensor-frame vectors into body-frame —
// this is what we store as `s2sInv` in MocapReceiver.  `s2s` itself is
// the inverse (body → sensor).
static Quat triadSolve(const QVector3D& v1b, const QVector3D& v2b,
                       const QVector3D& v1s, const QVector3D& v2s)
{
    const QVector3D r1 = v1b.normalized();
    const QVector3D r2raw = QVector3D::crossProduct(v1b, v2b);
    const QVector3D r2 = r2raw.normalized();
    const QVector3D r3 = QVector3D::crossProduct(r1, r2).normalized();

    const QVector3D s1 = v1s.normalized();
    const QVector3D s2raw = QVector3D::crossProduct(v1s, v2s);
    const QVector3D s2 = s2raw.normalized();
    const QVector3D s3 = QVector3D::crossProduct(s1, s2).normalized();

    // R = r1 s1^T + r2 s2^T + r3 s3^T — rotation mapping s_k → r_k.
    const double m00 = r1.x()*s1.x() + r2.x()*s2.x() + r3.x()*s3.x();
    const double m01 = r1.x()*s1.y() + r2.x()*s2.y() + r3.x()*s3.y();
    const double m02 = r1.x()*s1.z() + r2.x()*s2.z() + r3.x()*s3.z();
    const double m10 = r1.y()*s1.x() + r2.y()*s2.x() + r3.y()*s3.x();
    const double m11 = r1.y()*s1.y() + r2.y()*s2.y() + r3.y()*s3.y();
    const double m12 = r1.y()*s1.z() + r2.y()*s2.z() + r3.y()*s3.z();
    const double m20 = r1.z()*s1.x() + r2.z()*s2.x() + r3.z()*s3.x();
    const double m21 = r1.z()*s1.y() + r2.z()*s2.y() + r3.z()*s3.y();
    const double m22 = r1.z()*s1.z() + r2.z()*s2.z() + r3.z()*s3.z();
    const double det = m00 * (m11*m22 - m12*m21)
                     - m01 * (m10*m22 - m12*m20)
                     + m02 * (m10*m21 - m11*m20);
    double mm00 = m00, mm01 = m01, mm02 = m02;
    double mm10 = m10, mm11 = m11, mm12 = m12;
    double mm20 = m20, mm21 = m21, mm22 = m22;
    if (det < 0.0) {
        mm00 = -mm00; mm10 = -mm10; mm20 = -mm20;
    }
    Quat q = matToQuat(mm00, mm01, mm02, mm10, mm11, mm12, mm20, mm21, mm22);
    if (q.w < 0.0) {
        q = Quat(-q.w, -q.x, -q.y, -q.z);
    }
    return q;
}

static double triadResidualDeg(const Quat& q_s2b,
                               const QVector3D& v1b, const QVector3D& v2b,
                               const QVector3D& v1s, const QVector3D& v2s)
{
    auto angBetweenDeg = [](const QVector3D& a, const QVector3D& b) -> double {
        const double na = double(a.length()), nb = double(b.length());
        if (na < 1e-9 || nb < 1e-9) return 0.0;
        const double c = double(QVector3D::dotProduct(a, b)) / (na * nb);
        const double cc = c > 1.0 ? 1.0 : (c < -1.0 ? -1.0 : c);
        return std::acos(cc) * 180.0 / M_PI;
    };
    const QVector3D r1 = vec_rotate(v1s.normalized(), q_s2b);
    const QVector3D r2 = vec_rotate(v2s.normalized(), q_s2b);
    return std::max(angBetweenDeg(r1, v1b.normalized()),
                    angBetweenDeg(r2, v2b.normalized()));
}

// One candidate observation pair for the double-pose TRIAD solver.
struct TriadPairCand {
    float sep;                       // |cross(b1,b2)| — pose separation
    QVector3D b1, b2, s1, s2;        // body- and sensor-frame observations
    const char* name;
};

// Pick the lowest-residual TRIAD solution among candidate pairs (skipping pairs
// whose separation is too small to be well-conditioned).  Shared by the K-pose
// calibration's Wahba-fallback and TRIAD-only branches so the selection logic
// lives in exactly one place.
static void bestTriadOfPairs(const TriadPairCand (&cands)[3],
                             Quat& outQ, double& outResidualDeg,
                             const char*& outName, float& outSep)
{
    outQ = Quat(1, 0, 0, 0);
    outResidualDeg = 999.0;
    outName = "none";
    outSep = 0.0f;
    for (const auto& c : cands) {
        if (c.sep <= 0.3f) continue;
        const Quat q = triadSolve(c.b1, c.b2, c.s1, c.s2);
        const double r = triadResidualDeg(q, c.b1, c.b2, c.s1, c.s2);
        if (r < outResidualDeg) {
            outResidualDeg = r;
            outQ = q;
            outName = c.name;
            outSep = c.sep;
        }
    }
}

static double mirrorYDeviationDeg(const Quat& qR, const Quat& qL)
{
    const Quat mY = mirror_y_quat(qL);
    double dot = qR.w*mY.w + qR.x*mY.x + qR.y*mY.y + qR.z*mY.z;
    if (dot < 0.0) dot = -dot;
    if (dot > 1.0) dot = 1.0;
    return 2.0 * std::acos(dot) * 180.0 / M_PI;
}

static double parallelDeviationDeg(const Quat& qR, const Quat& qL)
{
    double dot = qR.w*qL.w + qR.x*qL.x + qR.y*qL.y + qR.z*qL.z;
    if (dot < 0.0) dot = -dot;
    if (dot > 1.0) dot = 1.0;
    return 2.0 * std::acos(dot) * 180.0 / M_PI;
}

static double ecompResidualDeg(const Quat& q_s2b,
                               const QVector3D& vb, const QVector3D& vs)
{
    const QVector3D r = vec_rotate(vs.normalized(), q_s2b);
    const QVector3D b = vb.normalized();
    const double na = double(r.length()), nb = double(b.length());
    if (na < 1e-9 || nb < 1e-9) return 0.0;
    double c = double(QVector3D::dotProduct(r, b)) / (na * nb);
    if (c < 0.0) c = -c;
    if (c > 1.0) c = 1.0;
    return std::acos(c) * 180.0 / M_PI;
}

static double confidenceFromResidual(double residualDeg, int mode)
{
    if (mode == 2) return 0.0;
    const double r = residualDeg;
    double c;
    if (r <= 3.0)       c = 1.0;
    else if (r <= 10.0) c = 1.0 - (r - 3.0) / 7.0 * 0.4;
    else if (r <= 20.0) c = 0.6 - (r - 10.0) / 10.0 * 0.3;
    else if (r <= 40.0) c = 0.3 - (r - 20.0) / 20.0 * 0.3;
    else                c = 0.0;
    if (mode == 1) c *= 0.85;
    if (c < 0.0) c = 0.0;
    if (c > 1.0) c = 1.0;
    return c;
}

static void jacobiSym4(double A[4][4], double U[4][4])
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            U[i][j] = (i == j) ? 1.0 : 0.0;

    for (int sweep = 0; sweep < 60; ++sweep) {
        int p = 0, q = 1;
        double maxOff = std::abs(A[0][1]);
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                if (std::abs(A[i][j]) > maxOff) {
                    maxOff = std::abs(A[i][j]); p = i; q = j;
                }
            }
        }
        if (maxOff < 1e-14) break;

        const double app = A[p][p];
        const double aqq = A[q][q];
        const double apq = A[p][q];
        double t;
        if (std::abs(apq) < 1e-300) { t = 0.0; }
        else {
            const double theta = (aqq - app) / (2.0 * apq);
            t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta*theta + 1.0));
        }
        const double c = 1.0 / std::sqrt(t*t + 1.0);
        const double s = t * c;

        A[p][p] = app - t * apq;
        A[q][q] = aqq + t * apq;
        A[p][q] = A[q][p] = 0.0;
        for (int k = 0; k < 4; ++k) {
            if (k == p || k == q) continue;
            const double akp = A[k][p];
            const double akq = A[k][q];
            A[k][p] = A[p][k] = c * akp - s * akq;
            A[k][q] = A[q][k] = s * akp + c * akq;
        }
        for (int k = 0; k < 4; ++k) {
            const double ukp = U[k][p];
            const double ukq = U[k][q];
            U[k][p] = c * ukp - s * ukq;
            U[k][q] = s * ukp + c * ukq;
        }
    }
}

static void jacobiSym3(double A[3][3], double U[3][3])
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            U[i][j] = (i == j) ? 1.0 : 0.0;

    for (int sweep = 0; sweep < 40; ++sweep) {
        int p = 0, q = 1;
        double maxOff = std::abs(A[0][1]);
        const double off02 = std::abs(A[0][2]);
        const double off12 = std::abs(A[1][2]);
        if (off02 > maxOff) { maxOff = off02; p = 0; q = 2; }
        if (off12 > maxOff) { maxOff = off12; p = 1; q = 2; }
        if (maxOff < 1e-14) break;

        const double app = A[p][p];
        const double aqq = A[q][q];
        const double apq = A[p][q];
        double t;
        if (std::abs(apq) < 1e-300) { t = 0.0; }
        else {
            const double theta = (aqq - app) / (2.0 * apq);
            t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta*theta + 1.0));
        }
        const double c = 1.0 / std::sqrt(t*t + 1.0);
        const double s = t * c;

        A[p][p] = app - t * apq;
        A[q][q] = aqq + t * apq;
        A[p][q] = A[q][p] = 0.0;
        for (int k = 0; k < 3; ++k) {
            if (k == p || k == q) continue;
            const double akp = A[k][p];
            const double akq = A[k][q];
            A[k][p] = A[p][k] = c * akp - s * akq;
            A[k][q] = A[q][k] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; ++k) {
            const double ukp = U[k][p];
            const double ukq = U[k][q];
            U[k][p] = c * ukp - s * ukq;
            U[k][q] = s * ukp + c * ukq;
        }
    }
}

static Quat wahbaSolve(const QVector3D* vb, const QVector3D* vs,
                       const double* w, int n)
{
    double B[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (int k = 0; k < n; ++k) {
        const QVector3D r = vb[k].normalized();
        const QVector3D b = vs[k].normalized();
        const double wk = w[k];
        const double rr[3] = { double(r.x()), double(r.y()), double(r.z()) };
        const double bb[3] = { double(b.x()), double(b.y()), double(b.z()) };
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                B[i][j] += wk * rr[i] * bb[j];
    }
    const double sigma = B[0][0] + B[1][1] + B[2][2];
    const double z0 = B[1][2] - B[2][1];
    const double z1 = B[2][0] - B[0][2];
    const double z2 = B[0][1] - B[1][0];

    double K[4][4];
    K[0][0] = sigma;
    K[0][1] = z0; K[0][2] = z1; K[0][3] = z2;
    K[1][0] = z0; K[2][0] = z1; K[3][0] = z2;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            K[i+1][j+1] = B[i][j] + B[j][i] - (i == j ? sigma : 0.0);

    double A[4][4], U[4][4];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            A[i][j] = K[i][j];
    jacobiSym4(A, U);

    int maxIdx = 0;
    for (int i = 1; i < 4; ++i)
        if (A[i][i] > A[maxIdx][maxIdx]) maxIdx = i;

    Quat q(U[0][maxIdx], U[1][maxIdx], U[2][maxIdx], U[3][maxIdx]);
    if (q.w < 0.0) q = Quat(-q.w, -q.x, -q.y, -q.z);
    return q.normalized();
}

static double wahbaResidualDeg(const Quat& q_s2b,
                               const QVector3D* vb, const QVector3D* vs, int n)
{
    auto angBetweenDeg = [](const QVector3D& a, const QVector3D& b) -> double {
        const double na = double(a.length()), nb = double(b.length());
        if (na < 1e-9 || nb < 1e-9) return 0.0;
        const double c = double(QVector3D::dotProduct(a, b)) / (na * nb);
        const double cc = c > 1.0 ? 1.0 : (c < -1.0 ? -1.0 : c);
        return std::acos(cc) * 180.0 / M_PI;
    };
    double worst = 0.0;
    for (int k = 0; k < n; ++k) {
        const QVector3D rotated = vec_rotate(vs[k].normalized(), q_s2b);
        const double e = angBetweenDeg(rotated, vb[k].normalized());
        if (e > worst) worst = e;
    }
    return worst;
}

struct SoftIronFit {
    double M[3][3];
    QVector3D offset;
    double residual;
    bool valid;
};

static SoftIronFit fitMagSoftIron(const QVector3D& sumMag,
                                  const double outer[6],
                                  int count)
{
    SoftIronFit out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out.M[i][j] = (i == j) ? 1.0 : 0.0;
    out.offset = QVector3D(0, 0, 0);
    out.residual = 0.0;
    out.valid = false;

    if (count < 200) return out;
    const double invN = 1.0 / double(count);
    const double mx = double(sumMag.x()) * invN;
    const double my = double(sumMag.y()) * invN;
    const double mz = double(sumMag.z()) * invN;

    double C[3][3];
    C[0][0] = outer[0] * invN - mx * mx;
    C[1][1] = outer[1] * invN - my * my;
    C[2][2] = outer[2] * invN - mz * mz;
    C[0][1] = C[1][0] = outer[3] * invN - mx * my;
    C[0][2] = C[2][0] = outer[4] * invN - mx * mz;
    C[1][2] = C[2][1] = outer[5] * invN - my * mz;

    const double trC = C[0][0] + C[1][1] + C[2][2];
    if (trC < 1e-9) return out;

    double E[3][3], V[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            E[i][j] = C[i][j];
    jacobiSym3(E, V);

    double lam[3] = { E[0][0], E[1][1], E[2][2] };
    const double lamMax = std::max({ lam[0], lam[1], lam[2] });
    const double lamMin = std::min({ lam[0], lam[1], lam[2] });
    if (lamMin < 1e-9 * lamMax) return out;
    if (lamMin <= 0.0) return out;

    const double lamGeo = std::cbrt(std::max(1e-30, lam[0] * lam[1] * lam[2]));
    double D[3];
    for (int i = 0; i < 3; ++i) D[i] = std::sqrt(lamGeo / lam[i]);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k)
                s += V[i][k] * D[k] * V[j][k];
            out.M[i][j] = s;
        }
    out.offset = QVector3D(float(mx), float(my), float(mz));
    out.residual = std::sqrt(lamMax / std::max(1e-30, lamMin));
    out.valid = true;
    return out;
}

// Gravity direction as expressed in each body-segment's own frame for the
// T-pose and N-pose reference.  Pre-computed once from SkeletonXsens's
// default_seg_angles tables — the body frame is defined by those angles.
// gravity_world = (0, 0, -1) in NWU; body = inv(default_seg_angles) · world.
static QVector3D gravityInBodyFrame(const Quat& defAng)
{
    return vec_rotate(QVector3D(0.0f, 0.0f, -1.0f), defAng.inv());
}

    void LocomotionSolver::reset()
    {
        m_haveLast     = false;
        m_initialised  = false;
        m_support      = RIGHT;
        m_anchor       = QVector3D(0, 0, 0);
        m_rAngV = m_lAngV = 0.0;
        m_rPlantTicks = m_lPlantTicks = m_rLiftTicks = m_lLiftTicks = 0;
        m_contact = {};

        m_offsetLast = QVector3D(0, 0, 0);
        m_offsetCommitted = QVector3D(0, 0, 0);
        m_offsetReady = false;

        // v3 state
        m_prevPelvisQ     = Quat(1, 0, 0, 0);
        m_pelvisAngV      = 0.0;
        m_pelvisYawAngV   = 0.0;
        m_yawFrozenPrev   = false;
        for (auto& v : m_rFKXY) v = QVector2D(0, 0);
        for (auto& v : m_lFKXY) v = QVector2D(0, 0);
        m_fkxyHead  = 0;
        m_fkxyCount = 0;

        m_confR = m_confL = 0.0;
        m_committedR = m_committedL = false;
        m_anchorR = m_anchorL = QVector3D(0, 0, 0);
        m_pose = PoseUnknown;
        m_poseTicks = 0;
        m_zuptTicks = 0;
        m_lowZTicksR = m_lowZTicksL = 0;
        m_zSnapBlendTicks = 0;
        m_confRFrozenForRoll = m_confLFrozenForRoll = false;
        m_confRFrozenValue = m_confLFrozenValue = 0.0;

        // FIX (heel/toe + airborne reset)
        m_footPitchZR = m_footPitchZL = 0.0;
        m_contactBlendR = m_contactBlendL = 0.0;
        m_heelLiftConfR = m_heelLiftConfL = 0.0;
        m_heelLiftR = m_heelLiftL = false;
        m_pelvisZVel = 0.0;
        m_pelvisZPrev = 0.0;
        m_havePelvisZPrev = false;
        m_airborneTicks = 0;
        m_landedTicks = 0;
    }

// Angular velocity (rad/s) between two unit quaternions at given dt.
static double quatAngVel(const Quat& a, const Quat& b, double dt)
{
    if (dt < 1e-6) return 0.0;
    double d = std::abs(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z);
    if (d > 1.0) d = 1.0;
    return 2.0 * std::acos(d) / dt;
}

QVector3D LocomotionSolver::update(const Quat& qR,
                                       const Quat& qL,
                                       const Quat& qPelvis,
                                       const QVector3D& fkRHeel,
                                       const QVector3D& fkRBall,
                                       const QVector3D& fkRTip,
                                       const QVector3D& fkLHeel,
                                       const QVector3D& fkLBall,
                                       const QVector3D& fkLTip,
                                       double t)
    {
        // Upper-clamp dt so a long pause/resume gap can't inject a huge step
        // that throws the foot-lock / velocity estimators (lower clamp avoids
        // div-by-zero).
        const double dt = m_haveLast ? std::clamp(t - m_lastT, 1e-3, 0.1) : 0.01;

        // One-shot dump of EVERY locomotion tunable (named members only — no
        // literal duplication) so a captured -test log is self-describing: the
        // thresholds/rates that drive commit/release, ZUPT, pose classification
        // and the airborne gates are recorded once, already rate-adjusted for
        // the active suit cadence (Link 240 / Awinda 60).
        if (m_verbose && !m_dbgConfigLogged) {
            m_dbgConfigLogged = true;
            std::ostringstream cs;
            cs << std::fixed << std::setprecision(3)
               << "\n========== [LOCO CONFIG] tunables @ " << m_procRateHz << " Hz ==========\n"
               << "  actorHeight=" << m_actorHeightM << "m footLen=" << m_footLengthM << "m\n"
               << "  stillRad=" << m_stillRad << " pelvisStillRad=" << m_pelvisStillRad
               << " heightMargin=" << m_heightMargin << " heightMarginSlow=" << m_heightMarginSlow
               << " switchMargin=" << m_switchMargin << " latchTicks=" << m_latchTicks << "\n"
               << "  fkxyStableRange=" << m_fkxyStableRange << "m fkxyWindow=" << m_fkxyWindow
               << " conf[commit=" << m_confCommit << " release=" << m_confRelease
               << " hystBand=" << m_confHystBand
               << " rise=" << m_confRiseRate << " fall=" << m_confFallRate << "]\n"
               << "  offsetRate[primary=" << m_offsetRatePrimary << " double=" << m_offsetRateDouble
               << "] zRate[moving=" << m_zRatePelvisMoving << " still=" << m_zRatePelvisStill
               << " drive=" << m_zDriveRate << "]\n"
               << "  pose[stableTicks=" << m_poseStableTicks << " zuptTicks=" << m_zuptTicksThresh
               << " lieTiltCos=" << m_lieTiltCosThresh << " squatKnee=" << m_squatKneeThresh
               << " sitKnee=" << m_sitKneeThresh << "]\n"
               << "  squat[lowZTicks=" << m_lowZTicksRequired << " lowZBand=" << m_lowZBandM
               << "m zSnapFrames=" << m_zSnapBlendFrames << "]\n"
               << "  roll[angVThresh=" << m_rollAngVThresh << " xyRangeMax=" << m_rollXYRangeMax << "m]"
               << " airborne[stableTicks=" << m_airborneStableTicks
               << " landedRamp=" << m_landedRampTicks << " commitFade=" << m_commitFadeTicks << "]\n"
               << "============================================================\n";
            std::cout << cs.str();
            std::cout.flush();
        }

        // Rate-adjusted first-order blend coefficients.  The inline EMAs below
        // were tuned at 90 Hz; rateAdjustAlpha re-expresses them for the actual
        // step dt so smoothing time-constants are preserved at 60 / 240 Hz.
        const double a030 = rateAdjustAlpha(0.30, dt);   // ang-vel / pitch / z-vel EMAs
        const double a020 = rateAdjustAlpha(0.20, dt);   // contact-blend EMA
        const double a010 = rateAdjustAlpha(0.10, dt);   // heel-lift confidence EMA

        // FIX (heel/toe contact discrimination): pick active contact point
        // based on foot pitch.  При pitch≈0 (плоская стопа) = lowest3 как
        // раньше.  pitch > +15° (heel-down toe-up) → fkHeel.  pitch < -15°
        // (heel-up toe-down) → fkBall (ball of foot = front part).
        //
        // m_footPitchZR/L пока что 0 на первом кадре — поэтому до тех пор
        // пока pitch не накопится через LP, используется lowest3 (zero weights).
        // На последующих кадрах pitch обновится сразу же.
        auto lowest3 = [](const QVector3D& a, const QVector3D& b, const QVector3D& c){
            const QVector3D& ab = a.z() < b.z() ? a : b;
            return ab.z() < c.z() ? ab : c;
        };
        auto sstep_local = [](double x){ x = std::clamp(x,0.0,1.0); return x*x*(3.0-2.0*x); };
        const QVector3D fkRLowest = lowest3(fkRHeel, fkRBall, fkRTip);
        const QVector3D fkLLowest = lowest3(fkLHeel, fkLBall, fkLTip);
        // smoothstep zone: |sin(pitch)| ∈ [0.17..0.34] = [≈10°..≈20°]
        const double heelContactWR_pre = sstep_local((m_footPitchZR - 0.17) / 0.17);
        const double toeContactWR_pre  = sstep_local((-m_footPitchZR - 0.17) / 0.17);
        const double heelContactWL_pre = sstep_local((m_footPitchZL - 0.17) / 0.17);
        const double toeContactWL_pre  = sstep_local((-m_footPitchZL - 0.17) / 0.17);
        const double neutralR = std::max(0.0, 1.0 - heelContactWR_pre - toeContactWR_pre);
        const double neutralL = std::max(0.0, 1.0 - heelContactWL_pre - toeContactWL_pre);
        const QVector3D fkR = float(neutralR) * fkRLowest
                            + float(heelContactWR_pre) * fkRHeel
                            + float(toeContactWR_pre)  * fkRBall;
        const QVector3D fkL = float(neutralL) * fkLLowest
                            + float(heelContactWL_pre) * fkLHeel
                            + float(toeContactWL_pre)  * fkLBall;

        // 1. Angular velocities of feet + pelvis (LP-smoothed).
        auto angVel = [](const Quat& a, const Quat& b, double dt_) {
            if (dt_ < 1e-6) return 0.0;
            double d = std::abs(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z);
            if (d > 1.0) d = 1.0;
            return 2.0 * std::acos(d) / dt_;
        };
        const double rawR = m_haveLast ? angVel(qR,      m_prevRQ,      dt) : 0.0;
        const double rawL = m_haveLast ? angVel(qL,      m_prevLQ,      dt) : 0.0;
        const double rawP = m_haveLast ? angVel(qPelvis, m_prevPelvisQ, dt) : 0.0;

        double rawYawRate = 0.0;
        if (m_haveLast) {
            const Quat qDP = quat_mult(qPelvis, m_prevPelvisQ.inv()).normalized();
            const double twN2 = qDP.w*qDP.w + qDP.z*qDP.z;
            if (twN2 > 1e-12) {
                const double twN = std::sqrt(twN2);
                const double tw_w = qDP.w / twN;
                const double tw_z = qDP.z / twN;
                const double sw   = std::min(1.0, std::abs(tw_w));
                const double sgn  = (tw_z >= 0.0) ? 1.0 : -1.0;
                const double yawDelta = 2.0 * sgn * std::acos(sw);
                rawYawRate = yawDelta / dt;
            }
        }

        const double kAlpha = a030;
        m_rAngV         = (1.0 - kAlpha) * m_rAngV         + kAlpha * rawR;
        m_lAngV         = (1.0 - kAlpha) * m_lAngV         + kAlpha * rawL;
        m_pelvisAngV    = (1.0 - kAlpha) * m_pelvisAngV    + kAlpha * rawP;
        m_pelvisYawAngV = (1.0 - kAlpha) * m_pelvisYawAngV + kAlpha * std::abs(rawYawRate);
        m_prevRQ = qR; m_prevLQ = qL; m_prevPelvisQ = qPelvis;
        m_lastT = t; m_haveLast = true;
        m_contact.rightAngV = m_rAngV;
        m_contact.leftAngV  = m_lAngV;

        // 3. FK-XY ring buffers + stability check.
        m_rFKXY[m_fkxyHead] = QVector2D(fkR.x(), fkR.y());
        m_lFKXY[m_fkxyHead] = QVector2D(fkL.x(), fkL.y());
        m_fkxyHead  = (m_fkxyHead + 1) % m_fkxyWindow;
        m_fkxyCount = std::min(m_fkxyCount + 1, m_fkxyWindow);
        auto xyRange = [&](const std::array<QVector2D, kFKXYWindowMax>& buf) -> float {
            if (m_fkxyCount < 3) return 0.0f;
            float xmin = std::numeric_limits<float>::infinity();
            float xmax = -xmin, ymin = xmin, ymax = xmax;
            for (int i = 0; i < m_fkxyCount; ++i) {
                float x = buf[i].x(), y = buf[i].y();
                if (x < xmin) xmin = x; if (x > xmax) xmax = x;
                if (y < ymin) ymin = y; if (y > ymax) ymax = y;
            }
            const float dx = xmax - xmin, dy = ymax - ymin;
            return std::sqrt(dx*dx + dy*dy);
        };
        // Threshold raised from 0.10 rad/s (~5.7°/s) to 0.50 rad/s (~28°/s).
        // Walking forward produces 5-10°/s yaw oscillation from heel-strike;
        // the old threshold suspended foot commits during normal walking,
        // which is why the skeleton "walked in place" — anchors never
        // updated.  0.5 rad/s reserves yawFreeze for actual deliberate
        // turning.
        // FIX (terminator smoothing): smoothstep zone [0.35..0.65] rad/s
        // вместо жёсткого 0.50, чтобы при ходьбе с heel-strike-yaw-oscillation
        // ~0.4 rad/s не было flicker'а в rFKXYStable.  Bool yawFreeze
        // оставляем для edge-handler ниже (ring buffer clear), который
        // должен срабатывать ровно один раз на переходе.
        auto smoothstep01 = [](double x) {
            x = std::clamp(x, 0.0, 1.0);
            return x * x * (3.0 - 2.0 * x);
        };
        const double yawFreezeW = smoothstep01((m_pelvisYawAngV - 0.35) / 0.30);
        const bool yawFreeze = (yawFreezeW > 0.5);

        if (yawFreeze && !m_yawFrozenPrev) {
            m_fkxyHead  = 0;
            m_fkxyCount = 0;
        }
        if (!yawFreeze && m_yawFrozenPrev && m_initialised) {
            // Re-base off the committed (travel-correct) offset so a turn
            // doesn't reset accumulated displacement back toward origin.
            if (m_committedR) {
                m_anchorR.setX(fkR.x() + m_offsetCommitted.x());
                m_anchorR.setY(fkR.y() + m_offsetCommitted.y());
            }
            if (m_committedL) {
                m_anchorL.setX(fkL.x() + m_offsetCommitted.x());
                m_anchorL.setY(fkL.y() + m_offsetCommitted.y());
            }
            m_fkxyHead  = 0;
            m_fkxyCount = 0;
        }

        // FIX (terminator smoothing): rFKXYStable теперь continuous weight.
        // Smoothstep zone [stableRange*0.5 .. stableRange*1.5] = [0.02..0.06].
        // При xy < 0.02 m → stableW=1; при xy > 0.06 → 0; в середине плавно.
        const float xyR = xyRange(m_rFKXY);
        const float xyL = xyRange(m_lFKXY);
        const double stableHi = m_fkxyStableRange * 1.5;
        const double stableDen = std::max(1e-6, m_fkxyStableRange);
        const double rFKXYStableW = std::max(yawFreezeW,
                smoothstep01((stableHi - double(xyR)) / stableDen));
        const double lFKXYStableW = std::max(yawFreezeW,
                smoothstep01((stableHi - double(xyL)) / stableDen));
        const bool rFKXYStable = (rFKXYStableW > 0.5);
        const bool lFKXYStable = (lFKXYStableW > 0.5);
        m_dbgFkxyRangeR = double(xyR);    m_dbgFkxyRangeL = double(xyL);
        m_dbgFkxyStableWR = rFKXYStableW; m_dbgFkxyStableWL = lFKXYStableW;
        m_dbgYawFreezeW = yawFreezeW;

        // 4. Classify pose.
        double tiltCos = 1.0;
        const PoseKind newPose = _classifyPose(qPelvis, fkR, fkL, tiltCos);
        m_lastTiltCos = tiltCos;
        if (newPose == m_pose)
            m_poseTicks = std::min(m_poseTicks + 1, 4096);
        else
            m_poseTicks = 0;
        m_pose = newPose;

        const float fkMinZ = std::min(fkR.z(), fkL.z());

        // FIX issue 9: low-Z счётчик нужен для мягкого блёнда при посадке.
        // PoseSquat-commit форсит world.z=0, но только если стопа реально
        // была низко несколько кадров подряд — иначе блёндим.
        m_lowZTicksR = (fkR.z() - fkMinZ < float(m_lowZBandM))
                       ? std::min(m_lowZTicksR + 1, 4096) : 0;
        m_lowZTicksL = (fkL.z() - fkMinZ < float(m_lowZBandM))
                       ? std::min(m_lowZTicksL + 1, 4096) : 0;

        // FIX (heel/toe contact discrimination): отслеживаем sin(pitch)
        // обеих стоп.  defAngFor(SEG_RFoot/LFoot)==identity → qR/qL уже
        // совпадает с world quat стопы; X = vec_rotate((1,0,0), q) =
        // (1-2(y²+z²), 2(xy+wz), 2(xz-wy)).  Z-компонента = 2*(qx*qz - qw*qy).
        //   z > 0  → ball выше heel (носок поднят, опора на пятке)
        //   z < 0  → ball ниже heel (пятка поднята, опора на мыске)
        const double pzR = 2.0 * (qR.x * qR.z - qR.w * qR.y);
        const double pzL = 2.0 * (qL.x * qL.z - qL.w * qL.y);
        m_footPitchZR = (1.0 - a030) * m_footPitchZR + a030 * pzR;
        m_footPitchZL = (1.0 - a030) * m_footPitchZL + a030 * pzL;

        // FIX (squat heel-lift): smoothstep confidence в полосе 21°..33°
        // (sin: 0.36..0.55).  Активируется только в Squat/Sit — в Stand
        // pitch такой величины встречается на ходу (heel strike) и не
        // должен переключать contact-point.  LP α=0.10 (333ms) — heel-lift
        // в приседе обычно держится секундами; быстрая фильтрация уберёт
        // ложные пики.
        auto sstep = [](double x){ x = std::clamp(x,0.0,1.0); return x*x*(3.0-2.0*x); };
        const bool poseAllowsHeelLift = (m_pose == PoseSquat || m_pose == PoseSit);
        const double hlR_raw = poseAllowsHeelLift
                ? sstep((-m_footPitchZR - 0.36) / 0.19) : 0.0;
        const double hlL_raw = poseAllowsHeelLift
                ? sstep((-m_footPitchZL - 0.36) / 0.19) : 0.0;
        m_heelLiftConfR = (1.0 - a010) * m_heelLiftConfR + a010 * hlR_raw;
        m_heelLiftConfL = (1.0 - a010) * m_heelLiftConfL + a010 * hlL_raw;
        m_heelLiftR = (m_heelLiftConfR > 0.5);
        m_heelLiftL = (m_heelLiftConfL > 0.5);

        // FIX issue 10 + terminator smoothing: rolling-foot detector.
        // Стопа быстро вращается (|angV| ~2 rad/s) при почти стоячем
        // FK-XY (range ~3cm) = перекат мыска-на-мысок.
        // Smoothstep angV [1.5..2.5] rad/s, range [stableRange/3 .. stableRange*2/3]
        // (т.е. [0.013..0.027] вокруг 0.03), bool rollingR=W>0.5 для downstream
        // hysteresis (replaces hard cliff at exactly 2.0 rad/s).
        const float rangeR = xyR;   // already computed above for FK-XY-stable
        const float rangeL = xyL;
        const double angFastR = smoothstep01((m_rAngV - (m_rollAngVThresh - 0.5)) / 1.0);
        const double angFastL = smoothstep01((m_lAngV - (m_rollAngVThresh - 0.5)) / 1.0);
        const double xyHi    = m_rollXYRangeMax * 1.33;     // ~0.04
        const double xyLo    = m_rollXYRangeMax * 0.67;     // ~0.02
        const double xyDen   = std::max(1e-6, xyHi - xyLo);
        const double xyTightR = smoothstep01((xyHi - double(rangeR)) / xyDen);
        const double xyTightL = smoothstep01((xyHi - double(rangeL)) / xyDen);
        const double rollingWR = angFastR * xyTightR;
        const double rollingWL = angFastL * xyTightL;
        const bool rollingR = (rollingWR > 0.5);
        const bool rollingL = (rollingWL > 0.5);
        m_dbgRollingWR = rollingWR;  m_dbgRollingWL = rollingWL;

        // 5. Initialisation on first frame.
        if (!m_initialised) {
            // Put pelvis at the pose-expected world height; anchor feet at
            // fk + offset (floor-snapped for stand/squat).
            const double pelvisZ_loco_init = 0.55 * m_actorHeightM;
            float targetZ = -fkMinZ;
            if (m_pose == PoseLying) targetZ = float(-pelvisZ_loco_init);
            else if (m_pose == PoseSit) targetZ = float(0.28 * m_actorHeightM - pelvisZ_loco_init);
            m_offsetLast = QVector3D(0.0f, 0.0f, targetZ);
            m_anchorR = QVector3D(fkR.x() + m_offsetLast.x(),
                                  fkR.y() + m_offsetLast.y(),
                                  fkR.z() + m_offsetLast.z());
            m_anchorL = QVector3D(fkL.x() + m_offsetLast.x(),
                                  fkL.y() + m_offsetLast.y(),
                                  fkL.z() + m_offsetLast.z());
            if (m_pose == PoseStand || m_pose == PoseSquat) {
                m_anchorR.setZ(0.0f);
                m_anchorL.setZ(0.0f);
            }
            m_confR = m_confL = 1.0;
            m_committedR = m_committedL = true;
            m_offsetReady = true;
            m_initialised = true;
            m_anchor  = m_anchorR;          // legacy field for UI
            m_support = (fkR.z() <= fkL.z()) ? RIGHT : LEFT;
            return m_offsetLast;
        }

        // 6. Per-foot confidence (smoothed).  Signals:
        //    s_still = (STILL_RAD - angv) / STILL_RAD clamped [0..1]
        //    s_xy    = fkxy_stable ? 1 : 0
        //    s_low   = (HEIGHT_MARGIN_SLOW - (fk.z - fkMinZ)) / ... clamped
        //    conf_raw = max(s_still, s_xy) * s_low
        auto sStill = [&](double angv) -> double {
            double s = (m_stillRad - angv) / m_stillRad;
            return std::max(0.0, std::min(1.0, s));
        };
        auto sLow = [&](const QVector3D& fk) -> double {
            double dz = fk.z() - fkMinZ;
            double s = 1.0 - dz / std::max(m_heightMarginSlow, 1e-3);
            return std::max(0.0, std::min(1.0, s));
        };
        // FIX (terminator smoothing): use continuous rFKXYStableW (0..1)
        // вместо bool — плавный переход в погранзоне stable/non-stable.
        const double rawCR = std::max(sStill(m_rAngV), rFKXYStableW) * sLow(fkR);
        const double rawCL = std::max(sStill(m_lAngV), lFKXYStableW) * sLow(fkL);
        m_dbgRawCR = rawCR;  m_dbgRawCL = rawCL;
        auto smooth = [](double prev, double raw, double rise, double fall) {
            const double r = (raw > prev) ? rise : fall;
            return (1.0 - r) * prev + r * raw;
        };
        const double newConfR = smooth(m_confR, rawCR, m_confRiseRate, m_confFallRate);
        const double newConfL = smooth(m_confL, rawCL, m_confRiseRate, m_confFallRate);

        // FIX issue 10: гистерезис.  Если стопа сейчас в режиме roll AND
        // новая conf падает в полосу вокруг commit/release порогов —
        // удерживаем conf на «замороженном» значении, чтобы не дрейфовать
        // через порог.  Когда rolling кончился — расшифровываемся.
        auto applyRollHyst = [&](bool rolling, double oldConf, double newConf,
                                 bool& frozen, double& frozenVal) -> double {
            const double bandLo = m_confRelease - m_confHystBand;
            const double bandHi = m_confCommit  + m_confHystBand;
            if (rolling && newConf >= bandLo && newConf <= bandHi) {
                if (!frozen) { frozen = true; frozenVal = oldConf; }
                return frozenVal;
            }
            frozen = false;
            return newConf;
        };
        m_confR = applyRollHyst(rollingR, m_confR, newConfR,
                                m_confRFrozenForRoll, m_confRFrozenValue);
        m_confL = applyRollHyst(rollingL, m_confL, newConfL,
                                m_confLFrozenForRoll, m_confLFrozenValue);

        const double pelvisRotKill = std::max(0.0, std::min(1.0, (m_pelvisAngV - 0.6) / 0.8));
        const bool pelvisRotating = pelvisRotKill > 0.5;
        m_dbgPelvisRotKill = pelvisRotKill;

        // FIX (airborne phase): отдельная PoseAirborne фаза для прыжков.
        // Оценка vertical velocity таза по изменению m_offsetLast.z за dt.
        // Активация двумя путями:
        //   (a) ballistic: feetLifted (confR/L<0.10) + zVel > 0.50 m/s
        //   (b) drift:     feetLifted + ни одна нога не committed +
        //                  (airborneTicks>0 || zVel>0.15) — fallback для
        //                  низкоскоростных подскоков.
        // Стабилизация >= 5 кадров (55ms @ 90Hz) → m_pose = PoseAirborne.
        // Выход → m_landedTicks = 12 (133ms ramp в landing re-anchor).
        {
            // bestPelvisEstimate() ещё не определён в lambda, но мы можем
            // повторить ту же логику: pelvisZ from anchor ноги, иначе offsetLast.
            double zNow;
            if (m_committedR && !m_committedL) zNow = m_anchorR.z() - fkR.z();
            else if (m_committedL && !m_committedR) zNow = m_anchorL.z() - fkL.z();
            else zNow = m_offsetLast.z();
            if (m_havePelvisZPrev) {
                const double rawVZ = (zNow - m_pelvisZPrev) / dt;
                m_pelvisZVel = (1.0 - a030) * m_pelvisZVel + a030 * rawVZ;
            }
            m_pelvisZPrev = zNow;
            m_havePelvisZPrev = true;

            const bool feetLifted = (m_confR < 0.10) && (m_confL < 0.10);
            const bool ballistic  = feetLifted && (m_pelvisZVel > 0.50);
            const bool driftAir   = feetLifted && (!m_committedR) && (!m_committedL)
                                  && (m_airborneTicks > 0 || m_pelvisZVel > 0.15);
            m_dbgFeetLifted = feetLifted;  m_dbgBallistic = ballistic;
            m_dbgDriftAir   = driftAir;

            if (ballistic || driftAir) {
                m_airborneTicks = std::min(m_airborneTicks + 1, 4096);
                if (m_airborneTicks >= m_airborneStableTicks && m_pose != PoseLying) {
                    // Override: PoseAirborne для последующих блоков.
                    // _classifyPose уже отработал, m_pose установлен; меняем.
                    if (m_pose != PoseAirborne) {
                        m_pose = PoseAirborne;
                        m_poseTicks = 0;
                    } else {
                        m_poseTicks = std::min(m_poseTicks + 1, 4096);
                    }
                }
            } else {
                if (m_airborneTicks > 0) {
                    // Только что приземлились — взводим re-anchor ramp.
                    m_landedTicks = m_landedRampTicks;
                }
                m_airborneTicks = 0;
            }
            if (m_landedTicks > 0) --m_landedTicks;
        }

        // FIX «walks in place»: при commit нового анкора используем
        // НАИБОЛЕЕ АКТУАЛЬНУЮ оценку pelvis_world, а не отстающий
        // m_offsetLast.  Если другая нога committed — её anchor минус её
        // FK даёт мгновенную мировую позицию таза.  m_offsetLast же
        // обновляется через LP-фильтр и отстаёт от истины на ~100 мс,
        // что при шаге 0.5 м означает 5-10 cm ошибку якоря и
        // последующие "прыжки" + накопление дрифта.
        auto bestPelvisEstimate = [&]() -> QVector3D {
            // Ищем committed-ногу с другой стороны и считаем по ней.
            if (m_committedR && !m_committedL)
                return QVector3D(m_anchorR.x() - fkR.x(),
                                 m_anchorR.y() - fkR.y(),
                                 m_offsetLast.z());
            if (m_committedL && !m_committedR)
                return QVector3D(m_anchorL.x() - fkL.x(),
                                 m_anchorL.y() - fkL.y(),
                                 m_offsetLast.z());
            // Обе или ни одной committed: при пере-anchor'е (новый plant)
            // m_offsetLast отстаёт и подтянут к origin → шаг "теряется".
            // Берём XY из m_offsetCommitted (offset на момент прошлого commit,
            // несёт накопленный travel), Z — из m_offsetLast (его ведёт
            // отдельная zSnap/drift-kill логика ниже).
            return QVector3D(m_offsetCommitted.x(),
                             m_offsetCommitted.y(),
                             m_offsetLast.z());
        };

        // FIX (heel/toe contact discrimination): при смене contact-point
        // на той же ноге, anchor "помнит" старую точку → offset = anchor-fk
        // делает скачок на длину стопы × sin(pitch) (~13см при 30°).
        // Решение: soft anchor shift в сторону актуального FK + offsetLast.
        // m_contactBlendR ∈ [-1..+1]: +1=heel, -1=toe.  Скачок > 0.05 в
        // contact-blend = реальная смена точки контакта.
        const double cbR_new = heelContactWR_pre - toeContactWR_pre;
        const double cbL_new = heelContactWL_pre - toeContactWL_pre;
        if (m_committedR && std::abs(cbR_new - m_contactBlendR) > 0.05) {
            const double alpha = std::min(1.0, 0.40 * std::abs(cbR_new - m_contactBlendR));
            const QVector3D pelvis = bestPelvisEstimate();
            const QVector3D newAnchorR(fkR.x() + pelvis.x(),
                                       fkR.y() + pelvis.y(),
                                       m_anchorR.z());   // Z держим (zSnap-logic ниже)
            m_anchorR = (1.0 - float(alpha)) * m_anchorR + float(alpha) * newAnchorR;
        }
        if (m_committedL && std::abs(cbL_new - m_contactBlendL) > 0.05) {
            const double alpha = std::min(1.0, 0.40 * std::abs(cbL_new - m_contactBlendL));
            const QVector3D pelvis = bestPelvisEstimate();
            const QVector3D newAnchorL(fkL.x() + pelvis.x(),
                                       fkL.y() + pelvis.y(),
                                       m_anchorL.z());
            m_anchorL = (1.0 - float(alpha)) * m_anchorL + float(alpha) * newAnchorL;
        }
        // LP-фильтр m_contactBlend (α=0.20 @90Hz, ~55ms response; rate-adjusted).
        m_contactBlendR = (1.0 - a020) * m_contactBlendR + a020 * cbR_new;
        m_contactBlendL = (1.0 - a020) * m_contactBlendL + a020 * cbL_new;

        // 7. Commit / release anchors with hysteresis.
        //    - rising edge (conf >= COMMIT && !committed): snap anchor
        //    - falling edge (conf < RELEASE): release (keep anchor memory)
        bool didCommitThisFrame = false;
        const bool wasCommittedR = m_committedR;
        const bool wasCommittedL = m_committedL;
        auto maybeCommitRelease = [&](double conf, bool& committed,
                                      QVector3D& anchor, const QVector3D& fk,
                                      bool isRight, bool rolling) {
            // FIX issue 10: rolling-foot — заморожен commit/release,
            // skeleton XY не скачет от перехода через порог.
            if (pelvisRotating || yawFreeze || rolling) return;
            if (!committed && conf >= m_confCommit) {
                // FIX: оценка таза от ДРУГОЙ committed-ноги (мгновенная),
                // а не от LP-сглаженного m_offsetLast.
                const QVector3D pelvis = bestPelvisEstimate();
                // Carry the accumulated travel forward: snapshot the (now
                // travel-correct, contralateral-anchored) pelvis offset so the
                // next swing phase re-anchors off real progress, not the
                // origin-biased m_offsetLast.  Fixes "walks in place".
                m_offsetCommitted = QVector3D(pelvis.x(), pelvis.y(),
                                              m_offsetCommitted.z());
                QVector3D world(fk.x() + pelvis.x(),
                                fk.y() + pelvis.y(),
                                fk.z() + pelvis.z());
                // FIX issue 9: PoseStand — жёсткий snap как раньше
                // (ходьба работает).  PoseSquat — только если стопа
                // была низко >= m_lowZTicksRequired кадров; иначе
                // оставляем естественный world.z и блёндим offsetZ
                // позже.  Без этого при переходе stand→sit таз
                // 'падает' за 10 cm до пола в момент commit.
                if (m_pose == PoseStand) {
                    world.setZ(0.0f);
                } else if (m_pose == PoseSquat) {
                    const int lowTicks = isRight ? m_lowZTicksR : m_lowZTicksL;
                    const bool heelLifted = isRight ? m_heelLiftR : m_heelLiftL;
                    if (lowTicks >= m_lowZTicksRequired) {
                        if (heelLifted) {
                            // FIX (squat heel-lift): при поднятой пятке anchor
                            // НЕ снапим к z=0 — реальный пол под носком, а fk
                            // (после A.2 это active-contact = ball) и так
                            // близко к полу.  Heel-keypoint висит выше пола
                            // на bone_foot * sin(|pitch|).  anchor.z должен
                            // отражать ЭТО: высота над полом на ту же величину.
                            const double bone = 0.60 * m_footLengthM;  // heel→ball
                            const double sinp = isRight
                                    ? std::abs(m_footPitchZR)
                                    : std::abs(m_footPitchZL);
                            // fk здесь — active contact point (ball-like), z
                            // близко к полу.  Чтобы commit fkZ ≈ 0 на полу
                            // → anchor.z = world.z уже выставлено выше (fk.z+pelvis.z).
                            // Snapшимся как в plain squat, но не до 0:
                            world.setZ(float(bone * sinp));
                        } else {
                            world.setZ(0.0f);
                        }
                        m_zSnapBlendTicks = m_zSnapBlendFrames;
                    }
                    // else: world.z = fk.z + pelvis.z (естественный)
                }
                // FIX (airborne): soft re-anchor при приземлении.
                // m_landedTicks отсчитывается от 12 (только что landed) к 0.
                // anchor = blend(currentEst, world, t) где t = 1 - landedTicks/12.
                // Без этого XY-anchor мгновенно прыгает на новое место →
                // skeleton "телепортируется" на pol 5-10cm после прыжка.
                if (m_landedTicks > 0) {
                    const double t = 1.0 - double(m_landedTicks) / double(m_landedRampTicks);
                    const QVector3D currentEst(fk.x() + m_offsetLast.x(),
                                               fk.y() + m_offsetLast.y(),
                                               fk.z() + m_offsetLast.z());
                    anchor = float(1.0 - t) * currentEst + float(t) * world;
                } else {
                    anchor = world;
                }
                committed = true;
                didCommitThisFrame = true;
            } else if (committed && conf < m_confRelease) {
                committed = false;   // anchor unchanged
            }
        };
        maybeCommitRelease(m_confR, m_committedR, m_anchorR, fkR, true,  rollingR);
        maybeCommitRelease(m_confL, m_committedL, m_anchorL, fkL, false, rollingL);
        if (didCommitThisFrame) m_recentCommitTicks = m_commitFadeTicks;
        else if (m_recentCommitTicks > 0) --m_recentCommitTicks;

        if (m_verbose) {
            if (!wasCommittedR && m_committedR) {
                std::cout << "[loco commit R] anchor=("
                          << std::fixed << std::setprecision(3)
                          << m_anchorR.x() << "," << m_anchorR.y() << "," << m_anchorR.z()
                          << ") conf=" << m_confR << "\n";
            }
            if (wasCommittedR && !m_committedR) {
                std::cout << "[loco release R] conf=" << std::fixed << std::setprecision(3)
                          << m_confR << "\n";
            }
            if (!wasCommittedL && m_committedL) {
                std::cout << "[loco commit L] anchor=("
                          << std::fixed << std::setprecision(3)
                          << m_anchorL.x() << "," << m_anchorL.y() << "," << m_anchorL.z()
                          << ") conf=" << m_confL << "\n";
            }
            if (wasCommittedL && !m_committedL) {
                std::cout << "[loco release L] conf=" << std::fixed << std::setprecision(3)
                          << m_confL << "\n";
            }
        }

        // 8. ZUPT: all still → full freeze.
        // FIX: ужесточили пороги — старые (0.25 rad/s = ~14°/s, и обычный
        // pelvisStillRad = 0.12 rad/s) триггерили ZUPT во время медленной
        // ходьбы и держали персонаж на месте.  Новые пороги:
        //   • feet ang vel < 0.15 rad/s (было 0.25) — реальная неподвижность
        //   • pelvis ang vel < m_pelvisStillRad — это ужесточено выше
        const bool allStill = (m_pelvisAngV < m_pelvisStillRad)
                           && (m_rAngV < 0.15) && (m_lAngV < 0.15);
        m_zuptTicks = allStill ? (m_zuptTicks + 1) : 0;
        if (m_zuptTicks >= m_zuptTicksThresh && m_offsetReady) {
            m_contact.rightDown = m_committedR;
            m_contact.leftDown  = m_committedL;
            return m_offsetLast;
        }

        // 10. Weighted dual-anchor raw offset.
        const double effR = m_committedR ? m_confR : 0.0;
        const double effL = m_committedL ? m_confL : 0.0;
        const double total = effR + effL;

        QVector3D rawOff = m_offsetLast;
        if (total > 1e-3) {
            QVector3D contribR = m_anchorR - fkR;
            QVector3D contribL = m_anchorL - fkL;
            rawOff = QVector3D(
                float((effR*contribR.x() + effL*contribL.x()) / total),
                float((effR*contribR.y() + effL*contribL.y()) / total),
                float((effR*contribR.z() + effL*contribL.z()) / total));
        }
        // else: no committed anchor → keep offset_last (freeze).

        // 11. Apply adaptive LP rates.  XY gets the imbalance-weighted rate;
        //     Z gets the pelvis-motion-dependent rate.
        const double imbalance = (total > 1e-3) ? std::abs(effR - effL) / total : 0.0;
        m_dbgImbalance = imbalance;  m_dbgEffR = effR;  m_dbgEffL = effL;
        const double xyRate = m_offsetRateDouble
                            + (m_offsetRatePrimary - m_offsetRateDouble) * imbalance;
        const double effXyRate = xyRate * (1.0 - pelvisRotKill);
        double zRate = (m_pelvisAngV > m_pelvisStillRad)
                              ? m_zRatePelvisMoving
                              : m_zRatePelvisStill;

        // FIX issue 9: пока активен soft Z blend (PoseSquat-commit запустил
        // его), boost-им zRate чтобы за m_zSnapBlendFrames кадров доехать
        // до целевого Z мягко вместо мгновенного snap.  Декрементируется
        // ниже после применения newOff.
        if (m_zSnapBlendTicks > 0) {
            const double blendRate = 1.0 / double(std::max(1, m_zSnapBlendTicks));
            zRate = std::max(zRate, blendRate);
        }

        QVector3D newOff = m_offsetLast;
        // FIX (airborne): когда PoseAirborne, заморозим offset (XY и Z).
        // Безопасное приближение: персонаж "висит" на месте, ожидая
        // landing.  Реальное movement в воздухе обычно < 0.5м, а без
        // anchor его нельзя оценить надёжно — лучше не двигать чем
        // ошибиться в направлении.
        if (m_pose != PoseAirborne && total > 1e-3) {
            if (!yawFreeze) {
                newOff.setX(float((1.0 - effXyRate) * m_offsetLast.x()
                                  + effXyRate * rawOff.x()));
                newOff.setY(float((1.0 - effXyRate) * m_offsetLast.y()
                                  + effXyRate * rawOff.y()));
            }
            newOff.setZ(float((1.0 - zRate) * m_offsetLast.z()
                              + zRate * rawOff.z()));
        }
        // else: PoseAirborne — newOff = m_offsetLast (полная заморозка).

        {
            // Per-frame offset cap.  FIX «walks in place»: старые
            // 0.10/0.18 при LP rate 0.18-0.30 всё равно ограничивали
            // catch-up до ~1-2 м/с, что НИЖЕ нормальной ходьбы (1-1.5
            // m/s) при подстраивании после commit нового якоря.
            // 0.20/0.35 даёт 18-32 m/s при 90 Hz — заведомо хватит на
            // быструю ходьбу + бег, при этом предохранитель остаётся
            // (защищает от выбросов при глюке датчика).
            // The caps are per-frame displacements; scaling by dt/dt0 (dt0=1/90)
            // keeps the underlying velocity limit constant across 60/90/240 Hz.
            const float stepScale = float(dt * 90.0);
            float maxStepXY = ((imbalance > 0.7) ? 0.35f : 0.20f) * stepScale;
            if (m_recentCommitTicks > 0) {
                const float fade = float(m_recentCommitTicks) / float(m_commitFadeTicks);
                const float minCap = 0.04f * stepScale;
                maxStepXY = minCap + (maxStepXY - minCap) * (1.0f - fade);
            }
            const float dx = newOff.x() - m_offsetLast.x();
            const float dy = newOff.y() - m_offsetLast.y();
            const float dl = std::sqrt(dx*dx + dy*dy);
            m_dbgMaxStepXY = double(maxStepXY);  m_dbgStepClampedXY = (dl > maxStepXY);
            if (dl > maxStepXY) {
                const float k = maxStepXY / dl;
                newOff.setX(m_offsetLast.x() + dx * k);
                newOff.setY(m_offsetLast.y() + dy * k);
            }
        }

        // 12. Pose-aware Z drift-kill (STAND pose only — the one pose
        //     where we can reliably predict pelvis world Z from actor
        //     height).  Very slow pull, within ±10 cm window.
        if (m_pose == PoseStand && m_poseTicks >= m_poseStableTicks) {
            const double targetZ = 0.55 * m_actorHeightM;
            if (std::abs(newOff.z() - targetZ) < 0.10) {
                newOff.setZ(float((1.0 - m_zDriveRate) * newOff.z()
                                  + m_zDriveRate * targetZ));
            }
        }

        if (m_verbose) {
            const float dx = newOff.x() - m_offsetLast.x();
            const float dy = newOff.y() - m_offsetLast.y();
            const float dz = newOff.z() - m_offsetLast.z();
            const float dxy = std::sqrt(dx*dx + dy*dy);
            if (dxy > 0.04f || std::abs(dz) > 0.04f) {
                std::cout << "[loco jump] dxy=" << std::fixed << std::setprecision(3) << dxy
                          << "m dz=" << dz << "m off=("
                          << newOff.x() << "," << newOff.y() << "," << newOff.z() << ")"
                          << " cR=" << m_confR << " cL=" << m_confL
                          << " commR=" << (m_committedR ? 1 : 0)
                          << " commL=" << (m_committedL ? 1 : 0)
                          << "\n";
            }
        }

        m_offsetLast = newOff;
        m_offsetReady = true;
        m_yawFrozenPrev = yawFreeze;
        if (m_zSnapBlendTicks > 0) --m_zSnapBlendTicks;

        // Legacy: for UI / debugging, expose which foot is currently dominant.
        m_contact.rightDown = m_committedR;
        m_contact.leftDown  = m_committedL;
        m_support = (effR >= effL) ? RIGHT : LEFT;
        m_anchor  = (m_support == RIGHT) ? m_anchorR : m_anchorL;

        return newOff;
    }

    LocomotionSolver::PoseKind
    LocomotionSolver::_classifyPose(const Quat& qPelvis,
                                    const QVector3D& fkR,
                                    const QVector3D& fkL,
                                    double& outTiltCos) const
    {
        // Pelvis body +Z direction → world.  If world-Z component < ~0.5,
        // pelvis is tilted ≥ 60° from vertical → lying.
        const double ux = 2.0 * (qPelvis.x*qPelvis.z + qPelvis.w*qPelvis.y);
        const double uy = 2.0 * (qPelvis.y*qPelvis.z - qPelvis.w*qPelvis.x);
        const double uz = 1.0 - 2.0 * (qPelvis.x*qPelvis.x + qPelvis.y*qPelvis.y);
        (void)ux; (void)uy;
        outTiltCos = uz;
        if (uz < m_lieTiltCosThresh) return PoseLying;
        const double pelvisZ_loco = 0.55 * m_actorHeightM;
        const double pelvisToFoot = pelvisZ_loco - double(std::min(fkR.z(), fkL.z()));
        if (pelvisToFoot < m_squatKneeThresh) return PoseSquat;
        if (pelvisToFoot < m_sitKneeThresh)   return PoseSit;
        return PoseStand;
    }


// ============================================================================
//  ConnStatus name
// ============================================================================

const char* connStatusName(ConnStatus s)
{
    switch (s) {
        case ConnStatus::NotInitialized: return "not initialised";
        case ConnStatus::NoDriver:       return "driver missing";
        case ConnStatus::Scanning:       return "scanning";
        case ConnStatus::NoDevice:       return "no device";
        case ConnStatus::Connecting:     return "connecting";
        case ConnStatus::Streaming:      return "streaming";
        case ConnStatus::Stale:          return "stale";
        case ConnStatus::Failed:         return "failed";
    }
    return "?";
}

static double monotonicSec()
{
    using clk = std::chrono::steady_clock;
    return std::chrono::duration<double>(clk::now().time_since_epoch()).count();
}

// ============================================================================
//  XDA direct connection via xsensdeviceapi64.dll  +  xstypes64.dll
//
//  Minimum viable port of XESNSE/Software/app/src/pose_sources.cpp focussed
//  on the read-only path: scan → open → enumerate → measure → poll
//  orientation quaternions → store them in SuitPose.
//
//  The XDA function signatures below are reverse-engineered from XESNSE's
//  function-pointer tables; they match the layout static_asserted there.
// ============================================================================

// --- Sensor location-ID → SkeletonXsens segment index (from XESNSE) -------
static int segmentFromLocationId(int loc)
{
    switch (loc) {
        case 1:  return SEG_Pelvis;
        case 5:  return SEG_T8;
        case 7:  return SEG_Head;
        case 8:  return SEG_RShoulder;
        case 9:  return SEG_RUpperArm;
        case 10: return SEG_RForearm;
        case 11: return SEG_RHand;
        case 12: return SEG_LShoulder;
        case 13: return SEG_LUpperArm;
        case 14: return SEG_LForearm;
        case 15: return SEG_LHand;
        case 16: return SEG_RUpperLeg;
        case 17: return SEG_RLowerLeg;
        case 18: return SEG_RFoot;
        case 20: return SEG_LUpperLeg;
        case 21: return SEG_LLowerLeg;
        case 22: return SEG_LFoot;
        default: return -1;           // non-tracker (dongle, master, ...)
    }
}

// --- XsArray is 5 pointer-size fields (<= 64 bytes on 64-bit) ------------
namespace xda {

using FnControlConstruct              = void*(*)();
using FnControlClose                  = void(*)(void*);
using FnControlDestruct               = void(*)(void*);
using FnControlOpenPort1              = int (*)(void*, void*, int, int);
using FnControlDeviceIds              = void(*)(void*, void*);
using FnControlDevice                 = void*(*)(void*, const XsDeviceIdBlob*);
using FnControlBroadcast              = void*(*)(void*);
using FnControlLastResultText         = void*(*)(void*, void*);
using FnControlLoadFilterProfiles     = int (*)(void*);
using FnControlSetOptions             = void(*)(void*, int, int);
using FnEnableNetworkScanning         = void(*)();       // xdaEnableNetworkScanning

using FnDeviceGotoConfig              = int(*)(void*);
using FnDeviceGotoMeasurement         = int(*)(void*);
using FnDeviceLocationId              = int(*)(void*);
using FnDeviceGetDataPacketCount      = int(*)(void*);
using FnDeviceTakeFirstDataPacketInQueue =
                                        XsDataPacketBlob*(*)(void*, XsDataPacketBlob*);
using FnDeviceLastAvailableLiveData   = XsDataPacketBlob*(*)(void*, XsDataPacketBlob*);
using FnDevicePacketErrorRate         = int(*)(void*);

using FnScanPorts                     = void(*)(void*, int, int, int, int);
using FnArrayAt                       = void*(*)(void*, std::size_t);
using FnArrayDestruct                 = void(*)(void*);
using FnArraySize                     = std::size_t (*)(const void*);
using FnPortInfoArrayConstruct        = void(*)(void*, std::size_t, void*);
using FnDeviceIdArrayConstruct        = void(*)(void*, std::size_t, void*);
using FnStringConstruct               = void(*)(void*);

using FnDataPacketConstruct               = void(*)(XsDataPacketBlob*);
using FnDataPacketDestruct                = void(*)(XsDataPacketBlob*);
using FnDataPacketContainsOrientation     = int (*)(const XsDataPacketBlob*);
using FnDataPacketCoordinateSystemOrient  = int (*)(const XsDataPacketBlob*);
using FnDataPacketOrientationQuaternion   =
      XsQuaternionBlob*(*)(const XsDataPacketBlob*, XsQuaternionBlob*, int);
using FnDataPacketContainsStoredLocationId= int (*)(const XsDataPacketBlob*);
using FnDataPacketStoredLocationId        = quint16(*)(const XsDataPacketBlob*);
using FnDataPacketContainsPacketCounter   = int (*)(const XsDataPacketBlob*);
using FnDataPacketPacketCounter           = quint16(*)(const XsDataPacketBlob*);
using FnDataPacketContainsOrientationIncrement =
                                            int (*)(const XsDataPacketBlob*);
using FnDataPacketOrientationIncrement    =
    XsQuaternionBlob*(*)(const XsDataPacketBlob*, XsQuaternionBlob*);
using FnDeviceUpdateRate                  = int (*)(void*);

// Calibrated IMU data — the inputs hipose's Madgwick filter wants.
using FnDataPacketContainsAcc    = int (*)(const XsDataPacketBlob*);
using FnDataPacketAcc            = XsVectorBlob*(*)(const XsDataPacketBlob*, XsVectorBlob*);
using FnDataPacketContainsGyro   = int (*)(const XsDataPacketBlob*);
using FnDataPacketGyro           = XsVectorBlob*(*)(const XsDataPacketBlob*, XsVectorBlob*);
using FnDataPacketContainsMag    = int (*)(const XsDataPacketBlob*);
using FnDataPacketMag            = XsVectorBlob*(*)(const XsDataPacketBlob*, XsVectorBlob*);
using FnVectorDestruct           = void(*)(XsVectorBlob*);

// SDI (Strap-down Integration) fallbacks — what Body Pack V2 actually
// sends instead of absolute acc/gyr (see XESNSE pose_sources.cpp line 1909).
using FnDataPacketContainsVelocityIncrement = int (*)(const XsDataPacketBlob*);
using FnDataPacketVelocityIncrement         =
    XsVectorBlob*(*)(const XsDataPacketBlob*, XsVectorBlob*);
using FnDataPacketContainsFreeAcc = int (*)(const XsDataPacketBlob*);
using FnDataPacketFreeAcc         =
    XsVectorBlob*(*)(const XsDataPacketBlob*, XsVectorBlob*);
using FnDataPacketContainsSampleTimeFine = int (*)(const XsDataPacketBlob*);
using FnDataPacketSampleTimeFine         = quint32(*)(const XsDataPacketBlob*);

struct Api {
    HMODULE xda = nullptr;
    HMODULE xst = nullptr;
    HMODULE iomp = nullptr;            // OpenMP runtime required by XDA

    // XsControl
    FnControlConstruct               controlConstruct = nullptr;
    FnControlClose                   controlClose     = nullptr;
    FnControlDestruct                controlDestruct  = nullptr;
    FnControlOpenPort1               controlOpenPort1 = nullptr;
    FnControlDeviceIds               controlDeviceIds = nullptr;
    FnControlDevice                  controlDevice    = nullptr;
    FnControlBroadcast               controlBroadcast = nullptr;
    FnControlLoadFilterProfiles      controlLoadFilterProfiles = nullptr;
    FnControlSetOptions              controlSetOptions         = nullptr;
    FnEnableNetworkScanning          enableNetworkScanning     = nullptr;

    // XsDevice
    FnDeviceGotoConfig               deviceGotoConfig       = nullptr;
    FnDeviceGotoMeasurement          deviceGotoMeasurement  = nullptr;
    FnDeviceLocationId               deviceLocationId       = nullptr;
    FnDeviceUpdateRate               deviceUpdateRate       = nullptr;
    FnDeviceGetDataPacketCount       deviceGetDataPacketCount = nullptr;
    FnDeviceTakeFirstDataPacketInQueue deviceTakeFirstDataPacketInQueue = nullptr;
    FnDeviceLastAvailableLiveData    deviceLastAvailableLiveData = nullptr;
    FnDevicePacketErrorRate          devicePacketErrorRate  = nullptr;

    // XsScanner + arrays (scanPorts/enumerateNetworkDevices in xsensdeviceapi64,
    // arrays in xstypes64 — see loadApi).
    FnScanPorts                      scanPorts              = nullptr;
    using FnEnumerateNetworkDevices  = void(*)(void*);     // fills XsArray*
    FnEnumerateNetworkDevices        enumerateNetworkDevices = nullptr;
    FnArrayAt                        arrayAt                = nullptr;
    FnArrayDestruct                  arrayDestruct          = nullptr;
    FnPortInfoArrayConstruct         portInfoArrayConstruct = nullptr;
    FnDeviceIdArrayConstruct         deviceIdArrayConstruct = nullptr;

    // XsDataPacket (xstypes64.dll)
    FnDataPacketConstruct                 dataPacketConstruct          = nullptr;
    FnDataPacketDestruct                  dataPacketDestruct           = nullptr;
    FnDataPacketContainsOrientation       dataPacketContainsOrientation= nullptr;
    FnDataPacketCoordinateSystemOrient    dataPacketCoordSysOrient     = nullptr;
    FnDataPacketOrientationQuaternion     dataPacketOrientationQuaternion = nullptr;
    FnDataPacketContainsStoredLocationId  dataPacketContainsStoredLocationId = nullptr;
    FnDataPacketStoredLocationId          dataPacketStoredLocationId   = nullptr;
    FnDataPacketContainsPacketCounter     dataPacketContainsPacketCounter = nullptr;
    FnDataPacketPacketCounter             dataPacketPacketCounter      = nullptr;
    FnDataPacketContainsOrientationIncrement
                                          dataPacketContainsOrientationIncrement = nullptr;
    FnDataPacketOrientationIncrement      dataPacketOrientationIncrement = nullptr;
    FnDataPacketContainsAcc               dataPacketContainsAcc  = nullptr;
    FnDataPacketAcc                       dataPacketAcc          = nullptr;
    FnDataPacketContainsGyro              dataPacketContainsGyro = nullptr;
    FnDataPacketGyro                      dataPacketGyro         = nullptr;
    FnDataPacketContainsMag               dataPacketContainsMag  = nullptr;
    FnDataPacketMag                       dataPacketMag          = nullptr;
    FnVectorDestruct                      vectorDestruct         = nullptr;
    FnDataPacketContainsVelocityIncrement dataPacketContainsVelInc = nullptr;
    FnDataPacketVelocityIncrement         dataPacketVelInc         = nullptr;
    FnDataPacketContainsFreeAcc           dataPacketContainsFreeAcc= nullptr;
    FnDataPacketFreeAcc                   dataPacketFreeAcc        = nullptr;
    FnDataPacketContainsSampleTimeFine    dataPacketContainsSTF    = nullptr;
    FnDataPacketSampleTimeFine            dataPacketSTF            = nullptr;

    // Diagnostic: generic contains-* probes. All have identical signature
    // int(*)(const XsDataPacketBlob*).  We resolve every possible channel
    // so we can dump a single-frame snapshot of "what Body Pack V2 streams".
    struct ContainsProbe {
        const char* name;
        FnDataPacketContainsAcc fn = nullptr;   // reusing same signature
    };
    std::array<ContainsProbe, 22> probes{{
        {"ContainsCalibratedAcceleration"},
        {"ContainsCalibratedGyroscopeData"},
        {"ContainsCalibratedMagneticField"},
        {"ContainsCalibratedData"},
        {"ContainsOrientation"},
        {"ContainsOrientationQuaternionStd"},
        {"ContainsOrientationEulerStd"},
        {"ContainsOrientationIncrement"},
        {"ContainsVelocityIncrement"},
        {"ContainsSdiData"},
        {"ContainsFreeAcceleration"},
        {"ContainsRawAcceleration"},
        {"ContainsRawGyroscopeData"},
        {"ContainsRawMagneticField"},
        {"ContainsAccelerationHR"},
        {"ContainsRateOfTurnHR"},
        {"ContainsPacketCounter"},
        {"ContainsSampleTimeFine"},
        {"ContainsStatus"},
        {"ContainsStoredLocationId"},
        {"ContainsTemperature"},
        {"ContainsPositionLLA"},
    }};
};

template<typename T>
static bool resolveProc(HMODULE mod, const char* name, T& out)
{
    FARPROC p = GetProcAddress(mod, name);
    if (!p) return false;
    out = reinterpret_cast<T>(p);
    return true;
}

// Locate the XDA DLL folder.  We look next to the exe (build/bin/), then in
// a sibling `dll/` folder, then on PATH (last-chance fallback).
static QString locateDllDir()
{
    const QString exeDir = QCoreApplication::applicationDirPath();
    for (const QString& sub : {QString(""), QString("/dll"), QString("/../dll")}) {
        const QString candidate = exeDir + sub;
        if (QFile::exists(candidate + "/xsensdeviceapi64.dll"))
            return QDir::cleanPath(candidate);
    }
    return exeDir;                                  // fall back (LoadLibrary uses PATH)
}

static bool loadApi(Api& api, QString& errDetail)
{
    const QString dllDir = locateDllDir();
    const QString iompPath = dllDir + "/libiomp5md.dll";
    const QString xstPath  = dllDir + "/xstypes64.dll";
    const QString xdaPath  = dllDir + "/xsensdeviceapi64.dll";

    // Windows won't look next to the DLL being loaded for its transitive
    // dependencies unless we say so — hence LOAD_WITH_ALTERED_SEARCH_PATH
    // plus SetDllDirectoryW.  Without this,  xsensdeviceapi64.dll fails
    // with ERROR_MOD_NOT_FOUND (126) even though xstypes64/libiomp5md are
    // right beside it.
    SetDllDirectoryW(reinterpret_cast<LPCWSTR>(dllDir.utf16()));
    constexpr DWORD kFlags = LOAD_WITH_ALTERED_SEARCH_PATH;

    api.iomp = LoadLibraryExW(reinterpret_cast<LPCWSTR>(iompPath.utf16()), nullptr, kFlags);
    const DWORD iompErr = api.iomp ? 0 : GetLastError();
    api.xst  = LoadLibraryExW(reinterpret_cast<LPCWSTR>(xstPath.utf16()),  nullptr, kFlags);
    const DWORD xstErr  = api.xst  ? 0 : GetLastError();
    api.xda  = LoadLibraryExW(reinterpret_cast<LPCWSTR>(xdaPath.utf16()),  nullptr, kFlags);
    const DWORD xdaErr  = api.xda  ? 0 : GetLastError();

    if (!api.xda) {
        errDetail = QString("LoadLibraryEx '%1' failed (err=%2);  "
                            "iomp err=%3, xstypes err=%4")
                        .arg(xdaPath).arg(xdaErr).arg(iompErr).arg(xstErr);
        return false;
    }
    if (!api.xst) {
        errDetail = QString("LoadLibraryEx '%1' failed (err=%2)")
                        .arg(xstPath).arg(xstErr);
        return false;
    }

    bool ok = true;
    ok &= resolveProc(api.xda, "XsControl_construct",               api.controlConstruct);
    ok &= resolveProc(api.xda, "XsControl_close",                   api.controlClose);
    resolveProc (api.xda, "XsControl_destruct",                     api.controlDestruct);
    ok &= resolveProc(api.xda, "XsControl_openPort_1",              api.controlOpenPort1);
    ok &= resolveProc(api.xda, "XsControl_deviceIds",               api.controlDeviceIds);
    ok &= resolveProc(api.xda, "XsControl_device",                  api.controlDevice);
    resolveProc (api.xda, "XsControl_broadcast",                    api.controlBroadcast);
    // Control init helpers — XESNSE calls these right after construct and
    // they are what turns on USB-NCM network scanning for Body Pack V2.
    resolveProc (api.xda, "XsControl_loadFilterProfiles",           api.controlLoadFilterProfiles);
    resolveProc (api.xda, "XsControl_setOptions",                   api.controlSetOptions);
    resolveProc (api.xda, "xdaEnableNetworkScanning",               api.enableNetworkScanning);

    ok &= resolveProc(api.xda, "XsDevice_gotoConfig",               api.deviceGotoConfig);
    ok &= resolveProc(api.xda, "XsDevice_gotoMeasurement",          api.deviceGotoMeasurement);
    ok &= resolveProc(api.xda, "XsDevice_locationId",               api.deviceLocationId);
    resolveProc (api.xda, "XsDevice_updateRate",                    api.deviceUpdateRate);
    ok &= resolveProc(api.xda, "XsDevice_getDataPacketCount",       api.deviceGetDataPacketCount);
    ok &= resolveProc(api.xda, "XsDevice_takeFirstDataPacketInQueue",
                      api.deviceTakeFirstDataPacketInQueue);
    resolveProc (api.xda, "XsDevice_lastAvailableLiveData",         api.deviceLastAvailableLiveData);
    resolveProc (api.xda, "XsDevice_packetErrorRate",               api.devicePacketErrorRate);

    ok &= resolveProc(api.xda, "XsScanner_scanPorts",               api.scanPorts);
    resolveProc (api.xda, "XsScanner_enumerateNetworkDevices",      api.enumerateNetworkDevices);
    // Array helpers live in xstypes64.dll.
    ok &= resolveProc(api.xst, "XsArray_at",                        api.arrayAt);
    ok &= resolveProc(api.xst, "XsArray_destruct",                  api.arrayDestruct);
    ok &= resolveProc(api.xst, "XsPortInfoArray_construct",         api.portInfoArrayConstruct);
    ok &= resolveProc(api.xst, "XsDeviceIdArray_construct",         api.deviceIdArrayConstruct);

    ok &= resolveProc(api.xst, "XsDataPacket_construct",            api.dataPacketConstruct);
    ok &= resolveProc(api.xst, "XsDataPacket_destruct",             api.dataPacketDestruct);
    ok &= resolveProc(api.xst, "XsDataPacket_containsOrientation",  api.dataPacketContainsOrientation);
    ok &= resolveProc(api.xst, "XsDataPacket_coordinateSystemOrientation",
                      api.dataPacketCoordSysOrient);
    ok &= resolveProc(api.xst, "XsDataPacket_orientationQuaternion",
                      api.dataPacketOrientationQuaternion);
    resolveProc (api.xst, "XsDataPacket_containsStoredLocationId",
                 api.dataPacketContainsStoredLocationId);
    resolveProc (api.xst, "XsDataPacket_storedLocationId",          api.dataPacketStoredLocationId);
    resolveProc (api.xst, "XsDataPacket_containsPacketCounter",     api.dataPacketContainsPacketCounter);
    resolveProc (api.xst, "XsDataPacket_packetCounter",             api.dataPacketPacketCounter);

    // Calibrated IMU channels (exact names taken from xstypes64 exports).
    resolveProc (api.xst, "XsDataPacket_containsCalibratedAcceleration",
                 api.dataPacketContainsAcc);
    resolveProc (api.xst, "XsDataPacket_calibratedAcceleration",
                 api.dataPacketAcc);
    resolveProc (api.xst, "XsDataPacket_containsCalibratedGyroscopeData",
                 api.dataPacketContainsGyro);
    resolveProc (api.xst, "XsDataPacket_calibratedGyroscopeData",
                 api.dataPacketGyro);
    resolveProc (api.xst, "XsDataPacket_containsCalibratedMagneticField",
                 api.dataPacketContainsMag);
    resolveProc (api.xst, "XsDataPacket_calibratedMagneticField",
                 api.dataPacketMag);
    resolveProc (api.xst, "XsVector_destruct",                      api.vectorDestruct);

    // SDI-mode extras used by Body Pack V2.
    resolveProc (api.xst, "XsDataPacket_containsVelocityIncrement", api.dataPacketContainsVelInc);
    resolveProc (api.xst, "XsDataPacket_velocityIncrement",         api.dataPacketVelInc);
    resolveProc (api.xst, "XsDataPacket_containsFreeAcceleration",  api.dataPacketContainsFreeAcc);
    resolveProc (api.xst, "XsDataPacket_freeAcceleration",          api.dataPacketFreeAcc);
    resolveProc (api.xst, "XsDataPacket_containsSampleTimeFine",    api.dataPacketContainsSTF);
    resolveProc (api.xst, "XsDataPacket_sampleTimeFine",            api.dataPacketSTF);

    // Orientation increment (SDI delta-q) — we need dataPacketOrientationIncrement
    // too, not just the "contains".
    resolveProc (api.xst, "XsDataPacket_containsOrientationIncrement",
                 api.dataPacketContainsOrientationIncrement);
    resolveProc (api.xst, "XsDataPacket_orientationIncrement",
                 api.dataPacketOrientationIncrement);

    // Resolve every probe once — each "XsDataPacket_contains*" name.
    for (auto& p : api.probes) {
        std::string sym = std::string("XsDataPacket_") + p.name;
        // Convert first letter to lowercase to match actual export casing.
        if (!sym.empty() && sym.length() > 13) sym[13] = char(std::tolower(sym[13]));
        resolveProc(api.xst, sym.c_str(), p.fn);
    }

    if (!ok) {
        errDetail = "missing required XDA exports";
        return false;
    }
    return true;
}

static void unloadApi(Api& api)
{
    if (api.xda) { FreeLibrary(api.xda); api.xda = nullptr; }
    if (api.xst) { FreeLibrary(api.xst); api.xst = nullptr; }
    if (api.iomp){ FreeLibrary(api.iomp); api.iomp = nullptr; }
}

} // namespace xda

// --- Impl ----------------------------------------------------------------

struct MocapReceiver::Impl {
    bool             test;
    std::atomic<bool> stop{false};
    mutable QMutex   lock;
    SuitPose         frame;

    std::atomic<int> status{(int)ConnStatus::NotInitialized};
    QString          statusDetail;
    std::atomic<int> activeTrackers{0};

    double           lastDump   = 0.0;
    double           lastPacket = 0.0;     // monotonic, for stale detection

    // Manus SDK handle.  `manusDllLoaded` just means the DLL is present next
    // to the exe.  `manusCoreReady` means CoreSdk_Initialize actually
    // connected to a running ManusCore.  `manusGloveCount` is the live
    // count reported by CoreSdk_GetNumberOfAvailableGloves — only positive
    // when at least one glove is powered on AND paired to that Core.
    HMODULE          manusModule    = nullptr;
    bool             manusDllLoaded = false;
    bool             manusCoreReady = false;
    int              manusGloveCount = 0;

    // Per-segment AHRS state — xio Fusion.  Body Pack V2 never ships
    // absolute quaternions (only SDI Δq/Δv + mag), so we fuse per-sensor
    // acc/gyr/mag into proper NWU quaternions exactly like hipose's
    // InertialPoseFusionFilter — but using the newer, more robust xio AHRS
    // (gyro-bias correction, accel/magnetic rejection, startup ramp).
    std::array<FusionAhrs, kXsensSegmentCount> fusion{};
    std::array<bool,       kXsensSegmentCount> fusionReady{};
    std::array<FusionBias,           kXsensSegmentCount> bias{};
    std::array<bool,                 kXsensSegmentCount> biasReady{};
    std::array<FusionAhrsSettings,   kXsensSegmentCount> ahrsCfg{};
    double           freqHz       = 240.0;   // queried from XsDevice_updateRate

    // Sensor-to-segment alignment — identity by default, overwritten when
    // the wizard finalises calibration.  Applied as inv(s2s[i]) to acc /
    // gyr / mag before they enter the fusion filter.
    std::array<Quat, kXsensSegmentCount> s2s{};
    std::array<Quat, kXsensSegmentCount> s2sInv{};
    bool                                 s2sActive = false;
    // Per-sensor magnetometer scaling (hipose mag_magn).  1.0 = do nothing.
    std::array<double, kXsensSegmentCount> magMagn{};
    bool                                   magNormActive = false;
    // Per-sensor accelerometer scaling (hipose acc_magn).  1.0 = off.
    std::array<double, kXsensSegmentCount> accMagn{};
    bool                                   accNormActive = false;
    // Per-sensor gyroscope DC bias (hipose gyr_bias), in deg/s.  All zero = off.
    std::array<QVector3D, kXsensSegmentCount> gyrBias{};
    bool                                      gyrBiasActive = false;

    // -test diagnostics: the IMU channels AFTER the full calibration chain
    // (acc-norm, gyr-bias, mag soft-iron) and the sensor->body s2s rotation,
    // plus the gyro after FusionBias removal.  Captured by the fusion loop so
    // the [FUSED SNAPSHOT] can show raw -> body-frame -> filter-input per axis.
    std::array<QVector3D, kXsensSegmentCount> dbgAccBody{};   // g, body frame
    std::array<QVector3D, kXsensSegmentCount> dbgGyrBody{};   // deg/s, body frame
    std::array<QVector3D, kXsensSegmentCount> dbgMagBody{};   // norm, body frame
    std::array<QVector3D, kXsensSegmentCount> dbgGyrFused{};  // deg/s, post-bias
    // -test per-stage capture so [FUSED SNAPSHOT] can print every transform of
    // every axis from arrival to filter input: raw SDI -> reconstructed ->
    // acc-norm -> gyr-bias -> mag soft-iron -> s2s.  Receiver-thread only.
    std::array<QVector3D, kXsensSegmentCount> dbgVelInc{};    // raw Δv (SDI)
    std::array<QVector3D, kXsensSegmentCount> dbgDqXyz{};     // raw Δq.xyz (SDI)
    std::array<QVector3D, kXsensSegmentCount> dbgAccPre{};    // g, post-SDI, pre-cal
    std::array<QVector3D, kXsensSegmentCount> dbgGyrPre{};    // deg/s, post-SDI, pre-cal
    std::array<QVector3D, kXsensSegmentCount> dbgMagPre{};    // raw mag, pre soft-iron
    std::array<QVector3D, kXsensSegmentCount> dbgAccNorm{};   // g, post acc-norm
    std::array<QVector3D, kXsensSegmentCount> dbgGyrUnbias{}; // deg/s, post gyr-bias
    std::array<QVector3D, kXsensSegmentCount> dbgMagSoft{};   // post soft-iron/norm
    std::array<Quat,      kXsensSegmentCount> dbgFusedQuat{}; // fusion output (world)
    // Per-segment dynamic AHRS rejection (the knob that widens accel/mag
    // trust during fast motion — the prime suspect when a limb's orientation
    // flips mid-jump because gravity is momentarily corrupted by linear accel).
    std::array<float,     kXsensSegmentCount> dbgDynAccRej{}; // deg, accel rejection this frame
    std::array<float,     kXsensSegmentCount> dbgDynMagRej{}; // deg, mag rejection this frame
    std::array<float,     kXsensSegmentCount> dbgAccErr{};    // ||acc|-1g| gravity-estimate error
    std::array<quint8,    kXsensSegmentCount> dbgChainFlags{};// bit0 haveMag bit1 SDI bit2 absAccGyr

    std::array<std::array<double, 9>, kXsensSegmentCount> magSoftMat{};
    std::array<QVector3D, kXsensSegmentCount>             magSoftOff{};
    bool                                                  magSoftActive = false;

    std::array<float, kXsensSegmentCount> segGain{};
    bool                                  segGainActive = false;

    // Bumped (atomically) by the calibration setters whenever s2s / gyr-bias /
    // segment-gain change, to ask the network thread to re-initialise every
    // fusion / bias filter.  Replaces the old cross-thread writes to
    // fusionReady/biasReady (which were reset under the lock here but read &
    // written lock-free in the poll loop — a data race).  fusion/bias/ahrsCfg/
    // fusionReady/biasReady are now strictly owned by the network thread.
    std::atomic<uint32_t> calGen{0};

    // Connection transport preference: COM = scanPorts first, Network =
    // enumerateNetworkDevices first (skip serial scan for faster WiFi boot).
    std::atomic<int> transport{0};              // 0 = ComPort, 1 = Network
    std::atomic<double> expectedRateHz{240.0};  // suit-implied rate (Link 240 / Awinda 60)

    explicit Impl(bool t) : test(t) {}

    void setStatus(ConnStatus s, const QString& detail,
                   MocapReceiver* parent)
    {
        {
            QMutexLocker lk(&lock);
            statusDetail = detail;
        }
        status.store((int)s);
        emit parent->statusChanged((int)s, detail);
        testLog(QString("[xda] status=%1  %2")
                    .arg(connStatusName(s))
                    .arg(detail.isEmpty() ? "" : "— " + detail)
                    .toStdString(), test);
    }
};

MocapReceiver::MocapReceiver(bool testMode, QObject* parent)
    : QThread(parent), m_impl(std::make_unique<Impl>(testMode))
{
}

MocapReceiver::~MocapReceiver()
{
    stop();
    if (isRunning()) wait(2500);
}

void MocapReceiver::stop() { m_impl->stop.store(true); }

void MocapReceiver::setTransport(Transport t)
{
    m_impl->transport.store(static_cast<int>(t));
}

void MocapReceiver::setExpectedRate(double hz)
{
    if (hz > 1.0) m_impl->expectedRateHz.store(hz);
}

void MocapReceiver::restart()
{
    // Tear down any in-flight scan / measurement loop.  XDA's scanPorts /
    // openPort calls can block inside the DLL for a full second at a time,
    // so we give the worker a generous grace period before assuming it is
    // wedged.  Starting a second QThread on top of a still-running one is
    // undefined behaviour and was the main crash path when the user clicked
    // "Connect suit" twice in a row.
    m_impl->stop.store(true);
    if (isRunning()) {
        if (!wait(8000)) {
            testLog("[xda] restart: previous worker did not exit in 8 s — "
                    "refusing to spawn a new one", m_impl->test);
            m_impl->setStatus(ConnStatus::Failed,
                              "previous scan still running — try again in a few seconds",
                              this);
            return;
        }
    }
    m_impl->stop.store(false);
    m_impl->status.store((int)ConnStatus::NotInitialized);
    m_impl->activeTrackers.store(0);
    { QMutexLocker lk(&m_impl->lock); m_impl->statusDetail.clear(); m_impl->frame = SuitPose{}; }
    start();
}

// SEH-guarded call helper — ManusSDK export signatures vary between SDK
// versions, so a wrong guess could corrupt the stack and take the whole app
// down.  Wrapping each call in __try/__except converts any access violation
// or stack mismatch into a clean false return.
template <typename Fn>
static bool sehCall(Fn fn)
{
    __try { fn(); return true; }
    __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}

// ---------------------------------------------------------------------------
//  ManusSDK ergonomics stream — shared state so the SDK-level callback
//  (which runs on ManusCore's thread) can publish glove count + per-glove
//  IDs into a place the Qt thread can inspect safely.
// ---------------------------------------------------------------------------
namespace {
struct ManusErgoSnapshot {
    std::atomic<std::uint64_t> lastTimeMs { 0 };
    std::atomic<std::uint32_t> lastCount  { 0 };
    std::atomic<std::uint32_t> lastLeft   { 0 };   // userId for left
    std::atomic<std::uint32_t> lastRight  { 0 };

    // Parsed finger pose — hand-local frame (wrist at origin, +X along
    // the forearm, +Y up, +Z out-of-palm).  Positions in metres.
    std::atomic<bool> haveLeft  { false };
    std::atomic<bool> haveRight { false };
    QMutex              lock;                       // guards the arrays
    std::array<Quat,      kFingerSegmentsHand> leftQ{};
    std::array<Quat,      kFingerSegmentsHand> rightQ{};
    std::array<QVector3D, kFingerSegmentsHand> leftP{};
    std::array<QVector3D, kFingerSegmentsHand> rightP{};

    // Test-mode raw-dump controls.  Set to true from MocapReceiver when
    // the CLI passes -test so the callback emits full ergonomics-stream
    // contents to the log for reverse-engineering.
    std::atomic<bool>          rawDump    { false };
    std::atomic<std::uint64_t> rawTicks   { 0 };

    std::array<float, 20> emaLeft  {};
    std::array<float, 20> emaRight {};
    bool emaLeftInit  = false;
    bool emaRightInit = false;
};
static ManusErgoSnapshot g_ergo;

// FIX (gloves polish): per-actor finger baseline захваченный в T-pose
// калибровке.  parseErgoHand вычитает baseline[i] из raw[i] (signed
// для spread, flex обрезается клампами в kFingerLimits).  Включается
// после MainWindow ctor; если baselineValid=false — старое поведение.
struct FingerBaselineState {
    QMutex lock;
    std::array<float, 20> left  {};
    std::array<float, 20> right {};
    std::atomic<bool> valid { false };
};
static FingerBaselineState g_fingerBaseline;

// Per-finger bone lengths in metres — approximate adult-male anatomy.
// kFingerBoneLen[finger][joint].  joint 0 = MCP/CMC, joint 3 = tip.
static const double kFingerBoneLen[5][4] = {
    { 0.045, 0.030, 0.025, 0.020 },   // thumb
    { 0.045, 0.040, 0.025, 0.020 },   // index
    { 0.045, 0.045, 0.028, 0.022 },   // middle
    { 0.045, 0.040, 0.027, 0.022 },   // ring
    { 0.045, 0.033, 0.021, 0.019 },   // pinky
};
// MCP knuckle offsets from wrist, hand-LOCAL frame.  The same table is
// used for BOTH hands — mirroring across the body is carried by the
// wrist's world quaternion applied in MainWindow::onRenderTick, so the
// local frame is identical for right and left.
//   +X = finger forward (out from wrist toward finger tip)
//   +Y = thumb side of the hand (radial direction)
//   +Z = dorsal (back of hand, away from palm)
static const QVector3D kFingerBaseOffset[5] = {
    QVector3D(0.035f,  0.030f,  0.015f),   // thumb  — radial + forward, slightly dorsal
    QVector3D(0.080f,  0.020f,  0.000f),   // index
    QVector3D(0.083f,  0.005f,  0.000f),   // middle
    QVector3D(0.080f, -0.010f,  0.000f),   // ring
    QVector3D(0.075f, -0.025f,  0.000f),   // pinky
};
// Per-finger spread sign.  Positive spread in the ergonomics stream
// means abduction (fanning out from the middle finger).  We realise it
// as a rotation of the MCP joint around the dorsal axis (+Z):
//   thumb:  toward +Y  (radial)     → +1
//   index:  toward +Y               → +0.5
//   middle: hardly spreads          →  0.0
//   ring:   toward -Y               → -0.5
//   pinky:  toward -Y               → -1.0
static const double kSpreadSign[5] = { +1.0, +0.5, 0.0, -0.5, -1.0 };

const FingerJointLimit kFingerLimits[5][3] = {
    {
        // FIX (gloves polish): расширили thumb ROM.
        // CMC: spread shifted to ±0.40π/0.60π (было ±0.30π/0.50π) — больше
        //      opposition-к-ладони / hyper-radial.  flex от -π/5 до 0.65π
        //      (было -π/12 до 0.50π) — больше hyperextension и сгиба.
        // MCP: добавили small spread ±π/10 (было 0) — анатомически
        //      thumb MCP всё-таки имеет минимальную abduction.
        //      flex до 0.60π (было 0.55π).
        // IP : добавили легкий hyperextension -π/20 (было 0) — реалистично.
        { -M_PI * 0.40,  M_PI * 0.60,  -M_PI / 5.0,    M_PI * 0.65 },
        { -M_PI / 10.0,  M_PI / 10.0,  -M_PI / 24.0,   M_PI * 0.60 },
        {  0.0,          0.0,          -M_PI / 24.0,   M_PI * 0.40 }
    },
    {
        { -M_PI / 9.0,   M_PI / 9.0,   -M_PI / 12.0,  M_PI * 0.50 },
        {  0.0,          0.0,           0.0,          M_PI * 0.65 },
        {  0.0,          0.0,           0.0,          M_PI / 3.0  }
    },
    {
        { -M_PI / 18.0,  M_PI / 18.0,  -M_PI / 12.0,  M_PI * 0.50 },
        {  0.0,          0.0,           0.0,          M_PI * 0.65 },
        {  0.0,          0.0,           0.0,          M_PI / 3.0  }
    },
    {
        { -M_PI / 9.0,   M_PI / 9.0,   -M_PI / 12.0,  M_PI * 0.50 },
        {  0.0,          0.0,           0.0,          M_PI * 0.65 },
        {  0.0,          0.0,           0.0,          M_PI / 3.0  }
    },
    {
        { -M_PI / 8.0,   M_PI / 8.0,   -M_PI / 12.0,  M_PI * 0.50 },
        {  0.0,          0.0,           0.0,          M_PI * 0.65 },
        {  0.0,          0.0,           0.0,          M_PI / 3.0  }
    }
};

// Build a quaternion from a unit axis and an angle in radians.
static Quat axisAngleQuat(const QVector3D& axis, double rad)
{
    const double h = rad * 0.5;
    const double s = std::sin(h);
    return Quat(std::cos(h),
                axis.x() * s, axis.y() * s, axis.z() * s).normalized();
}

// Estimate the toe (MTP / ball-of-foot joint) orientation from the foot.  There
// is no toe IMU sensor, so the toe is derived biomechanically: during toe-off /
// push-off the metatarsophalangeal joint extends so the toes stay near the
// ground while the foot pitches forward over the ball — mirroring OpenSim's
// gait2392 MTP joint and the Unreal plugin's separate ball/toe bone.  When the
// foot is flat or heel-down the toe stays collinear with the foot (ext == 0),
// so standing and heel-strike are identical to the previous foot==toe copy
// (zero regression for the locomotion solver and pelvis height).
//
// footQuat is the foot's world-frame orientation (the foot segment's defAng is
// identity, so the live q[SEG_*Foot] is already that world quat).  contactConf
// in [0,1] attenuates the bend so a foot in mid-swing gets no spurious flex.
static Quat estimateToeOrientation(const Quat& footQuat, double contactConf = 1.0)
{
    // Foot forward-pitch as sin(pitch): world Z of the foot's local +X (bone)
    // axis.  Same formula the locomotion solver uses (see m_footPitchZ).
    //   pz >= 0 → ball at/above heel (flat / heel-down) → toe stays flat
    //   pz <  0 → ball below heel    (heel-up / push-off) → toe extends
    const double pz = 2.0 * (footQuat.x * footQuat.z - footQuat.w * footQuat.y);
    if (pz >= 0.0)
        return footQuat;

    // Extension that keeps the toe roughly parallel to the ground ≈ how far the
    // foot front has dipped below horizontal, clamped to the anatomical MTP
    // range and gated by ground-contact confidence.
    constexpr double kMaxExtRad = 60.0 * M_PI / 180.0;
    double ext = std::asin(std::clamp(-pz, 0.0, 1.0));
    ext = std::min(ext, kMaxExtRad) * std::clamp(contactConf, 0.0, 1.0);
    if (ext <= 1e-4)
        return footQuat;

    // Hinge about the foot's local medio-lateral (+Y) axis so the toe moves in
    // the sagittal plane (tip up/down), like a real ball-of-foot joint.
    return quat_mult(footQuat, axisAngleQuat(QVector3D(0.0f, 1.0f, 0.0f), -ext));
}

// Per-hand finger diagnostic capture for the -test FINGER SNAPSHOT.  Holds
// the REAL angles parseErgoHand worked with — incoming ergo degrees, the
// baseline-subtracted "effective" values, the per-joint spread/flex before
// and after the anatomical kFingerLimits clamp, and which joints were
// actually clamped.  Single source: written by parseErgoHand itself.
struct FingerDiagHand {
    bool   valid           = false;
    bool   baselineApplied = false;
    float  raw[20]         = {};   // incoming ergo degrees (pre-baseline)
    float  effective[20]   = {};   // after baseline subtraction (degrees)
    double spreadEffDeg[5] = {};   // spread*sign, degrees, pre-clamp
    double flexDeg[5][3]   = {};   // MCP/PIP/DIP flex, degrees, pre-clamp
    double spreadClDeg[5]  = {};   // spread after clamp, degrees
    double flexClDeg[5][3] = {};   // flex after clamp, degrees
    bool   spreadClamped[5]  = {};
    bool   flexClamped[5][3] = {};
};
struct FingerDiag {
    QMutex lock;
    FingerDiagHand left, right;
};
static FingerDiag g_fingerDiag;

// Parse one hand's 20 ergonomics floats (spread/MCP/PIP/DIP × 5 fingers,
// in degrees) into (local per-joint quats, hand-local joint positions).
// Frame convention (same for BOTH hands):
//   bone forward = +X,  flex rotates around +Y (palmar curl positive),
//   spread rotates around +Z (dorsal axis).  The left-right mirroring
//   is supplied by the wrist world quaternion at render time.
static void parseErgoHand(const float* degs20, bool isLeft,
                          std::array<Quat,      kFingerSegmentsHand>& outQ,
                          std::array<QVector3D, kFingerSegmentsHand>& outP)
{
    FingerDiagHand dg;
    dg.valid = true;
    const QVector3D flexAxis (0, 1, 0);
    const QVector3D spreadAx (0, 0, 1);
    const double sideSign = isLeft ? -1.0 : +1.0;

    // FIX (gloves polish): finger baseline subtraction.  T-pose calibration
    // прокапывает avg degrees когда actor стоит ладонями вниз с прямыми
    // пальцами.  Эти значения и есть "neutral / zero".  Без вычитания
    // glove-specific bias (например MCP=15° по умолчанию) интерпретируется
    // как "actor с согнутыми пальцами" — отсюда визуально согнутые finger
    // даже когда они прямые.
    float effective[20];
    const float* effectivePtr = degs20;
    if (g_fingerBaseline.valid.load()) {
        QMutexLocker lkBL(&g_fingerBaseline.lock);
        const auto& baseline = isLeft ? g_fingerBaseline.left
                                      : g_fingerBaseline.right;
        for (int i = 0; i < 20; ++i) effective[i] = degs20[i] - baseline[i];
        effectivePtr = effective;
    }
    dg.baselineApplied = (effectivePtr == effective);
    for (int i = 0; i < 20; ++i) { dg.raw[i] = degs20[i]; dg.effective[i] = effectivePtr[i]; }

    // ============================================================
    // FIX: thumb-specific pre-rotation для CMC (Carpometacarpal).
    //
    // Большой палец анатомически устроен принципиально иначе чем
    // остальные пальцы: сустав CMC — седловидный, а не блоковидный,
    // и его оси сгибания/отведения наклонены к плоскости ладони
    // примерно на 45°.  В исходной версии все 5 пальцев получали
    // одинаковые оси (flexAxis=+Y, spreadAx=+Z) в Manus-local
    // системе координат, из-за чего:
    //   1. большой палец рос «прямо вперёд» вместо естественного
    //      положения «вперёд-радиально-чуть-дорсально»;
    //   2. сгибание поворачивало его строго в сторону палмарной
    //      плоскости (-Z), вместо натурального движения «к ладони
    //      и к мизинцу» (oppositio policis).
    //
    // Решение: pre-rotate локальной системы координат большого
    // пальца на R_z(+45°) · R_x(-30°) в Manus-local frame:
    //   • R_z(+45°) разворачивает «направление кости» от +X
    //     (вперёд) к (cos45°, sin45°, 0) — вперёд+радиально, что
    //     соответствует естественной оси первой фаланги большого
    //     пальца.
    //   • R_x(-30°) наклоняет ось сгибания (после R_z) так что
    //     дальнейший flex вокруг локального +Y физически означает
    //     движение конца к ладони И к мизинцу одновременно
    //     (классический жест «opposition» против указательного).
    //
    // Цепочка дальнейших суставов (q1=MCP, q2=IP) использует те же
    // (0,1,0) и (0,0,1), но эти оси теперь в УЖЕ-предвернутой
    // системе, поэтому MCP и IP сгибаются в правильной плоскости.
    //
    // Для LEFT-руки render-pipeline уже применяет yflipQuat
    // (см. строку ~7659), которое — в силу того что yflip
    // является гомоморфизмом (yflip(a*b)=yflip(a)·yflip(b)) —
    // автоматически зеркалит pre-rotation в нужную сторону:
    //   yflip(R_z(+45°)·R_x(-30°)) = R_z(-45°)·R_x(+30°)
    // что в body-frame левой руки даёт правильно отзеркалённое
    // положение большого пальца.  Дополнительные знаковые правки
    // не требуются.
    //
    // Параметры можно подкрутить под конкретного актёра (типичный
    // диапазон: радиальный 30–60°, opposition 15–35°).
    // ============================================================
    constexpr double kThumbCmcRadialDeg     = 40.0;
    constexpr double kThumbCmcOppositionDeg = 15.0;
    // FIX issue 3: для LEFT-руки инвертируем знаки радиальной и
    // оппозиционной осей.  Manus отдаёт обе руки в идентичной локальной
    // системе, и хотя render-pipeline применяет yflip к ПОЗИЦИЯМ
    // (mirrorManusL), thumbPreRot заходит в КВАТЕРНИОН outQ.  Для
    // правой руки знаки положительные → большой палец смотрит
    // forward-radial + opposition к ладони.  Для левой нужны
    // отрицательные знаки в обеих компонентах, чтобы анатомия
    // отзеркалилась симметрично.  Защитный флаг — на случай если
    // визуально потребуется откат.
    static constexpr bool kEnableLeftThumbVariant = true;
    const double radSign = (isLeft && kEnableLeftThumbVariant) ? -1.0 : +1.0;
    const double oppSign = (isLeft && kEnableLeftThumbVariant) ? -1.0 : +1.0;
    const Quat thumbPreRot = quat_mult(
        axisAngleQuat(QVector3D(0,0,1),
                      radSign * kThumbCmcRadialDeg     * M_PI / 180.0),
        axisAngleQuat(QVector3D(1,0,0),
                      oppSign * -kThumbCmcOppositionDeg * M_PI / 180.0)
    ).normalized();

    for (int f = 0; f < 5; ++f) {
        const float* d = effectivePtr + f * 4;
        const double spread = d[0] * M_PI / 180.0;
        const double a1     = d[1] * M_PI / 180.0;
        const double a2     = d[2] * M_PI / 180.0;
        const double a3     = d[3] * M_PI / 180.0;

        const auto& Lm = kFingerLimits[f];
        (void)sideSign;
        const double spreadEff = spread * kSpreadSign[f];
        const double spreadC = std::clamp(spreadEff, Lm[0].spreadMin, Lm[0].spreadMax);
        const double a1c     = std::clamp(a1, Lm[0].flexMin, Lm[0].flexMax);
        const double a2c     = std::clamp(a2, Lm[1].flexMin, Lm[1].flexMax);
        const double a3c     = std::clamp(a3, Lm[2].flexMin, Lm[2].flexMax);

        // Capture the real per-joint angles + clamp outcomes for the -test log.
        {
            const double Kdeg = 180.0 / M_PI;
            dg.spreadEffDeg[f]   = spreadEff * Kdeg;
            dg.flexDeg[f][0]     = a1 * Kdeg;
            dg.flexDeg[f][1]     = a2 * Kdeg;
            dg.flexDeg[f][2]     = a3 * Kdeg;
            dg.spreadClDeg[f]    = spreadC * Kdeg;
            dg.flexClDeg[f][0]   = a1c * Kdeg;
            dg.flexClDeg[f][1]   = a2c * Kdeg;
            dg.flexClDeg[f][2]   = a3c * Kdeg;
            dg.spreadClamped[f]  = (spreadC != spreadEff);
            dg.flexClamped[f][0] = (a1c != a1);
            dg.flexClamped[f][1] = (a2c != a2);
            dg.flexClamped[f][2] = (a3c != a3);
        }

        Quat q0 = quat_mult(axisAngleQuat(spreadAx, spreadC),
                            axisAngleQuat(flexAxis, a1c));
        Quat q1 = axisAngleQuat(flexAxis, a2c);
        Quat q2 = axisAngleQuat(flexAxis, a3c);

        // FIX: для большого пальца стартуем с предвернутой
        // локальной системой координат вместо identity.
        Quat worldQ = (f == 0) ? thumbPreRot : Quat(1, 0, 0, 0);
        QVector3D p = kFingerBaseOffset[f];
        const int baseIdx = f * 4;

        outQ[baseIdx + 0] = worldQ;
        outP[baseIdx + 0] = p;

        p = p + vec_rotate(QVector3D(float(kFingerBoneLen[f][0]), 0, 0), worldQ);
        worldQ = quat_mult(worldQ, q0);
        outQ[baseIdx + 1] = worldQ;
        outP[baseIdx + 1] = p;

        p = p + vec_rotate(QVector3D(float(kFingerBoneLen[f][1]), 0, 0), worldQ);
        worldQ = quat_mult(worldQ, q1);
        outQ[baseIdx + 2] = worldQ;
        outP[baseIdx + 2] = p;

        p = p + vec_rotate(QVector3D(float(kFingerBoneLen[f][2]), 0, 0), worldQ);
        worldQ = quat_mult(worldQ, q2);
        outQ[baseIdx + 3] = worldQ;
        outP[baseIdx + 3] = p;
    }
    {
        QMutexLocker lk(&g_fingerDiag.lock);
        (isLeft ? g_fingerDiag.left : g_fingerDiag.right) = dg;
    }
}

// v4: Mirror Manus-local finger frame for the LEFT hand.
//
// The Manus SDK exposes BOTH hands in an identical local frame:
//   +X = fingers forward,  +Y = thumb side,  +Z = dorsal.
//
// For the RIGHT hand in T-pose (Xsens body frame: +X → -Y_world,
// +Y → +X_world, +Z → +Z_world) Manus-local matches Xsens-hand-body
// frame with a pure rotation (det = +1).  For the LEFT hand in T-pose
// (Xsens body frame: +X → +Y_world, +Y → -X_world, +Z → +Z_world) the
// mapping from Manus-local to Xsens-hand-body frame is a REFLECTION
// (det = -1), which cannot be realised by any quaternion.  We fix it
// by flipping Manus-local +Y (thumb side) → -Y on the L-hand data
// before rotating into world.  Math check: L thumb base (0.035,0.030,
// 0.015) → mirrored (0.035,-0.030,0.015) → rotated by defLHand_T =
// Rot_Z(+π/2) → (+0.030,+0.035,+0.015) = forward, left, dorsal ✓.
static inline QVector3D mirrorManusL(const QVector3D& p)
{
    return QVector3D(p.x(), -p.y(), p.z());
}
}

// Callback signature matches CoreSdk_RegisterCallbackForErgonomicsStream:
// `void(const ErgonomicsStream*)`.  The struct layout is from XESNSE.
struct ManusErgoData {        // sizeof == 0xA8
    std::uint32_t id = 0;
    std::uint32_t isUserID = 0;
    float data[40]{};
};
// FFI ABI guard: foxManusErgonomicsCb reinterpret_casts the Manus SDK buffer
// to these layouts. If the SDK/compiler ever changes the size/padding, fail at
// compile time instead of reading garbage / overrunning at runtime.
static_assert(sizeof(ManusErgoData) == 0xA8,
              "ManusErgoData must match the Manus ergonomics ABI (4+4+40*4 bytes)");
struct ManusErgoStream {      // sizeof == 0x1510
    std::uint64_t publishTime = 0;
    ManusErgoData data[32]{};
    std::uint32_t dataCount   = 0;
};
static_assert(sizeof(ManusErgoStream) == 0x1510,
              "ManusErgoStream must match the Manus ergonomics ABI "
              "(8 + 32*0xA8 + 4, padded to 8-byte alignment)");

static void __cdecl foxManusErgonomicsCb(const void* raw)
{
    if (!raw) return;
    const auto* s = reinterpret_cast<const ManusErgoStream*>(raw);
    g_ergo.lastTimeMs.store(GetTickCount64());
    g_ergo.lastCount.store(s->dataCount);

    // ---- Test-mode raw dump ------------------------------------------
    // First 20 frames go out in full so the exact layout of stream->data
    // is recoverable from the log.  After that we emit one full dump per
    // second-ish (every 120th tick @ ≈60 Hz) to keep the log readable.
    if (g_ergo.rawDump.load()) {
        const std::uint64_t tick = g_ergo.rawTicks.fetch_add(1);
        const bool dumpFull = (tick < 20) || ((tick % 120) == 0);
        if (dumpFull) {
            std::ostringstream os;
            os << std::fixed << std::setprecision(3);
            os << "[ergo-raw] tick=" << tick
               << " publishTime=" << s->publishTime
               << " dataCount="   << s->dataCount << "\n";
            for (std::uint32_t i = 0; i < s->dataCount && i < 32; ++i) {
                const auto& e = s->data[i];
                os << "  [entry " << i << "] id=" << e.id
                   << " isUserID=" << e.isUserID
                   << "\n    data[ 0..19]:";
                for (int j = 0; j < 20; ++j) {
                    if (j % 5 == 0) os << "\n      ";
                    os << " " << std::setw(8) << e.data[j];
                }
                os << "\n    data[20..39]:";
                for (int j = 20; j < 40; ++j) {
                    if ((j - 20) % 5 == 0) os << "\n      ";
                    os << " " << std::setw(8) << e.data[j];
                }
                os << "\n";
            }
            std::cout << os.str();
            std::cout.flush();
        }
    }

    bool haveL = false, haveR = false;
    std::array<Quat,      kFingerSegmentsHand> lQ{}, rQ{};
    std::array<QVector3D, kFingerSegmentsHand> lP{}, rP{};
    const float* srcL = nullptr;        // points into s->data[?].data[0]
    const float* srcR = nullptr;        // points into s->data[?].data[20]

    // Each ergonomics entry carries 40 floats laid out as
    //   data[ 0..19]  — LEFT hand  (5 fingers × 4 joints in degrees)
    //   data[20..39]  — RIGHT hand
    // Per-entry probe: the half with non-zero activity belongs to that
    // glove; XESNSE uses the same convention.
    for (std::uint32_t i = 0; i < s->dataCount && i < 32; ++i) {
        const auto& e = s->data[i];
        bool anyL = false, anyR = false;
        for (int j = 0; j < 20; ++j) {
            if (std::abs(e.data[j])      > 0.01f) anyL = true;
            if (std::abs(e.data[20 + j]) > 0.01f) anyR = true;
        }
        const bool testMode = g_ergo.rawDump.load();
        auto alphaForJoint = [testMode](int idx, float delta, const char* hand) -> float {
            // FIX (gloves polish): thumb base α 0.20 → 0.15 — thumb sensor
            // на Manus традиционно шумнее остальных; больше LP-сглаживания
            // не даёт заметной latency (15ms vs 11ms @ 90Hz) но визуально
            // убирает джиттер большого пальца.
            const bool isThumb = (idx < 4);
            const float baseAlpha = isThumb ? 0.15f : 0.35f;
            const float outlierAlpha = isThumb ? 0.04f : 0.10f;
            const float outlierThresh = isThumb ? 15.0f : 30.0f;
            const bool outlier = (delta > outlierThresh);
            if (outlier && testMode) {
                static const char* kFingerName[5] = { "thumb", "index", "middle", "ring", "pinky" };
                static const char* kJointName[4]  = { "spread", "MCP", "PIP", "DIP" };
                const int fi = idx / 4, ji = idx % 4;
                std::cout << "[glove-outlier] " << hand << " " << kFingerName[fi]
                          << " " << kJointName[ji] << " delta=" << std::fixed << std::setprecision(1)
                          << delta << "° → α=" << outlierAlpha << "\n";
            }
            return outlier ? outlierAlpha : baseAlpha;
        };
        if (anyL) {
            float smoothed[20];
            {
                QMutexLocker lk(&g_ergo.lock);
                if (!g_ergo.emaLeftInit) {
                    for (int j = 0; j < 20; ++j)
                        g_ergo.emaLeft[j] = std::isfinite(e.data[j]) ? e.data[j] : 0.0f;
                    g_ergo.emaLeftInit = true;
                } else {
                    for (int j = 0; j < 20; ++j) {
                        const float prev = g_ergo.emaLeft[j];
                        // Reject a non-finite SDK sample — otherwise the EMA is
                        // poisoned to NaN permanently (NaN propagates every tick).
                        const float cur  = std::isfinite(e.data[j]) ? e.data[j] : prev;
                        const float delta = std::abs(cur - prev);
                        const float a = alphaForJoint(j, delta, "L");
                        g_ergo.emaLeft[j] = prev + a * (cur - prev);
                    }
                }
                for (int j = 0; j < 20; ++j) smoothed[j] = g_ergo.emaLeft[j];
            }
            parseErgoHand(smoothed, true, lQ, lP);
            srcL = &e.data[0]; haveL = true;
        }
        if (anyR) {
            float smoothed[20];
            {
                QMutexLocker lk(&g_ergo.lock);
                if (!g_ergo.emaRightInit) {
                    for (int j = 0; j < 20; ++j)
                        g_ergo.emaRight[j] = std::isfinite(e.data[20 + j]) ? e.data[20 + j] : 0.0f;
                    g_ergo.emaRightInit = true;
                } else {
                    for (int j = 0; j < 20; ++j) {
                        const float prev = g_ergo.emaRight[j];
                        // Reject a non-finite SDK sample (see left-hand note above).
                        const float cur  = std::isfinite(e.data[20 + j]) ? e.data[20 + j] : prev;
                        const float delta = std::abs(cur - prev);
                        const float a = alphaForJoint(j, delta, "R");
                        g_ergo.emaRight[j] = prev + a * (cur - prev);
                    }
                }
                for (int j = 0; j < 20; ++j) smoothed[j] = g_ergo.emaRight[j];
            }
            parseErgoHand(smoothed, false, rQ, rP);
            srcR = &e.data[20]; haveR = true;
        }
    }

    if (haveL || haveR) {
        QMutexLocker lk(&g_ergo.lock);
        if (haveL) { g_ergo.leftQ  = lQ;  g_ergo.leftP  = lP;  g_ergo.haveLeft.store(true); }
        if (haveR) { g_ergo.rightQ = rQ;  g_ergo.rightP = rP;  g_ergo.haveRight.store(true); }
    }

    // Parsed dump — comprehensive per-phalanx report every ~2 seconds.
    // Lists each finger's raw ergonomics angles (spread/MCP/PIP/DIP),
    // the local per-joint quaternion we assembled, the hand-local FK
    // position of every joint, and the skeleton segment the glove
    // attaches to (SEG_LHand / SEG_RHand).  Frequency unchanged.
    if (g_ergo.rawDump.load()) {
        const std::uint64_t tk = g_ergo.rawTicks.load();
        if ((tk % 120) == 0 && (haveL || haveR)) {
            static const char* const kFingerTag[5] =
                { "thumb", "index", "middle", "ring ", "pinky" };
            std::ostringstream os;
            os << std::fixed;
            auto dumpHand = [&](const char* tag, bool any, const float* src20,
                                const std::array<Quat, kFingerSegmentsHand>& Q,
                                const std::array<QVector3D, kFingerSegmentsHand>& P,
                                int skelSeg) {
                if (!any) return;
                os << "\n---------- [GLOVE " << tag << "] ----------\n";
                os << "  maps to skeleton segment SEG_"
                   << (skelSeg == SEG_LHand ? "LHand" : "RHand")
                   << " (idx " << skelSeg << "), base offset applied per finger:\n";
                for (int f = 0; f < 5; ++f) {
                    const float* d = src20 + f * 4;
                    os << "  "
                       << std::left << std::setw(6) << kFingerTag[f] << std::right
                       << " raw°  spread=" << std::setw(8) << std::setprecision(3) << d[0]
                       << "  MCP=" << std::setw(8) << d[1]
                       << "  PIP=" << std::setw(8) << d[2]
                       << "  DIP=" << std::setw(8) << d[3]
                       << "  base=(" << std::setprecision(3)
                       << std::setw(6) << kFingerBaseOffset[f].x() << ","
                       << std::setw(6) << kFingerBaseOffset[f].y() << ","
                       << std::setw(6) << kFingerBaseOffset[f].z() << ")"
                       << "  sprSign=" << std::setprecision(2) << kSpreadSign[f]
                       << "\n";
                    static const char* const kJointTag[4] =
                        { "J0-MCP", "J1-PIP", "J2-DIP", "J3-TIP" };
                    for (int j = 0; j < 4; ++j) {
                        const int idx = f * 4 + j;
                        const Quat& lq = Q[idx];
                        const QVector3D& lp = P[idx];
                        os << "        " << kJointTag[j]
                           << "  locQ=(" << std::setprecision(4)
                           << std::setw(7) << lq.w << ","
                           << std::setw(7) << lq.x << ","
                           << std::setw(7) << lq.y << ","
                           << std::setw(7) << lq.z << ")"
                           << "  |Q|=" << std::setw(6) << std::setprecision(2)
                           << quat_angle_deg(lq) << "°"
                           << "  pos=(" << std::setprecision(4)
                           << std::setw(7) << lp.x() << ","
                           << std::setw(7) << lp.y() << ","
                           << std::setw(7) << lp.z() << ")"
                           << "  boneL=" << std::setw(6) << std::setprecision(3)
                           << kFingerBoneLen[f][j] << "m\n";
                    }
                }
            };
            if (haveL && srcL) dumpHand("LEFT",  true, srcL, lQ, lP, SEG_LHand);
            if (haveR && srcR) dumpHand("RIGHT", true, srcR, rQ, rP, SEG_RHand);
            os << "-----------------------------------------\n";
            std::cout << os.str();
            std::cout.flush();
        }
    }
}

// ManusSDK ABI — copied verbatim from XESNSE/Software/app/src/pose_sources.cpp
// where these structs ship with static_assert()s against offsets / sizes
// verified against the production SDK headers.
namespace manus_abi {

struct VersionInfo {
    std::uint32_t major = 0;
    std::uint32_t minor = 0;
    std::uint32_t patch = 0;
    char label[16]{};
    char sha[16]{};
    char tag[16]{};
    char versionInfo[16]{};
};
struct Host {                         // sizeof == 0x174
    char hostName[256]{};
    char ipAddress[40]{};
    VersionInfo version{};
};
struct CoordinateSystemVUH {          // sizeof == 0x10
    std::int32_t view       = 0;
    std::int32_t up         = 0;
    std::int32_t handedness = 0;
    float        unitScale  = 1.0f;
};
static_assert(sizeof(Host) == 0x174, "ManusHost size mismatch");
static_assert(sizeof(CoordinateSystemVUH) == 0x10, "CoordinateSystemVUH size");

constexpr int kSessionTypeUnreal     = 2;
constexpr int kAxisViewYFromViewer   = 2;
constexpr int kAxisPolarityPositiveY = 5;
constexpr int kHandednessRight       = 2;

} // namespace manus_abi

bool MocapReceiver::connectGloves()
{
    auto& I = *m_impl;

    // Raw ergonomics-stream logging in -test mode so we can reverse the
    // exact wire layout per SDK build.  Idempotent — stays on until app
    // shutdown.
    if (I.test) {
        g_ergo.rawDump.store(true);
        g_ergo.rawTicks.store(0);
    }

    auto dllDir = [](){
        char buf[MAX_PATH]{};
        GetModuleFileNameA(nullptr, buf, MAX_PATH);
        std::string d(buf);
        const auto s = d.find_last_of("\\/");
        if (s != std::string::npos) d.resize(s);
        return d;
    }();

    // ---- 1. Load the SDK DLL (idempotent) ----------------------------------
    if (!I.manusDllLoaded) {
        const std::string sdk = dllDir + "\\ManusSDK.dll";
        HMODULE mod = LoadLibraryA(sdk.c_str());
        if (!mod) {
            testLog("[manus] LoadLibrary failed — ManusSDK.dll not usable",
                    I.test);
            I.manusCoreReady  = false;
            I.manusGloveCount = 0;
            emit gloveStatusChanged(false);
            return false;
        }
        I.manusModule    = mod;
        I.manusDllLoaded = true;
    }

    // ---- 2. Resolve exports (signatures taken from XESNSE) ---------------
    using namespace manus_abi;
    auto resolve = [&](const char* name) -> FARPROC {
        return GetProcAddress(I.manusModule, name);
    };

    using FnInit            = int (*)();
    using FnShutDown        = int (*)();
    using FnSetSessionType  = int (*)(int);
    using FnLookForHosts    = int (*)(unsigned int, bool);
    using FnGetNumHosts     = int (*)(std::uint32_t*);
    using FnGetHosts        = int (*)(Host*, std::uint32_t);
    using FnConnectToHost   = int (*)(Host);
    using FnGetConnected    = int (*)(bool*);
    using FnInitCoordVUH    = int (*)(CoordinateSystemVUH, bool);
    using FnRegCb           = int (*)(void(*)(const void*));
    using FnGetNumUsers     = int (*)(std::uint32_t*);
    using FnGetUserIds      = int (*)(std::uint32_t*, std::uint32_t);
    using FnGetGloveId      = int (*)(std::uint32_t, int, std::uint32_t*);
    using FnGetNumGloves    = int (*)(std::uint32_t*);

    auto init      = reinterpret_cast<FnInit>          (resolve("CoreSdk_InitializeCore"));
    auto shutDown  = reinterpret_cast<FnShutDown>      (resolve("CoreSdk_ShutDown"));
    auto setSess   = reinterpret_cast<FnSetSessionType>(resolve("CoreSdk_SetSessionType"));
    auto lookFor   = reinterpret_cast<FnLookForHosts>  (resolve("CoreSdk_LookForHosts"));
    auto getNumH   = reinterpret_cast<FnGetNumHosts>   (resolve("CoreSdk_GetNumberOfAvailableHostsFound"));
    auto getHosts  = reinterpret_cast<FnGetHosts>      (resolve("CoreSdk_GetAvailableHostsFound"));
    auto connectH  = reinterpret_cast<FnConnectToHost> (resolve("CoreSdk_ConnectToHost"));
    auto isConn    = reinterpret_cast<FnGetConnected>  (resolve("CoreSdk_GetIsConnectedToCore"));
    auto initCoord = reinterpret_cast<FnInitCoordVUH>  (resolve("CoreSdk_InitializeCoordinateSystemWithVUH"));
    auto regErgo   = reinterpret_cast<FnRegCb>         (resolve("CoreSdk_RegisterCallbackForErgonomicsStream"));
    auto regSkel   = reinterpret_cast<FnRegCb>         (resolve("CoreSdk_RegisterCallbackForSkeletonStream"));
    auto regSys    = reinterpret_cast<FnRegCb>         (resolve("CoreSdk_RegisterCallbackForSystemStream"));
    auto getNumGl  = reinterpret_cast<FnGetNumGloves>  (resolve("CoreSdk_GetNumberOfAvailableGloves"));
    auto getNumUsr = reinterpret_cast<FnGetNumUsers>   (resolve("CoreSdk_GetNumberOfAvailableUsers"));
    auto getUserIds= reinterpret_cast<FnGetUserIds>    (resolve("CoreSdk_GetIdsOfAvailableUsers"));
    auto getGloveId= reinterpret_cast<FnGetGloveId>    (resolve("CoreSdk_GetGloveIdOfUser_UsingUserId"));

    if (!init || !setSess || !lookFor || !getNumH || !getHosts || !connectH
        || !isConn || !initCoord || !regErgo) {
        testLog("[manus] ManusSDK.dll is present but the v2 handshake exports "
                "are missing — wrong DLL version?", I.test);
        I.manusCoreReady  = false;
        I.manusGloveCount = 0;
        emit gloveStatusChanged(false);
        return false;
    }

    // ---- 3. CoreSdk_InitializeCore() — NO ARGS ---------------------------
    // rc=0  → fresh init; rc=10 → already initialised (repeated click);
    // anything else is a hard failure.  Already-init is treated as
    // success so we can still query glove state on the existing session.
    int rc = -1;
    const bool okInit = sehCall([&]() { rc = init(); });
    testLog("[manus] CoreSdk_InitializeCore rc=" + std::to_string(rc), I.test);
    const bool alreadyInit = (rc == 10);
    if (!okInit || (rc != 0 && !alreadyInit)) {
        I.manusCoreReady  = false;
        I.manusGloveCount = 0;
        emit gloveStatusChanged(false);
        return false;
    }

    bool coreUp = false;
    if (alreadyInit) {
        // Previous session already installed callbacks + picked a host.
        // Just verify the connection is still live.
        if (isConn) {
            bool flag = false;
            sehCall([&]() { isConn(&flag); });
            coreUp = flag;
        }
        testLog(std::string("[manus] re-entry on existing session, connected=")
                + (coreUp ? "true" : "false"), I.test);
    } else {
        // ---- 4. Register the streams BEFORE SetSessionType (UE5 order) --
        auto noop = [](const void*) {};
        if (regSkel) { int r = 0; sehCall([&]() { r = regSkel(noop); });
                       testLog("[manus] RegSkeletonStream rc=" + std::to_string(r), I.test); }
        // Ergonomics = where glove presence is actually reported on this
        // SDK build.  Wire the real callback so we get dataCount ticks.
        { int r = 0; sehCall([&]() { r = regErgo(&foxManusErgonomicsCb); });
          testLog("[manus] RegErgonomicsStream rc=" + std::to_string(r), I.test); }
        if (regSys)  { int r = 0; sehCall([&]() { r = regSys(noop); });
                       testLog("[manus] RegSystemStream rc=" + std::to_string(r), I.test); }

        // ---- 5. SetSessionType(Unreal=2) -------------------------------
        int sRc = -1;
        sehCall([&]() { sRc = setSess(kSessionTypeUnreal); });
        testLog("[manus] CoreSdk_SetSessionType(2) rc=" + std::to_string(sRc), I.test);

        // ---- 6. Coordinate system — Y-up, right-handed, meters ---------
        {
            CoordinateSystemVUH cs{};
            cs.view       = kAxisViewYFromViewer;
            cs.up         = kAxisPolarityPositiveY;
            cs.handedness = kHandednessRight;
            cs.unitScale  = 1.0f;
            int cRc = -1;
            sehCall([&]() { cRc = initCoord(cs, false); });
            testLog("[manus] InitCoordSystemVUH rc=" + std::to_string(cRc), I.test);
        }

        // ---- 7. LookForHosts + ConnectToHost ---------------------------
        int lRc = -1;
        sehCall([&]() { lRc = lookFor(1, true); });
        testLog("[manus] CoreSdk_LookForHosts rc=" + std::to_string(lRc), I.test);

        std::uint32_t hostCount = 0;
        sehCall([&]() { getNumH(&hostCount); });
        testLog("[manus] hosts found = " + std::to_string(hostCount), I.test);

        if (hostCount > 0) {
            std::vector<Host> hosts(hostCount);
            int hRc = -1;
            sehCall([&]() { hRc = getHosts(hosts.data(), hostCount); });
            for (std::uint32_t i = 0; i < hostCount && !coreUp; ++i) {
                int cRc = -1;
                sehCall([&]() { cRc = connectH(hosts[i]); });
                testLog(std::string("[manus] ConnectToHost[")
                        + std::to_string(i) + "] rc=" + std::to_string(cRc),
                        I.test);
                if (cRc != 0) continue;
                for (int k = 0; k < 10; ++k) {
                    bool flag = false;
                    sehCall([&]() { isConn(&flag); });
                    if (flag) { coreUp = true; break; }
                    Sleep(100);
                }
            }
        }
    }

    if (!coreUp) {
        testLog("[manus] ManusCore handshake failed — is ManusCore running?",
                I.test);
        if (shutDown) sehCall([&]() { shutDown(); });
        I.manusCoreReady  = false;
        I.manusGloveCount = 0;
        emit gloveStatusChanged(false);
        return false;
    }
    I.manusCoreReady = true;

    // ---- 8. Poll for a powered-on, paired glove --------------------------
    // ManusCore does not populate GetNumberOfAvailableGloves reliably
    // (observed: returns 0 even with two gloves online and paired).  The
    // XESNSE way is to enumerate "users" and query each side's glove id.
    int gloveCount = 0;
    if (getNumUsr && getUserIds && getGloveId) {
        std::uint32_t userN = 0;
        sehCall([&]() { getNumUsr(&userN); });
        testLog("[manus] users seen = " + std::to_string(userN), I.test);
        if (userN > 0) {
            std::vector<std::uint32_t> ids(userN);
            sehCall([&]() { getUserIds(ids.data(), userN); });
            for (std::uint32_t u = 0; u < userN; ++u) {
                for (int side : { 0 /*left*/, 1 /*right*/ }) {
                    std::uint32_t gid = 0;
                    sehCall([&]() { getGloveId(ids[u], side, &gid); });
                    if (gid != 0) {
                        ++gloveCount;
                        testLog(std::string("[manus] user ")
                                + std::to_string(ids[u])
                                + " side=" + (side==0?"L":"R")
                                + " gloveId=" + std::to_string(gid),
                                I.test);
                    }
                }
            }
        }
    }
    // Fall back to the direct count API if the user table is empty — some
    // Core builds populate one but not the other.
    if (gloveCount == 0 && getNumGl) {
        std::uint32_t n = 0;
        sehCall([&]() { getNumGl(&n); });
        gloveCount = int(n);
    }
    // Last resort — wait for the ergonomics stream to deliver at least one
    // packet.  On this Core build that's actually the authoritative signal
    // the gloves are powered on and paired; the users / glove-id tables
    // stay empty until the ergonomics stream fires.
    if (gloveCount == 0) {
        const std::uint64_t t0 = GetTickCount64();
        while (GetTickCount64() - t0 < 2500) {
            const std::uint32_t c = g_ergo.lastCount.load();
            if (c > 0) { gloveCount = int(c); break; }
            Sleep(100);
        }
        testLog("[manus] ergonomics-stream dataCount = "
                + std::to_string(g_ergo.lastCount.load()), I.test);
    }
    I.manusGloveCount = gloveCount;

    testLog("[manus] CoreSdk connected, gloves visible = "
            + std::to_string(gloveCount), I.test);
    const bool glovesOn = (gloveCount > 0);
    emit gloveStatusChanged(glovesOn);
    return glovesOn;
}

bool MocapReceiver::glovesReady()     const { return m_impl->manusGloveCount > 0; }
bool MocapReceiver::glovesCoreReady() const { return m_impl->manusCoreReady; }
bool MocapReceiver::glovesDllLoaded() const { return m_impl->manusDllLoaded; }

void MocapReceiver::resetFusion()
{
    // Ask the network thread to re-init every fusion / bias filter on its next
    // packet (fusionReady/biasReady are owned by that thread; see Impl::calGen).
    m_impl->calGen.fetch_add(1, std::memory_order_relaxed);
    testLog("[fusion] reset — all 17 xio AHRS filters will re-init", m_impl->test);
}

void MocapReceiver::setS2sAlignment(const std::array<Quat, kXsensSegmentCount>& s2s)
{
    QMutexLocker lk(&m_impl->lock);
    m_impl->s2s    = s2s;
    for (int i = 0; i < kXsensSegmentCount; ++i)
        m_impl->s2sInv[i] = s2s[i].inv();
    m_impl->s2sActive = true;
    // Ask the network thread to re-init every fusion filter so the first few
    // samples after s2s goes live don't corrupt the existing steady state.
    m_impl->calGen.fetch_add(1, std::memory_order_relaxed);
    testLog("[s2s] sensor-to-segment alignment installed", m_impl->test);
}

Quat MocapReceiver::getS2s(int idx) const
{
    if (idx < 0 || idx >= kXsensSegmentCount) return Quat(1, 0, 0, 0);
    QMutexLocker lk(&m_impl->lock);
    return m_impl->s2s[idx];
}

void MocapReceiver::resetS2sAlignment()
{
    QMutexLocker lk(&m_impl->lock);
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        m_impl->s2s[i]     = Quat(1, 0, 0, 0);
        m_impl->s2sInv[i]  = Quat(1, 0, 0, 0);
        m_impl->magMagn[i] = 1.0;
        m_impl->accMagn[i] = 1.0;
        m_impl->gyrBias[i] = QVector3D(0, 0, 0);
        for (int k = 0; k < 9; ++k)
            m_impl->magSoftMat[i][k] = (k == 0 || k == 4 || k == 8) ? 1.0 : 0.0;
        m_impl->magSoftOff[i] = QVector3D(0, 0, 0);
    }
    m_impl->s2sActive     = false;
    m_impl->magNormActive = false;
    m_impl->accNormActive = false;
    m_impl->gyrBiasActive = false;
    m_impl->magSoftActive = false;
}

void MocapReceiver::setMagNormalisation(const std::array<double, kXsensSegmentCount>& mm)
{
    QMutexLocker lk(&m_impl->lock);
    m_impl->magMagn = mm;
    m_impl->magNormActive = true;
    m_impl->magSoftActive = false;
    testLog("[s2s] per-sensor mag_magn normalisation installed", m_impl->test);
}

void MocapReceiver::setAccNormalisation(const std::array<double, kXsensSegmentCount>& am)
{
    QMutexLocker lk(&m_impl->lock);
    m_impl->accMagn = am;
    m_impl->accNormActive = true;
    testLog("[s2s] per-sensor acc_magn normalisation installed", m_impl->test);
}

void MocapReceiver::setGyroBias(const std::array<QVector3D, kXsensSegmentCount>& gb)
{
    QMutexLocker lk(&m_impl->lock);
    m_impl->gyrBias = gb;
    m_impl->gyrBiasActive = true;
    m_impl->calGen.fetch_add(1, std::memory_order_relaxed);
    testLog("[s2s] per-sensor gyr_bias correction installed", m_impl->test);
}

void MocapReceiver::setMagSoftIron(const std::array<std::array<double, 9>, kXsensSegmentCount>& mat,
                                   const std::array<QVector3D, kXsensSegmentCount>& offset)
{
    QMutexLocker lk(&m_impl->lock);
    m_impl->magSoftMat = mat;
    m_impl->magSoftOff = offset;
    m_impl->magSoftActive = true;
    m_impl->magNormActive = false;
    testLog("[s2s] per-sensor mag soft-iron correction installed", m_impl->test);
}

void MocapReceiver::setSegmentGain(const std::array<float, kXsensSegmentCount>& gain)
{
    QMutexLocker lk(&m_impl->lock);
    m_impl->segGain = gain;
    m_impl->segGainActive = true;
    m_impl->calGen.fetch_add(1, std::memory_order_relaxed);
    testLog("[s2s] per-segment AHRS gain installed", m_impl->test);
}

QVector3D MocapReceiver::snapshotGyroAvg(int idx, int samples) const
{
    if (idx < 0 || idx >= kXsensSegmentCount) return QVector3D(0, 0, 0);
    QMutexLocker lk(&m_impl->lock);
    return m_impl->frame.gyrSensor[idx];
}

QVector3D MocapReceiver::liveGyrSensor(int idx) const
{
    if (idx < 0 || idx >= kXsensSegmentCount) return QVector3D(0, 0, 0);
    QMutexLocker lk(&m_impl->lock);
    return m_impl->frame.gyrSensor[idx];
}

SuitPose MocapReceiver::snapshot() const
{
    QMutexLocker lk(&m_impl->lock);
    return m_impl->frame;
}

ConnStatus MocapReceiver::status() const
{
    return (ConnStatus)m_impl->status.load();
}

QString MocapReceiver::statusDetail() const
{
    QMutexLocker lk(&m_impl->lock);
    return m_impl->statusDetail;
}

int MocapReceiver::activeSensors() const { return m_impl->activeTrackers.load(); }

bool MocapReceiver::hasGloves() const
{
    QMutexLocker lk(&m_impl->lock);
    return m_impl->frame.hasGloves;
}

void MocapReceiver::run()
{
    using namespace xda;

    auto& I = *m_impl;
    I.setStatus(ConnStatus::Scanning, "loading XDA driver…", this);

    Api api;
    QString detail;
    if (!loadApi(api, detail)) {
        I.setStatus(ConnStatus::NoDriver, detail, this);
        return;
    }

    // SEH-wrap every XDA call: when no suit/dongle is plugged, XDA internals
    // can throw structured exceptions that would crash the app silently.
    void* control = nullptr;
    if (!sehCall([&]() { control = api.controlConstruct(); }) || !control) {
        unloadApi(api);
        I.setStatus(ConnStatus::Failed,
                    "Не удалось инициализировать XDA. Проверьте драйвер Xsens "
                    "(Awinda dongle / USB-Link / сеть).",
                    this);
        return;
    }

    if (api.enableNetworkScanning) {
        sehCall([&]() { api.enableNetworkScanning(); });
        testLog("[xda] xdaEnableNetworkScanning() called", I.test);
    }
    if (api.controlLoadFilterProfiles) {
        sehCall([&]() { api.controlLoadFilterProfiles(control); });
    }
    if (api.controlSetOptions) {
        sehCall([&]() { api.controlSetOptions(control, 0x04 | 0x08, 0); });
    }
    void* broadcastDev = nullptr;
    if (api.controlBroadcast) {
        sehCall([&]() { broadcastDev = api.controlBroadcast(control); });
    }

    alignas(void*) unsigned char portsStorage[64]{};
    alignas(void*) unsigned char deviceIdsStorage[64]{};
    alignas(void*) unsigned char packetStorage[128]{};
    void* ports     = portsStorage;
    void* deviceIds = deviceIdsStorage;

    if (!sehCall([&]() { api.portInfoArrayConstruct(ports, 0, nullptr); })) {
        unloadApi(api);
        I.setStatus(ConnStatus::Failed,
                    "XDA: ошибка инициализации массива портов", this);
        return;
    }
    const bool preferNet = (I.transport.load() == 1);
    std::size_t portCount = 0;

    // Network discovery is ASYNCHRONOUS.  xdaEnableNetworkScanning() (called
    // above) starts a background listener; a Link/Awinda on the same WiFi or
    // Ethernet network announces itself over the next few seconds.  A single
    // enumerateNetworkDevices() right after enabling almost always returns
    // empty — that was the root cause of "WiFi doesn't work".  Poll it instead,
    // rebuilding the port array each pass so repeated scans don't accumulate
    // stale entries; bail out early on stop or on the first device that answers.
    auto pollNetworkDevices = [&](int maxAttempts) -> std::size_t {
        if (!api.enumerateNetworkDevices) return 0;
        for (int attempt = 1; attempt <= maxAttempts && !I.stop.load(); ++attempt) {
            sehCall([&]() { api.arrayDestruct(ports); });
            sehCall([&]() { api.portInfoArrayConstruct(ports, 0, nullptr); });
            sehCall([&]() { api.enumerateNetworkDevices(ports); });
            const std::size_t n =
                *reinterpret_cast<std::size_t*>(portsStorage + sizeof(void*));
            testLog("[xda] enumerateNetworkDevices attempt "
                    + std::to_string(attempt) + "/" + std::to_string(maxAttempts)
                    + " found " + std::to_string(n) + " device(s)", I.test);
            if (n > 0) return n;
            if (attempt < maxAttempts) {
                I.setStatus(ConnStatus::Scanning,
                            QString("Поиск Xsens в сети (WiFi/Ethernet)… %1/%2")
                                .arg(attempt).arg(maxAttempts), this);
                QThread::msleep(700);
            }
        }
        return 0;
    };

    if (preferNet) {
        I.setStatus(ConnStatus::Scanning,
                    "Поиск Xsens в сети (WiFi/Ethernet)…", this);
        portCount = pollNetworkDevices(17);          // ~12 s of async discovery
        // Fallback to a serial COM scan if nothing answered on the network.
        if (portCount == 0 && !I.stop.load()) {
            I.setStatus(ConnStatus::Scanning,
                        "Сеть пуста — пробуем COM порты…", this);
            sehCall([&]() { api.arrayDestruct(ports); });
            sehCall([&]() { api.portInfoArrayConstruct(ports, 0, nullptr); });
            sehCall([&]() { api.scanPorts(ports, 0, 1000, 1, 1); });
            portCount =
                *reinterpret_cast<std::size_t*>(portsStorage + sizeof(void*));
        }
    } else {
        I.setStatus(ConnStatus::Scanning, "Сканирование Xsens портов…", this);
        sehCall([&]() { api.scanPorts(ports, 0, 1000, 1, 1); });
        portCount =
            *reinterpret_cast<std::size_t*>(portsStorage + sizeof(void*));
        testLog("[xda] scanPorts found " + std::to_string(portCount) + " port(s)",
                I.test);
        // Fallback: network discovery (Link/Awinda over WiFi/Ethernet, Body
        // Pack V2 on USB-NCM).  Same async poll as the WiFi path, shorter budget.
        if (portCount == 0 && !I.stop.load()) {
            I.setStatus(ConnStatus::Scanning,
                        "Поиск Xsens в сети (WiFi/Ethernet)…", this);
            portCount = pollNetworkDevices(11);      // ~7 s
        }
    }

    if (portCount == 0) {
        api.arrayDestruct(ports);
        api.controlClose(control);
        if (api.controlDestruct) api.controlDestruct(control);
        unloadApi(api);
        const QString noDevMsg = preferNet
            ? QStringLiteral(
                "Костюм не найден в сети. Убедитесь, что этот ПК и костюм "
                "подключены к ОДНОЙ Wi-Fi сети (проверьте Wi-Fi в Windows).")
            : QStringLiteral(
                "Xsens не найден. Подключите Awinda dongle или USB-кабель Link "
                "и проверьте драйвер Xsens.");
        I.setStatus(ConnStatus::NoDevice, noDevMsg, this);
        return;
    }

    // ---- OPEN first port that yields tracker devices -------------------
    std::vector<void*> trackerHandles;
    std::vector<int>   trackerSegments;

    for (std::size_t i = 0; i < portCount && trackerHandles.empty(); ++i) {
        void* portInfo = api.arrayAt(ports, i);
        if (!portInfo) continue;

        I.setStatus(ConnStatus::Connecting,
                    QString("opening Xsens port [%1/%2]…")
                        .arg(i + 1).arg(portCount),
                    this);

        const int openRes = api.controlOpenPort1(control, portInfo, 2000, 1);
        if (!openRes) {
            testLog("[xda] openPort failed for port " + std::to_string(i), I.test);
            continue;
        }
        QThread::msleep(500);

        api.deviceIdArrayConstruct(deviceIds, 0, nullptr);
        api.controlDeviceIds(control, deviceIds);

        const std::size_t devCount =
            *reinterpret_cast<std::size_t*>(deviceIdsStorage + sizeof(void*));
        testLog("[xda]   found " + std::to_string(devCount)
                + " device(s) on this port", I.test);

        for (std::size_t d = 0; d < devCount; ++d) {
            void* idPtr = api.arrayAt(deviceIds, d);
            if (!idPtr) continue;
            auto* id = reinterpret_cast<XsDeviceIdBlob*>(idPtr);

            void* dev = api.controlDevice(control, id);
            if (!dev) continue;

            const int locId  = api.deviceLocationId(dev);
            const int segIdx = segmentFromLocationId(locId);
            testLog("[xda]     device locationId=" + std::to_string(locId)
                    + " → " + (segIdx >= 0 ? kSegmentNames[segIdx]
                                            : std::string("(service device)")),
                    I.test);
            if (segIdx < 0) continue;
            trackerHandles.push_back(dev);
            trackerSegments.push_back(segIdx);
        }
        api.arrayDestruct(deviceIds);
        if (trackerHandles.empty()) {
            testLog("[xda]   no trackers here, moving on", I.test);
        }
    }
    api.arrayDestruct(ports);

    if (trackerHandles.empty()) {
        api.controlClose(control);
        if (api.controlDestruct) api.controlDestruct(control);
        unloadApi(api);
        I.setStatus(ConnStatus::NoDevice,
                    "port(s) opened but no Xsens motion trackers reported",
                    this);
        return;
    }

    I.activeTrackers.store((int)trackerHandles.size());
    I.setStatus(ConnStatus::Connecting,
                QString("suit detected — %1 trackers, starting measurement…")
                    .arg(trackerHandles.size()),
                this);

    // ---- Broadcast gotoMeasurement ------------------------------------
    bool measuring = false;
    if (broadcastDev) measuring = api.deviceGotoMeasurement(broadcastDev) != 0;
    if (!measuring) {
        for (void* d : trackerHandles) {
            if (api.deviceGotoMeasurement(d)) { measuring = true; break; }
        }
    }
    if (!measuring) {
        api.controlClose(control);
        if (api.controlDestruct) api.controlDestruct(control);
        unloadApi(api);
        I.setStatus(ConnStatus::Failed,
                    "XsDevice_gotoMeasurement refused — check firmware / radio", this);
        return;
    }
    QThread::msleep(1000);     // let the stream spin up

    // Seed the working rate from the suit selection (Link 240 / Awinda 60) so
    // the SDI → acc/gyr conversion is correct before the device is queried.
    const double expectedHz = I.expectedRateHz.load();
    I.freqHz = expectedHz;

    // Query native update rate and reconcile with the suit's expected rate.  We
    // honour whatever the firmware reports (it drives the IMU math) but warn the
    // operator if it disagrees with the suit they picked.
    if (api.deviceUpdateRate && !trackerHandles.empty()) {
        const int rate = api.deviceUpdateRate(trackerHandles.front());
        if (rate > 0) I.freqHz = double(rate);
        testLog("[xda] native update rate = " + std::to_string(I.freqHz) + " Hz"
                " (expected " + std::to_string(int(expectedHz)) + ")", I.test);
        if (rate > 0 && std::abs(double(rate) - expectedHz) > 1.0) {
            testLog("[xda] WARNING: device reports " + std::to_string(rate)
                    + " Hz but the selected suit implies "
                    + std::to_string(int(expectedHz)) + " Hz", I.test);
        }
    }

    // Per-segment calibration/config copied out of Impl under the lock once
    // per packet, so the fusion math below never reads a half-written array
    // (torn Quat / QVector3D) while the calibration wizard commits new values
    // from the GUI thread.  Defaults are "inactive / identity" so an invalid
    // segment id simply applies nothing.
    struct SegCal {
        bool   s2sActive = false, magNormActive = false, accNormActive = false,
               gyrBiasActive = false, magSoftActive = false, segGainActive = false;
        double accMagn = 1.0, magMagn = 1.0;
        QVector3D gyrBias, magSoftOff;
        std::array<double, 9> magSoftMat{};
        Quat   s2sInv;
        float  segGain = 0.0f;
    };

    // ---- Poll loop -----------------------------------------------------
    SuitPose staging;
    int framesThisSec = 0;
    double fpsT0 = monotonicSec();
    I.lastPacket = monotonicSec();

    // Per-sensor "dumped once in -test" flag — we want to see each sensor's
    // first three complete frames, then stop spamming.
    std::array<int, kXsensSegmentCount> dumpCount{};

    // Last observed calibration generation; when a setter bumps Impl::calGen we
    // re-initialise every fusion / bias filter on the next packet.
    uint32_t lastCalGen = I.calGen.load(std::memory_order_relaxed);

    while (!I.stop.load()) {
        bool gotAny = false;

        for (std::size_t t = 0; t < trackerHandles.size(); ++t) {
            void* dev     = trackerHandles[t];
            const int seg = trackerSegments[t];
            // SEH-wrap the XDA entry calls: if the suit/dongle drops mid-session
            // these can raise a structured exception that would otherwise crash
            // the whole app (the comment above promised this; the hot path was
            // not actually wrapped).  On a fault we skip this tracker for the tick.
            int pktCount = 0;
            if (!sehCall([&]{ pktCount = api.deviceGetDataPacketCount(dev); })
                || pktCount <= 0) continue;

            XsDataPacketBlob* pkt = nullptr;
            if (!sehCall([&]{
                    api.dataPacketConstruct(reinterpret_cast<XsDataPacketBlob*>(packetStorage));
                    pkt = api.deviceTakeFirstDataPacketInQueue(
                        dev, reinterpret_cast<XsDataPacketBlob*>(packetStorage));
                })) continue;

            // Resolve which segment this packet came from — prefer an
            // embedded stored-location-id over our static mapping.
            int targetSeg = seg;
            if (pkt && api.dataPacketContainsStoredLocationId &&
                api.dataPacketStoredLocationId &&
                api.dataPacketContainsStoredLocationId(pkt)) {
                const int locId = api.dataPacketStoredLocationId(pkt);
                const int ss = segmentFromLocationId(locId);
                if (ss >= 0) targetSeg = ss;
            }

            // Snapshot the GUI-thread-written calibration/config for this
            // segment under the lock, and pick up any pending re-init request.
            SegCal cal;
            {
                QMutexLocker lk(&I.lock);
                const uint32_t gen = I.calGen.load(std::memory_order_relaxed);
                if (gen != lastCalGen) {
                    I.fusionReady.fill(false);
                    I.biasReady.fill(false);
                    lastCalGen = gen;
                }
                if (targetSeg >= 0 && targetSeg < kXsensSegmentCount) {
                    cal.s2sActive     = I.s2sActive;
                    cal.magNormActive = I.magNormActive;
                    cal.accNormActive = I.accNormActive;
                    cal.gyrBiasActive = I.gyrBiasActive;
                    cal.magSoftActive = I.magSoftActive;
                    cal.segGainActive = I.segGainActive;
                    cal.accMagn       = I.accMagn[targetSeg];
                    cal.magMagn       = I.magMagn[targetSeg];
                    cal.gyrBias       = I.gyrBias[targetSeg];
                    cal.magSoftOff    = I.magSoftOff[targetSeg];
                    cal.magSoftMat    = I.magSoftMat[targetSeg];
                    cal.s2sInv        = I.s2sInv[targetSeg];
                    cal.segGain       = I.segGain[targetSeg];
                }
            }

            // --- Quaternion (if packet carries one) ----------------------
            Quat qo;
            bool haveQuat = false;
            if (pkt && api.dataPacketContainsOrientation &&
                api.dataPacketContainsOrientation(pkt) &&
                api.dataPacketOrientationQuaternion) {
                XsQuaternionBlob q{};
                const int cs = api.dataPacketCoordSysOrient ?
                               api.dataPacketCoordSysOrient(pkt) : 0;
                api.dataPacketOrientationQuaternion(pkt, &q, cs);
                qo = Quat(q.w, q.x, q.y, q.z).normalized();
                haveQuat = true;
            }

            // --- Pull every IMU channel the packet might carry ----------
            // Body Pack V2 streams SDI (velocityIncrement / orientationIncrement)
            // + mag; absolute calibratedAcc/Gyro are usually absent.
            QVector3D acc, gyr, mag, velInc, freeAcc;
            Quat      dq;
            bool haveAcc=false, haveGyr=false, haveMag=false,
                 haveVelInc=false, haveFreeAcc=false, haveDq=false;
            auto readVec = [&](FnDataPacketContainsAcc contains,
                               FnDataPacketAcc         getter,
                               QVector3D& out) -> bool {
                if (!contains || !getter) return false;
                if (!contains(pkt)) return false;
                XsVectorBlob v{};
                getter(pkt, &v);
                bool ok = false;
                if (v.data && v.size >= 3) {
                    out = QVector3D(float(v.data[0]),
                                    float(v.data[1]),
                                    float(v.data[2]));
                    ok = true;
                }
                if (api.vectorDestruct) api.vectorDestruct(&v);
                return ok;
            };
            haveAcc    = readVec(api.dataPacketContainsAcc,    api.dataPacketAcc,    acc);
            haveGyr    = readVec(api.dataPacketContainsGyro,   api.dataPacketGyro,   gyr);
            haveMag    = readVec(api.dataPacketContainsMag,    api.dataPacketMag,    mag);
            haveVelInc = readVec(api.dataPacketContainsVelInc, api.dataPacketVelInc, velInc);
            haveFreeAcc= readVec(api.dataPacketContainsFreeAcc,api.dataPacketFreeAcc,freeAcc);
            if (api.dataPacketContainsOrientationIncrement &&
                api.dataPacketOrientationIncrement &&
                api.dataPacketContainsOrientationIncrement(pkt)) {
                XsQuaternionBlob q{};
                api.dataPacketOrientationIncrement(pkt, &q);
                dq = Quat(q.w, q.x, q.y, q.z);
                haveDq = true;
            }
            quint32 stf = 0;
            bool haveStf = false;
            if (api.dataPacketContainsSTF && api.dataPacketSTF &&
                api.dataPacketContainsSTF(pkt)) {
                stf = api.dataPacketSTF(pkt); haveStf = true;
            }

            // --- Reconstruct acc / gyr from SDI (Body Pack V2 path) ------
            // velInc = Δv per sample  ⇒ acc = Δv · freq  (m/s²)
            // dq    ≈ (1, ω·dt/2)     ⇒ gyr ≈ 2·dq.xyz · freq  (rad/s)
            //
            // xio Fusion's API expects gyr in DEG/S and acc in G — NOT the
            // SI units we reconstruct.  Convert here so FusionAhrsUpdate
            // sees the magnitudes it was calibrated on (2000 dps range,
            // ~1 g at rest).  Without the conversion the gyro is ~57×
            // weaker than expected and the filter drags behind motion.
            const double dt = 1.0 / I.freqHz;
            constexpr double kRadToDeg = 57.29577951308232;
            constexpr double kMs2ToG   = 1.0 / 9.80665;
            QVector3D accForFilter, gyrForFilter;
            bool fuseReady = false;
            if (haveVelInc && haveDq) {
                // SI units first (for the diagnostics dump), then xio units.
                const QVector3D accSI(float(velInc.x() * I.freqHz),
                                      float(velInc.y() * I.freqHz),
                                      float(velInc.z() * I.freqHz));
                const QVector3D gyrSI(float(2.0 * dq.x * I.freqHz),
                                      float(2.0 * dq.y * I.freqHz),
                                      float(2.0 * dq.z * I.freqHz));
                accForFilter = accSI * float(kMs2ToG);     // m/s² → g
                gyrForFilter = gyrSI * float(kRadToDeg);   // rad/s → deg/s
                fuseReady = true;
            } else if (haveAcc && haveGyr) {
                accForFilter = acc * float(kMs2ToG);
                gyrForFilter = gyr * float(kRadToDeg);
                fuseReady = true;
            }

            // Expose the raw calibrated sensor vectors so the wizard can
            // average them for the static_sensor_to_segment_calibration +
            // acc_magn + gyr_bias + mag_magn (all four apply_imu_calibration
            // terms from hipose).
            if (fuseReady && targetSeg >= 0 && targetSeg < kXsensSegmentCount) {
                QMutexLocker raw(&I.lock);
                staging.accSensor[targetSeg] = accForFilter;   // g
                staging.gyrSensor[targetSeg] = gyrForFilter;   // deg/s
                if (haveMag) staging.magSensor[targetSeg] = mag;
            }

            // -test: capture the raw SDI increments and the post-SDI,
            // pre-calibration sample so [FUSED] can trace every axis from
            // arrival forward.  Receiver-thread only (same thread as the dump).
            if (I.test && targetSeg >= 0 && targetSeg < kXsensSegmentCount) {
                I.dbgVelInc[targetSeg] = velInc;
                I.dbgDqXyz[targetSeg]  = QVector3D(float(dq.x), float(dq.y), float(dq.z));
                I.dbgAccPre[targetSeg] = accForFilter;
                I.dbgGyrPre[targetSeg] = gyrForFilter;
                I.dbgMagPre[targetSeg] = mag;
                I.dbgChainFlags[targetSeg] =
                    quint8((haveMag ? 1 : 0) | ((haveVelInc && haveDq) ? 2 : 0)
                           | ((haveAcc && haveGyr) ? 4 : 0));
            }

            // Hipose apply_imu_calibration, in order:
            //   acc_cal = acc / acc_magn   (unit-g scaled)
            //   gyr_cal = gyr - gyr_bias   (DC offset removed)
            //   mag_cal = mag / mag_magn   (unit-length scaled)
            //   (acc,gyr,mag)_body = rotate(.., inv(s2s))
            //
            // Acc magnitude normalisation — cancels per-tracker accel
            // scaling bias so gravity evaluates to exactly 1 g in rest.
            if (cal.accNormActive) {
                const double a = cal.accMagn;
                if (a > 1e-6) accForFilter = accForFilter / float(a);
            }
            if (I.test && targetSeg >= 0 && targetSeg < kXsensSegmentCount)
                I.dbgAccNorm[targetSeg] = accForFilter;

            // Gyro DC-bias removal — without this a sensor's tiny constant
            // drift accumulates into a visible yaw/pitch creep over the
            // span of a minute of motion, which is precisely what broke
            // elbows / wrists / twists in the previous runs.
            if (cal.gyrBiasActive) {
                gyrForFilter = gyrForFilter - cal.gyrBias;
            }
            if (I.test && targetSeg >= 0 && targetSeg < kXsensSegmentCount)
                I.dbgGyrUnbias[targetSeg] = gyrForFilter;

            if (cal.magSoftActive && haveMag)
            {
                const auto& M = cal.magSoftMat;
                const QVector3D off = cal.magSoftOff;
                const double dx = double(mag.x()) - double(off.x());
                const double dy = double(mag.y()) - double(off.y());
                const double dz = double(mag.z()) - double(off.z());
                const double rx = M[0]*dx + M[1]*dy + M[2]*dz;
                const double ry = M[3]*dx + M[4]*dy + M[5]*dz;
                const double rz = M[6]*dx + M[7]*dy + M[8]*dz;
                mag = QVector3D(float(rx), float(ry), float(rz));
            }
            else if (cal.magNormActive && haveMag)
            {
                const double m = cal.magMagn;
                if (m > 1e-6) mag = mag / float(m);
            }
            if (I.test && targetSeg >= 0 && targetSeg < kXsensSegmentCount)
                I.dbgMagSoft[targetSeg] = mag;

            // Sensor-to-segment rotation so the fusion output is already
            // in body-segment-world space.
            if (cal.s2sActive) {
                const Quat& inv = cal.s2sInv;
                accForFilter = vec_rotate(accForFilter, inv);
                gyrForFilter = vec_rotate(gyrForFilter, inv);
                if (haveMag) mag = vec_rotate(mag, inv);
            }

            // -test: snapshot the fully-calibrated, body-frame IMU sample
            // (post acc-norm / gyr-bias / mag soft-iron / s2s) for [FUSED SNAPSHOT].
            if (I.test && targetSeg >= 0 && targetSeg < kXsensSegmentCount) {
                I.dbgAccBody[targetSeg] = accForFilter;
                I.dbgGyrBody[targetSeg] = gyrForFilter;
                I.dbgMagBody[targetSeg] = mag;
            }

            // A corrupt / partial packet (or a degenerate s2s) can yield a
            // non-finite IMU sample.  Feeding NaN/Inf to FusionAhrsUpdate
            // permanently poisons that sensor's filter state (and Quat::
            // normalized() does NOT sanitise NaN), so we skip the update this
            // tick and let the segment hold its last good orientation.
            const bool inputsFinite =
                std::isfinite(accForFilter.x()) && std::isfinite(accForFilter.y()) &&
                std::isfinite(accForFilter.z()) &&
                std::isfinite(gyrForFilter.x()) && std::isfinite(gyrForFilter.y()) &&
                std::isfinite(gyrForFilter.z()) &&
                (!haveMag || (std::isfinite(mag.x()) && std::isfinite(mag.y()) &&
                              std::isfinite(mag.z())));

            Quat fusedQuat;
            bool haveFused = false;
            if (fuseReady && inputsFinite &&
                targetSeg >= 0 && targetSeg < kXsensSegmentCount) {
                FusionAhrs& ahrs = I.fusion[targetSeg];
                FusionAhrsSettings& s = I.ahrsCfg[targetSeg];
                if (!I.fusionReady[targetSeg]) {
                    FusionAhrsInitialise(&ahrs);
                    s = fusionAhrsDefaultSettings;
                    s.convention            = FusionConventionNwu;
                    // Live mocap settings:
                    //   * moderate gain (0.5) — responsive without overshoot
                    //   * rejection thresholds squared: accel 25° ≈ 0.18
                    //     field units (we pass the squared value below
                    //     because the lib stores the pre-squared magnitude)
                    //   * recoveryTriggerPeriod=0 so we never get locked in
                    //     gyro-only mode during fast motion
                    s.gain                  = (cal.segGainActive && cal.segGain > 0.0f)
                                              ? cal.segGain : 0.7f;
                    s.gyroscopeRange        = 2000.0f;
                    // Softened rejection thresholds — tight values (15°/30°)
                    // parked the filter in gyro-only mode during static but
                    // complex poses (sitting, arms crossed, leg-over-leg),
                    // causing visible drift until recovery.  These values
                    // correct gravity / mag faster while still filtering
                    // real disturbances.
                    s.accelerationRejection = 30.0f;
                    s.magneticRejection     = 50.0f;
                    s.recoveryTriggerPeriod = int(I.freqHz / 2);  // 0.5s recovery
                    FusionAhrsSetSettings(&ahrs, &s);
                    // Kill the 3-second startup ramp — `startup` + `rampedGain`
                    // add real latency to the first motion after a reset.
                    // Forcing them here lets the filter respond from the
                    // very first packet at the steady-state gain.
                    ahrs.startup    = false;
                    ahrs.rampedGain = s.gain;
                    I.fusionReady[targetSeg] = true;
                }

                FusionBias& biasRef = I.bias[targetSeg];
                if (!I.biasReady[targetSeg]) {
                    FusionBiasInitialise(&biasRef);
                    FusionBiasSettings bs = fusionBiasDefaultSettings;
                    bs.sampleRate          = float(std::max(60.0, I.freqHz));
                    bs.stationaryThreshold = 1.5f;
                    bs.stationaryPeriod    = 1.0f;
                    FusionBiasSetSettings(&biasRef, &bs);
                    I.biasReady[targetSeg] = true;
                }

                FusionVector g = {{ float(gyrForFilter.x()),
                                    float(gyrForFilter.y()),
                                    float(gyrForFilter.z()) }};
                g = FusionBiasUpdate(&biasRef, g);
                if (I.test)
                    I.dbgGyrFused[targetSeg] =
                        QVector3D(g.axis.x, g.axis.y, g.axis.z);  // post-bias gyro

                const FusionVector a = {{ float(accForFilter.x()),
                                          float(accForFilter.y()),
                                          float(accForFilter.z()) }};

                const float aLen = std::sqrt(a.axis.x*a.axis.x + a.axis.y*a.axis.y + a.axis.z*a.axis.z);
                const float aErr = std::abs(aLen - 1.0f);
                const float beta = std::exp(-3.0f * aErr * aErr);
                const float dynAccRej = 30.0f + (80.0f - 30.0f) * (1.0f - beta);

                const bool useMag = haveMag && (mag.length() > 1e-6);
                float dynMagRej = 50.0f;
                if (useMag) {
                    const float mLen = std::sqrt(float(mag.x()*mag.x() + mag.y()*mag.y() + mag.z()*mag.z()));
                    const float mErr = std::abs(mLen - 1.0f);
                    if (mErr > 0.40f)      dynMagRej = 30.0f;
                    else if (mErr > 0.20f) dynMagRej = 40.0f;
                    else                   dynMagRej = 50.0f;
                }

                if (s.accelerationRejection != dynAccRej || s.magneticRejection != dynMagRej) {
                    s.accelerationRejection = dynAccRej;
                    s.magneticRejection     = dynMagRej;
                    FusionAhrsSetSettings(&ahrs, &s);
                }
                if (I.test) {
                    I.dbgDynAccRej[targetSeg] = dynAccRej;
                    I.dbgDynMagRej[targetSeg] = dynMagRej;
                    I.dbgAccErr[targetSeg]    = aErr;
                }

                if (useMag) {
                    const FusionVector m = {{ float(mag.x()),
                                              float(mag.y()),
                                              float(mag.z()) }};
                    FusionAhrsUpdate(&ahrs, g, a, m, float(dt));
                } else {
                    FusionAhrsUpdateNoMagnetometer(&ahrs, g, a, float(dt));
                }
                // xio AHRS natively outputs in the convention we picked
                // (NWU), no extra rotation needed.
                const FusionQuaternion fq = FusionAhrsGetQuaternion(&ahrs);
                if (std::isfinite(fq.element.w) && std::isfinite(fq.element.x) &&
                    std::isfinite(fq.element.y) && std::isfinite(fq.element.z)) {
                    fusedQuat = Quat(fq.element.w, fq.element.x,
                                     fq.element.y, fq.element.z).normalized();
                    haveFused = true;
                    if (I.test) I.dbgFusedQuat[targetSeg] = fusedQuat;
                } else {
                    // Filter state went non-finite — drop it and force a clean
                    // re-init on the next packet so the sensor can recover.
                    I.fusionReady[targetSeg] = false;
                }
            }

            // --- Publish into the shared frame ---------------------------
            if (targetSeg >= 0 && targetSeg < kXsensSegmentCount) {
                QMutexLocker lk(&I.lock);
                if      (haveFused) staging.quat[targetSeg] = fusedQuat;
                else if (haveQuat)  staging.quat[targetSeg] = qo;
                staging.segValid[targetSeg] = haveFused || haveQuat;
                staging.segLastT[targetSeg] = monotonicSec();
                if (api.dataPacketContainsPacketCounter &&
                    api.dataPacketPacketCounter &&
                    api.dataPacketContainsPacketCounter(pkt))
                    staging.sampleCounter = api.dataPacketPacketCounter(pkt);
                staging.recvTime = monotonicSec();

                // --- Merge latest glove snapshot (Manus ergonomics stream)
                // into staging.  Positions are in HAND-LOCAL frame here —
                // the render tick rotates them by the wrist's world quat.
                // Read the connection flags AND the joint arrays under the same
                // g_ergo lock.  Loading the atomics first and locking second
                // left a window in which the Manus callback thread could flip a
                // hand's "have" state between the load and the lock, pairing a
                // stale flag with freshly-(re)written arrays.
                {
                    QMutexLocker lk2(&g_ergo.lock);
                    const bool haveL = g_ergo.haveLeft.load();
                    const bool haveR = g_ergo.haveRight.load();
                    if (haveL) {
                        staging.leftGloveQ  = g_ergo.leftQ;
                        staging.leftGloveP  = g_ergo.leftP;
                    }
                    if (haveR) {
                        staging.rightGloveQ = g_ergo.rightQ;
                        staging.rightGloveP = g_ergo.rightP;
                    }
                    if (haveL || haveR)
                        staging.hasGloves = true;
                }

                I.frame = staging;
                gotAny = true;
            }

            // --- First-N per-sensor dump in -test ------------------------
            if (I.test && targetSeg >= 0 && targetSeg < kXsensSegmentCount &&
                dumpCount[targetSeg] < 3)
            {
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(5);
                ss << "[raw] seg=" << std::left << std::setw(14)
                   << kSegmentNames[targetSeg] << std::right;
                if (haveStf) ss << " stf=" << stf;
                ss << "\n       quat  =";
                if (haveQuat) ss << "(" << qo.w << "," << qo.x << ","
                                 << qo.y << "," << qo.z << ")";
                else          ss << "-";
                ss << "\n       dq    =";
                if (haveDq)  ss << "(" << dq.w << "," << dq.x << ","
                                << dq.y << "," << dq.z << ")";
                else         ss << "-";
                ss << "\n       acc   =";
                if (haveAcc) ss << "(" << acc.x() << "," << acc.y() << ","
                                << acc.z() << ") m/s²";
                else         ss << "-";
                ss << "\n       gyr   =";
                if (haveGyr) ss << "(" << gyr.x() << "," << gyr.y() << ","
                                << gyr.z() << ") rad/s";
                else         ss << "-";
                ss << "\n       mag   =";
                if (haveMag) ss << "(" << mag.x() << "," << mag.y() << ","
                                << mag.z() << ") a.u.";
                else         ss << "-";
                ss << "\n       vInc  =";
                if (haveVelInc) ss << "(" << velInc.x() << "," << velInc.y() << ","
                                   << velInc.z() << ") m/s   [SDI Δv]";
                else            ss << "-";
                ss << "\n       fAcc  =";
                if (haveFreeAcc) ss << "(" << freeAcc.x() << "," << freeAcc.y()
                                    << "," << freeAcc.z() << ") m/s²";
                else             ss << "-";
                std::cout << ss.str() << '\n';
                // One-time "which channels does this packet actually carry"
                // snapshot.  Tells us exactly what Body Pack V2 streams.
                if (dumpCount[targetSeg] == 0) {
                    std::ostringstream cs;
                    cs << "       channels:";
                    for (auto& p : api.probes) {
                        if (p.fn && p.fn(pkt))
                            cs << " " << p.name;
                    }
                    std::cout << cs.str() << '\n';
                }
                std::cout.flush();
                ++dumpCount[targetSeg];
            }

            sehCall([&]{
                api.dataPacketDestruct(reinterpret_cast<XsDataPacketBlob*>(packetStorage));
            });
        }

        const double now = monotonicSec();

        if (gotAny) {
            I.lastPacket = now;
            ++framesThisSec;
            if (I.status.load() != (int)ConnStatus::Streaming) {
                I.setStatus(ConnStatus::Streaming,
                    QString("%1 trackers streaming")
                        .arg(trackerHandles.size()),
                    this);
            }
            emit poseReceived();

            if (now - fpsT0 >= 1.0) {
                emit fpsUpdated(framesThisSec / (now - fpsT0));
                framesThisSec = 0;
                fpsT0 = now;
            }

            // Once-every-1.5-seconds full snapshot of every tracker's
            // state across the full calibration pipeline: raw IMU →
            // normalisation → s2s rotation → fusion output → final
            // staged quaternion with Euler decomposition.  The goal is
            // to surface every stage the filter touches so micro-bugs
            // are visible in the log without needing the viewport.
            if (I.test && (now - I.lastDump) > 1.5) {
                QMutexLocker lk(&I.lock);
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(3);
                ss << "\n========== [FUSED SNAPSHOT] sample="
                   << staging.sampleCounter
                   << "  t=" << std::setprecision(2) << now
                   << "s ==========\n";

                // Quaternion → intrinsic XYZ Euler (deg).  Matches
                // scipy.spatial.transform.Rotation.from_quat(w,x,y,z).as_euler('XYZ').
                auto quatEulerDeg = [](const Quat& q, double& rx,
                                       double& ry, double& rz) {
                    const double m00 = 1-2*(q.y*q.y+q.z*q.z);
                    const double m01 = 2*(q.x*q.y - q.z*q.w);
                    const double m02 = 2*(q.x*q.z + q.y*q.w);
                    const double m10 = 2*(q.x*q.y + q.z*q.w);
                    const double m12 = 2*(q.y*q.z - q.x*q.w);
                    const double m22 = 1-2*(q.x*q.x+q.y*q.y);
                    (void)m10; (void)m00;
                    const double sy = std::clamp(-m02, -1.0, 1.0);
                    const double K = 180.0 / M_PI;
                    ry = std::asin(sy) * K;
                    rx = std::atan2(m12, m22) * K;
                    rz = std::atan2(m01, m00) * K;
                };

                // Per-segment expanded dump.  Columns on the body rows:
                //   raw sensor acc (g) and gyr (deg/s), as captured after
                //   the SI-reconstruction step; the world-frame output
                //   quaternion; the corresponding Euler XYZ in degrees;
                //   a "live" flag from segValid.
                ss << std::setprecision(3);
                ss << "--- per-segment state (acc/gyr raw [g, deg/s], "
                      "|mag|, out-quat, euler XYZ deg) ---\n";
                for (int i = 0; i < kXsensSegmentCount; ++i) {
                    const Quat& q = staging.quat[i];
                    const QVector3D& a = staging.accSensor[i];
                    const QVector3D& g = staging.gyrSensor[i];
                    const QVector3D& m = staging.magSensor[i];
                    double rx, ry, rz;
                    quatEulerDeg(q, rx, ry, rz);
                    const double aLen = std::sqrt(a.x()*a.x()+a.y()*a.y()+a.z()*a.z());
                    const double mLen = std::sqrt(m.x()*m.x()+m.y()*m.y()+m.z()*m.z());
                    ss << "  seg[" << std::setw(2) << i << "] "
                       << std::left << std::setw(14) << kSegmentNames[i]
                       << std::right
                       << (staging.segValid[i] ? " live" : " ----")
                       << "  acc=(" << std::setw(6) << a.x() << ","
                                    << std::setw(6) << a.y() << ","
                                    << std::setw(6) << a.z()
                       << ")|" << std::setw(5) << aLen << "g"
                       << "  gyr=(" << std::setw(7) << g.x() << ","
                                    << std::setw(7) << g.y() << ","
                                    << std::setw(7) << g.z() << ")°/s"
                       << "  |mag|=" << std::setw(5) << mLen
                       << "\n           "
                       << "quat=(" << std::setw(6) << q.w << ","
                                   << std::setw(6) << q.x << ","
                                   << std::setw(6) << q.y << ","
                                   << std::setw(6) << q.z << ")"
                       << "  euler=(" << std::setw(7) << rx << ","
                                      << std::setw(7) << ry << ","
                                      << std::setw(7) << rz << ")°"
                       << "  |ang|=" << std::setw(6) << quat_angle_deg(q) << "°\n";
                }
                // Post-calibration, body-frame IMU sample per sensor — the
                // exact values fed to the AHRS after acc-norm / gyr-bias / mag
                // soft-iron and the sensor->body s2s rotation, so every axis
                // transform from raw (above) to filter input is visible.
                ss << std::setprecision(4);
                ss << "--- per-sensor calibrated body-frame IMU (post acc-norm/gyr-bias/"
                      "mag-soft-iron/s2s; gyrFused = post-FusionBias) ---\n";
                for (int i = 0; i < kXsensSegmentCount; ++i) {
                    const QVector3D& ab = I.dbgAccBody[i];
                    const QVector3D& gb = I.dbgGyrBody[i];
                    const QVector3D& mb = I.dbgMagBody[i];
                    const QVector3D& gf = I.dbgGyrFused[i];
                    ss << "  cal[" << std::setw(2) << i << "] "
                       << std::left << std::setw(14) << kSegmentNames[i] << std::right
                       << " accB=("  << std::setw(8) << ab.x() << "," << std::setw(8) << ab.y()
                                     << "," << std::setw(8) << ab.z() << ")g"
                       << " gyrB=("  << std::setw(9) << gb.x() << "," << std::setw(9) << gb.y()
                                     << "," << std::setw(9) << gb.z() << ")°/s"
                       << " gyrFused=(" << std::setw(9) << gf.x() << "," << std::setw(9) << gf.y()
                                     << "," << std::setw(9) << gf.z() << ")"
                       << " magB=("  << std::setw(8) << mb.x() << "," << std::setw(8) << mb.y()
                                     << "," << std::setw(8) << mb.z() << ")"
                       << " accErr=" << std::setw(7) << I.dbgAccErr[i]
                       << " rej[acc=" << std::setw(5) << I.dbgDynAccRej[i]
                       << " mag="     << std::setw(5) << I.dbgDynMagRej[i] << "]deg\n";
                }
                // -test: full per-axis transform chain from arrival to fusion
                // input for every sensor that produced a sample this snapshot.
                // Values are the EXACT ones the fusion loop used (single source),
                // so a per-axis sign/scale/units bug is visible without the
                // viewport.  Verbose, but periodic (every 1.5 s) — detail, not
                // frequency, per the logging spec.
                ss << std::setprecision(4);
                ss << "--- per-axis sensor transform chain (raw SDI -> recon -> "
                      "accNorm/gyrBias/magSoft -> s2s body -> fused) ---\n";
                {
                    auto V = [&](const QVector3D& v){
                        ss << "(" << std::setw(8) << v.x() << "," << std::setw(8) << v.y()
                           << "," << std::setw(8) << v.z() << ")"; };
                    for (int i = 0; i < kXsensSegmentCount; ++i) {
                        const quint8 fl = I.dbgChainFlags[i];
                        if (fl == 0) continue;              // no sample from this sensor
                        const char* src = (fl & 2) ? "SDI" : (fl & 4) ? "absAccGyr" : "?";
                        const Quat& fq = I.dbgFusedQuat[i];
                        ss << "  chain[" << std::setw(2) << i << "] "
                           << std::left << std::setw(14) << kSegmentNames[i] << std::right
                           << " src=" << src << (fl & 1 ? " +mag" : " no-mag") << "\n";
                        ss << "      acc velInc="; V(I.dbgVelInc[i]);
                        ss << " pre(g)=";   V(I.dbgAccPre[i]);
                        ss << " norm(/" << std::setw(6) << I.accMagn[i] << ")="; V(I.dbgAccNorm[i]);
                        ss << " s2sBody="; V(I.dbgAccBody[i]); ss << "\n";
                        ss << "      gyr dq.xyz="; V(I.dbgDqXyz[i]);
                        ss << " pre(d/s)="; V(I.dbgGyrPre[i]);
                        ss << " unbias=";   V(I.dbgGyrUnbias[i]);
                        ss << " s2sBody=";  V(I.dbgGyrBody[i]);
                        ss << " fused=";    V(I.dbgGyrFused[i]); ss << "\n";
                        ss << "      mag raw="; V(I.dbgMagPre[i]);
                        ss << " soft/norm="; V(I.dbgMagSoft[i]);
                        ss << " s2sBody=";   V(I.dbgMagBody[i]); ss << "\n";
                        ss << "      fusedQuat=(" << std::setw(8) << fq.w << "," << std::setw(8) << fq.x
                           << "," << std::setw(8) << fq.y << "," << std::setw(8) << fq.z << ")\n";
                    }
                }
                ss << std::setprecision(3);
                // Calibration state — printed once so misreads at the
                // raw→normalised boundary are immediately diagnosable.
                ss << "--- calibration flags: s2s=" << (I.s2sActive ? "on" : "off")
                   << "  accNorm=" << (I.accNormActive ? "on" : "off")
                   << "  magNorm=" << (I.magNormActive ? "on" : "off")
                   << "  gyrBias=" << (I.gyrBiasActive ? "on" : "off")
                   << "  freq=" << std::setprecision(1) << I.freqHz << "Hz ---\n";
                // Madgwick/xio AHRS settings (Pelvis sensor, representative) so
                // the fusion stage's tunables are auditable from the log.
                {
                    const FusionAhrsSettings& fs = I.ahrsCfg[SEG_Pelvis];
                    ss << "--- fusion (xio AHRS, seg[0] Pelvis): gain="
                       << std::setprecision(2) << fs.gain
                       << " gyroRange=" << std::setprecision(0) << fs.gyroscopeRange
                       << " accelRej=" << std::setprecision(1) << fs.accelerationRejection << "°"
                       << " magRej=" << fs.magneticRejection << "°"
                       << " recoveryTrig=" << fs.recoveryTriggerPeriod << " ticks ---\n";
                }
                if (I.accNormActive || I.magNormActive || I.gyrBiasActive
                    || I.s2sActive) {
                    ss << std::setprecision(4);
                    for (int i = 0; i < kXsensSegmentCount; ++i) {
                        ss << "  cal[" << std::setw(2) << i << "] "
                           << std::left << std::setw(14) << kSegmentNames[i]
                           << std::right
                           << "  accMagn="  << std::setw(7) << I.accMagn[i]
                           << "  magMagn="  << std::setw(7) << I.magMagn[i]
                           << "  gyrBias=(" << std::setw(6) << I.gyrBias[i].x()
                                            << "," << std::setw(6) << I.gyrBias[i].y()
                                            << "," << std::setw(6) << I.gyrBias[i].z()
                           << ")  s2s=("    << std::setw(6) << I.s2s[i].w
                                            << "," << std::setw(6) << I.s2s[i].x
                                            << "," << std::setw(6) << I.s2s[i].y
                                            << "," << std::setw(6) << I.s2s[i].z
                           << ")\n";
                    }
                }
                // L/R pair deltas kept so asymmetry still pops out at a
                // glance — now printed at the end of the full snapshot.
                ss << std::setprecision(2);
                ss << "--- L/R pair Δ (|angle|): ---\n";
                struct Pair { const char* lbl; int lSeg; int rSeg; };
                const Pair pairs[] = {
                    { "shoulder",   SEG_LShoulder,   SEG_RShoulder   },
                    { "upper_arm",  SEG_LUpperArm,   SEG_RUpperArm   },
                    { "forearm",    SEG_LForearm,    SEG_RForearm    },
                    { "hand",       SEG_LHand,       SEG_RHand       },
                    { "upper_leg",  SEG_LUpperLeg,   SEG_RUpperLeg   },
                    { "lower_leg",  SEG_LLowerLeg,   SEG_RLowerLeg   },
                    { "foot",       SEG_LFoot,       SEG_RFoot       },
                };
                for (const auto& p : pairs) {
                    ss << "  " << std::left << std::setw(10) << p.lbl
                       << std::right
                       << "  L=" << std::setw(7) << quat_angle_deg(staging.quat[p.lSeg])
                       << "°  R=" << std::setw(7) << quat_angle_deg(staging.quat[p.rSeg])
                       << "°  ΔLR=" << (quat_angle_deg(staging.quat[p.lSeg])
                                      - quat_angle_deg(staging.quat[p.rSeg]))
                       << "°\n";
                }
                ss << "  " << std::left << std::setw(10) << "spine"
                   << std::right
                   << "  pelvis=" << std::setw(7) << quat_angle_deg(staging.quat[SEG_Pelvis])
                   << "°  t8=" << std::setw(7) << quat_angle_deg(staging.quat[SEG_T8])
                   << "°  head=" << std::setw(7) << quat_angle_deg(staging.quat[SEG_Head])
                   << "°\n";
                ss << "============================================================\n";
                std::cout << ss.str();
                std::cout.flush();
                I.lastDump = now;
            }
        } else {
            if (I.status.load() == (int)ConnStatus::Streaming &&
                (now - I.lastPacket) > kStaleSeconds)
            {
                I.setStatus(ConnStatus::Stale,
                            "no data for " + QString::number(kStaleSeconds, 'f', 1) + "s",
                            this);
            }
            QThread::msleep(2);         // avoid 100% CPU when idle
        }
    }

    // ---- Teardown ------------------------------------------------------
    for (void* d : trackerHandles) api.deviceGotoConfig(d);
    api.controlClose(control);
    if (api.controlDestruct) api.controlDestruct(control);
    unloadApi(api);
    I.setStatus(ConnStatus::NotInitialized, "closed", this);
}

// ============================================================================
//  Localisation — RU / EN dictionary + runtime switch
// ============================================================================

Lang& Lang::instance() { static Lang g; return g; }

struct Tr { const char* key; const char* ru; const char* en; };
static const Tr kTr[] = {
    {"app_title",          "Fox-Mocap — Beta 0.1",              "Fox-Mocap — Beta 0.1"},
    {"start_new_session",  "Начать новую сессию",               "Start new session"},
    {"welcome_sub",        "Интерактивная mocap-студия для костюмов Xsens Link и Awinda",
                           "Interactive mocap studio for the Xsens Link and Awinda suits"},
    {"language",           "Язык",                              "Language"},
    {"back",               "Назад",                             "Back"},
    {"continue",           "Продолжить",                        "Continue"},
    {"cancel",             "Отмена",                            "Cancel"},
    {"finish",             "Готово — открыть сцену",            "Finish — open scene"},
    {"mode_title",         "Выберите режим и подключите оборудование",
                           "Choose mode and connect hardware"},
    {"suit_only",          "Только костюм (17 датчиков)",       "Suit only (17 sensors)"},
    {"suit_with_gloves",   "Костюм + перчатки Manus",           "Suit + Manus gloves"},
    {"connect_suit",       "Подключить костюм",                 "Connect suit"},
    {"disconnect_suit",    "Повторить поиск",                   "Retry scan"},
    {"connect_gloves",     "Подключить перчатки",               "Connect gloves"},
    {"gloves_ready",       "перчатки подключены",               "gloves connected"},
    {"gloves_missing",     "ManusSDK.dll не найден или не отвечает",
                           "ManusSDK.dll not found or not responding"},
    {"gloves_no_core",     "ManusCore не запущен",              "ManusCore is not running"},
    {"gloves_no_device",   "ManusCore запущен, но перчатки не обнаружены",
                           "ManusCore running, no gloves detected"},
    {"need_suit_to_continue", "Подключите костюм, чтобы продолжить.",
                           "Connect the suit to continue."},
    {"need_gloves_to_continue", "Подключите перчатки, чтобы продолжить.",
                           "Connect the gloves to continue."},
    {"mode_hint_ok",       "Оборудование готово — можно продолжать.",
                           "Hardware is ready — you can continue."},
    {"dims_title",         "Размеры актёра",                    "Actor dimensions"},
    {"dims_hint",          "Остальные длины костей считаются по росту (антропометрия hipose / XESNSE).",
                           "Other bone lengths are computed from height (hipose / XESNSE anthropometry)."},
    {"dims_type_hint",     "Введите значение с клавиатуры или воспользуйтесь стрелками.",
                           "Type a value from the keyboard or use the arrow buttons."},
    {"body_height",        "Рост",                              "Body height"},
    {"foot_length",        "Длина стопы",                       "Foot length"},
    {"body_arm_span",      "Размах",                            "Arm span"},
    {"body_leg_length",    "Длина ноги",                        "Leg length"},
    {"body_panel_label",   "Размеры (см)",                      "Body sizes (cm)"},
    {"body_panel_sub",     "0 = расчёт по росту",               "0 = derived from height"},
    {"dims_primary",       "Основные размеры",                  "Primary dimensions"},
    {"dims_breakdown",     "Расчётные длины сегментов",         "Derived segment lengths"},
    {"calib_pose_box",     "Поза актёра",                       "Actor pose"},
    {"calib_status_box",   "Статус",                            "Status"},
    {"calib_progress_box", "Прогресс калибровки",               "Calibration progress"},
    {"bk_trunk",           "Торс",                              "Trunk"},
    {"bk_upper_arm",       "Плечо",                             "Upper arm"},
    {"bk_forearm",         "Предплечье",                        "Forearm"},
    {"bk_thigh",           "Бедро",                             "Thigh"},
    {"bk_shin",            "Голень",                            "Shin"},
    {"bk_foot",            "Стопа",                             "Foot"},
    {"bk_hip",             "Ширина таза",                       "Hip width"},
    {"bk_shoulder",        "Ширина плеч",                       "Shoulder width"},
    {"bk_trunk_len",       "Длина туловища",                    "Trunk length"},
    {"body_hip_width",     "Ширина таза",                       "Hip width"},
    {"body_shoulder_width","Ширина плеч",                       "Shoulder width"},
    {"body_trunk_length",  "Длина туловища",                    "Trunk length"},
    {"calib_title",        "Калибровка",                        "Calibration"},
    {"tpose",              "T-поза",                            "T-Pose"},
    {"npose",              "N-поза",                            "N-Pose"},
    {"tpose_hint",         "Встаньте в T-позу: руки горизонтально, ладони вниз.",
                           "Stand in T-Pose: arms horizontal, palms down."},
    {"npose_hint",         "Встаньте в N-позу: руки по бокам, ладони к телу.",
                           "Stand in N-Pose: arms at sides, palms inward."},
    {"start_calib",        "Начать калибровку",                 "Start calibration"},
    {"prepare",            "Приготовьтесь",                     "Prepare"},
    {"readiness",          "Готовность",                        "Readiness"},
    {"calib_t_prepare",    "Приготовьтесь к T-позе (закрепите sensors крепко на ногах и руках)…", "Prepare for T-pose (fasten sensors firmly)…"},
    {"calib_t_capture",    "T-поза — не двигайтесь",            "T-pose — hold still"},
    {"calib_n_prepare",    "Теперь N-поза — приготовьтесь (12с неподвижно)…", "Now N-pose — prepare (12s still)…"},
    {"calib_n_capture",    "N-поза — не двигайтесь 12с",        "N-pose — hold still 12s"},
    {"calib_k_prepare",    "Теперь K-поза: сядьте на стул (бёдра горизонтально) + руки прямо вперёд — приготовьтесь…", "Now K-pose: sit on chair (thighs horizontal) + arms forward — prepare…"},
    {"calib_k_capture",    "K-поза — сидя, руки вперёд, не двигайтесь", "K-pose — sitting, arms forward, hold still"},
    {"calib_pose_empty",   "Калибровка позы не получила стабильных данных (актёр двигался или сенсоры не передают). Качество будет низким — рекомендуется повторить калибровку.", "Pose calibration captured no stable data (actor moved or sensors not streaming). Quality will be poor — recommend recalibrating."},
    {"still",              "СТОИТЕ СПОКОЙНО",                   "STILL"},
    {"moving",             "ДВИЖЕНИЕ",                          "MOVING"},
    {"suit_connected",     "костюм подключён",                  "suit connected"},
    {"suit_disconnected",  "костюм не подключён",               "suit disconnected"},
    {"suit_scanning",      "поиск костюма…",                    "scanning for suit…"},
    {"suit_connecting",    "подключение…",                      "connecting…"},
    {"suit_nodriver",      "нет XDA-драйвера (dll)",            "XDA driver missing (dll)"},
    {"suit_nodevice",      "оборудование не найдено",           "hardware not detected"},
    {"suit_stale",         "сигнал потерян",                    "signal lost"},
    {"waiting_for_suit",   "Калибровка будет доступна после подключения костюма.",
                           "Calibration will unlock once the suit is connected."},
    {"ready_title",        "Всё готово",                        "Everything is ready"},
    {"ready_summary",      "Костюм подключён, калибровка применена. Нажмите «%1», "
                           "чтобы открыть сцену и начать захват движений.",
                           "Suit is connected, calibration applied. Press “%1” to "
                           "open the scene and begin capture."},
    {"mode_label",         "Режим",                             "Mode"},
    {"sensors_label",      "Датчики костюма",                   "Suit sensors"},
    {"sensors_sub",        "17 IMU-трекеров · левая / правая сторона",
                           "17 IMU trackers · left / right side"},
    {"conn_sub",           "XDA-поток · 240 Гц",                "XDA stream · 240 Hz"},
    {"fingers_label",      "Пальцы",                            "Fingers"},
    {"fingers_sub",        "10 пальцев · 2 перчатки Manus",     "10 fingers · 2 Manus gloves"},
    {"fps_label",          "Частота потока",                    "Stream rate"},
    {"session_label",      "Сессия",                            "Session"},
    {"session_running",    "идёт",                              "running"},
    {"session_paused",     "на паузе",                          "paused"},
    {"pause",              "⏸  Пауза",                          "⏸  Pause"},
    {"resume",             "▶  Продолжить",                     "▶  Resume"},
    {"motion_hint",        "Движение:",                         "Motion:"},
    {"tab_live",           "\xF0\x9F\x93\xA1  Live",             "\xF0\x9F\x93\xA1  Live"},
    {"tab_record",         "\xF0\x9F\x94\xB4  Запись",            "\xF0\x9F\x94\xB4  Record"},
    {"tab_settings",       "\xE2\x9A\x99\xEF\xB8\x8F  Настройки", "\xE2\x9A\x99\xEF\xB8\x8F  Settings"},

    // --- Joint-orientation settings window ---
    {"js_title",           "Коррекция ориентации суставов",      "Joint orientation correction"},
    {"js_intro",           "Подправьте ориентацию любого сустава по осям X/Y/Z. "
                           "Изменения видны на скелете сразу и применяются к стриму и записи. "
                           "Сохранённый пресет загружается как дефолтный при следующем запуске.",
                           "Fine-tune any joint's orientation along X/Y/Z. Changes show on the "
                           "skeleton instantly and apply to the stream and recording too. A saved "
                           "preset loads as the default on the next launch."},
    {"js_axis_x",          "X",                                  "X"},
    {"js_axis_y",          "Y",                                  "Y"},
    {"js_axis_z",          "Z",                                  "Z"},
    {"js_save",            "Сохранить пресет",                   "Save preset"},
    {"js_load",            "Загрузить пресет",                   "Load preset"},
    {"js_reset",           "Сброс",                              "Reset"},
    {"js_saved",           "Пресет сохранён",                    "Preset saved"},
    {"js_save_err",        "Не удалось сохранить пресет",         "Failed to save preset"},
    {"js_loaded",          "Пресет загружен",                    "Preset loaded"},
    {"js_load_err",        "Пресет не найден",                   "No preset found"},
    {"js_reset_done",      "Все поправки обнулены",              "All offsets cleared"},
    {"js_grp_torso",       "Корпус и голова",                    "Torso & head"},
    {"js_grp_rarm",        "Правая рука",                        "Right arm"},
    {"js_grp_larm",        "Левая рука",                         "Left arm"},
    {"js_grp_rleg",        "Правая нога",                        "Right leg"},
    {"js_grp_lleg",        "Левая нога",                         "Left leg"},
    {"js_pelvis",          "Таз",                                "Pelvis"},
    {"js_l5",              "Поясница (L5)",                      "Lower back (L5)"},
    {"js_l3",              "Поясница (L3)",                      "Lower back (L3)"},
    {"js_t12",             "Грудной отдел (T12)",                "Mid spine (T12)"},
    {"js_t8",              "Грудной отдел (T8)",                 "Upper spine (T8)"},
    {"js_neck",            "Шея",                                "Neck"},
    {"js_head",            "Голова",                             "Head"},
    {"js_r_shoulder",      "Правое плечо",                       "Right shoulder"},
    {"js_r_upper_arm",     "Правое предплечье (верх)",           "Right upper arm"},
    {"js_r_forearm",       "Правое предплечье",                  "Right forearm"},
    {"js_r_hand",          "Правая кисть",                       "Right hand"},
    {"js_l_shoulder",      "Левое плечо",                        "Left shoulder"},
    {"js_l_upper_arm",     "Левое предплечье (верх)",            "Left upper arm"},
    {"js_l_forearm",       "Левое предплечье",                   "Left forearm"},
    {"js_l_hand",          "Левая кисть",                        "Left hand"},
    {"js_r_upper_leg",     "Правое бедро",                       "Right thigh"},
    {"js_r_lower_leg",     "Правая голень",                      "Right shin"},
    {"js_r_foot",          "Правая стопа",                       "Right foot"},
    {"js_r_toe",           "Правый носок",                       "Right toe"},
    {"js_l_upper_leg",     "Левое бедро",                        "Left thigh"},
    {"js_l_lower_leg",     "Левая голень",                       "Left shin"},
    {"js_l_foot",          "Левая стопа",                        "Left foot"},
    {"js_l_toe",           "Левый носок",                        "Left toe"},
    {"menu_start_stream",  "Начать трансляцию",                 "Start streaming"},
    {"menu_camera",        "Камера",                            "Camera"},
    {"menu_view_opts",     "Параметры отображения",             "View options"},
    {"menu_start_record",  "Начать запись",                     "Start recording"},
    {"menu_stop_record",   "Остановить запись",                 "Stop recording"},
    {"menu_open_folder",   "Открыть папку записей",             "Open recordings folder"},
    {"menu_record_settings","Параметры записи",                 "Recording settings"},
    {"reset_coords",       "Сбросить положение",                "Reset position"},
    {"freeze_coords",      "Заморозить координаты",             "Freeze coordinates"},
    {"unfreeze_coords",    "Разморозить координаты",            "Unfreeze coordinates"},
    {"coords_frozen",      "Координаты заморожены",             "Coordinates frozen"},
    {"coords_unfrozen",    "Координаты разморожены",            "Coordinates unfrozen"},
    {"battery_label",      "Батарея:",                          "Battery:"},
    {"fng_thumb",          "большой",                           "thumb"},
    {"fng_index",          "указательный",                      "index"},
    {"fng_middle",         "средний",                           "middle"},
    {"fng_ring",           "безымянный",                        "ring"},
    {"fng_pinky",          "мизинец",                           "pinky"},

    // --- Record wizard ---
    {"rec_wiz_title",      "Запись сессии",                     "Record session"},
    {"rec_pick_format",    "Формат и качество",                 "Format and quality"},
    {"rec_format",         "Формат файла",                      "File format"},
    {"rec_quality",        "Качество",                          "Quality"},
    {"rec_quality_normal", "Normal",                            "Normal"},
    {"rec_quality_hd",     "HD Post-processing",                "HD post-processing"},
    {"rec_pick_fps",       "Частота кадров",                    "Frame rate"},
    {"rec_fps",            "Кадров в секунду",                  "Frames per second"},
    {"rec_ready",          "Готовы к записи",                   "Ready to record"},
    {"rec_ready_hint",     "Нажмите «Старт записи», чтобы начать. "
                           "Остановить можно из окна записи во вьюпорте.",
                           "Press \"Start recording\" to begin. Stop from the HUD "
                           "inside the viewport."},
    {"rec_start",          "\xE2\x8F\xBA  Старт записи",          "\xE2\x8F\xBA  Start recording"},
    {"rec_stop",           "\xE2\x8F\xB9  Стоп",                   "\xE2\x8F\xB9  Stop"},
    {"rec_frames",         "кадр.",                             "frames"},
    {"rec_save_title",     "Сохранить запись",                  "Save recording"},
    {"rec_save_failed",    "Не удалось сохранить файл.",        "Failed to save file."},
    {"rec_save_ok",        "Запись сохранена",                  "Recording saved"},
    {"rec_take_kept",      "Запись не сохранена — дубль остался в памяти. Нажмите «Стоп» ещё раз, чтобы сохранить, или закройте окно, чтобы отменить.", "Recording not saved — the take is kept in memory. Press Stop again to save, or close the window to discard."},
    {"rec_unsaved",        "● НЕ СОХРАНЕНО — Стоп",             "● UNSAVED — Stop"},
    {"rec_hd_progress",    "Обработка HD post-processing…",     "Running HD post-processing…"},
    {"rec_close_prompt",   "Идёт запись. Сохранить перед закрытием?", "Recording in progress. Save before closing?"},

    // --- Live-stream wizard ---
    {"live_wiz_title",     "Live-трансляция",                   "Live streaming"},
    {"live_target",        "Плагин-приёмник",                   "Target plugin"},
    {"live_frame_ty",      "Тип кадра MXTP",                    "MXTP frame type"},
    {"live_host",          "Адрес",                             "Host"},
    {"live_port",          "Порт",                              "Port"},
    {"live_start",         "\xF0\x9F\x9A\x80  Запустить трансляцию","\xF0\x9F\x9A\x80  Start streaming"},
    {"live_stop",          "\xE2\x8F\xB9  Остановить трансляцию", "\xE2\x8F\xB9  Stop streaming"},
    {"live_running",       "идёт трансляция",                   "streaming"},
    {"live_stopped",       "остановлена",                       "stopped"},
    {"live_err_bind",      "Не удалось открыть UDP-сокет.",     "UDP socket open failed."},
    {"live_fps",           "Кадров в секунду",                  "Frames per second"},

    {"sns_pelvis",         "таз",                               "pelvis"},
    {"sns_t8",             "грудь",                             "chest"},
    {"sns_head",           "голова",                            "head"},
    {"sns_r_shoulder",     "правое плечо",                      "r shoulder"},
    {"sns_r_upper_arm",    "правое плечо",                      "r upper arm"},
    {"sns_r_forearm",      "правое предплечье",                 "r forearm"},
    {"sns_r_hand",         "правая кисть",                      "r hand"},
    {"sns_l_shoulder",     "левое плечо",                       "l shoulder"},
    {"sns_l_upper_arm",    "левое плечо",                       "l upper arm"},
    {"sns_l_forearm",      "левое предплечье",                  "l forearm"},
    {"sns_l_hand",         "левая кисть",                       "l hand"},
    {"sns_r_upper_leg",    "правое бедро",                      "r thigh"},
    {"sns_r_lower_leg",    "правая голень",                     "r shin"},
    {"sns_r_foot",         "правая стопа",                      "r foot"},
    {"sns_l_upper_leg",    "левое бедро",                       "l thigh"},
    {"sns_l_lower_leg",    "левая голень",                      "l shin"},
    {"sns_l_foot",         "левая стопа",                       "l foot"},

    // Transport / suit selectors (combo items keep the leading emoji).
    {"transport_com",      "\xF0\x9F\x94\x8C  COM-порт — USB / Awinda dongle",
                           "\xF0\x9F\x94\x8C  COM port — USB / Awinda dongle"},
    {"transport_wifi",     "\xF0\x9F\x93\xA1  WiFi / Ethernet — по сети (Link / Awinda)",
                           "\xF0\x9F\x93\xA1  WiFi / Ethernet — network (Link / Awinda)"},
    {"suit_awinda",        "\xF0\x9F\x9F\xA0  Xsens Awinda — 60 Гц",
                           "\xF0\x9F\x9F\xA0  Xsens Awinda — 60 Hz"},
    {"suit_link",          "\xF0\x9F\x9F\xA3  Xsens Link — 240 Гц",
                           "\xF0\x9F\x9F\xA3  Xsens Link — 240 Hz"},

    // WiFi connection hint (the PC must already be on the suit's network).
    {"wifi_hint",          "Подключите этот ПК к той же Wi-Fi сети, что и костюм "
                           "(через настройки Wi-Fi Windows), затем нажмите «Подключить».",
                           "Connect this PC to the same Wi-Fi network as the suit "
                           "(via Windows Wi-Fi settings), then press Connect."},

    // K-pose seated calibration hint.
    {"kpose_hint",         "K-поза: сядьте на стул (бёдра горизонтально, колени 90°) + руки прямо вперёд горизонтально",
                           "K-Pose: sit on a chair (thighs horizontal, knees 90°) + arms straight forward, horizontal"},

    // Calibration status messages (%1 = seconds / index, %2 = total).
    {"calib_tpose_tune",     "T-поза — точная настройка: стойте %1с неподвижно",
                             "T-Pose — fine-tuning: stand still for %1 s"},
    {"calib_tpose_converge", "T-поза: сходимость фильтра %1с — стойте",
                             "T-Pose: filter converging %1 s — hold still"},
    {"calib_tpose_capture",  "T-поза: захват %1/%2 — стойте",
                             "T-Pose: capture %1/%2 — hold still"},
    {"calib_kpose_applied",  "K-калибровка применена — фильтр стабилизируется %1с…",
                             "K-calibration applied — filter stabilizing %1 s…"},
    {"calib_kpose_converge", "K-калибровка: фильтр %1с — не двигайтесь",
                             "K-calibration: filter %1 s — don't move"},
    {"calib_npose_settle",   "Стойте неподвижно %1с — фильтр настраивается…",
                             "Stand still for %1 s — filter tuning…"},
    {"calib_npose_converge", "Сходимость фильтра: %1с осталось — не двигайтесь",
                             "Filter convergence: %1 s left — don't move"},
    {"calib_npose_capture",  "Захват %1/%2 — продолжайте неподвижно",
                             "Capture %1/%2 — keep still"},
    {"calib_ok",             "OK",                              "OK"},
};

void Lang::setLanguage(Code c) { if (c == m_code) return; m_code = c; emit changed(); }

QString Lang::t(const char* key)
{
    const Code c = instance().current();
    for (const Tr& e : kTr)
        if (std::strcmp(e.key, key) == 0)
            return QString::fromUtf8(c == RU ? e.ru : e.en);
    return QString::fromLatin1(key);           // key as fallback
}

// Helper: render a connection dot (green = live, red = off, grey = inactive).
static void paintDot(QLabel* lab, const char* colorHex)
{
    lab->setFixedSize(14, 14);
    lab->setStyleSheet(QString("background:%1; border-radius:7px;").arg(colorHex));
}

// Procedurally-drawn flag icons (no external assets).  28×20 px with a
// 1-px dark border + rounded 3-px radius so they sit cleanly inside the
// QComboBox list even on a black background.
static QIcon makeFlagIcon(const char* code)
{
    const int W = 28, H = 20;
    QPixmap pm(W, H);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing, true);

    QPainterPath clip;
    clip.addRoundedRect(QRectF(0.5, 0.5, W - 1, H - 1), 3.0, 3.0);
    p.setClipPath(clip);

    if (std::strcmp(code, "RU") == 0) {
        p.fillRect(QRectF(0, 0,          W, H / 3.0), QColor("#FFFFFF"));
        p.fillRect(QRectF(0, H / 3.0,    W, H / 3.0), QColor("#0039A6"));
        p.fillRect(QRectF(0, 2*H / 3.0,  W, H / 3.0), QColor("#D52B1E"));
    } else {                                        // EN / UK
        p.fillRect(QRectF(0, 0, W, H), QColor("#012169"));    // navy blue
        QPen wide(QColor("#FFFFFF"));  wide.setWidth(5);
        QPen thin(QColor("#C8102E"));  thin.setWidth(2);
        // diagonals (saltire): two crossings corner-to-corner
        p.setPen(wide);
        p.drawLine(0, 0, W, H);
        p.drawLine(W, 0, 0, H);
        p.setPen(thin);
        p.drawLine(0, 0, W, H);
        p.drawLine(W, 0, 0, H);
        // horizontal + vertical white cross
        wide.setWidth(6); p.setPen(wide);
        p.drawLine(W / 2, 0, W / 2, H);
        p.drawLine(0, H / 2, W, H / 2);
        // red cross on top
        thin.setWidth(3); p.setPen(thin);
        p.drawLine(W / 2, 0, W / 2, H);
        p.drawLine(0, H / 2, W, H / 2);
    }
    // subtle outline so the flag reads on the dark background.
    p.setClipping(false);
    p.setPen(QPen(QColor(0, 0, 0, 180), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(QRectF(0.5, 0.5, W - 1, H - 1), 3.0, 3.0);
    return QIcon(pm);
}

// ============================================================================
//  NewSessionWizard
// ============================================================================

NewSessionWizard::NewSessionWizard(MocapReceiver* rx, bool testMode, QWidget* parent)
    : QDialog(parent), m_rx(rx), m_test(testMode)
{
    setModal(true);
    setWindowTitle(Lang::t("app_title"));
    setMinimumSize(760, 640);

    buildPages();

    m_btnBack = new QPushButton(this);
    m_btnNext = new QPushButton(this);
    m_btnNext->setObjectName("primary");
    connect(m_btnBack, &QPushButton::clicked, this, &NewSessionWizard::goBack);
    connect(m_btnNext, &QPushButton::clicked, this, &NewSessionWizard::goNext);

    auto* nav = new QHBoxLayout();
    nav->addWidget(m_btnBack);
    nav->addStretch();
    nav->addWidget(m_btnNext);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(24, 24, 24, 18);
    outer->addWidget(m_pages, 1);
    outer->addLayout(nav);

    // Poll the receiver's connection status a few times a second so the
    // calibration page's badge + button state track reality automatically.
    m_statusTimer.setInterval(250);
    connect(&m_statusTimer, &QTimer::timeout, this, &NewSessionWizard::onStatusTick);
    m_statusTimer.start();

    m_countTimer.setInterval(100);
    m_captureTimer.setInterval(10);
    connect(&m_countTimer,   &QTimer::timeout, this, &NewSessionWizard::onCountdownTick);
    connect(&m_captureTimer, &QTimer::timeout, this, &NewSessionWizard::onCaptureTick);

    connect(&Lang::instance(), &Lang::changed, this, &NewSessionWizard::retranslate);

    retranslate();
    onModeChanged();          // sets initial visibility of glove row
    onStatusTick();           // paint initial badges
    updateNavButtons();

    // -------- Autopilot for -test ---------------------------------------
    // Drives the full wizard without a human: page 0 → click Continue → on
    // page 1 click Connect suit, wait until Streaming, Continue → page 2
    // Continue → page 3 triggers its own auto-click on Start calibration.
    if (m_test) {
        auto* pilot = new QTimer(this);
        pilot->setInterval(500);
        connect(pilot, &QTimer::timeout, this, [this, pilot]() {
            static bool suitKicked   = false;
            static bool gloveKicked  = false;
            switch (m_pageIdx) {
                case 0:
                    testLog("[pilot] page 0 → continue", m_test);
                    goNext();
                    break;
                case 1: {
                    const bool needGlv = (m_rbSuitG && m_rbSuitG->isChecked());
                    if (!suitKicked) {
                        suitKicked = true;
                        testLog("[pilot] page 1 → connect suit", m_test);
                        onConnectSuit();
                    }
                    // Try the gloves handshake once the SDK is ready — only
                    // meaningful when the operator launched with --gloves.
                    if (needGlv && !gloveKicked && m_rx) {
                        gloveKicked = true;
                        testLog("[pilot] page 1 → connect gloves", m_test);
                        onConnectGloves();
                    }
                    const bool suitUp  = m_rx && m_rx->isStreaming();
                    const bool gloveUp = m_rx && m_rx->glovesReady();
                    if (suitUp && (!needGlv || gloveUp)) {
                        testLog("[pilot] suit+gloves ready → continue", m_test);
                        goNext();
                    } else if (needGlv && suitUp && !gloveUp) {
                        // Hold on the Mode page with a breadcrumb so the
                        // operator can see exactly what's missing.
                        testLog("[pilot] waiting for gloves — core="
                                + std::string(m_rx->glovesCoreReady() ? "up" : "down")
                                + " count=" + std::to_string(
                                    m_rx->glovesReady() ? 1 : 0), m_test);
                    }
                    break;
                }
                case 2:
                    testLog("[pilot] page 2 (dims) → continue", m_test);
                    goNext();
                    break;
                case 3:
                    pilot->stop();
                    testLog("[pilot] reached calibration page — handing over", m_test);
                    break;
                default:
                    pilot->stop();
                    break;
            }
        });
        pilot->start();
    }
}

void NewSessionWizard::buildPages()
{
    m_pages = new QStackedWidget(this);

    // ---------- Page 1 : Welcome -------------------------------------------
    {
        auto* p = new QWidget();
        m_welcomeImg = new QLabel(p);
        m_welcomeImg->setAlignment(Qt::AlignCenter);
        {
            QPixmap pm(":/img/newsession.png");
            if (!pm.isNull())
                m_welcomeImg->setPixmap(pm.scaledToWidth(560,
                    Qt::SmoothTransformation));
        }
        m_welcomeHeading = new QLabel(p);
        m_welcomeHeading->setObjectName("heroHeading");
        m_welcomeHeading->setAlignment(Qt::AlignCenter);

        m_welcomeSub = new QLabel(p);
        m_welcomeSub->setObjectName("subtle");
        m_welcomeSub->setAlignment(Qt::AlignCenter);
        m_welcomeSub->setWordWrap(true);

        auto* langLabel = new QLabel(p);
        langLabel->setObjectName("subtle");
        m_langCombo = new QComboBox(p);
        m_langCombo->setObjectName("langCombo");
        m_langCombo->setCursor(Qt::PointingHandCursor);
        m_langCombo->setIconSize(QSize(28, 20));
        m_langCombo->setMinimumHeight(40);
        m_langCombo->setMinimumWidth(200);
        m_langCombo->addItem(makeFlagIcon("RU"), "  Русский",  int(Lang::RU));
        m_langCombo->addItem(makeFlagIcon("EN"), "  English",  int(Lang::EN));
        m_langCombo->setCurrentIndex(Lang::instance().current() == Lang::RU ? 0 : 1);
        connect(m_langCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &NewSessionWizard::onLanguageChanged);

        auto* langRow = new QHBoxLayout();
        langRow->addStretch();
        langRow->addWidget(langLabel);
        langRow->addWidget(m_langCombo);
        langRow->addStretch();

        m_btnStart = new QPushButton(p);
        m_btnStart->setObjectName("hero");
        connect(m_btnStart, &QPushButton::clicked, this, &NewSessionWizard::goNext);

        auto* lay = new QVBoxLayout(p);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->addStretch(1);
        lay->addWidget(m_welcomeImg);
        lay->addSpacing(14);
        lay->addWidget(m_welcomeHeading);
        lay->addWidget(m_welcomeSub);
        lay->addSpacing(18);
        lay->addLayout(langRow);
        lay->addSpacing(24);
        lay->addWidget(m_btnStart, 0, Qt::AlignHCenter);
        lay->addStretch(2);

        langLabel->setProperty("isLangLabel", true);    // for retranslate()
        m_pages->addWidget(p);
    }

    // ---------- Page 2 : Mode + Hardware connect ---------------------------
    {
        auto* p = new QWidget();
        m_modeTitle = new QLabel(p);
        m_modeTitle->setObjectName("heroHeading");
        m_modeTitle->setAlignment(Qt::AlignCenter);

        m_rbSuit  = new QRadioButton(p);
        m_rbSuitG = new QRadioButton(p);
        m_rbSuit->setChecked(true);
        m_rbSuit->setObjectName("bigRadio");
        m_rbSuitG->setObjectName("bigRadio");
        connect(m_rbSuit,  &QRadioButton::toggled, this, &NewSessionWizard::onModeChanged);
        connect(m_rbSuitG, &QRadioButton::toggled, this, &NewSessionWizard::onModeChanged);

        // Transport selector: COM port or WiFi.
        m_cbxTransport = new QComboBox(p);
        m_cbxTransport->addItem(Lang::t("transport_com"));
        m_cbxTransport->addItem(Lang::t("transport_wifi"));
        m_cbxTransport->setMinimumHeight(36);
        m_cbxTransport->setMinimumWidth(360);
        m_cbxTransport->setStyleSheet(
            "QComboBox { background:#1b1b1b; color:#eee; border:1px solid #3a3a3a;"
            " border-radius:8px; padding:6px 12px; font-weight:600; }"
            "QComboBox::drop-down { border:0; width:26px; }"
            "QComboBox QAbstractItemView { background:#1b1b1b; color:#eee;"
            " selection-background-color:#FF7A1A; selection-color:#000; }");

        // Suit family selector — drives the whole-system update rate.  Item
        // order matches SuitType (0 = Awinda, 1 = Link).
        m_cbxSuit = new QComboBox(p);
        m_cbxSuit->addItem(Lang::t("suit_awinda"), int(SuitType::Awinda));
        m_cbxSuit->addItem(Lang::t("suit_link"),   int(SuitType::Link));
        m_cbxSuit->setCurrentIndex(m_result.suit == SuitType::Link ? 1 : 0);
        m_cbxSuit->setMinimumHeight(36);
        m_cbxSuit->setMinimumWidth(360);
        m_cbxSuit->setStyleSheet(m_cbxTransport->styleSheet());
        connect(m_cbxSuit, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, [this](int idx) {
            const SuitType s = (idx == 1) ? SuitType::Link : SuitType::Awinda;
            m_result.suit = s;
            if (m_rx) m_rx->setExpectedRate(nativeRateHz(s));
            // Convenience default: Link ships over WiFi, Awinda over the dongle.
            if (m_cbxTransport) m_cbxTransport->setCurrentIndex(s == SuitType::Link ? 1 : 0);
        });

        // WiFi hint (shown only in WiFi mode).  XDA discovers the suit over the
        // network — it does NOT join Wi-Fi — so the PC must already be on the
        // same access point as the suit (joined via Windows).  No SSID/password.
        m_wifiHint = new QLabel(p);
        m_wifiHint->setWordWrap(true);
        m_wifiHint->setAlignment(Qt::AlignCenter);
        m_wifiHint->setObjectName("subtle");
        m_wifiHint->setText(Lang::t("wifi_hint"));
        m_wifiHint->hide();

        connect(m_cbxTransport, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, [this](int idx) {
            const bool wifi = (idx == 1);
            if (m_wifiHint) m_wifiHint->setVisible(wifi);
            if (m_rx)
                m_rx->setTransport(wifi
                    ? MocapReceiver::Transport::Network
                    : MocapReceiver::Transport::ComPort);
        });

        // Suit connect row
        m_btnConnectSuit = new QPushButton(p);
        m_btnConnectSuit->setObjectName("primary");
        m_btnConnectSuit->setMinimumHeight(38);
        m_btnConnectSuit->setMinimumWidth(220);
        connect(m_btnConnectSuit, &QPushButton::clicked,
                this, &NewSessionWizard::onConnectSuit);
        m_suitDot  = new QLabel(p);
        paintDot(m_suitDot, "#C03838");
        m_suitText = new QLabel(p);
        m_suitText->setStyleSheet("color:#DDDDDD; font-weight:600;");

        auto* suitRow = new QHBoxLayout();
        suitRow->setContentsMargins(0, 0, 0, 0);
        suitRow->addWidget(m_btnConnectSuit);
        suitRow->addSpacing(16);
        suitRow->addWidget(m_suitDot);
        suitRow->addSpacing(6);
        suitRow->addWidget(m_suitText, 1);

        // Glove connect row (only visible in suit+gloves mode)
        m_btnConnectGloves = new QPushButton(p);
        m_btnConnectGloves->setObjectName("primary");
        m_btnConnectGloves->setMinimumHeight(38);
        m_btnConnectGloves->setMinimumWidth(220);
        connect(m_btnConnectGloves, &QPushButton::clicked,
                this, &NewSessionWizard::onConnectGloves);
        m_gloveDot  = new QLabel(p);
        paintDot(m_gloveDot, "#C03838");
        m_gloveText = new QLabel(p);
        m_gloveText->setStyleSheet("color:#DDDDDD; font-weight:600;");

        auto* gloveRow = new QHBoxLayout();
        gloveRow->setContentsMargins(0, 0, 0, 0);
        gloveRow->addWidget(m_btnConnectGloves);
        gloveRow->addSpacing(16);
        gloveRow->addWidget(m_gloveDot);
        gloveRow->addSpacing(6);
        gloveRow->addWidget(m_gloveText, 1);

        m_modeHint = new QLabel(p);
        m_modeHint->setWordWrap(true);
        m_modeHint->setAlignment(Qt::AlignCenter);
        m_modeHint->setObjectName("subtle");

        auto* lay = new QVBoxLayout(p);
        lay->setContentsMargins(60, 20, 60, 20);
        lay->addWidget(m_modeTitle);
        lay->addSpacing(20);
        lay->addWidget(m_rbSuit,  0, Qt::AlignHCenter);
        lay->addSpacing(8);
        lay->addWidget(m_rbSuitG, 0, Qt::AlignHCenter);
        lay->addSpacing(24);
        lay->addWidget(m_cbxSuit, 0, Qt::AlignHCenter);
        lay->addSpacing(8);
        lay->addWidget(m_cbxTransport, 0, Qt::AlignHCenter);
        lay->addSpacing(8);
        lay->addWidget(m_wifiHint, 0, Qt::AlignHCenter);
        lay->addSpacing(16);
        lay->addLayout(suitRow);
        lay->addSpacing(10);
        lay->addLayout(gloveRow);
        lay->addSpacing(20);
        lay->addWidget(m_modeHint);
        lay->addStretch();

        m_pages->addWidget(p);
    }

    // ---------- Page 3 : Dimensions ----------------------------------------
    {
        auto* p = new QWidget();
        m_dimsTitle = new QLabel(p);
        m_dimsTitle->setObjectName("heroHeading");
        m_dimsTitle->setAlignment(Qt::AlignCenter);

        // --- Primary: body height + foot length (MVN/hipose canonical pair)
        // The inputs are fully editable — we install a focus-event hook
        // that selects all text the instant the field is tabbed into, so
        // typing a value overwrites the previous one without manual
        // drag-selection first.
        auto configSpin = [](QDoubleSpinBox* s, double v, double lo,
                             double hi, double step) {
            s->setRange(lo, hi);
            s->setDecimals(1);
            s->setSingleStep(step);
            s->setValue(v);
            s->setSuffix(" cm");
            s->setObjectName("bigSpin");
            s->setMinimumHeight(44);
            s->setMinimumWidth(200);
            s->setAlignment(Qt::AlignCenter);
            s->setKeyboardTracking(true);
            s->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
            s->setCursor(Qt::IBeamCursor);
            // Tooltip tells the user this box accepts typed values too.
            s->setToolTip(Lang::t("dims_type_hint"));
            // Centred text + strong focus policy so keyboard input works
            // without needing to click the line-edit first.
            if (auto* le = s->findChild<QLineEdit*>()) {
                le->setAlignment(Qt::AlignCenter);
                le->setFocusPolicy(Qt::StrongFocus);
            }
        };
        m_height = new QDoubleSpinBox(p);
        configSpin(m_height, 175.0, 100.0, 230.0, 0.5);
        m_foot   = new QDoubleSpinBox(p);
        configSpin(m_foot,    26.0,  15.0,  35.0, 0.5);
        // FIX: размах рук и длина ноги перенесены из вьюпорта в wizard.
        // 0 = «вычислить из роста» (фоллбэк в SkeletonXsens::buildLengths).
        m_arm = new QDoubleSpinBox(p);
        configSpin(m_arm,      0.0,   0.0, 250.0, 0.5);
        m_leg = new QDoubleSpinBox(p);
        configSpin(m_leg,      0.0,   0.0, 130.0, 0.5);
        // FIX issue 5: новые опциональные поля.  0 = вычислить из роста.
        m_hip      = new QDoubleSpinBox(p);
        configSpin(m_hip,      0.0,   0.0,  60.0, 0.5);
        m_shoulder = new QDoubleSpinBox(p);
        configSpin(m_shoulder, 0.0,   0.0,  70.0, 0.5);
        m_trunk    = new QDoubleSpinBox(p);
        configSpin(m_trunk,    0.0,   0.0, 120.0, 0.5);

        // True "select-all-on-focus" — attached via an event filter on
        // each spin's internal QLineEdit.  Needed because QAbstractSpinBox
        // doesn't expose a selectAllOnFocus property.
        struct SelAllFilter : QObject {
            bool eventFilter(QObject* o, QEvent* e) override {
                if (e->type() == QEvent::FocusIn) {
                    if (auto* le = qobject_cast<QLineEdit*>(o))
                        QTimer::singleShot(0, le, &QLineEdit::selectAll);
                }
                return QObject::eventFilter(o, e);
            }
        };
        static SelAllFilter s_selAll;
        for (auto* s : { m_height, m_foot, m_arm, m_leg,
                         m_hip, m_shoulder, m_trunk }) {
            if (auto* le = s->findChild<QLineEdit*>())
                le->installEventFilter(&s_selAll);
        }

        m_lblHeight   = new QLabel(p);
        m_lblFoot     = new QLabel(p);
        m_lblArm      = new QLabel(p);
        m_lblLeg      = new QLabel(p);
        m_lblHip      = new QLabel(p);
        m_lblShoulder = new QLabel(p);
        m_lblTrunk    = new QLabel(p);
        m_lblHeight  ->setStyleSheet("font-weight:600;");
        m_lblFoot    ->setStyleSheet("font-weight:600;");
        m_lblArm     ->setStyleSheet("font-weight:600;");
        m_lblLeg     ->setStyleSheet("font-weight:600;");
        m_lblHip     ->setStyleSheet("font-weight:600;");
        m_lblShoulder->setStyleSheet("font-weight:600;");
        m_lblTrunk   ->setStyleSheet("font-weight:600;");

        auto* primaryBox = new QGroupBox(p);
        primaryBox->setProperty("isPrimaryDims", true);
        auto* primaryLay = new QGridLayout(primaryBox);
        primaryLay->setContentsMargins(24, 20, 24, 20);
        primaryLay->setHorizontalSpacing(32);
        primaryLay->setVerticalSpacing(10);
        primaryLay->addWidget(m_lblHeight, 0, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_height,    0, 1);
        primaryLay->addWidget(m_lblFoot,   1, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_foot,      1, 1);
        primaryLay->addWidget(m_lblArm,    2, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_arm,       2, 1);
        primaryLay->addWidget(m_lblLeg,    3, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_leg,       3, 1);
        // FIX issue 5: hip/shoulder/trunk — три новых ряда.
        primaryLay->addWidget(m_lblHip,      4, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_hip,         4, 1);
        primaryLay->addWidget(m_lblShoulder, 5, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_shoulder,    5, 1);
        primaryLay->addWidget(m_lblTrunk,    6, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_trunk,       6, 1);

        // --- Secondary: anthropometric breakdown (read-only, live update) ---
        // Mirrors the 27 segment_lengths that hipose SkeletonXsens consumes —
        // but surfaces only the 6 biomechanically-meaningful values and derives
        // the rest from standard Drillis-Contini ratios (same that XESNSE's
        // biomech_model.cpp uses as defaults).
        auto* breakdownBox = new QGroupBox(p);
        breakdownBox->setProperty("isBreakdownBox", true);
        auto* bg = new QGridLayout(breakdownBox);
        bg->setContentsMargins(24, 20, 24, 20);
        bg->setHorizontalSpacing(40);
        bg->setVerticalSpacing(6);

        const char* rowKeys[9] = {
            "bk_trunk", "bk_upper_arm", "bk_forearm",
            "bk_thigh", "bk_shin", "bk_foot",
            // FIX issue 5: новые разбивки в breakdown.
            "bk_hip", "bk_shoulder", "bk_trunk_len"
        };
        auto makeLabel = [&](int row, const char* captionKey) {
            auto* cap = new QLabel(p);
            cap->setProperty("isBreakdownCap", true);
            cap->setProperty("bkKey", captionKey);
            cap->setStyleSheet("color:#9B9B9B;");
            auto* val = new QLabel("—", p);
            val->setProperty("isBreakdownVal", true);
            val->setProperty("bkKey", captionKey);
            val->setStyleSheet("color:#FF7A1A; font-weight:700;");
            val->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            bg->addWidget(cap, row, 0);
            bg->addWidget(val, row, 1);
        };
        for (int i = 0; i < 9; ++i) makeLabel(i, rowKeys[i]);

        auto updateBreakdown = [this, p]() {
            const double h  = m_height->value() / 100.0;
            const double fl = m_foot->value()   / 100.0;
            // FIX: учитываем m_arm / m_leg если actor ввёл их (>0).  Та же
            // формула что в SkeletonXsens::buildLengths — armScale = arm
            // per side / (0.44·h), legScale = leg / (0.491·h).
            double armScale = 1.0;
            const double armSpanCm = m_arm ? m_arm->value() : 0.0;
            if (armSpanCm > 0.0) {
                const double bodyWidthM = 0.30 * (h / 1.75);
                const double armPerSideM = std::max(0.10,
                    (armSpanCm / 100.0 - bodyWidthM) * 0.5);
                const double defArmM = 0.44 * h;
                armScale = (defArmM > 1e-6) ? (armPerSideM / defArmM) : 1.0;
            }
            double legScale = 1.0;
            const double legCm = m_leg ? m_leg->value() : 0.0;
            if (legCm > 0.0) {
                const double legPerSideM = std::max(0.20, legCm / 100.0);
                const double defLegM = 0.491 * h;
                legScale = (defLegM > 1e-6) ? (legPerSideM / defLegM) : 1.0;
            }
            // FIX issue 5: hip / shoulder / trunk breakdowns.
            const double trunkScale = h / 1.75;
            const double hipCm = m_hip ? m_hip->value() : 0.0;
            const double hipM   = (hipCm > 0.0) ? std::max(0.05, hipCm / 200.0)
                                                : 0.10 * trunkScale;
            const double shldCm = m_shoulder ? m_shoulder->value() : 0.0;
            const double shldM  = (shldCm > 0.0) ? std::max(0.05, shldCm / 200.0)
                                                 : 0.05 * trunkScale;
            const double trunkCm = m_trunk ? m_trunk->value() : 0.0;
            const double trunkM  = (trunkCm > 0.0) ? std::max(0.30, trunkCm / 100.0)
                                                   : 0.55 * h;
            struct V { const char* k; double m; };
            V vals[9] = {
                { "bk_trunk",     0.288 * h               },
                { "bk_upper_arm", 0.186 * h * armScale    },
                { "bk_forearm",   0.146 * h * armScale    },
                { "bk_thigh",     0.245 * h * legScale    },
                { "bk_shin",      0.246 * h * legScale    },
                { "bk_foot",      fl                      },
                { "bk_hip",       hipM * 2.0              },  // показываем full width
                { "bk_shoulder",  shldM * 2.0             },
                { "bk_trunk_len", trunkM                  },
            };
            for (auto* lab : p->findChildren<QLabel*>()) {
                if (!lab->property("isBreakdownVal").toBool()) continue;
                const QString k = lab->property("bkKey").toString();
                for (auto& v : vals)
                    if (k == v.k) lab->setText(QString::number(v.m * 100.0, 'f', 1) + " cm");
            }
        };
        connect(m_height, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                p, [updateBreakdown](double){ updateBreakdown(); });
        connect(m_foot,   QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                p, [updateBreakdown](double){ updateBreakdown(); });
        connect(m_arm,    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                p, [updateBreakdown](double){ updateBreakdown(); });
        connect(m_leg,    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                p, [updateBreakdown](double){ updateBreakdown(); });
        connect(m_hip,    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                p, [updateBreakdown](double){ updateBreakdown(); });
        connect(m_shoulder, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                p, [updateBreakdown](double){ updateBreakdown(); });
        connect(m_trunk,  QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                p, [updateBreakdown](double){ updateBreakdown(); });
        updateBreakdown();

        m_dimsHint = new QLabel(p);
        m_dimsHint->setObjectName("subtle");
        m_dimsHint->setWordWrap(true);
        m_dimsHint->setAlignment(Qt::AlignCenter);

        // FIX: оборачиваем все 4 спина + breakdown в QScrollArea, чтобы
        // окно мастера не вырастало вертикально при добавлении новых
        // полей — пользователь скроллит контент внутри той же высоты.
        auto* scrollHost = new QWidget(p);
        auto* hostLay    = new QVBoxLayout(scrollHost);
        hostLay->setContentsMargins(0, 0, 0, 0);
        hostLay->setSpacing(12);
        hostLay->addWidget(primaryBox);
        hostLay->addWidget(breakdownBox);
        hostLay->addWidget(m_dimsHint);
        hostLay->addStretch();

        auto* scroll = new QScrollArea(p);
        scroll->setWidget(scrollHost);
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        scroll->setStyleSheet(
            "QScrollArea { background:transparent; }"
            "QScrollArea > QWidget > QWidget { background:transparent; }"
            "QScrollBar:vertical {"
            "  background:#1a1a1a; width:10px; margin:0; border-radius:4px; }"
            "QScrollBar::handle:vertical {"
            "  background:#FF7A1A; min-height:24px; border-radius:4px; }"
            "QScrollBar::handle:vertical:hover { background:#FF8A2A; }"
            "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {"
            "  height:0; }"
            "QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {"
            "  background:transparent; }"
        );

        auto* lay = new QVBoxLayout(p);
        lay->setContentsMargins(40, 20, 40, 20);
        lay->addWidget(m_dimsTitle);
        lay->addSpacing(20);
        lay->addWidget(scroll, 1);

        m_pages->addWidget(p);
    }

    // ---------- Page 4 : Calibration ---------------------------------------
    // Pose illustration in its own framed box up top (so the heading can
    // never visually crash into the image), status / progress column
    // directly underneath.  The text sizes below are all explicit — the
    // previous pass left countLabel at 52 pt which was overwhelming.
    {
        auto* p = new QWidget();
        m_calibTitle = new QLabel(p);
        m_calibTitle->setObjectName("heroHeading");
        m_calibTitle->setAlignment(Qt::AlignCenter);

        auto* imgFrame = new QWidget(p);
        imgFrame->setObjectName("poseFrame");
        m_poseImage = new QLabel(imgFrame);
        m_poseImage->setAlignment(Qt::AlignCenter);
        m_poseImage->setMinimumHeight(320);
        m_poseImage->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        auto* frameLay = new QVBoxLayout(imgFrame);
        frameLay->setContentsMargins(10, 10, 10, 10);
        frameLay->addWidget(m_poseImage);

        m_poseHint = new QLabel(p);
        m_poseHint->setAlignment(Qt::AlignCenter);
        m_poseHint->setWordWrap(true);
        m_poseHint->setStyleSheet(
            "color:#CFCFCF; font-size:11pt; font-weight:500;");

        m_connBadge = new QLabel(p);
        m_connBadge->setAlignment(Qt::AlignCenter);
        setBadge(m_connBadge, Lang::t("suit_scanning"), false);

        m_countLabel = new QLabel("", p);
        m_countLabel->setAlignment(Qt::AlignCenter);
        m_countLabel->setStyleSheet(
            "color:#FF7A1A; font-size:28pt; font-weight:900; letter-spacing:1px;");
        m_countLabel->setMinimumHeight(44);

        m_countdownBar = new QProgressBar(p);
        m_countdownBar->setObjectName("countdownBar");
        m_countdownBar->setRange(0, kCountdownSeconds * 10);
        m_countdownBar->setValue(0);
        m_countdownBar->setFormat("%p %");
        m_countdownBar->setFixedHeight(18);

        m_readyBar = new QProgressBar(p);
        m_readyBar->setObjectName("readyBar");
        m_readyBar->setRange(0, kCalibrationSamples);
        m_readyBar->setValue(0);
        m_readyBar->setFormat("%v / %m");
        m_readyBar->setFixedHeight(18);

        m_stillLabel = new QLabel(p);
        m_stillLabel->setAlignment(Qt::AlignCenter);
        m_stillLabel->setStyleSheet("color:#AAAAAA; font-weight:700;");

        m_calibStatus = new QLabel(p);
        m_calibStatus->setAlignment(Qt::AlignCenter);
        m_calibStatus->setStyleSheet("color:#9B9B9B; font-size:10pt;");

        m_calibQuality = new QLabel(p);
        m_calibQuality->setAlignment(Qt::AlignCenter);
        m_calibQuality->setStyleSheet("color:#9B9B9B; font-size:9pt;");
        m_calibQuality->setWordWrap(true);

        m_btnCalibBegin = new QPushButton(p);
        m_btnCalibBegin->setObjectName("primary");
        m_btnCalibBegin->setMinimumHeight(42);
        m_btnCalibBegin->setMinimumWidth(240);
        connect(m_btnCalibBegin, &QPushButton::clicked,
                this, &NewSessionWizard::onCalibrationBegin);

        auto* lay = new QVBoxLayout(p);
        lay->setContentsMargins(28, 10, 28, 14);
        lay->setSpacing(0);
        lay->addWidget(m_calibTitle);
        lay->addSpacing(10);
        lay->addWidget(imgFrame, 1);
        lay->addSpacing(10);
        lay->addWidget(m_poseHint);
        lay->addSpacing(6);
        lay->addWidget(m_connBadge, 0, Qt::AlignHCenter);
        lay->addSpacing(10);
        lay->addWidget(m_countLabel);
        lay->addSpacing(4);
        lay->addWidget(m_countdownBar);
        lay->addSpacing(4);
        lay->addWidget(m_readyBar);
        lay->addSpacing(6);
        lay->addWidget(m_stillLabel);
        lay->addWidget(m_calibStatus);
        lay->addWidget(m_calibQuality);
        lay->addSpacing(12);
        lay->addWidget(m_btnCalibBegin, 0, Qt::AlignHCenter);

        m_pages->addWidget(p);
    }

    // ---------- Page 5 : Ready ---------------------------------------------
    {
        auto* p = new QWidget();
        m_readyTitle = new QLabel(p);
        m_readyTitle->setObjectName("heroHeading");
        m_readyTitle->setAlignment(Qt::AlignCenter);
        m_readySummary = new QLabel(p);
        m_readySummary->setWordWrap(true);
        m_readySummary->setAlignment(Qt::AlignCenter);
        m_readySummary->setObjectName("subtle");

        m_btnFinish = new QPushButton(p);
        m_btnFinish->setObjectName("hero");
        connect(m_btnFinish, &QPushButton::clicked, this, &QDialog::accept);

        auto* lay = new QVBoxLayout(p);
        lay->addStretch(1);
        lay->addWidget(m_readyTitle);
        lay->addSpacing(18);
        lay->addWidget(m_readySummary);
        lay->addStretch(1);
        lay->addWidget(m_btnFinish, 0, Qt::AlignHCenter);
        lay->addStretch(1);

        m_pages->addWidget(p);
    }

    refreshPoseImage();
}

void NewSessionWizard::refreshPoseImage()
{
    if (!m_poseImage) return;
    const bool nHalf = (m_phase == CalibPhase::PrepN
                     || m_phase == CalibPhase::CaptureN);
    const bool kHalf = (m_phase == CalibPhase::PrepK
                     || m_phase == CalibPhase::CaptureK);
    const char* path = kHalf ? ":/img/kpose.png"
                      : (nHalf ? ":/img/npose.png" : ":/img/tpose.png");
    QPixmap pm(path);
    if (!pm.isNull()) {
        m_poseImage->setPixmap(pm.scaled(420, 420,
                                         Qt::KeepAspectRatio,
                                         Qt::SmoothTransformation));
    }
}

void NewSessionWizard::retranslate()
{
    setWindowTitle(Lang::t("app_title"));
    if (m_welcomeHeading) m_welcomeHeading->setText(Lang::t("start_new_session"));
    if (m_welcomeSub)     m_welcomeSub->setText(Lang::t("welcome_sub"));
    if (m_btnStart)       m_btnStart->setText(Lang::t("start_new_session"));
    if (m_modeTitle)      m_modeTitle->setText(Lang::t("mode_title"));
    if (m_rbSuit)         m_rbSuit->setText(Lang::t("suit_only"));
    if (m_rbSuitG)        m_rbSuitG->setText(Lang::t("suit_with_gloves"));
    // Combo items are populated once in the ctor — re-label them here so a
    // language switch updates them (preserving item order and userData).
    if (m_cbxTransport && m_cbxTransport->count() >= 2) {
        m_cbxTransport->setItemText(0, Lang::t("transport_com"));
        m_cbxTransport->setItemText(1, Lang::t("transport_wifi"));
    }
    if (m_cbxSuit && m_cbxSuit->count() >= 2) {
        m_cbxSuit->setItemText(0, Lang::t("suit_awinda"));
        m_cbxSuit->setItemText(1, Lang::t("suit_link"));
    }
    if (m_wifiHint)       m_wifiHint->setText(Lang::t("wifi_hint"));
    if (m_btnConnectSuit) m_btnConnectSuit->setText(Lang::t(
        (m_rx && m_rx->isStreaming()) ? "suit_connected" : "connect_suit"));
    if (m_btnConnectGloves) m_btnConnectGloves->setText(Lang::t("connect_gloves"));
    if (m_gloveText) {
        const char* k = "gloves_missing";
        if (m_rx) {
            if (m_rx->glovesReady())          k = "gloves_ready";
            else if (m_rx->glovesCoreReady()) k = "gloves_no_device";
            else if (m_rx->glovesDllLoaded()) k = "gloves_no_core";
        }
        m_gloveText->setText(Lang::t(k));
    }
    if (m_dimsTitle)      m_dimsTitle->setText(Lang::t("dims_title"));
    if (m_lblHeight)      m_lblHeight->setText(Lang::t("body_height") + ":");
    if (m_lblFoot)        m_lblFoot->setText(Lang::t("foot_length") + ":");
    // FIX: переведённые подписи к новым полям размаха рук и длины ноги.
    if (m_lblArm)         m_lblArm->setText(Lang::t("body_arm_span") + ":");
    if (m_lblLeg)         m_lblLeg->setText(Lang::t("body_leg_length") + ":");
    // FIX issue 5: подписи для трёх новых полей.
    if (m_lblHip)         m_lblHip->setText(Lang::t("body_hip_width") + ":");
    if (m_lblShoulder)    m_lblShoulder->setText(Lang::t("body_shoulder_width") + ":");
    if (m_lblTrunk)       m_lblTrunk->setText(Lang::t("body_trunk_length") + ":");
    if (m_dimsHint)       m_dimsHint->setText(Lang::t("dims_hint"));
    for (QGroupBox* gb : findChildren<QGroupBox*>()) {
        if (gb->property("isPrimaryDims"  ).toBool()) gb->setTitle(Lang::t("dims_primary"));
        if (gb->property("isBreakdownBox" ).toBool()) gb->setTitle(Lang::t("dims_breakdown"));
        if (gb->property("isCalibPose"    ).toBool()) gb->setTitle(Lang::t("calib_pose_box"));
        if (gb->property("isCalibStatus"  ).toBool()) gb->setTitle(Lang::t("calib_status_box"));
        if (gb->property("isCalibProgress").toBool()) gb->setTitle(Lang::t("calib_progress_box"));
    }
    for (QLabel* lab : findChildren<QLabel*>()) {
        if (lab->property("isBreakdownCap").toBool()) {
            const QString k = lab->property("bkKey").toString();
            lab->setText(Lang::t(k.toUtf8().constData()) + ":");
        }
    }
    if (m_calibTitle)     m_calibTitle->setText(Lang::t("calib_title"));
    if (m_poseHint) {
        const bool nHalf = (m_phase == CalibPhase::PrepN
                         || m_phase == CalibPhase::CaptureN);
        const bool kHalf = (m_phase == CalibPhase::PrepK
                         || m_phase == CalibPhase::CaptureK);
        if (kHalf)
            m_poseHint->setText(Lang::t("kpose_hint"));
        else
            m_poseHint->setText(Lang::t(nHalf ? "npose_hint" : "tpose_hint"));
    }
    if (m_btnCalibBegin)  m_btnCalibBegin->setText(Lang::t("start_calib"));
    if (m_readyTitle)     m_readyTitle->setText(Lang::t("ready_title"));
    if (m_readySummary)   m_readySummary->setText(Lang::t("ready_summary")
                            .arg(Lang::t("finish")));
    if (m_btnFinish)      m_btnFinish->setText(Lang::t("finish"));
    if (m_btnBack)        m_btnBack->setText(Lang::t("back"));
    if (m_btnNext)        m_btnNext->setText(Lang::t("continue"));

    // Welcome "Language" label discovery
    for (QLabel* lab : findChildren<QLabel*>()) {
        if (lab->property("isLangLabel").toBool())
            lab->setText(Lang::t("language") + ":");
    }
}

void NewSessionWizard::setBadge(QLabel* lab, const QString& txt, bool green)
{
    lab->setText(txt);
    lab->setStyleSheet(QString(
        "padding:6px 14px; border-radius:14px; font-weight:700; "
        "background:%1; color:%2;")
        .arg(green ? "#2EC25A" : "#C03838")
        .arg("#FFFFFF"));
}

void NewSessionWizard::onLanguageChanged(int idx)
{
    Lang::instance().setLanguage(idx == 0 ? Lang::RU : Lang::EN);
}

void NewSessionWizard::updateNavButtons()
{
    m_btnBack->setVisible(m_pageIdx > 0 && m_pageIdx < 4);
    m_btnNext->setVisible(m_pageIdx > 0 && m_pageIdx < 3);
    // page 0 has its own hero button, page 3 has "Start calibration",
    // page 4 has "Finish".  Back is hidden on page 0 and page 4.
    m_btnBack->setEnabled(!calibBusy());

    // Continue from Mode page is gated on hardware being connected.
    if (m_pageIdx == 1 && m_btnNext) {
        const bool suitUp   = (m_rx && m_rx->isStreaming());
        const bool gloveUp  = (m_rx && m_rx->glovesReady());
        const bool needGlv  = (m_rbSuitG && m_rbSuitG->isChecked());
        m_btnNext->setEnabled(suitUp && (!needGlv || gloveUp));
    }
}

void NewSessionWizard::onConnectSuit()
{
    if (!m_rx || !m_btnConnectSuit) return;
    // Don't kick off a fresh scan on top of a scan / connect already in
    // flight — restart() tears down the running thread in the middle,
    // abandons trackerHandles, and we lose the device.  Also re-entering
    // from the UI thread while the worker is mid-open is a reliable crash.
    const ConnStatus s = m_rx->status();
    const bool inFlight = (s == ConnStatus::Scanning ||
                           s == ConnStatus::Connecting ||
                           s == ConnStatus::Streaming);
    if (inFlight) {
        testLog("[wizard] Connect suit ignored — already "
                + std::string(connStatusName(s)), m_test);
        return;
    }
    // Debounce at the UI level — every click disables the button for a
    // second and re-enables it via the status tick.  Protects against
    // the user double-tapping while the worker is still on its way down.
    m_suitBtnCooldown = true;
    m_btnConnectSuit->setEnabled(false);
    QTimer::singleShot(1200, this, [this]() {
        m_suitBtnCooldown = false;
        onStatusTick();          // let the status logic decide the enabled state
    });
    testLog("[wizard] Connect suit clicked", m_test);
    // Bind the chosen suit's native rate before the scan so the receiver seeds
    // freqHz correctly and validates the device query against it.
    if (m_cbxSuit)
        m_result.suit = (m_cbxSuit->currentIndex() == 1) ? SuitType::Link : SuitType::Awinda;
    m_rx->setExpectedRate(nativeRateHz(m_result.suit));
    m_rx->restart();
    onStatusTick();
}

void NewSessionWizard::onConnectGloves()
{
    if (!m_rx || !m_btnConnectGloves) return;
    testLog("[wizard] Connect gloves clicked", m_test);
    // Debounce — SDK handshake can take ~300 ms and a double-click used to
    // reach the probe code twice, which was the other crash report.
    m_btnConnectGloves->setEnabled(false);
    QTimer::singleShot(800, this, [this]() {
        if (m_btnConnectGloves) m_btnConnectGloves->setEnabled(true);
    });
    const bool ok = m_rx->connectGloves();
    const char* key = ok ? "gloves_ready"
                   : (m_rx->glovesCoreReady() ? "gloves_no_device"
                                              : "gloves_no_core");
    // If the DLL itself is missing there is no Core to talk to — show the
    // original "ManusSDK.dll not found" copy instead of the Core message.
    if (!ok && !m_rx->glovesDllLoaded()) key = "gloves_missing";
    if (m_gloveDot)  paintDot(m_gloveDot, ok ? "#2EC25A" : "#C03838");
    if (m_gloveText) m_gloveText->setText(Lang::t(key));
    updateNavButtons();
}

void NewSessionWizard::preselectGloves(bool on)
{
    if (m_rbSuitG && on) m_rbSuitG->setChecked(true);
    else if (m_rbSuit)   m_rbSuit->setChecked(true);
    onModeChanged();
}

void NewSessionWizard::preselectSuit(SuitType suit)
{
    m_result.suit = suit;
    if (m_cbxSuit) m_cbxSuit->setCurrentIndex(suit == SuitType::Link ? 1 : 0);
    if (m_rx)      m_rx->setExpectedRate(nativeRateHz(suit));
}

void NewSessionWizard::onModeChanged()
{
    const bool gloves = m_rbSuitG && m_rbSuitG->isChecked();
    if (m_btnConnectGloves) m_btnConnectGloves->setVisible(gloves);
    if (m_gloveDot)         m_gloveDot->setVisible(gloves);
    if (m_gloveText)        m_gloveText->setVisible(gloves);
    updateNavButtons();
}

void NewSessionWizard::goNext()
{
    if (m_pageIdx == 1) {
        m_result.useGloves = m_rbSuitG->isChecked();
        if (m_cbxSuit)
            m_result.suit = (m_cbxSuit->currentIndex() == 1) ? SuitType::Link : SuitType::Awinda;
    } else if (m_pageIdx == 2) {
        m_result.heightCm     = m_height->value();
        m_result.footLengthCm = m_foot->value();
        // FIX: размах рук и длина ноги теперь захватываются здесь же,
        // а не во вьюпорте.  0 → SkeletonXsens::buildLengths использует
        // антропометрический фоллбэк по росту.
        m_result.armSpanCm        = m_arm      ? m_arm->value()      : 0.0;
        m_result.legLengthCm      = m_leg      ? m_leg->value()      : 0.0;
        // FIX issue 5: hip width / shoulder width / trunk length.
        m_result.hipWidthCm       = m_hip      ? m_hip->value()      : 0.0;
        m_result.shoulderWidthCm  = m_shoulder ? m_shoulder->value() : 0.0;
        m_result.trunkLengthCm    = m_trunk    ? m_trunk->value()    : 0.0;
    }
    if (m_pageIdx < m_pages->count() - 1) {
        ++m_pageIdx;
        m_pages->setCurrentIndex(m_pageIdx);
        updateNavButtons();
        retranslate();
    }
}

void NewSessionWizard::goBack()
{
    if (m_pageIdx > 0) {
        --m_pageIdx;
        m_pages->setCurrentIndex(m_pageIdx);
        updateNavButtons();
        retranslate();
    }
}

void NewSessionWizard::onStatusTick()
{
    if (!m_rx) return;
    const ConnStatus s = m_rx->status();
    const bool streaming = (s == ConnStatus::Streaming);
    const char* key = "suit_disconnected";
    switch (s) {
        case ConnStatus::Scanning:    key = "suit_scanning";     break;
        case ConnStatus::Connecting:  key = "suit_connecting";   break;
        case ConnStatus::Streaming:   key = "suit_connected";    break;
        case ConnStatus::NoDriver:    key = "suit_nodriver";     break;
        case ConnStatus::NoDevice:    key = "suit_nodevice";     break;
        case ConnStatus::Stale:       key = "suit_stale";        break;
        case ConnStatus::Failed:      key = "suit_disconnected"; break;
        case ConnStatus::NotInitialized:
        default:                      key = "suit_scanning";     break;
    }
    // Calibration page badge.
    if (m_connBadge) setBadge(m_connBadge, Lang::t(key), streaming);

    // Mode page row badge / button.
    if (m_suitDot)  paintDot(m_suitDot, streaming ? "#2EC25A" : "#C03838");
    if (m_suitText) m_suitText->setText(Lang::t(key));
    if (m_btnConnectSuit) {
        // No real "disconnect" path exists here (the receiver thread owns the
        // device lifecycle), so never advertise an action the button can't
        // perform.  Connected → show connected + disable; scanning/connecting
        // or within the click cooldown → disable; otherwise enable to connect.
        const bool inFlight = (s == ConnStatus::Scanning || s == ConnStatus::Connecting);
        m_btnConnectSuit->setText(Lang::t(streaming ? "suit_connected" : "connect_suit"));
        m_btnConnectSuit->setEnabled(!streaming && !inFlight && !m_suitBtnCooldown);
    }
    if (m_modeHint) {
        const bool needGlv = (m_rbSuitG && m_rbSuitG->isChecked());
        const bool glvUp   = m_rx->glovesReady();
        if (!streaming)            m_modeHint->setText(Lang::t("need_suit_to_continue"));
        else if (needGlv && !glvUp) m_modeHint->setText(Lang::t("need_gloves_to_continue"));
        else                        m_modeHint->setText(Lang::t("mode_hint_ok"));
    }
    updateNavButtons();

    if (m_btnCalibBegin) {
        m_btnCalibBegin->setEnabled(streaming
                                  && !calibBusy()
                                  && !m_calibComplete);
        if (!streaming && m_calibStatus)
            m_calibStatus->setText(Lang::t("waiting_for_suit"));
    }
    if (!streaming && calibBusy()) {
        // Lost connection mid-calibration (countdown, capture OR the timer-less
        // settle phases) — abort so pending settle callbacks can't bake garbage
        // from a dead suit into the calibration reference.
        abortCalibration();
        if (m_calibStatus) m_calibStatus->setText(Lang::t("waiting_for_suit"));
    }
}

void NewSessionWizard::abortCalibration()
{
    ++m_settleGen;          // invalidate any pending settle / singleShot callbacks
    m_countTimer.stop();
    m_captureTimer.stop();
    if (m_countdownBar) m_countdownBar->setValue(0);
    if (m_readyBar) {
        m_readyBar->setRange(0, kCalibrationSamples);
        m_readyBar->setValue(0);
        m_readyBar->setFormat("%v / %m");
    }
    m_goodSamples = 0;
    m_samples.clear();
    m_havePrev = false;
    m_calibComplete = false;
    m_phase = CalibPhase::Idle;
    refreshPoseImage();
    if (m_countLabel) m_countLabel->setText("—");
}

void NewSessionWizard::onCalibrationBegin()
{
    if (!m_rx || m_rx->status() != ConnStatus::Streaming) return;

    ++m_settleGen;          // a fresh run invalidates callbacks from any prior aborted run

    m_result.poseKind = "npose";

    m_samples.clear();
    m_goodSamples = 0;
    m_lastSampleCtr = -1;
    m_havePrev = false;
    m_countdownBar->setValue(0);
    m_readyBar->setValue(0);
    m_btnCalibBegin->setEnabled(false);

    // Reset BOTH accumulators (T and N) for the fresh run.
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        m_accAccumT[i]    = QVector3D(0, 0, 0);
        m_gyrAccumT[i]    = QVector3D(0, 0, 0);
        m_magAccumT[i]    = QVector3D(0, 0, 0);
        m_accMagAccumT[i] = 0.0;
        m_accumCountT[i]  = 0;
        m_accAccumN[i]    = QVector3D(0, 0, 0);
        m_gyrAccumN[i]    = QVector3D(0, 0, 0);
        m_magAccumN[i]    = QVector3D(0, 0, 0);
        m_accMagAccumN[i] = 0.0;
        m_accumCountN[i]  = 0;
        m_gyrSqAccumT[i]  = QVector3D(0, 0, 0);
        m_gyrSqAccumN[i]  = QVector3D(0, 0, 0);
        m_gyrSqAccumK[i]  = QVector3D(0, 0, 0);
        for (int k = 0; k < 6; ++k) {
            m_magOuterAccumT[i][k] = 0.0;
            m_magOuterAccumN[i][k] = 0.0;
            m_magOuterAccumK[i][k] = 0.0;
        }
    }
    // FIX (gloves polish): finger baseline accumulator также сбросить.
    for (int j = 0; j < 20; ++j) {
        m_fingerAccumR[j] = 0.0;
        m_fingerAccumL[j] = 0.0;
    }
    m_fingerAccumCount = 0;

    // Bind pointer aliases to the T-pose buffers (CaptureT writes through
    // these so the capture loop stays pose-agnostic).
    m_accAccum    = &m_accAccumT;
    m_gyrAccum    = &m_gyrAccumT;
    m_magAccum    = &m_magAccumT;
    m_accMagAccum = &m_accMagAccumT;
    m_accumCount  = &m_accumCountT;
    m_gyrSqAccum    = &m_gyrSqAccumT;
    m_magOuterAccum = &m_magOuterAccumT;

    // Drop current s2s + restart Madgwick.  The first countdown that
    // follows doubles as a fusion warm-up AND gives the actor time to
    // settle into the T-pose reference.
    m_rx->resetS2sAlignment();
    m_rx->resetFusion();

    m_phase = CalibPhase::PrepT;
    refreshPoseImage();
    m_countTicksLeft = kCountdownSeconds * 10;
    m_countLabel->setText(QString::number(kCountdownSeconds));
    if (m_calibStatus)
        m_calibStatus->setText(Lang::t("calib_t_prepare"));
    if (m_poseHint)
        m_poseHint->setText(Lang::t("tpose_hint"));
    testLog("[calib] double-pose sequence started, PrepT "
            "(Madgwick re-init scheduled)", m_test);
    m_countTimer.start();
    updateNavButtons();
}

void NewSessionWizard::onCountdownTick()
{
    --m_countTicksLeft;
    const int total = kCountdownSeconds * 10;
    m_countdownBar->setValue(total - m_countTicksLeft);
    const int secLeft = (m_countTicksLeft + 9) / 10;
    m_countLabel->setText(QString::number(secLeft > 0 ? secLeft : 0));
    if (m_countTicksLeft <= 0) {
        m_countTimer.stop();
        m_countLabel->setText(Lang::t("still"));

        // Countdown-end dispatcher: enter the matching capture phase and
        // reset still-detector + per-phase readiness bar.
        m_goodSamples = 0;
        m_havePrev    = false;
        m_samples.clear();
        m_readyBar->setValue(0);

        if (m_phase == CalibPhase::PrepT) {
            m_phase = CalibPhase::CaptureT;
            if (m_calibStatus)
                m_calibStatus->setText(Lang::t("calib_t_capture"));
        } else if (m_phase == CalibPhase::PrepN) {
            m_phase = CalibPhase::CaptureN;
            if (m_calibStatus)
                m_calibStatus->setText(Lang::t("calib_n_capture"));
        } else if (m_phase == CalibPhase::PrepK) {
            m_phase = CalibPhase::CaptureK;
            if (m_calibStatus)
                m_calibStatus->setText(Lang::t("calib_k_capture"));
        }
        m_captureTimer.start();
    }
}

void NewSessionWizard::onCaptureTick()
{
    const SuitPose fr = m_rx->snapshot();
    if (qint64(fr.sampleCounter) == m_lastSampleCtr) return;
    m_lastSampleCtr = fr.sampleCounter;

    std::array<Quat, kXsensSegmentCount> snap{};
    for (int i = 0; i < kXsensSegmentCount; ++i) snap[i] = fr.quat[i];

    // Still threshold — SECOND-largest frame delta so one noisy outlier
    // (usually the head tracker) can't halt the whole actor.  0.025 rad
    // per ~4 ms frame is ≈ 340°/s — way above breathing jitter, well below
    // macroscopic body motion.
    double maxD = 0.0, second = 0.0;
    if (m_havePrev) {
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            const double dot = std::abs(snap[i].w * m_prevSnap[i].w
                                      + snap[i].x * m_prevSnap[i].x
                                      + snap[i].y * m_prevSnap[i].y
                                      + snap[i].z * m_prevSnap[i].z);
            const double c = dot > 1 ? 1 : (dot < -1 ? -1 : dot);
            const double a = 2.0 * std::acos(c);
            if (a > maxD)        { second = maxD; maxD = a; }
            else if (a > second)   second = a;
        }
    }
    m_prevSnap = snap; m_havePrev = true;

    constexpr double kStillRad = 0.025;
    const bool still = second < kStillRad;

    if (still) {
        m_samples.push_back(snap);
        constexpr float kPerSegmentGyrLimit = 3.0f;
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            if (!fr.segValid[i]) continue;
            const QVector3D g = fr.gyrSensor[i];
            const float gMag = std::sqrt(g.x()*g.x() + g.y()*g.y() + g.z()*g.z());
            if (gMag > kPerSegmentGyrLimit) continue;
            (*m_accAccum)[i]    += fr.accSensor[i];
            (*m_gyrAccum)[i]    += fr.gyrSensor[i];
            (*m_magAccum)[i]    += fr.magSensor[i];
            (*m_accMagAccum)[i] += fr.accSensor[i].length();
            (*m_accumCount)[i]++;
            if (m_gyrSqAccum) {
                (*m_gyrSqAccum)[i] += QVector3D(g.x()*g.x(), g.y()*g.y(), g.z()*g.z());
            }
            if (m_magOuterAccum) {
                const double mx = double(fr.magSensor[i].x());
                const double my = double(fr.magSensor[i].y());
                const double mz = double(fr.magSensor[i].z());
                auto& o = (*m_magOuterAccum)[i];
                o[0] += mx*mx;
                o[1] += my*my;
                o[2] += mz*mz;
                o[3] += mx*my;
                o[4] += mx*mz;
                o[5] += my*mz;
            }
        }
        ++m_goodSamples;
        // FIX (gloves polish): копим Manus finger degrees ТОЛЬКО в T-pose.
        // Используем уже EMA-сглаженные значения из g_ergo, чтобы не
        // удваивать LP-фильтрацию.  Lock мьютекса g_ergo короткий.
        if (m_phase == CalibPhase::CaptureT) {
            QMutexLocker lkErgo(&g_ergo.lock);
            if (g_ergo.emaLeftInit) {
                for (int j = 0; j < 20; ++j)
                    m_fingerAccumL[j] += double(g_ergo.emaLeft[j]);
            }
            if (g_ergo.emaRightInit) {
                for (int j = 0; j < 20; ++j)
                    m_fingerAccumR[j] += double(g_ergo.emaRight[j]);
            }
            if (g_ergo.emaLeftInit || g_ergo.emaRightInit) {
                ++m_fingerAccumCount;
            }
        }
        m_stillLabel->setStyleSheet("color:#2EC25A; font-weight:700;");
        m_stillLabel->setText(Lang::t("motion_hint") + " "
            + Lang::t("still") + QString("  (%1°)")
              .arg(qRadiansToDegrees(second), 0, 'f', 2));
    } else {
        m_goodSamples = std::max(0, m_goodSamples - 2);
        m_stillLabel->setStyleSheet("color:#E04040; font-weight:700;");
        m_stillLabel->setText(Lang::t("motion_hint") + " "
            + Lang::t("moving") + QString("  (%1°)")
              .arg(qRadiansToDegrees(second), 0, 'f', 2));
    }
    const int v = std::min(m_goodSamples, kCalibrationSamples);
    m_readyBar->setValue(v);

    if (m_goodSamples < kCalibrationSamples) return;

    // ---------- CaptureT complete → settle + refine T-pose reference -----
    if (m_phase == CalibPhase::CaptureT) {
        m_captureTimer.stop();
        testLog("[calib] T-pose capture complete, samples="
                + std::to_string(m_samples.size())
                + " — settling filter for refined T-pose reference", m_test);

        if (!m_samples.empty()) {
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                const Quat& anchor = m_samples.front()[i];
                double sw = 0, sx = 0, sy = 0, sz = 0;
                for (const auto& s : m_samples) {
                    const double d = s[i].w * anchor.w + s[i].x * anchor.x
                                   + s[i].y * anchor.y + s[i].z * anchor.z;
                    const double sgn = d < 0 ? -1.0 : 1.0;
                    sw += sgn * s[i].w; sx += sgn * s[i].x;
                    sy += sgn * s[i].y; sz += sgn * s[i].z;
                }
                m_result.tposeReference[i] = Quat(sw, sx, sy, sz).normalized();
            }
            m_result.tposePelvisPos = QVector3D(0.0f, 0.0f, 0.0f);
            m_result.tposeCaptured = true;
        }

        // FIX (gloves polish): финализируем finger baseline.  Делим
        // накопленную сумму на m_fingerAccumCount, копируем в Result.
        // Если actor без перчаток — g_ergo.emaLeftInit/RightInit = false
        // → counter остаётся 0 → branch скипается.
        if (m_fingerAccumCount > 0) {
            const double inv = 1.0 / double(m_fingerAccumCount);
            for (int j = 0; j < 20; ++j) {
                m_result.fingerBaselineR[j] = float(m_fingerAccumR[j] * inv);
                m_result.fingerBaselineL[j] = float(m_fingerAccumL[j] * inv);
            }
            m_result.fingerBaselineCaptured = true;
            if (m_test) {
                std::cout << "[calib T finger baseline] count="
                          << m_fingerAccumCount << " R thumb spread/MCP="
                          << m_result.fingerBaselineR[0] << "/"
                          << m_result.fingerBaselineR[1]
                          << " L thumb spread/MCP="
                          << m_result.fingerBaselineL[0] << "/"
                          << m_result.fingerBaselineL[1] << "\n";
            }
        }

        m_phase = CalibPhase::SettleT;
        const int gen = m_settleGen;

        constexpr int kSettleMs       = 8000;
        constexpr int kSnapshotCount  = 16;
        constexpr int kSnapshotStepMs = 250;
        constexpr int kTotalCalibMs   = 12500;
        if (m_calibStatus)
            m_calibStatus->setText(Lang::t("calib_tpose_tune")
                                   .arg(kTotalCalibMs / 1000));
        if (m_readyBar) {
            m_readyBar->setRange(0, kTotalCalibMs);
            m_readyBar->setValue(0);
            m_readyBar->setFormat("%p %");
        }

        auto tSnapshots = std::make_shared<std::vector<
            std::array<Quat, kXsensSegmentCount>>>();
        auto tDropped   = std::make_shared<int>(0);
        auto progressTimer = std::make_shared<QTimer>(this);
        auto progressStart = std::make_shared<qint64>(QDateTime::currentMSecsSinceEpoch());
        progressTimer->setInterval(100);
        connect(progressTimer.get(), &QTimer::timeout, this,
                [this, gen, tSnapshots, progressTimer, progressStart, kTotalCalibMs, kSettleMs, kSnapshotCount, kSnapshotStepMs]() {
            if (gen != m_settleGen) { progressTimer->stop(); return; }
            const qint64 elapsed = QDateTime::currentMSecsSinceEpoch() - *progressStart;
            // Honest progress: first half tracks the timed filter-convergence
            // wait; second half reflects the REAL number of snapshots captured
            // (stalls if the actor moves and snapshots are dropped) rather than
            // raw elapsed time.
            if (m_readyBar) {
                if (elapsed < kSettleMs) {
                    m_readyBar->setValue(int(kTotalCalibMs / 2
                        * std::min<qint64>(elapsed, kSettleMs) / kSettleMs));
                } else {
                    const int captured = int(tSnapshots->size());
                    m_readyBar->setValue(kTotalCalibMs / 2
                        + (kTotalCalibMs / 2) * std::min(captured, kSnapshotCount) / kSnapshotCount);
                }
            }
            if (m_calibStatus) {
                if (elapsed < kSettleMs) {
                    const int rem = int(std::max<qint64>(0, kSettleMs - elapsed));
                    m_calibStatus->setText(Lang::t("calib_tpose_converge")
                                           .arg((rem + 999) / 1000));
                } else {
                    const int idx = int(std::min(kSnapshotCount - 1,
                        int((elapsed - kSettleMs) / kSnapshotStepMs) + 1));
                    m_calibStatus->setText(Lang::t("calib_tpose_capture")
                                           .arg(idx).arg(kSnapshotCount));
                }
            }
            if (elapsed >= kTotalCalibMs) progressTimer->stop();
        });
        progressTimer->start();
        QApplication::processEvents();

        for (int k = 0; k < kSnapshotCount; ++k) {
            const int delayMs = kSettleMs + k * kSnapshotStepMs;
            QTimer::singleShot(delayMs, this, [this, gen, tSnapshots, tDropped, k]() {
                if (gen != m_settleGen) return;
                const SuitPose post = m_rx->snapshot();
                const double pelvisGyr = double(post.gyrSensor[SEG_Pelvis].length());
                const double rArmGyr   = double(post.gyrSensor[SEG_RUpperArm].length());
                const double lArmGyr   = double(post.gyrSensor[SEG_LUpperArm].length());
                const double rLegGyr   = double(post.gyrSensor[SEG_RUpperLeg].length());
                const double lLegGyr   = double(post.gyrSensor[SEG_LUpperLeg].length());
                const double maxGyr    = std::max({pelvisGyr, rArmGyr, lArmGyr, rLegGyr, lLegGyr});
                if (maxGyr > 8.0) { ++(*tDropped); return; }
                std::array<Quat, kXsensSegmentCount> sn{};
                for (int i = 0; i < kXsensSegmentCount; ++i) {
                    sn[i] = post.segValid[i] ? post.quat[i] : Quat(1, 0, 0, 0);
                }
                tSnapshots->push_back(sn);
            });
        }

        QTimer::singleShot(kTotalCalibMs, this, [this, gen, tSnapshots, tDropped]() {
            if (gen != m_settleGen) return;
            if (m_test) {
                std::cout << "[calib] T-pose settled snapshots: " << tSnapshots->size()
                          << " kept, " << *tDropped << " dropped\n";
                std::cout.flush();
            }
            if (!tSnapshots->empty()) {
                for (int i = 0; i < kXsensSegmentCount; ++i) {
                    const Quat& anchor = (*tSnapshots)[0][i];
                    double sw=0, sx=0, sy=0, sz=0;
                    for (const auto& s : *tSnapshots) {
                        const double d = s[i].w*anchor.w + s[i].x*anchor.x
                                       + s[i].y*anchor.y + s[i].z*anchor.z;
                        const double sgn = d < 0 ? -1.0 : 1.0;
                        sw += sgn*s[i].w; sx += sgn*s[i].x;
                        sy += sgn*s[i].y; sz += sgn*s[i].z;
                    }
                    m_result.tposeReference[i] = Quat(sw, sx, sy, sz).normalized();
                }
                m_result.tposePelvisPos = QVector3D(0.0f, 0.0f, 0.0f);
                m_result.tposeCaptured = true;
            }
            testLog("[calib] T-pose refined (filter-settled, "
                    + std::to_string(tSnapshots->size()) + " snapshots)", m_test);

            m_accAccum    = &m_accAccumN;
            m_gyrAccum    = &m_gyrAccumN;
            m_magAccum    = &m_magAccumN;
            m_accMagAccum = &m_accMagAccumN;
            m_accumCount  = &m_accumCountN;
            m_gyrSqAccum    = &m_gyrSqAccumN;
            m_magOuterAccum = &m_magOuterAccumN;

            m_phase = CalibPhase::PrepN;
            refreshPoseImage();
            if (m_poseHint)
                m_poseHint->setText(Lang::t("npose_hint"));
            if (m_calibStatus)
                m_calibStatus->setText(Lang::t("calib_n_prepare"));

            m_countTicksLeft = kCountdownSeconds * 10;
            m_countdownBar->setValue(0);
            m_readyBar->setValue(0);
            m_readyBar->setRange(0, kCalibrationSamples);
            m_readyBar->setFormat("%v / %m");
            m_goodSamples = 0;
            m_havePrev    = false;
            m_countLabel->setText(QString::number(kCountdownSeconds));
            m_countTimer.start();
            updateNavButtons();
        });
        return;
    }

    if (m_phase == CalibPhase::CaptureK) {
        m_captureTimer.stop();
        testLog("[calib] K-pose capture complete, refining s2s with best pair…", m_test);

        std::array<Quat, kXsensSegmentCount> s2sOld{};
        for (int i = 0; i < kXsensSegmentCount; ++i) s2sOld[i] = m_rx->getS2s(i);

        const auto defAng_T = defaultSegAnglesFor("tpose");
        const auto defAng_N = defaultSegAnglesFor("npose");
        const auto defAng_K = defaultSegAnglesFor("kpose");

        std::array<Quat, kXsensSegmentCount> s2sNew = s2sOld;
        std::array<int,  kXsensSegmentCount> s2sNewMode{};
        std::array<double, kXsensSegmentCount> s2sResidual{};
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            s2sNewMode[i] = 2;
            s2sResidual[i] = 99.0;
        }

        auto clampBiasTight = [](float vv) {
            const float kMax = 0.2f;
            return vv > kMax ? kMax : (vv < -kMax ? -kMax : vv);
        };

        std::array<double,    kXsensSegmentCount> magMagn{};
        std::array<double,    kXsensSegmentCount> accMagn{};
        std::array<QVector3D, kXsensSegmentCount> gyrBias{};
        std::array<QVector3D, kXsensSegmentCount> gyrStdDev{};
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            magMagn[i] = 1.0;
            accMagn[i] = 1.0;
            gyrBias[i] = QVector3D(0, 0, 0);
            gyrStdDev[i] = QVector3D(0, 0, 0);
        }

        std::array<std::array<double, 9>, kXsensSegmentCount> magSoftMat{};
        std::array<QVector3D, kXsensSegmentCount>             magSoftOff{};
        std::array<bool,      kXsensSegmentCount>             magSoftOk{};
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            for (int k = 0; k < 9; ++k) magSoftMat[i][k] = (k == 0 || k == 4 || k == 8) ? 1.0 : 0.0;
            magSoftOff[i] = QVector3D(0, 0, 0);
            magSoftOk[i] = false;
        }

        for (int i = 0; i < kXsensSegmentCount; ++i) {
            const int cT = m_accumCountT[i];
            const int cN = m_accumCountN[i];
            const int cK = m_accumCountK[i];
            if (cT < 10 || cN < 10 || cK < 10) continue;

            const double invT = 1.0 / double(cT);
            const double invN = 1.0 / double(cN);
            const double invK = 1.0 / double(cK);
            const QVector3D meanA_T = m_accAccumT[i] * float(invT);
            const QVector3D meanA_N = m_accAccumN[i] * float(invN);
            const QVector3D meanA_K = m_accAccumK[i] * float(invK);
            const QVector3D meanM_N = m_magAccumN[i] * float(invN);
            if (meanA_T.length() < 1e-6 || meanA_N.length() < 1e-6 ||
                meanA_K.length() < 1e-6) continue;

            const int    cTot = cT + cN + cK;
            const double invTot = 1.0 / double(cTot);
            accMagn[i] = (m_accMagAccumT[i] + m_accMagAccumN[i] + m_accMagAccumK[i]) * invTot;
            const QVector3D meanM_all = (m_magAccumT[i] + m_magAccumN[i] + m_magAccumK[i])
                                        * float(invTot);
            magMagn[i] = double(meanM_all.length());

            const QVector3D meanG_T = m_gyrAccumT[i] * float(invT);
            const QVector3D meanG_N = m_gyrAccumN[i] * float(invN);
            const QVector3D meanG_K = m_gyrAccumK[i] * float(invK);
            const QVector3D gSqT = m_gyrSqAccumT[i] * float(invT);
            const QVector3D gSqN = m_gyrSqAccumN[i] * float(invN);
            const QVector3D gSqK = m_gyrSqAccumK[i] * float(invK);
            const QVector3D vT(std::max(0.0f, gSqT.x() - meanG_T.x()*meanG_T.x()),
                               std::max(0.0f, gSqT.y() - meanG_T.y()*meanG_T.y()),
                               std::max(0.0f, gSqT.z() - meanG_T.z()*meanG_T.z()));
            const QVector3D vN(std::max(0.0f, gSqN.x() - meanG_N.x()*meanG_N.x()),
                               std::max(0.0f, gSqN.y() - meanG_N.y()*meanG_N.y()),
                               std::max(0.0f, gSqN.z() - meanG_N.z()*meanG_N.z()));
            const QVector3D vK(std::max(0.0f, gSqK.x() - meanG_K.x()*meanG_K.x()),
                               std::max(0.0f, gSqK.y() - meanG_K.y()*meanG_K.y()),
                               std::max(0.0f, gSqK.z() - meanG_K.z()*meanG_K.z()));
            auto medianBias = [&](double mT, double mN, double mK,
                                  double vTc, double vNc, double vKc) -> double {
                struct Pair { double m; double v; };
                Pair p[3] = { {mT, vTc}, {mN, vNc}, {mK, vKc} };
                std::sort(std::begin(p), std::end(p), [](const Pair& a, const Pair& b){ return a.m < b.m; });
                if (std::abs(p[2].m - p[0].m) > 0.4) return p[1].m;
                return (p[0].m + p[1].m + p[2].m) / 3.0;
            };
            const double biasX = medianBias(meanG_T.x(), meanG_N.x(), meanG_K.x(),
                                            vT.x(), vN.x(), vK.x());
            const double biasY = medianBias(meanG_T.y(), meanG_N.y(), meanG_K.y(),
                                            vT.y(), vN.y(), vK.y());
            const double biasZ = medianBias(meanG_T.z(), meanG_N.z(), meanG_K.z(),
                                            vT.z(), vN.z(), vK.z());
            gyrBias[i] = QVector3D(clampBiasTight(float(biasX)),
                                   clampBiasTight(float(biasY)),
                                   clampBiasTight(float(biasZ)));
            const QVector3D vAvg = (vT * float(cT) + vN * float(cN) + vK * float(cK))
                                   * float(invTot);
            gyrStdDev[i] = QVector3D(std::sqrt(vAvg.x()), std::sqrt(vAvg.y()), std::sqrt(vAvg.z()));

            QVector3D magSumAll = m_magAccumT[i] + m_magAccumN[i] + m_magAccumK[i];
            double outerAll[6] = {0,0,0,0,0,0};
            for (int k = 0; k < 6; ++k) {
                outerAll[k] = m_magOuterAccumT[i][k] + m_magOuterAccumN[i][k] + m_magOuterAccumK[i][k];
            }
            const SoftIronFit si = fitMagSoftIron(magSumAll, outerAll, cTot);
            if (si.valid && si.residual < 5.0) {
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        magSoftMat[i][r * 3 + c] = si.M[r][c];
                magSoftOff[i] = si.offset;
                magSoftOk[i] = true;
            }

            const QVector3D aT_s = meanA_T.normalized();
            const QVector3D aN_s = meanA_N.normalized();
            const QVector3D aK_s = meanA_K.normalized();
            const QVector3D gT_b = gravityInBodyFrame(defAng_T[i]).normalized();
            const QVector3D gN_b = gravityInBodyFrame(defAng_N[i]).normalized();
            const QVector3D gK_b = gravityInBodyFrame(defAng_K[i]).normalized();

            const float sepTN = QVector3D::crossProduct(gT_b, gN_b).length();
            const float sepTK = QVector3D::crossProduct(gT_b, gK_b).length();
            const float sepNK = QVector3D::crossProduct(gN_b, gK_b).length();
            const float minSep = std::min({sepTN, sepTK, sepNK});
            const float maxSep = std::max({sepTN, sepTK, sepNK});

            auto poseWeight = [](int cnt, const QVector3D& var) -> double {
                const double vMag = std::sqrt(double(var.lengthSquared()));
                return double(cnt) / (1.0 + 5.0 * vMag);
            };

            if (minSep > 0.15f || maxSep > 0.5f) {
                const QVector3D vbArr[3] = { gT_b, gN_b, gK_b };
                const QVector3D vsArr[3] = { aT_s, aN_s, aK_s };
                const double wArr[3] = {
                    poseWeight(cT, vT),
                    poseWeight(cN, vN),
                    poseWeight(cK, vK),
                };
                const Quat q_s2b = wahbaSolve(vbArr, vsArr, wArr, 3);
                const double wahbaRes = wahbaResidualDeg(q_s2b, vbArr, vsArr, 3);

                double triadFallbackRes = 999.0;
                Quat triadFallbackQ(1, 0, 0, 0);
                const char* fbPair = "none";
                float fbSep = 0.0f;
                const TriadPairCand cands[3] = {
                    { sepTN, gT_b, gN_b, aT_s, aN_s, "TN" },
                    { sepTK, gT_b, gK_b, aT_s, aK_s, "TK" },
                    { sepNK, gN_b, gK_b, aN_s, aK_s, "NK" },
                };
                bestTriadOfPairs(cands, triadFallbackQ, triadFallbackRes, fbPair, fbSep);

                if (wahbaRes <= 15.0 || wahbaRes <= triadFallbackRes + 2.0) {
                    s2sNew[i] = q_s2b.inv().normalized();
                    s2sResidual[i] = wahbaRes;
                    if (m_test) {
                        std::cout << "[calib K] " << kSegmentNames[i]
                                  << " refined via Wahba(T,N,K) seps=("
                                  << std::fixed << std::setprecision(2) << sepTN << "/" << sepTK << "/" << sepNK
                                  << ") weights=(" << wArr[0] << "/" << wArr[1] << "/" << wArr[2]
                                  << ") residual=" << wahbaRes << "°"
                                  << " gyrStd=(" << gyrStdDev[i].x() << "," << gyrStdDev[i].y() << ","
                                  << gyrStdDev[i].z() << ")\n";
                    }
                } else {
                    s2sNew[i] = triadFallbackQ.inv().normalized();
                    s2sResidual[i] = triadFallbackRes;
                    if (m_test) {
                        std::cout << "[calib K] " << kSegmentNames[i]
                                  << " Wahba bad (residual=" << std::fixed << std::setprecision(2) << wahbaRes
                                  << "°), fell back to TRIAD via " << fbPair
                                  << " sep=" << fbSep << " residual=" << triadFallbackRes << "°\n";
                    }
                }
                s2sNewMode[i] = 0;
            } else if (maxSep > 0.3f) {
                double bestRes2 = 999.0;
                Quat bestQ2(1, 0, 0, 0);
                const char* bestPair2 = "none";
                float bestSep2_ = 0.0f;
                const TriadPairCand cands2[3] = {
                    { sepTN, gT_b, gN_b, aT_s, aN_s, "TN" },
                    { sepTK, gT_b, gK_b, aT_s, aK_s, "TK" },
                    { sepNK, gN_b, gK_b, aN_s, aK_s, "NK" },
                };
                bestTriadOfPairs(cands2, bestQ2, bestRes2, bestPair2, bestSep2_);
                s2sNew[i] = bestQ2.inv().normalized();
                s2sNewMode[i] = 0;
                s2sResidual[i] = bestRes2;
                if (m_test) {
                    std::cout << "[calib K] " << kSegmentNames[i]
                              << " TRIAD-only via " << bestPair2 << " sep=" << bestSep2_
                              << " residual=" << s2sResidual[i] << "°\n";
                }
            } else if (meanM_N.length() > 1e-6) {
                const QVector3D mNorm = meanM_N / float(meanM_N.length());
                Quat qNed = ecompassNED(aN_s, mNorm);
                const Quat nedToNwu(0, 1, 0, 0);
                const Quat qNwu = quat_mult(nedToNwu, qNed).normalized();
                s2sNew[i] = qNwu.inv().normalized();
                s2sNewMode[i] = 1;
                const QVector3D worldGravity(0.0f, 0.0f, -1.0f);
                s2sResidual[i] = ecompResidualDeg(qNwu, worldGravity, aN_s);
                if (m_test) {
                    std::cout << "[calib K] " << kSegmentNames[i]
                              << " ecompass-fallback residual=" << std::fixed << std::setprecision(2)
                              << s2sResidual[i] << "°\n";
                }
            }
        }

        {
            const std::pair<int,int> pairsK[8] = {
                { SEG_RShoulder,  SEG_LShoulder  }, { SEG_RUpperArm,  SEG_LUpperArm  },
                { SEG_RForearm,   SEG_LForearm   }, { SEG_RHand,      SEG_LHand      },
                { SEG_RUpperLeg,  SEG_LUpperLeg  }, { SEG_RLowerLeg,  SEG_LLowerLeg  },
                { SEG_RFoot,      SEG_LFoot      }, { SEG_RToe,       SEG_LToe       },
            };
            for (const auto& pr : pairsK) {
                if (s2sNewMode[pr.first] == 2 || s2sNewMode[pr.second] == 2) continue;
                const Quat qR = s2sNew[pr.first];
                const Quat qL = s2sNew[pr.second];
                const Quat mYL = mirror_y_quat(qL);
                const double dotMY = qR.w*mYL.w + qR.x*mYL.x + qR.y*mYL.y + qR.z*mYL.z;
                const double dotPar = qR.w*qL.w + qR.x*qL.x + qR.y*qL.y + qR.z*qL.z;
                const double devMirr = mirrorYDeviationDeg(qR, qL);
                const double devPar  = parallelDeviationDeg(qR, qL);
                const bool mirrLikely = (devMirr < 30.0) && (devMirr + 10.0 < devPar);
                const bool parLikely  = (devPar  < 30.0) && (devPar  + 10.0 < devMirr);
                if (!mirrLikely && !parLikely) {
                    if (m_test) {
                        std::cout << "[calib K] hemisphere-unify skipped " << kSegmentNames[pr.first]
                                  << "/" << kSegmentNames[pr.second]
                                  << " (no clear symmetry: devMirr=" << std::fixed << std::setprecision(1)
                                  << devMirr << "° devPar=" << devPar << "°)\n";
                    }
                    continue;
                }
                const bool needFlip = mirrLikely ? (dotMY < 0.0) : (dotPar < 0.0);
                if (needFlip) {
                    s2sNew[pr.second] = Quat(-qL.w, -qL.x, -qL.y, -qL.z);
                    if (m_test) {
                        std::cout << "[calib K] hemisphere-unify L " << kSegmentNames[pr.second]
                                  << " via " << (mirrLikely ? "mirror-Y" : "parallel")
                                  << " (was opposite of R " << kSegmentNames[pr.first] << ")\n";
                    }
                }
            }
        }

        std::array<bool, kXsensSegmentCount> s2sFused{};
        std::array<int,  kXsensSegmentCount> mountType{};
        {
            // FIX issue 4 (правое колено в позе цапли гнётся вбок): mount-detection
            // только по T-pose accel ненадёжна для ног — gravity почти полностью
            // вдоль сенсорного Z, Y-компонента ≈ 0 → mirrDot ≈ parDot ≈ 1.
            // Расширяем сигналы: T-vs-K axis (cross product гравитаций в T и K
            // позах в сенсорной системе) — это направление body-lateral оси в
            // сенсорной системе, mirror-Y vs parallel меняет знак Y-компоненты.
            // Старый путь через m_gyrAccumK был мёртвым: накопитель фильтрует
            // gMag>3°/s, а порог использования >5°/s — условие никогда не
            // выполнялось.  Axis из cross(aT, aK) работает на статике и не
            // требует движения во время capture.
            auto voteFromPair = [&](const QVector3D& vR, const QVector3D& vL) -> std::pair<int, double> {
                if (vR.length() < 1e-3f || vL.length() < 1e-3f) return {0, 0.0};
                const QVector3D nR = vR.normalized();
                const QVector3D nL = vL.normalized();
                const QVector3D mY_nL(nL.x(), -nL.y(), nL.z());
                const double mirrDot = double(QVector3D::dotProduct(nR, mY_nL));
                const double parDot  = double(QVector3D::dotProduct(nR, nL));
                if (std::abs(mirrDot) < 0.05 && std::abs(parDot) < 0.05) return {0, 0.0};
                const int type   = (std::abs(mirrDot) >= std::abs(parDot)) ? 1 : 2;
                const double sc  = std::abs(std::abs(mirrDot) - std::abs(parDot));
                return {type, sc};
            };

            auto detectMountSymmetry = [&](int rSeg, int lSeg, double* outScore = nullptr) -> int {
                if (m_accumCountT[rSeg] < 10 || m_accumCountT[lSeg] < 10) {
                    if (outScore) *outScore = 0.0;
                    return 0;
                }
                const bool isLeg = (rSeg == SEG_RUpperLeg) || (rSeg == SEG_LUpperLeg)
                                || (rSeg == SEG_RLowerLeg) || (rSeg == SEG_LLowerLeg);

                const QVector3D aR_T = m_accAccumT[rSeg] / float(m_accumCountT[rSeg]);
                const QVector3D aL_T = m_accAccumT[lSeg] / float(m_accumCountT[lSeg]);
                auto voteT = voteFromPair(aR_T, aL_T);

                std::pair<int, double> voteN = {0, 0.0};
                if (m_accumCountN[rSeg] >= 10 && m_accumCountN[lSeg] >= 10) {
                    const QVector3D aR_N = m_accAccumN[rSeg] / float(m_accumCountN[rSeg]);
                    const QVector3D aL_N = m_accAccumN[lSeg] / float(m_accumCountN[lSeg]);
                    voteN = voteFromPair(aR_N, aL_N);
                }

                // Axis from accel cross product — body-lateral direction in
                // sensor frame.  Для НОГ используем T↔K (в K-позе бёдра
                // горизонтально, гравитация в сегменте поворачивается ~90°
                // вокруг ±Y body относительно T-позы).  Для РУК используем T↔N
                // (T = руки горизонтально, N = руки вниз — два ортогональных
                // направления гравитации в сегменте), так как T↔K для рук
                // слабый (в K-позе руки тоже близки к горизонтальным).
                // Mirror-Y mount меняет знак Y-компоненты этой оси между R и L;
                // parallel mount оставляет тот же знак.
                std::pair<int, double> voteG = {0, 0.0};
                auto axisVote = [&](const QVector3D& aR1, const QVector3D& aL1,
                                    const QVector3D& aR2, const QVector3D& aL2)
                                    -> std::pair<int, double> {
                    if (aR1.length() < 1e-3f || aL1.length() < 1e-3f
                     || aR2.length() < 1e-3f || aL2.length() < 1e-3f) return {0, 0.0};
                    const QVector3D axR = QVector3D::crossProduct(aR1.normalized(),
                                                                   aR2.normalized());
                    const QVector3D axL = QVector3D::crossProduct(aL1.normalized(),
                                                                   aL2.normalized());
                    if (axR.length() < 0.3f || axL.length() < 0.3f) return {0, 0.0};
                    return voteFromPair(axR, axL);
                };
                if (isLeg && m_accumCountK[rSeg] >= 10 && m_accumCountK[lSeg] >= 10) {
                    const QVector3D aR_K = m_accAccumK[rSeg] / float(m_accumCountK[rSeg]);
                    const QVector3D aL_K = m_accAccumK[lSeg] / float(m_accumCountK[lSeg]);
                    voteG = axisVote(aR_T, aL_T, aR_K, aL_K);
                } else if (!isLeg && m_accumCountN[rSeg] >= 10 && m_accumCountN[lSeg] >= 10) {
                    const QVector3D aR_N = m_accAccumN[rSeg] / float(m_accumCountN[rSeg]);
                    const QVector3D aL_N = m_accAccumN[lSeg] / float(m_accumCountN[lSeg]);
                    voteG = axisVote(aR_T, aL_T, aR_N, aL_N);
                }

                // Per-segment weights.  When axis vote is strong (legs via T↔K,
                // arms via T↔N), it dominates; otherwise fall back to T+N accel
                // (legs) or T-only (arms, original behavior).
                double wT = 1.0, wN = 0.0, wG = 0.0;
                if (isLeg) {
                    if (voteG.second > 0.05) { wT = 0.2; wN = 0.3; wG = 0.5; }
                    else                      { wT = 0.4; wN = 0.6; wG = 0.0; }
                } else {
                    if (voteG.second > 0.05) { wT = 0.4; wN = 0.0; wG = 0.6; }
                }

                double sMirr = 0.0, sPar = 0.0;
                if (voteT.first == 1) sMirr += wT * voteT.second;
                if (voteT.first == 2) sPar  += wT * voteT.second;
                if (voteN.first == 1) sMirr += wN * voteN.second;
                if (voteN.first == 2) sPar  += wN * voteN.second;
                if (voteG.first == 1) sMirr += wG * voteG.second;
                if (voteG.first == 2) sPar  += wG * voteG.second;

                const double score = std::abs(sMirr - sPar);
                if (outScore) *outScore = score;
                if (sMirr > sPar && score > 0.10) return 1;
                if (sPar > sMirr && score > 0.10) return 2;
                return 0;
            };

            auto procrustesPair = [&](int rSeg, int lSeg) {
                if (s2sNewMode[rSeg] == 2 || s2sNewMode[lSeg] == 2) return;
                double score = 0.0;
                const int mType = detectMountSymmetry(rSeg, lSeg, &score);
                mountType[rSeg] = mountType[lSeg] = mType;
                const Quat qR = s2sNew[rSeg];
                const Quat qL = s2sNew[lSeg];
                const double devMirr = mirrorYDeviationDeg(qR, qL);
                const double devPar  = parallelDeviationDeg(qR, qL);
                // FIX issue 4: trust strong vote (score >= 0.5) including
                // explicit "parallel" — old code defaulted to mirror when
                // mType==0 via devMirr<=devPar (which was the common case
                // for legs because gravity dominates).
                const bool trustVote = (score >= 0.5);
                const bool useMirror = (mType == 1) ||
                                       (mType == 0 && !trustVote && devMirr <= devPar);
                const double dev = useMirror ? devMirr : devPar;
                const char* sym = useMirror ? "mirror-Y" : "parallel";
                if (m_test) {
                    std::cout << "[calib refine] pair " << kSegmentNames[rSeg]
                              << "/" << kSegmentNames[lSeg]
                              << " mount=" << (mType == 1 ? "mirror-Y" : mType == 2 ? "parallel" : "unknown")
                              << " devMirr=" << std::fixed << std::setprecision(2) << devMirr << "°"
                              << " devPar=" << devPar << "°"
                              << " using=" << sym << "\n";
                }
                if (dev <= 3.0) return;
                if (dev >= 60.0) {
                    if (m_test) {
                        std::cout << "[calib refine] paired " << kSegmentNames[rSeg]
                                  << "/" << kSegmentNames[lSeg]
                                  << " " << sym << "_dev=" << std::fixed << std::setprecision(2) << dev
                                  << "° — too large, skipping (real asymmetry or sensor mount error)\n";
                    }
                    return;
                }

                if (s2sNewMode[rSeg] == 0 && s2sNewMode[lSeg] == 0) {
                    Quat counterpartL;
                    if (useMirror) {
                        counterpartL = mirror_y_quat(qL);
                    } else {
                        counterpartL = qL;
                    }
                    double dot = qR.w*counterpartL.w + qR.x*counterpartL.x
                               + qR.y*counterpartL.y + qR.z*counterpartL.z;
                    if (dot < 0.0) {
                        counterpartL = Quat(-counterpartL.w, -counterpartL.x,
                                            -counterpartL.y, -counterpartL.z);
                        dot = -dot;
                    }
                    const double wR = std::max(0.0, 1.0 - s2sResidual[rSeg] / 30.0);
                    const double wL = std::max(0.0, 1.0 - s2sResidual[lSeg] / 30.0);
                    const double wTot = wR + wL;
                    const double tR = (wTot > 1e-6) ? wL / wTot : 0.5;
                    const Quat qAvg = slerp_quat(qR, counterpartL, tR);
                    Quat qAvgForL;
                    if (useMirror) qAvgForL = Quat(qAvg.w, -qAvg.x, qAvg.y, -qAvg.z);
                    else            qAvgForL = qAvg;
                    s2sNew[rSeg] = qAvg;
                    s2sNew[lSeg] = qAvgForL;
                    s2sFused[rSeg] = s2sFused[lSeg] = true;
                    if (m_test) {
                        std::cout << "[calib refine] paired " << kSegmentNames[rSeg]
                                  << "/" << kSegmentNames[lSeg]
                                  << " averaged " << sym << ", dev was " << std::fixed << std::setprecision(2)
                                  << dev << "° (R=" << s2sResidual[rSeg]
                                  << "°/wR=" << wR << ", L=" << s2sResidual[lSeg]
                                  << "°/wL=" << wL << ")\n";
                    }
                    return;
                }

                int srcSeg, dstSeg;
                if (s2sNewMode[rSeg] == 0 && s2sNewMode[lSeg] != 0) { srcSeg = rSeg; dstSeg = lSeg; }
                else if (s2sNewMode[lSeg] == 0 && s2sNewMode[rSeg] != 0) { srcSeg = lSeg; dstSeg = rSeg; }
                else return;
                const Quat& qSrc = s2sNew[srcSeg];
                if (useMirror) s2sNew[dstSeg] = mirror_y_quat(qSrc);
                else            s2sNew[dstSeg] = qSrc;
                s2sFused[srcSeg] = s2sFused[dstSeg] = true;
                if (m_test) {
                    std::cout << "[calib refine] copied " << sym << " from " << kSegmentNames[srcSeg]
                              << " (triad) into " << kSegmentNames[dstSeg]
                              << " (ecomp), dev was " << std::fixed << std::setprecision(2)
                              << dev << "°\n";
                }
            };
            procrustesPair(SEG_RShoulder,  SEG_LShoulder);
            procrustesPair(SEG_RUpperArm,  SEG_LUpperArm);
            procrustesPair(SEG_RForearm,   SEG_LForearm);
            procrustesPair(SEG_RHand,      SEG_LHand);
            procrustesPair(SEG_RUpperLeg,  SEG_LUpperLeg);
            procrustesPair(SEG_RLowerLeg,  SEG_LLowerLeg);
            procrustesPair(SEG_RFoot,      SEG_LFoot);
            procrustesPair(SEG_RToe,       SEG_LToe);
        }

        {
            auto symYawS2S_K = [&](int rSeg, int lSeg) {
                if (s2sFused[rSeg] || s2sFused[lSeg]) return;
                const int mR = s2sNewMode[rSeg];
                const int mL = s2sNewMode[lSeg];
                if (mR == 2 || mL == 2) return;
                const bool bothTriad = (mR == 0 && mL == 0);
                const bool bothEcomp = (mR == 1 && mL == 1);
                if (!bothTriad && !bothEcomp) return;
                const double guard = bothTriad ? 0.95 : 0.7;
                const Quat& qR = s2sNew[rSeg];
                const Quat& qL = s2sNew[lSeg];
                const Quat yawR  = yaw_only_quat(qR);
                const Quat yawL  = yaw_only_quat(qL);
                const Quat yawLmir(yawL.w, 0.0, 0.0, -yawL.z);
                const double d  = yawR.w * yawLmir.w + yawR.z * yawLmir.z;
                if (std::abs(d) < guard) return;
                const Quat tiltR = quat_mult(qR, yawR.inv()).normalized();
                const Quat tiltL = quat_mult(qL, yawL.inv()).normalized();
                const double sgn = d < 0 ? -1.0 : 1.0;
                Quat yawAvg(0.5 * (yawR.w + sgn * yawLmir.w), 0.0, 0.0,
                            0.5 * (yawR.z + sgn * yawLmir.z));
                const double yn2 = yawAvg.w*yawAvg.w + yawAvg.z*yawAvg.z;
                if (yn2 < 1e-12) return;
                const double yn = 1.0 / std::sqrt(yn2);
                yawAvg.w *= yn; yawAvg.z *= yn;
                const Quat yawAvgMir(yawAvg.w, 0.0, 0.0, -yawAvg.z);
                s2sNew[rSeg] = quat_mult(yawAvg,    tiltR).normalized();
                s2sNew[lSeg] = quat_mult(yawAvgMir, tiltL).normalized();
            };
            symYawS2S_K(SEG_RShoulder,  SEG_LShoulder);
            symYawS2S_K(SEG_RUpperArm,  SEG_LUpperArm);
            symYawS2S_K(SEG_RForearm,   SEG_LForearm);
            symYawS2S_K(SEG_RHand,      SEG_LHand);
            symYawS2S_K(SEG_RUpperLeg,  SEG_LUpperLeg);
            symYawS2S_K(SEG_RLowerLeg,  SEG_LLowerLeg);
            symYawS2S_K(SEG_RFoot,      SEG_LFoot);
            symYawS2S_K(SEG_RToe,       SEG_LToe);
        }

        // FIX issue 1/4: yaw-anchor для ног через T↔K orientation diff.
        // T-pose: гравитация вдоль -Z body; K-pose (сидя, бёдра горизонтально):
        // гравитация в системе сегмента смещается, вектор aT × aK в сенсорной
        // системе указывает на ось вращения = ±Y body (lateral).  Применяем
        // s2s.inv() — получаем body-frame оценку оси.  Если итог отклоняется
        // от ±Y, это yaw-error монтажа сенсора (поворот вокруг bone axis);
        // компенсируем per-leg независимо.  Старая версия использовала
        // m_gyrAccumK, но накопитель фильтровал движения (kPerSegmentGyrLimit),
        // оставался ~bias→условие |g|>5°/s никогда не выполнялось.  Acc-based
        // axis работает на статических данных, которые мы уже копим.
        {
            auto correctLegYaw = [&](int seg, const Quat& s2s) -> Quat {
                const int cT = m_accumCountT[seg];
                const int cK = m_accumCountK[seg];
                if (cT < 10 || cK < 10) return s2s;
                const QVector3D aT = m_accAccumT[seg] / float(cT);
                const QVector3D aK = m_accAccumK[seg] / float(cK);
                if (aT.length() < 1e-3f || aK.length() < 1e-3f) return s2s;
                const QVector3D axisS = QVector3D::crossProduct(aT.normalized(),
                                                                 aK.normalized());
                if (axisS.length() < 0.3f) return s2s;        // позы коллинеарны
                const QVector3D axisB = vec_rotate(axisS.normalized(), s2s.inv());
                const double yawRes = std::atan2(double(axisB.y()), double(axisB.x()));
                const double expected = (axisB.y() > 0) ? +M_PI/2 : -M_PI/2;
                const double yawErr = std::atan2(std::sin(yawRes - expected),
                                                 std::cos(yawRes - expected));
                if (std::abs(yawErr) > 0.6) return s2s;
                // FIX (sign+order): для коррекции body-frame yaw ошибки нужно
                // композировать справа: new_s2s = s2s * Rot_Z(+yawErr).  Это
                // соответствует "сначала body-frame yawErr rotation, потом s2s".
                // Старая версия — quat_mult(Rot_Z(-yawErr), s2s) — давала
                // правильный результат ТОЛЬКО для чистых Z-rotation s2s; для
                // нетривиальных tilt'ов удваивала ошибку.  Не проявлялось
                // раньше потому что correctLegYaw был мёртв из-за gyro-порога.
                const Quat yawCorr = axisAngleQuat(QVector3D(0,0,1), yawErr);
                if (m_test) {
                    std::cout << "[calib K] leg yaw correction "
                              << kSegmentNames[seg]
                              << " yawErr=" << std::fixed << std::setprecision(2)
                              << yawErr * 180.0 / M_PI << "°\n";
                }
                return quat_mult(s2s, yawCorr).normalized();
            };
            for (int seg : { SEG_RUpperLeg, SEG_LUpperLeg,
                             SEG_RLowerLeg, SEG_LLowerLeg }) {
                s2sNew[seg] = correctLegYaw(seg, s2sNew[seg]);
            }
        }

        std::array<float, kXsensSegmentCount> segGain{};
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            const float noise = std::sqrt(float(gyrStdDev[i].lengthSquared() / 3.0f));
            float g = 0.7f;
            if (noise < 0.4f)      g = 0.78f;
            else if (noise > 2.5f) g = 0.62f;
            else                   g = 0.78f - (noise - 0.4f) / 2.1f * 0.16f;
            if (g < 0.62f) g = 0.62f;
            if (g > 0.82f) g = 0.82f;
            segGain[i] = g;
        }

        m_rx->setAccNormalisation(accMagn);
        m_rx->setGyroBias(gyrBias);
        int softIronCount = 0;
        for (int i = 0; i < kXsensSegmentCount; ++i) if (magSoftOk[i]) ++softIronCount;
        if (softIronCount >= 8) {
            m_rx->setMagSoftIron(magSoftMat, magSoftOff);
            if (m_test) {
                std::cout << "[mag soft-iron] fitted for " << softIronCount << "/"
                          << int(kXsensSegmentCount) << " segments — applied\n";
            }
        } else {
            m_rx->setMagNormalisation(magMagn);
            if (m_test) {
                std::cout << "[mag soft-iron] only " << softIronCount << "/"
                          << int(kXsensSegmentCount) << " segments fitted — falling back to scalar mag_magn\n";
            }
        }
        m_rx->setSegmentGain(segGain);
        m_rx->setS2sAlignment(s2sNew);

        for (int i = 0; i < kXsensSegmentCount; ++i) {
            const Quat delta = quat_mult(s2sOld[i].inv(), s2sNew[i]).normalized();
            m_result.calibReference[i] = quat_mult(m_result.calibReference[i], delta).normalized();
            if (m_result.tposeCaptured) {
                m_result.tposeReference[i] = quat_mult(m_result.tposeReference[i], delta).normalized();
            }
        }

        auto confidenceScore = [&](int i) -> double {
            const double cRes = confidenceFromResidual(s2sResidual[i], s2sNewMode[i]);
            const double cSamples = std::min(1.0, double(m_accumCountK[i]) / double(kCalibrationSamples));
            const double std3 = std::sqrt(double(gyrStdDev[i].lengthSquared() / 3.0));
            const double cGyr = std::max(0.0, 1.0 - std3 / 5.0);
            if (s2sNewMode[i] == 2) return 0.0;
            return std::min({cRes, cSamples, cGyr});
        };

        std::array<double, kXsensSegmentCount> confidence{};
        for (int i = 0; i < kXsensSegmentCount; ++i) confidence[i] = confidenceScore(i);

        const std::pair<int,int> pairsK[8] = {
            { SEG_RShoulder,  SEG_LShoulder  }, { SEG_RUpperArm,  SEG_LUpperArm  },
            { SEG_RForearm,   SEG_LForearm   }, { SEG_RHand,      SEG_LHand      },
            { SEG_RUpperLeg,  SEG_LUpperLeg  }, { SEG_RLowerLeg,  SEG_LLowerLeg  },
            { SEG_RFoot,      SEG_LFoot      }, { SEG_RToe,       SEG_LToe       },
        };
        for (const auto& pr : pairsK) {
            const double dev = mirrorYDeviationDeg(s2sNew[pr.first], s2sNew[pr.second]);
            const double f = std::max(0.0, 1.0 - dev / 30.0);
            confidence[pr.first]  = std::min(confidence[pr.first],  f);
            confidence[pr.second] = std::min(confidence[pr.second], f);
        }

        if (m_test) {
            static const char* kModeStrK[3] = {"triad ", "ecomp ", "ident "};
            std::cout << "[calib K] s2s refined (triple-pose):\n";
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                std::cout << "        " << std::left << std::setw(14)
                          << kSegmentNames[i] << std::right
                          << "  mode=" << kModeStrK[s2sNewMode[i]]
                          << "  confidence=" << std::fixed << std::setprecision(2) << confidence[i]
                          << "  (residual=" << s2sResidual[i] << "°"
                          << "  gyrStd=" << std::sqrt(double(gyrStdDev[i].lengthSquared() / 3.0)) << "°/s"
                          << "  gain=" << segGain[i] << ")\n";
            }
            std::cout.flush();
        }

        {
            double sumConf = 0.0;
            int problems = 0;
            QStringList problemList;
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                sumConf += confidence[i];
                if (confidence[i] < 0.6 && s2sNewMode[i] != 2) {
                    ++problems;
                    problemList << QString("%1(%2)")
                                    .arg(QString::fromUtf8(kSegmentNames[i]))
                                    .arg(confidence[i], 0, 'f', 2);
                }
            }
            const double avgConf = sumConf / double(kXsensSegmentCount);
            if (m_calibQuality) {
                QString qual = QString("Quality: %1").arg(avgConf, 0, 'f', 2);
                if (problems > 0)
                    qual += QString(" — issues: %1").arg(problemList.join(", "));
                m_calibQuality->setText(qual);
                m_calibQuality->setStyleSheet(avgConf > 0.8 ? "color:#2EC25A; font-weight:700;"
                                            : avgConf > 0.6 ? "color:#E0A030; font-weight:700;"
                                                            : "color:#E04040; font-weight:700;");
            }
        }

        m_phase = CalibPhase::Settle;
        const int gen = m_settleGen;
        constexpr int kSettleMsK = 6000;
        if (m_calibStatus)
            m_calibStatus->setText(Lang::t("calib_kpose_applied")
                                   .arg(kSettleMsK / 1000));
        if (m_readyBar) {
            m_readyBar->setRange(0, kSettleMsK);
            m_readyBar->setValue(0);
            m_readyBar->setFormat("%v / %m мс");
        }
        auto progressTimerK = std::make_shared<QTimer>(this);
        auto progressStartK = std::make_shared<qint64>(QDateTime::currentMSecsSinceEpoch());
        progressTimerK->setInterval(100);
        connect(progressTimerK.get(), &QTimer::timeout, this,
                [this, gen, progressTimerK, progressStartK, kSettleMsK]() {
            if (gen != m_settleGen) { progressTimerK->stop(); return; }
            const qint64 elapsed = QDateTime::currentMSecsSinceEpoch() - *progressStartK;
            if (m_readyBar) m_readyBar->setValue(int(std::min<qint64>(elapsed, kSettleMsK)));
            if (m_calibStatus) {
                const int rem = int(std::max<qint64>(0, kSettleMsK - elapsed));
                m_calibStatus->setText(Lang::t("calib_kpose_converge")
                                       .arg((rem + 999) / 1000));
            }
            if (elapsed >= kSettleMsK) progressTimerK->stop();
        });
        progressTimerK->start();
        QApplication::processEvents();

        auto baselineGyrBias = std::make_shared<std::array<QVector3D, kXsensSegmentCount>>(gyrBias);
        QTimer::singleShot(kSettleMsK - 1500, this, [this, gen, baselineGyrBias]() {
            if (gen != m_settleGen) return;
            const int kSamples = 8;
            std::array<QVector3D, kXsensSegmentCount> sums{};
            std::array<int,       kXsensSegmentCount> cnt{};
            for (int s = 0; s < kSamples; ++s) {
                const SuitPose fr = m_rx->snapshot();
                for (int i = 0; i < kXsensSegmentCount; ++i) {
                    if (fr.segValid[i]) { sums[i] += fr.gyrSensor[i]; ++cnt[i]; }
                }
                QThread::msleep(150);
            }
            std::array<QVector3D, kXsensSegmentCount> refined = *baselineGyrBias;
            int updated = 0;
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                if (cnt[i] < kSamples / 2) continue;
                const QVector3D liveMean = sums[i] / float(cnt[i]);
                const QVector3D base = (*baselineGyrBias)[i];
                const QVector3D delta = liveMean - base;
                if (delta.length() < 0.05f) continue;
                auto clamp = [](float v) { return v > 0.3f ? 0.3f : (v < -0.3f ? -0.3f : v); };
                refined[i] = QVector3D(clamp(liveMean.x()), clamp(liveMean.y()), clamp(liveMean.z()));
                ++updated;
            }
            if (updated > 0) {
                m_rx->setGyroBias(refined);
                if (m_test) {
                    std::cout << "[calib K] post-settle gyrBias refined for "
                              << updated << "/" << int(kXsensSegmentCount)
                              << " segments\n";
                    std::cout.flush();
                }
            }
        });

        QTimer::singleShot(kSettleMsK, this, [this, gen]() {
            if (gen != m_settleGen) return;
            m_phase = CalibPhase::Done;
            this->goNext();
        });
        return;
    }

    // ---------- CaptureN complete → finalise TRIAD/ecompass -----
    if (m_phase != CalibPhase::CaptureN) return;
    m_captureTimer.stop();
    m_calibComplete = true;
    m_phase = CalibPhase::Settle;
    if (m_calibStatus) m_calibStatus->setText(Lang::t("calib_ok"));
    testLog("[calib] N-pose capture complete, solving s2s…", m_test);

    // ----- hipose apply_imu_calibration terms --------------------
    //
    // Acc/mag/gyro statistics are averaged across BOTH poses (double the
    // samples → better bias estimate, and the mag norm should be consistent
    // in either stance).  The sensor-to-body orientation (`s2s`) is where
    // the two poses actually pay off:
    //
    //  * TRIAD when the body-frame gravity differs enough between poses
    //    (arms and hands flip ~90° T→N, so gravity in the forearm frame
    //    rotates from lateral to downward — two linearly-independent
    //    observations, yaw-free solve).
    //  * Single-pose ecompass fallback when gravity in the body frame is
    //    nearly parallel in both poses (torso, legs, feet — upright in
    //    both).  There's no extra info, so fall back to the mag-assisted
    //    N-pose estimate.
    const auto defAng_T = defaultSegAnglesFor("tpose");
    const auto defAng_N = defaultSegAnglesFor("npose");

    std::array<Quat,      kXsensSegmentCount> s2s{};
    std::array<double,    kXsensSegmentCount> magMagn{};
    std::array<double,    kXsensSegmentCount> accMagn{};
    std::array<QVector3D, kXsensSegmentCount> gyrBias{};
    std::array<int,       kXsensSegmentCount> s2sMode{}; // 0=triad, 1=ecompass, 2=identity
    std::array<double,    kXsensSegmentCount> s2sResidualN{};
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        s2s[i]     = Quat(1, 0, 0, 0);
        magMagn[i] = 1.0;
        accMagn[i] = 1.0;
        gyrBias[i] = QVector3D(0, 0, 0);
        s2sMode[i] = 2;
        s2sResidualN[i] = 99.0;
    }

    auto clampBias = [](float vv) {
        const float kMax = 0.2f;
        return vv > kMax ? kMax : (vv < -kMax ? -kMax : vv);
    };

    std::array<QVector3D, kXsensSegmentCount> gyrStdDevN{};
    std::array<std::array<double, 9>, kXsensSegmentCount> magSoftMatN{};
    std::array<QVector3D, kXsensSegmentCount>             magSoftOffN{};
    std::array<bool,      kXsensSegmentCount>             magSoftOkN{};
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        gyrStdDevN[i] = QVector3D(0, 0, 0);
        for (int k = 0; k < 9; ++k) magSoftMatN[i][k] = (k == 0 || k == 4 || k == 8) ? 1.0 : 0.0;
        magSoftOffN[i] = QVector3D(0, 0, 0);
        magSoftOkN[i] = false;
    }

    for (int i = 0; i < kXsensSegmentCount; ++i) {
        const int cT = m_accumCountT[i];
        const int cN = m_accumCountN[i];
        if (cT < 10 || cN < 10) continue;

        const double invT = 1.0 / double(cT);
        const double invN = 1.0 / double(cN);
        const QVector3D meanA_T = m_accAccumT[i] * float(invT);
        const QVector3D meanA_N = m_accAccumN[i] * float(invN);
        const QVector3D meanG_T = m_gyrAccumT[i] * float(invT);
        const QVector3D meanG_N = m_gyrAccumN[i] * float(invN);
        const QVector3D meanM_T = m_magAccumT[i] * float(invT);
        const QVector3D meanM_N = m_magAccumN[i] * float(invN);
        if (meanA_T.length() < 1e-6 || meanA_N.length() < 1e-6 ||
            meanM_N.length() < 1e-6) continue;

        const int    cTot = cT + cN;
        const double invTot = 1.0 / double(cTot);
        accMagn[i] = (m_accMagAccumT[i] + m_accMagAccumN[i]) * invTot;
        const QVector3D meanM_all = (m_magAccumT[i] + m_magAccumN[i]) * float(invTot);
        magMagn[i] = double(meanM_all.length());

        const QVector3D gSqT = m_gyrSqAccumT[i] * float(invT);
        const QVector3D gSqN = m_gyrSqAccumN[i] * float(invN);
        const QVector3D vT(std::max(0.0f, gSqT.x() - meanG_T.x()*meanG_T.x()),
                           std::max(0.0f, gSqT.y() - meanG_T.y()*meanG_T.y()),
                           std::max(0.0f, gSqT.z() - meanG_T.z()*meanG_T.z()));
        const QVector3D vN(std::max(0.0f, gSqN.x() - meanG_N.x()*meanG_N.x()),
                           std::max(0.0f, gSqN.y() - meanG_N.y()*meanG_N.y()),
                           std::max(0.0f, gSqN.z() - meanG_N.z()*meanG_N.z()));
        auto medianBiasTN = [](double mT, double mN) -> double {
            if (std::abs(mT - mN) > 0.4) return std::abs(mT) < std::abs(mN) ? mT : mN;
            return 0.5 * (mT + mN);
        };
        const double biasX = medianBiasTN(meanG_T.x(), meanG_N.x());
        const double biasY = medianBiasTN(meanG_T.y(), meanG_N.y());
        const double biasZ = medianBiasTN(meanG_T.z(), meanG_N.z());
        gyrBias[i] = QVector3D(clampBias(float(biasX)),
                               clampBias(float(biasY)),
                               clampBias(float(biasZ)));
        const QVector3D vAvg = (vT * float(cT) + vN * float(cN)) * float(invTot);
        gyrStdDevN[i] = QVector3D(std::sqrt(vAvg.x()), std::sqrt(vAvg.y()), std::sqrt(vAvg.z()));

        QVector3D magSumAll = m_magAccumT[i] + m_magAccumN[i];
        double outerAll[6] = {0,0,0,0,0,0};
        for (int k = 0; k < 6; ++k) {
            outerAll[k] = m_magOuterAccumT[i][k] + m_magOuterAccumN[i][k];
        }
        const SoftIronFit si = fitMagSoftIron(magSumAll, outerAll, cTot);
        if (si.valid && si.residual < 5.0) {
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    magSoftMatN[i][r * 3 + c] = si.M[r][c];
            magSoftOffN[i] = si.offset;
            magSoftOkN[i] = true;
        }

        const QVector3D aT_s = meanA_T.normalized();
        const QVector3D aN_s = meanA_N.normalized();
        const QVector3D gT_b = gravityInBodyFrame(defAng_T[i]).normalized();
        const QVector3D gN_b = gravityInBodyFrame(defAng_N[i]).normalized();
        const float sep = QVector3D::crossProduct(gT_b, gN_b).length();

        if (sep > 0.3f) {
            const Quat q_s2b = triadSolve(gT_b, gN_b, aT_s, aN_s);
            s2s[i] = q_s2b.inv().normalized();
            s2sMode[i] = 0;
            s2sResidualN[i] = triadResidualDeg(q_s2b, gT_b, gN_b, aT_s, aN_s);
            if (m_test) {
                std::cout << "[calib] " << kSegmentNames[i]
                          << " TRIAD T+N sep=" << std::fixed << std::setprecision(3) << sep
                          << " residual=" << std::setprecision(2) << s2sResidualN[i] << "°\n";
            }
        } else {
            const QVector3D mNorm = meanM_N / float(meanM_N.length());
            Quat qNed = ecompassNED(aN_s, mNorm);
            const Quat nedToNwu(0, 1, 0, 0);
            const Quat qNwu = quat_mult(nedToNwu, qNed).normalized();
            s2s[i] = qNwu.inv().normalized();
            s2sMode[i] = 1;
            const QVector3D worldGravity(0.0f, 0.0f, -1.0f);
            s2sResidualN[i] = ecompResidualDeg(qNwu, worldGravity, aN_s);
            if (m_test) {
                std::cout << "[calib] " << kSegmentNames[i]
                          << " ecompass T+N residual=" << std::fixed << std::setprecision(2)
                          << s2sResidualN[i] << "°\n";
            }
        }
    }

    {
        const std::pair<int,int> pairsN[8] = {
            { SEG_RShoulder,  SEG_LShoulder  }, { SEG_RUpperArm,  SEG_LUpperArm  },
            { SEG_RForearm,   SEG_LForearm   }, { SEG_RHand,      SEG_LHand      },
            { SEG_RUpperLeg,  SEG_LUpperLeg  }, { SEG_RLowerLeg,  SEG_LLowerLeg  },
            { SEG_RFoot,      SEG_LFoot      }, { SEG_RToe,       SEG_LToe       },
        };
        for (const auto& pr : pairsN) {
            if (s2sMode[pr.first] == 2 || s2sMode[pr.second] == 2) continue;
            const Quat qR = s2s[pr.first];
            const Quat qL = s2s[pr.second];
            const Quat mYL = mirror_y_quat(qL);
            const double dotMY = qR.w*mYL.w + qR.x*mYL.x + qR.y*mYL.y + qR.z*mYL.z;
            const double dotPar = qR.w*qL.w + qR.x*qL.x + qR.y*qL.y + qR.z*qL.z;
            const double devMirr = mirrorYDeviationDeg(qR, qL);
            const double devPar  = parallelDeviationDeg(qR, qL);
            const bool mirrLikely = (devMirr < 30.0) && (devMirr + 10.0 < devPar);
            const bool parLikely  = (devPar  < 30.0) && (devPar  + 10.0 < devMirr);
            if (!mirrLikely && !parLikely) {
                if (m_test) {
                    std::cout << "[calib] hemisphere-unify skipped " << kSegmentNames[pr.first]
                              << "/" << kSegmentNames[pr.second]
                              << " (no clear symmetry: devMirr=" << std::fixed << std::setprecision(1)
                              << devMirr << "° devPar=" << devPar << "°)\n";
                }
                continue;
            }
            const bool needFlip = mirrLikely ? (dotMY < 0.0) : (dotPar < 0.0);
            if (needFlip) {
                s2s[pr.second] = Quat(-qL.w, -qL.x, -qL.y, -qL.z);
                if (m_test) {
                    std::cout << "[calib] hemisphere-unify L " << kSegmentNames[pr.second]
                              << " via " << (mirrLikely ? "mirror-Y" : "parallel")
                              << " (was opposite of R " << kSegmentNames[pr.first] << ")\n";
                }
            }
        }
    }

    std::array<bool, kXsensSegmentCount> s2sFusedN{};
    {
        // FIX issue 4 (parallel-path в режиме без K-pose).  Расширяем
        // mount-detection T+N accel + T↔N axis cross product.  Для рук T- и
        // N-позы дают сильно различающиеся гравитации (рука вниз vs вперёд),
        // axis = aT × aN указывает body-lateral в сенсорной системе и хорошо
        // различает mirror-Y vs parallel.  Для ног T- и N-позы близки (обе
        // стоя), axis слабый — остаётся acc T+N path.
        auto voteFromPairN = [&](const QVector3D& vR, const QVector3D& vL) -> std::pair<int, double> {
            if (vR.length() < 1e-3f || vL.length() < 1e-3f) return {0, 0.0};
            const QVector3D nR = vR.normalized();
            const QVector3D nL = vL.normalized();
            const QVector3D mY_nL(nL.x(), -nL.y(), nL.z());
            const double mirrDot = double(QVector3D::dotProduct(nR, mY_nL));
            const double parDot  = double(QVector3D::dotProduct(nR, nL));
            if (std::abs(mirrDot) < 0.05 && std::abs(parDot) < 0.05) return {0, 0.0};
            const int type   = (std::abs(mirrDot) >= std::abs(parDot)) ? 1 : 2;
            const double sc  = std::abs(std::abs(mirrDot) - std::abs(parDot));
            return {type, sc};
        };
        auto detectMountSymmetryN = [&](int rSeg, int lSeg, double* outScore = nullptr) -> int {
            if (m_accumCountT[rSeg] < 10 || m_accumCountT[lSeg] < 10) {
                if (outScore) *outScore = 0.0;
                return 0;
            }
            const bool isLeg = (rSeg == SEG_RUpperLeg) || (rSeg == SEG_LUpperLeg)
                            || (rSeg == SEG_RLowerLeg) || (rSeg == SEG_LLowerLeg);
            const QVector3D aR_T = m_accAccumT[rSeg] / float(m_accumCountT[rSeg]);
            const QVector3D aL_T = m_accAccumT[lSeg] / float(m_accumCountT[lSeg]);
            auto voteT = voteFromPairN(aR_T, aL_T);

            std::pair<int, double> voteN = {0, 0.0};
            QVector3D aR_N(0, 0, 0), aL_N(0, 0, 0);
            const bool haveN = (m_accumCountN[rSeg] >= 10 && m_accumCountN[lSeg] >= 10);
            if (haveN) {
                aR_N = m_accAccumN[rSeg] / float(m_accumCountN[rSeg]);
                aL_N = m_accAccumN[lSeg] / float(m_accumCountN[lSeg]);
                voteN = voteFromPairN(aR_N, aL_N);
            }

            // Axis vote T↔N — для рук работает (>~60° поза-разделение),
            // для ног слабо (легs остаются вертикально в обеих позах).
            std::pair<int, double> voteAx = {0, 0.0};
            if (haveN && aR_T.length() > 1e-3f && aL_T.length() > 1e-3f
                      && aR_N.length() > 1e-3f && aL_N.length() > 1e-3f) {
                const QVector3D axR = QVector3D::crossProduct(aR_T.normalized(),
                                                               aR_N.normalized());
                const QVector3D axL = QVector3D::crossProduct(aL_T.normalized(),
                                                               aL_N.normalized());
                if (axR.length() > 0.3f && axL.length() > 0.3f) {
                    voteAx = voteFromPairN(axR, axL);
                }
            }

            double wT = 1.0, wN = 0.0, wAx = 0.0;
            if (isLeg) {
                wT = 0.4; wN = 0.6; wAx = 0.0;
            } else if (voteAx.second > 0.05) {
                wT = 0.4; wN = 0.0; wAx = 0.6;
            }

            double sMirr = 0.0, sPar = 0.0;
            if (voteT.first == 1) sMirr += wT * voteT.second;
            if (voteT.first == 2) sPar  += wT * voteT.second;
            if (voteN.first == 1) sMirr += wN * voteN.second;
            if (voteN.first == 2) sPar  += wN * voteN.second;
            if (voteAx.first == 1) sMirr += wAx * voteAx.second;
            if (voteAx.first == 2) sPar  += wAx * voteAx.second;

            const double score = std::abs(sMirr - sPar);
            if (outScore) *outScore = score;
            if (sMirr > sPar && score > 0.10) return 1;
            if (sPar > sMirr && score > 0.10) return 2;
            return 0;
        };
        auto procrustesPairN = [&](int rSeg, int lSeg) {
            if (s2sMode[rSeg] == 2 || s2sMode[lSeg] == 2) return;
            double score = 0.0;
            const int mType = detectMountSymmetryN(rSeg, lSeg, &score);
            const Quat qR = s2s[rSeg];
            const Quat qL = s2s[lSeg];
            const double devMirr = mirrorYDeviationDeg(qR, qL);
            const double devPar  = parallelDeviationDeg(qR, qL);
            const bool trustVote = (score >= 0.5);
            const bool useMirror = (mType == 1) ||
                                   (mType == 0 && !trustVote && devMirr <= devPar);
            const double dev = useMirror ? devMirr : devPar;
            const char* sym = useMirror ? "mirror-Y" : "parallel";
            if (m_test) {
                std::cout << "[calib refine] pair " << kSegmentNames[rSeg]
                          << "/" << kSegmentNames[lSeg]
                          << " mount=" << (mType == 1 ? "mirror-Y" : mType == 2 ? "parallel" : "unknown")
                          << " devMirr=" << std::fixed << std::setprecision(2) << devMirr << "°"
                          << " devPar=" << devPar << "°"
                          << " using=" << sym << "\n";
            }
            if (dev <= 3.0) return;
            if (dev >= 60.0) {
                if (m_test) {
                    std::cout << "[calib refine] paired " << kSegmentNames[rSeg]
                              << "/" << kSegmentNames[lSeg]
                              << " " << sym << "_dev=" << std::fixed << std::setprecision(2) << dev
                              << "° — too large, skipping (real asymmetry or sensor mount error)\n";
                }
                return;
            }

            if (s2sMode[rSeg] == 0 && s2sMode[lSeg] == 0) {
                Quat counterpartL;
                if (useMirror) counterpartL = mirror_y_quat(qL);
                else           counterpartL = qL;
                double dot = qR.w*counterpartL.w + qR.x*counterpartL.x
                           + qR.y*counterpartL.y + qR.z*counterpartL.z;
                if (dot < 0.0) {
                    counterpartL = Quat(-counterpartL.w, -counterpartL.x,
                                        -counterpartL.y, -counterpartL.z);
                    dot = -dot;
                }
                const double wR = std::max(0.0, 1.0 - s2sResidualN[rSeg] / 30.0);
                const double wL = std::max(0.0, 1.0 - s2sResidualN[lSeg] / 30.0);
                const double wTot = wR + wL;
                const double tR = (wTot > 1e-6) ? wL / wTot : 0.5;
                const Quat qAvg = slerp_quat(qR, counterpartL, tR);
                Quat qAvgForL;
                if (useMirror) qAvgForL = Quat(qAvg.w, -qAvg.x, qAvg.y, -qAvg.z);
                else            qAvgForL = qAvg;
                s2s[rSeg] = qAvg;
                s2s[lSeg] = qAvgForL;
                s2sFusedN[rSeg] = s2sFusedN[lSeg] = true;
                if (m_test) {
                    std::cout << "[calib refine] paired " << kSegmentNames[rSeg]
                              << "/" << kSegmentNames[lSeg]
                              << " averaged " << sym << ", dev was " << std::fixed << std::setprecision(2)
                              << dev << "° (residuals R=" << s2sResidualN[rSeg]
                              << "° L=" << s2sResidualN[lSeg] << "°)\n";
                }
                return;
            }

            int srcSeg, dstSeg;
            if (s2sMode[rSeg] == 0 && s2sMode[lSeg] != 0) { srcSeg = rSeg; dstSeg = lSeg; }
            else if (s2sMode[lSeg] == 0 && s2sMode[rSeg] != 0) { srcSeg = lSeg; dstSeg = rSeg; }
            else return;
            const Quat& qSrc = s2s[srcSeg];
            if (useMirror) s2s[dstSeg] = mirror_y_quat(qSrc);
            else            s2s[dstSeg] = qSrc;
            s2sFusedN[srcSeg] = s2sFusedN[dstSeg] = true;
            if (m_test) {
                std::cout << "[calib refine] copied " << sym << " from " << kSegmentNames[srcSeg]
                          << " (triad) into " << kSegmentNames[dstSeg]
                          << " (ecomp), dev was " << std::fixed << std::setprecision(2)
                          << dev << "°\n";
            }
        };
        procrustesPairN(SEG_RShoulder,  SEG_LShoulder);
        procrustesPairN(SEG_RUpperArm,  SEG_LUpperArm);
        procrustesPairN(SEG_RForearm,   SEG_LForearm);
        procrustesPairN(SEG_RHand,      SEG_LHand);
        procrustesPairN(SEG_RUpperLeg,  SEG_LUpperLeg);
        procrustesPairN(SEG_RLowerLeg,  SEG_LLowerLeg);
        procrustesPairN(SEG_RFoot,      SEG_LFoot);
        procrustesPairN(SEG_RToe,       SEG_LToe);
    }

    {
        auto symYawS2S = [&](int rSeg, int lSeg) {
            if (s2sFusedN[rSeg] || s2sFusedN[lSeg]) return;
            const int mR = s2sMode[rSeg];
            const int mL = s2sMode[lSeg];
            if (mR == 2 || mL == 2) return;
            const bool bothTriad = (mR == 0 && mL == 0);
            const bool bothEcomp = (mR == 1 && mL == 1);
            if (!bothTriad && !bothEcomp) return;
            const double guard = bothTriad ? 0.95 : 0.7;
            const Quat& qR = s2s[rSeg];
            const Quat& qL = s2s[lSeg];
            const Quat yawR  = yaw_only_quat(qR);
            const Quat yawL  = yaw_only_quat(qL);
            const Quat yawLmir(yawL.w, 0.0, 0.0, -yawL.z);
            const double d  = yawR.w * yawLmir.w + yawR.z * yawLmir.z;
            if (std::abs(d) < guard) return;
            const Quat tiltR = quat_mult(qR, yawR.inv()).normalized();
            const Quat tiltL = quat_mult(qL, yawL.inv()).normalized();
            const double sgn = d < 0 ? -1.0 : 1.0;
            Quat yawAvg(0.5 * (yawR.w + sgn * yawLmir.w),
                        0.0, 0.0,
                        0.5 * (yawR.z + sgn * yawLmir.z));
            const double yn2 = yawAvg.w*yawAvg.w + yawAvg.z*yawAvg.z;
            if (yn2 < 1e-12) return;
            const double yn = 1.0 / std::sqrt(yn2);
            yawAvg.w *= yn; yawAvg.z *= yn;
            const Quat yawAvgMir(yawAvg.w, 0.0, 0.0, -yawAvg.z);
            s2s[rSeg] = quat_mult(yawAvg,    tiltR).normalized();
            s2s[lSeg] = quat_mult(yawAvgMir, tiltL).normalized();
        };
        symYawS2S(SEG_RShoulder,  SEG_LShoulder);
        symYawS2S(SEG_RUpperArm,  SEG_LUpperArm);
        symYawS2S(SEG_RForearm,   SEG_LForearm);
        symYawS2S(SEG_RHand,      SEG_LHand);
        symYawS2S(SEG_RUpperLeg,  SEG_LUpperLeg);
        symYawS2S(SEG_RLowerLeg,  SEG_LLowerLeg);
        symYawS2S(SEG_RFoot,      SEG_LFoot);
        symYawS2S(SEG_RToe,       SEG_LToe);
    }

    std::array<float, kXsensSegmentCount> segGainN{};
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        const float noise = std::sqrt(float(gyrStdDevN[i].lengthSquared() / 3.0f));
        float g = 0.7f;
        if (noise < 0.4f)      g = 0.78f;
        else if (noise > 2.5f) g = 0.62f;
        else                   g = 0.78f - (noise - 0.4f) / 2.1f * 0.16f;
        if (g < 0.62f) g = 0.62f;
        if (g > 0.82f) g = 0.82f;
        segGainN[i] = g;
    }

    m_rx->setAccNormalisation(accMagn);
    m_rx->setGyroBias(gyrBias);
    int softIronCountN = 0;
    for (int i = 0; i < kXsensSegmentCount; ++i) if (magSoftOkN[i]) ++softIronCountN;
    if (softIronCountN >= 8) {
        m_rx->setMagSoftIron(magSoftMatN, magSoftOffN);
        if (m_test) {
            std::cout << "[mag soft-iron] fitted for " << softIronCountN << "/"
                      << int(kXsensSegmentCount) << " segments (T+N) — applied\n";
        }
    } else {
        m_rx->setMagNormalisation(magMagn);
        if (m_test) {
            std::cout << "[mag soft-iron] only " << softIronCountN << "/"
                      << int(kXsensSegmentCount) << " segments fitted in T+N — falling back to scalar mag_magn\n";
        }
    }
    m_rx->setSegmentGain(segGainN);

    m_rx->setS2sAlignment(s2s);

    if (m_test) {
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            const bool isArm = (i >= SEG_RUpperArm && i <= SEG_RHand)
                            || (i >= SEG_LUpperArm && i <= SEG_LHand);
            const bool isLeg = (i >= SEG_RUpperLeg && i <= SEG_LToe);
            if (!isArm && !isLeg) continue;
            const double absW = std::min(1.0, std::abs(s2s[i].w));
            const double angleDeg = 2.0 * std::acos(absW) * 180.0 / M_PI;
            if (angleDeg > 60.0) {
                std::cout << "[calib] WARNING: " << kSegmentNames[i]
                          << " s2s rotation = " << std::fixed << std::setprecision(1)
                          << angleDeg << "deg (>60deg) — actor may not have held the pose, "
                          << "sensor placement may be off, or recalibration is recommended\n";
            }
        }
        std::cout.flush();
    }

    if (m_test) {
        static const char* kModeStr[3] = {"triad ", "ecomp ", "ident "};
        std::cout << std::fixed << std::setprecision(5);
        std::cout << "[calib] hipose calibration terms (double-pose):\n";
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            std::cout << "        " << std::left << std::setw(14)
                      << kSegmentNames[i] << std::right
                      << "  mode="   << kModeStr[s2sMode[i]]
                      << "  acc_magn=" << accMagn[i]
                      << "  mag_magn=" << magMagn[i]
                      << "  gyr_bias=(" << gyrBias[i].x() << ","
                                        << gyrBias[i].y() << ","
                                        << gyrBias[i].z() << ") deg/s\n";
        }
        std::cout.flush();
    }

    // ----- Multi-snapshot reference + L/R symmetry (in N-pose) -----
    constexpr int kSettleMs        = 8000;
    constexpr int kSnapshotCount   = 16;
    constexpr int kSnapshotStepMs  = 250;
    constexpr int kTotalCalibMs    = 12500;
    if (m_calibStatus)
        m_calibStatus->setText(Lang::t("calib_npose_settle")
                               .arg(kTotalCalibMs / 1000));
    if (m_readyBar) {
        m_readyBar->setRange(0, kTotalCalibMs);
        m_readyBar->setValue(0);
        m_readyBar->setFormat("%p %");
    }
    const int gen = m_settleGen;
    // Declared before the progress timer so the timer lambda can reflect the
    // real captured-snapshot count in the second half of the bar.
    auto snapshots = std::make_shared<std::vector<
        std::array<Quat, kXsensSegmentCount>>>();
    auto badCount     = std::make_shared<int>(0);
    auto droppedCount = std::make_shared<int>(0);
    auto progressTimer = std::make_shared<QTimer>(this);
    auto progressStart = std::make_shared<qint64>(QDateTime::currentMSecsSinceEpoch());
    progressTimer->setInterval(100);
    connect(progressTimer.get(), &QTimer::timeout, this,
            [this, gen, snapshots, progressTimer, progressStart, kTotalCalibMs, kSettleMs, kSnapshotCount, kSnapshotStepMs]() {
        if (gen != m_settleGen) { progressTimer->stop(); return; }
        const qint64 elapsed = QDateTime::currentMSecsSinceEpoch() - *progressStart;
        // Honest progress: first half = timed filter-convergence wait, second
        // half = REAL captured-snapshot count (stalls on dropped snapshots).
        if (m_readyBar) {
            if (elapsed < kSettleMs) {
                m_readyBar->setValue(int(kTotalCalibMs / 2
                    * std::min<qint64>(elapsed, kSettleMs) / kSettleMs));
            } else {
                const int captured = int(snapshots->size());
                m_readyBar->setValue(kTotalCalibMs / 2
                    + (kTotalCalibMs / 2) * std::min(captured, kSnapshotCount) / kSnapshotCount);
            }
        }
        if (m_calibStatus) {
            if (elapsed < kSettleMs) {
                const int remaining = int(std::max<qint64>(0, kSettleMs - elapsed));
                m_calibStatus->setText(Lang::t("calib_npose_converge")
                                       .arg((remaining + 999) / 1000));
            } else {
                const int captureIdx = int(std::min(kSnapshotCount - 1,
                    int((elapsed - kSettleMs) / kSnapshotStepMs) + 1));
                m_calibStatus->setText(Lang::t("calib_npose_capture")
                                       .arg(captureIdx).arg(kSnapshotCount));
            }
        }
        if (elapsed >= kTotalCalibMs) progressTimer->stop();
    });
    progressTimer->start();
    QApplication::processEvents();

    for (int k = 0; k < kSnapshotCount; ++k) {
        const int delayMs = kSettleMs + k * kSnapshotStepMs;
        QTimer::singleShot(delayMs, this, [this, gen, snapshots, badCount, droppedCount, k]() {
            if (gen != m_settleGen) return;
            const SuitPose post = m_rx->snapshot();
            const double pelvisGyrDegS = double(post.gyrSensor[SEG_Pelvis].length());
            const double rArmGyr  = double(post.gyrSensor[SEG_RUpperArm].length());
            const double lArmGyr  = double(post.gyrSensor[SEG_LUpperArm].length());
            const double rLegGyr  = double(post.gyrSensor[SEG_RUpperLeg].length());
            const double lLegGyr  = double(post.gyrSensor[SEG_LUpperLeg].length());
            const double maxGyr   = std::max({pelvisGyrDegS, rArmGyr, lArmGyr, rLegGyr, lLegGyr});
            if (maxGyr > 8.0) {
                ++(*droppedCount);
                if (m_test) {
                    std::cout << "[calib] dropping snapshot " << k
                              << " — actor moved (pelvisGyr=" << pelvisGyrDegS
                              << " maxGyr=" << maxGyr << " deg/s)\n";
                    std::cout.flush();
                }
                return;
            }
            std::array<Quat, kXsensSegmentCount> sn{};
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                if (post.segValid[i]) sn[i] = post.quat[i];
                else { sn[i] = Quat(1, 0, 0, 0); if (k==0) ++(*badCount); }
            }
            snapshots->push_back(sn);
        });
    }

    QTimer::singleShot(12500, this, [this, gen, snapshots, badCount, droppedCount]() {
        if (gen != m_settleGen) return;
        if (m_test) {
            std::cout << "[calib] N-pose snapshots collected: " << snapshots->size()
                      << " kept, " << *droppedCount << " dropped (actor moved)\n";
            std::cout.flush();
        }
        if (snapshots->empty()) {
            // Don't fail silently — tell the operator the pose produced no
            // usable data so they can recalibrate instead of recording a
            // session built on a degenerate single-frame reference.
            QMessageBox::warning(this, Lang::t("calib_title"),
                                 Lang::t("calib_pose_empty"));
            const SuitPose post = m_rx->snapshot();
            for (int i = 0; i < kXsensSegmentCount; ++i)
                m_result.calibReference[i] = post.segValid[i] ?
                    post.quat[i] : Quat(1, 0, 0, 0);
            m_phase = CalibPhase::Done;
            this->goNext();
            return;
        }

        // Hemisphere-corrected mean across all snapshots.
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            double sw=0, sx=0, sy=0, sz=0;
            const Quat& anchor = (*snapshots)[0][i];
            for (const auto& s : *snapshots) {
                const double d = s[i].w*anchor.w + s[i].x*anchor.x
                               + s[i].y*anchor.y + s[i].z*anchor.z;
                const double sgn = d < 0 ? -1.0 : 1.0;
                sw += sgn*s[i].w; sx += sgn*s[i].x;
                sy += sgn*s[i].y; sz += sgn*s[i].z;
            }
            m_result.calibReference[i] = Quat(sw, sx, sy, sz).normalized();
        }

        if (m_test) {
            std::cout << "[calib] multi-snapshot reference + symmetry "
                         "applied, snapshots=" << snapshots->size()
                      << " bad_first=" << *badCount << "\n";
            std::cout << std::fixed << std::setprecision(4);
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                const Quat& q = m_result.calibReference[i];
                std::cout << "        " << std::left << std::setw(14)
                          << kSegmentNames[i] << std::right
                          << " (" << q.w << "," << q.x << ","
                          << q.y << "," << q.z << ")\n";
            }
            std::cout.flush();
        }

        for (int i = 0; i < kXsensSegmentCount; ++i) {
            m_accAccumK[i]    = QVector3D(0, 0, 0);
            m_gyrAccumK[i]    = QVector3D(0, 0, 0);
            m_magAccumK[i]    = QVector3D(0, 0, 0);
            m_accMagAccumK[i] = 0.0;
            m_accumCountK[i]  = 0;
            m_gyrSqAccumK[i]  = QVector3D(0, 0, 0);
            for (int k = 0; k < 6; ++k) m_magOuterAccumK[i][k] = 0.0;
        }
        m_accAccum    = &m_accAccumK;
        m_gyrAccum    = &m_gyrAccumK;
        m_magAccum    = &m_magAccumK;
        m_accMagAccum = &m_accMagAccumK;
        m_accumCount  = &m_accumCountK;
        m_gyrSqAccum    = &m_gyrSqAccumK;
        m_magOuterAccum = &m_magOuterAccumK;

        m_phase = CalibPhase::PrepK;
        refreshPoseImage();
        if (m_poseHint)
            m_poseHint->setText(Lang::t("kpose_hint"));
        if (m_calibStatus)
            m_calibStatus->setText(Lang::t("calib_k_prepare"));

        m_countTicksLeft = kCountdownSeconds * 10;
        m_countdownBar->setValue(0);
        m_readyBar->setRange(0, kCalibrationSamples);
        m_readyBar->setValue(0);
        m_readyBar->setFormat("%v / %m");
        m_goodSamples = 0;
        m_havePrev    = false;
        m_samples.clear();
        m_countLabel->setText(QString::number(kCountdownSeconds));
        m_countTimer.start();
        updateNavButtons();
    });
}

void NewSessionWizard::closeEvent(QCloseEvent* e)
{
    ++m_settleGen;          // drop any pending settle callbacks before teardown
    m_countTimer.stop();
    m_captureTimer.stop();
    m_statusTimer.stop();
    QDialog::closeEvent(e);
}

// ============================================================================
//  SensorIndicatorsPanel
// ============================================================================

// Pretty, human-readable labels for every tracker segment.  Falls back to
// the lowercase internal name when a translation key is missing, which
// keeps the panel readable even before the UI strings are loaded.
static const char* kSensorLabelKey[kXsensSegmentCount] = {
    "sns_pelvis",    "",              "",              "",
    "sns_t8",        "",              "sns_head",
    "sns_r_shoulder","sns_r_upper_arm","sns_r_forearm","sns_r_hand",
    "sns_l_shoulder","sns_l_upper_arm","sns_l_forearm","sns_l_hand",
    "sns_r_upper_leg","sns_r_lower_leg","sns_r_foot",  "",
    "sns_l_upper_leg","sns_l_lower_leg","sns_l_foot",  "",
};

SensorIndicatorsPanel::SensorIndicatorsPanel(bool useGloves, QWidget* parent)
    : QWidget(parent), m_useGloves(useGloves)
{
    setObjectName("sidePanel");
    setFixedWidth(340);

    // ---- Top mode / status card ------------------------------------------
    auto* headerMode  = new QLabel(this);
    headerMode->setObjectName("sectionHeader");
    headerMode->setProperty("isModeHeader", true);

    m_lblMode = new QLabel(this);
    m_lblMode->setStyleSheet("font-size:11pt; font-weight:700; color:#FFFFFF;");

    auto* headerConn = new QLabel(this);
    headerConn->setObjectName("sectionHeader");
    headerConn->setProperty("isConnHeader", true);

    m_lblSuit = new QLabel(this);
    paintDot(m_lblSuit, "#555");
    auto* lblSuitTxt = new QLabel(this);
    lblSuitTxt->setProperty("isSuitLabel", true);
    lblSuitTxt->setStyleSheet("font-weight:700; color:#FFFFFF;");
    auto* suitRow = new QHBoxLayout();
    suitRow->addWidget(m_lblSuit);
    suitRow->addSpacing(8);
    suitRow->addWidget(lblSuitTxt, 1);

    m_lblFps = new QLabel("—", this);
    m_lblFps->setStyleSheet("color:#FF7A1A; font-weight:700;");
    auto* fpsLbl = new QLabel(this);
    fpsLbl->setProperty("isFpsLabel", true);
    fpsLbl->setStyleSheet("color:#9B9B9B;");
    auto* fpsRow = new QHBoxLayout();
    fpsRow->addWidget(fpsLbl);
    fpsRow->addStretch();
    fpsRow->addWidget(m_lblFps);

    // Battery row — populated from SuitPose.batteryLevel each tick.
    m_lblBattery = new QLabel("—", this);
    m_lblBattery->setStyleSheet("color:#2EC25A; font-weight:700;");
    auto* batLbl = new QLabel(Lang::t("battery_label"), this);
    batLbl->setProperty("isBatteryLbl", true);
    batLbl->setStyleSheet("color:#9B9B9B;");
    auto* batRow = new QHBoxLayout();
    batRow->addWidget(batLbl);
    batRow->addStretch();
    batRow->addWidget(m_lblBattery);

    auto* statusCard = new QWidget(this);
    statusCard->setObjectName("statusCard");
    auto* statusLay = new QVBoxLayout(statusCard);
    statusLay->setContentsMargins(14, 12, 14, 12);
    statusLay->setSpacing(6);
    statusLay->addLayout(suitRow);
    statusLay->addLayout(fpsRow);
    statusLay->addLayout(batRow);

    // ---- Sensor card grid -------------------------------------------------
    // Two clean columns of paired body segments — L on the left column, R
    // on the right — with centred spine/head rows that span both.  Every
    // card has the same width so the whole grid lines up vertically.
    auto* headerSensors = new QLabel(this);
    headerSensors->setObjectName("sectionHeader");
    headerSensors->setProperty("isSensorsHeader", true);

    auto makeCard = [this](int seg, QWidget* parent = nullptr) -> QWidget* {
        auto* card = new QWidget(parent ? parent : this);
        card->setObjectName("sensorCard");
        card->setProperty("sensorState", "off");
        auto* dot  = new QLabel(card);
        dot->setFixedSize(10, 10);
        dot->setStyleSheet("background:#555; border-radius:5px;");
        auto* name = new QLabel(QString::fromLatin1(kSegmentNames[seg]), card);
        name->setProperty("isSensorName", true);
        name->setProperty("segIdx", seg);
        name->setStyleSheet("color:#EAEAEA; font-weight:600;");
        m_trackers[seg] = { dot, name };
        auto* lay = new QHBoxLayout(card);
        lay->setContentsMargins(10, 6, 10, 6);
        lay->setSpacing(8);
        lay->addWidget(dot);
        lay->addWidget(name, 1);
        card->setMinimumHeight(30);
        return card;
    };

    auto* grid = new QGridLayout();
    grid->setHorizontalSpacing(6);
    grid->setVerticalSpacing(6);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);

    // Row 0..2 — spine / head, centred across both columns.
    grid->addWidget(makeCard(SEG_Head),    0, 0, 1, 2);
    grid->addWidget(makeCard(SEG_T8),      1, 0, 1, 2);
    grid->addWidget(makeCard(SEG_Pelvis),  2, 0, 1, 2);

    // Paired limbs — left column = L side, right column = R side.
    struct Pair { int left; int right; };
    const Pair pairs[] = {
        { SEG_LShoulder, SEG_RShoulder  },
        { SEG_LUpperArm, SEG_RUpperArm  },
        { SEG_LForearm,  SEG_RForearm   },
        { SEG_LHand,     SEG_RHand      },
        { SEG_LUpperLeg, SEG_RUpperLeg  },
        { SEG_LLowerLeg, SEG_RLowerLeg  },
        { SEG_LFoot,     SEG_RFoot      },
    };
    int r = 3;
    for (const Pair& pp : pairs) {
        grid->addWidget(makeCard(pp.left),  r, 0);
        grid->addWidget(makeCard(pp.right), r, 1);
        ++r;
    }

    // ---- Reset + Freeze buttons, directly under the sensor grid ---------
    // Reset: skeleton snaps to scene origin (0, 0), feet on floor.
    // Freeze (toggle): pin XY while allowing local sit/stand. One-button toggle.
    m_btnReset = new QPushButton(this);
    m_btnReset->setObjectName("primary");
    m_btnReset->setMinimumHeight(38);
    m_btnReset->setCursor(Qt::PointingHandCursor);
    connect(m_btnReset, &QPushButton::clicked, this, [this]() {
        emit resetClicked();
    });

    m_btnFreeze = new QPushButton(this);
    m_btnFreeze->setObjectName("primary");
    m_btnFreeze->setMinimumHeight(38);
    m_btnFreeze->setCursor(Qt::PointingHandCursor);
    m_btnFreeze->setCheckable(true);
    connect(m_btnFreeze, &QPushButton::toggled, this, [this](bool on) {
        m_frozen = on;
        m_btnFreeze->setText(Lang::t(on ? "unfreeze_coords" : "freeze_coords"));
        emit freezeToggled(on);
    });

    // ---- Fingers (only shown when gloves are selected) -------------------
    // Paired thumb/index/middle/ring/pinky — left hand in left column,
    // right hand in right column, same card look as the suit grid.
    m_fingersBox = new QWidget(this);
    // Start hidden — it reappears from updateFromPose() as soon as glove
    // data actually flows.  Prevents an empty finger block from lingering
    // when the operator selected gloves mode but ManusCore never replied.
    m_fingersBox->setVisible(false);
    auto* fHeader = new QLabel(m_fingersBox);
    fHeader->setObjectName("sectionHeader");
    fHeader->setProperty("isFingersHeader", true);
    auto* fSub = new QLabel(m_fingersBox);
    fSub->setObjectName("sectionSub");
    fSub->setProperty("isFingersSub", true);
    auto* fGrid = new QGridLayout();
    fGrid->setHorizontalSpacing(6);
    fGrid->setVerticalSpacing(6);
    fGrid->setColumnStretch(0, 1);
    fGrid->setColumnStretch(1, 1);
    static const char* kFingerKey[5] = {
        "fng_thumb", "fng_index", "fng_middle", "fng_ring", "fng_pinky"
    };
    auto makeFingerCard = [this](int idx, const char* key) -> QWidget* {
        auto* card = new QWidget(m_fingersBox);
        card->setObjectName("sensorCard");
        card->setProperty("sensorState", "off");
        auto* dot  = new QLabel(card);
        dot->setFixedSize(10, 10);
        dot->setStyleSheet("background:#555; border-radius:5px;");
        auto* name = new QLabel(Lang::t(key), card);
        name->setProperty("isFingerName", true);
        name->setProperty("fngKey", key);
        name->setStyleSheet("color:#EAEAEA; font-weight:600;");
        m_fingers[idx] = { dot, name };
        auto* clay = new QHBoxLayout(card);
        clay->setContentsMargins(10, 6, 10, 6);
        clay->setSpacing(8);
        clay->addWidget(dot);
        clay->addWidget(name, 1);
        card->setMinimumHeight(30);
        return card;
    };
    for (int f = 0; f < 5; ++f) {
        // Panel order: left hand on the left column, right hand on the
        // right, paired by finger — thumb row, index row, etc.
        fGrid->addWidget(makeFingerCard(5 + f, kFingerKey[f]), f, 0); // L
        fGrid->addWidget(makeFingerCard(f,     kFingerKey[f]), f, 1); // R
    }
    auto* fLay = new QVBoxLayout(m_fingersBox);
    fLay->setContentsMargins(0, 6, 0, 0);
    fLay->setSpacing(0);
    fLay->addWidget(fHeader);
    fLay->addWidget(fSub);
    fLay->addSpacing(6);
    fLay->addLayout(fGrid);

    // Kept for API compatibility — the session label is no longer shown,
    // but SensorIndicatorsPanel::setSessionRunning() still updates it.
    m_lblSession = new QLabel(this);
    m_lblSession->setVisible(false);

    // Sub-captions under each section header — quick "what / how many"
    // summaries so it's obvious at a glance which cards represent what.
    auto* subConn    = new QLabel(this);
    subConn->setObjectName("sectionSub");
    subConn->setProperty("isConnSub", true);
    auto* subSensors = new QLabel(this);
    subSensors->setObjectName("sectionSub");
    subSensors->setProperty("isSensorsSub", true);

    m_bodyBox = new QWidget(this);
    m_bodyHeader = new QLabel(m_bodyBox);
    m_bodyHeader->setObjectName("sectionHeader");
    m_bodySub = new QLabel(m_bodyBox);
    m_bodySub->setObjectName("sectionSub");

    // FIX: убрали блок ввода размеров из вьюпорта.  Все 4 параметра
    // (рост / стопа / размах рук / длина ноги) теперь вводятся ОДИН РАЗ
    // в NewSessionWizard перед стартом сессии.  Виджет m_bodyBox
    // остаётся как контейнер, но содержит только информационный заголовок
    // (без spinbox'ов и сигнала actorChanged).
    m_bodyHeight = nullptr;
    m_bodyFoot   = nullptr;
    m_bodyArm    = nullptr;
    m_bodyLeg    = nullptr;
    m_bodyLblH   = nullptr;
    m_bodyLblF   = nullptr;
    m_bodyLblA   = nullptr;
    m_bodyLblL   = nullptr;
    m_bodyDebounce = nullptr;

    auto* bLay = new QVBoxLayout(m_bodyBox);
    bLay->setContentsMargins(0, 6, 0, 0);
    bLay->setSpacing(2);
    bLay->addWidget(m_bodyHeader);
    bLay->addWidget(m_bodySub);
    // m_bodyBox теперь занимает мало места; полностью спрячем если хотите —
    // но оставляем заголовок как индикатор «Body sizes set in wizard»,
    // чтобы пользователь видел что блок не пропал, а перенесён.
    m_bodyBox->setVisible(false);  // полностью убираем из вьюпорта

    auto* lay = new QVBoxLayout(this);
    lay->setContentsMargins(14, 14, 14, 14);
    lay->setSpacing(0);
    lay->addWidget(headerMode);
    lay->addWidget(m_lblMode);
    lay->addSpacing(10);
    lay->addWidget(headerConn);
    lay->addWidget(subConn);
    lay->addSpacing(4);
    lay->addWidget(statusCard);
    lay->addSpacing(12);
    lay->addWidget(headerSensors);
    lay->addWidget(subSensors);
    lay->addSpacing(6);
    lay->addLayout(grid);
    lay->addSpacing(10);
    lay->addWidget(m_btnReset);
    lay->addSpacing(6);
    lay->addWidget(m_btnFreeze);
    lay->addSpacing(10);
    lay->addWidget(m_bodyBox);
    lay->addSpacing(10);
    lay->addWidget(m_fingersBox);
    lay->addStretch(1);

    connect(&Lang::instance(), &Lang::changed, this, &SensorIndicatorsPanel::retranslate);
    setSuitLive(false, Lang::t("suit_scanning"));
    setSessionRunning(true);
    retranslate();
}

void SensorIndicatorsPanel::setActorDefaults(const ActorConfig& a)
{
    if (!m_bodyHeight) return;
    m_bodySuppress = true;
    m_bodyHeight->setValue(a.heightCm);
    m_bodyFoot->setValue(a.footLengthCm);
    m_bodyArm->setValue(a.armSpanCm);
    m_bodyLeg->setValue(a.legLengthCm);
    m_bodySuppress = false;
}

void SensorIndicatorsPanel::retranslate()
{
    m_lblMode->setText(m_useGloves
        ? Lang::t("suit_with_gloves")
        : Lang::t("suit_only"));
    for (QLabel* lab : findChildren<QLabel*>()) {
        if (lab->property("isModeHeader").toBool())     lab->setText(Lang::t("mode_label"));
        if (lab->property("isConnHeader").toBool())     lab->setText(Lang::t("session_label"));
        if (lab->property("isSensorsHeader").toBool())  lab->setText(Lang::t("sensors_label"));
        if (lab->property("isSensorsSub").toBool())     lab->setText(Lang::t("sensors_sub"));
        if (lab->property("isConnSub").toBool())        lab->setText(Lang::t("conn_sub"));
        if (lab->property("isFingersHeader").toBool())  lab->setText(Lang::t("fingers_label"));
        if (lab->property("isFingersSub").toBool())     lab->setText(Lang::t("fingers_sub"));
        if (lab->property("isSessionHeader").toBool())  lab->setText(Lang::t("session_label"));
        if (lab->property("isFpsLabel").toBool())       lab->setText(Lang::t("fps_label") + ":");
        if (lab->property("isSensorName").toBool()) {
            const int seg = lab->property("segIdx").toInt();
            if (seg >= 0 && seg < kXsensSegmentCount) {
                const char* k = kSensorLabelKey[seg];
                lab->setText((k && *k) ? Lang::t(k)
                                       : QString::fromLatin1(kSegmentNames[seg]));
            }
        }
        if (lab->property("isFingerName").toBool()) {
            const QString k = lab->property("fngKey").toString();
            if (!k.isEmpty()) lab->setText(Lang::t(k.toUtf8().constData()));
        }
    }
    if (m_btnReset)  m_btnReset->setText(Lang::t("reset_coords"));
    if (m_btnFreeze) m_btnFreeze->setText(Lang::t(
        m_frozen ? "unfreeze_coords" : "freeze_coords"));
    m_lblSession->setText(m_running ? Lang::t("session_running")
                                     : Lang::t("session_paused"));
    if (m_bodyHeader) m_bodyHeader->setText(Lang::t("body_panel_label"));
    if (m_bodySub)    m_bodySub->setText(Lang::t("body_panel_sub"));
    if (m_bodyLblH)   m_bodyLblH->setText(Lang::t("body_height"));
    if (m_bodyLblF)   m_bodyLblF->setText(Lang::t("foot_length"));
    if (m_bodyLblA)   m_bodyLblA->setText(Lang::t("body_arm_span"));
    if (m_bodyLblL)   m_bodyLblL->setText(Lang::t("body_leg_length"));
}

void SensorIndicatorsPanel::setFreezeState(bool frozen)
{
    if (m_frozen == frozen) return;
    m_frozen = frozen;
    if (m_btnFreeze) {
        QSignalBlocker bl(m_btnFreeze);
        m_btnFreeze->setChecked(frozen);
        m_btnFreeze->setText(Lang::t(frozen ? "unfreeze_coords" : "freeze_coords"));
    }
}

void SensorIndicatorsPanel::setMode(bool useGloves)
{
    m_useGloves = useGloves;
    if (m_fingersBox) m_fingersBox->setVisible(useGloves);
    retranslate();
}

void SensorIndicatorsPanel::setDot(QLabel* dot, bool live)
{
    if (!dot) return;
    // Dot + its containing card share state — restyling the dot alone
    // looks half-baked, so we update the whole parent card through a
    // dynamic property and let QSS do the colour work.
    dot->setStyleSheet(live ? "background:#2EC25A; border-radius:5px;"
                            : "background:#C03838; border-radius:5px;");
    if (QWidget* card = dot->parentWidget()) {
        const QString st = live ? "on" : "off";
        if (card->property("sensorState").toString() != st) {
            card->setProperty("sensorState", st);
            card->style()->unpolish(card);
            card->style()->polish(card);
        }
    }
}

void SensorIndicatorsPanel::setSuitLive(bool live, const QString& /*detail*/)
{
    paintDot(m_lblSuit, live ? "#2EC25A" : "#C03838");
    for (QLabel* lab : findChildren<QLabel*>()) {
        if (lab->property("isSuitLabel").toBool()) {
            lab->setText(live ? Lang::t("suit_connected")
                              : Lang::t("suit_disconnected"));
        }
    }
}

void SensorIndicatorsPanel::setFps(double hz) { m_lblFps->setText(QString("%1 Hz").arg(hz, 0, 'f', 1)); }

void SensorIndicatorsPanel::setSessionRunning(bool running)
{
    m_running = running;
    if (m_lblSession)
        m_lblSession->setText(running ? Lang::t("session_running")
                                       : Lang::t("session_paused"));
    // Reset / Freeze act on the live skeleton — disable them while the session
    // is paused (suit down) so the operator can't fire them with nothing to act
    // on, and re-enable on resume.
    if (m_btnReset)  { m_btnReset->setEnabled(running);  m_btnReset->setText(Lang::t("reset_coords")); }
    if (m_btnFreeze) { m_btnFreeze->setEnabled(running); m_btnFreeze->setText(Lang::t(
        m_frozen ? "unfreeze_coords" : "freeze_coords")); }
}

void SensorIndicatorsPanel::updateFromPose(const SuitPose& f)
{
    // Per-sensor staleness: green if received packet within 2s, else gray.
    // Uses segLastT (populated in MocapReceiver::run). If suit disconnects
    // mid-session all dots go gray immediately as timestamps age.
    using clk = std::chrono::steady_clock;
    const double nowSec = std::chrono::duration<double>(
                              clk::now().time_since_epoch()).count();
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        if (!m_trackers[i].dot) continue;
        const double age = nowSec - f.segLastT[i];
        const bool fresh = f.segValid[i] && f.segLastT[i] > 0.0 && age < 2.0;
        setDot(m_trackers[i].dot, fresh);
    }
    // Finger panel: only show when gloves are actually streaming now.
    const bool showFingers = m_useGloves && f.hasGloves;
    if (m_fingersBox && m_fingersBox->isVisible() != showFingers)
        m_fingersBox->setVisible(showFingers);
    if (showFingers) {
        for (int i = 0; i < 10; ++i)
            if (m_fingers[i].dot) setDot(m_fingers[i].dot, true);
    } else {
        // Gloves disconnected mid-session — grey out finger dots.
        for (int i = 0; i < 10; ++i)
            if (m_fingers[i].dot) setDot(m_fingers[i].dot, false);
    }
    // Battery display.
    if (m_lblBattery) {
        if (f.batteryLevel < 0) {
            m_lblBattery->setText("—");
            m_lblBattery->setStyleSheet("color:#888; font-weight:700;");
        } else {
            const QString txt = QString::number(f.batteryLevel) + "%";
            m_lblBattery->setText(txt);
            const char* color = (f.batteryLevel >= 30) ? "#2EC25A"
                              : (f.batteryLevel >= 15) ? "#F5A623" : "#E04040";
            m_lblBattery->setStyleSheet(
                QString("color:%1; font-weight:700;").arg(color));
        }
    }
}

// ============================================================================
//  MocapViewport — simple GL orbit view of the skeleton bones.
// ============================================================================

MocapViewport::MocapViewport(const ActorConfig& actor, const std::string& pose,
                             QWidget* parent)
    : QOpenGLWidget(parent), m_actor(actor)
{
    m_skel = std::make_unique<SkeletonXsens>(actor, pose);
    m_loco.setActorHeight(actor.heightCm / 100.0);
    m_loco.setFootLength(actor.footLengthCm / 100.0);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
}

void MocapViewport::setPose(const std::string& pose)
{
    m_skel = std::make_unique<SkeletonXsens>(m_actor, pose);
    m_loco.reset();           // fresh calibration = fresh anchor
    m_loco.setActorHeight(m_actor.heightCm / 100.0);
    m_loco.setFootLength(m_actor.footLengthCm / 100.0);
    update();
}

void MocapViewport::setActor(const ActorConfig& actor)
{
    if (m_actor.heightCm == actor.heightCm
        && m_actor.footLengthCm == actor.footLengthCm
        && m_actor.armSpanCm == actor.armSpanCm
        && m_actor.legLengthCm == actor.legLengthCm
        && m_actor.hipWidthCm == actor.hipWidthCm
        && m_actor.shoulderWidthCm == actor.shoulderWidthCm
        && m_actor.trunkLengthCm == actor.trunkLengthCm
        && m_actor.useGloves == actor.useGloves)
        return;
    m_actor = actor;
    if (m_skel) {
        const std::string pose = m_skel->poseKind();
        m_skel = std::make_unique<SkeletonXsens>(actor, pose);
    }
    m_loco.setActorHeight(actor.heightCm / 100.0);
    m_loco.setFootLength(actor.footLengthCm / 100.0);
    update();
}

// FIX (gloves polish): pin wrist drift anchor from T-pose calibration.
// Сейчас m_anchorLocal[wrist] обновляется при каждом lock-моменте; если
// первый lock попал на кривую позу — anchor зафиксирован неверно.
// С T-pose anchor anchor берётся из 500-кадрового averaging T-позы
// (actor стоит ладонями вниз), что даёт стабильный анатомический ноль.
// После вызова — m_locked-моменты больше не перезаписывают anchor.
void MocapViewport::setTposeHandAnchor(const Quat& fR, const Quat& hR,
                                       const Quat& fL, const Quat& hL)
{
    const Quat dA_h_R = m_skel ? m_skel->defAngFor(SEG_RHand)    : Quat(1, 0, 0, 0);
    const Quat dA_f_R = m_skel ? m_skel->defAngFor(SEG_RForearm) : Quat(1, 0, 0, 0);
    const Quat dA_h_L = m_skel ? m_skel->defAngFor(SEG_LHand)    : Quat(1, 0, 0, 0);
    const Quat dA_f_L = m_skel ? m_skel->defAngFor(SEG_LForearm) : Quat(1, 0, 0, 0);
    const Quat fW_R = quat_mult(fR, dA_f_R).normalized();
    const Quat hW_R = quat_mult(hR, dA_h_R).normalized();
    const Quat fW_L = quat_mult(fL, dA_f_L).normalized();
    const Quat hW_L = quat_mult(hL, dA_h_L).normalized();
    m_anchorLocal[SEG_RHand] = quat_mult(fW_R.inv(), hW_R).normalized();
    m_anchorLocal[SEG_LHand] = quat_mult(fW_L.inv(), hW_L).normalized();
    m_anchorValid[SEG_RHand] = true;
    m_anchorValid[SEG_LHand] = true;
    m_tposeHandAnchorValid   = true;
    m_driftLocal[SEG_RHand]  = Quat(1, 0, 0, 0);
    m_driftLocal[SEG_LHand]  = Quat(1, 0, 0, 0);
}

// FIX (T-pose foot direction reference): pin foot-yaw anchor from T-pose
// calibration.  В T-pose обе стопы смотрят вперёд по +X.  Сохраняем
// foot-quat в pelvis-yaw frame как ground truth.  Это лучше анкора по
// lowerLeg (используемого в FIX issue 7), потому что lowerLeg меняется
// при flex'е колена 30-60° → искажает yaw reference.  Pelvis-yaw frame
// стабилен — меняется только при повороте таза.
//
// defAngFor(SEG_Pelvis) != identity (в T-pose это Rot_Y(-π/2)), поэтому
// сначала переводим raw pelvis sensor в world, потом извлекаем yaw.
// Аналогично для стоп — но defAngFor(SEG_RFoot/LFoot) == identity, так что
// raw sensor quat = world quat.
void MocapViewport::setTposeFootAnchor(const Quat& pelvis_T,
                                       const Quat& rFoot_T, const Quat& lFoot_T)
{
    const Quat dA_pel = m_skel ? m_skel->defAngFor(SEG_Pelvis) : Quat(1, 0, 0, 0);
    const Quat dA_rf  = m_skel ? m_skel->defAngFor(SEG_RFoot)  : Quat(1, 0, 0, 0);
    const Quat dA_lf  = m_skel ? m_skel->defAngFor(SEG_LFoot)  : Quat(1, 0, 0, 0);
    const Quat pelvisWorld = quat_mult(pelvis_T, dA_pel).normalized();
    const Quat rFootWorld  = quat_mult(rFoot_T,  dA_rf).normalized();
    const Quat lFootWorld  = quat_mult(lFoot_T,  dA_lf).normalized();
    const Quat pelvisYaw    = yaw_only_quat(pelvisWorld);
    const Quat pelvisYawInv = pelvisYaw.inv();
    m_tposeFootRefPelR = quat_mult(pelvisYawInv, rFootWorld).normalized();
    m_tposeFootRefPelL = quat_mult(pelvisYawInv, lFootWorld).normalized();
    m_tposeFootAnchorValid = true;
}

void MocapViewport::updatePose(const std::array<Quat, kXsensSegmentCount>& orient,
                               const QVector3D& root)
{
    // Per-segment drift-lock. Render-side only — does NOT touch motion pipeline.
    // Rule: if a bone's angular speed stays <0.8 deg/s for >0.5 s AND angular
    // acceleration is low (drift is linear in time → ~zero accel), freeze its
    // output to the locked quat. Real motion starts with non-zero accel or
    // speed >0.8 deg/s → lock releases within 1 frame.
    using clk = std::chrono::steady_clock;
    const double now = std::chrono::duration<double>(
                           clk::now().time_since_epoch()).count();
    const double dt = (m_lastRenderT > 0.0)
                      ? std::max(1e-3, std::min(0.5, now - m_lastRenderT))
                      : 1.0 / 60.0;
    m_lastRenderT = now;

    std::array<Quat, kXsensSegmentCount> filtered = orient;

    auto nlerpQ = [](const Quat& a, const Quat& b, double t) -> Quat {
        const double d = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
        const double s = (d < 0.0) ? -1.0 : 1.0;
        Quat out(
            (1.0 - t) * a.w + t * s * b.w,
            (1.0 - t) * a.x + t * s * b.x,
            (1.0 - t) * a.y + t * s * b.y,
            (1.0 - t) * a.z + t * s * b.z);
        return out.normalized();
    };

    auto twistAngle = [](const Quat& q, const QVector3D& axisU) -> double {
        const double dotR = q.x*axisU.x() + q.y*axisU.y() + q.z*axisU.z();
        Quat tw(q.w, dotR*axisU.x(), dotR*axisU.y(), dotR*axisU.z());
        const double tn2 = tw.w*tw.w + tw.x*tw.x + tw.y*tw.y + tw.z*tw.z;
        if (tn2 < 1e-12) return 0.0;
        const double tn = std::sqrt(tn2);
        const double tw_w = tw.w / tn;
        const double sw = std::min(1.0, std::abs(tw_w));
        const double sgnTw = (tw_w < 0.0) ? -1.0 : 1.0;
        const double sgnR  = (dotR >= 0.0) ? 1.0 : -1.0;
        return 2.0 * sgnR * sgnTw * std::acos(sw);
    };

    double pelvisYawRate = 0.0;
    if (m_havePrevQ) {
        const Quat qDP = quat_mult(orient[SEG_Pelvis], m_prevQ[SEG_Pelvis].inv()).normalized();
        const QVector3D zUp(0, 0, 1);
        pelvisYawRate = std::abs(twistAngle(qDP, zUp) / dt);
    }

    if (m_havePrevQ) {
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            // Angular speed from |q_delta| angle per dt.
            const Quat dq = quat_mult(orient[i], m_prevQ[i].inv()).normalized();
            const double w = std::abs(dq.w) > 1.0 ? 1.0 : std::abs(dq.w);
            const double angRad = 2.0 * std::acos(w);
            const double angVel = angRad * 180.0 / M_PI / dt;    // deg/s
            // LP smooth so single-frame noise doesn't release lock.
            const double alpha = rateAdjustAlpha(0.30, dt);
            m_angVelLP[i] = (1.0 - alpha) * m_angVelLP[i] + alpha * angVel;
            // Angular acceleration magnitude (change of speed).
            const double angAcc = std::abs(angVel - m_angVelPrev[i]) / dt;
            m_angVelPrev[i] = angVel;
            m_dbgAngAcc[i]  = angAcc;   // -test: lock-gate "steady" input

            const bool isWrist  = (i == SEG_RHand || i == SEG_LHand);
            const bool isFootSeg = (i == SEG_RFoot || i == SEG_LFoot
                                  || i == SEG_RToe || i == SEG_LToe);
            const double slowThresh   = isWrist ? 0.6  : (isFootSeg ? 0.6  : 0.8);
            const double steadyThresh = isWrist ? 12.0 : (isFootSeg ? 14.0 : 20.0);
            const double lockTime     = isWrist ? 0.20 : (isFootSeg ? 0.25 : 0.50);

            // Threshold tuned for realism:
            //  - drift is typically 0.1-0.5 deg/s with ~0 acceleration
            //  - finger tremor, breathing: 1-3 deg/s with non-zero accel
            //  - intentional motion: >5 deg/s and/or accel >50 deg/s²
            const bool slow = m_angVelLP[i] < slowThresh;
            const bool steady = angAcc < steadyThresh;
            const bool stillFrame = slow && steady;

            if (stillFrame) {
                m_stillTicks[i] += dt;
                if (m_stillTicks[i] > lockTime && !m_locked[i]) {
                    // Engage lock — freeze at current quat.
                    m_locked[i]   = true;
                    m_lockQuat[i] = m_prevQ[i];
                    // FIX (terminator smoothing): start lock-in ramp at 0 → 1
                    // over ~7 frames (77ms @90Hz).  Without this lock jumps
                    // from orient[i] straight to m_lockQuat[i] which may
                    // differ by 0.5°-1° → visible step.
                    m_lockBlend[i] = 0.0;
                }
            } else {
                // Motion detected → release lock, reset counter.
                m_stillTicks[i] = 0.0;
                m_locked[i] = false;
            }

            if (m_locked[i]) {
                // FIX (terminator smoothing): lock-in blend — ramp от orient
                // в сторону m_lockQuat за ~77ms.  m_lockBlend[i]==1 → full
                // lock как раньше; <1 → linear-ish blend.
                if (m_lockBlend[i] < 1.0) {
                    filtered[i] = nlerpQ(orient[i], m_lockQuat[i], m_lockBlend[i]);
                    m_lockBlend[i] = std::min(1.0, m_lockBlend[i] + 0.15 * (dt * 90.0));
                } else {
                    filtered[i] = m_lockQuat[i];
                }
                m_unlockBlend[i] = 0.30;
            } else if (m_unlockBlend[i] < 1.0) {
                filtered[i] = nlerpQ(m_outPrevQ[i], orient[i], m_unlockBlend[i]);
                m_unlockBlend[i] = std::min(1.0, m_unlockBlend[i] + 0.15 * (dt * 90.0));
            } else {
                filtered[i] = orient[i];
            }

            if (i == SEG_RHand || i == SEG_LHand) {
                const int iForearm = (i == SEG_RHand) ? SEG_RForearm : SEG_LForearm;
                const double faAngV = m_angVelLP[iForearm];
                const bool calmHand   = m_angVelLP[i] < 2.0;
                const bool calmFA     = faAngV < 2.0;
                const bool calmPelvis = pelvisYawRate < 0.20;
                const bool allCalm = calmHand && calmFA && calmPelvis;

                // FIX issue 11: счётчик "спокойствия" — растёт пока allCalm
                // держится непрерывно.  При >=5 сек применяем доп. damped
                // twist коррекцию (она снижает накопленный yaw дрейф вокруг
                // продольной оси предплечья).
                if (allCalm) m_calmSeconds[i] += dt;
                else         m_calmSeconds[i] = 0.0;

                const Quat dA_h = m_skel ? m_skel->defAngFor(i)        : Quat(1, 0, 0, 0);
                const Quat dA_f = m_skel ? m_skel->defAngFor(iForearm) : Quat(1, 0, 0, 0);
                const Quat hWorld = quat_mult(orient[i],        dA_h).normalized();
                const Quat fWorld = quat_mult(orient[iForearm], dA_f).normalized();

                if (m_locked[i]) {
                    // FIX (gloves polish): когда T-pose anchor pinned —
                    // НЕ перезаписываем anchor lock-моментами.  Кисть
                    // всегда стремится к T-pose геометрии (ладонь вниз).
                    if (!m_tposeHandAnchorValid) {
                        m_anchorLocal[i] = quat_mult(fWorld.inv(), hWorld).normalized();
                        m_anchorValid[i] = true;
                    }
                    m_driftLocal[i]  = nlerpQ(m_driftLocal[i], Quat(1, 0, 0, 0),
                                              std::min(1.0, dt / 5.0));
                } else if (m_anchorValid[i]) {
                    const Quat current_local = quat_mult(fWorld.inv(), hWorld).normalized();
                    Quat drift = quat_mult(current_local, m_anchorLocal[i].inv()).normalized();
                    if (drift.w < 0) {
                        drift.w = -drift.w; drift.x = -drift.x;
                        drift.y = -drift.y; drift.z = -drift.z;
                    }
                    const double sw = std::min(1.0, std::abs(drift.w));
                    const double driftAngle = 2.0 * std::acos(sw);
                    // FIX issue 11: бюджет 45°→30°.  Wrist в покое редко
                    // отклоняется от anchor>30°; сужение даёт быстрее
                    // отрабатывать "медленное плавание" кисти.
                    const double driftBudget = M_PI / 6.0;

                    if (allCalm && driftAngle < driftBudget) {
                        const double alpha = std::min(1.0, dt / 3.0);
                        m_driftLocal[i] = nlerpQ(m_driftLocal[i], drift, alpha);
                    } else if (!allCalm) {
                        const double decayAlpha = std::min(1.0, dt / 1.0);
                        m_driftLocal[i] = nlerpQ(m_driftLocal[i], Quat(1, 0, 0, 0), decayAlpha);
                    }

                    const Quat& dL = m_driftLocal[i];
                    const double dw = std::min(1.0, std::abs(dL.w));
                    if (2.0 * std::acos(dw) > 1e-4) {
                        Quat dLSwing, dLTwist;
                        swingTwistDecompose(dL, QVector3D(1.0f, 0.0f, 0.0f), dLSwing, dLTwist);
                        const Quat dLSwingInv = dLSwing.inv();
                        const Quat correctionWorld = quat_mult(fWorld, quat_mult(dLSwingInv, fWorld.inv())).normalized();
                        const Quat hCorrectedWorld = quat_mult(correctionWorld, hWorld).normalized();
                        filtered[i] = quat_mult(hCorrectedWorld, dA_h.inv()).normalized();

                        // FIX issue 11: damped twist correction при долгой
                        // стабильности.  Берём 50% твист-коррекции —
                        // полная может резко крутнуть thumb при дрейфе на
                        // границе бюджета.  Только когда allCalm >= 5s.
                        if (m_calmSeconds[i] >= 5.0) {
                            const Quat dLTwistInv = dLTwist.inv();
                            const Quat twistWorld = quat_mult(fWorld,
                                                       quat_mult(dLTwistInv, fWorld.inv())).normalized();
                            const Quat twistHalf  = slerp_quat(Quat(1,0,0,0), twistWorld, 0.5);
                            const Quat hWithTwist = quat_mult(twistHalf,
                                                       quat_mult(filtered[i], dA_h)).normalized();
                            filtered[i] = quat_mult(hWithTwist, dA_h.inv()).normalized();
                        }
                    }
                }
                const auto& cfg = (i == SEG_RHand) ? m_wristCfgR : m_wristCfgL;
                if (cfg.enabled && !m_locked[i]) {
                    const Quat hWorldFiltered = quat_mult(filtered[i],        dA_h).normalized();
                    const Quat fWorldFiltered = quat_mult(filtered[iForearm], dA_f).normalized();
                    // FIX (gloves polish): clamp wrist flex/lat-dev
                    // относительно T-pose anchor (= ладонь вниз).  Если
                    // anchor невалиден (T-pose skip) — identity = старое
                    // поведение.
                    const Quat anchorLocal = m_anchorValid[i]
                            ? m_anchorLocal[i] : Quat(1, 0, 0, 0);
                    const Quat hConstrainedWorld = constrain_wrist_twist(
                            hWorldFiltered, fWorldFiltered, anchorLocal,
                            cfg.maxFlexRad, cfg.maxLatDevRad, cfg.twistWeight,
                            i == SEG_RHand);
                    filtered[i] = quat_mult(hConstrainedWorld, dA_h.inv()).normalized();
                }
            }

            // FIX issue 7: foot yaw stabilization при перекрёстных позах.
            // Аналогично wrist-блоку, но другая ось разложения — корректируем
            // только yaw (вокруг world-Z), бюджет 15°, активируется только
            // когда стопа + голень + таз спокойны >= 2 сек.  Носки (RToe/LToe)
            // слейв стоп, поэтому их направление автоматически фиксится.
            if (i == SEG_RFoot || i == SEG_LFoot) {
                const int iLowerLeg = (i == SEG_RFoot) ? SEG_RLowerLeg : SEG_LLowerLeg;
                const double llAngV  = m_angVelLP[iLowerLeg];
                const bool calmFoot   = m_angVelLP[i] < 1.5;
                const bool calmLL     = llAngV       < 1.5;
                const bool calmPelvis = pelvisYawRate < 0.20;
                const bool allCalm = calmFoot && calmLL && calmPelvis;

                if (allCalm) m_calmSeconds[i] += dt;
                else         m_calmSeconds[i] = 0.0;

                const Quat dA_f = m_skel ? m_skel->defAngFor(i) : Quat(1, 0, 0, 0);
                const Quat dA_ll = m_skel ? m_skel->defAngFor(iLowerLeg) : Quat(1, 0, 0, 0);
                const Quat dA_pel = m_skel ? m_skel->defAngFor(SEG_Pelvis) : Quat(1, 0, 0, 0);
                const Quat fWorld  = quat_mult(filtered[i],         dA_f).normalized();
                const Quat llWorld = quat_mult(filtered[iLowerLeg], dA_ll).normalized();
                const Quat pelvisWorld = quat_mult(filtered[SEG_Pelvis], dA_pel).normalized();

                // FIX (T-pose foot direction reference + cross-legged):
                // если есть T-pose anchor → используем pelvis-yaw как
                // stable reference (lowerLeg уезжает при flex'е колена,
                // искажая yaw drift detection).  Cross-legged confidence
                // (0..1) глушит коррекцию: при перекрёщенных ногах
                // lowerLeg/pelvis-yaw геометрия "запутана" и коррекция
                // может развернуть стопу неправильно.
                const bool useTposeRef = m_tposeFootAnchorValid;
                const Quat refWorld = useTposeRef
                                    ? yaw_only_quat(pelvisWorld)
                                    : llWorld;
                const double crossConf = (i == SEG_RFoot) ? m_crossLeggedConfR
                                                          : m_crossLeggedConfL;
                const bool   crossLeg  = (i == SEG_RFoot) ? m_crossLeggedR
                                                          : m_crossLeggedL;
                const double gateAttn = std::max(0.0, 1.0 - crossConf);
                const Quat tposeAnchorLocal = (i == SEG_RFoot)
                                            ? m_tposeFootRefPelR
                                            : m_tposeFootRefPelL;

                if (m_locked[i]) {
                    // Анкор: если T-pose pinned — НЕ перезаписываем (как
                    // в hand block для T-pose hand anchor).  Иначе старый
                    // путь: lowerLeg-relative.
                    if (!useTposeRef) {
                        m_anchorLocal[i] = quat_mult(llWorld.inv(), fWorld).normalized();
                        m_anchorValid[i] = true;
                    }
                    m_driftLocal[i]  = nlerpQ(m_driftLocal[i], Quat(1, 0, 0, 0),
                                              std::min(1.0, dt / 5.0));
                } else if ((useTposeRef || m_anchorValid[i]) && m_calmSeconds[i] >= 2.0) {
                    const Quat anchorLocal = useTposeRef ? tposeAnchorLocal
                                                         : m_anchorLocal[i];
                    const Quat current_local = quat_mult(refWorld.inv(), fWorld).normalized();
                    Quat drift = quat_mult(current_local, anchorLocal.inv()).normalized();
                    if (drift.w < 0) {
                        drift.w = -drift.w; drift.x = -drift.x;
                        drift.y = -drift.y; drift.z = -drift.z;
                    }
                    const double sw = std::min(1.0, std::abs(drift.w));
                    const double driftAngle = 2.0 * std::acos(sw);
                    // Бюджет 15° — стопа в покое почти не отклоняется,
                    // ужесточаем чтобы убрать настоящий yaw drift.
                    const double driftBudget = M_PI / 12.0;
                    // FIX (terminator smoothing): smoothstep attenuation
                    // [0.7*budget .. budget].  Старый код hard-cut при
                    // driftAngle >= budget — коррекция мгновенно отключалась.
                    // Теперь плавно затухает в полосе 10.5°..15°.
                    // FIX (cross-legged): дополнительно глушим при crossLeg.
                    if (allCalm && !crossLeg) {
                        auto smoothstep01 = [](double x){ x = std::clamp(x,0.0,1.0); return x*x*(3.0-2.0*x); };
                        const double budgetAttn = smoothstep01(
                                (driftBudget - driftAngle) / (driftBudget * 0.3));
                        if (budgetAttn > 0.0) {
                            const double alpha = std::min(1.0, dt / 3.0) * budgetAttn * gateAttn;
                            m_driftLocal[i] = nlerpQ(m_driftLocal[i], drift, alpha);
                        }
                    }
                    // Корректируем только TWIST вокруг мирового +Z (yaw),
                    // swing (наклон стопы) НЕ ТРОГАЕМ.
                    const Quat& dL = m_driftLocal[i];
                    const double dw = std::min(1.0, std::abs(dL.w));
                    if (2.0 * std::acos(dw) > 1e-4) {
                        Quat dLSwing, dLTwist;
                        swingTwistDecompose(dL, QVector3D(0.0f, 0.0f, 1.0f), dLSwing, dLTwist);
                        Quat dLTwistInv = dLTwist.inv();
                        // FIX (cross-legged): дополнительно слегка damp
                        // twist correction при crossConf > 0.
                        if (crossConf > 0.0) {
                            dLTwistInv = slerp_quat(dLTwistInv, Quat(1, 0, 0, 0), crossConf);
                        }
                        const Quat correctionWorld = quat_mult(refWorld,
                                                       quat_mult(dLTwistInv, refWorld.inv())).normalized();
                        const Quat fCorrected = quat_mult(correctionWorld, fWorld).normalized();
                        filtered[i] = quat_mult(fCorrected, dA_f.inv()).normalized();
                    }
                }
            }

            m_outPrevQ[i] = filtered[i];
        }
    } else {
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            m_angVelLP[i] = 0.0;
            m_stillTicks[i] = 0.0;
            m_locked[i] = false;
            m_unlockBlend[i] = 1.0;
            m_lockBlend[i] = 1.0;
            m_outPrevQ[i] = orient[i];
            m_anchorValid[i] = false;
            m_anchorLocal[i] = Quat(1, 0, 0, 0);
            m_driftLocal[i] = Quat(1, 0, 0, 0);
        }
    }

    m_prevQ = orient;   // store RAW for next-frame angular-speed
    m_havePrevQ = true;

    m_orient = filtered;
    m_root   = root;
    // Throttle the actual redraw to the display rate.  The solve / record /
    // stream work already ran this tick in MainWindow::onRenderTick at the full
    // suit rate; drawing faster than the monitor refreshes wastes the GPU.
    if (now - m_lastPaintSec >= m_paintMinIntervalSec) {
        m_lastPaintSec = now;
        update();
    }
}

void MocapViewport::resetSceneOrigin()
{
    // Pin the character at scene (0, 0) and straighten his facing so
    // exported BVH/FBX sits neatly in Blender / UE. Z is kept natural
    // (floor-contact logic inside drawSkeleton handles feet-on-floor —
    // user explicitly asked "не трогать Z чтобы не провалился под пол").
    if (!m_skel) return;

    // Step 1: cancel current yaw. Extract pelvis yaw (rotation about world
    // Z) from the live orientation and set m_sceneYaw = -yaw so rotation
    // becomes identity. We measure yaw by projecting the pelvis forward
    // axis onto world XY.
    const Quat& qp = m_orient[SEG_Pelvis];
    // Rotate local +X by qp and measure atan2(y, x) — that's yaw in NWU.
    const double fx = 1.0 - 2.0*(qp.y*qp.y + qp.z*qp.z);
    const double fy = 2.0*(qp.x*qp.y + qp.w*qp.z);
    const double yaw = std::atan2(fy, fx);
    m_sceneYaw = float(-yaw);

    // Step 2: pin XY to 0. Use the last rendered pelvis (which already
    // carries the loco walk offset from the previous frame). We subtract
    // the OLD sceneShift because it will be re-applied with the new shift
    // below, and rotate the pre-shift pelvis by the new yaw.
    QVector3D raw = m_lastRenderedPelvis - m_sceneShift;   // pre-shift world
    const float cy = std::cos(m_sceneYaw);
    const float sy = std::sin(m_sceneYaw);
    QVector3D rotated(cy*raw.x() - sy*raw.y(),
                      sy*raw.x() + cy*raw.y(),
                      raw.z());
    m_sceneShift = QVector3D(-rotated.x(), -rotated.y(), 0.0f);
    m_freezeAnchor = QVector3D(0, 0, 0);
    update();
}

void MocapViewport::setFreezeXY(bool frozen)
{
    if (m_freezeXY == frozen) return;
    m_freezeXY = frozen;
    if (frozen) {
        // Capture pelvis XY at freeze moment (in post-shift scene coords).
        m_freezeAnchor = QVector3D(m_lastRenderedPelvis.x(),
                                   m_lastRenderedPelvis.y(), 0);
    }
    update();
}

void MocapViewport::updateHands(bool haveGloves,
                                const std::array<QVector3D, kFingerSegmentsHand>& right,
                                const std::array<QVector3D, kFingerSegmentsHand>& left)
{
    m_haveGloves = haveGloves;
    m_rightHand  = right;
    m_leftHand   = left;
}

void MocapViewport::initializeGL()
{
    auto* f = QOpenGLContext::currentContext()->functions();
    f->glClearColor(0.05f, 0.05f, 0.06f, 1.0f);
    f->glEnable(GL_DEPTH_TEST);
    f->glEnable(GL_LINE_SMOOTH);
    f->glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
}

void MocapViewport::resizeGL(int w, int h)
{
    auto* f = QOpenGLContext::currentContext()->functions();
    f->glViewport(0, 0, w, h);
}

// --- legacy GL is fine for a line renderer; Qt's compatibility profile works
//     on Windows where MVN is typically run.

void MocapViewport::paintGL()
{
    auto* gl = QOpenGLContext::currentContext()->functions();
    gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Fixed-function setup (legacy profile).  Viewport coordinate system
    // matches hipose's default (use_isb_ref=False): Z-up, identity axis_rot.
    // std::max guards a zero-height surface (degenerate aspect / div-by-zero).
    QMatrix4x4 proj;  proj.perspective(45.0f, float(width())/float(std::max(1, height())), 0.05f, 100.0f);

    const float yawR   = qDegreesToRadians(m_yaw);
    const float pitchR = qDegreesToRadians(m_pitch);
    QVector3D eye(m_dist * std::cos(pitchR) * std::sin(yawR),
                  m_dist * std::cos(pitchR) * std::cos(yawR),
                  m_dist * std::sin(pitchR) + 1.0f);                   // raise cam
    QMatrix4x4 view;  view.lookAt(eye, QVector3D(0, 0, 1.0f), QVector3D(0, 0, 1));

    glMatrixMode(GL_PROJECTION); glLoadMatrixf(proj.constData());
    glMatrixMode(GL_MODELVIEW);  glLoadMatrixf(view.constData());

    drawFloor();
    drawReferenceFrame();
    drawSkeleton();
}

void MocapViewport::drawFloor()
{
    glLineWidth(1.0f);
    glColor4f(0.22f, 0.22f, 0.24f, 1.0f);
    glBegin(GL_LINES);
    const int   N = 10;
    const float S = 1.0f;
    for (int i = -N; i <= N; ++i) {
        glVertex3f(-N*S, i*S, 0); glVertex3f(N*S, i*S, 0);
        glVertex3f(i*S, -N*S, 0); glVertex3f(i*S, N*S, 0);
    }
    glEnd();
}

void MocapViewport::drawReferenceFrame()
{
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1, 0.25f, 0.25f); glVertex3f(0,0,0); glVertex3f(0.3f,0,0);    // X
    glColor3f(0.25f, 1, 0.25f); glVertex3f(0,0,0); glVertex3f(0,0.3f,0);    // Y
    glColor3f(0.25f, 0.4f, 1); glVertex3f(0,0,0); glVertex3f(0,0,0.3f);     // Z (up)
    glEnd();
}

QVector3D MocapViewport::tickLoco(
        const std::array<Quat, kXsensSegmentCount>& q,
        const QVector3D& fkRHeel,
        const QVector3D& fkRBall,
        const QVector3D& fkRTip,
        const QVector3D& fkLHeel,
        const QVector3D& fkLBall,
        const QVector3D& fkLTip,
        double tSec)
{
    m_lastLocoOffset = m_loco.update(
            q[SEG_RFoot], q[SEG_LFoot], q[SEG_Pelvis],
            fkRHeel, fkRBall, fkRTip,
            fkLHeel, fkLBall, fkLTip,
            tSec);
    return m_lastLocoOffset;
}

void MocapViewport::drawSkeleton()
{
    if (!m_skel) return;

    // FK — skeleton gives 28 keypoints in world space.
    auto kp = m_skel->computeKeypoints(m_orient, m_root);

    // ----- Locomotion solver (walking + drift protection) --------------------
    // Ports MVN's contract: `setContacts` → biomech solver locks foot, pelvis
    // is derived from the locked anchor.  See LocomotionSolver comments in
    // main.h for the ghidra-derived rationale.
    {
        for (auto& p : kp) p += m_lastLocoOffset;
    }
    // Safety: if anything pathological drops a keypoint below the floor,
    // tug the whole skeleton up.  Should be rare now that locomotion pins Z.
    float minZ = kp[0].z();
    for (const auto& p : kp) if (p.z() < minZ) minZ = p.z();
    m_lastFloorClamp = (minZ < -0.02f) ? -minZ : 0.0f;   // -test [VIEW]
    if (minZ < -0.02f) {
        const QVector3D shift(0, 0, -minZ);
        for (auto& p : kp) p += shift;
    }

    // ----- Scene reset / freeze (pure view-side, post-loco) -------------
    // Reset: m_sceneShift applies a constant XY translation so user-selected
    // origin becomes scene (0,0). m_sceneYaw rotates the world about the Z
    // axis so a "reset" button also straightens the character's facing so
    // exported BVH/FBX doesn't drift on a Blender map. Freeze: per-frame
    // correction keeps pelvis at m_freezeAnchor XY regardless of loco.
    const float cy = std::cos(m_sceneYaw);
    const float sy = std::sin(m_sceneYaw);
    auto rotZ = [&](const QVector3D& v) {
        return QVector3D(cy*v.x() - sy*v.y(),
                         sy*v.x() + cy*v.y(),
                         v.z());
    };
    if (m_sceneYaw != 0.0f) {
        for (auto& p : kp) p = rotZ(p);
    }
    if (m_freezeXY) {
        QVector3D pelvisScene = kp[SEG_Pelvis] + m_sceneShift;
        QVector3D freezeFix(m_freezeAnchor.x() - pelvisScene.x(),
                            m_freezeAnchor.y() - pelvisScene.y(), 0);
        for (auto& p : kp) p += m_sceneShift + freezeFix;
    } else {
        for (auto& p : kp) p += m_sceneShift;
    }
    // Cache the final on-screen pelvis so the Reset button can zero out XY
    // on the next click without guessing the loco offset.
    m_lastRenderedPelvis = kp[SEG_Pelvis];

    // FIX issue 6: prayer-pose сходимость кистей.  При drift'е wrist S2S
    // кисти могут не сходиться точно когда пользователь складывает ладони.
    // Если запястья ближе 15 cm И ладони смотрят друг на друга (палм-нормали
    // противоположны), мягко притягиваем оба wrist к XY-серединой.
    // Z не трогаем (не топим руки вниз).  Сила пропорциональна близости.
    // Триггерится только в реальном prayer'е, не в обычных жестах.
    {
        const QVector3D pR = kp[SEG_RHand];
        const QVector3D pL = kp[SEG_LHand];
        const float d = (pR - pL).length();
        constexpr float kPrayerRange = 0.15f;
        if (d < kPrayerRange && d > 1e-3f) {
            const Quat qRW = quat_mult(m_orient[SEG_RHand], m_skel->defAngFor(SEG_RHand));
            const Quat qLW = quat_mult(m_orient[SEG_LHand], m_skel->defAngFor(SEG_LHand));
            const QVector3D nR = vec_rotate(QVector3D(0, 1, 0), qRW);
            const QVector3D nL = vec_rotate(QVector3D(0, 1, 0), qLW);
            if (QVector3D::dotProduct(nR, nL) < -0.5f) {
                const QVector3D mid = 0.5f * (pR + pL);
                const float w = 0.15f * (1.0f - d / kPrayerRange);
                const QVector3D shiftR(w * (mid.x() - pR.x()),
                                       w * (mid.y() - pR.y()), 0.0f);
                const QVector3D shiftL(w * (mid.x() - pL.x()),
                                       w * (mid.y() - pL.y()), 0.0f);
                // Применяем тот же сдвиг к wrist и кончику пальца (24/25),
                // чтобы вся кисть двигалась как одно целое.  Пальцы рисуются
                // от wrist+rel, поэтому они автоматически следуют.
                kp[SEG_RHand] += shiftR;
                kp[SEG_LHand] += shiftL;
                if (kXsensKeypointCount >= 26) {
                    kp[24] += shiftR;
                    kp[25] += shiftL;
                }
            }
        }
    }

    // -test [VIEW]: cache the keypoints the operator literally sees this frame
    // (post loco + floor-clamp + yaw + shift + freeze + prayer nudge) so the
    // log can compare on-screen pose vs raw FK without recomputing framing.
    m_lastRenderedKp = kp;

    // Bones (GL_LINES).  Orange.
    glLineWidth(3.0f);
    glColor3f(1.0f, 0.48f, 0.10f);                       // accent orange
    glBegin(GL_LINES);
    const auto& S = m_skel->startPts();
    const auto& E = m_skel->endPts();
    for (int s = 0; s < kXsensSegmentCountWithDummies; ++s) {
        // Skip FK hand bone (wrist→fingertip avg) when real glove fingers are
        // drawn from wrist — two overlapping geometries confuse the eye,
        // making fingers look like they start mid-arm.
        if (m_haveGloves && (E[s] == 24 || E[s] == 25)) continue;
        const auto& a = kp[S[s]];
        const auto& b = kp[E[s]];
        glVertex3f(a.x(), a.y(), a.z());
        glVertex3f(b.x(), b.y(), b.z());
    }
    glEnd();

    // Joints as thick points (white).
    glPointSize(6.0f);
    glColor3f(0.95f, 0.95f, 0.95f);
    glBegin(GL_POINTS);
    for (int i = 0; i < kXsensKeypointCount; ++i)
        glVertex3f(kp[i].x(), kp[i].y(), kp[i].z());
    glEnd();

    // Root (pelvis) highlighted bright orange.
    glPointSize(10.0f);
    glColor3f(1.0f, 0.62f, 0.18f);
    glBegin(GL_POINTS);
    glVertex3f(kp[0].x(), kp[0].y(), kp[0].z());
    glEnd();

    // ----- Per-segment XYZ triads --------------------------------------------
    // Direct port of visualization.py's display_segment_axis path: for each of
    // the 27 post-dummy segments, draw a 7.5 cm R/G/B axis triad at the bone's
    // start joint, rotated by the segment's global orientation.
    {
        // Re-run the FK orientation chain just as computeKeypoints does so
        // axes and bones stay in sync.
        std::array<Quat, kXsensSegmentCount> oriented{};
        for (int i = 0; i < kXsensSegmentCount; ++i)
            oriented[i] = quat_mult(m_orient[i], m_skel->defaultSegAngles()[i]);
        const auto global = m_skel->addDummySegments(oriented);

        constexpr float L = 0.075f;
        glLineWidth(1.6f);
        glBegin(GL_LINES);
        for (int s = 0; s < kXsensSegmentCountWithDummies; ++s) {
            // Axis triad at segment END keypoint (the joint/tip the bone
            // actually ends at). Placing at start put head triad on the neck.
            const QVector3D origin = kp[E[s]];
            const QVector3D ax = vec_rotate(QVector3D(L, 0, 0), global[s]);
            const QVector3D ay = vec_rotate(QVector3D(0, L, 0), global[s]);
            const QVector3D az = vec_rotate(QVector3D(0, 0, L), global[s]);
            glColor4f(1, 0.25f, 0.25f, 0.85f);
            glVertex3f(origin.x(), origin.y(), origin.z());
            glVertex3f(origin.x() + ax.x(), origin.y() + ax.y(), origin.z() + ax.z());
            glColor4f(0.25f, 1, 0.25f, 0.85f);
            glVertex3f(origin.x(), origin.y(), origin.z());
            glVertex3f(origin.x() + ay.x(), origin.y() + ay.y(), origin.z() + ay.z());
            glColor4f(0.35f, 0.55f, 1, 0.85f);
            glVertex3f(origin.x(), origin.y(), origin.z());
            glVertex3f(origin.x() + az.x(), origin.y() + az.y(), origin.z() + az.z());
        }
        glEnd();
    }

    // -------- Fingers -----------------------------------------------------
    // m_rightHand / m_leftHand hold RELATIVE offsets (raw_finger - raw_wrist)
    // supplied by MainWindow.  We anchor each chain to the FK wrist so the
    // hand visually continues the forearm (not snapped off sideways) while
    // the finger geometry stays exactly as MVN computed it.
    if (!m_haveGloves) return;

    const QVector3D rWrist = kp[SEG_RHand];
    const QVector3D lWrist = kp[SEG_LHand];

    auto drawHand = [&](const std::array<QVector3D, kFingerSegmentsHand>& rel,
                        const QVector3D& wrist,
                        float cr, float cg, float cb,         // bone color
                        float pr, float pg, float pb) {       // joint color
        // Skip if we have no finger data yet (all zero).  This keeps the
        // visualisation clean when a session is "suit + gloves" but the
        // Manus stream hasn't started.
        bool any = false;
        for (const auto& v : rel) if (!v.isNull()) { any = true; break; }
        if (!any) return;

        glLineWidth(2.2f);
        glColor3f(cr, cg, cb);
        glBegin(GL_LINES);
        for (int c = 0; c < kFingerChainCount; ++c) {
            QVector3D prev = wrist;
            for (int k = 0; k < kFingerChainLen; ++k) {
                const QVector3D cur = wrist + rel[kFingerChains[c][k]];
                glVertex3f(prev.x(), prev.y(), prev.z());
                glVertex3f(cur.x(),  cur.y(),  cur.z());
                prev = cur;
            }
        }
        glEnd();

        glPointSize(3.5f);
        glColor3f(pr, pg, pb);
        glBegin(GL_POINTS);
        for (int c = 0; c < kFingerChainCount; ++c)
            for (int k = 0; k < kFingerChainLen; ++k) {
                const QVector3D p = wrist + rel[kFingerChains[c][k]];
                glVertex3f(p.x(), p.y(), p.z());
            }
        glEnd();
    };

    // Right hand — warm orange;  Left hand — cyan.
    drawHand(m_rightHand, rWrist, 1.00f, 0.55f, 0.12f,  1.00f, 0.85f, 0.55f);
    drawHand(m_leftHand,  lWrist, 0.10f, 0.70f, 1.00f,  0.55f, 0.85f, 1.00f);
}

void MocapViewport::mousePressEvent(QMouseEvent* e) { m_lastMouse = e->pos(); }
void MocapViewport::mouseMoveEvent (QMouseEvent* e)
{
    const QPoint d = e->pos() - m_lastMouse;
    m_lastMouse = e->pos();
    if (e->buttons() & Qt::LeftButton) {
        m_yaw   += d.x() * 0.4f;
        m_pitch  = std::max(-85.0f, std::min(85.0f, m_pitch + d.y() * 0.3f));
        update();
    }
}
void MocapViewport::wheelEvent(QWheelEvent* e)
{
    m_dist = std::max(0.5f, std::min(10.0f, m_dist - e->angleDelta().y() * 0.002f));
    update();
}

// ============================================================================
//  Live streaming — MVN MXTP over UDP (MXTP02 pose).
//
//  Wire format matches what MVNBlenderPlugin and XsensLivc both read:
//    header 24 B :  "MXTP02" + int32 sampleCounter + byte dgCounter
//                    + byte count + int32 frameTimeMs + byte avatarId
//                    + byte bodySegments + byte props + byte fingers
//                    + 4 B padding
//    payload 32 B/segment : int32 segmentId (1..23) + 3×float xyz (m)
//                           + 4×float quaternion WXYZ, all big-endian.
//
//  Startup: one MXTP12 metadata packet ("name:FoxMocap\n"), one MXTP13
//  scaling packet (23 segments with zeroed origins for now), then stream
//  MXTP02 at render tick frequency.  Plugins accept dgCounter=0x80 to
//  mark "single datagram per frame".
// ============================================================================

// buildMxtpHeader / appendFloatBE / appendInt32BE now live in foxwire.{h,cpp}
// (unit-tested against the immutable Plugins MXTP byte contract).  appendFloatBE
// there also coerces any non-finite value to 0.0f so a degenerate frame can
// never push NaN/Inf onto the wire.

// ============================================================================
// COORDINATE CONTRACT — the complete frame chain, end to end (verified audit).
//
// All quaternions are WXYZ (scalar-first); quat_mult = Hamilton product;
// vec_rotate(v,q) = q·[0,v]·q⁻¹. World frame is NWU: X=forward, Y=left, Z=up,
// right-handed. Left-side mirroring uses mirror_y_quat(q) = j·q·j⁻¹ = (w,-x,y,-z),
// a homomorphism (reflection of the rotation across the body XZ-plane).
//
//   1. sensor → body : receiver rotates raw acc/gyr/mag by inv(s2s[i]) (= s2b);
//                      s2s (body→sensor) is solved per segment by TRIAD /
//                      Davenport-Wahba / ecompass from the T/N/K calibration.
//   2. fusion        : xio FusionAhrs (convention = NWU) → per-segment world
//                      quaternion raw[i].
//   3. calibration   : cand[i] = quat_mult(raw[i], calibReference[i].inv());
//                      identity at the N-pose, world delta otherwise.
//   4. FK            : oriented[i] = quat_mult(cand[i], defAng[i]) (defAng =
//                      N-pose rest) + shoulder cone; addDummySegments inserts 4
//                      stubs that co-rotate with T8/pelvis yaw and branch L/R by
//                      ±π/2; boneVec = vec_rotate([len,0,0], global[i]); chain-walk.
//   5. movement      : LocomotionSolver returns a world offset (≈ anchor − fk) in
//                      metres so a planted foot stays put; floor-clamp lifts the
//                      lowest keypoint to z=0. worldPelvisWithLoco() is the single
//                      source shared by the live-stream and recording paths.
//   6. wire (MXTP02) : the world pose is emitted unchanged in NWU = MVN-default
//                      Z-up RH; each consumer plugin does its own conversion (see
//                      "Streaming coordinate frame" just below).
// ============================================================================

// ============================================================================
// Streaming coordinate frame — single MVN-default Z-up stream for all targets.
//
// Our pipeline works natively in NWU (X=forward, Y=left, Z=up, right-handed).
// The MVN real-time protocol's quaternion pose stream (MXTP02) uses, by
// default, a **Z-Up, right-handed** coordinate system — the same axes as our
// NWU.  Both bundled plugins consume exactly that default Z-up wire:
//
//   * UE LiveLink (Plugins/XsensLivc/.../LiveLinkMvnSource.h:49-50) converts
//     wire→UE with FVector(x,-y,z) and FQuat(-qx,qy,-qz,qw) — a pure Y-axis
//     handedness flip from Z-up RH into UE's Z-up LH.  ScaleDatagram.cpp:26,59
//     spells it out: "Z-Up, right-handed coordinate system".
//   * The Blender add-on builds its rest skeleton from the scale (MXTP13)
//     message mapping wire.z → Blender.z (up), and remaps each bone group from
//     that Z-up global frame in source_animator.calculate_rotation.
//
// MVN Animate emits ONE identical stream to every listener, so we do the same:
// a single MVN-default Z-up stream, no per-target axis conversion — the wire
// frame already equals our NWU.  (Earlier code pre-rotated the Blender stream
// to a "Y-up" frame; that double-transformed both the live pose and the T-pose
// skeleton origins and was the root cause of the wrong Blender starting pose.)
// ============================================================================

// FIX (stream polish): hex-dump первого фрейма для verification против
// MVN spec.  Печатается в stdout, читается совместно с логом и tcpdump.
static void dumpFirstFrameHex(const char* tag, const QByteArray& pkt)
{
    std::cout << "[stream first-frame hex] " << tag << " bytes="
              << pkt.size() << "\n";
    const int n = std::min(pkt.size(), qsizetype(24 + 64));  // header + 2 segs max
    std::cout << "  hex:";
    for (int i = 0; i < n; ++i) {
        if (i % 16 == 0) std::cout << "\n    ";
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%02x ",
                      static_cast<unsigned>(static_cast<unsigned char>(pkt[i])));
        std::cout << buf;
    }
    std::cout << "\n";
}

struct LiveStreamSender::Impl {
    QUdpSocket   sock;
    QHostAddress host;
    quint16      port  = 9763;
    QElapsedTimer timer;
    QByteArray   metaPkt;
    QByteArray   scalePkt;
    bool         firstFrameDumped = false;
    qint64       lastHandshakeMs = -1;
    qint64       lastWireDumpMs  = -1;   // [STREAM SNAPSHOT] cadence (verboseLog)
    static constexpr qint64 kWireDumpIntervalMs = 2000;
    qint64       lastEmitMs = -1;
    static constexpr qint64 kHandshakeIntervalMs = 1000;  // re-send MXTP12+13 ~1/s

    std::array<Quat, kXsensSegmentCount> baselineBodyQ{};
    std::array<Quat, kFingerSegmentsHand> baselineLeftGloveQ{};
    std::array<Quat, kFingerSegmentsHand> baselineRightGloveQ{};
    QVector3D    baselinePelvisPos{};
    bool         baselineCaptured = false;
    int          fingerBaselineSamples = 0;
    static constexpr int kFingerBaselineWindow = 30;

    // Hemisphere-continuity state for the wire delta-quaternion stream — the
    // last quaternion actually sent on each wire slot (23 body + 20 per hand).
    // The next frame is forced into the same hemisphere so the stream never
    // sign-flips between frames (see hemisphereContinuous()).
    std::array<Quat, kXsensSegmentCount>  prevWireBodyQ{};
    std::array<Quat, kFingerSegmentsHand> prevWireLeftQ{};
    std::array<Quat, kFingerSegmentsHand> prevWireRightQ{};
    void resetWireContinuity() {
        prevWireBodyQ.fill(Quat(1, 0, 0, 0));
        prevWireLeftQ.fill(Quat(1, 0, 0, 0));
        prevWireRightQ.fill(Quat(1, 0, 0, 0));
    }

    // Throttled checked UDP send.  A failed writeDatagram (no route, send
    // buffer full, interface down) was previously ignored silently; warn at
    // most once per second so a mid-session dropout is visible without flooding.
    qint64 lastSendWarnMs = -1;
    void sendChecked(const QByteArray& pkt) {
        if (sock.writeDatagram(pkt, host, port) >= 0) return;
        const qint64 now = timer.elapsed();
        if (lastSendWarnMs < 0 || (now - lastSendWarnMs) >= 1000) {
            lastSendWarnMs = now;
            std::cerr << "[stream] WARNING: UDP send failed ("
                      << sock.errorString().toStdString() << ") to "
                      << host.toString().toStdString() << ':' << port
                      << " - frame dropped\n";
        }
    }

    // Re-send the MXTP12 (metadata) + MXTP13 (scale/T-pose) handshake on a
    // wall-clock interval so a Blender/UE instance opened long after the
    // stream started still acquires the actor name and rebuilds the T-pose
    // skeleton.  Time-based (not frame-based) so the delay is ~1 s regardless
    // of stream fps.
    void maybeRetransmitHandshake() {
        const qint64 now = timer.elapsed();
        if (lastHandshakeMs >= 0 && (now - lastHandshakeMs) < kHandshakeIntervalMs)
            return;
        lastHandshakeMs = now;
        if (!metaPkt.isEmpty())
            sendChecked(metaPkt);
        if (!scalePkt.isEmpty())
            sendChecked(scalePkt);
    }
};

LiveStreamSender::LiveStreamSender(QObject* parent)
    : QObject(parent), m_impl(std::make_unique<Impl>()) {}

LiveStreamSender::~LiveStreamSender() { stop(); }

bool LiveStreamSender::start(const LiveSettings& cfg, QString* err)
{
    m_cfg = cfg;
    m_impl->host.setAddress(cfg.host);
    if (m_impl->host.isNull()) {
        if (err) *err = "Invalid host address";
        return false;
    }
    m_impl->port = quint16(cfg.port);
    if (!m_impl->sock.bind(QHostAddress::AnyIPv4, 0)) {
        if (err) *err = m_impl->sock.errorString();
        return false;
    }
    m_impl->timer.start();
    m_impl->lastEmitMs = -1;
    m_impl->resetWireContinuity();

    {
        QByteArray text = QByteArray(
            "name:FoxMocapLive\nname:FoxMocapLive\ntimeOffset:0\ncolor:255 128 64\n");
        QByteArray payload;
        appendInt32BE(payload, qint32(text.size()));
        payload.append(text);
        QByteArray hdr = buildMxtpHeader("12", 0, 0x80, 1, 0, 23, 0);
        m_impl->metaPkt = hdr + payload;
        m_impl->sock.writeDatagram(m_impl->metaPkt, m_impl->host, m_impl->port);
        QThread::msleep(50);
    }
    {
        static const char* const kBodyMvn[kXsensSegmentCount] = {
            "Pelvis", "L5", "L3", "T12", "T8", "Neck", "Head",
            "RightShoulder", "RightUpperArm", "RightForeArm", "RightHand",
            "LeftShoulder",  "LeftUpperArm",  "LeftForeArm",  "LeftHand",
            "RightUpperLeg", "RightLowerLeg", "RightFoot", "RightToe",
            "LeftUpperLeg",  "LeftLowerLeg",  "LeftFoot",  "LeftToe",
        };
        static const char* const kFingerMvn[40] = {
            "LeftCarpus",
            "LeftFirstMC",  "LeftFirstPP",  "LeftFirstDP",
            "LeftSecondMC", "LeftSecondPP", "LeftSecondMP", "LeftSecondDP",
            "LeftThirdMC",  "LeftThirdPP",  "LeftThirdMP",  "LeftThirdDP",
            "LeftFourthMC", "LeftFourthPP", "LeftFourthMP", "LeftFourthDP",
            "LeftFifthMC",  "LeftFifthPP",  "LeftFifthMP",  "LeftFifthDP",
            "RightCarpus",
            "RightFirstMC",  "RightFirstPP",  "RightFirstDP",
            "RightSecondMC", "RightSecondPP", "RightSecondMP", "RightSecondDP",
            "RightThirdMC",  "RightThirdPP",  "RightThirdMP",  "RightThirdDP",
            "RightFourthMC", "RightFourthPP", "RightFourthMP", "RightFourthDP",
            "RightFifthMC",  "RightFifthPP",  "RightFifthMP",  "RightFifthDP",
        };
        const qint32 segCount   = cfg.useGloves ? 63 : 23;
        const quint8 fingerHdr  = cfg.useGloves ? 40 : 0;     // фикс: было 0
        // FIX (stream polish): защита от пустого tposeOriginM.  Если все 23
        // элемента нулевые — MXTP13 уходит со scale=0 и плагины не могут
        // отнормировать pelvis (в LiveLinkMvnSource scale становится 0
        // и pelvis улетает на ~47x или клампится).  Проверяем и логируем.
        bool tposeOriginsValid = false;
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            if (cfg.tposeOriginM[i].lengthSquared() > 1e-6f) {
                tposeOriginsValid = true;
                break;
            }
        }
        if (!tposeOriginsValid) {
            std::cout << "[stream] WARNING: MXTP13 tposeOriginM is empty —"
                         " plugins may render rig at wrong scale.  "
                         "Caller must populate cfg.tposeOriginM[] from FK "
                         "before LiveStreamSender::start().\n";
        }
        QByteArray payload;
        appendInt32BE(payload, segCount);
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            // T-pose origin position in METERS, MVN wire frame = Z-Up,
            // right-handed = our NWU (X-fwd, Y-left, Z-up).  Plugins use these
            // for rig scaling (UE ScaleDatagram / Blender create_target_skeleton).
            const QVector3D o = cfg.tposeOriginM[i];
            appendScaleSegment(payload, kBodyMvn[i], o.x(), o.y(), o.z());
        }
        if (cfg.useGloves) {
            for (int i = 0; i < 40; ++i)
                appendScaleSegment(payload, kFingerMvn[i], 0.0f, 0.0f, 0.0f);
        }
        // MXTP13 datagram counter MUST be 0 (not 0x80).  Real MVN sends the
        // scale message as a multi-packet sequence whose first (segments)
        // packet carries counter 0; the Blender add-on rejects the scale
        // packet unless datagram_counter == 0 (receiver.py:398), so 0x80 made
        // it silently drop the T-pose and never build the skeleton.  UE has no
        // counter gate on the scale datagram, so 0 works for both targets.
        QByteArray hdr = buildMxtpHeader("13", 0, 0x00, quint8(segCount),
                                         0, 23, fingerHdr);
        m_impl->scalePkt = hdr + payload;
        m_impl->sock.writeDatagram(m_impl->scalePkt, m_impl->host, m_impl->port);
        QThread::msleep(50);
    }

    m_impl->lastHandshakeMs = m_impl->timer.elapsed();  // initial handshake just sent
    m_running = true;
    return true;
}

void LiveStreamSender::stop()
{
    if (!m_running) return;
    m_impl->sock.close();
    m_running = false;
}

void LiveStreamSender::recalibrate()
{
    m_impl->baselineCaptured = false;
    m_impl->fingerBaselineSamples = 0;
    m_impl->resetWireContinuity();
}

void LiveStreamSender::setTposeBaseline(
    const std::array<Quat, kXsensSegmentCount>& tposeQ,
    const QVector3D& tposePelvis)
{
    m_impl->baselineBodyQ = tposeQ;
    m_impl->baselinePelvisPos = tposePelvis;
    m_impl->baselineCaptured = true;
    m_impl->resetWireContinuity();
}

// ---------------------------------------------------------------------------
//  Wire frame transforms — the SINGLE tunable point for live Blender/UE diff.
// ---------------------------------------------------------------------------
// Orientation: the wire carries each segment's *absolute calibrated world
// orientation* — exactly the `qOut` the viewport renders (identity at the
// calibration pose).  Both bundled plugins do their OWN neutral referencing
// (Blender: local = parent_global^-1 * child_global + per-bone remap in
// source_animator.calculate_rotation; UE: FQuat(-qx,qy,-qz,qw) flip), which is
// why real MVN streams the absolute global pose and we must too.  The previous
// `qOut * baselineBodyQ^-1` pre-subtracted a *reconstructed* T-pose baseline
// (tposeReference*calibReference^-1) that does NOT match the runtime
// `raw*calibReference^-1` pipeline, injecting a ~165° per-segment rotation that
// laid the skeleton flat (logs/fox_mocap.log: pelvis |wire|=165° while internal
// qOut≈23°).  If a single residual global roll appears live, a uniform
// conjugation q -> G*q*G^-1 belongs HERE and nowhere else.
static inline Quat mvnWireOrient(const Quat& worldSeg, LiveTarget /*target*/)
{
    return worldSeg;
}

// Pelvis world position (metres, our NWU = X-forward, Y-left, Z-up, RH) -> wire.
// UE consumes Z-up RH directly (LiveLinkMvnSource.h: FVector(x,-y,z)*100), so we
// emit native NWU for it.  The Blender add-on remaps the *pose* vector
// (x,y,z)->(y,z,x) (pose.py:_convert_vectors) which would route our forward
// (+X) onto Blender's vertical (Z); pre-remap NWU(X,Y,Z)->wire(Z,X,Y) so Blender
// recovers (X_fwd, Y_left, Z_up) and walking reads horizontally.  (Sign/order to
// be confirmed against a live Blender + UE capture — see PR notes.)
static inline QVector3D mvnWirePelvisPos(const QVector3D& nwu, LiveTarget target)
{
    if (target == LiveTarget::BlenderMVN)
        return QVector3D(nwu.z(), nwu.x(), nwu.y());
    return nwu;  // UnrealLiveLink: native Z-up RH
}

void LiveStreamSender::pushFrame(quint32 sample,
    const std::array<Quat, kXsensSegmentCount>& segQuat,
    const QVector3D& pelvisPos)
{
    if (!m_running) return;

    if (m_cfg.fps > 0) {
        const qint64 now      = m_impl->timer.elapsed();
        const qint64 periodMs = 1000 / m_cfg.fps;
        if (m_impl->lastEmitMs >= 0 && (now - m_impl->lastEmitMs) < periodMs)
            return;
        m_impl->lastEmitMs = now;
    }

    if (!m_impl->baselineCaptured) {
        m_impl->baselineBodyQ = segQuat;
        m_impl->baselinePelvisPos = pelvisPos;
        m_impl->baselineCaptured = true;
    }

    m_impl->maybeRetransmitHandshake();
    const quint32 ft = quint32(m_impl->timer.elapsed());
    const bool wireDue = m_cfg.verboseLog &&
        (m_impl->lastWireDumpMs < 0 ||
         (m_impl->timer.elapsed() - m_impl->lastWireDumpMs) >= Impl::kWireDumpIntervalMs);
    std::ostringstream wireSS;
    if (wireDue) {
        m_impl->lastWireDumpMs = m_impl->timer.elapsed();
        wireSS << std::fixed << std::setprecision(4)
               << "\n========== [STREAM SNAPSHOT] body-only  sample=" << sample
               << "  -> " << m_cfg.host.toStdString() << ":" << m_cfg.port
               << "  ==========\n"
               << "  wire=MVN MXTP02; q = absolute calibrated world pose (= qOut); "
                  "pelvis pos absolute, per-target remap; metres\n";
    }
    QByteArray body;
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        const QVector3D p = (i == SEG_Pelvis)
                ? mvnWirePelvisPos(pelvisPos, m_cfg.target)
                : QVector3D(0, 0, 0);
        Quat qWire = mvnWireOrient(segQuat[i], m_cfg.target).normalized();
        qWire = hemisphereContinuous(qWire, m_impl->prevWireBodyQ[i]);
        m_impl->prevWireBodyQ[i] = qWire;
        const Quat q = qWire;
        appendPoseSegment(body, i + 1, p.x(), p.y(), p.z(),
                          float(q.w), float(q.x), float(q.y), float(q.z));
        if (wireDue) {
            wireSS << "  wire[" << std::setw(2) << (i + 1) << "] "
                   << std::left << std::setw(14) << kSegmentNames[i] << std::right
                   << " pos=(" << std::setw(8) << p.x() << "," << std::setw(8) << p.y()
                   << "," << std::setw(8) << p.z() << ")"
                   << " q=(" << std::setw(8) << q.w << "," << std::setw(8) << q.x
                   << "," << std::setw(8) << q.y << "," << std::setw(8) << q.z << ")"
                   << " |delta|=" << std::setw(7) << quat_angle_deg(q) << "deg\n";
        }
    }
    QByteArray hdr = buildMxtpHeader("02", sample, 0x80,
                                     quint8(kXsensSegmentCount), ft, 23, 0);
    const QByteArray pkt = hdr + body;
    if (m_cfg.debugDumpFirstFrame && !m_impl->firstFrameDumped) {
        dumpFirstFrameHex("MXTP02 body-only", pkt);
        m_impl->firstFrameDumped = true;
    }
    m_impl->sendChecked(pkt);
    if (wireDue) {
        wireSS << "  pelvis(world,m)=(" << pelvisPos.x() << "," << pelvisPos.y()
               << "," << pelvisPos.z() << ")  packetBytes=" << pkt.size() << "\n"
               << "============================================================\n";
        std::cout << wireSS.str();
        std::cout.flush();
    }
}

void LiveStreamSender::pushFrameWithGloves(quint32 sample,
    const std::array<Quat, kXsensSegmentCount>& segQuat,
    const QVector3D& pelvisPos,
    const std::array<Quat, kFingerSegmentsHand>& rightGloveQ,
    const std::array<QVector3D, kFingerSegmentsHand>& rightGloveP,
    const std::array<Quat, kFingerSegmentsHand>& leftGloveQ,
    const std::array<QVector3D, kFingerSegmentsHand>& leftGloveP)
{
    if (!m_running) return;

    if (m_cfg.fps > 0) {
        const qint64 now      = m_impl->timer.elapsed();
        const qint64 periodMs = 1000 / m_cfg.fps;
        if (m_impl->lastEmitMs >= 0 && (now - m_impl->lastEmitMs) < periodMs)
            return;
        m_impl->lastEmitMs = now;
    }

    if (!m_impl->baselineCaptured) {
        m_impl->baselineBodyQ = segQuat;
        m_impl->baselinePelvisPos = pelvisPos;
        m_impl->baselineCaptured = true;
        if (m_cfg.verboseLog) {
            // -test §6: the T-pose body baseline that makes the wire q identity
            // at the calibration pose (wire q = world . baseline^-1).
            std::ostringstream bs;
            bs << std::fixed << std::setprecision(4)
               << "\n========== [STREAM BASELINE] body T-pose captured ==========\n"
               << "  pelvisPos=(" << pelvisPos.x() << "," << pelvisPos.y() << "," << pelvisPos.z() << ")\n";
            for (int i = 0; i < kXsensSegmentCount; ++i)
                bs << "  base[" << std::setw(2) << i << "] " << std::left << std::setw(14)
                   << kSegmentNames[i] << std::right << " q=(" << segQuat[i].w << ","
                   << segQuat[i].x << "," << segQuat[i].y << "," << segQuat[i].z << ")\n";
            std::cout << bs.str(); std::cout.flush();
        }
    }

    if (m_impl->fingerBaselineSamples < Impl::kFingerBaselineWindow) {
        if (m_impl->fingerBaselineSamples == 0) {
            m_impl->baselineLeftGloveQ = leftGloveQ;
            m_impl->baselineRightGloveQ = rightGloveQ;
        } else {
            const double t = 1.0 / double(m_impl->fingerBaselineSamples + 1);
            for (int i = 0; i < kFingerSegmentsHand; ++i) {
                m_impl->baselineLeftGloveQ[i]  = slerp_quat(m_impl->baselineLeftGloveQ[i],  leftGloveQ[i],  t);
                m_impl->baselineRightGloveQ[i] = slerp_quat(m_impl->baselineRightGloveQ[i], rightGloveQ[i], t);
            }
        }
        m_impl->fingerBaselineSamples++;
        if (m_cfg.verboseLog && m_impl->fingerBaselineSamples == Impl::kFingerBaselineWindow) {
            // -test §6: the averaged finger baselines (identity at T-pose).
            std::ostringstream fb;
            fb << std::fixed << std::setprecision(4)
               << "\n========== [STREAM BASELINE] finger baselines captured ("
               << Impl::kFingerBaselineWindow << " samples) ==========\n";
            for (int i = 0; i < kFingerSegmentsHand; ++i)
                fb << "  L[" << std::setw(2) << i << "]=(" << m_impl->baselineLeftGloveQ[i].w << ","
                   << m_impl->baselineLeftGloveQ[i].x << "," << m_impl->baselineLeftGloveQ[i].y << ","
                   << m_impl->baselineLeftGloveQ[i].z << ")  R[" << std::setw(2) << i << "]=("
                   << m_impl->baselineRightGloveQ[i].w << "," << m_impl->baselineRightGloveQ[i].x << ","
                   << m_impl->baselineRightGloveQ[i].y << "," << m_impl->baselineRightGloveQ[i].z << ")\n";
            std::cout << fb.str(); std::cout.flush();
        }
    }

    m_impl->maybeRetransmitHandshake();
    const quint32 ft = quint32(m_impl->timer.elapsed());
    const quint8  bodyCount = quint8(kXsensSegmentCount);
    const quint8  fingerCount = quint8(2 * kFingerSegmentsHand);
    const bool wireDue = m_cfg.verboseLog &&
        (m_impl->lastWireDumpMs < 0 ||
         (m_impl->timer.elapsed() - m_impl->lastWireDumpMs) >= Impl::kWireDumpIntervalMs);
    std::ostringstream wireSS;
    if (wireDue) {
        m_impl->lastWireDumpMs = m_impl->timer.elapsed();
        wireSS << std::fixed << std::setprecision(4)
               << "\n========== [STREAM SNAPSHOT] body+gloves  sample=" << sample
               << "  -> " << m_cfg.host.toStdString() << ":" << m_cfg.port
               << "  (body=" << int(bodyCount) << " fingers=" << int(fingerCount) << ") ==========\n"
               << "  wire=MVN MXTP02; q = absolute calibrated world pose (= qOut); pelvis pos absolute, per-target remap; metres\n";
    }
    QByteArray body;
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        const QVector3D p = (i == SEG_Pelvis)
                ? mvnWirePelvisPos(pelvisPos, m_cfg.target)
                : QVector3D(0, 0, 0);
        Quat qWire = mvnWireOrient(segQuat[i], m_cfg.target).normalized();
        qWire = hemisphereContinuous(qWire, m_impl->prevWireBodyQ[i]);
        m_impl->prevWireBodyQ[i] = qWire;
        const Quat q = qWire;
        appendPoseSegment(body, i + 1, p.x(), p.y(), p.z(),
                          float(q.w), float(q.x), float(q.y), float(q.z));
        if (wireDue) {
            wireSS << "  wire[" << std::setw(2) << (i + 1) << "] "
                   << std::left << std::setw(14) << kSegmentNames[i] << std::right
                   << " pos=(" << std::setw(8) << p.x() << "," << std::setw(8) << p.y()
                   << "," << std::setw(8) << p.z() << ")"
                   << " q=(" << std::setw(8) << q.w << "," << std::setw(8) << q.x
                   << "," << std::setw(8) << q.y << "," << std::setw(8) << q.z << ")"
                   << " |delta|=" << std::setw(7) << quat_angle_deg(q) << "deg\n";
        }
    }
    static constexpr int kXsensSlotToManus[kFingerSegmentsHand] = {
        -1,  1,  2,  3,
         4,  5,  6,  7,
         8,  9, 10, 11,
        12, 13, 14, 15,
        16, 17, 18, 19,
    };
    auto emitFinger = [&](int slot, int segmentIdBase, int handSeg,
                          const std::array<Quat, kFingerSegmentsHand>& qArr,
                          const std::array<QVector3D, kFingerSegmentsHand>& pArr,
                          const std::array<Quat, kFingerSegmentsHand>& baseArr,
                          std::array<Quat, kFingerSegmentsHand>& prevArr) {
        const int mIdx = kXsensSlotToManus[slot];
        if (mIdx < 0) {
            Quat qWire = mvnWireOrient(segQuat[handSeg], m_cfg.target).normalized();
            qWire = hemisphereContinuous(qWire, prevArr[slot]);
            prevArr[slot] = qWire;
            const Quat q = qWire;  // carpus follows the (now absolute) hand pose
            appendPoseSegment(body, segmentIdBase + slot, 0.f, 0.f, 0.f,
                              float(q.w), float(q.x), float(q.y), float(q.z));
            if (wireDue)
                wireSS << "  wireF[" << std::setw(2) << (segmentIdBase + slot) << "] "
                       << (segmentIdBase >= 44 ? "R" : "L") << "carpus(slot " << slot
                       << ") pos=(0,0,0) q=(" << q.w << "," << q.x << ","
                       << q.y << "," << q.z << ")  [follows hand]\n";
        } else {
            (void)baseArr;  // fingers now stream absolute (see body change)
            const QVector3D p = pArr[mIdx];  // MVN wire = NWU (Z-up RH); no conversion.
            // Absolute world finger orientation (qArr is already wrist-composed
            // world pose).  Matches the body/carpus going absolute so Blender's
            // parent^-1*child stays in one frame across the hand→finger joint.
            Quat qWire = mvnWireOrient(qArr[mIdx], m_cfg.target).normalized();
            qWire = hemisphereContinuous(qWire, prevArr[slot]);
            prevArr[slot] = qWire;
            const Quat q = qWire;
            appendPoseSegment(body, segmentIdBase + slot, p.x(), p.y(), p.z(),
                              float(q.w), float(q.x), float(q.y), float(q.z));
            if (wireDue)
                wireSS << "  wireF[" << std::setw(2) << (segmentIdBase + slot) << "] "
                       << (segmentIdBase >= 44 ? "R" : "L") << "fing(slot " << std::setw(2) << slot
                       << " mIdx " << std::setw(2) << mIdx << ") pos=(" << p.x() << ","
                       << p.y() << "," << p.z() << ") q=(" << q.w << "," << q.x << ","
                       << q.y << "," << q.z << ")\n";
        }
    };
    // emitFinger lambda append'ит сегменты к `body`.  Запомним размер
    // body-only до этого, чтобы потом split mode мог вырезать только
    // finger-часть.
    const int bodyOnlyBytes = body.size();
    for (int slot = 0; slot < kFingerSegmentsHand; ++slot)
        emitFinger(slot, 24, SEG_LHand, leftGloveQ, leftGloveP,
                   m_impl->baselineLeftGloveQ, m_impl->prevWireLeftQ);
    for (int slot = 0; slot < kFingerSegmentsHand; ++slot)
        emitFinger(slot, 44, SEG_RHand, rightGloveQ, rightGloveP,
                   m_impl->baselineRightGloveQ, m_impl->prevWireRightQ);

    // FIX (stream polish): split-mode шлёт body и fingers как два UDP
    // datagram'а через MXTP dgCounter splitting (bit 0..6 = index, bit 7 =
    // last).  Single-mode (default) шлёт всё в одном datagram'е (~2040 байт)
    // — на loopback IP fragmentation работает, на LAN risk потери.
    if (m_cfg.splitGloveDatagrams) {
        const QByteArray bodyOnly    = body.left(bodyOnlyBytes);
        const QByteArray fingerOnly  = body.mid(bodyOnlyBytes);
        QByteArray hdrBody = buildMxtpHeader("02", sample, 0x00,
                                             quint8(bodyCount),
                                             ft, bodyCount, fingerCount);
        const QByteArray pkt1 = hdrBody + bodyOnly;
        if (m_cfg.debugDumpFirstFrame && !m_impl->firstFrameDumped) {
            dumpFirstFrameHex("MXTP02 split body (1/2)", pkt1);
        }
        m_impl->sendChecked(pkt1);
        QByteArray hdrFingers = buildMxtpHeader("02", sample, 0x81,
                                                quint8(fingerCount),
                                                ft, bodyCount, fingerCount);
        const QByteArray pkt2 = hdrFingers + fingerOnly;
        if (m_cfg.debugDumpFirstFrame && !m_impl->firstFrameDumped) {
            dumpFirstFrameHex("MXTP02 split fingers (2/2)", pkt2);
            m_impl->firstFrameDumped = true;
        }
        m_impl->sendChecked(pkt2);
    } else {
        QByteArray hdr = buildMxtpHeader("02", sample, 0x80,
                                         quint8(bodyCount + fingerCount),
                                         ft, bodyCount, fingerCount);
        const QByteArray pkt = hdr + body;
        if (m_cfg.debugDumpFirstFrame && !m_impl->firstFrameDumped) {
            dumpFirstFrameHex("MXTP02 combined", pkt);
            m_impl->firstFrameDumped = true;
        }
        m_impl->sendChecked(pkt);
    }
    if (wireDue) {
        wireSS << "  pelvis(world,m)=(" << pelvisPos.x() << "," << pelvisPos.y()
               << "," << pelvisPos.z() << ")  mode="
               << (m_cfg.splitGloveDatagrams ? "split" : "combined")
               << "  bodyBytes=" << bodyOnlyBytes << "\n"
               << "============================================================\n";
        std::cout << wireSS.str();
        std::cout.flush();
    }
}

// ============================================================================
//  RecordHud — small translucent overlay shown during recording.
// ============================================================================

RecordHud::RecordHud(QWidget* parent) : QWidget(parent)
{
    setObjectName("recordHud");
    setAttribute(Qt::WA_StyledBackground, true);

    m_lblFormat = new QLabel("● REC  BVH · 30 fps", this);
    m_lblFormat->setStyleSheet("color:#E04040; font-weight:900; font-size:11pt;");

    m_lblFrames = new QLabel("0 кадров", this);
    m_lblFrames->setStyleSheet("color:#FFFFFF; font-weight:700;");

    m_lblTime = new QLabel("00:00.0", this);
    m_lblTime->setStyleSheet("color:#FFFFFF; font-family:'Consolas'; font-weight:700;");

    m_btnStop = new QPushButton(Lang::t("rec_stop"), this);
    m_btnStop->setObjectName("primary");
    m_btnStop->setMinimumHeight(34);
    connect(m_btnStop, &QPushButton::clicked, this, &RecordHud::stopClicked);

    auto* lay = new QVBoxLayout(this);
    lay->setContentsMargins(14, 12, 14, 12);
    lay->setSpacing(4);
    lay->addWidget(m_lblFormat);
    auto* row = new QHBoxLayout();
    row->addWidget(m_lblFrames);
    row->addStretch();
    row->addWidget(m_lblTime);
    lay->addLayout(row);
    lay->addSpacing(6);
    lay->addWidget(m_btnStop);

    setFixedWidth(220);
}

void RecordHud::updateStats(qint64 frames, double elapsedSec)
{
    m_lblFrames->setText(QString::number(frames) + "  " + Lang::t("rec_frames"));
    const int mm = int(elapsedSec) / 60;
    const int ss = int(elapsedSec) % 60;
    const int ds = int(elapsedSec * 10) % 10;
    m_lblTime->setText(QString::asprintf("%02d:%02d.%1d", mm, ss, ds));
}

void RecordHud::setFormatLabel(const QString& text) { m_lblFormat->setText(text); }

// ============================================================================
//  RecordWizard — Format → Quality → FPS → Start.
// ============================================================================

RecordWizard::RecordWizard(SuitType suit, QWidget* parent) : QDialog(parent), m_suit(suit)
{
    setModal(true);
    setWindowTitle(Lang::t("rec_wiz_title"));
    setMinimumSize(520, 360);
    setStyleSheet(kStyleSheet);
    buildPages();
}

void RecordWizard::buildPages()
{
    m_pages = new QStackedWidget(this);

    // Page 0 — Format.
    {
        auto* p = new QWidget();
        auto* title = new QLabel(Lang::t("rec_pick_format"), p);
        title->setObjectName("heroHeading");
        title->setAlignment(Qt::AlignCenter);

        m_format = new QComboBox(p);
        m_format->addItem("BVH", int(RecordFormat::BVH));
        m_format->addItem("FBX (ASCII)", int(RecordFormat::FBX));
        m_format->setMinimumHeight(40);

        m_quality = new QComboBox(p);
        m_quality->addItem(Lang::t("rec_quality_normal"), int(RecordQuality::Normal));
        m_quality->addItem(Lang::t("rec_quality_hd"),     int(RecordQuality::HdPostProcessing));
        m_quality->setMinimumHeight(40);

        auto* fLay = new QFormLayout();
        fLay->setContentsMargins(40, 20, 40, 20);
        fLay->setHorizontalSpacing(20);
        fLay->setVerticalSpacing(14);
        fLay->addRow(Lang::t("rec_format") + ":",  m_format);
        fLay->addRow(Lang::t("rec_quality") + ":", m_quality);

        auto* lay = new QVBoxLayout(p);
        lay->addSpacing(20);
        lay->addWidget(title);
        lay->addSpacing(30);
        lay->addLayout(fLay);
        lay->addStretch();

        m_pages->addWidget(p);
    }
    // Page 1 — FPS.
    {
        auto* p = new QWidget();
        auto* title = new QLabel(Lang::t("rec_pick_fps"), p);
        title->setObjectName("heroHeading");
        title->setAlignment(Qt::AlignCenter);

        m_fps = new QComboBox(p);
        m_fps->addItem("24 fps", 24);
        m_fps->addItem("30 fps", 30);
        m_fps->addItem("60 fps", 60);
        if (m_suit == SuitType::Link) {          // Link runs at 240 Hz
            m_fps->addItem("120 fps", 120);
            m_fps->addItem("240 fps", 240);
        }
        m_fps->setCurrentIndex(1);
        m_fps->setMinimumHeight(40);

        auto* fLay = new QFormLayout();
        fLay->setContentsMargins(40, 20, 40, 20);
        fLay->setHorizontalSpacing(20);
        fLay->setVerticalSpacing(14);
        fLay->addRow(Lang::t("rec_fps") + ":", m_fps);

        auto* lay = new QVBoxLayout(p);
        lay->addSpacing(20);
        lay->addWidget(title);
        lay->addSpacing(30);
        lay->addLayout(fLay);
        lay->addStretch();

        m_pages->addWidget(p);
    }
    // Page 2 — Review / Start.
    {
        auto* p = new QWidget();
        auto* title = new QLabel(Lang::t("rec_ready"), p);
        title->setObjectName("heroHeading");
        title->setAlignment(Qt::AlignCenter);

        auto* hint = new QLabel(Lang::t("rec_ready_hint"), p);
        hint->setAlignment(Qt::AlignCenter);
        hint->setWordWrap(true);
        hint->setStyleSheet("color:#CFCFCF; font-size:11pt;");

        auto* lay = new QVBoxLayout(p);
        lay->addSpacing(30);
        lay->addWidget(title);
        lay->addSpacing(20);
        lay->addWidget(hint);
        lay->addStretch();

        m_pages->addWidget(p);
    }

    m_btnBack   = new QPushButton(Lang::t("back"),     this);
    m_btnNext   = new QPushButton(Lang::t("continue"), this);
    m_btnStart  = new QPushButton(Lang::t("rec_start"),this);
    m_btnStart->setObjectName("primary");
    m_btnCancel = new QPushButton(Lang::t("cancel"),   this);

    connect(m_btnBack,   &QPushButton::clicked, this, &RecordWizard::goBack);
    connect(m_btnNext,   &QPushButton::clicked, this, &RecordWizard::goNext);
    connect(m_btnCancel, &QPushButton::clicked, this, &QDialog::reject);
    connect(m_btnStart,  &QPushButton::clicked, this, [this]() {
        m_result.format  = RecordFormat (m_format ->currentData().toInt());
        m_result.quality = RecordQuality(m_quality->currentData().toInt());
        m_result.fps     = m_fps->currentData().toInt();
        accept();
    });

    auto* nav = new QHBoxLayout();
    nav->addWidget(m_btnCancel);
    nav->addStretch();
    nav->addWidget(m_btnBack);
    nav->addWidget(m_btnNext);
    nav->addWidget(m_btnStart);

    auto* main = new QVBoxLayout(this);
    main->addWidget(m_pages, 1);
    main->addLayout(nav);
    updateNav();
}

void RecordWizard::goNext()
{
    if (m_pageIdx < m_pages->count() - 1) {
        ++m_pageIdx;
        m_pages->setCurrentIndex(m_pageIdx);
        updateNav();
    }
}
void RecordWizard::goBack()
{
    if (m_pageIdx > 0) {
        --m_pageIdx;
        m_pages->setCurrentIndex(m_pageIdx);
        updateNav();
    }
}
void RecordWizard::updateNav()
{
    const int last = m_pages->count() - 1;
    m_btnBack->setEnabled(m_pageIdx > 0);
    m_btnNext->setVisible(m_pageIdx < last);
    m_btnStart->setVisible(m_pageIdx == last);
}

// ============================================================================
//  LiveStreamWizard — target, host, port, Start.
// ============================================================================

LiveStreamWizard::LiveStreamWizard(SuitType suit, QWidget* parent) : QDialog(parent), m_suit(suit)
{
    setModal(true);
    setWindowTitle(Lang::t("live_wiz_title"));
    setMinimumSize(460, 300);
    setStyleSheet(kStyleSheet);

    auto* title = new QLabel(Lang::t("live_wiz_title"), this);
    title->setObjectName("heroHeading");
    title->setAlignment(Qt::AlignCenter);

    m_target = new QComboBox(this);
    m_target->addItem(QString::fromUtf8("\xF0\x9F\x94\xB6  Blender — MVN Live (MXTP)"),
                      int(LiveTarget::BlenderMVN));
    m_target->addItem(QString::fromUtf8("\xF0\x9F\x8E\xAE  Unreal Engine — LiveLink (MXTP)"),
                      int(LiveTarget::UnrealLiveLink));
    m_target->setMinimumHeight(34);

    m_host = new QComboBox(this);
    m_host->setEditable(true);
    m_host->setMinimumHeight(34);
    m_host->addItem("127.0.0.1", "127.0.0.1");        // дать data чтобы парсинг был единообразный
    {
        QStringList seen;
        seen << "127.0.0.1";
        const auto ifaces = QNetworkInterface::allInterfaces();
        for (const auto& iface : ifaces) {
            if (!(iface.flags() & QNetworkInterface::IsUp))      continue;
            if (!(iface.flags() & QNetworkInterface::IsRunning)) continue;
            if (iface.flags() & QNetworkInterface::IsLoopBack)   continue;
            const auto entries = iface.addressEntries();
            for (const auto& e : entries) {
                const QHostAddress a = e.ip();
                if (a.protocol() != QAbstractSocket::IPv4Protocol) continue;
                const QString s = a.toString();
                if (seen.contains(s)) continue;
                seen << s;
                const QString label = QStringLiteral("%1   (%2)").arg(s, iface.humanReadableName());
                m_host->addItem(label, s);
            }
        }
    }

    m_port = new QSpinBox(this);
    m_port->setRange(1024, 65535);
    m_port->setValue(9763);
    m_port->setMinimumHeight(34);
    m_port->setStyleSheet(
        "QSpinBox { background: #1a1a1a; color: #FFFFFF;"
        " border: 1px solid #3a3a3a; border-radius: 6px; padding: 4px 8px; }"
        "QSpinBox:focus { border-color: #FF7A1A; }");

    // Новый: выбор частоты выходного потока 24/30/60.
    m_fps = new QComboBox(this);
    m_fps->addItem("24 fps", 24);
    m_fps->addItem("30 fps", 30);
    m_fps->addItem("60 fps", 60);
    if (m_suit == SuitType::Link) {             // Link runs at 240 Hz
        m_fps->addItem("120 fps", 120);
        m_fps->addItem("240 fps", 240);
    }
    m_fps->setCurrentIndex(2);                  // дефолт 60
    m_fps->setMinimumHeight(34);

    m_btnStart  = new QPushButton(Lang::t("live_start"), this);
    m_btnStart->setObjectName("primary");
    m_btnStart->setMinimumHeight(40);
    m_btnCancel = new QPushButton(Lang::t("cancel"), this);
    m_btnCancel->setMinimumHeight(40);

    connect(m_btnStart, &QPushButton::clicked, this, [this]() {
        m_result.target = LiveTarget(m_target->currentData().toInt());

        // Хост: всегда из currentText() — currentData() устаревает при
        // ручном вводе, а split-по-пробелу одинаково режет и "1.2.3.4",
        // и "1.2.3.4   (Wi-Fi)".
        QString host = m_host->currentText().trimmed();
        const int sp = host.indexOf(' ');
        if (sp > 0) host = host.left(sp);
        m_result.host = host;

        m_result.port = m_port->value();
        m_result.fps  = m_fps->currentData().toInt();
        accept();
    });
    connect(m_btnCancel, &QPushButton::clicked, this, &QDialog::reject);

    auto* fLay = new QFormLayout();
    fLay->setContentsMargins(32, 10, 32, 10);
    fLay->setHorizontalSpacing(20);
    fLay->setVerticalSpacing(14);
    fLay->addRow(Lang::t("live_target")   + ":", m_target);
    fLay->addRow(Lang::t("live_host")     + ":", m_host);
    fLay->addRow(Lang::t("live_port")     + ":", m_port);
    fLay->addRow(Lang::t("live_fps")      + ":", m_fps);

    auto* nav = new QHBoxLayout();
    nav->addWidget(m_btnCancel);
    nav->addStretch();
    nav->addWidget(m_btnStart);

    auto* lay = new QVBoxLayout(this);
    lay->setContentsMargins(20, 20, 20, 20);
    lay->addWidget(title);
    lay->addSpacing(10);
    lay->addLayout(fLay);
    lay->addStretch();
    lay->addLayout(nav);
}

// ============================================================================
//  BVH writer — 23-bone Xsens skeleton, intrinsic ZXY Euler channels.
//
//  Output follows the BVH spec (Paul Torrelli).  We write world-space
//  pelvis translation on every frame, and Euler rotations per segment in
//  ZXY order (industry default for mocap, matches Blender/Maya FBX import).
// ============================================================================

namespace {

struct BvhBone {
    const char* name;
    int         parent;
    double      offx, offy, offz;   // in cm
    bool        isEnd = false;
};

// Skeleton table built from SkeletonXsens segment layout.  Offsets are
// ratio-of-height so we resolve them against the actor's actual height at
// write time.  Numbers are the same Drillis-Contini ratios used by
// SkeletonXsens in the viewport — kept in sync by eyeball, not compiled.
struct BvhBoneRatio { const char* name; int parent; double dx, dy, dz; };
static const BvhBoneRatio kBvh[] = {
    // 0  Hips (pelvis) — root
    { "Hips",             -1,   0.0,   0.000, 0.0 },
    { "Spine",             0,   0.0,   0.100, 0.0 },    // L5
    { "Spine1",            1,   0.0,   0.095, 0.0 },    // L3
    { "Spine2",            2,   0.0,   0.093, 0.0 },    // T12
    { "Chest",             3,   0.0,   0.090, 0.0 },    // T8
    { "Neck",              4,   0.0,   0.060, 0.0 },
    { "Head",              5,   0.0,   0.095, 0.0 },
    { "RightShoulder",     4,  -0.030, 0.055, 0.0 },
    { "RightArm",          7,  -0.105, 0.0,   0.0 },
    { "RightForeArm",      8,  -0.186, 0.0,   0.0 },
    { "RightHand",         9,  -0.146, 0.0,   0.0 },
    { "LeftShoulder",      4,   0.030, 0.055, 0.0 },
    { "LeftArm",          11,   0.105, 0.0,   0.0 },
    { "LeftForeArm",      12,   0.186, 0.0,   0.0 },
    { "LeftHand",         13,   0.146, 0.0,   0.0 },
    { "RightUpLeg",        0,  -0.055,-0.050, 0.0 },
    { "RightLeg",         15,   0.0,  -0.245, 0.0 },
    { "RightFoot",        16,   0.0,  -0.246, 0.0 },
    { "RightToeBase",     17,   0.0,  -0.02,  0.16 },
    { "LeftUpLeg",         0,   0.055,-0.050, 0.0 },
    { "LeftLeg",          19,   0.0,  -0.245, 0.0 },
    { "LeftFoot",         20,   0.0,  -0.246, 0.0 },
    { "LeftToeBase",      21,   0.0,  -0.02,  0.16 },
};

// Quaternion → intrinsic ZXY Euler (degrees).  Matches Blender / Maya
// "rotate order ZXY" convention used by most mocap toolchains.
static void quatToEulerZXYdeg(const Quat& q, double& rz, double& rx, double& ry)
{
    // Rotation matrix from quaternion.
    const double w=q.w, x=q.x, y=q.y, z=q.z;
    const double m00 = 1-2*(y*y+z*z);
    const double m01 = 2*(x*y - z*w);
    const double m02 = 2*(x*z + y*w);
    const double m10 = 2*(x*y + z*w);
    const double m11 = 1-2*(x*x+z*z);
    const double m12 = 2*(y*z - x*w);
    const double m20 = 2*(x*z - y*w);
    const double m21 = 2*(y*z + x*w);
    const double m22 = 1-2*(x*x+y*y);
    (void)m00; (void)m02;
    // ZXY: X = asin(m21); Z = atan2(-m01, m11); Y = atan2(-m20, m22);
    const double sx = std::clamp(m21, -1.0, 1.0);
    const double xrad = std::asin(sx);
    const double zrad = std::atan2(-m01, m11);
    const double yrad = std::atan2(-m20, m22);
    const double K = 180.0 / M_PI;
    rx = xrad * K;
    ry = yrad * K;
    rz = zrad * K;
}

// kBvh joint index → Xsens SEG index (shared by the BVH and FBX writers).
static const int kBoneToSeg[] = {
    SEG_Pelvis,      SEG_L5,          SEG_L3,
    SEG_T12,         SEG_T8,          SEG_Neck,
    SEG_Head,        SEG_RShoulder,   SEG_RUpperArm,
    SEG_RForearm,    SEG_RHand,       SEG_LShoulder,
    SEG_LUpperArm,   SEG_LForearm,    SEG_LHand,
    SEG_RUpperLeg,   SEG_RLowerLeg,   SEG_RFoot,
    SEG_RToe,        SEG_LUpperLeg,   SEG_LLowerLeg,
    SEG_LFoot,       SEG_LToe,
};
static_assert(std::size(kBoneToSeg) == std::size(kBvh),
              "kBoneToSeg must cover every kBvh joint");

// The viewport poses segment i with WORLD orientation W[i] = raw[i]·defAng[i]
// (see SkeletonXsens::computeKeypoints).  segQuat stores raw[i].
static std::array<Quat, kXsensSegmentCount>
exportWorldOrients(const RecordedFrame& fr, const SkeletonXsens& skel)
{
    std::array<Quat, kXsensSegmentCount> W;
    for (int s = 0; s < kXsensSegmentCount; ++s)
        W[s] = quat_mult(fr.segQuat[s], skel.defAngFor(s)).normalized();
    return W;
}

// Keep successive Euler samples on the same 2π branch (no ±360° pops in long
// clips).  Gimbal lock at ZXY X=±90° is inherent and not addressed here.
static inline double unwrapDeg(double prev, double cur)
{
    double d = cur - prev;
    while (d >  180.0) { cur -= 360.0; d = cur - prev; }
    while (d < -180.0) { cur += 360.0; d = cur - prev; }
    return cur;
}

// Rest OFFSET (cm) per kBvh joint, expressed in its parent's rest frame,
// baked from the FK rest pose (raw = identity → W_rest = defAng) so the
// exported bind pose and joint positions reproduce the viewport exactly.
static std::array<QVector3D, std::size(kBvh)>
exportBakedOffsetsCm(const SkeletonXsens& skel)
{
    std::array<Quat, kXsensSegmentCount> ident;
    ident.fill(Quat(1, 0, 0, 0));
    const auto kpRest = skel.computeKeypoints(ident, QVector3D(0, 0, 0));
    std::array<QVector3D, std::size(kBvh)> off{};
    for (size_t j = 0; j < std::size(kBvh); ++j) {
        if (kBvh[j].parent < 0) { off[j] = QVector3D(0, 0, 0); continue; }
        const int ownSeg    = kBoneToSeg[j];
        const int parentSeg = kBoneToSeg[kBvh[j].parent];
        const QVector3D d = kpRest[ownSeg] - kpRest[parentSeg];
        off[j] = vec_rotate(d, skel.defAngFor(parentSeg).inv()) * 100.0f;
    }
    return off;
}

// Parent-local rotation of kBvh joint j for one frame's world orientations.
static inline Quat exportLocalRot(int j,
                                  const std::array<Quat, kXsensSegmentCount>& W)
{
    const int ownSeg = kBoneToSeg[j];
    if (kBvh[j].parent < 0) return W[ownSeg];
    const int parentSeg = kBoneToSeg[kBvh[j].parent];
    return quat_mult(W[parentSeg].inv(), W[ownSeg]).normalized();
}

static bool writeBvh(const QString& path,
                     const std::vector<RecordedFrame>& frames,
                     int fps,
                     double heightMeters,
                     const SkeletonXsens& skel)
{
    if (fps <= 0) fps = 30;            // guard the 1.0/fps Frame-Time division
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
        return false;
    QTextStream os(&f);
    os.setLocale(QLocale::c());        // never emit decimal commas
    os.setRealNumberPrecision(6);
    os.setRealNumberNotation(QTextStream::FixedNotation);

    (void)heightMeters;                // offsets now come from the FK rest pose
    const auto offCm = exportBakedOffsetsCm(skel);

    // --- Skeleton (HIERARCHY) ---
    std::vector<int> childCount(std::size(kBvh), 0);
    for (size_t i = 1; i < std::size(kBvh); ++i) childCount[kBvh[i].parent]++;

    // DFS order is captured during emission so the MOTION columns below can
    // never desync from the hierarchy.
    std::vector<int> dfsOrder;
    dfsOrder.reserve(std::size(kBvh));

    // "emit" is a Qt macro — using it as an identifier expands into empty
    // and trips C2513.  Rename the recursive lambda to emitBone.
    std::function<void(int, int)> emitBone = [&](int idx, int indent) {
        auto pad = [&](int n){ return QString(n, QChar('\t')); };
        const auto& b = kBvh[idx];
        dfsOrder.push_back(idx);
        if (idx == 0) {
            os << "ROOT " << b.name << "\n";
        } else {
            os << pad(indent) << "JOINT " << b.name << "\n";
        }
        os << pad(indent) << "{\n";
        os << pad(indent+1) << "OFFSET "
           << offCm[idx].x() << " " << offCm[idx].y() << " " << offCm[idx].z() << "\n";
        if (idx == 0) {
            os << pad(indent+1)
               << "CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n";
        } else {
            os << pad(indent+1) << "CHANNELS 3 Zrotation Xrotation Yrotation\n";
        }
        for (size_t j = 0; j < std::size(kBvh); ++j) {
            if (int(kBvh[j].parent) == idx) emitBone(int(j), indent + 1);
        }
        if (childCount[idx] == 0) {
            os << pad(indent+1) << "End Site\n";
            os << pad(indent+1) << "{\n";
            os << pad(indent+2) << "OFFSET 0.0 5.0 0.0\n";
            os << pad(indent+1) << "}\n";
        }
        os << pad(indent) << "}\n";
    };

    os << "HIERARCHY\n";
    emitBone(0, 0);

    // --- Motion ---
    os << "MOTION\n";
    os << "Frames: " << frames.size() << "\n";
    os << "Frame Time: " << (1.0 / double(fps)) << "\n";

    // Per-joint Euler-unwrap state, indexed by kBvh joint index.
    std::array<double, std::size(kBvh)> prZ{}, prX{}, prY{};
    bool havePrev = false;

    for (const RecordedFrame& fr : frames) {
        const auto W = exportWorldOrients(fr, skel);

        // Root translation in cm.
        os << (fr.pelvisPos.x() * 100.0) << " "
           << (fr.pelvisPos.y() * 100.0) << " "
           << (fr.pelvisPos.z() * 100.0);

        for (int idx : dfsOrder) {
            double rz, rx, ry;
            quatToEulerZXYdeg(exportLocalRot(idx, W), rz, rx, ry);
            if (havePrev) {
                rz = unwrapDeg(prZ[idx], rz);
                rx = unwrapDeg(prX[idx], rx);
                ry = unwrapDeg(prY[idx], ry);
            }
            prZ[idx] = rz; prX[idx] = rx; prY[idx] = ry;
            os << " " << rz << " " << rx << " " << ry;
        }
        havePrev = true;
        os << "\n";
    }
    return true;
}

// ---------------------------------------------------------------------------
//  ASCII FBX writer — minimal skeleton + animation curves.
//  FBX 7.4 ASCII subset that Maya / Blender / MotionBuilder accept.  We
//  emit Euler animation curves per bone on a single AnimationLayer.
// ---------------------------------------------------------------------------
static bool writeFbxAscii(const QString& path,
                          const std::vector<RecordedFrame>& frames,
                          int fps,
                          double heightMeters,
                          const SkeletonXsens& skel)
{
    if (fps <= 0) fps = 30;            // guard the kTimePerSecond/fps division
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
        return false;
    QTextStream os(&f);
    os.setLocale(QLocale::c());        // never emit decimal commas
    os.setRealNumberPrecision(6);
    os.setRealNumberNotation(QTextStream::FixedNotation);

    (void)heightMeters;                // offsets now come from the FK rest pose
    const auto offCm = exportBakedOffsetsCm(skel);

    // Header boilerplate.
    os << "; FBX 7.4.0 project file\n";
    os << "; Created by Fox Mocap\n";
    os << "FBXHeaderExtension:  {\n";
    os << "    FBXHeaderVersion: 1003\n";
    os << "    FBXVersion: 7400\n";
    os << "    CreationTimeStamp:  { Version: 1000 }\n";
    os << "    Creator: \"Fox Mocap ASCII FBX Writer\"\n";
    os << "}\n";
    // Skeleton frame is NWU (Z-up); declare it honestly so the translation
    // and orientation land the right way up in Maya / Blender / UE.
    os << "GlobalSettings:  {\n";
    os << "    Version: 1000\n";
    os << "    Properties70:  {\n";
    os << "        P: \"UpAxis\", \"int\", \"Integer\", \"\",2\n";
    os << "        P: \"UpAxisSign\", \"int\", \"Integer\", \"\",1\n";
    os << "        P: \"FrontAxis\", \"int\", \"Integer\", \"\",1\n";
    os << "        P: \"FrontAxisSign\", \"int\", \"Integer\", \"\",-1\n";
    os << "        P: \"CoordAxis\", \"int\", \"Integer\", \"\",0\n";
    os << "        P: \"CoordAxisSign\", \"int\", \"Integer\", \"\",1\n";
    os << "        P: \"UnitScaleFactor\", \"double\", \"Number\", \"\",1\n";
    // FBX FbxTime::EMode: 24→11, 30→6, 60→3, 120→1 (eFrames120); 240 has no
    // native mode → eCustom(14).  Keyframe KTime below is exact for any fps, so
    // playback speed stays correct even when the mode is only a timeline hint.
    const int timeMode = (fps == 24)  ? 11
                       : (fps == 30)  ? 6
                       : (fps == 60)  ? 3
                       : (fps == 120) ? 1
                       : 14;
    os << "        P: \"TimeMode\", \"enum\", \"\", \"\"," << timeMode << "\n";
    os << "    }\n";
    os << "}\n";

    const size_t boneN = std::size(kBvh);

    // Definitions.  Count = number of ObjectType blocks (5).  Curve-node /
    // curve counts include the extra Hips translation node + its 3 curves.
    os << "Definitions:  {\n";
    os << "    Version: 100\n";
    os << "    Count: 5\n";
    os << "    ObjectType: \"Model\" {\n";
    os << "        Count: " << boneN << "\n";
    os << "    }\n";
    os << "    ObjectType: \"AnimationStack\" { Count: 1 }\n";
    os << "    ObjectType: \"AnimationLayer\" { Count: 1 }\n";
    os << "    ObjectType: \"AnimationCurveNode\" { Count: " << (boneN + 1) << " }\n";
    os << "    ObjectType: \"AnimationCurve\" { Count: " << (boneN * 3 + 3) << " }\n";
    os << "}\n";

    // Animation id ranges + time base.
    const qint64 stackId = 20000, layerId = 20001;
    const qint64 curveIdBase = 30000;
    const qint64 curveNodeIdBase = 25000;
    const qint64 transNodeId = curveNodeIdBase + 1000;
    const qint64 transCurveId0 = curveIdBase + 100000;
    const qint64 kTimePerSecond = 46186158000LL;   // FBX KTime ticks per second
    const qint64 stepTicks = kTimePerSecond / fps;
    const size_t N = frames.size();
    const qint64 stopTicks = qint64(N > 0 ? N - 1 : 0) * stepTicks;
    auto emitKeyTimes = [&]() {
        os << "        KeyTime: *" << N << " {\n            a: ";
        for (size_t k = 0; k < N; ++k) { if (k) os << ","; os << (qint64(k) * stepTicks); }
        os << "\n        }\n";
    };

    // Objects — bone models (rest Lcl Translation = baked offset, cm).
    os << "Objects:  {\n";
    auto boneId = [](size_t i) { return 10000LL + qint64(i) * 16; };
    for (size_t i = 0; i < boneN; ++i) {
        const auto& b = kBvh[i];
        os << "    Model: " << boneId(i) << ", \"Model::" << b.name << "\", \"LimbNode\" {\n";
        os << "        Version: 232\n";
        os << "        Properties70:  {\n";
        os << "            P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A+\","
           << offCm[i].x() << "," << offCm[i].y() << "," << offCm[i].z() << "\n";
        os << "            P: \"RotationOrder\", \"enum\", \"\", \"\", 2\n";   // ZXY
        os << "        }\n";
        os << "    }\n";
    }
    // AnimationStack + Layer.
    os << "    AnimationStack: " << stackId << ", \"AnimStack::Take 001\", \"\" {\n";
    os << "        Properties70:  {\n";
    os << "            P: \"LocalStart\", \"KTime\", \"Time\", \"\",0\n";
    os << "            P: \"LocalStop\", \"KTime\", \"Time\", \"\"," << stopTicks << "\n";
    os << "        }\n";
    os << "    }\n";
    os << "    AnimationLayer: " << layerId << ", \"AnimLayer::BaseLayer\", \"\" {}\n";

    // Per-bone rotation curve nodes + three curves, in PARENT-LOCAL Euler
    // (ZXY) with continuity unwrap.
    for (size_t i = 0; i < boneN; ++i) {
        const qint64 nodeId = curveNodeIdBase + qint64(i);
        os << "    AnimationCurveNode: " << nodeId
           << ", \"AnimCurveNode::R\", \"\" {}\n";

        // Local rotation = inv(W[parent])·W[own], W[seg] = raw[seg]·defAng[seg].
        // Compute only the two segments this joint needs (cheap per frame).
        const int ownSeg    = kBoneToSeg[i];
        const int parentSeg = (kBvh[i].parent < 0) ? -1 : kBoneToSeg[kBvh[i].parent];
        const Quat dOwn = skel.defAngFor(ownSeg);
        const Quat dPar = (parentSeg < 0) ? Quat(1,0,0,0) : skel.defAngFor(parentSeg);

        std::vector<double> vx(N), vy(N), vz(N);
        double pz = 0, px = 0, py = 0;
        for (size_t k = 0; k < N; ++k) {
            const Quat Wown = quat_mult(frames[k].segQuat[ownSeg], dOwn).normalized();
            Quat L;
            if (parentSeg < 0) {
                L = Wown;
            } else {
                const Quat Wpar = quat_mult(frames[k].segQuat[parentSeg], dPar).normalized();
                L = quat_mult(Wpar.inv(), Wown).normalized();
            }
            double rz, rx, ry;
            quatToEulerZXYdeg(L, rz, rx, ry);
            if (k) { rz = unwrapDeg(pz, rz); rx = unwrapDeg(px, rx); ry = unwrapDeg(py, ry); }
            pz = rz; px = rx; py = ry;
            vx[k] = rx; vy[k] = ry; vz[k] = rz;
        }
        for (int ax = 0; ax < 3; ++ax) {
            const qint64 cid = curveIdBase + qint64(i) * 3 + ax;
            const std::vector<double>& vals = (ax == 0) ? vx : (ax == 1 ? vy : vz);
            os << "    AnimationCurve: " << cid << ", \"AnimCurve::\", \"\" {\n";
            os << "        Default: 0\n";
            os << "        KeyVer: 4008\n";
            emitKeyTimes();
            os << "        KeyValueFloat: *" << N << " {\n            a: ";
            for (size_t k = 0; k < N; ++k) { if (k) os << ","; os << vals[k]; }
            os << "\n        }\n";
            os << "        KeyAttrFlags: *1 { a: 24840 }\n";
            os << "        KeyAttrDataFloat: *4 { a: 0,0,218434821,0 }\n";
            os << "        KeyAttrRefCount: *1 { a: " << N << " }\n";
            os << "    }\n";
        }
    }

    // Root (Hips) translation curve node + three curves (cm) — the locomotion.
    os << "    AnimationCurveNode: " << transNodeId
       << ", \"AnimCurveNode::T\", \"\" {}\n";
    for (int ax = 0; ax < 3; ++ax) {
        const qint64 cid = transCurveId0 + ax;
        os << "    AnimationCurve: " << cid << ", \"AnimCurve::\", \"\" {\n";
        os << "        Default: 0\n";
        os << "        KeyVer: 4008\n";
        emitKeyTimes();
        os << "        KeyValueFloat: *" << N << " {\n            a: ";
        for (size_t k = 0; k < N; ++k) {
            if (k) os << ",";
            const QVector3D& p = frames[k].pelvisPos;
            os << ((ax == 0) ? p.x() : (ax == 1 ? p.y() : p.z())) * 100.0;
        }
        os << "\n        }\n";
        os << "        KeyAttrFlags: *1 { a: 24840 }\n";
        os << "        KeyAttrDataFloat: *4 { a: 0,0,218434821,0 }\n";
        os << "        KeyAttrRefCount: *1 { a: " << N << " }\n";
        os << "    }\n";
    }
    os << "}\n";

    // Connections — parent/child graph + curve nodes → bones.
    os << "Connections:  {\n";
    for (size_t i = 0; i < boneN; ++i) {
        if (kBvh[i].parent >= 0)
            os << "    C: \"OO\"," << boneId(i)
               << "," << boneId(kBvh[i].parent) << "\n";
    }
    os << "    C: \"OO\"," << layerId << "," << stackId << "\n";
    for (size_t i = 0; i < boneN; ++i) {
        const qint64 nodeId = curveNodeIdBase + qint64(i);
        os << "    C: \"OO\"," << nodeId << "," << layerId << "\n";
        os << "    C: \"OP\"," << nodeId << "," << boneId(i)
           << ",\"Lcl Rotation\"\n";
        for (int ax = 0; ax < 3; ++ax) {
            const qint64 cid = curveIdBase + qint64(i) * 3 + ax;
            const char* prop = (ax == 0) ? "d|X" : (ax == 1 ? "d|Y" : "d|Z");
            os << "    C: \"OP\"," << cid << "," << nodeId
               << ",\"" << prop << "\"\n";
        }
    }
    // Hips translation node → Hips model.
    os << "    C: \"OO\"," << transNodeId << "," << layerId << "\n";
    os << "    C: \"OP\"," << transNodeId << "," << boneId(0)
       << ",\"Lcl Translation\"\n";
    for (int ax = 0; ax < 3; ++ax) {
        const qint64 cid = transCurveId0 + ax;
        const char* prop = (ax == 0) ? "d|X" : (ax == 1 ? "d|Y" : "d|Z");
        os << "    C: \"OP\"," << cid << "," << transNodeId
           << ",\"" << prop << "\"\n";
    }
    os << "}\n";
    return true;
}

// ---------------------------------------------------------------------------
//  HD post-processing — offline cleanup pass for a finished recording.
//
//  Balanced cleanup that removes jitter / spikes / foot-skate while keeping
//  motion crisp.  Pipeline (each pass deterministic and O(N)):
//      1) per-segment outlier rejection (3σ on Δquat angular distance),
//      2) per-segment Gaussian SLERP smoothing — ONE zero-phase pass
//         (window ≈ ±5 frames); a second low-pass over-smoothed the take,
//      3) finger-chain Gaussian smoothing (only if gloves present),
//      4) root-position Butterworth low-pass (cutoff ≈ 5 Hz, filtfilt),
//      5) foot-contact ZUPT (zero-velocity update at still frames).
//
//  Real MVN also does biomechanical joint-limit projection + Kalman/RTS,
//  but those require the proprietary XME body model we don't have here.
//  The drop-in equivalents above remove the bulk of visible jitter without
//  touching the coordinate system.  Everything operates on RecordedFrame
//  in-place; passes 4–5 act on the now-real pelvis trajectory.
// ---------------------------------------------------------------------------

static double angBetween(const Quat& a, const Quat& b)
{
    double d = std::abs(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z);
    if (d > 1.0) d = 1.0;
    return 2.0 * std::acos(d);
}

// Pass 1 — reject quaternion outliers.  For each segment, compute
// frame-to-frame angular delta; any frame whose delta exceeds μ + 3σ is
// replaced by SLERP(prev, next) so the smoother downstream doesn't get
// hit by one-sample spikes.
static void hdOutlierReject(std::vector<RecordedFrame>& fr,
                            const std::function<void(double)>& cb = {})
{
    if (fr.size() < 3) return;
    const size_t M = fr.size() - 1;          // count of frame-to-frame deltas
    for (int s = 0; s < kXsensSegmentCount; ++s) {
        if (cb) cb(double(s) / double(kXsensSegmentCount));
        std::vector<double> d(fr.size(), 0.0);
        for (size_t i = 1; i < fr.size(); ++i)
            d[i] = angBetween(fr[i-1].segQuat[s], fr[i].segQuat[s]);
        double mean = 0.0;
        for (size_t i = 1; i < fr.size(); ++i) mean += d[i];
        mean /= double(M);
        double var = 0.0;
        for (size_t i = 1; i < fr.size(); ++i) { const double e = d[i]-mean; var += e*e; }
        const double sd  = std::sqrt(var / double(M));
        const double thr = mean + 3.0 * sd;
        for (size_t i = 1; i + 1 < fr.size(); ++i) {
            if (d[i] > thr && d[i+1] > thr * 0.6) {
                fr[i].segQuat[s] = slerp_quat(fr[i-1].segQuat[s],
                                              fr[i+1].segQuat[s], 0.5);
            }
        }
    }
}

// Pass 2 — Gaussian SLERP smoother (zero-phase, symmetric kernel, truncated &
// renormalised at clip ends).  Half-width 6, σ = 2.5 (≈ ±5-frame support):
// removes jitter while preserving the crispness of fast motion.  This is the
// SINGLE body-rotation smoother — stacking a second low-pass (the old RTS
// pass) over-smoothed the take and doubled peak memory, so it was removed.
static void hdQuatSmooth(std::vector<RecordedFrame>& fr,
                         const std::function<void(double)>& cb = {})
{
    const int half = 6;
    const double sigma = 2.5;
    std::vector<double> k(2*half + 1);
    double sumK = 0.0;
    for (int i = -half; i <= half; ++i) {
        k[i + half] = std::exp(-0.5 * (i*i) / (sigma*sigma));
        sumK += k[i + half];
    }
    for (auto& v : k) v /= sumK;

    const int N = int(fr.size());
    if (N < 3) return;
    std::vector<std::array<Quat, kXsensSegmentCount>> out(N);
    for (int i = 0; i < N; ++i) {
        if (cb && (i & 0x0FFF) == 0) cb(double(i) / double(N));
        for (int s = 0; s < kXsensSegmentCount; ++s) {
            // Weighted quaternion average via eigen-of-4x4 accumulator.
            // With a Gaussian window the iterative SLERP blend is stable
            // and avoids needing Eigen — start from the centre sample and
            // slerp neighbours in, weighted by kernel.
            Quat acc = fr[i].segQuat[s];
            double wAcc = k[half];
            for (int off = 1; off <= half; ++off) {
                if (i - off >= 0) {
                    const double w = k[half - off];
                    const double t = w / (wAcc + w);
                    acc = slerp_quat(acc, fr[i - off].segQuat[s], t);
                    wAcc += w;
                }
                if (i + off < N) {
                    const double w = k[half + off];
                    const double t = w / (wAcc + w);
                    acc = slerp_quat(acc, fr[i + off].segQuat[s], t);
                    wAcc += w;
                }
            }
            out[i][s] = acc.normalized();
        }
    }
    for (int i = 0; i < N; ++i) fr[i].segQuat = out[i];
}

// Pass 3 — Butterworth 2nd-order low-pass on pelvis position.  Cutoff
// ≈ 5 Hz at 240 Hz sampling yields coefficients below.  Applied twice
// (forward and reverse) for zero phase delay, à la scipy filtfilt.
static void hdRootLowpass(std::vector<RecordedFrame>& fr, int fps)
{
    if (fr.size() < 4) return;
    const double fc = 5.0;
    const double fs = std::max(30.0, double(fps));
    const double c = std::tan(M_PI * fc / fs);
    const double c2 = c * c;
    const double denom = 1 + std::sqrt(2.0) * c + c2;
    const double b0 = c2 / denom, b1 = 2*b0, b2 = b0;
    const double a1 = 2*(c2 - 1) / denom;
    const double a2 = (1 - std::sqrt(2.0)*c + c2) / denom;

    auto pass = [&](std::function<QVector3D&(size_t)> get) {
        const QVector3D init = get(0);
        QVector3D x1 = init, x2 = init, y1 = init, y2 = init;
        for (size_t i = 0; i < fr.size(); ++i) {
            QVector3D& p = get(i);
            const QVector3D xn = p;
            const QVector3D yn = b0*xn + b1*x1 + b2*x2 - a1*y1 - a2*y2;
            x2 = x1; x1 = xn;
            y2 = y1; y1 = yn;
            p = yn;
        }
    };
    // Forward.
    pass([&](size_t i) -> QVector3D& { return fr[i].pelvisPos; });
    // Reverse (zero-phase).
    std::reverse(fr.begin(), fr.end());
    pass([&](size_t i) -> QVector3D& { return fr[i].pelvisPos; });
    std::reverse(fr.begin(), fr.end());
}

// Pass 4 — ZUPT: zero out pelvis XY drift on frames where both feet
// quaternions barely rotate between consecutive samples.  Catches the
// "skating" artefact caused by IMU integration drift.
static void hdZupt(std::vector<RecordedFrame>& fr)
{
    if (fr.size() < 3) return;
    for (size_t i = 1; i < fr.size(); ++i) {
        const double dR = angBetween(fr[i-1].segQuat[SEG_RFoot],
                                     fr[i].segQuat[SEG_RFoot]);
        const double dL = angBetween(fr[i-1].segQuat[SEG_LFoot],
                                     fr[i].segQuat[SEG_LFoot]);
        if (dR < 0.01 && dL < 0.01) {
            // Both feet essentially still → freeze pelvis translation.
            fr[i].pelvisPos.setX(fr[i-1].pelvisPos.x());
            fr[i].pelvisPos.setY(fr[i-1].pelvisPos.y());
        }
    }
}

static void hdFingerSmooth(std::vector<RecordedFrame>& fr,
                           const std::function<void(double)>& cb = {})
{
    const int N = int(fr.size());
    if (N < 3) return;
    bool any = false;
    for (const auto& f : fr) if (f.hasGloves) { any = true; break; }
    if (!any) return;

    const int half = 5;
    const double sigma = 2.5;
    std::vector<double> k(2*half + 1);
    double sumK = 0.0;
    for (int i = -half; i <= half; ++i) {
        k[i + half] = std::exp(-0.5 * (i*i) / (sigma*sigma));
        sumK += k[i + half];
    }
    for (auto& v : k) v /= sumK;

    auto smoothChain = [&](std::array<Quat, kFingerSegmentsHand> RecordedFrame::*member,
                           double base) {
        std::vector<std::array<Quat, kFingerSegmentsHand>> out(N);
        for (int i = 0; i < N; ++i) {
            if (cb && (i & 0x0FFF) == 0) cb(base + 0.5 * double(i) / double(N));
            for (int j = 0; j < kFingerSegmentsHand; ++j) {
                Quat acc = (fr[i].*member)[j];
                double wAcc = k[half];
                for (int off = 1; off <= half; ++off) {
                    if (i - off >= 0) {
                        const double w = k[half - off];
                        const double t = w / (wAcc + w);
                        acc = slerp_quat(acc, (fr[i - off].*member)[j], t);
                        wAcc += w;
                    }
                    if (i + off < N) {
                        const double w = k[half + off];
                        const double t = w / (wAcc + w);
                        acc = slerp_quat(acc, (fr[i + off].*member)[j], t);
                        wAcc += w;
                    }
                }
                out[i][j] = acc.normalized();
            }
        }
        for (int i = 0; i < N; ++i) (fr[i].*member) = out[i];
    };
    smoothChain(&RecordedFrame::rightGloveQ, 0.0);
    smoothChain(&RecordedFrame::leftGloveQ,  0.5);
}

// Anatomical joint-limit projection for a hinge (knee / elbow).  Clamps the
// child segment's rotation relative to its parent so a mag/jump glitch can't
// fold the joint through itself or spin it about its own long axis.  This is
// the "biomechanical projection" MVN does, done convention-safely:
//   * cap the SWING magnitude (gross over-bend), preserving bend DIRECTION;
//   * cap the long-axis TWIST (unphysical spin) — disabled for the elbow so
//     forearm pronation is preserved (pass maxTwist >= pi).
// World orientation of segment s is raw[s]*defAng[s]; we modify only the
// child's raw quat, leaving the parent untouched.  No-op unless a limit is hit
// (so normal motion is never altered).
// Diagnostic record of the most recent hinge-limit evaluation per child
// segment, captured by projectHingeLimit() so the -test RENDER SNAPSHOT can
// report the REAL joint swing/twist the limiter measured and whether the
// anatomical cap actually fired.  These are the exact values the limiter used
// (single source — not a parallel recompute in the logger).
struct HingeLimitDiag {
    bool   valid       = false;
    double swingDeg    = 0.0;
    double twistDeg    = 0.0;
    double maxSwingDeg = 0.0;
    double maxTwistDeg = 0.0;
    bool   clamped     = false;
};
static std::array<HingeLimitDiag, kXsensSegmentCount> g_hingeDiag{};

static void projectHingeLimit(std::array<Quat, kXsensSegmentCount>& q,
                              int upSeg, int lowSeg, const SkeletonXsens& skel,
                              double maxSwingRad, double maxTwistRad)
{
    const Quat dUp  = skel.defAngFor(upSeg);
    const Quat dLow = skel.defAngFor(lowSeg);
    const Quat Wup  = quat_mult(q[upSeg],  dUp).normalized();
    const Quat Wlow = quat_mult(q[lowSeg], dLow).normalized();
    Quat L = quat_mult(Wup.inv(), Wlow).normalized();   // child-in-parent

    Quat swing, twist;
    swingTwistDecompose(L, QVector3D(1.0f, 0.0f, 0.0f), swing, twist);

    bool changed = false;
    const double sw = std::clamp(std::abs(swing.w), 0.0, 1.0);
    const double swingAng = 2.0 * std::acos(sw);
    if (swingAng > maxSwingRad && swingAng > 1e-6) {
        swing = slerp_quat(Quat(1, 0, 0, 0), swing, maxSwingRad / swingAng);
        changed = true;
    }
    const double twAng = 2.0 * std::atan2(std::abs(double(twist.x)),
                                          std::abs(double(twist.w)));
    if (twAng > maxTwistRad && twAng > 1e-6) {
        twist = slerp_quat(Quat(1, 0, 0, 0), twist, maxTwistRad / twAng);
        changed = true;
    }
    // Record the real measured joint angles + caps for the -test snapshot,
    // every call (so the log shows the live joint angle even when no clamp).
    {
        const double K = 180.0 / M_PI;
        HingeLimitDiag& dg = g_hingeDiag[lowSeg];
        dg.valid       = true;
        dg.swingDeg    = swingAng * K;
        dg.twistDeg    = twAng * K;
        dg.maxSwingDeg = maxSwingRad * K;
        dg.maxTwistDeg = maxTwistRad * K;
        dg.clamped     = changed;
    }
    if (!changed) return;

    L = quat_mult(swing, twist).normalized();           // q = swing * twist
    const Quat WlowNew = quat_mult(Wup, L).normalized();
    q[lowSeg] = quat_mult(WlowNew, dLow.inv()).normalized();
}

// HD pass — clamp both knees and elbows to anatomical ranges.  Caps are
// intentionally loose (only catch fold-through / spin glitches, never valid
// deep flexion), so this strictly removes impossible poses.
static void hdJointLimits(std::vector<RecordedFrame>& fr, const SkeletonXsens& skel,
                          const std::function<void(double)>& cb = {})
{
    const int N = int(fr.size());
    const double kneeSwing  = 175.0 * M_PI / 180.0;
    const double kneeTwist  =  40.0 * M_PI / 180.0;
    const double elbowSwing = 175.0 * M_PI / 180.0;
    const double elbowTwist = M_PI;            // >= pi → no twist clamp (pronation)
    for (int i = 0; i < N; ++i) {
        if (cb && (i & 0x0FFF) == 0) cb(double(i) / double(N));
        projectHingeLimit(fr[i].segQuat, SEG_RUpperLeg, SEG_RLowerLeg, skel, kneeSwing,  kneeTwist);
        projectHingeLimit(fr[i].segQuat, SEG_LUpperLeg, SEG_LLowerLeg, skel, kneeSwing,  kneeTwist);
        projectHingeLimit(fr[i].segQuat, SEG_RUpperArm, SEG_RForearm,  skel, elbowSwing, elbowTwist);
        projectHingeLimit(fr[i].segQuat, SEG_LUpperArm, SEG_LForearm,  skel, elbowSwing, elbowTwist);
    }
}

static void runHdPostProcessing(std::vector<RecordedFrame>& fr,
                                int fps,
                                const SkeletonXsens* skel,
                                std::function<void(int /*percent*/)> progress,
                                const std::function<bool()>& cancelled = {})
{
    auto stop = [&]{ return cancelled && cancelled(); };
    // Map a pass-local 0..1 fraction into a global percentage band so the
    // progress bar advances smoothly *within* long passes, not only between.
    auto band = [&](int lo, int hi) {
        return [progress, lo, hi](double f) {
            if (progress) progress(lo + int(double(hi - lo) * std::clamp(f, 0.0, 1.0)));
        };
    };
    if (progress) progress(0);
    hdOutlierReject(fr, band(0, 15));   if (stop()) return; if (progress) progress(15);
    hdQuatSmooth(fr,   band(15, 40));   if (stop()) return; if (progress) progress(40);
    hdFingerSmooth(fr, band(40, 60));   if (stop()) return; if (progress) progress(60);
    if (skel) hdJointLimits(fr, *skel, band(60, 75));  if (stop()) return;
    if (progress) progress(75);
    hdRootLowpass(fr, fps);             if (progress) progress(90);
    hdZupt(fr);                         if (progress) progress(100);
}

} // anonymous namespace

// ============================================================================
//  JointOffsets — JSON persistence next to the executable
// ============================================================================

QString JointOffsets::filePath()
{
    return QCoreApplication::applicationDirPath() + "/joint_offsets.json";
}

bool JointOffsets::save(const QString& path) const
{
    QJsonObject joints;
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        QJsonObject xyz;
        xyz["x"] = double(deg[i].x());
        xyz["y"] = double(deg[i].y());
        xyz["z"] = double(deg[i].z());
        joints[QString::fromUtf8(kSegmentNames[i])] = xyz;
    }
    QJsonObject root;
    root["version"] = 1;
    root["joints"]  = joints;

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    f.close();
    return true;
}

bool JointOffsets::load(const QString& path)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QByteArray bytes = f.readAll();
    f.close();

    QJsonParseError err{};
    const QJsonDocument doc = QJsonDocument::fromJson(bytes, &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) return false;

    const QJsonObject joints = doc.object().value("joints").toObject();
    std::array<QVector3D, kXsensSegmentCount> tmp{};
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        const QJsonObject xyz =
            joints.value(QString::fromUtf8(kSegmentNames[i])).toObject();
        tmp[i] = QVector3D(float(xyz.value("x").toDouble(0.0)),
                           float(xyz.value("y").toDouble(0.0)),
                           float(xyz.value("z").toDouble(0.0)));
    }
    deg = tmp;
    return true;
}

// ============================================================================
//  JointOffsetsDialog — non-modal X/Y/Z editor
// ============================================================================

namespace {
// Lang key for the human-readable name of each segment (display only; the JSON
// keys stay the canonical kSegmentNames).  Indexed by the Seg enum.
const char* jointDispKey(int seg)
{
    static const char* k[kXsensSegmentCount] = {
        "js_pelvis", "js_l5", "js_l3", "js_t12", "js_t8", "js_neck", "js_head",
        "js_r_shoulder", "js_r_upper_arm", "js_r_forearm", "js_r_hand",
        "js_l_shoulder", "js_l_upper_arm", "js_l_forearm", "js_l_hand",
        "js_r_upper_leg", "js_r_lower_leg", "js_r_foot", "js_r_toe",
        "js_l_upper_leg", "js_l_lower_leg", "js_l_foot", "js_l_toe",
    };
    return (seg >= 0 && seg < kXsensSegmentCount) ? k[seg] : "js_pelvis";
}
} // anonymous namespace

JointOffsetsDialog::JointOffsetsDialog(JointOffsets* offsets, QWidget* parent)
    : QDialog(parent), m_offsets(offsets)
{
    setWindowFlag(Qt::Window, true);
    setModal(false);                       // never block the live viewport
    setWindowTitle(Lang::t("js_title"));
    resize(580, 780);
    buildUi();
    syncControlsFromModel();
}

void JointOffsetsDialog::buildUi()
{
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(14, 14, 14, 12);
    outer->setSpacing(10);

    auto* intro = new QLabel(Lang::t("js_intro"), this);
    intro->setObjectName("subtle");
    intro->setWordWrap(true);
    outer->addWidget(intro);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto* content = new QWidget(scroll);
    auto* cl = new QVBoxLayout(content);
    cl->setContentsMargins(2, 2, 6, 2);
    cl->setSpacing(10);

    const char* axisKey[3] = { "js_axis_x", "js_axis_y", "js_axis_z" };

    auto buildRegion = [&](const char* titleKey, const std::vector<int>& segs) {
        auto* box  = new QGroupBox(Lang::t(titleKey), content);
        auto* grid = new QGridLayout(box);
        grid->setContentsMargins(10, 6, 10, 8);
        grid->setHorizontalSpacing(8);
        grid->setVerticalSpacing(5);
        int row = 0;
        for (int seg : segs) {
            auto* segLbl = new QLabel(Lang::t(jointDispKey(seg)), box);
            segLbl->setStyleSheet("font-weight:700; color:#FFB066; margin-top:4px;");
            grid->addWidget(segLbl, row, 0, 1, 3);
            ++row;
            for (int a = 0; a < 3; ++a) {
                auto* axLbl = new QLabel(Lang::t(axisKey[a]), box);
                axLbl->setObjectName("subtle");
                axLbl->setMinimumWidth(22);

                auto* sld = new QSlider(Qt::Horizontal, box);
                sld->setRange(-180, 180);
                sld->setPageStep(5);

                auto* spin = new QDoubleSpinBox(box);
                spin->setRange(-180.0, 180.0);
                spin->setSingleStep(0.5);
                spin->setDecimals(1);
                spin->setSuffix(QString::fromUtf8("°"));
                spin->setMinimumWidth(86);

                m_ctl[seg][a] = { sld, spin };

                // Two-way slider<->spinbox sync; both write the model so the
                // next render tick picks the correction up live.
                auto applyAxis = [this, seg, a](double v) {
                    QVector3D& vv = m_offsets->deg[seg];
                    if      (a == 0) vv.setX(float(v));
                    else if (a == 1) vv.setY(float(v));
                    else             vv.setZ(float(v));
                };
                connect(sld, &QSlider::valueChanged, this, [this, seg, a, applyAxis](int v) {
                    if (m_syncing) return;
                    m_syncing = true;
                    m_ctl[seg][a].spin->setValue(double(v));
                    m_syncing = false;
                    applyAxis(double(v));
                });
                connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                        [this, seg, a, applyAxis](double v) {
                    if (m_syncing) return;
                    m_syncing = true;
                    m_ctl[seg][a].slider->setValue(int(std::lround(v)));
                    m_syncing = false;
                    applyAxis(v);
                });

                grid->addWidget(axLbl, row, 0);
                grid->addWidget(sld,   row, 1);
                grid->addWidget(spin,  row, 2);
                ++row;
            }
        }
        grid->setColumnStretch(1, 1);
        cl->addWidget(box);
    };

    buildRegion("js_grp_torso", { SEG_Pelvis, SEG_L5, SEG_L3, SEG_T12, SEG_T8, SEG_Neck, SEG_Head });
    buildRegion("js_grp_rarm",  { SEG_RShoulder, SEG_RUpperArm, SEG_RForearm, SEG_RHand });
    buildRegion("js_grp_larm",  { SEG_LShoulder, SEG_LUpperArm, SEG_LForearm, SEG_LHand });
    buildRegion("js_grp_rleg",  { SEG_RUpperLeg, SEG_RLowerLeg, SEG_RFoot, SEG_RToe });
    buildRegion("js_grp_lleg",  { SEG_LUpperLeg, SEG_LLowerLeg, SEG_LFoot, SEG_LToe });
    cl->addStretch();
    scroll->setWidget(content);
    outer->addWidget(scroll, 1);

    auto* btnRow = new QHBoxLayout();
    m_status = new QLabel(QString(), this);
    m_status->setObjectName("subtle");
    btnRow->addWidget(m_status, 1);

    auto* btnReset = new QPushButton(Lang::t("js_reset"), this);
    auto* btnLoad  = new QPushButton(Lang::t("js_load"),  this);
    auto* btnSave  = new QPushButton(Lang::t("js_save"),  this);
    btnSave->setObjectName("primary");
    btnRow->addWidget(btnReset);
    btnRow->addWidget(btnLoad);
    btnRow->addWidget(btnSave);
    outer->addLayout(btnRow);

    connect(btnReset, &QPushButton::clicked, this, [this]() {
        m_offsets->clear();
        syncControlsFromModel();
        if (m_status) m_status->setText(Lang::t("js_reset_done"));
    });
    connect(btnSave, &QPushButton::clicked, this, [this]() {
        const bool ok = m_offsets->save(JointOffsets::filePath());
        if (m_status) m_status->setText(ok ? Lang::t("js_saved") : Lang::t("js_save_err"));
    });
    connect(btnLoad, &QPushButton::clicked, this, [this]() {
        const bool ok = m_offsets->load(JointOffsets::filePath());
        if (ok) syncControlsFromModel();
        if (m_status) m_status->setText(ok ? Lang::t("js_loaded") : Lang::t("js_load_err"));
    });
}

void JointOffsetsDialog::syncControlsFromModel()
{
    m_syncing = true;
    for (int seg = 0; seg < kXsensSegmentCount; ++seg) {
        const QVector3D v = m_offsets->deg[seg];
        const double comp[3] = { double(v.x()), double(v.y()), double(v.z()) };
        for (int a = 0; a < 3; ++a) {
            if (m_ctl[seg][a].slider) m_ctl[seg][a].slider->setValue(int(std::lround(comp[a])));
            if (m_ctl[seg][a].spin)   m_ctl[seg][a].spin->setValue(comp[a]);
        }
    }
    m_syncing = false;
}

// ============================================================================
//  MainWindow
// ============================================================================

MainWindow::MainWindow(MocapReceiver* rx,
                       const NewSessionWizard::Result& wizardResult,
                       bool testMode)
    : m_setup(wizardResult), m_test(testMode), m_rx(rx)
{
    setWindowTitle(Lang::t("app_title"));
    resize(1360, 820);

    // Actor config derived from wizard result.
    ActorConfig actor;
    actor.heightCm        = m_setup.heightCm;
    actor.footLengthCm    = m_setup.footLengthCm;
    actor.armSpanCm       = m_setup.armSpanCm;
    actor.legLengthCm     = m_setup.legLengthCm;
    actor.hipWidthCm      = m_setup.hipWidthCm;
    actor.shoulderWidthCm = m_setup.shoulderWidthCm;
    actor.trunkLengthCm   = m_setup.trunkLengthCm;
    actor.useGloves       = m_setup.useGloves;

    // Whole-system update rate is driven by the chosen suit (Link 240 /
    // Awinda 60).  Bind it to the locomotion solver so its @90 Hz-tuned timings
    // are re-derived for this cadence.
    m_procRateHz = nativeRateHz(m_setup.suit);

    m_viewport = new MocapViewport(actor, m_setup.poseKind, this);
    m_viewport->setProcRate(m_procRateHz);
    m_skel     = std::make_unique<SkeletonXsens>(actor, m_setup.poseKind);
    if (m_test) m_viewport->setLocoVerbose(true);
    logTest("[rate] processing rate = " + std::to_string(int(m_procRateHz)) + " Hz ("
            + (m_setup.suit == SuitType::Link ? "Xsens Link" : "Xsens Awinda") + ")");

    // Left-hand indicators panel.
    m_panel = new SensorIndicatorsPanel(m_setup.useGloves, this);
    // Reset: skeleton snaps to scene origin with feet on floor.
    connect(m_panel, &SensorIndicatorsPanel::resetClicked,
            this, [this]() {
        if (m_viewport) m_viewport->resetSceneOrigin();
        if (statusBar()) statusBar()->showMessage(Lang::t("reset_coords"), 1500);
        logTest("[action] reset-scene-origin");
    });
    // Freeze toggle: one button, pin XY at current or unfreeze.
    connect(m_panel, &SensorIndicatorsPanel::freezeToggled,
            this, [this](bool on) {
        if (m_viewport) m_viewport->setFreezeXY(on);
        if (statusBar()) statusBar()->showMessage(Lang::t(
            on ? "coords_frozen" : "coords_unfrozen"), 1500);
        logTest(std::string("[action] freeze=") + (on ? "on" : "off"));
    });

    {
        ActorConfig defaults;
        defaults.heightCm        = m_setup.heightCm;
        defaults.footLengthCm    = m_setup.footLengthCm;
        defaults.armSpanCm       = m_setup.armSpanCm;
        defaults.legLengthCm     = m_setup.legLengthCm;
        defaults.hipWidthCm      = m_setup.hipWidthCm;
        defaults.shoulderWidthCm = m_setup.shoulderWidthCm;
        defaults.trunkLengthCm   = m_setup.trunkLengthCm;
        defaults.useGloves       = m_setup.useGloves;
        m_panel->setActorDefaults(defaults);
    }
    connect(m_panel, &SensorIndicatorsPanel::actorChanged,
            this, [this](ActorConfig a) {
        m_setup.heightCm        = a.heightCm;
        m_setup.footLengthCm    = a.footLengthCm;
        m_setup.armSpanCm       = a.armSpanCm;
        m_setup.legLengthCm     = a.legLengthCm;
        m_setup.hipWidthCm      = a.hipWidthCm;
        m_setup.shoulderWidthCm = a.shoulderWidthCm;
        m_setup.trunkLengthCm   = a.trunkLengthCm;
        a.useGloves             = m_setup.useGloves;
        if (m_viewport) m_viewport->setActor(a);
        if (m_skel) m_skel = std::make_unique<SkeletonXsens>(a, m_setup.poseKind);
        std::ostringstream ss;
        ss << "[actor] h=" << a.heightCm << " foot=" << a.footLengthCm
           << " arm=" << a.armSpanCm << " leg=" << a.legLengthCm
           << " hip=" << a.hipWidthCm << " shld=" << a.shoulderWidthCm
           << " trunk=" << a.trunkLengthCm;
        logTest(ss.str());
    });

    // ---- Top tabs: Live vs. Record ---------------------------------------
    // Each pill-button pops up a small QMenu on click.  The central
    // content is always the live scene — actions from the menu open
    // wizard dialogs instead of switching the whole viewport.
    m_streamer = new LiveStreamSender(this);
    if (m_setup.tposeCaptured) {
        // FIX (gloves polish): передаём T-pose hand-vs-forearm rotation
        // в viewport как pinned anchor для wrist drift-correction.
        // tposeReference[i] это сырая сенсор-ориентация во время T-позы
        // (актёр стоит ладонями вниз); из неё computes anchor =
        // (forearm_world).inv() * hand_world в покое.  Без вызова —
        // anchor продолжает обновляться по lock-моментам как раньше.
        if (m_viewport) {
            m_viewport->setTposeHandAnchor(
                m_setup.tposeReference[SEG_RForearm],
                m_setup.tposeReference[SEG_RHand],
                m_setup.tposeReference[SEG_LForearm],
                m_setup.tposeReference[SEG_LHand]);
            // FIX (T-pose foot direction reference): pin foot-yaw anchor.
            // В T-pose стопы смотрят вперёд по +X в pelvis-yaw-frame;
            // anchor берётся из 500-кадрового averaging T-позы.  После
            // этого foot-yaw drift correction использует pelvis-yaw как
            // reference (не lowerLeg, который меняется при flex'е колена).
            m_viewport->setTposeFootAnchor(
                m_setup.tposeReference[SEG_Pelvis],
                m_setup.tposeReference[SEG_RFoot],
                m_setup.tposeReference[SEG_LFoot]);
        }

        // FIX (gloves polish): установить per-actor finger baseline,
        // захваченный в T-pose (расслабленные пальцы ладонями вниз).
        // parseErgoHand читает g_fingerBaseline и вычитает baseline
        // из raw degrees — finger "ноль" теперь анатомически правильный.
        if (m_setup.fingerBaselineCaptured) {
            QMutexLocker lkBL(&g_fingerBaseline.lock);
            for (int i = 0; i < 20; ++i) {
                g_fingerBaseline.left[i]  = m_setup.fingerBaselineL[i];
                g_fingerBaseline.right[i] = m_setup.fingerBaselineR[i];
            }
            g_fingerBaseline.valid.store(true);
        }

        // NOTE: the wire no longer pre-subtracts a T-pose baseline.  We stream
        // the absolute calibrated world pose (qOut) and let each plugin do its
        // own neutral referencing — see mvnWireOrient().  The old
        // setTposeBaseline(tposeReference*calibReference^-1) reconstruction did
        // not match the runtime raw*calibReference^-1 pipeline and flipped the
        // skeleton ~165°, so it has been removed.
    }

    if (m_test) {
        LiveSettings cfg;
        cfg.useGloves = m_setup.useGloves;
        // FIX (stream polish): в -test режиме всегда target=BlenderMVN
        // (соответствует требованию задачи "если -test → стримим в Blender"),
        // и включаем одноразовый hex-dump первого фрейма для byte-уровня
        // verification.
        cfg.target              = LiveTarget::BlenderMVN;
        cfg.debugDumpFirstFrame = true;
        cfg.verboseLog          = true;   // -test: emit periodic [STREAM SNAPSHOT]
        if (m_skel) {
            std::array<Quat, kXsensSegmentCount> identity{};
            for (auto& qq : identity) qq = Quat(1, 0, 0, 0);
            const float pelvisZ = float(m_setup.heightCm * 0.55 / 100.0);
            auto kp = m_skel->computeKeypoints(identity, QVector3D(0.0f, 0.0f, pelvisZ));
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                cfg.tposeOriginM[i] = kp[i];  // NWU == MVN Z-up RH; no conversion.
                cfg.defAngT[i] = m_skel->defAngFor(i);
            }
        }
        QString err;
        if (m_streamer->start(cfg, &err)) {
            logTest(std::string("[test] auto-start stream -> ")
                    + cfg.host.toStdString() + ":" + std::to_string(cfg.port)
                    + " (gloves=" + (cfg.useGloves ? "yes" : "no") + ")");
        } else {
            logTest(std::string("[test] auto-start stream FAILED: ")
                    + err.toStdString());
        }
    }

    auto makeTab = [this](const char* labelKey) {
        auto* btn = new QToolButton(this);
        btn->setObjectName("topTabBtn");
        btn->setProperty("tabKey", labelKey);
        btn->setText(Lang::t(labelKey));
        btn->setCursor(Qt::PointingHandCursor);
        btn->setPopupMode(QToolButton::InstantPopup);
        btn->setToolButtonStyle(Qt::ToolButtonTextOnly);
        auto* menu = new QMenu(btn);
        menu->setObjectName("topTabMenu");
        btn->setMenu(menu);
        return btn;
    };

    auto* liveBtn   = makeTab("tab_live");
    auto* recordBtn = makeTab("tab_record");

    // Settings tab — same pill styling, but a plain click opens the non-modal
    // joint-orientation editor instead of dropping a menu.
    auto* settingsBtn = new QToolButton(this);
    settingsBtn->setObjectName("topTabBtn");
    settingsBtn->setProperty("tabKey", "tab_settings");
    settingsBtn->setText(Lang::t("tab_settings"));
    settingsBtn->setCursor(Qt::PointingHandCursor);
    settingsBtn->setToolButtonStyle(Qt::ToolButtonTextOnly);
    connect(settingsBtn, &QToolButton::clicked, this, &MainWindow::onOpenJointSettings);

    auto refreshLiveMenu = [this, liveBtn]() {
        auto* m = liveBtn->menu();
        m->clear();
        if (!m_streamer || !m_streamer->isRunning()) {
            auto* a = m->addAction(Lang::t("live_start"));
            connect(a, &QAction::triggered, this, &MainWindow::onOpenLiveWizard);
        } else {
            auto* aRecal = m->addAction(QString::fromUtf8("Recalibrate Stream (T-pose)"));
            aRecal->setShortcut(QKeySequence("Ctrl+R"));
            connect(aRecal, &QAction::triggered, this, [this]() {
                if (m_streamer) {
                    m_streamer->recalibrate();
                    if (statusBar()) statusBar()->showMessage(
                            QString::fromUtf8("Stream baseline re-captured (стой в T-pose)"), 2000);
                }
            });
            auto* a = m->addAction(Lang::t("live_stop"));
            connect(a, &QAction::triggered, this, [this, liveBtn]() {
                m_streamer->stop();
                if (statusBar()) statusBar()->showMessage(Lang::t("live_stopped"), 1500);
                QMetaObject::invokeMethod(liveBtn, [liveBtn]() {
                    liveBtn->menu()->close();
                }, Qt::QueuedConnection);
            });
        }
    };
    auto refreshRecordMenu = [this, recordBtn]() {
        auto* m = recordBtn->menu();
        m->clear();
        if (!m_recording) {
            auto* a = m->addAction(Lang::t("rec_start"));
            connect(a, &QAction::triggered, this, &MainWindow::onOpenRecordWizard);
        } else {
            auto* a = m->addAction(Lang::t("rec_stop"));
            connect(a, &QAction::triggered, this, &MainWindow::onRecordStop);
        }
    };
    refreshLiveMenu();
    refreshRecordMenu();
    connect(liveBtn->menu(),   &QMenu::aboutToShow, this, refreshLiveMenu);
    connect(recordBtn->menu(), &QMenu::aboutToShow, this, refreshRecordMenu);

    connect(&Lang::instance(), &Lang::changed, this, [liveBtn, recordBtn, settingsBtn]() {
        for (auto* b : { liveBtn, recordBtn, settingsBtn }) {
            const QString tk = b->property("tabKey").toString();
            if (!tk.isEmpty()) b->setText(Lang::t(tk.toUtf8().constData()));
        }
    });

    // Live scene — panel on the left, 3-D viewport filling the rest.
    auto* liveWidget = new QWidget(this);
    auto* cl         = new QHBoxLayout(liveWidget);
    cl->setContentsMargins(4, 4, 4, 4);
    cl->addWidget(m_panel, 0);
    cl->addWidget(m_viewport, 1);

    auto* central = new QWidget(this);
    auto* vl      = new QVBoxLayout(central);
    vl->setContentsMargins(0, 0, 0, 0);
    vl->setSpacing(0);
    auto* tabRow = new QHBoxLayout();
    tabRow->setContentsMargins(10, 6, 10, 4);
    tabRow->setSpacing(6);
    tabRow->addWidget(liveBtn);
    tabRow->addWidget(recordBtn);
    tabRow->addWidget(settingsBtn);
    tabRow->addStretch();
    vl->addLayout(tabRow);
    vl->addWidget(liveWidget, 1);
    setCentralWidget(central);

    setStatusBar(new QStatusBar(this));
    statusBar()->showMessage(Lang::t("session_running"));

    // Receiver signal wiring — the thread is already running (started before
    // the wizard), so we just hook into its signals here.
    connect(m_rx, &MocapReceiver::statusChanged,      this, &MainWindow::onConnStatusChanged);
    connect(m_rx, &MocapReceiver::gloveStatusChanged, this, &MainWindow::onGloveStatus);
    connect(m_rx, &MocapReceiver::fpsUpdated,         this, &MainWindow::onFps);

    // Joint-correction preset.  If one was saved next to the executable it
    // becomes the default for this session; otherwise the default is the
    // current state (all zeros) and we write that file so the operator has a
    // starting point to hand-tune from.
    {
        const QString jp = JointOffsets::filePath();
        if (QFile::exists(jp)) m_jointOffsets.load(jp);
        else                   m_jointOffsets.save(jp);
    }

    // Tick the solve / record / stream loop at the suit's native rate.  A
    // precise timer is requested so 240 Hz (Link) is honoured as closely as the
    // platform allows; GL repaint is throttled inside onRenderTick.
    m_renderTimer.setTimerType(Qt::PreciseTimer);
    m_renderTimer.setInterval(int(1000.0 / m_procRateHz));
    connect(&m_renderTimer, &QTimer::timeout, this, &MainWindow::onRenderTick);
    m_renderTimer.start();

    // Initial status badge reflects whatever state the receiver is in now.
    onConnStatusChanged(int(m_rx->status()), m_rx->statusDetail());
}

MainWindow::~MainWindow() = default;

void MainWindow::setWristConstraintEnabled(bool enabled)
{
    if (!m_viewport) return;
    WristAnatomicalCfg r = m_viewport->wristCfg(true);
    WristAnatomicalCfg l = m_viewport->wristCfg(false);
    r.enabled = enabled;
    l.enabled = enabled;
    m_viewport->setWristCfg(true,  r);
    m_viewport->setWristCfg(false, l);
}

void MainWindow::onConnStatusChanged(int status, const QString& /*detail*/)
{
    const ConnStatus s = (ConnStatus)status;
    const bool streaming = (s == ConnStatus::Streaming);
    m_panel->setSuitLive(streaming, {});
    // Pause the solve/render loop while the suit is down, and — crucially —
    // resume it when the suit comes back. Without the resume the session stayed
    // paused forever after the first blip and onRenderTick() never ran again.
    if (!streaming && m_sessionRunning)      onPauseSession();
    else if (streaming && !m_sessionRunning) onResumeSession();
    logTest(std::string("[suit] ") + connStatusName(s));
}

void MainWindow::onGloveStatus(bool /*up*/)
{
    // Finger activity is updated per-frame in updateFromPose().
}

void MainWindow::onFps(double hz)
{
    if (m_panel) m_panel->setFps(hz);
}

// -test render-pipeline diagnostics (main thread only).  Every field is filled
// INSIDE the real transform in onRenderTick() — the [RENDER]/[pulse]/[evt:*]
// logging only reads it, so the log can never diverge from the live math.
namespace {
struct RenderDiag {
    // §2 calibration offset  cand[i] = raw[i] * refWorldInv[i]; jump-reject.
    std::array<double, kXsensSegmentCount> jumpDeg{};   // |cand vs lastOut|, deg
    std::array<double, kXsensSegmentCount> rejectW{};   // 0..1 smoothstep reject
    std::array<bool,   kXsensSegmentCount> gyroQuiet{};
    std::array<double, kXsensSegmentCount> localAng{};  // angle vs parent, deg
    // §3 spine/neck smoothstep blend weights (constant, captured at source).
    double spineW_L5 = 0.0, spineW_L3 = 0.0, spineW_T12 = 0.0, neckW = 0.5;
    // §3 scapular-humeral coupling (per shoulder).
    double scapUpZR = 0.0, scapAngR = 0.0, scapUpZL = 0.0, scapAngL = 0.0;
    bool   scapActiveR = false, scapActiveL = false;
    // §3 wrist-forearm coupling (per wrist): forearm twist follows hand twist.
    double wTwistHalfR = 0.0, faTwistAddR = 0.0;
    double wTwistHalfL = 0.0, faTwistAddL = 0.0;
};
RenderDiag g_renderDiag{};

// Shared compact formatters so every -test render line reads identically and
// is grep/column friendly.
inline std::string fmtQ4(const Quat& q) {
    char b[96];
    std::snprintf(b, sizeof(b), "(% .4f,% .4f,% .4f,% .4f)", q.w, q.x, q.y, q.z);
    return std::string(b);
}
inline std::string fmtV3(const QVector3D& v) {
    char b[80];
    std::snprintf(b, sizeof(b), "(% .4f,% .4f,% .4f)", v.x(), v.y(), v.z());
    return std::string(b);
}

// World pelvis position (NWU metres): forward-kinematics with the pelvis at the
// origin, plus the locomotion travel offset, then floor-clamped so the lowest
// keypoint rests on z=0. Single source shared by the live-stream and recording
// paths so their root motion can never silently diverge. View-only framing
// (sceneYaw / sceneShift / freeze) is intentionally NOT applied here — it must
// not leak onto the wire / into the recording.
QVector3D worldPelvisWithLoco(const SkeletonXsens& skel,
                              const std::array<Quat, kXsensSegmentCount>& segWorld,
                              const QVector3D& locoOffset)
{
    auto kp = skel.computeKeypoints(segWorld, QVector3D(0.0f, 0.0f, 0.0f));
    for (auto& p : kp) p += locoOffset;
    float minZ = kp[0].z();
    for (const auto& p : kp) if (p.z() < minZ) minZ = p.z();
    if (minZ < -0.02f) {              // lift so the lowest keypoint sits on the floor
        const QVector3D up(0.0f, 0.0f, -minZ);
        for (auto& p : kp) p += up;
    }
    return kp[SEG_Pelvis];
}
} // namespace

void MainWindow::onRenderTick()
{
    const SuitPose f = m_rx->snapshot();
    if (f.recvTime == 0.0) return;

    if (m_panel) m_panel->updateFromPose(f);
    if (!m_sessionRunning) return;

    static const MainWindow* s_owner = nullptr;
    static bool s_stateReady = false;

    static std::array<Quat, kXsensSegmentCount> s_refWorld{};
    static std::array<Quat, kXsensSegmentCount> s_refWorldInv{};
    static std::array<Quat, kXsensSegmentCount> s_lastRaw{};
    static std::array<Quat, kXsensSegmentCount> s_prevRaw{};
    static std::array<Quat, kXsensSegmentCount> s_lastOut{};
    static std::array<bool, kXsensSegmentCount> s_haveRaw{};
    static std::array<bool, kXsensSegmentCount> s_haveOut{};
    static std::array<double, kXsensSegmentCount> s_worldOmegaLP{};
    static std::array<int,    kXsensSegmentCount> s_stillCount{};
    static double s_lastT = 0.0;

    static constexpr bool kTracked[kXsensSegmentCount] = {
        true,  false, false, false, true,  false, true,   // pelvis..head
        true,  true,  true,  true,                         // r arm
        true,  true,  true,  true,                         // l arm
        true,  true,  true,  false,                        // r leg
        true,  true,  true,  false                         // l leg
    };

    if (s_owner != this) { s_owner = this; s_stateReady = false; }

    auto safeQuat = [](const Quat& in, const Quat& fallback) -> Quat {
        const double n2 = in.w*in.w + in.x*in.x + in.y*in.y + in.z*in.z;
        if (!std::isfinite(n2) || n2 < 1e-10) return fallback;
        if (std::abs(n2 - 1.0) > 1e-6)        return in.normalized();
        return in;
    };

    if (!s_stateReady) {
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            s_refWorld[i]     = safeQuat(m_setup.calibReference[i], Quat(1,0,0,0));
            s_refWorldInv[i]  = s_refWorld[i].inv();
            s_lastRaw[i]      = s_refWorld[i];
            s_prevRaw[i]      = s_refWorld[i];
            s_lastOut[i]      = Quat(1, 0, 0, 0);
            s_haveRaw[i]      = !kTracked[i];
            s_haveOut[i]      = false;
            s_worldOmegaLP[i] = 0.0;
            s_stillCount[i]   = 0;
        }
        s_lastT = 0.0;
        s_stateReady = true;
    }

    const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const double dt = (s_lastT > 0.0)
        ? std::max(1e-3, std::min(0.1, now - s_lastT))
        : (1.0 / m_procRateHz);
    s_lastT = now;

    // --- Собираем raw + считаем угловую скорость и stillness. ---
    std::array<Quat, kXsensSegmentCount> raw{};
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        if (kTracked[i] && f.segValid[i]) {
            raw[i] = safeQuat(f.quat[i], s_lastRaw[i]);
            s_lastRaw[i] = raw[i];
            s_haveRaw[i] = true;
        } else if (s_haveRaw[i]) {
            raw[i] = s_lastRaw[i];
        } else {
            raw[i] = s_refWorld[i];
        }
        const double rawOmega = quatAngVel(raw[i], s_prevRaw[i], dt);
        const double aOmega = rateAdjustAlpha(0.20, dt);
        s_worldOmegaLP[i] = (1.0 - aOmega) * s_worldOmegaLP[i] + aOmega * rawOmega;
        s_stillCount[i]   = (s_worldOmegaLP[i] < 0.10)
                            ? std::min(s_stillCount[i] + 1, 8192)
                            : 0;
        s_prevRaw[i] = raw[i];
    }

    // ---------------------------------------------------------------
    //  ЯДРО ИСПРАВЛЕНИЯ:  cand[i] = raw[i] * refRaw[i].inv()
    //
    //  FK ожидает, что cand[i] * defAng_N[i] = реальная world-ориентация
    //  body-frame сегмента i.  Поскольку в N-позе raw[i] == refWorld[i] (и
    //  refWorld[i] ≈ defAng_N[i] после хорошей калибровки), формула
    //  cand = raw * refInv даёт ровно identity в калибровочной позе и
    //  мировую дельту `R_world_i` в любой другой позе.
    //
    //  Старая pelvis-relative форма
    //      cand = pelvisDelta * (pelvisInv*raw) * (refPelvisInv*refRaw).inv()
    //  алгебраически сворачивается в
    //      cand = refPelvisInv * R_world * refPelvis,
    //  т.е. world-дельту, СОПРЯЖЁННУЮ через defAng_N[pelvis] = Rot_Y(-π/2).
    //  Это даёт 0° только для вращений ВОКРУГ Y — а X-/Z-компоненты уезжают
    //  до ±90°.  Юнит-тест на "руки вперёд" → 82.82° ошибка в старом
    //  коде, 0.00° в новом.
    // ---------------------------------------------------------------
    std::array<Quat, kXsensSegmentCount> q{};
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        if (!kTracked[i]) continue;

        g_renderDiag.jumpDeg[i]   = 0.0;   // -test §2: reset per-frame
        g_renderDiag.rejectW[i]   = 0.0;
        g_renderDiag.gyroQuiet[i] = false;

        Quat cand = quat_mult(raw[i], s_refWorldInv[i]).normalized();

        if (f.segValid[i] && s_haveOut[i]) {
            const double jumpDeg = quat_angle_deg(quat_mult(cand, s_lastOut[i].inv()));
            const bool gyroQuiet =
                (f.gyrSensor[SEG_Pelvis].lengthSquared() < (25.0f * 25.0f)) &&
                (f.gyrSensor[i].lengthSquared()          < (25.0f * 25.0f));
            g_renderDiag.jumpDeg[i]   = jumpDeg;   // -test §2 capture
            g_renderDiag.gyroQuiet[i] = gyroQuiet;
            // FIX (terminator smoothing): smoothstep blend [20..35]°.
            // Раньше: hard cliff на 35° — 34.9° принимается полностью,
            // 35.1° отвергается полностью.  Теперь: rejectW=0 при <20°,
            // rejectW=1 при >35°, плавный slerp_quat в середине.  Соблюдаем
            // gyroQuiet gate — IMU должен быть тихим, иначе доверяем.
            if (gyroQuiet && jumpDeg > 20.0) {
                auto smoothstep01 = [](double x){ x = std::clamp(x,0.0,1.0); return x*x*(3.0-2.0*x); };
                const double rejectW = smoothstep01((jumpDeg - 20.0) / 15.0);
                g_renderDiag.rejectW[i] = rejectW;   // -test §2 capture
                if (rejectW > 0.999) {
                    if (m_test) {
                        std::cout << "[fk-jump] seg[" << i << "] " << kSegmentNames[i]
                                  << " jump=" << std::fixed << std::setprecision(1) << jumpDeg
                                  << "° full-reject (gyrQuiet, pelvisGyr="
                                  << std::sqrt(double(f.gyrSensor[SEG_Pelvis].lengthSquared()))
                                  << " segGyr=" << std::sqrt(double(f.gyrSensor[i].lengthSquared()))
                                  << ")\n";
                    }
                    cand = s_lastOut[i];
                } else if (rejectW > 0.0) {
                    cand = slerp_quat(cand, s_lastOut[i], rejectW);
                    if (m_test) {
                        static std::array<int, kXsensSegmentCount> s_lastLogTick{};
                        static int s_globalTick = 0;
                        s_globalTick++;
                        if (s_globalTick - s_lastLogTick[i] > 60) {
                            s_lastLogTick[i] = s_globalTick;
                            std::cout << "[fk-jump] seg[" << i << "] " << kSegmentNames[i]
                                      << " jump=" << std::fixed << std::setprecision(1) << jumpDeg
                                      << "° blend rejectW=" << std::setprecision(2) << rejectW
                                      << " (segGyr=" << std::sqrt(double(f.gyrSensor[i].lengthSquared()))
                                      << ")\n";
                        }
                    }
                }
            }
        }
        q[i]         = cand;
        s_lastOut[i] = cand;
        s_haveOut[i] = true;
    }

    if (m_test) {
        static int s_symTick = 0;
        if ((s_symTick++ % 60) == 0) {
            auto qDiff = [](const Quat& a, const Quat& b) {
                const double d  = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
                const double s  = d < 0 ? -1.0 : 1.0;
                const double dw = a.w - s*b.w, dx = a.x - s*b.x;
                const double dy = a.y - s*b.y, dz = a.z - s*b.z;
                return std::sqrt(dw*dw + dx*dx + dy*dy + dz*dz);
            };
            auto fmtQ = [](const Quat& qq) {
                char buf[80];
                std::snprintf(buf, sizeof(buf), "(% .3f,% .3f,% .3f,% .3f)",
                              qq.w, qq.x, qq.y, qq.z);
                return std::string(buf);
            };
            struct Pair { int r, l; const char* name; };
            const Pair pairs[] = {
                { SEG_RUpperArm, SEG_LUpperArm, "UA" },
                { SEG_RForearm,  SEG_LForearm,  "FA" },
                { SEG_RHand,     SEG_LHand,     "HD" },
                { SEG_RUpperLeg, SEG_LUpperLeg, "UL" },
                { SEG_RLowerLeg, SEG_LLowerLeg, "LL" },
                { SEG_RFoot,     SEG_LFoot,     "FT" },
            };
            auto fmtV = [](const QVector3D& v) {
                char buf[80];
                std::snprintf(buf, sizeof(buf), "(% .3f,% .3f,% .3f)|%.3f",
                              v.x(), v.y(), v.z(), v.length());
                return std::string(buf);
            };
            std::cout << "[sym tick=" << s_symTick << "]\n";
            for (const Pair& pp : pairs) {
                const Quat raw_R = raw[pp.r], raw_L = raw[pp.l];
                const Quat candR = q[pp.r],   candL = q[pp.l];
                const double rawD  = qDiff(raw_R, mirror_y_quat(raw_L));
                const double candD = qDiff(candR, mirror_y_quat(candL));
                const Quat refR = s_refWorld[pp.r], refL = s_refWorld[pp.l];
                const double refD = qDiff(refR, mirror_y_quat(refL));
                const QVector3D aR = f.accSensor[pp.r], aL = f.accSensor[pp.l];
                const QVector3D gR = f.gyrSensor[pp.r], gL = f.gyrSensor[pp.l];
                const QVector3D mR = f.magSensor[pp.r], mL = f.magSensor[pp.l];
                std::cout << "  " << pp.name
                          << " rawD="  << std::fixed << std::setprecision(3) << rawD
                          << " candD=" << candD
                          << " refD="  << refD
                          << "\n    raw_R=" << fmtQ(raw_R)
                          << " raw_L="      << fmtQ(raw_L)
                          << " mirror_y_quat(raw_L)="  << fmtQ(mirror_y_quat(raw_L))
                          << "\n    accR=" << fmtV(aR) << " accL=" << fmtV(aL)
                          << "\n    gyrR=" << fmtV(gR) << " gyrL=" << fmtV(gL)
                          << "\n    magR=" << fmtV(mR) << " magL=" << fmtV(mL)
                          << "\n";
            }
            std::cout.flush();
        }
    }

    // --- Спина и шея: smoothstep-распределение между тазом и T8.
    //     Мат: weight(t) = t² · (3 - 2t), C¹ на границах → нет kink
    //     при сильных наклонах; L5 "лежит" на тазу, T12 "лежит" на T8
    //     сильнее, чем при линейном slerp — соответствует реальному
    //     coupling позвоночных сегментов. ---
    auto smoothstep = [](double x) { return x * x * (3.0 - 2.0 * x); };
    q[SEG_L5]   = slerp_quat(q[SEG_Pelvis], q[SEG_T8],   smoothstep(0.22));
    q[SEG_L3]   = slerp_quat(q[SEG_Pelvis], q[SEG_T8],   smoothstep(0.50));
    q[SEG_T12]  = slerp_quat(q[SEG_Pelvis], q[SEG_T8],   smoothstep(0.78));
    q[SEG_Neck] = slerp_quat(q[SEG_T8],     q[SEG_Head], 0.50);
    // Toe (MTP joint) — no toe sensor, so estimate ball-of-foot flexion from
    // foot pitch so the toe hinges during push-off instead of staying a rigid
    // extension of the foot.  ext==0 when the foot is flat/heel-down → identical
    // to the old q[toe]=q[foot] copy, so standing & locomotion are unchanged.
    q[SEG_RToe] = estimateToeOrientation(q[SEG_RFoot]);
    q[SEG_LToe] = estimateToeOrientation(q[SEG_LFoot]);
    g_renderDiag.spineW_L5  = smoothstep(0.22);   // -test §3 capture (single source)
    g_renderDiag.spineW_L3  = smoothstep(0.50);
    g_renderDiag.spineW_T12 = smoothstep(0.78);
    g_renderDiag.neckW      = 0.50;

    if (m_skel) {
        const Quat dA_pel = m_skel->defAngFor(SEG_Pelvis);
        const Quat qPelvisWorld = quat_mult(q[SEG_Pelvis], dA_pel).normalized();
        const Quat qPelvisWorldInv = qPelvisWorld.inv();
        auto coupleScapHumeral = [&](int iSh, int iUA, bool isRight) {
            const Quat dA_sh = m_skel->defAngFor(iSh);
            const Quat dA_ua = m_skel->defAngFor(iUA);
            const Quat shWorld = quat_mult(q[iSh], dA_sh).normalized();
            const Quat uaWorld = quat_mult(q[iUA], dA_ua).normalized();
            const QVector3D armDir = vec_rotate(QVector3D(1.0f, 0.0f, 0.0f), uaWorld);
            const double upZ = std::clamp(double(armDir.z()), -1.0, 1.0);
            const double activate = 0.30;
            // -test §3 capture: arm-elevation gate + applied scapular angle.
            if (isRight) { g_renderDiag.scapUpZR = upZ; g_renderDiag.scapActiveR = (upZ >= activate); }
            else         { g_renderDiag.scapUpZL = upZ; g_renderDiag.scapActiveL = (upZ >= activate); }
            if (upZ < activate) {
                if (isRight) g_renderDiag.scapAngR = 0.0; else g_renderDiag.scapAngL = 0.0;
                return;
            }
            const double normalised = (upZ - activate) / (1.0 - activate);
            const double scapAng = normalised * 0.30;
            const double signedAng = isRight ? -scapAng : scapAng;
            if (isRight) g_renderDiag.scapAngR = signedAng; else g_renderDiag.scapAngL = signedAng;
            // Scapular shrug must rotate around the BODY's forward axis,
            // not world X.  When the actor yaws the body 90°, world X is
            // no longer the body's forward axis, so the old code applied
            // the boost around the wrong direction → asymmetric tilt
            // visible during turning.  Convert the body-frame rotation
            // to world frame via similarity through pelvis world.
            const Quat scapBoostBody = axisAngleQuat(QVector3D(1.0, 0.0, 0.0), signedAng);
            const Quat scapBoostWorld = quat_mult(quat_mult(qPelvisWorld, scapBoostBody),
                                                  qPelvisWorldInv).normalized();
            const Quat shWorldNew = quat_mult(scapBoostWorld, shWorld).normalized();
            q[iSh] = quat_mult(shWorldNew, dA_sh.inv()).normalized();
        };
        coupleScapHumeral(SEG_RShoulder, SEG_RUpperArm, true);
        coupleScapHumeral(SEG_LShoulder, SEG_LUpperArm, false);

        auto coupleWristForearm = [&](int iH, int iFA) {
            const Quat dA_h = m_skel->defAngFor(iH);
            const Quat dA_f = m_skel->defAngFor(iFA);
            const Quat hWorld = quat_mult(q[iH],  dA_h).normalized();
            const Quat fWorld = quat_mult(q[iFA], dA_f).normalized();
            const Quat localHand = quat_mult(fWorld.inv(), hWorld).normalized();
            Quat hSwing, hTwist;
            swingTwistDecompose(localHand, QVector3D(1.0f, 0.0f, 0.0f), hSwing, hTwist);
            const double tx = std::clamp(double(hTwist.x), -1.0, 1.0);
            const double tw = std::clamp(double(hTwist.w),  0.0, 1.0);
            const double twistHalf = std::atan2(tx, tw);
            const double couplingFraction = 0.20;
            const double faAdditionalTwist = twistHalf * 2.0 * couplingFraction;
            // -test §3 capture: hand long-axis twist that the forearm follows.
            if (iH == SEG_RHand) { g_renderDiag.wTwistHalfR = twistHalf; g_renderDiag.faTwistAddR = faAdditionalTwist; }
            else                 { g_renderDiag.wTwistHalfL = twistHalf; g_renderDiag.faTwistAddL = faAdditionalTwist; }
            const Quat faTwistAdd = axisAngleQuat(QVector3D(1.0, 0.0, 0.0), faAdditionalTwist);
            const Quat fWorldNew = quat_mult(fWorld, faTwistAdd).normalized();
            q[iFA] = quat_mult(fWorldNew, dA_f.inv()).normalized();
            const Quat faTwistInv = faTwistAdd.inv();
            const Quat localHandRebal = quat_mult(faTwistInv, localHand).normalized();
            const Quat hWorldNew = quat_mult(fWorldNew, localHandRebal).normalized();
            q[iH] = quat_mult(hWorldNew, dA_h.inv()).normalized();
        };
        coupleWristForearm(SEG_RHand, SEG_RForearm);
        coupleWristForearm(SEG_LHand, SEG_LForearm);
    }

    s_lastOut[SEG_L5]   = q[SEG_L5];
    s_lastOut[SEG_L3]   = q[SEG_L3];
    s_lastOut[SEG_T12]  = q[SEG_T12];
    s_lastOut[SEG_Neck] = q[SEG_Neck];
    s_lastOut[SEG_RToe] = q[SEG_RToe];
    s_lastOut[SEG_LToe] = q[SEG_LToe];
    s_haveOut[SEG_L5] = s_haveOut[SEG_L3] = s_haveOut[SEG_T12] =
    s_haveOut[SEG_Neck] = s_haveOut[SEG_RToe] = s_haveOut[SEG_LToe] = true;

    // Anatomical joint-limit safety net — always on, no flag.  Caps gross
    // knee/elbow fold-through and unphysical long-axis spin so a mag/jump
    // glitch can't push the live pose somewhere impossible.  Thresholds are
    // deliberately loose: normal motion (deep flexion, forearm pronation) is
    // never touched — only broken poses get pulled back.  Same convention-safe
    // clamp the HD pass uses, so live and recorded output stay consistent.
    if (m_skel) {
        const double kneeSwing  = 178.0 * M_PI / 180.0;
        const double kneeTwist  =  45.0 * M_PI / 180.0;
        const double elbowSwing = 178.0 * M_PI / 180.0;
        projectHingeLimit(q, SEG_RUpperLeg, SEG_RLowerLeg, *m_skel, kneeSwing,  kneeTwist);
        projectHingeLimit(q, SEG_LUpperLeg, SEG_LLowerLeg, *m_skel, kneeSwing,  kneeTwist);
        projectHingeLimit(q, SEG_RUpperArm, SEG_RForearm,  *m_skel, elbowSwing, M_PI);
        projectHingeLimit(q, SEG_LUpperArm, SEG_LForearm,  *m_skel, elbowSwing, M_PI);
    }

    // --- Manual per-joint orientation correction (Settings window) -------
    // Final operator override, applied AFTER the anatomical/joint-limit pass
    // so a deliberate fix isn't clamped back.  Composed in the segment's own
    // body frame (local post-multiply) for every segment; because qOut below
    // is derived from this same q[], the correction reaches the viewport, the
    // UDP stream and the recording identically.
    if (!m_jointOffsets.isZero()) {
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            const QVector3D d = m_jointOffsets.deg[i];
            if (d.x() == 0.0f && d.y() == 0.0f && d.z() == 0.0f) continue;
            const Quat off = euler_to_quat(d.x() * M_PI / 180.0,
                                           d.y() * M_PI / 180.0,
                                           d.z() * M_PI / 180.0, "XYZ");
            q[i] = quat_mult(q[i], off).normalized();
        }
    }

    m_viewport->updatePose(q, QVector3D(0.0f, 0.0f, 0.0f));
    const auto& qOut = m_viewport->filteredOrient();

    if (m_skel) {
        const float pelvisZ_loco = float(m_setup.heightCm * 0.55 / 100.0);
        auto kpLoco = m_skel->computeKeypoints(qOut, QVector3D(0.0f, 0.0f, pelvisZ_loco));
        // FIX (heel/toe contact discrimination): передаём heel/ball/tip
        // трёх точек стопы, а не lowest.  Solver сам определит active
        // contact point по footPitchZ.
        // kpLoco индексы: SEG_RFoot=17 (heel), SEG_RToe=18 (ball), 26 (tip);
        //                  SEG_LFoot=21, SEG_LToe=22, 27.
        const double tSec = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();

        // FIX (cross-legged direction protection): determine cross-legged
        // state in pelvis-yaw frame.  В Xsens NWU pelvis +Y = left, +X = forward.
        // Right foot expected: rPel.y < 0; left foot: lPel.y > 0.  Cross =
        // когда знак инвертирован.  crossConf = smoothstep(y / 0.08m) на
        // wrong side, 0 на правильной стороне.
        {
            auto sstep01 = [](double x){ x = std::clamp(x,0.0,1.0); return x*x*(3.0-2.0*x); };
            const QVector3D pelvisXY(kpLoco[SEG_Pelvis].x(),
                                     kpLoco[SEG_Pelvis].y(), 0.0f);
            const QVector3D rXY(kpLoco[SEG_RFoot].x() - pelvisXY.x(),
                                kpLoco[SEG_RFoot].y() - pelvisXY.y(), 0.0f);
            const QVector3D lXY(kpLoco[SEG_LFoot].x() - pelvisXY.x(),
                                kpLoco[SEG_LFoot].y() - pelvisXY.y(), 0.0f);
            const Quat qPYawInv = yaw_only_quat(qOut[SEG_Pelvis]).inv();
            const QVector3D rPel = vec_rotate(rXY, qPYawInv);
            const QVector3D lPel = vec_rotate(lXY, qPYawInv);
            // Right cross when rPel.y > 0; left cross when lPel.y < 0.
            const double cR = sstep01(double(rPel.y()) / 0.08);
            const double cL = sstep01(double(-lPel.y()) / 0.08);
            m_viewport->setCrossLeggedHints(cR > 0.5, cL > 0.5, cR, cL);
        }

        m_viewport->tickLoco(qOut,
                             kpLoco[SEG_RFoot], kpLoco[SEG_RToe], kpLoco[26],
                             kpLoco[SEG_LFoot], kpLoco[SEG_LToe], kpLoco[27],
                             tSec);
    }

    // --- Live streaming --------------------------------------------------
    // Forward the just-computed skeleton to the active UDP stream (if any).
    // We send the post-calibration world pose `qOut` (23 segments) in our
    // native NWU (= MVN Z-up RH) frame; viewport-only framing (yaw/shift/
    // freeze) is excluded so the plugin reproduces the performer's true
    // world orientation and position.
    if (m_streamer && m_streamer->isRunning()) {
        QVector3D pelvisM(0.0f, 0.0f, 0.0f);
        std::array<Quat, kXsensSegmentCount> qStream = qOut;
        // Viewport-only framing (sceneYaw / sceneShift / freeze anchor) is
        // deliberately NOT applied to the stream — those rotate/shift the
        // character purely for our OpenGL camera view; leaking them onto the
        // wire would mis-orient/translate the character in the plugin. We send
        // the raw calibrated world pose plus genuine locomotion travel.
        if (m_skel)
            pelvisM = worldPelvisWithLoco(*m_skel, qOut, m_viewport->lastLocoOffset());
        const bool gloves = f.hasGloves && m_setup.useGloves;
        if (gloves) {
            // Компонуем пальцы с мировой ротацией запястья.
            // f.rightGloveQ[i] / f.leftGloveQ[i] — это cumulative-rotation
            // в hand-local frame.  qOut[SEG_RHand] / qOut[SEG_LHand] —
            // мировые ротации запястий (NWU).
            const Quat qRWristWorld = quat_mult(qStream[SEG_RHand],
                                                m_skel->defAngFor(SEG_RHand));
            const Quat qLWristWorld = quat_mult(qStream[SEG_LHand],
                                                m_skel->defAngFor(SEG_LHand));

            // Y-flip для левой руки (отражение Manus → Xsens body frame
            // для пальцев).

            const Quat finger90 = axisAngleQuat(QVector3D(1, 0, 0), -M_PI / 2.0);
            std::array<Quat, kFingerSegmentsHand> rGloveWorld, lGloveWorld;
            std::array<QVector3D, kFingerSegmentsHand> rGloveWorldP, lGloveWorldP;
            for (int i = 0; i < kFingerSegmentsHand; ++i) {
                const Quat rQ = quat_mult(f.rightGloveQ[i], finger90);
                const Quat lQ = quat_mult(f.leftGloveQ[i],  finger90);
                rGloveWorld[i] = quat_mult(qRWristWorld, rQ);
                lGloveWorld[i] = quat_mult(qLWristWorld, mirror_y_quat(lQ));
                rGloveWorldP[i] = vec_rotate(f.rightGloveP[i],              qRWristWorld);
                lGloveWorldP[i] = vec_rotate(mirrorManusL(f.leftGloveP[i]), qLWristWorld);
            }

            m_streamer->pushFrameWithGloves(
                quint32(f.sampleCounter), qStream,
                pelvisM,
                rGloveWorld, rGloveWorldP,
                lGloveWorld, lGloveWorldP);
        } else {
            m_streamer->pushFrame(quint32(f.sampleCounter), qStream, pelvisM);
        }

        if (m_test) {
            static QVector3D s_lastStreamPelvis(0, 0, 0);
            static bool s_havePrevPelvis = false;
            static int s_streamTick = 0;
            // Self-check (Bug 2 travel): cumulative path length + net horizontal
            // displacement from session start.  "Walks in place" reads as
            // cumHoriz growing while netHoriz stays tiny; real traversal grows
            // both.  Cross-reference [loco commit] density.
            static double s_cumHoriz = 0.0;
            static QVector3D s_startPelvis(0, 0, 0);
            static bool s_haveStart = false;
            if (!s_haveStart) { s_startPelvis = pelvisM; s_haveStart = true; }
            if (s_havePrevPelvis) {
                const QVector3D d = pelvisM - s_lastStreamPelvis;
                const float dxy = std::sqrt(d.x()*d.x() + d.y()*d.y());
                if (dxy < 0.5f) s_cumHoriz += dxy;   // cap rejects teleport spikes
                if (dxy > 0.03f || std::abs(d.z()) > 0.03f) {
                    std::cout << "[stream Δpelvis] dxy=" << std::fixed << std::setprecision(3)
                              << dxy << "m dz=" << d.z() << "m pelvisM=("
                              << pelvisM.x() << "," << pelvisM.y() << "," << pelvisM.z() << ")"
                              << " sample=" << f.sampleCounter << "\n";
                }
            }
            s_lastStreamPelvis = pelvisM;
            s_havePrevPelvis = true;
            if (++s_streamTick % 240 == 0) {
                std::cout << "[stream tick=" << s_streamTick << "] pelvisM=("
                          << std::fixed << std::setprecision(3)
                          << pelvisM.x() << "," << pelvisM.y() << "," << pelvisM.z() << ")"
                          << " qPelvis=(" << qStream[SEG_Pelvis].w << ","
                          << qStream[SEG_Pelvis].x << "," << qStream[SEG_Pelvis].y << ","
                          << qStream[SEG_Pelvis].z << ")"
                          << " qRHand=(" << qStream[SEG_RHand].w << ","
                          << qStream[SEG_RHand].x << "," << qStream[SEG_RHand].y << ","
                          << qStream[SEG_RHand].z << ")"
                          << " qLHand=(" << qStream[SEG_LHand].w << ","
                          << qStream[SEG_LHand].x << "," << qStream[SEG_LHand].y << ","
                          << qStream[SEG_LHand].z << ")"
                          << " sample=" << f.sampleCounter << "\n";
                const QVector3D dn = pelvisM - s_startPelvis;
                const double netHoriz = std::sqrt(dn.x()*dn.x() + dn.y()*dn.y());
                std::cout << "[stream travel] cumHoriz=" << std::fixed
                          << std::setprecision(2) << s_cumHoriz << "m netHoriz="
                          << netHoriz << "m (walks-in-place if cum>>net)\n";
            }
        }
    }

    // --- Recording -------------------------------------------------------
    // Append one RecordedFrame per incoming sample (deduped on sampleCounter
    // so bursts of identical snapshots don't spam the buffer).  Target FPS
    // is enforced at save time, not here — we keep raw samples so HD post
    // processing has the richest possible input.
    if (m_recording && qint64(f.sampleCounter) != m_recLastSample) {
        m_recLastSample = qint64(f.sampleCounter);
        const double tNow =
            double(QDateTime::currentMSecsSinceEpoch() - m_recStartMs) / 1000.0;

        // Hard cap (~60 min at the suit rate).  The soft warning below fires at
        // ~10 min; if capture still hasn't been stopped we FREEZE the buffer
        // here so a runaway take can't grow until the process is OOM-killed.
        // The already-captured frames are kept and stay saveable; we only stop
        // appending.  The save dialog must not be opened from the render tick,
        // so this is intentionally a passive cap (stop + warn once), not an
        // auto-stop.
        const size_t kHardCapFrames = size_t(m_procRateHz * 60.0 * 60.0);
        if (m_recBuffer.size() >= kHardCapFrames) {
            if (!m_recHardCapped) {
                m_recHardCapped = true;
                std::cerr << "[record] ERROR: take hit the ~60 min hard cap ("
                          << m_recBuffer.size() << " frames); further frames are "
                             "dropped to protect memory - stop and save now.\n";
            }
        } else {
            RecordedFrame rf;
            rf.t         = tNow;
            rf.segQuat   = qOut;
            // Real world-space pelvis: locomotion-aware travel + floor clamp,
            // mirroring the live streamer but WITHOUT the view-only yaw/shift/
            // freeze, which would inject discontinuities the HD root filter rings
            // on.  Without this the recording has no root motion (walks in place)
            // and the HD root low-pass / foot-lock passes are no-ops.
            QVector3D pelvisM(0.0f, 0.0f, 0.0f);
            if (m_skel)
                pelvisM = worldPelvisWithLoco(*m_skel, qOut, m_viewport->lastLocoOffset());
            rf.pelvisPos = pelvisM;
            rf.hasGloves = f.hasGloves && m_setup.useGloves;
            if (rf.hasGloves) {
                rf.rightGloveQ = f.rightGloveQ;
                rf.leftGloveQ  = f.leftGloveQ;
            }
            m_recBuffer.push_back(std::move(rf));
            if (!m_recOverflowWarned &&
                m_recBuffer.size() > size_t(m_procRateHz * 60 * 10)) {
                m_recOverflowWarned = true;
                std::cerr << "[record] WARNING: take exceeded ~10 min ("
                          << m_recBuffer.size() << " frames); the capture buffer "
                             "keeps growing in RAM - stop and save soon.\n";
            }
        }
        if (m_hud) m_hud->updateStats(qint64(m_recBuffer.size()), tNow);
    }

    if (m_test) {
        static double t0 = 0.0;
        if (now - t0 > 2.0) {
            const double snapDt = now - t0;          // wall time since last snapshot
            t0 = now;
            static quint64 s_lastSnapSample = f.sampleCounter;
            const double measHz = (snapDt > 1e-3)
                ? double(f.sampleCounter - s_lastSnapSample) / snapDt : 0.0;
            s_lastSnapSample = f.sampleCounter;
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(3);
            ss << "\n========== [RENDER SNAPSHOT] t=" << std::setprecision(2)
               << now << "s  dt=" << std::setprecision(4)
               << dt << "s ==========\n";
            // Active suit + update rate drive every rate-tuned formula (filter
            // time-constants, loco thresholds, dt).  measuredSuit = real sample
            // throughput (sampleCounter delta / wall time): far below procRate
            // means dropped frames.
            ss << std::setprecision(3)
               << "  suit=" << (m_setup.suit == SuitType::Link ? "Link" : "Awinda")
               << " procRate=" << int(m_procRateHz) << "Hz"
               << " measuredSuit=" << std::setprecision(1) << measHz << "Hz over "
               << std::setprecision(2) << snapDt << "s"
               << " gloves=" << (m_setup.useGloves ? "on" : "off")
               << std::setprecision(3) << "\n";

            auto quatEulerDeg = [](const Quat& q, double& rx,
                                   double& ry, double& rz) {
                const double m00 = 1-2*(q.y*q.y+q.z*q.z);
                const double m01 = 2*(q.x*q.y - q.z*q.w);
                const double m02 = 2*(q.x*q.z + q.y*q.w);
                const double m12 = 2*(q.y*q.z - q.x*q.w);
                const double m22 = 1-2*(q.x*q.x+q.y*q.y);
                const double sy = std::clamp(-m02, -1.0, 1.0);
                const double K = 180.0 / M_PI;
                ry = std::asin(sy) * K;
                rx = std::atan2(m12, m22) * K;
                rz = std::atan2(m01, m00) * K;
            };
            auto diffDeg = [&](const Quat& a, const Quat& b) {
                return quat_angle_deg(quat_mult(a, b.inv()));
            };

            // --- All 23 segments: raw vs. post-calibration-output,
            //     Δ between frames, Euler XYZ, still-ticks, ω ---
            // Xsens 23-segment parent map (for local joint-angle readout).
            static const int kSegParent[kXsensSegmentCount] = {
                -1, 0, 1, 2, 3, 4, 5,   // Pelvis L5 L3 T12 T8 Neck Head
                 4, 7, 8, 9,            // R: shoulder upperarm forearm hand (root T8)
                 4,11,12,13,            // L: shoulder upperarm forearm hand (root T8)
                 0,15,16,17,            // R: upperleg lowerleg foot toe (root pelvis)
                 0,19,20,21,            // L: upperleg lowerleg foot toe (root pelvis)
            };
            ss << std::setprecision(3);
            ss << "--- 23 segments: raw -> post-calib q -> drift-locked qOut; "
                  "world quat, local joint angle, drift, lock ---\n";
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                const Quat& in  = raw[i];   // fused sensor world quat
                const Quat& out = q[i];     // after calib offset + coupling + limits
                const Quat& flt = qOut[i];  // after viewport drift-lock (streamed)
                double rx, ry, rz;
                quatEulerDeg(out, rx, ry, rz);
                const int par = kSegParent[i];
                const double localAng = (par >= 0)
                    ? quat_angle_deg(quat_mult(q[par].inv(), out)) : 0.0;
                const double driftAng = diffDeg(flt, out);
                ss << "  seg[" << std::setw(2) << i << "] "
                   << std::left << std::setw(14) << kSegmentNames[i]
                   << std::right << (kTracked[i] ? " *" : "  ")
                   << " raw=(" << std::setw(6) << in.w  << "," << std::setw(6) << in.x  << ","
                               << std::setw(6) << in.y  << "," << std::setw(6) << in.z  << ")"
                   << " q=("   << std::setw(6) << out.w << "," << std::setw(6) << out.x << ","
                               << std::setw(6) << out.y << "," << std::setw(6) << out.z << ")"
                   << " |q|="  << std::setw(6) << quat_angle_deg(out) << "°"
                   << " Δcal=" << std::setw(6) << diffDeg(out, in) << "°"
                   << " eul=(" << std::setw(7) << rx << "," << std::setw(7) << ry << ","
                               << std::setw(7) << rz << ")°\n";
                ss << "             qOut=(" << std::setw(6) << flt.w << "," << std::setw(6) << flt.x << ","
                               << std::setw(6) << flt.y << "," << std::setw(6) << flt.z << ")"
                   << " Δdriftlock=" << std::setw(6) << driftAng << "°"
                   << " localVsParent=" << std::setw(7) << localAng << "°"
                   << " lock=" << (m_viewport && m_viewport->segLocked(i) ? "LOCK" : "live")
                   << " ω=" << std::setprecision(2) << std::setw(6)
                   << (m_viewport ? m_viewport->segAngVelLP(i) : 0.0) << "°/s"
                   << " αacc=" << std::setw(7)
                   << (m_viewport ? m_viewport->segAngAcc(i) : 0.0) << "°/s²"
                   << " stillT=" << std::setw(5)
                   << (m_viewport ? m_viewport->segStillTicks(i) : 0.0) << "s"
                   << " stC=" << std::setw(4) << s_stillCount[i]
                   << " wω=" << std::setw(6) << s_worldOmegaLP[i]
                   << std::setprecision(3) << "\n";
            }

            // --- FK keypoints (all 28) in world-frame meters ---
            // SINGLE-SOURCE: FK is computed on the REAL skeleton (m_skel) the
            // renderer / locomotion / stream use, capturing every intermediate
            // into `fk`.  No parallel skeleton object — the logged values ARE
            // what the pipeline produced for this q.  Root at origin and no
            // loco/camera offset, so this is the raw body shape; the world
            // position (FK + loco offset) is reported in the next section.
            FkDiag fk;
            if (m_skel) m_skel->computeKeypoints(q, QVector3D(0, 0, 0), &fk);
            const auto& pts = fk.kp;
            ss << "--- 28 FK keypoints (world, m) [single-source m_skel] ---\n";
            for (int i = 0; i < kXsensKeypointCount; ++i) {
                ss << "  kp[" << std::setw(2) << i << "] = ("
                   << std::setw(7) << pts[i].x() << ","
                   << std::setw(7) << pts[i].y() << ","
                   << std::setw(7) << pts[i].z() << ")\n";
            }

            // [fidelity] guard — proves the logged FK == the real formula on the
            // REAL skeleton: oriented = quat_mult(q, defAng) for every non-cone
            // segment (the two upper arms add the shoulder cone, so they are
            // reported elsewhere but excluded from this strict check).  A WARN
            // here means the log would be lying about the formula ("parallel").
            if (m_skel) {
                double maxDev = 0.0; int worst = -1;
                for (int i = 0; i < kXsensSegmentCount; ++i) {
                    if (i == SEG_RUpperArm || i == SEG_LUpperArm) continue;
                    const Quat expect =
                        quat_mult(q[i].normalized(), m_skel->defAngFor(i)).normalized();
                    const double dev =
                        quat_angle_deg(quat_mult(fk.oriented[i], expect.inv()));
                    if (dev > maxDev) { maxDev = dev; worst = i; }
                }
                ss << "--- [fidelity] FK single-source check (log == reality) ---\n";
                ss << "  maxDev(oriented vs quat_mult(q,defAng))="
                   << std::setprecision(4) << maxDev << "° @ "
                   << (worst >= 0 ? kSegmentNames[worst] : "-")
                   << (maxDev > 0.01 ? "  *** FK FORMULA MISMATCH ***"
                                     : "  ok")
                   << std::setprecision(3) << "\n";
            }

            {
                const QVector3D loco = m_viewport ? m_viewport->lastLocoOffset()
                                                  : QVector3D(0, 0, 0);
                ss << "--- world position (FK + LocomotionSolver offset) ---\n";
                ss << "  loco_offset  = ("
                   << std::setw(7) << loco.x() << ","
                   << std::setw(7) << loco.y() << ","
                   << std::setw(7) << loco.z() << ")\n";
                ss << "  pelvis_world = ("
                   << std::setw(7) << (pts[0].x() + loco.x()) << ","
                   << std::setw(7) << (pts[0].y() + loco.y()) << ","
                   << std::setw(7) << (pts[0].z() + loco.z()) << ")\n";
                ss << "  rfoot_world  = ("
                   << std::setw(7) << (pts[SEG_RFoot].x() + loco.x()) << ","
                   << std::setw(7) << (pts[SEG_RFoot].y() + loco.y()) << ","
                   << std::setw(7) << (pts[SEG_RFoot].z() + loco.z()) << ")\n";
                ss << "  lfoot_world  = ("
                   << std::setw(7) << (pts[SEG_LFoot].x() + loco.x()) << ","
                   << std::setw(7) << (pts[SEG_LFoot].y() + loco.y()) << ","
                   << std::setw(7) << (pts[SEG_LFoot].z() + loco.z()) << ")\n";
            }

            if (m_rx) {
                ss << "--- s2s pair-symmetry check (L vs R) ---\n";
                const std::pair<int,int> pairs[8] = {
                    { SEG_RShoulder,  SEG_LShoulder  },
                    { SEG_RUpperArm,  SEG_LUpperArm  },
                    { SEG_RForearm,   SEG_LForearm   },
                    { SEG_RHand,      SEG_LHand      },
                    { SEG_RUpperLeg,  SEG_LUpperLeg  },
                    { SEG_RLowerLeg,  SEG_LLowerLeg  },
                    { SEG_RFoot,      SEG_LFoot      },
                    { SEG_RToe,       SEG_LToe       },
                };
                for (const auto& pr : pairs) {
                    const Quat sR = m_rx->getS2s(pr.first);
                    const Quat sL = m_rx->getS2s(pr.second);
                    const double diffW = std::abs(sR.w - sL.w);
                    const bool hemiBad = (sR.w * sL.w < 0.0);
                    const double devMirr = mirrorYDeviationDeg(sR, sL);
                    const double devPar  = parallelDeviationDeg(sR, sL);
                    const double devBest = std::min(devMirr, devPar);
                    const char* bestSym = (devMirr <= devPar) ? "MY" : "PR";
                    const char* tag = (devBest > 30.0) ? "  *** SYMMETRY FAIL ***"
                                    : (devBest > 10.0) ? "  *** SYMMETRY WARN ***"
                                    : "";
                    ss << "  pair seg[" << std::setw(2) << pr.first << "/" << std::setw(2) << pr.second << "]"
                       << "  R.w=" << std::setw(7) << sR.w
                       << "  L.w=" << std::setw(7) << sL.w
                       << "  |diff|=" << std::setw(7) << diffW
                       << "  devMirr=" << std::setw(7) << std::fixed << std::setprecision(2) << devMirr << "°"
                       << "  devPar=" << std::setw(7) << devPar << "°"
                       << "  best=" << bestSym
                       << (hemiBad ? "  *** HEMISPHERE MISMATCH ***" : "")
                       << tag
                       << "\n";
                }
            }

            // --- All 27 bones-with-dummies: length, parent angle ---
            // SINGLE-SOURCE: topology from the real m_skel; `lens` is fk.len —
            // the exact bone-length array this frame's FK used (actor-size driven).
            if (m_skel) {
            const auto& sIdx = m_skel->startPts();
            const auto& eIdx = m_skel->endPts();
            const auto& lens = fk.len;
            ss << "--- 27 bones (with dummy scap/pelvis stubs): length · direction · angle-to-vertical ---\n";
            for (int b = 0; b < kXsensSegmentCountWithDummies; ++b) {
                const QVector3D A = pts[sIdx[b]];
                const QVector3D B = pts[eIdx[b]];
                const QVector3D v = B - A;
                const double L = std::sqrt(double(v.x()*v.x() + v.y()*v.y() + v.z()*v.z()));
                const QVector3D d = L > 1e-6 ? v / float(L) : QVector3D(0, 0, 0);
                const double angVert = (L > 1e-6)
                    ? std::acos(std::clamp(double(d.z()), -1.0, 1.0)) * 180.0 / M_PI
                    : 0.0;
                ss << "  bone[" << std::setw(2) << b << "] "
                   << "kp" << std::setw(2) << sIdx[b] << "→kp" << std::setw(2) << eIdx[b]
                   << "  L=" << std::setw(6) << L << "m"
                   << " (skelL=" << std::setw(6) << lens[b] << "m)"
                   << "  dir=(" << std::setw(6) << d.x() << ","
                                << std::setw(6) << d.y() << ","
                                << std::setw(6) << d.z() << ")"
                   << "  ∠vert=" << std::setw(6) << angVert << "°"
                   << "  qWorld=" << fmtQ4(fk.global[b])
                   << "  localVec=(" << std::setw(6) << lens[b] << ",0,0)\n";
            }
            // boneVec[s] = vec_rotate(localVec, qWorld); kp[end]=kp[start]+boneVec.
            // So the two lines above fully document the local→world position step.
            }  // if (m_skel)

            // --- Anatomical hinge limits: measured joint swing/twist vs cap ---
            ss << "--- hinge joint limits (measured swing/twist vs anatomical cap) ---\n";
            {
                struct HJ { const char* lbl; int seg; };
                const HJ hinges[] = {
                    { "R_knee",  SEG_RLowerLeg }, { "L_knee",  SEG_LLowerLeg },
                    { "R_elbow", SEG_RForearm  }, { "L_elbow", SEG_LForearm  },
                };
                for (const auto& h : hinges) {
                    const HingeLimitDiag& d = g_hingeDiag[h.seg];
                    ss << "  " << std::left << std::setw(8) << h.lbl << std::right;
                    if (!d.valid) { ss << "  (not evaluated)\n"; continue; }
                    ss << "  swing=" << std::setw(7) << d.swingDeg << "°/" << std::setw(6) << d.maxSwingDeg << "°"
                       << "  twist=" << std::setw(7) << d.twistDeg << "°/" << std::setw(6) << d.maxTwistDeg << "°"
                       << (d.clamped ? "  *** CLAMPED ***" : "  ok") << "\n";
                }
            }

            // --- Fingers (gloves only): ergo joint angle raw→effective→clamped
            //     vs anatomical limit, plus hand-local FK quats/tip and the
            //     wrist world quaternion the finger chain hangs off. ---
            if (m_setup.useGloves && f.hasGloves && m_skel) {
                static const char* kFN[5] = { "thumb", "index", "middle", "ring", "pinky" };
                static const char* kJN[3] = { "MCP", "PIP", "DIP" };
                const double K = 180.0 / M_PI;
                FingerDiagHand dgL, dgR;
                { QMutexLocker lk(&g_fingerDiag.lock); dgL = g_fingerDiag.left; dgR = g_fingerDiag.right; }
                const Quat wristR = quat_mult(qOut[SEG_RHand], m_skel->defAngFor(SEG_RHand)).normalized();
                const Quat wristL = quat_mult(qOut[SEG_LHand], m_skel->defAngFor(SEG_LHand)).normalized();
                ss << "--- fingers: ergo angle [raw -> baseline-eff -> clamped] vs limit ---\n";
                auto dumpHand = [&](const char* hand, const FingerDiagHand& dh, const Quat& wrist,
                                    const std::array<Quat, kFingerSegmentsHand>& lq,
                                    const std::array<QVector3D, kFingerSegmentsHand>& lp) {
                    ss << "  [" << hand << "] baselineApplied="
                       << (dh.valid && dh.baselineApplied ? "yes" : "no")
                       << "  wristWorld=(" << std::setprecision(4)
                       << wrist.w << "," << wrist.x << "," << wrist.y << "," << wrist.z << ")"
                       << std::setprecision(3) << "\n";
                    // §9: wrist world-frame direction so hand orientation/rotation
                    // is readable directly (forward=+X, up=+Z, palmNormal=+Y).
                    {
                        const QVector3D fwd  = vec_rotate(QVector3D(1,0,0), wrist);
                        const QVector3D up   = vec_rotate(QVector3D(0,0,1), wrist);
                        const QVector3D palm = vec_rotate(QVector3D(0,1,0), wrist);
                        ss << "    wristDir fwd=" << fmtV3(fwd) << " up=" << fmtV3(up)
                           << " palmN=" << fmtV3(palm) << "\n";
                    }
                    if (!dh.valid) { ss << "    (no ergo data this frame)\n"; return; }
                    for (int fg = 0; fg < 5; ++fg) {
                        const auto& Lm = kFingerLimits[fg];
                        ss << "    " << std::left << std::setw(7) << kFN[fg] << std::right
                           << " spread[" << std::setw(7) << dh.raw[fg*4+0] << " ->"
                           << std::setw(8) << dh.spreadEffDeg[fg] << " ->"
                           << std::setw(8) << dh.spreadClDeg[fg] << "] lim["
                           << std::setw(6) << Lm[0].spreadMin*K << "," << std::setw(6) << Lm[0].spreadMax*K << "]°"
                           << (dh.spreadClamped[fg] ? " *CLAMP*" : "") << "\n";
                        for (int j = 0; j < 3; ++j) {
                            ss << "            " << std::left << std::setw(4) << kJN[j] << std::right
                               << " flex[" << std::setw(7) << dh.raw[fg*4+1+j] << " ->"
                               << std::setw(8) << dh.flexDeg[fg][j] << " ->"
                               << std::setw(8) << dh.flexClDeg[fg][j] << "] lim["
                               << std::setw(6) << Lm[j].flexMin*K << "," << std::setw(6) << Lm[j].flexMax*K << "]°"
                               << (dh.flexClamped[fg][j] ? " *CLAMP*" : "") << "\n";
                        }
                        const int b = fg*4;
                        ss << "            FK-local tipPos=(" << std::setprecision(4)
                           << lp[b+3].x() << "," << lp[b+3].y() << "," << lp[b+3].z() << ")"
                           << " jointAng(MC/PP/MP/DP)=" << std::setprecision(1)
                           << quat_angle_deg(lq[b+0]) << "/" << quat_angle_deg(lq[b+1]) << "/"
                           << quat_angle_deg(lq[b+2]) << "/" << quat_angle_deg(lq[b+3]) << "°"
                           << std::setprecision(3) << "\n";
                    }
                };
                dumpHand("LEFT",  dgL, wristL, f.leftGloveQ,  f.leftGloveP);
                dumpHand("RIGHT", dgR, wristR, f.rightGloveQ, f.rightGloveP);
            }

            // === §2  Calibration offset:  cand[i] = raw[i] · refWorld[i]^-1 ===
            // refWorld is the T/N-pose calibration reference; the jump-reject
            // smoothstep [20..35]° (gyroQuiet-gated) guards mag/IMU glitches.
            ss << std::setprecision(3);
            ss << "--- calibration offset (cand = raw . refWorld^-1; jump-reject smoothstep[20..35]deg) ---\n";
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                if (!kTracked[i]) continue;
                ss << "  off[" << std::setw(2) << i << "] "
                   << std::left << std::setw(14) << kSegmentNames[i] << std::right
                   << " refWorld=" << fmtQ4(s_refWorld[i])
                   << " jump=" << std::setw(6) << std::setprecision(2) << g_renderDiag.jumpDeg[i] << "deg"
                   << " rejectW=" << std::setw(5) << std::setprecision(3) << g_renderDiag.rejectW[i]
                   << (g_renderDiag.gyroQuiet[i] ? " gyrQuiet" : " gyrLive")
                   << (g_renderDiag.rejectW[i] > 0.999 ? "  *FULL-REJECT*"
                       : g_renderDiag.rejectW[i] > 0.0 ? "  *blend*" : "")
                   << "\n";
            }

            // === §3  Spine/neck smoothstep distribution + arm coupling ===
            ss << std::setprecision(3);
            ss << "--- spine/neck interpolation (slerp pelvis..T8..head, w=t^2(3-2t)) ---\n";
            ss << "  L5  w=" << g_renderDiag.spineW_L5  << " q=" << fmtQ4(q[SEG_L5])
               << " |a|=" << quat_angle_deg(q[SEG_L5])  << "deg\n";
            ss << "  L3  w=" << g_renderDiag.spineW_L3  << " q=" << fmtQ4(q[SEG_L3])
               << " |a|=" << quat_angle_deg(q[SEG_L3])  << "deg\n";
            ss << "  T12 w=" << g_renderDiag.spineW_T12 << " q=" << fmtQ4(q[SEG_T12])
               << " |a|=" << quat_angle_deg(q[SEG_T12]) << "deg\n";
            ss << "  Nck w=" << g_renderDiag.neckW      << " q=" << fmtQ4(q[SEG_Neck])
               << " |a|=" << quat_angle_deg(q[SEG_Neck])<< "deg\n";
            ss << "--- arm coupling (scapular-humeral shrug; wrist->forearm twist follow) ---\n";
            ss << "  scapular R: upZ=" << g_renderDiag.scapUpZR
               << " active=" << (g_renderDiag.scapActiveR ? "yes" : "no")
               << " appliedAng=" << (g_renderDiag.scapAngR * 180.0 / M_PI) << "deg | L: upZ="
               << g_renderDiag.scapUpZL << " active=" << (g_renderDiag.scapActiveL ? "yes" : "no")
               << " appliedAng=" << (g_renderDiag.scapAngL * 180.0 / M_PI) << "deg\n";
            ss << "  wrist     R: handTwist=" << (g_renderDiag.wTwistHalfR * 2.0 * 180.0 / M_PI)
               << "deg forearmFollow=" << (g_renderDiag.faTwistAddR * 180.0 / M_PI) << "deg | L: handTwist="
               << (g_renderDiag.wTwistHalfL * 2.0 * 180.0 / M_PI) << "deg forearmFollow="
               << (g_renderDiag.faTwistAddL * 180.0 / M_PI) << "deg\n";

            // === §4  Locomotion solver — pose / feet / fast-movement context ===
            if (m_viewport) {
                const LocoDiag L = m_viewport->locoDiag();
                auto footState = [](double pz){
                    return pz > 0.17 ? "heel-down(toe-up)"
                         : pz < -0.17 ? "toe-down(heel-up)" : "flat"; };
                ss << "--- locomotion: pose / foot contact / heel-toe / pelvis-Z (single source) ---\n";
                ss << "  pose=" << locoPoseName(L.pose) << " ticks=" << L.poseTicks
                   << " support=" << (L.support==0?"RIGHT":L.support==1?"LEFT":"BOTH")
                   << " tiltCos=" << L.tiltCos
                   << " pelvisZVel=" << L.pelvisZVel << "m/s"
                   << " airborneT=" << L.airborneTicks << " landedT=" << L.landedTicks
                   << " zupt=" << L.zuptTicks << "\n";
                ss << "  R: conf=" << L.confR << (L.committedR?"(commit)":"")
                   << " angV=" << L.rAngV << " footPitchZ=" << L.footPitchZR
                   << " blend=" << L.contactBlendR
                   << " heelLift=" << L.heelLiftConfR << (L.heelLiftR?"*":"")
                   << " stance=" << footState(L.footPitchZR)
                   << " anchor=" << fmtV3(L.anchorR) << "\n";
                ss << "  L: conf=" << L.confL << (L.committedL?"(commit)":"")
                   << " angV=" << L.lAngV << " footPitchZ=" << L.footPitchZL
                   << " blend=" << L.contactBlendL
                   << " heelLift=" << L.heelLiftConfL << (L.heelLiftL?"*":"")
                   << " stance=" << footState(L.footPitchZL)
                   << " anchor=" << fmtV3(L.anchorL) << "\n";
                ss << "  pelvisAngV=" << L.pelvisAngV << " yawAngV=" << L.pelvisYawAngV
                   << " locoOffset=" << fmtV3(L.offset) << "\n";
                // Decision internals — why the offset/anchors moved (single source).
                ss << "  [decide] rawC R=" << L.rawCR << " L=" << L.rawCL
                   << " eff R=" << L.effR << " L=" << L.effL
                   << " imbalance=" << L.imbalance
                   << " stepCapXY=" << L.maxStepXY << "m"
                   << (L.stepClampedXY ? " *CLAMPED*" : "") << "\n";
                ss << "  [decide] fkXYrange R=" << L.fkxyRangeR << " L=" << L.fkxyRangeL
                   << " fkXYstableW R=" << L.fkxyStableWR << " L=" << L.fkxyStableWL
                   << " yawFreezeW=" << L.yawFreezeW
                   << " pelvisRotKill=" << L.pelvisRotKill
                   << " rollW R=" << L.rollingWR << " L=" << L.rollingWL << "\n";
                ss << "  [decide] airborne: feetLifted=" << (L.feetLifted ? 1 : 0)
                   << " ballistic=" << (L.ballistic ? 1 : 0)
                   << " driftAir=" << (L.driftAir ? 1 : 0) << "\n";
                // Foot roll (single-source fk.kp, body frame, root@origin):
                // heel=SEG_*Foot, ball=SEG_*Toe, tip=26/27.  Lowest Z = ground
                // contact; footPitchZ sign says heel-down(+) vs toe-down(-).  Lets
                // the log show toe-stand / heel-stand / mid-roll per foot.
                auto footRoll = [&](const char* s, int heel, int ball, int tip, double pz){
                    const QVector3D H = fk.kp[heel], B = fk.kp[ball], T = fk.kp[tip];
                    const char* contact = (H.z() <= B.z() && H.z() <= T.z()) ? "heel"
                                        : (T.z() <= H.z() && T.z() <= B.z()) ? "tip" : "ball";
                    ss << "  " << s << " heel=" << fmtV3(H) << " ball=" << fmtV3(B)
                       << " tip=" << fmtV3(T) << " lowest=" << contact
                       << " pitchSin=" << std::setprecision(3) << pz << "\n";
                };
                footRoll("Rfoot", SEG_RFoot, SEG_RToe, 26, L.footPitchZR);
                footRoll("Lfoot", SEG_LFoot, SEG_LToe, 27, L.footPitchZL);
            }

            // === §5  Viewport / operator view (what the operator literally sees) ===
            if (m_viewport) {
                const auto& rkp = m_viewport->lastRenderedKeypoints();
                ss << "--- viewport / operator view (post loco+floor+yaw+shift+freeze) ---\n";
                ss << std::setprecision(2);
                ss << "  sceneYaw=" << (m_viewport->sceneYaw() * 180.0 / M_PI) << "deg"
                   << " sceneShift=" << fmtV3(m_viewport->sceneShift())
                   << " frozen=" << (m_viewport->isFrozen() ? "yes" : "no")
                   << " freezeAnchor=" << fmtV3(m_viewport->freezeAnchor())
                   << " floorClamp=" << std::setprecision(4) << m_viewport->lastFloorClamp() << "m\n";
                ss << "  rendered pelvis=" << fmtV3(rkp[SEG_Pelvis])
                   << " rfoot=" << fmtV3(rkp[SEG_RFoot])
                   << " lfoot=" << fmtV3(rkp[SEG_LFoot]) << "\n";
                ss << "  rawFK    pelvis=" << fmtV3(pts[SEG_Pelvis])
                   << " rfoot=" << fmtV3(pts[SEG_RFoot])
                   << " lfoot=" << fmtV3(pts[SEG_LFoot]) << "\n";
                int lockedN = 0;
                for (int i = 0; i < kXsensSegmentCount; ++i) if (m_viewport->segLocked(i)) ++lockedN;
                ss << "  drift-lock held=" << lockedN << "/" << int(kXsensSegmentCount);
                for (int i = 0; i < kXsensSegmentCount; ++i)
                    if (m_viewport->segLocked(i)) ss << " " << kSegmentNames[i];
                ss << "\n";
                ss << std::setprecision(3);
            }

            ss << "============================================================\n";
            std::cout << ss.str();
            std::cout.flush();
        }
    }

    // === §7  Per-frame compact pulse + §8 threshold events ===============
    // The pulse is one line per UNIQUE sample (deduped on sampleCounter so we
    // don't reprint a frame when the render timer outruns the suit): a
    // continuous timeline so drift (steady angles, |w|~0) is told apart from
    // real motion, and jitter is visible frame-by-frame.  A gap in the
    // sampleCounter sequence in the log therefore flags a dropped frame.  At
    // 240 Hz (Link) this is ~240 lines/s; raise kPulseStride to thin it.
    // Events fire only on transients the periodic 2 s snapshots miss (pose
    // change, heel<->toe, pelvis-Z spike, |w| jitter), each rate-limited so
    // rest stays quiet.  All values are read from the single-source diag
    // structs — no formula is recomputed here.
    if (m_test) {
        static constexpr quint64 kPulseStride = 1;   // emit every Nth unique sample
        const LocoDiag L = m_viewport ? m_viewport->locoDiag() : LocoDiag{};
        auto jAng = [&](int parent, int child){
            return quat_angle_deg(quat_mult(q[parent].inv(), q[child])); };
        int maxSeg = 0; double maxWdeg = 0.0;
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            const double wdeg = s_worldOmegaLP[i] * 180.0 / M_PI;   // rad/s -> deg/s
            if (wdeg > maxWdeg) { maxWdeg = wdeg; maxSeg = i; }
        }
        const QVector3D pel = m_viewport
            ? m_viewport->lastRenderedKeypoints()[SEG_Pelvis] : QVector3D(0,0,0);
        static quint64 s_pulseLastSample = ~quint64(0);
        if (f.sampleCounter != s_pulseLastSample &&
            (f.sampleCounter % kPulseStride) == 0) {
            s_pulseLastSample = f.sampleCounter;
            std::ostringstream ps;
            ps << std::fixed << std::setprecision(1);
            ps << "[pulse] f=" << f.sampleCounter << " t=" << std::setprecision(2) << now
               << " dt=" << std::setprecision(1) << (dt * 1000.0) << "ms"
               << " pose=" << locoPoseName(L.pose)
               << " pelvis=(" << std::setprecision(3) << pel.x() << "," << pel.y() << "," << pel.z() << ")"
               << std::setprecision(1)
               << " ang{Rkn=" << jAng(SEG_RUpperLeg, SEG_RLowerLeg)
               << " Lkn=" << jAng(SEG_LUpperLeg, SEG_LLowerLeg)
               << " Rel=" << jAng(SEG_RUpperArm, SEG_RForearm)
               << " Lel=" << jAng(SEG_LUpperArm, SEG_LForearm)
               << " Rhip=" << jAng(SEG_Pelvis, SEG_RUpperLeg)
               << " Lhip=" << jAng(SEG_Pelvis, SEG_LUpperLeg)
               << " spine=" << jAng(SEG_Pelvis, SEG_T8)
               << " neck=" << jAng(SEG_T8, SEG_Head) << "}"
               << " footPitch{R=" << std::setprecision(3) << L.footPitchZR << " L=" << L.footPitchZL << "}"
               << " heelLift{R=" << (L.heelLiftR ? 1 : 0) << " L=" << (L.heelLiftL ? 1 : 0) << "}"
               << " pelvisZVel=" << L.pelvisZVel
               << " maxW=" << kSegmentNames[maxSeg] << ":" << std::setprecision(1) << maxWdeg << "deg/s\n";
            std::cout << ps.str();
        }

        static int s_evtTick = 0; s_evtTick++;
        // (a) pose transition (Stand/Sit/Squat/Lying/Airborne).
        static int s_prevPose = -1;
        if (int(L.pose) != s_prevPose) {
            std::cout << "[evt:pose] f=" << f.sampleCounter << " "
                      << locoPoseName(s_prevPose < 0 ? 0 : s_prevPose) << " -> " << locoPoseName(L.pose)
                      << " (tiltCos=" << std::setprecision(3) << L.tiltCos
                      << " pelvisZVel=" << L.pelvisZVel << "m/s)\n";
            s_prevPose = int(L.pose);
        }
        // (b) per-foot heel<->toe / flat stance change.
        auto stanceOf = [](double pz){ return pz > 0.17 ? 1 : pz < -0.17 ? -1 : 0; };
        auto stanceName = [](int s){ return s > 0 ? "heel" : s < 0 ? "toe" : "flat"; };
        static int s_stanceR = 0, s_stanceL = 0;
        const int srR = stanceOf(L.footPitchZR), srL = stanceOf(L.footPitchZL);
        if (srR != s_stanceR) {
            std::cout << "[evt:foot] f=" << f.sampleCounter << " R " << stanceName(s_stanceR)
                      << "->" << stanceName(srR) << " footPitchZ=" << std::setprecision(3)
                      << L.footPitchZR << " pose=" << locoPoseName(L.pose) << "\n";
            s_stanceR = srR;
        }
        if (srL != s_stanceL) {
            std::cout << "[evt:foot] f=" << f.sampleCounter << " L " << stanceName(s_stanceL)
                      << "->" << stanceName(srL) << " footPitchZ=" << std::setprecision(3)
                      << L.footPitchZL << " pose=" << locoPoseName(L.pose) << "\n";
            s_stanceL = srL;
        }
        // (c) pelvis vertical-velocity spike (jump / land / fast squat).
        static int s_lastPelvisZEvt = -1000;
        if (std::abs(L.pelvisZVel) > 0.40 && (s_evtTick - s_lastPelvisZEvt) > 15) {
            s_lastPelvisZEvt = s_evtTick;
            std::cout << "[evt:pelvisZ] f=" << f.sampleCounter << " pelvisZVel="
                      << std::setprecision(3) << L.pelvisZVel << "m/s pose=" << locoPoseName(L.pose)
                      << " airborneT=" << L.airborneTicks << " landedT=" << L.landedTicks << "\n";
        }
        // (d) per-segment angular-velocity jitter (rate-limited per segment).
        static std::array<int, kXsensSegmentCount> s_lastOmegaEvt{};
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            const double wdeg = s_worldOmegaLP[i] * 180.0 / M_PI;
            if (wdeg > 250.0 && (s_evtTick - s_lastOmegaEvt[i]) > 30) {
                s_lastOmegaEvt[i] = s_evtTick;
                std::cout << "[evt:omega] f=" << f.sampleCounter << " seg[" << i << "] "
                          << kSegmentNames[i] << " |w|=" << std::setprecision(1) << wdeg
                          << "deg/s jump=" << std::setprecision(2) << g_renderDiag.jumpDeg[i]
                          << "deg rejectW=" << std::setprecision(3) << g_renderDiag.rejectW[i] << "\n";
            }
        }

        // === §9  Fast-motion BURST capture =================================
        // A ring buffer of rich per-frame records is always filled (cheap POD
        // copy, no formatting).  When a fast-motion trigger fires (pelvis-Z
        // spike, big |ω|, pose change, foot-stance flip) we flush the PRE-trigger
        // window in full per-frame detail and keep emitting detailed [burst]
        // lines for a POST window — so the run-up to and recovery from a jerk
        // (squat→jump, heel↔toe roll) is captured frame-by-frame, while rest
        // stays silent.  All values come from the same single-source diag/q used
        // above — nothing is recomputed differently.
        struct BurstRec {
            quint64 f; double t; float dtms; int pose;
            float pelX, pelY, pelZ, pelvisZVel, footPzR, footPzL, maxWdeg; int maxWseg;
            float rKnee, lKnee, rElb, lElb, rHip, lHip, spine, neck;
        };
        static constexpr int kBurstCap = 128;        // ~0.5 s pre-context at 240 Hz
        static std::array<BurstRec, kBurstCap> s_burst{};
        static int s_burstHead = 0, s_burstCount = 0, s_burstPost = 0;
        BurstRec rec{};
        rec.f = f.sampleCounter; rec.t = now; rec.dtms = float(dt * 1000.0);
        rec.pose = int(L.pose);
        rec.pelX = pel.x(); rec.pelY = pel.y(); rec.pelZ = pel.z();
        rec.pelvisZVel = float(L.pelvisZVel);
        rec.footPzR = float(L.footPitchZR); rec.footPzL = float(L.footPitchZL);
        rec.maxWdeg = float(maxWdeg); rec.maxWseg = maxSeg;
        rec.rKnee = float(jAng(SEG_RUpperLeg, SEG_RLowerLeg));
        rec.lKnee = float(jAng(SEG_LUpperLeg, SEG_LLowerLeg));
        rec.rElb  = float(jAng(SEG_RUpperArm, SEG_RForearm));
        rec.lElb  = float(jAng(SEG_LUpperArm, SEG_LForearm));
        rec.rHip  = float(jAng(SEG_Pelvis, SEG_RUpperLeg));
        rec.lHip  = float(jAng(SEG_Pelvis, SEG_LUpperLeg));
        rec.spine = float(jAng(SEG_Pelvis, SEG_T8));
        rec.neck  = float(jAng(SEG_T8, SEG_Head));
        s_burst[s_burstHead] = rec;
        s_burstHead = (s_burstHead + 1) % kBurstCap;
        if (s_burstCount < kBurstCap) ++s_burstCount;

        auto fmtBurst = [](const BurstRec& r) {
            std::ostringstream b; b << std::fixed << std::setprecision(3)
              << "[burst] f=" << r.f << " t=" << std::setprecision(2) << r.t
              << " dt=" << std::setprecision(1) << r.dtms << "ms pose=" << locoPoseName(r.pose)
              << " pelvis=(" << std::setprecision(3) << r.pelX << "," << r.pelY << "," << r.pelZ << ")"
              << " pelvisZVel=" << r.pelvisZVel
              << " foot{R=" << r.footPzR << " L=" << r.footPzL << "}"
              << std::setprecision(1)
              << " ang{Rkn=" << r.rKnee << " Lkn=" << r.lKnee << " Rel=" << r.rElb << " Lel=" << r.lElb
              << " Rhip=" << r.rHip << " Lhip=" << r.lHip << " spine=" << r.spine << " neck=" << r.neck << "}"
              << " maxW=" << kSegmentNames[r.maxWseg] << ":" << r.maxWdeg << "deg/s";
            return b.str();
        };

        // Trigger edge detection (separate statics so [evt:*] above is untouched).
        static int s_burstPrevPose = -1, s_burstStanceR = 0, s_burstStanceL = 0;
        const int  bsR = stanceOf(L.footPitchZR), bsL = stanceOf(L.footPitchZL);
        const bool trgZ    = std::abs(L.pelvisZVel) > 0.40;
        const bool trgW    = maxWdeg > 250.0;
        const bool trgPose = (s_burstPrevPose >= 0 && int(L.pose) != s_burstPrevPose);
        const bool trgFoot = (bsR != s_burstStanceR) || (bsL != s_burstStanceL);
        s_burstPrevPose = int(L.pose); s_burstStanceR = bsR; s_burstStanceL = bsL;

        if ((trgZ || trgW || trgPose || trgFoot) && s_burstPost == 0) {
            std::ostringstream hb;
            hb << "\n----- [burst] TRIGGER f=" << f.sampleCounter << " reason=";
            if (trgZ)    hb << "pelvisZVel(" << std::setprecision(2) << L.pelvisZVel << "m/s) ";
            if (trgW)    hb << "omega(" << kSegmentNames[maxSeg] << ":" << std::setprecision(0) << maxWdeg << "deg/s) ";
            if (trgPose) hb << "pose->" << locoPoseName(L.pose) << " ";
            if (trgFoot) hb << "footStance ";
            hb << "— flushing " << s_burstCount << " pre-frames -----\n";
            std::cout << hb.str();
            const int start = (s_burstHead - s_burstCount + kBurstCap) % kBurstCap;
            for (int n = 0; n < s_burstCount; ++n)        // oldest -> newest
                std::cout << fmtBurst(s_burst[(start + n) % kBurstCap]) << " [pre]\n";
            s_burstPost = 120;                            // ~0.5 s post-context
        }
        if (s_burstPost > 0) { std::cout << fmtBurst(rec) << " [post]\n"; --s_burstPost; }

        std::cout.flush();
    }

    // v4: Rotate Manus-local finger positions into WORLD frame.
    //
    // Two things are fixed vs the earlier version:
    //   1. Use cand * defAng (the actual world orientation of the hand
    //      body-frame), not just cand (the delta from reference).  At T-pose
    //      cand = identity, so the old code placed fingers along Manus +X
    //      (forward-of-hand-local) in world — which in T-pose was forward,
    //      NOT sideways as anatomy requires.  "Broken wrist" bug.
    //   2. Mirror-Y the Manus-local positions for the LEFT hand.  The Manus
    //      convention is the same for both hands (+Y = thumb side), but Xsens
    //      L-hand body-frame is a reflection of the R-hand frame in terms of
    //      how Manus +Y maps to world.  No quaternion can realise a reflection
    //      → we fix it with a coord-flip.  After the flip, L-thumb lands
    //      anatomically forward (+X_world in T-pose), matching R-thumb.
    //
    // No filters applied: Manus SDK already delivers stable per-finger data.
    // Suit-only mode is untouched — this block runs regardless, but the
    // downstream `if (!m_haveGloves) return;` in drawSkeleton prevents any
    // finger rendering when gloves aren't active.
    std::array<QVector3D, kFingerSegmentsHand> relR{}, relL{};
    const Quat qRHandFull = quat_mult(q[SEG_RHand],
                                      m_skel->defAngFor(SEG_RHand));
    const Quat qLHandFull = quat_mult(q[SEG_LHand],
                                      m_skel->defAngFor(SEG_LHand));
    for (int i = 0; i < kFingerSegmentsHand; ++i) {
        relR[i] = vec_rotate(f.rightGloveP[i],              qRHandFull);
        relL[i] = vec_rotate(mirrorManusL(f.leftGloveP[i]), qLHandFull);
    }
    m_viewport->updateHands(f.hasGloves && m_setup.useGloves, relR, relL);
}

void MainWindow::onPauseSession()
{
    m_sessionRunning = false;
    if (m_panel) m_panel->setSessionRunning(false);
    logTest("[session] paused");
}

void MainWindow::onResumeSession()
{
    m_sessionRunning = true;
    if (m_panel) m_panel->setSessionRunning(true);
    logTest("[session] resumed");
}

void MainWindow::onOpenLiveWizard()
{
    // Stream & Record are mutually exclusive — prevent double-session.
    if (m_recording) {
        QMessageBox::warning(this, Lang::t("live_wiz_title"),
            QStringLiteral("Нельзя запустить стрим пока идёт запись. "
                           "Остановите запись сначала."));
        return;
    }
    LiveStreamWizard w(m_setup.suit, this);
    (void)w.winId();
    applyDarkTitleBar(&w);
    if (w.exec() != QDialog::Accepted) return;
    QString err;
    LiveSettings cfg = w.result();
    cfg.useGloves = m_setup.useGloves;
    cfg.verboseLog = m_test;   // periodic [STREAM SNAPSHOT] when running -test

    // T-pose origin positions per Xsens segment (meters). The MVN plugin
    // (LiveLinkMvnSource) uses these as the scale field in FTransform; the
    // retarget asset divides Unreal world T-pose length by this length to
    // size the pelvis. Without real values pelvis is mis-scaled by ~47x and
    // the rig "explodes".
    if (m_skel) {
        std::array<Quat, kXsensSegmentCount> identity{};
        for (auto& q : identity) q = Quat(1, 0, 0, 0);
        const float pelvisZ = float(m_setup.heightCm * 0.55 / 100.0);
        auto kp = m_skel->computeKeypoints(identity, QVector3D(0.0f, 0.0f, pelvisZ));
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            cfg.tposeOriginM[i] = kp[i];  // NWU == MVN Z-up RH; no conversion.
            cfg.defAngT[i] = m_skel->defAngFor(i);
        }
    }

    if (!m_streamer->start(cfg, &err)) {
        QMessageBox::warning(this, Lang::t("live_wiz_title"),
                             Lang::t("live_err_bind") + "\n" + err);
        return;
    }
    m_streamStartMs = QDateTime::currentMSecsSinceEpoch();
    if (statusBar()) statusBar()->showMessage(Lang::t("live_running"), 1500);
    logTest("[stream] started → " + w.result().host.toStdString() + ":"
            + std::to_string(w.result().port));
    layoutHud();
}

void MainWindow::onOpenRecordWizard()
{
    if (m_streamer && m_streamer->isRunning()) {
        QMessageBox::warning(this, Lang::t("rec_wiz_title"),
            QStringLiteral("Нельзя запустить запись пока идёт стрим. "
                           "Остановите стрим сначала."));
        return;
    }
    if (m_finishing) return;          // a save is in progress
    if (m_takePending) {              // an unsaved take exists — resolve it first
        QMessageBox::warning(this, Lang::t("rec_wiz_title"), Lang::t("rec_take_kept"));
        return;
    }
    RecordWizard w(m_setup.suit, this);
    (void)w.winId();
    applyDarkTitleBar(&w);
    if (w.exec() != QDialog::Accepted) return;
    startRecording(w.result());
}

void MainWindow::onOpenJointSettings()
{
    if (!m_jointDlg) {
        m_jointDlg = new JointOffsetsDialog(&m_jointOffsets, this);
        (void)m_jointDlg->winId();
        applyDarkTitleBar(m_jointDlg);
    }
    m_jointDlg->show();
    m_jointDlg->raise();
    m_jointDlg->activateWindow();
}

void MainWindow::startRecording(const RecordSettings& cfg)
{
    if (m_finishing || m_takePending) return;   // don't clobber an unsaved take
    m_recCfg = cfg;
    m_recBuffer.clear();
    m_recBuffer.reserve(size_t(m_procRateHz * 60 * 10)); // ~10 min at the suit rate (one frame per unique sample)
    m_recOverflowWarned = false;
    m_recHardCapped     = false;
    m_recLastSample = -1;
    m_recStartMs = QDateTime::currentMSecsSinceEpoch();
    m_recording = true;

    if (!m_hud) {
        m_hud = new RecordHud(centralWidget());
        connect(m_hud, &RecordHud::stopClicked,
                this, &MainWindow::onRecordStop);
    }
    const char* fmtName = (cfg.format == RecordFormat::BVH) ? "BVH" : "FBX";
    m_hud->setFormatLabel(QString("● REC  %1 · %2 fps")
                          .arg(fmtName).arg(cfg.fps));
    m_hud->show();
    m_hud->raise();
    layoutHud();
    logTest(std::string("[rec] started fmt=") + fmtName
            + " fps=" + std::to_string(cfg.fps)
            + " quality=" + (cfg.quality == RecordQuality::HdPostProcessing
                             ? "HD" : "Normal"));
}

void MainWindow::onRecordStop()
{
    if (m_finishing) return;                    // a save is already running
    if (!m_recording && !m_takePending) return;
    m_recording = false;
    finishRecording();
}

void MainWindow::finishRecording()
{
    if (m_finishing) return;          // guard re-entrancy via the modal loops below
    if (m_recBuffer.empty()) {
        m_takePending = false;
        if (m_hud) m_hud->hide();
        logTest("[rec] stop — empty buffer, nothing to save");
        return;
    }
    m_finishing   = true;
    m_takePending = true;             // hold the take until a save actually succeeds
    if (m_hud) m_hud->hide();         // not actively recording during the save dialogs

    // On any cancel/failure: re-show the HUD (relabelled) and clear the in-flight
    // flag so the operator can retry via Stop or discard via window-close —
    // instead of the take vanishing silently.
    auto keepUnsavedTake = [this](const char* logMsg) {
        m_finishing = false;
        if (m_hud) {
            m_hud->setFormatLabel(Lang::t("rec_unsaved"));
            m_hud->show(); m_hud->raise(); layoutHud();
        }
        logTest(logMsg);
    };

    // --- Optional HD post-processing pass -----------------------------------
    if (m_recCfg.quality == RecordQuality::HdPostProcessing) {
        auto computeMetrics = [&](const std::vector<RecordedFrame>& fr,
                                  double& maxAngDelta, double& rootPathLen,
                                  double& footLowest) {
            maxAngDelta = 0.0;
            rootPathLen = 0.0;
            footLowest  = 1e9;
            for (size_t i = 1; i < fr.size(); ++i) {
                for (int s = 0; s < kXsensSegmentCount; ++s) {
                    double d = std::abs(fr[i-1].segQuat[s].w * fr[i].segQuat[s].w
                                      + fr[i-1].segQuat[s].x * fr[i].segQuat[s].x
                                      + fr[i-1].segQuat[s].y * fr[i].segQuat[s].y
                                      + fr[i-1].segQuat[s].z * fr[i].segQuat[s].z);
                    if (d > 1.0) d = 1.0;
                    const double ang = 2.0 * std::acos(d);
                    if (ang > maxAngDelta) maxAngDelta = ang;
                }
                rootPathLen += (fr[i].pelvisPos - fr[i-1].pelvisPos).length();
            }
            for (const auto& f : fr) {
                if (f.pelvisPos.z() < footLowest) footLowest = f.pelvisPos.z();
            }
        };
        double mA0=0, rL0=0, fL0=0, mA1=0, rL1=0, fL1=0;
        computeMetrics(m_recBuffer, mA0, rL0, fL0);

        QProgressDialog dlg(Lang::t("rec_hd_progress"), Lang::t("cancel"),
                            0, 100, this);
        dlg.setWindowTitle(Lang::t("rec_wiz_title"));
        dlg.setWindowModality(Qt::ApplicationModal);
        dlg.setMinimumDuration(0);
        dlg.setAutoClose(false);
        dlg.setValue(0);
        dlg.show();
        QCoreApplication::processEvents();
        int recFps = int(m_procRateHz);
        if (m_recBuffer.size() >= 2) {
            const double totalSec = m_recBuffer.back().t - m_recBuffer.front().t;
            if (totalSec > 1e-3) {
                recFps = int(std::round(double(m_recBuffer.size() - 1) / totalSec));
                recFps = std::max(15, std::min(480, recFps));
            }
        }
        runHdPostProcessing(m_recBuffer, recFps, m_skel.get(),
            [&](int p) { dlg.setValue(p); QCoreApplication::processEvents(); },
            [&]{ return dlg.wasCanceled(); });
        const bool hdCancelled = dlg.wasCanceled();
        dlg.close();
        if (hdCancelled) {
            QMessageBox::warning(this, Lang::t("rec_save_title"), Lang::t("rec_take_kept"));
            keepUnsavedTake("[rec] HD post-processing cancelled — take kept (not saved)");
            return;
        }

        computeMetrics(m_recBuffer, mA1, rL1, fL1);
        std::ostringstream hd;
        hd << "[rec][hd] N=" << m_recBuffer.size()
           << "  maxAngDelta " << (mA0 * 180.0 / M_PI) << "° → "
                               << (mA1 * 180.0 / M_PI) << "°"
           << "  rootPath "    << rL0 << "m → " << rL1 << "m"
           << "  pelvisMinZ "  << fL0 << "m → " << fL1 << "m";
        logTest(hd.str());
    }

    // --- Resample the raw buffer down to the operator-selected fps ---------
    // Raw samples arrive at ~240 Hz; the BVH / FBX headers declare a fixed
    // step based on cfg.fps.  We pick the nearest raw frame to each target
    // tick instead of averaging — averaging across quaternions needs SLERP
    // and the HD pass already removed the jitter we'd otherwise smooth.
    std::vector<RecordedFrame> out;
    out.reserve(size_t(m_recBuffer.back().t * m_recCfg.fps) + 4);
    const double step = 1.0 / double(m_recCfg.fps);
    size_t src = 0;
    for (double t = 0.0; t <= m_recBuffer.back().t + 1e-6; t += step) {
        while (src + 1 < m_recBuffer.size() && m_recBuffer[src+1].t <= t)
            ++src;
        const RecordedFrame& A = m_recBuffer[src];
        const RecordedFrame& B = (src + 1 < m_recBuffer.size())
                                 ? m_recBuffer[src + 1] : A;
        const double dtAB = B.t - A.t;
        const double u    = (dtAB > 1e-9)
                            ? std::clamp((t - A.t) / dtAB, 0.0, 1.0)
                            : 0.0;
        RecordedFrame fr;
        fr.t = t;
        fr.pelvisPos = A.pelvisPos + float(u) * (B.pelvisPos - A.pelvisPos);
        for (int s = 0; s < kXsensSegmentCount; ++s)
            fr.segQuat[s] = slerp_quat(A.segQuat[s], B.segQuat[s], u);
        fr.hasGloves = A.hasGloves && B.hasGloves;
        if (fr.hasGloves) {
            for (int j = 0; j < kFingerSegmentsHand; ++j) {
                fr.rightGloveQ[j] = slerp_quat(A.rightGloveQ[j], B.rightGloveQ[j], u);
                fr.leftGloveQ[j]  = slerp_quat(A.leftGloveQ[j],  B.leftGloveQ[j],  u);
            }
        }
        out.push_back(std::move(fr));
    }

    // --- Ask the operator where to drop the file ---------------------------
    const bool bvh = (m_recCfg.format == RecordFormat::BVH);
    const QString filter = bvh ? "BVH (*.bvh)"
                               : "FBX (ASCII) (*.fbx)";
    const QString defSuffix = bvh ? ".bvh" : ".fbx";
    QString suggest = QDir::home().filePath(
        QString("fox_mocap_%1%2")
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"))
            .arg(defSuffix));
    QString path = QFileDialog::getSaveFileName(
        this, Lang::t("rec_save_title"), suggest, filter);
    if (path.isEmpty()) {
        QMessageBox::warning(this, Lang::t("rec_save_title"), Lang::t("rec_take_kept"));
        keepUnsavedTake("[rec] save cancelled — take kept (not saved)");
        return;
    }
    if (!path.endsWith(defSuffix, Qt::CaseInsensitive))
        path += defSuffix;

    const double h = m_setup.heightCm / 100.0;
    if (!m_skel) {
        QMessageBox::warning(this, Lang::t("rec_save_title"),
                             Lang::t("rec_save_failed"));
        keepUnsavedTake("[rec] no skeleton — cannot save, take kept");
        return;
    }
    const bool ok = bvh ? writeBvh(path, out, m_recCfg.fps, h, *m_skel)
                        : writeFbxAscii(path, out, m_recCfg.fps, h, *m_skel);
    if (!ok) {
        QMessageBox::warning(this, Lang::t("rec_save_title"),
                             Lang::t("rec_save_failed"));
        keepUnsavedTake("[rec] write failed — take kept");
        return;
    }
    QMessageBox::information(this, Lang::t("rec_save_title"),
                             Lang::t("rec_save_ok") + "\n" + path);
    logTest("[rec] saved " + std::to_string(out.size())
            + " frames → " + path.toStdString());
    m_recBuffer.clear();
    m_takePending = false;
    m_finishing   = false;
    if (m_hud) m_hud->hide();
}

void MainWindow::layoutHud()
{
    if (!m_viewport) return;
    // RecordHud stays top-RIGHT with stop button (if recording).
    if (m_hud) {
        const QPoint tl = m_viewport->mapTo(centralWidget(), QPoint(0, 0));
        const int x = tl.x() + m_viewport->width() - m_hud->width() - 16;
        const int y = tl.y() + 16;
        m_hud->move(std::max(0, x), std::max(0, y));
    }
    // Mode HUD (REC/STREAM + seconds) in top-LEFT, created on demand.
    if (!m_modeHud) {
        m_modeHud = new QLabel(centralWidget());
        m_modeHud->setStyleSheet(
            "background: rgba(20,20,20,220); color: #E04040;"
            " padding: 6px 12px; border-radius: 8px;"
            " font-weight: 900; font-size: 12pt;"
            " border: 1px solid #3a3a3a;");
        m_modeHud->hide();
        m_modeHud->raise();
        m_modeHudTimer = new QTimer(this);
        m_modeHudTimer->setInterval(250);
        connect(m_modeHudTimer, &QTimer::timeout, this, [this]() {
            const bool streaming = (m_streamer && m_streamer->isRunning());
            const bool recording = m_recording;
            if (!streaming && !recording) {
                m_modeHud->hide();
                return;
            }
            const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
            qint64 elapsedMs = 0;
            QString mode;
            QString color;
            if (recording && streaming) {
                mode = "REC+STREAM";
                elapsedMs = nowMs - std::min(m_recStartMs, m_streamStartMs);
                color = "#E04040";
            } else if (recording) {
                mode = "● REC";
                elapsedMs = nowMs - m_recStartMs;
                color = "#E04040";
            } else {
                mode = "● LIVE";
                elapsedMs = nowMs - m_streamStartMs;
                color = "#2EC25A";
            }
            const int total = int(elapsedMs / 1000);
            const int h = total / 3600;
            const int m = (total % 3600) / 60;
            const int s = total % 60;
            const QString dot = QString::fromUtf8("\xE2\x97\x8F");   // ●
            // Blink the dot every second — professional "live" feel.
            const bool blink = (total % 2 == 0);
            m_modeHud->setText(QString("<span style='color:%1'>%2</span>"
                                       "&nbsp;&nbsp;<b>%3</b>&nbsp;&nbsp;"
                                       "<span style='font-family:Consolas,monospace;"
                                       "font-size:13pt;color:#F5F5F5;"
                                       "letter-spacing:2px'>%4:%5:%6</span>")
                .arg(blink ? color : "#555555")
                .arg(dot)
                .arg(mode.contains("REC") ? "REC" :
                     mode.contains("LIVE") ? "LIVE" : "REC·LIVE")
                .arg(h, 2, 10, QChar('0'))
                .arg(m, 2, 10, QChar('0'))
                .arg(s, 2, 10, QChar('0')));
            m_modeHud->setTextFormat(Qt::RichText);
            m_modeHud->setStyleSheet(QString(
                "background: qlineargradient("
                " x1:0,y1:0,x2:0,y2:1,"
                " stop:0 rgba(18,18,18,235), stop:1 rgba(10,10,10,235));"
                "color: #EAEAEA;"
                "padding: 8px 18px;"
                "border-radius: 14px;"
                "font-weight: 800; font-size: 11pt;"
                "border: 1px solid %1;").arg(color));
            m_modeHud->adjustSize();
            const QPoint tl = m_viewport->mapTo(centralWidget(), QPoint(0, 0));
            m_modeHud->move(tl.x() + 16, tl.y() + 16);
            m_modeHud->show();
            m_modeHud->raise();
        });
        m_modeHudTimer->start();
    }
    if (m_modeHud && m_modeHud->isVisible()) {
        const QPoint tl = m_viewport->mapTo(centralWidget(), QPoint(0, 0));
        m_modeHud->move(tl.x() + 16, tl.y() + 16);
        m_modeHud->raise();
    }
}

void MainWindow::resizeEvent(QResizeEvent* e)
{
    QMainWindow::resizeEvent(e);
    layoutHud();
}

void MainWindow::logTest(const std::string& msg) const
{
    testLog(msg, m_test);
}

void MainWindow::closeEvent(QCloseEvent* e)
{
    // A save is mid-flight (modal dialogs up) — don't tear down under it.
    if (m_finishing) { e->ignore(); return; }
    // Don't silently drop an in-progress OR unsaved take when the window closes.
    if (m_recording || m_takePending) {
        const auto btn = QMessageBox::question(
            this, Lang::t("rec_wiz_title"), Lang::t("rec_close_prompt"),
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
            QMessageBox::Save);
        if (btn == QMessageBox::Cancel) { e->ignore(); return; }
        if (btn == QMessageBox::Save) {
            onRecordStop();              // m_recording=false, run save dialog
            if (m_takePending) { e->ignore(); return; }  // save cancelled — stay open
        } else {                         // Discard
            m_recording   = false;
            m_takePending = false;
            if (m_hud) m_hud->hide();
            m_recBuffer.clear();
        }
    }
    if (m_rx) { m_rx->stop(); m_rx->wait(1500); }
    QMainWindow::closeEvent(e);
}

// ============================================================================
//  Black / orange stylesheet
// ============================================================================

const char* kStyleSheet = R"(
  QMainWindow, QDialog { background: #0E0E0E; }
  QWidget           { color: #EAEAEA; font-family: 'Segoe UI', sans-serif; font-size: 10pt; }
  QWidget#sidePanel { background: #141414; border-right: 1px solid #1F1F1F; }

  /* Suit-status summary card (top of the side panel). */
  QWidget#statusCard { background: #181818; border: 1px solid #242424;
                       border-radius: 10px; }

  /* Calibration pose illustration — framed so the heading and hint never
     crash into the image edge. */
  QWidget#poseFrame  { background: #121212; border: 1px solid #262626;
                       border-radius: 14px; }

  /* Per-sensor indicator cards — state flips via dynamic property. */
  QWidget#sensorCard { background: #161616; border: 1px solid #242424;
                       border-radius: 8px; }
  QWidget#sensorCard[sensorState="on"]  { background: #16281C;
                                          border: 1px solid #2EC25A; }
  QWidget#sensorCard[sensorState="off"] { background: #161616;
                                          border: 1px solid #242424; }

  /* Live / Record top tabs — each pill opens a popup menu on click. */
  QToolButton#topTabBtn        { background: qlineargradient(
                                   x1:0,y1:0,x2:0,y2:1,
                                   stop:0 #1D1D1D, stop:1 #121212);
                                 color: #EAEAEA;
                                 padding: 6px 22px; min-width: 140px;
                                 min-height: 32px; border-radius: 10px;
                                 border: 1px solid #2A2A2A;
                                 font-weight: 700; font-size: 10pt;
                                 letter-spacing: 0.5px; }
  QToolButton#topTabBtn:hover  { color: #FFFFFF; border-color: #FF7A1A;
                                 background: qlineargradient(
                                   x1:0,y1:0,x2:0,y2:1,
                                   stop:0 #262626, stop:1 #181818); }
  QToolButton#topTabBtn:pressed,
  QToolButton#topTabBtn::menu-button,
  QToolButton#topTabBtn[popupMode="2"] {
                                 background: qlineargradient(
                                   x1:0,y1:0,x2:0,y2:1,
                                   stop:0 #2A1810, stop:1 #1A0F07);
                                 color: #FF9340;
                                 border: 1px solid #FF7A1A; }
  QToolButton#topTabBtn::menu-indicator { image: none; width: 0; }

  /* Popup menus opened from the top tabs. */
  QMenu#topTabMenu             { background: #141414; color: #EAEAEA;
                                 border: 1px solid #2A2A2A; border-radius: 8px;
                                 padding: 6px; }
  QMenu#topTabMenu::item       { padding: 10px 22px; border-radius: 6px;
                                 font-weight: 600; font-size: 10pt; }
  QMenu#topTabMenu::item:selected { background: #1F1F1F; color: #FF9340; }
  QMenu#topTabMenu::separator  { height: 1px; background: #2A2A2A;
                                 margin: 4px 8px; }

  QLabel#subtle     { color: #8A8A8A; }
  QLabel#sectionHeader { color: #FF7A1A; font-weight: 700; font-size: 10pt;
                         text-transform: uppercase; letter-spacing: 1px;
                         border-bottom: 1px solid #2A2A2A; padding-bottom: 3px; }
  QLabel#heroHeading { color: #FF7A1A; font-size: 22pt; font-weight: 800;
                       letter-spacing: 1px; padding: 6px 0; }
  QLabel#countdown  { color: #FF7A1A; font-size: 52pt; font-weight: 900;
                       padding: 10px 0; letter-spacing: 2px; }

  QPushButton#hero  { background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                                  stop:0 #FF9240, stop:1 #FF7A1A);
                      color: #111; border: 0; border-radius: 30px;
                      padding: 18px 48px;
                      font-size: 14pt; font-weight: 800; letter-spacing: 1px; }
  QPushButton#hero:hover    { background: #FFA053; }
  QPushButton#hero:pressed  { background: #E56A14; }
  QPushButton#hero:disabled { background: #553211; color: #999; }

  QRadioButton#bigRadio { padding: 14px 20px; font-size: 12pt; spacing: 14px;
                          background: #181818; border: 1px solid #242424;
                          border-radius: 10px; min-width: 340px; }
  QRadioButton#bigRadio::indicator { width: 20px; height: 20px; border-radius: 11px;
                                     border: 2px solid #FF7A1A; }
  QRadioButton#bigRadio::indicator:checked { background: #FF7A1A; }

  QGroupBox         { background: #181818; border: 1px solid #262626;
                      border-radius: 8px; margin-top: 10px; padding: 10px 8px 6px; }
  QGroupBox::title  { color: #FF7A1A; subcontrol-origin: margin; left: 10px;
                      padding: 0 6px; font-weight: 600; }

  QPushButton       { background: #1F1F1F; color: #EAEAEA; border: 1px solid #2B2B2B;
                      border-radius: 6px; padding: 6px 14px; font-weight: 600; }
  QPushButton:hover { background: #2A2A2A; border-color: #FF7A1A; color: #FFFFFF; }
  QPushButton:pressed { background: #151515; }
  QPushButton:disabled{ background: #161616; color: #555; border-color: #222; }
  QPushButton#primary       { background: #FF7A1A; color: #111; border: 0; }
  QPushButton#primary:hover { background: #FF9340; }
  QPushButton#primary:disabled { background: #553211; color: #999; }

  QComboBox, QDoubleSpinBox, QSpinBox {
      background: #151515; color: #EAEAEA; border: 1px solid #2A2A2A;
      border-radius: 6px; padding: 4px 8px; min-height: 22px;
  }
  QComboBox:hover, QDoubleSpinBox:hover, QSpinBox:hover { border-color: #FF7A1A; }
  QDoubleSpinBox#bigSpin, QSpinBox#bigSpin {
      background: #121212; border: 2px solid #2A2A2A; border-radius: 12px;
      padding: 6px 16px; font-size: 16pt; font-weight: 700;
      color: #FF9340; min-height: 40px;
  }
  QDoubleSpinBox#bigSpin:focus, QSpinBox#bigSpin:focus { border-color: #FF7A1A; }
  QDoubleSpinBox#bigSpin::up-button, QSpinBox#bigSpin::up-button,
  QDoubleSpinBox#bigSpin::down-button, QSpinBox#bigSpin::down-button {
      width: 26px; background: #1E1E1E; border-left: 1px solid #2A2A2A;
  }
  QDoubleSpinBox#bigSpin::up-button:hover, QSpinBox#bigSpin::up-button:hover,
  QDoubleSpinBox#bigSpin::down-button:hover, QSpinBox#bigSpin::down-button:hover {
      background: #FF7A1A;
  }
  QComboBox QAbstractItemView { background: #151515; selection-background-color: #FF7A1A;
                                selection-color: #111; border: 1px solid #2A2A2A; }

  QRadioButton           { spacing: 8px; padding: 2px; }
  QRadioButton::indicator{ width: 14px; height: 14px; border: 2px solid #FF7A1A;
                           border-radius: 8px; background: #0E0E0E; }
  QRadioButton::indicator:checked { background: #FF7A1A; }

  QProgressBar       { background: #141414; border: 1px solid #2A2A2A; border-radius: 8px;
                       text-align: center; color: #FFFFFF; height: 22px;
                       font-weight: 700; }
  QProgressBar::chunk{ background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                                   stop:0 #FF7A1A, stop:1 #FFA053);
                       border-radius: 6px; margin: 1px; }
  /* Countdown = warm amber gradient (prep / "get ready"). */
  QProgressBar#countdownBar        { border-color: #4A2A10; }
  QProgressBar#countdownBar::chunk { background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                                 stop:0 #FFB050, stop:1 #FF7A1A); }
  /* Readiness = cool teal-green gradient (capture / "we are recording"). */
  QProgressBar#readyBar            { border-color: #14402A; }
  QProgressBar#readyBar::chunk     { background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                                 stop:0 #2EC25A, stop:1 #6BE090); }

  QStatusBar  { background: #0A0A0A; color: #9B9B9B;
                border-top: 1px solid #1A1A1A; padding: 3px 10px; }
  QScrollBar:vertical { background: #0E0E0E; width: 10px; }
  QScrollBar::handle:vertical { background: #2A2A2A; border-radius: 5px; }
  QScrollBar::handle:vertical:hover { background: #FF7A1A; }
  QScrollBar:horizontal { background: #0E0E0E; height: 10px; }
  QScrollBar::handle:horizontal { background: #2A2A2A; border-radius: 5px; }
  QScrollBar::handle:horizontal:hover { background: #FF7A1A; }
  QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }

  /* Joint-correction sliders (Settings window). */
  QSlider::groove:horizontal { height: 4px; background: #2A2A2A; border-radius: 2px; }
  QSlider::sub-page:horizontal { background: #3A2A1A; border-radius: 2px; }
  QSlider::add-page:horizontal { background: #2A2A2A; border-radius: 2px; }
  QSlider::handle:horizontal { width: 14px; height: 14px; margin: -6px 0;
                               border-radius: 7px; background: #FF7A1A;
                               border: 1px solid #FF9340; }
  QSlider::handle:horizontal:hover { background: #FF9340; }

  /* ---- Language selector on the welcome page ------------------------- */
  QComboBox#langCombo {
      background: #141414; border: 1px solid #2A2A2A; border-radius: 10px;
      padding: 6px 12px; font-size: 11pt; font-weight: 600;
      color: #EAEAEA;
  }
  QComboBox#langCombo:hover { border-color: #FF7A1A; }
  QComboBox#langCombo::drop-down { border: 0; width: 22px; }
  QComboBox#langCombo::down-arrow { image: none; }
  QComboBox#langCombo QAbstractItemView {
      background: #141414; color: #EAEAEA;
      border: 1px solid #2A2A2A; border-radius: 8px;
      padding: 4px; outline: 0;
  }
  QComboBox#langCombo QAbstractItemView::item {
      padding: 8px 10px; border-radius: 6px; min-height: 28px;
  }
  QComboBox#langCombo QAbstractItemView::item:selected {
      background: #1F1F1F; color: #FF9340;
  }

  /* ---- Actor-dimension inputs: behave like text fields ---------------- */
  /* Keep the arrows small; emphasize the typing affordance — thick focus
     border, big font, centred caret, visible text-selection colour. */
  QDoubleSpinBox#bigSpin, QSpinBox#bigSpin {
      selection-background-color: #FF7A1A;
      selection-color: #111;
  }
  QDoubleSpinBox#bigSpin:focus, QSpinBox#bigSpin:focus {
      border: 2px solid #FF7A1A;
      background: #161616;
  }
  QDoubleSpinBox#bigSpin:hover, QSpinBox#bigSpin:hover {
      border-color: #3A3A3A;
  }

  /* ---- BigRadio (mode page) gets a hover+checked affordance ---------- */
  QRadioButton#bigRadio:hover     { background: #1E1E1E;
                                    border-color: #FF7A1A; }
  QRadioButton#bigRadio:checked   { background: #1E1E1E;
                                    border: 1px solid #FF7A1A; }

  /* ---- Hero button glow ---------------------------------------------- */
  QPushButton#hero:focus { outline: 0; }

  /* ---- Dialog + tooltip polish --------------------------------------- */
  QMessageBox { background: #111111; }
  QMessageBox QLabel { color: #EAEAEA; font-size: 10pt; }
  QToolTip { background: #141414; color: #EAEAEA;
             border: 1px solid #2A2A2A; border-radius: 4px;
             padding: 4px 8px; }

  /* ---- Sensor cards: richer typography with a per-state accent line --- */
  QWidget#sensorCard > QLabel { font-size: 10pt; }
  QWidget#sensorCard[sensorState="on"]  > QLabel { color: #FFFFFF; }
  QWidget#sensorCard[sensorState="off"] > QLabel { color: #AAAAAA; }

  /* Sub-label under each section header in the side panel. */
  QLabel#sectionSub { color: #6E6E6E; font-size: 8pt;
                      font-weight: 500; letter-spacing: 0.5px;
                      margin-top: -2px; margin-bottom: 4px; }
)";

// ============================================================================
//  CLI + test logger
// ============================================================================

CliArgs parseCli(int argc, char** argv)
{
    CliArgs out;
    // Track whether the suit was chosen explicitly so --link/--awinda override
    // -test's Link default regardless of flag ORDER (e.g. `-test -gloves -awinda`
    // and `-awinda -test` both select Awinda).
    bool suitExplicit = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-test" || a == "--test")          out.test = true;
        else if (a == "--gloves" || a == "-gloves") out.gloves = true;
        else if (a == "--wrist-constraint")         out.wristConstraint = true;
        else if (a == "--link"   || a == "-link")   { out.suit = SuitType::Link;   suitExplicit = true; }
        else if (a == "--awinda" || a == "-awinda") { out.suit = SuitType::Awinda; suitExplicit = true; }
        else if (a == "-h" || a == "--help") {
            std::cout <<
                "Fox Mocap — MVN-style Xsens client (Link 240 Hz / Awinda 60 Hz)\n"
                "Usage:\n"
                "  fox_mocap [-test] [--gloves] [--link|--awinda]\n"
                "\n"
                "  -test      Auto-run mode (defaults to --link): skips the session wizard,\n"
                "             waits for the XDA driver to report the suit is streaming,\n"
                "             then auto-runs calibration and starts the session.  All\n"
                "             state changes and sample-level dumps are logged to the\n"
                "             parent console (cmd/PowerShell) or to fox_mocap.log\n"
                "             when launched from Explorer.\n"
                "  --gloves   Reserve space in the session for Manus finger data.\n"
                "  --link     Xsens Link suit — 240 Hz update rate (default for -test).\n"
                "  --awinda   Xsens Awinda suit — 60 Hz update rate (default).\n"
                "             --link/--awinda override -test's default in ANY order,\n"
                "             so `-test -gloves -awinda` runs the 60 Hz Awinda suit.\n";
            std::exit(0);
        }
    }
    // -test defaults to the Link suit (240 Hz) UNLESS the user explicitly chose a
    // suit with --link/--awinda — order-independent (see suitExplicit above).
    if (out.test && !suitExplicit) out.suit = SuitType::Link;
    return out;
}

void testLog(const std::string& msg, bool enabled)
{
    if (!enabled) return;
    std::cout << msg << '\n';
    std::cout.flush();
}

// Windows-specific: the program is built with the WIN32 subsystem so a
// double-click from Explorer opens no console.  In -test mode we want logs
// regardless, so:
//   * if the process has a parent console  (cmd / PowerShell launch)
//     → attach to it and pipe std::cout there,
//   * otherwise (launched as GUI, no parent console)
//     → redirect std::cout to fox_mocap.log next to the exe.
static void attachTestOutput()
{
#ifdef _WIN32
    // In -test we always mirror stdout/stderr into fox_mocap.log next to the
    // exe.  No AttachConsole shenanigans — those silently broke output when
    // launched without a parent terminal (Start-Process, Explorer).
    char buf[MAX_PATH]{};
    GetModuleFileNameA(nullptr, buf, MAX_PATH);
    std::string exeDir(buf);
    const std::size_t slash = exeDir.find_last_of("\\/");
    if (slash != std::string::npos) exeDir.resize(slash);
    const std::string logPath = exeDir + "\\fox_mocap.log";
    FILE* f = nullptr;
    freopen_s(&f, logPath.c_str(), "w", stdout);
    freopen_s(&f, logPath.c_str(), "a", stderr);
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);
    std::cout.clear();
    std::cerr.clear();
    std::cout.sync_with_stdio(true);
    std::cout << "[boot] log opened at " << logPath << '\n';
    std::cout.flush();
#endif
}

}  // namespace fox

// ============================================================================
//  Entry point
// ============================================================================

int main(int argc, char** argv)
{
    using namespace fox;

    const CliArgs cli = parseCli(argc, argv);
    if (cli.test) {
        attachTestOutput();
        // Make the active suit + native update rate the first thing in the log:
        // every downstream formula (filter time-constants, rate-adjusted loco
        // thresholds, dt) depends on it.  240 Hz Link vs 60 Hz Awinda.
        std::cout << "[boot] suit=" << (cli.suit == SuitType::Link ? "Link" : "Awinda")
                  << " nativeRate=" << nativeRateHz(cli.suit) << "Hz"
                  << " gloves=" << (cli.gloves ? "on" : "off")
                  << " wristConstraint=" << (cli.wristConstraint ? "on" : "off") << "\n";
        std::cout.flush();
    }

    // Request a compatibility-profile OpenGL context so legacy immediate-mode
    // GL (used by the viewport line/point renderer) works.  Must be set BEFORE
    // QApplication is constructed.
    {
        QSurfaceFormat fmt;
        fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
        fmt.setVersion(3, 3);
        fmt.setDepthBufferSize(24);
        fmt.setSamples(4);                 // 4× MSAA for smooth skeleton lines
        QSurfaceFormat::setDefaultFormat(fmt);
    }

    QApplication app(argc, argv);
    app.setStyleSheet(kStyleSheet);
    app.setApplicationName("Fox-Mocap");
    app.setApplicationVersion("0.1");
    // Stylised fox icon used in title bar, taskbar, Alt+Tab, dialogs.
    const QIcon foxIcon = makeFoxAppIcon();
    app.setWindowIcon(foxIcon);

    testLog(std::string("[boot] fox_mocap starting, test_mode=")
            + (cli.test ? "true" : "false"), cli.test);

    // The XDA receiver is constructed now but NOT started.  The wizard's
    // Mode page has a "Connect suit" button that triggers scan on demand so
    // the user sees explicit progress instead of a silent background poll.
    auto* rx = new MocapReceiver(cli.test, &app);

    // ---- New-session wizard -----------------------------------------------
    NewSessionWizard wiz(rx, cli.test);
    // Pre-select the suit (and its update rate): -test ⇒ Link, else the
    // --link/--awinda flag or the Awinda default.
    testLog(std::string("[boot] suit = ")
            + (cli.suit == SuitType::Link ? "Xsens Link (240 Hz)" : "Xsens Awinda (60 Hz)"),
            cli.test);
    wiz.preselectSuit(cli.suit);
    if (cli.gloves) {
        testLog("[boot] --gloves flag set — pre-selecting suit+gloves mode", cli.test);
        wiz.preselectGloves(true);
    }
    (void)wiz.winId();
    applyDarkTitleBar(&wiz);
    if (wiz.exec() != QDialog::Accepted) {
        rx->stop(); rx->wait(2000);
        return 0;
    }
    const auto result = wiz.result();
    testLog(QString("[session] gloves=%1 h=%2cm foot=%3cm pose=%4")
                .arg(result.useGloves ? "yes" : "no")
                .arg(result.heightCm, 0, 'f', 1)
                .arg(result.footLengthCm, 0, 'f', 1)
                .arg(QString::fromStdString(result.poseKind))
                .toStdString(), cli.test);

    // ---- Main window ------------------------------------------------------
    auto* win = new MainWindow(rx, result, cli.test);
    if (cli.wristConstraint) {
        win->setWristConstraintEnabled(true);
    }
    win->show();
    applyDarkTitleBar(win);

    const int rc = app.exec();
    delete win;
    return rc;
}
