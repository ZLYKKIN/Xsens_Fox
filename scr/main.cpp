
#include "main.h"
#include "foxwire.h"
#include "foxbody.h"
#include "foxergo.h"

#include <onnxruntime_cxx_api.h>

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
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QProgressDialog>
#include <QtCore/QFile>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QtCore/QTextStream>
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

extern "C" {
#include "fusion/Fusion.h"
}

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
#include <atomic>
#include <mutex>
#include <algorithm>
#include <deque>
#include <vector>

#include <Eigen/Dense>

namespace fox {

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

const char* kSegmentNames[kXsensSegmentCount] = {
    "pelvis", "l5", "l3", "t12", "t8", "neck", "head",
    "r_shoulder", "r_upper_arm", "r_forearm", "r_hand",
    "l_shoulder", "l_upper_arm", "l_forearm", "l_hand",
    "r_upper_leg", "r_lower_leg", "r_foot", "r_toe",
    "l_upper_leg", "l_lower_leg", "l_foot", "l_toe",
};

const int kFingerChains[kFingerChainCount][kFingerChainLen] = {
    { 0,  1,  2,  3 },
    { 4,  5,  6,  7 },
    { 8,  9, 10, 11 },
    { 12, 13, 14, 15 },
    { 16, 17, 18, 19 },
};
const char* kFingerChainNames[kFingerChainCount] = {
    "thumb", "index", "middle", "ring", "pinky",
};

SkeletonXsens::SkeletonXsens(const ActorConfig& actor, const std::string& pose)
    : m_pose(pose)
{
    buildTopology();
    buildDefaultAngles();
    buildLengths(actor);
}

void SkeletonXsens::buildTopology()
{

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

namespace pose_solver {

using fox::Quat;
using fox::quat_mult;
using fox::quat_exp_rotvec;
using fox::quat_log;
using fox::vec_rotate;
namespace fb = fox::body;

constexpr double kR2D = fb::constants::kRad2Deg;
constexpr double kD2R = fb::constants::kDeg2Rad;

inline std::atomic<bool>& g_testFlag();
inline std::atomic<bool>& g_glovesFlag();

inline double sigmoid(double x) {
    if (x >  40.0) return 1.0;
    if (x < -40.0) return 0.0;
    return 1.0 / (1.0 + std::exp(-x));
}

class SkinArtifactState {
public:
    void reset() {
        for (auto& x : m_x)   x = QVector3D(0, 0, 0);
        for (auto& p : m_var) p = (fb::kSkin.sigmaOriDeg * kD2R) *
                                   (fb::kSkin.sigmaOriDeg * kD2R);
        for (auto& x : m_xPos)   x = QVector3D(0, 0, 0);
        for (auto& p : m_varPos) p = fb::kSkin.sigmaPosM * fb::kSkin.sigmaPosM;
        m_inited = true;
    }

    void predict(int seg, double dt, double omegaRad = 0.0) {
        if (seg < 0 || seg >= fb::kSegmentCount) return;
        if (!m_inited) reset();
        double tauEff = fb::kSkin.tauSec;
        if (fb::kSkin.doSkinArtifactBasedOnDynamics && omegaRad > 0.0) {
            const double ref   = std::max(1e-6, fb::kSkin.tauMotionRefRad);
            const double blend = 1.0 - std::exp(-omegaRad / ref);
            tauEff = fb::kSkin.tauSlowSec
                   + blend * (fb::kSkin.tauFastSec - fb::kSkin.tauSlowSec);
        }
        m_tauLast[seg] = tauEff;
        const double a = std::exp(-dt / std::max(1e-6, tauEff));
        m_x[seg]   = m_x[seg] * float(a);
        const double sigmaOriRad = fb::kSkin.sigmaOriDeg * kD2R;
        const double sigmaEta2   = sigmaOriRad * sigmaOriRad * (1.0 - a * a);
        m_var[seg] = a * a * m_var[seg] + sigmaEta2;
    }

    double tauLast(int seg) const {
        if (seg < 0 || seg >= fb::kSegmentCount) return fb::kSkin.tauSec;
        return m_tauLast[seg] > 0.0 ? m_tauLast[seg] : fb::kSkin.tauSec;
    }

    void predictPos(int seg, double dt, double linAccMag = 0.0) {
        if (seg < 0 || seg >= fb::kSegmentCount) return;
        if (!m_inited) reset();
        double tauEff = fb::kSkin.tauSec;
        if (fb::kSkin.doSkinArtifactBasedOnDynamics && linAccMag > 0.0) {
            const double ref   = std::max(1e-6, fb::kSkin.linAccRefMps2);
            const double blend = 1.0 - std::exp(-linAccMag / ref);
            tauEff = fb::kSkin.tauSlowSec
                   + blend * (fb::kSkin.tauFastSec - fb::kSkin.tauSlowSec);
        }
        const double a = std::exp(-dt / std::max(1e-6, tauEff));
        m_xPos[seg] = m_xPos[seg] * float(a);
        const double sigmaPos  = fb::kSkin.sigmaPosM;
        const double sigmaEta2 = sigmaPos * sigmaPos * (1.0 - a * a);
        m_varPos[seg] = a * a * m_varPos[seg] + sigmaEta2;
    }

    void updatePos(int seg, const QVector3D& rMeas, double R_pos) {
        if (seg < 0 || seg >= fb::kSegmentCount) return;
        if (!m_inited) reset();
        const double R = R_pos + 1e-12;
        const double K = m_varPos[seg] / (m_varPos[seg] + R);
        m_xPos[seg] = m_xPos[seg] + float(K) * rMeas;
        m_varPos[seg] = (1.0 - K) * m_varPos[seg];
    }

    QVector3D driftPos(int seg) const {
        if (seg < 0 || seg >= fb::kSegmentCount) return QVector3D(0, 0, 0);
        return m_inited ? m_xPos[seg] : QVector3D(0, 0, 0);
    }

    QVector3D applyPosTo(int seg, const QVector3D& r_local,
                         const Quat& q_seg_world) const {
        if (seg < 0 || seg >= fb::kSegmentCount) return r_local;
        if (!m_inited) return r_local;
        const QVector3D drift = m_xPos[seg];
        if (drift.length() < 1e-6) return r_local;
        return r_local + vec_rotate(drift, q_seg_world);
    }

    void update(int seg, const QVector3D& r, double R_orient) {
        if (seg < 0 || seg >= fb::kSegmentCount) return;
        if (!m_inited) reset();
        const double R = R_orient + 1e-9;
        const double K = m_var[seg] / (m_var[seg] + R);
        m_x[seg]   = m_x[seg] + float(K) * r;
        m_var[seg] = (1.0 - K) * m_var[seg];
    }

    QVector3D drift(int seg) const {
        if (seg < 0 || seg >= fb::kSegmentCount) return QVector3D(0, 0, 0);
        return m_inited ? m_x[seg] : QVector3D(0, 0, 0);
    }
    double variance(int seg) const {
        if (seg < 0 || seg >= fb::kSegmentCount) return 0.0;
        return m_inited ? m_var[seg]
                        : (fb::kSkin.sigmaOriDeg * kD2R) *
                          (fb::kSkin.sigmaOriDeg * kD2R);
    }

    Quat applyTo(int seg, const Quat& q) const {
        if (seg < 0 || seg >= fb::kSegmentCount) return q;
        if (!m_inited) return q;
        const QVector3D x = m_x[seg];
        const Quat dq = quat_exp_rotvec(double(x.x()), double(x.y()), double(x.z()));
        return quat_mult(q, dq).normalized();
    }

private:
    bool m_inited = false;
    std::array<QVector3D, fb::kSegmentCount> m_x{};
    std::array<double,    fb::kSegmentCount> m_var{};
    std::array<double,    fb::kSegmentCount> m_tauLast{};

    std::array<QVector3D, fb::kSegmentCount> m_xPos{};
    std::array<double,    fb::kSegmentCount> m_varPos{};
};

struct ActiveContact {
    int       seg;
    int       pointId;
    QVector3D r_local;
    QVector3D p_world;
    QVector3D v_world;
    double    probability;
    double    sd_height;
    double    z_floor;
};

// §XIV/§52 детектор контакта стопы с полом и ZUPT-якорей: вероятностная модель
//   (contactParameters §138) + перекат пятка->носок (§49), multiLevel-лестница (§XIV) (formules.txt)
class ContactDetector {
public:
    struct FrameInput {
        const std::array<Quat, fb::kSegmentCount>* worldOrient;
        const std::array<QVector3D, fb::kSegmentCount>* segCenter;
        const std::array<QVector3D, fb::kSegmentCount>* segVelocity;
        const std::array<QVector3D, fb::kSegmentCount>* segOmega;
        const std::array<QVector3D, fb::kSegmentCount>* accLPBody;
        const SkinArtifactState*                        skinState = nullptr;
        double                                          floorLevelZ;
        double                                          dt;
    };

    struct Result {
        std::vector<ActiveContact> active;
        std::array<double, 26>     allProbabilities{};
        bool                       impactDetected = false;
        int                        impactSeg = -1;
    };

    void reset() {
        m_accPeakWindow.clear();
        m_prevAccNorm = 0.0;
        m_floorEstimate = 0.0;
        m_floorInited = false;
        m_lastCoP = QVector3D(0, 0, 0);
    }

    Result detect(const FrameInput& in) {
        Result out;
        if (!in.worldOrient || !in.segCenter || !in.segVelocity ||
            !in.segOmega || !in.accLPBody) return out;

        double peakWindowAccMax = 0.0;
        double peakWindowAccMin = 1e9;
        int    peakWindowSeg    = -1;
        for (int seg : { fb::kFootContacts[0].seg, fb::kFootContacts[4].seg }) {
            if (seg < 0 || seg >= fb::kSegmentCount) continue;
            const double an = std::sqrt(double((*in.accLPBody)[seg].x()*(*in.accLPBody)[seg].x()) +
                                        double((*in.accLPBody)[seg].y()*(*in.accLPBody)[seg].y()) +
                                        double((*in.accLPBody)[seg].z()*(*in.accLPBody)[seg].z()));
            if (an > peakWindowAccMax) {
                peakWindowAccMax = an;
                peakWindowSeg    = seg;
            }
            peakWindowAccMin = std::min(peakWindowAccMin, an);
        }
        m_accPeakWindow.push_back(peakWindowAccMax);
        const size_t maxWin = size_t(fb::kPeakDetection[3]);
        while (m_accPeakWindow.size() > maxWin) m_accPeakWindow.pop_front();
        if (!m_accPeakWindow.empty()) {
            const double winMax = *std::max_element(m_accPeakWindow.begin(),
                                                    m_accPeakWindow.end());
            const double winMin = *std::min_element(m_accPeakWindow.begin(),
                                                    m_accPeakWindow.end());
            if ((winMax - winMin) > fb::kContact.impactTh) {
                out.impactDetected = true;
                out.impactSeg      = peakWindowSeg;
            }
        }

        std::array<double, 26> probs{};
        std::array<ActiveContact, 26> cands{};
        size_t nCand = 0;

        const QVector3D pelvisVel = (*in.segVelocity)[0];
        const double pelvisSpeed  = std::sqrt(double(pelvisVel.x()*pelvisVel.x()) +
                                              double(pelvisVel.y()*pelvisVel.y()) +
                                              double(pelvisVel.z()*pelvisVel.z()));

        auto pushPoint = [&](int seg, int pid, const QVector3D& r_local) {
            if (seg < 0 || seg >= fb::kSegmentCount) return;
            if (nCand >= cands.size()) return;
            const Quat& q = (*in.worldOrient)[seg];
            const QVector3D rWorldRot = vec_rotate(r_local, q);

            const QVector3D posDrift = in.skinState
                ? in.skinState->driftPos(seg) : QVector3D(0, 0, 0);
            const QVector3D p_world   = (*in.segCenter)[seg] + rWorldRot + posDrift;

            const QVector3D w = (*in.segOmega)[seg];
            const QVector3D wxr(
                float(double(w.y())*double(rWorldRot.z()) -
                      double(w.z())*double(rWorldRot.y())),
                float(double(w.z())*double(rWorldRot.x()) -
                      double(w.x())*double(rWorldRot.z())),
                float(double(w.x())*double(rWorldRot.y()) -
                      double(w.y())*double(rWorldRot.x())));
            const QVector3D v_world = (*in.segVelocity)[seg] + wxr;

            const double pSpeed = std::sqrt(double(v_world.x()*v_world.x()) +
                                            double(v_world.y()*v_world.y()) +
                                            double(v_world.z()*v_world.z()));
            const double aNorm  = std::sqrt(double((*in.accLPBody)[seg].x()*(*in.accLPBody)[seg].x()) +
                                            double((*in.accLPBody)[seg].y()*(*in.accLPBody)[seg].y()) +
                                            double((*in.accLPBody)[seg].z()*(*in.accLPBody)[seg].z()));

            const double f_acc = fb::kAccProb[0] +
                                 (fb::kAccProb[2] - fb::kAccProb[0]) *
                                 sigmoid((aNorm - fb::kAccProb[1]) /
                                         std::max(1e-6, fb::kAccProb[3]));

            const double f_vel = fb::kVelProb[2] -
                                 (fb::kVelProb[2] - fb::kVelProb[0]) *
                                 sigmoid((pSpeed - fb::kVelProb[1]) /
                                         std::max(1e-6, fb::kVelProb[3]));

            const QVector3D cop_xy(m_lastCoP.x(), m_lastCoP.y(), 0.0f);
            const QVector3D p_xy  (p_world.x(),    p_world.y(),    0.0f);
            const double dCom = (cop_xy - p_xy).length();
            const double f_com = fb::kCom[1] + (fb::kCom[4] - fb::kCom[1]) *
                                 sigmoid((dCom - fb::kCom[3]) /
                                         std::max(1e-6, fb::kCom[2]));

            const double lowZ        = std::max(0.0,
                fb::kDLevelDefault - (double(p_world.z()) - in.floorLevelZ));
            const double aLowThresh =
                fb::constants::kGravityMs2 + fb::kContact.impactTh / 8.0;
            const double f_lowFreq   = (aNorm < aLowThresh) ? 1.0 : 0.0;
            constexpr double kZuptStillOmegaRad = 0.3 * fb::constants::kDeg2Rad;
            const double f_lowOmega  = (std::abs(double(w.x())) +
                                        std::abs(double(w.y())) +
                                        std::abs(double(w.z()))
                                        < kZuptStillOmegaRad) ? 1.0 : 0.0;

            const double vNorm = std::sqrt(double(v_world.x()) * double(v_world.x()) +
                                           double(v_world.y()) * double(v_world.y()) +
                                           double(v_world.z()) * double(v_world.z()));
            const double wNorm = std::sqrt(double(w.x()) * double(w.x()) +
                                           double(w.y()) * double(w.y()) +
                                           double(w.z()) * double(w.z()));
            const double f_air = fb::kAir[0] +
                                 fb::kAir[1] * vNorm +
                                 fb::kAir[2] * (1.0 - std::min(1.0,
                                     std::abs(aNorm - fb::constants::kGravityMs2) / 4.0)) +
                                 fb::kAir[3] * f_lowFreq +
                                 fb::kAir[4] * f_lowOmega +
                                 fb::kAir[5] * std::abs(double(v_world.z())) +
                                 fb::kAir[6] * std::min(1.0, lowZ / fb::kDLevelFoot) +
                                 fb::kAir[7] * wNorm +
                                 fb::kAir[8] * std::min(1.0, pelvisSpeed / 2.0) +
                                 fb::kAir[9] * std::min(1.0, double(v_world.z())) +
                                 fb::kAir[10] * std::min(1.0, std::abs(double(v_world.z())) / 0.5);

            const double f_general = fb::kGeneralProb[0] +
                                     fb::kGeneralProb[1] *
                                         sigmoid(std::max(0.0, lowZ) /
                                                 std::max(1e-6, fb::kGeneralProb[2])) +
                                     fb::kGeneralProb[3] * std::max(0.0, lowZ);

            const double f_boost = fb::kBoost[0] * f_acc +
                                   fb::kBoost[1] * f_vel;

            const double f_pos = fb::kPos[0];

            const double f_level = fb::kLevelProb[0] *
                                   (1.0 - std::min(1.0,
                                       std::abs(double(p_world.z()) - in.floorLevelZ) /
                                       std::max(1e-6, fb::kDLevelDefault)));

            const double f_peak = (out.impactDetected && (seg == out.impactSeg))
                                ? fb::kPeakDetection[5]
                                : 0.0;

            const double samePosDist = (p_xy - cop_xy).length();
            const double f_samepos = fb::kSamepos[7] *
                                     std::exp(-samePosDist /
                                              std::max(1e-6, fb::kSamepos[6]));

            const double score = fb::kContactWeights.wAcc           * f_acc
                               + fb::kContactWeights.wVel           * f_vel
                               + fb::kContactWeights.wCom           * f_com
                               + fb::kContactWeights.wAir           * f_air
                               + fb::kContactWeights.wGeneral       * f_general
                               + fb::kContactWeights.wLevel         * f_level
                               + fb::kContactWeights.wBoost         * f_boost
                               + fb::kContactWeights.wPos           * f_pos
                               + fb::kContactWeights.wPeakDetection * f_peak
                               + fb::kContactWeights.wSamepos       * f_samepos
                               + fb::kContactWeights.bias;
            const double P = sigmoid(score);

            cands[nCand].seg          = seg;
            cands[nCand].pointId      = pid;
            cands[nCand].r_local      = r_local;
            cands[nCand].p_world      = p_world;
            cands[nCand].v_world      = v_world;
            cands[nCand].probability  = P;
            cands[nCand].sd_height    = fb::stdHeightMeasFor(seg);

            double zFloorSeg = in.floorLevelZ;
            if (seg == fb::kSEG_RFoot || seg == fb::kSEG_RToe ||
                seg == fb::kSEG_RLowerLeg) {
                zFloorSeg = floorLevelRight();
            } else if (seg == fb::kSEG_LFoot || seg == fb::kSEG_LToe ||
                       seg == fb::kSEG_LLowerLeg) {
                zFloorSeg = floorLevelLeft();
            }
            cands[nCand].z_floor      = zFloorSeg;

            probs[nCand]              = P;
            ++nCand;
        };

        for (const auto& fp : fb::kFootPointsRight) {
            pushPoint(17, fp.pointId, fp.r_local);
        }

        for (const auto& fp : fb::kFootPointsLeft) {
            pushPoint(21, fp.pointId, fp.r_local);
        }

        pushPoint(18, 2, fb::kToeTipPoint);
        pushPoint(22, 2,
                  QVector3D(fb::kToeTipPoint.x(), -fb::kToeTipPoint.y(),
                            fb::kToeTipPoint.z()));

        pushPoint(fb::kSEG_RLowerLeg, 5, fb::kKneeFrontPointR);
        pushPoint(fb::kSEG_LLowerLeg, 5, fb::kKneeFrontPointL);

        const double pelvisTiltDeg = [&]() {
            if (!in.worldOrient) return 0.0;
            const QVector3D zAxisLocal(0, 0, 1);
            const QVector3D zAxisWorld =
                vec_rotate(zAxisLocal, (*in.worldOrient)[fb::kSEG_Pelvis]);
            return std::acos(std::clamp(double(zAxisWorld.z()), -1.0, 1.0)) *
                   fb::constants::kRad2Deg;
        }();
        const double t8TiltDeg = [&]() {
            if (!in.worldOrient) return 0.0;
            const QVector3D zAxisLocal(0, 0, 1);
            const QVector3D zAxisWorld =
                vec_rotate(zAxisLocal, (*in.worldOrient)[fb::kSEG_T8]);
            return std::acos(std::clamp(double(zAxisWorld.z()), -1.0, 1.0)) *
                   fb::constants::kRad2Deg;
        }();
        const bool pelvisTiltOk =
            pelvisTiltDeg >= fb::kContact.secondaryPelvisT8RejMinDeg &&
            pelvisTiltDeg <= fb::kContact.secondaryPelvisT8RejMaxDeg;
        const bool t8TiltOk =
            t8TiltDeg >= fb::kContact.secondaryPelvisT8RejMinDeg &&
            t8TiltDeg <= fb::kContact.secondaryPelvisT8RejMaxDeg;
        (void)t8TiltOk;

        if (pelvisTiltOk) {
            pushPoint(fb::kSEG_Pelvis, 5, fb::kPelvisSIPSRight);
            pushPoint(fb::kSEG_Pelvis, 6, fb::kPelvisSIPSLeft);
            pushPoint(fb::kSEG_Pelvis, 14, fb::kPelvisCentralButtock);
        }

        std::vector<size_t> order(nCand);
        for (size_t i = 0; i < nCand; ++i) order[i] = i;
        std::sort(order.begin(), order.end(),
                  [&](size_t a, size_t b) { return cands[a].probability > cands[b].probability; });

        const int K = std::min(fb::kContact.maxDetectedContacts, int(nCand));
        for (int i = 0; i < K; ++i) {
            const ActiveContact& c = cands[order[size_t(i)]];
            if (c.probability < fb::kZuptTh.th1) break;
            out.active.push_back(c);
        }

        if (!out.active.empty()) {
            QVector3D sum(0, 0, 0);
            double w_sum = 0.0;
            for (const auto& c : out.active) {
                sum += c.p_world * float(c.probability);
                w_sum += c.probability;
            }
            if (w_sum > 0.0)
                m_lastCoP = sum / float(w_sum);
        }

        std::copy(probs.begin(), probs.begin() + std::min(nCand, probs.size()),
                  out.allProbabilities.begin());

        if (!out.active.empty()) {
            double zMean = 0.0;
            double zR = 1e9, zL = 1e9;
            bool   haveR = false, haveL = false;
            for (const auto& c : out.active) {
                zMean += double(c.p_world.z());
                if (c.seg == fb::kSEG_RFoot || c.seg == fb::kSEG_RToe) {
                    if (double(c.p_world.z()) < zR) { zR = double(c.p_world.z()); haveR = true; }
                }
                if (c.seg == fb::kSEG_LFoot || c.seg == fb::kSEG_LToe) {
                    if (double(c.p_world.z()) < zL) { zL = double(c.p_world.z()); haveL = true; }
                }
            }
            zMean /= double(out.active.size());

            const double newest = zMean;
            m_zRing[m_zRingHead] = newest;
            m_zRingHead = (m_zRingHead + 1) % kZRingCap;
            if (m_zRingCount < kZRingCap) ++m_zRingCount;

            const double bin = std::max(1e-4, fb::kMultiLevel.sameLevelMargin);
            constexpr int kMaxLevels = 10;
            std::array<int, kMaxLevels>     binCnt{};
            std::array<double, kMaxLevels>  binSum{};
            int nBins = 0;
            for (int i = 0; i < m_zRingCount; ++i) {
                const double z = m_zRing[i];
                int idx = -1;
                for (int b = 0; b < nBins; ++b) {
                    if (std::abs(binSum[b] / std::max(1, binCnt[b]) - z) < bin) { idx = b; break; }
                }
                if (idx < 0 && nBins < kMaxLevels) {
                    idx = nBins++; binCnt[idx] = 0; binSum[idx] = 0.0;
                }
                if (idx >= 0) { binCnt[idx]++; binSum[idx] += z; }
            }
            std::array<double, 4> top4{};
            std::array<int,    4> top4Cnt{};
            int kept = 0;
            for (int b = 0; b < nBins; ++b) {
                if (kept < 4 || binCnt[b] > top4Cnt[0]) {
                    int slot = kept < 4 ? kept : 0;
                    int minS = 0;
                    if (kept >= 4) {
                        for (int s = 1; s < 4; ++s) if (top4Cnt[s] < top4Cnt[minS]) minS = s;
                        slot = minS;
                    }
                    top4[slot]    = binSum[b] / std::max(1, binCnt[b]);
                    top4Cnt[slot] = binCnt[b];
                    if (kept < 4) ++kept;
                }
            }

            auto pickLevel = [&](double zMeas) -> double {
                if (kept == 0) return zMeas;
                double best = top4[0];
                double bestD = std::abs(zMeas - top4[0]);
                for (int s = 1; s < kept; ++s) {
                    const double d = std::abs(zMeas - top4[s]);
                    if (d < bestD) { bestD = d; best = top4[s]; }
                }
                return best;
            };

            const double aMix = std::exp(-in.dt /
                                         std::max(1e-3, fb::kMultiLevel.tauSmoothSec));
            if (haveR) {
                const double zRl = pickLevel(zR);
                if (m_floorRInited) m_floorR = aMix * m_floorR + (1.0 - aMix) * zRl;
                else { m_floorR = zRl; m_floorRInited = true; }
            }
            if (haveL) {
                const double zLl = pickLevel(zL);
                if (m_floorLInited) m_floorL = aMix * m_floorL + (1.0 - aMix) * zLl;
                else { m_floorL = zLl; m_floorLInited = true; }
            }

            if (!m_floorInited) {
                m_floorEstimate = pickLevel(zMean);
                m_floorInited = true;
            } else {
                m_floorEstimate = aMix * m_floorEstimate + (1.0 - aMix) * pickLevel(zMean);
            }

            m_stairWalking = (nBins >= fb::kMultiLevel.maxLevelsToDetectStairWalking);

            if (g_testFlag().load(std::memory_order_relaxed) &&
                g_glovesFlag().load(std::memory_order_relaxed)) {
                static int floorTick = 0;
                if ((++floorTick % 60) == 0) {
                    std::cout << std::fixed << std::setprecision(4);
                    std::cout << "[floor] mean=" << m_floorEstimate
                              << " R=" << (m_floorRInited ? m_floorR : 0.0)
                              << " L=" << (m_floorLInited ? m_floorL : 0.0)
                              << " levels=" << kept << " {";
                    for (int s = 0; s < kept; ++s)
                        std::cout << (s ? ", " : "") << top4[s] << "/" << top4Cnt[s];
                    std::cout << "}\n";
                    std::cout.flush();
                }
            }
        }
        return out;
    }

    QVector3D centerOfPressure() const { return m_lastCoP; }
    double    floorLevel() const { return m_floorInited ? m_floorEstimate : 0.0; }
    double    floorLevelRight() const { return m_floorRInited ? m_floorR : floorLevel(); }
    double    floorLevelLeft()  const { return m_floorLInited ? m_floorL : floorLevel(); }
    bool      stairWalking() const { return m_stairWalking; }

private:
    static constexpr int kZRingCap = 240;
    std::deque<double> m_accPeakWindow;
    double             m_prevAccNorm   = 0.0;
    double             m_floorEstimate = 0.0;
    bool               m_floorInited   = false;
    QVector3D          m_lastCoP       = {0, 0, 0};

    std::array<double, kZRingCap> m_zRing{};
    int                m_zRingHead    = 0;
    int                m_zRingCount   = 0;
    double             m_floorR       = 0.0;
    double             m_floorL       = 0.0;
    bool               m_floorRInited = false;
    bool               m_floorLInited = false;
    bool               m_stairWalking = false;
};

class BodyPoseSolver {
public:
    struct Diag {
        double     residualMeanRad     = 0.0;
        double     residualMeanPostRad = 0.0;
        int        numRows             = 0;
        int        poseQualityBand     = fb::PoseQualityGood;
        double     stepNormRad         = 0.0;
        int        iterations          = 0;
        int        rejectedSteps       = 0;
        double     lambda              = 0.0;

        int        outlierRejected     = 0;
        int        outlierHuber        = 0;
        int        outlierSoft         = 0;

        int        zuptActiveRows      = 0;

        double     aidingBiasMaxMps    = 0.0;

        int        outlierWindowed     = 0;

        double     spineFullDeg        = 0.0;
        double     spineFracL5         = 0.0;
        double     spineFracL3         = 0.0;
        double     spineFracT12        = 0.0;
        double     scapThetaRDeg       = 0.0;
        double     scapThetaLDeg       = 0.0;
        double     scapCEffR           = 0.0;
        double     scapCEffL           = 0.0;
        double     kneeFlexRDeg        = 0.0;
        double     kneeFlexLDeg        = 0.0;
        double     kneeScrewRDeg       = 0.0;
        double     kneeScrewLDeg       = 0.0;
        double     anklePfRDeg         = 0.0;
        double     anklePfLDeg         = 0.0;
        bool       ankleClampedR       = false;
        bool       ankleClampedL       = false;
        double     toeMtpRDeg          = 0.0;
        double     toeMtpLDeg          = 0.0;
        double     toeWeightR          = 0.0;
        double     toeWeightL          = 0.0;

        double     covMaxOriRad        = 0.0;
        int        covMaxOriSeg        = -1;
    };

    static constexpr double kLambdaMin =
        fb::kJointLaxitySolver * fb::kJointLaxitySolver;
    static constexpr double kLambdaMax = 1.0;
    static constexpr int    kMaxLambdaRetries = 5;

    Diag solve(std::array<Quat, fb::kSegmentCount>& orient,
               const std::array<Quat, fb::kSegmentCount>& sensorMeas,
               const std::array<bool, fb::kSegmentCount>& sensorPresent,
               const SkinArtifactState& skin,
               const std::vector<ActiveContact>& activeContacts,
               double dt) {
        Diag d{};
        const int N = fb::kSegmentCount;
        const int DOF = N * 3;

        const double aBias = std::exp(-dt / std::max(1e-6, fb::kAidingBias.cT));
        for (int i = 0; i < fb::kSegmentCount; ++i) {
            m_aidingBias[i] = m_aidingBias[i] * float(aBias);
        }

        Eigen::MatrixXd JtWJ  = Eigen::MatrixXd::Zero(DOF, DOF);
        Eigen::VectorXd JtWr  = Eigen::VectorXd::Zero(DOF);
        double initialResidSum = 0.0;
        int    initialResidN   = 0;

        auto evalResidual =
            [&](const std::array<Quat, fb::kSegmentCount>& o) -> double {
            double sum = 0.0;

            for (int i = 0; i < N; ++i) {
                if (!sensorPresent[i]) continue;
                const Quat qPred = skin.applyTo(i, o[i]);
                const Quat qRel  = quat_mult(sensorMeas[i],
                                             qPred.conj()).normalized();
                sum += quat_log(qRel).length();
            }

            for (int g = 0; g < fb::kLumpGroups; ++g) {
                std::vector<int> segs;
                for (int j = 0; j < fb::kJointCount; ++j) {
                    if (fb::kJointLump[j] != g) continue;
                    segs.push_back(fb::kJoints[j].child);
                }
                if (segs.size() < 2) continue;
                for (size_t s = 1; s < segs.size(); ++s) {
                    const Quat qRel = quat_mult(o[segs[s - 1]],
                                                o[segs[s]].conj()).normalized();
                    sum += quat_log(qRel).length();
                }
            }

            {
                const int idxPelvis = 0, idxL5 = 1, idxL3 = 2, idxT12 = 3, idxT8 = 4;
                const double sumC = fb::kCSpine[0] + fb::kCSpine[1] +
                                    fb::kCSpine[2] + fb::kCSpine[3];
                if (sumC > 1e-9) {
                    const QVector3D phi = quat_log(quat_mult(
                        o[idxT8], o[idxPelvis].conj()).normalized());
                    const double cAxial = fb::kCSpine[4];
                    auto enforceR = [&](int idx, double fraction) {
                        const Quat qExp = quat_mult(
                            quat_exp_rotvec(fraction * double(phi.x()),
                                            fraction * double(phi.y()),
                                            fraction * cAxial * double(phi.z())),
                            o[idxPelvis]).normalized();
                        sum += quat_log(quat_mult(qExp,
                            o[idx].conj()).normalized()).length();
                    };
                    enforceR(idxL5,  fb::kCSpine[0] / sumC);
                    enforceR(idxL3, (fb::kCSpine[0] + fb::kCSpine[1]) / sumC);
                    enforceR(idxT12,(fb::kCSpine[0] + fb::kCSpine[1] +
                                     fb::kCSpine[2]) / sumC);
                }
            }

            constexpr double kBarrierD2R = fox::body::constants::kDeg2Rad;
            for (int j = 0; j < fb::kJointCount; ++j) {
                const auto& jd  = fb::kJoints[j];
                const auto& rom = fb::kJointRom[j];
                const Quat qRel = quat_mult(o[jd.child],
                                            o[jd.parent].conj()).normalized();
                const QVector3D phi = quat_log(qRel);
                const double v[3] = { double(phi.x()),
                                      double(phi.y()),
                                      double(phi.z()) };
                const double mn[3] = { rom.abdMin * kBarrierD2R,
                                       rom.flxMin * kBarrierD2R,
                                       rom.rotMin * kBarrierD2R };
                const double mx[3] = { rom.abdMax * kBarrierD2R,
                                       rom.flxMax * kBarrierD2R,
                                       rom.rotMax * kBarrierD2R };
                for (int k = 0; k < 3; ++k) {
                    if      (v[k] > mx[k]) sum += v[k] - mx[k];
                    else if (v[k] < mn[k]) sum += mn[k] - v[k];
                }
            }

            {
                const double frac  = fb::kCPelvis[0];
                const double scale = fb::kCPelvis[1];
                if (std::abs(frac) > 1e-6) {
                    const QVector3D phiPel = quat_log(o[0]);
                    const double tiltDeg = std::abs(double(phiPel.y())) *
                                           fb::constants::kRad2Deg;
                    const double ramp   = (scale > 1e-6)
                        ? std::min(1.0, tiltDeg / scale) : 1.0;
                    const double frac_eff = frac * ramp;
                    if (std::abs(frac_eff) > 1e-9) {
                        const Quat dq = quat_exp_rotvec(0.0,
                                                        frac_eff * double(phiPel.y()),
                                                        0.0);
                        const Quat targetL5 = quat_mult(dq, o[0]).normalized();
                        const QVector3D r = quat_log(quat_mult(targetL5,
                                                               o[1].conj()).normalized());
                        sum += r.length();
                    }
                }
            }

            {
                const QVector3D phiNH = quat_log(quat_mult(o[6],
                                                           o[4].conj()).normalized());
                const Quat dqHalf = quat_exp_rotvec(0.5 * double(phiNH.x()),
                                                    0.5 * double(phiNH.y()),
                                                    0.5 * double(phiNH.z()));
                const Quat neckHalf = quat_mult(dqHalf, o[4]).normalized();
                const QVector3D phiRelNeck = quat_log(quat_mult(neckHalf,
                                                                o[4].conj()).normalized());
                const Quat dqAtt = quat_exp_rotvec(double(phiRelNeck.x()),
                                                   double(phiRelNeck.y()),
                                                   fb::kSpineNeck.cNeck * double(phiRelNeck.z()));
                const Quat targetNeck = quat_mult(dqAtt, o[4]).normalized();
                const QVector3D r = quat_log(quat_mult(targetNeck,
                                                       o[5].conj()).normalized());
                sum += r.length();
            }

            auto axisCEff = [&](double thetaAxisRad, double cLow) {
                const double low  = fb::kScapHumThetaLowDeg;
                const double high = fb::kScapHumThetaHighDeg;
                const double a = std::abs(thetaAxisRad) *
                                 fb::constants::kRad2Deg;
                if (a <= low)  return cLow;
                if (a >= high) return fb::kCArms[2];
                return cLow + (a - low) / (high - low) *
                              (fb::kCArms[2] - cLow);
            };
            auto evalScap = [&](int shoulderSeg, int upperArmSeg) {
                const QVector3D phiH = quat_log(quat_mult(o[upperArmSeg],
                                                          o[4].conj()).normalized());
                const double cEffX = axisCEff(double(phiH.x()), fb::kCArms[0]);
                const double cEffY = axisCEff(double(phiH.y()), fb::kCArms[1]);
                const Quat dqScap = quat_exp_rotvec(cEffX * double(phiH.x()),
                                                    cEffY * double(phiH.y()),
                                                    0.0);
                const Quat targetScap = quat_mult(dqScap, o[4]).normalized();
                const QVector3D r = quat_log(quat_mult(targetScap,
                                                       o[shoulderSeg].conj()).normalized());
                sum += r.length();
            };
            evalScap(7, 8);
            evalScap(11, 12);

            auto evalKnee = [&](int upperSeg, int lowerSeg, bool isRight) {
                const QVector3D phiK = quat_log(quat_mult(o[lowerSeg],
                                                          o[upperSeg].conj()).normalized());
                const double thKnee = std::abs(double(phiK.y()));
                const double thScrew = fb::kCKnees[1] * (1.0 - std::cos(thKnee)) *
                                        fb::kKneeScrewMaxDeg *
                                        fb::constants::kDeg2Rad;
                const double signed_screw = isRight ? thScrew : -thScrew;
                if (std::abs(signed_screw) < 1e-7) return;
                const Quat dq(std::cos(0.5*signed_screw), 0.0, 0.0,
                              std::sin(0.5*signed_screw));
                const Quat targetLower = quat_mult(o[lowerSeg], dq).normalized();
                const QVector3D r = quat_log(quat_mult(targetLower,
                                                       o[lowerSeg].conj()).normalized());
                sum += r.length();
            };
            evalKnee(15, 16, true);
            evalKnee(19, 20, false);

            auto evalAnkle = [&](int lowerSeg, int footSeg) {
                const Quat qRel = quat_mult(o[footSeg],
                                            o[lowerSeg].conj()).normalized();
                const fox::Matrix3 R = fox::quat_to_matrix(qRel);
                const fox::Euler3 e = fox::matrix_to_euler_B(R);
                const double pfMax = fb::kCAnkles[1];
                const double pfMin = -fb::kAnkleDorsiLimitRad;
                double thPf = std::clamp(double(e.e1), pfMin, pfMax);
                double thEv = fb::kCAnkles[0]*double(e.e2) +
                              fb::kCAnkles[2]*std::sin(thPf);
                const double thAx = double(e.e0);
                const double cP = std::cos(0.5*thPf), sP = std::sin(0.5*thPf);
                const double cE = std::cos(0.5*thEv), sE = std::sin(0.5*thEv);
                const double cA = std::cos(0.5*thAx), sA = std::sin(0.5*thAx);
                const Quat qZ(cA, 0.0, 0.0, sA);
                const Quat qX(cE, sE, 0.0, 0.0);
                const Quat qY(cP, 0.0, sP, 0.0);
                const Quat qRelNew = quat_mult(quat_mult(qZ, qX), qY).normalized();
                const Quat targetFoot = quat_mult(qRelNew, o[lowerSeg]).normalized();
                const QVector3D r = quat_log(quat_mult(targetFoot,
                                                       o[footSeg].conj()).normalized());
                sum += r.length();
            };
            evalAnkle(16, 17);
            evalAnkle(20, 21);

            auto evalToe = [&](int footSeg, int toeSeg) {
                const QVector3D phiT = quat_log(quat_mult(o[toeSeg],
                                                          o[footSeg].conj()).normalized());
                const double absT = std::abs(double(phiT.y()));
                const double lo = fb::kToeRockerLowRad;
                const double hi = fb::kToeRockerHighRad;
                double wToe;
                if      (absT <= lo) wToe = 0.0;
                else if (absT >= hi) wToe = 1.0;
                else                  wToe = (absT - lo) / (hi - lo);
                const double toeExt = wToe * absT * (phiT.y() >= 0 ? +1 : -1);
                const Quat dq(std::cos(0.5*toeExt), 0.0, std::sin(0.5*toeExt), 0.0);
                const Quat targetToe = quat_mult(o[footSeg], dq).normalized();
                const QVector3D r = quat_log(quat_mult(targetToe,
                                                       o[toeSeg].conj()).normalized());
                sum += r.length();
            };
            evalToe(17, 18);
            evalToe(21, 22);

            auto evalLegBi = [&](int upperLeg, int foot) {
                const QVector3D phiHip = quat_log(
                    quat_mult(o[upperLeg], o[0].conj()).normalized());
                const double targetDorsiY = fb::kCLegs[1] * double(phiHip.y());
                const QVector3D phiFoot = quat_log(
                    quat_mult(o[foot], o[upperLeg].conj()).normalized());
                const Quat dqTarget = quat_exp_rotvec(double(phiFoot.x()),
                                                      targetDorsiY,
                                                      double(phiFoot.z()));
                const Quat targetFoot = quat_mult(dqTarget, o[upperLeg]).normalized();
                const QVector3D r = quat_log(
                    quat_mult(targetFoot, o[foot].conj()).normalized());
                sum += r.length();
            };
            evalLegBi(15, 17);
            evalLegBi(19, 21);
            return sum;
        };

        for (d.iterations = 0; d.iterations < fb::kMaxIKSteps; ++d.iterations) {
            JtWJ.setZero();
            JtWr.setZero();
            double residSum = 0.0;
            int    residN   = 0;

            d.outlierRejected = 0;
            d.outlierHuber    = 0;
            d.outlierSoft     = 0;
            d.outlierWindowed = 0;

            const int winSize = std::min(BodyPoseSolver::kChi2WindowMax,
                std::max(1, int(std::round(fb::kOutlierRej.jointResWin1 / std::max(1e-6, dt)))));
            for (int i = 0; i < N; ++i) {
                if (!sensorPresent[i]) continue;
                const Quat qPred = skin.applyTo(i, orient[i]);
                const Quat qRel  = quat_mult(sensorMeas[i], qPred.conj()).normalized();
                const QVector3D r = quat_log(qRel);
                const double sigmaOriRad = fb::kSkin.sigmaOriDeg * kD2R;
                const double R_orient    = sigmaOriRad * sigmaOriRad + skin.variance(i);
                double       w           = 1.0 / std::max(1e-9, R_orient);

                const double rNorm2 = double(r.x()*r.x() + r.y()*r.y() + r.z()*r.z());
                const double chi2   = rNorm2 / std::max(1e-9, R_orient);

                if (d.iterations == 0) {
                    auto& ring = m_chi2Window[i];
                    ring.buf[ring.head] = chi2;
                    ring.head = (ring.head + 1) % BodyPoseSolver::kChi2WindowMax;
                    if (ring.count < BodyPoseSolver::kChi2WindowMax) ++ring.count;
                }

                int overTh3 = 0;
                {
                    const auto& ring = m_chi2Window[i];
                    const int n = std::min(ring.count, winSize);
                    for (int k = 0; k < n; ++k) {
                        const int idx = (ring.head - 1 - k + BodyPoseSolver::kChi2WindowMax)
                                         % BodyPoseSolver::kChi2WindowMax;
                        if (ring.buf[idx] > fb::kOutlierRej.outRejTh3) ++overTh3;
                    }
                }
                const double winN     = std::max(1, std::min(m_chi2Window[i].count, winSize));
                const double fracTh3  = double(overTh3) / winN;
                const bool   winSoftEscalate = (fracTh3 > fb::kOutlierRej.countTh[0]);

                if (chi2 > fb::kOutlierRej.outRejTh1) {
                    ++d.outlierRejected;
                    continue;
                } else if (chi2 > fb::kOutlierRej.outRejTh2) {
                    w *= fb::kOutlierRej.outRejTh2 / chi2;
                    ++d.outlierHuber;
                } else if (chi2 > fb::kOutlierRej.outRejTh3) {
                    w *= std::sqrt(fb::kOutlierRej.outRejTh3 / chi2);
                    ++d.outlierSoft;
                }

                if (winSoftEscalate && chi2 > 1.0) {
                    w *= 0.5;
                    ++d.outlierWindowed;
                }

                const int row = i * 3;
                for (int k = 0; k < 3; ++k) {
                    JtWJ(row + k, row + k) += w;
                    JtWr(row + k)          -= w * (k == 0 ? r.x()
                                                  : k == 1 ? r.y() : r.z());
                }
                residSum += r.length();
                ++residN;
            }

            const double w_lump = 1.0 / (fb::kSdLumpRad * fb::kSdLumpRad);
            for (int g = 0; g < fb::kLumpGroups; ++g) {

                std::vector<int> segs;
                for (int j = 0; j < fb::kJointCount; ++j) {
                    if (fb::kJointLump[j] != g) continue;
                    segs.push_back(fb::kJoints[j].child);
                }
                if (segs.size() < 2) continue;

                for (size_t s = 1; s < segs.size(); ++s) {
                    const int a = segs[s - 1];
                    const int b = segs[s];
                    const Quat qRel = quat_mult(orient[a], orient[b].conj()).normalized();
                    const QVector3D r = quat_log(qRel);
                    const int rowA = a * 3;
                    const int rowB = b * 3;
                    for (int k = 0; k < 3; ++k) {

                        JtWJ(rowA + k, rowA + k) += w_lump;
                        JtWJ(rowB + k, rowB + k) += w_lump;
                        JtWJ(rowA + k, rowB + k) -= w_lump;
                        JtWJ(rowB + k, rowA + k) -= w_lump;
                        const double rk = (k == 0 ? r.x() : k == 1 ? r.y() : r.z());
                        JtWr(rowA + k) += w_lump * rk;
                        JtWr(rowB + k) -= w_lump * rk;
                    }
                    residSum += r.length();
                    ++residN;
                }
            }

            const int idxPelvis = 0, idxL5 = 1, idxL3 = 2, idxT12 = 3, idxT8 = 4;
            {
                const double sum = fb::kCSpine[0] + fb::kCSpine[1] +
                                   fb::kCSpine[2] + fb::kCSpine[3];
                if (sum > 1e-9) {
                    const Quat qPel = orient[idxPelvis];
                    const Quat qT8  = orient[idxT8];
                    const QVector3D phi = quat_log(quat_mult(qT8, qPel.conj()).normalized());
                    const double w_spine = 1.0 / (fb::kSpineNeck.stdSpine *
                                                  fb::kSpineNeck.stdSpine);

                    const double cAxial = fb::kCSpine[4];
                    const int rowPel = idxPelvis * 3;
                    auto enforce = [&](int idx, double fraction) {
                        const Quat qExp = quat_mult(
                            quat_exp_rotvec(fraction * double(phi.x()),
                                            fraction * double(phi.y()),
                                            fraction * cAxial * double(phi.z())),
                            qPel).normalized();

                        const QVector3D r = quat_log(
                            quat_mult(qExp, orient[idx].conj()).normalized());
                        const int row = idx * 3;
                        for (int k = 0; k < 3; ++k) {
                            JtWJ(row + k, row + k) += w_spine;
                            JtWr(row + k)          -= w_spine *
                                (k == 0 ? r.x() : k == 1 ? r.y() : r.z());
                        }
                        const double w_coupleParent = w_spine * fraction;
                        for (int k = 0; k < 3; ++k) {
                            const double axisFactor = (k == 2) ? cAxial : 1.0;
                            const double couple = w_coupleParent * axisFactor;
                            JtWJ(rowPel + k, rowPel + k) += couple * fraction;
                            JtWJ(row    + k, rowPel + k) -= couple;
                            JtWJ(rowPel + k, row    + k) -= couple;
                        }
                        residSum += r.length();
                        ++residN;
                    };
                    const double fL5  =  fb::kCSpine[0] / sum;
                    const double fL3  = (fb::kCSpine[0] + fb::kCSpine[1]) / sum;
                    const double fT12 = (fb::kCSpine[0] + fb::kCSpine[1] +
                                         fb::kCSpine[2]) / sum;
                    enforce(idxL5,  fL5);
                    enforce(idxL3,  fL3);
                    enforce(idxT12, fT12);
                    d.spineFracL5  = fL5;
                    d.spineFracL3  = fL3;
                    d.spineFracT12 = fT12;
                    d.spineFullDeg = std::sqrt(double(phi.x())*double(phi.x()) +
                                               double(phi.y())*double(phi.y()) +
                                               double(phi.z())*double(phi.z())) *
                                      fb::constants::kRad2Deg;
                }
            }

            {
                const int idxT8In = 4, idxNeck = 5, idxHead = 6;
                const double cNeck = 0.5 * (fb::kCSpine[5] + fb::kCSpine[6]);
                const double cHead = 0.5 * (fb::kCSpine[7] + fb::kCSpine[8]);
                const double sumNH = cNeck + cHead;
                if (sumNH > 1e-9) {
                    const Quat qT8In  = orient[idxT8In];
                    const Quat qHeadIn = orient[idxHead];
                    const QVector3D phiNH = quat_log(
                        quat_mult(qHeadIn, qT8In.conj()).normalized());
                    const double w_neck = 1.0 / (fb::kSpineNeck.stdNeck *
                                                 fb::kSpineNeck.stdNeck);
                    const double fNeck = cNeck / sumNH;
                    const Quat qNeckTarget = quat_mult(
                        quat_exp_rotvec(fNeck * double(phiNH.x()),
                                        fNeck * double(phiNH.y()),
                                        fNeck * double(phiNH.z())),
                        qT8In).normalized();
                    const QVector3D r = quat_log(
                        quat_mult(qNeckTarget, orient[idxNeck].conj()).normalized());
                    const int row = idxNeck * 3;
                    for (int k = 0; k < 3; ++k) {
                        JtWJ(row + k, row + k) += w_neck;
                        JtWr(row + k)          -= w_neck *
                            (k == 0 ? r.x() : k == 1 ? r.y() : r.z());
                    }
                    residSum += r.length();
                    ++residN;
                }
            }

            const double w_bar = 1.0 /
                (fb::kHypExtPenaltySd * fb::kHypExtPenaltySd);
            constexpr double kBarrierD2R = fox::body::constants::kDeg2Rad;
            for (int j = 0; j < fb::kJointCount; ++j) {
                const auto& jd  = fb::kJoints[j];
                const auto& rom = fb::kJointRom[j];
                const Quat qRel = quat_mult(orient[jd.child],
                                            orient[jd.parent].conj()).normalized();
                const QVector3D phi = quat_log(qRel);
                const double v[3] = { double(phi.x()),
                                      double(phi.y()),
                                      double(phi.z()) };
                const double romMin[3] = { rom.abdMin * kBarrierD2R,
                                           rom.flxMin * kBarrierD2R,
                                           rom.rotMin * kBarrierD2R };
                const double romMax[3] = { rom.abdMax * kBarrierD2R,
                                           rom.flxMax * kBarrierD2R,
                                           rom.rotMax * kBarrierD2R };
                for (int k = 0; k < 3; ++k) {
                    double over = 0.0;
                    if      (v[k] > romMax[k]) over = v[k] - romMax[k];
                    else if (v[k] < romMin[k]) over = v[k] - romMin[k];
                    if (over == 0.0) continue;
                    const int row = jd.child * 3 + k;
                    JtWJ(row, row) += w_bar;
                    JtWr(row)      += w_bar * over;
                    residSum += std::abs(over);
                    ++residN;
                }
            }

            d.zuptActiveRows = 0;
            for (const auto& ac : activeContacts) {
                if (ac.probability < fb::kZuptTh.th3) continue;
                if (ac.seg < 0 || ac.seg >= N) continue;

                const double weightFactor =
                    (ac.probability >= fb::kZuptTh.th4)
                        ? fb::kZuptTh.weightTh4
                        : fb::kZuptTh.weightTh3;

                const QVector3D bias = m_aidingBias[ac.seg];

                const double sigmaVxy = fb::kStdSamePosMeasXY;
                const double sigmaVz  = fb::kStdSamePosMeasXY;
                const double w_xy = (weightFactor * ac.probability) /
                                    (sigmaVxy * sigmaVxy + 1e-12);
                const double w_z  = (weightFactor * ac.probability) /
                                    (sigmaVz  * sigmaVz  + 1e-12);
                const double r_v[3] = { double(ac.v_world.x()) - double(bias.x()),
                                        double(ac.v_world.y()) - double(bias.y()),
                                        double(ac.v_world.z()) - double(bias.z()) };
                const int row = ac.seg * 3;
                for (int k = 0; k < 3; ++k) {
                    const double w_axis = (k == 2) ? w_z : w_xy;
                    JtWJ(row + k, row + k) += w_axis;
                    JtWr(row + k)          += w_axis * r_v[k];
                    residSum += std::abs(r_v[k]);
                }
                ++residN;
                ++d.zuptActiveRows;

                const QVector3D residWorld = ac.v_world - bias;
                m_aidingBias[ac.seg] = bias + residWorld * float(fb::kAidingBias.cV);
                const double biasNorm = std::sqrt(
                    double(m_aidingBias[ac.seg].x()) * double(m_aidingBias[ac.seg].x()) +
                    double(m_aidingBias[ac.seg].y()) * double(m_aidingBias[ac.seg].y()) +
                    double(m_aidingBias[ac.seg].z()) * double(m_aidingBias[ac.seg].z()));
                if (biasNorm > d.aidingBiasMaxMps) d.aidingBiasMaxMps = biasNorm;

                if (ac.sd_height > 0.0) {
                    const double sigmaZ = std::max(1e-4, ac.sd_height);
                    const double w_h    = ac.probability / (sigmaZ * sigmaZ);
                    const double r_z    = double(ac.p_world.z()) - ac.z_floor;
                    JtWJ(row + 2, row + 2) += w_h;
                    JtWr(row + 2)          += w_h * r_z;
                    residSum += std::abs(r_z);
                    ++residN;
                }
            }

            const double w_coup = 1.0 / (fb::kSpineNeck.stdSpine *
                                         fb::kSpineNeck.stdSpine);
            const double w_lax  = 1.0 / (fb::kJointLaxitySolver *
                                         fb::kJointLaxitySolver);

            {
                const double frac     = fb::kCPelvis[0];
                const double scale    = fb::kCPelvis[1];
                const double latPen   = fb::kPelvisLatTiltPenalty;
                if (std::abs(frac) > 1e-6) {
                    const QVector3D phiPel = quat_log(orient[0]);
                    const double tiltDeg = std::abs(double(phiPel.y())) *
                                           fb::constants::kRad2Deg;
                    const double ramp   = (scale > 1e-6)
                        ? std::min(1.0, tiltDeg / scale) : 1.0;
                    const double frac_eff = frac * ramp;
                    if (std::abs(frac_eff) > 1e-9) {
                        const double latPel = double(phiPel.x());
                        const Quat dq = quat_exp_rotvec(latPen * latPel,
                                                        frac_eff * double(phiPel.y()),
                                                        0.0);
                        const Quat targetL5 = quat_mult(dq, orient[0]).normalized();
                        const QVector3D r = quat_log(quat_mult(targetL5,
                                                               orient[1].conj()).normalized());
                        const int rowL5 = 1 * 3;
                        for (int k = 0; k < 3; ++k) {
                            JtWJ(rowL5 + k, rowL5 + k) += w_coup;
                            JtWr(rowL5 + k)            -= w_coup *
                                (k == 0 ? r.x() : k == 1 ? r.y() : r.z());
                        }
                        residSum += r.length();
                        ++residN;
                    }
                }
            }

            {
                const QVector3D phiNH = quat_log(quat_mult(orient[6],
                                                           orient[4].conj()).normalized());
                const Quat dqHalf = quat_exp_rotvec(0.5 * double(phiNH.x()),
                                                    0.5 * double(phiNH.y()),
                                                    0.5 * double(phiNH.z()));
                const Quat neckHalf = quat_mult(dqHalf, orient[4]).normalized();
                const QVector3D phiRel = quat_log(quat_mult(neckHalf,
                                                            orient[4].conj()).normalized());
                const Quat dqAtt = quat_exp_rotvec(double(phiRel.x()),
                                                   double(phiRel.y()),
                                                   fb::kSpineNeck.cNeck *
                                                   double(phiRel.z()));
                const Quat targetNeck = quat_mult(dqAtt, orient[4]).normalized();
                const QVector3D r = quat_log(quat_mult(targetNeck,
                                                       orient[5].conj()).normalized());
                const double w_neck = 1.0 / (fb::kSpineNeck.stdNeck *
                                             fb::kSpineNeck.stdNeck);
                const int rowN = 5 * 3;
                for (int k = 0; k < 3; ++k) {
                    JtWJ(rowN + k, rowN + k) += w_neck;
                    JtWr(rowN + k)           -= w_neck *
                        (k == 0 ? r.x() : k == 1 ? r.y() : r.z());
                }
                residSum += r.length();
                ++residN;
            }

            auto axisCEffWLS = [&](double thetaAxisRad, double cLow) {
                const double low  = fb::kScapHumThetaLowDeg;
                const double high = fb::kScapHumThetaHighDeg;
                const double a = std::abs(thetaAxisRad) *
                                 fb::constants::kRad2Deg;
                if (a <= low)  return cLow;
                if (a >= high) return fb::kCArms[2];
                return cLow + (a - low) / (high - low) *
                              (fb::kCArms[2] - cLow);
            };
            auto applyScap = [&](int shoulderSeg, int upperArmSeg, bool isRight) {
                const QVector3D phiH = quat_log(quat_mult(orient[upperArmSeg],
                                                          orient[4].conj()).normalized());
                const double thetaH = std::sqrt(double(phiH.x())*double(phiH.x()) +
                                                double(phiH.y())*double(phiH.y()) +
                                                double(phiH.z())*double(phiH.z())) *
                                       fb::constants::kRad2Deg;
                const double cEffX = axisCEffWLS(double(phiH.x()), fb::kCArms[0]);
                const double cEffY = axisCEffWLS(double(phiH.y()), fb::kCArms[1]);
                const Quat dqScap = quat_exp_rotvec(cEffX * double(phiH.x()),
                                                    cEffY * double(phiH.y()),
                                                    0.0);
                const Quat targetScap = quat_mult(dqScap, orient[4]).normalized();
                const QVector3D r = quat_log(quat_mult(targetScap,
                                                       orient[shoulderSeg].conj()).normalized());
                const int rowS  = shoulderSeg * 3;
                const int rowT8 = 4 * 3;
                const int rowH  = upperArmSeg * 3;
                for (int k = 0; k < 3; ++k) {
                    JtWJ(rowS + k, rowS + k) += w_coup;
                    JtWr(rowS + k)           -= w_coup *
                        (k == 0 ? r.x() : k == 1 ? r.y() : r.z());
                }
                const double couplerX = w_coup * cEffX;
                const double couplerY = w_coup * cEffY;
                JtWJ(rowS + 0, rowT8 + 0) -= couplerX;
                JtWJ(rowT8 + 0, rowS + 0) -= couplerX;
                JtWJ(rowS + 1, rowT8 + 1) -= couplerY;
                JtWJ(rowT8 + 1, rowS + 1) -= couplerY;
                JtWJ(rowS + 0, rowH  + 0) -= couplerX;
                JtWJ(rowH  + 0, rowS + 0) -= couplerX;
                JtWJ(rowS + 1, rowH  + 1) -= couplerY;
                JtWJ(rowH  + 1, rowS + 1) -= couplerY;
                residSum += r.length();
                ++residN;

                const double cEffAvg = 0.5 * (cEffX + cEffY);
                if (isRight) { d.scapThetaRDeg = thetaH; d.scapCEffR = cEffAvg; }
                else         { d.scapThetaLDeg = thetaH; d.scapCEffL = cEffAvg; }
            };
            applyScap(7, 8, true);
            applyScap(11, 12, false);

            auto applyKnee = [&](int upperSeg, int lowerSeg, bool isRight) {
                const QVector3D phiK = quat_log(quat_mult(orient[lowerSeg],
                                                          orient[upperSeg].conj()).normalized());
                const double thKnee = std::abs(double(phiK.y()));
                const double thScrew = fb::kCKnees[1] * (1.0 - std::cos(thKnee)) *
                                        fb::kKneeScrewMaxDeg *
                                        fb::constants::kDeg2Rad;
                const double signedScrew = isRight ? thScrew : -thScrew;
                if (isRight) {
                    d.kneeFlexRDeg  = thKnee * fb::constants::kRad2Deg;
                    d.kneeScrewRDeg = signedScrew * fb::constants::kRad2Deg;
                } else {
                    d.kneeFlexLDeg  = thKnee * fb::constants::kRad2Deg;
                    d.kneeScrewLDeg = signedScrew * fb::constants::kRad2Deg;
                }
                if (std::abs(signedScrew) < 1e-7) return;

                const Quat dq(std::cos(0.5*signedScrew), 0.0, 0.0,
                              std::sin(0.5*signedScrew));
                const Quat targetLower = quat_mult(orient[lowerSeg], dq).normalized();
                const QVector3D r = quat_log(quat_mult(targetLower,
                                                       orient[lowerSeg].conj()).normalized());
                const int rowL = lowerSeg * 3;
                const int rowU = upperSeg * 3;
                for (int k = 0; k < 3; ++k) {
                    JtWJ(rowL + k, rowL + k) += w_lax;
                    JtWr(rowL + k)           -= w_lax *
                        (k == 0 ? r.x() : k == 1 ? r.y() : r.z());
                }
                const double thKneeSlope =
                    fb::kCKnees[1] * std::sin(thKnee) *
                    fb::kKneeScrewMaxDeg * fb::constants::kDeg2Rad;
                const double w_couple = w_lax * thKneeSlope;
                JtWJ(rowL + 2, rowU + 1) -= w_couple;
                JtWJ(rowU + 1, rowL + 2) -= w_couple;
                residSum += r.length();
                ++residN;
            };
            applyKnee(15, 16, true);
            applyKnee(19, 20, false);

            auto applyAnkle = [&](int lowerSeg, int footSeg, bool isRight) {
                const Quat qRel = quat_mult(orient[footSeg],
                                            orient[lowerSeg].conj()).normalized();
                const fox::Matrix3 R = fox::quat_to_matrix(qRel);
                const fox::Euler3 e = fox::matrix_to_euler_B(R);
                const double pfMax = fb::kCAnkles[1];
                const double pfMin = -fb::kAnkleDorsiLimitRad;
                const bool clamped = (double(e.e1) > pfMax) ||
                                     (double(e.e1) < pfMin);
                const double thPf = std::clamp(double(e.e1), pfMin, pfMax);
                const double thEv = fb::kCAnkles[0]*double(e.e2) +
                                    fb::kCAnkles[2]*std::sin(thPf);
                const double thAx = double(e.e0);
                if (isRight) { d.anklePfRDeg = thPf * fb::constants::kRad2Deg;
                               d.ankleClampedR = clamped; }
                else         { d.anklePfLDeg = thPf * fb::constants::kRad2Deg;
                               d.ankleClampedL = clamped; }
                const double cP = std::cos(0.5*thPf), sP = std::sin(0.5*thPf);
                const double cE = std::cos(0.5*thEv), sE = std::sin(0.5*thEv);
                const double cA = std::cos(0.5*thAx), sA = std::sin(0.5*thAx);
                const Quat qZ(cA, 0.0, 0.0, sA);
                const Quat qX(cE, sE, 0.0, 0.0);
                const Quat qY(cP, 0.0, sP, 0.0);
                const Quat qRelNew = quat_mult(quat_mult(qZ, qX), qY).normalized();
                const Quat targetFoot = quat_mult(qRelNew, orient[lowerSeg]).normalized();
                const QVector3D r = quat_log(quat_mult(targetFoot,
                                                       orient[footSeg].conj()).normalized());
                const int rowF = footSeg  * 3;
                const int rowT = lowerSeg * 3;
                for (int k = 0; k < 3; ++k) {
                    JtWJ(rowF + k, rowF + k) += w_coup;
                    JtWr(rowF + k)           -= w_coup *
                        (k == 0 ? r.x() : k == 1 ? r.y() : r.z());
                }
                const double evDpfSlope = fb::kCAnkles[2] * std::cos(thPf);
                const double w_couple = w_coup * evDpfSlope;
                JtWJ(rowF + 0, rowT + 1) -= w_couple;
                JtWJ(rowT + 1, rowF + 0) -= w_couple;
                residSum += r.length();
                ++residN;
            };
            applyAnkle(16, 17, true);
            applyAnkle(20, 21, false);

            auto applyLegBi = [&](int upperLeg, int foot, bool isRight) {

                const QVector3D phiHip = quat_log(
                    quat_mult(orient[upperLeg], orient[0].conj()).normalized());
                const double hipFlexY = double(phiHip.y());
                const double targetDorsiY = fb::kCLegs[1] * hipFlexY;

                const QVector3D phiFoot = quat_log(
                    quat_mult(orient[foot], orient[upperLeg].conj()).normalized());
                const Quat dqTarget = quat_exp_rotvec(double(phiFoot.x()),
                                                      targetDorsiY,
                                                      double(phiFoot.z()));
                const Quat targetFoot = quat_mult(dqTarget, orient[upperLeg]).normalized();
                const QVector3D r = quat_log(
                    quat_mult(targetFoot, orient[foot].conj()).normalized());

                // §X/§XI σ билатеральной связи ног 0.05 рад -> вес w=1/σ² в МНК (formules.txt)
                const double sdLegBi = 0.05;
                const double w_leg = 1.0 / (sdLegBi * sdLegBi);
                const int rowF = foot * 3;
                for (int k = 0; k < 3; ++k) {
                    JtWJ(rowF + k, rowF + k) += w_leg;
                    JtWr(rowF + k)           -= w_leg *
                        (k == 0 ? r.x() : k == 1 ? r.y() : r.z());
                }
                residSum += r.length();
                ++residN;
                (void)isRight;
            };
            applyLegBi(15, 17, true);
            applyLegBi(19, 21, false);

            auto applyToe = [&](int footSeg, int toeSeg, bool isRight) {
                const QVector3D phiT = quat_log(quat_mult(orient[toeSeg],
                                                          orient[footSeg].conj()).normalized());
                const double signed_y = double(phiT.y());
                const double absT = std::abs(signed_y);
                const double lo = fb::kToeRockerLowRad;
                const double hi = fb::kToeRockerHighRad;
                double wToe;
                if      (absT <= lo) wToe = 0.0;
                else if (absT >= hi) wToe = 1.0;
                else                  wToe = (absT - lo) / (hi - lo);
                const double toeExt = wToe * signed_y;
                if (isRight) { d.toeMtpRDeg = toeExt * fb::constants::kRad2Deg;
                               d.toeWeightR = wToe; }
                else         { d.toeMtpLDeg = toeExt * fb::constants::kRad2Deg;
                               d.toeWeightL = wToe; }
                const Quat dq(std::cos(0.5*toeExt), 0.0, std::sin(0.5*toeExt), 0.0);
                const Quat targetToe = quat_mult(orient[footSeg], dq).normalized();
                const QVector3D r = quat_log(quat_mult(targetToe,
                                                       orient[toeSeg].conj()).normalized());
                const int rowT = toeSeg * 3;
                for (int k = 0; k < 3; ++k) {
                    JtWJ(rowT + k, rowT + k) += w_lax;
                    JtWr(rowT + k)           -= w_lax *
                        (k == 0 ? r.x() : k == 1 ? r.y() : r.z());
                }
                residSum += r.length();
                ++residN;
            };
            applyToe(17, 18, true);
            applyToe(21, 22, false);

            if (d.iterations == 0) {
                initialResidSum = residSum;
                initialResidN   = residN;
            }
            d.numRows = residN;

            bool accepted = false;
            double thisStepNorm = 0.0;
            double thisResidPost = residSum;
            for (int retry = 0; retry < kMaxLambdaRetries; ++retry) {

                Eigen::MatrixXd JtWJ_damped = JtWJ;
                for (int k = 0; k < DOF; ++k) {
                    const double dKK = std::max(1e-12, JtWJ(k, k));
                    JtWJ_damped(k, k) += m_lambda * dKK;
                }

                Eigen::LDLT<Eigen::MatrixXd> ldlt(JtWJ_damped);
                if (ldlt.info() != Eigen::Success) {

                    m_lambda = std::min(kLambdaMax, m_lambda * 10.0);
                    ++d.rejectedSteps;
                    continue;
                }
                Eigen::VectorXd dx = ldlt.solve(-JtWr);

                double stepNorm = 0.0;
                for (int k = 0; k < DOF; ++k) stepNorm += dx[k] * dx[k];
                stepNorm = std::sqrt(stepNorm);

                std::array<Quat, fb::kSegmentCount> orientTrial = orient;
                for (int i = 0; i < N; ++i) {
                    const int row = i * 3;
                    const QVector3D phi(static_cast<float>(dx[row]),
                                        static_cast<float>(dx[row + 1]),
                                        static_cast<float>(dx[row + 2]));
                    const Quat dq = quat_exp_rotvec(double(phi.x()),
                                                    double(phi.y()),
                                                    double(phi.z()));
                    Quat q = quat_mult(orientTrial[i], dq).normalized();
                    if (q.w < 0.0) {
                        q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z;
                    }
                    orientTrial[i] = q;
                }

                const double residTrial = evalResidual(orientTrial);
                if (residTrial < residSum) {
                    orient        = orientTrial;
                    thisStepNorm  = stepNorm;
                    thisResidPost = residTrial;
                    m_lambda      = std::max(kLambdaMin, m_lambda * 0.1);
                    accepted      = true;
                    break;
                }
                m_lambda = std::min(kLambdaMax, m_lambda * 10.0);
                ++d.rejectedSteps;
            }

            d.stepNormRad         = thisStepNorm;
            d.residualMeanPostRad = residN > 0
                ? (thisResidPost / double(residN)) : 0.0;

            if (!accepted) break;

            const double gradNorm = JtWr.norm() / std::max(1, residN);
            if (gradNorm < fb::kIKGradTolRad &&
                thisStepNorm < fb::kIKStepTolRad) {
                break;
            }
        }

        d.residualMeanRad = initialResidN > 0
            ? (initialResidSum / double(initialResidN)) : 0.0;
        d.poseQualityBand = fb::poseQualityFromResidual(d.residualMeanPostRad);
        d.lambda          = m_lambda;

        {
            Eigen::MatrixXd A = JtWJ;
            for (int k = 0; k < DOF; ++k) {
                const double dKK = std::max(1e-12, JtWJ(k, k));
                A(k, k) += m_lambda * dKK;
            }
            Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
            if (ldlt.info() == Eigen::Success) {

                double maxCov = 0.0;
                int    maxSeg = -1;
                Eigen::VectorXd ei = Eigen::VectorXd::Zero(DOF);
                for (int i = 0; i < N; ++i) {
                    double segCov = 0.0;
                    for (int k = 0; k < 3; ++k) {
                        ei.setZero();
                        ei[i * 3 + k] = 1.0;
                        Eigen::VectorXd sigCol = ldlt.solve(ei);
                        segCov += sigCol[i * 3 + k];
                    }

                    const double sigmaRad = std::sqrt(std::max(0.0, segCov / 3.0));
                    if (sigmaRad > maxCov) { maxCov = sigmaRad; maxSeg = i; }
                }
                d.covMaxOriRad = maxCov;
                d.covMaxOriSeg = maxSeg;

                if (maxCov > 0.0 && maxCov < 1.0) {
                    const int sigmaBand =
                        (maxCov < fb::kPoseQualityResidBands[0]) ? fb::PoseQualityExcellent :
                        (maxCov < fb::kPoseQualityResidBands[1]) ? fb::PoseQualityGood      :
                        (maxCov < fb::kPoseQualityResidBands[2]) ? fb::PoseQualityAdequate  :
                        (maxCov < fb::kPoseQualityResidBands[3]) ? fb::PoseQualityPoor :
                                            fb::PoseQualityInvalid;
                    d.poseQualityBand = std::min(d.poseQualityBand, sigmaBand);
                }
            }
        }
        return d;
    }

private:

    double m_lambda = kLambdaMin;

    std::array<QVector3D, fb::kSegmentCount> m_aidingBias{};

    static constexpr int kChi2WindowMax = 32;
    struct Chi2Ring {
        std::array<double, kChi2WindowMax> buf{};
        int head = 0;
        int count = 0;
    };
    std::array<Chi2Ring, fb::kSegmentCount> m_chi2Window{};
};

enum class LocomotionPhase : std::uint8_t {
    Unknown   = 0,
    Standing  = 1,
    Walking   = 2,
    Running   = 3,
    Sitting   = 4,
    Acrobatic = 5,
};

inline const char* locomotionPhaseName(LocomotionPhase p) {
    switch (p) {
        case LocomotionPhase::Standing:  return "standing";
        case LocomotionPhase::Walking:   return "walking";
        case LocomotionPhase::Running:   return "running";
        case LocomotionPhase::Sitting:   return "sitting";
        case LocomotionPhase::Acrobatic: return "acrobatic";
        default: return "unknown";
    }
}

// §29/§1543 классификация активности (стоит/идёт/бежит/сидит/прыжок/акробатика) и §49 фазы походки
//   HS->FF->HO->TO->SW; пороги в fb::kGait/fb::kJumpDetect (§foxbody) (formules.txt)
class LocomotionClassifier {
public:
    enum class GaitPhase : std::uint8_t {
        NA = 0,
        HS,
        FF,
        HO,
        TO,
        SW
    };
    static const char* gaitPhaseName(GaitPhase p) {
        switch (p) {
            case GaitPhase::HS: return "HS";
            case GaitPhase::FF: return "FF";
            case GaitPhase::HO: return "HO";
            case GaitPhase::TO: return "TO";
            case GaitPhase::SW: return "SW";
            default:            return "NA";
        }
    }
    GaitPhase gaitPhaseR() const { return m_phaseR; }
    GaitPhase gaitPhaseL() const { return m_phaseL; }
    double    gaitDurR()   const { return m_durR; }
    double    gaitDurL()   const { return m_durL; }

    LocomotionPhase phase() const { return m_phase; }
    double          flightFracSec() const { return m_flightSec; }
    double          contactFracSec() const { return m_contactSec; }
    double          pelvisTiltDeg() const { return m_pelvisTiltDeg; }
    double          t8TiltDeg()     const { return m_t8TiltDeg; }

    void updateGaitPhases(bool rContact, bool lContact,
                          double rFootPitchZ, double lFootPitchZ,
                          double rFootVelZ,   double lFootVelZ,
                          double dt) {
        const GaitPhase prevR = m_phaseR;
        const GaitPhase prevL = m_phaseL;
        m_phaseR = classifyLeg(m_phaseR, rContact, rFootPitchZ, rFootVelZ, dt, m_durR);
        m_phaseL = classifyLeg(m_phaseL, lContact, lFootPitchZ, lFootVelZ, dt, m_durL);
        if (prevR != GaitPhase::HS && m_phaseR == GaitPhase::HS) ++m_heelStrikeR;
        if (prevL != GaitPhase::HS && m_phaseL == GaitPhase::HS) ++m_heelStrikeL;
    }

    void updateStepMetrics(const QVector3D& rHeelWorld,
                           const QVector3D& lHeelWorld,
                           const QVector3D& comWorld,
                           double dt)
    {
        (void)dt;
        if (m_phaseR == GaitPhase::HS && m_lastPhaseR != GaitPhase::HS) {
            const double sl = (rHeelWorld - m_lastHeelR_HS).length();
            const double sh = std::abs(double(rHeelWorld.z() - m_lastHeelR_HS.z()));
            if (m_haveLastHeelR && sl > 0.01 && sl < 2.5) {
                m_lastStrideLengthR = sl;
                m_lastStrideHeightR = sh;
            }
            m_lastHeelR_HS = rHeelWorld;
            m_haveLastHeelR = true;
        }
        if (m_phaseL == GaitPhase::HS && m_lastPhaseL != GaitPhase::HS) {
            const double sl = (lHeelWorld - m_lastHeelL_HS).length();
            const double sh = std::abs(double(lHeelWorld.z() - m_lastHeelL_HS.z()));
            if (m_haveLastHeelL && sl > 0.01 && sl < 2.5) {
                m_lastStrideLengthL = sl;
                m_lastStrideHeightL = sh;
            }
            m_lastHeelL_HS = lHeelWorld;
            m_haveLastHeelL = true;
        }
        m_lastPhaseR = m_phaseR;
        m_lastPhaseL = m_phaseL;

        if (m_haveLastCom) {
            const double z = double(comWorld.z());
            if (z > m_comZMax) m_comZMax = z;
            if (z < m_comZMin) m_comZMin = z;
            m_lastVertCoMOscM = m_comZMax - m_comZMin;
            if (m_comOscDecayTicks++ > 240) {
                m_comZMin =  1e9;
                m_comZMax = -1e9;
                m_comOscDecayTicks = 0;
            }
        } else {
            m_comZMin = double(comWorld.z());
            m_comZMax = double(comWorld.z());
            m_haveLastCom = true;
        }
    }

    double lastStrideLengthR() const { return m_lastStrideLengthR; }
    double lastStrideLengthL() const { return m_lastStrideLengthL; }
    double lastStrideHeightR() const { return m_lastStrideHeightR; }
    double lastStrideHeightL() const { return m_lastStrideHeightL; }
    double vertCoMOscillationM() const { return m_lastVertCoMOscM; }
    int    heelStrikeCountR() const { return m_heelStrikeR; }
    int    heelStrikeCountL() const { return m_heelStrikeL; }

    enum class BodyPosture : std::uint8_t {
        Vertical = 0,
        TiltedForward,
        Bent,
        Sitting,
        Inverted
    };
    BodyPosture posture() const { return m_posture; }

    static const char* postureName(BodyPosture p) {
        switch (p) {
            case BodyPosture::Vertical:       return "vertical";
            case BodyPosture::TiltedForward:  return "tiltedFwd";
            case BodyPosture::Bent:           return "bent";
            case BodyPosture::Sitting:        return "sitting";
            case BodyPosture::Inverted:       return "inverted";
        }
        return "?";
    }

    LocomotionPhase update(double pelvisZ, double pelvisSpeed,
                            bool rFootContact, bool lFootContact,
                            double pelvisTiltDeg,
                            double t8TiltDeg,
                            double sitHeightM,
                            double dt) {

        const bool anyContact = rFootContact || lFootContact;
        if (anyContact) {
            m_contactSec += dt;
            m_flightSec   = std::max(0.0, m_flightSec - dt);
        } else {
            m_flightSec  += dt;
            m_contactSec  = std::max(0.0, m_contactSec - dt);
        }
        m_flightSec   = std::min(m_flightSec,   1.0);
        m_contactSec  = std::min(m_contactSec,  1.0);

        const int curSide = rFootContact ? +1 : (lFootContact ? -1 : 0);
        if (curSide != 0 && curSide != m_lastContactSide) {
            m_lastContactSide = curSide;
            m_altSec = 0.0;
        } else {
            m_altSec += dt;
        }
        m_pelvisTiltDeg = pelvisTiltDeg;
        m_t8TiltDeg     = t8TiltDeg;

        const double inclMin = fox::body::kContact.secondaryPelvisT8RejMinDeg;
        const double inclMax = fox::body::kContact.secondaryPelvisT8RejMaxDeg;
        const bool pelvisVertical = (pelvisTiltDeg < inclMin);
        const bool t8Vertical     = (t8TiltDeg     < inclMin);
        const bool t8Inverted     = (t8TiltDeg     > inclMax);

        if (t8Inverted)                       m_posture = BodyPosture::Inverted;
        else if (pelvisVertical && t8Vertical) m_posture = BodyPosture::Vertical;
        else if (pelvisVertical && !t8Vertical) m_posture = BodyPosture::TiltedForward;
        else if (!pelvisVertical && t8Vertical) m_posture = BodyPosture::Sitting;
        else                                    m_posture = BodyPosture::Bent;

        const double sittingZ = std::max(0.30, sitHeightM);
        const bool   lowPelvis = pelvisZ < sittingZ;

        const bool doubleSupport = rFootContact && lFootContact;
        const bool anyFlight     = m_flightSec > fox::body::kGait.flightSecForRun;
        if (t8Inverted)                            m_phase = LocomotionPhase::Acrobatic;
        else if (anyFlight)                        m_phase = LocomotionPhase::Running;
        else if (m_posture == BodyPosture::Sitting && lowPelvis)
                                                   m_phase = LocomotionPhase::Sitting;
        else if (!pelvisVertical && lowPelvis)     m_phase = LocomotionPhase::Sitting;
        else if (doubleSupport &&
                 pelvisSpeed < fox::body::kGait.standingPelvisSpeedMax &&
                 pelvisVertical)
                                                   m_phase = LocomotionPhase::Standing;
        else if (anyContact && pelvisVertical) {
            const double winW = (pelvisSpeed > fox::body::kContact.highVelTh)
                ? fox::body::kContact.firstWinWidthHighVel
                : fox::body::kContact.firstWinWidth;
            if (m_altSec < winW) m_phase = LocomotionPhase::Walking;
        }
        else if (!anyContact &&
                 (pelvisTiltDeg > fox::body::kGait.acrobaticTiltDeg ||
                  t8TiltDeg     > fox::body::kGait.acrobaticTiltDeg))
                                                   m_phase = LocomotionPhase::Acrobatic;

        return m_phase;
    }

private:
    static GaitPhase classifyLeg(GaitPhase cur, bool contact,
                                 double pitchZ, double footVelZ,
                                 double dt, double& dur)
    {
        const double pitchHeel = fox::body::kGait.pitchHeelRad;
        const double pitchToe  = fox::body::kGait.pitchToeRad;
        const double velGround = fox::body::kGait.velGround;
        const double velFlight = fox::body::kGait.velFlight;
        const double ffHold    = fox::body::kGait.ffHoldSec;
        dur += dt;
        const double absV = std::abs(footVelZ);
        if (!contact && absV > velFlight) { dur = 0.0; return GaitPhase::SW; }
        if (contact && pitchZ < pitchHeel && absV < velGround) {
            dur = 0.0; return GaitPhase::HS;
        }
        if (contact && std::abs(pitchZ) < pitchToe && absV < velGround && dur > ffHold)
            return GaitPhase::FF;
        if (contact && pitchZ > pitchToe) return GaitPhase::HO;
        if (!contact && cur == GaitPhase::HO) { dur = 0.0; return GaitPhase::TO; }
        return cur;
    }

    LocomotionPhase m_phase           = LocomotionPhase::Unknown;
    double          m_flightSec       = 0.0;
    double          m_contactSec      = 0.0;
    double          m_altSec          = 0.0;
    int             m_lastContactSide = 0;
    double          m_pelvisTiltDeg   = 0.0;
    double          m_t8TiltDeg       = 0.0;
    BodyPosture     m_posture         = BodyPosture::Vertical;

    GaitPhase       m_phaseR          = GaitPhase::NA;
    GaitPhase       m_phaseL          = GaitPhase::NA;
    GaitPhase       m_lastPhaseR      = GaitPhase::NA;
    GaitPhase       m_lastPhaseL      = GaitPhase::NA;
    double          m_durR            = 0.0;
    double          m_durL            = 0.0;
    int             m_heelStrikeR     = 0;
    int             m_heelStrikeL     = 0;
    QVector3D       m_lastHeelR_HS    {0, 0, 0};
    QVector3D       m_lastHeelL_HS    {0, 0, 0};
    bool            m_haveLastHeelR   = false;
    bool            m_haveLastHeelL   = false;
    double          m_lastStrideLengthR = 0.0;
    double          m_lastStrideLengthL = 0.0;
    double          m_lastStrideHeightR = 0.0;
    double          m_lastStrideHeightL = 0.0;
    double          m_comZMin         =  1e9;
    double          m_comZMax         = -1e9;
    bool            m_haveLastCom     = false;
    int             m_comOscDecayTicks = 0;
    double          m_lastVertCoMOscM = 0.0;
};

struct SmootherFrame {
    std::array<Quat, fb::kSegmentCount> orient_filtered{};
    std::array<double, fb::kSegmentCount * 3> P_diag{};
    double dt = 0.0;
    bool   valid = false;
};

class BatchSmoother {
public:
    static constexpr int kWindow = 240;
    BatchSmoother() { m_ring.fill({}); }
    void reset() {
        m_head = 0; m_count = 0;
        m_ring.fill({});
    }
    void pushFrame(const std::array<Quat, fb::kSegmentCount>& orient,
                   const std::array<double, fb::kSegmentCount * 3>& P_diag,
                   double dt) {
        m_ring[m_head].orient_filtered = orient;
        m_ring[m_head].P_diag = P_diag;
        m_ring[m_head].dt    = dt;
        m_ring[m_head].valid = true;
        m_head = (m_head + 1) % kWindow;
        if (m_count < kWindow) ++m_count;
    }
    void backwardPass(std::array<Quat, fb::kSegmentCount>& smoothedOut) const {
        if (m_count == 0) return;
        const int newest = (m_head + kWindow - 1) % kWindow;
        smoothedOut = m_ring[newest].orient_filtered;
        if (m_count < 2) return;
        std::array<Quat, fb::kSegmentCount> x_s = smoothedOut;
        for (int back = 1; back < m_count; ++back) {
            const int t = (m_head + kWindow - 1 - back) % kWindow;
            const int t1 = (t + 1) % kWindow;
            if (!m_ring[t].valid || !m_ring[t1].valid) continue;
            for (int s = 0; s < fb::kSegmentCount; ++s) {
                const Quat qf = m_ring[t].orient_filtered[s];
                const Quat qp = m_ring[t1].orient_filtered[s];
                const Quat dq = quat_mult(x_s[s], qp.conj()).normalized();
                const QVector3D phi = quat_log(dq);
                double g[3];
                for (int k = 0; k < 3; ++k) {
                    const double P_f = m_ring[t].P_diag[s * 3 + k];
                    const double P_p = m_ring[t1].P_diag[s * 3 + k];
                    g[k] = (P_p > 1e-12) ? std::clamp(P_f / P_p, 0.0, 1.0) : 0.0;
                }
                const Quat qShift = quat_exp_rotvec(g[0] * double(phi.x()),
                                                     g[1] * double(phi.y()),
                                                     g[2] * double(phi.z()));
                x_s[s] = quat_mult(qShift, qf).normalized();
            }
        }
        smoothedOut = x_s;
    }
    void marginalizeOldest() {
        if (m_count < kWindow) return;
        const int oldest = m_head;
        m_ring[oldest].valid = false;
    }
    int count() const { return m_count; }
private:
    std::array<SmootherFrame, kWindow> m_ring{};
    int m_head  = 0;
    int m_count = 0;
};

// §X FoxFE — фьюжн тела (разрежённый взвешенный МНК §13): объединяет ориентации датчиков в согласованную позу
//   скелета с биомех-связями (§XI: спина §1098.8, лопаточно-плечевой ритм §46, колено §1300, стопа §48/§49),
//   контактами/ZUPT (§XIV/§52/§XIII), локомоцией (§29) и skin-моделью (§XI). (formules.txt)
class PoseRefiner {
public:
    void reset() {
        m_skin.reset();
        m_contacts.reset();
        m_prevOrient.fill(Quat(1, 0, 0, 0));
        m_prevSegCenter.fill(QVector3D(0, 0, 0));
        m_havePrev = false;
        m_lastT    = 0.0;
        m_lastCoM       = QVector3D(0, 0, 0);
        m_prevCoM       = QVector3D(0, 0, 0);
        m_lastCoMVel     = QVector3D(0, 0, 0);
        m_lastCoMAccel   = QVector3D(0, 0, 0);
        m_lastGRFNewtons = QVector3D(0, 0, 0);
        m_lastBodyMass   = 0.0;
        m_haveCoMPrev   = false;
        m_smoother.reset();
    }

    void predictSkin(int seg, double dt) { m_skin.predict(seg, dt); }

    struct FrameContext {
        const std::array<Quat,      fb::kSegmentCount>* sensorMeas;
        const std::array<bool,      fb::kSegmentCount>* sensorPresent;
        const std::array<QVector3D, fb::kSegmentCount>* segCenter;
        const std::array<QVector3D, fb::kSegmentCount>* accLPBody;
        double                                          floorLevelZ;
        double                                          dt;
    };

    // §X/§30 главный шаг фьюжна тела на кадр: детект контактов -> разрежённый МНК-решатель (§13)
    //   с биомех-связями и подсказками -> уточнённые ориентации сегментов orient (formules.txt)
    BodyPoseSolver::Diag refine(std::array<Quat, fb::kSegmentCount>& orient,
                                const FrameContext& ctx,
                                ContactDetector::Result* outContacts = nullptr) {

        std::array<QVector3D, fb::kSegmentCount> velocity{};
        std::array<QVector3D, fb::kSegmentCount> omega{};
        const double dt = std::max(1e-4, ctx.dt);
        if (m_havePrev && ctx.segCenter) {
            for (int i = 0; i < fb::kSegmentCount; ++i) {
                velocity[i] = ((*ctx.segCenter)[i] - m_prevSegCenter[i]) / float(dt);
                const Quat dq = quat_mult(orient[i], m_prevOrient[i].conj());
                omega[i] = fox::angular_velocity_from_quat(dq, dt);
            }
        }

        ContactDetector::Result cr;
        if (ctx.segCenter && ctx.accLPBody) {
            ContactDetector::FrameInput in{};
            in.worldOrient   = &orient;
            in.segCenter     = ctx.segCenter;
            in.segVelocity   = &velocity;
            in.segOmega      = &omega;
            in.accLPBody     = ctx.accLPBody;
            in.skinState     = &m_skin;
            in.floorLevelZ   = ctx.floorLevelZ;
            in.dt            = dt;
            cr = m_contacts.detect(in);
        }
        if (outContacts) *outContacts = cr;

        BodyPoseSolver::Diag dg{};
        if (ctx.sensorMeas && ctx.sensorPresent) {
            dg = m_solver.solve(orient, *ctx.sensorMeas, *ctx.sensorPresent,
                                m_skin, cr.active, dt);

            {
                std::array<double, fb::kSegmentCount * 3> P_diag{};
                const double sigma2 = std::max(1e-9,
                    dg.covMaxOriRad * dg.covMaxOriRad);
                for (int s = 0; s < fb::kSegmentCount; ++s)
                    for (int k = 0; k < 3; ++k)
                        P_diag[s * 3 + k] = sigma2;
                m_smoother.pushFrame(orient, P_diag, dt);
                if (m_smoother.count() >= BatchSmoother::kWindow) {
                    std::array<Quat, fb::kSegmentCount> smoothed{};
                    m_smoother.backwardPass(smoothed);
                    orient = smoothed;
                    m_smoother.marginalizeOldest();
                }
            }

            for (int i = 0; i < fb::kSegmentCount; ++i) {
                if (!(*ctx.sensorPresent)[i]) continue;
                const double omegaNorm = m_havePrev
                    ? std::sqrt(double(omega[i].x()) * double(omega[i].x()) +
                                double(omega[i].y()) * double(omega[i].y()) +
                                double(omega[i].z()) * double(omega[i].z()))
                    : 0.0;
                m_skin.predict(i, dt, omegaNorm);
                double accMag = 0.0;
                if (ctx.accLPBody) {
                    const QVector3D& a = (*ctx.accLPBody)[i];
                    accMag = std::sqrt(double(a.x()) * double(a.x()) +
                                       double(a.y()) * double(a.y()) +
                                       double(a.z()) * double(a.z()));
                }
                m_skin.predictPos(i, dt, accMag);
            }

            int updatedSegs = 0;
            for (const auto& ac : cr.active) {
                if (ac.probability < fb::kZuptTh.th3) continue;
                if (ac.seg < 0 || ac.seg >= fb::kSegmentCount) continue;
                const double dz = ac.z_floor - double(ac.p_world.z());
                if (std::abs(dz) < 1e-5) continue;

                const double sdH = fb::stdHeightMeasFor(ac.seg);
                const double R_pos = sdH * sdH + 1e-9;
                m_skin.updatePos(ac.seg,
                                 QVector3D(0.0f, 0.0f, float(dz)), R_pos);
                ++updatedSegs;
            }
            m_lastSkinPosUpdates = updatedSegs;
        }

        if (ctx.segCenter) {
            bool rContact = false, lContact = false;
            for (const auto& ac : cr.active) {
                if (ac.probability < fb::kZuptTh.th3) continue;
                if (ac.seg == fb::kSEG_RFoot || ac.seg == fb::kSEG_RToe) rContact = true;
                if (ac.seg == fb::kSEG_LFoot || ac.seg == fb::kSEG_LToe) lContact = true;
            }
            const QVector3D pelvis = (*ctx.segCenter)[0];
            const double pelvisSpeed = m_havePrev
                ? (pelvis - m_prevSegCenter[0]).length() / std::max(1e-4, dt)
                : 0.0;

            const QVector3D pelvisZAxisWorld = vec_rotate(QVector3D(0, 0, 1), orient[0]);
            const double pelvisTiltDeg =
                std::acos(std::clamp(double(pelvisZAxisWorld.z()), -1.0, 1.0)) *
                fb::constants::kRad2Deg;

            const QVector3D t8ZAxisWorld = vec_rotate(QVector3D(0, 0, 1), orient[4]);
            const double t8TiltDeg =
                std::acos(std::clamp(double(t8ZAxisWorld.z()), -1.0, 1.0)) *
                fb::constants::kRad2Deg;

            const double sitH = fb::pelvisSitHeightM(1.75);
            m_locomotion.update(double(pelvis.z()), pelvisSpeed,
                                rContact, lContact,
                                pelvisTiltDeg, t8TiltDeg,
                                sitH, dt);

            const QVector3D rFootZAxisWorld =
                vec_rotate(QVector3D(0, 0, 1), orient[fb::kSEG_RFoot]);
            const QVector3D lFootZAxisWorld =
                vec_rotate(QVector3D(0, 0, 1), orient[fb::kSEG_LFoot]);
            const double rFootPitchZ = double(rFootZAxisWorld.z());
            const double lFootPitchZ = double(lFootZAxisWorld.z());
            const double rFootVelZ = m_havePrev
                ? double(velocity[fb::kSEG_RFoot].z()) : 0.0;
            const double lFootVelZ = m_havePrev
                ? double(velocity[fb::kSEG_LFoot].z()) : 0.0;
            m_locomotion.updateGaitPhases(rContact, lContact,
                                          rFootPitchZ, lFootPitchZ,
                                          rFootVelZ, lFootVelZ, dt);

            // §90/§138.16 точка пятки pHeel (= fb::kFootPointsRight[0].r_local) для CoM/детекции heel-strike
            const QVector3D rHeelW = (*ctx.segCenter)[fb::kSEG_RFoot]
                + vec_rotate(QVector3D(-0.036f, 0.0f, -0.080f),
                              orient[fb::kSEG_RFoot]);
            const QVector3D lHeelW = (*ctx.segCenter)[fb::kSEG_LFoot]
                + vec_rotate(QVector3D(-0.036f, 0.0f, -0.080f),
                              orient[fb::kSEG_LFoot]);
            const QVector3D comNow = fb::centerOfMass(*ctx.segCenter, nullptr);
            m_locomotion.updateStepMetrics(rHeelW, lHeelW, comNow, dt);

            if (g_testFlag().load(std::memory_order_relaxed) &&
                g_glovesFlag().load(std::memory_order_relaxed)) {
                static int phaseTick = 0;
                if ((++phaseTick % 30) == 0) {
                    std::cout << "[phase] R="
                              << LocomotionClassifier::gaitPhaseName(m_locomotion.gaitPhaseR())
                              << " durR=" << std::fixed << std::setprecision(3)
                              << m_locomotion.gaitDurR()
                              << "s  L="
                              << LocomotionClassifier::gaitPhaseName(m_locomotion.gaitPhaseL())
                              << " durL=" << m_locomotion.gaitDurL() << "s  "
                              << "strideR=" << m_locomotion.lastStrideLengthR()
                              << "m hR=" << m_locomotion.lastStrideHeightR() << "m "
                              << "strideL=" << m_locomotion.lastStrideLengthL()
                              << "m hL=" << m_locomotion.lastStrideHeightL() << "m "
                              << "vertOsc=" << m_locomotion.vertCoMOscillationM() << "m"
                              << "\n";
                    std::cout.flush();
                }
            }

            double bodyMass = 0.0;
            const QVector3D com = fb::centerOfMass(*ctx.segCenter, &bodyMass);
            if (m_haveCoMPrev) {
                const QVector3D newVel =
                    (com - m_prevCoM) / float(std::max(1e-4, dt));
                m_lastCoMAccel = (newVel - m_lastCoMVel) / float(std::max(1e-4, dt));
                m_lastCoMVel = newVel;
            } else {
                m_lastCoMVel   = QVector3D(0, 0, 0);
                m_lastCoMAccel = QVector3D(0, 0, 0);
            }
            m_prevCoM      = com;
            m_lastCoM      = com;
            m_lastBodyMass = bodyMass;
            m_haveCoMPrev  = true;

            const QVector3D gravityWorld(0.0f, 0.0f,
                float(fb::constants::kGravityMs2));
            m_lastGRFNewtons =
                float(bodyMass) * (m_lastCoMAccel + gravityWorld);

            if (g_testFlag().load(std::memory_order_relaxed) &&
                g_glovesFlag().load(std::memory_order_relaxed)) {
                static int comTick = 0;
                if ((++comTick % 60) == 0) {
                    std::cout << std::fixed << std::setprecision(4);
                    std::cout << "[com] pos=(" << m_lastCoM.x() << ","
                              << m_lastCoM.y() << "," << m_lastCoM.z()
                              << ") m  vel=(" << m_lastCoMVel.x() << ","
                              << m_lastCoMVel.y() << "," << m_lastCoMVel.z()
                              << ") m/s  M_kg=" << m_lastBodyMass
                              << "  F_GRF_z=" << m_lastGRFNewtons.z() << " N"
                              << "  contactR=" << (rContact ? 1 : 0)
                              << "  contactL=" << (lContact ? 1 : 0)
                              << "\n";
                    std::cout.flush();
                }
            }
        }

        if (ctx.segCenter) {
            m_prevSegCenter = *ctx.segCenter;
            m_prevOrient    = orient;
            m_havePrev      = true;
        }
        return dg;
    }

    const SkinArtifactState&    skin()     const { return m_skin; }
    const ContactDetector&      contacts() const { return m_contacts; }
    const LocomotionClassifier& locomotion() const { return m_locomotion; }
    int                         lastSkinPosUpdates() const { return m_lastSkinPosUpdates; }
    QVector3D                   lastCoM()       const { return m_lastCoM; }
    QVector3D                   lastCoMVel()    const { return m_lastCoMVel; }
    QVector3D                   lastCoMAccel()  const { return m_lastCoMAccel; }
    QVector3D                   lastGRFNewtons() const { return m_lastGRFNewtons; }
    double                      lastBodyMass()  const { return m_lastBodyMass; }

private:
    SkinArtifactState     m_skin;
    ContactDetector       m_contacts;
    BodyPoseSolver        m_solver;
    BatchSmoother         m_smoother;
    LocomotionClassifier  m_locomotion;
    std::array<Quat,      fb::kSegmentCount> m_prevOrient{};
    std::array<QVector3D, fb::kSegmentCount> m_prevSegCenter{};
    bool   m_havePrev = false;
    double m_lastT    = 0.0;

    int       m_lastSkinPosUpdates = 0;
    QVector3D m_lastCoM        {0, 0, 0};
    QVector3D m_prevCoM        {0, 0, 0};
    QVector3D m_lastCoMVel     {0, 0, 0};
    QVector3D m_lastCoMAccel   {0, 0, 0};
    QVector3D m_lastGRFNewtons {0, 0, 0};
    double    m_lastBodyMass   = 0.0;
    bool      m_haveCoMPrev    = false;
};

inline PoseRefiner&  g_refiner() {
    static PoseRefiner inst;
    return inst;
}
inline std::mutex&   g_refinerMtx() {
    static std::mutex mtx;
    return mtx;
}

inline std::atomic<bool>& g_testFlag()   { static std::atomic<bool> v{false}; return v; }
inline std::atomic<bool>& g_glovesFlag() { static std::atomic<bool> v{false}; return v; }

void dumpFrameDiag(bool testEnabled, bool glovesEnabled,
                   const BodyPoseSolver::Diag& d,
                   const ContactDetector::Result& cr,
                   const SkinArtifactState& skin,
                   const LocomotionClassifier& loco)
{
    if (!testEnabled) return;
    static int frameCounter = 0;
    ++frameCounter;

    if (glovesEnabled) {
        static bool antropDumped = false;
        if (!antropDumped) {
            antropDumped = true;
            const double H = 1.75;  // §57 эталонный рост (= fb::kRefHeightM), только для диагностического дампа
            const double hStand = fb::pelvisStandHeightM(H);
            const double hSit   = fb::pelvisSitHeightM(H);
            const double tLen   = fb::trunkLengthM(H);
            std::cout << "[antrop] ref H=" << H << " m"
                      << "  pelvisStand=" << hStand << " m"
                      << "  pelvisSit=" << hSit << " m"
                      << "  trunk=" << tLen << " m"
                      << "  ankle=" << fb::ankleHeightM(H) << " m"
                      << "  (legacy 0.55·H=" << 0.55 * H << " m, delta="
                      << (hStand - 0.55 * H) << " m)\n";
            std::cout << "[engine] gravity=" << fb::constants::kGravityMs2
                      << " m/s²"
                      << "  sampleRate=" << fb::constants::kSampleRateHz
                      << " Hz"
                      << "  dt=" << fb::constants::kSampleDtSec << " s"
                      << "  (spec §41.1)\n";

            const QVector3D m0Body = fb::referenceM0BodyVec();
            const QVector3D m0Free = fb::referenceM0FreeFieldVec();
            std::cout << "[mag]    m0_body=(" << std::setprecision(3)
                      << m0Body.x() << "," << m0Body.y() << "," << m0Body.z()
                      << ") dip=" << fb::kMagnet.inclinationDeg << "°"
                      << "  m0_free=(" << m0Free.x() << "," << m0Free.y()
                      << "," << m0Free.z() << ") dip="
                      << (fb::kMagnet.inclinationDipRad * fb::constants::kRad2Deg) << "°"
                      << "  (spec §51.1)\n";

            std::cout << "[mag-gate] arms ×" << fb::kMagGateRelax[8].angleMul
                      << " (e_incl_arm)  head ×" << fb::kMagGateRelax[6].angleMul
                      << " (skull)  feet ×" << fb::kMagGateRelax[17].angleMul
                      << " (sole steel)  T8 ×" << fb::kMagGateRelax[4].angleMul
                      << "  (spec §51.3 / §51.6)\n";

            std::cout << "[bio §45.1] c_spine=["
                      << fb::kCSpine[0] << ", " << fb::kCSpine[1] << ", "
                      << fb::kCSpine[2] << ", " << fb::kCSpine[3] << ", "
                      << fb::kCSpine[4] << " (spineNeck.cNeck), "
                      << fb::kCSpine[5] << ", " << fb::kCSpine[6] << ", "
                      << fb::kCSpine[7] << ", " << fb::kCSpine[8]
                      << "]  (Pelvis→T8 distribution + cervical chain)\n";
            std::cout << "[bio §45.3] c_pelvis=[" << fb::kCPelvis[0]
                      << " (frac), " << fb::kCPelvis[1] << "° (ramp scale)]  latTiltPenalty="
                      << fb::kPelvisLatTiltPenalty << " (engine heuristic, not in xsb V[2])\n";
            std::cout << "[bio §316.6] c_femoropelvic=" << fb::kCFemoropelvic
                      << " (anti-pelvic-tilt on hip flexion)\n";
            std::cout << "[bio §46.1] c_arms=[" << fb::kCArms[0]
                      << " (X), " << fb::kCArms[1] << " (Y), "
                      << fb::kCArms[2] << " (max)]"
                      << "  scapulo-humeral piecewise-linear\n";
            std::cout << "[bio §47.1] c_knees=[" << fb::kCKnees[0]
                      << ", " << fb::kCKnees[1] << "]"
                      << "  screw-home max " << fb::kKneeScrewMaxDeg << "°\n";
            std::cout << "[bio §48.1] c_ankles=[" << fb::kCAnkles[0]
                      << ", " << fb::kCAnkles[1] << " (=" << (fb::kCAnkles[1]
                        * fb::constants::kRad2Deg) << "° pf), "
                      << fb::kCAnkles[2] << ", " << fb::kCAnkles[3] << "]\n";
            std::cout << "[bio §48.2] c_toes=[" << fb::kCToes[0]
                      << ", " << fb::kCToes[1] << ", " << fb::kCToes[2]
                      << ", " << fb::kCToes[3] << ", " << fb::kCToes[4]
                      << ", " << fb::kCToes[5] << " (= sin5°)]"
                      << "  rocker " << (fb::kToeRockerLowRad *
                          fb::constants::kRad2Deg) << "°→"
                      << (fb::kToeRockerHighRad *
                          fb::constants::kRad2Deg) << "°\n";

            std::cout << "[bio §37.4] sd_lump_joint=" << fb::kSdLumpRad
                      << "  weight=" << fb::kLumpStiffness
                      << " (7 groups: upperbody, R/L leg, R/L foot, R/L arm)\n";

            std::cout << "[bio §38.5] skin GM tau=" << fb::kSkin.tauSec
                      << " s base, fast=" << fb::kSkin.tauFastSec
                      << " s slow=" << fb::kSkin.tauSlowSec
                      << " s  σ_ori=" << fb::kSkin.sigmaOriDeg
                      << "° σ_pos=" << fb::kSkin.sigmaPosM << " m\n";

            {
                bool ratOk = true, dirOk = true;
                for (double b : {0.0, 0.5, 1.0}) {
                    const double s = std::sqrt(std::max(0.0,
                        fox::kSolverC2 - fox::kSolverC1 * b * b));
                    for (double x : {-1.0, 0.0, 2.0}) {
                        const double den = x * s + (fox::kSolverC2 - fox::kSolverC1);
                        if (std::abs(den) > 1e-12) {
                            const double expect = (x * s + fox::kSolverC2) / den;
                            if (std::abs(fox::solverRationalRatio(x, b) - expect) > 1e-9)
                                ratOk = false;
                        }
                        const double dir = fox::solverDirection(x, b, 1.0);
                        if (!std::isfinite(dir) || std::abs(dir) > 1.0 + 1e-9)
                            dirOk = false;
                    }
                }
                std::cout << "[solver §13.2д] C₁=" << fox::kSolverC1
                          << " C₂=" << fox::kSolverC2
                          << "  rational-form self-check: ratio="
                          << (ratOk ? "PASS" : "FAIL")
                          << " dir=" << (dirOk ? "PASS" : "FAIL") << "\n";
            }

            std::cout << "[finger-fk §91] 19-joint hand model (per joint: "
                      "type, flex range, abd range):\n";
            for (int j = 0; j < int(fb::kFingerRom.size()); ++j) {
                const auto& r = fb::kFingerRom[j];
                std::cout << "[finger-fk §91]  j=" << std::setw(2) << j
                          << "  " << std::left << std::setw(10) << r.label
                          << std::right
                          << "  " << std::left << std::setw(18)
                          << fb::fingerJointTypeName(fb::kFingerJointTypes[j])
                          << std::right
                          << "  flex=[" << std::setw(6) << std::fixed
                          << std::setprecision(1) << r.flxMin << "°,"
                          << std::setw(6) << r.flxMax << "°]"
                          << "  abd=[" << std::setw(6) << r.abdMin << "°,"
                          << std::setw(6) << r.abdMax << "°]"
                          << "\n";
            }

            std::cout << "[finger-fk §92.1] Right Carpus 6 anchor points "
                      "(m, hand-local):\n";
            for (const auto& cp : fb::kRightCarpusPoints) {
                std::cout << "[finger-fk §92.1]  " << std::left << std::setw(14)
                          << cp.label << std::right << "  ("
                          << std::setw(7) << std::fixed << std::setprecision(4)
                          << cp.x << "," << std::setw(8) << cp.y << ","
                          << std::setw(8) << cp.z << ")\n";
            }

            std::cout << "[glove-kfa §94.2] gloveBase   (per glove sensor):"
                      "  nominalT=" << fb::kGloveBase.nominalT << " s"
                      "  σ_init_ori=" << fb::kGloveBase.sdInitOrientDeg << "°"
                      "  σ_init_gyrBias=" << fb::kGloveBase.sdInitGyrBiasDeg << "°"
                      "  gyrBiasStd=[" << fb::kGloveBase.gyrBiasStdMinDeg
                      << "°, " << fb::kGloveBase.gyrBiasStdMaxDeg << "°]"
                      "  magResThr=" << fb::kGloveBase.magResThreshold
                      << "  magResWin=" << fb::kGloveBase.magResWinTime << " s\n";
            std::cout << "[glove-kfa §94.2]  accDivMon: th=" << fb::kGloveBase.accDivMonThreshold
                      << " velTh=" << fb::kGloveBase.accDivMonVelThreshold
                      << " time=" << fb::kGloveBase.accDivMonTime << " s"
                      << " decay=" << fb::kGloveBase.accDivMonTauDecay
                      << " highBoost=" << fb::kGloveBase.accDivMonThresholdHighBoost
                      << "\n";
            std::cout << "[glove-kfa §94.2]  fAccBoost: max=" << fb::kGloveBase.fAccBoost
                      << " inc=" << fb::kGloveBase.fAccBoostIncreaseTime << " s"
                      << " dec=" << fb::kGloveBase.fAccBoostDecreaseTime << " s\n";
            std::cout << "[glove-kfa §94.2]  τ_M0: fast=" << fb::kGloveBase.tauM0AvgFast
                      << " med=" << fb::kGloveBase.tauM0AvgMedium
                      << " slow=" << fb::kGloveBase.tauM0AvgSlow << " s"
                      << "  Q_acc_lp=" << fb::kGloveBase.sQvAccLowPass
                      << "  Q_mag_rw=" << fb::kGloveBase.sQvMagRandomWalk << "\n";
            std::cout << "[glove-kfa §94.3] gloveHuman overrides:"
                      "  accDivMonHighBoost=" << fb::kGloveHuman.accDivMonThresholdHighBoost
                      << "  doProjectMagOnHoriPlane="
                      << (fb::kGloveHuman.doProjectMagOnHoriPlane ? "true" : "false")
                      << "  doRedefineTemporal="
                      << (fb::kGloveHuman.doRedefineTemporal ? "true" : "false")
                      << "\n";
            std::cout << "[glove-kfa §94.4] gloveVRU overrides:"
                      "  doMagnetometerUpdate="
                      << (fb::kGloveVRU.doMagnetometerUpdate ? "true" : "false")
                      << "  doM0Update="
                      << (fb::kGloveVRU.doM0Update ? "true" : "false")
                      << "  doZru=" << (fb::kGloveVRU.doZru ? "true" : "false")
                      << "\n";
            std::cout << std::setprecision(4);
            std::cout.flush();
        }
    }

    if ((frameCounter % 100) != 0) return;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "[wls] iter=" << d.iterations
       << "  r_avg=" << d.residualMeanRad
       << "  r_post=" << d.residualMeanPostRad
       << "  step=" << d.stepNormRad
       << "  lambda=" << std::scientific << std::setprecision(2) << d.lambda
       << std::fixed << std::setprecision(4)
       << "  rej=" << d.rejectedSteps
       << "  σ_ori_max=" << std::setprecision(3) << (d.covMaxOriRad * 180.0 / M_PI) << "°"
       << "  σ_seg=" << d.covMaxOriSeg
       << std::setprecision(4)
       << "  poseQ=" << d.poseQualityBand
       << " (" << (d.poseQualityBand == fb::PoseQualityExcellent ? "excellent"
                  : d.poseQualityBand == fb::PoseQualityGood      ? "good"
                  : d.poseQualityBand == fb::PoseQualityAdequate  ? "adequate"
                  : d.poseQualityBand == fb::PoseQualityPoor      ? "poor"
                                                                   : "invalid")
       << ")  rows=" << d.numRows
       << "  outlier(rej/h/s/win)=" << d.outlierRejected
       << "/" << d.outlierHuber
       << "/" << d.outlierSoft
       << "/" << d.outlierWindowed
       << "  zupt_rows=" << d.zuptActiveRows
       << "  aiding_bias_max=" << std::setprecision(3)
                                << (d.aidingBiasMaxMps * 1000.0) << "mm/s"
       << std::setprecision(4)
       << "\n";

    ss << "[loco] phase=" << locomotionPhaseName(loco.phase())
       << "  flight=" << std::setprecision(2) << loco.flightFracSec() << "s"
       << "  contact=" << loco.contactFracSec() << "s"
       << "  pelvis_tilt=" << std::setprecision(1) << loco.pelvisTiltDeg() << "°"
       << "  t8_tilt=" << loco.t8TiltDeg() << "°"
       << "  posture=" << LocomotionClassifier::postureName(loco.posture())
       << std::setprecision(4) << "\n";

    if (glovesEnabled) {
        const QVector3D cop = cr.active.empty()
            ? QVector3D(0, 0, 0)
            : [&cr]() {
                  QVector3D sum(0, 0, 0); double w = 0.0;
                  for (const auto& c : cr.active) {
                      sum += c.p_world * float(c.probability);
                      w += c.probability;
                  }
                  return (w > 0.0) ? (sum / float(w)) : QVector3D(0, 0, 0);
              }();
        ss << "[CoP]  active=" << cr.active.size()
           << "  centre=(" << std::setprecision(3) << cop.x()
           << "," << cop.y() << "," << cop.z() << ")"
           << std::setprecision(4) << "\n";
    }

    if (!cr.active.empty()) {
        ss << "[zupt-wls] active=" << cr.active.size();
        for (const auto& c : cr.active) {
            ss << " seg" << c.seg << "pt" << c.pointId
               << "(P=" << std::setprecision(2) << c.probability
               << " v=" << std::setprecision(3)
               << std::sqrt(double(c.v_world.x()*c.v_world.x()) +
                            double(c.v_world.y()*c.v_world.y()) +
                            double(c.v_world.z()*c.v_world.z()))
               << ")" << std::setprecision(4);
        }
        ss << "\n";
    }

    if (glovesEnabled) {

        ss << std::fixed << std::setprecision(2);
        ss << "[coupling-wls spine]  full=" << d.spineFullDeg
           << "° fracs L5/L3/T12=" << d.spineFracL5
           << "/" << d.spineFracL3
           << "/" << d.spineFracT12
           << "  c_axial(Z)=" << fb::kCSpine[4] << "\n";

        ss << "[coupling-wls pelvis] c_pelvis=[" << fb::kCPelvis[0]
           << ", " << fb::kCPelvis[1] << "°]"
           << "  (frac=0.35, ramp=25° engagement scale)\n";

        ss << "[coupling-wls arm]    R: humerusInT8=" << d.scapThetaRDeg
           << "° scap=" << (d.scapCEffR * d.scapThetaRDeg) << "° c_eff_avg=" << d.scapCEffR
           << "   L: humerusInT8=" << d.scapThetaLDeg
           << "° scap=" << (d.scapCEffL * d.scapThetaLDeg) << "° c_eff_avg=" << d.scapCEffL
           << "\n";
        ss << "[coupling-wls arm-c]  c_arms=["
           << fb::kCArms[0] << " (X), " << fb::kCArms[1] << " (Y), "
           << fb::kCArms[2] << " (max)]"
           << "  ramp=[" << fb::kScapHumThetaLowDeg << "°, "
           << fb::kScapHumThetaHighDeg << "°]\n";
        ss << "[coupling-wls knee]   R: flex=" << d.kneeFlexRDeg
           << "° screw=" << d.kneeScrewRDeg
           << "°   L: flex=" << d.kneeFlexLDeg
           << "° screw=" << d.kneeScrewLDeg << "°\n";
        ss << "[coupling-wls ankle]  R: pf=" << d.anklePfRDeg
           << "°" << (d.ankleClampedR ? " CLAMP" : "")
           << "   L: pf=" << d.anklePfLDeg
           << "°" << (d.ankleClampedL ? " CLAMP" : "") << "\n";
        ss << "[coupling-wls toe]    R: mtp=" << d.toeMtpRDeg
           << "° w=" << d.toeWeightR
           << "   L: mtp=" << d.toeMtpLDeg
           << "° w=" << d.toeWeightL << "\n";

        ss << "[coupling-wls leg-bi] c_legs=["
           << fb::kCLegs[0] << ", " << fb::kCLegs[1]
           << "] (xsb V[2])  effective gain=" << fb::kCLegs[1]
           << " (hipFlexY · c_legs[1] → target ankle dorsi Y)\n";
        ss << std::setprecision(4);
    }

    if (glovesEnabled) {

        ss << "[skin]";
        for (int i = 0; i < fb::kSegmentCount; ++i) {
            if (!fb::kSensorPresent[i]) continue;
            const QVector3D x = skin.drift(i);
            const double mag = std::sqrt(double(x.x()*x.x()) +
                                         double(x.y()*x.y()) +
                                         double(x.z()*x.z())) * kR2D;
            if (mag > 0.05) {
                ss << " seg" << i << "=" << std::setprecision(2) << mag
                   << "°τ=" << std::setprecision(3) << skin.tauLast(i) << "s"
                   << std::setprecision(4);
            }
        }
        ss << "\n";

        ss << "[skin-pos]";
        bool any = false;
        for (int i = 0; i < fb::kSegmentCount; ++i) {
            if (!fb::kSensorPresent[i]) continue;
            const QVector3D x = skin.driftPos(i);
            const double mag = std::sqrt(double(x.x()*x.x()) +
                                         double(x.y()*x.y()) +
                                         double(x.z()*x.z())) * 1000.0;
            if (mag > 0.5) {
                ss << " seg" << i << "=" << std::setprecision(1) << mag
                   << "mm" << std::setprecision(4);
                any = true;
            }
        }
        if (!any) ss << " (all <0.5 mm)";

        ss << "  updatedSegs=" << pose_solver::g_refiner().lastSkinPosUpdates();
        ss << "\n";
    }

    if (glovesEnabled) {

        ss << "[wls-batch §44.8] mode=rts  windowFrames="
           << pose_solver::BatchSmoother::kWindow
           << "  maxIKSteps=" << fb::kMaxIKSteps << "\n";
        ss << "[marg §44.7] mode=schur (ring decimation, drop-oldest;"
              " stdOriFreeze=" << fb::kEstimator.stdOriFreeze
           << ", stdPosFreeze=" << fb::kEstimator.stdPosFreeze << ")\n";
    }

    if (d.rejectedSteps >= BodyPoseSolver::kMaxLambdaRetries) {
        ss << "[wls-event] no descent direction"
           << "  lambda=" << std::scientific << std::setprecision(2)
                          << d.lambda
           << "  rej=" << std::fixed << d.rejectedSteps << "\n";
    }

    std::cout << ss.str();
    std::cout.flush();
}

}

void SkeletonXsens::buildDefaultAngles()
{
    m_defAng = defaultSegAnglesFor(m_pose);

    if (fox::pose_solver::g_testFlag().load(std::memory_order_relaxed) &&
        fox::pose_solver::g_glovesFlag().load(std::memory_order_relaxed)) {
        namespace fb = fox::body;
        const auto& ref = (m_pose == "tpose") ? fb::kRefQuatT : fb::kRefQuatN;
        std::cout << "[def-ang] pose=" << m_pose
                  << "  per-segment coord transform vs anatomic reference\n";
        for (int s = 0; s < kXsensSegmentCount; ++s) {
            const Quat& qC = m_defAng[s];
            const Quat& qR = ref[s];
            std::cout << "  s=" << std::setw(2) << s
                      << " " << std::left << std::setw(14) << kSegmentNames[s] << std::right
                      << std::fixed << std::setprecision(4)
                      << "  defAng=(" << qC.w << "," << qC.x << ","
                                       << qC.y << "," << qC.z << ")"
                      << "  ref=("    << qR.w << "," << qR.x << ","
                                       << qR.y << "," << qR.z << ")"
                      << "  refAngle=" << std::setprecision(2)
                      << quat_angle_deg(qR) << "°\n";
        }
        std::cout.flush();
    }
}

std::array<double, 5> SkeletonXsens::defaultLimbCm(fox::body::Gender gender, double heightCm)
{
    namespace fb = fox::body;
    auto specLen = [](int seg) {
        return double(fb::kSensorToBone[seg].L_bone.length());
    };
    const double h = heightCm / 100.0;
    const double heightScale = (h > 1e-3) ? (h / fb::kRefHeightM) : 1.0;
    const auto&  anthro = fb::anthroFor(gender);

    const double refArmOneSide =
        specLen(fb::kSEG_RShoulder)     + specLen(fb::kSEG_RShoulder + 1) +
        specLen(fb::kSEG_RShoulder + 2) + specLen(fb::kSEG_RShoulder + 3);
    const double refSpanM = 2.0 * refArmOneSide + 2.0 * 0.08;
    const double anthroArmSpanM =
        2.0 * (anthro.upperArmRatio + anthro.forearmRatio + anthro.handRatio) * h;
    const double armScale = (refSpanM > 1e-6) ? anthroArmSpanM / refSpanM : heightScale;

    const double refLegM = specLen(fb::kSEG_RUpperLeg) + specLen(fb::kSEG_RLowerLeg)
                         + fb::ankleHeightM(fb::kRefHeightM);
    const double anthroLegM =
        (anthro.thighRatio + anthro.shankRatio + anthro.ankleHeightRatio) * h;
    const double legScale = (refLegM > 1e-6) ? anthroLegM / refLegM : heightScale;

    return {{
        specLen(fb::kSEG_RShoulder + 1) * armScale * 100.0,
        specLen(fb::kSEG_RShoulder + 2) * armScale * 100.0,
        specLen(fb::kSEG_RShoulder + 3) * armScale * 100.0,
        specLen(fb::kSEG_RUpperLeg)     * legScale * 100.0,
        specLen(fb::kSEG_RLowerLeg)     * legScale * 100.0,
    }};
}

void SkeletonXsens::buildLengths(const ActorConfig& actor)
{

    namespace fb = fox::body;

    auto specLen = [](int seg) {
        return double(fb::kSensorToBone[seg].L_bone.length());
    };

    const double h  = actor.heightCm     / 100.0;
    const double fl = actor.footLengthCm / 100.0;
    const double heightScale = (h > 1e-3) ? (h / fb::kRefHeightM) : 1.0;
    const auto&  anthro = fb::anthroFor(actor.gender);

    const double refArmOneSide =
        specLen(fb::kSEG_RShoulder) +
        specLen(fb::kSEG_RShoulder + 1) +
        specLen(fb::kSEG_RShoulder + 2) +
        specLen(fb::kSEG_RShoulder + 3);
    // §57 полуширина лопаток 0.08 м — опорный размах рук для антропометрической подгонки armSpan
    const double refScapHalfY = 0.08;
    const double refSpanM = 2.0 * refArmOneSide + 2.0 * refScapHalfY;
    const double anthroArmSpanM = 2.0 * (anthro.upperArmRatio + anthro.forearmRatio + anthro.handRatio) * h;
    const double targetArmSpanM = (actor.armSpanCm > 0.0)
        ? std::max(fb::kAnthroFloors.armSpanMin, actor.armSpanCm / 100.0)
        : anthroArmSpanM;
    double armScale = (refSpanM > 1e-6) ? targetArmSpanM / refSpanM : heightScale;

    const double refLegM = specLen(fb::kSEG_RUpperLeg)
                         + specLen(fb::kSEG_RLowerLeg)
                         + fb::ankleHeightM(fb::kRefHeightM);
    const double anthroLegM = (anthro.thighRatio + anthro.shankRatio + anthro.ankleHeightRatio) * h;
    const double targetLegM = (actor.legLengthCm > 0.0)
        ? std::max(fb::kAnthroFloors.legLengthMin, actor.legLengthCm / 100.0)
        : anthroLegM;
    double legScale = (refLegM > 1e-6) ? targetLegM / refLegM : heightScale;

    double heelToBallM, ballToTipM;
    {
        const QVector3D footBoneVec = fb::kSensorToBone[fb::kSEG_RFoot].L_bone;
        const QVector3D toeBoneVec  = fb::kSensorToBone[18].L_bone;
        const double specFootX = std::abs(double(footBoneVec.x()));
        const double specToeX  = std::abs(double(toeBoneVec.x()));
        if (fl > 0.05) {
            const double denom = specFootX + specToeX;
            const double frac = (denom > 1e-6) ? (specFootX / denom) : 0.69;
            heelToBallM = fl * frac;
            ballToTipM  = fl * (1.0 - frac);
        } else {
            heelToBallM = specFootX * heightScale;
            ballToTipM  = specToeX  * heightScale;
        }
    }

    const double pelvisHalfM = (actor.hipWidthCm > 0.0)
        ? std::max(fb::kAnthroFloors.hipHalfMin, actor.hipWidthCm / 200.0)
        : 0.5 * anthro.hipWidthRatio * h;
    const double scapHalfM = (actor.shoulderWidthCm > 0.0)
        ? std::max(fb::kAnthroFloors.scapHalfMin, actor.shoulderWidthCm / 200.0)
        : 0.5 * anthro.shoulderWidthRatio * h * (0.08 / (0.5 * 0.234));

    double refTrunkM = 0.0;
    for (int s = 0; s <= 5; ++s) refTrunkM += specLen(s);
    const double targetTrunkM = (actor.trunkLengthCm > 0.0)
        ? std::max(fb::kAnthroFloors.trunkLengthMin, actor.trunkLengthCm / 100.0)
        : anthro.trunkRatio * h;
    double trunkScale = (refTrunkM > 1e-6) ? targetTrunkM / refTrunkM : heightScale;

    auto spineLen = [&](int s) { return float(specLen(s) * trunkScale); };
    auto armLen   = [&](int s) { return float(specLen(s) * armScale);   };
    auto legLen   = [&](int s) { return float(specLen(s) * legScale);   };

    // §57/§XVIII анатомический инсет плеча внутрь от ширины плеч (0.10·trunkScale)
    const double inShoulderOffsetM = 0.10 * trunkScale;

    const std::array<float, kXsensSegmentCountWithDummies> L = {

        spineLen(0),
        spineLen(1),
        spineLen(2),
        spineLen(3),
        spineLen(4),
        spineLen(5),
        spineLen(6),

        float(scapHalfM),
        float(inShoulderOffsetM),
        armLen(8),
        armLen(9),
        armLen(10),

        float(scapHalfM),
        float(inShoulderOffsetM),
        armLen(12),
        armLen(13),
        armLen(14),

        float(pelvisHalfM),
        legLen(15),
        legLen(16),
        float(heelToBallM),
        float(ballToTipM),

        float(pelvisHalfM),
        legLen(19),
        legLen(20),
        float(heelToBallM),
        float(ballToTipM),
    };
    m_len = L;

    auto localFor = [&](int chainIdx, int origSeg, double scale) -> QVector3D {
        const QVector3D Lspec = fb::kSensorToBone[origSeg].L_bone * float(scale);
        return vec_rotate(Lspec, m_defAng[origSeg].conj());
    };

    const double specFootX_m = std::abs(double(fb::kSensorToBone[fb::kSEG_RFoot].L_bone.x()));
    const double specToeX_m  = std::abs(double(fb::kSensorToBone[18].L_bone.x()));
    const double footScaleR  = (specFootX_m > 1e-6) ? heelToBallM / specFootX_m : heightScale;
    const double toeScaleR   = (specToeX_m  > 1e-6) ? ballToTipM  / specToeX_m  : heightScale;

    m_localOffset = {{

        localFor(0, 0, trunkScale),  localFor(1, 1, trunkScale),
        localFor(2, 2, trunkScale),  localFor(3, 3, trunkScale),
        localFor(4, 4, trunkScale),  localFor(5, 5, trunkScale),
        localFor(6, 6, trunkScale),

        QVector3D(float(scapHalfM), 0.0f, 0.0f),

        localFor(8,  7,  armScale),  localFor(9,  8,  armScale),
        localFor(10, 9,  armScale),  localFor(11, 10, armScale),

        QVector3D(float(scapHalfM), 0.0f, 0.0f),

        localFor(13, 11, armScale),  localFor(14, 12, armScale),
        localFor(15, 13, armScale),  localFor(16, 14, armScale),

        QVector3D(float(pelvisHalfM), 0.0f, 0.0f),

        localFor(18, 15, legScale),  localFor(19, 16, legScale),
        localFor(20, 17, footScaleR), localFor(21, 18, toeScaleR),

        QVector3D(float(pelvisHalfM), 0.0f, 0.0f),

        localFor(23, 19, legScale),  localFor(24, 20, legScale),
        localFor(25, 21, footScaleR), localFor(26, 22, toeScaleR),
    }};

    auto overrideBone = [&](int arrayIdx, int origSeg, double cm) {
        if (cm <= 0.0) return;
        const double spec = specLen(origSeg);
        if (spec < 1e-9) return;
        const double sc = (cm / 100.0) / spec;
        m_len[arrayIdx] = float(cm / 100.0);
        m_localOffset[arrayIdx] = localFor(arrayIdx, origSeg, sc);
    };
    overrideBone(9,  8,  actor.upperArmCm); overrideBone(14, 12, actor.upperArmCm);
    overrideBone(10, 9,  actor.forearmCm);  overrideBone(15, 13, actor.forearmCm);
    overrideBone(11, 10, actor.handCm);     overrideBone(16, 14, actor.handCm);
    overrideBone(18, 15, actor.thighCm);    overrideBone(23, 19, actor.thighCm);
    overrideBone(19, 16, actor.shankCm);    overrideBone(24, 20, actor.shankCm);

    if (fox::pose_solver::g_testFlag().load(std::memory_order_relaxed) &&
        fox::pose_solver::g_glovesFlag().load(std::memory_order_relaxed)) {
        std::cout << "[fk-3d §39] actor h=" << h << " m  scales: trunk=" << trunkScale
                  << "  arm=" << armScale << "  leg=" << legScale
                  << "  foot=" << footScaleR << "\n";
        static const char* names[] = {
            "Pelvis→L5", "L5→L3", "L3→T12", "T12→T8", "T8→Neck", "Neck→Head",
            "Head→top", "[dummy R-scap]", "R-acromion→up", "RUpperArm→elbow",
            "RForeArm→wrist", "RHand→tip", "[dummy L-scap]", "L-acromion→up",
            "LUpperArm→elbow", "LForeArm→wrist", "LHand→tip", "[dummy R-hip]",
            "RUpperLeg→knee", "RLowerLeg→ankle", "RFoot→ball", "RToe→tip",
            "[dummy L-hip]", "LUpperLeg→knee", "LLowerLeg→ankle",
            "LFoot→ball", "LToe→tip" };
        for (int s = 0; s < kXsensSegmentCountWithDummies; ++s) {

            const QVector3D scalarVec(m_len[s], 0.0f, 0.0f);
            const QVector3D delta = m_localOffset[s] - scalarVec;
            std::cout << "[fk-3d §39]  s=" << std::setw(2) << s
                      << "  " << std::left << std::setw(18) << names[s] << std::right
                      << "  L_local=(" << std::fixed << std::setprecision(4)
                      << std::setw(8) << m_localOffset[s].x() << ","
                      << std::setw(8) << m_localOffset[s].y() << ","
                      << std::setw(8) << m_localOffset[s].z()
                      << ")  |L|=" << std::setw(7) << m_len[s] << " m"
                      << "  Δ_vs_scalar=(" << std::setw(8) << delta.x()
                      << "," << std::setw(8) << delta.y()
                      << "," << std::setw(8) << delta.z() << ")\n";
        }
        std::cout.flush();
    }
}

namespace {
struct RenderDiag {

    std::array<double, kXsensSegmentCount> jumpDeg{};
    std::array<double, kXsensSegmentCount> rejectW{};
    std::array<bool,   kXsensSegmentCount> gyroQuiet{};
    std::array<double, kXsensSegmentCount> localAng{};

    double spineW_L5 = 0.0, spineW_L3 = 0.0, spineW_T12 = 0.0, neckW = 0.5;

    double scapUpZR = 0.0, scapAngR = 0.0, scapUpZL = 0.0, scapAngL = 0.0;
    bool   scapActiveR = false, scapActiveL = false;
};
RenderDiag g_renderDiag{};
}

std::array<Quat, kXsensSegmentCountWithDummies>
SkeletonXsens::addDummySegments(const std::array<Quat, kXsensSegmentCount>& s) const
{
    namespace fb = fox::body;

    const double P = M_PI;
    const Quat t8yaw     = yaw_only_quat(s[SEG_T8]);
    const Quat pelvisYaw = yaw_only_quat(s[SEG_Pelvis]);
    const Quat rScapBase = quat_mult(t8yaw,     euler_to_quat(0, -P/2, -P/2, "XYZ"));
    const Quat lScapBase = quat_mult(t8yaw,     euler_to_quat(0, -P/2,  P/2, "XYZ"));
    const Quat rHipBase  = quat_mult(pelvisYaw, euler_to_quat(0,  0,   -P/2, "XYZ"));
    const Quat lHipBase  = quat_mult(pelvisYaw, euler_to_quat(0,  0,    P/2, "XYZ"));

    auto canon = [](const Quat& q) {
        return (q.w < 0.0) ? Quat(-q.w, -q.x, -q.y, -q.z) : q;
    };
    auto scapHumOffset = [&](const Quat& qUpperArm) -> Quat {
        const Quat qRel = canon(quat_mult(qUpperArm, s[SEG_T8].conj()).normalized());
        const QVector3D phi = quat_log(qRel);
        constexpr double kR2D = fb::constants::kRad2Deg;
        auto ramp = [](double aDeg, double cLow) {
            const double lo = fb::kScapHumThetaLowDeg;
            const double hi = fb::kScapHumThetaHighDeg;
            const double a = std::abs(aDeg);
            if (a <= lo) return cLow;
            if (a >= hi) return fb::kCArms[2];
            return cLow + (a - lo) / (hi - lo) * (fb::kCArms[2] - cLow);
        };
        const double cX = ramp(double(phi.x()) * kR2D, fb::kCArms[0]);
        const double cY = ramp(double(phi.y()) * kR2D, fb::kCArms[1]);
        return quat_exp_rotvec(cX * double(phi.x()),
                               cY * double(phi.y()), 0.0);
    };
    auto femoropelvicOffset = [&](const Quat& qUpperLeg) -> Quat {
        const Quat qRel = canon(quat_mult(qUpperLeg, s[SEG_Pelvis].conj()).normalized());
        const QVector3D phi = quat_log(qRel);
        const double cR = fb::kCFemoropelvic;
        return quat_exp_rotvec(cR * double(phi.x()),
                               cR * double(phi.y()), 0.0);
    };

    const Quat rScap = quat_mult(scapHumOffset(s[SEG_RUpperArm]), rScapBase).normalized();
    const Quat lScap = quat_mult(scapHumOffset(s[SEG_LUpperArm]), lScapBase).normalized();
    const Quat rHip  = quat_mult(femoropelvicOffset(s[SEG_RUpperLeg]), rHipBase).normalized();
    const Quat lHip  = quat_mult(femoropelvicOffset(s[SEG_LUpperLeg]), lHipBase).normalized();

    std::array<Quat, kXsensSegmentCountWithDummies> out{};
    int k = 0;
    for (int i = 0; i < 7; ++i) out[k++] = s[i];
    out[k++] = rScap;
    for (int i = 7;  i < 11; ++i) out[k++] = s[i];
    out[k++] = lScap;
    for (int i = 11; i < 15; ++i) out[k++] = s[i];
    out[k++] = rHip;
    for (int i = 15; i < 19; ++i) out[k++] = s[i];
    out[k++] = lHip;
    for (int i = 19; i < 23; ++i) out[k++] = s[i];

    if (fox::pose_solver::g_testFlag().load(std::memory_order_relaxed) &&
        fox::pose_solver::g_glovesFlag().load(std::memory_order_relaxed)) {
        static int dummyTick = 0;
        if ((++dummyTick % 120) == 0) {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "[dummy-seg] rScap_off="
                      << quat_angle_deg(quat_mult(rScap, rScapBase.conj()))
                      << "°  lScap_off="
                      << quat_angle_deg(quat_mult(lScap, lScapBase.conj()))
                      << "°  rHip_off="
                      << quat_angle_deg(quat_mult(rHip,  rHipBase.conj()))
                      << "°  lHip_off="
                      << quat_angle_deg(quat_mult(lHip,  lHipBase.conj())) << "°\n";
            std::cout.flush();
        }
    }
    return out;
}

// §V/§1158/§1159 прямая кинематика (FK) от корня: oriented[i]=raw⊗m_defAng[i] (поза §24),
//   уточнение позы PoseRefiner (§X), затем обход скелета kp[b]=kp[a]+R(global)·L_bone -> 28 keypoints (formules.txt)
std::array<QVector3D, kXsensKeypointCount>
SkeletonXsens::computeKeypoints(const std::array<Quat, kXsensSegmentCount>& raw,
                                const QVector3D& rootPos,
                                FkDiag* diag) const
{

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

    {

        const std::array<Quat, kXsensSegmentCount> sensorTargets = oriented;

        pose_solver::PoseRefiner::FrameContext ctx{};
        ctx.sensorMeas    = &sensorTargets;
        ctx.sensorPresent = &fox::body::kSensorPresent;
        ctx.segCenter     = m_haveLastSegCenter ? &m_lastSegCenter : nullptr;
        ctx.accLPBody     = m_accLPBodyValid    ? &m_accLPBodyHint : nullptr;
        ctx.floorLevelZ   = 0.0;
        ctx.dt            = fox::body::constants::kSampleDtSec;

        pose_solver::ContactDetector::Result contacts;
        std::lock_guard<std::mutex> lk(pose_solver::g_refinerMtx());
        const auto dg = pose_solver::g_refiner().refine(oriented, ctx, &contacts);

        g_renderDiag.spineW_L5  = dg.spineFracL5;
        g_renderDiag.spineW_L3  = dg.spineFracL3;
        g_renderDiag.spineW_T12 = dg.spineFracT12;
        {
            const double cNeck = 0.5 * (fox::body::kCSpine[5] +
                                        fox::body::kCSpine[6]);
            const double cHead = 0.5 * (fox::body::kCSpine[7] +
                                        fox::body::kCSpine[8]);
            const double sumNH = cNeck + cHead;
            g_renderDiag.neckW = (sumNH > 1e-9) ? (cNeck / sumNH) : 0.5;
        }
        g_renderDiag.scapUpZR   = std::sin(dg.scapThetaRDeg * M_PI / 180.0);
        g_renderDiag.scapUpZL   = std::sin(dg.scapThetaLDeg * M_PI / 180.0);
        g_renderDiag.scapActiveR= dg.scapThetaRDeg > 0.0;
        g_renderDiag.scapActiveL= dg.scapThetaLDeg > 0.0;
        g_renderDiag.scapAngR   = dg.scapCEffR;
        g_renderDiag.scapAngL   = dg.scapCEffL;

        pose_solver::dumpFrameDiag(
            pose_solver::g_testFlag().load(),
            pose_solver::g_glovesFlag().load(),
            dg, contacts, pose_solver::g_refiner().skin(),
            pose_solver::g_refiner().locomotion());
    }

    const auto global = addDummySegments(oriented);

    std::array<QVector3D, kXsensSegmentCountWithDummies> boneVec{};
    for (int s = 0; s < kXsensSegmentCountWithDummies; ++s) {
        boneVec[s] = vec_rotate(m_localOffset[s], global[s]);
    }

    std::array<QVector3D, kXsensKeypointCount> kp{};
    std::array<bool,      kXsensKeypointCount> seen{};
    kp[0]   = rootPos;
    seen[0] = true;
    for (int s = 0; s < kXsensSegmentCountWithDummies; ++s) {
        const int a = m_start[s];
        const int b = m_end[s];
        if (!seen[a]) continue;
        kp[b]  = kp[a] + boneVec[s];
        seen[b] = true;
    }

    if (diag) {
        diag->oriented    = oriented;
        diag->global      = global;
        diag->boneVec     = boneVec;
        diag->len         = m_len;
        diag->kp          = kp;
        diag->rootPos     = rootPos;
        diag->couplingOut = oriented;

        diag->ergo        = fox::ergo::jointAnglesErgoAll(oriented);
    }

    for (int s = 0; s < kXsensSegmentCountWithDummies; ++s) {
        const int a = m_start[s];
        const int b = m_end[s];
        if (a < kXsensSegmentCount) {
            const double ratio = fox::body::kWinterProxToComRatio[a];
            if (b < kXsensKeypointCount && b >= 0) {
                m_lastSegCenter[a] = kp[a] + (kp[b] - kp[a]) * float(ratio);
            } else {
                m_lastSegCenter[a] = kp[a];
            }
        }
    }
    m_haveLastSegCenter = true;

    for (int i = 0; i < kXsensSegmentCount; ++i) {
        const QVector3D r_bs = fox::body::kSensorToBone[i].r_bs;
        if (r_bs.lengthSquared() < 1e-12f) {
            m_lastSensorPos[i] = m_lastSegCenter[i];
        } else {

            m_lastSensorPos[i] = m_lastSegCenter[i] + vec_rotate(r_bs, oriented[i]);
        }
    }
    m_haveLastSensorPos = true;

    if (fox::pose_solver::g_testFlag().load(std::memory_order_relaxed) &&
        fox::pose_solver::g_glovesFlag().load(std::memory_order_relaxed)) {
        static bool sensorRbsDumped = false;
        if (!sensorRbsDumped) {
            sensorRbsDumped = true;
            std::cout << "[sensor-rbs §89] per-segment IMU mount r_bs (m) — sensor"
                         " offset on the bone, used for rendering and contact reasoning\n";
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                const QVector3D r_bs = fox::body::kSensorToBone[i].r_bs;
                if (r_bs.lengthSquared() < 1e-12f) continue;
                std::cout << "[sensor-rbs §89]  seg=" << std::setw(2) << i
                          << "  r_bs=(" << std::fixed << std::setprecision(5)
                          << std::setw(9) << r_bs.x() << ","
                          << std::setw(9) << r_bs.y() << ","
                          << std::setw(9) << r_bs.z() << ") m"
                          << "  |r_bs|=" << std::setw(8) << r_bs.length() << " m"
                          << "\n";
            }
            std::cout.flush();
        }
    }
    return kp;
}

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
    Quat n = q.normalized();
    if (n.w < 0.0) {
        n.w = -n.w; n.x = -n.x; n.y = -n.y; n.z = -n.z;
    }
    return n;
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
        m_posePrev = PoseUnknown;
        m_poseTicks = 0;
        m_zuptTicks = 0;
        m_lowZTicksR = m_lowZTicksL = 0;
        m_zSnapBlendTicks = 0;
        m_confRFrozenForRoll = m_confLFrozenForRoll = false;
        m_confRFrozenValue = m_confLFrozenValue = 0.0;

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

        const double dt = m_haveLast ? std::clamp(t - m_lastT, 1e-3, 0.1) : 0.01;

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

        const double a030 = rateAdjustAlpha(0.30, dt);
        const double a020 = rateAdjustAlpha(0.20, dt);
        const double a010 = rateAdjustAlpha(0.10, dt);

        auto lowest3 = [](const QVector3D& a, const QVector3D& b, const QVector3D& c){
            const QVector3D& ab = a.z() < b.z() ? a : b;
            return ab.z() < c.z() ? ab : c;
        };
        auto sstep_local = [](double x){ x = std::clamp(x,0.0,1.0); return x*x*(3.0-2.0*x); };
        const QVector3D fkRLowest = lowest3(fkRHeel, fkRBall, fkRTip);
        const QVector3D fkLLowest = lowest3(fkLHeel, fkLBall, fkLTip);

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
            const Quat qDP    = quat_mult(qPelvis, m_prevPelvisQ.inv()).normalized();
            const Quat dqYaw  = fox::yaw_only_quat(qDP);
            const double yawDelta = 2.0 * std::atan2(dqYaw.z, dqYaw.w);
            rawYawRate = yawDelta / dt;
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

        auto smoothstep01 = [](double x) {
            x = std::clamp(x, 0.0, 1.0);
            return x * x * (3.0 - 2.0 * x);
        };
        const bool freezeRing = (m_pose == PoseAirborne);
        if (freezeRing && !m_yawFrozenPrev) {
            m_fkxyHead  = 0;
            m_fkxyCount = 0;
        }
        if (!freezeRing && m_yawFrozenPrev && m_initialised) {

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

        const float xyR = xyRange(m_rFKXY);
        const float xyL = xyRange(m_lFKXY);
        const double stableHi = m_fkxyStableRange * 1.5;
        const double stableDen = std::max(1e-6, m_fkxyStableRange);
        const double rFKXYStableW = smoothstep01((stableHi - double(xyR)) / stableDen);
        const double lFKXYStableW = smoothstep01((stableHi - double(xyL)) / stableDen);
        const bool rFKXYStable = (rFKXYStableW > 0.5);
        const bool lFKXYStable = (lFKXYStableW > 0.5);
        m_dbgFkxyRangeR = double(xyR);    m_dbgFkxyRangeL = double(xyL);
        m_dbgFkxyStableWR = rFKXYStableW; m_dbgFkxyStableWL = lFKXYStableW;
        m_dbgYawFreezeW = 0.0;

        double tiltCos = 1.0;
        const PoseKind newPose = _classifyPose(qPelvis, fkR, fkL, tiltCos);
        m_lastTiltCos = tiltCos;
        if (newPose == m_pose)
            m_poseTicks = std::min(m_poseTicks + 1, 4096);
        else
            m_poseTicks = 0;
        m_pose = newPose;

        const float fkMinZ = std::min(fkR.z(), fkL.z());

        m_lowZTicksR = (fkR.z() - fkMinZ < float(m_lowZBandM))
                       ? std::min(m_lowZTicksR + 1, 4096) : 0;
        m_lowZTicksL = (fkL.z() - fkMinZ < float(m_lowZBandM))
                       ? std::min(m_lowZTicksL + 1, 4096) : 0;

        const QVector3D pHeelL = fb::kFootPointsRight[0].r_local;
        const QVector3D pBallL = fb::kFootPointsRight[3].r_local;
        const double footSep = std::max(0.05, double(pBallL.x() - pHeelL.x()));
        const double pzR = double(fox::vec_rotate(pBallL, qR).z()
                                - fox::vec_rotate(pHeelL, qR).z()) / footSep;
        const double pzL = double(fox::vec_rotate(pBallL, qL).z()
                                - fox::vec_rotate(pHeelL, qL).z()) / footSep;
        m_footPitchZR = (1.0 - a030) * m_footPitchZR + a030 * pzR;
        m_footPitchZL = (1.0 - a030) * m_footPitchZL + a030 * pzL;

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

        const float rangeR = xyR;
        const float rangeL = xyL;
        const double angFastR = smoothstep01((m_rAngV - (m_rollAngVThresh - 0.5)) / 1.0);
        const double angFastL = smoothstep01((m_lAngV - (m_rollAngVThresh - 0.5)) / 1.0);
        const double xyHi    = m_rollXYRangeMax * 1.33;
        const double xyLo    = m_rollXYRangeMax * 0.67;
        const double xyDen   = std::max(1e-6, xyHi - xyLo);
        const double xyTightR = smoothstep01((xyHi - double(rangeR)) / xyDen);
        const double xyTightL = smoothstep01((xyHi - double(rangeL)) / xyDen);
        const double rollingWR = angFastR * xyTightR;
        const double rollingWL = angFastL * xyTightL;
        const bool rollingR = (rollingWR > 0.5);
        const bool rollingL = (rollingWL > 0.5);
        m_dbgRollingWR = rollingWR;  m_dbgRollingWL = rollingWL;

        if (!m_initialised) {

            const double pelvisZ_loco_init = fox::body::pelvisStandHeightM(m_actorHeightM);
            float targetZ = -fkMinZ;
            if (m_pose == PoseLying) targetZ = float(-pelvisZ_loco_init);
            else if (m_pose == PoseSit) targetZ = float(fox::body::pelvisSitHeightM(m_actorHeightM)
                                                        - pelvisZ_loco_init);
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
            m_anchor  = m_anchorR;
            m_support = (fkR.z() <= fkL.z()) ? RIGHT : LEFT;
            return m_offsetLast;
        }

        auto sStill = [&](double angv) -> double {
            double s = (m_stillRad - angv) / m_stillRad;
            return std::max(0.0, std::min(1.0, s));
        };
        auto sLow = [&](const QVector3D& fk) -> double {
            double dz = fk.z() - fkMinZ;
            double s = 1.0 - dz / std::max(m_heightMarginSlow, 1e-3);
            return std::max(0.0, std::min(1.0, s));
        };

        const double rawCR = std::max(sStill(m_rAngV), rFKXYStableW) * sLow(fkR);
        const double rawCL = std::max(sStill(m_lAngV), lFKXYStableW) * sLow(fkL);
        m_dbgRawCR = rawCR;  m_dbgRawCL = rawCL;
        auto smooth = [](double prev, double raw, double rise, double fall) {
            const double r = (raw > prev) ? rise : fall;
            return (1.0 - r) * prev + r * raw;
        };
        const double newConfR = smooth(m_confR, rawCR, m_confRiseRate, m_confFallRate);
        const double newConfL = smooth(m_confL, rawCL, m_confRiseRate, m_confFallRate);

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

        const double pelvisStill = std::max(1e-3, m_pelvisStillRad);
        const double pelvisRange = std::max(pelvisStill,
                                            fox::body::kContact.highVelTh);
        const double pelvisRotKill =
            std::max(0.0,
                     std::min(1.0,
                              (m_pelvisAngV - pelvisStill) / pelvisRange));
        const bool pelvisRotating = pelvisRotKill > 0.5;
        m_dbgPelvisRotKill = pelvisRotKill;

        {

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

                    if (m_pose != PoseAirborne) {
                        m_pose = PoseAirborne;
                        m_poseTicks = 0;
                    } else {
                        m_poseTicks = std::min(m_poseTicks + 1, 4096);
                    }
                }
            } else {
                if (m_airborneTicks > 0) {

                    m_landedTicks = m_landedRampTicks;
                }
                m_airborneTicks = 0;
            }
            if (m_landedTicks > 0) --m_landedTicks;
        }

        auto bestPelvisEstimate = [&]() -> QVector3D {

            if (m_committedR && !m_committedL)
                return QVector3D(m_anchorR.x() - fkR.x(),
                                 m_anchorR.y() - fkR.y(),
                                 m_offsetLast.z());
            if (m_committedL && !m_committedR)
                return QVector3D(m_anchorL.x() - fkL.x(),
                                 m_anchorL.y() - fkL.y(),
                                 m_offsetLast.z());

            return QVector3D(m_offsetCommitted.x(),
                             m_offsetCommitted.y(),
                             m_offsetLast.z());
        };

        const double cbR_new = heelContactWR_pre - toeContactWR_pre;
        const double cbL_new = heelContactWL_pre - toeContactWL_pre;
        if (m_committedR && std::abs(cbR_new - m_contactBlendR) > 0.05) {
            const double alpha = std::min(1.0, 0.40 * std::abs(cbR_new - m_contactBlendR));
            const QVector3D pelvis = bestPelvisEstimate();
            const QVector3D newAnchorR(fkR.x() + pelvis.x(),
                                       fkR.y() + pelvis.y(),
                                       m_anchorR.z());
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

        m_contactBlendR = (1.0 - a020) * m_contactBlendR + a020 * cbR_new;
        m_contactBlendL = (1.0 - a020) * m_contactBlendL + a020 * cbL_new;

        bool didCommitThisFrame = false;
        const bool wasCommittedR = m_committedR;
        const bool wasCommittedL = m_committedL;
        auto maybeCommitRelease = [&](double conf, bool& committed,
                                      QVector3D& anchor, const QVector3D& fk,
                                      bool isRight, bool rolling) {

            if (pelvisRotating || rolling) return;
            if (!committed && conf >= m_confCommit) {

                const QVector3D pelvis = bestPelvisEstimate();

                m_offsetCommitted = QVector3D(pelvis.x(), pelvis.y(),
                                              m_offsetCommitted.z());
                QVector3D world(fk.x() + pelvis.x(),
                                fk.y() + pelvis.y(),
                                fk.z() + pelvis.z());

                if (m_pose == PoseStand) {
                    world.setZ(0.0f);
                } else if (m_pose == PoseSquat) {
                    const int lowTicks = isRight ? m_lowZTicksR : m_lowZTicksL;
                    const bool heelLifted = isRight ? m_heelLiftR : m_heelLiftL;
                    if (lowTicks >= m_lowZTicksRequired) {
                        if (heelLifted) {

                            const double bone = 0.60 * m_footLengthM;
                            const double sinp = isRight
                                    ? std::abs(m_footPitchZR)
                                    : std::abs(m_footPitchZL);

                            world.setZ(float(bone * sinp));
                        } else {
                            world.setZ(0.0f);
                        }
                        m_zSnapBlendTicks = m_zSnapBlendFrames;
                    }

                }

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
                committed = false;
            }
        };
        maybeCommitRelease(m_confR, m_committedR, m_anchorR, fkR, true,  rollingR);
        maybeCommitRelease(m_confL, m_committedL, m_anchorL, fkL, false, rollingL);
        if (didCommitThisFrame) m_recentCommitTicks = m_commitFadeTicks;
        else if (m_recentCommitTicks > 0) --m_recentCommitTicks;

        if (m_verbose || pose_solver::g_testFlag().load(
                std::memory_order_relaxed)) {
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

        const bool allStill = (m_pelvisAngV < m_pelvisStillRad)
                           && (m_rAngV < 0.15) && (m_lAngV < 0.15);
        m_zuptTicks = allStill ? (m_zuptTicks + 1) : 0;
        if (m_zuptTicks >= m_zuptTicksThresh && m_offsetReady) {
            m_contact.rightDown = m_committedR;
            m_contact.leftDown  = m_committedL;
            return m_offsetLast;
        }

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

        const double imbalance = (total > 1e-3) ? std::abs(effR - effL) / total : 0.0;
        m_dbgImbalance = imbalance;  m_dbgEffR = effR;  m_dbgEffL = effL;
        const double xyRate = m_offsetRateDouble
                            + (m_offsetRatePrimary - m_offsetRateDouble) * imbalance;
        const double effXyRate = xyRate * (1.0 - pelvisRotKill);
        double zRate = (m_pelvisAngV > m_pelvisStillRad)
                              ? m_zRatePelvisMoving
                              : m_zRatePelvisStill;

        if (m_zSnapBlendTicks > 0) {
            const double blendRate = 1.0 / double(std::max(1, m_zSnapBlendTicks));
            zRate = std::max(zRate, blendRate);
        }

        QVector3D newOff = m_offsetLast;

        if (m_pose != PoseAirborne && total > 1e-3) {
            newOff.setX(float((1.0 - effXyRate) * m_offsetLast.x()
                              + effXyRate * rawOff.x()));
            newOff.setY(float((1.0 - effXyRate) * m_offsetLast.y()
                              + effXyRate * rawOff.y()));
            newOff.setZ(float((1.0 - zRate) * m_offsetLast.z()
                              + zRate * rawOff.z()));
        }

        {

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

        if (m_pose == PoseStand && m_poseTicks >= m_poseStableTicks) {
            const double targetZ = fox::body::pelvisStandHeightM(m_actorHeightM);
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

        if (m_pose != m_posePrev && pose_solver::g_testFlag().load(
                std::memory_order_relaxed)) {
            auto poseName = [](LocomotionSolver::PoseKind p) -> const char* {
                switch (p) {
                    case LocomotionSolver::PoseUnknown:  return "Unknown";
                    case LocomotionSolver::PoseStand:    return "Stand";
                    case LocomotionSolver::PoseSit:      return "Sit";
                    case LocomotionSolver::PoseSquat:    return "Squat";
                    case LocomotionSolver::PoseLying:    return "Lying";
                    case LocomotionSolver::PoseAirborne: return "Airborne";
                }
                return "?";
            };
            std::cout << "[loco-event] pose: " << poseName(m_posePrev)
                      << " -> " << poseName(m_pose)
                      << "  ticks=" << m_poseTicks
                      << "  off=(" << std::fixed << std::setprecision(3)
                      << newOff.x() << "," << newOff.y() << "," << newOff.z()
                      << ")\n" << std::flush;
        }
        m_posePrev = m_pose;

        m_yawFrozenPrev = (m_pose == PoseAirborne);
        if (m_zSnapBlendTicks > 0) --m_zSnapBlendTicks;

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

        const double ux = 2.0 * (qPelvis.x*qPelvis.z + qPelvis.w*qPelvis.y);
        const double uy = 2.0 * (qPelvis.y*qPelvis.z - qPelvis.w*qPelvis.x);
        const double uz = 1.0 - 2.0 * (qPelvis.x*qPelvis.x + qPelvis.y*qPelvis.y);
        (void)ux; (void)uy;
        outTiltCos = uz;
        if (uz < m_lieTiltCosThresh) return PoseLying;
        const double pelvisZ_loco = fox::body::pelvisStandHeightM(m_actorHeightM);
        const double pelvisToFoot = pelvisZ_loco - double(std::min(fkR.z(), fkL.z()));
        if (pelvisToFoot < m_squatKneeThresh) return PoseSquat;
        if (pelvisToFoot < m_sitKneeThresh)   return PoseSit;
        return PoseStand;
    }

const char* connStatusName(ConnStatus s)
{
    switch (s) {
        case ConnStatus::NotInitialized: return "не инициализирован";
        case ConnStatus::NoDriver:       return "драйвер отсутствует";
        case ConnStatus::Scanning:       return "сканирование";
        case ConnStatus::NoDevice:       return "устройство не найдено";
        case ConnStatus::Connecting:     return "подключение";
        case ConnStatus::Streaming:      return "поток";
        case ConnStatus::Stale:          return "устаревший поток";
        case ConnStatus::Failed:         return "сбой";
    }
    return "?";
}

static double monotonicSec()
{
    using clk = std::chrono::steady_clock;
    static std::atomic<double> sLastT{0.0};
    const double now =
        std::chrono::duration<double>(clk::now().time_since_epoch()).count();
    constexpr double kEps = 1e-6;
    double last = sLastT.load(std::memory_order_relaxed);
    double t    = (now > last + kEps) ? now : (last + kEps);
    while (!sLastT.compare_exchange_weak(last, t,
                                         std::memory_order_relaxed)) {
        t = (now > last + kEps) ? now : (last + kEps);
    }
    return t;
}

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
        default: return -1;
    }
}

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
using FnEnableNetworkScanning         = void(*)();

using FnDeviceGotoConfig              = int(*)(void*);
using FnDeviceGotoMeasurement         = int(*)(void*);
using FnDeviceLocationId              = int(*)(void*);
using FnDeviceGetDataPacketCount      = int(*)(void*);
using FnDeviceTakeFirstDataPacketInQueue =
                                        XsDataPacketBlob*(*)(void*, XsDataPacketBlob*);
using FnDeviceLastAvailableLiveData   = XsDataPacketBlob*(*)(void*, XsDataPacketBlob*);
using FnDevicePacketErrorRate         = int(*)(void*);
using FnDeviceProductCode             = void*(*)(void*, void*);
using FnDeviceHardwareVersion         = void*(*)(void*, void*);

using FnScanPorts                     = void(*)(void*, int, int, int, int);
using FnArrayAt                       = void*(*)(void*, std::size_t);
using FnArrayDestruct                 = void(*)(void*);
using FnArraySize                     = std::size_t (*)(const void*);
using FnPortInfoArrayConstruct        = void(*)(void*, std::size_t, void*);
using FnDeviceIdArrayConstruct        = void(*)(void*, std::size_t, void*);
using FnStringConstruct               = void(*)(void*);
using FnStringDestruct                = void(*)(void*);
using FnStringCopyToWChar             = std::size_t(*)(const void*, wchar_t*, std::size_t);

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

using FnDataPacketContainsAcc    = int (*)(const XsDataPacketBlob*);
using FnDataPacketAcc            = XsVectorBlob*(*)(const XsDataPacketBlob*, XsVectorBlob*);
using FnDataPacketContainsGyro   = int (*)(const XsDataPacketBlob*);
using FnDataPacketGyro           = XsVectorBlob*(*)(const XsDataPacketBlob*, XsVectorBlob*);
using FnDataPacketContainsMag    = int (*)(const XsDataPacketBlob*);
using FnDataPacketMag            = XsVectorBlob*(*)(const XsDataPacketBlob*, XsVectorBlob*);
using FnVectorDestruct           = void(*)(XsVectorBlob*);

using FnDataPacketContainsVelocityIncrement = int (*)(const XsDataPacketBlob*);
using FnDataPacketVelocityIncrement         =
    XsVectorBlob*(*)(const XsDataPacketBlob*, XsVectorBlob*);
using FnDataPacketContainsFreeAcc = int (*)(const XsDataPacketBlob*);
using FnDataPacketFreeAcc         =
    XsVectorBlob*(*)(const XsDataPacketBlob*, XsVectorBlob*);
using FnDataPacketContainsSampleTimeFine = int (*)(const XsDataPacketBlob*);
using FnDataPacketSampleTimeFine         = quint32(*)(const XsDataPacketBlob*);
using FnDataPacketContainsTemperature    = int (*)(const XsDataPacketBlob*);
using FnDataPacketTemperature            = double (*)(const XsDataPacketBlob*);

struct Api {
    HMODULE xda = nullptr;
    HMODULE xst = nullptr;
    HMODULE iomp = nullptr;

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

    FnDeviceGotoConfig               deviceGotoConfig       = nullptr;
    FnDeviceGotoMeasurement          deviceGotoMeasurement  = nullptr;
    FnDeviceLocationId               deviceLocationId       = nullptr;
    FnDeviceProductCode              deviceProductCode      = nullptr;
    FnDeviceHardwareVersion          deviceHardwareVersion  = nullptr;
    FnStringConstruct                stringConstruct        = nullptr;
    FnStringDestruct                 stringDestruct         = nullptr;
    FnStringCopyToWChar              stringCopyToWChar      = nullptr;
    FnDeviceUpdateRate               deviceUpdateRate       = nullptr;
    FnDeviceGetDataPacketCount       deviceGetDataPacketCount = nullptr;
    FnDeviceTakeFirstDataPacketInQueue deviceTakeFirstDataPacketInQueue = nullptr;
    FnDeviceLastAvailableLiveData    deviceLastAvailableLiveData = nullptr;
    FnDevicePacketErrorRate          devicePacketErrorRate  = nullptr;

    FnScanPorts                      scanPorts              = nullptr;
    using FnEnumerateNetworkDevices  = void(*)(void*);
    FnEnumerateNetworkDevices        enumerateNetworkDevices = nullptr;
    FnArrayAt                        arrayAt                = nullptr;
    FnArrayDestruct                  arrayDestruct          = nullptr;
    FnPortInfoArrayConstruct         portInfoArrayConstruct = nullptr;
    FnDeviceIdArrayConstruct         deviceIdArrayConstruct = nullptr;

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
    FnDataPacketContainsTemperature       dataPacketContainsTemperature = nullptr;
    FnDataPacketTemperature               dataPacketTemperature    = nullptr;

    struct ContainsProbe {
        const char* name;
        FnDataPacketContainsAcc fn = nullptr;
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

static QString locateDllDir()
{
    const QString exeDir = QCoreApplication::applicationDirPath();
    for (const QString& sub : {QString(""), QString("/dll"), QString("/../dll")}) {
        const QString candidate = exeDir + sub;
        if (QFile::exists(candidate + "/xsensdeviceapi64.dll"))
            return QDir::cleanPath(candidate);
    }
    return exeDir;
}

static bool loadApi(Api& api, QString& errDetail)
{
    const QString dllDir = locateDllDir();
    const QString iompPath = dllDir + "/libiomp5md.dll";
    const QString xstPath  = dllDir + "/xstypes64.dll";
    const QString xdaPath  = dllDir + "/xsensdeviceapi64.dll";

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

    resolveProc (api.xda, "XsControl_loadFilterProfiles",           api.controlLoadFilterProfiles);
    resolveProc (api.xda, "XsControl_setOptions",                   api.controlSetOptions);
    resolveProc (api.xda, "xdaEnableNetworkScanning",               api.enableNetworkScanning);

    ok &= resolveProc(api.xda, "XsDevice_gotoConfig",               api.deviceGotoConfig);
    ok &= resolveProc(api.xda, "XsDevice_gotoMeasurement",          api.deviceGotoMeasurement);
    ok &= resolveProc(api.xda, "XsDevice_locationId",               api.deviceLocationId);
    resolveProc (api.xda, "XsDevice_productCode",                   api.deviceProductCode);
    resolveProc (api.xda, "XsDevice_hardwareVersion",               api.deviceHardwareVersion);
    resolveProc (api.xst, "XsString_construct",                     api.stringConstruct);
    resolveProc (api.xst, "XsString_destruct",                      api.stringDestruct);
    resolveProc (api.xst, "XsString_copyToWCharArray",              api.stringCopyToWChar);
    resolveProc (api.xda, "XsDevice_updateRate",                    api.deviceUpdateRate);
    ok &= resolveProc(api.xda, "XsDevice_getDataPacketCount",       api.deviceGetDataPacketCount);
    ok &= resolveProc(api.xda, "XsDevice_takeFirstDataPacketInQueue",
                      api.deviceTakeFirstDataPacketInQueue);
    resolveProc (api.xda, "XsDevice_lastAvailableLiveData",         api.deviceLastAvailableLiveData);
    resolveProc (api.xda, "XsDevice_packetErrorRate",               api.devicePacketErrorRate);

    ok &= resolveProc(api.xda, "XsScanner_scanPorts",               api.scanPorts);
    resolveProc (api.xda, "XsScanner_enumerateNetworkDevices",      api.enumerateNetworkDevices);

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

    resolveProc (api.xst, "XsDataPacket_containsVelocityIncrement", api.dataPacketContainsVelInc);
    resolveProc (api.xst, "XsDataPacket_velocityIncrement",         api.dataPacketVelInc);
    resolveProc (api.xst, "XsDataPacket_containsFreeAcceleration",  api.dataPacketContainsFreeAcc);
    resolveProc (api.xst, "XsDataPacket_freeAcceleration",          api.dataPacketFreeAcc);
    resolveProc (api.xst, "XsDataPacket_containsSampleTimeFine",    api.dataPacketContainsSTF);
    resolveProc (api.xst, "XsDataPacket_sampleTimeFine",            api.dataPacketSTF);
    resolveProc (api.xst, "XsDataPacket_containsTemperature",       api.dataPacketContainsTemperature);
    resolveProc (api.xst, "XsDataPacket_temperature",               api.dataPacketTemperature);

    resolveProc (api.xst, "XsDataPacket_containsOrientationIncrement",
                 api.dataPacketContainsOrientationIncrement);
    resolveProc (api.xst, "XsDataPacket_orientationIncrement",
                 api.dataPacketOrientationIncrement);

    for (auto& p : api.probes) {
        std::string sym = std::string("XsDataPacket_") + p.name;

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

}

struct MocapReceiver::Impl {
    bool             test;
    std::atomic<bool> stop{false};
    mutable QMutex   lock;
    SuitPose         frame;

    std::atomic<int> status{(int)ConnStatus::NotInitialized};
    QString          statusDetail;
    std::atomic<int> activeTrackers{0};

    double           lastDump   = 0.0;
    double           lastAhrsDump = 0.0;
    double           lastPacket = 0.0;

    HMODULE          manusModule    = nullptr;
    bool             manusDllLoaded = false;
    bool             manusCoreReady = false;
    int              manusGloveCount = 0;

    std::array<FusionAhrs, kXsensSegmentCount>           fusion{};
    std::array<bool,       kXsensSegmentCount>           fusionReady{};
    std::array<FusionAhrsSettings, kXsensSegmentCount>   ahrsCfg{};
    double           freqHz       = 240.0;

    std::atomic<double> magDeclinationDeg{fox::body::kMagnet.declinationDeg};
    std::atomic<double> magInclinationDeg{fox::body::kMagnet.inclinationDeg};

    std::array<quint32, kXsensSegmentCount> lastStf{};
    std::array<bool,    kXsensSegmentCount> haveLastStf{};

    std::array<fox::body::ImuChipType, kXsensSegmentCount> detectedChip{};

    std::array<Quat, kXsensSegmentCount> s2s{};
    std::array<Quat, kXsensSegmentCount> s2sInv{};
    bool                                 s2sActive = false;

    std::array<double, kXsensSegmentCount> magMagn{};
    bool                                   magNormActive = false;

    std::array<double, kXsensSegmentCount> accMagn{};
    bool                                   accNormActive = false;

    std::array<QVector3D, kXsensSegmentCount> gyrBias{};
    bool                                      gyrBiasActive = false;

    std::array<QVector3D, kXsensSegmentCount> dbgAccBody{};
    std::array<QVector3D, kXsensSegmentCount> dbgGyrBody{};
    std::array<QVector3D, kXsensSegmentCount> dbgMagBody{};
    std::array<QVector3D, kXsensSegmentCount> dbgGyrFused{};

    std::array<QVector3D, kXsensSegmentCount> dbgVelInc{};
    std::array<QVector3D, kXsensSegmentCount> dbgDqXyz{};
    std::array<QVector3D, kXsensSegmentCount> dbgAccPre{};
    std::array<QVector3D, kXsensSegmentCount> dbgGyrPre{};
    std::array<QVector3D, kXsensSegmentCount> dbgMagPre{};
    std::array<QVector3D, kXsensSegmentCount> dbgAccNorm{};
    std::array<QVector3D, kXsensSegmentCount> dbgGyrUnbias{};
    std::array<QVector3D, kXsensSegmentCount> dbgMagSoft{};
    std::array<Quat,      kXsensSegmentCount> dbgFusedQuat{};

    std::array<float,     kXsensSegmentCount> dbgDynAccRej{};
    std::array<float,     kXsensSegmentCount> dbgDynMagRej{};
    std::array<float,     kXsensSegmentCount> dbgAccErr{};
    std::array<quint8,    kXsensSegmentCount> dbgChainFlags{};

    std::array<std::array<double, 9>, kXsensSegmentCount> magSoftMat{};
    std::array<QVector3D, kXsensSegmentCount>             magSoftOff{};
    bool                                                  magSoftActive = false;

    std::array<float, kXsensSegmentCount> segGain{};
    bool                                  segGainActive = false;

    std::array<QVector3D, kXsensSegmentCount> segLinAccBody{};
    std::array<QVector3D, kXsensSegmentCount> segLinVelWorld{};

    std::array<float, kXsensSegmentCount> segAccErrEma{};
    std::array<float, kXsensSegmentCount> segMagErrEma{};
    std::array<float, kXsensSegmentCount> segAccErrPeak{};
    double                                 segAccDumpAccumSec = 0.0;

    std::atomic<uint32_t> calGen{0};

    std::atomic<int> transport{0};
    std::atomic<double> expectedRateHz{240.0};

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
    if (hz > 1.0 && hz <= 1000.0) m_impl->expectedRateHz.store(hz);
}

double MocapReceiver::expectedRate() const
{
    return m_impl->expectedRateHz.load();
}

void MocapReceiver::restart()
{

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

template <typename Fn>
static bool sehCall(Fn fn)
{
    __try { fn(); return true; }
    __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}

namespace {
struct ManusErgoSnapshot {
    std::atomic<std::uint64_t> lastTimeMs { 0 };
    std::atomic<std::uint32_t> lastCount  { 0 };
    std::atomic<std::uint32_t> lastLeft   { 0 };
    std::atomic<std::uint32_t> lastRight  { 0 };

    std::atomic<bool> haveLeft  { false };
    std::atomic<bool> haveRight { false };
    QMutex              lock;
    std::array<Quat,      kFingerSegmentsHand> leftQ{};
    std::array<Quat,      kFingerSegmentsHand> rightQ{};
    std::array<QVector3D, kFingerSegmentsHand> leftP{};
    std::array<QVector3D, kFingerSegmentsHand> rightP{};

    std::atomic<bool>          rawDump    { false };
    std::atomic<std::uint64_t> rawTicks   { 0 };

    std::array<float, 20> emaLeft  {};
    std::array<float, 20> emaRight {};
    bool emaLeftInit  = false;
    bool emaRightInit = false;
};
static ManusErgoSnapshot g_ergo;

struct FingerBaselineState {
    QMutex lock;
    std::array<float, 20> left  {};
    std::array<float, 20> right {};
    std::atomic<bool> valid { false };
};
static FingerBaselineState g_fingerBaseline;

static const double kFingerBoneLen[5][4] = {
    { 0.0311, 0.0311, 0.0000,  0.0354318 },
    { 0.0414, 0.0226, 0.0203,  0.0572504 },
    { 0.0453, 0.0268, 0.0213,  0.0555000 },
    { 0.0430, 0.0255, 0.0215,  0.0538932 },
    { 0.0359, 0.0195, 0.0207,  0.0511957 },
};

static const QVector3D kFingerBaseOffset[5] = {
    QVector3D(0.0628f,  0.0270f,  0.000f),
    QVector3D(0.0916f,  0.0121f,  0.000f),
    QVector3D(0.0917f,  0.0000f,  0.000f),
    QVector3D(0.0882f, -0.0122f,  0.000f),
    QVector3D(0.0810f, -0.0218f,  0.000f),
};

static const double kSpreadSign[5] = { +1.0, +1.0, +1.0, +1.0, +1.0 };

const FingerJointLimit kFingerLimits[5][3] = {
    {
        { -M_PI / 18.0,  M_PI / 3.0,   -M_PI / 9.0,    M_PI * 5.0/18.0 },
        { -M_PI / 36.0,  M_PI / 36.0,   0.0,           M_PI * 4.0/9.0  },
        {  0.0,          0.0,           0.0,           M_PI * 0.50     }
    },
    {
        { -M_PI / 9.0,   M_PI / 9.0,    0.0,           M_PI * 0.50 },
        {  0.0,          0.0,           0.0,           M_PI * 11.0/18.0 },
        {  0.0,          0.0,           0.0,           M_PI * 4.0/9.0   }
    },
    {
        { -M_PI / 9.0,   M_PI / 9.0,    0.0,           M_PI * 0.50 },
        {  0.0,          0.0,           0.0,           M_PI * 11.0/18.0 },
        {  0.0,          0.0,           0.0,           M_PI * 4.0/9.0   }
    },
    {
        { -M_PI / 9.0,   M_PI / 9.0,    0.0,           M_PI * 0.50 },
        {  0.0,          0.0,           0.0,           M_PI * 11.0/18.0 },
        {  0.0,          0.0,           0.0,           M_PI * 4.0/9.0   }
    },
    {
        { -M_PI / 9.0,   M_PI / 9.0,    0.0,           M_PI * 0.50 },
        {  0.0,          0.0,           0.0,           M_PI * 11.0/18.0 },
        {  0.0,          0.0,           0.0,           M_PI * 4.0/9.0   }
    }
};

static Quat axisAngleQuat(const QVector3D& axis, double rad)
{
    const double h = rad * 0.5;
    const double s = std::sin(h);
    return Quat(std::cos(h),
                axis.x() * s, axis.y() * s, axis.z() * s).normalized();
}

struct FingerDiagHand {
    bool   valid           = false;
    bool   baselineApplied = false;
    float  raw[20]         = {};
    float  effective[20]   = {};
    double spreadEffDeg[5] = {};
    double flexDeg[5][3]   = {};
    double spreadClDeg[5]  = {};
    double flexClDeg[5][3] = {};
    bool   spreadClamped[5]  = {};
    bool   flexClamped[5][3] = {};
};
struct FingerDiag {
    QMutex lock;
    FingerDiagHand left, right;
};
static FingerDiag g_fingerDiag;

class FoxKFAGloveFilter {
public:
    void reset() {
        for (auto& q : m_state) q = Quat(1, 0, 0, 0);
        for (auto& p : m_var)   p = (5.0 * 0.017453292519943295) * (5.0 * 0.017453292519943295);
        m_initialised = false;
    }
    void setFingerTrackingData(bool isLeft,
                               const std::array<Quat, kFingerSegmentsHand>& q_meas) {
        auto& dst = isLeft ? m_pendingLeft : m_pendingRight;
        dst = q_meas;
        (isLeft ? m_havePendingLeft : m_havePendingRight) = true;
    }
    void process(double dt) {
        if (!m_initialised) {
            if (m_havePendingRight) m_state = m_pendingRight;
            if (m_havePendingLeft)  m_stateLeft = m_pendingLeft;
            m_initialised = true;
        }
        const double tau = 0.080;
        const double alpha = 1.0 - std::exp(-dt / std::max(1e-3, tau));
        const double R = (1.0 * 0.017453292519943295) * (1.0 * 0.017453292519943295);
        for (int i = 0; i < kFingerSegmentsHand; ++i) {
            if (m_havePendingRight) {
                const double K = m_var[i] / (m_var[i] + R);
                m_state[i] = slerp_quat(m_state[i], m_pendingRight[i], float(std::clamp(K, alpha, 1.0)));
                m_state[i] = m_state[i].normalized();
                m_var[i]   = (1.0 - K) * m_var[i] + 1e-7;
            }
            if (m_havePendingLeft) {
                const double K = m_varLeft[i] / (m_varLeft[i] + R);
                m_stateLeft[i] = slerp_quat(m_stateLeft[i], m_pendingLeft[i], float(std::clamp(K, alpha, 1.0)));
                m_stateLeft[i] = m_stateLeft[i].normalized();
                m_varLeft[i]   = (1.0 - K) * m_varLeft[i] + 1e-7;
            }
        }
        m_havePendingRight = false;
        m_havePendingLeft  = false;
    }
    void outputRight(std::array<Quat, kFingerSegmentsHand>& out) const { out = m_state; }
    void outputLeft (std::array<Quat, kFingerSegmentsHand>& out) const { out = m_stateLeft; }
private:
    std::array<Quat,   kFingerSegmentsHand> m_state{};
    std::array<Quat,   kFingerSegmentsHand> m_stateLeft{};
    std::array<double, kFingerSegmentsHand> m_var{};
    std::array<double, kFingerSegmentsHand> m_varLeft{};
    std::array<Quat,   kFingerSegmentsHand> m_pendingRight{};
    std::array<Quat,   kFingerSegmentsHand> m_pendingLeft{};
    bool m_havePendingRight = false;
    bool m_havePendingLeft  = false;
    bool m_initialised      = false;
};

static FoxKFAGloveFilter g_gloveFilter;

static void parseErgoHand(const float* degs20, bool isLeft,
                          std::array<Quat,      kFingerSegmentsHand>& outQ,
                          std::array<QVector3D, kFingerSegmentsHand>& outP)
{
    FingerDiagHand dg;
    dg.valid = true;
    const QVector3D flexAxis (0, 1, 0);
    const QVector3D spreadAx (0, 0, 1);
    const double sideSign = isLeft ? -1.0 : +1.0;

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

    constexpr Quat kThumbBaseRot(1.0, 0.0, 0.0, 0.0);

    for (int f = 0; f < 5; ++f) {
        const float* d = effectivePtr + f * 4;
        const double spread = d[0] * M_PI / 180.0;
        const double a1     = d[1] * M_PI / 180.0;
        const double a2     = d[2] * M_PI / 180.0;
        const double a3     = d[3] * M_PI / 180.0;

        const auto& Lm = kFingerLimits[f];
        const double spreadEff = spread * sideSign * kSpreadSign[f];
        const double spreadC = std::clamp(spreadEff, Lm[0].spreadMin, Lm[0].spreadMax);
        const double a1c     = std::clamp(a1, Lm[0].flexMin, Lm[0].flexMax);
        const double a2c     = std::clamp(a2, Lm[1].flexMin, Lm[1].flexMax);
        double a3c           = std::clamp(a3, Lm[2].flexMin, Lm[2].flexMax);
        (void)a3;

        if (f > 0) {
            const double a3Linked = 0.7 * a2c;
            a3c = std::clamp(a3Linked, Lm[2].flexMin, Lm[2].flexMax);
        }

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

        Quat worldQ = (f == 0) ? kThumbBaseRot : Quat(1, 0, 0, 0);
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
        const auto& qbs = isLeft ? fox::body::kFingerQBSLeft
                                  : fox::body::kFingerQBSRight;
        for (int i = 0; i < int(outQ.size()) && i < int(qbs.size()); ++i) {
            outQ[i] = quat_mult(outQ[i], qbs[i]).normalized();
        }
    }
    {
        QMutexLocker lk(&g_fingerDiag.lock);
        (isLeft ? g_fingerDiag.left : g_fingerDiag.right) = dg;
    }
}

static inline QVector3D mirrorManusL(const QVector3D& p)
{
    return QVector3D(p.x(), -p.y(), p.z());
}
}

struct ManusErgoData {
    std::uint32_t id = 0;
    std::uint32_t isUserID = 0;
    float data[40]{};
};

static_assert(sizeof(ManusErgoData) == 0xA8,
              "ManusErgoData must match the Manus ergonomics ABI (4+4+40*4 bytes)");
struct ManusErgoStream {
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
    const float* srcL = nullptr;
    const float* srcR = nullptr;

    for (std::uint32_t i = 0; i < s->dataCount && i < 32; ++i) {
        const auto& e = s->data[i];
        bool anyL = false, anyR = false;
        for (int j = 0; j < 20; ++j) {
            if (std::abs(e.data[j])      > 0.01f) anyL = true;
            if (std::abs(e.data[20 + j]) > 0.01f) anyR = true;
        }
        const bool testMode = g_ergo.rawDump.load();
        auto alphaForJoint = [testMode](int idx, float delta, const char* hand) -> float {
            const auto& cfg = fox::body::kFingerSmooth;
            const bool isThumb = (idx < 4);
            const float baseAlpha    = isThumb ? cfg.emaAlphaThumb    : cfg.emaAlphaFinger;
            const float outlierAlpha = isThumb ? cfg.outlierAlphaThumb : cfg.outlierAlphaFinger;
            const float outlierThresh = isThumb
                ? cfg.outlierThreshThumbDeg
                : cfg.outlierThreshFingerDeg;
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
        if (haveL) g_gloveFilter.setFingerTrackingData(true,  lQ);
        if (haveR) g_gloveFilter.setFingerTrackingData(false, rQ);
        const double kFilterDt = 1.0 / 120.0;
        g_gloveFilter.process(kFilterDt);
        if (haveL) g_gloveFilter.outputLeft(lQ);
        if (haveR) g_gloveFilter.outputRight(rQ);

        QMutexLocker lk(&g_ergo.lock);
        if (haveL) { g_ergo.leftQ  = lQ;  g_ergo.leftP  = lP;  g_ergo.haveLeft.store(true); }
        if (haveR) { g_ergo.rightQ = rQ;  g_ergo.rightP = rP;  g_ergo.haveRight.store(true); }
    }

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
struct Host {
    char hostName[256]{};
    char ipAddress[40]{};
    VersionInfo version{};
};
struct CoordinateSystemVUH {
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

}

bool MocapReceiver::connectGloves()
{
    auto& I = *m_impl;

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

        if (isConn) {
            bool flag = false;
            sehCall([&]() { isConn(&flag); });
            coreUp = flag;
        }
        testLog(std::string("[manus] re-entry on existing session, connected=")
                + (coreUp ? "true" : "false"), I.test);
    } else {

        auto noop = [](const void*) {};
        if (regSkel) {
            auto skelCb = [](const void* msg) {
                if (!msg) return;
                static std::atomic<int> s_skelLogTick{0};
                if (s_skelLogTick.fetch_add(1) % 240 == 0) {
                    std::cerr << "[manus-skeleton] frame received (using ergonomics stream as primary)\n";
                }
            };
            int r = 0; sehCall([&]() { r = regSkel(skelCb); });
            testLog("[manus] RegSkeletonStream rc=" + std::to_string(r), I.test);
        }

        { int r = 0; sehCall([&]() { r = regErgo(&foxManusErgonomicsCb); });
          testLog("[manus] RegErgonomicsStream rc=" + std::to_string(r), I.test); }
        if (regSys)  {
            auto sysCb = [](const void* msg) {
                if (!msg) return;
                std::cerr << "[manus-system] event received (low-level diagnostic)\n";
            };
            int r = 0; sehCall([&]() { r = regSys(sysCb); });
            testLog("[manus] RegSystemStream rc=" + std::to_string(r), I.test);
        }

        int sRc = -1;
        sehCall([&]() { sRc = setSess(kSessionTypeUnreal); });
        testLog("[manus] CoreSdk_SetSessionType(2) rc=" + std::to_string(sRc), I.test);

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

        constexpr int kMaxRetries = 5;
        int backoffSec = 1;
        for (int attempt = 0; attempt < kMaxRetries && !coreUp; ++attempt) {
            int lRc = -1;
            sehCall([&]() { lRc = lookFor(1, true); });
            testLog(std::string("[manus] CoreSdk_LookForHosts attempt ")
                    + std::to_string(attempt + 1) + "/" + std::to_string(kMaxRetries)
                    + " rc=" + std::to_string(lRc), I.test);

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

            if (!coreUp && attempt + 1 < kMaxRetries) {
                testLog(std::string("[manus] retry in ")
                        + std::to_string(backoffSec) + "s", I.test);
                Sleep(static_cast<DWORD>(backoffSec * 1000));
                backoffSec *= 2;
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

    int gloveCount = 0;
    if (getNumUsr && getUserIds && getGloveId) {
        std::uint32_t userN = 0;
        sehCall([&]() { getNumUsr(&userN); });
        testLog("[manus] users seen = " + std::to_string(userN), I.test);
        if (userN > 0) {
            std::vector<std::uint32_t> ids(userN);
            sehCall([&]() { getUserIds(ids.data(), userN); });
            for (std::uint32_t u = 0; u < userN; ++u) {
                for (int side : { 0 , 1  }) {
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

    if (gloveCount == 0 && getNumGl) {
        std::uint32_t n = 0;
        sehCall([&]() { getNumGl(&n); });
        gloveCount = int(n);
    }

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

bool MocapReceiver::glovesReady()     const { return m_impl->manusGloveCount >= 2; }
bool MocapReceiver::glovesCoreReady() const { return m_impl->manusCoreReady; }
bool MocapReceiver::glovesDllLoaded() const { return m_impl->manusDllLoaded; }

void MocapReceiver::resetFusion()
{
    resetS2sAlignment();
    m_impl->calGen.fetch_add(1, std::memory_order_relaxed);
    testLog("[fusion] reset — all per-sensor §43 EKFs will re-init "
            "(incl. s2s reset per §24.5)", m_impl->test);
}

void MocapReceiver::setS2sAlignment(const std::array<Quat, kXsensSegmentCount>& s2s)
{
    QMutexLocker lk(&m_impl->lock);
    m_impl->s2s    = s2s;
    for (int i = 0; i < kXsensSegmentCount; ++i)
        m_impl->s2sInv[i] = s2s[i].inv();
    m_impl->s2sActive = true;

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
    m_impl->calGen.fetch_add(1, std::memory_order_relaxed);
    testLog("[s2s] per-sensor mag_magn normalisation installed", m_impl->test);
}

void MocapReceiver::setAccNormalisation(const std::array<double, kXsensSegmentCount>& am)
{
    QMutexLocker lk(&m_impl->lock);
    m_impl->accMagn = am;
    m_impl->accNormActive = true;
    m_impl->calGen.fetch_add(1, std::memory_order_relaxed);
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
    m_impl->calGen.fetch_add(1, std::memory_order_relaxed);
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

void MocapReceiver::setMagneticDeclinationDeg(double deg)
{
    m_impl->magDeclinationDeg.store(deg, std::memory_order_relaxed);
    m_impl->calGen.fetch_add(1, std::memory_order_relaxed);
    testLog("[mag] declination updated", m_impl->test);
}

void MocapReceiver::setMagneticInclinationDeg(double deg)
{
    m_impl->magInclinationDeg.store(deg, std::memory_order_relaxed);
    m_impl->calGen.fetch_add(1, std::memory_order_relaxed);
    testLog("[mag] inclination (dip) updated", m_impl->test);
}

double MocapReceiver::magneticDeclinationDeg() const
{
    return m_impl->magDeclinationDeg.load(std::memory_order_relaxed);
}

double MocapReceiver::magneticInclinationDeg() const
{
    return m_impl->magInclinationDeg.load(std::memory_order_relaxed);
}

bool MocapReceiver::saveCalibration(const QString& path) const
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QTextStream out(&file);
    out.setRealNumberPrecision(12);
    QMutexLocker lk(&m_impl->lock);
    out << "{\n";
    out << "  \"version\": 1,\n";
    out << "  \"magDeclinationDeg\": "
        << m_impl->magDeclinationDeg.load(std::memory_order_relaxed) << ",\n";
    out << "  \"magInclinationDeg\": "
        << m_impl->magInclinationDeg.load(std::memory_order_relaxed) << ",\n";
    out << "  \"accMagn\":  [";
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        out << m_impl->accMagn[i];
        if (i + 1 < kXsensSegmentCount) out << ",";
    }
    out << "],\n";
    out << "  \"magMagn\":  [";
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        out << m_impl->magMagn[i];
        if (i + 1 < kXsensSegmentCount) out << ",";
    }
    out << "],\n";
    out << "  \"gyrBias\":  [";
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        const auto& g = m_impl->gyrBias[i];
        out << "[" << g.x() << "," << g.y() << "," << g.z() << "]";
        if (i + 1 < kXsensSegmentCount) out << ",";
    }
    out << "],\n";
    out << "  \"s2s\": [";
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        const auto& q = m_impl->s2s[i];
        out << "[" << q.w << "," << q.x << "," << q.y << "," << q.z << "]";
        if (i + 1 < kXsensSegmentCount) out << ",";
    }
    out << "],\n";
    out << "  \"segGainActive\": "
        << (m_impl->segGainActive ? "true" : "false") << ",\n";
    out << "  \"segGain\": [";
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        out << m_impl->segGain[i];
        if (i + 1 < kXsensSegmentCount) out << ",";
    }
    out << "]\n";
    out << "}\n";
    return file.error() == QFile::NoError;
}

bool MocapReceiver::loadCalibration(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
    const QByteArray buf = file.readAll();
    QJsonParseError err{};
    const QJsonDocument doc = QJsonDocument::fromJson(buf, &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) return false;
    const QJsonObject root = doc.object();

    if (root.contains("magDeclinationDeg"))
        setMagneticDeclinationDeg(root["magDeclinationDeg"].toDouble());
    if (root.contains("magInclinationDeg"))
        setMagneticInclinationDeg(root["magInclinationDeg"].toDouble());

    auto readArr = [&](const QString& key,
                       std::array<double, kXsensSegmentCount>& out) {
        const QJsonArray a = root[key].toArray();
        for (int i = 0; i < kXsensSegmentCount && i < a.size(); ++i)
            out[i] = a[i].toDouble(1.0);
    };

    std::array<double,    kXsensSegmentCount> accMagn{};
    std::array<double,    kXsensSegmentCount> magMagn{};
    std::array<QVector3D, kXsensSegmentCount> gyrBias{};
    std::array<Quat,      kXsensSegmentCount> s2s{};
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        accMagn[i] = 1.0;
        magMagn[i] = 1.0;
        s2s[i]     = Quat(1, 0, 0, 0);
    }

    readArr("accMagn", accMagn);
    readArr("magMagn", magMagn);

    if (root.contains("gyrBias")) {
        const QJsonArray a = root["gyrBias"].toArray();
        for (int i = 0; i < kXsensSegmentCount && i < a.size(); ++i) {
            const QJsonArray v = a[i].toArray();
            if (v.size() >= 3) {
                gyrBias[i] = QVector3D(float(v[0].toDouble()),
                                       float(v[1].toDouble()),
                                       float(v[2].toDouble()));
            }
        }
    }
    if (root.contains("s2s")) {
        const QJsonArray a = root["s2s"].toArray();
        for (int i = 0; i < kXsensSegmentCount && i < a.size(); ++i) {
            const QJsonArray q = a[i].toArray();
            if (q.size() >= 4) {
                s2s[i] = Quat(q[0].toDouble(1.0),
                              q[1].toDouble(0.0),
                              q[2].toDouble(0.0),
                              q[3].toDouble(0.0));
            }
        }
    }

    setAccNormalisation(accMagn);
    setMagNormalisation(magMagn);
    setGyroBias(gyrBias);
    setS2sAlignment(s2s);
    if (root.contains("segGain") && root.value("segGainActive").toBool(true)) {
        const QJsonArray a = root["segGain"].toArray();
        if (a.size() == kXsensSegmentCount) {
            std::array<float, kXsensSegmentCount> sg{};
            for (int i = 0; i < kXsensSegmentCount; ++i)
                sg[i] = float(a[i].toDouble(1.0));
            setSegmentGain(sg);
        }
    }
    return true;
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

// §XXX сквозной конвейер на датчик->поза (поток приёма): сырые acc/gyr/mag -> нормировка/смещения/s2s ->
//   FoxKF (§IX FusionAhrs) -> ориентации сегментов + перчатка -> публикация кадра staging (formules.txt)
void MocapReceiver::run()
{
    using namespace xda;

    auto& I = *m_impl;
    I.setStatus(ConnStatus::Scanning, "loading XDA driver…", this);

    if (I.test) {
        const double expectedHz = I.expectedRateHz.load();
        const int    transSel   = I.transport.load();
        const bool   gloves     = pose_solver::g_glovesFlag().load(
                                      std::memory_order_relaxed);
        const char*  suitName   = (expectedHz >= 200.0) ? "Link"
                                : (expectedHz >= 50.0)  ? "Awinda"
                                                        : "unknown";
        std::ostringstream sys;
        sys << std::fixed << std::setprecision(0);
        sys << "[sys] fox-mocap session: suit=" << suitName
            << "(" << expectedHz << "Hz)"
            << " transport=" << (transSel == 1 ? "WiFi" : "COM")
            << " gloves=" << (gloves ? "on" : "off")
            << " test=on";

        const QString exeDir = QCoreApplication::applicationDirPath();
        const QString modelPath = QDir(exeDir).filePath(
            "fox_sensor_placement_classifier.onnx");
        sys << " foxspc=" << (QFile::exists(modelPath) ? "available" : "missing");
        sys << "\n";
        std::cout << sys.str();
        std::cout.flush();
    }

    Api api;
    QString detail;
    if (!loadApi(api, detail)) {
        I.setStatus(ConnStatus::NoDriver, detail, this);
        return;
    }

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
        portCount = pollNetworkDevices(17);

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

        if (portCount == 0 && !I.stop.load()) {
            I.setStatus(ConnStatus::Scanning,
                        "Поиск Xsens в сети (WiFi/Ethernet)…", this);
            portCount = pollNetworkDevices(11);
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

            if (api.deviceProductCode && api.stringConstruct
                && api.stringDestruct && api.stringCopyToWChar) {
                std::array<unsigned char, 256> xsStr{};
                api.stringConstruct(xsStr.data());
                api.deviceProductCode(dev, xsStr.data());
                std::array<wchar_t, 64> buf{};
                api.stringCopyToWChar(xsStr.data(), buf.data(), buf.size());
                api.stringDestruct(xsStr.data());
                const std::wstring product(buf.data());
                fox::body::ImuChipType chip = fox::body::ImuChipType::W2;
                if (product.find(L"X3") != std::wstring::npos) {
                    chip = fox::body::ImuChipType::X3;
                } else if (product.find(L"X2") != std::wstring::npos) {
                    chip = fox::body::ImuChipType::X2;
                }
                if (I.test) {
                    char asc[64]{};
                    for (size_t k = 0; k < buf.size() && k < 63; ++k)
                        asc[k] = char(buf[k]);
                    testLog(std::string("[xda]     productCode=") + asc
                            + " → ImuChip="
                            + (chip == fox::body::ImuChipType::X3 ? "X3"
                                : chip == fox::body::ImuChipType::X2 ? "X2" : "W2"),
                            I.test);
                }
                if (segIdx >= 0 && segIdx < fox::body::kSegmentCount) {
                    I.detectedChip[segIdx] = chip;
                }
            }

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
    QThread::msleep(1000);

    const double expectedHz = I.expectedRateHz.load();
    I.freqHz = expectedHz;

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

    struct SegCal {
        bool   s2sActive = false, magNormActive = false, accNormActive = false,
               gyrBiasActive = false, magSoftActive = false, segGainActive = false;
        double accMagn = 1.0, magMagn = 1.0;
        QVector3D gyrBias, magSoftOff;
        std::array<double, 9> magSoftMat{};
        Quat   s2sInv;
        float  segGain = 0.0f;
    };

    SuitPose staging;
    int framesThisSec = 0;
    double fpsT0 = monotonicSec();
    I.lastPacket = monotonicSec();

    std::array<int, kXsensSegmentCount> dumpCount{};

    uint32_t lastCalGen = I.calGen.load(std::memory_order_relaxed);

    while (!I.stop.load()) {
        bool gotAny = false;

        for (std::size_t t = 0; t < trackerHandles.size(); ++t) {
            void* dev     = trackerHandles[t];
            const int seg = trackerSegments[t];

            int pktCount = 0;
            if (!sehCall([&]{ pktCount = api.deviceGetDataPacketCount(dev); })
                || pktCount <= 0) continue;

            XsDataPacketBlob* pkt = nullptr;
            if (!sehCall([&]{
                    api.dataPacketConstruct(reinterpret_cast<XsDataPacketBlob*>(packetStorage));
                    pkt = api.deviceTakeFirstDataPacketInQueue(
                        dev, reinterpret_cast<XsDataPacketBlob*>(packetStorage));
                })) continue;

            int targetSeg = seg;
            if (pkt && api.dataPacketContainsStoredLocationId &&
                api.dataPacketStoredLocationId &&
                api.dataPacketContainsStoredLocationId(pkt)) {
                const int locId = api.dataPacketStoredLocationId(pkt);
                const int ss = segmentFromLocationId(locId);
                if (ss >= 0) targetSeg = ss;
            }

            SegCal cal;
            {
                QMutexLocker lk(&I.lock);
                const uint32_t gen = I.calGen.load(std::memory_order_relaxed);
                if (gen != lastCalGen) {

                    I.fusionReady.fill(false);
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

            double sensorTempC = std::numeric_limits<double>::quiet_NaN();
            if (api.dataPacketContainsTemperature && api.dataPacketTemperature &&
                api.dataPacketContainsTemperature(pkt)) {
                sensorTempC = api.dataPacketTemperature(pkt);
            }
            (void)sensorTempC;

            double dt = 1.0 / I.freqHz;
            if (haveStf && targetSeg >= 0 && targetSeg < kXsensSegmentCount) {
                if (I.haveLastStf[targetSeg]) {
                    const quint32 last = I.lastStf[targetSeg];
                    const quint32 diff = stf - last;
                    if (diff > 0 && diff < 100000) {
                        const double measured = double(diff) * 1.0e-4;
                        const double nominal = 1.0 / I.freqHz;
                        if (measured > 0.25 * nominal && measured < 4.0 * nominal) {
                            dt = measured;
                        }
                    }
                }
                I.lastStf[targetSeg]     = stf;
                I.haveLastStf[targetSeg] = true;
            }
            constexpr double kRadToDeg = 57.29577951308232;
            constexpr double kMs2ToG   = 1.0 / fox::body::constants::kGravityMs2;
            QVector3D accForFilter, gyrForFilter;
            bool fuseReady = false;
            if (haveVelInc && haveDq) {

                const QVector3D accSI(float(velInc.x() * I.freqHz),
                                      float(velInc.y() * I.freqHz),
                                      float(velInc.z() * I.freqHz));

                const double vNorm = std::sqrt(dq.x * dq.x +
                                               dq.y * dq.y +
                                               dq.z * dq.z);
                double phiX = 0.0, phiY = 0.0, phiZ = 0.0;
                if (vNorm > 1e-12) {
                    const double absW = std::min(1.0, std::abs(dq.w));
                    const double halfAngle = std::atan2(vNorm, absW);
                    const double sgn = (dq.w >= 0.0) ? 1.0 : -1.0;
                    const double scale = (2.0 * halfAngle) / vNorm;
                    phiX = sgn * dq.x * scale;
                    phiY = sgn * dq.y * scale;
                    phiZ = sgn * dq.z * scale;
                }
                const QVector3D gyrSI(float(phiX * I.freqHz),
                                      float(phiY * I.freqHz),
                                      float(phiZ * I.freqHz));
                accForFilter = accSI * float(kMs2ToG);
                gyrForFilter = gyrSI * float(kRadToDeg);
                fuseReady = true;
            } else if (haveAcc && haveGyr) {
                accForFilter = acc * float(kMs2ToG);
                gyrForFilter = gyr * float(kRadToDeg);
                fuseReady = true;
            }

            if (fuseReady && targetSeg >= 0 && targetSeg < kXsensSegmentCount) {
                QMutexLocker raw(&I.lock);
                staging.accSensor[targetSeg] = accForFilter;
                staging.gyrSensor[targetSeg] = gyrForFilter;
                if (haveMag) staging.magSensor[targetSeg] = mag;
            }

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

            if (cal.accNormActive) {
                const double a = cal.accMagn;
                if (a > 1e-6) accForFilter = accForFilter / float(a);
            }
            if (I.test && targetSeg >= 0 && targetSeg < kXsensSegmentCount)
                I.dbgAccNorm[targetSeg] = accForFilter;

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

            if (cal.s2sActive) {
                const Quat& inv = cal.s2sInv;
                accForFilter = vec_rotate(accForFilter, inv);
                gyrForFilter = vec_rotate(gyrForFilter, inv);
                if (haveMag) mag = vec_rotate(mag, inv);
            }

            if (I.test && targetSeg >= 0 && targetSeg < kXsensSegmentCount) {
                I.dbgAccBody[targetSeg] = accForFilter;
                I.dbgGyrBody[targetSeg] = gyrForFilter;
                I.dbgMagBody[targetSeg] = mag;
            }

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
                if (!I.fusionReady[targetSeg]) {
                    FusionAhrsInitialise(&ahrs);
                    FusionAhrsSettings& s = I.ahrsCfg[targetSeg];
                    s = fusionAhrsDefaultSettings;
                    s.convention   = FusionConventionNwu;
                    s.sampleRateHz = float(std::max(60.0, I.freqHz));

                    s.magDipModelDeg    = float(I.magInclinationDeg.load(std::memory_order_relaxed));
                    s.magDeclinationDeg = float(I.magDeclinationDeg.load(std::memory_order_relaxed));

                    const fox::body::CalibMagE& mE = fox::body::kCalibMagE;
                    float refNorm = 1.0f;
                    switch (targetSeg) {
                        case SEG_Head:                                  refNorm = float(1.0 + mE.e_norm_head);   break;
                        case SEG_RHand: case SEG_LHand:                 refNorm = float(1.0 + mE.e_norm_hands);  break;
                        case SEG_RFoot: case SEG_LFoot:                 refNorm = float(1.0 + mE.e_norm_feet);  break;
                        case SEG_Pelvis:                                refNorm = float(1.0 + mE.e_norm_pelvis); break;
                        default:                                        refNorm = 1.0f;                           break;
                    }
                    s.magNormReferenceLocal = refNorm;

                    auto chipForSeg = [&](int seg) {
                        const fox::body::ImuChipType det = I.detectedChip[seg];
                        const fox::body::ImuChipType deflt = fox::body::kImuChipPerSeg[seg];
                        return (det == fox::body::ImuChipType::W2 &&
                                deflt != fox::body::ImuChipType::W2)
                                ? deflt : det;
                    };

                    if (targetSeg >= 0 && targetSeg < fox::body::kSegmentCount) {
                        const auto& relax = fox::body::kMagGateRelax[targetSeg];
                        const float chipMul = fox::body::magNoiseScaleForChip(
                            chipForSeg(targetSeg));
                        s.magDipGateRelax  = relax.dipMul   * chipMul;
                        s.magAngGateRelax  = relax.angleMul * chipMul;
                        s.magNormGateRelax = relax.normMul;
                    }
                    FusionAhrsSetSettings(&ahrs, &s);

                    if (targetSeg >= 0 && targetSeg < fox::body::kSegmentCount) {
                        const fox::body::ImuChipNoise& nz =
                            fox::body::chipNoiseFor(chipForSeg(targetSeg));
                        FusionAhrsSetNoise(&ahrs, nz.sigmaAccMs2,
                                                   nz.sigmaGyrDegS,
                                                   nz.sigmaMagNorm);
                    }
                    I.fusionReady[targetSeg] = true;
                }

                const FusionVector g = {{ float(gyrForFilter.x()),
                                          float(gyrForFilter.y()),
                                          float(gyrForFilter.z()) }};
                const FusionVector a = {{ float(accForFilter.x()),
                                          float(accForFilter.y()),
                                          float(accForFilter.z()) }};
                if (dt > 1e-4) {
                    FusionAhrsSetSampleRate(&ahrs, float(1.0 / dt));
                }
                if (haveMag && mag.length() > 1e-6) {
                    const FusionVector m = {{ float(mag.x()),
                                              float(mag.y()),
                                              float(mag.z()) }};
                    FusionAhrsUpdate(&ahrs, g, a, m, float(dt));
                } else {
                    FusionAhrsUpdateNoMagnetometer(&ahrs, g, a, float(dt));
                }

                {
                    const FusionAhrsInternalStates st = FusionAhrsGetInternalStates(&ahrs);
                    if (I.test) {
                        I.dbgDynAccRej[targetSeg] = st.accelerationRecoveryTrigger * 90.0f;
                        I.dbgDynMagRej[targetSeg] = st.magneticError;
                        I.dbgAccErr[targetSeg]    = st.accelerationError;
                        I.dbgGyrFused[targetSeg]  = QVector3D(g.axis.x, g.axis.y, g.axis.z);
                    }
                    const float dtF = float(dt);
                    const double tauEma = std::max(0.1, fox::body::kFilter.tauAcc);
                    const float aEma = float(1.0 - std::exp(-double(dtF) / tauEma));
                    I.segAccErrEma[targetSeg] = (1.0f - aEma) * I.segAccErrEma[targetSeg]
                                              + aEma * st.accelerationError;
                    I.segMagErrEma[targetSeg] = (1.0f - aEma) * I.segMagErrEma[targetSeg]
                                              + aEma * st.magneticError;
                    if (st.accelerationError > I.segAccErrPeak[targetSeg])
                        I.segAccErrPeak[targetSeg] = st.accelerationError;
                }

                const FusionQuaternion fq = FusionAhrsGetQuaternion(&ahrs);
                if (std::isfinite(fq.element.w) && std::isfinite(fq.element.x) &&
                    std::isfinite(fq.element.y) && std::isfinite(fq.element.z)) {
                    fusedQuat = Quat(fq.element.w, fq.element.x,
                                     fq.element.y, fq.element.z).normalized();
                    haveFused = true;
                    if (I.test) I.dbgFusedQuat[targetSeg] = fusedQuat;
                } else {

                    I.fusionReady[targetSeg] = false;
                }

                if (haveFused) {
                    const FusionVector linG = FusionAhrsGetLinearAcceleration(&ahrs);
                    const QVector3D linBody(linG.axis.x * 9.812687f,
                                             linG.axis.y * 9.812687f,
                                             linG.axis.z * 9.812687f);
                    const QVector3D linWorld = vec_rotate(linBody, fusedQuat);
                    I.segLinAccBody[targetSeg] = linBody;
                    if (ahrs.zruActiveThisFrame) {
                        I.segLinVelWorld[targetSeg] = QVector3D(0, 0, 0);
                    } else {
                        const double decay = std::exp(-dt / 2.0);
                        I.segLinVelWorld[targetSeg] =
                            I.segLinVelWorld[targetSeg] * float(decay)
                            + linWorld * float(dt);
                    }
                }
            }

            if (targetSeg >= 0 && targetSeg < kXsensSegmentCount) {
                QMutexLocker lk(&I.lock);
                if      (haveFused) staging.quat[targetSeg] = fusedQuat;
                else if (haveQuat)  staging.quat[targetSeg] = qo;
                staging.segValid[targetSeg] = haveFused || haveQuat;
                staging.segLastT[targetSeg] = monotonicSec();
                staging.linAccBody[targetSeg]  = I.segLinAccBody[targetSeg];
                staging.linVelWorld[targetSeg] = I.segLinVelWorld[targetSeg];
                if (api.dataPacketContainsPacketCounter &&
                    api.dataPacketPacketCounter &&
                    api.dataPacketContainsPacketCounter(pkt))
                    staging.sampleCounter = api.dataPacketPacketCounter(pkt);
                staging.recvTime = monotonicSec();

                {
                    QMutexLocker lk2(&g_ergo.lock);
                    constexpr std::uint64_t kGloveStaleMs = 500;
                    const std::uint64_t nowTick  = GetTickCount64();
                    const std::uint64_t ergoTick = g_ergo.lastTimeMs.load();
                    const bool gloveFresh = (ergoTick != 0) && (nowTick - ergoTick) < kGloveStaleMs;
                    const bool haveL = gloveFresh && g_ergo.haveLeft.load();
                    const bool haveR = gloveFresh && g_ergo.haveRight.load();
                    if (haveL) {
                        staging.leftGloveQ  = g_ergo.leftQ;
                        staging.leftGloveP  = g_ergo.leftP;
                    }
                    if (haveR) {
                        staging.rightGloveQ = g_ergo.rightQ;
                        staging.rightGloveP = g_ergo.rightP;
                    }
                    // §XXI ФИКС залипания: staging персистентен между кадрами, поэтому hasGloves
                    // надо ОБНОВЛЯТЬ по текущей свежести (обрыв >500мс -> false), иначе флаг оставался
                    // true и стримились/писались устаревшие позы пальцев (ср. гашение датчиков костюма).
                    staging.hasGloves = (haveL || haveR);
                }

                I.frame = staging;
                gotAny = true;
            }

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

            if (I.test && (now - I.lastAhrsDump) > 0.4) {
                QMutexLocker lk(&I.lock);
                constexpr int kPelvis = 0;
                const QVector3D& gf = I.dbgGyrFused[kPelvis];
                const double gyrNorm = std::sqrt(double(gf.x())*gf.x()
                                              + double(gf.y())*gf.y()
                                              + double(gf.z())*gf.z());
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(3);
                ss << "[ahrs] Pelvis"
                   << "  accErr=" << std::setw(6) << I.dbgAccErr[kPelvis]
                   << "  dynAccRej=" << std::setw(5) << std::setprecision(2)
                                     << I.dbgDynAccRej[kPelvis] << "deg"
                   << "  magErr=" << std::setw(5) << std::setprecision(2)
                                  << I.dbgDynMagRej[kPelvis] << "deg"
                   << "  gyr|w|=" << std::setw(6) << std::setprecision(2)
                                  << gyrNorm << "deg/s"
                   << "\n";

                const double magGateDeg =
                    fox::body::kMagnet.angleDiffFromModelMaxDeg;
                const double accGateDeg =
                    fox::body::kMagnet.angleDiffFromModelMaxDeg;
                ss << "[mag]  Pelvis"
                   << "  magErr=" << std::setw(5) << std::setprecision(2)
                                  << I.dbgDynMagRej[kPelvis] << "deg"
                   << "  accGate=" << (I.dbgDynAccRej[kPelvis] > accGateDeg
                                        ? "REJECTED" : "open")
                   << "  magGate=" << (I.dbgDynMagRej[kPelvis] > magGateDeg
                                        ? "REJECTED" : "open")
                   << "\n";
                std::cout << ss.str() << std::flush;
                I.lastAhrsDump = now;
            }

            if (fox::pose_solver::g_testFlag().load(std::memory_order_relaxed) &&
                fox::pose_solver::g_glovesFlag().load(std::memory_order_relaxed) &&
                (now - I.segAccDumpAccumSec) > 2.0) {
                I.segAccDumpAccumSec = now;
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(3);
                ss << "[acc-seg] per-segment KF residual (EMA τ=5s)\n";
                for (int s = 0; s < kXsensSegmentCount; ++s) {
                    if (!fox::body::kSensorPresent[s]) continue;
                    ss << "  s=" << std::setw(2) << s
                       << " " << std::left << std::setw(14) << kSegmentNames[s] << std::right
                       << "  accErr=" << std::setw(6) << I.segAccErrEma[s] << " m/s²"
                       << "  peak="   << std::setw(6) << I.segAccErrPeak[s] << " m/s²"
                       << "  magErr=" << std::setw(6) << I.segMagErrEma[s] << " (norm)\n";
                    I.segAccErrPeak[s] = 0.0f;
                }
                std::cout << ss.str() << std::flush;
            }

            if (I.test && (now - I.lastDump) > 1.5) {
                QMutexLocker lk(&I.lock);
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(3);
                ss << "\n========== [FUSED SNAPSHOT] sample="
                   << staging.sampleCounter
                   << "  t=" << std::setprecision(2) << now
                   << "s ==========\n";

                {
                    const double sinceLastPkt = now - I.lastPacket;
                    const double instFps =
                        ((now - fpsT0) > 1e-3) ? (framesThisSec / (now - fpsT0))
                                               : 0.0;
                    ss << "[net] trackers=" << trackerHandles.size()
                       << " active=" << I.activeTrackers.load()
                       << " sinceLastPkt=" << std::setprecision(3) << sinceLastPkt << "s"
                       << " fpsInst=" << std::setprecision(1) << instFps
                       << " fpsExpected=" << I.freqHz << "Hz"
                       << " sampleCounter=" << staging.sampleCounter << "\n";
                    ss << "[thr] networkThread=alive"
                       << " calGen=" << I.calGen.load(std::memory_order_relaxed)
                       << " status=" << I.status.load()
                       << "\n";
                    ss << std::setprecision(3);
                }

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

                ss << std::setprecision(4);
                ss << "--- per-sensor calibrated body-frame IMU (post acc-norm/gyr-bias/"
                      "mag-soft-iron/s2s; gyrFused = raw input to FOX_KFA EKF) ---\n";
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

                ss << std::setprecision(4);
                ss << "--- per-axis sensor transform chain (raw SDI -> recon -> "
                      "accNorm/gyrBias/magSoft -> s2s body -> fused) ---\n";
                {
                    auto V = [&](const QVector3D& v){
                        ss << "(" << std::setw(8) << v.x() << "," << std::setw(8) << v.y()
                           << "," << std::setw(8) << v.z() << ")"; };
                    for (int i = 0; i < kXsensSegmentCount; ++i) {
                        const quint8 fl = I.dbgChainFlags[i];
                        if (fl == 0) continue;
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

                ss << "--- calibration flags: s2s=" << (I.s2sActive ? "on" : "off")
                   << "  accNorm=" << (I.accNormActive ? "on" : "off")
                   << "  magNorm=" << (I.magNormActive ? "on" : "off")
                   << "  gyrBias=" << (I.gyrBiasActive ? "on" : "off")
                   << "  freq=" << std::setprecision(1) << I.freqHz << "Hz ---\n";

                {
                    const FusionAhrsSettings& fs = I.ahrsCfg[SEG_Pelvis];
                    const FusionAhrsInternalStates st =
                        FusionAhrsGetInternalStates(&I.fusion[SEG_Pelvis]);
                    ss << "--- fusion (§43 EKF, seg[0] Pelvis): sampleRate="
                       << std::setprecision(0) << fs.sampleRateHz << "Hz"
                       << " dipModel=" << std::setprecision(1) << fs.magDipModelDeg << "°"
                       << " declination=" << fs.magDeclinationDeg << "°"
                       << "  | dAcc=" << std::setprecision(2) << st.accelerationError << "m/s²"
                       << " fAccBoost=" << std::setprecision(0) << (st.accelerationRecoveryTrigger * 1000.0f)
                       << " magResid=" << std::setprecision(2) << st.magneticError << "°"
                       << " magGate=" << (st.magnetometerIgnored ? "closed" : "open")
                       << " ---\n";
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
            QThread::msleep(2);
        }
    }

    for (void* d : trackerHandles) api.deviceGotoConfig(d);
    api.controlClose(control);
    if (api.controlDestruct) api.controlDestruct(control);
    unloadApi(api);
    I.setStatus(ConnStatus::NotInitialized, "closed", this);
}

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
    {"body_upper_arm",     "Плечо",                             "Upper arm"},
    {"body_forearm",       "Предплечье",                        "Forearm"},
    {"body_hand",          "Кисть",                             "Hand"},
    {"body_thigh",         "Бедро",                             "Thigh"},
    {"body_shank",         "Голень",                            "Shank"},
    {"body_panel_label",   "Размеры (см)",                      "Body sizes (cm)"},
    {"body_panel_sub",     "0 = расчёт по росту",               "0 = derived from height"},
    {"dims_primary",       "Основные размеры",                  "Primary dimensions"},
    {"dims_breakdown",     "Расчётные длины сегментов",         "Derived segment lengths"},
    {"gender_label",       "Пол",                               "Gender"},
    {"gender_male",        "Мужской",                           "Male"},
    {"gender_female",      "Женский",                           "Female"},
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

    {"calib_walk_prepare", "Авто-проверка датчиков: приготовьтесь пройти ~5 м вперёд…",
                           "Sensor auto-check: get ready to walk ~5 m forward…"},
    {"calib_walk_capture", "Идите вперёд ~5 м обычным шагом",   "Walk forward ~5 m at a normal pace"},
    {"calib_larm_prepare", "Приготовьтесь поднять ЛЕВУЮ руку до 90° и опустить…",
                           "Get ready to raise your LEFT arm to 90° and lower…"},
    {"calib_larm_capture", "Поднимите и опустите ЛЕВУЮ руку",   "Raise and lower your LEFT arm"},
    {"calib_rarm_prepare", "Приготовьтесь поднять ПРАВУЮ руку до 90° и опустить…",
                           "Get ready to raise your RIGHT arm to 90° and lower…"},
    {"calib_rarm_capture", "Поднимите и опустите ПРАВУЮ руку",  "Raise and lower your RIGHT arm"},
    {"calib_lleg_prepare", "Приготовьтесь поднять ЛЕВУЮ ногу (бедро ~90°) и опустить…",
                           "Get ready to raise your LEFT leg (hip ~90°) and lower…"},
    {"calib_lleg_capture", "Поднимите и опустите ЛЕВУЮ ногу",   "Raise and lower your LEFT leg"},
    {"calib_rleg_prepare", "Приготовьтесь поднять ПРАВУЮ ногу (бедро ~90°) и опустить…",
                           "Get ready to raise your RIGHT leg (hip ~90°) and lower…"},
    {"calib_rleg_capture", "Поднимите и опустите ПРАВУЮ ногу",  "Raise and lower your RIGHT leg"},
    {"calib_move_hint",    "Движение для авто-проверки размещения датчиков",
                           "Motion for sensor-placement auto-check"},
    {"asl_check_label",    "Проверить размещение датчиков (≈40 с движений после калибровки)",
                           "Verify sensor placement (≈40 s of motions after calibration)"},
    {"asl_skipped",        "FoxSPC: проверка размещения пропущена",
                           "FoxSPC: placement check skipped"},
    {"calib_pose_empty",   "Калибровка позы не получила стабильных данных (актёр двигался или сенсоры не передают). Качество будет низким — рекомендуется повторить калибровку.", "Pose calibration captured no stable data (actor moved or sensors not streaming). Quality will be poor — recommend recalibrating."},

    {"asl_loading_failed", "FoxSPC: модель размещения датчиков не загрузилась — авто-проверка отключена",
                           "FoxSPC: placement model failed to load — auto-verification disabled"},
    {"asl_no_data",        "FoxSPC: недостаточно данных для авто-проверки размещения (нужны движения рук и ног)",
                           "FoxSPC: insufficient motion data for placement check (move arms / legs)"},
    {"asl_ok",             "FoxSPC: размещение датчиков подтверждено (%1/%2)",
                           "FoxSPC: placement verified (%1/%2)"},
    {"asl_mismatch",       "FoxSPC (справочно, на трекинг не влияет): датчик %1 по движению похож на %2 (p=%3)",
                           "FoxSPC (advisory, does not affect tracking): sensor %1 resembles %2 by motion (p=%3)"},
    {"asl_low_confidence", "FoxSPC: проверка размещения неинформативна (низкая уверенность модели) — результат не используется",
                           "FoxSPC: placement check inconclusive (low model confidence) — result not used"},
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
    {"sns_r_shoulder",     "правая лопатка",                    "r shoulder"},
    {"sns_r_upper_arm",    "правое плечо",                      "r upper arm"},
    {"sns_r_forearm",      "правое предплечье",                 "r forearm"},
    {"sns_r_hand",         "правая кисть",                      "r hand"},
    {"sns_l_shoulder",     "левая лопатка",                     "l shoulder"},
    {"sns_l_upper_arm",    "левое плечо",                       "l upper arm"},
    {"sns_l_forearm",      "левое предплечье",                  "l forearm"},
    {"sns_l_hand",         "левая кисть",                       "l hand"},
    {"sns_r_upper_leg",    "правое бедро",                      "r thigh"},
    {"sns_r_lower_leg",    "правая голень",                     "r shin"},
    {"sns_r_foot",         "правая стопа",                      "r foot"},
    {"sns_l_upper_leg",    "левое бедро",                       "l thigh"},
    {"sns_l_lower_leg",    "левая голень",                      "l shin"},
    {"sns_l_foot",         "левая стопа",                       "l foot"},

    {"transport_com",      "\xF0\x9F\x94\x8C  COM-порт — USB / Awinda dongle",
                           "\xF0\x9F\x94\x8C  COM port — USB / Awinda dongle"},
    {"transport_wifi",     "\xF0\x9F\x93\xA1  WiFi / Ethernet — по сети (Link / Awinda)",
                           "\xF0\x9F\x93\xA1  WiFi / Ethernet — network (Link / Awinda)"},
    {"suit_awinda",        "\xF0\x9F\x9F\xA0  Xsens Awinda — 90 Гц",
                           "\xF0\x9F\x9F\xA0  Xsens Awinda — 90 Hz"},
    {"suit_link",          "\xF0\x9F\x9F\xA3  Xsens Link — 240 Гц",
                           "\xF0\x9F\x9F\xA3  Xsens Link — 240 Hz"},

    {"wifi_hint",          "Подключите этот ПК к той же Wi-Fi сети, что и костюм "
                           "(через настройки Wi-Fi Windows), затем нажмите «Подключить».",
                           "Connect this PC to the same Wi-Fi network as the suit "
                           "(via Windows Wi-Fi settings), then press Connect."},

    {"calib_tpose_tune",     "T-поза — точная настройка: стойте %1с неподвижно",
                             "T-Pose — fine-tuning: stand still for %1 s"},
    {"calib_tpose_converge", "T-поза: сходимость фильтра %1с — стойте",
                             "T-Pose: filter converging %1 s — hold still"},
    {"calib_tpose_capture",  "T-поза: захват %1/%2 — стойте",
                             "T-Pose: capture %1/%2 — hold still"},
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
    return QString::fromLatin1(key);
}

static void paintDot(QLabel* lab, const char* colorHex)
{
    lab->setFixedSize(14, 14);
    lab->setStyleSheet(QString("background:%1; border-radius:7px;").arg(colorHex));
}

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
    } else {
        p.fillRect(QRectF(0, 0, W, H), QColor("#012169"));
        QPen wide(QColor("#FFFFFF"));  wide.setWidth(5);
        QPen thin(QColor("#C8102E"));  thin.setWidth(2);

        p.setPen(wide);
        p.drawLine(0, 0, W, H);
        p.drawLine(W, 0, 0, H);
        p.setPen(thin);
        p.drawLine(0, 0, W, H);
        p.drawLine(W, 0, 0, H);

        wide.setWidth(6); p.setPen(wide);
        p.drawLine(W / 2, 0, W / 2, H);
        p.drawLine(0, H / 2, W, H / 2);

        thin.setWidth(3); p.setPen(thin);
        p.drawLine(W / 2, 0, W / 2, H);
        p.drawLine(0, H / 2, W, H / 2);
    }

    p.setClipping(false);
    p.setPen(QPen(QColor(0, 0, 0, 180), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(QRectF(0.5, 0.5, W - 1, H - 1), 3.0, 3.0);
    return QIcon(pm);
}

namespace spc {

#ifdef _WIN32
using PathChar = wchar_t;
static std::wstring toOrtPath(const QString& p) { return p.toStdWString(); }
#else
using PathChar = char;
static std::string  toOrtPath(const QString& p) { return p.toStdString(); }
#endif

constexpr int kMinWindow = 32;

static std::vector<float> realDftMag(const std::vector<float>& x)
{
    const int N = int(x.size());
    if (N < 2) return {};
    std::vector<float> mag(N / 2 + 1, 0.0f);
    for (int k = 0; k <= N / 2; ++k) {
        double re = 0.0, im = 0.0;
        for (int n = 0; n < N; ++n) {
            const double th = -2.0 * M_PI * double(k) * double(n) / double(N);
            re += double(x[n]) * std::cos(th);
            im += double(x[n]) * std::sin(th);
        }
        mag[k] = float(std::sqrt(re * re + im * im));
    }
    return mag;
}

static std::vector<float> axisWindow(
    const NewSessionWizard::RawImuBuf& buf, fox::body::SpcSignal sig,
    fox::body::SpcAxis ax, int s, int e)
{
    const auto& src = (sig == fox::body::SpcSignal::Acc) ? buf.acc : buf.gyr;
    if (s < 0 || e <= s || e > int(src.size())) return {};
    std::vector<float> out;
    out.reserve(e - s);
    for (int i = s; i < e; ++i) {
        const QVector3D& v = src[i];
        switch (ax) {
            case fox::body::SpcAxis::X:    out.push_back(v.x()); break;
            case fox::body::SpcAxis::Y:    out.push_back(v.y()); break;
            case fox::body::SpcAxis::Z:    out.push_back(v.z()); break;
            case fox::body::SpcAxis::XAbs: out.push_back(std::abs(v.x())); break;
            case fox::body::SpcAxis::YAbs: out.push_back(std::abs(v.y())); break;
            case fox::body::SpcAxis::ZAbs: out.push_back(std::abs(v.z())); break;
            case fox::body::SpcAxis::Normxyz:
                out.push_back(std::sqrt(v.x()*v.x() + v.y()*v.y() + v.z()*v.z()));
                break;
        }
    }
    return out;
}

static std::vector<float> bandSlice(
    const std::vector<float>& mag, int N, double fLo, double fHi, double fs)
{
    if (mag.empty() || N <= 1) return {};
    const int kLo = std::max(0, int(std::round(fLo * double(N) / fs)));
    const int kHi = std::min(int(mag.size()) - 1,
                             (fHi <= 0.0) ? int(mag.size()) - 1
                                          : int(std::round(fHi * double(N) / fs)));
    if (kHi <= kLo) return {};
    return std::vector<float>(mag.begin() + kLo, mag.begin() + kHi + 1);
}

static float stat_mean(const std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    double s = 0.0;
    for (float x : v) s += x;
    return float(s / double(v.size()));
}
static float stat_sum(const std::vector<float>& v) {
    double s = 0.0; for (float x : v) s += x; return float(s);
}
static float stat_std(const std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    const double m = stat_mean(v);
    double s = 0.0;
    for (float x : v) { const double d = x - m; s += d * d; }
    return float(std::sqrt(s / double(v.size())));
}
static float stat_var(const std::vector<float>& v) {
    const float s = stat_std(v); return s * s;
}
static float stat_rms(const std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    double s = 0.0; for (float x : v) s += double(x) * double(x);
    return float(std::sqrt(s / double(v.size())));
}
static float stat_max(const std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    float m = v[0]; for (float x : v) if (x > m) m = x; return m;
}
static float stat_maxIdx(const std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    int idx = 0; for (int i = 1; i < int(v.size()); ++i) if (v[i] > v[idx]) idx = i;
    return float(idx) / float(v.size());
}
static float stat_skew(const std::vector<float>& v) {
    if (v.size() < 3) return 0.0f;
    const double m = stat_mean(v);
    double m2 = 0.0, m3 = 0.0;
    for (float x : v) { const double d = x - m; m2 += d*d; m3 += d*d*d; }
    m2 /= double(v.size());
    m3 /= double(v.size());
    if (m2 < 1e-12) return 0.0f;
    return float(m3 / std::pow(m2, 1.5));
}
static float stat_kurt(const std::vector<float>& v) {
    if (v.size() < 4) return 0.0f;
    const double m = stat_mean(v);
    double m2 = 0.0, m4 = 0.0;
    for (float x : v) { const double d = x - m; m2 += d*d; m4 += d*d*d*d; }
    m2 /= double(v.size());
    m4 /= double(v.size());
    if (m2 < 1e-12) return -3.0f;
    return float(m4 / (m2 * m2) - 3.0);
}

static float pearson(const std::vector<float>& a, const std::vector<float>& b) {
    const int n = std::min(int(a.size()), int(b.size()));
    if (n < 2) return 0.0f;
    double ma = 0.0, mb = 0.0;
    for (int i = 0; i < n; ++i) { ma += a[i]; mb += b[i]; }
    ma /= n; mb /= n;
    double sab = 0.0, saa = 0.0, sbb = 0.0;
    for (int i = 0; i < n; ++i) {
        const double da = a[i] - ma, db = b[i] - mb;
        sab += da * db; saa += da * da; sbb += db * db;
    }
    if (saa < 1e-12 || sbb < 1e-12) return 0.0f;
    return float(sab / std::sqrt(saa * sbb));
}

static std::pair<int, int> epochWindow(
    const NewSessionWizard::RawImuBuf& buf, fox::body::SpcEpoch e)
{
    using fox::body::SpcEpoch;
    const NewSessionWizard::RawImuBuf::Win* w = nullptr;
    switch (e) {
        case SpcEpoch::Calibration:   w = &buf.epochCalibration; break;
        case SpcEpoch::LeftArmRaise:  w = &buf.epochLeftArm;     break;
        case SpcEpoch::RightArmRaise: w = &buf.epochRightArm;    break;
        case SpcEpoch::LeftLegRaise:  w = &buf.epochLeftLeg;     break;
        case SpcEpoch::RightLegRaise: w = &buf.epochRightLeg;    break;
    }
    return { w->start, w->end };
}

static float computeFeature(
    int t,
    const std::array<NewSessionWizard::RawImuBuf, kXsensSegmentCount>& bufs,
    const fox::body::SpcFeatureSpec& f)
{
    constexpr double kFs = 60.0;
    using fox::body::SpcStat;
    using fox::body::SpcBand;
    using fox::body::SpcAxis;

    auto wT = epochWindow(bufs[t], f.epoch);
    if (wT.first < 0) return 0.0f;
    std::vector<float> target = axisWindow(bufs[t], f.signal, f.axis,
                                            wT.first, wT.second);
    if (int(target.size()) < kMinWindow) return 0.0f;

    std::vector<float> work = target;
    if (f.band != SpcBand::None) {
        const int N = int(target.size());
        std::vector<float> centered = target;
        double mean = 0.0;
        for (float v : centered) mean += double(v);
        if (N > 0) mean /= double(N);
        for (float& v : centered) v = float(double(v) - mean);
        std::vector<float> mag = realDftMag(centered);
        double fLo = 0.0, fHi = 0.0;
        switch (f.band) {
            case SpcBand::Band0p5To4:   fLo = 0.5;  fHi = 4.0;  break;
            case SpcBand::Band4p5To10:  fLo = 4.5;  fHi = 10.0; break;
            case SpcBand::Band10ToNyq:  fLo = 10.0; fHi = kFs / 2.0; break;
            default: break;
        }
        work = bandSlice(mag, N, fLo, fHi, kFs);
        if (work.empty()) return 0.0f;
    }

    switch (f.stat) {
        case SpcStat::Mean:     return stat_mean(work);
        case SpcStat::Sum:      return stat_sum(work);
        case SpcStat::Std:      return stat_std(work);
        case SpcStat::Var:      return stat_var(work);
        case SpcStat::Rms:      return stat_rms(work);
        case SpcStat::Max:      return stat_max(work);
        case SpcStat::MaxIdx:   return stat_maxIdx(work);
        case SpcStat::Skew:     return stat_skew(work);
        case SpcStat::Kurtosis: return stat_kurt(work);
        case SpcStat::SameAxisInterSensorCorrMax:
        case SpcStat::SameAxisInterSensorCorrAbsMax:
        case SpcStat::SameAxisInterSensorCorrSum:
        case SpcStat::SameAxisInterSensorCorrAbsSum: {

            float maxC = -2.0f, maxAbsC = 0.0f, sumC = 0.0f, sumAbsC = 0.0f;
            int n = 0;
            for (int s = 0; s < kXsensSegmentCount; ++s) {
                if (s == t) continue;
                if (!fox::body::kSensorPresent[s]) continue;
                auto wS = epochWindow(bufs[s], f.epoch);
                if (wS.first < 0) continue;
                std::vector<float> other = axisWindow(bufs[s], f.signal, f.axis,
                                                      wS.first, wS.second);
                const int nm = std::min(int(target.size()), int(other.size()));
                if (nm < kMinWindow) continue;
                std::vector<float> a(target.begin(), target.begin() + nm);
                std::vector<float> b(other.begin(),   other.begin()  + nm);
                const float c = pearson(a, b);
                if (c > maxC)        maxC    = c;
                if (std::abs(c) > maxAbsC) maxAbsC = std::abs(c);
                sumC    += c;
                sumAbsC += std::abs(c);
                ++n;
            }
            if (n == 0) return 0.0f;
            if (f.stat == SpcStat::SameAxisInterSensorCorrMax)    return maxC;
            if (f.stat == SpcStat::SameAxisInterSensorCorrAbsMax) return maxAbsC;
            if (f.stat == SpcStat::SameAxisInterSensorCorrSum)    return sumC;
            return sumAbsC;
        }
        case SpcStat::SameSensorInterAxisCorrMax:
        case SpcStat::SameSensorInterAxisCorrAbsMax:
        case SpcStat::SameSensorInterAxisCorrSum:
        case SpcStat::SameSensorInterAxisCorrAbsSum: {

            const SpcAxis allAxes[3] = { SpcAxis::X, SpcAxis::Y, SpcAxis::Z };
            SpcAxis selfAxis = f.axis;
            if (selfAxis == SpcAxis::XAbs)    selfAxis = SpcAxis::X;
            if (selfAxis == SpcAxis::YAbs)    selfAxis = SpcAxis::Y;
            if (selfAxis == SpcAxis::ZAbs)    selfAxis = SpcAxis::Z;
            if (selfAxis == SpcAxis::Normxyz) selfAxis = SpcAxis::X;
            float maxC = -2.0f, maxAbsC = 0.0f, sumC = 0.0f, sumAbsC = 0.0f;
            int n = 0;
            for (SpcAxis other : allAxes) {
                if (other == selfAxis) continue;
                std::vector<float> b = axisWindow(bufs[t], f.signal, other,
                                                   wT.first, wT.second);
                const int nm = std::min(int(target.size()), int(b.size()));
                if (nm < kMinWindow) continue;
                std::vector<float> a(target.begin(), target.begin() + nm);
                b.resize(nm);
                const float c = pearson(a, b);
                if (c > maxC) maxC = c;
                if (std::abs(c) > maxAbsC) maxAbsC = std::abs(c);
                sumC += c;
                sumAbsC += std::abs(c);
                ++n;
            }
            if (n == 0) return 0.0f;
            if (f.stat == SpcStat::SameSensorInterAxisCorrMax)    return maxC;
            if (f.stat == SpcStat::SameSensorInterAxisCorrAbsMax) return maxAbsC;
            if (f.stat == SpcStat::SameSensorInterAxisCorrSum)    return sumC;
            return sumAbsC;
        }
    }
    return 0.0f;
}

static std::array<float, fox::body::kSpcFeatureCount> extractFeatures315(
    int target,
    const std::array<NewSessionWizard::RawImuBuf, kXsensSegmentCount>& bufs)
{
    std::array<float, fox::body::kSpcFeatureCount> out{};
    for (int m = 0; m < fox::body::kSpcFeatureCount; ++m) {
        const int   c   = fox::body::kSpcModelPerm[m];
        const float raw = computeFeature(target, bufs, fox::body::kFeatureSpecs[c]);
        const float lo  = fox::body::kFeatureMinM[m];
        const float hi  = fox::body::kFeatureMaxM[m];
        const float den = (hi > lo) ? (hi - lo) : 1.0f;
        float n = (raw - lo) / den;
        if (n < 0.0f) n = 0.0f;
        if (n > 1.0f) n = 1.0f;
        out[m] = n;
    }
    return out;
}

static std::array<int, fox::body::kSpcClassCount> hungarian17(
    const std::array<std::array<float, fox::body::kSpcClassCount>,
                     fox::body::kSpcClassCount>& cost)
{
    constexpr int N = fox::body::kSpcClassCount;
    std::array<double, N + 1> u{}, v{};
    std::array<int,    N + 1> p{}, way{};
    for (int i = 1; i <= N; ++i) {
        p[0] = i;
        int j0 = 0;
        std::array<double, N + 1> minv;  minv.fill(1e30);
        std::array<bool,   N + 1> used;  used.fill(false);
        do {
            used[j0] = true;
            const int i0 = p[j0];
            double delta = 1e30;
            int j1 = 0;
            for (int j = 1; j <= N; ++j) {
                if (used[j]) continue;
                const double cur = double(cost[i0 - 1][j - 1]) - u[i0] - v[j];
                if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                if (minv[j] < delta) { delta = minv[j]; j1 = j; }
            }
            for (int j = 0; j <= N; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else         { minv[j] -= delta; }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0    = j1;
        } while (j0 != 0);
    }
    std::array<int, N> assign{};
    for (int j = 1; j <= N; ++j) {
        if (p[j] >= 1 && p[j] <= N) assign[p[j] - 1] = j - 1;
    }
    return assign;
}

struct PlacementClassifier {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "fox_kfa_spc"};
    std::optional<Ort::Session> session;
    bool ready = false;

    void load(const QString& modelPath, bool log) {
        try {
            Ort::SessionOptions opts;
            opts.SetIntraOpNumThreads(1);
            opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            const auto p = toOrtPath(modelPath);
            session.emplace(env, p.c_str(), opts);
            ready = true;
            if (log) {
                std::cout << "[FoxSPC] model loaded: "
                          << modelPath.toStdString() << "\n";
            }
        } catch (const std::exception& ex) {
            ready = false;
            if (log) {
                std::cout << "[FoxSPC] load failed: " << ex.what() << "\n";
            }
        }
    }

    std::array<float, fox::body::kSpcClassCount> classify(
        const std::array<float, fox::body::kSpcFeatureCount>& x)
    {
        std::array<float, fox::body::kSpcClassCount> probs{};
        if (!ready || !session) return probs;
        try {
            Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(
                OrtArenaAllocator, OrtMemTypeDefault);
            std::array<int64_t, 2> shape{1, fox::body::kSpcFeatureCount};
            std::vector<float> data(x.begin(), x.end());
            Ort::Value input = Ort::Value::CreateTensor<float>(
                mem, data.data(), data.size(), shape.data(), shape.size());

            const char* inNames[]  = {"X"};
            const char* outNames[] = {"label", "probabilities"};
            auto outs = session->Run(Ort::RunOptions{nullptr},
                                     inNames, &input, 1,
                                     outNames, 2);
            if (outs.size() < 2) return probs;
            float* p = outs[1].GetTensorMutableData<float>();
            for (int i = 0; i < fox::body::kSpcClassCount; ++i) probs[i] = p[i];
        } catch (const std::exception& ex) {

            if (pose_solver::g_testFlag().load(std::memory_order_relaxed)) {
                std::cout << "[FoxSPC] inference failed: " << ex.what() << "\n";
                std::cout.flush();
            }
        }
        return probs;
    }
};

struct PlacementReport {
    bool        haveModel            = false;
    bool        haveData             = false;
    int         verified             = 0;
    int         total                = 0;
    QStringList mismatches;
    float       avgMaxP              = 0.0f;
    bool        suitUncertaintyAlarm = false;
};

static PlacementReport analyzePlacement(
    PlacementClassifier& clf,
    const std::array<NewSessionWizard::RawImuBuf, kXsensSegmentCount>& bufs,
    bool testLog)
{
    PlacementReport rep;
    rep.haveModel = clf.ready;
    if (!clf.ready) return rep;

    std::array<std::array<float, fox::body::kSpcClassCount>,
               fox::body::kSpcClassCount> probs{};
    std::array<int, fox::body::kSpcClassCount> probRow{};
    int row = 0;
    int dataRows = 0;
    for (int s = 0; s < kXsensSegmentCount; ++s) {
        if (!fox::body::kSensorPresent[s]) continue;
        const auto wC = bufs[s].epochCalibration;
        if (wC.start >= 0 && (wC.end - wC.start) >= kMinWindow) ++dataRows;
        auto feat = extractFeatures315(s, bufs);
        probs[row] = clf.classify(feat);
        probRow[row] = s;
        ++row;
    }
    rep.haveData = (dataRows > 0);
    if (row == 0) return rep;

    std::array<std::array<float, fox::body::kSpcClassCount>,
               fox::body::kSpcClassCount> cost{};
    constexpr float kPhantomCost = 1.0e6f;
    for (int r = 0; r < int(fox::body::kSpcClassCount); ++r) {
        if (r < row) {
            for (int c = 0; c < fox::body::kSpcClassCount; ++c) {
                const float p = std::max(probs[r][c], 1e-6f);
                cost[r][c] = -std::log(p);
            }
        } else {
            for (int c = 0; c < fox::body::kSpcClassCount; ++c) {
                cost[r][c] = kPhantomCost;
            }
        }
    }
    auto assign = hungarian17(cost);

    float sumMax = 0.0f;
    for (int r = 0; r < row; ++r) {
        const int seg = probRow[r];
        const int expectedClass = fox::body::kSegToClass[seg];
        const int assignedClass = assign[r];
        const float maxP = *std::max_element(probs[r].begin(), probs[r].end());
        sumMax += maxP;

        if (testLog) {
            std::cout << "[FoxSPC] sensor " << std::setw(2) << seg
                      << " (" << kSegmentNames[seg] << ") → class "
                      << assignedClass << " ("
                      << fox::body::kSensorPlacementClasses[assignedClass]
                      << ") max_p=" << std::fixed << std::setprecision(3) << maxP
                      << "  expected=" << expectedClass << "\n";
        }

        if (assignedClass == expectedClass) {
            ++rep.verified;
        } else if (maxP >= fox::body::kSpcAcceptanceP) {
            rep.mismatches << QString("%1→%2 (p=%3)")
                                .arg(QString::fromUtf8(kSegmentNames[seg]))
                                .arg(QString::fromUtf8(
                                    fox::body::kSensorPlacementClasses[assignedClass]))
                                .arg(maxP, 0, 'f', 2);
        }
        ++rep.total;
    }
    rep.avgMaxP = (row > 0) ? sumMax / float(row) : 0.0f;

    float suitUncertainty = 0.0f;
    for (int r = 0; r < row; ++r) {
        const float maxP = *std::max_element(probs[r].begin(), probs[r].end());
        suitUncertainty += (1.0f - maxP);
    }
    rep.suitUncertaintyAlarm = suitUncertainty > fox::body::kSpcSuitUncertSum;

    return rep;
}

}

static spc::PlacementClassifier& g_placementClf() {
    static spc::PlacementClassifier instance;
    return instance;
}

NewSessionWizard::NewSessionWizard(MocapReceiver* rx, bool testMode, QWidget* parent)
    : QDialog(parent), m_rx(rx), m_test(testMode)
{
    setModal(true);
    setWindowTitle(Lang::t("app_title"));
    setMinimumSize(760, 640);

    if (!g_placementClf().ready) {
        const QString modelPath =
            QCoreApplication::applicationDirPath()
            + "/fox_sensor_placement_classifier.onnx";
        g_placementClf().load(modelPath, m_test);
    }
    m_liveSpcEnabled = g_placementClf().ready;

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

    m_statusTimer.setInterval(250);
    connect(&m_statusTimer, &QTimer::timeout, this, &NewSessionWizard::onStatusTick);
    m_statusTimer.start();

    m_countTimer.setInterval(100);
    {
        const double nativeRateHz =
            std::max(60.0, fox::body::constants::kSampleRateHz);
        const int captureMs = std::max(1, int(1000.0 / nativeRateHz + 0.5));
        m_captureTimer.setInterval(captureMs);
    }
    connect(&m_countTimer,   &QTimer::timeout, this, &NewSessionWizard::onCountdownTick);
    connect(&m_captureTimer, &QTimer::timeout, this, &NewSessionWizard::onCaptureTick);

    connect(&Lang::instance(), &Lang::changed, this, &NewSessionWizard::retranslate);

    retranslate();
    onModeChanged();
    onStatusTick();
    updateNavButtons();

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

        langLabel->setProperty("isLangLabel", true);
        m_pages->addWidget(p);
    }

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

            if (m_cbxTransport) m_cbxTransport->setCurrentIndex(s == SuitType::Link ? 1 : 0);
        });

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

    {
        auto* p = new QWidget();
        m_dimsTitle = new QLabel(p);
        m_dimsTitle->setObjectName("heroHeading");
        m_dimsTitle->setAlignment(Qt::AlignCenter);

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

            s->setToolTip(Lang::t("dims_type_hint"));

            if (auto* le = s->findChild<QLineEdit*>()) {
                le->setAlignment(Qt::AlignCenter);
                le->setFocusPolicy(Qt::StrongFocus);
            }
        };
        m_height = new QDoubleSpinBox(p);
        configSpin(m_height, 175.0, 100.0, 230.0, 0.5);
        m_foot   = new QDoubleSpinBox(p);
        configSpin(m_foot,    26.0,  15.0,  35.0, 0.5);

        m_upperArm = new QDoubleSpinBox(p);
        configSpin(m_upperArm, 0.0,   0.0,  70.0, 0.5);
        m_forearm  = new QDoubleSpinBox(p);
        configSpin(m_forearm,  0.0,   0.0,  60.0, 0.5);
        m_hand     = new QDoubleSpinBox(p);
        configSpin(m_hand,     0.0,   0.0,  35.0, 0.5);
        m_thigh    = new QDoubleSpinBox(p);
        configSpin(m_thigh,    0.0,   0.0,  90.0, 0.5);
        m_shank    = new QDoubleSpinBox(p);
        configSpin(m_shank,    0.0,   0.0,  85.0, 0.5);

        m_hip      = new QDoubleSpinBox(p);
        configSpin(m_hip,      0.0,   0.0,  60.0, 0.5);
        m_shoulder = new QDoubleSpinBox(p);
        configSpin(m_shoulder, 0.0,   0.0,  70.0, 0.5);
        m_trunk    = new QDoubleSpinBox(p);
        configSpin(m_trunk,    0.0,   0.0, 120.0, 0.5);

        m_gender = new QComboBox(p);
        m_gender->addItem(Lang::t("gender_male"));
        m_gender->addItem(Lang::t("gender_female"));
        m_gender->setCurrentIndex(0);
        m_lblGender = new QLabel(p);
        m_lblGender->setStyleSheet("font-weight:600;");

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
        for (auto* s : { m_height, m_foot, m_upperArm, m_forearm, m_hand,
                         m_thigh, m_shank, m_hip, m_shoulder, m_trunk }) {
            if (auto* le = s->findChild<QLineEdit*>())
                le->installEventFilter(&s_selAll);
        }

        m_lblHeight   = new QLabel(p);
        m_lblFoot     = new QLabel(p);
        m_lblUpperArm = new QLabel(p);
        m_lblForearm  = new QLabel(p);
        m_lblHand     = new QLabel(p);
        m_lblThigh    = new QLabel(p);
        m_lblShank    = new QLabel(p);
        m_lblHip      = new QLabel(p);
        m_lblShoulder = new QLabel(p);
        m_lblTrunk    = new QLabel(p);
        for (auto* lb : { m_lblHeight, m_lblFoot, m_lblUpperArm, m_lblForearm,
                          m_lblHand, m_lblThigh, m_lblShank, m_lblHip,
                          m_lblShoulder, m_lblTrunk })
            lb->setStyleSheet("font-weight:600;");

        auto* primaryBox = new QGroupBox(p);
        primaryBox->setProperty("isPrimaryDims", true);
        auto* primaryLay = new QGridLayout(primaryBox);
        primaryLay->setContentsMargins(24, 20, 24, 20);
        primaryLay->setHorizontalSpacing(32);
        primaryLay->setVerticalSpacing(10);
        primaryLay->addWidget(m_lblGender,   0, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_gender,      0, 1);
        primaryLay->addWidget(m_lblHeight,   1, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_height,      1, 1);
        primaryLay->addWidget(m_lblFoot,     2, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_foot,        2, 1);
        primaryLay->addWidget(m_lblUpperArm, 3, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_upperArm,    3, 1);
        primaryLay->addWidget(m_lblForearm,  4, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_forearm,     4, 1);
        primaryLay->addWidget(m_lblHand,     5, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_hand,        5, 1);
        primaryLay->addWidget(m_lblThigh,    6, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_thigh,       6, 1);
        primaryLay->addWidget(m_lblShank,    7, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_shank,       7, 1);

        primaryLay->addWidget(m_lblHip,      8, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_hip,         8, 1);
        primaryLay->addWidget(m_lblShoulder, 9, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_shoulder,    9, 1);
        primaryLay->addWidget(m_lblTrunk,    10, 0, Qt::AlignRight | Qt::AlignVCenter);
        primaryLay->addWidget(m_trunk,       10, 1);

        auto* breakdownBox = new QGroupBox(p);
        breakdownBox->setProperty("isBreakdownBox", true);
        auto* bg = new QGridLayout(breakdownBox);
        bg->setContentsMargins(24, 20, 24, 20);
        bg->setHorizontalSpacing(40);
        bg->setVerticalSpacing(6);

        const char* rowKeys[9] = {
            "bk_trunk", "bk_upper_arm", "bk_forearm",
            "bk_thigh", "bk_shin", "bk_foot",

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
            const fox::body::Gender g =
                (m_gender && m_gender->currentIndex() == 1)
                    ? fox::body::GenderFemale : fox::body::GenderMale;
            const auto& anthro = fox::body::anthroFor(g);

            auto effM = [](QDoubleSpinBox* sb) {
                return (sb && sb->value() > 0.0) ? sb->value() / 100.0 : 0.0;
            };
            const double trunkScale = h / 1.75;
            const double hipCm = m_hip ? m_hip->value() : 0.0;
            const double hipM   = (hipCm > 0.0) ? std::max(0.05, hipCm / 200.0)
                                                : 0.10 * trunkScale;
            const double shldCm = m_shoulder ? m_shoulder->value() : 0.0;
            const double shldM  = (shldCm > 0.0) ? std::max(0.05, shldCm / 200.0)
                                                 : 0.05 * trunkScale;
            const double trunkCm = m_trunk ? m_trunk->value() : 0.0;
            const double trunkM  = (trunkCm > 0.0) ? std::max(0.30, trunkCm / 100.0)
                                                   : fox::body::trunkLengthM(h);
            struct V { const char* k; double m; };
            V vals[9] = {
                { "bk_trunk",     anthro.trunkRatio * h   },
                { "bk_upper_arm", effM(m_upperArm)        },
                { "bk_forearm",   effM(m_forearm)         },
                { "bk_thigh",     effM(m_thigh)           },
                { "bk_shin",      effM(m_shank)           },
                { "bk_foot",      fl                      },
                { "bk_hip",       hipM * 2.0              },
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
        auto fillLimbDefaults = [this, updateBreakdown]() {
            const fox::body::Gender g =
                (m_gender && m_gender->currentIndex() == 1)
                    ? fox::body::GenderFemale : fox::body::GenderMale;
            const double hcm = m_height ? m_height->value() : 175.0;
            const auto d = SkeletonXsens::defaultLimbCm(g, hcm);
            QDoubleSpinBox* fld[5] = { m_upperArm, m_forearm, m_hand, m_thigh, m_shank };
            for (int i = 0; i < 5; ++i) {
                if (!fld[i]) continue;
                fld[i]->blockSignals(true);
                fld[i]->setValue(d[static_cast<std::size_t>(i)]);
                fld[i]->blockSignals(false);
            }
            updateBreakdown();
        };
        connect(m_height, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                p, [fillLimbDefaults](double){ fillLimbDefaults(); });
        connect(m_foot,   QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                p, [updateBreakdown](double){ updateBreakdown(); });
        for (auto* sb : { m_upperArm, m_forearm, m_hand, m_thigh, m_shank,
                          m_hip, m_shoulder, m_trunk })
            connect(sb, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                    p, [updateBreakdown](double){ updateBreakdown(); });
        connect(m_gender, QOverload<int>::of(&QComboBox::currentIndexChanged),
                p, [fillLimbDefaults](int){ fillLimbDefaults(); });
        fillLimbDefaults();

        m_dimsHint = new QLabel(p);
        m_dimsHint->setObjectName("subtle");
        m_dimsHint->setWordWrap(true);
        m_dimsHint->setAlignment(Qt::AlignCenter);

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

    {
        auto* p = new QWidget();
        m_calibTitle = new QLabel(p);
        m_calibTitle->setObjectName("heroHeading");
        m_calibTitle->setAlignment(Qt::AlignCenter);

        auto* imgFrame = new QWidget(p);
        imgFrame->setObjectName("poseFrame");
        m_poseImage = new QLabel(imgFrame);
        m_poseImage->setAlignment(Qt::AlignCenter);

        m_poseImage->setMinimumHeight(180);
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

        m_placementInfo = new QLabel(p);
        m_placementInfo->setAlignment(Qt::AlignCenter);
        m_placementInfo->setStyleSheet("color:#9B9B9B; font-size:9pt;");
        m_placementInfo->setWordWrap(true);

        m_btnCalibBegin = new QPushButton(p);
        m_btnCalibBegin->setObjectName("primary");
        m_btnCalibBegin->setMinimumHeight(42);
        m_btnCalibBegin->setMinimumWidth(240);
        connect(m_btnCalibBegin, &QPushButton::clicked,
                this, &NewSessionWizard::onCalibrationBegin);

        m_chkSensorCheck = new QCheckBox(p);
        m_chkSensorCheck->setChecked(false);
        m_chkSensorCheck->setEnabled(m_liveSpcEnabled);
        m_chkSensorCheck->setStyleSheet("color:#9B9B9B; font-size:9pt;");
        connect(m_chkSensorCheck, &QCheckBox::toggled,
                this, [this](bool on) { m_doSensorCheck = on; });

        auto* scrollHost = new QWidget(p);
        auto* hostLay    = new QVBoxLayout(scrollHost);
        hostLay->setContentsMargins(0, 0, 0, 0);
        hostLay->setSpacing(0);
        hostLay->addWidget(imgFrame, 1);
        hostLay->addSpacing(10);
        hostLay->addWidget(m_poseHint);
        hostLay->addSpacing(6);
        hostLay->addWidget(m_connBadge, 0, Qt::AlignHCenter);
        hostLay->addSpacing(10);
        hostLay->addWidget(m_countLabel);
        hostLay->addSpacing(4);
        hostLay->addWidget(m_countdownBar);
        hostLay->addSpacing(4);
        hostLay->addWidget(m_readyBar);
        hostLay->addSpacing(6);
        hostLay->addWidget(m_stillLabel);
        hostLay->addWidget(m_calibStatus);
        hostLay->addWidget(m_calibQuality);
        hostLay->addWidget(m_placementInfo);
        hostLay->addSpacing(8);
        hostLay->addWidget(m_chkSensorCheck, 0, Qt::AlignHCenter);
        hostLay->addSpacing(12);
        hostLay->addWidget(m_btnCalibBegin, 0, Qt::AlignHCenter);

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
        lay->setContentsMargins(28, 10, 28, 14);
        lay->setSpacing(0);
        lay->addWidget(m_calibTitle);
        lay->addSpacing(10);
        lay->addWidget(scroll, 1);

        m_pages->addWidget(p);
    }

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

void NewSessionWizard::logCalibPhaseTransition(const char* tag)
{
    if (!m_test) return;
    static constexpr const char* kPhaseNames[] = {
        "Idle", "PrepT", "CaptureT", "SettleT",
        "PrepN", "CaptureN", "Settle", "PrepMove", "CaptureMove", "LiveSpc", "Done"
    };
    const int idx = static_cast<int>(m_phase);
    const char* name = (idx >= 0 && idx < int(std::size(kPhaseNames)))
                       ? kPhaseNames[idx] : "?";
    std::cout << "[calib] -> " << name;
    if (tag && *tag) std::cout << "  (" << tag << ")";
    std::cout << "  goodSamples=" << m_goodSamples
              << "  bufSize="     << int(m_samples.size());
    if (m_phaseStartMs > 0) {
        const qint64 now = QDateTime::currentMSecsSinceEpoch();
        std::cout << "  elapsed=" << (now - m_phaseStartMs) << "ms";
    }
    std::cout << "\n";
    std::cout.flush();
}

void NewSessionWizard::refreshPoseImage()
{
    if (!m_poseImage) return;
    const char* path = isNPosePhase() ? ":/img/npose.png" : ":/img/tpose.png";
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
    if (m_lblGender)      m_lblGender->setText(Lang::t("gender_label") + ":");
    if (m_gender && m_gender->count() >= 2) {
        m_gender->setItemText(0, Lang::t("gender_male"));
        m_gender->setItemText(1, Lang::t("gender_female"));
    }
    if (m_lblHeight)      m_lblHeight->setText(Lang::t("body_height") + ":");
    if (m_lblFoot)        m_lblFoot->setText(Lang::t("foot_length") + ":");

    if (m_lblUpperArm)    m_lblUpperArm->setText(Lang::t("body_upper_arm") + ":");
    if (m_lblForearm)     m_lblForearm->setText(Lang::t("body_forearm") + ":");
    if (m_lblHand)        m_lblHand->setText(Lang::t("body_hand") + ":");
    if (m_lblThigh)       m_lblThigh->setText(Lang::t("body_thigh") + ":");
    if (m_lblShank)       m_lblShank->setText(Lang::t("body_shank") + ":");

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
        m_poseHint->setText(Lang::t(isNPosePhase() ? "npose_hint" : "tpose_hint"));
    }
    if (m_btnCalibBegin)  m_btnCalibBegin->setText(Lang::t("start_calib"));
    if (m_chkSensorCheck) m_chkSensorCheck->setText(Lang::t("asl_check_label"));
    if (m_readyTitle)     m_readyTitle->setText(Lang::t("ready_title"));
    if (m_readySummary)   m_readySummary->setText(Lang::t("ready_summary")
                            .arg(Lang::t("finish")));
    if (m_btnFinish)      m_btnFinish->setText(Lang::t("finish"));
    if (m_btnBack)        m_btnBack->setText(Lang::t("back"));
    if (m_btnNext)        m_btnNext->setText(Lang::t("continue"));

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

    m_btnBack->setEnabled(!calibBusy());

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

    const ConnStatus s = m_rx->status();
    const bool inFlight = (s == ConnStatus::Scanning ||
                           s == ConnStatus::Connecting ||
                           s == ConnStatus::Streaming);
    if (inFlight) {
        testLog("[wizard] Connect suit ignored — already "
                + std::string(connStatusName(s)), m_test);
        return;
    }

    m_suitBtnCooldown = true;
    m_btnConnectSuit->setEnabled(false);
    QTimer::singleShot(1200, this, [this]() {
        m_suitBtnCooldown = false;
        onStatusTick();
    });
    testLog("[wizard] Connect suit clicked", m_test);

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

    m_btnConnectGloves->setEnabled(false);
    QTimer::singleShot(800, this, [this]() {
        if (m_btnConnectGloves) m_btnConnectGloves->setEnabled(true);
    });
    const bool ok = m_rx->connectGloves();
    const char* key = ok ? "gloves_ready"
                   : (m_rx->glovesCoreReady() ? "gloves_no_device"
                                              : "gloves_no_core");

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
        m_result.gender = (m_gender && m_gender->currentIndex() == 1)
                              ? fox::body::GenderFemale : fox::body::GenderMale;

        m_result.armSpanCm        = 0.0;
        m_result.legLengthCm      = 0.0;
        m_result.upperArmCm       = m_upperArm ? m_upperArm->value() : 0.0;
        m_result.forearmCm        = m_forearm  ? m_forearm->value()  : 0.0;
        m_result.handCm           = m_hand     ? m_hand->value()     : 0.0;
        m_result.thighCm          = m_thigh    ? m_thigh->value()    : 0.0;
        m_result.shankCm          = m_shank    ? m_shank->value()    : 0.0;

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

    if (m_connBadge) setBadge(m_connBadge, Lang::t(key), streaming);

    if (m_suitDot)  paintDot(m_suitDot, streaming ? "#2EC25A" : "#C03838");
    if (m_suitText) m_suitText->setText(Lang::t(key));
    if (m_btnConnectSuit) {

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

        abortCalibration();
        if (m_calibStatus) m_calibStatus->setText(Lang::t("waiting_for_suit"));
    }
}

void NewSessionWizard::abortCalibration()
{
    ++m_settleGen;
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

    for (int i = 0; i < kXsensSegmentCount; ++i) {
        m_accAccumT[i]    = QVector3D(0, 0, 0);
        m_gyrAccumT[i]    = QVector3D(0, 0, 0);
        m_magAccumT[i]    = QVector3D(0, 0, 0);
        m_accMagAccumT[i] = 0.0;
        m_magMagAccumT[i] = 0.0;
        m_accumCountT[i]  = 0;
        m_accAccumN[i]    = QVector3D(0, 0, 0);
        m_gyrAccumN[i]    = QVector3D(0, 0, 0);
        m_magAccumN[i]    = QVector3D(0, 0, 0);
        m_accMagAccumN[i] = 0.0;
        m_magMagAccumN[i] = 0.0;
        m_accumCountN[i]  = 0;
    }

    for (int j = 0; j < 20; ++j) {
        m_fingerAccumR[j] = 0.0;
        m_fingerAccumL[j] = 0.0;
    }
    m_fingerAccumCount = 0;

    m_phase = CalibPhase::Idle;
    logCalibPhaseTransition("aborted");
    refreshPoseImage();
    if (m_countLabel) m_countLabel->setText("—");
}

void NewSessionWizard::onCalibrationBegin()
{
    if (!m_rx || m_rx->status() != ConnStatus::Streaming) return;

    ++m_settleGen;

    m_result.poseKind = "npose";

    m_samples.clear();
    m_goodSamples = 0;
    m_lastSampleCtr = -1;
    m_havePrev = false;
    m_countdownBar->setValue(0);
    if (m_readyBar) {
        m_readyBar->setRange(0, kCalibrationSamples);
        m_readyBar->setValue(0);
        m_readyBar->setFormat("%v / %m");
    }
    m_btnCalibBegin->setEnabled(false);

    for (int i = 0; i < kXsensSegmentCount; ++i) {
        m_accAccumT[i]    = QVector3D(0, 0, 0);
        m_gyrAccumT[i]    = QVector3D(0, 0, 0);
        m_magAccumT[i]    = QVector3D(0, 0, 0);
        m_accMagAccumT[i] = 0.0;
        m_magMagAccumT[i] = 0.0;
        m_accumCountT[i]  = 0;
        m_accAccumN[i]    = QVector3D(0, 0, 0);
        m_gyrAccumN[i]    = QVector3D(0, 0, 0);
        m_magAccumN[i]    = QVector3D(0, 0, 0);
        m_accMagAccumN[i] = 0.0;
        m_magMagAccumN[i] = 0.0;
        m_accumCountN[i]  = 0;
        m_gyrSqAccumT[i]  = QVector3D(0, 0, 0);
        m_gyrSqAccumN[i]  = QVector3D(0, 0, 0);
        for (int k = 0; k < 6; ++k) {
            m_magOuterAccumT[i][k] = 0.0;
            m_magOuterAccumN[i][k] = 0.0;
        }
    }

    for (int j = 0; j < 20; ++j) {
        m_fingerAccumR[j] = 0.0;
        m_fingerAccumL[j] = 0.0;
    }
    m_fingerAccumCount = 0;

    for (auto& buf : m_imuBuf) {
        buf.acc.clear();
        buf.gyr.clear();
        buf.epochCalibration  = RawImuBuf::Win{-1, -1};
        buf.epochLeftArm      = RawImuBuf::Win{-1, -1};
        buf.epochRightArm     = RawImuBuf::Win{-1, -1};
        buf.epochLeftLeg      = RawImuBuf::Win{-1, -1};
        buf.epochRightLeg     = RawImuBuf::Win{-1, -1};
    }
    {
        const double nativeHz = (m_rx ? m_rx->expectedRate() : 240.0);
        m_aslResStep = std::max(1.0, (nativeHz > 1.0 ? nativeHz : 240.0) / 60.0);
    }
    m_aslResNextOut = 0.0;
    m_aslResInIdx   = -1;
    m_aslHavePrev   = false;
    m_aslOutCount   = 0;
    m_aslPrevAcc.fill(QVector3D(0, 0, 0));
    m_aslPrevGyr.fill(QVector3D(0, 0, 0));
    m_moveStage     = 0;
    m_moveStageStartIdx = 0;

    m_accAccum    = &m_accAccumT;
    m_gyrAccum    = &m_gyrAccumT;
    m_magAccum    = &m_magAccumT;
    m_accMagAccum = &m_accMagAccumT;
    m_magMagAccum = &m_magMagAccumT;
    m_accumCount  = &m_accumCountT;
    m_gyrSqAccum    = &m_gyrSqAccumT;
    m_magOuterAccum = &m_magOuterAccumT;

    m_rx->resetS2sAlignment();
    m_rx->resetFusion();

    m_phase = CalibPhase::PrepT;
    m_phaseStartMs = 0;
    refreshPoseImage();
    m_countTicksLeft = kCountdownSeconds * 10;
    m_countLabel->setText(QString::number(kCountdownSeconds));
    if (m_calibStatus)
        m_calibStatus->setText(Lang::t("calib_t_prepare"));
    if (m_poseHint)
        m_poseHint->setText(Lang::t("tpose_hint"));
    testLog("[calib] double-pose sequence started, PrepT "
            "(Madgwick re-init scheduled)", m_test);
    logCalibPhaseTransition("countdown begin");
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
        } else if (m_phase == CalibPhase::PrepMove) {
            m_phase = CalibPhase::CaptureMove;
            m_moveStageStartIdx = m_aslOutCount;
            static const char* kCap[5] = {
                "calib_walk_capture", "calib_larm_capture", "calib_rarm_capture",
                "calib_lleg_capture", "calib_rleg_capture" };
            const int s = (m_moveStage < 0) ? 0 : (m_moveStage > 4 ? 4 : m_moveStage);
            if (m_calibStatus) m_calibStatus->setText(Lang::t(kCap[s]));
        }

        m_phaseStartMs = QDateTime::currentMSecsSinceEpoch();
        logCalibPhaseTransition("capture begin");
        m_captureTimer.start();
    }
}

namespace {
constexpr double kAslWalkCaptureSec  = 6.0;
constexpr double kAslRaiseCaptureSec = 5.0;

NewSessionWizard::RawImuBuf::Win detectRaiseWindow(
    const std::vector<QVector3D>& gyr, int from, int to)
{
    NewSessionWizard::RawImuBuf::Win w{ from, to };
    const int n = to - from;
    if (from < 0 || to > int(gyr.size()) || n < 12) return w;

    std::vector<double> g(n);
    double peak = 0.0;
    for (int i = 0; i < n; ++i) {
        const QVector3D& v = gyr[from + i];
        const double m = std::sqrt(double(v.x()) * v.x() + double(v.y()) * v.y()
                                 + double(v.z()) * v.z());
        g[i] = m;
        if (m > peak) peak = m;
    }
    constexpr double kFs = 60.0;
    const int pfx = std::min(n, std::max(5, int(0.5 * kFs)));
    double mu = 0.0;
    for (int i = 0; i < pfx; ++i) mu += g[i];
    mu /= double(pfx);
    double var = 0.0;
    for (int i = 0; i < pfx; ++i) { const double d = g[i] - mu; var += d * d; }
    const double sd  = std::sqrt(std::max(1e-12, var / double(pfx)));
    const double thr = std::max(mu + 3.0 * sd, mu + 0.15 * (peak - mu));

    int first = -1, last = -1;
    for (int i = 0; i < n; ++i)
        if (g[i] > thr) { if (first < 0) first = i; last = i; }
    if (first < 0) return w;

    const int pad = int(0.05 * kFs);
    w.start = from + std::max(0, first - pad);
    w.end   = from + std::min(n, last + 1 + pad);
    return w;
}
}

void NewSessionWizard::onCaptureTick()
{
    const SuitPose fr = m_rx->snapshot();
    if (qint64(fr.sampleCounter) == m_lastSampleCtr) return;
    m_lastSampleCtr = fr.sampleCounter;

    std::array<Quat, kXsensSegmentCount> snap{};
    for (int i = 0; i < kXsensSegmentCount; ++i) snap[i] = fr.quat[i];

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

    // §XIII/§XVII порог неподвижности при сборе калибровочной позы (движение сегмента < 0.025 рад)
    constexpr double kStillRad = 0.025;
    const bool still = second < kStillRad;

    const bool aslCapturing = m_liveSpcEnabled && m_doSensorCheck
        && (m_phase == CalibPhase::CaptureT
         || m_phase == CalibPhase::CaptureN
         || m_phase == CalibPhase::CaptureMove);
    if (aslCapturing) {
        ++m_aslResInIdx;
        const int n = m_aslResInIdx;
        const bool calibEpoch = (m_phase == CalibPhase::CaptureT
                              || m_phase == CalibPhase::CaptureN
                              || (m_phase == CalibPhase::CaptureMove && m_moveStage == 0));
        while (m_aslResNextOut <= double(n) + 1e-9) {
            double frac = (n >= 1) ? (m_aslResNextOut - double(n - 1)) : 0.0;
            if (frac < 0.0) frac = 0.0; else if (frac > 1.0) frac = 1.0;
            const float f = float(frac);
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                const QVector3D pA = m_aslHavePrev ? m_aslPrevAcc[i] : fr.accSensor[i];
                const QVector3D pG = m_aslHavePrev ? m_aslPrevGyr[i] : fr.gyrSensor[i];
                const QVector3D cA = fr.segValid[i] ? fr.accSensor[i] : pA;
                const QVector3D cG = fr.segValid[i] ? fr.gyrSensor[i] : pG;
                m_imuBuf[i].acc.push_back(pA * (1.0f - f) + cA * f);
                m_imuBuf[i].gyr.push_back(pG * (1.0f - f) + cG * f);
                if (calibEpoch) {
                    if (m_imuBuf[i].epochCalibration.start < 0)
                        m_imuBuf[i].epochCalibration.start = m_aslOutCount;
                    m_imuBuf[i].epochCalibration.end = m_aslOutCount + 1;
                }
            }
            ++m_aslOutCount;
            m_aslResNextOut += m_aslResStep;
        }
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            if (fr.segValid[i]) {
                m_aslPrevAcc[i] = fr.accSensor[i];
                m_aslPrevGyr[i] = fr.gyrSensor[i];
            }
        }
        m_aslHavePrev = true;
    }

    if (m_phase == CalibPhase::CaptureMove) {
        const qint64 elapsedMs = (m_phaseStartMs > 0)
            ? (QDateTime::currentMSecsSinceEpoch() - m_phaseStartMs) : 0;
        const double stageSec = (m_moveStage == 0)
            ? kAslWalkCaptureSec : kAslRaiseCaptureSec;
        const qint64 budgetMs = qint64(stageSec * 1000.0);
        if (m_readyBar) {
            m_readyBar->setRange(0, int(budgetMs));
            m_readyBar->setValue(int(std::min<qint64>(elapsedMs, budgetMs)));
            m_readyBar->setFormat("%p%");
        }
        if (m_stillLabel) {
            m_stillLabel->setStyleSheet("color:#2EC25A; font-weight:700;");
            m_stillLabel->setText(Lang::t("calib_move_hint"));
        }
        if (elapsedMs < budgetMs) return;

        m_captureTimer.stop();
        if (m_moveStage >= 1) {
            static const int kRefSeg[5] = { -1, 12, 8, 19, 15 };
            const int ref = kRefSeg[m_moveStage];
            const RawImuBuf::Win wWin =
                detectRaiseWindow(m_imuBuf[ref].gyr, m_moveStageStartIdx, m_aslOutCount);
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                switch (m_moveStage) {
                    case 1: m_imuBuf[i].epochLeftArm  = wWin; break;
                    case 2: m_imuBuf[i].epochRightArm = wWin; break;
                    case 3: m_imuBuf[i].epochLeftLeg  = wWin; break;
                    case 4: m_imuBuf[i].epochRightLeg = wWin; break;
                    default: break;
                }
            }
            if (m_test) {
                std::cout << "[FoxSPC] move stage " << m_moveStage
                          << " window=[" << wWin.start << "," << wWin.end << ") of "
                          << m_aslOutCount << " (ref seg " << ref << ")\n";
                std::cout.flush();
            }
        }
        ++m_moveStage;
        if (m_moveStage <= 4) beginMoveStage();
        else                  finishCalibrationAsl();
        return;
    }

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
            if (m_magMagAccum) {
                (*m_magMagAccum)[i] += fr.magSensor[i].length();
            }
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

    {
        const qint64 elapsedMs = (m_phaseStartMs > 0)
            ? (QDateTime::currentMSecsSinceEpoch() - m_phaseStartMs) : 0;
        const qint64 stageBudgetMs = qint64(fox::body::kCalibStageDurationSec * 1000.0);
        const bool   timeBudgetMet = elapsedMs >= stageBudgetMs;
        const bool   sampleFloor   = m_goodSamples >= 50;
        const bool   countReached  = m_goodSamples >= kCalibrationSamples;
        if (!(countReached || (timeBudgetMet && sampleFloor))) return;
    }

    if (m_phase == CalibPhase::CaptureT) {
        m_captureTimer.stop();
        testLog("[calib §174.2] T-pose capture complete, samples="
                + std::to_string(m_samples.size())
                + " — Markley average → tposeReference", m_test);

        if (!m_samples.empty()) {

            std::array<std::vector<Quat>, kXsensSegmentCount> sampT;
            for (auto& v : sampT) v.reserve(m_samples.size());
            for (const auto& s : m_samples)
                for (int i = 0; i < kXsensSegmentCount; ++i) sampT[i].push_back(s[i]);
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                if (sampT[i].size() < 4) {
                    m_result.tposeReference[i] = Quat(1, 0, 0, 0);
                    continue;
                }
                m_result.tposeReference[i] = fox::quat_avg_markley(sampT[i]);
            }
            m_result.tposePelvisPos = QVector3D(0.0f, 0.0f, 0.0f);
            m_result.tposeCaptured = true;
        }

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

        m_accAccum      = &m_accAccumN;
        m_gyrAccum      = &m_gyrAccumN;
        m_magAccum      = &m_magAccumN;
        m_accMagAccum   = &m_accMagAccumN;
        m_magMagAccum   = &m_magMagAccumN;
        m_accumCount    = &m_accumCountN;
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
        if (m_readyBar) {
            m_readyBar->setValue(0);
            m_readyBar->setRange(0, kCalibrationSamples);
            m_readyBar->setFormat("%v / %m");
        }
        m_goodSamples = 0;
        m_havePrev    = false;
        m_samples.clear();
        m_countLabel->setText(QString::number(kCountdownSeconds));
        logCalibPhaseTransition("T-pose committed");
        m_countTimer.start();
        updateNavButtons();
        return;
    }

    if (m_phase != CalibPhase::CaptureN) return;
    m_captureTimer.stop();
    m_calibComplete = true;
    m_phase = CalibPhase::Settle;
    if (m_calibStatus) m_calibStatus->setText(Lang::t("calib_ok"));
    testLog("[calib] N-pose capture complete, solving q_align per §174.4…", m_test);
    logCalibPhaseTransition("N-pose captured, solving q_align");

    std::array<std::vector<Quat>, kXsensSegmentCount> sampN;
    for (auto& v : sampN) v.reserve(m_samples.size());
    for (const auto& s : m_samples) {
        for (int i = 0; i < kXsensSegmentCount; ++i) sampN[i].push_back(s[i]);
    }

    std::array<Quat,      kXsensSegmentCount> s2s{};
    std::array<double,    kXsensSegmentCount> residDeg{};
    std::array<int,       kXsensSegmentCount> qualityBand{};
    std::array<double,    kXsensSegmentCount> magMagn{};
    std::array<double,    kXsensSegmentCount> accMagn{};
    std::array<QVector3D, kXsensSegmentCount> gyrBias{};
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        s2s[i]         = Quat(1, 0, 0, 0);
        residDeg[i]    = 99.0;
        qualityBand[i] = fox::body::kCalibQualityInvalid;
        magMagn[i]     = 1.0;
        accMagn[i]     = 1.0;
        gyrBias[i]     = QVector3D(0, 0, 0);
    }

    auto clampBias = [](float v) {
        const float kMax = float(fox::body::kEstimator.gyrBiasStdMaxDeg);
        return v > kMax ? kMax : (v < -kMax ? -kMax : v);
    };

    for (int i = 0; i < kXsensSegmentCount; ++i)
        m_result.calibReference[i] = Quat(1, 0, 0, 0);

    std::array<bool, kXsensSegmentCount> tposeValid{};
    for (int i = 0; i < kXsensSegmentCount; ++i) {

        const Quat& q = m_result.tposeReference[i];
        const double dist2 = (q.w - 1.0) * (q.w - 1.0)
                           +  q.x * q.x +  q.y * q.y +  q.z * q.z;
        tposeValid[i] = (dist2 > 1e-12);
    }

    constexpr std::size_t kCalibMinSamplesPerSeg = 60;
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        if (!fox::body::kSensorPresent[i]) continue;
        if (sampN[i].size() < kCalibMinSamplesPerSeg) continue;

        const Quat qAvgN  = fox::quat_avg_markley(sampN[i]);
        m_result.calibReference[i] = qAvgN;
        const Quat& q_bs  = fox::body::kSensorToBone[i].q_bs;
        const Quat& qRefN = fox::body::kRefQuatN[i];
        const Quat& qRefT = fox::body::kRefQuatT[i];

        // §174.4/§1682/§2564 выравнивание датчик->сегмент q_align (усреднение поз Маркли §1824).
        // КОНВЕНЦИЯ ДВИЖКА: q_align применяется ПРЕД-поворотом измерений (s2sInv=conj(q_align), стр.~5966)
        // и в связке с m_defAng в FK (oriented=fused⊗m_defAng, стр.~3066) — это согласованная декомпозиция,
        // поэтому здесь conj(q_bs), а не постмультипликативная запись §1682. НЕ менять без сверки невязок калибровки.
        const Quat qAlignN = quat_mult(
            quat_mult(qRefN, qAvgN.conj()),
            q_bs.conj()).normalized();

        Quat qAlign = qAlignN;
        double residDegN = 0.0;
        double residDegT = 0.0;
        if (tposeValid[i]) {
            const Quat& qAvgT = m_result.tposeReference[i];
            const Quat qAlignT = quat_mult(
                quat_mult(qRefT, qAvgT.conj()),
                q_bs.conj()).normalized();
            std::vector<Quat> v{qAlignN, qAlignT};
            qAlign = fox::quat_avg_markley(v);

            const Quat qResidN = quat_mult(qAlignN, q_bs).normalized();
            double absN = std::abs(qResidN.w);
            if (absN > 1.0) absN = 1.0;
            residDegN = 2.0 * std::acos(absN) * 180.0 / M_PI;
            const Quat qResidT = quat_mult(qAlignT, q_bs).normalized();
            double absT = std::abs(qResidT.w);
            if (absT > 1.0) absT = 1.0;
            residDegT = 2.0 * std::acos(absT) * 180.0 / M_PI;
        }
        s2s[i] = qAlign;

        const Quat qResid = quat_mult(qAlign, q_bs).normalized();
        double absw = std::abs(qResid.w);
        if (absw > 1.0) absw = 1.0;
        residDeg[i]    = 2.0 * std::acos(absw) * 180.0 / M_PI;
        qualityBand[i] = fox::body::calibrationQuality(residDeg[i]);

        if (m_test && fox::pose_solver::g_glovesFlag().load(
                std::memory_order_relaxed) && tposeValid[i]) {
            std::cout << "[calib §174.6] " << std::left << std::setw(14)
                      << kSegmentNames[i] << std::right
                      << "  N-only=" << std::fixed << std::setprecision(2)
                      << residDegN << "°"
                      << "  T-only=" << residDegT << "°"
                      << "  N+T="    << residDeg[i] << "°  (Markley)\n";
        }

        const int cT = m_accumCountT[i];
        const int cN = m_accumCountN[i];
        if (cT >= 10 && cN >= 10) {
            const int    cTot   = cT + cN;
            const double invTot = 1.0 / double(cTot);
            accMagn[i] = (m_accMagAccumT[i] + m_accMagAccumN[i]) * invTot;
            magMagn[i] = (m_magMagAccumT[i] + m_magMagAccumN[i]) * invTot;
            const QVector3D meanG_all = (m_gyrAccumT[i] + m_gyrAccumN[i])
                                        * float(invTot);
            gyrBias[i] = QVector3D(clampBias(meanG_all.x()),
                                   clampBias(meanG_all.y()),
                                   clampBias(meanG_all.z()));
        }

        if (m_test) {
            std::cout << "[calib §174.4] " << std::left << std::setw(14)
                      << kSegmentNames[i] << std::right
                      << "  residual=" << std::fixed << std::setprecision(2)
                      << residDeg[i] << "°"
                      << "  quality=" << qualityBand[i] << "/5\n";
        }
    }

    m_rx->setAccNormalisation(accMagn);
    m_rx->setMagNormalisation(magMagn);
    m_rx->setGyroBias(gyrBias);
    m_rx->setS2sAlignment(s2s);

    {
        double sumResid = 0.0;
        int    nResid   = 0;
        QStringList problemList;
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            if (!fox::body::kSensorPresent[i]) continue;
            sumResid += residDeg[i];
            ++nResid;
            if (qualityBand[i] <= fox::body::kCalibQualityPoor) {
                problemList << QString("%1(%2°)")
                                .arg(QString::fromUtf8(kSegmentNames[i]))
                                .arg(residDeg[i], 0, 'f', 1);
            }
        }
        const double avgResid = (nResid > 0) ? sumResid / double(nResid) : 99.0;
        const int    band     = fox::body::calibrationQuality(avgResid);
        const char*  label    = fox::body::calibrationQualityLabel(band);

        if (m_calibQuality) {
            QString qual = QString("Quality: %1/5 %2 (residual %3°)")
                               .arg(band)
                               .arg(QString::fromUtf8(label))
                               .arg(avgResid, 0, 'f', 1);
            if (!problemList.isEmpty())
                qual += QString(" — issues: %1").arg(problemList.join(", "));
            m_calibQuality->setText(qual);
            m_calibQuality->setStyleSheet(
                band >= fox::body::kCalibQualityGood
                    ? "color:#2EC25A; font-weight:700;"
                : band >= fox::body::kCalibQualityAdequate
                    ? "color:#E0A030; font-weight:700;"
                    : "color:#E04040; font-weight:700;");
        }
    }

    if (m_test) {
        std::cout << std::fixed << std::setprecision(5);
        std::cout << "[calib] §174.4 q_align stored (T+N statistics for acc/mag/gyr):\n";
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            if (!fox::body::kSensorPresent[i]) continue;
            std::cout << "        " << std::left << std::setw(14)
                      << kSegmentNames[i] << std::right
                      << "  acc_magn=" << accMagn[i]
                      << "  mag_magn=" << magMagn[i]
                      << "  gyr_bias=(" << gyrBias[i].x() << ","
                                        << gyrBias[i].y() << ","
                                        << gyrBias[i].z() << ") rad/s\n";
        }
        std::cout.flush();
    }

    if (m_doSensorCheck && m_liveSpcEnabled && g_placementClf().ready) {
        m_moveStage = 0;
        beginMoveStage();
        return;
    }
    finishCalibrationAsl();
}

void NewSessionWizard::finishCalibrationAsl()
{
    if (m_placementInfo) {
        if (!m_doSensorCheck) {
            m_placementInfo->setText(Lang::t("asl_skipped"));
            m_placementInfo->setStyleSheet("color:#9B9B9B;");
        } else if (!m_liveSpcEnabled || !g_placementClf().ready) {
            m_placementInfo->setText(Lang::t("asl_loading_failed"));
            m_placementInfo->setStyleSheet("color:#9B9B9B;");
        } else {
            const spc::PlacementReport rep = spc::analyzePlacement(
                g_placementClf(), m_imuBuf, m_test);
            QString msg;
            QString style = "color:#9B9B9B;";
            if (!rep.haveData) {
                msg   = Lang::t("asl_no_data");
            // §XXVII тревога SPC: средняя макс. вероятность класса < 0.35 -> неуверенное размещение датчиков
            } else if (rep.suitUncertaintyAlarm || rep.avgMaxP < 0.35f) {
                msg   = Lang::t("asl_low_confidence");
            } else if (rep.mismatches.isEmpty()) {
                msg   = Lang::t("asl_ok").arg(rep.verified).arg(rep.total);
                style = "color:#2EC25A; font-weight:700;";
            } else {
                const QString first = rep.mismatches.first();
                const QStringList parts = first.split(QRegularExpression("[ →(=)]"),
                                                       Qt::SkipEmptyParts);
                QString a = parts.value(0);
                QString b = parts.value(1);
                QString p = parts.value(2);
                msg   = Lang::t("asl_mismatch").arg(a).arg(b).arg(p);
                if (rep.mismatches.size() > 1) {
                    msg += QString(" (+%1)").arg(rep.mismatches.size() - 1);
                }
            }
            m_placementInfo->setText(msg);
            m_placementInfo->setStyleSheet(style);
        }
    }

    if (m_test) {
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "[calib §174.4] calibReference (per-segment N-pose Markley mean):\n";
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            if (!fox::body::kSensorPresent[i]) continue;
            const Quat& q = m_result.calibReference[i];
            std::cout << "        " << std::left << std::setw(14)
                      << kSegmentNames[i] << std::right
                      << " (" << q.w << "," << q.x << ","
                      << q.y << "," << q.z << ")\n";
        }
        std::cout.flush();
    }

    m_phase = CalibPhase::Done;
    logCalibPhaseTransition("calibration complete");
    this->goNext();
}

void NewSessionWizard::beginMoveStage()
{
    static const char* kPrep[5] = {
        "calib_walk_prepare", "calib_larm_prepare", "calib_rarm_prepare",
        "calib_lleg_prepare", "calib_rleg_prepare" };
    const int s = (m_moveStage < 0) ? 0 : (m_moveStage > 4 ? 4 : m_moveStage);

    m_phase = CalibPhase::PrepMove;
    refreshPoseImage();
    if (m_calibStatus) m_calibStatus->setText(Lang::t(kPrep[s]));
    if (m_poseHint)    m_poseHint->setText(Lang::t("calib_move_hint"));

    m_countTicksLeft = kCountdownSeconds * 10;
    if (m_countdownBar) m_countdownBar->setValue(0);
    if (m_countLabel)   m_countLabel->setText(QString::number(kCountdownSeconds));
    if (m_readyBar) {
        m_readyBar->setValue(0);
        m_readyBar->setFormat("%p%");
    }
    logCalibPhaseTransition("move stage prep");
    m_countTimer.start();
    updateNavButtons();
}

void NewSessionWizard::closeEvent(QCloseEvent* e)
{
    ++m_settleGen;
    m_countTimer.stop();
    m_captureTimer.stop();
    m_statusTimer.stop();

    if (m_phase != CalibPhase::Idle && m_phase != CalibPhase::Done) {
        abortCalibration();
    }
    QDialog::closeEvent(e);
}

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

    grid->addWidget(makeCard(SEG_Head),    0, 0, 1, 2);
    grid->addWidget(makeCard(SEG_T8),      1, 0, 1, 2);
    grid->addWidget(makeCard(SEG_Pelvis),  2, 0, 1, 2);

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

    m_fingersBox = new QWidget(this);

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

        fGrid->addWidget(makeFingerCard(5 + f, kFingerKey[f]), f, 0);
        fGrid->addWidget(makeFingerCard(f,     kFingerKey[f]), f, 1);
    }
    auto* fLay = new QVBoxLayout(m_fingersBox);
    fLay->setContentsMargins(0, 6, 0, 0);
    fLay->setSpacing(0);
    fLay->addWidget(fHeader);
    fLay->addWidget(fSub);
    fLay->addSpacing(6);
    fLay->addLayout(fGrid);

    m_lblSession = new QLabel(this);
    m_lblSession->setVisible(false);

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

    m_bodyBox->setVisible(false);

    auto* inner = new QWidget(this);
    inner->setObjectName("sidePanelInner");
    auto* lay = new QVBoxLayout(inner);
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

    auto* scroll = new QScrollArea(this);
    scroll->setWidget(inner);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll->setStyleSheet(
        "QScrollArea { background:transparent; border:none; }"
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
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(0);
    outer->addWidget(scroll);

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

void SensorIndicatorsPanel::setSuitLive(bool live, const QString& )
{
    paintDot(m_lblSuit, live ? "#2EC25A" : "#C03838");
    for (QLabel* lab : findChildren<QLabel*>()) {
        if (lab->property("isSuitLabel").toBool()) {
            lab->setText(live ? Lang::t("suit_connected")
                              : Lang::t("suit_disconnected"));
        }
    }
}

void SensorIndicatorsPanel::clearLiveDots()
{
    for (auto& t : m_trackers) if (t.dot) setDot(t.dot, false);
    for (auto& g : m_fingers)  if (g.dot) setDot(g.dot, false);
}

void SensorIndicatorsPanel::setFps(double hz) { m_lblFps->setText(QString("%1 Hz").arg(hz, 0, 'f', 1)); }

void SensorIndicatorsPanel::setSessionRunning(bool running)
{
    m_running = running;
    if (m_lblSession)
        m_lblSession->setText(running ? Lang::t("session_running")
                                       : Lang::t("session_paused"));

    if (m_btnReset)  { m_btnReset->setEnabled(running);  m_btnReset->setText(Lang::t("reset_coords")); }
    if (m_btnFreeze) { m_btnFreeze->setEnabled(running); m_btnFreeze->setText(Lang::t(
        m_frozen ? "unfreeze_coords" : "freeze_coords")); }
}

void SensorIndicatorsPanel::updateFromPose(const SuitPose& f)
{

    using clk = std::chrono::steady_clock;
    const double nowSec = std::chrono::duration<double>(
                              clk::now().time_since_epoch()).count();
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        if (!m_trackers[i].dot) continue;
        const double age = nowSec - f.segLastT[i];
        const bool fresh = f.segValid[i] && f.segLastT[i] > 0.0 && age < 2.0;
        setDot(m_trackers[i].dot, fresh);
    }

    const bool showFingers = m_useGloves && f.hasGloves;
    if (m_fingersBox && m_fingersBox->isVisible() != showFingers)
        m_fingersBox->setVisible(showFingers);
    if (showFingers) {
        for (int i = 0; i < 10; ++i)
            if (m_fingers[i].dot) setDot(m_fingers[i].dot, true);
    } else {

        for (int i = 0; i < 10; ++i)
            if (m_fingers[i].dot) setDot(m_fingers[i].dot, false);
    }

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
    m_loco.reset();
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

            const Quat dq = quat_mult(orient[i], m_prevQ[i].inv()).normalized();
            const double w = std::abs(dq.w) > 1.0 ? 1.0 : std::abs(dq.w);
            const double angRad = 2.0 * std::acos(w);
            const double angVel = angRad * 180.0 / M_PI / dt;

            const double alpha = rateAdjustAlpha(0.30, dt);
            m_angVelLP[i] = (1.0 - alpha) * m_angVelLP[i] + alpha * angVel;

            const double angAcc = std::abs(angVel - m_angVelPrev[i]) / dt;
            m_angVelPrev[i] = angVel;
            m_dbgAngAcc[i]  = angAcc;

            const bool isWrist  = (i == SEG_RHand || i == SEG_LHand);
            const bool isFootSeg = (i == SEG_RFoot || i == SEG_LFoot
                                  || i == SEG_RToe || i == SEG_LToe);
            const double slowThresh   = isWrist ? 0.6  : (isFootSeg ? 0.6  : 0.8);
            const double steadyThresh = isWrist ? 12.0 : (isFootSeg ? 14.0 : 20.0);
            const double lockTime     = isWrist ? 0.20 : (isFootSeg ? 0.25 : 0.50);

            const bool slow = m_angVelLP[i] < slowThresh;
            const bool steady = angAcc < steadyThresh;
            const bool stillFrame = slow && steady;

            if (stillFrame) {
                m_stillTicks[i] += dt;
                if (m_stillTicks[i] > lockTime && !m_locked[i]) {

                    m_locked[i]   = true;
                    m_lockQuat[i] = m_prevQ[i];

                    m_lockBlend[i] = 0.0;
                }
            } else {

                m_stillTicks[i] = 0.0;
                m_locked[i] = false;
            }

            if (m_locked[i]) {

                if (m_lockBlend[i] < 1.0) {
                    filtered[i] = nlerpQ(orient[i], m_lockQuat[i], m_lockBlend[i]);

                    m_lockBlend[i] = std::min(1.0, m_lockBlend[i] + 0.15 * (std::min(dt, 2.0 / 90.0) * 90.0));
                } else {
                    filtered[i] = m_lockQuat[i];
                }
                m_unlockBlend[i] = 0.30;
            } else if (m_unlockBlend[i] < 1.0) {
                filtered[i] = nlerpQ(m_outPrevQ[i], orient[i], m_unlockBlend[i]);
                m_unlockBlend[i] = std::min(1.0, m_unlockBlend[i] + 0.15 * (std::min(dt, 2.0 / 90.0) * 90.0));
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

                if (allCalm) m_calmSeconds[i] += dt;
                else         m_calmSeconds[i] = 0.0;

                const Quat dA_h = m_skel ? m_skel->defAngFor(i)        : Quat(1, 0, 0, 0);
                const Quat dA_f = m_skel ? m_skel->defAngFor(iForearm) : Quat(1, 0, 0, 0);
                const Quat hWorld = quat_mult(orient[i],        dA_h).normalized();
                const Quat fWorld = quat_mult(orient[iForearm], dA_f).normalized();

                if (m_locked[i]) {

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

            }

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

                    const double driftBudget = M_PI / 12.0;

                    if (allCalm && !crossLeg) {
                        auto smoothstep01 = [](double x){ x = std::clamp(x,0.0,1.0); return x*x*(3.0-2.0*x); };
                        const double budgetAttn = smoothstep01(
                                (driftBudget - driftAngle) / (driftBudget * 0.3));
                        if (budgetAttn > 0.0) {
                            const double alpha = std::min(1.0, dt / 3.0) * budgetAttn * gateAttn;
                            m_driftLocal[i] = nlerpQ(m_driftLocal[i], drift, alpha);
                        }
                    }

                    const Quat& dL = m_driftLocal[i];
                    const double dw = std::min(1.0, std::abs(dL.w));
                    if (2.0 * std::acos(dw) > 1e-4) {
                        Quat dLSwing, dLTwist;
                        swingTwistDecompose(dL, QVector3D(0.0f, 0.0f, 1.0f), dLSwing, dLTwist);
                        Quat dLTwistInv = dLTwist.inv();

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

    m_prevQ = orient;
    m_havePrevQ = true;

    {
        static constexpr double kSlewGain = 1.8;
        static constexpr double kBaseRate = 300.0;
        static constexpr double kDtCapFac = 1.6;
        static constexpr double kFcMin    = 2.0;
        static constexpr double kOeBeta   = 1.5;
        const double dtCap = std::min(dt, kDtCapFac * m_nomDt);
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            const Quat in = filtered[i];
            if (!m_haveCond) { m_condPrev[i] = in; continue; }

            const double fc    = kFcMin + kOeBeta * m_angVelLP[i];
            const double tau   = 1.0 / (2.0 * M_PI * fc);
            const double alpha = 1.0 / (1.0 + tau / dt);
            Quat sm = nlerpQ(m_condPrev[i], in, alpha);

            const double ang = quat_angle_deg(quat_mult(sm, m_condPrev[i].inv()));
            const double budgetDeg =
                std::max(kBaseRate * dtCap, kSlewGain * m_angVelLP[i] * dtCap);
            if (ang > budgetDeg && ang > 1e-6) {
                const double tSlew = budgetDeg / ang;
                sm = slerp_quat(m_condPrev[i], sm, tSlew);
                if (m_condVerbose) {
                    static std::array<int, kXsensSegmentCount> s_condTick{};
                    static int s_condGlobal = 0; ++s_condGlobal;
                    if (s_condGlobal - s_condTick[i] > 60) {
                        s_condTick[i] = s_condGlobal;
                        std::cout << "[cond] seg[" << i << "] slew ang="
                                  << std::fixed << std::setprecision(1) << ang
                                  << "° budget=" << budgetDeg << "° t="
                                  << std::setprecision(2) << tSlew << " |w|="
                                  << std::setprecision(1) << m_angVelLP[i] << "°/s dt="
                                  << std::setprecision(1) << (dt * 1000.0) << "ms\n";
                    }
                }
            }

            m_condPrev[i] = sm;
            filtered[i]   = sm;
        }
        m_haveCond = true;
    }

    m_orient = filtered;
    m_root   = root;

    if (now - m_lastPaintSec >= m_paintMinIntervalSec) {
        m_lastPaintSec = now;
        update();
    }
}

void MocapViewport::resetSceneOrigin()
{

    if (!m_skel) return;

    const Quat& qp = m_orient[SEG_Pelvis];

    const double fx = 1.0 - 2.0*(qp.y*qp.y + qp.z*qp.z);
    const double fy = 2.0*(qp.x*qp.y + qp.w*qp.z);
    const double yaw = std::atan2(fy, fx);
    m_sceneYaw = float(-yaw);

    QVector3D raw = m_lastRenderedPelvis - m_sceneShift;
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

void MocapViewport::paintGL()
{
    auto* gl = QOpenGLContext::currentContext()->functions();
    gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 proj;  proj.perspective(45.0f, float(width())/float(std::max(1, height())), 0.05f, 100.0f);

    const float yawR   = qDegreesToRadians(m_yaw);
    const float pitchR = qDegreesToRadians(m_pitch);
    QVector3D eye(m_dist * std::cos(pitchR) * std::sin(yawR),
                  m_dist * std::cos(pitchR) * std::cos(yawR),
                  m_dist * std::sin(pitchR) + 1.0f);
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
    glColor3f(1, 0.25f, 0.25f); glVertex3f(0,0,0); glVertex3f(0.3f,0,0);
    glColor3f(0.25f, 1, 0.25f); glVertex3f(0,0,0); glVertex3f(0,0.3f,0);
    glColor3f(0.25f, 0.4f, 1); glVertex3f(0,0,0); glVertex3f(0,0,0.3f);
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

    auto kp = m_skel->computeKeypoints(m_orient, m_root);

    {
        for (auto& p : kp) p += m_lastLocoOffset;
    }

    float minZ = kp[0].z();
    for (const auto& p : kp) if (p.z() < minZ) minZ = p.z();
    constexpr float kFloorClampShallowM = -0.10f;
    const bool shallowPenetration = (minZ < -0.02f && minZ >= kFloorClampShallowM);
    m_lastFloorClamp = shallowPenetration ? -minZ : 0.0f;
    if (shallowPenetration) {
        const QVector3D shift(0, 0, -minZ);
        for (auto& p : kp) p += shift;
    }

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

    m_lastRenderedPelvis = kp[SEG_Pelvis];

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

                kp[SEG_RHand] += shiftR;
                kp[SEG_LHand] += shiftL;
                if (kXsensKeypointCount >= 26) {
                    kp[24] += shiftR;
                    kp[25] += shiftL;
                }
            }
        }
    }

    m_lastRenderedKp = kp;

    glLineWidth(3.0f);
    glColor3f(1.0f, 0.48f, 0.10f);
    glBegin(GL_LINES);
    const auto& S = m_skel->startPts();
    const auto& E = m_skel->endPts();
    for (int s = 0; s < kXsensSegmentCountWithDummies; ++s) {

        if (m_haveGloves && (E[s] == 24 || E[s] == 25)) continue;
        const auto& a = kp[S[s]];
        const auto& b = kp[E[s]];
        glVertex3f(a.x(), a.y(), a.z());
        glVertex3f(b.x(), b.y(), b.z());
    }
    glEnd();

    glPointSize(6.0f);
    glColor3f(0.95f, 0.95f, 0.95f);
    glBegin(GL_POINTS);
    for (int i = 0; i < kXsensKeypointCount; ++i)
        glVertex3f(kp[i].x(), kp[i].y(), kp[i].z());
    glEnd();

    glPointSize(10.0f);
    glColor3f(1.0f, 0.62f, 0.18f);
    glBegin(GL_POINTS);
    glVertex3f(kp[0].x(), kp[0].y(), kp[0].z());
    glEnd();

    {

        std::array<Quat, kXsensSegmentCount> oriented{};
        for (int i = 0; i < kXsensSegmentCount; ++i)
            oriented[i] = quat_mult(m_orient[i], m_skel->defaultSegAngles()[i]);
        const auto global = m_skel->addDummySegments(oriented);

        constexpr float L = 0.075f;
        glLineWidth(1.6f);
        glBegin(GL_LINES);
        for (int s = 0; s < kXsensSegmentCountWithDummies; ++s) {

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

    if (!m_haveGloves) return;

    const QVector3D rWrist = kp[SEG_RHand];
    const QVector3D lWrist = kp[SEG_LHand];

    auto drawHand = [&](const std::array<QVector3D, kFingerSegmentsHand>& rel,
                        const QVector3D& wrist,
                        float cr, float cg, float cb,
                        float pr, float pg, float pb) {

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

    drawHand(m_rightHand, rWrist, 1.00f, 0.55f, 0.12f,  1.00f, 0.85f, 0.55f);
    drawHand(m_leftHand,  lWrist, 0.10f, 0.70f, 1.00f,  0.55f, 0.85f, 1.00f);
}

void MocapViewport::mousePressEvent(QMouseEvent* e) { m_lastMouse = e->pos(); }
void MocapViewport::mouseMoveEvent (QMouseEvent* e)
{
    const QPoint d = e->pos() - m_lastMouse;
    m_lastMouse = e->pos();
    if (e->buttons() & Qt::LeftButton) {
        m_yaw    = std::fmod(m_yaw + d.x() * 0.4f, 360.0f);
        if (m_yaw <    0.0f) m_yaw += 360.0f;
        if (m_yaw >= 360.0f) m_yaw -= 360.0f;
        m_pitch  = std::max(-85.0f, std::min(85.0f, m_pitch + d.y() * 0.3f));
        update();
    }
}
void MocapViewport::wheelEvent(QWheelEvent* e)
{
    m_dist = std::max(0.5f, std::min(10.0f, m_dist - e->angleDelta().y() * 0.002f));
    update();
}

static void dumpFirstFrameHex(const char* tag, const QByteArray& pkt)
{
    std::cout << "[stream first-frame hex] " << tag << " bytes="
              << pkt.size() << "\n";
    const int n = std::min(pkt.size(), qsizetype(24 + 64));
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
    qint64       lastWireDumpMs  = -1;
    static constexpr qint64 kWireDumpIntervalMs = 2000;
    qint64       lastEmitMs = -1;
    static constexpr qint64 kHandshakeIntervalMs = 1000;

    std::array<Quat, kXsensSegmentCount> baselineBodyQ{};
    std::array<Quat, kFingerSegmentsHand> baselineLeftGloveQ{};
    std::array<Quat, kFingerSegmentsHand> baselineRightGloveQ{};
    QVector3D    baselinePelvisPos{};
    bool         baselineCaptured = false;
    int          fingerBaselineSamples = 0;
    static constexpr int kFingerBaselineWindow = 30;

    std::array<Quat, kXsensSegmentCount>  prevWireBodyQ{};
    std::array<Quat, kFingerSegmentsHand> prevWireLeftQ{};
    std::array<Quat, kFingerSegmentsHand> prevWireRightQ{};
    void resetWireContinuity() {
        prevWireBodyQ.fill(Quat(1, 0, 0, 0));
        prevWireLeftQ.fill(Quat(1, 0, 0, 0));
        prevWireRightQ.fill(Quat(1, 0, 0, 0));
    }

    qint64 lastSendWarnMs = -1;
    qint64 lastMtuWarnMs  = -1;
    static constexpr int kMtuSoftLimit = 1472;
    void sendChecked(const QByteArray& pkt) {
        if (pkt.size() > kMtuSoftLimit) {
            const qint64 now = timer.elapsed();
            if (lastMtuWarnMs < 0 || (now - lastMtuWarnMs) >= 5000) {
                lastMtuWarnMs = now;
                std::cerr << "[stream] WARNING: UDP datagram size "
                          << pkt.size() << " bytes > " << kMtuSoftLimit
                          << " (IPv4 MTU minus 28-byte header). Consider"
                          << " enabling splitGloveDatagrams to avoid"
                          << " fragmentation/drop.\n";
            }
        }
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
        const quint8 fingerHdr  = cfg.useGloves ? 40 : 0;

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

            const QVector3D o = cfg.tposeOriginM[i];
            appendScaleSegment(payload, kBodyMvn[i], o.x(), o.y(), o.z());
        }
        if (cfg.useGloves) {
            for (int i = 0; i < 40; ++i)
                appendScaleSegment(payload, kFingerMvn[i], 0.0f, 0.0f, 0.0f);
        }

        QByteArray hdr = buildMxtpHeader("13", 0, 0x00, quint8(segCount),
                                         0, 23, fingerHdr);
        m_impl->scalePkt = hdr + payload;
        m_impl->sock.writeDatagram(m_impl->scalePkt, m_impl->host, m_impl->port);
        QThread::msleep(50);
    }

    m_impl->lastHandshakeMs = m_impl->timer.elapsed();
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

static inline Quat mvnWireOrient(const Quat& worldSeg, LiveTarget )
{
    return worldSeg;
}

static inline QVector3D mvnWirePelvisPos(const QVector3D& nwu, LiveTarget target)
{
    if (target == LiveTarget::BlenderMVN)
        return QVector3D(nwu.z(), nwu.x(), nwu.y());
    return nwu;
}

// §XXIX стрим тела MXTP02 (23 сегмента) к Blender/Unreal: per-target ремап осей (§VI: Blender Z-up/-Y,
//   Unreal Z-up/X, q_swap) + непрерывность полушария; foxwire-сериализация big-endian (formules.txt)
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

    {
        const auto ergo = fox::ergo::jointAnglesErgoAll(segQuat);
        QByteArray ergoBody;
        for (int j = 0; j < fox::body::kJointCount; ++j) {
            appendErgoAngleSegment(ergoBody, j + 1,
                                   float(ergo[j].abductionDeg),
                                   float(ergo[j].flexionDeg),
                                   float(ergo[j].rotationDeg));
        }
        const QByteArray ergoHdr = buildMxtpHeader("21", sample, 0x80,
                                                   quint8(fox::body::kJointCount),
                                                   ft, quint8(fox::body::kJointCount), 0);
        m_impl->sendChecked(ergoHdr + ergoBody);
    }

    if (wireDue) {
        wireSS << "  pelvis(world,m)=(" << pelvisPos.x() << "," << pelvisPos.y()
               << "," << pelvisPos.z() << ")  packetBytes=" << pkt.size() << "\n"
               << "============================================================\n";
        std::cout << wireSS.str();
        std::cout.flush();
    }
}

// §XXIX/§XXI стрим тела+пальцев MXTP02 (63 сегмента: 23 тела + по 20 пальцев на руку);
//   пальцы через emitFinger с baseline калибровки и непрерывностью (formules.txt)
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
            const Quat q = qWire;
            appendPoseSegment(body, segmentIdBase + slot, 0.f, 0.f, 0.f,
                              float(q.w), float(q.x), float(q.y), float(q.z));
            if (wireDue)
                wireSS << "  wireF[" << std::setw(2) << (segmentIdBase + slot) << "] "
                       << (segmentIdBase >= 44 ? "R" : "L") << "carpus(slot " << slot
                       << ") pos=(0,0,0) q=(" << q.w << "," << q.x << ","
                       << q.y << "," << q.z << ")  [follows hand]\n";
        } else {
            (void)baseArr;
            const QVector3D p = pArr[mIdx];

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

    const int bodyOnlyBytes = body.size();
    for (int slot = 0; slot < kFingerSegmentsHand; ++slot)
        emitFinger(slot, 24, SEG_LHand, leftGloveQ, leftGloveP,
                   m_impl->baselineLeftGloveQ, m_impl->prevWireLeftQ);
    for (int slot = 0; slot < kFingerSegmentsHand; ++slot)
        emitFinger(slot, 44, SEG_RHand, rightGloveQ, rightGloveP,
                   m_impl->baselineRightGloveQ, m_impl->prevWireRightQ);

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

    {
        const auto ergo = fox::ergo::jointAnglesErgoAll(segQuat);
        QByteArray ergoBody;
        for (int j = 0; j < fox::body::kJointCount; ++j) {
            appendErgoAngleSegment(ergoBody, j + 1,
                                   float(ergo[j].abductionDeg),
                                   float(ergo[j].flexionDeg),
                                   float(ergo[j].rotationDeg));
        }
        const QByteArray ergoHdr = buildMxtpHeader("21", sample, 0x80,
                                                   quint8(fox::body::kJointCount),
                                                   ft, quint8(fox::body::kJointCount), 0);
        m_impl->sendChecked(ergoHdr + ergoBody);
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
    const double fpsActual = (elapsedSec > 0.05)
        ? double(frames) / elapsedSec
        : 0.0;
    m_lblFrames->setText(QString::number(frames) + "  " + Lang::t("rec_frames")
                         + "  " + QString::asprintf("%.1f Hz", fpsActual));
    const int mm = int(elapsedSec) / 60;
    const int ss = int(elapsedSec) % 60;
    const int ds = int(elapsedSec * 10) % 10;
    m_lblTime->setText(QString::asprintf("%02d:%02d.%1d", mm, ss, ds));
}

void RecordHud::setFormatLabel(const QString& text) { m_lblFormat->setText(text); }

RecordWizard::RecordWizard(SuitType suit, QWidget* parent) : QDialog(parent), m_suit(suit)
{
    setModal(true);
    setWindowTitle(Lang::t("rec_wiz_title"));
    setMinimumSize(560, 400);
    setStyleSheet(kStyleSheet);
    buildPages();
}

void RecordWizard::buildPages()
{
    m_pages = new QStackedWidget(this);

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

    {
        auto* p = new QWidget();
        auto* title = new QLabel(Lang::t("rec_pick_fps"), p);
        title->setObjectName("heroHeading");
        title->setAlignment(Qt::AlignCenter);

        m_fps = new QComboBox(p);
        m_fps->addItem("24 fps", 24);
        m_fps->addItem("30 fps", 30);
        m_fps->addItem("60 fps", 60);
        if (m_suit == SuitType::Link) {
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

LiveStreamWizard::LiveStreamWizard(SuitType suit, QWidget* parent) : QDialog(parent), m_suit(suit)
{
    setModal(true);
    setWindowTitle(Lang::t("live_wiz_title"));
    setMinimumSize(500, 400);
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
    m_host->addItem("127.0.0.1", "127.0.0.1");
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

    m_fps = new QComboBox(this);
    m_fps->addItem("24 fps", 24);
    m_fps->addItem("30 fps", 30);
    m_fps->addItem("60 fps", 60);
    if (m_suit == SuitType::Link) {
        m_fps->addItem("120 fps", 120);
        m_fps->addItem("240 fps", 240);
    }
    m_fps->setCurrentIndex(2);
    m_fps->setMinimumHeight(34);

    m_btnStart  = new QPushButton(Lang::t("live_start"), this);
    m_btnStart->setObjectName("primary");
    m_btnStart->setMinimumHeight(40);
    m_btnCancel = new QPushButton(Lang::t("cancel"), this);
    m_btnCancel->setMinimumHeight(40);

    connect(m_btnStart, &QPushButton::clicked, this, [this]() {
        m_result.target = LiveTarget(m_target->currentData().toInt());

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

namespace {

struct BvhBone {
    const char* name;
    int         parent;
    double      offx, offy, offz;
    bool        isEnd = false;
};

struct BvhBoneRatio { const char* name; int parent; double dx, dy, dz; };
static const BvhBoneRatio kBvh[] = {

    { "Hips",             -1,   0.0,   0.000, 0.0 },
    { "Spine",             0,   0.0,   0.100, 0.0 },
    { "Spine1",            1,   0.0,   0.095, 0.0 },
    { "Spine2",            2,   0.0,   0.093, 0.0 },
    { "Chest",             3,   0.0,   0.090, 0.0 },
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

static void quatToEulerZXYdeg(const Quat& q, double& rz, double& rx, double& ry)
{

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

    const double sx = std::clamp(m21, -1.0, 1.0);
    const double xrad = std::asin(sx);
    double zrad, yrad;
    if (std::abs(sx) > 0.99999) {
        zrad = std::atan2(m10, m00);
        yrad = 0.0;
    } else {
        zrad = std::atan2(-m01, m11);
        yrad = std::atan2(-m20, m22);
    }
    const double K = 180.0 / M_PI;
    rx = xrad * K;
    ry = yrad * K;
    rz = zrad * K;
}

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

static std::array<Quat, kXsensSegmentCount>
exportWorldOrients(const RecordedFrame& fr, const SkeletonXsens& skel)
{
    std::array<Quat, kXsensSegmentCount> W;
    for (int s = 0; s < kXsensSegmentCount; ++s)
        W[s] = quat_mult(fr.segQuat[s], skel.defAngFor(s)).normalized();
    return W;
}

static inline double unwrapDeg(double prev, double cur)
{
    double d = cur - prev;
    while (d >  180.0) { cur -= 360.0; d = cur - prev; }
    while (d < -180.0) { cur += 360.0; d = cur - prev; }
    return cur;
}

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

static QVector3D exportEndSiteOffsetCm(int j, const SkeletonXsens& skel,
                                       double heightMeters)
{
    const int seg = kBoneToSeg[j];
    if (seg < 0 || seg >= kXsensSegmentCount) return QVector3D(0, 0, 0);
    const QVector3D Lspec = fox::body::kSensorToBone[seg].L_bone;
    const double scale = (heightMeters > 1e-3)
                       ? (heightMeters / fox::body::kRefHeightM) : 1.0;
    const QVector3D Lscaled = Lspec * float(scale);
    const QVector3D localTipNwu = vec_rotate(Lscaled, skel.defAngFor(seg).inv());
    return localTipNwu * 100.0f;
}

static inline Quat exportLocalRot(int j,
                                  const std::array<Quat, kXsensSegmentCount>& W)
{
    const int ownSeg = kBoneToSeg[j];
    if (kBvh[j].parent < 0) return W[ownSeg];
    const int parentSeg = kBoneToSeg[kBvh[j].parent];
    return quat_mult(W[parentSeg].inv(), W[ownSeg]).normalized();
}

// §719/§1354/§2042 экспорт BVH: иерархия HIERARCHY + MOTION, углы Эйлера (порядок каналов BVH), см, система координат BVH (formules.txt)
static bool writeBvh(const QString& path,
                     const std::vector<RecordedFrame>& frames,
                     int fps,
                     double heightMeters,
                     const SkeletonXsens& skel)
{
    if (fps <= 0) fps = 30;
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
        return false;
    QTextStream os(&f);
    os.setLocale(QLocale::c());
    os.setRealNumberPrecision(6);
    os.setRealNumberNotation(QTextStream::FixedNotation);

    (void)heightMeters;
    const auto offCm = exportBakedOffsetsCm(skel);

    std::vector<int> childCount(std::size(kBvh), 0);
    for (size_t i = 1; i < std::size(kBvh); ++i) childCount[kBvh[i].parent]++;

    std::vector<int> dfsOrder;
    dfsOrder.reserve(std::size(kBvh));

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
            const QVector3D end = exportEndSiteOffsetCm(idx, skel, heightMeters);
            os << pad(indent+1) << "End Site\n";
            os << pad(indent+1) << "{\n";
            os << pad(indent+2) << "OFFSET "
               << end.x() << " " << end.y() << " " << end.z() << "\n";
            os << pad(indent+1) << "}\n";
        }
        os << pad(indent) << "}\n";
    };

    os << "HIERARCHY\n";
    emitBone(0, 0);

    os << "MOTION\n";
    os << "Frames: " << frames.size() << "\n";
    os << "Frame Time: " << (1.0 / double(fps)) << "\n";

    std::array<double, std::size(kBvh)> prZ{}, prX{}, prY{};
    bool havePrev = false;

    const Quat cZup2Yup(0.70710678118654752, -0.70710678118654752, 0.0, 0.0);
    for (const RecordedFrame& fr : frames) {
        auto W = exportWorldOrients(fr, skel);
        for (auto& q : W) q = quat_mult(cZup2Yup, q).normalized();

        os << (fr.pelvisPos.x() * 100.0) << " "
           << (fr.pelvisPos.z() * 100.0) << " "
           << (-fr.pelvisPos.y() * 100.0);

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
    os.flush();
    if (os.status() != QTextStream::Ok || f.error() != QFileDevice::NoError)
        return false;
    return true;
}

// §717/§720/§1355 экспорт FBX (ASCII): узлы Model/AnimationCurve, локальные повороты сегментов, см (formules.txt)
static bool writeFbxAscii(const QString& path,
                          const std::vector<RecordedFrame>& frames,
                          int fps,
                          double heightMeters,
                          const SkeletonXsens& skel)
{
    if (fps <= 0) fps = 30;
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
        return false;
    QTextStream os(&f);
    os.setLocale(QLocale::c());
    os.setRealNumberPrecision(6);
    os.setRealNumberNotation(QTextStream::FixedNotation);

    (void)heightMeters;
    const auto offCm = exportBakedOffsetsCm(skel);

    os << "; FBX 7.4.0 project file\n";
    os << "; Created by Fox Mocap\n";
    os << "FBXHeaderExtension:  {\n";
    os << "    FBXHeaderVersion: 1003\n";
    os << "    FBXVersion: 7400\n";
    os << "    CreationTimeStamp:  { Version: 1000 }\n";
    os << "    Creator: \"Fox Mocap ASCII FBX Writer\"\n";
    os << "}\n";

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

    const int timeMode = (fps == 24)  ? 11
                       : (fps == 30)  ? 6
                       : (fps == 60)  ? 3
                       : (fps == 120) ? 1
                       : 14;
    os << "        P: \"TimeMode\", \"enum\", \"\", \"\"," << timeMode << "\n";
    os << "    }\n";
    os << "}\n";

    const size_t boneN = std::size(kBvh);

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

    const qint64 stackId = 20000, layerId = 20001;
    const qint64 curveIdBase = 30000;
    const qint64 curveNodeIdBase = 25000;
    const qint64 transNodeId = curveNodeIdBase + 1000;
    const qint64 transCurveId0 = curveIdBase + 100000;
    const qint64 kTimePerSecond = 46186158000LL;
    const qint64 stepTicks = kTimePerSecond / fps;
    const size_t N = frames.size();
    const qint64 stopTicks = qint64(N > 0 ? N - 1 : 0) * stepTicks;
    auto emitKeyTimes = [&]() {
        os << "        KeyTime: *" << N << " {\n            a: ";
        for (size_t k = 0; k < N; ++k) { if (k) os << ","; os << (qint64(k) * stepTicks); }
        os << "\n        }\n";
    };

    os << "Objects:  {\n";
    auto boneId = [](size_t i) { return 10000LL + qint64(i) * 16; };
    for (size_t i = 0; i < boneN; ++i) {
        const auto& b = kBvh[i];
        os << "    Model: " << boneId(i) << ", \"Model::" << b.name << "\", \"LimbNode\" {\n";
        os << "        Version: 232\n";
        os << "        Properties70:  {\n";
        os << "            P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A+\","
           << offCm[i].x() << "," << offCm[i].y() << "," << offCm[i].z() << "\n";
        os << "            P: \"RotationOrder\", \"enum\", \"\", \"\", 4\n";
        os << "        }\n";
        os << "    }\n";
    }

    os << "    AnimationStack: " << stackId << ", \"AnimStack::Take 001\", \"\" {\n";
    os << "        Properties70:  {\n";
    os << "            P: \"LocalStart\", \"KTime\", \"Time\", \"\",0\n";
    os << "            P: \"LocalStop\", \"KTime\", \"Time\", \"\"," << stopTicks << "\n";
    os << "        }\n";
    os << "    }\n";
    os << "    AnimationLayer: " << layerId << ", \"AnimLayer::BaseLayer\", \"\" {}\n";

    for (size_t i = 0; i < boneN; ++i) {
        const qint64 nodeId = curveNodeIdBase + qint64(i);
        os << "    AnimationCurveNode: " << nodeId
           << ", \"AnimCurveNode::R\", \"\" {}\n";

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
    os.flush();
    if (os.status() != QTextStream::Ok || f.error() != QFileDevice::NoError)
        return false;
    return true;
}

static double angBetween(const Quat& a, const Quat& b)
{
    double d = std::abs(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z);
    if (d > 1.0) d = 1.0;
    return 2.0 * std::acos(d);
}

static void hdOutlierReject(std::vector<RecordedFrame>& fr,
                            const std::function<void(double)>& cb = {})
{
    if (fr.size() < 3) return;
    const size_t M = fr.size() - 1;
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

static void hdQuatSmooth(std::vector<RecordedFrame>& fr,
                         const std::function<void(double)>& cb = {},
                         int fps = 90)
{

    const double tauSec = fox::body::kSkin.tauSec;
    const double sigma  = std::max(1.0, tauSec * double(std::max(30, fps)));
    const int    half   = std::max(3, int(std::ceil(4.0 * sigma)));
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

static void hdRootLowpass(std::vector<RecordedFrame>& fr, int fps)
{
    if (fr.size() < 4) return;
    const double fc = 10.0;
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

    pass([&](size_t i) -> QVector3D& { return fr[i].pelvisPos; });

    std::reverse(fr.begin(), fr.end());
    pass([&](size_t i) -> QVector3D& { return fr[i].pelvisPos; });
    std::reverse(fr.begin(), fr.end());
}

static void hdZupt(std::vector<RecordedFrame>& fr, int fps)
{
    if (fr.size() < 3) return;
    using fox::body::kContact;
    const double dt = (fps > 0) ? (1.0 / double(fps)) : (1.0 / 90.0);

    const double stillAngStep = kContact.highVelTh * dt;
    const int    windowFrames = std::max(1,
        int(kContact.firstWinWidth * double(std::max(1, fps))));
    int consec = 0;
    for (size_t i = 1; i < fr.size(); ++i) {
        const double dR = angBetween(fr[i-1].segQuat[SEG_RFoot],
                                     fr[i].segQuat[SEG_RFoot]);
        const double dL = angBetween(fr[i-1].segQuat[SEG_LFoot],
                                     fr[i].segQuat[SEG_LFoot]);
        const bool feetStill = (dR < stillAngStep) && (dL < stillAngStep);
        if (feetStill) {
            ++consec;
            if (consec >= windowFrames) {
                fr[i].pelvisPos.setX(fr[i-1].pelvisPos.x());
                fr[i].pelvisPos.setY(fr[i-1].pelvisPos.y());
            }
        } else {
            consec = 0;
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
    Quat L = quat_mult(Wup.inv(), Wlow).normalized();

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

    L = quat_mult(swing, twist).normalized();
    const Quat WlowNew = quat_mult(Wup, L).normalized();
    q[lowSeg] = quat_mult(WlowNew, dLow.inv()).normalized();
}

static void hdJointLimits(std::vector<RecordedFrame>& fr, const SkeletonXsens& skel,
                          const std::function<void(double)>& cb = {})
{
    const int N = int(fr.size());
    const double kneeSwing  = 135.0 * M_PI / 180.0;
    const double kneeTwist  =  40.0 * M_PI / 180.0;
    const double elbowSwing = 150.0 * M_PI / 180.0;
    const double elbowTwist = M_PI;
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
                                std::function<void(int )> progress,
                                const std::function<bool()>& cancelled = {})
{
    auto stop = [&]{ return cancelled && cancelled(); };

    auto band = [&](int lo, int hi) {
        return [progress, lo, hi](double f) {
            if (progress) progress(lo + int(double(hi - lo) * std::clamp(f, 0.0, 1.0)));
        };
    };
    if (progress) progress(0);
    hdOutlierReject(fr, band(0, 15));   if (stop()) return; if (progress) progress(15);
    hdQuatSmooth(fr,   band(15, 40), fps); if (stop()) return; if (progress) progress(40);
    hdFingerSmooth(fr, band(40, 60));   if (stop()) return; if (progress) progress(60);
    if (skel) hdJointLimits(fr, *skel, band(60, 75));  if (stop()) return;
    if (progress) progress(75);
    hdRootLowpass(fr, fps);             if (progress) progress(90);
    hdZupt(fr, fps);                    if (progress) progress(100);
}

}

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

namespace {

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
}

JointOffsetsDialog::JointOffsetsDialog(JointOffsets* offsets, QWidget* parent)
    : QDialog(parent), m_offsets(offsets)
{
    setWindowFlag(Qt::Window, true);
    setModal(false);
    setWindowTitle(Lang::t("js_title"));
    resize(600, 800);
    setMinimumSize(560, 480);
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
        auto* box     = new QGroupBox(Lang::t(titleKey), content);
        auto* boxLay  = new QVBoxLayout(box);
        boxLay->setContentsMargins(10, 8, 10, 10);
        boxLay->setSpacing(8);
        for (int seg : segs) {

            auto* card = new QFrame(box);
            card->setObjectName("jointCard");
            auto* grid = new QGridLayout(card);
            grid->setContentsMargins(12, 9, 12, 10);
            grid->setHorizontalSpacing(10);
            grid->setVerticalSpacing(6);

            auto* segLbl = new QLabel(Lang::t(jointDispKey(seg)), card);
            segLbl->setObjectName("jointName");
            grid->addWidget(segLbl, 0, 0, 1, 3);
            int row = 1;
            for (int a = 0; a < 3; ++a) {
                auto* axLbl = new QLabel(Lang::t(axisKey[a]), card);
                axLbl->setObjectName("subtle");
                axLbl->setFixedWidth(20);
                axLbl->setAlignment(Qt::AlignVCenter);

                auto* sld = new QSlider(Qt::Horizontal, card);
                sld->setRange(-180, 180);
                sld->setPageStep(5);

                auto* spin = new QDoubleSpinBox(card);
                spin->setRange(-180.0, 180.0);
                spin->setSingleStep(0.1);
                spin->setDecimals(1);
                spin->setSuffix(QString::fromUtf8("°"));
                spin->setMinimumWidth(96);
                spin->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

                m_ctl[seg][a] = { sld, spin };

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
            grid->setColumnStretch(1, 1);
            boxLay->addWidget(card);
        }
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
    btnRow->setSpacing(8);
    m_status = new QLabel(QString(), this);
    m_status->setObjectName("subtle");
    m_status->setWordWrap(false);
    m_status->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
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

MainWindow::MainWindow(MocapReceiver* rx,
                       const NewSessionWizard::Result& wizardResult,
                       bool testMode)
    : m_setup(wizardResult), m_test(testMode), m_rx(rx)
{
    setWindowTitle(Lang::t("app_title"));
    resize(1360, 820);

    setMinimumSize(1100, 720);

    ActorConfig actor;
    actor.heightCm        = m_setup.heightCm;
    actor.footLengthCm    = m_setup.footLengthCm;
    actor.armSpanCm       = m_setup.armSpanCm;
    actor.legLengthCm     = m_setup.legLengthCm;
    actor.hipWidthCm      = m_setup.hipWidthCm;
    actor.shoulderWidthCm = m_setup.shoulderWidthCm;
    actor.trunkLengthCm   = m_setup.trunkLengthCm;
    actor.useGloves       = m_setup.useGloves;

    m_procRateHz = nativeRateHz(m_setup.suit);

    m_viewport = new MocapViewport(actor, m_setup.poseKind, this);
    m_viewport->setProcRate(m_procRateHz);
    m_skel     = std::make_unique<SkeletonXsens>(actor, m_setup.poseKind);
    if (m_test) m_viewport->setLocoVerbose(true);
    if (m_test) m_viewport->setCondVerbose(true);
    logTest("[rate] processing rate = " + std::to_string(int(m_procRateHz)) + " Hz ("
            + (m_setup.suit == SuitType::Link ? "Xsens Link" : "Xsens Awinda") + ")");

    m_panel = new SensorIndicatorsPanel(m_setup.useGloves, this);

    connect(m_panel, &SensorIndicatorsPanel::resetClicked,
            this, [this]() {
        if (m_viewport) m_viewport->resetSceneOrigin();
        if (statusBar()) statusBar()->showMessage(Lang::t("reset_coords"), 1500);
        logTest("[action] reset-scene-origin");
    });

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

    m_streamer = new LiveStreamSender(this);
    if (m_setup.tposeCaptured) {

        if (m_viewport) {
            m_viewport->setTposeHandAnchor(
                m_setup.tposeReference[SEG_RForearm],
                m_setup.tposeReference[SEG_RHand],
                m_setup.tposeReference[SEG_LForearm],
                m_setup.tposeReference[SEG_LHand]);

            m_viewport->setTposeFootAnchor(
                m_setup.tposeReference[SEG_Pelvis],
                m_setup.tposeReference[SEG_RFoot],
                m_setup.tposeReference[SEG_LFoot]);
        }

        if (m_setup.fingerBaselineCaptured) {
            QMutexLocker lkBL(&g_fingerBaseline.lock);
            for (int i = 0; i < 20; ++i) {
                g_fingerBaseline.left[i]  = m_setup.fingerBaselineL[i];
                g_fingerBaseline.right[i] = m_setup.fingerBaselineR[i];
            }
            g_fingerBaseline.valid.store(true);
        }

    }

    if (m_test) {
        LiveSettings cfg;
        cfg.useGloves = m_setup.useGloves;

        cfg.target              = LiveTarget::BlenderMVN;
        cfg.debugDumpFirstFrame = true;
        cfg.verboseLog          = true;
        if (m_skel) {
            std::array<Quat, kXsensSegmentCount> identity{};
            for (auto& qq : identity) qq = Quat(1, 0, 0, 0);
            const float pelvisZ = float(fox::body::pelvisStandHeightM(m_setup.heightCm / 100.0));
            auto kp = m_skel->computeKeypoints(identity, QVector3D(0.0f, 0.0f, pelvisZ));
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                cfg.tposeOriginM[i] = kp[i];
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

    connect(m_rx, &MocapReceiver::statusChanged,      this, &MainWindow::onConnStatusChanged);
    connect(m_rx, &MocapReceiver::gloveStatusChanged, this, &MainWindow::onGloveStatus);
    connect(m_rx, &MocapReceiver::fpsUpdated,         this, &MainWindow::onFps);

    {
        const QString jp = JointOffsets::filePath();
        if (QFile::exists(jp)) m_jointOffsets.load(jp);
        else                   m_jointOffsets.save(jp);
    }

    m_renderTimer.setTimerType(Qt::PreciseTimer);
    m_renderTimer.setInterval(int(1000.0 / m_procRateHz));
    connect(&m_renderTimer, &QTimer::timeout, this, &MainWindow::onRenderTick);
    m_renderTimer.start();

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

void MainWindow::onConnStatusChanged(int status, const QString& )
{
    const ConnStatus s = (ConnStatus)status;
    const bool streaming = (s == ConnStatus::Streaming);
    m_panel->setSuitLive(streaming, {});
    if (!streaming) m_panel->clearLiveDots();

    if (!streaming && m_sessionRunning)      onPauseSession();
    else if (streaming && !m_sessionRunning) onResumeSession();
    logTest(std::string("[suit] ") + connStatusName(s));
}

void MainWindow::onGloveStatus(bool on)
{
    logTest(std::string("[gloves] ") + (on ? "connected" : "disconnected"));
}

void MainWindow::onFps(double hz)
{
    if (m_panel) m_panel->setFps(hz);
}

namespace {
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

QVector3D worldPelvisWithLoco(const SkeletonXsens& skel,
                              const std::array<Quat, kXsensSegmentCount>& segWorld,
                              const QVector3D& locoOffset)
{
    auto kp = skel.computeKeypoints(segWorld, QVector3D(0.0f, 0.0f, 0.0f));
    for (auto& p : kp) p += locoOffset;
    float minZ = kp[0].z();
    for (const auto& p : kp) if (p.z() < minZ) minZ = p.z();
    constexpr float kFloorClampShallowM = -0.10f;
    if (minZ < -0.02f && minZ >= kFloorClampShallowM) {
        const QVector3D up(0.0f, 0.0f, -minZ);
        for (auto& p : kp) p += up;
    }
    return kp[SEG_Pelvis];
}
}

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
        true,  false, false, false, true,  false, true,
        true,  true,  true,  true,
        true,  true,  true,  true,
        true,  true,  true,  false,
        true,  true,  true,  false
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

    std::array<Quat, kXsensSegmentCount> q{};
    for (int i = 0; i < kXsensSegmentCount; ++i) {
        if (!kTracked[i]) continue;

        g_renderDiag.jumpDeg[i]   = 0.0;
        g_renderDiag.rejectW[i]   = 0.0;
        g_renderDiag.gyroQuiet[i] = false;

        Quat cand = quat_mult(raw[i], s_refWorldInv[i]).normalized();

        if (f.segValid[i] && s_haveOut[i]) {
            const double jumpDeg = quat_angle_deg(quat_mult(cand, s_lastOut[i].inv()));
            const float kGyroQuietSq = float(fox::body::kJumpDetect.gyroQuietDegS *
                                              fox::body::kJumpDetect.gyroQuietDegS);
            const bool gyroQuiet =
                (f.gyrSensor[SEG_Pelvis].lengthSquared() < kGyroQuietSq) &&
                (f.gyrSensor[i].lengthSquared()          < kGyroQuietSq);
            g_renderDiag.jumpDeg[i]   = jumpDeg;
            g_renderDiag.gyroQuiet[i] = gyroQuiet;

            if (gyroQuiet && jumpDeg > fox::body::kJumpDetect.threshDeg) {
                auto smoothstep01 = [](double x){ x = std::clamp(x,0.0,1.0); return x*x*(3.0-2.0*x); };
                const double rejectW = smoothstep01(
                    (jumpDeg - fox::body::kJumpDetect.threshDeg) /
                    std::max(1.0, fox::body::kJumpDetect.blendRangeDeg));
                g_renderDiag.rejectW[i] = rejectW;
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

    q[SEG_RToe] = Quat(1, 0, 0, 0);
    q[SEG_LToe] = Quat(1, 0, 0, 0);
    if (!m_skel) {

        q[SEG_L5]   = q[SEG_Pelvis];
        q[SEG_L3]   = q[SEG_Pelvis];
        q[SEG_T12]  = q[SEG_T8];
        q[SEG_Neck] = q[SEG_T8];
    }

    s_lastOut[SEG_L5]   = q[SEG_L5];
    s_lastOut[SEG_L3]   = q[SEG_L3];
    s_lastOut[SEG_T12]  = q[SEG_T12];
    s_lastOut[SEG_Neck] = q[SEG_Neck];
    s_lastOut[SEG_RToe] = q[SEG_RToe];
    s_lastOut[SEG_LToe] = q[SEG_LToe];
    s_haveOut[SEG_L5] = s_haveOut[SEG_L3] = s_haveOut[SEG_T12] =
    s_haveOut[SEG_Neck] = s_haveOut[SEG_RToe] = s_haveOut[SEG_LToe] = true;

    if (m_skel) {
        const double kneeSwing  = 135.0 * M_PI / 180.0;
        const double kneeTwist  =  45.0 * M_PI / 180.0;
        const double elbowSwing = 150.0 * M_PI / 180.0;
        projectHingeLimit(q, SEG_RUpperLeg, SEG_RLowerLeg, *m_skel, kneeSwing,  kneeTwist);
        projectHingeLimit(q, SEG_LUpperLeg, SEG_LLowerLeg, *m_skel, kneeSwing,  kneeTwist);
        projectHingeLimit(q, SEG_RUpperArm, SEG_RForearm,  *m_skel, elbowSwing, M_PI);
        projectHingeLimit(q, SEG_LUpperArm, SEG_LForearm,  *m_skel, elbowSwing, M_PI);
    }

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

        m_skel->setAccLPBodyHint(f.accSensor);
        const float pelvisZ_loco = float(fox::body::pelvisStandHeightM(m_setup.heightCm / 100.0));
        auto kpLoco = m_skel->computeKeypoints(qOut, QVector3D(0.0f, 0.0f, pelvisZ_loco));

        const double tSec = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();

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

            const double cR = sstep01(double(rPel.y()) / 0.08);
            const double cL = sstep01(double(-lPel.y()) / 0.08);
            m_viewport->setCrossLeggedHints(cR > 0.5, cL > 0.5, cR, cL);
        }

        m_viewport->tickLoco(qOut,
                             kpLoco[SEG_RFoot], kpLoco[SEG_RToe], kpLoco[26],
                             kpLoco[SEG_LFoot], kpLoco[SEG_LToe], kpLoco[27],
                             tSec);
    }

    if (m_streamer && m_streamer->isRunning()) {
        QVector3D pelvisM(0.0f, 0.0f, 0.0f);
        std::array<Quat, kXsensSegmentCount> qStream = qOut;

        if (m_skel)
            pelvisM = worldPelvisWithLoco(*m_skel, qOut, m_viewport->lastLocoOffset());
        const bool gloves = f.hasGloves && m_setup.useGloves;
        if (gloves) {

            const Quat qRWristWire = qStream[SEG_RHand];
            const Quat qLWristWire = qStream[SEG_LHand];

            const Quat finger90 = axisAngleQuat(QVector3D(1, 0, 0), -M_PI / 2.0);
            std::array<Quat, kFingerSegmentsHand> rGloveWorld, lGloveWorld;
            std::array<QVector3D, kFingerSegmentsHand> rGloveWorldP, lGloveWorldP;
            for (int i = 0; i < kFingerSegmentsHand; ++i) {
                const Quat rQ = quat_mult(f.rightGloveQ[i], finger90);
                const Quat lQ = quat_mult(f.leftGloveQ[i],  finger90);
                rGloveWorld[i] = quat_mult(qRWristWire, rQ);
                lGloveWorld[i] = quat_mult(qLWristWire, mirror_y_quat(lQ));
                rGloveWorldP[i] = vec_rotate(f.rightGloveP[i],              qRWristWire);
                lGloveWorldP[i] = vec_rotate(mirrorManusL(f.leftGloveP[i]), qLWristWire);
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

            static double s_cumHoriz = 0.0;
            static QVector3D s_startPelvis(0, 0, 0);
            static bool s_haveStart = false;
            if (!s_haveStart) { s_startPelvis = pelvisM; s_haveStart = true; }
            if (s_havePrevPelvis) {
                const QVector3D d = pelvisM - s_lastStreamPelvis;
                const float dxy = std::sqrt(d.x()*d.x() + d.y()*d.y());
                if (dxy < 0.5f) s_cumHoriz += dxy;
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

    if (m_recording && qint64(f.sampleCounter) != m_recLastSample) {
        m_recLastSample = qint64(f.sampleCounter);
        const double tNow =
            double(QDateTime::currentMSecsSinceEpoch() - m_recStartMs) / 1000.0;

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
            const double snapDt = now - t0;
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

            static const int kSegParent[kXsensSegmentCount] = {
                -1, 0, 1, 2, 3, 4, 5,
                 4, 7, 8, 9,
                 4,11,12,13,
                 0,15,16,17,
                 0,19,20,21,
            };
            ss << std::setprecision(3);
            ss << "--- 23 segments: raw -> post-calib q -> drift-locked qOut; "
                  "world quat, local joint angle, drift, lock ---\n";
            for (int i = 0; i < kXsensSegmentCount; ++i) {
                const Quat& in  = raw[i];
                const Quat& out = q[i];
                const Quat& flt = qOut[i];
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

            if (m_gloves) {
                std::array<QVector3D, fox::body::kSegmentCount> segCenters{};
                for (int i = 0; i < fox::body::kSegmentCount && i < kXsensSegmentCount; ++i) {
                    QVector3D origin = pts[i];
                    QVector3D end    = (i + 1 < kXsensKeypointCount) ? pts[i + 1] : pts[i];
                    const double ratio = fox::body::kWinterProxToComRatio[i];
                    segCenters[i] = origin + (end - origin) * float(ratio);
                }
                double M = 0.0;
                const QVector3D com = fox::body::centerOfMass(segCenters, &M);
                ss << "--- [COM] §12.1 body centre of mass ---\n";
                ss << "  com=(" << std::setw(7) << com.x() << ","
                   << std::setw(7) << com.y() << ","
                   << std::setw(7) << com.z() << ")  m=" << M << " kg\n";

                const auto ergo = fox::ergo::jointAnglesErgoAll(fk.oriented);
                ss << "--- [ROM] joints approaching limits (≥90% range) ---\n";
                int hits = 0;
                for (int j = 0; j < fox::body::kJointCount; ++j) {
                    const auto& r = fox::body::kJointRom[j];
                    auto frac = [](double v, double lo, double hi) {
                        const double mid = 0.5 * (lo + hi), half = 0.5 * (hi - lo);
                        return (half > 1e-6) ? std::abs((v - mid) / half) : 0.0;
                    };
                    const double fa = frac(ergo[j].abductionDeg, r.abdMin, r.abdMax);
                    const double ff = frac(ergo[j].flexionDeg,   r.flxMin, r.flxMax);
                    const double fr = frac(ergo[j].rotationDeg,  r.rotMin, r.rotMax);
                    const double fmx = std::max({fa, ff, fr});
                    if (fmx >= 0.9) {
                        ss << "  j[" << std::setw(2) << j
                           << "] abd=" << std::setw(7) << ergo[j].abductionDeg
                           << " flx=" << std::setw(7) << ergo[j].flexionDeg
                           << " rot=" << std::setw(7) << ergo[j].rotationDeg
                           << "  | frac=" << std::setprecision(2) << fmx
                           << std::setprecision(3) << "\n";
                        ++hits;
                    }
                }
                if (hits == 0) ss << "  (all joints within healthy ROM)\n";
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

            }

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
            ss << "--- arm coupling (scapular-humeral shrug, populated by WLS H-block) ---\n";
            ss << "  scapular R: upZ=" << g_renderDiag.scapUpZR
               << " active=" << (g_renderDiag.scapActiveR ? "yes" : "no")
               << " appliedAng=" << (g_renderDiag.scapAngR * 180.0 / M_PI) << "deg | L: upZ="
               << g_renderDiag.scapUpZL << " active=" << (g_renderDiag.scapActiveL ? "yes" : "no")
               << " appliedAng=" << (g_renderDiag.scapAngL * 180.0 / M_PI) << "deg\n";

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

    if (m_test) {
        static constexpr quint64 kPulseStride = 1;
        const LocoDiag L = m_viewport ? m_viewport->locoDiag() : LocoDiag{};
        auto jAng = [&](int parent, int child){
            return quat_angle_deg(quat_mult(q[parent].inv(), q[child])); };
        int maxSeg = 0; double maxWdeg = 0.0;
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            const double wdeg = s_worldOmegaLP[i] * 180.0 / M_PI;
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

        static int s_prevPose = -1;
        if (int(L.pose) != s_prevPose) {
            std::cout << "[evt:pose] f=" << f.sampleCounter << " "
                      << locoPoseName(s_prevPose < 0 ? 0 : s_prevPose) << " -> " << locoPoseName(L.pose)
                      << " (tiltCos=" << std::setprecision(3) << L.tiltCos
                      << " pelvisZVel=" << L.pelvisZVel << "m/s)\n";
            s_prevPose = int(L.pose);
        }

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

        static int s_lastPelvisZEvt = -1000;
        if (std::abs(L.pelvisZVel) > 0.40 && (s_evtTick - s_lastPelvisZEvt) > 15) {
            s_lastPelvisZEvt = s_evtTick;
            std::cout << "[evt:pelvisZ] f=" << f.sampleCounter << " pelvisZVel="
                      << std::setprecision(3) << L.pelvisZVel << "m/s pose=" << locoPoseName(L.pose)
                      << " airborneT=" << L.airborneTicks << " landedT=" << L.landedTicks << "\n";
        }

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

        struct BurstRec {
            quint64 f; double t; float dtms; int pose;
            float pelX, pelY, pelZ, pelvisZVel, footPzR, footPzL, maxWdeg; int maxWseg;
            float rKnee, lKnee, rElb, lElb, rHip, lHip, spine, neck;
        };
        static constexpr int kBurstCap = 128;
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
            for (int n = 0; n < s_burstCount; ++n)
                std::cout << fmtBurst(s_burst[(start + n) % kBurstCap]) << " [pre]\n";
            s_burstPost = 120;
        }
        if (s_burstPost > 0) { std::cout << fmtBurst(rec) << " [post]\n"; --s_burstPost; }

        std::cout.flush();
    }

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
    if (m_panel)    m_panel->setFreezeState(false);
    if (m_viewport) m_viewport->setFreezeXY(false);
    logTest("[session] resumed");
}

void MainWindow::onOpenLiveWizard()
{

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
    cfg.verboseLog = m_test;

    if (m_skel) {
        std::array<Quat, kXsensSegmentCount> identity{};
        for (auto& q : identity) q = Quat(1, 0, 0, 0);
        const float pelvisZ = float(fox::body::pelvisStandHeightM(m_setup.heightCm / 100.0));
        auto kp = m_skel->computeKeypoints(identity, QVector3D(0.0f, 0.0f, pelvisZ));
        for (int i = 0; i < kXsensSegmentCount; ++i) {
            cfg.tposeOriginM[i] = kp[i];
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
    if (m_finishing) return;
    if (m_takePending) {
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
    if (m_finishing || m_takePending) return;
    m_recCfg = cfg;
    m_recBuffer.clear();
    m_recBuffer.reserve(size_t(m_procRateHz * 60 * 10));
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
    if (m_finishing) return;
    if (!m_recording && !m_takePending) return;
    m_recording = false;
    finishRecording();
}

void MainWindow::finishRecording()
{
    if (m_finishing) return;
    if (m_recBuffer.empty()) {
        m_takePending = false;
        if (m_hud) m_hud->hide();
        logTest("[rec] stop — empty buffer, nothing to save");
        return;
    }
    m_finishing   = true;
    m_takePending = true;
    if (m_hud) m_hud->hide();

    auto keepUnsavedTake = [this](const char* logMsg) {
        m_finishing = false;
        if (m_hud) {
            m_hud->setFormatLabel(Lang::t("rec_unsaved"));
            m_hud->show(); m_hud->raise(); layoutHud();
        }
        logTest(logMsg);
    };

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

    if (m_hud) {
        const QPoint tl = m_viewport->mapTo(centralWidget(), QPoint(0, 0));
        const int x = tl.x() + m_viewport->width() - m_hud->width() - 16;
        int y = tl.y() + 16;

        if (m_modeHud && m_modeHud->isVisible()
            && x < m_modeHud->x() + m_modeHud->width() + 8) {
            y = std::max(y, m_modeHud->y() + m_modeHud->height() + 8);
        }
        m_hud->move(std::max(0, x), std::max(0, y));
    }

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
            const QString dot = QString::fromUtf8("\xE2\x97\x8F");

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

        if (m_hud && m_hud->isVisible()) {
            const int x = tl.x() + m_viewport->width() - m_hud->width() - 16;
            int y = tl.y() + 16;
            if (x < m_modeHud->x() + m_modeHud->width() + 8) {
                y = std::max(y, m_modeHud->y() + m_modeHud->height() + 8);
            }
            m_hud->move(std::max(0, x), std::max(0, y));
            m_hud->raise();
        }
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

    if (m_finishing) { e->ignore(); return; }

    if (m_recording || m_takePending) {
        const auto btn = QMessageBox::question(
            this, Lang::t("rec_wiz_title"), Lang::t("rec_close_prompt"),
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
            QMessageBox::Save);
        if (btn == QMessageBox::Cancel) { e->ignore(); return; }
        if (btn == QMessageBox::Save) {
            onRecordStop();
            if (m_takePending) { e->ignore(); return; }
        } else {
            m_recording   = false;
            m_takePending = false;
            if (m_hud) m_hud->hide();
            m_recBuffer.clear();
        }
    }
    if (m_rx) { m_rx->stop(); m_rx->wait(1500); }
    QMainWindow::closeEvent(e);
}

const char* kStyleSheet = R"(
  QMainWindow, QDialog { background: #0E0E0E; }
  QWidget           { color: #EAEAEA; font-family: 'Segoe UI', sans-serif; font-size: 10pt; }
  QWidget#sidePanel { background: #141414; border-right: 1px solid #1F1F1F; }

  /* Suit-status summary card (top of the side panel). */
  QWidget#statusCard { background: #181818; border: 1px solid #242424;
                       border-radius: 10px; }

  /* Record HUD overlay — translucent backdrop so the labels stay legible
     against the 3D viewport (matches ModeHud styling). */
  QWidget#recordHud  { background: rgba(20, 20, 20, 220);
                       border: 1px solid #3a3a3a; border-radius: 10px; }

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

  /* Joint-correction window (Settings): per-joint cards + header. */
  QFrame#jointCard  { background: #141414; border: 1px solid #242424;
                      border-radius: 8px; }
  QLabel#jointName  { color: #FFB066; font-weight: 700; }

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

CliArgs parseCli(int argc, char** argv)
{
    CliArgs out;

    bool suitExplicit = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-test" || a == "--test")          out.test = true;
        else if (a == "--gloves" || a == "-gloves") out.gloves = true;
        else if (a == "--wrist-constraint")         out.wristConstraint = true;
        else if (a == "--link"   || a == "-link")   { out.suit = SuitType::Link;   suitExplicit = true; }
        else if (a == "--awinda" || a == "-awinda") { out.suit = SuitType::Awinda; suitExplicit = true; }
        else if ((a == "--lang" || a == "--language") && i + 1 < argc) {
            out.language = QString::fromUtf8(argv[++i]).toLower();
        }
        else if (a == "-h" || a == "--help") {
            std::cout <<
                "Fox Mocap — MVN-style Xsens client (Link 240 Hz / Awinda 90 Hz)\n"
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
                "  --awinda   Xsens Awinda suit — 90 Hz update rate (default).\n"
                "             --link/--awinda override -test's default in ANY order,\n"
                "             so `-test -gloves -awinda` runs the 90 Hz Awinda suit.\n"
                "  --lang RU|EN  Force UI language without touching the user preference.\n";
            std::exit(0);
        }
    }

    if (out.test && !suitExplicit) out.suit = SuitType::Link;
    return out;
}

void testLog(const std::string& msg, bool enabled)
{
    if (!enabled) return;
    std::cout << msg << '\n';
    std::cout.flush();
}

static void attachTestOutput()
{
#ifdef _WIN32

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

}

int main(int argc, char** argv)
{
    using namespace fox;

    const CliArgs cli = parseCli(argc, argv);
    fox::pose_solver::g_testFlag().store(cli.test);
    fox::pose_solver::g_glovesFlag().store(cli.gloves);

    if (cli.language == "ru") Lang::instance().setLanguage(Lang::RU);
    else if (cli.language == "en") Lang::instance().setLanguage(Lang::EN);
    if (cli.test) {
        attachTestOutput();

        std::cout << "[boot] suit=" << (cli.suit == SuitType::Link ? "Link" : "Awinda")
                  << " nativeRate=" << nativeRateHz(cli.suit) << "Hz"
                  << " gloves=" << (cli.gloves ? "on" : "off")
                  << " wristConstraint=" << (cli.wristConstraint ? "on" : "off") << "\n";
        std::cout.flush();
    }

    {
        QSurfaceFormat fmt;
        fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
        fmt.setVersion(3, 3);
        fmt.setDepthBufferSize(24);
        fmt.setSamples(4);
        QSurfaceFormat::setDefaultFormat(fmt);
    }

    QApplication app(argc, argv);
    app.setStyleSheet(kStyleSheet);
    app.setApplicationName("Fox-Mocap");
    app.setApplicationVersion("0.1");

    const QIcon foxIcon = makeFoxAppIcon();
    app.setWindowIcon(foxIcon);

    testLog(std::string("[boot] fox_mocap starting, test_mode=")
            + (cli.test ? "true" : "false"), cli.test);

    auto* rx = new MocapReceiver(cli.test, &app);

    NewSessionWizard wiz(rx, cli.test);

    testLog(std::string("[boot] suit = ")
            + (cli.suit == SuitType::Link ? "Xsens Link (240 Hz)" : "Xsens Awinda (90 Hz)"),
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

    auto* win = new MainWindow(rx, result, cli.test);
    win->setGlovesMode(cli.gloves);
    if (cli.wristConstraint) {
        win->setWristConstraintEnabled(true);
    }
    win->show();
    applyDarkTitleBar(win);

    const int rc = app.exec();
    delete win;
    return rc;
}
