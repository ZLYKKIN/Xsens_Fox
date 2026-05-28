
#pragma once

#include "foxmath.h"

#include <QtGui/QVector3D>

#include <array>
#include <cstdint>

namespace fox::body {

// §VIII размеры модели тела: 23 сегмента, 22 сустава, 7 lump-групп (§1784),
//   26 контактных строк IcontactsConsidered M[26,6] (§138.16) (formules.txt)
constexpr int kSegmentCount = 23;
constexpr int kJointCount   = 22;
constexpr int kLumpGroups   = 7;
constexpr int kContactRows  = 26;

enum Pose : std::uint8_t   { PoseT = 0, PoseN = 1 };
enum Gender : std::uint8_t { GenderLegacy = 0, GenderMale = 1, GenderFemale = 2 };

struct AnthroProportions {
    double trunkRatio;
    double thighRatio;
    double shankRatio;
    double handRatio;
    double forearmRatio;
    double upperArmRatio;
    double shoulderWidthRatio;
    double hipWidthRatio;
    double hipHeightRatio;
    double kneeHeightRatio;
    double ankleHeightRatio;
    double footRatio;
};

// §57/§1990 антропометрические пропорции (доли от роста bodyHeight); uniform-масштабирование
//   от базовой модели ~1.75 м: длины сегментов *= H/1.75. Значения M/Ж/legacy сверены (formules.txt)
inline constexpr AnthroProportions kAnthroMale = {
    .trunkRatio          = 0.295,
    .thighRatio          = 0.245,
    .shankRatio          = 0.246,
    .handRatio           = 0.108,
    .forearmRatio        = 0.146,
    .upperArmRatio       = 0.186,
    .shoulderWidthRatio  = 0.259,
    .hipWidthRatio       = 0.190,
    .hipHeightRatio      = 0.530,
    .kneeHeightRatio     = 0.285,
    .ankleHeightRatio    = 0.039,
    .footRatio           = 0.152,
};

inline constexpr AnthroProportions kAnthroFemale = {
    .trunkRatio          = 0.292,
    .thighRatio          = 0.250,
    .shankRatio          = 0.241,
    .handRatio           = 0.106,
    .forearmRatio        = 0.144,
    .upperArmRatio       = 0.182,
    .shoulderWidthRatio  = 0.234,
    .hipWidthRatio       = 0.225,
    .hipHeightRatio      = 0.530,
    .kneeHeightRatio     = 0.285,
    .ankleHeightRatio    = 0.039,
    .footRatio           = 0.149,
};

inline constexpr AnthroProportions kAnthroLegacy = {
    .trunkRatio          = 0.288,
    .thighRatio          = 0.238,
    .shankRatio          = 0.232,
    .handRatio           = 0.105,
    .forearmRatio        = 0.140,
    .upperArmRatio       = 0.171,
    .shoulderWidthRatio  = 0.234,
    .hipWidthRatio       = 0.181,
    .hipHeightRatio      = 0.510,
    .kneeHeightRatio     = 0.293,
    .ankleHeightRatio    = 0.039,
    .footRatio           = 0.144,
};

inline constexpr const AnthroProportions& anthroFor(Gender g)
{
    switch (g) {
        case GenderMale:   return kAnthroMale;
        case GenderFemale: return kAnthroFemale;
        case GenderLegacy:
        default:           return kAnthroLegacy;
    }
}

enum class ConfigurationLabel : std::uint8_t {
    FullBody = 0,
    FullBodyNoSternum,
    UpperBody,
    UpperBodyNoSternum,
    LowerBody,
    PelvisSternum,
    PelvisOnly,
    Sternum,
    SingleDevice,
    Hands,
    UpperBodyWithHands,
    FullBodyWithHands,
    Generic,
};

inline constexpr int kConfigurationLabelCount = 13;

// §37.1 доли массы тела по сегментам, % (FOX_FE.bioMech.segmentMassRatios x100): Pelvis 11.7188% и т.д. (formules.txt)
inline constexpr std::array<double, kSegmentCount> kMassRatio = {
    11.7188,
     7.8125,
     6.8359,
     5.8594,
     5.8594,
     1.9531,
     5.8594,
     1.9531,
     2.9297,
     1.5625,
     0.5859,
     1.9531,
     2.9297,
     1.5625,
     0.5859,
    14.1602,
     4.3945,
     1.0742,
     0.3906,
    14.1602,
     4.3945,
     1.0742,
     0.3906,
};

// §1341 доля проксимальный_конец -> CoM на сегменте (ratio_i): UpperArm 0.436, ForeArm 0.430,
//   Hand 0.506, Up/LowLeg 0.433, прочие 0.50. Значения УНИКАЛЬНЫ для движка, НЕ Winter-1990 (§1123).
//   (имя kWinter* историческое; значения сверены с §1341) (formules.txt)
inline constexpr std::array<double, kSegmentCount> kWinterProxToComRatio = {
    0.500,
    0.500,
    0.500,
    0.500,
    0.500,
    0.500,
    0.500,
    0.500,
    0.436,
    0.430,
    0.506,
    0.500,
    0.436,
    0.430,
    0.506,
    0.433,
    0.433,
    0.500,
    0.500,
    0.433,
    0.433,
    0.500,
    0.500,
};

// §37.1/§311.2 эталонная масса субъекта 75 кг (m_i = mass_ratio_i * body_mass; Pelvis=8.79 кг) (formules.txt)
inline constexpr double kDefaultBodyMassKg = 75.0;

struct SensorToBone {
    Quat       q_bs;
    QVector3D  r_bs;
    QVector3D  L_bone;
};

// §88 q_bs — ориентация датчика в системе кости (sensor->bone); §89/§90 r_bs — смещение датчика на кости;
//   §57 L_bone — вектор длины кости. Pelvis q_bs=(0.048101,0.517692,-0.029168,-0.853716), angle=174.49° (formules.txt)
inline const std::array<SensorToBone, kSegmentCount> kSensorToBone = {{

    { Quat( 0.048101,  0.517692, -0.029168, -0.853716),
      QVector3D(-0.05563f,  0.00000f,  0.09514f),
      QVector3D(-0.01081f,  0.00000f,  0.09730f) },

    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.00000f,  0.00000f,  0.10790f) },

    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.00000f,  0.00000f,  0.09851f) },

    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.00000f,  0.00000f,  0.09840f) },

    { Quat( 0.812802, -0.010534,  0.582112,  0.019711),
      QVector3D( 0.14000f,  0.00000f,  0.07700f),
      QVector3D( 0.00000f,  0.00000f,  0.13790f) },

    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.00000f,  0.00000f,  0.09161f) },

    { Quat( 0.611905,  0.694764, -0.362720,  0.106347),
      QVector3D(-0.06928f,  0.00000f,  0.07616f),
      QVector3D( 0.00000f,  0.00000f,  0.17029f) },

    { Quat( 0.776381,  0.332211, -0.258954, -0.468841),
      QVector3D(-0.02000f, -0.05000f, -0.06000f),
      QVector3D( 0.00000f, -0.14000f,  0.00000f) },

    { Quat( 0.645046, -0.253557,  0.224966, -0.684846),
      QVector3D(-0.02000f, -0.10000f,  0.02000f),
      QVector3D( 0.00000f, -0.30000f,  0.00000f) },

    { Quat( 0.749726, -0.193893,  0.211256, -0.596396),
      QVector3D( 0.00184f, -0.20024f,  0.02000f),
      QVector3D( 0.00000f, -0.24520f,  0.00000f) },

    { Quat( 0.692346,  0.147448,  0.046195, -0.704828),
      QVector3D( 0.00000f, -0.05500f,  0.02000f),
      QVector3D( 0.00000f, -0.18300f,  0.00000f) },

    { Quat( 0.724949, -0.264417, -0.439236,  0.460004),
      QVector3D(-0.02000f,  0.05000f, -0.06000f),
      QVector3D( 0.00000f,  0.14000f,  0.00000f) },

    { Quat( 0.691599,  0.080354,  0.106548,  0.709846),
      QVector3D(-0.02000f,  0.10000f,  0.02000f),
      QVector3D( 0.00000f,  0.30000f,  0.00000f) },

    { Quat( 0.759964,  0.011096,  0.069391,  0.646155),
      QVector3D( 0.00184f,  0.20024f,  0.02000f),
      QVector3D( 0.00000f,  0.24520f,  0.00000f) },

    { Quat( 0.669439, -0.100277,  0.054421,  0.734053),
      QVector3D( 0.00000f,  0.05500f,  0.02000f),
      QVector3D( 0.00000f,  0.18300f,  0.00000f) },

    { Quat( 0.450969,  0.605623,  0.478005, -0.448730),
      QVector3D( 0.01205f, -0.06000f, -0.25071f),
      QVector3D( 0.00000f,  0.00000f, -0.41648f) },

    { Quat( 0.633032, -0.250941,  0.700127,  0.214757),
      QVector3D( 0.03090f,  0.01000f, -0.13293f),
      QVector3D( 0.00000f,  0.00000f, -0.40634f) },

    { Quat( 0.960726,  0.112046,  0.252661, -0.024793),
      QVector3D( 0.08500f,  0.00200f, -0.01200f),
      QVector3D( 0.14700f,  0.00000f, -0.06500f) },

    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.06400f,  0.00000f, -0.01500f) },

    { Quat( 0.383195, -0.605793,  0.460515,  0.523548),
      QVector3D( 0.01205f,  0.06000f, -0.25071f),
      QVector3D( 0.00000f,  0.00000f, -0.41648f) },

    { Quat( 0.652402,  0.304931,  0.665425, -0.196464),
      QVector3D( 0.03090f, -0.01000f, -0.13293f),
      QVector3D( 0.00000f,  0.00000f, -0.40634f) },

    { Quat( 0.953278, -0.149169,  0.227166,  0.131928),
      QVector3D( 0.08500f, -0.00200f, -0.01200f),
      QVector3D( 0.14700f,  0.00000f, -0.06500f) },

    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.06400f,  0.00000f, -0.01500f) },
}};

// §24 калибровочные углы эталонных поз: √2/2=sin45° (руки 90° в N-позе), sin5°/cos5° (плечо 10°) (formules.txt)
inline constexpr double kRefSqrtHalf  = 0.7071067811865475;
inline constexpr double kRefSin5      = 0.087155742747658166;
inline constexpr double kRefCos5      = 0.996194698091745532;

// §24/§1762 эталонные ориентации сегментов q_gb для T-позы (q_eta); сверено с дампом fox_definitions.xsb (formules.txt)
inline constexpr std::array<Quat, kSegmentCount> kRefQuatT = {{
     Quat( 0.9984697627340179, 0.0,  0.05530038793601835, 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 0.9987077007098614, 0.0, -0.05082252003612363, 0.0),
     Quat( 0.9987050781810652, 0.0, -0.05087402888854364, 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 0.9939511715005132, 0.0,  0.1098228968510563,  0.0),
     Quat( 0.999568500071736,  0.0,  0.02937369000210807, 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 0.9997115343780182, 0.0,  0.02401766082591783, 0.0),
     Quat( 0.999173864575052,  0.0,  0.04063973855914903, 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 0.9997115343780182, 0.0,  0.02401766082591783, 0.0),
     Quat( 0.999173864575052,  0.0,  0.04063973855914903, 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
     Quat( 1.0,                0.0,  0.0,                 0.0),
}};

// §24/§1762 эталонные ориентации q_gb для N-позы: руки опущены (√2/2), плечо 10° (cos5/sin5) (formules.txt)
inline constexpr std::array<Quat, kSegmentCount> kRefQuatN = {{
     Quat( 0.9984697627340179, 0.0,            0.05530038793601835, 0.0),
     Quat( 1.0,                0.0,            0.0,                 0.0),
     Quat( 0.9987077007098614, 0.0,           -0.05082252003612363, 0.0),
     Quat( 0.9987050781810652, 0.0,           -0.05087402888854364, 0.0),
     Quat( 1.0,                0.0,            0.0,                 0.0),
     Quat( 0.9939511715005132, 0.0,            0.1098228968510563,  0.0),
     Quat( 0.999568500071736,  0.0,            0.02937369000210807, 0.0),
     Quat( kRefCos5,           kRefSin5,       0.0,                 0.0),
     Quat( kRefSqrtHalf,       kRefSqrtHalf,   0.0,                 0.0),
     Quat( kRefSqrtHalf,       kRefSqrtHalf,   0.0,                 0.0),
     Quat( kRefSqrtHalf,       kRefSqrtHalf,   0.0,                 0.0),
     Quat( kRefCos5,          -kRefSin5,       0.0,                 0.0),
     Quat( kRefSqrtHalf,      -kRefSqrtHalf,   0.0,                 0.0),
     Quat( kRefSqrtHalf,      -kRefSqrtHalf,   0.0,                 0.0),
     Quat( kRefSqrtHalf,      -kRefSqrtHalf,   0.0,                 0.0),
     Quat( 0.9997115343780182, 0.0,            0.02401766082591783, 0.0),
     Quat( 0.999173864575052,  0.0,            0.04063973855914903, 0.0),
     Quat( 1.0,                0.0,            0.0,                 0.0),
     Quat( 1.0,                0.0,            0.0,                 0.0),
     Quat( 0.9997115343780182, 0.0,            0.02401766082591783, 0.0),
     Quat( 0.999173864575052,  0.0,            0.04063973855914903, 0.0),
     Quat( 1.0,                0.0,            0.0,                 0.0),
     Quat( 1.0,                0.0,            0.0,                 0.0),
}};

// §57 полуширина таза 0.080 м, полуширина плеч 0.140 м (Shoulder->UpperArm), эталонный рост 1.75 м (formules.txt)
inline constexpr float kHipHalfY      = 0.080f;
inline constexpr float kShoulderHalfY = 0.140f;
inline constexpr float kRefHeightM    = 1.75f;

struct JointDef {
    const char* label;
    int  parent;
    int  child;
};

// §137 22 сустава (label, родитель->потомок сегмент); сверено с FoxSkel.joints<N>.label/.indices (formules.txt)
inline constexpr std::array<JointDef, kJointCount> kJoints = {{
    { "jL5S1",            0,  1 },
    { "jL4L3",            1,  2 },
    { "jL1T12",           2,  3 },
    { "jT9T8",            3,  4 },
    { "jT1C7",            4,  5 },
    { "jC1Head",          5,  6 },
    { "jRightT4Shoulder", 4,  7 },
    { "jRightShoulder",   7,  8 },
    { "jRightElbow",      8,  9 },
    { "jRightWrist",      9, 10 },
    { "jLeftT4Shoulder",  4, 11 },
    { "jLeftShoulder",   11, 12 },
    { "jLeftElbow",      12, 13 },
    { "jLeftWrist",      13, 14 },
    { "jRightHip",        0, 15 },
    { "jRightKnee",      15, 16 },
    { "jRightAnkle",     16, 17 },
    { "jRightBallFoot",  17, 18 },
    { "jLeftHip",         0, 19 },
    { "jLeftKnee",       19, 20 },
    { "jLeftAnkle",      20, 21 },
    { "jLeftBallFoot",   21, 22 },
}};

struct LumpDef {
    const char* name;
    int kind;
    double sd;
};

// §1784/§138 7 lump-групп FoxFE (upperbody/ноги/стопы/руки), σ интегрирования 0.025 рад (formules.txt)
inline constexpr std::array<LumpDef, kLumpGroups> kLumps = {{
    { "upperbody", 1, 0.025 },
    { "rightleg",  2, 0.025 },
    { "rightfoot", 3, 0.025 },
    { "leftleg",   2, 0.025 },
    { "leftfoot",  3, 0.025 },
    { "rightarm",  4, 0.025 },
    { "leftarm",   4, 0.025 },
}};

// §1784 отображение сустав -> lump-группа FoxFE (formules.txt)
inline constexpr std::array<int, kJointCount> kJointLump = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    5,
    5,
    5,
    0,
    6,
    6,
    6,
    1,
    1,
    2,
    2,
    3,
    3,
    4,
    4,
};

// §1762/§1221-1223 наличие датчика на сегменте (конфигурация FullBody = 17 датчиков);
//   L5/L3/T12/Neck/носки — без своего датчика, ориентация интерполируется через c_spine/c_toes (formules.txt)
inline constexpr std::array<bool, kSegmentCount> kSensorPresent = {
     true,
     false,
     false,
     false,
     true,
     false,
     true,
     true,
     true,
     true,
     true,
     true,
     true,
     true,
     true,
     true,
     true,
     true,
     false,
     true,
     true,
     true,
     false,
};

// §24 калибровочные углы эталонных поз: cos/sin 5° (плечо 10°), √2/2=sin45° (рука 90°), cos/sin 6° (жен. таз 12°) (formules.txt)
constexpr double kCos5  = 0.99619469809174555;
constexpr double kSin5  = 0.087155742747658166;
constexpr double kSqrt2H = 0.7071067811865475;
constexpr double kCos6  = 0.9945219;
constexpr double kSin6  = 0.10452846;

Quat referenceQuat(int seg, Pose pose, Gender gender);

// §1663/§138 коэффициенты биомех-связей FoxFE (сверены с FOX_FE.bioMech.*):
//   c_spine §1098.8; c_pelvis=[0.35,25]; c_arms=[0.95,0.95,0.99]; c_legs=[0.9,0.95];
//   c_knees=[0.9,0.95]; c_ankles=[2,0.523599=30°,0.5,0] §48.1; c_toes §138.25 (formules.txt)
inline constexpr std::array<double, 9> kCSpine = {
    0.05, 0.45, 0.65, 0.85, 0.35, 0.9, 0.9, 0.9, 0.9
};

// проверено вручную: эвристический коэф. бедро-таз связи 0.12; в дампе xsb bioMech отсутствует (engine heuristic)
inline constexpr double kCFemoropelvic = 0.12;

inline constexpr std::array<double, 2> kCPelvis = { 0.35, 25.0 };

// проверено вручную: штраф латерального наклона таза 0.30 — engine heuristic, нет в xsb V[2]
inline constexpr double kPelvisLatTiltPenalty = 0.30;

inline constexpr std::array<double, 3> kCArms = { 0.95, 0.95, 0.99 };

inline constexpr std::array<double, 2> kCLegs  = { 0.9, 0.95 };

inline constexpr std::array<double, 2> kCKnees = { 0.9, 0.95 };

inline constexpr std::array<double, 4> kCAnkles = { 2.0, 0.523599, 0.5, 0.0 };

inline constexpr std::array<double, 6> kCToes =
    { 0.2, 1.05, -0.5, 1.0, 0.1, 0.0872 };

// §1784 lumpDef.A — масштаб шума интегрирования по активности: sub=1.0, jump1=0.9, jump2=[0.995,0.995,0.9995] (formules.txt)
inline constexpr std::array<double, 3> kALumpA_sub   = { 1.000, 1.000, 1.000  };
inline constexpr std::array<double, 3> kALumpA_jump1 = { 0.900, 0.900, 0.900  };
inline constexpr std::array<double, 3> kALumpA_jump2 = { 0.995, 0.995, 0.9995 };

// §XIX люфт сустава 0.005 рад; гиперэкстензия не допускается (max 0.0) (formules.txt)
inline constexpr double kJointLaxityRad     = 0.005;
inline constexpr double kHyperExtensionMax  = 0.0;

// §IX FoxCal.sd_theta_* — σ ориентации (рад) для веса МНК: pose=[1°,3°,2°], плечи/предплечье 6°/8° (formules.txt)
inline constexpr std::array<double, 3> kSdThetaPoseRad      = { 0.0174533, 0.0523599, 0.0349066 };
inline constexpr std::array<double, 3> kSdThetaShouldersRad = { 0.10472,   0.10472,   0.10472   };
inline constexpr std::array<double, 3> kSdThetaUpperArmRad  = { 0.10472,   0.139626,  0.10472   };
// §150 joint.stdHingeDegVec=[4°,2°] — σ шарнирных суставов (formules.txt)
inline constexpr std::array<double, 2> kStdHingeDeg         = { 4.0, 2.0 };
// §1630/§1930 init.floorLevel=1.25 м — стартовая высота пола при cold start (formules.txt)
inline constexpr double kFoxInitFloorLevelM = 1.25;

struct JointRom {
    double abdMin, abdMax;
    double flxMin, flxMax;
    double rotMin, rotMax;
};

// §XXIV/§XIX диапазоны движения ROM (abd/flx/rot, град) по 22 суставам; сверено: плечо §72 (flx -60..180),
//   локоть flx 0..150°, колено flx 0..135° (стр.1526-1529); shoulder/hip — конусное ROM (§ стр.1567) (formules.txt)
inline constexpr std::array<JointRom, kJointCount> kJointRom = {{
     {  -25.0,  25.0,   -30.0,  35.0,  -25.0, 25.0 },
     {  -20.0,  20.0,   -25.0,  30.0,  -20.0, 20.0 },
     {  -20.0,  20.0,   -25.0,  30.0,  -20.0, 20.0 },
     {  -20.0,  20.0,   -20.0,  25.0,  -25.0, 25.0 },
     {  -35.0,  35.0,   -50.0,  60.0,  -45.0, 45.0 },
     {  -25.0,  25.0,   -30.0,  30.0,  -30.0, 30.0 },
     {  -25.0,  25.0,   -25.0,  25.0,  -45.0, 45.0 },
     { -100.0, 180.0,   -60.0, 180.0,  -90.0, 90.0 },
     {   -2.0,   2.0,     0.0, 150.0,  -80.0, 80.0 },
     {  -30.0,  30.0,   -70.0,  80.0,  -25.0, 25.0 },
     {  -25.0,  25.0,   -25.0,  25.0,  -45.0, 45.0 },
     { -100.0, 180.0,   -60.0, 180.0,  -90.0, 90.0 },
     {   -2.0,   2.0,     0.0, 150.0,  -80.0, 80.0 },
     {  -30.0,  30.0,   -70.0,  80.0,  -25.0, 25.0 },
     {  -45.0,  45.0,   -30.0, 125.0,  -45.0, 45.0 },
     {   -2.0,   2.0,     0.0, 135.0,   -5.0,  5.0 },
     {  -25.0,  25.0,   -30.0,  20.0,  -20.0, 20.0 },
     {   -5.0,   5.0,   -30.0,  70.0,  -10.0, 10.0 },
     {  -45.0,  45.0,   -30.0, 125.0,  -45.0, 45.0 },
     {   -2.0,   2.0,     0.0, 135.0,   -5.0,  5.0 },
     {  -25.0,  25.0,   -30.0,  20.0,  -20.0, 20.0 },
     {   -5.0,   5.0,   -30.0,  70.0,  -10.0, 10.0 },
}};

// §1784/§1928 σ lump-группы 0.025 рад; жёсткость = 1/σ² (вес связи в МНК) (formules.txt)
inline constexpr double kSdLumpRad = 0.025;
inline constexpr double kLumpStiffness = 1.0 / (kSdLumpRad * kSdLumpRad);

// §1683 качество калибровки по остаточному углу residual: пороги 5/10/20/30° -> отлично/хорошо/удовл./плохо (formules.txt)
inline constexpr std::array<double, 4> kCalibQualityThresholdDeg = {
    5.0, 10.0, 20.0, 30.0
};
inline constexpr double kCalibStageDurationSec = 3.0;
inline constexpr int    kCalibQualityExcellent = 5;
inline constexpr int    kCalibQualityGood      = 4;
inline constexpr int    kCalibQualityAdequate  = 3;
inline constexpr int    kCalibQualityPoor      = 2;
inline constexpr int    kCalibQualityInvalid   = 1;

inline int calibrationQuality(double residualDeg)
{
    if (residualDeg < kCalibQualityThresholdDeg[0]) return kCalibQualityExcellent;
    if (residualDeg < kCalibQualityThresholdDeg[1]) return kCalibQualityGood;
    if (residualDeg < kCalibQualityThresholdDeg[2]) return kCalibQualityAdequate;
    if (residualDeg < kCalibQualityThresholdDeg[3]) return kCalibQualityPoor;
    return kCalibQualityInvalid;
}

inline const char* calibrationQualityLabel(int q)
{
    switch (q) {
        case kCalibQualityExcellent: return "excellent";
        case kCalibQualityGood:      return "good";
        case kCalibQualityAdequate:  return "adequate";
        case kCalibQualityPoor:      return "poor";
        case kCalibQualityInvalid:   return "invalid";
        default:                     return "unknown";
    }
}

// §XI/§IX skinArtifact + sensor.states init-σ: ориентация тело-сенсор 45° (до калибровки) -> 1.5°,
//   нижняя граница 0.3°; позиция 0.01 -> пол 0.004 м (formules.txt)
inline constexpr double kCalibInitStdOriBodyDeg          = 45.0;
inline constexpr double kCalibInitStdSensorToBodyDeg     = 1.5;
inline constexpr double kCalibStdSensorToBodyOriFloorDeg = 0.3;
inline constexpr double kCalibInitStdSensorToBodyPos     = 0.01;
inline constexpr double kCalibStdSensorToBodyPosFloor    = 0.004;

// §XIV сегменты, опирающиеся на пол при калибровке: RFoot(17), LFoot(21) (formules.txt)
inline constexpr bool kCalibSegmentOnFloor(int seg)
{

    return (seg == 17) || (seg == 21);
}

struct DimDef {
    const char* name;
    double sd_dim;
};

// §2006 σ измерения антропом. размеров (FoxCal.<dim>.sd_dim, вес в МНК): bodyHeight 0.0005,
//   footSize 0.001, ankle/knee/hip/shoulder/span 0.01..0.05, armSpan 0.05; сверено по FoxCal (formules.txt)
inline constexpr std::array<DimDef, 12> kDimensions = {{
    { "bodyHeight",     0.0005   },
    { "footSize",       0.001    },
    { "footFloor",      1.0e-7   },
    { "ankleHeight",    0.01     },
    { "kneeHeight",     0.02     },
    { "hipHeight",      0.03     },
    { "shoulderHeight", 0.03     },
    { "shoulderWidth",  0.02     },
    { "hipWidth",       0.02     },
    { "elbowSpan",      0.02     },
    { "wristSpan",      0.02     },
    { "armSpan",        0.05     },
}};

struct ContactRow {
    int    seg;
    int    pt;
    double th1, th2, th3, th4;
};

// §137 порядок сегментов модели (0-based в коде; в formules.txt нумерация 1-based: seg N <-> код N-1) (formules.txt)
inline constexpr int kSEG_Pelvis    = 0;
inline constexpr int kSEG_L5        = 1;
inline constexpr int kSEG_L3        = 2;
inline constexpr int kSEG_T12       = 3;
inline constexpr int kSEG_T8        = 4;
inline constexpr int kSEG_Neck      = 5;
inline constexpr int kSEG_Head      = 6;
inline constexpr int kSEG_RShoulder = 7;
inline constexpr int kSEG_RUpperArm = 8;
inline constexpr int kSEG_RForearm  = 9;
inline constexpr int kSEG_RHand     = 10;
inline constexpr int kSEG_LShoulder = 11;
inline constexpr int kSEG_LUpperArm = 12;
inline constexpr int kSEG_LForearm  = 13;
inline constexpr int kSEG_LHand     = 14;
inline constexpr int kSEG_RUpperLeg = 15;
inline constexpr int kSEG_RLowerLeg = 16;
inline constexpr int kSEG_RFoot     = 17;
inline constexpr int kSEG_RToe      = 18;
inline constexpr int kSEG_LUpperLeg = 19;
inline constexpr int kSEG_LLowerLeg = 20;
inline constexpr int kSEG_LFoot     = 21;
inline constexpr int kSEG_LToe      = 22;

// §138.16 контактные точки стоп IcontactsConsidered M[26,6] (пороги th1..th4): RFoot/RToe/LFoot/LToe +
//   нижние точки голеней; значения 0.05/0.25/0.40 и 0.08/1.0 сверены с xsb (formules.txt)
inline constexpr std::array<ContactRow, 12> kFootContacts = {{

    { kSEG_RFoot,     3, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_RFoot,     4, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_RFoot,     5, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_RFoot,     6, 0.05, 0.20, 0.20, 0.40 },

    { kSEG_RToe,      2, 0.05, 0.20, 0.20, 0.40 },

    { kSEG_LFoot,     3, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_LFoot,     4, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_LFoot,     5, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_LFoot,     6, 0.05, 0.20, 0.20, 0.40 },
    { kSEG_LToe,      2, 0.05, 0.20, 0.20, 0.40 },

    { kSEG_RLowerLeg, 5, 0.08, 1.0,  1.0,  0.40 },
    { kSEG_LLowerLeg, 5, 0.08, 1.0,  1.0,  0.40 },
}};

// §XIV stdHeightMeas (σ измерения высоты контакта): default 0.002 м; таз/T8/плечи/бёдра 0.03; предплечья/голени 0.005 (formules.txt)
inline constexpr double kStdHeightMeasDefault = 0.002;
inline double stdHeightMeasFor(int seg)
{
    switch (seg) {
        case 0:  return 0.03;
        case 4:  return 0.03;
        case 7:  return 0.03;
        case 11: return 0.03;
        case 15: return 0.03;
        case 19: return 0.03;
        case 9:  return 0.005;
        case 13: return 0.005;
        case 16: return 0.005;
        case 20: return 0.005;
        default: return kStdHeightMeasDefault;
    }
}

struct ContactParams {
    double dLevelDefault;
    double dLevelFoot;
    bool   enableImpactDetection;
    double impactTh;
    double impactWinDuration;
    int    maxDetectedContacts;
    double minimumAcceptableMeasure;
    double sameHeightTh;
    double secondaryPelvisT8RejMinDeg;
    double secondaryPelvisT8RejMaxDeg;
    double firstWinWidth;
    double firstWinWidthHighVel;
    double highVelTh;
    double secondWinWidthBefore;
    double secondWinWidthAfter;
};
// §138/§XIV FOX_FE.contactParameters + contactHandling: уровни dLevel 0.175/0.10 м, импакт-порог 15,
//   окно импакта 1 с, maxDetectedContacts=4, sameHeightTh 0.0015 м (formules.txt)
inline constexpr ContactParams kContact = {
    .dLevelDefault             = 0.175,
    .dLevelFoot                = 0.10,
    .enableImpactDetection     = true,
    .impactTh                  = 15.0,
    .impactWinDuration         = 1.0,
    .maxDetectedContacts       = 4,
    .minimumAcceptableMeasure  = 0.001,
    .sameHeightTh              = 0.0015,
    .secondaryPelvisT8RejMinDeg = 40.0,
    .secondaryPelvisT8RejMaxDeg = 140.0,
    .firstWinWidth             = 0.15,
    .firstWinWidthHighVel      = 0.085,
    .highVelTh                 = 0.8,
    .secondWinWidthBefore      = 0.01,
    .secondWinWidthAfter       = 0.01,
};

// §XXI/§155 кисть: 20 сегментов пальцев, 17 датчиков перчатки на руку; rightHand.jointsCount=19 (formules.txt)
inline constexpr int kFingerSensorsPerHand = 17;
inline constexpr int kFingerSegmentsPerHand = 20;

// §XXI/§91 число костей на палец: большой=3, остальные=4 (formules.txt)
inline constexpr std::array<int, 5> kFingerBoneCount = { 3, 4, 4, 4, 4 };



// §XXI q_bs пальцев = тождественный (leftHand/rightHand.segments.sensor.q_bs=[1,0,0,0]); сверено с xsb (formules.txt)
inline constexpr std::array<Quat, kFingerSegmentsPerHand> kFingerQBSRight = {{
    Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0),
    Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0),
    Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0),
    Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0),
    Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0),
}};

inline constexpr std::array<Quat, kFingerSegmentsPerHand> kFingerQBSLeft = {{
    Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0),
    Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0),
    Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0),
    Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0),
    Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0), Quat(1, 0, 0, 0),
}};

struct AnthroFloors {
    double armSpanMin;
    double legLengthMin;
    double trunkLengthMin;
    double hipHalfMin;
    double scapHalfMin;
};
// §57 нижние границы антропометрии (защита масштабирования от вырожденных размеров): руки/ноги>=0.30, торс>=0.40 м (formules.txt)
inline constexpr AnthroFloors kAnthroFloors = {
    .armSpanMin     = 0.30,
    .legLengthMin   = 0.30,
    .trunkLengthMin = 0.40,
    .hipHalfMin     = 0.04,
    .scapHalfMin    = 0.05,
};

struct JumpDetectParams {
    double threshDeg;
    double blendRangeDeg;
    double gyroQuietDegS;
};
// §1543/§29 детектор прыжка (классификация активности): порог наклона 20°, блендинг 15°, «тихий» гироскоп 25°/с (formules.txt)
inline constexpr JumpDetectParams kJumpDetect = {
    .threshDeg     = 20.0,
    .blendRangeDeg = 15.0,
    .gyroQuietDegS = 25.0,
};

struct FingerSmoothParams {
    float emaAlphaThumb;
    float emaAlphaFinger;
    float outlierAlphaThumb;
    float outlierAlphaFinger;
    float outlierThreshThumbDeg;
    float outlierThreshFingerDeg;
};
// §XXI сглаживание пальцев перчатки: EMA α (большой 0.15, прочие 0.35), фильтр выбросов 0.04/0.10 при >15°/30° (formules.txt)
inline constexpr FingerSmoothParams kFingerSmooth = {
    .emaAlphaThumb          = 0.15f,
    .emaAlphaFinger         = 0.35f,
    .outlierAlphaThumb      = 0.04f,
    .outlierAlphaFinger     = 0.10f,
    .outlierThreshThumbDeg  = 15.0f,
    .outlierThreshFingerDeg = 30.0f,
};

struct GaitParams {
    double flightSecForRun;
    double standingPelvisSpeedMax;
    double sittingDoubleSupportFrac;
    double pitchHeelRad;
    double pitchToeRad;
    double velGround;
    double velFlight;
    double ffHoldSec;
    double acrobaticTiltDeg;
};
// §49 цикл переката стопы HS->FF->HO->TO->SW (ZUPT пятка->носок) + §29/§1543 классификация активности:
//   полёт>0.05с -> бег; скорость таза<0.30 м/с -> стояние; double-support 0.10 (§стр.46467); наклон таза/T8>90° -> акробатика;
//   pitchHeel/Toe ±0.10 рад, velGround 0.05 / velFlight 0.30 м/с — пороги событий походки (formules.txt)
inline constexpr GaitParams kGait = {
    .flightSecForRun          = 0.05,
    .standingPelvisSpeedMax   = 0.30,
    .sittingDoubleSupportFrac = 0.10,
    .pitchHeelRad             = -0.10,
    .pitchToeRad              =  0.10,
    .velGround                =  0.05,
    .velFlight                =  0.30,
    .ffHoldSec                =  0.05,
    .acrobaticTiltDeg         = 90.0,
};

struct ZuptThresholds {
    double th1;
    double th2;
    double th3;
    double th4;
    double th2Toe;
    double th3Toe;
    double th1Knee;
    double weightTh3;
    double weightTh4;
};
// §52/§138.16 пороги детектора ZUPT (нулевая скорость) = контактные th1..th4: 0.05/0.25/0.25/0.40 м;
//   носок th2/th3=0.20; колено th1=0.08; веса связи 0.7/1.0 (formules.txt)
inline constexpr ZuptThresholds kZuptTh = {
    .th1       = 0.05,
    .th2       = 0.25,
    .th3       = 0.25,
    .th4       = 0.40,
    .th2Toe    = 0.20,
    .th3Toe    = 0.20,
    .th1Knee   = 0.08,
    .weightTh3 = 0.7,
    .weightTh4 = 1.0,
};

struct MagnetParams {
    double declinationDeg;
    double inclinationDeg;
    double inclinationDeg2;

    double inclinationDipRad;
    double normReference;
    double angleDiffFromModelMaxDeg;
    double dipDiffFromModelMaxDeg;
    double normDiffFromModelMax;
    double magResThreshold;
    double magResTimeUpSec;
    double magResTimeDownSec;
};
// §XII магнитная модель (sensor.magModel + FoxCal.e_dip_mag): dip -1.1750679 рад = -67.3°, |наклонение| 78°;
//   gates angleDiff 6°, dipDiff 3.5°, normDiff 0.03; порог magRes 0.9, tau вверх/вниз 0.6/3.0 с (formules.txt)
inline constexpr MagnetParams kMagnet = {
    .declinationDeg            = 0.0,
    .inclinationDeg            = 78.0,
    .inclinationDeg2           = 85.0,
    .inclinationDipRad         = -1.1750679,
    .normReference             = 1.0,
    .angleDiffFromModelMaxDeg  = 6.0,
    .dipDiffFromModelMaxDeg    = 3.5,
    .normDiffFromModelMax      = 0.03,
    .magResThreshold           = 0.9,
    .magResTimeUpSec           = 0.6,
    .magResTimeDownSec         = 3.0,
};

// §XII опорный вектор магнитного поля m0 из угла наклонения (dip) и склонения (declination) (formules.txt)
inline QVector3D referenceM0Vec(double dipDeg, double declinationDeg) {

    constexpr double kD2R = 0.017453292519943295;
    const double dipR = dipDeg * kD2R;
    const double decR = declinationDeg * kD2R;
    const double cI = std::cos(dipR);
    const double sI = std::sin(dipR);
    return QVector3D(float(cI * std::cos(decR)),
                     float(-cI * std::sin(decR)),
                     float(sI));
}

inline QVector3D referenceM0BodyVec() {
    return referenceM0Vec(-kMagnet.inclinationDeg, kMagnet.declinationDeg);
}

inline QVector3D referenceM0FreeFieldVec() {
    constexpr double kR2D = 57.29577951308232;
    return referenceM0Vec(kMagnet.inclinationDipRad * kR2D,
                          kMagnet.declinationDeg);
}

struct MagGateRelax {
    float angleMul;
    float dipMul;
    float normMul;
};

// §XII пер-сегментное ослабление магнитных gate (руки/кисти доверяют магнитометру меньше -> множители до 5x) (formules.txt)
inline constexpr std::array<MagGateRelax, kSegmentCount> kMagGateRelax = {{
     { 1.5f, 1.3f, 1.0f + 0.20f },
     { 1.0f, 1.0f, 1.0f },
     { 1.0f, 1.0f, 1.0f },
     { 1.0f, 1.0f, 1.0f },
     { 1.5f, 1.3f, 1.0f },
     { 1.0f, 1.0f, 1.0f },
     { 4.0f, 2.5f, 1.0f + 0.30f },
     { 1.0f, 1.0f, 1.0f },
     { 5.0f, 5.0f, 1.0f },
     { 5.0f, 5.0f, 1.0f },
     { 5.0f, 5.0f, 1.0f + 0.35f },
     { 1.0f, 1.0f, 1.0f },
     { 5.0f, 5.0f, 1.0f },
     { 5.0f, 5.0f, 1.0f },
     { 5.0f, 5.0f, 1.0f + 0.35f },
     { 1.0f, 1.0f, 1.0f },
     { 1.5f, 1.5f, 1.0f },
     { 2.0f, 2.0f, 1.0f + 0.22f },
     { 1.0f, 1.0f, 1.0f },
     { 1.0f, 1.0f, 1.0f },
     { 1.5f, 1.5f, 1.0f },
     { 2.0f, 2.0f, 1.0f + 0.22f },
     { 1.0f, 1.0f, 1.0f },
}};

enum class ImuChipType : std::uint8_t { W2, X2, X3 };

// §XXXIII тип IMU-чипа на сегмент (W2/X2/X3); базовая конфигурация — все W2 (formules.txt)
inline constexpr std::array<ImuChipType, kSegmentCount> kImuChipPerSeg = {{

    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
}};

// §XXXIII масштаб шума магнитометра по чипу: X3 = 7.91 = sigmaMag(X3 0.2215)/sigmaMag(W2 0.028) (formules.txt)
inline float magNoiseScaleForChip(ImuChipType c) {
    switch (c) {
        case ImuChipType::X3: return 7.91f;
        case ImuChipType::W2:
        case ImuChipType::X2:
        default: return 1.0f;
    }
}

struct ImuChipNoise {
    float sigmaAccMs2;
    float sigmaGyrDegS;
    float sigmaMagNorm;
    float dynRangeAccMs2;
    float dynRangeGyrDegS;
    float gainErrorAcc;
    float gainErrorGyr;
};
// §XXXIII шумы IMU-чипов (FoxHW.FoxIMU_*): σ acc/gyr/mag, dynRange acc 157 м/с² (±16g), gyr 2000°/с,
//   gainError 0.004; X3 точнее W2/X2 (acc 0.00899, gyr 0.075). Сверено с FoxHW дампом (formules.txt)
inline constexpr ImuChipNoise kImuChipNoiseW2 = {
    .sigmaAccMs2     = 0.0232f,
    .sigmaGyrDegS    = 0.20f,
    .sigmaMagNorm    = 0.028f,
    .dynRangeAccMs2  = 157.0f,
    .dynRangeGyrDegS = 2000.0f,
    .gainErrorAcc    = 0.004f,
    .gainErrorGyr    = 0.004f,
};
inline constexpr ImuChipNoise kImuChipNoiseX2 = {
    .sigmaAccMs2     = 0.0232f,
    .sigmaGyrDegS    = 0.20f,
    .sigmaMagNorm    = 0.028f,
    .dynRangeAccMs2  = 157.0f,
    .dynRangeGyrDegS = 2000.0f,
    .gainErrorAcc    = 0.004f,
    .gainErrorGyr    = 0.004f,
};
inline constexpr ImuChipNoise kImuChipNoiseX3 = {
    .sigmaAccMs2     = 0.00899f,
    .sigmaGyrDegS    = 0.075f,
    .sigmaMagNorm    = 0.2215f,
    .dynRangeAccMs2  = 157.0f,
    .dynRangeGyrDegS = 2000.0f,
    .gainErrorAcc    = 0.004f,
    .gainErrorGyr    = 0.004f,
};
inline constexpr const ImuChipNoise& chipNoiseFor(ImuChipType c) {
    switch (c) {
        case ImuChipType::X3: return kImuChipNoiseX3;
        case ImuChipType::X2: return kImuChipNoiseX2;
        case ImuChipType::W2:
        default:              return kImuChipNoiseW2;
    }
}

struct SkinParams {
    double tauSec;
    double sigmaOriDeg;
    double sigmaPosM;
    double sigmaOriGmDeg;
    double sigmaPosGmM;
    double initStdOriBodyDeg;
    double initStdSensorToBodyDeg;
    double initStdSensorToBodyPos;
    double stdSensorToBodyOriFloorDeg;
    double stdSensorToBodyPosFloor;
    bool   doGaussMarkov;
    bool   doChangeTauInCF;
    bool   doSkinArtifactBasedOnDynamics;

    double tauFastSec;
    double tauSlowSec;
    double tauMotionRefRad;
    double linAccRefMps2;
};
// §XI skin-артефакт (мягкие ткани, Гаусс-Марков): skinArtifact.tau=0.15 с, σ ориент. 3°/2.5°, σ позиции 0.02/0.025 м;
//   init-σ тело-сенсор 45°->1.5°, пол 0.3°/0.004 м; tau быстрая/медленная 0.05/0.30 с (formules.txt)
inline constexpr SkinParams kSkin = {
    .tauSec                       = 0.15,
    .sigmaOriDeg                  = 3.0,
    .sigmaPosM                    = 0.02,
    .sigmaOriGmDeg                = 2.5,
    .sigmaPosGmM                  = 0.025,
    .initStdOriBodyDeg            = 45.0,
    .initStdSensorToBodyDeg       = 1.5,
    .initStdSensorToBodyPos       = 0.01,
    .stdSensorToBodyOriFloorDeg   = 0.3,
    .stdSensorToBodyPosFloor      = 0.004,
    .doGaussMarkov                = true,
    .doChangeTauInCF              = false,
    .doSkinArtifactBasedOnDynamics= false,
    .tauFastSec                   = 0.05,
    .tauSlowSec                   = 0.30,
    .tauMotionRefRad              = 1.0,
    .linAccRefMps2                = 1.0,
};

struct FilterParams {
    double tauAcc;
    double tauFGyrLpfDynamic;
    double tauM0AvgFast;
    double tauM0AvgMedium;
    double tauM0AvgSlow;
};
// §XII/§XXV постоянные времени фильтров: LPA ускорения 10 с, дин. LPF гироскопа 6 с;
//   усреднение опорного магнит. поля m0 — быстрое/среднее/медленное 30/120/300 с по динамике (formules.txt)
inline constexpr FilterParams kFilter = {
    .tauAcc            = 10.0,
    .tauFGyrLpfDynamic = 6.0,
    .tauM0AvgFast      = 30.0,
    .tauM0AvgMedium    = 120.0,
    .tauM0AvgSlow      = 300.0,
};

struct EstimatorWeights {

    std::array<double, kLumpGroups> sdIntAccToVel;
    double sdIntVelToPos;
    double sdLumpJoint;
    double stdOriFreeze;
    double stdPosFreeze;
    double stdOriLocalBodyStillDeg;
    double initStdVel;
    double initStdAccBias;
    double initStdGyrBiasDeg;
    double gyrBiasStdMinDeg;
    double gyrBiasStdMaxDeg;
    double multiLevelZhcClipVert;
};
// §1928/§X шумы оценщика FoxFE: sd_int_acc_to_vel по 7 lump-группам (X-столбец M[7,3] = [2,2,1,4,4,2,8]);
//   sd_int_vel_to_pos 1e-5, freeze-σ ориент/позиции, init-σ скорости/смещений, gyr bias 0.005..0.07°/с (formules.txt)
inline constexpr EstimatorWeights kEstimator = {
    .sdIntAccToVel  = {{ 2.0, 2.0, 1.0, 4.0, 4.0, 2.0, 8.0 }},
    .sdIntVelToPos  = 1.0e-5,
    .sdLumpJoint    = 0.025,
    .stdOriFreeze   = 0.01,
    .stdPosFreeze   = 1.0e-4,
    .stdOriLocalBodyStillDeg = 1.5,
    .initStdVel     = 2.0,
    .initStdAccBias = 0.1,
    .initStdGyrBiasDeg = 0.4,
    .gyrBiasStdMinDeg  = 0.005,
    .gyrBiasStdMaxDeg  = 0.07,
    .multiLevelZhcClipVert = 0.005,
};

struct AidingBiasParams {
    double cT;
    double cV;
};
// §VII смещения подсказок aiding: cT 0.9 (временное), cV 0.01 (formules.txt)
inline constexpr AidingBiasParams kAidingBias = {
    .cT = 0.9,
    .cV = 0.01,
};

// §1928 FoxFE.sd_int_acc_to_vel = M[7,3] (lump-группы × оси XYZ): [[2,2,1],[4,4,2],[8,8,4],...,[20,20,20]] (formules.txt)
inline constexpr std::array<std::array<double, 3>, kLumpGroups> kSdIntAccToVelXYZ = {{
    {{  2.0,  2.0,  1.0 }},
    {{  4.0,  4.0,  2.0 }},
    {{  8.0,  8.0,  4.0 }},
    {{  4.0,  4.0,  2.0 }},
    {{  8.0,  8.0,  4.0 }},
    {{ 20.0, 20.0, 20.0 }},
    {{ 20.0, 20.0, 20.0 }},
}};

// §X веса МНК: σ длины кости 0.0002 м; σ совмещения точек XY 0.0003, Z(3D) 0.002, Z(2D) 10 м (formules.txt)
inline constexpr double kStdJointBoneLength = 0.0002;

inline constexpr double kStdSamePosMeasXY   = 0.0003;

inline constexpr double kStdSamePosMeasZ3d  = 0.002;
inline constexpr double kStdSamePosMeasZ    = 10.0;

// §241/§V решатель IK: до 5 шагов; толеранс градиента 1e-4 рад, шага 1e-5; люфт сустава 0.005; штраф гиперэкст. σ 0.0002 (formules.txt)
inline constexpr int    kMaxIKSteps         = 5;
inline constexpr double kIKGradTolRad       = 1.0e-4;
inline constexpr double kIKStepTolRad       = 1.0e-5;
inline constexpr double kJointLaxitySolver  = 0.005;
inline constexpr double kHypExtPenaltySd    = 0.0002;

// §138/§XIV уровни контакта: по умолчанию 0.175 м, для стопы 0.10 м (formules.txt)
inline constexpr double kDLevelDefault      = 0.175;
inline constexpr double kDLevelFoot         = 0.10;

struct SpineNeckParams {
    double cL3;
    double cL5;
    double cT12;
    double cNeck;
    double stdNeck;
    double stdSpine;
};
// §1098.8 связь спины/шеи (подмножество c_spine): cL5 0.45, cL3 0.65, cT12 0.85, cNeck 0.35; σ 0.001 (formules.txt)
inline constexpr SpineNeckParams kSpineNeck = {
    .cL3      = 0.65,
    .cL5      = 0.45,
    .cT12     = 0.85,
    .cNeck    = 0.35,
    .stdNeck  = 0.001,
    .stdSpine = 0.001,
};

// §46/§1114 лопаточно-плечевой ритм (scapulohumeral): активация при угле плеча 60°..90° (formules.txt)
inline constexpr double kScapHumThetaLowDeg  = 60.0;
inline constexpr double kScapHumThetaHighDeg = 90.0;

// §1300 колено screw-home: винтовая ротация = c_knees[1]*(1-cos(flex))*(15°·π/180) (formules.txt)
inline constexpr double kKneeScrewMaxDeg     = 15.0;

// §48 предел тыльного сгибания голеностопа 0.785398 рад = π/4 = 45° (formules.txt)
inline constexpr double kAnkleDorsiLimitRad  = 0.785398;

// §49 диапазон переката носка (toe rocker): 5°..30° (formules.txt)
inline constexpr double kToeRockerLowRad     = 5.0  * 0.017453292519943295;
inline constexpr double kToeRockerHighRad    = 30.0 * 0.017453292519943295;

// §138/§228 FoxFE.contactParameters.* (сверено с xsb-дампом): air/com/acc/vel/general/peakDetection/
//   level/samepos/boost/pos — параметры вероятностной модели детектора контакта стопы с полом (formules.txt)
inline constexpr std::array<double, 11> kAir = {
    -3.0, 0.0, 0.3, 3.0, 0.1, -0.05, 0.3, 0.0, -6.0, -0.05, -0.02
};

inline constexpr std::array<double, 5> kCom = { 0.05, 0.5, 0.08, 0.0, 0.6 };

inline constexpr std::array<double, 4> kAccProb = { 0.6, 0.3, 0.95, 0.4 };

inline constexpr std::array<double, 4> kVelProb = { 0.1, 0.4, 0.95, 0.2 };

inline constexpr std::array<double, 4> kGeneralProb = { 0.5, 0.4, 0.275, 0.0015 };

inline constexpr std::array<double, 7> kPeakDetection = {
    3.6515, 0.1667, 20.0, 10.0, 1.5, 1.0, 0.025
};

struct ContactWeights {
    double wAcc;
    double wVel;
    double wCom;
    double wAir;
    double wGeneral;
    double wLevel;
    double wBoost;
    double wPos;
    double wPeakDetection;
    double wSamepos;
    double bias;
};

// §XIV веса слагаемых детектора контакта (acc/vel/com/air/general/level/boost/pos/peak/samepos) — все 1.0 (formules.txt)
inline constexpr ContactWeights kContactWeights = {
    .wAcc           = 1.0,
    .wVel           = 1.0,
    .wCom           = 1.0,
    .wAir           = 1.0,
    .wGeneral       = 1.0,
    .wLevel         = 1.0,
    .wBoost         = 1.0,
    .wPos           = 1.0,
    .wPeakDetection = 1.0,
    .wSamepos       = 1.0,
    .bias           = 0.0,
};

inline constexpr std::array<double, 2> kLevelProb = { 0.1, 0.0 };

inline constexpr std::array<double, 8> kSamepos = {
    0.02, 0.02, 0.07, 0.05, 0.005, 0.005, 0.15, 0.8
};

inline constexpr std::array<double, 2> kBoost = { 2.0, 4.0 };

inline constexpr std::array<double, 3> kPos = { 0.12, 0.0, 0.0 };

struct OutlierRej {
    double outRejTh1;
    double outRejTh2;
    double outRejTh3;
    double jointResTh[6];
    double footSlidingTh[3];
    double countTh[3];
    double jointResWin1;
    double jointResWin2;
};
// §XV отбраковка выбросов (марг./RTS): пороги невязок 100/10/2.5, per-joint residual, скольжение стопы, окна 0.1/0.05 (formules.txt)
inline constexpr OutlierRej kOutlierRej = {
    .outRejTh1    = 100.0,
    .outRejTh2    = 10.0,
    .outRejTh3    = 2.5,
    .jointResTh   = { 0.000625, 0.005, 0.035, 0.002, 0.0075, 0.000625 },
    .footSlidingTh= { 0.3, 0.07, 0.9 },
    .countTh      = { 0.2, 0.4, 0.4 },
    .jointResWin1 = 0.1,
    .jointResWin2 = 0.05,
};

struct MultiLevelParams {
    int    averagingStairHeight;
    double maxDevFromAvgStairHeight;
    int    maxLevelsToDetectStairWalking;
    double newLevelMeasureThreshold;
    double sameLevelMargin;
    double zhcClipSdVertical;
    double zhcLevelMargin1;
    double zhcLevelMargin2;
    double zhcMaximumLikelihood;
    double zhcMaxLikelihoodBoost;
    double zhcMinimumLikelihood;
    double tauSmoothSec;
    double cdSameSegmentBonus;
    double cdSameSegmentPenalty;
    double zhSameSegmentBonus;
    double zhSameSegmentPenalty;
};
// §XIV multiLevel (ходьба по лестнице/уровням): усреднение высоты ступени 5, новый уровень 0.03 м,
//   запас уровня 0.09 м, ZHC (zero-height-contact) likelihood 0.4..0.9, boost 1000, tau сглаж. 0.2 с (formules.txt)
inline constexpr MultiLevelParams kMultiLevel = {
    .averagingStairHeight          = 5,
    .maxDevFromAvgStairHeight      = 0.05,
    .maxLevelsToDetectStairWalking = 10,
    .newLevelMeasureThreshold      = 0.03,
    .sameLevelMargin               = 0.09,
    .zhcClipSdVertical             = 0.005,
    .zhcLevelMargin1               = 0.12,
    .zhcLevelMargin2               = 0.09,
    .zhcMaximumLikelihood          = 0.9,
    .zhcMaxLikelihoodBoost         = 1000.0,
    .zhcMinimumLikelihood          = 0.4,
    .tauSmoothSec                  = 0.2,
    .cdSameSegmentBonus            = 0.03,
    .cdSameSegmentPenalty          = -0.02,
    .zhSameSegmentBonus            = 0.05,
    .zhSameSegmentPenalty          = -1.0,
};

struct CalibMagE {
    double e_dip_mag;
    double e_dip_mag2;
    double e_incl_arm;
    double e_inclx_pelvis;
    double e_inclx_sternum;
    double e_incly_pelvis;
    double e_mag_diff;
    double e_mag_feet;
    double e_mag_pelvis;
    double e_mag_pelvis2;
    double e_norm_hands;
    double e_norm_head;
    double e_norm_pelvis;
    double e_norm_feet;
    double e_sternum_pelvis;
};
// §24/FoxCal.e_* пороги калибровки по магнитному полю: dip 78°/85°, наклонения рук/таза/грудины 30/45/40/25°,
//   нормы поля рук/головы/таза/стоп и различия; сверено с FoxCal-дампом (formules.txt)
inline constexpr CalibMagE kCalibMagE = {
    .e_dip_mag        = 78.0,
    .e_dip_mag2       = 85.0,
    .e_incl_arm       = 30.0,
    .e_inclx_pelvis   = 45.0,
    .e_inclx_sternum  = 40.0,
    .e_incly_pelvis   = 25.0,
    .e_mag_diff       = 0.5,
    .e_mag_feet       = 2.2,
    .e_mag_pelvis     = 1.5,
    .e_mag_pelvis2    = 0.5,
    .e_norm_hands     = 0.35,
    .e_norm_head      = 0.3,
    .e_norm_pelvis    = 0.2,
    .e_norm_feet      = 0.22,
    .e_sternum_pelvis = 25.0,
};

struct FootPoint {
    int       pointId;
    QVector3D r_local;
};
// §90/§138.16 контактные точки стопы (локальн. коорд., м): 3=pHeel(-0.036,0,-0.08), 4=pFirstMet, 5=pFifthMet, 6=ball; сверено с xsb (formules.txt)
inline constexpr std::array<FootPoint, 4> kFootPointsRight = {{
    { 3, QVector3D(-0.036f,  0.000f, -0.080f) },
    { 4, QVector3D( 0.116f, -0.038f, -0.080f) },
    { 5, QVector3D( 0.116f,  0.036f, -0.080f) },
    { 6, QVector3D( 0.140f,  0.000f, -0.080f) },
}};
inline constexpr std::array<FootPoint, 4> kFootPointsLeft = {{
    { 3, QVector3D(-0.036f,  0.000f, -0.080f) },
    { 4, QVector3D( 0.116f,  0.038f, -0.080f) },
    { 5, QVector3D( 0.116f, -0.036f, -0.080f) },
    { 6, QVector3D( 0.140f,  0.000f, -0.080f) },
}};

// §90 анатомические anchor-точки сегментов (носок, перед колена, SIPS таза, центр ягодицы) для контактов/маркеров (formules.txt)
inline constexpr QVector3D kToeTipPoint = QVector3D(0.064f, 0.0f, -0.015f);

inline constexpr QVector3D kKneeFrontPointR = QVector3D(0.040f,  0.000f, -0.050f);
inline constexpr QVector3D kKneeFrontPointL = QVector3D(0.040f,  0.000f, -0.050f);

inline constexpr QVector3D kPelvisSIPSRight = QVector3D(-0.0407f, -0.050f,  0.0951f);
inline constexpr QVector3D kPelvisSIPSLeft  = QVector3D(-0.0407f,  0.050f,  0.0951f);

inline constexpr QVector3D kPelvisCentralButtock = QVector3D(-0.045f, 0.0f, -0.030f);

struct FingerRom {
    const char* label;
    double flxMin, flxMax;
    double abdMin, abdMax;
    double rotMin, rotMax;
};
// §91 ROM 19 суставов кисти (rightHand.joints, ballandsocket с огранич. DOF, град): большой TM_CMC/MCP/IP,
//   указат..мизинец MCP/PIP/DIP, Carpus/OPP/Spread/RSV; flx/abd/rot пределы (formules.txt)
inline constexpr std::array<FingerRom, 19> kFingerRom = {{
     { "TM_CMC",   -15.0,  60.0,  -45.0,  45.0,  -20.0, 30.0 },
     { "TM_MCP",     0.0,  60.0,   -5.0,   5.0,    0.0,  0.0 },
     { "TM_IP",    -10.0,  80.0,    0.0,   0.0,    0.0,  0.0 },
     { "II_MCP",   -20.0,  90.0,  -25.0,  25.0,    0.0,  0.0 },
     { "II_PIP",     0.0, 110.0,    0.0,   0.0,    0.0,  0.0 },
     { "II_DIP",     0.0,  80.0,    0.0,   0.0,    0.0,  0.0 },
     { "MI_MCP",   -20.0,  90.0,  -25.0,  25.0,    0.0,  0.0 },
     { "MI_PIP",     0.0, 110.0,    0.0,   0.0,    0.0,  0.0 },
     { "MI_DIP",     0.0,  80.0,    0.0,   0.0,    0.0,  0.0 },
     { "RI_MCP",   -20.0,  90.0,  -25.0,  25.0,    0.0,  0.0 },
     { "RI_PIP",     0.0, 110.0,    0.0,   0.0,    0.0,  0.0 },
     { "RI_DIP",     0.0,  80.0,    0.0,   0.0,    0.0,  0.0 },
     { "LI_MCP",   -20.0,  90.0,  -30.0,  30.0,    0.0,  0.0 },
     { "LI_PIP",     0.0, 110.0,    0.0,   0.0,    0.0,  0.0 },
     { "LI_DIP",     0.0,  80.0,    0.0,   0.0,    0.0,  0.0 },
     { "Carpus",   -10.0,  10.0,    0.0,   0.0,    0.0,  0.0 },
     { "TM_OPP",     0.0,  45.0,    0.0,   0.0,    0.0,  0.0 },
     { "Spread",   -20.0,  20.0,    0.0,   0.0,    0.0,  0.0 },
     { "RSV",       -5.0,   5.0,    0.0,   0.0,    0.0,  0.0 },
}};

// §XXI FoxKF.gloveBase.* — конфиг калмановского фильтра перчатки (FOX_KFA-Core): магнитная модель,
//   ZRU, acc/gyr boost/clip, redefine курса, skin/bias; значения в kGloveBase ниже сверены с xsb (formules.txt)
struct KfaGloveParams {

    double accDivMonTauDecay;
    double accDivMonThreshold;
    double accDivMonThresholdHighBoost;
    double accDivMonTime;
    double accDivMonTimeAboveThreshold;
    double accDivMonVelThreshold;
    bool   allowRedefineReset;

    double angleDiffFromModelMaxDeg;
    double dipDiffFromModelMaxDeg;
    double normDiffFromModelMax;
    double magResThreshold;
    double magResThresholdGyrBiasDeg;
    double magResTimeUp;
    double magResTimeDown;
    double magResWinTime;
    double magResWinThreshold;
    double magResWinCountDelayTime;

    double cMagSpatMin;
    double m0defNorm;
    double magDistAvgMax;
    double magDistAvgMin;
    double magDistMax;
    double magDistMin;
    double magDistNormMax;
    double magDistThreshold;
    double magDistTime;
    double magIndicatorGyroClipTimer;
    double magNormGyroClipThreshold;
    double magRedefEventTime;
    double magRedefWindowTime;
    double magVarInputClamp;
    double magVerticalAngleThresholdDeg;
    double normDiffMaxM0Avg;
    double pMagSpat;
    double sCmagBoost;

    bool   doAccCalib;
    bool   doAccDivMonHMScenario;
    bool   doAdaptiveInitAcc;
    bool   doAdaptiveMagnetometerTau;
    bool   doDominantFieldLearning;
    bool   doHeadingRedef;
    bool   doIcc;
    bool   doLinearMagnetometerDetection;
    bool   doLowPassAccUpdate;
    bool   doM0Update;
    bool   doMagneticRandomWalk;
    bool   doMagneticSdi;
    bool   doMagnetometerOutlierRejections;
    bool   doMagnetometerUpdate;
    bool   doProjectMagOnHoriPlane;
    bool   doRedefBasedOnDomField;
    bool   doRedefBasedOnState;
    bool   doRedefineAfterWindow;
    bool   doRedefineDistortionCheck;
    bool   doRedefineDominantCheck;
    bool   doRedefineNotInStatic;
    bool   doRedefineOnlyInStatic;
    bool   doRedefineSdHeading;
    bool   doRedefineSpatial;
    bool   doRedefineTemporal;
    bool   doStableHorizon;
    bool   doZru;

    double sdInitAcc;
    double sdInitGyrBiasDeg;
    double sdInitMag;
    double sdInitMagvar;
    double sdInitOrientDeg;
    double sQvAccLowPass;
    double sQvM0Def;
    double sQvMagConverging;
    double sQvMagRandomWalk;
    double sQwMagSpat;
    double dpBoost;
    double fAccBoost;
    double fAccBoostDecreaseTime;
    double fAccBoostIncreaseTime;
    double fAccClip;
    double fAccDynamic;
    double fAngleDiff;
    double fGyrClip;
    double fGyrClipSoft;
    double fGyrDynamic;
    double fGyrLpfClipBoost;
    double fGyrLpfDynamic;
    double fMagDist;
    double fMagGyrBiasStdExp;
    double fMagGyrBiasStdMax;
    double fMagGyrBiasStdMin;
    double fMagMagAvgVarExp;
    double fMagMagAvgVarMin;
    double fMagVar;
    double fStillnessRedefThreshold;
    double fTauGyrBiasStdMin;
    double fTauMagDistAvgMax;
    double fTauMagDistAvgMin;
    double fZruDynamicInit;
    double fZruThreshold;
    double gyrBiasStdMaxDeg;
    double gyrBiasStdMinDeg;
    double initialFAccBoostMax;
    double initialFAccBoostMin;
    double maxRedefRateDeg;
    int    minNumberOfZruSamples;
    double movementRedefThresholdDeg;
    double nominalT;
    double omegaRedefMinDeg;
    double omegaRedefTypicalDeg;
    double residualTresholdZru;
    double sdHeadingThreshold;
    double significantBiasChange;
    double softClipMaxDeg;
    double softClipMinDeg;
    bool   startInDominantField;
    double staticThresholdFieldLearningDeg;
    double tauAcc;
    double tauFGyrLpfDynamic;
    double tauM0AvgFast;
    double tauM0AvgMedium;
    double tauM0AvgSlow;
    double tauMagAvg0;
    double tauMagAvgLinear;
    double tauMagDistAvgDown;
    double tauMagDistAvgUp;
    double tauMagSpatMax;
    double tauMagVar;
    double tauMagVarLinear;
    double tauRedefineSdHeading;
    double tauRedefineTemporal;
    double tauVel;
    int    updateRateZru;
    int    validityCounter;
    double zruDetectionRate;

    double clipAcc;
    double clipGyrDegS;
};
// §XXI FoxKF.gloveBase (сверено с xsb-дампом FOX_KFA_Filter.gloveBase): nominalT=0.008333=1/120 (перчатка 120 Гц),
//   магнитные gate 6°/3.5°/0.03, ZRU, clipAcc 157 / clipGyr 2000, sd init, redefine курса (formules.txt)
inline constexpr KfaGloveParams kGloveBase = {
    .accDivMonTauDecay=0.1, .accDivMonThreshold=0.5,
    .accDivMonThresholdHighBoost=2.0, .accDivMonTime=2.0,
    .accDivMonTimeAboveThreshold=0.5, .accDivMonVelThreshold=2.0,
    .allowRedefineReset=true,
    .angleDiffFromModelMaxDeg=6.0, .dipDiffFromModelMaxDeg=3.5,
    .normDiffFromModelMax=0.03, .magResThreshold=0.9,
    .magResThresholdGyrBiasDeg=0.03, .magResTimeUp=0.6,
    .magResTimeDown=3.0, .magResWinTime=0.06,
    .magResWinThreshold=0.5, .magResWinCountDelayTime=0.05,
    .cMagSpatMin=0.5, .m0defNorm=1.0,
    .magDistAvgMax=0.002, .magDistAvgMin=0.0003,
    .magDistMax=1.0, .magDistMin=1e-6, .magDistNormMax=2.0,
    .magDistThreshold=0.1, .magDistTime=1.0,
    .magIndicatorGyroClipTimer=2.0, .magNormGyroClipThreshold=0.1,
    .magRedefEventTime=2.0, .magRedefWindowTime=1.0,
    .magVarInputClamp=0.04, .magVerticalAngleThresholdDeg=85.0,
    .normDiffMaxM0Avg=0.5, .pMagSpat=0.1, .sCmagBoost=0.015,
    .doAccCalib=false, .doAccDivMonHMScenario=true,
    .doAdaptiveInitAcc=true, .doAdaptiveMagnetometerTau=true,
    .doDominantFieldLearning=false, .doHeadingRedef=false,
    .doIcc=false, .doLinearMagnetometerDetection=false,
    .doLowPassAccUpdate=true, .doM0Update=false,
    .doMagneticRandomWalk=true, .doMagneticSdi=false,
    .doMagnetometerOutlierRejections=true, .doMagnetometerUpdate=false,
    .doProjectMagOnHoriPlane=false, .doRedefBasedOnDomField=false,
    .doRedefBasedOnState=true, .doRedefineAfterWindow=true,
    .doRedefineDistortionCheck=true, .doRedefineDominantCheck=true,
    .doRedefineNotInStatic=true, .doRedefineOnlyInStatic=false,
    .doRedefineSdHeading=false, .doRedefineSpatial=false,
    .doRedefineTemporal=false, .doStableHorizon=false, .doZru=true,
    .sdInitAcc=1.0, .sdInitGyrBiasDeg=0.3, .sdInitMag=0.2,
    .sdInitMagvar=0.1, .sdInitOrientDeg=3.0,
    .sQvAccLowPass=0.04, .sQvM0Def=0.014, .sQvMagConverging=0.01,
    .sQvMagRandomWalk=0.01, .sQwMagSpat=0.03,
    .dpBoost=10.0, .fAccBoost=1000.0,
    .fAccBoostDecreaseTime=60.0, .fAccBoostIncreaseTime=5.0,
    .fAccClip=1.0, .fAccDynamic=0.0, .fAngleDiff=0.04,
    .fGyrClip=0.5, .fGyrClipSoft=0.05, .fGyrDynamic=4e-8,
    .fGyrLpfClipBoost=3.0, .fGyrLpfDynamic=4e-8,
    .fMagDist=300000.0, .fMagGyrBiasStdExp=2.0,
    .fMagGyrBiasStdMax=1.0, .fMagGyrBiasStdMin=0.15,
    .fMagMagAvgVarExp=1.5, .fMagMagAvgVarMin=0.5,
    .fMagVar=500.0, .fStillnessRedefThreshold=20.0,
    .fTauGyrBiasStdMin=0.9, .fTauMagDistAvgMax=1.6,
    .fTauMagDistAvgMin=0.9, .fZruDynamicInit=2.0, .fZruThreshold=3.0,
    .gyrBiasStdMaxDeg=0.07, .gyrBiasStdMinDeg=0.005,
    .initialFAccBoostMax=1000.0, .initialFAccBoostMin=300.0,
    .maxRedefRateDeg=150.0, .minNumberOfZruSamples=15,
    .movementRedefThresholdDeg=0.3, .nominalT=0.008333,
    .omegaRedefMinDeg=5.0, .omegaRedefTypicalDeg=100.0,
    .residualTresholdZru=90.0, .sdHeadingThreshold=5.0,
    .significantBiasChange=0.005, .softClipMaxDeg=0.0,
    .softClipMinDeg=0.0, .startInDominantField=false,
    .staticThresholdFieldLearningDeg=10.0, .tauAcc=10.0,
    .tauFGyrLpfDynamic=6.0, .tauM0AvgFast=30.0,
    .tauM0AvgMedium=120.0, .tauM0AvgSlow=300.0,
    .tauMagAvg0=1.0, .tauMagAvgLinear=8.0,
    .tauMagDistAvgDown=15.0, .tauMagDistAvgUp=500.0,
    .tauMagSpatMax=30.0, .tauMagVar=0.05, .tauMagVarLinear=0.1,
    .tauRedefineSdHeading=10.0, .tauRedefineTemporal=120.0,
    .tauVel=2.0, .updateRateZru=2, .validityCounter=2,
    .zruDetectionRate=10.0,
    .clipAcc=157.0, .clipGyrDegS=2000.0,
};

// §XXI вариант фильтра для надетой перчатки (human): проекция магнита на горизонт, temporal redefine (formules.txt)
inline constexpr KfaGloveParams kGloveHuman = []() {
    KfaGloveParams p = kGloveBase;
    p.accDivMonThresholdHighBoost = 3.0;
    p.allowRedefineReset          = false;
    p.doProjectMagOnHoriPlane     = true;
    p.doRedefineAfterWindow       = false;
    p.doRedefineDistortionCheck   = false;
    p.doRedefineNotInStatic       = false;
    p.doRedefineTemporal          = true;

    return p;
}();

// §XXI вариант фильтра VRU (vertical reference unit): обновление m0, без ZRU и магнитометра (formules.txt)
inline constexpr KfaGloveParams kGloveVRU = []() {
    KfaGloveParams p = kGloveBase;
    p.doDominantFieldLearning = false;
    p.doHeadingRedef          = false;
    p.doM0Update              = true;
    p.doMagnetometerUpdate    = false;
    p.doZru                   = false;
    return p;
}();

struct CarpusPoint {
    const char* label;
    double      x, y, z;
};
// §92.1 6 опорных точек запястья (rightHand.segments1.points): длина пясти 0.027 м, разнос CMC по Y ±0.0274 (зеркалится L/R) (formules.txt)
inline constexpr std::array<CarpusPoint, 6> kRightCarpusPoints = {{
    { "wrist origin", 0.0000,  0.0000,  0.0 },
    { "Thumb CMC",    0.0270, -0.0274,  0.0 },
    { "Index CMC",    0.0270, -0.0137,  0.0 },
    { "Middle CMC",   0.0270,  0.0000,  0.0 },
    { "Ring CMC",     0.0270,  0.0137,  0.0 },
    { "Little CMC",   0.0270,  0.0274,  0.0 },
}};

enum class FingerJointType : std::uint8_t {
    SaddleCMC,
    HingeCMC,
    HingeMCP2DOF,
    HingeMCP1DOF,
    HingePIP,
    HingeDIP,
    HingeIP,
    Carpal,
    Opposition,
    Spread,
    Reserved,
};
// §91/§92 типы суставов пальцев (DOF): большой=saddle-CMC-3DOF/hinge-MCP-1DOF/hinge-IP; II..V=MCP-2DOF/PIP/DIP; Carpal/Opposition/Spread/Reserved (formules.txt)
inline constexpr std::array<FingerJointType, 19> kFingerJointTypes = {{
    FingerJointType::SaddleCMC,
    FingerJointType::HingeMCP1DOF,
    FingerJointType::HingeIP,
    FingerJointType::HingeMCP2DOF,
    FingerJointType::HingePIP,
    FingerJointType::HingeDIP,
    FingerJointType::HingeMCP2DOF,
    FingerJointType::HingePIP,
    FingerJointType::HingeDIP,
    FingerJointType::HingeMCP2DOF,
    FingerJointType::HingePIP,
    FingerJointType::HingeDIP,
    FingerJointType::HingeMCP2DOF,
    FingerJointType::HingePIP,
    FingerJointType::HingeDIP,
    FingerJointType::Carpal,
    FingerJointType::Opposition,
    FingerJointType::Spread,
    FingerJointType::Reserved,
}};
inline const char* fingerJointTypeName(FingerJointType t) {
    switch (t) {
        case FingerJointType::SaddleCMC:    return "saddle-CMC-3DOF";
        case FingerJointType::HingeCMC:     return "hinge-CMC-1DOF";
        case FingerJointType::HingeMCP2DOF: return "hinge-MCP-2DOF";
        case FingerJointType::HingeMCP1DOF: return "hinge-MCP-1DOF";
        case FingerJointType::HingePIP:     return "hinge-PIP-1DOF";
        case FingerJointType::HingeDIP:     return "hinge-DIP-1DOF";
        case FingerJointType::HingeIP:      return "hinge-IP-1DOF";
        case FingerJointType::Carpal:       return "carpal";
        case FingerJointType::Opposition:   return "opposition";
        case FingerJointType::Spread:       return "spread";
        case FingerJointType::Reserved:     return "reserved";
    }
    return "?";
}

// §1683 качество позы по остаточной невязке МНК (м): <0.02 отлично .. >0.10 невалидно (formules.txt)
inline constexpr std::array<double, 5> kPoseQualityResidBands = {
    0.02, 0.03, 0.05, 0.10, 1.00
};

enum PoseQualityBand : int {
    PoseQualityInvalid   = 0,
    PoseQualityPoor      = 1,
    PoseQualityAdequate  = 2,
    PoseQualityGood      = 3,
    PoseQualityExcellent = 4,
};
inline int poseQualityFromResidual(double residual) {
    if (residual < kPoseQualityResidBands[0]) return PoseQualityExcellent;
    if (residual < kPoseQualityResidBands[1]) return PoseQualityGood;
    if (residual < kPoseQualityResidBands[2]) return PoseQualityAdequate;
    if (residual < kPoseQualityResidBands[3]) return PoseQualityPoor;
    return PoseQualityInvalid;
}

// §30.4 байтовая таблица диспетчеризации эргоуглов по 22 суставам (точное совпадение со спекой):
//   тип0 ×7 (осевые) | тип1 правые | тип2 левые | тип3/4 стопы. Драйвит ergoTypeOf()/foxergo (formules.txt)
inline constexpr std::array<std::uint8_t, kJointCount> kErgoHandler = {

    0,
    0,
    0,
    0,
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    2,
    2,
    2,
    1,   // 14 jRightHip       right       (was 2=left handler: right hip's abduction/rotation came out sign-flipped)
    1,   // 15 jRightKnee      right
    3,   // 16 jRightAnkle     right-foot
    3,   // 17 jRightBallFoot  right-foot  (was 1: the right toe joint must use the foot extractor, not the generic right handler)
    2,   // 18 jLeftHip        left        (was 3=right-FOOT extractor: the left hip was decomposed with matrix_to_euler_B)
    2,   // 19 jLeftKnee       left
    4,   // 20 jLeftAnkle      left-foot
    4,   // 21 jLeftBallFoot   left-foot   (was 2: the left toe joint must use the foot extractor, not the generic left handler)
};

namespace constants {
    // §38 точные числовые константы движка: 180/π, π/180, π/2, π;
    //   порог малого угла SLERP 1e-6 (1e-6..1e-5); поправочные числа NLERP-ветви 0.2/0.8/(1/3)
    inline constexpr double kRad2Deg   = 57.29577951308232;
    inline constexpr double kDeg2Rad   = 0.017453292519943295;
    inline constexpr double kPi_2      = 1.5707963267948966;
    inline constexpr double kPi        = 3.141592653589793;
    inline constexpr double kSlerpEps  = 1.0e-6;
    inline constexpr double kNLerpA    = 0.2;
    inline constexpr double kNLerpB    = 0.8;
    inline constexpr double kNLerpC    = 1.0 / 3.0;
    // §13[д] коэффициенты решателя позы; §38/§31.2 шаг Гаусса–Ньютона α=0.25, демпфирование Левенберга λ=0.01
    inline constexpr double kSolverC1     = ::fox::kSolverC1;
    inline constexpr double kSolverC2     = ::fox::kSolverC2;
    inline constexpr double kSolverAlpha  = 0.25;
    inline constexpr double kSolverLambda = 0.01;

    // §41.1 гравитация модели g = 9.812687 м/с² (вектор (0,0,-g), мировая Z вверх) (formules.txt)
    inline constexpr double kGravityMs2 = 9.812687;

    // §XXX основная частота движка 240 Гц, шаг dt = 1/240 с (formules.txt)
    inline constexpr double kSampleRateHz = 240.0;
    inline constexpr double kSampleDtSec  = 1.0 / kSampleRateHz;
}

constexpr double totalMassRatio() {
    double s = 0.0;
    for (double m : kMassRatio) s += m;
    return s;
}

// §57 uniform-масштабирование скелета: коэффициент = рост_субъекта / 1.75 м (formules.txt)
inline double scaleFor(double subjectHeightM) {
    return (subjectHeightM > 1e-3) ? (subjectHeightM / kRefHeightM) : 1.0;
}

inline double ankleHeightM(double subjectHeightM) {
    return scaleFor(subjectHeightM) *
           std::abs(double(kFootPointsRight[0].r_local.z()));
}

inline double pelvisStandHeightM(double subjectHeightM) {
    return scaleFor(subjectHeightM) * (
        std::abs(double(kFootPointsRight[0].r_local.z())) +
        std::abs(double(kSensorToBone[16].L_bone.z())) +
        std::abs(double(kSensorToBone[15].L_bone.z())));
}

inline double pelvisSitHeightM(double subjectHeightM) {
    return scaleFor(subjectHeightM) * (
        std::abs(double(kFootPointsRight[0].r_local.z())) +
        std::abs(double(kSensorToBone[16].L_bone.z())));
}

inline double trunkLengthM(double subjectHeightM) {
    double s = 0.0;
    for (int i = 0; i <= 5; ++i)
        s += std::abs(double(kSensorToBone[i].L_bone.z()));
    return scaleFor(subjectHeightM) * s;
}

// §1341/§37.1 центр масс тела: CoM = Σ(m_i·c_i)/Σm_i по сегментам; полная масса M = bodyMass·(Σm/100) (formules.txt)
inline QVector3D centerOfMass(const std::array<QVector3D, kSegmentCount>& segCenters,
                              double* M = nullptr,
                              double bodyMassKg = kDefaultBodyMassKg) {
    double sx = 0.0, sy = 0.0, sz = 0.0, sm = 0.0;
    for (int i = 0; i < kSegmentCount; ++i) {
        const double m = kMassRatio[i];
        sx += m * double(segCenters[i].x());
        sy += m * double(segCenters[i].y());
        sz += m * double(segCenters[i].z());
        sm += m;
    }
    if (M) {
        const double total100 = totalMassRatio();
        *M = (total100 > 1e-9) ? (bodyMassKg * (sm / total100)) : 0.0;
    }
    if (sm <= 0.0) return QVector3D(0, 0, 0);
    const double inv = 1.0 / sm;
    return QVector3D(float(sx * inv), float(sy * inv), float(sz * inv));
}

inline bool hasSensor(int seg) {
    return (seg >= 0 && seg < kSegmentCount) ? kSensorPresent[seg] : false;
}

inline int lumpOf(int jointIdx) {
    return (jointIdx >= 0 && jointIdx < kJointCount) ? kJointLump[jointIdx] : -1;
}

// §30.4 тип обработчика эргоугла сустава: 0=осевой, 1=правый, 2=левый, 3/4=стопы (formules.txt)
inline int ergoTypeOf(int jointIdx) {
    return (jointIdx >= 0 && jointIdx < kJointCount) ? int(kErgoHandler[jointIdx]) : 0;
}

}
