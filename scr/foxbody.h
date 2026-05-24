
#pragma once

#include "foxmath.h"

#include <QtGui/QVector3D>

#include <array>
#include <cstdint>

namespace fox::body {

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

inline constexpr AnthroProportions kAnthroMale = {
    .trunkRatio          = 0.295,
    .thighRatio          = 0.245,
    .shankRatio          = 0.246,
    .handRatio           = 0.108,
    .forearmRatio        = 0.146,
    .upperArmRatio       = 0.186,
    .shoulderWidthRatio  = 0.234,
    .hipWidthRatio       = 0.181,
    .hipHeightRatio      = 0.510,
    .kneeHeightRatio     = 0.293,
    .ankleHeightRatio    = 0.039,
    .footRatio           = 0.144,
};

inline constexpr AnthroProportions kAnthroFemale = {
    .trunkRatio          = 0.292,
    .thighRatio          = 0.245,
    .shankRatio          = 0.246,
    .handRatio           = 0.108,
    .forearmRatio        = 0.146,
    .upperArmRatio       = 0.186,
    .shoulderWidthRatio  = 0.222,
    .hipWidthRatio       = 0.184,
    .hipHeightRatio      = 0.505,
    .kneeHeightRatio     = 0.290,
    .ankleHeightRatio    = 0.039,
    .footRatio           = 0.144,
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

inline constexpr std::array<double, kSegmentCount> kWinterProxToComRatio = {
    0.500,
    0.500,
    0.500,
    0.500,
    0.500,
    0.500,
    0.500,
    0.000,
    0.436,
    0.430,
    0.506,
    0.000,
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

inline constexpr double kDefaultBodyMassKg = 70.0;

inline constexpr std::array<QVector3D, kSegmentCount> kBoneVec = {{
    { -0.011f, 0.0f,   0.097f },
    {  0.0f,   0.0f,   0.108f },
    {  0.0f,   0.0f,   0.099f },
    {  0.0f,   0.0f,   0.098f },
    {  0.0f,   0.0f,   0.138f },
    {  0.0f,   0.0f,   0.092f },
    {  0.0f,   0.0f,   0.170f },
    {  0.0f,  -0.140f, 0.0f   },
    {  0.0f,  -0.300f, 0.0f   },
    {  0.0f,  -0.245f, 0.0f   },
    {  0.0f,  -0.183f, 0.0f   },
    {  0.0f,   0.140f, 0.0f   },
    {  0.0f,   0.300f, 0.0f   },
    {  0.0f,   0.245f, 0.0f   },
    {  0.0f,   0.183f, 0.0f   },
    {  0.0f,   0.0f,  -0.4165f},
    {  0.0f,   0.0f,  -0.4063f},
    {  0.147f, 0.0f,  -0.065f },
    {  0.064f, 0.0f,  -0.015f },
    {  0.0f,   0.0f,  -0.4165f},
    {  0.0f,   0.0f,  -0.4063f},
    {  0.147f, 0.0f,  -0.065f },
    {  0.064f, 0.0f,  -0.015f },
}};

struct SensorToBone {
    Quat       q_bs;
    QVector3D  r_bs;
    QVector3D  L_bone;
};

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

inline constexpr double kRefSqrtHalf  = 0.7071067811865475;
inline constexpr double kRefSin5      = 0.087155742747658166;
inline constexpr double kRefCos5      = 0.996194698091745532;

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

inline constexpr float kHipHalfY      = 0.080f;
inline constexpr float kShoulderHalfY = 0.140f;
inline constexpr float kRefHeightM    = 1.75f;

struct JointDef {
    const char* label;
    int  parent;
    int  child;
};

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

inline constexpr std::array<LumpDef, kLumpGroups> kLumps = {{
    { "upperbody", 1, 0.025 },
    { "rightleg",  2, 0.025 },
    { "rightfoot", 3, 0.025 },
    { "leftleg",   2, 0.025 },
    { "leftfoot",  3, 0.025 },
    { "rightarm",  4, 0.025 },
    { "leftarm",   4, 0.025 },
}};

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

constexpr double kCos5  = 0.99619469809174555;
constexpr double kSin5  = 0.087155742747658166;
constexpr double kSqrt2H = 0.7071067811865475;
constexpr double kCos6  = 0.9945219;
constexpr double kSin6  = 0.10452846;

Quat referenceQuat(int seg, Pose pose, Gender gender);

inline constexpr std::array<double, 9> kCSpine = {
    0.05, 0.45, 0.65, 0.85, 0.35, 0.9, 0.9, 0.9, 0.9
};

inline constexpr double kCFemoropelvic = 0.12;

inline constexpr std::array<double, 3> kCPelvis = { 0.35, 25.0, 0.30 };

inline constexpr std::array<double, 6> kCArms =
    { 0.95, 0.95, 0.99, 0.20, 0.15, 0.05 };

inline constexpr std::array<double, 3> kCLegs  = { 0.5, 0.3, 0.15 };

inline constexpr std::array<double, 4> kCKnees = { 0.05, 0.1, 0.95, 1.0 };

inline constexpr std::array<double, 4> kCAnkles = { 2.0, 0.523599, 0.5, 0.0 };

inline constexpr std::array<double, 6> kCToes =
    { 0.1, 1.05, 0.5, 0.785, 0.1, 0.0872 };

inline constexpr std::array<double, 3> kALumpA_sub   = { 1.000, 1.000, 1.000  };
inline constexpr std::array<double, 3> kALumpA_jump1 = { 0.900, 0.900, 0.900  };
inline constexpr std::array<double, 3> kALumpA_jump2 = { 0.995, 0.995, 0.9995 };

inline constexpr double kJointLaxityRad     = 0.005;
inline constexpr double kHyperExtensionMax  = 0.0;

struct JointRom {
    double abdMin, abdMax;
    double flxMin, flxMax;
    double rotMin, rotMax;
};

inline constexpr std::array<JointRom, kJointCount> kJointRom = {{
     {  -25.0,  25.0,   -30.0,  35.0,  -25.0, 25.0 },
     {  -20.0,  20.0,   -25.0,  30.0,  -20.0, 20.0 },
     {  -20.0,  20.0,   -25.0,  30.0,  -20.0, 20.0 },
     {  -20.0,  20.0,   -20.0,  25.0,  -25.0, 25.0 },
     {  -35.0,  35.0,   -50.0,  60.0,  -45.0, 45.0 },
     {  -25.0,  25.0,   -30.0,  30.0,  -30.0, 30.0 },
     {  -25.0,  25.0,   -25.0,  25.0,  -45.0, 45.0 },
     { -100.0, 180.0,   -60.0, 180.0,  -90.0, 90.0 },
     {   -2.0,   2.0,     0.0, 145.0,  -80.0, 80.0 },
     {  -30.0,  30.0,   -75.0,  85.0,  -25.0, 25.0 },
     {  -25.0,  25.0,   -25.0,  25.0,  -45.0, 45.0 },
     { -100.0, 180.0,   -60.0, 180.0,  -90.0, 90.0 },
     {   -2.0,   2.0,     0.0, 145.0,  -80.0, 80.0 },
     {  -30.0,  30.0,   -75.0,  85.0,  -25.0, 25.0 },
     {  -45.0,  45.0,   -30.0, 125.0,  -45.0, 45.0 },
     {   -2.0,   2.0,     0.0, 150.0,   -5.0,  5.0 },
     {  -25.0,  25.0,   -30.0,  20.0,  -20.0, 20.0 },
     {   -5.0,   5.0,   -30.0,  70.0,  -10.0, 10.0 },
     {  -45.0,  45.0,   -30.0, 125.0,  -45.0, 45.0 },
     {   -2.0,   2.0,     0.0, 150.0,   -5.0,  5.0 },
     {  -25.0,  25.0,   -30.0,  20.0,  -20.0, 20.0 },
     {   -5.0,   5.0,   -30.0,  70.0,  -10.0, 10.0 },
}};

inline constexpr double kSdLumpRad = 0.025;
inline constexpr double kLumpStiffness = 1.0 / (kSdLumpRad * kSdLumpRad);

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

inline constexpr double kCalibInitStdOriBodyDeg          = 45.0;
inline constexpr double kCalibInitStdSensorToBodyDeg     = 1.5;
inline constexpr double kCalibStdSensorToBodyOriFloorDeg = 0.3;
inline constexpr double kCalibInitStdSensorToBodyPos     = 0.01;
inline constexpr double kCalibStdSensorToBodyPosFloor    = 0.004;

inline constexpr bool kCalibSegmentOnFloor(int seg)
{

    return (seg == 17) || (seg == 21);
}

struct DimDef {
    const char* name;
    double sd_dim;
};

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

inline constexpr double kStdHeightMeasDefault = 0.002;
inline double stdHeightMeasFor(int seg)
{
    switch (seg) {
        case 0:  return 0.03;
        case 4:  return 0.03;
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

inline constexpr int kFingerSensorsPerHand = 17;
inline constexpr int kFingerSegmentsPerHand = 20;

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

inline constexpr float kSpcAcceptanceP   = 0.5f;
inline constexpr float kSpcSuitUncertSum = 4.0f;

struct AnthroFloors {
    double armSpanMin;
    double legLengthMin;
    double trunkLengthMin;
    double hipHalfMin;
    double scapHalfMin;
};
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

inline constexpr std::array<ImuChipType, kSegmentCount> kImuChipPerSeg = {{

    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
}};

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
    float dynRangeAccMs2;
    float dynRangeGyrDegS;
    float gainErrorAcc;
    float gainErrorGyr;
};
inline constexpr ImuChipNoise kImuChipNoiseW2 = {
    .sigmaAccMs2     = 0.0232f,
    .sigmaGyrDegS    = 0.20f,
    .dynRangeAccMs2  = 157.0f,
    .dynRangeGyrDegS = 2000.0f,
    .gainErrorAcc    = 0.004f,
    .gainErrorGyr    = 0.004f,
};
inline constexpr ImuChipNoise kImuChipNoiseX2 = {
    .sigmaAccMs2     = 0.0232f,
    .sigmaGyrDegS    = 0.20f,
    .dynRangeAccMs2  = 157.0f,
    .dynRangeGyrDegS = 2000.0f,
    .gainErrorAcc    = 0.004f,
    .gainErrorGyr    = 0.004f,
};
inline constexpr ImuChipNoise kImuChipNoiseX3 = {
    .sigmaAccMs2     = 0.00899f,
    .sigmaGyrDegS    = 0.075f,
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
inline constexpr AidingBiasParams kAidingBias = {
    .cT = 0.9,
    .cV = 0.01,
};

inline constexpr std::array<std::array<double, 3>, kLumpGroups> kSdIntAccToVelXYZ = {{
    {{  2.0,  2.0,  1.0 }},
    {{  4.0,  4.0,  2.0 }},
    {{  8.0,  8.0,  4.0 }},
    {{  4.0,  4.0,  2.0 }},
    {{  8.0,  8.0,  4.0 }},
    {{ 20.0, 20.0, 20.0 }},
    {{ 20.0, 20.0, 20.0 }},
}};

inline constexpr double kStdJointBoneLength = 0.0002;

inline constexpr double kStdSamePosMeasXY   = 0.0003;

inline constexpr double kStdSamePosMeasZ3d  = 0.002;
inline constexpr double kStdSamePosMeasZ    = 10.0;

inline constexpr int    kMaxIKSteps         = 5;
inline constexpr double kIKGradTolRad       = 1.0e-4;
inline constexpr double kIKStepTolRad       = 1.0e-5;
inline constexpr double kJointLaxitySolver  = 0.005;
inline constexpr double kHypExtPenaltySd    = 0.0002;

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
inline constexpr SpineNeckParams kSpineNeck = {
    .cL3      = 0.65,
    .cL5      = 0.45,
    .cT12     = 0.85,
    .cNeck    = 0.35,
    .stdNeck  = 0.001,
    .stdSpine = 0.001,
};

inline constexpr double kScapHumThetaLowDeg  = 60.0;
inline constexpr double kScapHumThetaHighDeg = 90.0;

inline constexpr double kKneeScrewMaxDeg     = 15.0;

inline constexpr double kAnkleDorsiLimitRad  = 0.349066;

inline constexpr double kToeRockerLowRad     = 5.0  * 0.017453292519943295;
inline constexpr double kToeRockerHighRad    = 30.0 * 0.017453292519943295;

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
    2,
    1,
    3,
    1,
    3,
    2,
    4,
    2,
};

namespace constants {
    inline constexpr double kRad2Deg   = 57.29577951308232;
    inline constexpr double kDeg2Rad   = 0.017453292519943295;
    inline constexpr double kPi_2      = 1.5707963267948966;
    inline constexpr double kPi        = 3.141592653589793;
    inline constexpr double kSlerpEps  = 1.0e-6;
    inline constexpr double kNLerpA    = 0.2;
    inline constexpr double kNLerpB    = 0.8;
    inline constexpr double kNLerpC    = 1.0 / 3.0;
    inline constexpr double kSolverC1  = 272332.63;
    inline constexpr double kSolverC2  = 40680634.23;
    inline constexpr double kSolverAlpha = 0.25;
    inline constexpr double kSolverLambda = 0.01;

    inline constexpr double kGravityMs2 = 9.812687;

    inline constexpr double kSampleRateHz = 240.0;
    inline constexpr double kSampleDtSec  = 1.0 / kSampleRateHz;
}

constexpr double totalMassRatio() {
    double s = 0.0;
    for (double m : kMassRatio) s += m;
    return s;
}

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

inline int ergoTypeOf(int jointIdx) {
    return (jointIdx >= 0 && jointIdx < kJointCount) ? int(kErgoHandler[jointIdx]) : 0;
}

constexpr int kSpcClassCount   = 17;
constexpr int kSpcFeatureCount = 315;

inline constexpr std::array<const char*, kSpcClassCount> kSensorPlacementClasses = {
    "Head", "LeftFoot", "LeftForeArm", "LeftHand", "LeftLowerLeg",
    "LeftShoulder", "LeftUpperArm", "LeftUpperLeg", "Pelvis",
    "RightFoot", "RightForeArm", "RightHand", "RightLowerLeg",
    "RightShoulder", "RightUpperArm", "RightUpperLeg", "T8",
};

inline constexpr std::array<int, kSpcClassCount> kClassToSeg = {
      6,
     21,
     13,
     14,
     20,
     11,
     12,
     19,
      0,
     17,
      9,
     10,
     16,
      7,
      8,
     15,
      4,
};

inline constexpr std::array<int, kSegmentCount> kSegToClass = {
      8,
     -1,
     -1,
     -1,
     16,
     -1,
      0,
     13,
     14,
     10,
     11,
      5,
      6,
      2,
      3,
     15,
     12,
      9,
     -1,
      7,
      4,
      1,
     -1,
};

enum class SpcEpoch  : std::uint8_t {
    Calibration, LeftArmRaise, RightArmRaise, LeftLegRaise, RightLegRaise
};
enum class SpcSignal : std::uint8_t { Acc, Gyr };
enum class SpcAxis   : std::uint8_t { X, Y, Z, XAbs, YAbs, ZAbs, Normxyz };
enum class SpcBand   : std::uint8_t {
    None, Band0p5To4, Band4p5To10, Band10ToNyq
};

enum class SpcStat : std::uint8_t {
    Mean, Sum, Std, Var, Rms, Max, MaxIdx, Skew, Kurtosis,
    SameAxisInterSensorCorrMax,    SameAxisInterSensorCorrAbsMax,
    SameAxisInterSensorCorrSum,    SameAxisInterSensorCorrAbsSum,
    SameSensorInterAxisCorrMax,    SameSensorInterAxisCorrAbsMax,
    SameSensorInterAxisCorrSum,    SameSensorInterAxisCorrAbsSum,
};

struct SpcFeatureSpec {
    SpcEpoch  epoch;
    SpcSignal signal;
    SpcAxis   axis;
    SpcBand   band;
    SpcStat   stat;
};

extern const std::array<const char*,     kSpcFeatureCount> kFeatureNames;
extern const std::array<SpcFeatureSpec,  kSpcFeatureCount> kFeatureSpecs;
extern const std::array<float,           kSpcFeatureCount> kFeatureMin;
extern const std::array<float,           kSpcFeatureCount> kFeatureMax;

}
