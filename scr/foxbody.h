// Fox Mocap — biomechanical model tables, hardcoded from the reverse-engineered
// Xsens FOX_KFA Motion Engine spec (sections 24, 30, 37, 38 and Appendix A).
//
// All numbers in this file are reproduced *verbatim* from the spec document
// (fox_definitions.xsb decrypted with the XOR-101 key). The spec lives outside
// the repo; the values below are the single source of truth for Fox.
//
// Conventions (spec section 25.2, right-handed world frame):
//   X = forward (anterior), Y = left, Z = up.  Quaternions are (w,x,y,z),
//   |q|=1, scalar-first.  All angles emitted to consumers are in DEGREES;
//   internally radians.
//
// Segment indexing follows main.h (0-based).  The spec uses 1-based indices;
// the offset of one is the only translation applied here.
#pragma once

#include "foxmath.h"

#include <QtGui/QVector3D>

#include <array>
#include <cstdint>

namespace fox::body {

// ---------------------------------------------------------------------------
//  Cardinality (mirrors main.h to keep indexing consistent).
// ---------------------------------------------------------------------------
constexpr int kSegmentCount = 23;     // 23-segment Xsens body model
constexpr int kJointCount   = 22;     // = kSegmentCount - 1 (root has no parent joint)
constexpr int kLumpGroups   = 7;      // joint coupling groups (spec §37.4)
constexpr int kContactRows  = 26;     // contact-candidate table (spec §38.1)

// ---------------------------------------------------------------------------
//  Body reference (spec §24): which calibration pose / gender model to use.
// ---------------------------------------------------------------------------
enum Pose : std::uint8_t   { PoseT = 0, PoseN = 1 };
enum Gender : std::uint8_t { GenderLegacy = 0, GenderMale = 1, GenderFemale = 2 };

// ---------------------------------------------------------------------------
//  §37.1 — segment mass ratios (% of body mass).  Sum ≈ 100.0 (rounded).
//  Order matches SEG_* in main.h.  Left/right mirrored exactly.
// ---------------------------------------------------------------------------
inline constexpr std::array<double, kSegmentCount> kMassRatio = {
    11.7188,  // 0  Pelvis
     7.8125,  // 1  L5
     6.8359,  // 2  L3
     5.8594,  // 3  T12
     5.8594,  // 4  T8
     1.9531,  // 5  Neck
     5.8594,  // 6  Head
     1.9531,  // 7  RShoulder
     2.9297,  // 8  RUpperArm
     1.5625,  // 9  RForearm
     0.5859,  // 10 RHand
     1.9531,  // 11 LShoulder
     2.9297,  // 12 LUpperArm
     1.5625,  // 13 LForearm
     0.5859,  // 14 LHand
    14.1602,  // 15 RUpperLeg
     4.3945,  // 16 RLowerLeg
     1.0742,  // 17 RFoot
     0.3906,  // 18 RToe
    14.1602,  // 19 LUpperLeg
     4.3945,  // 20 LLowerLeg
     1.0742,  // 21 LFoot
     0.3906,  // 22 LToe
};

// ---------------------------------------------------------------------------
//  §37.6 — bone vectors L_PC (segment origin → next joint), metres.
//  These are the reference (h=1.75 m) values; runtime code multiplies by a
//  global anthropometric scale derived from subject height (spec §17.3).
//  Direction conventions (verified against spec axis layout §25.2):
//    spine/head: +Z (up); right arm: −Y; left arm: +Y; legs: −Z (down);
//    foot: +X (forward) and slight −Z; toe: +X.
// ---------------------------------------------------------------------------
inline constexpr std::array<QVector3D, kSegmentCount> kBoneVec = {{
    { -0.011f, 0.0f,   0.097f },  // 0  Pelvis → L5
    {  0.0f,   0.0f,   0.108f },  // 1  L5 → L3
    {  0.0f,   0.0f,   0.099f },  // 2  L3 → T12
    {  0.0f,   0.0f,   0.098f },  // 3  T12 → T8
    {  0.0f,   0.0f,   0.138f },  // 4  T8 → Neck
    {  0.0f,   0.0f,   0.092f },  // 5  Neck → Head
    {  0.0f,   0.0f,   0.170f },  // 6  Head → top-of-head (vertex)
    {  0.0f,  -0.140f, 0.0f   },  // 7  RShoulder lateral (T8 → R shoulder joint)
    {  0.0f,  -0.300f, 0.0f   },  // 8  RUpperArm → R elbow
    {  0.0f,  -0.245f, 0.0f   },  // 9  RForearm  → R wrist
    {  0.0f,  -0.183f, 0.0f   },  // 10 RHand     → R finger tip
    {  0.0f,   0.140f, 0.0f   },  // 11 LShoulder lateral
    {  0.0f,   0.300f, 0.0f   },  // 12 LUpperArm
    {  0.0f,   0.245f, 0.0f   },  // 13 LForearm
    {  0.0f,   0.183f, 0.0f   },  // 14 LHand
    {  0.0f,   0.0f,  -0.4165f},  // 15 RUpperLeg → R knee
    {  0.0f,   0.0f,  -0.4063f},  // 16 RLowerLeg → R ankle
    {  0.147f, 0.0f,  -0.065f },  // 17 RFoot     → R toe (ball)
    {  0.064f, 0.0f,  -0.015f },  // 18 RToe      → R toe tip
    {  0.0f,   0.0f,  -0.4165f},  // 19 LUpperLeg
    {  0.0f,   0.0f,  -0.4063f},  // 20 LLowerLeg
    {  0.147f, 0.0f,  -0.065f },  // 21 LFoot
    {  0.064f, 0.0f,  -0.015f },  // 22 LToe
}};

// ---------------------------------------------------------------------------
//  §39 — per-bone sensor→bone factory transform, sensor position on the bone,
//  and the local-frame bone vector L_bone (origin → next joint).
//  Numbers reproduced verbatim from the spec table at lines 3196–3304
//  (fox_definitions.xsb XOR-101 decrypted).  Indexing is 0-based (matches
//  main.h SEG_*); the spec lists 1-based, the offset is applied here.
//
//  q_bs is the orientation of the sensor in the bone frame at factory
//  defaults (before subject calibration §24).  Segments without a physical
//  IMU (L5, L3, T12, Neck, RToe, LToe — see kSensorPresent) get an identity
//  q_bs as a placeholder; the runtime never reads them anyway because no
//  sensor sample maps to those segments.
// ---------------------------------------------------------------------------
struct SensorToBone {
    Quat       q_bs;     // sensor → bone factory rotation (|q|=1)
    QVector3D  r_bs;     // sensor position on the bone, metres
    QVector3D  L_bone;   // bone vector (origin → next joint), metres, local frame
};

inline const std::array<SensorToBone, kSegmentCount> kSensorToBone = {{
    // 0  Pelvis    — angle 174.49°, sensor at the small of the back
    { Quat( 0.048101,  0.517692, -0.029168, -0.853716),
      QVector3D(-0.05563f,  0.00000f,  0.09514f),
      QVector3D(-0.01081f,  0.00000f,  0.09730f) },
    // 1  L5        — no sensor (interpolated)
    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.00000f,  0.00000f,  0.10790f) },
    // 2  L3        — no sensor
    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.00000f,  0.00000f,  0.09851f) },
    // 3  T12       — no sensor
    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.00000f,  0.00000f,  0.09840f) },
    // 4  T8        — angle 71.26°, sensor on the sternum/upper back
    { Quat( 0.812802, -0.010534,  0.582112,  0.019711),
      QVector3D( 0.14000f,  0.00000f,  0.07700f),
      QVector3D( 0.00000f,  0.00000f,  0.13790f) },
    // 5  Neck      — no sensor
    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.00000f,  0.00000f,  0.09161f) },
    // 6  Head      — angle 104.55°
    { Quat( 0.611905,  0.694764, -0.362720,  0.106347),
      QVector3D(-0.06928f,  0.00000f,  0.07616f),
      QVector3D( 0.00000f,  0.00000f,  0.17029f) },
    // 7  RShoulder — angle 78.14°
    { Quat( 0.776381,  0.332211, -0.258954, -0.468841),
      QVector3D(-0.02000f, -0.05000f, -0.06000f),
      QVector3D( 0.00000f, -0.14000f,  0.00000f) },
    // 8  RUpperArm — angle 99.66°
    { Quat( 0.645046, -0.253557,  0.224966, -0.684846),
      QVector3D(-0.02000f, -0.10000f,  0.02000f),
      QVector3D( 0.00000f, -0.30000f,  0.00000f) },
    // 9  RForeArm  — angle 82.87°
    { Quat( 0.749726, -0.193893,  0.211256, -0.596396),
      QVector3D( 0.00184f, -0.20024f,  0.02000f),
      QVector3D( 0.00000f, -0.24520f,  0.00000f) },
    // 10 RHand     — angle 92.37°
    { Quat( 0.692346,  0.147448,  0.046195, -0.704828),
      QVector3D( 0.00000f, -0.05500f,  0.02000f),
      QVector3D( 0.00000f, -0.18300f,  0.00000f) },
    // 11 LShoulder — angle 87.07°
    { Quat( 0.724949, -0.264417, -0.439236,  0.460004),
      QVector3D(-0.02000f,  0.05000f, -0.06000f),
      QVector3D( 0.00000f,  0.14000f,  0.00000f) },
    // 12 LUpperArm — angle 92.49°
    { Quat( 0.691599,  0.080354,  0.106548,  0.709846),
      QVector3D(-0.02000f,  0.10000f,  0.02000f),
      QVector3D( 0.00000f,  0.30000f,  0.00000f) },
    // 13 LForeArm  — angle 81.08°
    { Quat( 0.759964,  0.011096,  0.069391,  0.646155),
      QVector3D( 0.00184f,  0.20024f,  0.02000f),
      QVector3D( 0.00000f,  0.24520f,  0.00000f) },
    // 14 LHand     — angle 95.95°
    { Quat( 0.669439, -0.100277,  0.054421,  0.734053),
      QVector3D( 0.00000f,  0.05500f,  0.02000f),
      QVector3D( 0.00000f,  0.18300f,  0.00000f) },
    // 15 RUpperLeg — angle 126.39°
    { Quat( 0.450969,  0.605623,  0.478005, -0.448730),
      QVector3D( 0.01205f, -0.06000f, -0.25071f),
      QVector3D( 0.00000f,  0.00000f, -0.41648f) },
    // 16 RLowerLeg — angle 101.45°
    { Quat( 0.633032, -0.250941,  0.700127,  0.214757),
      QVector3D( 0.03090f,  0.01000f, -0.13293f),
      QVector3D( 0.00000f,  0.00000f, -0.40634f) },
    // 17 RFoot     — angle 32.22°
    { Quat( 0.960726,  0.112046,  0.252661, -0.024793),
      QVector3D( 0.08500f,  0.00200f, -0.01200f),
      QVector3D( 0.14700f,  0.00000f, -0.06500f) },
    // 18 RToe      — no sensor
    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.06400f,  0.00000f, -0.01500f) },
    // 19 LUpperLeg — angle 134.94°
    { Quat( 0.383195, -0.605793,  0.460515,  0.523548),
      QVector3D( 0.01205f,  0.06000f, -0.25071f),
      QVector3D( 0.00000f,  0.00000f, -0.41648f) },
    // 20 LLowerLeg — angle 98.55°
    { Quat( 0.652402,  0.304931,  0.665425, -0.196464),
      QVector3D( 0.03090f, -0.01000f, -0.13293f),
      QVector3D( 0.00000f,  0.00000f, -0.40634f) },
    // 21 LFoot     — angle 35.17°
    { Quat( 0.953278, -0.149169,  0.227166,  0.131928),
      QVector3D( 0.08500f, -0.00200f, -0.01200f),
      QVector3D( 0.14700f,  0.00000f, -0.06500f) },
    // 22 LToe      — no sensor
    { Quat(1, 0, 0, 0),
      QVector3D(0, 0, 0),
      QVector3D( 0.06400f,  0.00000f, -0.01500f) },
}};

// ---------------------------------------------------------------------------
//  §24.1 / §24.2 — reference bone-in-world quaternions for the two
//  calibration poses (legacy male body model; female/robot variants belong
//  to future PRs).  Used by §174.4 q_align computation:
//      q_align(i) = kRefQuatN[i] ⊗ conj(q_S_avg(i)) ⊗ conj(q_bs(i)).
//
//  T-pose (arms horizontal) — the spine carries natural posture tilts:
//  pelvis tilts forward by 6.34°+Y, lumbar arches back (L3, T12 ≈ 5.83°−Y),
//  neck/head lean forward, legs have small forward tilts.  The arms are
//  exactly identity (perpendicular to the spine, pointing laterally).
//
//  N-pose (arms down at sides) — spine/legs identical to T-pose; the arm
//  chain rotates ±90°+X (right) / ±90°−X (left) so the arm bones point
//  along −Z (down).  Shoulder also tilts ±10°+X relative to T-pose so the
//  acromion sits a touch below the lateral horizontal.
//  Numbers reproduced verbatim from spec §24.1 / §24.2.  Magnitudes:
//  sin5°  = 0.087155742747658166,  sin45° = 0.7071067811865475,
//  sin(6.34°/2) = 0.0553003879..., sin(12.61°/2) = 0.109822896...,
//  cos(6.34°/2) = 0.998469762..., cos(90°/2)    = 0.7071067811865475,
//  cos(10°/2)   = 0.996194698091745.
// ---------------------------------------------------------------------------
inline constexpr double kRefSqrtHalf  = 0.7071067811865475;  // sin45°=cos45°
inline constexpr double kRefSin5      = 0.087155742747658166; // sin5°
inline constexpr double kRefCos5      = 0.996194698091745532; // cos5°

inline constexpr std::array<Quat, kSegmentCount> kRefQuatT = {{
    /* 0  Pelvis    */ Quat( 0.9984697627340179, 0.0,  0.05530038793601835, 0.0),
    /* 1  L5        */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 2  L3        */ Quat( 0.9987077007098614, 0.0, -0.05082252003612363, 0.0),
    /* 3  T12       */ Quat( 0.9987050781810652, 0.0, -0.05087402888854364, 0.0),
    /* 4  T8        */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 5  Neck      */ Quat( 0.9939511715005132, 0.0,  0.1098228968510563,  0.0),
    /* 6  Head      */ Quat( 0.999568500071736,  0.0,  0.02937369000210807, 0.0),
    /* 7  RShoulder */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 8  RUpperArm */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 9  RForeArm  */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 10 RHand     */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 11 LShoulder */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 12 LUpperArm */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 13 LForeArm  */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 14 LHand     */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 15 RUpperLeg */ Quat( 0.9997115343780182, 0.0,  0.02401766082591783, 0.0),
    /* 16 RLowerLeg */ Quat( 0.999173864575052,  0.0,  0.04063973855914903, 0.0),
    /* 17 RFoot     */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 18 RToe      */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 19 LUpperLeg */ Quat( 0.9997115343780182, 0.0,  0.02401766082591783, 0.0),
    /* 20 LLowerLeg */ Quat( 0.999173864575052,  0.0,  0.04063973855914903, 0.0),
    /* 21 LFoot     */ Quat( 1.0,                0.0,  0.0,                 0.0),
    /* 22 LToe      */ Quat( 1.0,                0.0,  0.0,                 0.0),
}};

// N-pose reference (§24.2): spine/legs identical to T-pose, shoulder ±10°+X,
// upper arm/forearm/hand ±90°+X.
inline constexpr std::array<Quat, kSegmentCount> kRefQuatN = {{
    /* 0  Pelvis    */ Quat( 0.9984697627340179, 0.0,            0.05530038793601835, 0.0),
    /* 1  L5        */ Quat( 1.0,                0.0,            0.0,                 0.0),
    /* 2  L3        */ Quat( 0.9987077007098614, 0.0,           -0.05082252003612363, 0.0),
    /* 3  T12       */ Quat( 0.9987050781810652, 0.0,           -0.05087402888854364, 0.0),
    /* 4  T8        */ Quat( 1.0,                0.0,            0.0,                 0.0),
    /* 5  Neck      */ Quat( 0.9939511715005132, 0.0,            0.1098228968510563,  0.0),
    /* 6  Head      */ Quat( 0.999568500071736,  0.0,            0.02937369000210807, 0.0),
    /* 7  RShoulder */ Quat( kRefCos5,           kRefSin5,       0.0,                 0.0),
    /* 8  RUpperArm */ Quat( kRefSqrtHalf,       kRefSqrtHalf,   0.0,                 0.0),
    /* 9  RForeArm  */ Quat( kRefSqrtHalf,       kRefSqrtHalf,   0.0,                 0.0),
    /* 10 RHand     */ Quat( kRefSqrtHalf,       kRefSqrtHalf,   0.0,                 0.0),
    /* 11 LShoulder */ Quat( kRefCos5,          -kRefSin5,       0.0,                 0.0),
    /* 12 LUpperArm */ Quat( kRefSqrtHalf,      -kRefSqrtHalf,   0.0,                 0.0),
    /* 13 LForeArm  */ Quat( kRefSqrtHalf,      -kRefSqrtHalf,   0.0,                 0.0),
    /* 14 LHand     */ Quat( kRefSqrtHalf,      -kRefSqrtHalf,   0.0,                 0.0),
    /* 15 RUpperLeg */ Quat( 0.9997115343780182, 0.0,            0.02401766082591783, 0.0),
    /* 16 RLowerLeg */ Quat( 0.999173864575052,  0.0,            0.04063973855914903, 0.0),
    /* 17 RFoot     */ Quat( 1.0,                0.0,            0.0,                 0.0),
    /* 18 RToe      */ Quat( 1.0,                0.0,            0.0,                 0.0),
    /* 19 LUpperLeg */ Quat( 0.9997115343780182, 0.0,            0.02401766082591783, 0.0),
    /* 20 LLowerLeg */ Quat( 0.999173864575052,  0.0,            0.04063973855914903, 0.0),
    /* 21 LFoot     */ Quat( 1.0,                0.0,            0.0,                 0.0),
    /* 22 LToe      */ Quat( 1.0,                0.0,            0.0,                 0.0),
}};

// Stub offsets (pelvis→hip-joint, T8→shoulder-joint) used by the FK dummy chain.
// Hip half-width ±0.08 m Y (spec §37.6: «таз: бёдра ±0.08 по Y, ширина таза 0.16 м»).
// Shoulder half-width ±0.16 m Y (typical bi-acromial half on a 1.75 m subject;
// spec §37.5 says shoulderWidth is a measured dimension — at default body it
// matches the existing 0.10–0.16 range used by SkeletonXsens).
inline constexpr float kHipHalfY      = 0.080f;
inline constexpr float kShoulderHalfY = 0.140f;   // §87 RightShoulder offset r=(0,-0.140,0)
inline constexpr float kRefHeightM    = 1.75f;   // reference height used by §37.6

// ---------------------------------------------------------------------------
//  §37.2 — joint list (22 ball-and-socket).  Each joint links a parent SEGMENT
//  to a child segment; joint index = child-segment index − 1 in our ordering.
//  Labels match spec table verbatim.  (jLeftBallFoot = LFoot↔LToe = MTP.)
// ---------------------------------------------------------------------------
struct JointDef {
    const char* label;
    int  parent;   // SEG_* of the parent segment
    int  child;    // SEG_* of the child segment (= joint owner)
};

inline constexpr std::array<JointDef, kJointCount> kJoints = {{
    { "jL5S1",            0,  1 },   // Pelvis → L5
    { "jL4L3",            1,  2 },   // L5 → L3
    { "jL1T12",           2,  3 },   // L3 → T12
    { "jT9T8",            3,  4 },   // T12 → T8
    { "jT1C7",            4,  5 },   // T8 → Neck
    { "jC1Head",          5,  6 },   // Neck → Head
    { "jRightT4Shoulder", 4,  7 },   // T8 → RShoulder (spec segpoint 5,3 → owner R8)
    { "jRightShoulder",   7,  8 },   // RShoulder → RUpperArm
    { "jRightElbow",      8,  9 },   // RUpperArm → RForearm
    { "jRightWrist",      9, 10 },   // RForearm → RHand
    { "jLeftT4Shoulder",  4, 11 },   // T8 → LShoulder
    { "jLeftShoulder",   11, 12 },   // LShoulder → LUpperArm
    { "jLeftElbow",      12, 13 },   // LUpperArm → LForearm
    { "jLeftWrist",      13, 14 },   // LForearm → LHand
    { "jRightHip",        0, 15 },   // Pelvis → RUpperLeg
    { "jRightKnee",      15, 16 },   // RUpperLeg → RLowerLeg
    { "jRightAnkle",     16, 17 },   // RLowerLeg → RFoot
    { "jRightBallFoot",  17, 18 },   // RFoot → RToe (MTP, metatarsophalangeal)
    { "jLeftHip",         0, 19 },   // Pelvis → LUpperLeg
    { "jLeftKnee",       19, 20 },
    { "jLeftAnkle",      20, 21 },
    { "jLeftBallFoot",   21, 22 },
}};

// ---------------------------------------------------------------------------
//  §37.4 — joint coupling lumps (7 groups).  Each group has a name, a kind
//  (1=upper-body, 2=leg, 3=foot, 4=arm) and the indices of joints inside it.
//  The exact joint membership in the spec table is sparse and not all 22
//  joints are listed; we store membership as a per-joint group index, with
//  −1 meaning «not in any lump».  Hard coupling weight (spec): sd = 0.025
//  (informational weight in the WLS solver = 1/sd² = 1600).
// ---------------------------------------------------------------------------
struct LumpDef {
    const char* name;
    int kind;            // 1=upperbody, 2=leg, 3=foot, 4=arm
    double sd;           // coupling sd (spec uniformly 0.025)
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

// Joint → lump-group index (0..6) or −1 if the joint is not coupled.
// Spine + scapulo-thoracic T4Shoulder joints form the upperbody lump.  Each
// gleno-humeral arm chain (shoulder → elbow → wrist) is its own arm lump.
// Hip+knee → leg, ankle+MTP → foot, mirrored L/R.
inline constexpr std::array<int, kJointCount> kJointLump = {
    0,        // 0  jL5S1            → upperbody
    0,        // 1  jL4L3            → upperbody
    0,        // 2  jL1T12           → upperbody
    0,        // 3  jT9T8            → upperbody
    0,        // 4  jT1C7            → upperbody
    0,        // 5  jC1Head          → upperbody
    0,        // 6  jRightT4Shoulder → upperbody (scapulo-thoracic posture)
    5,        // 7  jRightShoulder   → rightarm
    5,        // 8  jRightElbow      → rightarm
    5,        // 9  jRightWrist      → rightarm
    0,        // 10 jLeftT4Shoulder  → upperbody
    6,        // 11 jLeftShoulder    → leftarm
    6,        // 12 jLeftElbow       → leftarm
    6,        // 13 jLeftWrist       → leftarm
    1,        // 14 jRightHip        → rightleg
    1,        // 15 jRightKnee       → rightleg
    2,        // 16 jRightAnkle      → rightfoot
    2,        // 17 jRightBallFoot   → rightfoot
    3,        // 18 jLeftHip         → leftleg
    3,        // 19 jLeftKnee        → leftleg
    4,        // 20 jLeftAnkle       → leftfoot
    4,        // 21 jLeftBallFoot    → leftfoot
};

// ---------------------------------------------------------------------------
//  §37.7 — sensor placement.  Full-body configuration = 17 IMUs on these 17
//  segments; the remaining 6 (L5, L3, T12, Neck, RToe, LToe) are interpolated
//  from neighbours via lump coupling.
// ---------------------------------------------------------------------------
inline constexpr std::array<bool, kSegmentCount> kSensorPresent = {
    /* 0  Pelvis    */ true,
    /* 1  L5        */ false,
    /* 2  L3        */ false,
    /* 3  T12       */ false,
    /* 4  T8        */ true,
    /* 5  Neck      */ false,
    /* 6  Head      */ true,
    /* 7  RShoulder */ true,
    /* 8  RUpperArm */ true,
    /* 9  RForearm  */ true,
    /* 10 RHand     */ true,
    /* 11 LShoulder */ true,
    /* 12 LUpperArm */ true,
    /* 13 LForearm  */ true,
    /* 14 LHand     */ true,
    /* 15 RUpperLeg */ true,
    /* 16 RLowerLeg */ true,
    /* 17 RFoot     */ true,
    /* 18 RToe      */ false,
    /* 19 LUpperLeg */ true,
    /* 20 LLowerLeg */ true,
    /* 21 LFoot     */ true,
    /* 22 LToe      */ false,
};

// ---------------------------------------------------------------------------
//  §24 — reference quaternions for N/T calibration poses.  Values reproduced
//  verbatim from the legacy/male/female .xsa files.  (w,x,y,z), |q|=1.
//  Arms in T-pose are identity (extended laterally exactly as the reference);
//  arms in N-pose are ±90° around X with a small ±10° shoulder offset.
// ---------------------------------------------------------------------------
//
//  Helper constants:
//    s05 = sin(5°)/cos(5°) factor used in shoulder ±10° rotations:
//          w = cos(5°) = 0.99619469809174555
//          x = sin(5°) = 0.087155742747658166
//    s45 = sin(45°) = cos(45°) = 0.7071067811865475  (used for ±90° around X)
constexpr double kCos5  = 0.99619469809174555;
constexpr double kSin5  = 0.087155742747658166;
constexpr double kSqrt2H = 0.7071067811865475;
constexpr double kCos6  = 0.9945219;    // 12° tilt cosine
constexpr double kSin6  = 0.10452846;

// Returns the reference quaternion for segment `seg` in pose/gender combination.
// Both poses share the same arm/leg references; only the spine differs by gender,
// and only the arms differ between T-pose and N-pose.
Quat referenceQuat(int seg, Pose pose, Gender gender);

// ---------------------------------------------------------------------------
//  §40 — biomechanical joint-coupling coefficients (FOX_FE.bioMech.c_*).
//  In the engine these enter the WLS solver as Jacobian rows; in this
//  reduced-scope build we apply them deterministically as a post-FK
//  redistribution of a parent→child rotation across an anatomical chain
//  (foxcoupling.cpp).
//
//  All numbers are spec §40.2 verbatim (lines 3341–3358 of the spec doc).
// ---------------------------------------------------------------------------

// Spine rhythm: total Pelvis→T8 rotation is distributed across the 6 spinal
// joints (jL5S1, jL4L3, jL1T12, jT9T8) plus neck and head (jT1C7, jC1Head)
// using these weights.  Lumbar bends less (0.05..0.85 rising), cervical
// almost freely (0.9).  Weights ARE NOT normalised in the spec — the engine
// uses them as Jacobian factors; foxcoupling.cpp normalises by their sum so
// the redistributed angles add up to the original total.
inline constexpr std::array<double, 9> kCSpine = {
    0.05, 0.45, 0.65, 0.85, 0.35, 0.9, 0.9, 0.9, 0.9
};

// Pelvis tilt coupling [c_pelvis] — first is the fraction of pelvic tilt
// preserved (0.35), second is a degree/weight scale (25).
inline constexpr std::array<double, 2> kCPelvis = { 0.35, 25.0 };

// Scapulo-humeral ratio: shoulder elevation drags the scapula along with
// coefficients [0.95, 0.95, 0.99] — almost 1:1 at large abduction angles.
inline constexpr std::array<double, 3> kCArms = { 0.95, 0.95, 0.99 };

// Hip–knee leg coupling [c_legs] — biarticular muscle effect on the chain.
inline constexpr std::array<double, 2> kCLegs  = { 0.9,  0.95 };

// Knee screw-home mechanism [c_knees] — small obligatory tibial rotation
// during the final 20° of extension.
inline constexpr std::array<double, 2> kCKnees = { 0.9,  0.95 };

// Ankle plantar-flexion / eversion limits and coupling [c_ankles]:
//   [0] = 2.0           — eversion coupling gain
//   [1] = 0.523599 rad  — plantar-flexion limit (30°)
//   [2] = 0.5           — secondary coupling
//   [3] = 0.0           — offset
inline constexpr std::array<double, 4> kCAnkles = { 2.0, 0.523599, 0.5, 0.0 };

// Toe-off / metatarsal-phalangeal coupling [c_toes]:
//   [0] = 0.2     — base flex coupling
//   [1] = 1.05    — toe-extension gain
//   [2] = -0.5    — counter coupling
//   [3] = 1.0     — scale
//   [4] = 0.1     — small offset
//   [5] = 0.0872  — sin(5°): threshold for «toe lifted off» state
inline constexpr std::array<double, 6> kCToes = { 0.2, 1.05, -0.5, 1.0, 0.1, 0.0872 };

// Lump-state Gauss-Markov A matrices: chosen per phase (contact vs flight).
// A_sub  — sub-state, no decay (free to move);
// A_jump1— fast-decay branch used at impact / heel-strike;
// A_jump2— slow-decay branch used during stance.
inline constexpr std::array<double, 3> kALumpA_sub   = { 1.000, 1.000, 1.000  };
inline constexpr std::array<double, 3> kALumpA_jump1 = { 0.900, 0.900, 0.900  };
inline constexpr std::array<double, 3> kALumpA_jump2 = { 0.995, 0.995, 0.9995 };

// Joint laxity & hyper-extension limits applied as soft constraints.
inline constexpr double kJointLaxityRad     = 0.005;  // soft «play» per joint, radians
inline constexpr double kHyperExtensionMax  = 0.0;    // hyper-extension forbidden

// ---------------------------------------------------------------------------
//  §14 / §37 — per-joint range-of-motion clamps for the ergonomic triple
//  (abduction X, flexion Y, rotation Z), in degrees.  These are healthy-adult
//  RoMs used by the engine to gate biomechanically impossible angles after
//  fusion.  Wide limits (±180°) mean «no clinical limit» for that axis on
//  that joint.
// ---------------------------------------------------------------------------
struct JointRom {
    double abdMin, abdMax;   // X axis: ad-/abduction
    double flxMin, flxMax;   // Y axis: ext-/flexion
    double rotMin, rotMax;   // Z axis: ext-/internal rotation
};

inline constexpr std::array<JointRom, kJointCount> kJointRom = {{
    /*  0 jL5S1            */ {  -25.0,  25.0,   -30.0,  35.0,  -25.0, 25.0 },
    /*  1 jL4L3            */ {  -20.0,  20.0,   -25.0,  30.0,  -20.0, 20.0 },
    /*  2 jL1T12           */ {  -20.0,  20.0,   -25.0,  30.0,  -20.0, 20.0 },
    /*  3 jT9T8            */ {  -20.0,  20.0,   -20.0,  25.0,  -25.0, 25.0 },
    /*  4 jT1C7            */ {  -35.0,  35.0,   -50.0,  60.0,  -45.0, 45.0 },
    /*  5 jC1Head          */ {  -25.0,  25.0,   -30.0,  30.0,  -30.0, 30.0 },
    /*  6 jRightT4Shoulder */ {  -25.0,  25.0,   -25.0,  25.0,  -45.0, 45.0 },
    /*  7 jRightShoulder   */ { -100.0, 180.0,   -60.0, 180.0,  -90.0, 90.0 },
    /*  8 jRightElbow      */ {   -2.0,   2.0,     0.0, 145.0,  -80.0, 80.0 },
    /*  9 jRightWrist      */ {  -30.0,  30.0,   -75.0,  85.0,  -25.0, 25.0 },
    /* 10 jLeftT4Shoulder  */ {  -25.0,  25.0,   -25.0,  25.0,  -45.0, 45.0 },
    /* 11 jLeftShoulder    */ { -100.0, 180.0,   -60.0, 180.0,  -90.0, 90.0 },
    /* 12 jLeftElbow       */ {   -2.0,   2.0,     0.0, 145.0,  -80.0, 80.0 },
    /* 13 jLeftWrist       */ {  -30.0,  30.0,   -75.0,  85.0,  -25.0, 25.0 },
    /* 14 jRightHip        */ {  -45.0,  45.0,   -30.0, 125.0,  -45.0, 45.0 },
    /* 15 jRightKnee       */ {   -2.0,   2.0,     0.0, 150.0,   -5.0,  5.0 },
    /* 16 jRightAnkle      */ {  -25.0,  25.0,   -30.0,  20.0,  -20.0, 20.0 },
    /* 17 jRightBallFoot   */ {   -5.0,   5.0,   -30.0,  70.0,  -10.0, 10.0 },
    /* 18 jLeftHip         */ {  -45.0,  45.0,   -30.0, 125.0,  -45.0, 45.0 },
    /* 19 jLeftKnee        */ {   -2.0,   2.0,     0.0, 150.0,   -5.0,  5.0 },
    /* 20 jLeftAnkle       */ {  -25.0,  25.0,   -30.0,  20.0,  -20.0, 20.0 },
    /* 21 jLeftBallFoot    */ {   -5.0,   5.0,   -30.0,  70.0,  -10.0, 10.0 },
}};

// §37.4 — lump-group coupling stiffness.  Standard deviation of joint angle
// coupling between segments in the same lump group.  Engine uses 1/sd² as the
// MNK weight; for our deterministic redistribution we use it as a normaliser.
inline constexpr double kSdLumpRad = 0.025;                 // §37.4 verbatim
inline constexpr double kLumpStiffness = 1.0 / (kSdLumpRad * kSdLumpRad);  // = 1600

// ---------------------------------------------------------------------------
//  §174.5 / §174.6 — calibration quality scale and stage timing.
//
//  Quality bands (residual angle of q_align relative to the factory q_bs
//  baseline, see spec §174.5):
//    angle_resid < 5°    →  5  excellent
//        5°..10°         →  4  good
//       10°..20°         →  3  adequate
//       20°..30°         →  2  poor
//             > 30°      →  1  invalid (re-calibrate)
//
//  Stage timing (§174.6):
//    Stage 1: N-pose, 3 s
//    Stage 2: T-pose, 3 s     (verification)
//    Stage 3: K-pose / individual gestures, 3 s   (refinement)
// ---------------------------------------------------------------------------
inline constexpr std::array<double, 4> kCalibQualityThresholdDeg = {
    5.0, 10.0, 20.0, 30.0
};
inline constexpr double kCalibStageDurationSec = 3.0;     // §174.6 each pose
inline constexpr int    kCalibQualityExcellent = 5;
inline constexpr int    kCalibQualityGood      = 4;
inline constexpr int    kCalibQualityAdequate  = 3;
inline constexpr int    kCalibQualityPoor      = 2;
inline constexpr int    kCalibQualityInvalid   = 1;

// §174.5 — map a residual angle (degrees) to the 1..5 quality band.
inline int calibrationQuality(double residualDeg)
{
    if (residualDeg < kCalibQualityThresholdDeg[0]) return kCalibQualityExcellent;
    if (residualDeg < kCalibQualityThresholdDeg[1]) return kCalibQualityGood;
    if (residualDeg < kCalibQualityThresholdDeg[2]) return kCalibQualityAdequate;
    if (residualDeg < kCalibQualityThresholdDeg[3]) return kCalibQualityPoor;
    return kCalibQualityInvalid;
}

// §174.5 — human-readable label for the quality band (kept English; UI may
// localise via the existing Lang strings, but the band name is canonical).
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

// ---------------------------------------------------------------------------
//  §38.5 — sensor-to-body alignment standard deviations (re-exposed for the
//  weighted Wahba solver in the calibration wizard).  These are the same
//  numbers already stored in kSkin but pulled out at the foxbody level so
//  the calibrator can reach them without including the larger SkinParams
//  struct.  Spec wording: «начальные СКО до калибровки субъекта».
// ---------------------------------------------------------------------------
inline constexpr double kCalibInitStdOriBodyDeg          = 45.0;   // pre-calibration prior
inline constexpr double kCalibInitStdSensorToBodyDeg     = 1.5;    // post-stage SD
inline constexpr double kCalibStdSensorToBodyOriFloorDeg = 0.3;    // feet — tighter
inline constexpr double kCalibInitStdSensorToBodyPos     = 0.01;   // metres
inline constexpr double kCalibStdSensorToBodyPosFloor    = 0.004;  // feet — tighter

// True for the «floor-grade» tighter SD: only the feet in contact with the
// ground get the 0.3° / 0.004 m floor (spec §38.5 — «нижняя граница в работе»).
inline constexpr bool kCalibSegmentOnFloor(int seg)
{
    // 17/21 = RFoot/LFoot (0-based after main.h offset).
    return (seg == 17) || (seg == 21);
}

// ---------------------------------------------------------------------------
//  §37.5 — calibration body dimensions and their measurement sd (metres).
//  Used as weights when applying user-measured anthropometry.
// ---------------------------------------------------------------------------
struct DimDef {
    const char* name;
    double sd_dim;      // metres
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

// ---------------------------------------------------------------------------
//  §38.1 — foot contact candidate table (26×6: [segIdx, ptIdx, th1, th2, th3, th4]).
//  Spec gives feet (segs 17 RFoot, 21 LFoot) with three foot points each plus
//  the MTP-toe ball, plus the lower-leg patella for kneeling.
//  Stored compactly as a flat array; users index by row.
//
//  Foot point labels (FOX_Skeleton.segments18.pointlabels in spec):
//    3 = pHeelFoot (HEEL), 4 = pFirstMetatarsal, 5 = pFifthMetatarsal, 6 = ball/Toe
// ---------------------------------------------------------------------------
struct ContactRow {
    int    seg;    // SEG_* of the segment owning this contact point
    int    pt;     // point index within the segment (see comment above)
    double th1, th2, th3, th4;   // contact-probability thresholds / weights
};

// Foot/leg contact point candidates.  Spec §44.10 lists 26 candidates total;
// numbered probability thresholds are explicit for the 12 entries below
// (foot main + ball + toe-tip + kneeling pad).  The remaining 14 entries
// (hand-balance for acrobatic poses, pelvis sit-aid, etc.) are mentioned
// in §44.10 without explicit th1..th4 values, so they are not seeded
// here — adding them would require invented numbers.
inline constexpr int kSEG_RLowerLeg = 16;
inline constexpr int kSEG_RFoot     = 17;
inline constexpr int kSEG_RToe      = 18;
inline constexpr int kSEG_LLowerLeg = 20;
inline constexpr int kSEG_LFoot     = 21;
inline constexpr int kSEG_LToe      = 22;

inline constexpr std::array<ContactRow, 12> kFootContacts = {{
    // Right foot: heel + medial / lateral metatarsal + ball/toe-6
    { kSEG_RFoot,     3, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_RFoot,     4, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_RFoot,     5, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_RFoot,     6, 0.05, 0.20, 0.20, 0.40 },
    // Right toe-tip — spec §44.10 «аналогично носку стопы» (same thresholds
    // as the foot's toe-6 entry).
    { kSEG_RToe,      2, 0.05, 0.20, 0.20, 0.40 },
    // Left foot mirror
    { kSEG_LFoot,     3, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_LFoot,     4, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_LFoot,     5, 0.05, 0.25, 0.25, 0.40 },
    { kSEG_LFoot,     6, 0.05, 0.20, 0.20, 0.40 },
    { kSEG_LToe,      2, 0.05, 0.20, 0.20, 0.40 },
    // Lower-leg kneeling pads — high th1 means «rarely on the floor»
    { kSEG_RLowerLeg, 5, 0.08, 1.0,  1.0,  0.40 },
    { kSEG_LLowerLeg, 5, 0.08, 1.0,  1.0,  0.40 },
}};

// Spec §44.3 (Г) — per-segment height-measurement standard deviation
// (stdHeightMeas, metres).  Used by the WLS body solver as the weight
// 1/sd² on the z = floor constraint when a contact point on that
// segment is active.  Tighter SD = harder anchor.  Default 0.002;
// Pelvis/T8/UpperLeg (large bones, loose anchor) = 0.03; ForeArm/
// LowerLeg (close to ground in acrobatic poses) = 0.005.
inline constexpr double kStdHeightMeasDefault = 0.002;
inline double stdHeightMeasFor(int seg)
{
    switch (seg) {
        case 0:  return 0.03;    // Pelvis
        case 4:  return 0.03;    // T8
        case 11: return 0.03;    // LShoulder (spec lists seg index 12 = LShoulder in 1-based;
        case 15: return 0.03;    // RUpperLeg
        case 19: return 0.03;    // LUpperLeg
        case 9:  return 0.005;   // RForeArm
        case 13: return 0.005;   // LForeArm
        case 16: return 0.005;   // RLowerLeg
        case 20: return 0.005;   // LLowerLeg
        default: return kStdHeightMeasDefault;
    }
}

// ---------------------------------------------------------------------------
//  §38.2 — contact detection & update parameters.
// ---------------------------------------------------------------------------
struct ContactParams {
    double dLevelDefault;          // m — height tolerance to floor (default)
    double dLevelFoot;             // m — tighter tolerance for foot
    bool   enableImpactDetection;
    double impactTh;
    double impactWinDuration;      // s
    int    maxDetectedContacts;
    double minimumAcceptableMeasure;
    double sameHeightTh;           // m — «same floor level» threshold
    double secondaryPelvisT8RejMinDeg;
    double secondaryPelvisT8RejMaxDeg;
    double firstWinWidth;          // s
    double firstWinWidthHighVel;   // s
    double highVelTh;              // m/s
    double secondWinWidthBefore;   // s
    double secondWinWidthAfter;    // s
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

// ---------------------------------------------------------------------------
//  §19 / §38.4 — magnetic field reference & gating thresholds.
//  Declination D = 0 by default (set to local mag-dec at deployment site);
//  inclination I = 78° (default dip from spec §37.5 «e_dip_mag = 78»);
//  reference norm |m0| = 1 (unitless after sensor normalisation).
//  Gate: hint accepted ⇔ all three sub-tests pass.
// ---------------------------------------------------------------------------
struct MagnetParams {
    double declinationDeg;             // D, set by deployment site (0 by default)
    double inclinationDeg;             // I body-override (spec §37.5: e_dip_mag = 78°)
    double inclinationDeg2;            // secondary I (spec §37.5: e_dip_mag2 = 85°)
    // §51.1 reference dip of the FREE-FIELD model (m0DefDipAngleRad).  Signed
    // in spec convention (negative = field points down in NWU+Z-up).  Stored
    // alongside the +78° body override so a reader can see both side by side.
    // Note: the FusionAhrs library expects the positive-magnitude form for its
    // gate (FusionAhrs.c:316 takes cos(dip) and uses -sin(dip)·1 for m0.z), so
    // we keep `inclinationDeg = +78` as the value piped in; `inclinationDipRad`
    // is documentation + reference for the Python parity test.
    double inclinationDipRad;          // −1.1750679 (= −67.328°, free-field)
    double normReference;              // |m0| (1.0 after normalisation)
    double angleDiffFromModelMaxDeg;   // 6.0°
    double dipDiffFromModelMaxDeg;     // 3.5°
    double normDiffFromModelMax;       // 0.03 (3%)
    double magResThreshold;            // 0.9 (spec §38.6)
    double magResTimeUpSec;            // 0.6
    double magResTimeDownSec;          // 3.0
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

// §51.1 — reference magnetic-field vector m0 in the world frame, NWU with
// Z = up.  The textbook formula
//     m0 = (cos(D)·cos(I), -sin(D)·cos(I), sin(I))
// uses the signed dip I (negative in the Northern hemisphere where the field
// points down).  Our FusionAhrs library, in contrast, expects the
// positive-magnitude form because its internal RebuildM0() writes
//     m0.z = -sin(|dip|)
// to obtain the same downward-pointing component.  Both conventions agree on
// the vector direction; this helper returns whichever the caller asks for.
//
// `dipDeg` is the dip ANGLE (signed; positive when the field points UP in
// NWU, negative when it points DOWN — i.e. spec convention, opposite of the
// inclination MAGNITUDE the body override stores).
inline QVector3D referenceM0Vec(double dipDeg, double declinationDeg) {
    // Inline π/180 — the `constants` namespace is declared later in the
    // file and would otherwise create a forward-reference hazard.
    constexpr double kD2R = 0.017453292519943295;
    const double dipR = dipDeg * kD2R;
    const double decR = declinationDeg * kD2R;
    const double cI = std::cos(dipR);
    const double sI = std::sin(dipR);
    return QVector3D(float(cI * std::cos(decR)),
                     float(-cI * std::sin(decR)),
                     float(sI));
}

// Convenience: the body-override m0 vector that FusionAhrs actually uses
// internally (positive 78° dip → m0.z = -sin(78°) = -0.978).
inline QVector3D referenceM0BodyVec() {
    return referenceM0Vec(-kMagnet.inclinationDeg, kMagnet.declinationDeg);
}

// Convenience: the free-field m0 vector matching the spec literal
// inclinationDipRad = −1.175 (m0.z = sin(−1.175) = −0.923).
inline QVector3D referenceM0FreeFieldVec() {
    constexpr double kR2D = 57.29577951308232;
    return referenceM0Vec(kMagnet.inclinationDipRad * kR2D,
                          kMagnet.declinationDeg);
}

// ---------------------------------------------------------------------------
//  §51.6 / §43.10 — per-segment magnetic-gate relaxation table.
//
//  The base gate thresholds (kMagnet.angleDiffFromModelMaxDeg = 6°,
//  dipDiffFromModelMaxDeg = 3.5°, normDiffFromModelMax = 0.03) target a
//  body-mounted sensor far from external metal.  In practice the head,
//  hands, feet and (to a lesser extent) the chest read a noticeably
//  different field because of skull plates / glove electronics / shoe
//  steel / sternum-mounted electronics — so the strict body threshold
//  closes their magnetic gate almost permanently and their heading drifts
//  off the sole inertial input.  This table relaxes the per-segment gate
//  in proportion to the spec's FOX_Calib.e_* expectations.
//
//  Multipliers ≥ 1.  Caller multiplies them into the base thresholds.
//  Default for non-distorted segments (Pelvis, legs, shoulders) = 1.0.
// ---------------------------------------------------------------------------
struct MagGateRelax {
    float angleMul;     // multiplier on angleDiffFromModelMaxDeg (base 6°)
    float dipMul;       // multiplier on dipDiffFromModelMaxDeg   (base 3.5°)
    float normMul;      // multiplier on normDiffFromModelMax      (base 0.03)
};

inline constexpr std::array<MagGateRelax, kSegmentCount> kMagGateRelax = {{
    /* 0  Pelvis    */ { 1.5f, 1.3f, 1.0f + 0.20f }, // e_norm_pelvis=0.20
    /* 1  L5        */ { 1.0f, 1.0f, 1.0f },         // interpolated
    /* 2  L3        */ { 1.0f, 1.0f, 1.0f },
    /* 3  T12       */ { 1.0f, 1.0f, 1.0f },
    /* 4  T8        */ { 1.5f, 1.3f, 1.0f },         // e_inclx_sternum / spine
    /* 5  Neck      */ { 1.0f, 1.0f, 1.0f },
    /* 6  Head      */ { 4.0f, 2.5f, 1.0f + 0.30f }, // e_norm_head=0.30
    /* 7  RShoulder */ { 1.0f, 1.0f, 1.0f },         // close to torso
    /* 8  RUpperArm */ { 5.0f, 5.0f, 1.0f },         // e_incl_arm=30° → ×5
    /* 9  RForearm  */ { 5.0f, 5.0f, 1.0f },
    /* 10 RHand     */ { 5.0f, 5.0f, 1.0f + 0.35f }, // e_norm_hands=0.35
    /* 11 LShoulder */ { 1.0f, 1.0f, 1.0f },
    /* 12 LUpperArm */ { 5.0f, 5.0f, 1.0f },
    /* 13 LForearm  */ { 5.0f, 5.0f, 1.0f },
    /* 14 LHand     */ { 5.0f, 5.0f, 1.0f + 0.35f },
    /* 15 RUpperLeg */ { 1.0f, 1.0f, 1.0f },         // legs run cleaner
    /* 16 RLowerLeg */ { 1.5f, 1.5f, 1.0f },         // some shin metal
    /* 17 RFoot     */ { 2.0f, 2.0f, 1.0f + 0.22f }, // 0.1·e_mag_feet=0.22
    /* 18 RToe      */ { 1.0f, 1.0f, 1.0f },         // interpolated
    /* 19 LUpperLeg */ { 1.0f, 1.0f, 1.0f },
    /* 20 LLowerLeg */ { 1.5f, 1.5f, 1.0f },
    /* 21 LFoot     */ { 2.0f, 2.0f, 1.0f + 0.22f },
    /* 22 LToe      */ { 1.0f, 1.0f, 1.0f },
}};

// ---------------------------------------------------------------------------
//  §43.10 — per-segment IMU chip type.  The FOX_IMU_x3 chip (newest revision)
//  has 60× higher magnetometer noise density (ndCoefficient = 0.25 vs 0.004
//  for w2/x2).  Sensors using x3 need a substantially wider magnetic gate
//  or they essentially never open it.  Default = ImuW2 for compatibility
//  with existing deployments; override per segment in the table below as
//  hardware information becomes available.
// ---------------------------------------------------------------------------
enum class ImuChipType : std::uint8_t { W2, X2, X3 };

inline constexpr std::array<ImuChipType, kSegmentCount> kImuChipPerSeg = {{
    /*  0..22 */
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
    ImuChipType::W2, ImuChipType::W2, ImuChipType::W2,
}};

// Magnetic noise ratio for the chip's measurement model.  ndCoefficient
// scales the std-dev linearly; the gate threshold scales by the square
// root because we compare a dip angle against a noise standard deviation.
//   x3 / w2 = sqrt(0.25 / 0.004) ≈ 7.91
inline float magNoiseScaleForChip(ImuChipType c) {
    switch (c) {
        case ImuChipType::X3: return 7.91f;
        case ImuChipType::W2:
        case ImuChipType::X2:
        default: return 1.0f;
    }
}

// ---------------------------------------------------------------------------
//  §38.5 — skin artifact (Gauss-Markov soft-tissue model).  This is the
//  «viscosity» of the IMU relative to the underlying bone.
//      x_k = exp(-Δt/τ) · x_{k−1} + ε,  ε ~ N(0, σ²)
//  These numbers are spec defaults; subject calibration can shrink them.
// ---------------------------------------------------------------------------
struct SkinParams {
    double tauSec;             // 0.15 s — base relaxation time constant
    double sigmaOriDeg;        // 3.0°   — orientation artifact 1-σ
    double sigmaPosM;          // 0.02 m — position artifact 1-σ
    double sigmaOriGmDeg;      // 2.5°   — GM-equivalent ori σ
    double sigmaPosGmM;        // 0.025 m
    double initStdOriBodyDeg;        // 45° — pre-calibration prior
    double initStdSensorToBodyDeg;   // 1.5°
    double initStdSensorToBodyPos;   // 0.01 m
    double stdSensorToBodyOriFloorDeg;  // 0.3°
    double stdSensorToBodyPosFloor;     // 0.004 m
    bool   doGaussMarkov;
    bool   doChangeTauInCF;
    bool   doSkinArtifactBasedOnDynamics;
    // §38.5 doSkinArtifactBasedOnDynamics — adaptive τ.  Spec implies the
    // GM time constant shortens with high body dynamics (τ → 0.05 s on
    // peak motion, the artifact has to track and relax fast) and lengthens
    // when the segment is still (τ → 0.30 s, slow drift dominated by
    // gravity loading).  We interpolate between the two on per-sensor
    // motion energy ω in rad/s, normalised by tauMotionRefRad.
    double tauFastSec;         // 0.05  s — τ at peak motion energy
    double tauSlowSec;         // 0.30  s — τ at full rest
    double tauMotionRefRad;    // 1.0 rad/s — soft-clip for ω normalisation
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
    .doSkinArtifactBasedOnDynamics= true,
    .tauFastSec                   = 0.05,
    .tauSlowSec                   = 0.30,
    .tauMotionRefRad              = 1.0,
};

// ---------------------------------------------------------------------------
//  §38.6 — filter time constants and §38.7 — process/measurement noise.
// ---------------------------------------------------------------------------
struct FilterParams {
    double tauAcc;                 // 10  — accelerometer smoothing
    double tauFGyrLpfDynamic;      // 6
    double tauM0AvgFast;           // 30
    double tauM0AvgMedium;         // 120
};
inline constexpr FilterParams kFilter = {
    .tauAcc            = 10.0,
    .tauFGyrLpfDynamic = 6.0,
    .tauM0AvgFast      = 30.0,
    .tauM0AvgMedium    = 120.0,
};

struct EstimatorWeights {
    // §38.7 — process & measurement noise per axis/coupling.
    std::array<double, kLumpGroups> sdIntAccToVel;     // 7 lumps
    double sdIntVelToPos;          // 1e-5 (very tight)
    double sdLumpJoint;            // 0.025 (uniform)
    double stdOriFreeze;           // 0.01
    double stdPosFreeze;           // 1e-4
    double stdOriLocalBodyStillDeg;// 1.5°
    double initStdVel;             // 2 m/s
    double initStdAccBias;         // 0.1
    double initStdGyrBiasDeg;      // 0.4°
    double gyrBiasStdMinDeg;       // 0.005
    double gyrBiasStdMaxDeg;       // 0.07
    double multiLevelZhcClipVert;  // 0.005
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

// ---------------------------------------------------------------------------
//  §41.3 / §44.6 / §138.31-32 — aiding-bias Gauss-Markov.
//
//  Each ZUPT / aiding hint carries a slowly-evolving residual bias b that
//  tracks the persistent offset between the predicted contact velocity and
//  the measured one.  Spec model:
//      b(k+1) = b(k) · exp(−dt / c_t),   c_t = 0.9 s   (predict step)
//      b      += c_v · r_v_post,          c_v = 0.01    (after WLS update)
//  Without this bias the ZUPT row weight oscillates frame-to-frame when the
//  contact probability hovers around 0.5 (heel-off / toe-on transitions),
//  visibly jittering the foot anchor.  With the bias absorbed into the
//  residual, the anchor stays put across the noisy threshold crossing.
// ---------------------------------------------------------------------------
struct AidingBiasParams {
    double cT;        // 0.9 s — bias time constant (larger = more smoothing)
    double cV;        // 0.01  — fraction of post-residual absorbed each frame
};
inline constexpr AidingBiasParams kAidingBias = {
    .cT = 0.9,
    .cV = 0.01,
};

// §44.2 — per-lump per-axis acceleration→velocity integration noise (m/s²),
// 7 lumps × 3 axes (X,Y,Z).  The diagonal-only kEstimator.sdIntAccToVel is the
// max-of-axes summary used by legacy code; the WLS body solver wants the full
// matrix because vertical Z is consistently tighter than the horizontal plane
// (gravity anchor).  Order matches kLumps[] (upperbody/rightleg/rightfoot/
// leftleg/leftfoot/rightarm/leftarm).
inline constexpr std::array<std::array<double, 3>, kLumpGroups> kSdIntAccToVelXYZ = {{
    {{  2.0,  2.0,  1.0 }},   // 0 upperbody
    {{  4.0,  4.0,  2.0 }},   // 1 rightleg
    {{  8.0,  8.0,  4.0 }},   // 2 rightfoot
    {{  4.0,  4.0,  2.0 }},   // 3 leftleg
    {{  8.0,  8.0,  4.0 }},   // 4 leftfoot
    {{ 20.0, 20.0, 20.0 }},   // 5 rightarm
    {{ 20.0, 20.0, 20.0 }},   // 6 leftarm
}};

// §44.3 Б — bone-length kinematic constraint sd (m).  Bones don't stretch.
inline constexpr double kStdJointBoneLength = 0.0002;
// §52.1 — ZUPT velocity sd at contact point (m/s).  Hard anchor.
inline constexpr double kStdSamePosMeasXY   = 0.0003;
// §44.3 Г — ZUPT same-position sd Z (3D vs XY-plane).
inline constexpr double kStdSamePosMeasZ3d  = 0.002;
inline constexpr double kStdSamePosMeasZ    = 10.0;   // ZHC off: Z is free vertically
// §44.5 — Gauss-Newton settings for the body solver.
inline constexpr int    kMaxIKSteps         = 2;        // FOX_FE.LX.maxIKsteps
inline constexpr double kJointLaxitySolver  = 0.005;    // FOX_FE.LX.jointLaxity (rad)
inline constexpr double kHypExtPenaltySd    = 0.0002;   // hard barrier sd for knee/elbow ≥ 0
// §38.2 — body-wide height tolerance band used by the air-phase contact feature.
//   dLevel.default — general height tolerance (any body point near floor)
//   dLevel.foot    — tighter band reserved for foot-floor contact (already at kAir[6])
// The contact detector blends these to score "low-Z" without committing to a
// strict floor distance.  Values are the calibrated body-model defaults.
inline constexpr double kDLevelDefault      = 0.175;    // general height tolerance (m)
inline constexpr double kDLevelFoot         = 0.10;     // foot-floor tolerance       (m)

// §45.1 — spineNeck namespace coefficients (separate from c_spine, fox_definitions.xsb
// spineNeck.*).  These are the per-joint coupling strengths used by the
// spinal-rhythm distribution and the corresponding measurement sd's.
struct SpineNeckParams {
    double cL3;        // 0.65  — L3 fraction (matches c_spine[2])
    double cL5;        // 0.45  — L5 fraction (matches c_spine[1])
    double cT12;       // 0.85  — T12 fraction (matches c_spine[3])
    double cNeck;      // 0.35  — Neck axial-twist fraction (matches c_spine[4])
    double stdNeck;    // 0.001 — measurement sd for neck constraint (rad)
    double stdSpine;   // 0.001 — measurement sd for spine constraints (rad)
};
inline constexpr SpineNeckParams kSpineNeck = {
    .cL3      = 0.65,
    .cL5      = 0.45,
    .cT12     = 0.85,
    .cNeck    = 0.35,
    .stdNeck  = 0.001,
    .stdSpine = 0.001,
};

// §46.2 — scapulo-humeral piecewise-linear envelope.
inline constexpr double kScapHumThetaLowDeg  = 60.0;   // below: c_arms_effective = 0.95
inline constexpr double kScapHumThetaHighDeg = 90.0;   // above: c_arms_effective = 0.99
// §47.2 — knee screw-home maximum axial rotation.
inline constexpr double kKneeScrewMaxDeg     = 15.0;
// §48.1 — ankle dorsiflex limit (extension; spec says ~ -45°).
inline constexpr double kAnkleDorsiLimitRad  = 45.0 * 0.017453292519943295;
// §48.2 — toe rocker switching thresholds (rad).
inline constexpr double kToeRockerLowRad     = 5.0  * 0.017453292519943295;  // 5°
inline constexpr double kToeRockerHighRad    = 30.0 * 0.017453292519943295;  // 30°

// §226 — air[]: contact-air probability detector (11 coefficients).
inline constexpr std::array<double, 11> kAir = {
    -3.0, 0.0, 0.3, 3.0, 0.1, -0.05, 0.3, 0.0, -6.0, -0.05, -0.02
};
// §227 — com[]: CoM-distance contact factor (5 coefficients).
inline constexpr std::array<double, 5> kCom = { 0.05, 0.5, 0.08, 0.0, 0.6 };
// §228.1 — acc[]: ||a_lp|| → contact-likelihood (4 coefficients).
inline constexpr std::array<double, 4> kAccProb = { 0.6, 0.3, 0.95, 0.4 };
// §228.2 — vel[]: ||v_p|| → contact-likelihood (4 coefficients).
inline constexpr std::array<double, 4> kVelProb = { 0.1, 0.4, 0.95, 0.2 };
// §229 — general[]: residual contact factor (4 coefficients).
inline constexpr std::array<double, 4> kGeneralProb = { 0.5, 0.4, 0.275, 0.0015 };
// §230 — peakDetection[]: heel-strike peak detection (7 coefficients).
inline constexpr std::array<double, 7> kPeakDetection = {
    3.6515, 0.1667, 20.0, 10.0, 1.5, 1.0, 0.025
};
// §231 — samepos[]: same-position contact handling (8 coefficients).
inline constexpr std::array<double, 8> kSamepos = {
    0.02, 0.02, 0.07, 0.05, 0.005, 0.005, 0.15, 0.8
};
// §42.3 — outlier rejection thresholds.
struct OutlierRej {
    double outRejTh1;             // 100
    double outRejTh2;             // 10
    double outRejTh3;             // 2.5
    double jointResTh[6];         // §42.3 outRejJointResTh1..6
    double footSlidingTh[3];      // §42.3 outRejFootSlidingTh1/2/3
    double countTh[3];            // §42.3 outRejCountTh
    double jointResWin1;          // 0.1  (s)
    double jointResWin2;          // 0.05 (s)
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

// §52.6 — multi-level floor bank (stairs).
struct MultiLevelParams {
    int    averagingStairHeight;     // 5    (number of steps to average)
    double maxDevFromAvgStairHeight; // 0.05 m
    int    maxLevelsToDetectStairWalking; // 10
    double newLevelMeasureThreshold; // 0.03
    double sameLevelMargin;          // 0.09
    double zhcClipSdVertical;        // 0.005
    double zhcLevelMargin1;          // 0.12
    double zhcLevelMargin2;          // 0.09
    double zhcMaximumLikelihood;     // 0.9
    double zhcMaxLikelihoodBoost;    // 1000
    double zhcMinimumLikelihood;     // 0.4
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
};

// §51.6 — FOX_Calib.e_* per-region magnetic calibration constants (deg / unitless).
// These are expected per-segment dips and field-norm fractions, used by the
// per-segment magnetometer-gate to compensate for body-mounted distortion
// (different limbs see different external fields due to magnetisation of the
// suit hardware).
struct CalibMagE {
    double e_dip_mag;        // 78°  primary expected dip (body sensors)
    double e_dip_mag2;       // 85°  secondary (auxiliary placement)
    double e_incl_arm;       // 30°  expected N-pose arm inclination
    double e_inclx_pelvis;   // 45°  expected pelvis X-inclination
    double e_inclx_sternum;  // 40°  for T8
    double e_incly_pelvis;   // 25°
    double e_mag_diff;       // 0.5  max field diff between segments
    double e_mag_feet;       // 2.2  field coefficient for feet
    double e_mag_pelvis;     // 1.5
    double e_mag_pelvis2;    // 0.5
    double e_norm_hands;     // 0.35 expected field norm for hands
    double e_norm_head;      // 0.3
    double e_norm_pelvis;    // 0.2
    double e_sternum_pelvis; // 25°  T8↔Pelvis difference
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
    .e_sternum_pelvis = 25.0,
};

// §87 / §1582 — foot contact-point local coordinates (metres, segment frame).
// Points enumerated in spec §44.10 (point ids match): 3=pHeel, 4=p1stMet,
// 5=p5thMet, 6=pBall/Toe.  Y is mirrored for the left foot.
struct FootPoint {
    int       pointId;             // §44.10 point index
    QVector3D r_local;             // local-frame coordinate (m)
};
inline constexpr std::array<FootPoint, 4> kFootPointsRight = {{
    { 3, QVector3D(-0.036f,  0.000f, -0.080f) },   // pHeel
    { 4, QVector3D( 0.116f, -0.038f, -0.080f) },   // p1stMet
    { 5, QVector3D( 0.116f,  0.036f, -0.080f) },   // p5thMet
    { 6, QVector3D( 0.140f,  0.000f, -0.080f) },   // pBall/Toe6
}};
inline constexpr std::array<FootPoint, 4> kFootPointsLeft = {{
    { 3, QVector3D(-0.036f,  0.000f, -0.080f) },
    { 4, QVector3D( 0.116f,  0.038f, -0.080f) },
    { 5, QVector3D( 0.116f, -0.036f, -0.080f) },
    { 6, QVector3D( 0.140f,  0.000f, -0.080f) },
}};
// §82, §86 — toe-tip point on Toe segment (point id 2 = pToe in the spec).
inline constexpr QVector3D kToeTipPoint = QVector3D(0.064f, 0.0f, -0.015f);

// §91–§93 — finger ROM (per joint, degrees).  Twenty hand joints per side
// (matches FOX_Skeleton.rightHand.jointsCount=19 plus the wrist; we mirror
// the same table for the left hand because the spec spells out only the
// right-hand layout and notes (§93) that left is a Y-mirror).
//
// Order is the natural digit order: thumb → index → middle → ring → little,
// with each finger contributing CMC/MCP/PIP/DIP except the thumb which is
// CMC/MCP/IP (3 joints).  Total: 3 + 4·4 = 19 joints per hand.
struct FingerRom {
    const char* label;
    double flxMin, flxMax;   // flexion / extension (Y)
    double abdMin, abdMax;   // abduction / adduction (X — only at MCP)
    double rotMin, rotMax;   // axial rotation (Z — only at thumb CMC)
};
inline constexpr std::array<FingerRom, 19> kFingerRom = {{
    /* 0  thumb CMC (saddle) */ { "TM_CMC",   -15.0,  60.0,  -45.0,  45.0,  -20.0, 30.0 },
    /* 1  thumb MCP          */ { "TM_MCP",     0.0,  60.0,   -5.0,   5.0,    0.0,  0.0 },
    /* 2  thumb IP           */ { "TM_IP",    -10.0,  80.0,    0.0,   0.0,    0.0,  0.0 },
    /* 3  index MCP          */ { "II_MCP",   -20.0,  90.0,  -25.0,  25.0,    0.0,  0.0 },
    /* 4  index PIP          */ { "II_PIP",     0.0, 110.0,    0.0,   0.0,    0.0,  0.0 },
    /* 5  index DIP          */ { "II_DIP",     0.0,  80.0,    0.0,   0.0,    0.0,  0.0 },
    /* 6  middle MCP         */ { "MI_MCP",   -20.0,  90.0,  -25.0,  25.0,    0.0,  0.0 },
    /* 7  middle PIP         */ { "MI_PIP",     0.0, 110.0,    0.0,   0.0,    0.0,  0.0 },
    /* 8  middle DIP         */ { "MI_DIP",     0.0,  80.0,    0.0,   0.0,    0.0,  0.0 },
    /* 9  ring MCP           */ { "RI_MCP",   -20.0,  90.0,  -25.0,  25.0,    0.0,  0.0 },
    /* 10 ring PIP           */ { "RI_PIP",     0.0, 110.0,    0.0,   0.0,    0.0,  0.0 },
    /* 11 ring DIP           */ { "RI_DIP",     0.0,  80.0,    0.0,   0.0,    0.0,  0.0 },
    /* 12 little MCP         */ { "LI_MCP",   -20.0,  90.0,  -30.0,  30.0,    0.0,  0.0 },
    /* 13 little PIP         */ { "LI_PIP",     0.0, 110.0,    0.0,   0.0,    0.0,  0.0 },
    /* 14 little DIP         */ { "LI_DIP",     0.0,  80.0,    0.0,   0.0,    0.0,  0.0 },
    /* 15 carpus rotate      */ { "Carpus",   -10.0,  10.0,    0.0,   0.0,    0.0,  0.0 },
    /* 16 thumb opposition   */ { "TM_OPP",     0.0,  45.0,    0.0,   0.0,    0.0,  0.0 },
    /* 17 finger spread      */ { "Spread",   -20.0,  20.0,    0.0,   0.0,    0.0,  0.0 },
    /* 18 reserved / fillers */ { "RSV",       -5.0,   5.0,    0.0,   0.0,    0.0,  0.0 },
}};

// §44.12 — pose-quality residual bands (mean |r| across active rows).
inline constexpr std::array<double, 5> kPoseQualityResidBands = {
    0.005, 0.02, 0.05, 0.1, 1.0      // <0.005 excellent, ..., >0.1 invalid
};
// Pose quality integer band names — match spec §44.12.
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

// ---------------------------------------------------------------------------
//  §30.4 — per-joint ergonomic-extractor type (5 handlers).
//  Encodes left/right mirroring and special-case foot handling without a
//  single all-segments formula.
//    type 0 — axial / midline (spine, neck, head) — no L/R sign flip
//    type 1 — right-side limb (right arm, right leg)
//    type 2 — left-side  limb (left arm, left leg)
//    type 3 — right foot   (specialised contact/ankle handling)
//    type 4 — left  foot
//  Spec table, joint_idx = 0..21:
//    00 00 00 00 00 00 00 01 01 01 01 02 02 02 02 01 03 01 03 02 04 02
// ---------------------------------------------------------------------------
inline constexpr std::array<std::uint8_t, kJointCount> kErgoHandler = {
    // Spec raw bytes verbatim; joint index follows kJoints[] above.  Comments
    // give the joint label at each index — the type/label mapping is what the
    // spec ships and is not the obvious anatomical L/R split.
    0,   // 0  jL5S1            — axial
    0,   // 1  jL4L3            — axial
    0,   // 2  jL1T12           — axial
    0,   // 3  jT9T8            — axial
    0,   // 4  jT1C7            — axial
    0,   // 5  jC1Head          — axial
    0,   // 6  jRightT4Shoulder — axial (despite name; spec table classifies as type 0)
    1,   // 7  jRightShoulder
    1,   // 8  jRightElbow
    1,   // 9  jRightWrist
    1,   // 10 jLeftT4Shoulder  — type-1 handler (spec verbatim, see narrative §30.4 [S])
    2,   // 11 jLeftShoulder
    2,   // 12 jLeftElbow
    2,   // 13 jLeftWrist
    2,   // 14 jRightHip        — type-2 handler (spec verbatim)
    1,   // 15 jRightKnee
    3,   // 16 jRightAnkle      — right-foot specialised handler
    1,   // 17 jRightBallFoot
    3,   // 18 jLeftHip         — type-3 handler (spec verbatim)
    2,   // 19 jLeftKnee
    4,   // 20 jLeftAnkle       — left-foot specialised handler
    2,   // 21 jLeftBallFoot
};

// ---------------------------------------------------------------------------
//  Appendix A — numerical constants reproduced verbatim from the binary.
// ---------------------------------------------------------------------------
namespace constants {
    inline constexpr double kRad2Deg   = 57.29577951308232;   // 180/π
    inline constexpr double kDeg2Rad   = 0.017453292519943295; // π/180
    inline constexpr double kPi_2      = 1.5707963267948966;   // π/2
    inline constexpr double kPi        = 3.141592653589793;
    inline constexpr double kSlerpEps  = 1.0e-6;               // SLERP small-angle threshold
    inline constexpr double kNLerpA    = 0.2;                  // NLERP poly coeffs
    inline constexpr double kNLerpB    = 0.8;
    inline constexpr double kNLerpC    = 1.0 / 3.0;
    inline constexpr double kSolverC1  = 272332.63;            // rational-form fit (§13)
    inline constexpr double kSolverC2  = 40680634.23;
    inline constexpr double kSolverAlpha = 0.25;               // damped-step factor §31.2
    inline constexpr double kSolverLambda = 0.01;              // LM damping

    // Spec §41.1 — fox_definitions.xsb FOX_FE.gravity = 9.812687 m/s² (vector
    // (0, 0, -9.812687) with Z = up).  This is the gravity magnitude the
    // pose-engine EKF/MNK uses internally; the ISO standard 9.80665 m/s² is
    // still used at the sensor-IMU layer (calibrated-IMU → G unit conversion)
    // because raw-IMU output is denominated in standard g.
    inline constexpr double kGravityMs2 = 9.812687;
    // Spec §41.1 — FOX_FE.SampleRate = 240 Hz → dt = 1/240 ≈ 4.17 ms.
    inline constexpr double kSampleRateHz = 240.0;
    inline constexpr double kSampleDtSec  = 1.0 / kSampleRateHz;
}

// ---------------------------------------------------------------------------
//  Helpers exposed to the rest of the engine.
// ---------------------------------------------------------------------------

// Total mass ratio (should be ≈ 100; not exactly because the spec values are
// rounded to four decimals).  Computed at compile time for the sanity check.
constexpr double totalMassRatio() {
    double s = 0.0;
    for (double m : kMassRatio) s += m;
    return s;
}

// Anthropometric scale: bone vectors are stored at refHeight = 1.75 m.
// Subject vectors = kBoneVec[seg] * scaleFor(subjectHeightM).
inline double scaleFor(double subjectHeightM) {
    return (subjectHeightM > 1e-3) ? (subjectHeightM / kRefHeightM) : 1.0;
}

// Ankle-joint world height for a foot resting on the floor.  Equal to the
// vertical offset of the foot origin above the heel/ball points (=0.080 m
// at reference height), then scaled to the subject.
inline double ankleHeightM(double subjectHeightM) {
    return scaleFor(subjectHeightM) *
           std::abs(double(kFootPointsRight[0].r_local.z()));
}

// Hip-joint world height in a relaxed standing pose: ankle-to-floor offset
// + lower-leg + upper-leg bone lengths.  Pelvis origin in the skeleton tree
// sits at this Z above the floor when both feet are planted.
inline double pelvisStandHeightM(double subjectHeightM) {
    return scaleFor(subjectHeightM) * (
        std::abs(double(kFootPointsRight[0].r_local.z())) +
        std::abs(double(kSensorToBone[16].L_bone.z())) +     // RLowerLeg
        std::abs(double(kSensorToBone[15].L_bone.z())));     // RUpperLeg
}

// Hip-joint world height in a seated pose: thigh roughly horizontal, only
// the shin contributes a vertical component.  Used as the per-frame target
// for the "sitting" locomotion classification.
inline double pelvisSitHeightM(double subjectHeightM) {
    return scaleFor(subjectHeightM) * (
        std::abs(double(kFootPointsRight[0].r_local.z())) +
        std::abs(double(kSensorToBone[16].L_bone.z())));
}

// Sum of spine bone lengths Pelvis → Head (six segments).  Used as the
// "trunk length" anthropometric default when the user has not measured it.
inline double trunkLengthM(double subjectHeightM) {
    double s = 0.0;
    for (int i = 0; i <= 5; ++i)
        s += std::abs(double(kSensorToBone[i].L_bone.z()));
    return scaleFor(subjectHeightM) * s;
}

// Spec §12.1 — whole-body centre of mass:  r_cm = Σ m_i·r_i / Σ m_i.
// `segCenters[i]` is the world-frame centre of segment i.  Returns the
// mass-weighted mean using kMassRatio[] as the m_i weights (the absolute
// units do not matter — only the ratio of mass).  No external normalisation
// of segCenters is performed.  If `M` is non-null, it receives Σ m_i.
inline QVector3D centerOfMass(const std::array<QVector3D, kSegmentCount>& segCenters,
                              double* M = nullptr) {
    double sx = 0.0, sy = 0.0, sz = 0.0, sm = 0.0;
    for (int i = 0; i < kSegmentCount; ++i) {
        const double m = kMassRatio[i];
        sx += m * double(segCenters[i].x());
        sy += m * double(segCenters[i].y());
        sz += m * double(segCenters[i].z());
        sm += m;
    }
    if (M) *M = sm;
    if (sm <= 0.0) return QVector3D(0, 0, 0);
    const double inv = 1.0 / sm;
    return QVector3D(float(sx * inv), float(sy * inv), float(sz * inv));
}

// Returns true if the segment is one of the 17 IMU-instrumented sensors.
inline bool hasSensor(int seg) {
    return (seg >= 0 && seg < kSegmentCount) ? kSensorPresent[seg] : false;
}

// Returns the lump group index (0..6) for `jointIdx`, or −1 if uncoupled.
inline int lumpOf(int jointIdx) {
    return (jointIdx >= 0 && jointIdx < kJointCount) ? kJointLump[jointIdx] : -1;
}

// Returns the ergonomic-handler type (0..4) for `jointIdx`.
inline int ergoTypeOf(int jointIdx) {
    return (jointIdx >= 0 && jointIdx < kJointCount) ? int(kErgoHandler[jointIdx]) : 0;
}

// ---------------------------------------------------------------------------
//  §1699-1722 — FoxSPC sensor-placement classifier (sklearn RBF-SVM).
//  17 classes × 315 features.  Class index order is alphabetical (the order
//  the ONNX `label` output emits — see §1719).
// ---------------------------------------------------------------------------
constexpr int kSpcClassCount   = 17;
constexpr int kSpcFeatureCount = 315;

inline constexpr std::array<const char*, kSpcClassCount> kSensorPlacementClasses = {
    "Head", "LeftFoot", "LeftForeArm", "LeftHand", "LeftLowerLeg",
    "LeftShoulder", "LeftUpperArm", "LeftUpperLeg", "Pelvis",
    "RightFoot", "RightForeArm", "RightHand", "RightLowerLeg",
    "RightShoulder", "RightUpperArm", "RightUpperLeg", "T8",
};

// §1719 — class index → 23-segment SEG_* (matches main.h enum + foxbody segment
// ordering).  Pelvis=0, T8=4, Head=6 etc.
inline constexpr std::array<int, kSpcClassCount> kClassToSeg = {
    /* 0  Head          */  6,
    /* 1  LeftFoot      */ 21,
    /* 2  LeftForeArm   */ 13,
    /* 3  LeftHand      */ 14,
    /* 4  LeftLowerLeg  */ 20,
    /* 5  LeftShoulder  */ 11,
    /* 6  LeftUpperArm  */ 12,
    /* 7  LeftUpperLeg  */ 19,
    /* 8  Pelvis        */  0,
    /* 9  RightFoot     */ 17,
    /* 10 RightForeArm  */  9,
    /* 11 RightHand     */ 10,
    /* 12 RightLowerLeg */ 16,
    /* 13 RightShoulder */  7,
    /* 14 RightUpperArm */  8,
    /* 15 RightUpperLeg */ 15,
    /* 16 T8            */  4,
};

// Inverse mapping: SEG_* → class index, or −1 for segments without an IMU
// (L5, L3, T12, Neck, RToe, LToe — see kSensorPresent).
inline constexpr std::array<int, kSegmentCount> kSegToClass = {
    /* 0  Pelvis    */  8,
    /* 1  L5        */ -1,
    /* 2  L3        */ -1,
    /* 3  T12       */ -1,
    /* 4  T8        */ 16,
    /* 5  Neck      */ -1,
    /* 6  Head      */  0,
    /* 7  RShoulder */ 13,
    /* 8  RUpperArm */ 14,
    /* 9  RForeArm  */ 10,
    /* 10 RHand     */ 11,
    /* 11 LShoulder */  5,
    /* 12 LUpperArm */  6,
    /* 13 LForeArm  */  2,
    /* 14 LHand     */  3,
    /* 15 RUpperLeg */ 15,
    /* 16 RLowerLeg */ 12,
    /* 17 RFoot     */  9,
    /* 18 RToe      */ -1,
    /* 19 LUpperLeg */  7,
    /* 20 LLowerLeg */  4,
    /* 21 LFoot     */  1,
    /* 22 LToe      */ -1,
};

// FoxSPC feature spec (§1700–1716).  315 features defined by:
//   epoch  ∈ { calibration, leftArmRaise, rightArmRaise,
//              leftLegRaise, rightLegRaise }      — 5 epochs
//   signal ∈ { Acc, Gyr }                          — 2 sensor channels
//   axis   ∈ { x, y, z, xAbs, yAbs, zAbs, Normxyz } — 7 virtual axes
//   band   ∈ { none, freqBand0.5To4.0,
//              freqBand4.5To10.0, freqBand10.0To-1.0 } — 4 bands
//   stat   — see SpcStat below
enum class SpcEpoch  : std::uint8_t {
    Calibration, LeftArmRaise, RightArmRaise, LeftLegRaise, RightLegRaise
};
enum class SpcSignal : std::uint8_t { Acc, Gyr };
enum class SpcAxis   : std::uint8_t { X, Y, Z, XAbs, YAbs, ZAbs, Normxyz };
enum class SpcBand   : std::uint8_t {
    None, Band0p5To4, Band4p5To10, Band10ToNyq
};
// §1703 + §1704 statistics — combined enum for the parser.
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

// Defined in foxbody.cpp: parses kFeatureNames into SpcFeatureSpec, then
// derives per-feature min/max from spec §1705 typical ranges.
extern const std::array<const char*,     kSpcFeatureCount> kFeatureNames;
extern const std::array<SpcFeatureSpec,  kSpcFeatureCount> kFeatureSpecs;
extern const std::array<float,           kSpcFeatureCount> kFeatureMin;
extern const std::array<float,           kSpcFeatureCount> kFeatureMax;

}  // namespace fox::body
