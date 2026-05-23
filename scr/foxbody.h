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
//  T-pose (arms horizontal) — every segment is identity in world frame:
//  the body is upright, arms extend laterally along the standard MVN body
//  axes, so the bone-to-world rotation matches the skeleton's reference
//  pose with no per-segment correction.
//
//  N-pose (arms down at sides) — the arm chain (UpperArm, ForeArm, Hand)
//  rotates 90° about ±X relative to the T-pose, so the bones point along
//  −Z in world frame.  Right side rotates +90° about +X (→ +Y axis maps
//  to +Z, so the T-pose lateral arm (−Y) maps to −Z = down).  Left side
//  rotates +90° about −X (mirrored).  Spec §174.3 gives the example for
//  RUpperArm verbatim: q_эталон_9 = (0.7071, 0.7071, 0, 0).
// ---------------------------------------------------------------------------
inline constexpr std::array<Quat, kSegmentCount> kRefQuatT = {{
    /* 0  Pelvis    */ Quat(1, 0, 0, 0),
    /* 1  L5        */ Quat(1, 0, 0, 0),
    /* 2  L3        */ Quat(1, 0, 0, 0),
    /* 3  T12       */ Quat(1, 0, 0, 0),
    /* 4  T8        */ Quat(1, 0, 0, 0),
    /* 5  Neck      */ Quat(1, 0, 0, 0),
    /* 6  Head      */ Quat(1, 0, 0, 0),
    /* 7  RShoulder */ Quat(1, 0, 0, 0),
    /* 8  RUpperArm */ Quat(1, 0, 0, 0),
    /* 9  RForeArm  */ Quat(1, 0, 0, 0),
    /* 10 RHand     */ Quat(1, 0, 0, 0),
    /* 11 LShoulder */ Quat(1, 0, 0, 0),
    /* 12 LUpperArm */ Quat(1, 0, 0, 0),
    /* 13 LForeArm  */ Quat(1, 0, 0, 0),
    /* 14 LHand     */ Quat(1, 0, 0, 0),
    /* 15 RUpperLeg */ Quat(1, 0, 0, 0),
    /* 16 RLowerLeg */ Quat(1, 0, 0, 0),
    /* 17 RFoot     */ Quat(1, 0, 0, 0),
    /* 18 RToe      */ Quat(1, 0, 0, 0),
    /* 19 LUpperLeg */ Quat(1, 0, 0, 0),
    /* 20 LLowerLeg */ Quat(1, 0, 0, 0),
    /* 21 LFoot     */ Quat(1, 0, 0, 0),
    /* 22 LToe      */ Quat(1, 0, 0, 0),
}};

// Spec §24.2 — right arm chain: +90° about +X.  Left arm chain mirror:
// +90° about −X (equivalent: −90° about +X).  Magnitude = √½ ≈ 0.7071068.
inline constexpr double kRefSqrtHalf = 0.7071067811865475;
inline constexpr std::array<Quat, kSegmentCount> kRefQuatN = {{
    /* 0  Pelvis    */ Quat(1, 0, 0, 0),
    /* 1  L5        */ Quat(1, 0, 0, 0),
    /* 2  L3        */ Quat(1, 0, 0, 0),
    /* 3  T12       */ Quat(1, 0, 0, 0),
    /* 4  T8        */ Quat(1, 0, 0, 0),
    /* 5  Neck      */ Quat(1, 0, 0, 0),
    /* 6  Head      */ Quat(1, 0, 0, 0),
    /* 7  RShoulder */ Quat(1, 0, 0, 0),
    /* 8  RUpperArm */ Quat(kRefSqrtHalf,  kRefSqrtHalf, 0, 0),
    /* 9  RForeArm  */ Quat(kRefSqrtHalf,  kRefSqrtHalf, 0, 0),
    /* 10 RHand     */ Quat(kRefSqrtHalf,  kRefSqrtHalf, 0, 0),
    /* 11 LShoulder */ Quat(1, 0, 0, 0),
    /* 12 LUpperArm */ Quat(kRefSqrtHalf, -kRefSqrtHalf, 0, 0),
    /* 13 LForeArm  */ Quat(kRefSqrtHalf, -kRefSqrtHalf, 0, 0),
    /* 14 LHand     */ Quat(kRefSqrtHalf, -kRefSqrtHalf, 0, 0),
    /* 15 RUpperLeg */ Quat(1, 0, 0, 0),
    /* 16 RLowerLeg */ Quat(1, 0, 0, 0),
    /* 17 RFoot     */ Quat(1, 0, 0, 0),
    /* 18 RToe      */ Quat(1, 0, 0, 0),
    /* 19 LUpperLeg */ Quat(1, 0, 0, 0),
    /* 20 LLowerLeg */ Quat(1, 0, 0, 0),
    /* 21 LFoot     */ Quat(1, 0, 0, 0),
    /* 22 LToe      */ Quat(1, 0, 0, 0),
}};

// Stub offsets (pelvis→hip-joint, T8→shoulder-joint) used by the FK dummy chain.
// Hip half-width ±0.08 m Y (spec §37.6: «таз: бёдра ±0.08 по Y, ширина таза 0.16 м»).
// Shoulder half-width ±0.16 m Y (typical bi-acromial half on a 1.75 m subject;
// spec §37.5 says shoulderWidth is a measured dimension — at default body it
// matches the existing 0.10–0.16 range used by SkeletonXsens).
inline constexpr float kHipHalfY      = 0.080f;
inline constexpr float kShoulderHalfY = 0.160f;
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
    double inclinationDeg;             // I (spec §37.5 default: 78°)
    double inclinationDeg2;            // secondary I (spec §37.5: 85°)
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
    .normReference             = 1.0,
    .angleDiffFromModelMaxDeg  = 6.0,
    .dipDiffFromModelMaxDeg    = 3.5,
    .normDiffFromModelMax      = 0.03,
    .magResThreshold           = 0.9,
    .magResTimeUpSec           = 0.6,
    .magResTimeDownSec         = 3.0,
};

// ---------------------------------------------------------------------------
//  §38.5 — skin artifact (Gauss-Markov soft-tissue model).  This is the
//  «viscosity» of the IMU relative to the underlying bone.
//      x_k = exp(-Δt/τ) · x_{k−1} + ε,  ε ~ N(0, σ²)
//  These numbers are spec defaults; subject calibration can shrink them.
// ---------------------------------------------------------------------------
struct SkinParams {
    double tauSec;             // 0.15 s — relaxation time constant
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

}  // namespace fox::body
