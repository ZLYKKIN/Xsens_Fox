// Fox Mocap — MVN MXTP wire encoders (big-endian / network byte order).
//
// Extracted from main.cpp so the exact byte layout the plugins parse can be
// unit-tested against the immutable Plugins/ contract:
//   * header      24 bytes : ">6sIBBIB7x"  ("MXTP"+type, sample, dgCounter,
//                            dataCount, frameTimeMs, avatarId, 7 pad)
//   * pose segment 32 bytes : ">I3f4f"      (segId, posXYZ, quat WXYZ)
// Depends only on QtCore (QByteArray / qToBigEndian).
#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QtGlobal>

namespace fox {

// Build a 24-byte MXTP header.  `msgId2` is the 2-char type ("02", "12", …).
QByteArray buildMxtpHeader(const char* msgId2,
                           quint32 sample,
                           quint8  dgCounter,
                           quint8  dataCount,
                           quint32 frameTimeMs,
                           quint8  segCount,
                           quint8  fingerCount = 0);

// Append a big-endian IEEE-754 float.  Non-finite input (NaN / ±Inf) is coerced
// to 0.0f: the receivers (Blender mathutils, UE FQuat) produce garbage or crash
// on a non-finite wire value, so the serializer guarantees finiteness as a
// last-line invariant.  Quaternions are separately re-normalised to identity
// upstream, so this only ever rewrites a stray position component.
void appendFloatBE(QByteArray& pkt, float v);

// Append a big-endian 32-bit signed integer.
void appendInt32BE(QByteArray& pkt, qint32 v);

// Append one MXTP02 pose segment — the ">I3f4f" 32-byte record the receivers
// parse (Plugins/.../QuaternionDatagram.cpp, receiver.py): segId (1-based, i32
// BE) + position XYZ (3×f32 BE, metres, Z-up RH) + quaternion WXYZ (4×f32 BE,
// scalar first).  Floats go through appendFloatBE, so NaN/Inf is coerced to 0.
void appendPoseSegment(QByteArray& pkt, qint32 segId,
                       float px, float py, float pz,
                       float qw, float qx, float qy, float qz);

// Append one MXTP13 scale segment — "[nameLen i32 BE][name bytes][x y z f32 BE]"
// (Plugins/.../ScaleDatagram.cpp, receiver.py:decode_character_scale_pose_message).
// The receivers key the rest skeleton by `name`, so it must match segment_maps.
void appendScaleSegment(QByteArray& pkt, const char* name,
                        float x, float y, float z);

// Append one MXTP21 ergonomic-angle segment (spec §30):
//   ">I3f"  — jointId (1-based, i32 BE) + (abduction, flexion, rotation)
//             in DEGREES (3×f32 BE).
// Sign convention is the spec §30/§58 per-joint table — abduction (X axis,
// coronal plane), flexion (Y, sagittal), rotation (Z, transverse).  A full
// MXTP21 packet has 22 segments (one per joint in foxbody::kJoints).
void appendErgoAngleSegment(QByteArray& pkt, qint32 jointId,
                            float abductionDeg, float flexionDeg, float rotationDeg);

}  // namespace fox
