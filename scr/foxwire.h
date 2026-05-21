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

}  // namespace fox
