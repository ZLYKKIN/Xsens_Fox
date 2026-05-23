// Fox Mocap — MVN MXTP wire encoders.  Moved from main.cpp; the only behaviour
// change is the explicit NaN/Inf guard in appendFloatBE (documented in the
// header).
#include "foxwire.h"

#include <QtEndian>

#include <cmath>
#include <cstring>

namespace fox {

QByteArray buildMxtpHeader(const char* msgId2,
                           quint32 sample,
                           quint8  dgCounter,
                           quint8  dataCount,
                           quint32 frameTimeMs,
                           quint8  segCount,
                           quint8  fingerCount)
{
    QByteArray pkt;
    pkt.reserve(24);
    pkt.append("MXTP");
    pkt.append(msgId2, 2);
    const quint32 sampleBE = qToBigEndian<quint32>(sample);
    pkt.append(reinterpret_cast<const char*>(&sampleBE), 4);
    pkt.append(char(dgCounter));
    pkt.append(char(dataCount));
    const quint32 ftBE = qToBigEndian<quint32>(frameTimeMs);
    pkt.append(reinterpret_cast<const char*>(&ftBE), 4);
    pkt.append(char(0));               // avatarId
    pkt.append(char(segCount));        // bodySegmentCount
    pkt.append(char(0));               // props
    pkt.append(char(fingerCount));     // fingerSegmentCount
    pkt.append(4, '\0');               // padding
    return pkt;
}

void appendFloatBE(QByteArray& pkt, float v)
{
    if (!std::isfinite(v)) v = 0.0f;   // never emit NaN/Inf on the wire
    quint32 bits;
    std::memcpy(&bits, &v, 4);
    bits = qToBigEndian<quint32>(bits);
    pkt.append(reinterpret_cast<const char*>(&bits), 4);
}

void appendInt32BE(QByteArray& pkt, qint32 v)
{
    const quint32 be = qToBigEndian<quint32>(static_cast<quint32>(v));
    pkt.append(reinterpret_cast<const char*>(&be), 4);
}

void appendPoseSegment(QByteArray& pkt, qint32 segId,
                       float px, float py, float pz,
                       float qw, float qx, float qy, float qz)
{
    appendInt32BE(pkt, segId);
    appendFloatBE(pkt, px);
    appendFloatBE(pkt, py);
    appendFloatBE(pkt, pz);
    appendFloatBE(pkt, qw);
    appendFloatBE(pkt, qx);
    appendFloatBE(pkt, qy);
    appendFloatBE(pkt, qz);
}

void appendScaleSegment(QByteArray& pkt, const char* name,
                        float x, float y, float z)
{
    const qint32 len = qint32(std::strlen(name));
    appendInt32BE(pkt, len);
    pkt.append(name, len);
    appendFloatBE(pkt, x);
    appendFloatBE(pkt, y);
    appendFloatBE(pkt, z);
}

void appendErgoAngleSegment(QByteArray& pkt, qint32 jointId,
                            float abductionDeg, float flexionDeg, float rotationDeg)
{
    // 16-byte record matching the ">I3f" template in the receivers' parsers
    // (see ErgoDatagram.cpp in the Plugins/ tree once the consumer ships).
    // Big-endian throughout; the float helper coerces NaN/Inf to 0 so the
    // wire never carries a non-finite value.
    appendInt32BE(pkt, jointId);
    appendFloatBE(pkt, abductionDeg);
    appendFloatBE(pkt, flexionDeg);
    appendFloatBE(pkt, rotationDeg);
}

}  // namespace fox
