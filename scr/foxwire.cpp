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

}  // namespace fox
