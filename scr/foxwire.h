
#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QtGlobal>

namespace fox {

QByteArray buildMxtpHeader(const char* msgId2,
                           quint32 sample,
                           quint8  dgCounter,
                           quint8  dataCount,
                           quint32 frameTimeMs,
                           quint8  segCount,
                           quint8  fingerCount = 0);

void appendFloatBE(QByteArray& pkt, float v);

void appendInt32BE(QByteArray& pkt, qint32 v);

void appendPoseSegment(QByteArray& pkt, qint32 segId,
                       float px, float py, float pz,
                       float qw, float qx, float qy, float qz);

void appendScaleSegment(QByteArray& pkt, const char* name,
                        float x, float y, float z);

void appendErgoAngleSegment(QByteArray& pkt, qint32 jointId,
                            float abductionDeg, float flexionDeg, float rotationDeg);

}
