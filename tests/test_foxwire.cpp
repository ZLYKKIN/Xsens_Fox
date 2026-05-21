// Unit tests for scr/foxwire.{h,cpp} — the MVN MXTP byte encoders.
// Byte layout is checked against the immutable Plugins/ contract:
//   header  (24 B): ">6sIBBIB7x"  — Plugins/MVNBlenderPlugin-main/receiver.py
//   segment (32 B): ">I3f4f"      — Plugins/XsensLivc/.../QuaternionDatagram.cpp
#include "foxwire.h"
#include "foxtest.h"

#include <cstdint>
#include <cstring>
#include <limits>

namespace {

uint32_t rdU32(const QByteArray& b, int o)
{
    return (uint32_t(uint8_t(b[o]))   << 24) | (uint32_t(uint8_t(b[o+1])) << 16)
         | (uint32_t(uint8_t(b[o+2])) <<  8) |  uint32_t(uint8_t(b[o+3]));
}
int32_t rdI32(const QByteArray& b, int o) { return int32_t(rdU32(b, o)); }
float   rdF32(const QByteArray& b, int o)
{
    uint32_t u = rdU32(b, o); float f; std::memcpy(&f, &u, 4); return f;
}

void test_header_layout()
{
    // MXTP02 body-only frame, as emitted by LiveStreamSender::pushFrame.
    const QByteArray h = fox::buildMxtpHeader("02", 0x01020304u, 0x80, 23,
                                              0x0A0B0C0Du, 23, 0);
    CHECK(h.size() == 24);
    CHECK(h[0] == 'M' && h[1] == 'X' && h[2] == 'T' && h[3] == 'P');
    CHECK(h[4] == '0' && h[5] == '2');
    CHECK(rdU32(h, 6) == 0x01020304u);          // sample counter, big-endian
    CHECK(uint8_t(h[10]) == 0x80);              // datagram counter (single)
    CHECK(uint8_t(h[11]) == 23);                // dataCount (items)
    CHECK(rdU32(h, 12) == 0x0A0B0C0Du);         // frame time ms, big-endian
    CHECK(uint8_t(h[16]) == 0);                 // avatarId
    CHECK(uint8_t(h[17]) == 23);                // bodySegmentCount
    CHECK(uint8_t(h[18]) == 0);                 // props
    CHECK(uint8_t(h[19]) == 0);                 // fingerSegmentCount
    for (int i = 20; i < 24; ++i) CHECK(h[i] == 0);   // padding (Python "7x")
}

void test_header_gloves()
{
    // body+gloves combined frame: 63 items total, 40 finger segments.
    const QByteArray h = fox::buildMxtpHeader("02", 7, 0x80, 63, 100, 23, 40);
    CHECK(h.size() == 24);
    CHECK(uint8_t(h[11]) == 63);                // dataCount = 23 body + 40 finger
    CHECK(uint8_t(h[17]) == 23);                // body segment count
    CHECK(uint8_t(h[19]) == 40);                // finger segment count (gloves)
}

void test_int32_be()
{
    QByteArray b;
    fox::appendInt32BE(b, 1);
    fox::appendInt32BE(b, -1);
    fox::appendInt32BE(b, 23);
    CHECK(b.size() == 12);
    CHECK(uint8_t(b[0]) == 0 && uint8_t(b[1]) == 0 && uint8_t(b[2]) == 0 && uint8_t(b[3]) == 1);
    CHECK(rdI32(b, 4) == -1);
    CHECK(rdI32(b, 8) == 23);
}

void test_float_be_and_nan_guard()
{
    QByteArray b;
    fox::appendFloatBE(b, 1.0f);
    CHECK(b.size() == 4);
    // IEEE-754 1.0f == 0x3F800000, network (big-endian) byte order.
    CHECK(uint8_t(b[0]) == 0x3F && uint8_t(b[1]) == 0x80 &&
          uint8_t(b[2]) == 0x00 && uint8_t(b[3]) == 0x00);

    QByteArray b2;
    fox::appendFloatBE(b2, -2.5f);
    CHECK_NEAR(rdF32(b2, 0), -2.5f, 0.0);       // exact round-trip

    // *** The hardening under test: non-finite must serialise as 0.0f. ***
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    QByteArray b3;
    fox::appendFloatBE(b3, nan);
    fox::appendFloatBE(b3, inf);
    fox::appendFloatBE(b3, -inf);
    CHECK(rdU32(b3, 0) == 0u);
    CHECK(rdU32(b3, 4) == 0u);
    CHECK(rdU32(b3, 8) == 0u);
}

void test_pose_segment_roundtrip()
{
    // One MXTP02 segment exactly as pushFrame writes it: ">I3f4f" = 32 bytes.
    QByteArray seg;
    fox::appendInt32BE(seg, 1);          // segId (1-based)
    fox::appendFloatBE(seg, 0.10f);      // pos x  (metres, NWU)
    fox::appendFloatBE(seg, -0.20f);     // pos y
    fox::appendFloatBE(seg, 1.50f);      // pos z
    fox::appendFloatBE(seg, 0.70710677f);// quat w  (scalar first)
    fox::appendFloatBE(seg, 0.0f);       // quat x
    fox::appendFloatBE(seg, 0.0f);       // quat y
    fox::appendFloatBE(seg, 0.70710677f);// quat z

    CHECK(seg.size() == 32);
    CHECK(rdI32(seg, 0) == 1);
    CHECK_NEAR(rdF32(seg, 4),  0.10f, 1e-6);
    CHECK_NEAR(rdF32(seg, 8), -0.20f, 1e-6);
    CHECK_NEAR(rdF32(seg, 12), 1.50f, 1e-6);
    CHECK_NEAR(rdF32(seg, 16), 0.70710677f, 1e-6);   // w
    CHECK_NEAR(rdF32(seg, 20), 0.0f, 1e-6);          // x
    CHECK_NEAR(rdF32(seg, 24), 0.0f, 1e-6);          // y
    CHECK_NEAR(rdF32(seg, 28), 0.70710677f, 1e-6);   // z
}

}  // namespace

int main()
{
    RUN(test_header_layout);
    RUN(test_header_gloves);
    RUN(test_int32_be);
    RUN(test_float_be_and_nan_guard);
    RUN(test_pose_segment_roundtrip);
    return fox_report("foxwire");
}
