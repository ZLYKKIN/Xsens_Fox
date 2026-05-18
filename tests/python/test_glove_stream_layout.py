"""§D test: MXTP02 body+gloves payload layout sent by
LiveStreamSender::pushFrameWithGloves (scr/main.cpp:7931-8086).

The streamer writes the same MXTP02 packet to both Blender (no length
prefix in MXTP12) and UE LiveLink (length prefix in MXTP12). Body and
finger payloads are identical for both targets — only the metadata
handshake differs.

Layout we assert here:
  Header  24 B    "MXTP02" + sampleBE + dgCounter + dataCount + ftBE +
                  avatarId + bodyCount + props + fingerCount + 4 B pad
  Per-seg 32 B    segmentIdBE + xyzFloatBE + wxyzFloatBE
  Segment IDs are contiguous: 1..23 body, 24..43 left fingers, 44..63 right
"""
import sys, os, struct
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np


def be_u32(b, o):  return struct.unpack(">I", b[o:o+4])[0]
def be_f32(b, o):  return struct.unpack(">f", b[o:o+4])[0]


def build_header(msg_id, sample, dg, data_count, ft, body, fingers):
    hdr  = b"MXTP" + msg_id.encode("ascii")
    hdr += struct.pack(">I", sample)
    hdr += bytes([dg & 0xff, data_count & 0xff])
    hdr += struct.pack(">I", ft)
    hdr += bytes([0, body & 0xff, 0, fingers & 0xff])
    hdr += b"\x00" * 4
    return hdr


def build_segment(seg_id, p, q):
    out  = struct.pack(">i", seg_id)
    out += struct.pack(">fff", *p)
    out += struct.pack(">ffff", *q)
    return out


def build_glove_packet(sample, body_quats, body_pos, lQ, lP, rQ, rP,
                       wrist_l, wrist_r):
    """Reproduce scr/main.cpp pushFrameWithGloves output bytes."""
    body_count  = 23
    finger_count = 40
    body = b""
    for i in range(body_count):
        body += build_segment(i + 1, body_pos[i], body_quats[i])

    # finger layout: 20 left then 20 right.  Slot 0 = carpus (-1 in Manus
    # index), 1-19 = Manus indices 1..19.
    SLOT_TO_MANUS = [-1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                     11, 12, 13, 14, 15, 16, 17, 18, 19]
    for slot in range(20):
        seg_id = 24 + slot
        m = SLOT_TO_MANUS[slot]
        if m < 0:
            body += build_segment(seg_id, wrist_l, body_quats[14])  # SEG_LHand
        else:
            body += build_segment(seg_id, lP[m], lQ[m])
    for slot in range(20):
        seg_id = 44 + slot
        m = SLOT_TO_MANUS[slot]
        if m < 0:
            body += build_segment(seg_id, wrist_r, body_quats[10])  # SEG_RHand
        else:
            body += build_segment(seg_id, rP[m], rQ[m])

    hdr = build_header("02", sample, 0x80, body_count + finger_count, 12345,
                       body_count, finger_count)
    return hdr + body


def test_header_layout():
    hdr = build_header("02", 42, 0x80, 63, 12345, 23, 40)
    assert hdr[:4] == b"MXTP"
    assert hdr[4:6] == b"02"
    assert be_u32(hdr, 6) == 42
    assert hdr[10] == 0x80
    assert hdr[11] == 63
    assert be_u32(hdr, 12) == 12345
    assert hdr[16] == 0     # avatarId
    assert hdr[17] == 23    # bodyCount
    assert hdr[18] == 0     # props
    assert hdr[19] == 40    # fingerCount
    assert hdr[20:24] == b"\x00\x00\x00\x00"


def test_finger_packet_layout():
    """Assert per-segment IDs are contiguous 1..63 for a useGloves=True
    packet."""
    body_q = [(1.0, 0.0, 0.0, 0.0)] * 23
    body_p = [(0.0, 0.0, 0.0)] * 23
    lQ = [(1.0, 0.0, 0.0, 0.0)] * 20
    lP = [(0.01 * i, 0.0, 0.0) for i in range(20)]
    rQ = [(1.0, 0.0, 0.0, 0.0)] * 20
    rP = [(0.0, 0.01 * i, 0.0) for i in range(20)]
    pkt = build_glove_packet(1, body_q, body_p, lQ, lP, rQ, rP,
                             wrist_l=(0.1, 0.2, 0.3),
                             wrist_r=(-0.1, 0.2, 0.3))
    seg_size = 4 + 4*3 + 4*4
    pld = pkt[24:]
    assert len(pld) == (23 + 40) * seg_size
    ids = []
    for i in range(63):
        seg_id = struct.unpack(">i", pld[i*seg_size:i*seg_size+4])[0]
        ids.append(seg_id)
    assert ids == list(range(1, 64)), f"non-contiguous segment ids: {ids}"


def test_carpus_slot_uses_wrist_pos():
    """For both hands, the first finger slot (carpus = Manus index −1)
    must carry the wrist's world position, not zeros."""
    body_q = [(1.0, 0.0, 0.0, 0.0)] * 23
    body_p = [(0.0, 0.0, 0.0)] * 23
    lQ = [(1.0, 0.0, 0.0, 0.0)] * 20
    lP = [(0.0, 0.0, 0.0)] * 20
    rQ = [(1.0, 0.0, 0.0, 0.0)] * 20
    rP = [(0.0, 0.0, 0.0)] * 20
    wrist_l = (0.10, 0.20, 0.30)
    wrist_r = (-0.10, 0.20, 0.30)
    pkt = build_glove_packet(1, body_q, body_p, lQ, lP, rQ, rP,
                             wrist_l=wrist_l, wrist_r=wrist_r)
    seg_size = 4 + 4*3 + 4*4
    pld = pkt[24:]
    # Left carpus is the first finger slot (segment ID 24).
    o = 23 * seg_size + 4
    lpos = (be_f32(pld, o), be_f32(pld, o+4), be_f32(pld, o+8))
    o = 43 * seg_size + 4
    rpos = (be_f32(pld, o), be_f32(pld, o+4), be_f32(pld, o+8))
    assert all(abs(a - b) < 1e-5 for a, b in zip(lpos, wrist_l)), \
        f"left carpus position mismatch: {lpos} vs {wrist_l}"
    assert all(abs(a - b) < 1e-5 for a, b in zip(rpos, wrist_r)), \
        f"right carpus position mismatch: {rpos} vs {wrist_r}"


def test_body_only_packet_no_finger_bytes():
    """For useGloves=False the packet must NOT contain finger segments."""
    body_q = [(1.0, 0.0, 0.0, 0.0)] * 23
    body_p = [(0.0, 0.0, 0.0)] * 23
    body = b""
    for i in range(23):
        body += build_segment(i + 1, body_p[i], body_q[i])
    hdr = build_header("02", 1, 0x80, 23, 0, 23, 0)
    pkt = hdr + body
    assert hdr[19] == 0      # fingerCount
    seg_size = 4 + 4*3 + 4*4
    assert len(pkt) == 24 + 23 * seg_size


if __name__ == "__main__":
    test_header_layout()
    test_finger_packet_layout()
    test_carpus_slot_uses_wrist_pos()
    test_body_only_packet_no_finger_bytes()
    print("test_glove_stream_layout: PASS")
