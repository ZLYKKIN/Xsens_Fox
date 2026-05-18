"""S7: MXTP12 metadata datagram must be built per-plugin.

Blender plugin (`receiver.py:decode_character_metadata_message`):
    Walks bytes from HEADER_LENGTH (24) looking for `key:value\n` patterns.
    No length prefix — the text starts immediately at offset 24.

UE plugin (`MetaDatagram.cpp:deserializeData`):
    `streamer->read(textLength); streamer->read(text, textLength);`
    int32 BE length prefix, then text bytes.

Fox's `LiveStreamSender::start` (scr/main.cpp:~7748) branches on the
LiveTarget so each plugin gets its expected layout.  This test mirrors
both parsers in Python and verifies our builder is correct for both.
"""
import struct

HEADER_LEN = 24


def build_meta(target, text):
    """Mirror of scr/main.cpp:7748-7761 (header + meta body)."""
    hdr = (
        b"MXTP12"
        + struct.pack(">I", 0)        # sampleCounter
        + bytes([0x80])               # dgCounter
        + bytes([1])                  # dataCount
        + struct.pack(">I", 0)        # frameTimeMs
        + bytes([0])                  # avatarId
        + bytes([23])                 # bodySegmentCount
        + bytes([0])                  # propCount
        + bytes([0])                  # fingerSegmentCount
        + bytes(4)                    # padding
    )
    assert len(hdr) == HEADER_LEN
    payload = b""
    if target == "ue":
        payload += struct.pack(">I", len(text))
    payload += text.encode("utf-8")
    return hdr + payload


def parse_meta_blender(data):
    """Mirror of receiver.py:decode_character_metadata_message."""
    metadata = {}
    i = HEADER_LEN
    while i < len(data):
        colon = data.find(b":", i)
        if colon == -1:
            break
        newline = data.find(b"\n", colon)
        if newline == -1:
            break
        key = data[i:colon].decode("utf-8", errors="ignore").strip()
        val = data[colon + 1: newline].decode("utf-8", errors="ignore").strip()
        if key:
            metadata[key] = val
        i = newline + 1
    return metadata


def parse_meta_ue(data):
    """Mirror of MetaDatagram.cpp:deserializeData."""
    (text_len,) = struct.unpack(">I", data[HEADER_LEN:HEADER_LEN + 4])
    text = data[HEADER_LEN + 4: HEADER_LEN + 4 + text_len].decode("utf-8")
    # The plugin only extracts the value of 'name:' from the blob.
    nameTag = "name:"
    name = ""
    p = text.find(nameTag)
    if p != -1:
        end = text.find("\n", p)
        end = end if end != -1 else len(text)
        name = text[p + len(nameTag):end]
    return name


META_TEXT = "name:FoxMocapLive\ntimeOffset:0\ncolor:255 128 64\n"


def test_blender_meta_roundtrip():
    """Blender plugin parses our packet without an int32 prefix and
    recovers every key:value pair."""
    pkt = build_meta("blender", META_TEXT)
    md = parse_meta_blender(pkt)
    assert md.get("name") == "FoxMocapLive", md
    assert md.get("timeOffset") == "0", md
    assert md.get("color") == "255 128 64", md


def test_ue_meta_roundtrip():
    """UE plugin parses our packet WITH int32 prefix and recovers the name."""
    pkt = build_meta("ue", META_TEXT)
    name = parse_meta_ue(pkt)
    assert name == "FoxMocapLive", name


def test_blender_meta_rejects_ue_format():
    """If we accidentally sent the UE-format packet to Blender, the int32
    prefix bytes would corrupt the first key — verify that the parser
    drops it rather than returning garbage that looks like a valid key.
    This documents WHY the per-target branch matters."""
    pkt = build_meta("ue", META_TEXT)
    md = parse_meta_blender(pkt)
    # The 4-byte int32 length prefix puts non-printable bytes before
    # "name:", so the Blender scanner's first key contains those bytes
    # and is rejected (or mangled) — it must NOT come out as "name".
    assert md.get("name") != "FoxMocapLive", \
        "Blender parser should not silently recover the name from a " \
        "UE-formatted packet — that would hide a real protocol mismatch"


def test_ue_meta_rejects_blender_format():
    """Reverse case: UE-format read of a Blender packet reads garbage
    int32 length (the first 4 bytes of 'name' = 0x6e616d65) and either
    over-reads or returns junk.  Document the failure mode."""
    pkt = build_meta("blender", META_TEXT)
    name = parse_meta_ue(pkt)
    # 0x6e616d65 = 1851878757 bytes — way more than we have, so the
    # substring will be empty or partial; either way, NOT the correct
    # name.
    assert name != "FoxMocapLive", \
        "UE parser should not silently recover the name from a " \
        "Blender-formatted packet"


def test_cpp_branches_on_target():
    """Sanity check the C++ source: there must be an `isBlender` branch
    that suppresses the int32 prefix specifically for Blender."""
    import os
    HERE = os.path.dirname(os.path.abspath(__file__))
    cpp = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))
    with open(cpp, "r", encoding="utf-8") as f:
        src = f.read()
    # Search for the metadata-build site (right before MXTP12 header).
    # Must use !isBlender (or equivalent) to gate the appendInt32BE call.
    assert "isBlender" in src, \
        "no isBlender branch in main.cpp — metadata builder may be sending " \
        "the same payload to both plugins"
    # The literal pattern that lights the branch:
    assert "if (!isBlender)" in src, \
        "expected `if (!isBlender)` in metadata builder"


if __name__ == "__main__":
    test_blender_meta_roundtrip()
    test_ue_meta_roundtrip()
    test_blender_meta_rejects_ue_format()
    test_ue_meta_rejects_blender_format()
    test_cpp_branches_on_target()
    print("test_metadata_per_plugin: PASS")
