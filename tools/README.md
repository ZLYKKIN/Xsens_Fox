# tools/ — diagnostics & verification utilities

## `diff_streams.py`

Cross-log diff between Fox Mocap (sender) and the Blender plugin (receiver).
Finds where the chain breaks when the rendered skeleton in Blender does
not match what Fox intended.

### How the chain looks

```
   Fox (sender)        Blender plugin (receiver)
   ────────────        ──────────────────────────────────────
   [send]    UDP  ───► [raw]                  datagram arrived
                       [recv]                 wire-exact quat+pos
                       [axis_pos]             Vector(y,z,x) remap
                       [converted]            quat+pos bundle
                       [rule]                 per-segment quat rule
                       [bone_world]/          actual bone state
                       [bone_local]           written into Blender
```

Each stage is logged into `alllog.txt` next to the `.blend` file when the
"MVN: write alllog.txt" checkbox in the Blender side panel is ticked.
Fox emits `[send]` lines to stdout whenever it's launched with `-test`.

### How to capture the two logs

**Fox side:**
```
fox_mocap.exe -test  >  fox_send.log   2>&1
```
The `-test` flag flips `g_testStreamLog = true`, which makes
`LiveStreamSender::pushFrame{,WithGloves}` dump one `[send]` line per
segment per frame.

**Blender side:**
1. Open the .blend with the MVN plugin enabled.
2. In the side panel, tick the **MVN: write alllog.txt** checkbox.
3. Press **Start receiving**.  `alllog.txt` is written next to the .blend.
4. Press **Stop** when you have enough data — the file is flushed.

### Usage

```bash
# Full chain diff for every matched frame
python3 tools/diff_streams.py --fox fox_send.log --blender alllog.txt

# Just the rest pose (use when the skeleton looks wrong AT START, before any
# [recv] arrives — this captures the armature's edit-bone state at build)
python3 tools/diff_streams.py --blender alllog.txt --rest-pose

# Only T-pose frames (all [send] quats near identity — the cleanest signal,
# because there's no IMU noise and the chain should preserve identity end-to-end)
python3 tools/diff_streams.py --fox fox_send.log --blender alllog.txt --t-pose-only

# Filter to one segment, e.g. RightHand
python3 tools/diff_streams.py --fox fox_send.log --blender alllog.txt --segment RightHand

# Per-frame one-line summary (fast at-a-glance diagnostic)
python3 tools/diff_streams.py --fox fox_send.log --blender alllog.txt --summary

# Just the first frame whose chain breaks somewhere
python3 tools/diff_streams.py --fox fox_send.log --blender alllog.txt --first-divergence

# Tighter or looser numeric tolerance (default 1e-4)
python3 tools/diff_streams.py --fox fox_send.log --blender alllog.txt --tolerance 1e-6
```

### What divergences mean

| Stage flagged          | Meaning                                                                         | Where to look                          |
|------------------------|---------------------------------------------------------------------------------|----------------------------------------|
| `[recv] WIRE`          | Wire byte mismatch — UDP frame got corrupted or dropped between Fox and Blender | Network / socket / handshake           |
| `[axis_pos] CONVERT`   | Plugin's `Vector(y, z, x)` rule didn't fire correctly                           | `pose.py::_convert_vectors`            |
| `[converted] BUNDLE`   | Quaternion mutated at convert stage (it shouldn't — it's taken verbatim)        | `pose.py::_convert_to_quaternions`     |
| `[rule] REMAP`         | The named rule produced an unexpected output                                    | `source_animator.py::calculate_rotation` |
| `[bone_world]`/`[bone_local]` | What was set on the Blender bone differs from what the rule produced     | bone parent-inverse / delta_quaternion |

### Interpreting `--rest-pose`

The rest pose table shows what Blender's armature looks like in its
default (post-build) state.  When you see "wrong pose at start" before
streaming begins, this is the table that explains it.

For a healthy T-pose, look for:
- **Pelvis head**: around `(0, 0, h * 0.55)` where `h` is the actor's height in metres.
- **RightHand head**: around `(-0.6, 0, ~1.4)` for a 1.75 m actor (≈half-armspan).
- **LeftHand head**: around `(+0.6, 0, ~1.4)`.

If the head positions are scrambled or units look wrong, the bug is in
how the MXTP13 scale message gets turned into edit-bone positions —
either the axis conversion (`pose.py::_apply_axis_conversion_to_transforms`)
or the actor-height scaling.

### Self-test

The diff tool itself is covered by `tests/python/test_diff_streams.py`,
which builds synthetic clean and broken logs and asserts the right
stages get flagged.  It runs as part of `python3 tests/python/run_all.py`.
