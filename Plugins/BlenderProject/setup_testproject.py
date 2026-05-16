"""
setup_testproject.py — Blender headless setup script.

Run this script to populate testproject.blend with:
  • Default MVN armature (23 body bones in correct hierarchy, ready for streaming).
  • Scene properties (IP=localhost, port=9763, gloves enabled).
  • Plugin enabled.

Usage:
  blender --background testproject.blend --python setup_testproject.py
"""
import os
import sys
from pathlib import Path

import bpy
from mathutils import Vector


# --------------------------------------------------------------------------------------------------
# Bone hierarchy: (bone_name, parent_name, head_local_xyz, length, tail_offset_xyz)
# Heights and offsets reflect a 1.75m actor in standard T-pose.
# Coord system: Z up, Y forward, X right (Blender default).
# --------------------------------------------------------------------------------------------------
BONE_LAYOUT = [
    # spine
    ("Pelvis",         None,             (0.00, 0.00, 0.96), 0.10, (0.00, 0.00, 0.10)),
    ("L5",             "Pelvis",         (0.00, 0.00, 1.06), 0.10, (0.00, 0.00, 0.10)),
    ("L3",             "L5",             (0.00, 0.00, 1.16), 0.10, (0.00, 0.00, 0.10)),
    ("T12",            "L3",             (0.00, 0.00, 1.26), 0.10, (0.00, 0.00, 0.10)),
    ("T8",             "T12",            (0.00, 0.00, 1.36), 0.10, (0.00, 0.00, 0.10)),
    ("Neck",           "T8",             (0.00, 0.00, 1.46), 0.05, (0.00, 0.00, 0.05)),
    ("Head",           "Neck",           (0.00, 0.00, 1.51), 0.15, (0.00, 0.00, 0.15)),
    # right arm
    ("RightShoulder",  "T8",             (-0.05, 0.00, 1.46), 0.10, (-0.10, 0.00, 0.00)),
    ("RightUpperArm",  "RightShoulder",  (-0.15, 0.00, 1.46), 0.30, (-0.30, 0.00, 0.00)),
    ("RightForeArm",   "RightUpperArm",  (-0.45, 0.00, 1.46), 0.26, (-0.26, 0.00, 0.00)),
    ("RightHand",      "RightForeArm",   (-0.71, 0.00, 1.46), 0.18, (-0.18, 0.00, 0.00)),
    # left arm
    ("LeftShoulder",   "T8",             ( 0.05, 0.00, 1.46), 0.10, ( 0.10, 0.00, 0.00)),
    ("LeftUpperArm",   "LeftShoulder",   ( 0.15, 0.00, 1.46), 0.30, ( 0.30, 0.00, 0.00)),
    ("LeftForeArm",    "LeftUpperArm",   ( 0.45, 0.00, 1.46), 0.26, ( 0.26, 0.00, 0.00)),
    ("LeftHand",       "LeftForeArm",    ( 0.71, 0.00, 1.46), 0.18, ( 0.18, 0.00, 0.00)),
    # right leg
    ("RightUpperLeg",  "Pelvis",         (-0.10, 0.00, 0.96), 0.43, (0.00, 0.00, -0.43)),
    ("RightLowerLeg",  "RightUpperLeg",  (-0.10, 0.00, 0.53), 0.43, (0.00, 0.00, -0.43)),
    ("RightFoot",      "RightLowerLeg",  (-0.10, 0.00, 0.10), 0.10, (0.00,  0.15, 0.00)),
    ("RightToe",       "RightFoot",      (-0.10, 0.15, 0.10), 0.05, (0.00,  0.05, 0.00)),
    # left leg
    ("LeftUpperLeg",   "Pelvis",         ( 0.10, 0.00, 0.96), 0.43, (0.00, 0.00, -0.43)),
    ("LeftLowerLeg",   "LeftUpperLeg",   ( 0.10, 0.00, 0.53), 0.43, (0.00, 0.00, -0.43)),
    ("LeftFoot",       "LeftLowerLeg",   ( 0.10, 0.00, 0.10), 0.10, (0.00,  0.15, 0.00)),
    ("LeftToe",        "LeftFoot",       ( 0.10, 0.15, 0.10), 0.05, (0.00,  0.05, 0.00)),
]


# --------------------------------------------------------------------------------------------------
# Finger bone layout — 20 per hand, 40 total. T-pose with arms extended laterally.
# LeftHand ends at (0.89, 0, 1.46); fingers point +X (continuing along arm).
# RightHand ends at (-0.89, 0, 1.46); fingers point -X (mirror).
# Thumb branches off perpendicular toward body interior.
# Per-finger lateral offset (Y) reflects index/middle/ring/pinky spread.
# Format: (bone_name, parent_name, head_xyz, length, tail_offset_xyz)
# --------------------------------------------------------------------------------------------------
FINGER_LAYOUT = [
    ("LeftCarpus",     "LeftHand",       ( 0.89,  0.000, 1.460), 0.020, ( 0.020,  0.000,  0.000)),
    ("LeftFirstMC",    "LeftCarpus",     ( 0.91,  0.020, 1.455), 0.040, ( 0.030,  0.020, -0.005)),
    ("LeftFirstPP",    "LeftFirstMC",    ( 0.94,  0.040, 1.450), 0.030, ( 0.025,  0.015, -0.005)),
    ("LeftFirstDP",    "LeftFirstPP",    ( 0.965, 0.055, 1.445), 0.025, ( 0.020,  0.012, -0.003)),
    ("LeftSecondMC",   "LeftCarpus",     ( 0.91,  0.025, 1.460), 0.060, ( 0.060,  0.000,  0.000)),
    ("LeftSecondPP",   "LeftSecondMC",   ( 0.97,  0.025, 1.460), 0.040, ( 0.040,  0.000,  0.000)),
    ("LeftSecondMP",   "LeftSecondPP",   ( 1.01,  0.025, 1.460), 0.025, ( 0.025,  0.000,  0.000)),
    ("LeftSecondDP",   "LeftSecondMP",   ( 1.035, 0.025, 1.460), 0.020, ( 0.020,  0.000,  0.000)),
    ("LeftThirdMC",    "LeftCarpus",     ( 0.91,  0.008, 1.460), 0.065, ( 0.065,  0.000,  0.000)),
    ("LeftThirdPP",    "LeftThirdMC",    ( 0.975, 0.008, 1.460), 0.045, ( 0.045,  0.000,  0.000)),
    ("LeftThirdMP",    "LeftThirdPP",    ( 1.020, 0.008, 1.460), 0.028, ( 0.028,  0.000,  0.000)),
    ("LeftThirdDP",    "LeftThirdMP",    ( 1.048, 0.008, 1.460), 0.022, ( 0.022,  0.000,  0.000)),
    ("LeftFourthMC",   "LeftCarpus",     ( 0.91, -0.010, 1.460), 0.060, ( 0.060,  0.000,  0.000)),
    ("LeftFourthPP",   "LeftFourthMC",   ( 0.97, -0.010, 1.460), 0.040, ( 0.040,  0.000,  0.000)),
    ("LeftFourthMP",   "LeftFourthPP",   ( 1.01, -0.010, 1.460), 0.026, ( 0.026,  0.000,  0.000)),
    ("LeftFourthDP",   "LeftFourthMP",   ( 1.036,-0.010, 1.460), 0.020, ( 0.020,  0.000,  0.000)),
    ("LeftFifthMC",    "LeftCarpus",     ( 0.91, -0.025, 1.460), 0.055, ( 0.055,  0.000,  0.000)),
    ("LeftFifthPP",    "LeftFifthMC",    ( 0.965,-0.025, 1.460), 0.030, ( 0.030,  0.000,  0.000)),
    ("LeftFifthMP",    "LeftFifthPP",    ( 0.995,-0.025, 1.460), 0.020, ( 0.020,  0.000,  0.000)),
    ("LeftFifthDP",    "LeftFifthMP",    ( 1.015,-0.025, 1.460), 0.017, ( 0.017,  0.000,  0.000)),
    ("RightCarpus",    "RightHand",      (-0.89,  0.000, 1.460), 0.020, (-0.020,  0.000,  0.000)),
    ("RightFirstMC",   "RightCarpus",    (-0.91,  0.020, 1.455), 0.040, (-0.030,  0.020, -0.005)),
    ("RightFirstPP",   "RightFirstMC",   (-0.94,  0.040, 1.450), 0.030, (-0.025,  0.015, -0.005)),
    ("RightFirstDP",   "RightFirstPP",   (-0.965, 0.055, 1.445), 0.025, (-0.020,  0.012, -0.003)),
    ("RightSecondMC",  "RightCarpus",    (-0.91,  0.025, 1.460), 0.060, (-0.060,  0.000,  0.000)),
    ("RightSecondPP",  "RightSecondMC",  (-0.97,  0.025, 1.460), 0.040, (-0.040,  0.000,  0.000)),
    ("RightSecondMP",  "RightSecondPP",  (-1.01,  0.025, 1.460), 0.025, (-0.025,  0.000,  0.000)),
    ("RightSecondDP",  "RightSecondMP",  (-1.035, 0.025, 1.460), 0.020, (-0.020,  0.000,  0.000)),
    ("RightThirdMC",   "RightCarpus",    (-0.91,  0.008, 1.460), 0.065, (-0.065,  0.000,  0.000)),
    ("RightThirdPP",   "RightThirdMC",   (-0.975, 0.008, 1.460), 0.045, (-0.045,  0.000,  0.000)),
    ("RightThirdMP",   "RightThirdPP",   (-1.020, 0.008, 1.460), 0.028, (-0.028,  0.000,  0.000)),
    ("RightThirdDP",   "RightThirdMP",   (-1.048, 0.008, 1.460), 0.022, (-0.022,  0.000,  0.000)),
    ("RightFourthMC",  "RightCarpus",    (-0.91, -0.010, 1.460), 0.060, (-0.060,  0.000,  0.000)),
    ("RightFourthPP",  "RightFourthMC",  (-0.97, -0.010, 1.460), 0.040, (-0.040,  0.000,  0.000)),
    ("RightFourthMP",  "RightFourthPP",  (-1.01, -0.010, 1.460), 0.026, (-0.026,  0.000,  0.000)),
    ("RightFourthDP",  "RightFourthMP",  (-1.036,-0.010, 1.460), 0.020, (-0.020,  0.000,  0.000)),
    ("RightFifthMC",   "RightCarpus",    (-0.91, -0.025, 1.460), 0.055, (-0.055,  0.000,  0.000)),
    ("RightFifthPP",   "RightFifthMC",   (-0.965,-0.025, 1.460), 0.030, (-0.030,  0.000,  0.000)),
    ("RightFifthMP",   "RightFifthPP",   (-0.995,-0.025, 1.460), 0.020, (-0.020,  0.000,  0.000)),
    ("RightFifthDP",   "RightFifthMP",   (-1.015,-0.025, 1.460), 0.017, (-0.017,  0.000,  0.000)),
]


def clear_scene():
    """Remove all objects so we start clean."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False, confirm=False)
    for col in bpy.data.collections:
        bpy.data.collections.remove(col)
    for arm in bpy.data.armatures:
        bpy.data.armatures.remove(arm)


def create_mvn_armature(name: str = "MVN:Actor", include_fingers: bool = True):
    """Create armature with 23 body bones (+ 40 finger bones) matching MVN segment hierarchy."""
    bpy.ops.object.armature_add(enter_editmode=True, location=(0, 0, 0))
    armature_obj = bpy.context.active_object
    armature_obj.name = name
    armature_obj.data.name = name

    eb = armature_obj.data.edit_bones
    for bone in list(eb):
        eb.remove(bone)

    layout = BONE_LAYOUT + (FINGER_LAYOUT if include_fingers else [])

    created = {}
    for bone_name, parent_name, head_xyz, length, tail_off in layout:
        b = eb.new(bone_name)
        b.head = Vector(head_xyz)
        b.tail = Vector(head_xyz) + Vector(tail_off)
        if parent_name and parent_name in created:
            b.parent = created[parent_name]
            b.use_connect = False
        created[bone_name] = b

    bpy.ops.object.mode_set(mode="OBJECT")

    for bone in armature_obj.pose.bones:
        bone.rotation_mode = "QUATERNION"

    return armature_obj


def configure_scene():
    """Configure scene properties so the streamer points at our backend on localhost:9763."""
    scene = bpy.context.scene
    try:
        scene.mvn_stream_address = "localhost"
        scene.mvn_stream_port = "9763"
        scene.mvn_gloves_enabled = True
        scene.mvn_scene_units = "CM"
        scene.mvn_update_frequency = 240
    except AttributeError:
        # Properties only exist if the MVN plugin is enabled. Save anyway.
        print("[setup] Warning: MVN plugin properties not registered; "
              "ensure MVNBlenderPlugin-main is enabled in Add-ons.")


def enable_plugin():
    """Enable the MVN Live Plugin so its scene properties are registered."""
    try:
        bpy.ops.preferences.addon_enable(module="MVNBlenderPlugin-main")
        print("[setup] MVN plugin enabled.")
    except Exception as e:
        print(f"[setup] addon_enable failed: {e}")


def save_blend():
    """Save the current file."""
    out = bpy.data.filepath
    bpy.ops.wm.save_as_mainfile(filepath=out)
    print(f"[setup] Saved {out}")


# --------------------------------------------------------------------------------------------------
def main():
    enable_plugin()
    clear_scene()

    arm = create_mvn_armature("MVN:Actor")
    print(f"[setup] Created armature '{arm.name}' with {len(arm.data.bones)} bones.")

    configure_scene()
    save_blend()
    print("[setup] Done. testproject.blend is ready for streaming + alllog.txt.")


if __name__ == "__main__":
    main()
