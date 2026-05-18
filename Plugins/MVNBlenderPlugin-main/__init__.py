# **********************************************************************************************************************
# MVN Animate Integration for Blender:
# This Blender add-on facilitates real-time motion capture data streaming and recording from MVN into Blender.
# It provides tools for live-streaming, recording, retargeting, and configuring MVN motion capture data
# directly within Blender's UI.
#
# Features:
# - Real-time data streaming from MVN to Blender.
# - Recording motion capture data for future playback or analysis.
# - Retargeting MVN data to Blender armatures.
# - Configurable settings for stream address, port, and data precision.
# - UI panels for easy access and control within Blender.
#
# Usage:
# 1. Install the add-on in Blender via Preferences > Add-ons > Install.
# 2. Configure the streaming settings in the MVN Live Streaming panel.
# 3. Start the live stream and recording as needed.
# 4. Use the retargeting tools to apply motion data to your Blender armatures.
#
# Requirements:
# - Blender 3.6.0 or newer.
# **********************************************************************************************************************
from pathlib import Path

import bpy
from bpy.types import Object
from bpy.props import StringProperty
from bpy.app.handlers import persistent
from bpy.utils import previews, register_class, unregister_class

from .icons import ICON_COLLECTION
from .logger import LOGGING_OT_logger
from . import file_logger
from .rigging import target_euler, target_vector, apply_tpose, update_bonemap
from .ui import (
    WM_OT_StartStream, WM_OT_ToggleRecord, WM_OT_AutoMap, WM_OT_ClearMap,
    WM_OT_LoadMap, WM_OT_SaveMap, MVN_UL_BoneList, ReceiverPanel, RecorderPanel, RetargetingPanel,
    save_config_file, update_source_armature, update_target_armature, update_xsens, update_scene_scale,
    poll_target_armatures, poll_source_armatures, resolve_ik_armature, update_ik_feet, IconManager
    )


# ============================================================================================
# Add-on Information
bl_info = {
    "name": "MVN Live Plugin",
    "description": "MVN Animate Blender Integration",
    "author": "SuperAlloy Interactive",
    "version": (1, 0, 0),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar > Xsens",
    # "warning": "warning info",
    # "tracker_url": "tracker url info",
    "category": "Motion Capture",
}


# ============================================================================================
# Registration
classes = (WM_OT_StartStream, WM_OT_ToggleRecord, WM_OT_AutoMap, WM_OT_ClearMap, WM_OT_LoadMap, WM_OT_SaveMap, MVN_UL_BoneList, ReceiverPanel, RecorderPanel, RetargetingPanel, LOGGING_OT_logger)
driver_functions = (target_euler, target_vector)


@persistent
def load_handler(dummy):
    for drv in driver_functions:
        bpy.app.driver_namespace[drv.__name__] = drv

    # --- Auto-start receiver for dual-logging test runs ---------------------
    # If the freshly-loaded scene asks for it, fire WM_OT_StartStream from a
    # short timer so the property registration above is fully wired before
    # we touch it. A small delay also lets Blender finish rendering the new
    # scene before we start streaming pose updates into it.
    try:
        scene = bpy.context.scene
    except Exception:
        scene = None
    if scene is not None and getattr(scene, "mvn_auto_start_on_load", False):
        def _kick():
            try:
                # WM_OT_StartStream lives in ui.py and is the same operator
                # the green "Start Stream" button on the Receiver panel calls.
                bpy.ops.wm.startstream()
                print("[mvn] auto-start triggered after scene load.")
            except Exception as exc:
                print(f"[mvn] auto-start failed: {exc}")
            return None  # one-shot timer
        try:
            bpy.app.timers.register(_kick, first_interval=0.5)
        except Exception as exc:
            print(f"[mvn] could not register auto-start timer: {exc}")


def register():
    """
    Registers classes, properties, and driver functions for the MVN Live Plugin. Also initializes the plugin's icon
    collection from the "icons" directory.
    """
    for cls in classes:
        register_class(cls)

    bpy.types.Scene.mvn_stream_address = bpy.props.StringProperty(name="", default="localhost", update=save_config_file)
    bpy.types.Scene.mvn_stream_port = bpy.props.StringProperty(name="", default="9763", update=save_config_file)
    bpy.types.Scene.mvn_update_frequency = bpy.props.IntProperty(name="", default=240, update=save_config_file)  # Updates per second
    bpy.types.Scene.mvn_float_precision = bpy.props.IntProperty(name="", default=2, update=save_config_file)  # Number of decimal places to round to # Note this property is currently not used
    bpy.types.Scene.mvn_scene_units = bpy.props.StringProperty(name="", default="CM", update=save_config_file)
    bpy.types.Scene.mvn_gloves_enabled = bpy.props.BoolProperty(name="", default=False, update=save_config_file)
    # --- Fox Mocap test-mode switches: when a scene is opened with
    #     mvn_auto_start_on_load == True (e.g. testproject.blend) the load
    #     handler kicks the UDP receiver automatically so dual-logging
    #     works without any operator clicks.
    bpy.types.Scene.mvn_auto_start_on_load = bpy.props.BoolProperty(
        name="Auto-start stream on scene load",
        description="Start the MVN receiver as soon as this .blend opens — "
                    "useful for dual-logging tests with Fox Mocap.",
        default=False)
    bpy.types.Scene.mvn_test_logging = bpy.props.BoolProperty(
        name="Write alllog.txt next to .blend",
        description="Open a file-logger that mirrors the format of Fox Mocap "
                    "-test stdout so the two logs can be joined frame-by-frame.",
        default=True)
    bpy.types.Scene.mvn_log_only_mode = bpy.props.BoolProperty(
        name="Log-only mode (no armature update)",
        description="Receive UDP packets and write them to alllog.txt, but "
                    "skip the apply-to-armature step entirely. Use this for "
                    "Fox Mocap dual-logging tests where the visual rig is "
                    "not needed and crashes in the legacy apply path would "
                    "otherwise kill Blender.",
        default=True)

    bpy.types.Scene.mvn_record_rate = bpy.props.EnumProperty(
        items=[
            ("23.98", "23.98 fps", "23.98 frames per second"),
            ("24", "24 fps", "24 frames per second"),
            ("25", "25 fps", "25 frames per second"),
            ("29.97", "29.97 fps", "29.97 frames per second"),
            ("30", "30 fps", "30 frames per second"),
            ("50", "50 fps", "50 frames per second"),
            ("59.94", "59.94 fps", "59.94 frames per second"),
            ("60", "60 fps", "60 frames per second"),
            ("120", "120 fps", "120 frames per second"),
            ("240", "240 fps", "240 frames per second"),
        ],
        default="60",
    )

    # --- Driver Functions ---
    load_handler(None)
    bpy.app.handlers.load_post.append(load_handler)

    # --- Store Online Status ---
    bpy.types.Scene.mvn_online = bpy.props.BoolProperty(name="Online", default=False)

    # --- Store Recording Status ---
    bpy.types.Scene.mvn_recording = bpy.props.BoolProperty(name="Recording", default=False)

    # --- Store Batch Map Status ---
    bpy.types.Scene.mvn_auto_update_constraints = bpy.props.BoolProperty(name="Batch Mapping", default=True)

    # --- Store Rotation Enumeration Property ---
    bpy.types.Scene.rotation_type = bpy.props.EnumProperty(
        items=[
            ("QUATERNION", "Quaternion", "Use Quaternion rotation"),
            ("EULER", "XYZ Euler", "Use Euler rotation"),
        ],
        default="QUATERNION",
    )

    # --- Store Source Armature ---
    bpy.types.Scene.source_armature = bpy.props.PointerProperty(name="Source Armature", description="Choose an armature from the list", type=Object, poll=poll_source_armatures, update=update_source_armature)

    # --- Store Target Armature ---
    bpy.types.Scene.target_armature = bpy.props.PointerProperty(name="Target Armature", description="Choose an armature from the list", type=Object, poll=poll_target_armatures, update=update_target_armature)

    # --- Store index of selected bone ---
    bpy.types.Scene.bones_active_index = bpy.props.IntProperty(name="", default=0)

    # --- Store Take Name & Number ---
    bpy.types.Scene.mvn_take_name = bpy.props.StringProperty(name="", default="New Session", update=update_xsens)
    bpy.types.Scene.mvn_take_number = bpy.props.IntProperty(name="", default=1, min=0, max=999, update=update_xsens)

    # --- Store MVN Record Trigger Data ---
    bpy.types.Scene.mvn_trigger_xsens_record = bpy.props.BoolProperty(name="Trigger Xsens Record", default=False, update=update_xsens)
    bpy.types.Scene.mvn_trigger_xsens_port = bpy.props.StringProperty(name="Xsens Remote Port", default="6004", update=update_xsens)

    # --- Store Scene Scale Factor ---
    bpy.types.Scene.mvn_scene_scale = bpy.props.FloatProperty(name="", default=1.0, min=0.00001, max=999.0, update=update_scene_scale)

    # --- Object Properties ---

    # --- Define property to allow bones to have a source bone ---
    bpy.types.Bone.mvn_source_bone_name = bpy.props.StringProperty(name="Source Bone", update=update_bonemap)
    bpy.types.Bone.mvn_tpos = bpy.props.FloatVectorProperty(name="", default=(-9999, -9999, -9999))
    bpy.types.Bone.mvn_tpos_rot = bpy.props.FloatVectorProperty(name="", subtype="QUATERNION", size=4, default=(1.0, 0.0, 0.0, 0.0))
    bpy.types.Bone.mvn_qdif = bpy.props.FloatVectorProperty(name="", subtype="QUATERNION", size=4, default=(1.0, 0.0, 0.0, 0.0))

    # --- Define armature properties ---
    bpy.types.Armature.mvn_is_source = bpy.props.BoolProperty(name="MVN Is Source", default=False)
    bpy.types.Armature.mvn_is_ik = bpy.props.BoolProperty(name="MVN Is IK", default=False)
    bpy.types.Armature.mvn_source_armature = bpy.props.PointerProperty(name="Source Armature", type=Object)

    # --- Define armature IK properties ---
    bpy.types.Armature.mvn_ik_armature = bpy.props.PointerProperty(name="IK Armature", type=Object)
    bpy.types.Armature.mvn_tposemode = bpy.props.BoolProperty(name="TPose Mode", default=True, description="Set a T-Pose for bone mapping to take affect", update=apply_tpose)
    bpy.types.Armature.mvn_ik_enabled = bpy.props.BoolProperty(name="IK Enabled", default=False, update=resolve_ik_armature)
    bpy.types.Armature.mvn_ik_error = bpy.props.StringProperty(name="IK Error Message", default="")
    bpy.types.Armature.mvn_ik_matchsource = bpy.props.BoolProperty(name="IK Match Source", default=True, update=resolve_ik_armature)
    bpy.types.Armature.mvn_ik_walkonspot = bpy.props.BoolProperty(name="IK Walk On Spot", default=False)
    bpy.types.Armature.mvn_ik_feet = bpy.props.FloatProperty(name="IK Feet", default=1.0, min=0.0, max=1.0, update=update_ik_feet)
    bpy.types.Armature.mvn_ik_feetspread = bpy.props.FloatProperty(name="IK Feet Spread", default=0.0)
    bpy.types.Armature.mvn_ik_levelhips = bpy.props.FloatProperty(name="IK Hip Level", default=0.0)
    bpy.types.Armature.mvn_ik_offsethips = bpy.props.FloatVectorProperty(
        name="IK Hip Offset",
        default=(0, 0, 0),
    )

    # --- Register Icons ---
    icons_dir = Path(__file__).parent / "icons"
    pcoll = previews.new()
    icon_list = {"MVN": "icon_xsens_white", "RECORD": "icon_record", "STOP": "icon_stop"}
    for code, file in icon_list.items():
        pcoll.load(code, str(icons_dir / (file + ".png")), "IMAGE")
        ICON_COLLECTION[code] = pcoll


def unregister():
    """
    Unregisters classes, properties, and driver functions for the MVN Live Plugin. Also initializes
    the plugin's icon collection from the "icons" directory.
    """
    save_config_file()

    for cls in classes:
        unregister_class(cls)

    del bpy.types.Scene.mvn_stream_address
    del bpy.types.Scene.mvn_stream_port
    del bpy.types.Scene.mvn_update_frequency
    del bpy.types.Scene.mvn_float_precision
    del bpy.types.Scene.mvn_scene_units
    del bpy.types.Scene.mvn_gloves_enabled
    try:
        del bpy.types.Scene.mvn_auto_start_on_load
        del bpy.types.Scene.mvn_test_logging
        del bpy.types.Scene.mvn_log_only_mode
    except Exception:
        pass
    del bpy.types.Scene.mvn_online
    del bpy.types.Scene.mvn_recording
    del bpy.types.Scene.mvn_auto_update_constraints
    del bpy.types.Scene.rotation_type
    del bpy.types.Scene.source_armature
    del bpy.types.Scene.target_armature
    del bpy.types.Scene.bones_active_index
    del bpy.types.Scene.mvn_take_name
    del bpy.types.Scene.mvn_take_number
    del bpy.types.Scene.mvn_trigger_xsens_record
    del bpy.types.Scene.mvn_trigger_xsens_port
    del bpy.types.Scene.mvn_scene_scale

    del bpy.types.Bone.mvn_source_bone_name
    del bpy.types.Bone.mvn_tpos
    del bpy.types.Bone.mvn_tpos_rot
    del bpy.types.Bone.mvn_qdif

    del bpy.types.Armature.mvn_is_source
    del bpy.types.Armature.mvn_is_ik
    del bpy.types.Armature.mvn_source_armature
    del bpy.types.Armature.mvn_ik_armature
    del bpy.types.Armature.mvn_tposemode
    del bpy.types.Armature.mvn_ik_enabled
    del bpy.types.Armature.mvn_ik_error
    del bpy.types.Armature.mvn_ik_matchsource
    del bpy.types.Armature.mvn_ik_walkonspot
    del bpy.types.Armature.mvn_ik_feet
    del bpy.types.Armature.mvn_ik_feetspread
    del bpy.types.Armature.mvn_ik_levelhips
    del bpy.types.Armature.mvn_ik_offsethips
    if hasattr(bpy.types.Armature, "mvn_ik_hand"):
        del bpy.types.Armature.mvn_ik_hand

    try:
        for pcoll in ICON_COLLECTION.values():
            previews.remove(pcoll)
    except Exception as e:
        print(f"__init__.py: Failed to remove previews: {e}")
    IconManager.cleanup()

    bpy.app.handlers.load_post.remove(load_handler)

    # Close the per-session file log if it was opened during streaming.
    try:
        file_logger.stop_logging()
    except Exception:
        pass


# *********************************************************************************************************************
# Main Execution
if __name__ == "__main__":
    register()
