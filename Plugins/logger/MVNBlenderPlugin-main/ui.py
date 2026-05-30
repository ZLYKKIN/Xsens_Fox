# **********************************************************************************************************************
# ui.py: Manage the UI elements for the MVN Xsens/Blender integration.
# **********************************************************************************************************************
import os
import re
import csv
import configparser
from pathlib import Path
from typing import Optional

import bpy
from mathutils import Vector
from bpy_extras.io_utils import ImportHelper, ExportHelper

from .logger import FoxLog
from .rigging import resolve_ik_armature
from .receiver import start_receiver, stop_receiver
from .segment_maps import segment_map, auto_remappings
from .recorder import update_xsens_remote, start_recording, stop_recording


# ============================================================================================
# Constants
ICON_FILES = {
    "START_STREAM": "icon_play",
    "PAUSE_STREAM": "icon_pause",
    "XSENS": "icon_xsens_white",
    "RECORD": "icon_record",
    "STOP": "icon_stop",
}

CONFIG_MAIN = {
    "address": "mvn_stream_address",
    "port": "mvn_stream_port",
    "update_frequency": "mvn_update_frequency",
    "float_precision": "mvn_float_precision",
    "scene_units": "mvn_scene_units",
    "gloves_enabled": "mvn_gloves_enabled",
}

PREV_SESSION_TARGET_MAP = {}

# ============================================================================================
# Classes: UI Panels
class IconManager:
    """
    Singleton class used to manage icons.

    provides a centralized location to load and access the icons used throughout the plugin.
    The icons are loaded from the "icons" directory located in the same location as this file.

    Attributes
    ----------
    _instance : IconManager
    preview_collection : bpy.utils.previews
        The collection of previews (icons) used in the plugin.
    icons_dir : str
        The directory where the icons are located.

    Methods
    -------
    __new__(cls)
        Creates a new instance of the IconManager class if one doesn't exist already.
    load_icons()
        Loads the icons into the preview collection.
    get_icon_id(name: str) -> Optional[int]
        Returns the icon_id of the icon with the given name if it exists in the preview collection, otherwise returns None.
    cleanup()
        Removes the preview collection and resets the singleton instance.
    """

    _instance = None
    preview_collection = None
    icons_dir = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(IconManager, cls).__new__(cls)
            cls.preview_collection = bpy.utils.previews.new()
            cls.icons_dir = os.path.join(os.path.dirname(__file__), "icons")
            cls.load_icons()
        return cls._instance

    @classmethod
    def load_icons(cls):
        for key, filename in ICON_FILES.items():
            icon_path = os.path.join(cls.icons_dir, filename + ".png")
            cls.preview_collection.load(key, icon_path, "IMAGE")

    @classmethod
    def get_icon_id(cls, name):
        return cls.preview_collection[name].icon_id if name in cls.preview_collection else None

    @classmethod
    def cleanup(cls):
        if cls.preview_collection:
            bpy.utils.previews.remove(cls.preview_collection)
            cls.preview_collection = None
            cls._instance = None


# ------------------------------------------------------------------------------------------------------------------------------------------
# Functions: Config
def load_config_file():
    global CONFIG_MAIN
    scene = bpy.context.scene
    file_path = Path(__file__).parent / "config.ini"
    config = configparser.ConfigParser()
    config.read(file_path)

    for name, prop in CONFIG_MAIN.items():
        try:
            setattr(scene, prop, type(getattr(scene, prop))(config["MAIN"][name]))
        except Exception as e:
            print(e)
            print(f"Error loading config property: {name}")
    save_config_file()


def save_config_file(self=None, context=None):
    global CONFIG_MAIN
    scene = bpy.context.scene
    file_path = Path(__file__).parent / "config.ini"
    config = configparser.ConfigParser()
    config["MAIN"] = {}
    for name, prop in CONFIG_MAIN.items():
        try:
            config["MAIN"][name] = str(getattr(scene, prop))
        except Exception as e:
            message = f"Error saving config property: {name}: {e}"
            bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="ERROR", message_text=message)
    with open(file_path, "w") as config_file:
        config.write(config_file)


# ------------------------------------------------------------------------------------------------------------------------------------------
# Classes: UI
# --- Panels ---
class MainPanel(object):
    """
    Base class for creating UI panels.

    This class provides the basic attributes needed for creating a panel in the Blender UI.
    It is intended to be subclassed by other classes that define specific panels.

    Attributes
    ----------
    bl_label : str
        The name of the panel, displayed at the top of the panel.
    bl_idname : str
        The unique identifier for this panel.
    bl_category : str
        The panel category, this determines the text that's displayed in the tab.
    bl_space_type : str
        The space where the panel is displayed.
    bl_region_type : str
        The region where the panel is displayed.
    """

    bl_label = "Xsens"
    bl_idname = "VIEW3D_TS_xsens"
    bl_category = "Xsens"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"


class ReceiverPanel(MainPanel, bpy.types.Panel):
    """
    This class inherits from the MainPanel and bpy.types.Panel classes and is used to create the Receiver Panel in the Blender UI.
    The Receiver Panel is used to control the live-streaming of data from the Xsens suit.

    Attributes
    ----------
    bl_idname : str
        The unique identifier for this panel.
    bl_label : str
        The name of the panel, displayed at the top of the panel.

    Methods
    -------
    draw(context)
        Draws the UI elements of the Receiver Panel.
    """

    bl_idname = "VIEW3D_PT_mvn_streamer"
    bl_label = "Xsens Live Streaming"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = False

        row = add_row(layout, context, "mvn_stream_address", "Address ")
        row.enabled = not context.scene.mvn_online

        row = add_row(layout, context, "mvn_stream_port", "Port ")
        row.enabled = not context.scene.mvn_online

        add_separator(layout, scale=0.1)

        row = add_row(layout, context, "mvn_scene_scale", "Scene Scale ")
        row.enabled = not context.scene.mvn_online

        add_separator(layout, scale=0.1)

        start_stream_icon_id = IconManager().get_icon_id("START_STREAM")
        pause_stream_icon_id = IconManager().get_icon_id("PAUSE_STREAM")

        row, icon_id = add_button(layout, "START_STREAM")  # Stream button

        # if not in object mode, disable the stream button
        if bpy.context.mode != "OBJECT":
            row.enabled = False

        if context.scene.mvn_online:
            row.operator("wm.startstream", text="Pause Stream", icon_value=pause_stream_icon_id, depress=True)
        else:
            row.operator("wm.startstream", text="Start Stream", icon_value=start_stream_icon_id, depress=False)

        add_separator(layout, scale=0.2)


class RecorderPanel(MainPanel, bpy.types.Panel):
    """
    This class inherits from the MainPanel and bpy.types.Panel classes and is used to create the Recorder Panel in the Blender UI.
    The Recorder Panel is used to control the live recording of data from the Xsens suit.

    Attributes
    ----------
    bl_idname : str
        The unique identifier for this panel.
    bl_label : str
        The name of the panel, displayed at the top of the panel in the UI.

    Methods
    -------
    draw(context)
        Draws the UI elements of the Recorder Panel.
    """

    bl_idname = "VIEW3D_PT_mvn_recorder"
    bl_label = "Xsens Live Recording"

    def draw(self, context):
        icon_manager = IconManager()
        layout = self.layout
        layout.use_property_split = False

        # --- Take Entry ---
        row = add_row(layout, context, "mvn_take_name", "Take ", split_factor=0.75)
        row.prop(context.scene, "mvn_take_number")
        row.enabled = not context.scene.mvn_recording

        # --- Record Button ---
        record_icon_id = icon_manager.get_icon_id("RECORD")
        stop_icon_id = icon_manager.get_icon_id("STOP")

        row, icon_id = add_button(layout, "RECORD")

        if not context.scene.mvn_recording:
            row.operator("wm.startrecord", text="Start Recording", icon_value=record_icon_id, depress=False)
        else:
            row.operator("wm.startrecord", text="Stop Recording", icon_value=stop_icon_id, depress=True)

        row.enabled = context.scene.mvn_online

        # --- Trigger Xsens Recording Options ---
        row = add_row(layout, context, "mvn_trigger_xsens_record", "Trigger Xsens Recording ", split_factor=0.6)
        row.enabled = not context.scene.mvn_recording
        col = row.column(align=True)

        col = row.column(align=True)
        col.prop(context.scene, "mvn_trigger_xsens_port", text="")
        col.enabled = context.scene.mvn_trigger_xsens_record

        # --- Frame Rate Setting ---
        row = add_row(layout, context, "mvn_record_rate", "Frame Rate ", split_factor=0.5)
        row.enabled = not context.scene.mvn_recording

        add_separator(layout, scale=0.2)


class RetargetingPanel(MainPanel, bpy.types.Panel):
    """
    This class inherits from the MainPanel and bpy.types.Panel classes and is used to create the Retargeting Panel in the Blender UI.
    The Retargeting Panel is used to control the retargeting of data from the Xsens suit to a target armature.

    Attributes
    ----------
    bl_idname : str
        The unique identifier for this panel.
    bl_label : str
        The name of the panel, displayed at the top of the panel.

    Methods
    -------
    draw(context)
        Draws the UI elements of the Retargeting Panel.
    """

    bl_idname = "VIEW3D_PT_mvn_retargeting"
    bl_label = "Xsens Live Retargeting"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = False

        # --- Target Armature Field ---
        row = layout.row(align=True)
        row.prop(context.scene, "target_armature", text="Target :", icon="OUTLINER_OB_ARMATURE", icon_only=True)

        # --- Source Armature Field ---
        row = layout.row(align=True)
        row.prop(context.scene, "source_armature", text="Source :", icon="OUTLINER_OB_ARMATURE", icon_only=True)
        row.enabled = context.scene.target_armature is not None

        add_separator(layout, scale=0.1)

        # --- Check if target armature is selected ---
        if context.scene.target_armature:
            row = layout.row(align=True)
            if context.scene.target_armature.data.mvn_tposemode:
                row.prop(context.scene.target_armature.data, "mvn_tposemode", text="Apply T-Pose", toggle=True)
            else:
                row.prop(context.scene.target_armature.data, "mvn_tposemode", text="Edit T-Pose", toggle=True)
            add_separator(layout, scale=0.2)

        # --- Check if both armatures are selected ---
        if context.scene.source_armature and context.scene.target_armature:
            layout = layout.box()
            layout.enabled = not context.scene.target_armature.data.mvn_tposemode

            # --- Remap Label ---
            row = layout.row(align=True)
            row.label(text="Bone Remapping :", icon="GROUP_BONE")

            # --- Auto-mapping button ---
            row = layout.row(align=True)
            row.operator("wm.automap", text="Auto Map Bones")

            # --- Bone List ---
            layout.template_list(
                "MVN_UL_BoneList",
                "Bone List",
                context.scene.target_armature.data,
                "bones",
                context.scene,
                "bones_active_index",
                rows=1,
                maxrows=12,
            )

            # --- Buttons for importing and saving mappings ---
            row = layout.row(align=True)
            row.operator("wm.loadmap", text="Load Map")
            row.operator("wm.savemap", text="Save Map")
            row.operator("wm.clearmap", text="Clear Map")

            add_separator(layout, scale=0.1)

            # --- IK Label ---
            row = layout.row(align=True)
            row.prop(
                context.scene.target_armature.data,
                "mvn_ik_enabled",
                text="Enable IK Settings",
                icon="CON_KINEMATIC",
                toggle=True,
            )
            row.enabled = context.scene.source_armature.data.mvn_is_source

            # --- IK Settings ---
            if context.scene.target_armature.data.mvn_ik_enabled:
                if context.scene.target_armature.data.mvn_ik_error != "":
                    row = layout.row(align=True)
                    row.label(text=context.scene.target_armature.data.mvn_ik_error, icon="INFO")
                else:
                    add_separator(layout, scale=0.05)

                    # --- Match Source IK ---
                    row = layout.row(align=True)
                    row.label(text="Match Source : ")
                    row.prop(context.scene.target_armature.data, "mvn_ik_matchsource", text="")

                    # --- Walk on the spot ---
                    row = layout.row(align=True)
                    row.label(text="Walk On Spot : ")
                    row.prop(context.scene.target_armature.data, "mvn_ik_walkonspot", text="")
                    add_separator(layout, scale=0.05)

                    # --- Foot IK ---
                    row = layout.row(align=True)
                    row = row.split(factor=0.45, align=True)
                    row.label(text="Feet IK Influence : ")
                    row.prop(context.scene.target_armature.data, "mvn_ik_feet", text="", slider=True)

                    # --- Foot spread IK ---
                    row = layout.row(align=True)
                    row = row.split(factor=0.45, align=True)
                    row.label(text="Feet Spread : ")
                    row.prop(context.scene.target_armature.data, "mvn_ik_feetspread", text="")
                    add_separator(layout, scale=0.05)

                    # --- Hip Level IK --
                    row = layout.row(align=True)
                    row = row.split(factor=0.45, align=True)
                    row.label(text="Hip Level : ")
                    row.prop(context.scene.target_armature.data, "mvn_ik_levelhips", text="")

                    # --- Hip Offset IK ---
                    row = layout.row(align=True)
                    row = row.split(factor=0.45, align=True)
                    col = row.column(align=True)
                    col.label(text="Hip Offset : ")
                    col = row.column(align=True)
                    col.prop(context.scene.target_armature.data, "mvn_ik_offsethips", text="X", index=0)
                    col.prop(context.scene.target_armature.data, "mvn_ik_offsethips", text="Y", index=1)
                    col.prop(context.scene.target_armature.data, "mvn_ik_offsethips", text="Z", index=2)
        else:
            row = layout.row(align=True)
            row.label(text="Select a Target and Source to begin retargetting.", icon="INFO")

        add_separator(layout, scale=0.4)


# ============================================================================================
# Classes: Operators
class WM_OT_StartStream(bpy.types.Operator):
    """
    This class inherits from bpy.types.Operator and is used to control the starting of the stream in the MVN Live Plugin.
    The operator is triggered by a UI button press and toggles the live stream of data from the Xsens suit.

    Attributes
    ----------
    bl_idname : str
        The unique identifier for this operator.
    bl_label : str
        The name of the operator, displayed as the button text.
    bl_description : str
        A brief description of what the operator does, displayed as a tooltip

    Methods
    -------
    execute(self, context)
        The method that is called when the operator button is pressed. It toggles the live stream and returns a status set.
    """
    global PREV_SESSION_TARGET_MAP
    bl_idname = "wm.startstream"
    bl_label = "Start"
    bl_description = "Toggle Xsens live stream"

    @staticmethod
    def execute(self, context):
        # --- Toggle Online ---
        context.scene.mvn_online = not getattr(context.scene, "mvn_online", False)
        print(context.scene.mvn_stream_address, ":", context.scene.mvn_stream_port)
        print(f"MVN Online: {context.scene.mvn_online}")

        # --- If going offline while recording stop recording first ---
        if not context.scene.mvn_online and context.scene.mvn_recording:
            context.scene.mvn_recording = False
            stop_recording()

        if context.scene.mvn_online:
            port_number = int(context.scene.mvn_stream_port)
            ip_address = context.scene.mvn_stream_address
            # --- (Re)create blender_log.log fresh for this streaming session ---
            FoxLog.reopen(
                {
                    "addr": ip_address,
                    "port": port_number,
                    "units": context.scene.mvn_scene_units,
                    "gloves": context.scene.mvn_gloves_enabled,
                    "scale": context.scene.mvn_scene_scale,
                    "rate": context.scene.mvn_update_frequency,
                }
            )
            start_receiver(ip_address, port_number, PREV_SESSION_TARGET_MAP)
        else:
            try:
                PREV_SESSION_TARGET_MAP.clear()
                for target_armature in bpy.data.armatures:
                    if source_armature := target_armature.mvn_source_armature:
                        PREV_SESSION_TARGET_MAP[target_armature.name] = source_armature.name

                stop_receiver()
                FoxLog.close({"frames": FoxLog.frames, "drops": FoxLog.drops})
            except Exception as e:
                message = f"Failed to stop receiver: {e}"
                bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="ERROR", message_text=message)
                FoxLog.error(f"failed to stop receiver: {e}")

        return {"FINISHED"}


class WM_OT_ToggleRecord(bpy.types.Operator):
    """
    This class inherits from bpy.types.Operator and is used to control the toggling of the recording in the MVN Live Plugin.
    The operator is triggered by a UI button press and toggles the live recording of data from the Xsens suit.

    Attributes
    ----------
    bl_idname : str
        The unique identifier for this operator.
    bl_label : str
        The name of the operator, displayed as the button text.
    bl_description : str
        A brief description of what the operator does, displayed as a tooltip.

    Methods
    -------
    execute(self, context)
        The method that is called when the operator button is pressed. It toggles the live recording and returns a status set.
    """

    bl_idname = "wm.startrecord"
    bl_label = "Start"
    bl_description = "Toggle Xsens live recording"

    @staticmethod
    def execute(self, context):
        # --- Toggle Record ---
        context.scene.mvn_recording = not getattr(context.scene, "mvn_recording", False)

        if context.scene.mvn_recording:
            start_recording()
        else:
            stop_recording()

        return {"FINISHED"}


class WM_OT_AutoMap(bpy.types.Operator):
    """
    Operator class to automatically map bones in the MVN Live Plugin.

    This class inherits from bpy.types.Operator and is used to control the automatic mapping of bones in the MVN Live Plugin.
    The operator is triggered by a UI button press and generates bone mappings based on predefined rules.

    Attributes
    ----------
    bl_idname : str
        The unique identifier for this operator.
    bl_label : str
        The name of the operator, displayed as the button text.
    bl_description : str
        A brief description of what the operator does, displayed as a tooltip.

    Methods
    -------
    execute(self, context)
        The method that is called when the operator button is pressed. It generates bone mappings and returns a status set.
    find_map(source_bone, target_bones)
        Finds a suitable target bone for a given source bone based on predefined rules.
    """

    bl_idname = "wm.automap"
    bl_label = "Auto Map Bones"
    bl_description = "Generate bone mappings automatically"

    def execute(self, context):
        if context.scene.target_armature and context.scene.source_armature:
            context.scene.mvn_auto_update_constraints = False

            # --- # For each source bone find a good target bone to parent ---
            for target_bone in context.scene.target_armature.data.bones:
                target_bone.mvn_source_bone_name = ""
            for source_bone in context.scene.source_armature.data.bones:
                self.find_map(source_bone, context.scene.target_armature.data.bones)

            # --- Update constraints ---
            context.scene.mvn_auto_update_constraints = True
            resolve_ik_armature()
        return {"FINISHED"}

    @staticmethod
    def find_map(source_bone, target_bones) -> Optional[str]:
        """
        Finds a suitable target bone for a given source bone based on predefined rules.

        This method iterates over a list of possible names for the source bone,
        and for each possible name, it iterates over the target bones. If it finds a target bone
        whose name matches the possible name, it sets the source bone name for the target bone
        and returns the name of the target bone.

        Parameters
        ----------
        source_bone : bpy.types.Bone
            The source bone for which a target bone is to be found.
        target_bones : bpy.types.bpy_prop_collection of bpy.types.Bone
            The collection of target bones to search for a match.

        Returns
        -------
        str or None
            The name of the target bone if a match is found, otherwise None.
        """
        possible_names = []
        for index, bone_name in segment_map.items():
            if source_bone.name == bone_name:
                possible_names = auto_remappings[index]
                break
        for p_name in possible_names:
            for target_bone in target_bones:
                if re.search(p_name, target_bone.name, re.IGNORECASE):
                    if target_bone.mvn_source_bone_name == "":
                        target_bone.mvn_source_bone_name = source_bone.name
                        return target_bone.name
        return None


class WM_OT_ClearMap(bpy.types.Operator):
    """
    Operator class to clear bone mappings in the MVN Live Plugin.

    This class inherits from bpy.types.Operator and is used to control the clearing of bone mappings in the MVN Live Plugin.
    The operator is triggered by a UI button press and clears all existing bone mappings.

    Attributes
    ----------
    bl_idname : str
        The unique identifier for this operator.
    bl_label : str
        The name of the operator, displayed as the button text.
    bl_description : str
        A brief description of what the operator does, displayed as a tooltip.

    Methods
    -------
    execute(self, context)
        The method that is called when the operator button is pressed. It clears all bone mappings and returns a status set.
    """

    bl_idname = "wm.clearmap"
    bl_label = "Clear Map Bones"
    bl_description = "Clear bone mappings"

    @staticmethod
    def execute(self, context):
        context.scene.mvn_auto_update_constraints = False

        # --- for each target bone set source bone to empty ---
        for target_bone in context.scene.target_armature.data.bones:
            target_bone.mvn_source_bone_name = ""

        # --- Update constraints ---
        context.scene.mvn_auto_update_constraints = True
        resolve_ik_armature()
        return {"FINISHED"}


class WM_OT_LoadMap(bpy.types.Operator, ImportHelper):
    """
    Operator class to load bone mappings in the MVN Live Plugin.

    This class inherits from bpy.types.Operator and ImportHelper and is used to control the loading of bone mappings in the MVN Live Plugin.
    The operator is triggered by a UI button press and loads bone mappings from a CSV file.

    Attributes
    ----------
    bl_idname : str
        The unique identifier for this operator.
    bl_label : str
        The name of the operator, displayed as the button text .
    bl_description : str
        A brief description of what the operator does, displayed as a tooltip.
    filename_ext : str
        The extension of the file to be loaded.
    filter_glob: bpy.props.StringProperty
        A glob pattern used to filter the files displayed in the file selector.

    Methods
    -------
    invoke(self, context, event)
        Method called when the operator is called. It sets the default directory for the file dialog.
    execute(self, context)
        The method that is called when the operator button is pressed. It loads the bone mappings from the selected file and returns a status set.
    apply_map_data(context, bone_map)
        Applies the loaded bone mappings to the target armature.
    """

    bl_idname = "wm.loadmap"
    bl_label = "Load Map"
    bl_description = "Load bone mappings from a save file"
    filename_ext = ".csv"
    filter_glob: bpy.props.StringProperty(default="*.csv", options={"HIDDEN"})

    def invoke(self, context, event):
        # --- Set the default directory for the file dialog ---
        path = bpy.path.abspath(os.path.dirname(__file__)) + "/BoneMaps/"
        if not os.path.exists(path):
            os.makedirs(path)
        name = context.scene.source_armature.name + "_to_" + context.scene.target_armature.name
        self.filepath = os.path.join(path, name + ".csv")
        return super().invoke(context, event)

    def execute(self, context):
        if self.filepath:
            file_path = bpy.path.abspath(self.filepath)
            bone_map = {}
            with open(file_path, "r") as csv_file:
                reader = csv.reader(csv_file)
                for row in reader:
                    key, value = row
                    bone_map[key] = value
                context.scene.mvn_auto_update_constraints = False
                self.apply_map_data(context, bone_map)
                context.scene.mvn_auto_update_constraints = True
                resolve_ik_armature()
        return {"FINISHED"}

    @staticmethod
    def apply_map_data(context, bone_map):
        for bone in context.scene.target_armature.data.bones:
            if bone.name in bone_map:
                bone.mvn_source_bone_name = bone_map[bone.name]
            else:
                bone.mvn_source_bone_name = ""


class WM_OT_SaveMap(bpy.types.Operator, ExportHelper):
    """
    Operator class to save bone mappings in the MVN Live Plugin.

    This class inherits from bpy.types.Operator and ExportHelper and is used to control the saving of bone mappings in the MVN Live Plugin.
    The operator is triggered by a UI button press and saves bone mappings to a CSV file.

    Attributes
    ----------
    bl_idname : str
        The unique identifier for this operator.
    bl_label : str
        The name of the operator, displayed as the button text.
    bl_description : str
        A brief description of what the operator does, displayed as a tooltip.
    filename_ext : str
        The extension of the file to be saved.
    filter_glob: bpy.props.StringProperty
        A glob pattern used to filter the files displayed in the file selector.

    Methods
    -------
    invoke(self, context, event)
        Method called when the operator is called. It sets the default directory for the file dialog.
    execute(self, context)
        The method that is called when the operator button is pressed. It saves the bone mappings to the selected file and returns a status set.
    create_map_data(context)
        Creates a dictionary of bone mappings to be saved.
    """

    bl_idname = "wm.savemap"
    bl_label = "Save Map"
    bl_description = "Save bone mappings to file"
    filename_ext = ".csv"
    filter_glob: bpy.props.StringProperty(default="*.csv", options={"HIDDEN"})

    def invoke(self, context, event):
        # --- Set the default directory for the file dialog ---
        path = bpy.path.abspath(os.path.dirname(__file__)) + "/BoneMaps/"
        if not os.path.exists(path):
            os.makedirs(path)
        name = context.scene.source_armature.name + "_to_" + context.scene.target_armature.name
        name = name.replace(".", "_")
        self.filepath = os.path.join(path, name + ".csv")
        return super().invoke(context, event)

    def execute(self, context):
        if self.filepath:
            new_map = self.create_map_data(context)
            file_path = bpy.path.abspath(self.filepath)
            with open(file_path, "w", newline="") as csv_file:
                writer = csv.writer(csv_file)
                for key, value in new_map.items():
                    writer.writerow([key, value])
        return {"FINISHED"}

    @staticmethod
    def create_map_data(context):
        new_map = {}
        for bone in context.scene.target_armature.data.bones:
            if bone.mvn_source_bone_name:
                new_map[bone.name] = bone.mvn_source_bone_name
        return new_map


# ============================================================================================
# Classes: Lists
class MVN_UL_BoneList(bpy.types.UIList):
    """
    UI List class for displaying bone mappings in the MVN Live Plugin.

    This class inherits from bpy.types.UIList and is used to display the bone mappings in the MVN Live Plugin.
    The list is displayed in the Retargeting Panel of the plugin's UI.

    Methods
    -------
    draw_item(self, context, layout, data, item, icon, active_data, active_propname, index)
        Draws the UI elements of each item in the bone list.
    """

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        armature_target = context.scene.target_armature
        armature_source = context.scene.source_armature
        layout = layout.split(factor=0.5, align=True)
        layout.enabled = not context.scene.target_armature.data.mvn_tposemode
        col = layout.column(align=True)
        col2 = layout.column(align=True)

        # --- Display target bone ---
        if armature_target:
            col.label(text=item.name, icon="BONE_DATA")

        # --- Display source bone ---
        if armature_source:
            col2.prop_search(item, "mvn_source_bone_name", armature_source.data, "bones", text="", icon="GROUP_BONE")

        # --- Disable if middle of a chain ---
        if item.mvn_source_bone_name == "-":
            col2.enabled = False
        else:
            col2.enabled = True


# ============================================================================================
# Functions: UI
def add_row(layout, context, property_name: str, text_lable: str, split_factor=1.0):
    row = layout.row(align=True)
    row = row.split(factor=split_factor, align=True)
    row.prop(context.scene, property_name, text=text_lable)
    return row


def add_button(layout, icon_id):
    row = layout.row(align=True)
    icon_manager = IconManager()
    icon_id = icon_manager.get_icon_id(icon_id)
    return row, icon_id


def add_separator(layout, scale=1.0):
    row = layout.row(align=True)
    row.scale_y = scale
    row.label(text="")


# --- Polls ---
def poll_source_armatures(self, obj) -> bool:
    """
    Filters the armatures that can be selected as source armatures in the dropdown menu.

    Checks if the object is of type "ARMATURE" and if it is not an IK armature.
    If both conditions are met, the function returns True, indicating that the object is a valid choice for a source armature.
    Otherwise, it returns False.
    """
    return obj.type == "ARMATURE" and not obj.data.mvn_is_ik


def poll_target_armatures(self, obj) -> bool:
    """
    Filters the armatures that can be selected as target armatures in the dropdown menu.

    Checks if the object is of type "ARMATURE" and if it is not a source or IK armature.
    If both conditions are met, the function returns True, indicating that the object is a valid choice for a target armature.
    Otherwise, it returns False.
    """
    return obj.type == "ARMATURE" and not obj.data.mvn_is_source and not obj.data.mvn_is_ik


def update_target_armature(self, obj):  # Runs when target dropdown changes value
    scene = bpy.context.scene
    if scene.target_armature:
        scene.source_armature = scene.target_armature.data.mvn_source_armature
    else:
        scene.source_armature = None


def set_target_armature(self, target_armature_name, source_armature_name):
    scene = bpy.context.scene
    target_armature = bpy.data.objects.get(target_armature_name)
    source_armature = bpy.data.objects.get(source_armature_name)

    if target_armature and source_armature:
        scene.target_armature = target_armature
        scene.source_armature = source_armature
    else:
        scene.target_armature = None
        scene.source_armature = None


def update_source_armature(self, obj):
    """
    Updates the source armature when the target armature dropdown changes value.

    Checks if a target armature is selected in the scene. If so, it sets the source armature to be the
    source armature associated with the selected target armature. If no target armature is selected, it sets the source
    armature to None.
    """
    scene = bpy.context.scene
    if scene.target_armature:
        scene.target_armature.data.mvn_source_armature = scene.source_armature
        resolve_ik_armature(self, obj)


def update_ik_feet(self, object):
    """
    Updates the IK feet constraints when the IK feet slider value changes.

    Checks if an IK armature is associated with the target armature. If so, it iterates over the pose bones
    of the target armature. For each pose bone, it iterates over its constraints. The function is intended to update these
    constraints based on the new IK feet slider value.
    """
    scene = bpy.context.scene
    if scene.target_armature.data.mvn_ik_armature:
        for pose_bone in scene.target_armature.pose.bones:
            for const in pose_bone.constraints:
                if "MVN_IK_FEET" in const.name:
                    const.influence = scene.target_armature.data.mvn_ik_feet
                elif "MVN_FEET" in const.name:
                    const.influence = 1 - scene.target_armature.data.mvn_ik_feet


def update_xsens(self, object):
    """
    Updates the Xsens recording information when it changes.

    Triggered when the Xsens recording information is modified. It calls the `update_xsens_remote` function
    which is responsible for updating the remote Xsens device with the new recording information.
    """
    update_xsens_remote()


def update_scene_scale(self, object):
    """
    Updates the scale of the source armatures when the scene scale factor changes.

    Triggered when the scene scale factor is modified. It iterates over all objects in the scene,
    and for each object that is a source armature, it updates the scale to match the new scene scale factor.
    """
    scene = bpy.context.scene
    new_scale = Vector((scene.mvn_scene_scale, scene.mvn_scene_scale, scene.mvn_scene_scale))
    for obj in bpy.context.scene.objects:
        if obj.type == "ARMATURE" and obj.data.mvn_is_source:
            obj.scale = new_scale
    bpy.context.view_layer.update()
