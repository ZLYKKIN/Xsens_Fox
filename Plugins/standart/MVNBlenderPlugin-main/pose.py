# **********************************************************************************************************************
# pose.py:  contains the Bone and MocapPose dataclasses used to store and process incoming data from the MVN Live stream.
# **********************************************************************************************************************
from typing import Optional, Any
from dataclasses import dataclass, field

import bpy
from mathutils import Vector, Quaternion

from . import segment_maps

SCENE_SCALE_MULTIPLIER = 1


# ============================================================================================
@dataclass
class Bone:
    """
    Class used to represent a single bone in a skeleton hierarchy.

    Stores information such as the bone's name, its parent, its children, and whether it is a leaf or root bone.
    It also stores a segment index and a delta quaternion representing the angle between the bone and its parent or the vertical axis.

    Attributes
    ----------
    name : str
        The name of the bone.
    parent : Optional[Bone]
        The parent bone of the current bone. None if it's a root bone.
    children : list[Bone]
        The list of child bones of the current bone.
    is_leaf_bone : bool
        A flag indicating whether the bone is a leaf bone (i.e., it has no children).
    is_root_bone : bool
        A flag indicating whether the bone is a root bone (i.e., it has no parent).
    segment_index : Optional[int]
        The index of the segment that the bone represents.
    delta_quaternion : Optional[Quaternion]
        The quaternion angle between the bone and its parent or the vertical axis.
    """

    name: str
    parent: Optional["Bone"] = None
    children: list["Bone"] = field(default_factory=list)
    is_leaf_bone: bool = False
    is_root_bone: bool = False
    segment_index: Optional[int] = None
    delta_quaternion: Optional[Quaternion] = None  # quaternion angle between self and parent/vertical axis


# ============================================================================================
@dataclass
class MocapPose:
    """
    A class used to represent a single pose in a motion capture sequence.

    This class stores information about the character, the type of message, scale and quaternion data, and various other properties
    related to the pose.
    It also contains methods for processing scale and quaternion data, building a bone hierarchy, and handling prop data.

    Attributes
    ----------
    character_id : Optional[int]
    actor_name : Optional[str]
    actor_color : Optional[str]
    message_type : str
    scale_message : Optional[list[tuple[float, float, float]]]
    quaternion_message : Optional[list[tuple[float, float, float, float, float, float, float, float]]]
    scale_transform_dict : Optional[dict[tuple[Vector]]]
    skeleton_transform_dict : Optional[dict[tuple[Vector, Optional[Quaternion]]]]
    pause_stream : bool
    refresh_scale : bool
    glove_transforms : dict[str, Any]
    props : dict[str, Any]
    vive_trackers : list[Any]
    vive_headsets : list[Any]
    vive_objects : list[Any]
    create_object_cluster : bool
        A flag indicating whether to create an object cluster.
    is_object_cluster : bool
        A flag indicating whether it is an object cluster.
    gloves : bool
        A flag indicating whether gloves are used.
    prop_count : Optional[int]
    time_offset : Optional[float]
    timecode : Optional[str]
    tc_frame : Optional[int]
    frame_rate : float
    scene_scale_multiplier : float
    armature_object : Optional[bpy.types.Object]
    bone_hierarchy : Optional[dict[str, Bone]]
    bone_orientations : dict[str, Any]

    Methods
    -------
    process_scale_message()
    process_quaternion_data()
    """

    # --- Basic Properties ---
    character_id: Optional[int] = None
    actor_name: Optional[str] = None
    actor_color: Optional[str] = None

    # --- Stream Message Data ---
    message_type: str = "UnidentifiedMessage"
    scale_message: Optional[list[tuple[float, float, float]]] = None
    quaternion_message: Optional[list[tuple[float, float, float, float, float, float, float, float]]] = None

    # --- Transform Dictionaries ---
    scale_transform_dict: Optional[dict[tuple[Vector]]] = None
    skeleton_transform_dict: Optional[dict[tuple[Vector, Optional[Quaternion]]]] = None

    # --- Stream Control ---
    pause_stream: bool = False
    refresh_scale: bool = True

    # --- Blender Properties ---
    glove_transforms: dict[str, Any] = field(default_factory=dict)
    props: dict[str, Any] = field(default_factory=dict)
    vive_trackers: list[Any] = field(default_factory=list)
    vive_headsets: list[Any] = field(default_factory=list)
    vive_objects: list[Any] = field(default_factory=list)
    create_object_cluster: bool = False
    is_object_cluster: bool = False
    gloves: bool = False
    prop_count: Optional[int] = 0

    # --- Scene and timing Information ---
    time_offset: Optional[float] = None
    timecode: Optional[str] = None
    tc_frame: Optional[int] = None
    frame_rate: float = 240.0
    scene_scale_multiplier: float = SCENE_SCALE_MULTIPLIER

    # --- Blender Armature ---
    armature_object: Optional[bpy.types.Object] = None
    bone_hierarchy: Optional[dict[str, Bone]] = None
    bone_orientations: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        self.bone_orientations = segment_maps.bone_orientations_map

    # ---------------------------------------------------------------------------------------------------------------------------------------
    def process_scale_message(self):
        self._update_scale_transforms()
        self._apply_scene_scale_to_transforms()
        self._apply_axis_conversion_to_transforms()
        self.build_bone_hierarchy()
        self._process_prop_data()

    def _update_scale_transforms(self):
        """
        Updates the scale transforms of the MocapPose object.

        This method iterates over the scale_message attribute, which is a list of tuples containing segment data.
        Each tuple contains a segment ID and transform data. The method maps the segment ID to a segment name
        using the segment_map from the segment_maps module. It then updates the scale_transform_dict attribute
        with the segment name as the key and the transform data as the value.
        """
        preprocessed_data = {}
        for segment_data in self.scale_message:
            segment_id, *transform_data = segment_data

            if type(segment_id) == int:
                segment_name = segment_maps.segment_map.get(segment_id)
            else:
                segment_name = segment_id

            preprocessed_data[segment_name] = segment_data[1:]

        self.scale_transform_dict = preprocessed_data

    def _apply_scene_scale_to_transforms(self):
        """
        Applies scene scale to 'scale_transform_dict'. Adjusts each transform data to match the scale of the scene.
        """
        for segment_name, transform_data in self.scale_transform_dict.items():
            x_position, y_position, z_position = transform_data
            self.scale_transform_dict[segment_name] = (
                x_position * self.scene_scale_multiplier,
                y_position * self.scene_scale_multiplier,
                z_position * self.scene_scale_multiplier,
            )

    def _apply_axis_conversion_to_transforms(self):
        """
        Applies axis conversion to 'scale_transform_dict'. Converts each transform data to match the coordinate system of the scene.
        """
        for segment_name, transform_data in self.scale_transform_dict.items():
            x_pos, y_pos, z_pos = transform_data
            converted_axis_vector = Vector((y_pos, -x_pos, z_pos))
            self.scale_transform_dict[segment_name] = converted_axis_vector

    def build_bone_hierarchy(self, segment_map=segment_maps.stream_hierarchy_map):
        """
        Takes a segment map and creates a Bone object for each segment. Assigns parent and children attributes of each Bone
        based on the segment map. The resulting bone hierarchy is stored in 'bone_hierarchy'.
        """
        bone_hierarchy = {name: Bone(name) for name, _ in segment_map}

        for child_name, parent_name in segment_map:
            bone_hierarchy[child_name].parent = bone_hierarchy.get(parent_name)
            if parent_name and parent_name in bone_hierarchy:
                bone_hierarchy[parent_name].children.append(bone_hierarchy[child_name])

        self.bone_hierarchy = bone_hierarchy

    def _process_prop_data(self):
        """
        Initializes 'props' with keys "Prop1" to "Prop4" set to None. Iterates over 'scale_transform_dict'. If a segment
        name ends with a prop name from 'expected_prop_names', increments a count and updates the corresponding prop key in
        'props' with the segment name.
        """
        self.props = {"Prop1": None, "Prop2": None, "Prop3": None, "Prop4": None}

        count = 0
        for segment_name, transform_data in self.scale_transform_dict.items():
            for prop_name in segment_maps.expected_prop_names:
                if str(segment_name).endswith(prop_name):
                    count += 1
                    self.props[f"Prop{count}"] = segment_name

    def process_quaternion_data(self):
        self._determine_prop_count()
        self._create_skeleton_transform_dict(self.quaternion_message, self.prop_count, self.gloves)
        self._scale_transforms()
        self._convert_vectors()
        self._convert_to_quaternions()

    def _determine_prop_count(self):
        """
        Determines the prop count based on the length of 'quaternion_message'. Sets 'gloves' to True if gloves are present.
        Uses a hardcoded map to set 'prop_count' based on the number of segments.
        """
        prop_count_map = {
            23: 0,  # No props are used
            24: 1,  # Single prop is used
            25: 2,  # Two props are used
            26: 3,  # Three props are used
            27: 4,  # Four props are used
            63: 0,  # Gloves are used with no props
            64: 1,  # Gloves are used with one prop
            65: 2,  # Gloves are used with two props
            66: 3,  # Gloves are used with three props
            67: 4,  # Gloves are used with four props
        }

        number_of_segments = len(self.quaternion_message)
        self.gloves = False

        # --- Check for gloves ---
        if 32 <= number_of_segments <= 67:
            self.gloves = True

        if 32 <= number_of_segments < 63:
            # --- Gloves are present, prop count is undetermined ---
            self.prop_count = 0
        elif number_of_segments in prop_count_map:
            # --- Set prop count based on hardcoded map ---
            self.prop_count = prop_count_map[number_of_segments]
        else:
            self.prop_count = 0

    def _create_skeleton_transform_dict(self, stream_transform_data: list, prop_count: int, gloves: bool):
        """
        Creates a dictionary mapping each segment of a motion capture pose to its corresponding transformation data.
        Adjusts the segment mapping based on the presence of gloves and the prop count.

        Parameters:
        stream_transform_data (list): A list of tuples containing transformation data for each segment.
        prop_count (int): The number of props used in the motion capture session.
        gloves (bool): A flag indicating whether gloves were used in the motion capture session.
        """
        preprocessed_data = {}
        if prop_count == 0 and not gloves:
            for segment_data in stream_transform_data:
                segment_id, *transform_data = segment_data

                if type(segment_id) == int:
                    segment_name = segment_maps.segment_map.get(segment_id)
                else:
                    segment_name = segment_id

                preprocessed_data[segment_name] = segment_data[1:]

            self.skeleton_transform_dict = preprocessed_data

        elif prop_count == 0 and gloves:
            for segment_data in stream_transform_data:
                segment_id, *transform_data = segment_data

                if type(segment_id) == int and segment_id <= 23:
                    segment_name = segment_maps.segment_map.get(segment_id)

                elif type(segment_id) == int and segment_id > 23:
                    adjusted_segment_id = segment_id + 4
                    segment_name = segment_maps.segment_map.get(adjusted_segment_id)

                else:
                    segment_name = segment_id

                preprocessed_data[segment_name] = segment_data[1:]

            self.skeleton_transform_dict = preprocessed_data

        elif prop_count != 0:
            for segment_data in stream_transform_data:
                segment_id, *transform_data = segment_data

                if type(segment_id) == int and segment_id <= 23:
                    segment_name = segment_maps.segment_map.get(segment_id)

                elif type(segment_id) == int and segment_id <= (23 + prop_count):
                    segment_name = segment_maps.segment_map.get(segment_id)

                elif type(segment_id) == int and segment_id > (23 + prop_count):
                    adjusted_segment_id = segment_id + (4 - prop_count)
                    segment_name = segment_maps.segment_map.get(adjusted_segment_id)

                else:
                    segment_name = segment_id

                preprocessed_data[segment_name] = segment_data[1:]

            self.skeleton_transform_dict = preprocessed_data

    # ---------------------------------------------------------------------------------------------------------------------------------------

    def _scale_transforms(self):
        """
        Scales the position data in the skeleton_transform_dict by the scene_scale_multiplier.
        """
        for segment_name, transform_data in self.skeleton_transform_dict.items():
            x_position, y_position, z_position, w_rotation, x_rotation, y_rotation, z_rotation = transform_data
            self.skeleton_transform_dict[segment_name] = (
                x_position * self.scene_scale_multiplier,
                y_position * self.scene_scale_multiplier,
                z_position * self.scene_scale_multiplier,
                w_rotation,
                x_rotation,
                y_rotation,
                z_rotation,
            )

    def _convert_vectors(self):
        """
        Converts the position data in the skeleton_transform_dict to match the coordinate system of the scene.
        """
        for segment_name, transform_data in self.skeleton_transform_dict.items():
            x_position, y_position, z_position, *rotation_data = transform_data
            converted_axis_vector = Vector((y_position, z_position, x_position))
            self.skeleton_transform_dict[segment_name] = (converted_axis_vector, *rotation_data)

    def _convert_to_quaternions(self):
        """
        Converts the rotation data in the skeleton_transform_dict to Quaternion format.
        """
        for segment_name, transform_data in self.skeleton_transform_dict.items():
            position_vector, w_rotation, x_rotation, y_rotation, z_rotation = transform_data
            self.skeleton_transform_dict[segment_name] = (
                position_vector,
                Quaternion((w_rotation, x_rotation, y_rotation, z_rotation)),
            )


# *********************************************************************************************************************
# Execute
if __name__ == "__main__":
    pass
