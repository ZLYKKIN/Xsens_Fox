# *********************************************************************************************************************
# Source Manager: Handles Armature and source object creation and deletion
# *********************************************************************************************************************
import bpy
from mathutils import Vector, Quaternion

from . import segment_maps
from .pose import MocapPose
from .utility import add_to_collection


# ============================================================================================
# Functions: Armature
def rescale_character(mocap_pose: MocapPose):
    """
    Rescales the character's armature based on the mocap data.

    Uses the mocap_pose object, which contains the armature object and the scale transform dictionary.
    Iterates over the bones in the armature and adjusts their positions based on the mocap data.
    If the bone is the pelvis, adjusts the tail position and roll of the bone.
    After adjusting the bones, the function sets the scale of the armature to match the scene scale.

    For each prop in the mocap_pose, if the prop's segment ID and name are not None and the prop's name matches the bone name,
    Adjusts the location and rotation of the corresponding empty object in the scene.

    The function then sets the mode back to 'OBJECT'.
    """
    if not mocap_pose.armature_object:
        return

    armature = mocap_pose.armature_object

    bpy.context.view_layer.objects.active = armature
    armature.select_set(True)

    bpy.ops.object.mode_set(mode="EDIT")

    edit_bones = armature.data.edit_bones
    for bone_name, transform_data in mocap_pose.scale_transform_dict.items():

        if bone_name in edit_bones:

            bone = edit_bones[bone_name]

            if bone_name == "Pelvis":
                head_position: Vector = mocap_pose.scale_transform_dict[bone_name]
                tail_position: Vector = head_position + Vector((0, 0, 5))
                roll = mocap_pose.bone_orientations[bone_name]

                head_position, tail_position = compensate_for_head_tail_discrepancy(head_position, tail_position)

                bone.head = head_position
                bone.tail = tail_position
                bone.roll = roll

    mocap_pose.armature_object = armature
    bpy.ops.object.mode_set(mode="OBJECT")

    # --- Set scene scale according to users scene scale entry ---
    scene = bpy.context.scene
    new_scale = Vector((scene.mvn_scene_scale, scene.mvn_scene_scale, scene.mvn_scene_scale))
    armature.scale = new_scale

    for (
        prop_segment_id,
        prop_name,
    ) in mocap_pose.props.items():  # Example: bone name 'Prop1', prop name 'RightHandSword'

        if prop_segment_id and prop_name is not None:

            if bone_name == prop_name:

                target_empty_name = f"{mocap_pose.character_id}_{prop_segment_id}"

                if target_empty_name in bpy.data.objects:
                    obj = bpy.data.objects[target_empty_name]

                    obj.location = mocap_pose.skeleton_transform_dict[prop_segment_id][0]
                    obj.rotation_mode = "QUATERNION"
                    obj.rotation_quaternion = mocap_pose.skeleton_transform_dict[prop_segment_id][1]

    bpy.ops.object.mode_set(mode="OBJECT")


def create_bone(armature, bone_name: str, parent_name: str, head: Vector, tail: Vector, roll=0.0):
    """
    Creates a new bone in the given armature.

    Takes in the armature object, the name of the new bone, the name of the parent bone,
    the head and tail positions (as Vector objects), and the roll angle (default is 0.0).

    If the bone already exists in the armature, the function does nothing. Otherwise, it creates a new bone,
    sets its head, tail, and roll properties, and assigns it a parent bone if a valid parent name is provided.

    Also calculates and returns the delta quaternion between the edit bone and vertical axis.

    If an error occurs while setting the bone properties, a warning message is logged.
    """
    if bone_name in armature.data.edit_bones:
        return

    bpy.ops.object.mode_set(mode="EDIT")
    edit_bones = armature.data.edit_bones
    edit_bone = edit_bones.new(bone_name)

    edit_bone.head = head
    edit_bone.tail = tail
    edit_bone.roll = roll

    edit_bone_matrix = edit_bone.matrix.copy()
    delta_quaternion = edit_bone_matrix.to_quaternion() @ Quaternion((0.7071, 0.7071, 0, 0))

    if parent_name and parent_name in edit_bones:
        edit_bone.parent = edit_bones[parent_name]

    try:
        bpy.ops.object.mode_set(mode="OBJECT")
        bone = armature.data.bones[bone_name]
        pose_bone = armature.pose.bones[bone_name]
        bone.mvn_tpos = head
        rotation_matrix = armature.matrix_world @ pose_bone.matrix

        q_target = pose_bone.rotation_quaternion
        bone.mvn_tpos_rot = rotation_matrix.to_quaternion().cross(q_target.inverted())

    except Exception as e:
        print(f"Failed to set bone properties for {bone_name}: {e}")
        bpy.ops.logging.logger(message_type="WARNING", message_text=f"Failed to set bone properties for {bone_name}")

    return delta_quaternion


def create_target_skeleton(mocap_pose: MocapPose, previous_session_target: str) -> bpy.types.Object:
    """
    Creates a target skeleton based on the mocap data.

    Uses the mocap_pose object, which contains the hierarchy of the skeleton and the props.
    First checks if the armature already exists, if so, it deletes the existing armature and creates a new one.
    Then iterates over the hierarchy and creates bones for each entry in the hierarchy.
    For each bone, it sets the head, tail, and roll properties based on the mocap data.
    If the bone is a prop, it adjusts the location and rotation of the corresponding empty object in the scene.
    After creating all the bones, it sets the scale of the armature to match the scene scale.

    Returns the created armature object.
    """
    hierarchy = segment_maps.stream_hierarchy_map
    created_props = []

    # --- Add Props to Hierarchy ---
    prop_list: list[list[str, str]] = []
    for key, value in mocap_pose.props.items():
        if value is not None:
            prop_list.append([key, value])
    # --- for each prop in the prop list determine its parent segment based on its name ---
    # prop_list example: [['Prop1', 'RightHandSword'], ['Prop2', 'LeftFootGeneric']]
    if prop_list:
        new_hierarchy = hierarchy.copy()

        insertion_shift = 0

        for prop_segment_name, prop_name in prop_list:
            for i, (bone_name, parent_name) in enumerate(hierarchy):
                if bone_name in prop_name:
                    insert_index = i + 1 + insertion_shift
                    new_hierarchy.insert(insert_index, (prop_segment_name, bone_name))
                    insertion_shift += 1

        if new_hierarchy:
            hierarchy = new_hierarchy
            mocap_pose.build_bone_hierarchy(segment_map=new_hierarchy)

    armature_name = mocap_pose.actor_name
    # --- Check if armature already exists ---
    if armature_name in bpy.data.objects:
        armature = bpy.data.objects[armature_name]
        if armature is not None and armature_name in bpy.data.objects and armature_name in bpy.context.scene.objects:
            try:
                bpy.ops.object.mode_set(mode="OBJECT")
            except Exception as e:
                if armature is not None and armature_name in bpy.data.objects and armature_name in bpy.context.scene.objects:
                    bpy.context.view_layer.objects.active = armature
                    bpy.ops.object.mode_set(mode="OBJECT")
                else:
                    message = f"Failed to delete armature {armature_name}: {e}"
                    bpy.ops.logging.logger(message_type="WARNING", message_text=message)
                    return
            bpy.ops.object.select_all(action="DESELECT")
            armature.select_set(True)
            bpy.context.view_layer.objects.active = armature
            bpy.ops.object.delete()
            delete_target_skeleton(mocap_pose)

    # --- Create armature ---
    bpy.ops.object.armature_add()
    armature = bpy.context.object

    # --- Reset armature transform if the cursor is not at the origin ---
    armature.location = (0, 0, 0)
    armature.rotation_quaternion = (1, 0, 0, 0)
    armature.scale = (1, 1, 1)

    armature.name = armature_name
    armature.data.name = armature_name
    add_to_collection(armature)

    # --- Lock the armature's transforms ---
    armature.lock_location = (True, True, True)
    armature.lock_rotation = (True, True, True)
    armature.lock_scale = (True, True, True)

    # --- Set custom identifier for MVN source armatures ---
    armature.data.mvn_is_source = True

    # --- Remove default bone ---
    bpy.ops.object.mode_set(mode="EDIT")
    if "Bone" in armature.data.edit_bones:
        armature.data.edit_bones.remove(armature.data.edit_bones["Bone"])

    for bone_name, parent_name in hierarchy:
        if not parent_name:
            # --- Handle Pelvis ---
            if bone_name in mocap_pose.scale_transform_dict:
                head_position = mocap_pose.scale_transform_dict[bone_name]
                tail_position = head_position + Vector((0, 0, .05))
                roll = mocap_pose.bone_orientations[bone_name]

                head_position, tail_position = compensate_for_head_tail_discrepancy(head_position, tail_position)

                bone = mocap_pose.bone_hierarchy[bone_name]
                bone.delta_quaternion = create_bone(
                    armature, bone_name, parent_name, head_position, tail_position, roll
                )

    # --- Add dummy spine bones for chain retargeting ---
    try:
        head_position = mocap_pose.scale_transform_dict["Pelvis"]

    except Exception:
        # --- If pelvis is not in the transform dict, assume it is an object cluster actor ---
        mocap_pose.is_object_cluster = True
        bpy.ops.object.mode_set(mode="OBJECT")
        return

    roll = mocap_pose.bone_orientations["Pelvis"]

    for bone_name, parent_name in hierarchy:

        if parent_name and bone_name in mocap_pose.scale_transform_dict or "Prop" in bone_name:

            bone = mocap_pose.bone_hierarchy[bone_name]
            parent_name = bone.parent.name if bone.parent else "None"
            children_names = [child.name for child in bone.children]

            try:
                head_position = mocap_pose.scale_transform_dict[bone_name]
            except Exception:
                if "Prop" in bone_name:
                    pass
                else:
                    message = f"Bone '{bone_name}' not found in scale transform dict"
                    bpy.ops.logging.logger(message_type="WARNING", message_text=message)

                # # --- Handle prop bone creation ---
                bone = mocap_pose.bone_hierarchy[bone_name]
                parent_name = bone.parent.name if bone.parent else "None"
                # must use the prop name to get the correct position from the scale transform dict
                found_prop = False
                for key in mocap_pose.scale_transform_dict:

                    for expected_prop_name in segment_maps.expected_prop_names:
                        if key.endswith(expected_prop_name) and key not in created_props:
                            created_props.append(key)
                            # props are in order of appearance in the bone scale transform dict, so the first prop will be prop1, the second prop2, etc.
                            prop_head_position = mocap_pose.scale_transform_dict[key]
                            tail_position = prop_head_position + Vector((0, 0, .1))

                            prop_head_position, tail_position = compensate_for_head_tail_discrepancy(
                                prop_head_position, tail_position
                            )
                            bone.delta_quaternion = create_bone(
                                armature, bone_name, parent_name, prop_head_position, tail_position, roll
                            )
                            found_prop = True
                    if found_prop:
                        break

            if bone_name == "RightHand":

                tail_position = head_position + Vector((-.05, 0, 0))
                head_position, tail_position = compensate_for_head_tail_discrepancy(head_position, tail_position)
                bone.delta_quaternion = create_bone(
                    armature, bone_name, parent_name, head_position, tail_position, roll
                )

            elif bone_name == "LeftHand":

                tail_position = head_position + Vector((.05, 0, 0))
                head_position, tail_position = compensate_for_head_tail_discrepancy(head_position, tail_position)
                bone.delta_quaternion = create_bone(
                    armature, bone_name, parent_name, head_position, tail_position, roll
                )

            else:

                try:

                    tail_position = mocap_pose.scale_transform_dict[
                        children_names[0]
                    ]  # tail position = head of child bone

                except:

                    if bone_name == "Head":
                        tail_position = head_position + Vector((0, 0, .05))

                    elif bone_name == "RightToe":
                        tail_position = head_position + Vector((0, -.05, 0))

                    elif bone_name == "LeftToe":
                        tail_position = head_position + Vector((0, -.05, 0))

                    elif bone_name in [
                        "RightFirstDP",
                        "RightSecondDP",
                        "RightThirdDP",
                        "RightFourthDP",
                        "RightFifthDP",
                    ]:
                        tail_position = head_position + Vector((-.02, 0, 0))

                    elif bone_name in ["LeftFirstDP", "LeftSecondDP", "LeftThirdDP", "LeftFourthDP", "LeftFifthDP"]:
                        tail_position = head_position + Vector((.02, 0, 0))

                    else:
                        tail_position = head_position + Vector((0, .05, 0))

                head_position, tail_position = compensate_for_head_tail_discrepancy(head_position, tail_position)

                roll = mocap_pose.bone_orientations[bone_name]

                # --- Get the Quaternion between the edit bone and vertical axis ---
                if not "Prop" in bone_name:
                    bone.delta_quaternion = create_bone(
                        armature, bone_name, parent_name, head_position, tail_position, roll
                    )

    mocap_pose.armature_object = armature
    bpy.ops.object.mode_set(mode="OBJECT")

    # --- Set scale according to user's scale entry ---
    scene = bpy.context.scene
    new_scale = Vector((scene.mvn_scene_scale, scene.mvn_scene_scale, scene.mvn_scene_scale))
    armature.scale = new_scale
    if previous_session_target:
        set_target_armature(previous_session_target, mocap_pose.actor_name)


def set_target_armature(target_armature_name, source_armature_name):
    scene = bpy.context.scene
    target_armature = bpy.data.objects.get(target_armature_name)
    source_armature = bpy.data.objects.get(source_armature_name)

    if target_armature and source_armature:
        scene.target_armature = target_armature
        scene.source_armature = source_armature
    else:
        scene.target_armature = None
        scene.source_armature = None


def compensate_for_head_tail_discrepancy(head_position, tail_position) -> tuple:
    """
    Compensates for minor discrepancies between the head and tail positions of a bone.

    Takes in the head and tail positions (as Vector objects). If the difference in the x, y, or z
    coordinates between the head and tail is less than 1, the function sets the tail's coordinate to match the head's.

    Returns the adjusted head and tail positions as a tuple.
    """

    threshold = 0.01

    if abs(head_position[0] - tail_position[0]) < threshold:
        tail_position[0] = head_position[0]
    if abs(head_position[1] - tail_position[1]) < threshold:
        tail_position[1] = head_position[1]
    if abs(head_position[2] - tail_position[2]) < threshold:
        tail_position[2] = head_position[2]

    return head_position, tail_position


def create_hand_bones(armature, hand_root: str, mocap_pose: MocapPose) -> MocapPose:
    """
    Creates the hand bones in the given armature based on the mocap data.

    Takes in the armature object, the root of the hand (either 'Right' or 'Left'), and the mocap_pose object.
    First determines which hand to create the bones for based on the hand_root.
    Then iterates over the hierarchy of the hand and creates bones for each entry in the hierarchy.
    For each bone, it sets the head, tail, and roll properties based on the mocap data.
    The function returns the updated mocap_pose object.
    """
    if "Right" in hand_root:
        hand = "Right"

    elif "Left" in hand_root:
        hand = "Left"

    else:
        return mocap_pose

    finger_joint_offsets = segment_maps.finger_joint_offsets

    for bone_name, parent_name in segment_maps.stream_hierarchy_map:

        if bone_name in finger_joint_offsets and hand in bone_name:

            if bone_name in ["RightCarpus", "LeftCarpus"]:
                continue

            bone = mocap_pose.bone_hierarchy[bone_name]

            # --- Add the bone to the scale transform dictionary ---
            bone_offset_from_parent = finger_joint_offsets[bone_name]
            parent_name = bone.parent.name if bone.parent else "None"
            parent_position = mocap_pose.glove_transforms[parent_name]
            head_position = parent_position + bone_offset_from_parent
            tail_position = head_position + Vector((0, 1, 0))

            # --- Add position data to mocap pose ---
            mocap_pose.glove_transforms[bone_name] = head_position
            create_bone(armature, bone_name, parent_name, head_position, tail_position)

    return mocap_pose


def create_tracker_empties(mocap_pose: MocapPose):
    """
    Creates empty objects in the scene for each tracker in the mocap data.

    The function uses the mocap_pose object, which contains the scale transform dictionary.
    It iterates over the scale transform dictionary and creates an empty object for each tracker.
    The name of the empty object is a combination of the actor's name and the tracker's name.

    If the empty object already exists in the scene, the function deletes the existing empty object and creates a new one.
    The function also adds the created empty object to the mocap_pose's vive_objects list.

    The function sets the create_object_cluster attribute of the mocap_pose to False.
    """
    if mocap_pose.scale_transform_dict:

        for tracker_name in mocap_pose.scale_transform_dict:

            empty_name = f"{mocap_pose.actor_name}_{tracker_name}"

            if empty_name not in bpy.data.objects:
                mocap_pose.vive_trackers.append(tracker_name)
                bpy.ops.object.empty_add(type="ARROWS", align="WORLD", location=(0, 0, 0))
                empty = bpy.context.active_object
                empty.name = f"{mocap_pose.actor_name}_{tracker_name}"
                empty.scale = (.05, .05, .05)
                add_to_collection(empty)
                mocap_pose.vive_objects.append(tracker_name)

            else:
                objects = get_vive_objects_from_pose(mocap_pose, clear_data=True)
                delete_tracker_empties(objects)

    mocap_pose.create_object_cluster = False


def delete_target_skeleton(mocap_pose):
    """
    Deletes the target skeleton from the scene.

    The function takes in the mocap_pose object, which contains the armature object.
    If the armature object exists, the function deletes it from the scene.
    The function also deletes any empty objects associated with the armature.

    If an error occurs while deleting the armature or the empty objects, a warning message is logged.
    """
    target_armature = bpy.data.objects.get(mocap_pose.actor_name)

    if target_armature:
        bpy.data.armatures.remove(bpy.data.armatures[target_armature.name])

    mocap_pose.vive_objects.clear()
    mocap_pose.vive_trackers.clear()
    mocap_pose.vive_headsets.clear()
    mocap_pose.is_object_cluster = False
    mocap_pose.create_object_cluster = False


def get_vive_objects_from_scene(mocap_pose_dict) -> list:
    object_pose = next((pose for pose in mocap_pose_dict.values() if pose.vive_objects), None)
    if object_pose:
        scene_objects = [f"{object_pose.actor_name}_{obj}" for obj in object_pose.vive_objects]
        return scene_objects


def get_vive_objects_from_pose(mocap_pose, clear_data=False) -> list:
    vive_objects = [f"{mocap_pose.actor_name}_{obj}" for obj in mocap_pose.vive_objects]
    if clear_data:
        mocap_pose.vive_trackers.clear()
        mocap_pose.vive_objects.clear()
        mocap_pose.vive_headsets.clear()
    return vive_objects


def delete_tracker_empties(objects):
    """
    Deletes the empty objects associated with each tracker in the mocap data.

    The function takes in a list of objects. It iterates over this list and for each object,
    it checks if it exists in the scene. If it does, the function schedules its deletion to be executed later.

    The function uses a deferred deletion approach, where the deletion of objects is scheduled to be executed later.
    This is done to prevent potential crashes due to rapid deletion and creation of objects.
    """

    def deferred_delete(objects):
        for object_name in objects:
            if object_name in bpy.data.objects:
                obj = bpy.data.objects[object_name]
                try:
                    bpy.data.objects.remove(obj, do_unlink=True)
                    while object_name in bpy.data.objects:
                        pass
                except Exception as e:
                    message = f"Failed to remove {object_name}: {e}"
                    bpy.ops.logging.logger(message_type="WARNING", message_text=message)

    bpy.app.timers.register(lambda: deferred_delete(objects), first_interval=1.0)


def delete_prop_empties(mocap_pose: MocapPose):
    """
    Deletes the empty objects associated with each prop in the mocap data.

    The function takes in the mocap_pose object, which contains the list of props.
    It iterates over the props list and for each prop, it constructs the name of the associated empty object.
    If the empty object exists in the scene, the function deletes it.

    If an error occurs while deleting the empty objects, a warning message is logged.
    """
    for prop_name in mocap_pose.props:

        prop_name = f"{mocap_pose.actor_name}_{prop_name}"

        if prop_name in bpy.data.objects:
            obj = bpy.data.objects[prop_name]

            try:
                bpy.data.objects.remove(obj, do_unlink=True)
            except Exception as e:
                message = f"Failed to remove {prop_name}: {e}"
                bpy.ops.logging.logger(message_type="WARNING", message_text=message)


def remove_previous_sources(mocap_pose_dict) -> bool:
    """
    Removes armatures, props, and trackers from the previous stream session.

    The function takes in the mocap_pose_dict, which contains the mocap_pose objects for each actor.
    It iterates over the mocap_pose_dict and for each mocap_pose, it checks if the armature, vive_objects (trackers), and props exist.
    If they exist, the function deletes them from the scene.

    The function also deletes all objects in the 'MVN Collection' that start with 'MVN'.

    The function clears the mocap_pose_dict and returns False.

    If an error occurs while deleting the armature, trackers, props, or objects in the 'MVN Collection', an error message is logged.
    """
    for mocap_pose in mocap_pose_dict.values():
        armature = mocap_pose.armature_object

        if mocap_pose.vive_objects:
            vive_objects = get_vive_objects_from_pose(mocap_pose, clear_data=True)
            delete_tracker_empties(vive_objects)

        if mocap_pose.props:
            delete_prop_empties(mocap_pose)

    # delete all objects in the mvn collection
    try:
        for obj in bpy.data.collections["MVN Collection"].objects:
            if obj.name.startswith("MVN"):
                armature_data_name = obj.data.name
                bpy.data.objects.remove(obj, do_unlink=True)
                if armature_data_name in bpy.data.armatures:
                    bpy.data.armatures.remove(bpy.data.armatures[armature_data_name])
    except Exception as e:
        pass

    mocap_pose_dict.clear()
    return False


# *********************************************************************************************************************
# Execute
if __name__ == "__main__":
    pass
