# *********************************************************************************************************************
# source_animator.py : Apply the mocap data stream to the armature and tracker empties
# *********************************************************************************************************************
import bpy
from mathutils import Vector, Quaternion

from . import recorder, source_manager
from .pose import MocapPose, Bone
from .utility import add_to_collection


# ============================================================================================
# Functions: Armature
def apply_transforms_to_armature(mocap_pose: MocapPose) -> bpy.types.Object:
    """
    Applies motion capture data to a 3D armature in Blender.
    """
    if not mocap_pose.scale_transform_dict:
        return
    if armature := bpy.data.objects.get(mocap_pose.actor_name):
        armature_name = mocap_pose.actor_name
    else:
        return

    for bone_name, transform_data in mocap_pose.skeleton_transform_dict.items():
        if mocap_pose.props.get(bone_name) is not None:
            apply_transform_to_prop_empty(mocap_pose, bone_name)

        if not armature_name:
            continue

        if bone_name in bpy.data.objects[armature_name].pose.bones:
            bone = bpy.data.objects[armature_name].pose.bones[bone_name]

            # --- Transform Root ---
            if not bone.parent and bone.name == "Pelvis":
                apply_vector_to_pelvis(mocap_pose, bone, armature)
                recorder.add_record(armature, bone_name, bone.rotation_quaternion.copy(), bone.location.copy())

            # --- Apply Quaternion to all children ---
            elif bone.parent and bone.name != "Pelvis":
                apply_quaterion_to_children(mocap_pose, bone, armature)
                recorder.add_record(armature, bone_name, bone.rotation_quaternion.copy())


def apply_vector_to_pelvis(mocap_pose: MocapPose, pelvis_bone: Bone, armature: bpy.types.Object) -> bpy.types.Object:
    """
    Adjusts the position and rotation of the pelvis bone based on the incoming MVN datastream.

    The position vector and rotation quaternion for the pelvis bone are extracted and adjusted
    to match the Blender's coordinate system. These adjusted transformations are then applied to
    the pelvis bone in the armature.
    """
    # --- Convert the Vector ---
    blender_bone = armature.pose.bones[pelvis_bone.name]
    live_vector = Vector(mocap_pose.skeleton_transform_dict[pelvis_bone.name][0])
    scale_pose_transform_z = mocap_pose.scale_transform_dict["Pelvis"].z
    adjusted_vector = live_vector
    adjusted_vector.y -= scale_pose_transform_z

    blender_bone.location = adjusted_vector

    # --- Apply Rotation ---
    live_quaternion = mocap_pose.skeleton_transform_dict[pelvis_bone.name][1]
    mocap_bone = mocap_pose.bone_hierarchy["Pelvis"]
    blender_bone.rotation_quaternion = calculate_rotation(mocap_bone, live_quaternion)

    try:
        from . import file_logger
        if file_logger.is_open():
            q = blender_bone.rotation_quaternion
            loc = blender_bone.location
            file_logger.log(
                f"[bone_world] bone=Pelvis loc={loc.x:.6f},{loc.y:.6f},{loc.z:.6f} "
                f"quat={q.w:.6f},{q.x:.6f},{q.y:.6f},{q.z:.6f}"
            )
    except Exception:
        pass


def apply_quaterion_to_children(mocap_pose: MocapPose, bone: Bone, armature: bpy.types.Object) -> bpy.types.Object:
    """
    Applies the rotation from mocap data to the child bones in the armature.
    """
    parent_global_quaternion = mocap_pose.skeleton_transform_dict[bone.parent.name][1]
    target_bone_global_quaternion = mocap_pose.skeleton_transform_dict[bone.name][1]

    parent_inverse_quaternion = parent_global_quaternion.inverted()

    target_bone_local_quaternion = parent_inverse_quaternion @ target_bone_global_quaternion

    # --- Correct rotation based on edit transforms ---
    for mocap_bone_name, mocap_bone in mocap_pose.bone_hierarchy.items():
        if mocap_bone_name.lower() == bone.name.lower():
            mocap_bone = mocap_pose.bone_hierarchy[mocap_bone_name]
            target_bone_local_quaternion = calculate_rotation(mocap_bone, target_bone_local_quaternion)
            break

    # --- apply the local quaternion to the child bone ---
    blender_bone = bpy.data.objects[armature.name].pose.bones[bone.name]
    blender_bone.rotation_mode = "QUATERNION"
    blender_bone.rotation_quaternion = target_bone_local_quaternion

    try:
        from . import file_logger
        if file_logger.is_open():
            q = target_bone_local_quaternion
            file_logger.log(
                f"[bone_local] bone={bone.name} "
                f"quat={q.w:.6f},{q.x:.6f},{q.y:.6f},{q.z:.6f}"
            )
    except Exception:
        pass


def apply_transform_to_prop_empty(mocap_pose: MocapPose, bone_name: str):
    """
    Applies the position and rotation from mocap data to the corresponding prop empty in the scene.
    """
    target_empty_name = f"{mocap_pose.actor_name}_{bone_name}"
    # --- create prop empty if it does not exist in the scene ---
    if target_empty_name not in bpy.data.objects:
        bpy.ops.object.empty_add(type="ARROWS", align="WORLD", location=(0, 0, 0))
        empty = bpy.context.active_object
        empty.name = target_empty_name
        empty.scale = (.05, .05, .05)
        add_to_collection(empty)

    if target_empty_name in bpy.data.objects:
        obj = bpy.data.objects[target_empty_name]

        live_vector = mocap_pose.skeleton_transform_dict[bone_name][0]
        adjusted_vector = live_vector.copy()

        original_x, original_y, original_z = adjusted_vector.x, adjusted_vector.y, adjusted_vector.z
        adjusted_vector.y = -original_z
        adjusted_vector.z = original_y

        obj.location = adjusted_vector

        obj.rotation_mode = "QUATERNION"
        source_quaternion = mocap_pose.skeleton_transform_dict[bone_name][1]
        converted_quaternion = Quaternion(
            (source_quaternion[0], source_quaternion[2], -source_quaternion[1], source_quaternion[3])
        )
        obj.rotation_quaternion = converted_quaternion

        recorder.add_record(obj, "", obj.rotation_quaternion.copy(), obj.location.copy())

    else:
        message = f"Prop empty '{bone_name}' not found in the scene"
        bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="WARNING", message_text=message)


# --------------------------------------------------------------------------------------------------------------------------------------------
# Functions: Object Tracking
def apply_transforms_to_trackers(mocap_pose: MocapPose):
    """
    Applies the position and rotation from mocap data to the corresponding tracker empties in the scene.
    """
    vive_objects = mocap_pose.vive_objects
    for index, vive_object in enumerate(vive_objects):
        target_empty = f"{mocap_pose.actor_name}_{vive_object}"
        if target_empty in bpy.data.objects:
            obj = bpy.data.objects[target_empty]
            try:
                live_vector = list(mocap_pose.skeleton_transform_dict.values())[index][0]
                adjusted_vector = live_vector.copy()

                original_x, original_y, original_z = adjusted_vector.x, adjusted_vector.y, adjusted_vector.z
                adjusted_vector.y = -original_z
                adjusted_vector.z = original_y

                obj.location = adjusted_vector

                obj.rotation_mode = "QUATERNION"
                obj.rotation_quaternion = list(mocap_pose.skeleton_transform_dict.values())[index][1]

                recorder.add_record(obj, "", obj.rotation_quaternion.copy(), obj.location.copy())

            except IndexError:
                message = f"Index {index} not found in skeleton transform dict"
                bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="WARNING", message_text=message)
        else:
            vive_objects = source_manager.get_vive_objects_from_pose(mocap_pose, clear_data=True)
            source_manager.delete_tracker_empties(vive_objects)
            try:
                mocap_pose.vive_objects.remove(vive_object)
            except ValueError as e:
                print(f"Error while removing object data from pose: {e}")
            mocap_pose.create_object_cluster = True
    return None  # important to return None to prevent re-scheduling the function


def calculate_rotation(mocap_bone, t_quaternion):
    """
    Calculates the rotation quaternion for a given bone based on mocap data.

    Adjusts the target quaternion to match the Blender's coordinate system based on the
    bone's name. The adjusted quaternion is then returned.
    """
    try:
        from . import file_logger
        _log_open = file_logger.is_open()
    except Exception:
        _log_open = False

    def _emit(rule_name: str, q_out):
        if not _log_open:
            return
        try:
            from . import file_logger
            file_logger.log(
                f"[rule] bone={mocap_bone.name} rule={rule_name} "
                f"in={t_quaternion[0]:.6f},{t_quaternion[1]:.6f},"
                f"{t_quaternion[2]:.6f},{t_quaternion[3]:.6f} "
                f"out={q_out[0]:.6f},{q_out[1]:.6f},{q_out[2]:.6f},{q_out[3]:.6f}"
            )
        except Exception:
            pass

    if not mocap_bone.delta_quaternion:
        new_quaternion = t_quaternion
        _emit("identity-no-delta", new_quaternion)
        return new_quaternion
    # Global +Z
    if mocap_bone.name in ["Pelvis", "L5", "L3", "T12", "T8", "Neck", "Head"]:
        new_quaternion = Quaternion((t_quaternion[0], t_quaternion[2], t_quaternion[3], t_quaternion[1]))
        _emit("spine_wxyz->w,y,z,x", new_quaternion)
    # Global -Z
    elif mocap_bone.name in ["RightUpperLeg", "RightLowerLeg", "LeftUpperLeg", "LeftLowerLeg"]:
        new_quaternion = Quaternion((t_quaternion[0], t_quaternion[2], -t_quaternion[3], -t_quaternion[1]))
        _emit("leg_wxyz->w,y,-z,-x", new_quaternion)
    # Global +X
    elif mocap_bone.name in ["LeftShoulder", "LeftUpperArm", "LeftForeArm", "LeftHand"]:
        new_quaternion = Quaternion((t_quaternion[0], t_quaternion[1], t_quaternion[2], t_quaternion[3]))
        _emit("larm_identity", new_quaternion)
    # Global -X
    elif mocap_bone.name in ["RightShoulder", "RightUpperArm", "RightForeArm", "RightHand"]:
        new_quaternion = Quaternion((t_quaternion[0], -t_quaternion[1], -t_quaternion[2], t_quaternion[3]))
        _emit("rarm_wxyz->w,-x,-y,z", new_quaternion)
    # Hand bones
    elif mocap_bone.name in [
        "LeftCarpus",
        "LeftFirstMC",
        "LeftFirstPP",
        "LeftFirstDP",
        "LeftSecondMC",
        "LeftSecondPP",
        "LeftSecondMP",
        "LeftSecondDP",
        "LeftThirdMC",
        "LeftThirdPP",
        "LeftThirdMP",
        "LeftThirdDP",
        "LeftFourthMC",
        "LeftFourthPP",
        "LeftFourthMP",
        "LeftFourthDP",
        "LeftFifthMC",
        "LeftFifthPP",
        "LeftFifthMP",
        "LeftFifthDP",
    ]:
        new_quaternion = t_quaternion
        _emit("lfingers_identity", new_quaternion)
        return new_quaternion
    elif mocap_bone.name in [
        "RightCarpus",
        "RightFirstMC",
        "RightFirstPP",
        "RightFirstDP",
        "RightSecondMC",
        "RightSecondPP",
        "RightSecondMP",
        "RightSecondDP",
        "RightThirdMC",
        "RightThirdPP",
        "RightThirdMP",
        "RightThirdDP",
        "RightFourthMC",
        "RightFourthPP",
        "RightFourthMP",
        "RightFourthDP",
        "RightFifthMC",
        "RightFifthPP",
        "RightFifthMP",
        "RightFifthDP",
    ]:
        new_quaternion = Quaternion((t_quaternion[0], -t_quaternion[1], -t_quaternion[2], t_quaternion[3]))
        _emit("rfingers_wxyz->w,-x,-y,z", new_quaternion)
        return new_quaternion
    else:
        # Feet and toe bones
        converted_quaternion = Quaternion((t_quaternion[0], t_quaternion[2], -t_quaternion[3], -t_quaternion[1]))
        new_quaternion = mocap_bone.delta_quaternion.inverted() @ converted_quaternion @ mocap_bone.delta_quaternion
        _emit("foot/toe_similarity-delta", new_quaternion)

    return new_quaternion


# *********************************************************************************************************************
# Main
if __name__ == "__main__":
    pass
