# **********************************************************************************************************************
# Rigging.py: Automates the setup of IK constraints for Blender armatures
#
# Documentation and References:
# - MVN IK Constraint Layout Documentation:
#   - https://docs.google.com/spreadsheets/d/1W7plXUF_qHaKyaYgfVS0zwpEAu5RUtZFgeGWzyim6c4/edit?usp=sharing

# - Blender Constraint Documentation:
#   - https://docs.blender.org/api/current/bpy.types.Constraint.html
#   - https://docs.blender.org/api/current/bpy_types_enum_items/constraint_type_items.html#rna-enum-constraint-type-items
#
# **********************************************************************************************************************
import bpy
from mathutils import Vector, Quaternion

from . import segment_maps
from .utility import switch_mode, deselect, add_to_collection, MVN_COLLECTION

# =============================================================================================
# Constants
SEGMENT_MAP: dict[int, str] = segment_maps.segment_map
IK_BONE_SUFFIX = "-MvnIK"


def apply_tpose(self=None, object=None):
    """
    Applies the T-Pose to the target armature. Checks if the 'mvn_tposemode' flag is set in the Blender scene
    context. If it is, it clears all bone maps. Otherwise, it retrieves the world position and rotation of each
    bone in the target armature and stores them in the 'mvn_tpos' and 'mvn_tpos_rot' properties of the bone,
    respectively. It then calls the 'resolve_ik_armature' function to update the IK armature.
    """
    scene = bpy.context.scene
    if scene.target_armature:
        # --- Enter edit t-pose mode ---
        if scene.target_armature.data.mvn_tposemode:
            clear_all_bonemaps()
        # --- Apply t-pose ---
        else:
            bpy.context.view_layer.update()
            # --- Target locations ---
            world_matrix = scene.target_armature.matrix_world
            for pose_bone in scene.target_armature.pose.bones:
                world_position = world_matrix @ pose_bone.matrix @ pose_bone.location
                scene.target_armature.data.bones[pose_bone.name].mvn_tpos = world_position
            # --- Target rotations ---
            for bone in scene.target_armature.data.bones:
                target_bone = scene.target_armature.pose.bones[bone.name]
                rotation_matrix = scene.target_armature.matrix_world @ target_bone.matrix
                # if bone.mvn_source_bone_name == SEGMENT_MAP[1]:
                q_target = target_bone.rotation_quaternion
                bone.mvn_tpos_rot = rotation_matrix.to_quaternion().cross(q_target.inverted())
                # else:
                # bone.mvn_tpos_rot = rotation_matrix.to_quaternion()

            # if not a streamed character then tpose values must be set for source armature
            if not scene.source_armature.data.mvn_is_source:
                # --- Source locations ---
                world_matrix = scene.source_armature.matrix_world
                for pose_bone in scene.source_armature.pose.bones:
                    world_position = world_matrix @ pose_bone.matrix @ pose_bone.location
                    scene.source_armature.data.bones[pose_bone.name].mvn_tpos = world_position
                # --- Source Rotations ---
                for bone in scene.source_armature.data.bones:
                    target_bone = scene.source_armature.pose.bones[bone.name]
                    rotation_matrix = scene.source_armature.matrix_world @ target_bone.matrix
                    q_target = target_bone.rotation_quaternion
                    bone.mvn_tpos_rot = rotation_matrix.to_quaternion().cross(q_target.inverted())

            resolve_ik_armature()
            bpy.context.view_layer.update()


def clear_all_bonemaps():
    """
    Clears all bone mappings in the Blender scene context. Removes all constraints from the target armature.
    """
    scene = bpy.context.scene
    for pose_bone in scene.target_armature.pose.bones:
        remove_mvn_constraints(pose_bone, "MVN_")


def update_bonemap(self, object):
    """
    Checks if the 'mvn_auto_update_constraints' flag is set in the Blender scene context. If it is, it calls the
    'resolve_ik_armature' function to update the IK armature. This function is typically used when a change is
    made to the bone mapping in the user interface, and the IK armature needs to be updated
    to reflect this change.
    """
    scene = bpy.context.scene
    if scene.mvn_auto_update_constraints:
        resolve_ik_armature()


def update_all_bonemaps():
    """
    Updates all bone maps based on the current target and source armatures. If only a target armature is set,
    it clears all bone maps. If both a target and source armature are set, it updates the bone maps and IK
    constraints to match the current source armature.
    """

    scene = bpy.context.scene
    if scene.target_armature and scene.source_armature:
        if not scene.target_armature.data.mvn_tposemode:
            prev_view = scene.target_armature.hide_get()
            switch_mode("OBJECT", scene.target_armature)
            switch_mode("POSE", scene.target_armature)
            target_spine = []
            spine_start = None
            spine_end = None

            # --- Check for and clear non-existent mappings ---
            for target_bone in scene.target_armature.data.bones:
                pose_bone = scene.target_armature.pose.bones.get(target_bone.name)
                if (
                    target_bone.mvn_source_bone_name != ""
                    and target_bone.mvn_source_bone_name != "-"
                    and "Spine" not in target_bone.mvn_source_bone_name
                ):
                    if target_bone.mvn_source_bone_name not in scene.source_armature.data.bones:
                        target_bone.mvn_source_bone_name = ""
            """
            # --- Map Spine ---
            for target_bone in scene.target_armature.data.bones:
                pose_bone = scene.target_armature.pose.bones.get(target_bone.name)
                if "Spine" in target_bone.mvn_source_bone_name:
                    target_spine.append(pose_bone)
                    if "Start" in target_bone.mvn_source_bone_name or "Bottom" in target_bone.mvn_source_bone_name:
                        spine_start = pose_bone
                    if "End" in target_bone.mvn_source_bone_name or "Top" in target_bone.mvn_source_bone_name:
                        spine_end = pose_bone
                elif "-" in target_bone.mvn_source_bone_name:
                    target_bone.mvn_source_bone_name = ""

            # --- Create spine chain ---
            if spine_start and spine_end:
                scene.mvn_auto_update_constraints = False
                target_spine.clear()
                target_spine.append(spine_end)
                parent = spine_end.parent
                max_iterations = 15
                current_iteration = 0
                while parent != spine_start:
                    current_iteration += 1
                    if current_iteration > max_iterations:
                        target_spine.clear()
                        break
                    target_spine.append(parent)
                    scene.target_armature.data.bones[parent.name].mvn_source_bone_name = "-"
                    if parent.parent:
                        parent = parent.parent
                    else:
                        break
                target_spine.append(spine_start)
                target_spine.reverse()
                scene.mvn_auto_update_constraints = True
            """
            # --- Handle other constraints ---
            for target_bone in scene.target_armature.data.bones:
                pose_bone = scene.target_armature.pose.bones.get(target_bone.name)
                remove_mvn_constraints(pose_bone, "MVN_")
                if (
                    target_bone.mvn_source_bone_name != ""
                    and target_bone.mvn_source_bone_name != "-"
                    and "Spine" not in target_bone.mvn_source_bone_name
                ):
                    if pose_bone not in target_spine:
                        source_bone = scene.source_armature.data.bones[target_bone.mvn_source_bone_name]
                        target_bone.mvn_qdif = target_bone.mvn_tpos_rot.inverted() @ source_bone.mvn_tpos_rot

                        # --- Create FK Constraints ---
                        source_bone_name = target_bone.mvn_source_bone_name
                        # location
                        pelvis_names = ["hips", "pelvis"]
                        if source_bone_name == SEGMENT_MAP[1] or source_bone_name.lower() in pelvis_names:
                            target_scale = scene.target_armature.scale
                            multiplier_x, multiplier_y, multiplier_z = [1/scale if scale != 0 else 0 for scale in target_scale]

                            const = add_const(
                                pose_bone,
                                "TRANSFORM",
                                {
                                    "name": "MVN_TRANSFORM_LOC",
                                    "target": scene.source_armature,
                                    "subtarget": source_bone_name,
                                    "use_motion_extrapolate": False,
                                    "target_space": "LOCAL_OWNER_ORIENT",
                                    "owner_space": "LOCAL",
                                    "influence": 1.0,
                                    "mix_mode": "ADD",
                                },
                            )
                            add_driver(
                                const,
                                "to_min_x",
                                "SCRIPTED",
                                [
                                    [
                                        "v_source",
                                        "SINGLE_PROP",
                                        scene.source_armature,
                                        {"data_path": f'pose.bones["{source_bone_name}"].location'},
                                    ],
                                    [
                                        "q_dif",
                                        "SINGLE_PROP",
                                        scene.target_armature,
                                        {"data_path": f'pose.bones["{pose_bone.name}"].bone.mvn_qdif'},
                                    ],
                                ],
                                f"target_vector(v_source,q_dif)[0]*{multiplier_x}",
                            )
                            add_driver(
                                const,
                                "to_min_y",
                                "SCRIPTED",
                                [
                                    [
                                        "v_source",
                                        "SINGLE_PROP",
                                        scene.source_armature,
                                        {"data_path": f'pose.bones["{source_bone_name}"].location'},
                                    ],
                                    [
                                        "q_dif",
                                        "SINGLE_PROP",
                                        scene.target_armature,
                                        {"data_path": f'pose.bones["{pose_bone.name}"].bone.mvn_qdif'},
                                    ],
                                ],
                                f"target_vector(v_source,q_dif)[1]*{multiplier_y}",
                            )
                            add_driver(
                                const,
                                "to_min_z",
                                "SCRIPTED",
                                [
                                    [
                                        "v_source",
                                        "SINGLE_PROP",
                                        scene.source_armature,
                                        {"data_path": f'pose.bones["{source_bone_name}"].location'},
                                    ],
                                    [
                                        "q_dif",
                                        "SINGLE_PROP",
                                        scene.target_armature,
                                        {"data_path": f'pose.bones["{pose_bone.name}"].bone.mvn_qdif'},
                                    ],
                                ],
                                f"target_vector(v_source,q_dif)[2]*{multiplier_z}",
                            )
                        # rotation
                        add_mvn_constraint(pose_bone, "MVN_TRANSFORM_ROT", scene.source_armature, source_bone_name, 1.0)
            """
            # --- Spine Constraints ---
            # --- Spine Constraints ---
            for spine_index in range(len(target_spine)):
                spine_bone = target_spine[spine_index]
                remove_mvn_constraints(spine_bone, "MVN_")

                end_bone_count = 0  # How many end bones to match rotation of
                source_spine_names = [SEGMENT_MAP[2], SEGMENT_MAP[3], SEGMENT_MAP[4], SEGMENT_MAP[5]]
                max_influence = (len(source_spine_names) - end_bone_count) / (len(target_spine) - end_bone_count)
                occupied_influence = max_influence * spine_index
                temp_influence = 0

                # last spine bones should match
                if spine_index + end_bone_count == len(target_spine):
                    temp_influence = max_influence
                    source_spine_name = source_spine_names[-1]
                    add_mvn_constraint(spine_bone, "MVN_TRANSFORM_SPINE_" + source_spine_name, scene.source_armature, source_spine_name, 1.0)

                # for each constraint to make
                for source_spine_id in range(len(source_spine_names)):
                    source_spine_name = source_spine_names[source_spine_id]
                    if temp_influence >= max_influence:
                        break
                    current_occ = max(0, occupied_influence - source_spine_id)
                    value = min(1, min(1 - current_occ, max_influence - temp_influence))
                    if value > 0:
                        temp_influence += value
                        add_mvn_constraint(spine_bone, "MVN_TRANSFORM_SPINE_" + source_spine_name, scene.source_armature, source_spine_name, value)
            """
            # --- Return to object mode ---
            bpy.ops.object.mode_set(mode="OBJECT")
            deselect()
            scene.target_armature.hide_set(prev_view)

    elif scene.target_armature:
        clear_all_bonemaps()


def add_driver(constraint, prop, driver_type, variables, expression, index=-1):
    """
    Adds a driver to a constraint. The driver is configured with the given type, variables, and expression.
    The driver is added to the property specified by 'prop', and if 'index' is specified, it is used as the
    index for the property.
    """
    driver = constraint.driver_add(prop, index).driver
    driver.type = driver_type
    # Add variables to the driver
    # [name, type, obj, {name:prop}]
    for variable in variables:
        var = driver.variables.new()
        var.name = variable[0]
        var.type = variable[1]
        var.targets[0].id = variable[2]
        for name, value in variable[3].items():
            if hasattr(var.targets[0], name):
                setattr(var.targets[0], name, value)

    # --- Add the expression for the driver ---
    driver.use_self = False
    driver.expression = expression
    return driver


def target_euler(q_source, q_dif):
    try:
        if q_source is None or q_dif is None:
            return (0, 0, 0)  # Return a default value
        q_new = (q_dif.cross(q_source)).cross(q_dif.inverted())
        return q_new.to_euler()
    except Exception:
        return (0, 0, 0)  # Return a default value in case of any other exceptions

def target_vector(v_source, q_dif):
    try:
        if v_source is None or q_dif is None:
            return (0, 0, 0)  # Return a default value
        q_base = Quaternion((0, v_source[0], v_source[1], v_source[2]))
        q_new = (q_dif.cross(q_base)).cross(q_dif.inverted())
        return Vector((q_new[1], q_new[2], q_new[3]))
    except TypeError:
        return (0, 0, 0)  # Return a default value in case of any other exceptions

def add_mvn_constraint(pose_bone, name, target, subtarget, influence):
    """
    Adds a new MVN FK constraint to the target spine bone. The constraint is configured with the given name,
    target, subtarget, and influence. The constraint's rotation is driven by the source bone's rotation.
    """
    scene = bpy.context.scene
    const = add_const(
        pose_bone,
        "TRANSFORM",
        {
            "name": name,
            "target": target,
            "subtarget": subtarget,
            "use_motion_extrapolate": False,
            "target_space": "LOCAL_OWNER_ORIENT",
            "owner_space": "LOCAL",
            "map_from": "ROTATION",
            "map_to": "ROTATION",
            "influence": influence,
            "mix_mode_rot": "BEFORE",
        },
    )
    add_driver(
        const,
        "to_min_x_rot",
        "SCRIPTED",
        [
            [
                "q_source",
                "SINGLE_PROP",
                scene.source_armature,
                {"data_path": f'pose.bones["{subtarget}"].rotation_quaternion'},
            ],
            [
                "q_dif",
                "SINGLE_PROP",
                scene.target_armature,
                {"data_path": f'pose.bones["{pose_bone.name}"].bone.mvn_qdif'},
            ],
        ],
        f"target_euler(q_source,q_dif)[0]",
    )
    add_driver(
        const,
        "to_min_y_rot",
        "SCRIPTED",
        [
            [
                "q_source",
                "SINGLE_PROP",
                scene.source_armature,
                {"data_path": f'pose.bones["{subtarget}"].rotation_quaternion'},
            ],
            [
                "q_dif",
                "SINGLE_PROP",
                scene.target_armature,
                {"data_path": f'pose.bones["{pose_bone.name}"].bone.mvn_qdif'},
            ],
        ],
        f"target_euler(q_source,q_dif)[1]",
    )
    add_driver(
        const,
        "to_min_z_rot",
        "SCRIPTED",
        [
            [
                "q_source",
                "SINGLE_PROP",
                scene.source_armature,
                {"data_path": f'pose.bones["{subtarget}"].rotation_quaternion'},
            ],
            [
                "q_dif",
                "SINGLE_PROP",
                scene.target_armature,
                {"data_path": f'pose.bones["{pose_bone.name}"].bone.mvn_qdif'},
            ],
        ],
        f"target_euler(q_source,q_dif)[2]",
    )
    return const


def remove_mvn_constraints(pose_bone, substring):
    """
    Removes any constraint that contains the given substring in the name from the given pose bone. This is used
    to remove all MVN constraints from a bone.
    """
    for constraint in pose_bone.constraints:
        if substring in constraint.name:
            constraint.driver_remove("to_min_x_rot")
            constraint.driver_remove("to_min_y_rot")
            constraint.driver_remove("to_min_z_rot")
            constraint.driver_remove("to_min_x")
            constraint.driver_remove("to_min_y")
            constraint.driver_remove("to_min_z")
            pose_bone.constraints.remove(constraint)


def add_const(obj, type, props):
    """
    Adds a new constraint of the given type to the given object. The constraint is configured with the given
    properties.
    """
    constraint = obj.constraints.new(type=type)
    for property_name, value in props.items():
        if hasattr(constraint, property_name):
            setattr(constraint, property_name, value)
    return constraint


def mod_const(constraint, props):
    """
    Modifies the given constraint with the given properties. Each property in 'props' is set on the constraint
    if it exists.
    """
    for property_name, value in props.items():
        if hasattr(constraint, property_name):
            setattr(constraint, property_name, value)
    return constraint


def duplicate_bone(armature, new_bone_name, old_bone_name, parent=True) -> bpy.types.Bone:
    """
    Duplicates a bone in the given armature. The new bone is given the name 'new_bone_name', and its properties
    are copied from the bone named 'old_bone_name'. If 'parent' is True, the new bone's parent is set to the
    old bone's parent.
    """
    new_bone = armature.data.edit_bones.new(new_bone_name)
    new_bone.head = armature.data.edit_bones[old_bone_name].head
    new_bone.tail = armature.data.edit_bones[old_bone_name].tail
    new_bone.matrix = armature.data.edit_bones[old_bone_name].matrix
    new_bone.roll = armature.data.edit_bones[old_bone_name].roll
    if parent:
        new_bone.parent = armature.data.edit_bones[old_bone_name].parent
    return new_bone


def get_bone_map(target_armature, return_mode=True) -> dict:
    """
    Returns a dictionary mapping target bone names to source bone names for the given target armature. If
    'return_mode' is True, the function will return the armature to its previous mode before returning.
    """
    dict = {}
    prev_mode = switch_mode("OBJECT", target_armature)

    for target_bone in target_armature.data.bones:
        if target_bone.mvn_source_bone_name != "":
            dict[target_bone.name] = target_bone.mvn_source_bone_name

    if return_mode:
        if bpy.context.object.mode != prev_mode:
            bpy.ops.object.mode_set(mode=prev_mode)

    return dict  # {'target bone name':'source bone name'}


# ----------------------------------------------------------------------------------------------------------------------
# functions: IK Rig Setup
def check_ik_eligible(target_armature) -> list:
    """
    Checks if the given target armature is eligible for IK. An armature is eligible for IK if it has all the
    required bone maps. Returns a list where the first element is a boolean indicating eligibility, and the
    second element is a list of missing bone maps if not eligible.
    """
    required_bone_maps = [
        SEGMENT_MAP[1],  # Pelvis
        SEGMENT_MAP[16],  # RightUpperLeg
        SEGMENT_MAP[17],  # RightLowerLeg
        SEGMENT_MAP[18],  # RightFoot
        SEGMENT_MAP[19],  # RightToe
        SEGMENT_MAP[20],  # LeftUpperLeg
        SEGMENT_MAP[21],  # LeftLowerLeg
        SEGMENT_MAP[22],  # LeftFoot
        SEGMENT_MAP[23],  # LeftToe
    ]

    for bone in target_armature.data.bones:
        if bone.mvn_source_bone_name in required_bone_maps:
            required_bone_maps.remove(bone.mvn_source_bone_name)
    return [len(required_bone_maps) == 0, required_bone_maps]


def delete_ik_rig(armature):
    """
    Deletes the given IK rig. This function removes the armature data and the armature object from the Blender
    data.
    """
    try:
        if armature.type == "ARMATURE":
            # --- Delete the armature data ---
            bpy.data.armatures.remove(armature.data)

            # --- Delete the armature object ---
            bpy.data.objects.remove(armature, do_unlink=True)
    except Exception as e:
        print(f"Error: rigging.delete_ik_rig | {e}")


def resolve_ik_armature(self=None, object=None):
    """
    Creates or removes the proper IK armature with the currently set target/source armatures. Checks if the
    'mvn_tposemode' flag is set in the Blender scene context. If it is, it returns. If the 'mvn_ik_enabled' flag
    is set, it updates all bone maps, checks IK eligibility, and creates or updates the IK armature as needed.
    """
    scene = bpy.context.scene
    if scene.target_armature and scene.source_armature:
        if scene.target_armature.data.mvn_tposemode:
            return
        if scene.target_armature.data.mvn_ik_enabled:
            deselect()

            # --- set current visibility of armatures to return to later ---
            prev_view_target = scene.target_armature.hide_get()
            prev_view_target2 = scene.target_armature.hide_viewport
            prev_view_source = scene.source_armature.hide_get()
            prev_view_source2 = scene.source_armature.hide_viewport
            scene.target_armature.hide_set(False)
            scene.target_armature.hide_viewport = False
            scene.source_armature.hide_set(False)
            scene.source_armature.hide_viewport = False
            prev_view_collection = bpy.data.collections.get(MVN_COLLECTION).hide_viewport
            prev_view_collection2 = bpy.context.view_layer.layer_collection.children.get(MVN_COLLECTION).hide_viewport
            bpy.data.collections.get(MVN_COLLECTION).hide_viewport = False
            bpy.context.view_layer.layer_collection.children.get(MVN_COLLECTION).hide_viewport = False
            switch_mode("OBJECT", scene.target_armature)
            update_all_bonemaps()

            # --- Reset error message ---
            scene.target_armature.data.mvn_ik_error = ""

            # --- delete previous ik rig ---
            if scene.target_armature.data.mvn_ik_armature:
                delete_ik_rig(scene.target_armature.data.mvn_ik_armature)
            ik_eligibility = check_ik_eligible(scene.target_armature)
            if not ik_eligibility[0]:
                scene.target_armature.data.mvn_ik_error = (
                    "Target armature missing bone map for: " + ik_eligibility[1][0]
                )
                return

            # --- Create dictionaries ---
            target_bone_map: dict[str, str] = get_bone_map(
                scene.target_armature, False
            )  # {'target_bone.name' : target_bone.mvn_source_bone_name}
            source_bone_map: dict[str, str] = {}  # {'source_bone_name': target bone name}
            ik_dict: dict[str, tuple[Vector, Vector, float]] = {}  # {'source_bone_name':headV,tailV,roll}

            # --- Enter edit mode for target armature ---
            switch_mode("EDIT", scene.target_armature)

            # --- # populate ik dict with {'source_bone_name':headV,tailV,roll} ---
            for edit_bone in scene.target_armature.data.edit_bones:
                if edit_bone.name in target_bone_map:
                    source_bone_name = target_bone_map[edit_bone.name]
                    source_bone_map[source_bone_name] = edit_bone.name
                    ik_dict[source_bone_name] = [
                        Vector((edit_bone.head[0], edit_bone.head[1], edit_bone.head[2])),
                        Vector((edit_bone.tail[0], edit_bone.tail[1], edit_bone.tail[2])),
                        edit_bone.roll,
                    ]

            # --- Enter Object Mode ---
            bpy.ops.object.mode_set(mode="OBJECT")
            deselect()

            mvn_collection = bpy.data.collections.get(MVN_COLLECTION)
            mvn_collection.hide_viewport = False

            # --- Duplicate Source Armatures ---
            scene.source_armature.select_set(True)
            bpy.context.view_layer.objects.active = scene.source_armature
            bpy.ops.object.duplicate(linked=False)
            scene.source_armature.hide_set(prev_view_source)
            scene.source_armature.hide_viewport = prev_view_source2

            # --- Set duplicate as IK armature ---
            ik_armature = bpy.context.active_object
            ik_armature.name = scene.target_armature.name + "_MVN_IK"
            ik_armature.data.name = scene.target_armature.name + "_MVN_IK"
            ik_armature.data.mvn_is_source = False
            ik_armature.data.mvn_is_ik = True
            add_to_collection(ik_armature)

            # --- Constrain IK armature to source ---
            add_const(
                ik_armature,
                "COPY_LOCATION",
                {
                    "name": "MVN_IK_COPY_LOCATION",
                    "target": scene.source_armature,
                    "target_space": "WORLD",
                    "owner_space": "WORLD",
                    "influence": 1.0,
                },
            )
            add_const(
                ik_armature,
                "COPY_ROTATION",
                {
                    "name": "MVN_Constraint_COPY_ROTATION",
                    "target": scene.source_armature,
                    "mix_mode": "BEFORE",
                    "target_space": "WORLD",
                    "owner_space": "WORLD",
                    "influence": 1.0,
                },
            )

            # --- Remove duplicate action that was created ---
            if ik_armature.animation_data:
                action = ik_armature.animation_data.action
                bpy.data.actions.remove(action)
                ik_armature.animation_data_clear()

            # --- Set IK armature pointer property in target armature ---
            scene.target_armature.data.mvn_ik_armature = ik_armature

            # --- Select the armature ---
            bpy.context.view_layer.objects.active = ik_armature

            # --- Rename bones with IK suffix ---
            for ik_bone in ik_armature.data.bones:
                ik_bone.name = ik_bone.name + IK_BONE_SUFFIX

            # --- Set IK armature transform to match the target before adjusting the edit bones ---
            ik_armature.rotation_euler = scene.target_armature.rotation_euler
            ik_armature.location = scene.target_armature.location
            ik_armature.scale = scene.target_armature.scale
            """
            add_driver(ik_armature, 'scale', 'SCRIPTED', [ ['TargetScale','SINGLE_PROP',scene.target_armature,{'data_path':f'scale[0]'}] ], f'TargetScale', 0)
            add_driver(ik_armature, 'scale', 'SCRIPTED', [ ['TargetScale','SINGLE_PROP',scene.target_armature,{'data_path':f'scale[0]'}] ], f'TargetScale', 1)
            add_driver(ik_armature, 'scale', 'SCRIPTED', [ ['TargetScale','SINGLE_PROP',scene.target_armature,{'data_path':f'scale[0]'}] ], f'TargetScale', 2)
            """

            # --- Enter Edit Mode ---
            bpy.ops.object.mode_set(mode="EDIT")
            deselect()

            matchsource_val = 0
            if scene.target_armature.data.mvn_ik_matchsource:
                matchsource_val = 1

            # --- Source measure to calculate scale later ---
            for bone in scene.source_armature.data.bones:

                if bone.mvn_tpos[0] != -9999:  # Hips
                    if bone.name == SEGMENT_MAP[1]:
                        source_hip_height = bone.mvn_tpos[2]

                    elif bone.name == SEGMENT_MAP[16]:  # RightUpperLeg
                        s_r_leg_top = Vector(bone.mvn_tpos)

                    elif bone.name == SEGMENT_MAP[18]:  # RightFoot
                        srcRfoot = Vector(bone.mvn_tpos)

                    elif bone.name == SEGMENT_MAP[19]:  # RightToe
                        srcRtoe = Vector(bone.mvn_tpos)

                    elif bone.name == SEGMENT_MAP[20]:  # LeftUpperLeg
                        s_l_leg_top = Vector(bone.mvn_tpos)

                    elif bone.name == SEGMENT_MAP[22]:  # LeftFoot
                        srcLfoot = Vector(bone.mvn_tpos)

                    elif bone.name == SEGMENT_MAP[23]:  # LeftToe
                        srcLtoe = Vector(bone.mvn_tpos)

            source_right_leg_length = abs(s_r_leg_top[2] - srcRtoe[2])
            source_left_leg_length = abs(s_l_leg_top[2] - srcLtoe[2])
            source_max_leg_length = max(source_right_leg_length, source_left_leg_length)

            # --- Adjust IK bones to match target ---
            for ik_edit_bone in ik_armature.data.edit_bones:
                source_name = ik_edit_bone.name[:-6]

                if source_name in ik_dict:
                    ik_edit_bone.head = ik_dict[source_name][0]
                    ik_edit_bone.roll = ik_dict[source_name][2]

                    # upper legs tails to be at lower leg heads
                    if source_name == SEGMENT_MAP[16]:  # RightUpperLeg
                        ik_edit_bone.tail = ik_dict[SEGMENT_MAP[17]][0]

                    elif source_name == SEGMENT_MAP[20]:  # LeftUpperLeg
                        ik_edit_bone.tail = ik_dict[SEGMENT_MAP[21]][0]

                    # low legs tails to be at foot heads
                    elif source_name == SEGMENT_MAP[17]:  # RightLowerLeg
                        ik_edit_bone.tail = ik_dict[SEGMENT_MAP[18]][0]

                    elif source_name == SEGMENT_MAP[21]:  # LeftLowerLeg
                        ik_edit_bone.tail = ik_dict[SEGMENT_MAP[22]][0]

                    # --- Set other to match target armature ---
                    else:
                        ik_edit_bone.tail = ik_dict[source_name][1]

                    required_bone_maps = [
                        SEGMENT_MAP[1],  # Pelvis
                        SEGMENT_MAP[16],  # RightUpperLeg
                        SEGMENT_MAP[17],  # RightLowerLeg
                        SEGMENT_MAP[18],  # RightFoot
                        SEGMENT_MAP[19],  # RightToe
                        SEGMENT_MAP[20],  # LeftUpperLeg
                        SEGMENT_MAP[21],  # LeftLowerLeg
                        SEGMENT_MAP[22],  # LeftFoot
                        SEGMENT_MAP[23],  # LeftToe
                    ]

                    if source_name not in required_bone_maps:
                        ik_edit_bone.tail = ik_edit_bone.head

                else:  # Not mapped. Delete unnecessary bone
                    ik_edit_bone.head = Vector((0, 0, 0))
                    ik_edit_bone.tail = Vector((0, 0, 0))

            # --- Set the pose bones and apply the pose in edit mode ---
            bpy.ops.object.mode_set(mode="POSE")

            for pose_bone in scene.target_armature.pose.bones:

                source_name = pose_bone.bone.mvn_source_bone_name

                if (
                    source_name + IK_BONE_SUFFIX in ik_armature.pose.bones
                ):  # if source_name != "" and source_name != "-":
                    ik_pose_bone = ik_armature.pose.bones[source_name + IK_BONE_SUFFIX]
                    ik_pose_bone.rotation_quaternion = pose_bone.rotation_quaternion
                    ik_pose_bone.location = pose_bone.location

            bpy.ops.pose.armature_apply()
            bpy.ops.object.mode_set(mode="EDIT")

            for ik_edit_bone in ik_armature.data.edit_bones:

                source_name = ik_edit_bone.name[:-6]

                if source_name in ik_dict:  # orient legs to global -y

                    if source_name in [SEGMENT_MAP[16], SEGMENT_MAP[17], SEGMENT_MAP[20], SEGMENT_MAP[21]]:
                        deselect()
                        ik_armature.data.edit_bones.active = ik_edit_bone
                        bpy.ops.armature.calculate_roll(type="GLOBAL_NEG_Y")

            # --- Add additional IK bones ---
            for ik_edit_bone in ik_armature.data.edit_bones:

                source_name = ik_edit_bone.name[:-6]

                if source_name == SEGMENT_MAP[1]:  # Pelvis
                    new_bone = duplicate_bone(ik_armature, "Hips-MvnIK_Target", ik_edit_bone.name)

                elif source_name == SEGMENT_MAP[18]:  # RightFoot
                    new_bone = duplicate_bone(ik_armature, "RightFoot-MvnIK_Target", ik_edit_bone.name, False)

                elif source_name == SEGMENT_MAP[22]:  # LeftFoot
                    new_bone = duplicate_bone(ik_armature, "LeftFoot-MvnIK_Target", ik_edit_bone.name, False)

            # --- Add origin floor bone ---
            new_bone = ik_armature.data.edit_bones.new("MvnIK_Constraints_Target")
            new_bone.head = Vector((0, 0, 0))
            new_bone.tail = Vector((0, 5, 0))

            # --- Set the pose bones and apply the pose in edit mode ---
            bpy.ops.object.mode_set(mode="POSE")
            ik_pose_bone = ik_armature.pose.bones["MvnIK_Constraints_Target"]
            ik_pose_bone.rotation_quaternion = ik_armature.rotation_euler.to_quaternion().inverted()
            bpy.ops.pose.armature_apply()
            bpy.ops.object.mode_set(mode="EDIT")

            # --- Target measure to calculate scale ---
            trg_scale = scene.target_armature.scale[2]

            for bone in scene.target_armature.data.bones:

                if bone.mvn_tpos[0] != -9999:

                    if bone.mvn_source_bone_name == SEGMENT_MAP[1]:  # Pelvis
                        target_hip_height = bone.mvn_tpos[2] / trg_scale

                    elif bone.mvn_source_bone_name == SEGMENT_MAP[16]:  # RightUpperLeg
                        r_leg_top = Vector(bone.mvn_tpos)
                        r_leg_top = Vector(
                            (r_leg_top[0] / trg_scale, r_leg_top[1] / trg_scale, r_leg_top[2] / trg_scale)
                        )

                    elif bone.mvn_source_bone_name == SEGMENT_MAP[18]:  # RightFoot
                        tarRfoot = Vector(bone.mvn_tpos)
                        tarRfoot = Vector((tarRfoot[0] / trg_scale, tarRfoot[1] / trg_scale, tarRfoot[2] / trg_scale))

                    elif bone.mvn_source_bone_name == SEGMENT_MAP[19]:  # RightToe
                        tarRtoe = Vector(bone.mvn_tpos)
                        tarRtoe = Vector((tarRtoe[0] / trg_scale, tarRtoe[1] / trg_scale, tarRtoe[2] / trg_scale))

                    elif bone.mvn_source_bone_name == SEGMENT_MAP[20]:  # LeftUpperLeg
                        l_leg_top = Vector(bone.mvn_tpos)
                        l_leg_top = Vector(
                            (l_leg_top[0] / trg_scale, l_leg_top[1] / trg_scale, l_leg_top[2] / trg_scale)
                        )

                    elif bone.mvn_source_bone_name == SEGMENT_MAP[22]:  # LeftFoot
                        tarLfoot = Vector(bone.mvn_tpos)
                        tarLfoot = Vector((tarLfoot[0] / trg_scale, tarLfoot[1] / trg_scale, tarLfoot[2] / trg_scale))

                    elif bone.mvn_source_bone_name == SEGMENT_MAP[23]:  # LeftToe
                        tarLtoe = Vector(bone.mvn_tpos)
                        tarLtoe = Vector((tarLtoe[0] / trg_scale, tarLtoe[1] / trg_scale, tarLtoe[2] / trg_scale))

            target_right_leg_length = abs(r_leg_top[2] - tarRtoe[2])
            target_left_leg_length = abs(l_leg_top[2] - tarLtoe[2])
            target_max_leg_length = max(target_right_leg_length, target_left_leg_length)

            # --- Add Constraints ---
            bpy.ops.object.mode_set(mode="POSE")

            for pose_bone in ik_armature.pose.bones:
                source_name = pose_bone.name[:-6]
                pose_bone.rotation_quaternion = Quaternion((1.0, 0.0, 0.0, 0.0))
                pose_bone.location = (0.0, 0.0, 0.0)

                # Hips Target
                if pose_bone.name == "Hips-MvnIK_Target":
                    # rotation
                    add_const(
                        pose_bone,
                        "COPY_ROTATION",
                        {
                            "name": "MVN_IK_COPY_ROTATION",
                            "target": scene.source_armature,
                            "subtarget": SEGMENT_MAP[1],
                            "mix_mode": "BEFORE",
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": SEGMENT_MAP[1],
                            "influence": 1.0,
                        },
                    )
                    # position offset
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_PositionOffset",
                            "target": scene.source_armature,
                            "subtarget": SEGMENT_MAP[1],
                            "use_motion_extrapolate": True,
                            "target_space": "POSE",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "from_max_x": 0,
                            "from_min_x": 0,
                            "from_max_y": 0,
                            "from_min_y": 0,
                            "from_max_z": 1,
                            "from_min_z": 0,
                            "to_max_x": 0,
                            "to_min_x": 0,
                            "to_max_y": 0,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "influence": 1.0,
                            "mix_mode": "REPLACE",
                        },
                    )
                    add_driver(
                        const,
                        "from_max_x",
                        "SCRIPTED",
                        [
                            [
                                "WalkOnSpot",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_walkonspot"},
                            ]
                        ],
                        "1-WalkOnSpot",
                    )
                    add_driver(
                        const,
                        "from_max_y",
                        "SCRIPTED",
                        [
                            [
                                "WalkOnSpot",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_walkonspot"},
                            ]
                        ],
                        "1-WalkOnSpot",
                    )
                    add_driver(
                        const,
                        "to_max_x",
                        "SCRIPTED",
                        [
                            [
                                "PelvisScale",
                                "TRANSFORMS",
                                ik_armature,
                                {
                                    "bone_target": SEGMENT_MAP[1] + IK_BONE_SUFFIX,
                                    "transform_type": "SCALE_X",
                                    "transform_space": "TRANSFORM_SPACE",
                                },
                            ]
                        ],
                        "1/PelvisScale",
                    )
                    add_driver(
                        const,
                        "to_max_y",
                        "SCRIPTED",
                        [
                            [
                                "PelvisScale",
                                "TRANSFORMS",
                                ik_armature,
                                {
                                    "bone_target": SEGMENT_MAP[1] + IK_BONE_SUFFIX,
                                    "transform_type": "SCALE_Y",
                                    "transform_space": "TRANSFORM_SPACE",
                                },
                            ]
                        ],
                        "1/PelvisScale",
                    )
                    add_driver(
                        const,
                        "to_max_z",
                        "SCRIPTED",
                        [
                            ["TargetScale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SourceScale", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                            [
                                "MatchSource",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_matchsource"},
                            ],
                        ],
                        f"(1-MatchSource)*((TargetScale*{target_max_leg_length})/(SourceScale*{source_max_leg_length})) + MatchSource*((TargetScale*{target_hip_height})/(SourceScale*{source_hip_height}))",
                    )
                    # hips correction
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_HipsCorrection",
                            "target": ik_armature,
                            "subtarget": "MvnIK_Constraints_Target",
                            "use_motion_extrapolate": True,
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": SEGMENT_MAP[1],
                            "from_max_x": 0,
                            "from_min_x": 0,
                            "from_max_y": 1,
                            "from_min_y": 0,
                            "from_max_z": 0,
                            "from_min_z": 0,
                            "to_max_x": 0,
                            "to_min_x": 0,
                            "to_max_y": 0,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "influence": 1.0 - matchsource_val,
                            "mix_mode": "ADD",
                        },
                    )
                    add_driver(
                        const,
                        "to_max_y",
                        "SCRIPTED",
                        [
                            ["TargetScale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SourceScale", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                        ],
                        f"( (TargetScale*({target_hip_height}-{r_leg_top[2]})) * (SourceScale*{source_max_leg_length})/(TargetScale*{target_max_leg_length}) ) /SourceScale",
                    )
                    # hips level offset
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_HipsLevelOffset",
                            "target": ik_armature,
                            "subtarget": "MvnIK_Constraints_Target",
                            "use_motion_extrapolate": True,
                            "target_space": "LOCAL",
                            "owner_space": "WORLD",
                            "from_max_x": 0,
                            "from_min_x": 0,
                            "from_max_y": 1,
                            "from_min_y": 0,
                            "from_max_z": 0,
                            "from_min_z": 0,
                            "map_to_z_from": "Y",
                            "to_max_x": 0,
                            "to_min_x": 0,
                            "to_max_y": 0,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "influence": 1.0,
                            "mix_mode": "ADD",
                        },
                    )
                    add_driver(
                        const,
                        "to_max_z",
                        "SCRIPTED",
                        [["HipsLevel", "SINGLE_PROP", scene.target_armature, {"data_path": f"data.mvn_ik_levelhips"}]],
                        "HipsLevel",
                    )
                    # hips local offset
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_HipsLocalOffset",
                            "target": ik_armature,
                            "subtarget": "MvnIK_Constraints_Target",
                            "use_motion_extrapolate": True,
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": ik_armature,
                            "space_subtarget": "MvnIK_Constraints_Target",
                            "from_max_x": 0,
                            "from_min_x": 0,
                            "from_max_y": 1,
                            "from_min_y": 0,
                            "from_max_z": 0,
                            "from_min_z": 0,
                            "map_to_x_from": "Y",
                            "map_to_z_from": "Y",
                            "to_max_x": 0,
                            "to_min_x": 0,
                            "to_max_y": 0,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "influence": 1.0,
                            "mix_mode": "ADD",
                        },
                    )
                    add_driver(
                        const,
                        "to_max_x",
                        "SCRIPTED",
                        [
                            [
                                "HipsLocalOffset",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_offsethips[0]"},
                            ],
                            ["Scale", "SINGLE_PROP", ik_armature, {"data_path": f"scale[0]"}],
                        ],
                        f"HipsLocalOffset/Scale",
                    )
                    add_driver(
                        const,
                        "to_max_y",
                        "SCRIPTED",
                        [
                            [
                                "HipsLocalOffset",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_offsethips[1]"},
                            ],
                            ["Scale", "SINGLE_PROP", ik_armature, {"data_path": f"scale[1]"}],
                        ],
                        f"HipsLocalOffset/Scale",
                    )
                    add_driver(
                        const,
                        "to_max_z",
                        "SCRIPTED",
                        [
                            [
                                "HipsLocalOffset",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_offsethips[2]"},
                            ],
                            ["Scale", "SINGLE_PROP", ik_armature, {"data_path": f"scale[2]"}],
                        ],
                        f"HipsLocalOffset/Scale",
                    )

                # Feet Target
                elif pose_bone.name == "RightFoot-MvnIK_Target":
                    # location
                    add_const(
                        pose_bone,
                        "COPY_LOCATION",
                        {
                            "name": "MVN_IK_COPY_LOCATION",
                            "target": scene.source_armature,
                            "subtarget": SEGMENT_MAP[19],
                            "head_tail": 0.0,
                            "target_space": "WORLD",
                            "owner_space": "WORLD",
                            "influence": 1.0,
                        },
                    )
                    # foot correction
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_FootCorrection",
                            "target": ik_armature,
                            "subtarget": "MvnIK_Constraints_Target",
                            "use_motion_extrapolate": True,
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": SEGMENT_MAP[18],
                            "from_max_x": 0,
                            "from_min_x": 0,
                            "from_max_y": 1,
                            "from_min_y": 0,
                            "from_max_z": 0,
                            "from_min_z": 0,
                            "to_max_x": 0,
                            "to_min_x": 0,
                            "to_max_y": 0,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "map_to_x_from": "Y",
                            "map_to_z_from": "Y",
                            "influence": 1.0,
                            "mix_mode": "ADD",
                        },
                    )
                    add_driver(
                        const,
                        "to_max_x",
                        "SCRIPTED",
                        [
                            ["TS", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SS", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                            [
                                "PelvisScale",
                                "TRANSFORMS",
                                ik_armature,
                                {
                                    "bone_target": SEGMENT_MAP[1] + IK_BONE_SUFFIX,
                                    "transform_type": "SCALE_Y",
                                    "transform_space": "TRANSFORM_SPACE",
                                },
                            ],
                        ],
                        f"(TS*{tarRfoot.x-tarRtoe.x})*PelvisScale/SS",
                    )
                    add_driver(
                        const,
                        "to_max_y",
                        "SCRIPTED",
                        [
                            ["TS", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SS", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                            [
                                "PelvisScale",
                                "TRANSFORMS",
                                ik_armature,
                                {
                                    "bone_target": SEGMENT_MAP[1] + IK_BONE_SUFFIX,
                                    "transform_type": "SCALE_Y",
                                    "transform_space": "TRANSFORM_SPACE",
                                },
                            ],
                        ],
                        f"-(TS*{tarRfoot.z-tarRtoe.z}*PelvisScale*cos(atan2(SS*{srcRfoot.z-srcRtoe.z},SS*{srcRfoot.y-srcRtoe.y})-atan2(TS*{tarRfoot.z-tarRtoe.z},TS*{tarRfoot.y-tarRtoe.y})))/(sin(atan2(TS*{tarRfoot.z-tarRtoe.z},TS*{tarRfoot.y-tarRtoe.y})))/SS",
                    )
                    add_driver(
                        const,
                        "to_max_z",
                        "SCRIPTED",
                        [
                            ["TS", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SS", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                            [
                                "PelvisScale",
                                "TRANSFORMS",
                                ik_armature,
                                {
                                    "bone_target": SEGMENT_MAP[1] + IK_BONE_SUFFIX,
                                    "transform_type": "SCALE_Y",
                                    "transform_space": "TRANSFORM_SPACE",
                                },
                            ],
                        ],
                        f"(TS*{tarRfoot.z-tarRtoe.z}*PelvisScale*sin(atan2(SS*{srcRfoot.z-srcRtoe.z},SS*{srcRfoot.y-srcRtoe.y})-atan2(TS*{tarRfoot.z-tarRtoe.z},TS*{tarRfoot.y-tarRtoe.y})))/(sin(atan2(TS*{tarRfoot.z-tarRtoe.z},TS*{tarRfoot.y-tarRtoe.y})))/SS",
                    )
                    # rotation
                    add_const(
                        pose_bone,
                        "COPY_ROTATION",
                        {
                            "name": "MVN_IK_COPY_ROTATION",
                            "target": scene.source_armature,
                            "subtarget": SEGMENT_MAP[18],
                            "mix_mode": "BEFORE",
                            "target_space": "LOCAL_WITH_PARENT",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": SEGMENT_MAP[18],
                            "influence": 1.0,
                        },
                    )
                    # feet spacing
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_FootSpread",
                            "target": ik_armature,
                            "subtarget": "MvnIK_Constraints_Target",
                            "use_motion_extrapolate": True,
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": ik_armature,
                            "space_subtarget": "MvnIK_Constraints_Target",
                            "from_max_x": 0,
                            "from_min_x": 0,
                            "from_max_y": 1,
                            "from_min_y": 0,
                            "from_max_z": 0,
                            "from_min_z": 0,
                            "to_max_x": 0,
                            "to_min_x": 0,
                            "to_max_y": 0,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "map_to_x_from": "Y",
                            "influence": 1.0,
                            "mix_mode": "ADD",
                        },
                    )
                    add_driver(
                        const,
                        "to_max_x",
                        "SCRIPTED",
                        [
                            [
                                "FeetSpacing",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_feetspread"},
                            ],
                            ["Scale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[0]"}],
                        ],
                        f"-FeetSpacing/(2*Scale)",
                    )

                elif pose_bone.name == "LeftFoot-MvnIK_Target":
                    # location
                    add_const(
                        pose_bone,
                        "COPY_LOCATION",
                        {
                            "name": "MVN_IK_COPY_LOCATION",
                            "target": scene.source_armature,
                            "subtarget": SEGMENT_MAP[23],
                            "head_tail": 0.0,
                            "target_space": "WORLD",
                            "owner_space": "WORLD",
                            "influence": 1.0,
                        },
                    )
                    # foot correction
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_FootCorrection",
                            "target": ik_armature,
                            "subtarget": "MvnIK_Constraints_Target",
                            "use_motion_extrapolate": True,
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": SEGMENT_MAP[22],
                            "from_max_x": 0,
                            "from_min_x": 0,
                            "from_max_y": 1,
                            "from_min_y": 0,
                            "from_max_z": 0,
                            "from_min_z": 0,
                            "to_max_x": 0,
                            "to_min_x": 0,
                            "to_max_y": 0,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "map_to_x_from": "Y",
                            "map_to_z_from": "Y",
                            "influence": 1.0,
                            "mix_mode": "ADD",
                        },
                    )
                    add_driver(
                        const,
                        "to_max_x",
                        "SCRIPTED",
                        [
                            ["TS", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SS", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                            [
                                "PelvisScale",
                                "TRANSFORMS",
                                ik_armature,
                                {
                                    "bone_target": SEGMENT_MAP[1] + IK_BONE_SUFFIX,
                                    "transform_type": "SCALE_Y",
                                    "transform_space": "TRANSFORM_SPACE",
                                },
                            ],
                        ],
                        f"(TS*{tarLfoot.x-tarLtoe.x})*PelvisScale/SS",
                    )
                    add_driver(
                        const,
                        "to_max_y",
                        "SCRIPTED",
                        [
                            ["TS", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SS", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                            [
                                "PelvisScale",
                                "TRANSFORMS",
                                ik_armature,
                                {
                                    "bone_target": SEGMENT_MAP[1] + IK_BONE_SUFFIX,
                                    "transform_type": "SCALE_Y",
                                    "transform_space": "TRANSFORM_SPACE",
                                },
                            ],
                        ],
                        f"-(TS*{tarLfoot.z-tarLtoe.z}*PelvisScale*cos(atan2(SS*{srcLfoot.z-srcLtoe.z},SS*{srcLfoot.y-srcLtoe.y})-atan2(TS*{tarLfoot.z-tarLtoe.z},TS*{tarLfoot.y-tarLtoe.y})))/(sin(atan2(TS*{tarLfoot.z-tarLtoe.z},TS*{tarLfoot.y-tarLtoe.y})))/SS",
                    )
                    add_driver(
                        const,
                        "to_max_z",
                        "SCRIPTED",
                        [
                            ["TS", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SS", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                            [
                                "PelvisScale",
                                "TRANSFORMS",
                                ik_armature,
                                {
                                    "bone_target": SEGMENT_MAP[1] + IK_BONE_SUFFIX,
                                    "transform_type": "SCALE_Y",
                                    "transform_space": "TRANSFORM_SPACE",
                                },
                            ],
                        ],
                        f"(TS*{tarLfoot.z-tarLtoe.z}*PelvisScale*sin(atan2(SS*{srcLfoot.z-srcLtoe.z},SS*{srcLfoot.y-srcLtoe.y})-atan2(TS*{tarLfoot.z-tarLtoe.z},TS*{tarLfoot.y-tarLtoe.y})))/(sin(atan2(TS*{tarLfoot.z-tarLtoe.z},TS*{tarLfoot.y-tarLtoe.y})))/SS",
                    )
                    # rotation
                    add_const(
                        pose_bone,
                        "COPY_ROTATION",
                        {
                            "name": "MVN_IK_COPY_ROTATION",
                            "target": scene.source_armature,
                            "subtarget": SEGMENT_MAP[22],
                            "mix_mode": "BEFORE",
                            "target_space": "LOCAL_WITH_PARENT",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": SEGMENT_MAP[22],
                            "influence": 1.0,
                        },
                    )
                    # feet spacing
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_FootSpread",
                            "target": ik_armature,
                            "subtarget": "MvnIK_Constraints_Target",
                            "use_motion_extrapolate": True,
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": ik_armature,
                            "space_subtarget": "MvnIK_Constraints_Target",
                            "from_max_x": 0,
                            "from_min_x": 0,
                            "from_max_y": 1,
                            "from_min_y": 0,
                            "from_max_z": 0,
                            "from_min_z": 0,
                            "to_max_x": 0,
                            "to_min_x": 0,
                            "to_max_y": 0,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "map_to_x_from": "Y",
                            "influence": 1.0,
                            "mix_mode": "ADD",
                        },
                    )
                    add_driver(
                        const,
                        "to_max_x",
                        "SCRIPTED",
                        [
                            [
                                "FeetSpacing",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_feetspread"},
                            ],
                            ["Scale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[0]"}],
                        ],
                        f"FeetSpacing/(2*Scale)",
                    )

                # Other
                elif pose_bone.name == "MvnIK_Constraints_Target":
                    pose_bone.location = (0.0, 1.0, 0.0)
                    add_const(
                        pose_bone,
                        "COPY_ROTATION",
                        {
                            "name": "MVN_IK_COPY_ROTATION",
                            "target": scene.source_armature,
                            "subtarget": SEGMENT_MAP[1],
                            "mix_mode": "BEFORE",
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": SEGMENT_MAP[1],
                            "influence": 1.0,
                        },
                    )

                # Hips
                elif pose_bone.name == SEGMENT_MAP[1] + IK_BONE_SUFFIX:
                    # pose_bone.scale = [leg_len_ratio_ms, leg_len_ratio_ms, leg_len_ratio_ms]
                    add_driver(
                        pose_bone,
                        "scale",
                        "SCRIPTED",
                        [
                            ["TargetScale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[0]"}],
                            ["SourceScale", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[0]"}],
                            [
                                "MatchSource",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_matchsource"},
                            ],
                        ],
                        f"MatchSource+(1-MatchSource)*(SourceScale*{source_max_leg_length})/(TargetScale*{target_max_leg_length})",
                        0,
                    )
                    add_driver(
                        pose_bone,
                        "scale",
                        "SCRIPTED",
                        [
                            ["TargetScale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[1]"}],
                            ["SourceScale", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[1]"}],
                            [
                                "MatchSource",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_matchsource"},
                            ],
                        ],
                        f"MatchSource+(1-MatchSource)*(SourceScale*{source_max_leg_length})/(TargetScale*{target_max_leg_length})",
                        1,
                    )
                    add_driver(
                        pose_bone,
                        "scale",
                        "SCRIPTED",
                        [
                            ["TargetScale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SourceScale", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                            [
                                "MatchSource",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_matchsource"},
                            ],
                        ],
                        f"MatchSource+(1-MatchSource)*(SourceScale*{source_max_leg_length})/(TargetScale*{target_max_leg_length})",
                        2,
                    )
                    # rotation
                    const = add_const(
                        pose_bone,
                        "COPY_ROTATION",
                        {
                            "name": "MVN_IK_COPY_ROTATION",
                            "target": scene.source_armature,
                            "subtarget": SEGMENT_MAP[1],
                            "mix_mode": "BEFORE",
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": SEGMENT_MAP[1],
                            "influence": 1.0,
                        },
                    )
                    # position offset
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_PositionOffset",
                            "target": scene.source_armature,
                            "subtarget": SEGMENT_MAP[1],
                            "use_motion_extrapolate": True,
                            "target_space": "POSE",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "from_max_x": 1,
                            "from_min_x": 0,
                            "from_max_y": 1,
                            "from_min_y": 0,
                            "from_max_z": 1,
                            "from_min_z": 0,
                            "to_max_x": 1,
                            "to_min_x": 0,
                            "to_max_y": 1,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "influence": 1.0,
                            "mix_mode": "REPLACE",
                        },
                    )
                    add_driver(
                        const,
                        "to_max_z",
                        "SCRIPTED",
                        [
                            ["TargetScale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SourceScale", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                        ],
                        f"(TargetScale*{target_hip_height}) / (SourceScale*{source_hip_height})",
                    )
                    # location
                    const = add_const(
                        pose_bone,
                        "COPY_LOCATION",
                        {
                            "name": "MVN_IK_COPY_LOCATION",
                            "target": scene.source_armature,
                            "subtarget": SEGMENT_MAP[1],
                            "target_space": "WORLD",
                            "owner_space": "WORLD",
                            "influence": 1 - matchsource_val,
                        },
                    )
                    # hips correction
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_HipsCorrection",
                            "target": ik_armature,
                            "subtarget": "MvnIK_Constraints_Target",
                            "use_motion_extrapolate": True,
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": SEGMENT_MAP[1],
                            "from_max_x": 0,
                            "from_min_x": 0,
                            "from_max_y": 1,
                            "from_min_y": 0,
                            "from_max_z": 0,
                            "from_min_z": 0,
                            "to_max_x": 0,
                            "to_min_x": 0,
                            "to_max_y": 0,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "influence": 1.0 - matchsource_val,
                            "mix_mode": "ADD",
                        },
                    )
                    add_driver(
                        const,
                        "to_max_y",
                        "SCRIPTED",
                        [
                            ["TargetScale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SourceScale", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                        ],
                        f"( (TargetScale*({target_hip_height}-{r_leg_top[2]})) * (SourceScale*{source_max_leg_length})/(TargetScale*{target_max_leg_length}) ) /SourceScale",
                    )
                    # hips level offset
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_HipsLevelOffset",
                            "target": ik_armature,
                            "subtarget": "MvnIK_Constraints_Target",
                            "use_motion_extrapolate": True,
                            "target_space": "LOCAL",
                            "owner_space": "WORLD",
                            "from_max_x": 0,
                            "from_min_x": 0,
                            "from_max_y": 1,
                            "from_min_y": 0,
                            "from_max_z": 0,
                            "from_min_z": 0,
                            "map_to_z_from": "Y",
                            "to_max_x": 0,
                            "to_min_x": 0,
                            "to_max_y": 0,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "influence": 1.0,
                            "mix_mode": "ADD",
                        },
                    )
                    add_driver(
                        const,
                        "to_max_z",
                        "SCRIPTED",
                        [
                            [
                                "HipsLevel",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_levelhips"},
                            ],
                            [
                                "Scale",
                                "TRANSFORMS",
                                ik_armature,
                                {
                                    "bone_target": "Pelvis" + IK_BONE_SUFFIX,
                                    "transform_type": "SCALE_Z",
                                    "transform_space": "TRANSFORM_SPACE",
                                },
                            ],
                        ],
                        "HipsLevel * Scale",
                    )
                    # hips local offset
                    const = add_const(
                        pose_bone,
                        "TRANSFORM",
                        {
                            "name": "MVN_IK_TRANSFORM_HipsLocalOffset",
                            "target": ik_armature,
                            "subtarget": "MvnIK_Constraints_Target",
                            "use_motion_extrapolate": True,
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": ik_armature,
                            "space_subtarget": "MvnIK_Constraints_Target",
                            "from_max_x": 0,
                            "from_min_x": 0,
                            "from_max_y": 1,
                            "from_min_y": 0,
                            "from_max_z": 0,
                            "from_min_z": 0,
                            "map_to_x_from": "Y",
                            "map_to_z_from": "Y",
                            "to_max_x": 0,
                            "to_min_x": 0,
                            "to_max_y": 0,
                            "to_min_y": 0,
                            "to_max_z": 0,
                            "to_min_z": 0,
                            "influence": 1.0,
                            "mix_mode": "ADD",
                        },
                    )
                    add_driver(
                        const,
                        "to_max_x",
                        "SCRIPTED",
                        [
                            [
                                "MatchSource",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_matchsource"},
                            ],
                            [
                                "HipsLocalOffset",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_offsethips[0]"},
                            ],
                            ["TargetScale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[0]"}],
                            ["SourceScale", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[0]"}],
                            ["IkScale", "SINGLE_PROP", ik_armature, {"data_path": f"scale[0]"}],
                        ],
                        f"(MatchSource+(1-MatchSource)*( (SourceScale*{source_hip_height}) / (TargetScale*{target_hip_height}) ))*HipsLocalOffset/IkScale",
                    )
                    add_driver(
                        const,
                        "to_max_y",
                        "SCRIPTED",
                        [
                            [
                                "MatchSource",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_matchsource"},
                            ],
                            [
                                "HipsLocalOffset",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_offsethips[1]"},
                            ],
                            ["TargetScale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[1]"}],
                            ["SourceScale", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[1]"}],
                            ["IkScale", "SINGLE_PROP", ik_armature, {"data_path": f"scale[1]"}],
                        ],
                        f"(MatchSource+(1-MatchSource)*( (SourceScale*{source_hip_height}) / (TargetScale*{target_hip_height}) ))*HipsLocalOffset/IkScale",
                    )
                    add_driver(
                        const,
                        "to_max_z",
                        "SCRIPTED",
                        [
                            [
                                "MatchSource",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_matchsource"},
                            ],
                            [
                                "HipsLocalOffset",
                                "SINGLE_PROP",
                                scene.target_armature,
                                {"data_path": f"data.mvn_ik_offsethips[2]"},
                            ],
                            ["TargetScale", "SINGLE_PROP", scene.target_armature, {"data_path": f"scale[2]"}],
                            ["SourceScale", "SINGLE_PROP", scene.source_armature, {"data_path": f"scale[2]"}],
                            ["IkScale", "SINGLE_PROP", ik_armature, {"data_path": f"scale[2]"}],
                        ],
                        f"(MatchSource+(1-MatchSource)*( (SourceScale*{source_hip_height}) / (TargetScale*{target_hip_height}) ))*HipsLocalOffset/IkScale",
                    )

                # Legs
                elif pose_bone.name == SEGMENT_MAP[17] + IK_BONE_SUFFIX:
                    # copy rotation
                    add_const(
                        pose_bone,
                        "COPY_ROTATION",
                        {
                            "name": "MVN_Constraint_COPY_ROTATION",
                            "target": scene.source_armature,
                            "subtarget": source_name,
                            "mix_mode": "BEFORE",
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": source_name,
                            "influence": 1.0,
                        },
                    )
                    # ik
                    add_const(
                        pose_bone,
                        "IK",
                        {
                            "name": "MVN_IK",
                            "target": ik_armature,
                            "subtarget": "RightFoot-MvnIK_Target",
                            "iterations": 500,
                            "chain_count": 2,
                            "use_tail": True,
                            "use_stretch": True,
                            "influence": 1.0,
                        },
                    )
                    pose_bone.use_ik_limit_x = True
                    pose_bone.ik_max_x = 0.0
                    pose_bone.lock_ik_y = True
                    pose_bone.lock_ik_z = True

                elif pose_bone.name == SEGMENT_MAP[21] + IK_BONE_SUFFIX:
                    # copy rotation
                    add_const(
                        pose_bone,
                        "COPY_ROTATION",
                        {
                            "name": "MVN_Constraint_COPY_ROTATION",
                            "target": scene.source_armature,
                            "subtarget": source_name,
                            "mix_mode": "BEFORE",
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": source_name,
                            "influence": 1.0,
                        },
                    )
                    # ik
                    add_const(
                        pose_bone,
                        "IK",
                        {
                            "name": "MVN_IK",
                            "target": ik_armature,
                            "subtarget": "LeftFoot-MvnIK_Target",
                            "iterations": 500,
                            "chain_count": 2,
                            "use_tail": True,
                            "use_stretch": True,
                            "influence": 1.0,
                        },
                    )
                    pose_bone.use_ik_limit_x = True
                    pose_bone.ik_max_x = 0.0
                    pose_bone.lock_ik_y = True
                    pose_bone.lock_ik_z = True

                # Feet
                elif pose_bone.name == SEGMENT_MAP[18] + IK_BONE_SUFFIX:
                    add_const(
                        pose_bone,
                        "COPY_ROTATION",
                        {
                            "name": "MVN_IK_COPY_ROTATION",
                            "target": ik_armature,
                            "subtarget": "RightFoot-MvnIK_Target",
                            "mix_mode": "REPLACE",
                            "target_space": "LOCAL_OWNER_ORIENT",
                            "owner_space": "LOCAL_WITH_PARENT",
                            "influence": 1.0,
                        },
                    )

                elif pose_bone.name == SEGMENT_MAP[22] + IK_BONE_SUFFIX:
                    add_const(
                        pose_bone,
                        "COPY_ROTATION",
                        {
                            "name": "MVN_IK_COPY_ROTATION",
                            "target": ik_armature,
                            "subtarget": "LeftFoot-MvnIK_Target",
                            "mix_mode": "REPLACE",
                            "target_space": "LOCAL_OWNER_ORIENT",
                            "owner_space": "LOCAL_WITH_PARENT",
                            "influence": 1.0,
                        },
                    )

                # add base constraint to remaining
                else:  # elif pose_bone.name != SEGMENT_MAP[16]+IK_BONE_SUFFIX and pose_bone.name != SEGMENT_MAP[20]+IK_BONE_SUFFIX:
                    add_const(
                        pose_bone,
                        "COPY_ROTATION",
                        {
                            "name": "MVN_Constraint_COPY_ROTATION",
                            "target": scene.source_armature,
                            "subtarget": source_name,
                            "mix_mode": "BEFORE",
                            "target_space": "LOCAL",
                            "owner_space": "CUSTOM",
                            "space_object": scene.source_armature,
                            "space_subtarget": source_name,
                            "influence": 1.0,
                        },
                    )

                # lock ik pose bone transforms
                pose_bone.lock_location = (True, True, True)
                pose_bone.lock_rotation_w = True
                pose_bone.lock_rotation = (True, True, True)
                pose_bone.lock_scale = (True, True, True)

            # Lock the ik armature transforms
            ik_armature.lock_location = (True, True, True)
            ik_armature.lock_rotation = (True, True, True)
            ik_armature.lock_scale = (True, True, True)

            # object mode
            bpy.ops.object.mode_set(mode="OBJECT")
            deselect()
            # pose mode
            switch_mode("POSE", scene.target_armature)

            # Change Target constraints source to ik armature
            for pose_bone in scene.target_armature.pose.bones:
                source_bone_name = scene.target_armature.data.bones[pose_bone.name].mvn_source_bone_name
                # hips
                if source_bone_name == SEGMENT_MAP[1]:
                    for constraint in pose_bone.constraints:
                        if "MVN_TRANSFORM_LOC" in constraint.name:
                            constraint.influence = 0
                        elif "MVN_TRANSFORM_ROT" in constraint.name:
                            constraint.influence = 0
                    # location
                    if "MVN_IK_COPY_LOCATION" in pose_bone.constraints:
                        const = pose_bone.constraints["MVN_IK_COPY_LOCATION"]
                        mod_const(
                            const,
                            {
                                "name": "MVN_IK_COPY_LOCATION",
                                "target": ik_armature,
                                "subtarget": "Hips-MvnIK_Target",
                                "use_offset": True,
                                "target_space": "LOCAL_OWNER_ORIENT",
                                "owner_space": "LOCAL",
                                "influence": 1,
                            },
                        )
                    else:
                        add_const(
                            pose_bone,
                            "COPY_LOCATION",
                            {
                                "name": "MVN_IK_COPY_LOCATION",
                                "target": ik_armature,
                                "subtarget": "Hips-MvnIK_Target",
                                "use_offset": True,
                                "target_space": "LOCAL_OWNER_ORIENT",
                                "owner_space": "LOCAL",
                                "influence": 1,
                            },
                        )
                    # rotation
                    if "MVN_IK_COPY_ROTATION" in pose_bone.constraints:
                        const = pose_bone.constraints["MVN_IK_COPY_ROTATION"]
                        mod_const(
                            const,
                            {
                                "name": "MVN_IK_COPY_ROTATION",
                                "target": ik_armature,
                                "subtarget": "Hips-MvnIK_Target",
                                "mix_mode": "BEFORE",
                                "target_space": "LOCAL_OWNER_ORIENT",
                                "owner_space": "LOCAL",
                                "influence": 1,
                            },
                        )
                    else:
                        add_const(
                            pose_bone,
                            "COPY_ROTATION",
                            {
                                "name": "MVN_IK_COPY_ROTATION",
                                "target": ik_armature,
                                "subtarget": "Hips-MvnIK_Target",
                                "mix_mode": "BEFORE",
                                "target_space": "LOCAL_OWNER_ORIENT",
                                "owner_space": "LOCAL",
                                "influence": 1,
                            },
                        )

                # legs/feet
                elif source_bone_name in [
                    SEGMENT_MAP[16],
                    SEGMENT_MAP[17],
                    SEGMENT_MAP[18],
                    SEGMENT_MAP[19],
                    SEGMENT_MAP[20],
                    SEGMENT_MAP[21],
                    SEGMENT_MAP[22],
                    SEGMENT_MAP[23],
                ]:
                    for constraint in pose_bone.constraints:
                        if "MVN_TRANSFORM_ROT" in constraint.name:
                            constraint.name = "MVN_FEET_COPY_ROTATION"
                            constraint.influence = 1 - scene.target_armature.data.mvn_ik_feet
                    if "MVN_IK_FEET_COPY_ROTATION" in pose_bone.constraints:
                        const = pose_bone.constraints["MVN_IK_FEET_COPY_ROTATION"]
                        mod_const(
                            const,
                            {
                                "name": "MVN_IK_FEET_COPY_ROTATION",
                                "target": ik_armature,
                                "subtarget": source_bone_name + IK_BONE_SUFFIX,
                                "mix_mode": "AFTER",
                                "target_space": "LOCAL_OWNER_ORIENT",
                                "owner_space": "LOCAL",
                                "influence": scene.target_armature.data.mvn_ik_feet,
                            },
                        )
                    else:
                        add_const(
                            pose_bone,
                            "COPY_ROTATION",
                            {
                                "name": "MVN_IK_FEET_COPY_ROTATION",
                                "target": ik_armature,
                                "subtarget": source_bone_name + IK_BONE_SUFFIX,
                                "mix_mode": "AFTER",
                                "target_space": "LOCAL_OWNER_ORIENT",
                                "owner_space": "LOCAL",
                                "influence": scene.target_armature.data.mvn_ik_feet,
                            },
                        )

            bpy.ops.object.mode_set(mode="OBJECT")

            # --- Set IK Armature Visibility ---
            ik_armature.hide_set(True)
            ik_armature.hide_viewport = True
            ik_armature.hide_render = True
            deselect()

            scene.target_armature.hide_set(prev_view_target)
            scene.target_armature.hide_viewport = prev_view_target2
            bpy.data.collections.get(MVN_COLLECTION).hide_viewport = prev_view_collection
            bpy.context.view_layer.layer_collection.children.get(MVN_COLLECTION).hide_viewport = prev_view_collection2

        else:  # If IK is turned off delete the previous IK rig
            if scene.target_armature.data.mvn_ik_armature:
                delete_ik_rig(scene.target_armature.data.mvn_ik_armature)
            update_all_bonemaps()
            deselect()

    elif scene.target_armature:  # If no source delete the previous IK rig
        if scene.target_armature.data.mvn_ik_armature:
            delete_ik_rig(scene.target_armature.data.mvn_ik_armature)
        update_all_bonemaps()
