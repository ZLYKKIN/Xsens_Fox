# **********************************************************************************************************************
# Recorder.py: Handles recording, playback, and manipulation of motion capture data streamed from MVN Animate via UDP.
# Functionality:
#   - Start and stop recordings.
#   - Apply motion data to Blender armatures or objects.
#   - Save recorded animations.
#
# Reference:
#   Xsens UDP documentation: https://base.movella.com/s/article/UDP-Remote-Control?language=en_US
# **********************************************************************************************************************
import bpy
import socket
from math import pi, acos
from datetime import datetime
from mathutils import Quaternion

from .logger import FoxLog

# ============================================================================================
# Global Variables
values_dict = None
start_time = None


#  ============================================================================================
# Functions
def message_xsens(message: str) -> None:
    """
    Sends a UDP message Xsens.

    Checks if the 'mvn_trigger_xsens_record' flag is set in the Blender scene context.
    If it is, it creates a UDP socket and sends the provided message to the Xsens system at the IP address and port
    specified in the Blender scene context. After sending the message, it closes the socket.

    Args:
        message (str): The message to be sent to the Xsens system.
    """
    if bpy.context.scene.mvn_trigger_xsens_record:
        udp_ip = bpy.context.scene.mvn_stream_address
        udp_port = int(bpy.context.scene.mvn_trigger_xsens_port)
        message_bytes = message.encode("utf-8")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(message_bytes, (udp_ip, udp_port))
        sock.close()
    return


def start_recording():
    """
    Initializes the global variables `start_time` and `values_dict`, sets the 'mvn_recording' flag in the
    Blender scene context to True, and sends a UDP message to the Xsens system to start capturing data.

    The message sent to Xsens includes the take name and take number specified in the Blender scene context.
    """
    global start_time
    global values_dict
    values_dict = {}
    start_time = datetime.now()

    bpy.context.scene.mvn_recording = True

    take_name = bpy.context.scene.mvn_take_name
    take_number = bpy.context.scene.mvn_take_number
    FoxLog.log("record", "state=start", take=take_name, num=take_number, rate=bpy.context.scene.mvn_record_rate)
    message = f'<CaptureName> <Name VALUE="{take_name}" /> <Take VALUE="{take_number}" /> </CaptureName>'
    message_xsens(message)
    message = f'<CaptureStart> <Name VALUE="{take_name}" /> <Take VALUE="{take_number}" /> <TimeCode VALUE="" /> <Notes></Notes> </CaptureStart>'
    message_xsens(message)


def stop_recording():
    """
    Sets the 'mvn_recording' flag in the Blender scene context to False, sends a UDP message Xsens to stop capturing data,
    and saves the recorded animations in Blender. It also increments the take number in the Blender scene context.

    The message sent to Xsens includes the take name and take number specified in the Blender scene context.
    """
    bpy.context.scene.mvn_recording = False
    FoxLog.log("record", "state=stop", take=bpy.context.scene.mvn_take_name, num=bpy.context.scene.mvn_take_number)

    message = f'<CaptureStop> <Name VALUE="" /> <TimeCode VALUE="" /> <Notes></Notes> </CaptureStop>'
    message_xsens(message)
    save_recordings()

    bpy.context.scene.mvn_take_number = bpy.context.scene.mvn_take_number + 1


def update_xsens_remote():
    """
    Updates Xsens with the current take name and number.

    Retrieves the take name and take number from the Blender scene context and sends a UDP message
    to Xsens to update these values.
    """
    take_name = bpy.context.scene.mvn_take_name
    take_number = bpy.context.scene.mvn_take_number
    message = f'<CaptureName> <Name VALUE="{take_name}" /> <Take VALUE="{take_number}" /> </CaptureName>'
    message_xsens(message)


def add_record(obj, bone_name="", rotation=None, location=None):
    """
    Adds a record of the current frame's rotation and location data for a given object or bone.

    This function checks if the 'mvn_recording' flag is set in the Blender scene context. If it is, it adds a record of the current
    frame's rotation and location data for the given object or bone to the global `values_dict`.

    For armature objects, the function records data for a specific bone. For other objects, it records data for the object itself.

    Args:
        obj (bpy.types.Object): The object or armature to record data for.
        bone_name (str, optional): The name of the bone to record data for. Only used if `obj` is an armature. Defaults to "".
        rotation (Quaternion, optional): The rotation data to record. Defaults to None.
        location (Vector, optional): The location data to record. Defaults to None.
    """
    global values_dict

    if bpy.context.scene.mvn_recording:
        # --- get or create temp record data ---
        current = []
        prev_frame = None
        time_dif = (datetime.now() - start_time).total_seconds()
        frame = round(time_dif * float(bpy.context.scene.mvn_record_rate))

        # --- If Armature: ---
        if obj.type == "ARMATURE":
            if obj in values_dict:
                if bone_name in values_dict[obj]:
                    current = values_dict[obj][bone_name]
                    prev_frame = values_dict[obj][bone_name][-1][0]
            else:
                values_dict[obj] = {}
                FoxLog.log("record", "state=track-begin", obj=obj.name, type="ARMATURE", start_frame=frame)

            new = current

            if location and rotation:
                new_item = [frame, rotation, location]
            elif rotation:
                new_item = [frame, rotation]

            if prev_frame:
                if frame > prev_frame:
                    new.append(new_item)
                    values_dict[obj][bone_name] = new
            else:
                new.append(new_item)
                values_dict[obj][bone_name] = new

        # --- If NOT Armature ---
        else:
            if obj in values_dict:
                current = values_dict[obj]
                prev_frame = values_dict[obj][-1][0]

            new = current

            if location and rotation:
                new_item = [frame, rotation, location]
            elif rotation:
                new_item = [frame, rotation]

            if prev_frame:
                if frame > prev_frame:
                    new.append(new_item)
                    values_dict[obj] = new
            else:
                new.append(new_item)
                values_dict[obj] = new


def save_recordings():
    """
    Saves the recorded animations in Blender.

    This function checks if the global `values_dict` is not empty. If it is not, it iterates over each object and its associated data
    in `values_dict`. For each object, it creates a new action and sets it as the object's current action.
    It then iterates over each data point for the object and adds keyframes for the rotation and location data to the action.

    For armature objects, the function records data for each bone. For other objects, it records data for the object itself.

    After all data has been processed, it clears `values_dict`.
    """
    global values_dict
    interpolation_mode = "LINEAR"  # "CONSTANT"
    if not values_dict:
        return

    for obj, data in values_dict.items():
        if not data:
            message = f"Failed to save recording for {obj.name}!"
            bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="ERROR", message_text=message)
            FoxLog.error(message)
            return

        # --- Create a new action ---
        take_name = (
            bpy.context.scene.mvn_take_name + "-" + str(bpy.context.scene.mvn_take_number).zfill(3) + "_" + obj.name
        )
        action = bpy.data.actions.new(name=take_name)
        action.use_fake_user = True
        if obj.animation_data is None:
            obj.animation_data_create()
        obj.animation_data.action = action

        # --- Log a summary of what is being baked into this take ---
        if obj.type == "ARMATURE":
            _bone_count = len(data)
            _frame_count = max((len(v) for v in data.values()), default=0)
        else:
            _bone_count = 0
            _frame_count = len(data)
        FoxLog.log("record", "state=save", obj=obj.name, take=take_name, bones=_bone_count, frames=_frame_count)

        # --- If Armature: Plot Armature ---
        if obj.type == "ARMATURE":
            # {'Pelvis':[[frame,rotation,location],[frame,rotation,location],[frame,rotation,location]]}
            for bone_name, bone_data in data.items():
                frame_count = len(bone_data)

                # --- Rotation Curves ---
                if len(bone_data[0]) >= 2:
                    for axis_id in range(4):
                        fcurve = action.fcurves.new(
                            data_path=f'pose.bones["{bone_name}"].rotation_quaternion', index=axis_id
                        )
                        keyframes = fcurve.keyframe_points
                        keyframes.add(frame_count)

                        # --- Set Keys ---
                        prev_frame = -1
                        prev_q = Quaternion((1, 0, 0, 0))
                        for data_point_id in range(len(bone_data)):
                            data_point = bone_data[data_point_id]
                            if len(data_point) >= 2:
                                frame = data_point[0]
                                value = data_point[1][axis_id]
                                current_q = data_point[1]

                                # --- Correct flipped rotation if needed ---
                                try:
                                    angle = acos(min(1, max(-1, current_q.dot(prev_q)))) * 2
                                    if angle > pi:
                                        q_new = [1, 0, 0, 0]
                                        for i in range(4):
                                            q_new[i] = -current_q[i]
                                        value = q_new[axis_id]
                                        prev_q = q_new
                                    else:
                                        prev_q = current_q
                                except:
                                    prev_q = current_q
                                if frame > prev_frame:
                                    keyframes[data_point_id].co = (frame, value)
                                    keyframes[data_point_id].interpolation = interpolation_mode
                                    prev_frame = frame

                # --- Location Curves ---
                if len(bone_data[0]) >= 3:
                    for axis_id in range(3):
                        fcurve = action.fcurves.new(data_path=f'pose.bones["{bone_name}"].location', index=axis_id)
                        keyframes = fcurve.keyframe_points
                        keyframes.add(frame_count)

                        # --- Set Keys ---
                        prev_frame = -1
                        for data_point_id in range(len(bone_data)):
                            data_point = bone_data[data_point_id]
                            if len(data_point) >= 3:
                                frame = data_point[0]
                                value = data_point[2][axis_id]
                                if frame > prev_frame:
                                    keyframes[data_point_id].co = (frame, value)
                                    keyframes[data_point_id].interpolation = interpolation_mode
                                    prev_frame = frame

        # --- If Empty: Plot Empty ---
        else:
            # [[frame,rotation,location],[frame,rotation,location],[frame,rotation,location]]
            bone_data = data
            frame_count = len(bone_data)

            # --- Rotation Curves ---
            if len(bone_data[0]) >= 2:
                for axis_id in range(4):
                    fcurve = action.fcurves.new(data_path=f"rotation_quaternion", index=axis_id)
                    keyframes = fcurve.keyframe_points
                    keyframes.add(frame_count)

                    # --- Set Keys ---
                    prev_frame = -1
                    prev_q = Quaternion((1, 0, 0, 0))
                    for data_point_id in range(len(bone_data)):
                        data_point = bone_data[data_point_id]
                        if len(data_point) >= 2:
                            frame = data_point[0]
                            value = data_point[1][axis_id]

                            # --- Correct flipped rotation if needed ---
                            try:
                                angle = acos(min(1, max(-1, current_q.dot(prev_q)))) * 2
                                if angle > pi:
                                    q_new = [1, 0, 0, 0]
                                    for i in range(4):
                                        q_new[i] = -current_q[i]
                                    value = q_new[axis_id]
                                    prev_q = q_new
                                else:
                                    prev_q = current_q
                            except:
                                prev_q = current_q
                            if frame > prev_frame:
                                keyframes[data_point_id].co = (frame, value)
                                keyframes[data_point_id].interpolation = interpolation_mode
                                prev_frame = frame

            # --- Location Curves ---
            if len(bone_data[0]) >= 3:
                for axis_id in range(3):
                    fcurve = action.fcurves.new(data_path=f"location", index=axis_id)
                    keyframes = fcurve.keyframe_points
                    keyframes.add(frame_count)

                    # --- Set Keys ---
                    prev_frame = -1
                    for data_point_id in range(len(bone_data)):
                        data_point = bone_data[data_point_id]
                        if len(data_point) >= 3:
                            frame = data_point[0]
                            value = data_point[2][axis_id]
                            if frame > prev_frame:
                                keyframes[data_point_id].co = (frame, value)
                                keyframes[data_point_id].interpolation = interpolation_mode
                                prev_frame = frame

    values_dict.clear()
    return
