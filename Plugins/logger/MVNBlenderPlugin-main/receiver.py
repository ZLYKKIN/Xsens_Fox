# *********************************************************************************************************************
# Receiver: Receive and handle data from MVN Animate Network Streamer
# MVN network streaming protocol documentation:
# https://www.xsens.com/hubfs/Downloads/Manuals/MVN_real-time_network_streaming_protocol_specification.pdf
# *********************************************************************************************************************
import struct
import socket
import threading

import bpy

from .pose import MocapPose
from . import source_manager, source_animator

# ===========================================================================================
HEADER_LENGTH = 24
MAX_PACKET_SIZE = 4096
POSE_QUATERNION_LENGTH = 32
HEADER_DATA_LAYOUT = ">6sIBBIB7x"
POSE_QUATERNION_DATA_LAYOUT = ">I3f4f"

mocap_pose_dict = {}
sock = None
stop_event = threading.Event()
refresh_scene_armatures = True


# ----------------------------------------------------------------------------------------------------------------------------------------
# Socket Management
def get_ready_socket(ip_address: str, port_number: int) -> socket.socket:
    """
    Returns a ready-to-use UDP socket bound to the given IP address and port number.
    """
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((ip_address, port_number))
    return udp_socket


# ----------------------------------------------------------------------------------------------------------------------------------------
# Stream Parsing
def decode_header(encoded_data: bytes) -> tuple:
    """
    Uses the struct module to unpack the header data from the encoded data. The header data includes
    the message ID, datagram counter, number of items, character ID, and more. The function assumes that the encoded
    data is in the format used by the MVN Network streamer.

        Parameters
    ----------
    encoded_data : bytes
        The encoded data containing the header to be decoded.

    Returns
    -------
    tuple
        A tuple containing the unpacked header data.
    """
    unpacked_data = struct.unpack(HEADER_DATA_LAYOUT, encoded_data[:HEADER_LENGTH])
    return unpacked_data


def process_message(encoded_data: bytes, previous_session_target_map: dict) -> None:
    """
    Decodes the header from the encoded data, and then uses the message ID to determine the type of the message.
    Depending on the message type, it then decodes the rest of the message and applies the data to the corresponding MocapPose object.
    """

    # --- Parse header and get mocap pose object ---
    unpacked_header: tuple = decode_header(encoded_data)
    message_id: str = unpacked_header[0][-2:].decode("utf-8")
    character_id: int = unpacked_header[5]
    number_of_items: int = unpacked_header[3]

    mocap_pose: MocapPose = get_mocap_pose(character_id, number_of_items, message_id)

    # --- Handle Messages ---
    if message_id == "02":  # quaternion message
        if not mocap_pose.actor_name:
            return
        quaternion_message: list = decode_quaternion_message(encoded_data, number_of_items)
        apply_quaternion_message(mocap_pose, quaternion_message)

    elif message_id == "12":  # metadata message
        character_metadata_message: dict = decode_character_metadata_message(encoded_data)
        apply_character_metadata_message(mocap_pose, character_metadata_message)

    elif message_id == "13":  # scale message
        datagram_counter = unpacked_header[2]
        character_scale_pose_message: list = decode_character_scale_pose_message(encoded_data, datagram_counter)
        if not character_scale_pose_message:
            return
        apply_scale_message(mocap_pose, character_scale_pose_message, previous_session_target_map)

    elif message_id == "25":  # timecode message
        timecode_message: str = decode_character_timecode_message(encoded_data)
        apply_timecode_message(mocap_pose, timecode_message)
    else:
        print(f"Received unknown message ID: {message_id}")


def process_messages(udp_socket: socket.socket, previous_session_target_map: dict) -> None:
    """
    Continuously processes incoming messages from the MVN Animate Network Streamer.

    Runs in a loop until the stop event is set, receiving data from the network and processing each message.
    If the refresh_scene_armatures flag is set, it removes previous sources from the mocap_pose_dict.
    """
    global refresh_scene_armatures

    while not stop_event.is_set():
        if refresh_scene_armatures:
            refresh_scene_armatures = source_manager.remove_previous_sources(mocap_pose_dict)

        try:
            encoded_data, _ = udp_socket.recvfrom(MAX_PACKET_SIZE)
            if not encoded_data:
                continue

            process_message(encoded_data, previous_session_target_map)

        except socket.error as e:
            if e.errno == 10038:
                print("The socket has been closed.")
                break
            else:
                message = f"Socket error: {e}"
                bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="ERROR", message_text=message)
                break


# ----------------------------------------------------------------------------------------------------------------------------------------
# Decode Specific Message Types
def decode_quaternion_message(encoded_data: bytes, number_of_items: int) -> list:
    """
    Returns a list of tuples containing the unpacked quaternion data from the given encoded data.

    Uses the struct module to unpack the quaternion data from the encoded data. The quaternion data includes
    the segment ID and the quaternion values (x, y, z, w).

    Parameters
    ----------
    encoded_data : bytes
        The encoded data containing the quaternion data to be decoded.
    number_of_items : int
        The number of quaternion items to be decoded from the encoded data.

    Returns
    -------
    list
        A of tuples containing the unpacked quaternion data.
    """
    quaternion_data = []
    current_byte_index: int = HEADER_LENGTH
    data_layout: str = POSE_QUATERNION_DATA_LAYOUT
    segment_length: int = POSE_QUATERNION_LENGTH

    for _ in range(number_of_items):
        data = encoded_data[current_byte_index : current_byte_index + segment_length]
        segment_data = struct.unpack(data_layout, data)
        quaternion_data.append(segment_data)
        current_byte_index += segment_length

    return quaternion_data


def apply_quaternion_message(mocap_pose: MocapPose, quaternion_message: list) -> None:
    """
    Applies the quaternion message to the given MocapPose object.

    Sets the quaternion message of the MocapPose object and processes the quaternion data.
    Depending on whether the MocapPose object is an object cluster or not, it applies the transforms to the trackers or the armature.

    Parameters
    ----------
    mocap_pose : MocapPose
        The MocapPose object to which the quaternion message is to be applied.
    quaternion_message : list
        The quaternion message to be applied.
    """
    mocap_pose.quaternion_message = quaternion_message
    mocap_pose.process_quaternion_data()

    apply_transforms_function = (
        source_animator.apply_transforms_to_trackers
        if mocap_pose.is_object_cluster
        else source_animator.apply_transforms_to_armature
    )
    bpy.app.timers.register(lambda: apply_transforms_function(mocap_pose))


def decode_character_metadata_message(encoded_data: bytes) -> dict:
    """
    Returns a dictionary containing the unpacked metadata from the given encoded data.

    Uses the struct module to unpack the metadata from the encoded data. The metadata includes
    various character properties such as name, color, etc.

    Parameters
    ----------
    encoded_data : bytes
        The encoded data containing the metadata to be decoded.

    Returns
    -------
    dict
        The dictionary containing the unpacked metadata.
    """
    metadata = {}
    current_byte_index = HEADER_LENGTH
    while current_byte_index < len(encoded_data):
        colon_pos = encoded_data.find(b":", current_byte_index)
        newline_pos = encoded_data.find(b"\n", colon_pos)
        if colon_pos == -1 or newline_pos == -1:
            break

        # --- Decode and  strip null bytes ---
        key_name = encoded_data[current_byte_index:colon_pos]
        if b"\x00" in key_name:  # Check for null bytes in key
            current_byte_index = newline_pos + 1
            continue

        key_name = key_name.decode("utf-8").strip()
        key_name = "".join(filter(str.isprintable, key_name))
        key_name = key_name[1:] if not key_name[0].isalpha() else key_name

        key_value = encoded_data[colon_pos + 1 : newline_pos].decode("utf-8").strip()

        metadata[key_name] = key_value
        current_byte_index = newline_pos + 1

    return metadata


def apply_character_metadata_message(mocap_pose: MocapPose, metadata_message: dict) -> None:
    """
    Applies the metadata message to the given MocapPose object.

    Sets the actor name and time offset of the MocapPose object based on the metadata message.
    It also sets the actor color if it is included in the metadata.
    """
    mocap_pose.actor_name = f"MVN:{metadata_message['name']}"
    mocap_pose.time_offset = metadata_message["timeOffset"]
    for key in metadata_message:
        if "color" in key:
            mocap_pose.actor_color = metadata_message[key]
            break


def decode_character_scale_pose_message(encoded_data: bytes, datagram_counter: int) -> list:
    """
    Returns a list of tuples containing the unpacked scale pose data from the given encoded data.

    Uses the struct module to unpack the scale pose data from the encoded data. The scale pose data includes
    the segment name and the x, y, z coordinates.

    Parameters
    ----------
    encoded_data : bytes
        The encoded data containing the scale pose data to be decoded.
    datagram_counter : int
        The datagram counter value from the header of the message.

    Returns
    -------
    list
        The list of tuples containing the unpacked scale pose data.
    """
    scale_vectors = []
    current_byte_index = HEADER_LENGTH

    if not datagram_counter == 0:
        return scale_vectors

    if current_byte_index + 4 <= len(encoded_data):
        segment_count = struct.unpack_from("!I", encoded_data, current_byte_index)[0]
        current_byte_index += 4
        # --- Read segment names and coordinates (4 bytes) ---
        for _ in range(segment_count):
            segment_name_length = struct.unpack_from(">I", encoded_data, current_byte_index)[0]
            current_byte_index += 4
            # --- Extract the segment name based on the length ---
            segment_name = encoded_data[current_byte_index : current_byte_index + segment_name_length].decode("utf-8")
            current_byte_index += segment_name_length
            # ---  Read x, y, z coordinates ---
            x, y, z = struct.unpack_from("!fff", encoded_data, current_byte_index)
            current_byte_index += 12
            scale_vectors.append((segment_name, x, y, z))

        return scale_vectors


def apply_scale_message(mocap_pose: MocapPose, scale_message: list, previous_session_target_map: dict) -> None:
    """
    Applies the scale message to the given MocapPose object.

    Checks if the MocapPose object is an object cluster and if it is, it processes the scale message and creates tracker empties.
    If the MocapPose object is not an object cluster, it checks if the armature object exists. If it doesn't, it processes the scale message and creates a target skeleton.
    If the scale message is different from the existing one, it deletes the target skeleton, processes the new scale message and creates a new target skeleton.

    Parameters
    ----------
    mocap_pose : MocapPose
        The MocapPose object to which the scale message is to be applied.
    scale_message : list
        The scale message to be applied.
    """
    global mocap_pose_dict

    update_skel = False
    try:
        if mocap_pose.armature_object:
            _ = len(mocap_pose.armature_object.pose.bones)
    except Exception:
        print(f"Exception")
        update_skel = True

    if mocap_pose.is_object_cluster and not mocap_pose.create_object_cluster:
        # Check that all objects are still in the scene
        if not all([f"{mocap_pose.actor_name}_{obj}" in bpy.data.objects for obj in mocap_pose.vive_objects]):
            objects = source_manager.get_vive_objects_from_pose(mocap_pose, clear_data=True)
            source_manager.delete_tracker_empties(objects)
            mocap_pose.create_object_cluster = True
            return
        message_objects = [item[0] for item in scale_message]
        if not all([f"{mocap_pose.actor_name}_{obj}" in bpy.data.objects for obj in message_objects]):
            source_manager.delete_tracker_empties(message_objects)
            mocap_pose.create_object_cluster = True
            return

    if mocap_pose.create_object_cluster:
        handle_object_cluster(mocap_pose, scale_message)
    elif not mocap_pose.armature_object and not mocap_pose.is_object_cluster:
        create_target_skeleton(mocap_pose, scale_message, previous_session_target_map)
    elif mocap_pose.scale_message != scale_message and mocap_pose.scale_message is not None:
        update_target_skeleton(mocap_pose, scale_message, previous_session_target_map)
    elif update_skel:
        update_target_skeleton(mocap_pose, scale_message, previous_session_target_map)
    elif mocap_pose.scale_message is None:
        print(f" mocap_pose.scale_message is None for {mocap_pose.character_id} {mocap_pose.actor_name}")

    mocap_pose_dict[mocap_pose.character_id] = mocap_pose


def handle_object_cluster(mocap_pose: MocapPose, scale_message: list) -> None:
    if len(scale_message) > 8:
        if scale_message[0][0] == "Pelvis":
            mocap_pose.is_object_cluster = False
            mocap_pose.create_object_cluster = False
            mocap_pose_dict[mocap_pose.character_id] = mocap_pose
            if existing_vive_objects := source_manager.get_vive_objects_from_scene(mocap_pose_dict):
                source_manager.delete_tracker_empties(existing_vive_objects)
            return
    if scale_message != mocap_pose.scale_message:
        if existing_vive_objects := source_manager.get_vive_objects_from_scene(mocap_pose_dict):
            source_manager.delete_tracker_empties(existing_vive_objects)
    mocap_pose.scale_message = scale_message
    mocap_pose.process_scale_message()
    mocap_pose.is_object_cluster = True
    if mocap_pose.actor_name:
        if existing_vive_objects := source_manager.get_vive_objects_from_scene(mocap_pose_dict):
            source_manager.delete_tracker_empties(existing_vive_objects)
        bpy.app.timers.register(lambda: source_manager.create_tracker_empties(mocap_pose))


def create_target_skeleton(mocap_pose: MocapPose, scale_message: list, previous_session_target_map) -> None:
    if mocap_pose.armature_object and mocap_pose.actor_name in bpy.data.objects:
        return
    mocap_pose.scale_message = scale_message
    mocap_pose.process_scale_message()
    previous_session_target = check_previous_session_targets(mocap_pose.actor_name, previous_session_target_map)
    for armature in bpy.data.armatures:
        print(armature.name, mocap_pose.actor_name)
        if armature.name == mocap_pose.actor_name:
            source_manager.delete_target_skeleton(mocap_pose)
    bpy.app.timers.register(lambda: source_manager.create_target_skeleton(mocap_pose, previous_session_target))


def update_target_skeleton(mocap_pose: MocapPose, scale_message: list, previous_session_target_map) -> None:
    if mocap_pose.actor_name in bpy.data.objects:
        source_manager.delete_target_skeleton(mocap_pose)
    else:
        if len(scale_message) <= 8 and scale_message[0][0] != "Pelvis":
            if existing_vive_objects := source_manager.get_vive_objects_from_scene(mocap_pose_dict):
                source_manager.delete_tracker_empties(existing_vive_objects)
            new_pose = MocapPose(mocap_pose.character_id)
            new_pose.is_object_cluster = True
            new_pose.create_object_cluster = True
            mocap_pose_dict[mocap_pose.character_id] = new_pose
            handle_object_cluster(new_pose, scale_message)
            return
    mocap_pose.scale_message = scale_message
    mocap_pose.process_scale_message()
    if not mocap_pose.create_object_cluster:
        previous_session_target = check_previous_session_targets(mocap_pose.actor_name, previous_session_target_map)
        bpy.app.timers.register(lambda: source_manager.create_target_skeleton(mocap_pose, previous_session_target))


def check_previous_session_targets(actor_name, previous_session_target_map):
    for target_name, source_name in previous_session_target_map.items():
        if actor_name == source_name:
            return target_name
    return None


def decode_character_timecode_message(encoded_data: bytes) -> str:
    """
    Decodes the timecode message from the given encoded data.

    Extracts the timecode from the encoded data. The timecode is a string that represents the current time in the animation.
    The timecode is extracted from the encoded data by finding the specific bytes that represent the timecode and decoding them into a string.
    The function assumes that the encoded data is in the correct format and does not perform any error checking.

    Parameters
    ----------
    encoded_data : bytes
        The encoded data containing the timecode message to be decoded.

    Returns
    -------
    str
        The decoded timecode message.
    """
    current_byte_index = HEADER_LENGTH
    timecode_data = encoded_data[current_byte_index + 4 : current_byte_index + 16]
    timecode_data_string = timecode_data.decode("utf-8", errors="ignore").strip()

    return timecode_data_string


def apply_timecode_message(mocap_pose: MocapPose, timecode_message: str) -> None:
    """
    Sets the timecode of the MocapPose object based on the timecode message.
    The function then calculates the total seconds from the timecode and multiplies it by the frame rate to get the corresponding frame number.
    This frame number is then set as the timecode frame of the MocapPose object.
    """
    mocap_pose.timecode = timecode_message
    fps = mocap_pose.frame_rate
    tc_array = timecode_message.split(":")
    hours = int(tc_array[0])
    minutes = int(tc_array[1])
    seconds = float(tc_array[2])
    total_seconds = (hours * 3600) + (minutes * 60) + seconds
    mocap_pose.tc_frame = round(total_seconds * fps)


def get_mocap_pose(character_id: int, number_of_items: int, message_id: str) -> MocapPose:
    """
    Retrieves or creates a MocapPose object for the given character ID.

    Checks if a MocapPose object for the given character ID already exists in the mocap_pose_dict.
    If it does, the function returns the existing MocapPose object.
    If it doesn't, the function creates a new MocapPose object, sets its properties based on the number of items,
    and adds it to the mocap_pose_dict.

    If the MocapPose object is an object cluster, it deletes tracker empties if they exist.
    If the MocapPose object is not an object cluster and the scale message is different from the existing one,
    it creates a new MocapPose object and sets its properties.

    Parameters
    ----------
    character_id : int
        The ID of the character for which to retrieve or create a MocapPose object.
    number_of_items : int
        The number of items in the message. This is used to determine if the MocapPose object is an object cluster.
    message_id : str
        The ID of the message. This is used to check the type of the message.
    """

    is_object_cluster = number_of_items <= 8

    if message_id == "02" and mocap_pose_dict.get(character_id) and mocap_pose_dict[character_id].scale_message:
        if len(mocap_pose_dict[character_id].scale_message) != number_of_items:
            mocap_pose = MocapPose(character_id)
            if mocap_pose_dict[character_id].is_object_cluster:
                objects = source_manager.get_vive_objects_from_pose(mocap_pose_dict[character_id], clear_data=True)
                source_manager.delete_tracker_empties(objects)

            mocap_pose.is_object_cluster = is_object_cluster
            mocap_pose.create_object_cluster = is_object_cluster
            mocap_pose_dict[character_id] = mocap_pose
            return mocap_pose_dict[character_id]

    if character_id in mocap_pose_dict:
        return mocap_pose_dict[character_id]

    mocap_pose = MocapPose(character_id)

    if is_object_cluster:
        if existing_vive_objects := source_manager.get_vive_objects_from_scene(mocap_pose_dict):
            source_manager.delete_tracker_empties(existing_vive_objects)

    mocap_pose.is_object_cluster = is_object_cluster
    mocap_pose.create_object_cluster = is_object_cluster

    mocap_pose_dict[character_id] = mocap_pose

    return mocap_pose_dict[character_id]


# ----------------------------------------------------------------------------------------------------------------------------------------
# Character Management
def flag_character_to_pause_streaming(character_id: int):
    """
    Checks if a MocapPose object for the given character ID exists in the mocap_pose_dict.
    If it does, and if the MocapPose object's pause_stream attribute is currently False,
    it sets the pause_stream attribute to True. This effectively pauses the streaming of data for the character.
    """
    global mocap_pose_dict

    if character_id in mocap_pose_dict:
        mocap_pose = mocap_pose_dict[character_id]
        if mocap_pose and mocap_pose.pause_stream is False:
            mocap_pose.pause_stream = True


def flag_character_to_resume_streaming(character_id: int):
    """
    Checks if a MocapPose object for the given character ID exists in the mocap_pose_dict. If it does,
    and if the MocapPose object's pause_stream attribute is currently True,
    it sets the pause_stream attribute to False. This effectively resumes the streaming of data for the character.
    """
    global mocap_pose_dict

    if character_id in mocap_pose_dict:
        mocap_pose = mocap_pose_dict[character_id]
        if mocap_pose and mocap_pose.pause_stream is True:
            mocap_pose.pause_stream = False


def flag_character_for_scale_update(character_id: int):
    """
    This function checks if a MocapPose object for the given character ID exists in the mocap_pose_dict.
    If it does, and if the MocapPose object's refresh_scale attribute is currently False,
    it sets the refresh_scale attribute to True. This effectively flags the character for a scale update.
    """
    global mocap_pose_dict

    if character_id in mocap_pose_dict:
        mocap_pose = mocap_pose_dict[character_id]
        if mocap_pose and mocap_pose.refresh_scale is False:
            mocap_pose.refresh_scale = True


# ===========================================================================================
# Main Execution
def start_receiver(ip_address: str, port_number: int, previous_session_target_map: dict) -> None:
    """
    Logs a message indicating the start of the receiver, clears the stop event,
    and attempts to create a ready-to-use UDP socket bound to the given IP address and port number.
    If successful, it starts a new daemon thread that continuously processes incoming messages from the network streamer.
    """
    message = f"Starting MVN Receiver on {ip_address}:{port_number}"
    bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="INFO", message_text=message)
    global sock
    stop_event.clear()

    try:
        sock = get_ready_socket(ip_address, port_number)
        threading.Thread(target=process_messages, args=(sock, previous_session_target_map,), daemon=True).start()
    except socket.error as e:
        if e.errno == 10048:
            message = "ERROR: The specified port is already open by another program or Blender instance."
            bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="ERROR", message_text=message)
        else:
            message = f"Socket error: {e}"
            bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="ERROR", message_text=message)
    except Exception as e:
        message = f"Unexpected error when starting MVN Receiver: {e}"
        bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="ERROR", message_text=message)


def stop_receiver() -> None:
    """
    Logs a message indicating the receiver is stopping, sets the stop event, and attempts to close the UDP socket.
    If successful, it sets the refresh_scene_armatures flag to True.
    If there are any errors during this process, it logs the error message.
    """
    message = "Stopping MVN Receiver..."
    bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="INFO", message_text=message)

    global sock
    global refresh_scene_armatures
    refresh_scene_armatures = True

    try:
        stop_event.set()
    except Exception as e:
        message = f"Error stopping receiver: {e}"
        bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="ERROR", message_text=message)
    if sock is not None and isinstance(sock, socket.socket):
        try:
            sock.close()
        except Exception as e:
            message = f"Error closing socket: {e}"
            bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="ERROR", message_text=message)
        sock = None
    else:
        message = "Socket is not a valid socket object."
        bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="ERROR", message_text=message)


# *********************************************************************************************************************
if __name__ == "__main__":
    ip_address = "localhost"
    port = 9763
    start_receiver(ip_address, port)
