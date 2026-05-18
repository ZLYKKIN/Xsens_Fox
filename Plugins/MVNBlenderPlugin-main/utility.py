# **********************************************************************************************************************
# utility.py: Helper functions for changing mode, deselecting objects, and adding to collections in Blender
# **********************************************************************************************************************
import bpy

# ============================================================================================
# Constants
MVN_COLLECTION = "MVN Collection"


# =============================================================================================
def switch_mode(mode: str, obj=None) -> str:
    """
    Switches the mode of the given object in Blender.

    Ensures that the object and its collection are visible. It then deselects all objects,
    selects the given object, and sets it as the active object. If the object's current mode is not the desired mode,
    it switches the object to the desired mode. Finally, it deselects all objects again.

    Parameters
    ----------
    mode : str
        The mode to switch the object to. This should be one of Blender's object modes, such as 'EDIT', 'OBJECT', etc.
    obj : bpy.types.Object, optional
        The object to switch the mode of. If not provided, the function will operate on the currently active object.

    Returns
    -------
    str
        The previous mode of the object before the switch.
    """
    for collection in obj.users_collection:
        collection.hide_viewport = False
    if obj.hide_get():
        obj.hide_viewport = False
        obj.hide_set(False)
    if bpy.ops.object.select_all.poll():
        bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    prev_mode = bpy.context.object.mode
    if bpy.context.object.mode != mode:
        bpy.ops.object.mode_set(mode=mode)
    if bpy.ops.object.select_all.poll():
        bpy.ops.object.select_all(action="DESELECT")

    return prev_mode


def deselect():
    """
    Deselects all objects in the Blender scene.

    Checks if the deselect all operation can be performed. If it can, it deselects all objects in the scene.
    """
    if bpy.ops.object.select_all.poll():
        bpy.ops.object.select_all(action="DESELECT")


def add_to_collection(obj, collection_name=MVN_COLLECTION, remove_old=True):
    """
    Adds an object to a specified collection in Blender.

    Checks if the specified collection exists. If it doesn't, it creates a new collection with the
    specified name and links it to the current scene. If the collection does exist, it checks if the object is not None
    and then proceeds to unlink the object from its current collections (if remove_old is True) and link it to the new
    collection.

    Parameters
    ----------
    obj : bpy.types.Object
        The object to add to the collection.
    collection_name : str, optional
        The name of the collection to add the object to. If not provided, the function will use 'MVN Collection' as the default.
    remove_old : bool, optional
        Whether to remove the object from its current collections before adding it to the new one. If not provided, the function will default to True.
    """
    collection = bpy.data.collections.get(collection_name)

    if collection is None:
        collection = bpy.data.collections.new(collection_name)
        bpy.context.scene.collection.children.link(collection)

    if collection is not None:
        if obj is not None:
            # --- Remove from current collection ---
            if obj.users_collection and remove_old:
                for old_collection in obj.users_collection:
                    old_collection.objects.unlink(obj)
            # --- Add to new collection ---
            if obj.name not in collection.objects:
                collection.objects.link(obj)


# *********************************************************************************************************************
# Main
if __name__ == "__main__":
    pass
