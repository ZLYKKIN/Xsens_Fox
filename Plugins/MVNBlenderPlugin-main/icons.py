# **********************************************************************************************************************
# icons.py: manages the icons used in the plugin
# **********************************************************************************************************************

import os

from bpy.utils import previews

# ============================================================================================
# Variables
ICON_COLLECTION = {}


# ============================================================================================
class Icons:
    """
    A manager class for icons used in the MVN Live Plugin.

    Provides a centralized location to define and access the icons used throughout the plugin.
    The icons are loaded from the "icons" directory located in the same location as this file.
    """

    MVN = "MVN"
    XSENS = "XSENS"
    RECORD = "RECORD"
    STOP = "STOP"

    @staticmethod
    def get(name):
        if name in ICON_COLLECTION:
            return ICON_COLLECTION[name][name].icon_id
        else:
            return None


def register():
    """
    Loads the icons from the "icons" directory and stores them in the ICON_COLLECTION dictionary.
    """
    icons_dir = os.path.join(os.path.dirname(__file__), "icons")
    pcoll = previews.new()

    icon_list = {"MVN": "icon_xsens_white", "XSENS": "icon_xsens_white", "RECORD": "icon_record", "STOP": "icon_stop"}
    for code, file in icon_list.items():
        pcoll.load(code, os.path.join(icons_dir, file + ".png"), "IMAGE")
        ICON_COLLECTION[code] = pcoll


def unregister():
    """
    Removes the loaded icons from the ICON_COLLECTION dictionary and clears it.
    """
    for pcoll in ICON_COLLECTION.values():
        previews.remove(pcoll)
    ICON_COLLECTION.clear()
