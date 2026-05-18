# **********************************************************************************************************************
# logger: Operator to log messages to the info bar
# **********************************************************************************************************************

import bpy
import time


# ============================================================================================
class LOGGING_OT_logger(bpy.types.Operator):
    """
    A class used to log messages to the Blender info bar.

    This class is a subclass of bpy.types.Operator and is used to create a custom operator in Blender.
    The operator logs messages of various types (INFO, WARNING, ERROR) to the Blender info bar.

    Attributes
    ----------
    bl_idname : str
        The unique identifier for this operator.
    bl_label : str
        The user-friendly name of this operator.
    _timer : Timer
        A timer used to control the frequency of logging.
    _last_log_time : float
        The time the last log message was sent.
    _interval : float
        The interval between log messages.
    message_type : EnumProperty
        The type of message to report (INFO, WARNING, ERROR).
    message_text : StringProperty
        The custom text to display in the log message.

    Methods
    -------
    modal(context, event) -> set
        The modal operator function. Called by Blender in response to events.
    execute(context) -> set
        The execute function. Called when the operator is run.
    cancel(context)
        The cancel function. Called when the operator is cancelled.
    """

    bl_idname = "logging.logger"
    bl_label = "Log Messages To Info Bar"

    _timer = None
    _last_log_time = 0
    _interval = 0

    # Message type and text properties
    message_type: bpy.props.EnumProperty(
        name="Message Type",
        description="Type of message to report",
        items=[
            ("INFO", "Info", "Informational message"),
            ("WARNING", "Warning", "Warning message"),
            ("ERROR", "Error", "Error message"),
        ],
        default="INFO",
    )

    message_text: bpy.props.StringProperty(
        name="Message Text", description="Custom text to display", default="Default condition met message"
    )

    def modal(self, context, event):
        """
        The modal operator function. Called by Blender in response to events.
        If the event type is 'TIMER' and the time elapsed since the last log is greater than the interval,
        it reports the message and cancels the operator.
        """
        if event.type == "TIMER":
            current_time = time.time()
            if current_time - self._last_log_time > self._interval:
                self._last_log_time = current_time
                self.report({self.message_type}, self.message_text)
                self.cancel(context)  # Cancel the operator after reporting once
                return {"FINISHED"}

        return {"PASS_THROUGH"}

    def execute(self, context):
        """
        Called when the operator is run.
        It sets the last log time to the current time minus the interval, adds an event timer, and adds the operator to
        the window manager's modal handlers.
        """
        self._last_log_time = time.time() - self._interval
        self._timer = context.window_manager.event_timer_add(1.0, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def cancel(self, context):
        """
        Called when the operator is cancelled.
        It removes the event timer from the window manager.
        """
        if self._timer:
            context.window_manager.event_timer_remove(self._timer)
            self._timer = None


# *********************************************************************************************************************
# Execute
if __name__ == "__main__":

    def register():
        bpy.utils.register_class(LOGGING_OT_logger)

    def unregister():
        bpy.utils.unregister_class(LOGGING_OT_logger)

    register()
    bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="WARNING", message_text="Custom warning issued!")
