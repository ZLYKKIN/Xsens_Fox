# **********************************************************************************************************************
# logger: Operator to log messages to the info bar
# **********************************************************************************************************************

import bpy
import os
import time
import threading
from datetime import datetime


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


# ============================================================================================
# FoxLog: persistent, thread-safe pipeline logger
# ============================================================================================
class FoxLog:
    """
    Thread-safe file logger that records the *entire* MVN streaming pipeline of this
    add-on into ``blender_log.log`` (created next to this file, in the add-on folder).

    The on-disk format is identical to the main application's ``fox_mocap.log`` so the
    whole system (fox_mocap.log + blender_log.log + ue_log.log) can be analysed the same way::

        [+   12.345s] [tag] key=value key=value ...

    ``uptime`` is the number of seconds since the file was last (re)created.  The file is
    truncated/recreated on every stream start via :meth:`reopen`.  Full per-frame detail is
    written: every received segment (raw values) and every bone applied to the armature.

    Logging is best-effort and must never raise: if the file is not open every call is a
    no-op, and all I/O is wrapped so a logging failure can never interrupt the mocap stream.

    Tags (one per pipeline stage): ``boot`` (open/banner), ``net`` (datagram received),
    ``header`` (parsed header), ``decode`` (raw per-segment pose), ``scale`` (T-pose/scale),
    ``pose`` (per-frame summary), ``apply`` (final per-bone transform on the armature),
    ``retarget`` (segment -> bone mapping), ``record`` (keyframes), ``warn``, ``error``,
    ``stop`` (session summary).
    """

    _file = None
    _path = None
    _start = 0.0
    _lock = threading.Lock()
    _last_tick = None
    frames = 0
    drops = 0

    # ---- file lifecycle ----------------------------------------------------------------
    @classmethod
    def default_path(cls) -> str:
        """Default log location: ``blender_log.log`` inside the add-on directory."""
        return os.path.join(os.path.dirname(__file__), "blender_log.log")

    @classmethod
    def reopen(cls, banner: dict = None, path: str = None) -> None:
        """
        (Re)create the log file in truncate mode and write the boot banner.

        Called every time streaming starts so each session begins with a fresh file.
        ``banner`` is an optional ``{key: value}`` dict describing the session
        (address, port, units, gloves, ...).
        """
        with cls._lock:
            try:
                if cls._file is not None:
                    cls._file.close()
            except Exception:
                pass
            cls._file = None
            cls._path = path or cls.default_path()
            cls._start = time.monotonic()
            cls._last_tick = None
            cls.frames = 0
            cls.drops = 0
            try:
                cls._file = open(cls._path, "w", buffering=1, encoding="utf-8")
            except Exception:
                cls._file = None
                return
            cls._emit("boot", "log opened at %s" % cls._path)
            cls._emit("boot", "%s plugin=MVNBlenderPlugin ver=1.0.0" % datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
            if banner:
                cls._emit("boot", cls._join(banner))

    @classmethod
    def close(cls, summary: dict = None) -> None:
        """Write the ``[stop]`` session summary and close the file."""
        with cls._lock:
            if cls._file is None:
                return
            cls._emit("stop", cls._join(summary) if summary else "")
            try:
                cls._file.close()
            except Exception:
                pass
            cls._file = None

    @classmethod
    def is_open(cls) -> bool:
        """True when the file is open. Use to skip building expensive per-frame dumps."""
        return cls._file is not None

    # ---- emit --------------------------------------------------------------------------
    @classmethod
    def _emit(cls, tag: str, body: str) -> None:
        """Write a single timestamped line. MUST be called while holding ``_lock``."""
        f = cls._file
        if f is None:
            return
        try:
            f.write("[+%9.3fs] [%s] %s\n" % (time.monotonic() - cls._start, tag, body))
        except Exception:
            pass

    @classmethod
    def log(cls, tag: str, *args, **fields) -> None:
        """
        Write one event line: ``[+ t] [tag] <args...> k=v k=v``.

        Positional ``args`` are emitted verbatim (free text / pre-formatted tokens),
        keyword ``fields`` are emitted as ``key=value`` with numeric formatting.
        """
        if cls._file is None:
            return
        body = cls._join(fields, *args)
        with cls._lock:
            cls._emit(tag, body)

    @classmethod
    def block(cls, tag: str, header: dict, rows: list) -> None:
        """
        Write a header line followed by indented detail rows.

        Used for full per-frame dumps (every segment / every bone) so a single frame is
        one ``[tag]`` header plus N indented lines, keeping the log greppable.
        """
        if cls._file is None:
            return
        head = cls._join(header)
        indent = " " * 16
        with cls._lock:
            f = cls._file
            if f is None:
                return
            try:
                stamp = time.monotonic() - cls._start
                f.write("[+%9.3fs] [%s] %s\n" % (stamp, tag, head))
                for row in rows:
                    f.write(indent + row + "\n")
            except Exception:
                pass

    # ---- convenience -------------------------------------------------------------------
    @classmethod
    def warn(cls, msg) -> None:
        cls.log("warn", msg)

    @classmethod
    def error(cls, msg) -> None:
        cls.log("error", msg)

    @classmethod
    def count_drop(cls) -> None:
        cls.drops += 1

    @classmethod
    def tick(cls):
        """Return ``(frame_index, dt_ms)`` for per-frame apply logging."""
        now = time.monotonic()
        dt = 0.0 if cls._last_tick is None else (now - cls._last_tick) * 1000.0
        cls._last_tick = now
        cls.frames += 1
        return cls.frames, dt

    # ---- formatting --------------------------------------------------------------------
    @classmethod
    def _join(cls, fields: dict = None, *args) -> str:
        parts = [str(a) for a in args]
        if fields:
            parts.extend("%s=%s" % (k, cls.fmt(v)) for k, v in fields.items())
        return " ".join(parts)

    @staticmethod
    def fmt(v) -> str:
        if isinstance(v, bool):
            return "1" if v else "0"
        if isinstance(v, float):
            return "%.3f" % v
        return str(v)

    @staticmethod
    def vec(v) -> str:
        """Format a 3D vector/sequence as ``(x,y,z)`` with 3 decimals."""
        try:
            return "(%.3f,%.3f,%.3f)" % (v[0], v[1], v[2])
        except Exception:
            return "(?)"

    @staticmethod
    def quat(q) -> str:
        """Format a quaternion/sequence as ``q(w,x,y,z)`` with 4 decimals."""
        try:
            return "q(%.4f,%.4f,%.4f,%.4f)" % (q[0], q[1], q[2], q[3])
        except Exception:
            return "q(?)"


# *********************************************************************************************************************
# Execute
if __name__ == "__main__":

    def register():
        bpy.utils.register_class(LOGGING_OT_logger)

    def unregister():
        bpy.utils.unregister_class(LOGGING_OT_logger)

    register()
    bpy.ops.logging.logger("INVOKE_DEFAULT", message_type="WARNING", message_text="Custom warning issued!")
