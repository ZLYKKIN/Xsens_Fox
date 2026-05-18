# **********************************************************************************************************************
# file_logger.py — append-only file logger for Fox Mocap dual-logging tests.
#
# The official MVN Blender plugin only logs to the Blender info bar via
# `logger.LOGGING_OT_logger`, which is transient and useless for cross-log
# diffing against Fox Mocap's `-test` console dump. This module adds a tiny
# stdlib-only file logger that writes to `<blend_dir>/alllog.txt` and follows
# exactly the same line format Fox emits, so the two logs can be joined on
# `sample=N seg=I` and compared frame-for-frame.
#
# Activation:
#   * `start_logging(path)` — open the file (or fall back to next to the .blend).
#   * `stop_logging()` — flush + close.
#   * `log(line)` — append one line (no newline needed).
#
# The logger is a module-level singleton so receiver.py and pose.py can call
# `from . import file_logger; file_logger.log(...)` from any context (main
# thread or socket-reader thread). All writes are guarded by a lock so the
# socket thread and the bpy.app.timers callback can both write safely.
# **********************************************************************************************************************
import threading
import time
from pathlib import Path
from typing import Optional

import bpy


# ============================================================================================
class _FileLogger:
    """Thread-safe append-only file logger."""

    def __init__(self) -> None:
        self._fh = None
        self._path: Optional[Path] = None
        self._lock = threading.Lock()

    # --------------------------------------------------------------------
    def open(self, path: Optional[str] = None) -> Path:
        """Open the log file. If path is None, fall back to <blend_dir>/alllog.txt."""
        with self._lock:
            if self._fh is not None:
                # Re-open requested — close existing handle first to keep
                # everything in a single file.
                try:
                    self._fh.close()
                except Exception:
                    pass
                self._fh = None
            target = Path(path) if path else self._default_path()
            target.parent.mkdir(parents=True, exist_ok=True)
            self._path = target
            self._fh = open(target, "w", encoding="utf-8", buffering=1)
            # Schema header — tells the diff tool (tools/diff_streams.py) how
            # to parse lines.  Bumping SCHEMA_VERSION below is the protocol
            # for forward/backward compat with the diff tool.
            self._fh.write(
                f"[boot] alllog opened at {target}  (host_t={time.time():.6f})\n"
                f"[schema] version=2 events=raw,recv,converted,axis_pos,"
                f"rule,bone_world,bone_local,rest_pose,apply,start,stop,"
                f"err,gloves_state\n"
            )
            self._fh.flush()
            return target

    # --------------------------------------------------------------------
    def close(self) -> None:
        with self._lock:
            if self._fh is not None:
                try:
                    self._fh.write(
                        f"[boot] alllog closed (host_t={time.time():.6f})\n"
                    )
                    self._fh.flush()
                    self._fh.close()
                except Exception:
                    pass
                self._fh = None

    # --------------------------------------------------------------------
    def log(self, line: str) -> None:
        """Append a single line. Newline is added if missing."""
        with self._lock:
            if self._fh is None:
                return
            try:
                if not line.endswith("\n"):
                    line = line + "\n"
                self._fh.write(line)
            except Exception:
                # File got pulled out from under us — silently stop logging
                # rather than crashing the receiver thread.
                pass

    # --------------------------------------------------------------------
    def flush(self) -> None:
        """Force OS-level flush.  Useful before Blender quits / crashes so
        the diff tool sees a complete trace."""
        with self._lock:
            if self._fh is not None:
                try:
                    self._fh.flush()
                except Exception:
                    pass

    # --------------------------------------------------------------------
    def is_open(self) -> bool:
        return self._fh is not None

    def path(self) -> Optional[Path]:
        return self._path

    # --------------------------------------------------------------------
    @staticmethod
    def _default_path() -> Path:
        """alllog.txt next to the currently open .blend.

        Falls back to the user's home directory when Blender is started
        without a saved scene (running --background --python from a fresh
        Blender, before the test project gets loaded)."""
        blend = bpy.data.filepath
        if blend:
            return Path(blend).parent / "alllog.txt"
        return Path.home() / "fox_mocap_alllog.txt"


# Module-level singleton. Import as `from . import file_logger` and call
# `file_logger.log(...)`.
_INSTANCE = _FileLogger()


# ============================================================================================
def start_logging(path: Optional[str] = None) -> Path:
    """Open the file logger. Returns the resolved path."""
    return _INSTANCE.open(path)


def stop_logging() -> None:
    """Close the file logger."""
    _INSTANCE.close()


def log(line: str) -> None:
    """Append a single line to the log (no-op if logger is closed)."""
    _INSTANCE.log(line)


def is_open() -> bool:
    return _INSTANCE.is_open()


def flush() -> None:
    """Force OS-level flush (no-op if logger is closed)."""
    _INSTANCE.flush()


def current_path() -> Optional[Path]:
    return _INSTANCE.path()
