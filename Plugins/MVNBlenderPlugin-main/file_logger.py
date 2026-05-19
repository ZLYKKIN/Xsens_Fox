import os
import threading
import time
from pathlib import Path
from typing import Optional, Sequence, Tuple

import bpy


SegmentRow = Tuple[int, str, Tuple[float, float, float, float], Tuple[float, float, float]]


class _FileLogger:
    def __init__(self) -> None:
        self._fh = None
        self._path: Optional[Path] = None
        self._lock = threading.Lock()
        self._max_bytes: int = 10 * 1024 * 1024
        self._rotate_index: int = 0
        self._bytes_written: int = 0
        self._precision: int = 3

    def open(self, path: Optional[str] = None,
             max_size_mb: float = 10.0,
             precision: int = 3) -> Path:
        with self._lock:
            if self._fh is not None:
                try:
                    self._fh.close()
                except Exception:
                    pass
                self._fh = None
            target = Path(path) if path else self._default_path()
            target.parent.mkdir(parents=True, exist_ok=True)
            self._path = target
            self._max_bytes = max(1, int(max_size_mb * 1024 * 1024))
            self._precision = max(1, min(6, int(precision)))
            self._rotate_index = 0
            self._bytes_written = 0
            self._fh = open(target, "w", encoding="utf-8", buffering=1)
            header = (
                f"[boot] alllog opened at {target}  (host_t={time.time():.6f})\n"
                f"[schema] version=3 events=start,stop,snapshot,err,gloves_state,rest_pose,apply\n"
            )
            self._fh.write(header)
            self._bytes_written += len(header.encode("utf-8"))
            self._fh.flush()
            return target

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

    def _maybe_rotate_locked(self) -> None:
        if self._fh is None or self._path is None:
            return
        if self._bytes_written < self._max_bytes:
            return
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass
        self._rotate_index += 1
        suffix = self._path.suffix or ".txt"
        stem = self._path.stem
        rotated = self._path.with_name(f"{stem}.{self._rotate_index}{suffix}")
        try:
            if rotated.exists():
                rotated.unlink()
            os.replace(self._path, rotated)
        except Exception:
            pass
        try:
            self._fh = open(self._path, "w", encoding="utf-8", buffering=1)
            header = (
                f"[boot] alllog rotated (prev={rotated.name}) "
                f"(host_t={time.time():.6f})\n"
                f"[schema] version=3 events=start,stop,snapshot,err,gloves_state,rest_pose,apply\n"
            )
            self._fh.write(header)
            self._bytes_written = len(header.encode("utf-8"))
        except Exception:
            self._fh = None
            self._bytes_written = 0

    def log(self, line: str) -> None:
        with self._lock:
            if self._fh is None:
                return
            try:
                if not line.endswith("\n"):
                    line = line + "\n"
                self._fh.write(line)
                self._bytes_written += len(line.encode("utf-8"))
                self._maybe_rotate_locked()
            except Exception:
                pass

    def log_snapshot(self, sample: int, t: float,
                     segments: Sequence[SegmentRow]) -> None:
        prec = self._precision
        qfmt = f"{{:+.{prec}f}}"
        pfmt = f"{{:+.{prec}f}}"
        rows = [
            f"========== [BLENDER SNAPSHOT] sample={sample}  t={t:.2f}s ==========\n"
        ]
        for idx, name, q, p in segments:
            qw, qx, qy, qz = q
            px, py, pz = p
            rows.append(
                f"  seg[{idx:2d}] {name:<14}"
                f" quat=({qfmt.format(qw)},{qfmt.format(qx)},"
                f"{qfmt.format(qy)},{qfmt.format(qz)})"
                f" pos=({pfmt.format(px)},{pfmt.format(py)},"
                f"{pfmt.format(pz)})\n"
            )
        rows.append("=" * 60 + "\n")
        blob = "".join(rows)
        with self._lock:
            if self._fh is None:
                return
            try:
                self._fh.write(blob)
                self._bytes_written += len(blob.encode("utf-8"))
                self._maybe_rotate_locked()
            except Exception:
                pass

    def flush(self) -> None:
        with self._lock:
            if self._fh is not None:
                try:
                    self._fh.flush()
                except Exception:
                    pass

    def is_open(self) -> bool:
        return self._fh is not None

    def path(self) -> Optional[Path]:
        return self._path

    def precision(self) -> int:
        return self._precision

    @staticmethod
    def _default_path() -> Path:
        blend = bpy.data.filepath
        if blend:
            return Path(blend).parent / "alllog.txt"
        return Path.home() / "fox_mocap_alllog.txt"


_INSTANCE = _FileLogger()


def start_logging(path: Optional[str] = None,
                  max_size_mb: float = 10.0,
                  precision: int = 3) -> Path:
    return _INSTANCE.open(path, max_size_mb=max_size_mb, precision=precision)


def stop_logging() -> None:
    _INSTANCE.close()


def log(line: str) -> None:
    _INSTANCE.log(line)


def log_snapshot(sample: int, t: float,
                 segments: Sequence[SegmentRow]) -> None:
    _INSTANCE.log_snapshot(sample, t, segments)


def is_open() -> bool:
    return _INSTANCE.is_open()


def flush() -> None:
    _INSTANCE.flush()


def current_path() -> Optional[Path]:
    return _INSTANCE.path()


def precision() -> int:
    return _INSTANCE.precision()
