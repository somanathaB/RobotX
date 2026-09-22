"""Camera acquisition for the Raspberry Pi Camera Module 3 (libcamera/Picamera2).

Responsibilities: open the camera, keep the latest frame available, report
whether capture is actually working, and release the device on shutdown.
Nothing here interprets a frame -- that is `robotx.perception`.

Frames are BGR numpy arrays so OpenCV consumers need no conversion.

The Picamera2 device itself is held by a process-wide, reference-counted manager
because libcamera allows only one open handle per camera: the agent's
`CameraStream` and a bench script running in the same process would otherwise
fight over the device. Each `CameraStream` is a consumer handle, not a second
capture loop.
"""

from __future__ import annotations

import atexit
import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional

from robotx.config.logging_setup import log_event


logger = logging.getLogger(__name__)


class CameraStatus(str, Enum):
    STOPPED = "STOPPED"            # not started (or already shut down)
    STARTING = "STARTING"          # device opened, no frame delivered yet
    STREAMING = "STREAMING"        # delivering frames
    STALLED = "STALLED"            # opened, but frames stopped arriving
    FAILED = "FAILED"              # device could not be opened or capture died


@dataclass(frozen=True)
class CameraConfig:
    width: int = 640
    height: int = 480
    fps: int = 20
    stale_after_s: float = 2.0

    @classmethod
    def from_settings(cls, settings: Any) -> "CameraConfig":
        return cls(
            width=settings.camera_width,
            height=settings.camera_height,
            fps=settings.camera_fps,
            stale_after_s=settings.camera_stale_after_s,
        )


class CameraError(RuntimeError):
    """Raised when the camera cannot be opened. Carries operator guidance."""


class CameraStream:
    """Consumer handle on the shared Picamera2 device."""

    def __init__(
        self,
        cfg: Optional[CameraConfig] = None,
        *,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fps: Optional[int] = None,
    ) -> None:
        base = cfg or CameraConfig()
        self.cfg = CameraConfig(
            width=base.width if width is None else int(width),
            height=base.height if height is None else int(height),
            fps=base.fps if fps is None else int(fps),
            stale_after_s=base.stale_after_s,
        )

        self._manager: Optional[_Picamera2Manager] = None
        self._started = False
        self._error: Optional[str] = None

    @property
    def _active_cfg(self) -> CameraConfig:
        """The configuration actually in force.

        libcamera allows one configuration per process, so a second stream
        requesting a different size gets the first one's frames. Report what
        the device is really delivering, not what this handle asked for.
        """

        return self._manager.cfg if self._manager is not None else self.cfg

    @property
    def width(self) -> int:
        return self._active_cfg.width

    @property
    def height(self) -> int:
        return self._active_cfg.height

    def start(self) -> None:
        """Open the camera. Raises `CameraError` if the device is unusable."""

        if self._started:
            return

        manager = _get_manager(self.cfg)
        try:
            manager.acquire()
        except CameraError as e:
            self._error = str(e)
            log_event(
                logger,
                "camera.failed",
                "camera could not be started",
                level=logging.ERROR,
                error=repr(e),
            )
            raise

        self._manager = manager
        self._started = True
        self._error = None
        log_event(
            logger,
            "camera.connected",
            width=self.cfg.width,
            height=self.cfg.height,
            fps=self.cfg.fps,
        )

    def stop(self) -> None:
        if not self._started:
            return
        self._started = False

        manager = self._manager
        self._manager = None
        if manager is not None:
            manager.release()
        log_event(logger, "camera.disconnected")

    def get_frame(self):
        """Latest frame as a BGR array, or None if none is available yet."""

        if self._manager is None:
            return None
        return self._manager.get_frame()

    def last_frame_age_s(self) -> Optional[float]:
        if self._manager is None:
            return None
        return self._manager.last_frame_age_s()

    def get_status(self) -> CameraStatus:
        if not self._started or self._manager is None:
            return CameraStatus.FAILED if self._error else CameraStatus.STOPPED
        if self._manager.capture_failed:
            return CameraStatus.FAILED

        age = self._manager.last_frame_age_s()
        if age is None:
            return CameraStatus.STARTING
        if age > self.cfg.stale_after_s:
            return CameraStatus.STALLED
        return CameraStatus.STREAMING

    def describe(self) -> Dict[str, Any]:
        age = self.last_frame_age_s()
        cfg = self._active_cfg
        return {
            "status": self.get_status().value,
            "width": cfg.width,
            "height": cfg.height,
            "fps": cfg.fps,
            "last_frame_age_s": None if age is None else round(age, 3),
            "error": self._error or (self._manager.last_error if self._manager else None),
        }

    def get_jpeg(self, quality: int = 80) -> Optional[bytes]:
        """Encode the latest frame as JPEG (used by the local MJPEG endpoint)."""

        frame = self.get_frame()
        if frame is None:
            return None

        import cv2  # type: ignore

        ok, buf = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        )
        return bytes(buf) if ok else None


class _Picamera2Manager:
    """Owns the one Picamera2 handle and its capture thread."""

    def __init__(self, cfg: CameraConfig) -> None:
        self.cfg = cfg
        self._lock = threading.Lock()
        self._users = 0

        self._picam2 = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame = None
        self._last_ok_t = 0.0

        self.capture_failed = False
        self.last_error: Optional[str] = None

    def acquire(self) -> None:
        with self._lock:
            if self._running:
                self._users += 1
                return

            try:
                from picamera2 import Picamera2  # type: ignore
            except ImportError as e:
                raise CameraError(
                    "Picamera2 is required for the Raspberry Pi Camera Module 3. "
                    "Install it with: sudo apt install -y python3-picamera2. "
                    "In a venv, create it with: python3 -m venv --system-site-packages venv"
                ) from e

            try:
                picam2 = Picamera2()
                config = picam2.create_preview_configuration(
                    main={
                        "size": (int(self.cfg.width), int(self.cfg.height)),
                        "format": "RGB888",
                    }
                )
                picam2.configure(config)
                try:
                    picam2.set_controls({"FrameRate": float(self.cfg.fps)})
                except Exception:
                    # Not every sensor/driver exposes FrameRate as a control.
                    logger.debug("Camera FrameRate control unavailable", exc_info=True)
                picam2.start()
            except Exception as e:
                raise CameraError(
                    f"Failed to start Picamera2 ({e}). If this says 'No cameras available', "
                    "libcamera cannot see the camera: run `rpicam-hello --list-cameras` "
                    "and check the ribbon cable and CSI connector."
                ) from e

            self._picam2 = picam2
            self._running = True
            self.capture_failed = False
            self.last_error = None
            self._users += 1
            self._thread = threading.Thread(
                target=self._loop, name="camera-capture", daemon=True
            )
            self._thread.start()

    def release(self) -> None:
        with self._lock:
            self._users = max(0, self._users - 1)
            if self._users > 0:
                return

            self._running = False
            thread, self._thread = self._thread, None
            picam2, self._picam2 = self._picam2, None
            self._frame = None
            self._last_ok_t = 0.0

        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                logger.warning("Camera capture thread did not exit within 2s")

        if picam2 is not None:
            for close_step in (picam2.stop, picam2.close):
                try:
                    close_step()
                except Exception:
                    logger.debug("Ignoring error during camera shutdown", exc_info=True)

    def _loop(self) -> None:
        period = 1.0 / max(1.0, float(self.cfg.fps))
        consecutive_errors = 0

        while True:
            with self._lock:
                if not self._running or self._picam2 is None:
                    return
                picam2 = self._picam2

            try:
                rgb = picam2.capture_array()
            except Exception as e:
                consecutive_errors += 1
                self.last_error = repr(e)
                if consecutive_errors == 1:
                    log_event(
                        logger,
                        "camera.capture_error",
                        "frame capture failed",
                        level=logging.WARNING,
                        error=repr(e),
                    )
                if consecutive_errors >= 10 and not self.capture_failed:
                    # Repeated failures mean the device is gone, not a hiccup.
                    self.capture_failed = True
                    log_event(
                        logger,
                        "camera.failed",
                        "capture failed repeatedly; camera is unusable",
                        level=logging.ERROR,
                        consecutive_errors=consecutive_errors,
                    )
                time.sleep(0.1)
                continue

            if rgb is not None:
                if consecutive_errors:
                    log_event(logger, "camera.recovered", after_errors=consecutive_errors)
                consecutive_errors = 0
                self.capture_failed = False
                bgr = rgb[..., ::-1].copy()
                with self._lock:
                    self._frame = bgr
                    self._last_ok_t = time.monotonic()

            time.sleep(period)

    def get_frame(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def last_frame_age_s(self) -> Optional[float]:
        with self._lock:
            if self._last_ok_t <= 0:
                return None
            return time.monotonic() - self._last_ok_t


_GLOBAL_MANAGER: Optional[_Picamera2Manager] = None
_GLOBAL_MANAGER_LOCK = threading.Lock()


def _get_manager(cfg: CameraConfig) -> _Picamera2Manager:
    global _GLOBAL_MANAGER
    with _GLOBAL_MANAGER_LOCK:
        if _GLOBAL_MANAGER is None:
            _GLOBAL_MANAGER = _Picamera2Manager(cfg)
            atexit.register(_shutdown_global)
        elif (_GLOBAL_MANAGER.cfg.width, _GLOBAL_MANAGER.cfg.height) != (cfg.width, cfg.height):
            logger.warning(
                "Camera already open at %dx%d; ignoring request for %dx%d "
                "(libcamera allows one configuration per process)",
                _GLOBAL_MANAGER.cfg.width,
                _GLOBAL_MANAGER.cfg.height,
                cfg.width,
                cfg.height,
            )
        return _GLOBAL_MANAGER


def _shutdown_global() -> None:
    """atexit safety net: release the device even if a handle leaked."""

    global _GLOBAL_MANAGER
    with _GLOBAL_MANAGER_LOCK:
        manager, _GLOBAL_MANAGER = _GLOBAL_MANAGER, None

    if manager is not None:
        with manager._lock:
            manager._users = min(manager._users, 1)
        try:
            manager.release()
        except Exception:
            logger.debug("Ignoring error during atexit camera shutdown", exc_info=True)
