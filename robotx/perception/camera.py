import atexit
import threading
import time
from dataclasses import dataclass
from typing import Optional


class CameraStream:
    """Threaded libcamera (Picamera2) camera reader.

    Keeps the latest frame available for consumers (controller, MJPEG stream).

    Notes:
    - Designed for Raspberry Pi Camera Module (libcamera), not legacy stack.
    - Returns frames in OpenCV-friendly BGR format.
    """

    def __init__(self, index: int = 0, width: int = 640, height: int = 480, fps: int = 20):
        # `index` is accepted for backward compatibility but is unused with Picamera2.
        self.index = index
        self.width = width
        self.height = height
        self.fps = fps

        self.opened_index: Optional[int] = None

        self._manager: Optional[_Picamera2Manager] = None
        self._lock = threading.Lock()
        self._frame = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_ok_t = 0.0

    def start(self) -> None:
        if self._running:
            return
        self._running = True

        self._manager = _get_manager(width=self.width, height=self.height, fps=self.fps)
        self._manager.acquire()

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._manager is not None:
            self._manager.release()
            self._manager = None

    def _loop(self) -> None:
        target_period = 1.0 / max(1.0, float(self.fps))
        while self._running:
            if self._manager is None:
                time.sleep(0.2)
                continue

            frame = self._manager.get_frame()
            if frame is not None:
                with self._lock:
                    self._frame = frame
                self._last_ok_t = time.monotonic()
            else:
                time.sleep(0.05)

            time.sleep(target_period)

    def get_frame(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def last_frame_age_s(self) -> Optional[float]:
        if self._last_ok_t <= 0:
            return None
        return time.monotonic() - self._last_ok_t

    def get_jpeg(self, quality: int = 80) -> Optional[bytes]:
        frame = self.get_frame()
        if frame is None:
            return None

        import cv2  # type: ignore

        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        if not ok:
            return None
        return bytes(buf)


@dataclass
class _Picamera2Config:
    width: int
    height: int
    fps: int


class _Picamera2Manager:
    def __init__(self, cfg: _Picamera2Config) -> None:
        self.cfg = cfg
        self._lock = threading.Lock()
        self._users = 0

        self._picam2 = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame = None
        self._last_ok_t = 0.0

    def acquire(self) -> None:
        with self._lock:
            self._users += 1
            if self._running:
                return

            try:
                from picamera2 import Picamera2  # type: ignore
            except Exception as e:
                self._users -= 1
                raise RuntimeError(
                    "Picamera2 is required for Raspberry Pi Camera Module 3 (libcamera). "
                    "Install it with: sudo apt install -y python3-picamera2\n"
                    "If you're using a venv, create it with system packages: python3 -m venv --system-site-packages venv"
                ) from e

            try:
                picam2 = Picamera2()
                config = picam2.create_preview_configuration(
                    main={"size": (int(self.cfg.width), int(self.cfg.height)), "format": "RGB888"}
                )
                picam2.configure(config)
                try:
                    picam2.set_controls({"FrameRate": float(self.cfg.fps)})
                except Exception:
                    # Not all drivers/platforms expose this control.
                    pass
                picam2.start()
            except Exception as e:
                self._users -= 1
                raise RuntimeError(
                    "Failed to start Picamera2. If you see 'No cameras available', the camera is not detected by libcamera. "
                    "Run: rpicam-hello --list-cameras and check the ribbon cable / camera connector."
                ) from e

            self._picam2 = picam2
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def release(self) -> None:
        with self._lock:
            self._users = max(0, self._users - 1)
            if self._users > 0:
                return

            self._running = False
            thread = self._thread
            self._thread = None
            picam2 = self._picam2
            self._picam2 = None

        if thread is not None:
            thread.join(timeout=1.0)

        if picam2 is not None:
            try:
                picam2.stop()
            except Exception:
                pass
            try:
                picam2.close()
            except Exception:
                pass

    def _loop(self) -> None:
        target_period = 1.0 / max(1.0, float(self.cfg.fps))
        while True:
            with self._lock:
                if not self._running or self._picam2 is None:
                    break
                picam2 = self._picam2

            try:
                rgb = picam2.capture_array()
                if rgb is not None:
                    bgr = rgb[..., ::-1].copy()
                    with self._lock:
                        self._frame = bgr
                        self._last_ok_t = time.monotonic()
            except Exception:
                time.sleep(0.05)

            time.sleep(target_period)

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
_DEFAULT_STREAM: Optional[CameraStream] = None
_DEFAULT_STREAM_LOCK = threading.Lock()


def _get_manager(width: int, height: int, fps: int) -> _Picamera2Manager:
    global _GLOBAL_MANAGER
    with _GLOBAL_MANAGER_LOCK:
        if _GLOBAL_MANAGER is None:
            _GLOBAL_MANAGER = _Picamera2Manager(_Picamera2Config(width=int(width), height=int(height), fps=int(fps)))
            atexit.register(_shutdown_global)
        return _GLOBAL_MANAGER


def _shutdown_global() -> None:
    global _GLOBAL_MANAGER
    global _DEFAULT_STREAM

    with _DEFAULT_STREAM_LOCK:
        stream = _DEFAULT_STREAM
        _DEFAULT_STREAM = None
    if stream is not None:
        try:
            stream.stop()
        except Exception:
            pass

    with _GLOBAL_MANAGER_LOCK:
        mgr = _GLOBAL_MANAGER
        _GLOBAL_MANAGER = None
    if mgr is not None:
        try:
            # Force cleanup even if users leaked.
            mgr._users = 1
            mgr.release()
        except Exception:
            pass


def get_frame(width: int = 640, height: int = 480, fps: int = 20):
    """Convenience API: returns latest frame (BGR) from a persistent singleton camera."""

    global _DEFAULT_STREAM
    with _DEFAULT_STREAM_LOCK:
        if _DEFAULT_STREAM is None:
            _DEFAULT_STREAM = CameraStream(index=0, width=int(width), height=int(height), fps=int(fps))
            _DEFAULT_STREAM.start()
        return _DEFAULT_STREAM.get_frame()