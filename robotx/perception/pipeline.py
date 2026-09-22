"""The one production perception path: camera frame -> PerceptionResult.

Inference runs on a background thread at `detection_hz` so a slow frame never
stalls the agent loop; the agent only ever reads the most recent result via
`latest()`. If the camera stops producing frames, or inference raises, the
result carries a non-OK status rather than an empty detection list -- "I could
not see" must never be indistinguishable from "I saw nothing".

`robotx/perception/experimental/` holds an older, richer vision pipeline that is
NOT used here. See its README for why it is kept.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional, Protocol, Sequence

from robotx.config.logging_setup import log_event
from robotx.perception.object_detector import DetectorConfig, ObjectDetector
from robotx.perception.types import (
    Detection,
    FrameMetadata,
    PerceptionResult,
    PerceptionStatus,
    detections_from_dicts,
)


logger = logging.getLogger(__name__)


class FrameSource(Protocol):
    """What the pipeline needs from a camera. `CameraStream` satisfies this."""

    def get_frame(self) -> Any: ...

    def last_frame_age_s(self) -> Optional[float]: ...


class Detector(Protocol):
    """What the pipeline needs from a detector."""

    backend: str

    def detect_objects(self, frame) -> Sequence[dict]: ...


@dataclass(frozen=True)
class PipelineConfig:
    detection_hz: float = 2.0
    inference_width: int = 320
    inference_height: int = 240
    stale_after_s: float = 3.0
    # Camera frames older than this mean capture has stalled.
    frame_stale_after_s: float = 2.0

    @classmethod
    def from_settings(cls, settings: Any) -> "PipelineConfig":
        return cls(
            detection_hz=settings.detection_hz,
            inference_width=settings.detection_inference_width,
            inference_height=settings.detection_inference_height,
            stale_after_s=settings.perception_stale_after_s,
            frame_stale_after_s=settings.camera_stale_after_s,
        )


class PerceptionPipeline:
    """Runs detection on the latest camera frame, off the agent's loop thread."""

    def __init__(
        self,
        camera: Optional[FrameSource],
        detector: Optional[Detector],
        cfg: PipelineConfig,
    ) -> None:
        self.camera = camera
        self.detector = detector
        self.cfg = cfg

        self._lock = threading.Lock()
        self._result = PerceptionResult.unavailable(
            PerceptionStatus.NO_FRAME if camera is not None else PerceptionStatus.DISABLED,
            backend=getattr(detector, "backend", "none"),
        )
        self._prev_largest_area = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_status: Optional[PerceptionStatus] = None

    @classmethod
    def build(
        cls,
        camera: Optional[FrameSource],
        settings: Any,
    ) -> "PerceptionPipeline":
        """Construct from settings. Returns a disabled pipeline if it cannot init."""

        cfg = PipelineConfig.from_settings(settings)
        if not settings.perception_enabled or camera is None:
            return cls(camera=None, detector=None, cfg=cfg)

        try:
            detector = ObjectDetector(DetectorConfig.from_settings(settings))
        except Exception as e:
            log_event(
                logger,
                "perception.init_failed",
                "detector could not be initialized; perception disabled",
                level=logging.ERROR,
                error=repr(e),
            )
            return cls(camera=None, detector=None, cfg=cfg)

        return cls(camera=camera, detector=detector, cfg=cfg)

    @property
    def enabled(self) -> bool:
        return self.camera is not None and self.detector is not None

    def start(self) -> None:
        if self._running:
            return
        if not self.enabled:
            log_event(logger, "perception.disabled", "no camera or detector available")
            return

        self._running = True
        self._thread = threading.Thread(
            target=self._loop, name="perception", daemon=True
        )
        self._thread.start()
        log_event(
            logger,
            "perception.started",
            backend=getattr(self.detector, "backend", "unknown"),
            hz=self.cfg.detection_hz,
        )

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                logger.warning("Perception thread did not exit within 2s")
        log_event(logger, "perception.stopped")

    def latest(self) -> PerceptionResult:
        """Most recent result, downgraded to STALE if it has aged out."""

        with self._lock:
            result = self._result

        if result.status is PerceptionStatus.OK:
            age = time.time() - result.timestamp
            if age > self.cfg.stale_after_s:
                return PerceptionResult.unavailable(
                    PerceptionStatus.STALE,
                    backend=result.backend,
                    error=f"last result is {age:.1f}s old",
                )
        return result

    def step_once(self) -> PerceptionResult:
        """Run one cycle synchronously. Used by the loop and by tests."""

        result = self._run_detection()
        self._publish(result)
        return result

    # --- internals -----------------------------------------------------------

    def _loop(self) -> None:
        period = 1.0 / max(0.1, float(self.cfg.detection_hz))
        while self._running:
            started = time.monotonic()
            try:
                self.step_once()
            except Exception:
                # The thread itself must survive anything the detector does.
                log_event(
                    logger,
                    "perception.cycle_failed",
                    "unexpected error in perception cycle",
                    level=logging.ERROR,
                    exc_info=True,
                )
                self._publish(
                    PerceptionResult.unavailable(
                        PerceptionStatus.DETECTOR_ERROR,
                        backend=getattr(self.detector, "backend", "unknown"),
                        error="perception cycle raised",
                    )
                )
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, period - elapsed))

    def _run_detection(self) -> PerceptionResult:
        camera = self.camera
        detector = self.detector
        backend = getattr(detector, "backend", "none")

        if camera is None or detector is None:
            return PerceptionResult.unavailable(PerceptionStatus.DISABLED, backend=backend)

        frame = camera.get_frame()
        if frame is None:
            return PerceptionResult.unavailable(
                PerceptionStatus.NO_FRAME, backend=backend, error="camera returned no frame"
            )

        frame_age = camera.last_frame_age_s()
        if frame_age is not None and frame_age > self.cfg.frame_stale_after_s:
            return PerceptionResult.unavailable(
                PerceptionStatus.NO_FRAME,
                backend=backend,
                error=f"camera frame is {frame_age:.1f}s old",
            )

        height, width = frame.shape[:2]
        started = time.monotonic()
        try:
            small, scale = self._downscale(frame)
            raw = detector.detect_objects(small)
            detections = self._rescale(detections_from_dicts(raw), scale, width, height)
        except Exception as e:
            log_event(
                logger,
                "perception.detector_error",
                "inference failed",
                level=logging.ERROR,
                error=repr(e),
            )
            return PerceptionResult.unavailable(
                PerceptionStatus.DETECTOR_ERROR, backend=backend, error=repr(e)
            )

        processing_ms = (time.monotonic() - started) * 1000.0
        largest = max((d.area_px for d in detections), default=0)
        # Guarded: step_once() may be called from a bench script while the
        # pipeline thread is running, and this is a read-modify-write.
        with self._lock:
            delta = largest - self._prev_largest_area
            self._prev_largest_area = largest

        return PerceptionResult(
            timestamp=time.time(),
            status=PerceptionStatus.OK,
            detections=detections,
            frame=FrameMetadata(width=int(width), height=int(height), age_s=frame_age),
            processing_ms=processing_ms,
            backend=backend,
            largest_area_delta_px=int(delta),
        )

    def _downscale(self, frame):
        """Resize for inference. Returns (frame, (sx, sy)) with original = small * s."""

        height, width = frame.shape[:2]
        target_w = max(1, int(self.cfg.inference_width))
        target_h = max(1, int(self.cfg.inference_height))
        if (width, height) == (target_w, target_h):
            return frame, (1.0, 1.0)

        import cv2  # type: ignore

        small = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)
        return small, (width / target_w, height / target_h)

    @staticmethod
    def _rescale(
        detections: Sequence[Detection], scale, width: int, height: int
    ) -> tuple:
        sx, sy = scale
        if sx == 1.0 and sy == 1.0:
            return tuple(detections)

        out = []
        for d in detections:
            x1, y1, x2, y2 = d.bbox
            x1 = max(0, min(width - 1, int(round(x1 * sx))))
            x2 = max(0, min(width, int(round(x2 * sx))))
            y1 = max(0, min(height - 1, int(round(y1 * sy))))
            y2 = max(0, min(height, int(round(y2 * sy))))
            if x2 <= x1 or y2 <= y1:
                continue
            out.append(
                Detection(
                    label=d.label,
                    confidence=d.confidence,
                    bbox=(x1, y1, x2, y2),
                    area_px=(x2 - x1) * (y2 - y1),
                )
            )
        return tuple(out)

    def _publish(self, result: PerceptionResult) -> None:
        with self._lock:
            self._result = result

        # Log only on transition, never per frame.
        if result.status is not self._last_status:
            if result.status is PerceptionStatus.OK:
                log_event(logger, "perception.ok", backend=result.backend)
            else:
                log_event(
                    logger,
                    "perception.unavailable",
                    level=logging.WARNING,
                    status=result.status.value,
                    error=result.error,
                )
            self._last_status = result.status
