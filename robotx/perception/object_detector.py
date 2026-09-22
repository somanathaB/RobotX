"""Frame -> object detections. Detection only: no decisions, no actuation.

Backends:
- `opencv` : CPU-only. MOG2 background subtraction for moving obstacles, with a
             Canny/contour fallback for static ones. No model download needed.
- `yolo`   : YOLOv8 via `ultralytics` (optional dependency; not installed here).
- `auto`   : prefer `opencv` on the Pi, fall back to `yolo` if OpenCV init fails.

This module returns plain dicts (`label`, `confidence`, `bbox`, `area`) so that
both the production pipeline (which converts them to `PerceptionResult`) and
the retained experimental pipeline can consume the same detector. It does not
decide anything, does not print, and does not write files -- an earlier version
did all three from inside the hot path.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class DetectorConfig:
    """Tuning for the detector. Built from `Settings` by the caller."""

    backend: str = "opencv"
    min_conf: float = 0.35
    yolo_model_path: str = "yolov8n.pt"

    # Motion path (MOG2)
    mog2_history: int = 250
    mog2_var_threshold: float = 32.0
    mog2_warmup_frames: int = 20
    mog2_min_area_px: int = 4000
    min_motion_ratio: float = 0.001

    # Static path (Canny + contours)
    static_canny1: int = 60
    static_canny2: int = 160
    static_min_area_px: int = 2000
    static_min_wh: Tuple[int, int] = (30, 30)
    static_aspect_min: float = 0.2
    static_aspect_max: float = 5.0
    static_min_extent: float = 0.10

    equalize: bool = True

    @classmethod
    def from_settings(cls, settings: Any) -> "DetectorConfig":
        return cls(
            backend=settings.detection_backend,
            min_conf=settings.detection_min_conf,
            yolo_model_path=settings.yolo_model_path,
            mog2_history=settings.detector_mog2_history,
            mog2_var_threshold=settings.detector_mog2_var_threshold,
            mog2_warmup_frames=settings.detector_mog2_warmup_frames,
            mog2_min_area_px=settings.detector_mog2_min_area_px,
            min_motion_ratio=settings.detector_min_motion_ratio,
            static_canny1=settings.detector_static_canny1,
            static_canny2=settings.detector_static_canny2,
            static_min_area_px=settings.detector_static_min_area_px,
            equalize=settings.detector_equalize,
        )


# --- Output filtering ---------------------------------------------------------
# Applied at the detector boundary so every consumer sees the same clean list.

HARD_MIN_AREA = 800
HARD_MAX_AREA = 200000

# Reject background-like boxes that span most of the frame.
FULL_SPAN_RATIO = 0.80
VERTICAL_SPAN_Y1_RATIO = 0.05
VERTICAL_SPAN_Y2_RATIO = 0.95

ASPECT_RATIO_MIN = 0.15
ASPECT_RATIO_MAX = 6.0

MAX_DETECTIONS_KEEP = 3


def _clip_bbox(bbox: Sequence[int], w: int, h: int) -> Tuple[int, int, int, int]:
    x1, y1, x2, y2 = [int(v) for v in bbox]
    x1 = max(0, min(int(w) - 1, x1))
    y1 = max(0, min(int(h) - 1, y1))
    x2 = max(0, min(int(w), x2))
    y2 = max(0, min(int(h), y2))
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    return int(x1), int(y1), int(x2), int(y2)


def _bbox_area(bbox: Sequence[int]) -> int:
    x1, y1, x2, y2 = [int(v) for v in bbox]
    return int(max(0, x2 - x1) * max(0, y2 - y1))


def filter_detections(
    detections: Sequence[Dict[str, Any]],
    *,
    frame_w: int,
    frame_h: int,
    max_keep: int = MAX_DETECTIONS_KEEP,
) -> List[Dict[str, Any]]:
    """Drop implausible boxes and keep the top-ranked ones.

    Rejects: full-frame spans, boxes outside the hard area band, boxes whose raw
    coordinates fall outside the frame, and extreme aspect ratios. Survivors are
    ranked by `area * confidence` and truncated to `max_keep`.
    """

    w = int(max(1, frame_w))
    h = int(max(1, frame_h))

    out: List[Dict[str, Any]] = []
    for d in detections:
        if not isinstance(d, dict):
            continue

        bbox = d.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            continue
        try:
            ox1, oy1, ox2, oy2 = [int(v) for v in bbox]
        except (TypeError, ValueError):
            continue

        x1, y1, x2, y2 = _clip_bbox((ox1, oy1, ox2, oy2), w=w, h=h)
        bw = max(0, x2 - x1)
        bh = max(0, y2 - y1)
        if bw <= 0 or bh <= 0:
            continue

        # Background-like: spans most of the frame in either axis.
        if bw >= FULL_SPAN_RATIO * w or bh >= FULL_SPAN_RATIO * h:
            continue
        # Background-like: spans nearly the full vertical extent.
        if y1 <= VERTICAL_SPAN_Y1_RATIO * h and y2 >= VERTICAL_SPAN_Y2_RATIO * h:
            continue

        area = bw * bh
        if area < HARD_MIN_AREA or area > HARD_MAX_AREA:
            continue

        # Raw box extended past the frame: it was clipped, so its true extent is
        # unknown. Drop rather than report a truncated size as real.
        if ox1 < 0 or oy1 < 0 or ox2 > w or oy2 > h:
            continue

        ratio = bw / max(1, bh)
        if ratio < ASPECT_RATIO_MIN or ratio > ASPECT_RATIO_MAX:
            continue

        try:
            confidence = float(d.get("confidence", 0.0) or 0.0)
        except (TypeError, ValueError):
            confidence = 0.0

        out.append(
            {
                "label": str(d.get("label", "obstacle")) or "obstacle",
                "confidence": confidence,
                "bbox": (x1, y1, x2, y2),
                "area": int(area),
            }
        )

    out.sort(key=lambda dd: (dd["area"] * dd["confidence"], dd["area"]), reverse=True)
    return out[: int(max_keep)]


class ObjectDetector:
    """Runs one detection backend over a frame and returns filtered detections."""

    def __init__(self, cfg: Optional[DetectorConfig] = None) -> None:
        self.cfg = cfg or DetectorConfig()
        self.backend = self.cfg.backend.strip().lower()
        self.min_conf = float(self.cfg.min_conf)

        # Per-call diagnostic counters, for the manual bench scripts.
        self.last_debug: Dict[str, Any] = {}

        self._hog = None
        self._yolo = None
        self._bg = None
        self._bg_frames = 0

        if self.backend == "opencv":
            self._init_opencv()
        elif self.backend == "yolo":
            self._init_yolo()
        elif self.backend == "auto":
            # On the Pi, OpenCV is the realistic default: YOLO needs a torch
            # build that is not installed here.
            try:
                self._init_opencv()
                self.backend = "opencv"
            except Exception:
                logger.warning("OpenCV detector init failed; falling back to YOLO")
                self._init_yolo()
                self.backend = "yolo"
        else:
            raise ValueError(f"Unsupported detection backend: {self.cfg.backend!r}")

        logger.info("Object detector ready (backend=%s)", self.backend)

    # --- backend initialization ---------------------------------------------

    def _init_opencv(self) -> None:
        import cv2  # type: ignore

        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self._hog = hog

        self._bg = cv2.createBackgroundSubtractorMOG2(
            history=max(50, int(self.cfg.mog2_history)),
            varThreshold=float(self.cfg.mog2_var_threshold),
            detectShadows=False,
        )
        self._bg_frames = 0

    def _init_yolo(self) -> None:
        try:
            from ultralytics import YOLO  # type: ignore
        except Exception as e:  # pragma: no cover - optional dependency
            raise RuntimeError(
                "YOLO backend requested but `ultralytics` is not installed"
            ) from e

        self._yolo = YOLO(self.cfg.yolo_model_path)

    # --- public API ----------------------------------------------------------

    def detect_objects(self, frame) -> List[Dict[str, Any]]:
        """Detect objects in a BGR frame. Returns filtered detection dicts."""

        if frame is None:
            return []

        h, w = frame.shape[:2]
        self.last_debug = {"backend": self.backend}

        raw = self._detect_opencv(frame) if self.backend == "opencv" else self._detect_yolo(frame)
        final = filter_detections(raw, frame_w=int(w), frame_h=int(h))

        self.last_debug["raw_detections"] = len(raw)
        self.last_debug["final_detections"] = len(final)
        return final

    # --- OpenCV backend ------------------------------------------------------

    def _detect_opencv(self, frame) -> List[Dict[str, Any]]:
        import cv2  # type: ignore

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
        if self.cfg.equalize:
            try:
                gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
            except cv2.error:
                logger.debug("CLAHE unavailable; continuing without equalization")

        if self._bg is None:
            self.last_debug["mode"] = "STATIC"
            return self._detect_static_edges(frame)

        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
        try:
            fg = self._bg.apply(gray_blur)
        except cv2.error as e:
            # Fail closed: report no motion and fall back to the static path.
            logger.warning("Background subtraction failed: %s", e)
            self.last_debug["mode"] = "STATIC"
            self.last_debug["motion_detected"] = False
            return self._detect_static_edges(frame)

        # MOG2 reports most of the frame as foreground until the model settles.
        self._bg_frames += 1
        if self._bg_frames <= max(0, int(self.cfg.mog2_warmup_frames)):
            self.last_debug["mode"] = "STATIC"
            self.last_debug["motion_detected"] = False
            self.last_debug["warmup"] = True
            return self._detect_static_edges(frame)

        _, mask = cv2.threshold(fg, 200, 255, cv2.THRESH_BINARY)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

        h, w = frame.shape[:2]
        motion_px = int(cv2.countNonZero(mask))
        min_motion_px = int(self.cfg.min_motion_ratio * max(1, w * h))
        if motion_px < min_motion_px:
            self.last_debug["mode"] = "STATIC"
            self.last_debug["motion_detected"] = False
            return self._detect_static_edges(frame)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.last_debug["motion_contours"] = len(contours)

        # A single blob is usually sensor noise rather than a real moving object.
        if len(contours) < 2:
            self.last_debug["mode"] = "STATIC"
            self.last_debug["motion_detected"] = False
            return self._detect_static_edges(frame)

        self.last_debug["mode"] = "MOTION"
        self.last_debug["motion_detected"] = True

        min_w, min_h = self.cfg.static_min_wh
        out: List[Dict[str, Any]] = []
        for c in contours:
            x, y, cw, ch = cv2.boundingRect(c)
            # The top of the frame is sky/ceiling: not drivable space.
            if y < int(0.3 * h):
                continue
            if cw * ch < int(self.cfg.mog2_min_area_px):
                continue
            if cw < int(min_w) or ch < int(min_h):
                continue

            bbox = _clip_bbox((x, y, x + cw, y + ch), w=w, h=h)
            out.append(
                {
                    # Confidence is a fixed heuristic: motion implies salience,
                    # but this backend has no per-object confidence to report.
                    "label": "obstacle",
                    "confidence": 0.60,
                    "bbox": bbox,
                    "area": _bbox_area(bbox),
                }
            )

        self.last_debug["motion_kept"] = len(out)
        return out

    def _detect_static_edges(self, frame) -> List[Dict[str, Any]]:
        """Coarse static-obstacle detection: grayscale -> Canny -> contours."""

        import cv2  # type: ignore

        if frame is None:
            return []
        h, w = frame.shape[:2]
        if h <= 0 or w <= 0:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, int(self.cfg.static_canny1), int(self.cfg.static_canny2))
        edges = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.last_debug["static_contours"] = len(contours)

        min_w, min_h = self.cfg.static_min_wh
        out: List[Dict[str, Any]] = []
        for c in contours:
            x, y, cw, ch = cv2.boundingRect(c)
            if cw <= 0 or ch <= 0:
                continue

            contour_area = float(cv2.contourArea(c))
            if contour_area < float(self.cfg.static_min_area_px):
                continue
            if cw < int(min_w) or ch < int(min_h):
                continue

            ratio = cw / max(1, ch)
            if ratio < self.cfg.static_aspect_min or ratio > self.cfg.static_aspect_max:
                continue

            # Contours touching the frame border are usually walls/floor edges.
            if x <= 0 or y <= 0 or (x + cw) >= w or (y + ch) >= h:
                continue

            bbox_area = cw * ch
            if contour_area / max(1.0, float(bbox_area)) < self.cfg.static_min_extent:
                continue

            out.append(
                {
                    "label": "obstacle",
                    "confidence": 0.45,
                    "bbox": (int(x), int(y), int(x + cw), int(y + ch)),
                    "area": int(bbox_area),
                }
            )

        self.last_debug["static_kept"] = len(out)
        return out

    # --- YOLO backend --------------------------------------------------------

    def _detect_yolo(self, frame) -> List[Dict[str, Any]]:
        model = self._yolo
        if model is None:
            return []

        h, w = frame.shape[:2]
        results = model.predict(frame, verbose=False)

        out: List[Dict[str, Any]] = []
        for r in results:
            names = getattr(r, "names", {})
            boxes = getattr(r, "boxes", None)
            if boxes is None:
                continue

            for b in boxes:
                conf = float(getattr(b, "conf", 0.0))
                if conf < self.min_conf:
                    continue

                xyxy = getattr(b, "xyxy", None)
                if xyxy is None:
                    continue
                x1, y1, x2, y2 = (int(v) for v in xyxy[0].tolist())
                bbox = _clip_bbox((x1, y1, x2, y2), w=w, h=h)

                cls = int(getattr(b, "cls", -1))
                name = str(names.get(cls, "object"))
                label = "person" if name == "person" else "obstacle"

                out.append(
                    {
                        "label": label,
                        "confidence": conf,
                        "bbox": bbox,
                        "area": _bbox_area(bbox),
                    }
                )
        return out


def summarize_detections(detections: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Legacy dict-level summary, used by the retained direct-drive controller."""

    return {
        "person": any(d.get("label") == "person" for d in detections),
        "obstacle": any(d.get("label") == "obstacle" for d in detections),
        "count": len(detections),
    }


def draw_detections(frame, detections: Sequence[Dict[str, Any]]) -> None:
    """Draw boxes onto `frame` in place. Used by the manual bench scripts."""

    if frame is None:
        return
    try:
        import cv2  # type: ignore
    except ImportError:
        return

    h, w = frame.shape[:2]
    for d in detections:
        bbox = d.get("bbox")
        if not bbox or len(bbox) != 4:
            continue

        x1, y1, x2, y2 = _clip_bbox(bbox, w=w, h=h)
        label = str(d.get("label", "object"))
        conf = float(d.get("confidence", 0.0))
        color = (0, 0, 255) if label == "person" else (0, 255, 255)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            frame,
            f"{label} {conf:.2f}",
            (x1, max(15, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )
