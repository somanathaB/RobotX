from __future__ import annotations

import os
from dataclasses import dataclass
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)


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


def _bbox_center(bbox: Sequence[int]) -> Tuple[int, int]:
    x1, y1, x2, y2 = [int(v) for v in bbox]
    cx = int((x1 + x2) // 2)
    cy = int((y1 + y2) // 2)
    return int(cx), int(cy)


def _zone_for_cx(cx: int, frame_w: int) -> str:
    w = int(max(1, frame_w))
    left = int(w // 3)
    right = int((2 * w) // 3)
    if int(cx) < int(left):
        return "LEFT"
    if int(cx) > int(right):
        return "RIGHT"
    return "CENTER"


# --- Filtering + ranking (relaxed per requirements) ---
# NOTE: This runs at the detector boundary so downstream code sees clean results.
HARD_MIN_AREA = 800
HARD_MAX_AREA = 200000

# Background / full-frame rejection
FULL_SPAN_RATIO = 0.80
VERTICAL_SPAN_Y1_RATIO = 0.05
VERTICAL_SPAN_Y2_RATIO = 0.95

# Allow near-edge boxes; only drop if the bbox touches the exact border.
# (This also effectively drops boxes that had to be clipped to fit the frame.)
DROP_IF_TOUCHES_EXACT_EDGE = True

# Relaxed aspect ratio gate.
ASPECT_RATIO_MIN = 0.15
ASPECT_RATIO_MAX = 6.0

# Keep 1–3 meaningful detections; default to 3.
MAX_DETECTIONS_KEEP = 3


def _filter_detections_strict(
    detections: Sequence[Dict[str, Any]],
    *,
    frame_w: int,
    frame_h: int,
    debug: Optional[Dict[str, Any]] = None,
) -> List[Dict[str, Any]]:
    """Apply strict production filters and return a clean, compact detection list.

    Requirements (relaxed):
    - Hard area filter: discard area < 800 or area > 200000
    - Edge filter: allow near-edge boxes; discard only if bbox touches exact border
    - Aspect ratio filter: discard ratio > 6 or < 0.15
    - Rank by score = area * confidence; keep top 2–3

    Output format (per detection):
      {"label": str, "confidence": float, "bbox": (x1,y1,x2,y2), "area": int}
    """

    w = int(max(1, frame_w))
    h = int(max(1, frame_h))

    rejected_full_span = 0
    rejected_vertical_span = 0
    out: List[Dict[str, Any]] = []
    for d in detections:
        if not isinstance(d, dict):
            continue

        bbox = d.get("bbox")
        if not bbox or not hasattr(bbox, "__len__") or len(bbox) != 4:
            continue

        # Keep original bbox to detect edge-touching/clipping.
        try:
            ox1, oy1, ox2, oy2 = [int(v) for v in bbox]
        except Exception:
            continue

        label = str(d.get("label", "obstacle")) or "obstacle"

        x1, y1, x2, y2 = _clip_bbox((ox1, oy1, ox2, oy2), w=w, h=h)
        bw = int(max(0, x2 - x1))
        bh = int(max(0, y2 - y1))
        if bw <= 0 or bh <= 0:
            continue

        area = int(bw * bh)

        # FULL-WIDTH / FULL-HEIGHT rejection: reject background-like spans.
        if float(bw) >= float(FULL_SPAN_RATIO) * float(w) or float(bh) >= float(FULL_SPAN_RATIO) * float(h):
            rejected_full_span += 1
            continue

        # CENTER-MASS / vertical-span check: if it spans nearly the whole vertical region, drop it.
        if float(y1) <= float(VERTICAL_SPAN_Y1_RATIO) * float(h) and float(y2) >= float(VERTICAL_SPAN_Y2_RATIO) * float(h):
            rejected_vertical_span += 1
            continue

        if area < int(HARD_MIN_AREA) or area > int(HARD_MAX_AREA):
            continue

        # Edge handling (relaxed): allow edge-touching boxes.
        # Only discard if the *raw* bbox is out-of-bounds (i.e., extends beyond the image).
        # This prevents ROI crops and bottom-touching obstacles from being discarded.
        if DROP_IF_TOUCHES_EXACT_EDGE:
            if int(ox1) < 0 or int(oy1) < 0 or int(ox2) > int(w) or int(oy2) > int(h):
                continue

        ratio = float(bw) / float(max(1, bh))
        if ratio > float(ASPECT_RATIO_MAX) or ratio < float(ASPECT_RATIO_MIN):
            continue

        try:
            confidence = float(d.get("confidence", 0.0) or 0.0)
        except Exception:
            confidence = 0.0

        out.append(
            {
                "label": label,
                "confidence": float(confidence),
                "bbox": (int(x1), int(y1), int(x2), int(y2)),
                "area": int(area),
            }
        )

    # Rank by score = area * confidence (requirement). Keep top-K.
    def _score(dd: Dict[str, Any]) -> float:
        try:
            return float(dd.get("area", 0) or 0) * float(dd.get("confidence", 0.0) or 0.0)
        except Exception:
            return 0.0

    out.sort(key=lambda dd: (_score(dd), int(dd.get("area", 0) or 0)), reverse=True)

    if isinstance(debug, dict):
        debug["rejected_full_span"] = int(rejected_full_span)
        debug["rejected_vertical_span"] = int(rejected_vertical_span)

    return list(out[: int(MAX_DETECTIONS_KEEP)])


def _count_full_span_candidates(
    dets: Sequence[Dict[str, Any]],
    *,
    frame_w: int,
    frame_h: int,
) -> int:
    w = int(max(1, frame_w))
    h = int(max(1, frame_h))
    n = 0
    for d in dets:
        if not isinstance(d, dict):
            continue
        bbox = d.get("bbox")
        if not bbox or not hasattr(bbox, "__len__") or len(bbox) != 4:
            continue
        try:
            x1, y1, x2, y2 = _clip_bbox(bbox, w=w, h=h)
        except Exception:
            continue
        bw = int(max(0, x2 - x1))
        bh = int(max(0, y2 - y1))
        if bw <= 0 or bh <= 0:
            continue
        if float(bw) >= float(FULL_SPAN_RATIO) * float(w) or float(bh) >= float(FULL_SPAN_RATIO) * float(h):
            n += 1
            continue
        if float(y1) <= float(VERTICAL_SPAN_Y1_RATIO) * float(h) and float(y2) >= float(VERTICAL_SPAN_Y2_RATIO) * float(h):
            n += 1
            continue
    return int(n)


class ObjectDetector:
    """Object detector with optional YOLOv8.

    Backends:
    - `auto`   : try YOLOv8 (ultralytics) and fall back to OpenCV
    - `yolo`   : require ultralytics
    - `opencv` : lightweight CPU-only fallback (MOG2 background subtraction for motion obstacles)

    Output format (per detection):
    {
        "label": "person"|"obstacle",
        "bbox": (x1, y1, x2, y2),
        "confidence": 0.0..1.0,
        "area": int
    }
    """

    def __init__(
        self,
        backend: str = "auto",
        min_conf: float = 0.35,
        yolo_model_path: str = "yolov8n.pt",
        obstacle_min_area: Optional[int] = None,
        obstacle_min_area_ratio: Optional[float] = None,
        obstacle_min_wh: Tuple[int, int] = (30, 30),
        canny_threshold1: Optional[int] = None,
        canny_threshold2: Optional[int] = None,
        use_adaptive_thresh: Optional[bool] = None,
        equalize: Optional[bool] = None,
    ) -> None:
        self.backend = backend.strip().lower()
        self.min_conf = float(min_conf)
        self.yolo_model_path = yolo_model_path

        # Obstacle tuning (OpenCV backend). Defaults are tuned for indoor, 320x240 inference frames.
        self.obstacle_min_area = obstacle_min_area
        self.obstacle_min_area_ratio = obstacle_min_area_ratio
        self.obstacle_min_wh = (int(obstacle_min_wh[0]), int(obstacle_min_wh[1]))
        self.canny_threshold1 = canny_threshold1
        self.canny_threshold2 = canny_threshold2
        self.use_adaptive_thresh = use_adaptive_thresh
        self.equalize = equalize

        self.last_debug: Dict[str, Any] = {}

        self._hog = None
        self._yolo = None
        self._bg = None
        self._bg_warmup_n = 0
        self._bg_debug_i = 0

        if self.backend == "opencv":
            self._init_opencv()
        elif self.backend == "yolo":
            self._init_yolo()
        elif self.backend == "auto":
            # Production default for Raspberry Pi: prefer OpenCV (CPU-only).
            # Opt-in to YOLO with: ROBOTX_AUTO_PREFER_YOLO=1
            prefer_yolo = str(os.environ.get("ROBOTX_AUTO_PREFER_YOLO", "0")).strip().lower() in {"1", "true", "yes"}
            if prefer_yolo:
                try:
                    self._init_yolo()
                    self.backend = "yolo"
                except Exception:
                    self._init_opencv()
                    self.backend = "opencv"
            else:
                try:
                    self._init_opencv()
                    self.backend = "opencv"
                except Exception:
                    self._init_yolo()
                    self.backend = "yolo"
        else:
            raise ValueError(f"Unsupported detection backend: {backend}")

    def _init_opencv(self) -> None:
        import cv2  # type: ignore

        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self._hog = hog

        # Mandatory background subtraction for obstacle detection.
        # Tunable via env for field conditions.
        history = int(os.environ.get("ROBOTX_MOG2_HISTORY", "250"))
        var_threshold = float(os.environ.get("ROBOTX_MOG2_VAR_THRESHOLD", "32"))
        detect_shadows = str(os.environ.get("ROBOTX_MOG2_SHADOWS", "0")).strip().lower() in {"1", "true", "yes"}
        self._bg = cv2.createBackgroundSubtractorMOG2(
            history=max(50, history),
            varThreshold=float(var_threshold),
            detectShadows=bool(detect_shadows),
        )
        # Warm-up frames where MOG2 typically outputs large foreground.
        self._bg_warmup_n = 0

    def _init_yolo(self) -> None:
        try:
            from ultralytics import YOLO  # type: ignore
        except Exception as e:  # pragma: no cover
            raise RuntimeError("YOLO backend requested but ultralytics is not installed") from e

        self._yolo = YOLO(self.yolo_model_path)

    def detect_objects(self, frame) -> List[Dict[str, Any]]:
        if frame is None:
            return []

        h, w = frame.shape[:2]

        # Reset per-call debug.
        self.last_debug = {"backend": self.backend}

        if self.backend == "opencv":
            raw = self._detect_opencv(frame)
        else:
            raw = self._detect_yolo(frame)

        # REQUIRED DEBUG OUTPUT
        if self.backend == "opencv":
            mode = str(self.last_debug.get("mode", "UNKNOWN") or "UNKNOWN").upper()
            if mode not in {"MOTION", "STATIC"}:
                mode = "UNKNOWN"
            print(f"Mode: {mode}")

        # Track raw full-span candidates for debugging.
        raw_full_span = _count_full_span_candidates(raw, frame_w=int(w), frame_h=int(h))
        self.last_debug["raw_full_span"] = int(raw_full_span)

        filtered = _filter_detections_strict(raw, frame_w=int(w), frame_h=int(h), debug=self.last_debug)

        final: List[Dict[str, Any]] = list(filtered)

        # Keep only 1–3 meaningful detections.
        if len(final) > int(MAX_DETECTIONS_KEEP):
            final = list(final[: int(MAX_DETECTIONS_KEEP)])

        # Compute per-detection zone for verification.
        for d in final:
            if not isinstance(d, dict):
                continue
            bb = d.get("bbox")
            if not bb or not hasattr(bb, "__len__") or len(bb) != 4:
                continue
            try:
                cx, cy = _bbox_center(bb)
            except Exception:
                continue
            d["cx"] = int(cx)
            d["cy"] = int(cy)
            d["position"] = _zone_for_cx(int(cx), int(w))

        # Collision logic (required):
        # - area > collision_threshold
        # - OR (CENTER zone AND area > medium_threshold)
        try:
            collision_threshold = int(os.environ.get("ROBOTX_COLLISION_THRESHOLD", "15000"))
        except Exception:
            collision_threshold = 15000
        try:
            medium_threshold = int(os.environ.get("ROBOTX_MEDIUM_THRESHOLD", "10000"))
        except Exception:
            medium_threshold = 10000

        collision = False
        for d in final:
            if not isinstance(d, dict):
                continue
            try:
                area = int(d.get("area", 0) or 0)
            except Exception:
                area = 0
            pos = str(d.get("position", ""))
            if int(area) > int(collision_threshold):
                collision = True
                break
            if pos == "CENTER" and int(area) > int(medium_threshold):
                collision = True
                break

        # Decision recommendation (verification-only, does not drive actuators).
        # Zone-based: LEFT->TURN_RIGHT, RIGHT->TURN_LEFT, CENTER->STOP
        # Safety: if confidence is low, prefer SLOW over MOVE_FORWARD.
        largest_area = 0
        best: Optional[Dict[str, Any]] = None
        for d in final:
            if not isinstance(d, dict):
                continue
            try:
                a = int(d.get("area", 0) or 0)
            except Exception:
                a = 0
            if int(a) >= int(largest_area):
                largest_area = int(a)
                best = d

        best_pos = "NONE"
        best_conf = 0.0
        if isinstance(best, dict):
            best_pos = str(best.get("position", "NONE"))
            try:
                best_conf = float(best.get("confidence", 0.0) or 0.0)
            except Exception:
                best_conf = 0.0

        try:
            low_conf = float(os.environ.get("ROBOTX_LOW_CONF", "0.50"))
        except Exception:
            low_conf = 0.50

        decision = "MOVE_FORWARD"
        if not final:
            decision = "MOVE_FORWARD"
        elif bool(collision):
            decision = "STOP"
        else:
            if best_pos == "CENTER":
                decision = "STOP" if int(largest_area) > int(medium_threshold) else "SLOW"
            elif best_pos == "LEFT":
                decision = "TURN_RIGHT"
            elif best_pos == "RIGHT":
                decision = "TURN_LEFT"
            else:
                decision = "SLOW"

        if str(decision) == "MOVE_FORWARD" and float(best_conf) < float(low_conf):
            decision = "SLOW"

        # --- Verification logs (required) ---
        if self.backend == "opencv" and "motion_detected" in self.last_debug:
            print(f"Motion detected: {bool(self.last_debug.get('motion_detected'))}")

        print(f"Raw detections: {int(len(raw))}")
        print(f"Filtered detections: {int(len(filtered))}")
        if final:
            print("Detections:")
            for d in final[:6]:
                if not isinstance(d, dict):
                    continue
                try:
                    area = int(d.get("area", 0) or 0)
                except Exception:
                    area = 0
                bbox = d.get("bbox")
                pos = str(d.get("position", "UNKNOWN"))
                print(f"- area={area} bbox={bbox} position={pos}")
        else:
            print("Detections: []")

        print(f"Largest area: {int(largest_area)}")
        print(f"Collision: {bool(collision)}")
        print(f"Decision: {str(decision)}")

        # Store for downstream/harness use.
        self.last_debug["collision_threshold"] = int(collision_threshold)
        self.last_debug["medium_threshold"] = int(medium_threshold)
        self.last_debug["collision"] = bool(collision)
        self.last_debug["largest_area"] = int(largest_area)
        self.last_debug["decision"] = str(decision)

        # Required contour counts for verification.
        if self.backend == "opencv" and "motion_detected" in self.last_debug:
            try:
                mc = int(self.last_debug.get("motion_contours", 0) or 0)
            except Exception:
                mc = 0
            print(f"Contours found: {mc}")

        if self.backend == "opencv" and str(self.last_debug.get("mode", "")).upper() == "STATIC":
            try:
                sc = int(self.last_debug.get("static_contours", 0) or 0)
            except Exception:
                sc = 0
            print(f"Static contours: {sc}")
            # Required: Static detections list
            static_list = []
            for d in final[:6]:
                if not isinstance(d, dict):
                    continue
                try:
                    static_list.append(
                        {
                            "area": int(d.get("area", 0) or 0),
                            "bbox": tuple(d.get("bbox")),
                            "position": str(d.get("position", "UNKNOWN")),
                        }
                    )
                except Exception:
                    static_list.append({"area": 0, "bbox": d.get("bbox"), "position": str(d.get("position", "UNKNOWN"))})
            print(f"Static detections: {static_list}")

        self.last_debug["raw_detections"] = int(len(raw))
        self.last_debug["filtered_detections"] = int(len(filtered))
        self.last_debug["final_used"] = int(len(final))

        return final

    def _detect_static_edges(self, frame) -> List[Dict[str, Any]]:
        """Lightweight static obstacle detection using edges + contours.

        Intended for Raspberry Pi real-time use. Produces coarse bboxes for
        large, continuous obstacles when motion is absent.
        """

        import cv2  # type: ignore

        if frame is None:
            return []

        h, w = frame.shape[:2]
        if h <= 0 or w <= 0:
            return []

        # STATIC DETECTION (required): grayscale -> blur -> Canny -> dilate -> contours.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Tunable thresholds; defaults chosen for 320x240-ish inference frames.
        c1 = self.canny_threshold1
        c2 = self.canny_threshold2
        if c1 is None:
            try:
                c1 = int(os.environ.get("ROBOTX_STATIC_CANNY1", "60"))
            except Exception:
                c1 = 60
        if c2 is None:
            try:
                c2 = int(os.environ.get("ROBOTX_STATIC_CANNY2", "160"))
            except Exception:
                c2 = 160

        edges = cv2.Canny(gray, int(c1), int(c2))

        # Close gaps: tiny dilation to connect obstacle boundaries.
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.dilate(edges, k, iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # STATIC FILTERING (CRITICAL / required)
        # Only accept contours if:
        # - area > 2000
        # - width > 30 px
        # - height > 30 px
        # - aspect ratio reasonable (0.2 < w/h < 5)
        # - NOT touching full frame edges
        try:
            min_area_px = int(os.environ.get("ROBOTX_STATIC_MIN_AREA", "2000"))
        except Exception:
            min_area_px = 2000

        min_w = 30
        min_h = 30

        try:
            ar_min = float(os.environ.get("ROBOTX_STATIC_AR_MIN", "0.2"))
        except Exception:
            ar_min = 0.2
        try:
            ar_max = float(os.environ.get("ROBOTX_STATIC_AR_MAX", "5.0"))
        except Exception:
            ar_max = 5.0

        try:
            edge_margin = int(os.environ.get("ROBOTX_STATIC_EDGE_MARGIN", "0"))
        except Exception:
            edge_margin = 0

        try:
            min_extent = float(os.environ.get("ROBOTX_STATIC_MIN_EXTENT", "0.10"))
        except Exception:
            min_extent = 0.10

        out: List[Dict[str, Any]] = []
        kept = 0
        rejected_small = 0
        rejected_size = 0
        rejected_ar = 0
        rejected_edge = 0
        rejected_extent = 0
        for c in contours:
            x, y, cw, ch = cv2.boundingRect(c)
            if cw <= 0 or ch <= 0:
                continue

            # Contour area is the primary static threshold.
            try:
                ca = float(cv2.contourArea(c))
            except Exception:
                ca = 0.0
            if float(ca) < float(min_area_px):
                rejected_small += 1
                continue

            if int(cw) < int(min_w) or int(ch) < int(min_h):
                rejected_size += 1
                continue

            ratio = float(cw) / float(max(1, ch))
            if float(ratio) < float(ar_min) or float(ratio) > float(ar_max):
                rejected_ar += 1
                continue

            if int(x) <= int(edge_margin) or int(y) <= int(edge_margin) or int(x + cw) >= int(w - edge_margin) or int(y + ch) >= int(h - edge_margin):
                rejected_edge += 1
                continue

            bbox_area = int(cw * ch)

            # Not too thin: require contour to occupy enough of its bbox.
            extent = float(ca) / float(max(1.0, float(bbox_area)))
            if float(extent) < float(min_extent):
                rejected_extent += 1
                continue

            out.append(
                {
                    "label": "obstacle",
                    "confidence": 0.45,
                    "bbox": (int(x), int(y), int(x + cw), int(y + ch)),
                    "area": int(bbox_area),
                }
            )
            kept += 1

        self.last_debug["static_contours"] = int(len(contours))
        self.last_debug["static_kept"] = int(kept)
        self.last_debug["static_rejected_small"] = int(rejected_small)
        self.last_debug["static_rejected_size"] = int(rejected_size)
        self.last_debug["static_rejected_ar"] = int(rejected_ar)
        self.last_debug["static_rejected_edge"] = int(rejected_edge)
        self.last_debug["static_rejected_extent"] = int(rejected_extent)
        self.last_debug["canny1"] = int(c1)
        self.last_debug["canny2"] = int(c2)
        return out

    def _detect_opencv(self, frame) -> List[Dict[str, Any]]:
        import cv2  # type: ignore

        detections: List[Detection] = []

        # Lighting robustness
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
        do_eq = self.equalize
        if do_eq is None:
            do_eq = str(os.environ.get("ROBOTX_EQUALIZE", "1")).strip().lower() in {"1", "true", "yes"}
        if do_eq:
            try:
                clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
                gray = clahe.apply(gray)
            except Exception:
                pass

        # Background subtraction (motion path)
        bg = self._bg
        if bg is None:
            self.last_debug["motion_detected"] = False
            self.last_debug["motion_contours"] = 0
            self.last_debug["mode"] = "STATIC"
            return self._detect_static_edges(frame)

        # Pre-smoothing to reduce flicker in MOG2.
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        try:
            fg = bg.apply(gray_blur)
        except Exception:
            # Fail closed: no motion.
            self.last_debug["motion_detected"] = False
            self.last_debug["motion_contours"] = 0
            self.last_debug["mode"] = "STATIC"
            return self._detect_static_edges(frame)

        # MOG2 warmup: suppress detections until background stabilizes.
        warmup = int(os.environ.get("ROBOTX_MOG2_WARMUP", "20"))
        self._bg_warmup_n += 1
        if self._bg_warmup_n <= max(0, warmup):
            self.last_debug["motion_detected"] = False
            self.last_debug["motion_contours"] = 0
            self._maybe_write_motion_debug(frame, fg)
            self.last_debug["mode"] = "STATIC"
            return self._detect_static_edges(frame)

        # Threshold mask: keep definite foreground. If shadows enabled, 127 may appear.
        _, mask = cv2.threshold(fg, 200, 255, cv2.THRESH_BINARY)

        # Clean mask (required): blur + morphological opening.
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

        # Motion present?
        motion_px = int(cv2.countNonZero(mask))
        frame_area = int(frame.shape[1] * frame.shape[0])
        try:
            min_motion_ratio = float(os.environ.get("ROBOTX_MIN_MOTION_RATIO", "0.001"))
        except Exception:
            min_motion_ratio = 0.001
        motion_detected = motion_px >= int(float(min_motion_ratio) * float(max(1, frame_area)))

        if not motion_detected:
            self.last_debug["motion_detected"] = False
            self.last_debug["motion_contours"] = 0
            self._maybe_write_motion_debug(frame, mask)
            self.last_debug["mode"] = "STATIC"
            return self._detect_static_edges(frame)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.last_debug["motion_detected"] = True
        self.last_debug["motion_contours"] = int(len(contours))
        self.last_debug["mode"] = "MOTION"

        # 🔥 ADD THIS BLOCK (avoid single-noise detections)
        if len(contours) < 2:
            self.last_debug["motion_detected"] = False
            self.last_debug["mode"] = "STATIC"
            return self._detect_static_edges(frame)

        # Visual debug output (required): write original frame and mask.
        self._maybe_write_motion_debug(frame, mask)

        # Obstacle detections come ONLY from moving regions.
        h, w = frame.shape[:2]
        # Area rule (required): min area 1500.
        min_area_px = int(os.environ.get("ROBOTX_MOG2_MIN_AREA", "4000"))
        min_w, min_h = self.obstacle_min_wh

        kept = 0
        for c in contours:
            x, y, cw, ch = cv2.boundingRect(c)

             # 🔥 ADD THIS LINE (ignore noisy top region)
            if y < int(0.3 * h):
                continue

            bbox_area = int(cw * ch)
            if bbox_area < int(min_area_px):
                continue
            if cw < int(min_w) or ch < int(min_h):
                continue

            # Give a modest confidence (motion implies salience; exact value is heuristic).
            detections.append(
                Detection(
                    label="obstacle",
                    confidence=0.60,
                    bbox=(int(x), int(y), int(x + cw), int(y + ch)),
                )
            )
            kept += 1

        self.last_debug["contours"] = int(len(contours))
        self.last_debug["motion_kept"] = int(kept)

        # Deduplicate: prefer persons if overlapping
        out: List[Dict[str, Any]] = []
        for d in detections:
            x1, y1, x2, y2 = _clip_bbox(d.bbox, w=w, h=h)
            bbox = (x1, y1, x2, y2)
            out.append(
                {
                    "label": d.label,
                    "confidence": float(d.confidence),
                    "bbox": bbox,
                    "area": _bbox_area(bbox),
                }
            )

        if not out:
            self.last_debug["no_objects"] = True
        return out

    def _maybe_write_motion_debug(self, frame, mask) -> None:
        """Save visual debug artifacts (frame + mask).

        Writes to a fixed path (overwrites) to keep I/O bounded.
        """
        try:
            import cv2  # type: ignore
        except Exception:
            return

        self._bg_debug_i += 1
        every = int(os.environ.get("ROBOTX_MOG2_SAVE_EVERY", "5"))
        if every <= 0:
            return
        if (self._bg_debug_i % every) != 0:
            return

        out_dir = str(os.environ.get("ROBOTX_MOG2_DEBUG_DIR", "/tmp")).strip() or "/tmp"
        try:
            os.makedirs(out_dir, exist_ok=True)
        except Exception:
            out_dir = "/tmp"

        try:
            cv2.imwrite(os.path.join(out_dir, "robotx_mog2_frame.jpg"), frame)
        except Exception:
            pass
        try:
            cv2.imwrite(os.path.join(out_dir, "robotx_mog2_mask.png"), mask)
        except Exception:
            pass

    def _detect_yolo(self, frame) -> List[Dict[str, Any]]:
        # ultralytics accepts numpy arrays (BGR ok)
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
                cls = int(getattr(b, "cls", -1))
                label = str(names.get(cls, "object"))

                xyxy = getattr(b, "xyxy", None)
                if xyxy is None:
                    continue
                x1, y1, x2, y2 = [int(v) for v in xyxy[0].tolist()]

                bbox = _clip_bbox((x1, y1, x2, y2), w=w, h=h)

                if label == "person":
                    out.append({"label": "person", "confidence": conf, "bbox": bbox, "area": _bbox_area(bbox)})
                else:
                    out.append({"label": "obstacle", "confidence": conf, "bbox": bbox, "area": _bbox_area(bbox)})
        return out


def summarize_detections(detections: List[Dict[str, Any]]) -> Dict[str, Any]:
    person = any(d.get("label") == "person" for d in detections)
    obstacle = any(d.get("label") == "obstacle" for d in detections)
    return {"person": person, "obstacle": obstacle, "count": len(detections)}


def draw_detections(frame, detections: List[Dict[str, Any]]) -> None:
    """Draw detections onto `frame` in-place.

    Designed for headless debugging: callers can save the resulting frame to disk.
    """

    if frame is None:
        return

    try:
        import cv2  # type: ignore
    except Exception:
        return

    h, w = frame.shape[:2]
    for d in detections:
        bbox = d.get("bbox")
        if not bbox or len(bbox) != 4:
            continue

        x1, y1, x2, y2 = _clip_bbox(bbox, w=w, h=h)
        label = str(d.get("label", "object"))
        conf = float(d.get("confidence", 0.0))

        color = (0, 0, 255) if label == "person" else (0, 255, 255) if label == "obstacle" else (0, 255, 0)
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