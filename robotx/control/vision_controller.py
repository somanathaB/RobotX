from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from robotx.perception.camera import CameraStream
from robotx.perception.detection import ObjectDetector
from robotx.perception.filter import TemporalFilter, TemporalFilterConfig
from robotx.perception.tracking import ObjectTracker, PrimaryObjectTracker, PrimaryTrack, Track


# --- Fixed production thresholds / rules (per requirements) ---
MIN_VALID_AREA = 1500
CLOSE_THRESHOLD = 30000
MEDIUM_THRESHOLD = 10000

# Motion-depth proxy (single camera): stop early if area is increasing quickly.
# Tunable via env var for field calibration.
FAST_APPROACH_THRESHOLD = int(os.environ.get("ROBOTX_FAST_APPROACH_THRESHOLD", "4000"))

# ROI: process lower 70% of the frame by default.
# Override at runtime:
# - Disable ROI: ROBOTX_DISABLE_ROI=1
# - Set ratio:   ROBOTX_ROI_TOP_RATIO=0.2 (top 20% ignored)
ROI_TOP_RATIO = 0.30

# Anti-flicker decision buffer.
DECISION_WINDOW = 5
DECISION_MIN_COUNT = 3


# --- Obstacle avoidance (continuous turning) ---
# Collision zone definition for a 640px-wide frame (given in requirements).
# We compute using ratios so it stays correct if the camera width changes.
FRAME_WIDTH_REF = 640
CENTER_ZONE_LEFT_REF = 213
CENTER_ZONE_RIGHT_REF = 426

CENTER_ZONE_LEFT_RATIO = float(CENTER_ZONE_LEFT_REF) / float(FRAME_WIDTH_REF)
CENTER_ZONE_RIGHT_RATIO = float(CENTER_ZONE_RIGHT_REF) / float(FRAME_WIDTH_REF)

# Hysteresis (prevents flicker). Keep small to remain responsive.
ENTER_TURN_FRAMES = 2
EXIT_TURN_FRAMES = 3

# When obstacle center is "exactly" in the middle, treat it as centered.
CENTER_EPS_PX = 5


def _safe_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return int(default)


def select_primary_detection(detections: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Pick the single primary detection by max `area`.

    Deterministic: ties resolve by first seen.
    """

    if not detections:
        return None
    best: Optional[Dict[str, Any]] = None
    best_area = -1
    for d in detections:
        a = _safe_int(d.get("area"), 0)
        if a > best_area:
            best_area = a
            best = d
    return best




def _center_zone_for_width(frame_width: int) -> Tuple[int, int]:
    w = int(max(1, frame_width))
    left = int(round(CENTER_ZONE_LEFT_RATIO * float(w)))
    right = int(round(CENTER_ZONE_RIGHT_RATIO * float(w)))
    left = max(0, min(w - 1, left))
    right = max(0, min(w, right))
    if right < left:
        left, right = right, left
    return left, right


def _bbox_center_x(bbox: Any) -> Optional[int]:
    if not bbox or len(bbox) != 4:
        return None
    try:
        x1, _, x2, _ = [int(v) for v in bbox]
    except Exception:
        return None
    return int(round((float(x1) + float(x2)) / 2.0))


def draw_avoidance_debug(
    frame,
    *,
    primary: Optional[Dict[str, Any]],
    action: str,
    primary_track: Optional[Dict[str, Any]] = None,
) -> None:
    """Optional visual debug overlay.

    Draws:
    - Collision (center) zone as a red rectangle
    - Primary object center point (blue)
    - Zone label (LEFT/CENTER/RIGHT)
    - State + action text
    """

    if frame is None:
        return
    try:
        import cv2  # type: ignore
    except Exception:
        return

    h, w = frame.shape[:2]
    left, right = _center_zone_for_width(int(w))
    cv2.rectangle(frame, (left, 0), (right, int(h) - 1), (0, 0, 255), 2)

    zone = "NONE"
    if isinstance(primary, dict):
        bbox = primary.get("bbox")
        cx = _bbox_center_x(bbox)
        if cx is not None:
            if int(cx) < int(left):
                zone = "LEFT"
            elif int(cx) > int(right):
                zone = "RIGHT"
            else:
                zone = "CENTER"

            # Blue center point.
            try:
                _, y1, _, y2 = [int(v) for v in bbox]
                cy = int((int(y1) + int(y2)) // 2)
                cv2.circle(frame, (int(cx), int(cy)), 6, (255, 0, 0), -1)
            except Exception:
                pass

    cv2.putText(frame, f"ZONE: {zone}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"ACTION: {str(action)}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # Motion-depth visual debug: area + delta + trend arrow.
    if isinstance(primary_track, dict):
        try:
            a = int(primary_track.get("area", 0) or 0)
        except Exception:
            a = 0
        try:
            da = int(primary_track.get("delta_area", 0) or 0)
        except Exception:
            da = 0
        trend = str(primary_track.get("trend", ""))

        cv2.putText(frame, f"AREA: {a}  DELTA: {da:+d}", (20, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"TREND: {trend}", (20, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Arrow: approaching=up, away=down, stable=right.
        x0, y0 = 320, 220
        if trend == "APPROACHING":
            x1, y1 = x0, y0 - 35
        elif trend == "AWAY":
            x1, y1 = x0, y0 + 35
        else:
            x1, y1 = x0 + 45, y0
        cv2.arrowedLine(frame, (int(x0), int(y0)), (int(x1), int(y1)), (255, 255, 0), 2, tipLength=0.25)


class DecisionBuffer:
    """Anti-flicker buffer: confirm an action only if repeated >= N times in last window."""

    def __init__(self, window: int = DECISION_WINDOW, min_count: int = DECISION_MIN_COUNT) -> None:
        from collections import deque

        self.window = int(max(1, window))
        self.min_count = int(max(1, min_count))
        self._buf = deque(maxlen=self.window)
        self._last = "MOVE_FORWARD"

    @property
    def last(self) -> str:
        return str(self._last)

    def update(self, proposed: str, *, has_detection: bool) -> str:
        proposed = str(proposed)

        if not has_detection:
            self._buf.clear()
            self._last = "MOVE_FORWARD"
            return "MOVE_FORWARD"

        self._buf.append(proposed)
        counts: Dict[str, int] = {}
        for a in self._buf:
            counts[a] = int(counts.get(a, 0)) + 1

        # Confirm the highest-count action; prefer more restrictive if tied.
        rank = {"MOVE_FORWARD": 0, "SLOW": 1, "TURN_LEFT": 1, "TURN_RIGHT": 1, "STOP": 2}
        best = None
        best_c = 0
        for a, c in counts.items():
            if int(c) < int(self.min_count):
                continue
            if best is None:
                best, best_c = str(a), int(c)
                continue
            if int(c) > int(best_c) or (int(c) == int(best_c) and rank.get(a, 1) > rank.get(best, 1)):
                best, best_c = str(a), int(c)

        if best is not None:
            self._last = best
            return best

        # Not enough consistency: keep previous; fail-safe away from MOVE_FORWARD.
        if str(self._last) == "MOVE_FORWARD":
            return "SLOW"
        return str(self._last)


@dataclass(frozen=True)
class VisionControllerConfig:
    backend: str = "auto"  # auto | opencv | yolo
    yolo_model_path: str = "yolov8n.pt"
    min_conf: float = 0.35
    detection_hz: float = 5.0

    # Performance: run detection on a smaller frame and scale bboxes back.
    inference_width: int = 320
    inference_height: int = 240

    # Decision thresholds (area is a distance proxy).
    # Defaults match the required production thresholds.
    close_threshold: int = CLOSE_THRESHOLD
    medium_threshold: int = MEDIUM_THRESHOLD

    # Back-compat (deprecated): ratio-based thresholds are no longer used.
    stop_area_ratio: float = 0.12
    slow_area_ratio: float = 0.05

    # Tracking / stability
    track_iou_threshold: float = 0.3
    track_max_missed: int = 8
    track_min_hits: int = 2
    track_bbox_ema_alpha: float = 0.6

    temporal_window: int = 6
    temporal_min_presence: int = 3

    # Action stability: require N consecutive identical raw decisions to change
    # into a *less* restrictive action. STOP transitions are immediate.
    action_stability_frames: int = 3

    # Back-compat (deprecated): no longer used.
    action_smooth_window: int = 5


class VisionController:
    """Production-style vision perception + decision pipeline.

    Camera -> Detection -> Tracking -> Temporal Filtering -> Decision
    
    Output action is one of: STOP, SLOW, MOVE_FORWARD.
    Safe for headless usage (no GUI calls).
    """

    def __init__(
        self,
        cfg: VisionControllerConfig,
        camera_index: int = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 20,
    ) -> None:
        self.cfg = cfg
        self.camera = CameraStream(index=camera_index, width=width, height=height, fps=fps)
        self.camera.start()

        self.detector = self._create_detector(cfg)

        self.tracker = ObjectTracker(
            iou_threshold=cfg.track_iou_threshold,
            max_missed=cfg.track_max_missed,
            min_hits=cfg.track_min_hits,
            bbox_ema_alpha=cfg.track_bbox_ema_alpha,
        )
        self.temporal = TemporalFilter(
            TemporalFilterConfig(window_size=cfg.temporal_window, min_presence=cfg.temporal_min_presence)
        )

        # Lightweight primary-object tracking (for smooth control decisions).
        self.primary_tracker = PrimaryObjectTracker(
            # Light mode: lower smoothing and shorter history.
            iou_threshold=max(0.10, min(0.30, float(cfg.track_iou_threshold))),
            bbox_ema_alpha=min(0.35, max(0.10, float(cfg.track_bbox_ema_alpha))),
            history_len=3,
            max_missed=2,
        )

        self._decision_buf = DecisionBuffer(window=DECISION_WINDOW, min_count=DECISION_MIN_COUNT)

        # Continuous-avoidance state machine.
        self.current_state: str = "FORWARD"  # FORWARD | TURN_LEFT | TURN_RIGHT
        self._turn_last: str = "TURN_RIGHT"  # tie-break when perfectly centered
        self._enter_collision_n: int = 0
        self._exit_collision_n: int = 0
        self._invalid_n: int = 0

        self._last_det_t = 0.0
        self._last_detections: List[Dict[str, Any]] = []
        self._last_detections_inference: List[Dict[str, Any]] = []
        self._last_tracks: List[Track] = []
        self._last_confirmed: List[Track] = []
        self._last_stable: List[Track] = []
        # Hard rule: default to MOVE_FORWARD when no objects.
        self._last_action_raw: str = "MOVE_FORWARD"
        self._last_action: str = "MOVE_FORWARD"

        self._last_primary_inference: Optional[Dict[str, Any]] = None
        self._last_primary_scaled: Optional[Dict[str, Any]] = None
        self._last_primary_track: Optional[PrimaryTrack] = None
        self._last_roi: Optional[Tuple[int, int, int, int]] = None  # (x1,y1,x2,y2) in full-res frame

    def close(self) -> None:
        self.camera.stop()

    @staticmethod
    def _create_detector(cfg: VisionControllerConfig) -> ObjectDetector:
        backend = cfg.backend.strip().lower()
        return ObjectDetector(backend=backend, min_conf=cfg.min_conf, yolo_model_path=cfg.yolo_model_path)

    def step(self) -> Dict[str, Any]:
        frame = self.camera.get_frame()
        if frame is None:
            # Camera failure: be safe.
            return {
                "frame": None,
                "detections": [],
                "tracks": [],
                "confirmed": [],
                "stable": [],
                "stable_labels": [],
                "action_raw": "STOP",
                "action": "STOP",
                "backend": getattr(self.detector, "backend", "unknown"),
                "detector_ran": False,
            }

        now = time.monotonic()
        min_period = 1.0 / max(0.1, float(self.cfg.detection_hz))
        detector_ran = False
        if now - self._last_det_t >= min_period:
            detector_ran = True
            try:
                det_frame, scale = self._prepare_inference_frame(frame)

                # ROI crop on the inference frame (lower 70%).
                det_h, det_w = det_frame.shape[:2]
                disable_roi = str(os.environ.get("ROBOTX_DISABLE_ROI", "0")).strip().lower() in {"1", "true", "yes"}
                roi_top_ratio = ROI_TOP_RATIO
                env_roi = os.environ.get("ROBOTX_ROI_TOP_RATIO")
                if env_roi:
                    try:
                        roi_top_ratio = float(env_roi)
                    except Exception:
                        roi_top_ratio = ROI_TOP_RATIO
                roi_top_ratio = float(max(0.0, min(0.95, roi_top_ratio)))

                roi_y0 = 0 if disable_roi else int(round(float(det_h) * float(roi_top_ratio)))
                roi_y0 = max(0, min(det_h - 1, roi_y0))
                roi_frame = det_frame[roi_y0:det_h, 0:det_w]

                # Track ROI in full-res coords for visualization.
                sx, sy = scale
                full_y0 = int(round(float(roi_y0) * float(sy)))
                self._last_roi = (0, int(full_y0), int(frame.shape[1]), int(frame.shape[0]))

                dets = self.detector.detect_objects(roi_frame)

                # Adjust bboxes from ROI coords -> full inference-frame coords.
                dets_adj: List[Dict[str, Any]] = []
                for d in dets:
                    bb = d.get("bbox")
                    if not bb or len(bb) != 4:
                        continue
                    x1, y1, x2, y2 = [int(v) for v in bb]
                    y1 += int(roi_y0)
                    y2 += int(roi_y0)
                    if x2 <= x1 or y2 <= y1:
                        continue
                    dets_adj.append(
                        {
                            "label": str(d.get("label", "object")),
                            "confidence": float(d.get("confidence", 0.0)),
                            "bbox": (int(x1), int(y1), int(x2), int(y2)),
                            "area": int(max(0, (x2 - x1) * (y2 - y1))),
                        }
                    )

                # Keep the inference-scale detections for area-threshold decisions.
                self._last_detections_inference = list(dets_adj)
                self._last_detections = self._scale_detections(dets_adj, scale=scale, frame_shape=frame.shape)
            except Exception as e:
                # Never crash the main loop on detector failures.
                self._last_detections = []
                self._last_detections_inference = []
                self._last_action_raw = "SLOW"
                self._last_action = "SLOW"
                return {
                    "frame": frame,
                    "detections": [],
                    "tracks": [],
                    "confirmed": [],
                    "stable": [],
                    "stable_labels": [],
                    "action_raw": self._last_action_raw,
                    "action": self._last_action,
                    "backend": getattr(self.detector, "backend", "unknown"),
                    "detector_ran": True,
                    "error": f"detection failed: {e}",
                    "detector_debug": getattr(self.detector, "last_debug", {}),
                }

            self._last_tracks = self.tracker.update(self._last_detections, now=now)
            self._last_confirmed = self.tracker.confirmed_tracks()
            filt = self.temporal.update(self._last_confirmed)
            self._last_stable = list(filt.get("stable_tracks") or [])
            stable_labels = list(filt.get("stable_labels") or [])

            # Primary selection + lightweight tracking (uses full-res scaled detections).
            # Soft filtering: rank by effective area (area * confidence) instead of discarding.
            def eff(d: Dict[str, Any]) -> int:
                try:
                    return int(round(float(d.get("area", 0) or 0) * float(d.get("confidence", 0.0) or 0.0)))
                except Exception:
                    return 0

            sorted_by_eff = sorted(self._last_detections, key=eff, reverse=True)
            selected = [d for d in sorted_by_eff[:3] if isinstance(d, dict)]

            self._last_primary_inference = select_primary_detection(self._last_detections_inference)
            self._last_primary_scaled = select_primary_detection(selected)

            # Track using up to top-3 objects; tracker chooses max-area internally.
            track = self.primary_tracker.update(selected)
            self._last_primary_track = track

            # Soft-select the best obstacle (used for collision zone + actions).
            best_obs: Optional[Dict[str, Any]] = None
            best_eff = -1
            for d in self._last_detections:
                if str(d.get("label", "")) != "obstacle":
                    continue
                e = eff(d)
                if e > best_eff:
                    best_eff = e
                    best_obs = d

            obs_area = 0
            obs_cx: Optional[int] = None
            obs_bbox = None
            if isinstance(best_obs, dict):
                obs_bbox = best_obs.get("bbox")
                try:
                    obs_area = int(best_obs.get("area") or 0)
                except Exception:
                    obs_area = 0
                if obs_bbox and len(obs_bbox) == 4:
                    try:
                        x1, y1, x2, y2 = [int(v) for v in obs_bbox]
                        obs_cx = int(round((float(x1) + float(x2)) / 2.0))
                    except Exception:
                        obs_cx = None

            total_dets = int(len(self._last_detections))
            largest_raw = 0
            for d in self._last_detections:
                try:
                    largest_raw = max(largest_raw, int(d.get("area") or 0))
                except Exception:
                    pass

            frame_h, frame_w = frame.shape[:2]
            zone_left, zone_right = _center_zone_for_width(int(frame_w))

            obstacle_valid = (best_obs is not None) and int(obs_area) >= int(MIN_VALID_AREA) and (obs_cx is not None)

            in_collision = bool(obstacle_valid and int(zone_left) <= int(obs_cx) <= int(zone_right))

            position = "NONE"
            if obstacle_valid and obs_cx is not None:
                if int(obs_cx) < int(zone_left):
                    position = "LEFT"
                elif int(obs_cx) > int(zone_right):
                    position = "RIGHT"
                else:
                    position = "CENTER"
            elif best_obs is not None and obs_cx is None:
                position = "UNKNOWN"

            # Decision priority (production-safe):
            # - Person -> STOP
            # - Valid obstacle in center zone -> TURN_LEFT/RIGHT continuously (hysteresis)
            # - Else obstacle area -> STOP/SLOW/MOVE_FORWARD
            # - Uncertain -> SLOW
            action = "MOVE_FORWARD"

            # --- Motion-depth proxy from primary track (single camera, no known size) ---
            motion_area = int(obs_area)
            motion_delta = 0
            motion_status = "NONE"
            if isinstance(track, PrimaryTrack) and str(track.label) == "obstacle":
                motion_area = int(track.area)
                motion_delta = int(track.delta_area)
                motion_status = str(track.trend)

            # Person detection (highest priority) using all detections.
            person_seen = False
            for d in self._last_detections:
                if str(d.get("label", "")) == "person":
                    try:
                        if float(d.get("confidence", 0.0) or 0.0) >= float(self.cfg.min_conf):
                            person_seen = True
                            break
                    except Exception:
                        person_seen = True
                        break

            if person_seen:
                self.current_state = "FORWARD"
                self._enter_collision_n = 0
                self._exit_collision_n = 0
                self._invalid_n = 0
                action = "STOP"
            elif not self._last_detections:
                self._enter_collision_n = 0
                self._exit_collision_n = 0
                self._invalid_n = 0
                self.current_state = "FORWARD"
                action = "MOVE_FORWARD"
            elif not obstacle_valid:
                # Detections exist but no valid obstacle center/area -> fail-safe.
                self._invalid_n += 1
                if self.current_state in {"TURN_LEFT", "TURN_RIGHT"} and self._invalid_n < int(EXIT_TURN_FRAMES):
                    action = str(self.current_state)
                else:
                    self.current_state = "FORWARD"
                    action = "MOVE_FORWARD"
                    self._enter_collision_n = 0
                    self._exit_collision_n = 0
            else:
                self._invalid_n = 0

                # Safety override: stop if very close OR fast approaching.
                if int(motion_area) > int(CLOSE_THRESHOLD) or int(motion_delta) > int(FAST_APPROACH_THRESHOLD):
                    self.current_state = "FORWARD"
                    self._enter_collision_n = 0
                    self._exit_collision_n = 0
                    action = "STOP"
                else:

                    # Continuous avoidance state machine.
                    if self.current_state == "FORWARD":
                        if in_collision:
                            self._enter_collision_n += 1
                        else:
                            self._enter_collision_n = 0

                        if self._enter_collision_n >= int(ENTER_TURN_FRAMES):
                            centerline = int(round(float(frame_w) / 2.0))
                            if abs(int(obs_cx) - int(centerline)) <= int(CENTER_EPS_PX):
                                turn = str(self._turn_last or "TURN_RIGHT")
                            elif int(obs_cx) < int(centerline):
                                turn = "TURN_RIGHT"
                            else:
                                turn = "TURN_LEFT"

                            self._turn_last = turn
                            self.current_state = turn
                            self._exit_collision_n = 0
                            action = turn
                        else:
                            # Required decision logic (area + delta):
                            # - area > CLOSE_THRESHOLD -> STOP
                            # - delta_area > FAST_APPROACH_THRESHOLD -> STOP
                            # - area > MEDIUM_THRESHOLD -> SLOW
                            # - else -> MOVE_FORWARD
                            if int(motion_area) > int(CLOSE_THRESHOLD):
                                action = "STOP"
                            elif int(motion_delta) > int(FAST_APPROACH_THRESHOLD):
                                action = "STOP"
                            elif int(motion_area) > int(MEDIUM_THRESHOLD):
                                action = "SLOW"
                            else:
                                action = "MOVE_FORWARD"

                    elif self.current_state == "TURN_LEFT":
                        if in_collision:
                            self._exit_collision_n = 0
                            action = "TURN_LEFT"
                        else:
                            self._exit_collision_n += 1
                            if self._exit_collision_n >= int(EXIT_TURN_FRAMES):
                                self.current_state = "FORWARD"
                                self._enter_collision_n = 0
                                action = "MOVE_FORWARD"
                            else:
                                action = "TURN_LEFT"

                    elif self.current_state == "TURN_RIGHT":
                        if in_collision:
                            self._exit_collision_n = 0
                            action = "TURN_RIGHT"
                        else:
                            self._exit_collision_n += 1
                            if self._exit_collision_n >= int(EXIT_TURN_FRAMES):
                                self.current_state = "FORWARD"
                                self._enter_collision_n = 0
                                action = "MOVE_FORWARD"
                            else:
                                action = "TURN_RIGHT"
                    else:
                        self.current_state = "FORWARD"
                        action = "SLOW"

            # Anti-flicker buffer: confirm decisions, otherwise hold previous / fail-safe to SLOW.
            if str(action) in {"TURN_LEFT", "TURN_RIGHT"}:
                self._decision_buf._buf.clear()  # local helper: keep turns immediate
                self._decision_buf._last = str(action)
            else:
                action = self._decision_buf.update(action, has_detection=bool(self._last_detections))

            self._last_action_raw = str(action)
            self._last_action = str(action)

            # Required debug output for motion-depth.
            print(f"Area: {int(motion_area)}")
            print(f"Delta: {int(motion_delta):+d}")
            print(f"Status: {str(motion_status)}")
            print(f"Action: {self._last_action}")

            # Required debug output.
            sel_dbg = []
            for d in selected:
                try:
                    sel_dbg.append(
                        {
                            "label": str(d.get("label", "object")),
                            "area": int(d.get("area", 0) or 0),
                            "conf": float(d.get("confidence", 0.0) or 0.0),
                            "eff": int(eff(d)),
                        }
                    )
                except Exception:
                    pass

            print(f"Total detections: {int(total_dets)}")
            print(f"Largest area: {int(largest_raw)}")
            print(f"Selected objects: {sel_dbg}")
            print(f"Max area: {int(obs_area)}")
            print(f"Position: {position}")
            print(f"In collision: {bool(in_collision)}")
            print(f"State: {self.current_state}")
            # (Action already printed above in motion-depth block.)
            self._last_det_t = now

            # Visual debug: draw motion-depth trend on the outgoing frame (fast overlay).
            if frame is not None and isinstance(track, PrimaryTrack):
                try:
                    import cv2  # type: ignore

                    # Draw near bottom-left; keep deterministic.
                    cv2.putText(
                        frame,
                        f"AREA {int(motion_area)}  DELTA {int(motion_delta):+d}  {str(motion_status)}",
                        (20, int(frame.shape[0]) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                    )

                    # Arrow: approaching=up, away=down, stable=right.
                    x0 = 320
                    y0 = int(frame.shape[0]) - 45
                    if str(motion_status) == "APPROACHING":
                        x1, y1 = x0, y0 - 30
                    elif str(motion_status) == "AWAY":
                        x1, y1 = x0, y0 + 30
                    else:
                        x1, y1 = x0 + 40, y0
                    cv2.arrowedLine(frame, (int(x0), int(y0)), (int(x1), int(y1)), (255, 255, 0), 2, tipLength=0.25)
                except Exception:
                    pass

        return {
            "frame": frame,
            "detections": list(self._last_detections),
            "detections_inference": list(self._last_detections_inference),
            "tracks": list(self._last_tracks),
            "confirmed": list(self._last_confirmed),
            "stable": list(self._last_stable),
            "stable_labels": sorted({t.label for t in self._last_stable}),
            "stable_detections": [
                {
                    "label": t.label,
                    "bbox": t.bbox,
                    "confidence": float(t.confidence),
                    "area": int(t.area),
                }
                for t in self._last_stable
            ],
            "action_raw": self._last_action_raw,
            "action": self._last_action,
            "state": str(self.current_state),
            "collision_zone": _center_zone_for_width(int(frame.shape[1])),
            "position": position if detector_ran else "UNKNOWN",
            "in_collision": bool(in_collision) if detector_ran else False,
            "roi": None if self._last_roi is None else tuple(self._last_roi),
            "primary_track": None
            if self._last_primary_track is None
            else {
                "label": str(self._last_primary_track.label),
                "bbox": tuple(self._last_primary_track.bbox),
                "area": int(self._last_primary_track.area),
                "cx": int(self._last_primary_track.cx),
                "cy": int(self._last_primary_track.cy),
                "vx": float(self._last_primary_track.vx),
                "vy": float(self._last_primary_track.vy),
                "missed": int(self._last_primary_track.missed),
            },
            "primary_inference": None
            if self._last_primary_inference is None
            else dict(self._last_primary_inference),
            "primary": None if self._last_primary_scaled is None else dict(self._last_primary_scaled),
            "motion": {
                "area": 0 if self._last_primary_track is None else int(self._last_primary_track.area),
                "delta_area": 0 if self._last_primary_track is None else int(self._last_primary_track.delta_area),
                "trend": "NONE" if self._last_primary_track is None else str(self._last_primary_track.trend),
            },
            "backend": getattr(self.detector, "backend", "unknown"),
            "detector_ran": detector_ran,
            "detector_debug": getattr(self.detector, "last_debug", {}),
        }

    def _prepare_inference_frame(self, frame):
        """Resize for inference; returns (frame_for_detection, scale_tuple).

        scale = (sx, sy) where original = det * scale.
        """
        import cv2  # type: ignore

        h, w = frame.shape[:2]
        tw = int(max(1, self.cfg.inference_width))
        th = int(max(1, self.cfg.inference_height))
        if (w, h) == (tw, th):
            return frame, (1.0, 1.0)

        det_frame = cv2.resize(frame, (tw, th), interpolation=cv2.INTER_AREA)
        sx = float(w) / float(tw)
        sy = float(h) / float(th)
        return det_frame, (sx, sy)

    def _scale_detections(self, dets: List[Dict[str, Any]], scale, frame_shape) -> List[Dict[str, Any]]:
        sx, sy = scale
        h, w = frame_shape[:2]
        out: List[Dict[str, Any]] = []
        for d in dets:
            bbox = d.get("bbox")
            if not bbox or len(bbox) != 4:
                continue
            x1, y1, x2, y2 = [int(v) for v in bbox]
            x1 = int(round(x1 * sx))
            x2 = int(round(x2 * sx))
            y1 = int(round(y1 * sy))
            y2 = int(round(y2 * sy))
            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w, x2))
            y2 = max(0, min(h, y2))
            if x2 <= x1 or y2 <= y1:
                continue
            bbox2 = (x1, y1, x2, y2)
            out.append(
                {
                    "label": str(d.get("label", "object")),
                    "bbox": bbox2,
                    "confidence": float(d.get("confidence", 0.0)),
                    "area": int(max(0, (x2 - x1) * (y2 - y1))),
                }
            )
        return out
