from __future__ import annotations

import time
from dataclasses import dataclass
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple


BBox = Tuple[int, int, int, int]


def _bbox_area(bbox: Sequence[int]) -> int:
    x1, y1, x2, y2 = [int(v) for v in bbox]
    return int(max(0, x2 - x1) * max(0, y2 - y1))


def _iou(a: Sequence[int], b: Sequence[int]) -> float:
    ax1, ay1, ax2, ay2 = [int(v) for v in a]
    bx1, by1, bx2, by2 = [int(v) for v in b]

    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)

    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0

    union = _bbox_area(a) + _bbox_area(b) - inter
    if union <= 0:
        return 0.0
    return float(inter) / float(union)


def _ema_bbox(prev: Sequence[int], new: Sequence[int], alpha: float) -> BBox:
    # alpha in [0..1]; higher alpha = more weight on previous (smoother)
    px1, py1, px2, py2 = [float(v) for v in prev]
    nx1, ny1, nx2, ny2 = [float(v) for v in new]
    x1 = alpha * px1 + (1.0 - alpha) * nx1
    y1 = alpha * py1 + (1.0 - alpha) * ny1
    x2 = alpha * px2 + (1.0 - alpha) * nx2
    y2 = alpha * py2 + (1.0 - alpha) * ny2
    return int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))


@dataclass
class Track:
    track_id: int
    label: str
    bbox: BBox
    confidence: float

    hits: int = 0
    age: int = 0
    missed: int = 0

    created_t: float = 0.0
    last_seen_t: float = 0.0

    @property
    def area(self) -> int:
        return _bbox_area(self.bbox)

    @property
    def is_confirmed(self) -> bool:
        # This is a convenience; the actual min-hits is controlled by the tracker.
        return self.hits > 0 and self.missed == 0


class ObjectTracker:
    """Simple IoU-based multi-object tracker.

    Goals (for Pi/CPU):
    - Keep IDs stable across frames
    - Suppress one-frame flicker by requiring min_hits before "confirmed"
    - Be robust to occasional missed detections

    This intentionally avoids heavier dependencies (e.g., SORT/DeepSORT).
    """

    def __init__(
        self,
        iou_threshold: float = 0.3,
        max_missed: int = 8,
        min_hits: int = 2,
        bbox_ema_alpha: float = 0.6,
    ) -> None:
        self.iou_threshold = float(iou_threshold)
        self.max_missed = int(max_missed)
        self.min_hits = int(min_hits)
        self.bbox_ema_alpha = float(bbox_ema_alpha)

        self._next_id = 1
        self._tracks: List[Track] = []

    def reset(self) -> None:
        self._next_id = 1
        self._tracks = []

    def tracks(self) -> List[Track]:
        return list(self._tracks)

    def update(self, detections: List[Dict[str, Any]], now: Optional[float] = None) -> List[Track]:
        if now is None:
            now = time.monotonic()

        # Age existing tracks.
        for t in self._tracks:
            t.age += 1

        # Greedy assignment per label for simplicity.
        unmatched_det = set(range(len(detections)))
        matched_track = set()

        # Pre-extract det fields (validated len=4 assumed upstream).
        det_label = [str(d.get("label", "")) for d in detections]
        det_bbox: List[BBox] = [tuple(int(v) for v in d.get("bbox", (0, 0, 0, 0))) for d in detections]  # type: ignore
        det_conf = [float(d.get("confidence", 0.0)) for d in detections]

        # For each track, find the best IoU detection of the same label.
        # Then resolve conflicts by sorting all candidate matches by IoU.
        candidates: List[Tuple[float, int, int]] = []  # (iou, track_idx, det_idx)
        for ti, t in enumerate(self._tracks):
            for di, (lbl, bb) in enumerate(zip(det_label, det_bbox)):
                if lbl != t.label:
                    continue
                iou = _iou(t.bbox, bb)
                if iou >= self.iou_threshold:
                    candidates.append((iou, ti, di))

        candidates.sort(key=lambda x: x[0], reverse=True)

        used_tracks = set()
        used_dets = set()
        for iou, ti, di in candidates:
            if ti in used_tracks or di in used_dets:
                continue
            used_tracks.add(ti)
            used_dets.add(di)

            t = self._tracks[ti]
            # Smooth bbox to reduce jitter.
            t.bbox = _ema_bbox(t.bbox, det_bbox[di], alpha=self.bbox_ema_alpha)
            t.confidence = float(det_conf[di])
            t.hits += 1
            t.missed = 0
            t.last_seen_t = float(now)

            matched_track.add(ti)
            if di in unmatched_det:
                unmatched_det.remove(di)

        # Unmatched tracks: increment missed and drop if stale.
        kept: List[Track] = []
        for ti, t in enumerate(self._tracks):
            if ti not in matched_track:
                t.missed += 1
            if t.missed <= self.max_missed:
                kept.append(t)
        self._tracks = kept

        # New tracks for unmatched detections.
        for di in sorted(unmatched_det):
            lbl = det_label[di]
            if not lbl:
                continue
            bb = det_bbox[di]
            conf = float(det_conf[di])
            tid = self._next_id
            self._next_id += 1
            self._tracks.append(
                Track(
                    track_id=tid,
                    label=lbl,
                    bbox=bb,
                    confidence=conf,
                    hits=1,
                    age=1,
                    missed=0,
                    created_t=float(now),
                    last_seen_t=float(now),
                )
            )

        return list(self._tracks)

    def confirmed_tracks(self) -> List[Track]:
        return [t for t in self._tracks if t.hits >= self.min_hits and t.missed == 0]


def _bbox_center(bbox: Sequence[int]) -> Tuple[int, int]:
    x1, y1, x2, y2 = [int(v) for v in bbox]
    cx = int((x1 + x2) // 2)
    cy = int((y1 + y2) // 2)
    return cx, cy


@dataclass
class PrimaryTrack:
    label: str
    bbox: BBox
    confidence: float
    area: int
    delta_area: int
    trend: str
    cx: int
    cy: int
    vx: float
    vy: float
    missed: int


class PrimaryObjectTracker:
    """Lightweight tracker for the single most important (largest-area) object.

    Designed for low latency on Raspberry Pi:
    - Select primary by max `area`
    - Match to previous using IoU
    - Smooth bbox using EMA
    - Smooth center/area using moving average
    - Provide simple velocity estimate in px/update
    """

    def __init__(
        self,
        iou_threshold: float = 0.25,
        bbox_ema_alpha: float = 0.6,
        history_len: int = 5,
        max_missed: int = 3,
    ) -> None:
        self.iou_threshold = float(iou_threshold)
        self.bbox_ema_alpha = float(bbox_ema_alpha)
        self.history_len = int(max(1, history_len))
        self.max_missed = int(max(0, max_missed))

        self._bbox: Optional[BBox] = None
        self._label: str = ""
        self._conf: float = 0.0
        self._missed: int = 0

        self._cx_hist: Deque[int] = deque(maxlen=self.history_len)
        self._cy_hist: Deque[int] = deque(maxlen=self.history_len)
        self._area_hist: Deque[int] = deque(maxlen=self.history_len)

        self._prev_cx: Optional[float] = None
        self._prev_cy: Optional[float] = None
        self._vx: float = 0.0
        self._vy: float = 0.0

        # Area trend (motion-depth proxy): use smoothed area deltas.
        self._prev_area_s: Optional[float] = None
        self._delta_area: int = 0
        self._trend: str = "STABLE"  # APPROACHING | STABLE | AWAY

    def reset(self) -> None:
        self.__init__(
            iou_threshold=self.iou_threshold,
            bbox_ema_alpha=self.bbox_ema_alpha,
            history_len=self.history_len,
            max_missed=self.max_missed,
        )

    def update(self, detections: List[Dict[str, Any]]) -> Optional[PrimaryTrack]:
        if not detections:
            self._missed += 1
            if self._missed > self.max_missed:
                self._bbox = None
                self._label = ""
                self._conf = 0.0
                self._cx_hist.clear()
                self._cy_hist.clear()
                self._area_hist.clear()
                self._prev_cx = None
                self._prev_cy = None
                self._vx = 0.0
                self._vy = 0.0
                self._prev_area_s = None
                self._delta_area = 0
                self._trend = "STABLE"
                return None
            return self.current()

        # Select primary detection by max area.
        primary: Optional[Dict[str, Any]] = None
        best_area = -1
        for d in detections:
            try:
                a = int(d.get("area") or 0)
            except Exception:
                a = 0
            if a > best_area:
                best_area = a
                primary = d

        if primary is None:
            self._missed += 1
            return self.current()

        lbl = str(primary.get("label", "object"))
        conf = float(primary.get("confidence", 0.0))
        bb_raw = primary.get("bbox")
        if not bb_raw or len(bb_raw) != 4:
            self._missed += 1
            return self.current()

        bb: BBox = tuple(int(v) for v in bb_raw)  # type: ignore

        # IoU match with previous.
        if self._bbox is None:
            self._bbox = bb
            self._label = lbl
            self._conf = conf
        else:
            iou = _iou(self._bbox, bb)
            if float(iou) < float(self.iou_threshold):
                # New object (or large jump).
                self._bbox = bb
                self._label = lbl
                self._conf = conf
                self._cx_hist.clear()
                self._cy_hist.clear()
                self._area_hist.clear()
                self._prev_cx = None
                self._prev_cy = None
                self._vx = 0.0
                self._vy = 0.0
                self._prev_area_s = None
                self._delta_area = 0
                self._trend = "STABLE"
            else:
                # Smooth bbox.
                self._bbox = _ema_bbox(self._bbox, bb, alpha=self.bbox_ema_alpha)
                self._label = lbl
                self._conf = conf

        self._missed = 0

        cx, cy = _bbox_center(self._bbox)
        area = _bbox_area(self._bbox)

        # Moving averages.
        self._cx_hist.append(int(cx))
        self._cy_hist.append(int(cy))
        self._area_hist.append(int(area))

        cx_s = float(sum(self._cx_hist)) / float(max(1, len(self._cx_hist)))
        cy_s = float(sum(self._cy_hist)) / float(max(1, len(self._cy_hist)))

        # Velocity estimate (px/update).
        if self._prev_cx is None or self._prev_cy is None:
            self._vx = 0.0
            self._vy = 0.0
        else:
            self._vx = float(cx_s) - float(self._prev_cx)
            self._vy = float(cy_s) - float(self._prev_cy)

        self._prev_cx = float(cx_s)
        self._prev_cy = float(cy_s)

        area_s = float(sum(self._area_hist)) / float(max(1, len(self._area_hist)))

        # Delta area based on smoothed (moving-average) area.
        if self._prev_area_s is None:
            self._delta_area = 0
        else:
            self._delta_area = int(round(float(area_s) - float(self._prev_area_s)))
        self._prev_area_s = float(area_s)

        # Trend classification.
        # Small threshold to avoid flicker (tuned for inference-scale areas).
        eps = 600
        if int(self._delta_area) > int(eps):
            self._trend = "APPROACHING"
        elif int(self._delta_area) < int(-eps):
            self._trend = "AWAY"
        else:
            self._trend = "STABLE"

        return PrimaryTrack(
            label=str(self._label),
            bbox=self._bbox,
            confidence=float(self._conf),
            area=int(round(area_s)),
            delta_area=int(self._delta_area),
            trend=str(self._trend),
            cx=int(round(cx_s)),
            cy=int(round(cy_s)),
            vx=float(self._vx),
            vy=float(self._vy),
            missed=int(self._missed),
        )

    def current(self) -> Optional[PrimaryTrack]:
        if self._bbox is None:
            return None

        cx, cy = _bbox_center(self._bbox)
        area = _bbox_area(self._bbox)

        cx_s = float(sum(self._cx_hist)) / float(max(1, len(self._cx_hist))) if self._cx_hist else float(cx)
        cy_s = float(sum(self._cy_hist)) / float(max(1, len(self._cy_hist))) if self._cy_hist else float(cy)
        area_s = float(sum(self._area_hist)) / float(max(1, len(self._area_hist))) if self._area_hist else float(area)

        return PrimaryTrack(
            label=str(self._label or "object"),
            bbox=self._bbox,
            confidence=float(self._conf),
            area=int(round(area_s)),
            delta_area=int(self._delta_area),
            trend=str(self._trend),
            cx=int(round(cx_s)),
            cy=int(round(cy_s)),
            vx=float(self._vx),
            vy=float(self._vy),
            missed=int(self._missed),
        )