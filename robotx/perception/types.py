"""Perception result model.

Deliberately absent: distance, depth, and time-to-collision fields.

The robot has a single Raspberry Pi Camera Module 3 and no depth sensor, no
stereo pair and no calibrated object-size database. A monocular camera cannot
measure distance, so this model does not carry a field that would imply it can.
What the camera *can* support is a bounding-box area in pixels and the change in
that area over time -- a coarse, uncalibrated "is it getting bigger" signal.
Those are exposed as `area_px` and (in the pipeline) frame-to-frame deltas,
under names that do not claim to be metres.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence, Tuple


BBox = Tuple[int, int, int, int]  # (x1, y1, x2, y2) in full-resolution pixels


class PerceptionStatus(str, Enum):
    """Whether a perception result can be trusted by the decision layer."""

    OK = "OK"                          # a frame was captured and processed
    NO_FRAME = "NO_FRAME"              # camera produced nothing (starting, stalled, failed)
    DETECTOR_ERROR = "DETECTOR_ERROR"  # inference raised
    DISABLED = "DISABLED"              # perception is switched off by configuration
    STALE = "STALE"                    # last result is older than the configured limit


@dataclass(frozen=True)
class Detection:
    """One detected object in full-resolution frame coordinates."""

    label: str
    confidence: float
    bbox: BBox
    area_px: int

    @property
    def center(self) -> Tuple[int, int]:
        x1, y1, x2, y2 = self.bbox
        return (x1 + x2) // 2, (y1 + y2) // 2

    @property
    def cx(self) -> int:
        return self.center[0]

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> Optional["Detection"]:
        """Build from the detector's raw dict output; None if malformed."""

        bbox = data.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return None
        try:
            x1, y1, x2, y2 = (int(v) for v in bbox)
            confidence = float(data.get("confidence", 0.0) or 0.0)
        except (TypeError, ValueError):
            return None
        if x2 <= x1 or y2 <= y1:
            return None

        area = data.get("area")
        try:
            area_px = int(area) if area is not None else (x2 - x1) * (y2 - y1)
        except (TypeError, ValueError):
            area_px = (x2 - x1) * (y2 - y1)

        return cls(
            label=str(data.get("label", "object")),
            confidence=confidence,
            bbox=(x1, y1, x2, y2),
            area_px=area_px,
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "label": self.label,
            "confidence": round(float(self.confidence), 3),
            "bbox": list(self.bbox),
            "area_px": int(self.area_px),
        }


@dataclass(frozen=True)
class FrameMetadata:
    """What the perception layer knows about the frame it processed."""

    width: int
    height: int
    source: str = "picamera2"
    age_s: Optional[float] = None  # age of the frame when inference started

    def to_dict(self) -> Dict[str, Any]:
        return {
            "width": int(self.width),
            "height": int(self.height),
            "source": self.source,
            "age_s": None if self.age_s is None else round(float(self.age_s), 3),
        }


@dataclass(frozen=True)
class PerceptionResult:
    """Immutable output of one perception cycle."""

    timestamp: float
    status: PerceptionStatus
    detections: Tuple[Detection, ...] = ()
    frame: Optional[FrameMetadata] = None
    processing_ms: float = 0.0
    backend: str = "none"
    error: Optional[str] = None
    # Frame-to-frame growth of the largest detection's bbox area, in px^2.
    # A positive value means "the biggest thing in view is getting bigger".
    # This is NOT a distance or a closing speed.
    largest_area_delta_px: int = 0

    @classmethod
    def unavailable(
        cls,
        status: PerceptionStatus,
        *,
        backend: str = "none",
        error: Optional[str] = None,
        timestamp: Optional[float] = None,
    ) -> "PerceptionResult":
        return cls(
            timestamp=time.time() if timestamp is None else timestamp,
            status=status,
            backend=backend,
            error=error,
        )

    @property
    def is_usable(self) -> bool:
        """True only when the decision layer may treat this as real information.

        Requires frame metadata as well as an `OK` status. Without the frame's
        dimensions the decision layer cannot run its geometric corridor check,
        and an obstacle dead ahead would pass through unexamined -- a result
        carrying detections but no frame must therefore count as unusable, not
        as a clear path.
        """

        return self.status is PerceptionStatus.OK and self.frame is not None

    @property
    def largest(self) -> Optional[Detection]:
        if not self.detections:
            return None
        return max(self.detections, key=lambda d: d.area_px)

    def has_label(self, label: str, *, min_confidence: float = 0.0) -> bool:
        return any(
            d.label == label and d.confidence >= min_confidence for d in self.detections
        )

    def labels(self) -> List[str]:
        return sorted({d.label for d in self.detections})

    def summary(self) -> Dict[str, Any]:
        """Compact form for telemetry -- counts and the largest object only."""

        largest = self.largest
        return {
            "status": self.status.value,
            "backend": self.backend,
            "count": len(self.detections),
            "labels": self.labels(),
            "largest_area_px": None if largest is None else int(largest.area_px),
            "largest_area_delta_px": int(self.largest_area_delta_px),
            "processing_ms": round(float(self.processing_ms), 2),
            "age_s": round(max(0.0, time.time() - self.timestamp), 3),
            "error": self.error,
        }

    def to_dict(self, *, max_detections: int = 10) -> Dict[str, Any]:
        out = self.summary()
        out["timestamp"] = self.timestamp
        out["frame"] = None if self.frame is None else self.frame.to_dict()
        out["detections"] = [d.to_dict() for d in self.detections[:max_detections]]
        return out


def detections_from_dicts(raw: Sequence[Dict[str, Any]]) -> Tuple[Detection, ...]:
    """Convert detector dict output to Detection objects, dropping malformed rows."""

    out: List[Detection] = []
    for item in raw:
        if not isinstance(item, dict):
            continue
        det = Detection.from_dict(item)
        if det is not None:
            out.append(det)
    return tuple(out)
