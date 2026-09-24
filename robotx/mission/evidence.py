"""Would a TASK_COMPLETE claim meet the backend's L1 evidence bar?

The backend grades a completion claim against the **measured** position track
it has received since the commitment was granted (handoff §12,
`verification.js:312-385`). The Pi checks the same five conditions against the
fixes it actually sent, and does not claim completion unless they hold.

This is the Pi's own safeguard, not a courtesy. The handoff notes that when the
backend's grading is not configured it completes a task **on the claim alone**,
so a Pi that claimed loosely would be believed.

The thresholds are the backend's V1 values, copied exactly. Nothing here may be
loosened to make a claim pass.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

LatLon = Tuple[float, float]

VERIFY_ARRIVAL_RADIUS_M = 25.0
VERIFY_TRACK_MIN_FIX_RATE_PER_MIN = 10.0
VERIFY_TRACK_MAX_GAP_S = 10.0
VERIFY_TRACK_MIN_CORRIDOR_FRACTION = 0.8
VERIFY_CORRIDOR_HALF_WIDTH_M = 30.0
VERIFY_MAX_SPEED_MS = 8.33

_EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class TrackFix:
    """One measured fix that was actually sent to the backend."""

    t_ms: int
    lat: float
    lon: float


@dataclass(frozen=True)
class EvidenceVerdict:
    sufficient: bool
    failures: Tuple[str, ...]


def distance_m(a: LatLon, b: LatLon) -> float:
    lat1, lon1, lat2, lon2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((lat2 - lat1) / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2) ** 2
    return 2 * _EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def _distance_to_segment_m(p: LatLon, a: LatLon, b: LatLon) -> float:
    """Point-to-segment distance on a local flat projection (fine at corridor scale)."""

    scale = math.cos(math.radians(p[0]))
    def xy(q: LatLon) -> Tuple[float, float]:
        return (math.radians(q[1] - p[1]) * scale * _EARTH_RADIUS_M,
                math.radians(q[0] - p[0]) * _EARTH_RADIUS_M)
    ax, ay = xy(a)
    bx, by = xy(b)
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / length2))
    return math.hypot(ax + t * dx, ay + t * dy)


def distance_to_path_m(p: LatLon, path: Sequence[LatLon]) -> float:
    if not path:
        return math.inf
    if len(path) == 1:
        return distance_m(p, path[0])
    return min(_distance_to_segment_m(p, path[i], path[i + 1]) for i in range(len(path) - 1))


def assess_completion(
    track: Sequence[TrackFix],
    *,
    granted_at_ms: int,
    claim_at_ms: int,
    final_stop: LatLon,
    commanded_path: Sequence[LatLon],
) -> EvidenceVerdict:
    """All five L1 conditions, evaluated on the fixes the backend holds."""

    failures: List[str] = []
    if len(track) < 2:
        return EvidenceVerdict(False, (f"only {len(track)} measured fix(es) since the commitment was granted",))

    last = track[-1]
    if distance_m((last.lat, last.lon), final_stop) > VERIFY_ARRIVAL_RADIUS_M:
        failures.append(
            f"last fix is {distance_m((last.lat, last.lon), final_stop):.1f} m from the final stop "
            f"(limit {VERIFY_ARRIVAL_RADIUS_M:.0f} m)"
        )

    duration_min = (last.t_ms - granted_at_ms) / 60_000.0
    rate = len(track) / duration_min if duration_min > 0 else math.inf
    if rate < VERIFY_TRACK_MIN_FIX_RATE_PER_MIN:
        failures.append(f"fix rate {rate:.1f}/min (minimum {VERIFY_TRACK_MIN_FIX_RATE_PER_MIN:.0f}/min)")

    # Gaps include the lead-in from the grant and the tail up to the claim:
    # the backend's track starts at the grant, and a claim made long after the
    # last fix is a claim about a position nobody has seen for a while.
    instants = [granted_at_ms] + [f.t_ms for f in track] + [claim_at_ms]
    worst_gap = max((b - a) / 1000.0 for a, b in zip(instants, instants[1:]))
    if worst_gap > VERIFY_TRACK_MAX_GAP_S:
        failures.append(f"track gap of {worst_gap:.1f}s (maximum {VERIFY_TRACK_MAX_GAP_S:.0f}s)")

    inside = sum(
        1 for f in track
        if distance_to_path_m((f.lat, f.lon), commanded_path) <= VERIFY_CORRIDOR_HALF_WIDTH_M
    )
    fraction = inside / len(track)
    if fraction < VERIFY_TRACK_MIN_CORRIDOR_FRACTION:
        failures.append(
            f"{fraction:.0%} of fixes inside the commanded corridor "
            f"(minimum {VERIFY_TRACK_MIN_CORRIDOR_FRACTION:.0%})"
        )

    for a, b in zip(track, track[1:]):
        dt = (b.t_ms - a.t_ms) / 1000.0
        if dt <= 0:
            failures.append("fix timestamps are not strictly increasing")
            break
        speed = distance_m((a.lat, a.lon), (b.lat, b.lon)) / dt
        if speed > VERIFY_MAX_SPEED_MS:
            failures.append(f"implied speed {speed:.1f} m/s (maximum {VERIFY_MAX_SPEED_MS} m/s)")
            break

    return EvidenceVerdict(not failures, tuple(failures))
