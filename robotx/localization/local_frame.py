"""A local metric frame, and dead reckoning within it.

Why this exists
---------------
Everything downstream of localization -- `Navigator`, `RoutePlanner`,
`haversine_m`, `bearing_deg`, and the backend telemetry contract -- already
speaks latitude and longitude, and all of it works. The rover's problem is not
that lat/lon is the wrong language; it is that the only thing that *spoke* it
was a GPS, and a GPS is useless indoors and absent from this rover entirely.

So this module does not introduce a second coordinate system for navigation to
learn. It introduces a **projection**: a flat, metres-based frame pinned to an
origin, with exact conversions in both directions. A waypoint three metres
ahead becomes a lat/lon; a dead-reckoned pose becomes a `Position` shaped
exactly like a GPS one. Navigation and telemetry keep working unchanged, and
the entire local-frame story stays inside this file.

    local (x=east m, y=north m)  <--LocalFrame-->  (lat, lon)
                                                        |
                                    Navigator / telemetry, unmodified

Honesty about what a dead-reckoned position is worth
----------------------------------------------------
`DeadReckoner` integrates the motion the Pi *commanded*. It is open loop: no
encoder, no IMU, no external reference. Nothing corrects it, so its error grows
without bound -- wheel slip, carpet, a stalled motor and a battery sagging under
load are all invisible to it, and each one makes the estimate confidently
wrong.

That is acceptable for a short indoor run and unacceptable as a measurement,
which is why every `Position` this module produces is stamped
`PositionSource.DEAD_RECKONING`. No consumer can mistake it for a fix, and the
backend telemetry path refuses it unless an operator has explicitly opted in.
When ESP32 encoders arrive they correct this estimate rather than replace it,
and the frame below does not change.

Projection limits
-----------------
`LocalFrame` uses an equirectangular approximation about its origin. Within a
few hundred metres the error is well under the accuracy of anything feeding it;
it degrades with distance from the origin and is meaningless near the poles.
It is chosen because it is exactly invertible and cheap, and because a rover
demo that ranges far enough for the projection to matter has a GPS.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from robotx.localization.position import (
    EARTH_RADIUS_M,
    HeadingSource,
    LatLon,
    Position,
    PositionSource,
)


@dataclass(frozen=True)
class LocalFrame:
    """A flat metric frame pinned to a lat/lon origin.

    Axes are `x` east and `y` north, both in metres, which keeps the frame
    right-handed in the usual survey sense and means a heading of 0 degrees
    (north, clockwise-positive) points along +y.
    """

    origin: LatLon

    def to_latlon(self, x_m: float, y_m: float) -> LatLon:
        """Local metres -> (lat, lon)."""

        lat0, lon0 = self.origin
        lat = lat0 + math.degrees(y_m / EARTH_RADIUS_M)
        # Longitude degrees shrink with latitude. Guarded so a frame defined at
        # a pole degrades to "no east/west movement" instead of dividing by
        # zero and producing infinities that would propagate into navigation.
        cos_lat = math.cos(math.radians(lat0))
        if abs(cos_lat) < 1e-12:
            return (lat, lon0)
        lon = lon0 + math.degrees(x_m / (EARTH_RADIUS_M * cos_lat))
        return (lat, lon)

    def to_local(self, point: LatLon) -> Tuple[float, float]:
        """(lat, lon) -> local metres. The exact inverse of `to_latlon`."""

        lat0, lon0 = self.origin
        lat, lon = point
        y = math.radians(lat - lat0) * EARTH_RADIUS_M
        cos_lat = math.cos(math.radians(lat0))
        if abs(cos_lat) < 1e-12:
            return (0.0, y)
        x = math.radians(lon - lon0) * EARTH_RADIUS_M * cos_lat
        return (x, y)


# Where a rover with no GPS and no configured origin places its local frame.
# Null Island: deliberately absurd, and chosen for that reason. An origin that
# looked like a real location would put a dashboard pin on a street the rover
# has never seen, and nobody would question it. A rover apparently operating at
# (0, 0) is obviously running on a synthetic origin, which is exactly the
# impression a synthetic origin should give.
DEFAULT_LOCAL_ORIGIN: LatLon = (0.0, 0.0)


@dataclass(frozen=True)
class LocalPose:
    """Where the dead reckoner believes the rover is, in local metres."""

    x_m: float = 0.0
    y_m: float = 0.0
    heading_deg: float = 0.0
    # Metres of path integrated since the last reset. Not displacement: this
    # only grows. It is the honest way to describe accumulated uncertainty,
    # because dead-reckoning error tracks distance travelled rather than
    # distance from home -- a rover that drives a loop back to its start has
    # not thereby become accurate again.
    distance_travelled_m: float = 0.0
    updated_at: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "x_m": round(self.x_m, 3),
            "y_m": round(self.y_m, 3),
            "heading_deg": round(self.heading_deg, 1),
            "distance_travelled_m": round(self.distance_travelled_m, 2),
            "updated_at": self.updated_at,
        }


@dataclass(frozen=True)
class DeadReckoningConfig:
    """Calibration tying normalized motion intent to real-world motion.

    These two numbers are the entire model, and **both must be measured on the
    actual rover** -- they are not derivable from anything in this repository.
    The defaults are placeholders chosen to be slow and plausible for a small
    indoor rover; used uncalibrated they will produce a pose that is wrong by a
    large factor, not a small one.

    To calibrate `max_speed_mps`: drive straight at a known normalized speed
    for a measured time over a measured distance, then solve. To calibrate
    `turn_rate_dps`: command a rotation for a measured time and measure the
    angle actually swept.
    """

    # Ground speed, in m/s, at a normalized linear velocity of 1.0.
    max_speed_mps: float = 0.4
    # Rotation rate, in deg/s, at a normalized angular velocity of 1.0.
    turn_rate_dps: float = 90.0
    # Integration steps longer than this are discarded rather than integrated.
    # A long gap means the loop stalled or the process was suspended, and in
    # neither case does the last commanded intent describe what the rover did
    # for those seconds. Dropping the step loses a little motion; trusting it
    # would inject a large fabricated jump.
    max_step_s: float = 0.5

    @classmethod
    def from_settings(cls, settings: Any) -> "DeadReckoningConfig":
        return cls(
            max_speed_mps=settings.deadreckon_max_speed_mps,
            turn_rate_dps=settings.deadreckon_turn_rate_dps,
            max_step_s=settings.deadreckon_max_step_s,
        )


class DeadReckoner:
    """Integrates commanded motion into a local pose.

    Deliberately takes plain normalized `linear` and `angular` floats rather
    than a `MotionIntent`. Those are exactly the two numbers the model needs,
    and taking them keeps the localization package from importing the control
    package -- the caller passes `intent.linear` and `intent.angular`, which
    `MotionIntent` already exposes.

    Thread-safe: the agent loop integrates while an HTTP handler may read.
    """

    def __init__(self, cfg: Optional[DeadReckoningConfig] = None) -> None:
        self.cfg = cfg or DeadReckoningConfig()
        self._lock = threading.Lock()
        self._pose = LocalPose()
        self._last_t: Optional[float] = None

    def reset(self, *, heading_deg: float = 0.0, now: Optional[float] = None) -> None:
        """Return to the frame origin, facing `heading_deg`.

        Called when a mission starts, so "here" is wherever the rover actually
        is at that moment and accumulated drift is discarded rather than
        carried into the new run.
        """

        with self._lock:
            self._pose = LocalPose(
                heading_deg=heading_deg % 360.0,
                updated_at=time.time() if now is None else now,
            )
            self._last_t = None

    def integrate(
        self, *, linear: float, angular: float, now: Optional[float] = None
    ) -> LocalPose:
        """Advance the pose by one step of commanded motion.

        Rotation is applied first, then translation along the resulting
        heading. At loop rates this ordering is a detail, but fixing it matters
        for reproducibility: the same inputs must always give the same pose.
        """

        now = time.monotonic() if now is None else now

        with self._lock:
            last = self._last_t
            self._last_t = now

            # First call establishes the clock; there is no elapsed time to
            # integrate over yet, and assuming one would invent motion.
            if last is None:
                return self._pose

            dt = now - last
            if dt <= 0.0 or dt > self.cfg.max_step_s:
                return self._pose

            pose = self._pose
            heading = (pose.heading_deg + angular * self.cfg.turn_rate_dps * dt) % 360.0

            step_m = linear * self.cfg.max_speed_mps * dt
            rad = math.radians(heading)
            # Heading is clockwise from north, so north is +y and east is +x:
            # sin drives east, cos drives north.
            self._pose = LocalPose(
                x_m=pose.x_m + step_m * math.sin(rad),
                y_m=pose.y_m + step_m * math.cos(rad),
                heading_deg=heading,
                distance_travelled_m=pose.distance_travelled_m + abs(step_m),
                updated_at=time.time(),
            )
            return self._pose

    @property
    def pose(self) -> LocalPose:
        with self._lock:
            return self._pose

    def position(self, frame: LocalFrame) -> Position:
        """Project the current pose into a `Position`, shaped like a GPS one.

        Identical in structure to what `PositionEstimator` produces, and
        distinguishable from it by exactly one thing: `source`. That field is
        the whole reason a consumer cannot be fooled, so it is never optional
        and never defaulted here.

        `satellites` stays `None` rather than `0`. Zero satellites is a claim
        about a receiver this rover does not have; null is the absence of one.
        """

        pose = self.pose
        lat, lon = frame.to_latlon(pose.x_m, pose.y_m)
        return Position(
            latitude=lat,
            longitude=lon,
            timestamp=pose.updated_at or time.time(),
            altitude_m=None,
            # Commanded speed is not measured speed, and reporting it as though
            # it were is precisely the fabrication this codebase refuses
            # elsewhere. A consumer wanting to know how fast the rover was told
            # to go can read the motion intent, where that is what it means.
            speed_mps=None,
            heading_deg=pose.heading_deg,
            heading_source=HeadingSource.DEAD_RECKONED,
            satellites=None,
            source=PositionSource.DEAD_RECKONING,
        )
