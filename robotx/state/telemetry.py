"""Telemetry aggregation: one robot snapshot -> one serializable payload.

Telemetry is **local only** at this stage. This module builds the payload and
nothing more -- it does not know about sockets, HTTP, or any backend. When a
backend link is added later, it consumes this same payload rather than
assembling its own, so there is exactly one telemetry schema.

Rule: no fabricated values. A quantity this robot cannot measure is reported
with an explicit `UNAVAILABLE` status and a `null` value. It is never given a
plausible-looking number.
"""

from __future__ import annotations

from typing import Any, Dict

from robotx.hardware.battery import battery_status
from robotx.state.robot_state import RobotSnapshot


TELEMETRY_SCHEMA_VERSION = 2


def battery_telemetry() -> Dict[str, Any]:
    """Battery block for telemetry.

    Delegates to `robotx.hardware.battery`, which is the single source of
    truth for what this robot can and cannot measure. There is no battery
    sensing hardware, so this reports UNAVAILABLE with null values rather
    than a plausible-looking number.
    """

    return battery_status()


def build_telemetry(snapshot: RobotSnapshot) -> Dict[str, Any]:
    """Build the local telemetry payload from an authoritative state snapshot."""

    return {
        "schema_version": TELEMETRY_SCHEMA_VERSION,
        "timestamp": snapshot.updated_at,
        "robot_id": snapshot.robot_id,
        "mode": snapshot.mode.value,
        "uptime_s": round(snapshot.uptime_s, 1),
        "gps": snapshot.gps.to_dict(),
        "position": None if snapshot.position is None else snapshot.position.to_dict(),
        "navigation": snapshot.navigation.to_dict(),
        "perception": snapshot.perception.summary(),
        "motion_intent": snapshot.motion_intent.to_dict(),
        "battery": battery_telemetry(),
        "health": snapshot.health.to_dict(),
        "communication": snapshot.communication.to_dict(),
        "last_error": snapshot.last_error,
    }
