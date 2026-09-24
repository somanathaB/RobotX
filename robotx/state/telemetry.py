"""Telemetry aggregation: one robot snapshot -> one serializable payload.

This is the **Rover's own** telemetry: local, and independent of whether the
RobotX backend exists or is reachable. It knows nothing about sockets, HTTP or
any backend, and the backend link deliberately builds its own payload to the
backend's contract rather than reusing this one -- the Rover's diagnostics and
a fleet platform's schema are not the same thing and should not be coupled.

Snapshot-only
-------------
Every value here comes from the `RobotSnapshot` passed in. This module reads no
hardware, calls no sensor, and holds no state of its own. That is not a style
preference: a frame that took most of its fields from a snapshot and then
reached past it for a live battery read would describe two different instants
while presenting as one observation. Producers write `RobotState`; consumers
read a snapshot of it.

Rule: no fabricated values. A quantity this robot cannot measure is reported
with an explicit `UNAVAILABLE` status and a `null` value. It is never given a
plausible-looking number.
"""

from __future__ import annotations

from typing import Any, Dict

from robotx.state.robot_state import RobotSnapshot


TELEMETRY_SCHEMA_VERSION = 2


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
        # The assigned task and its progress. Null on a Rover that has never
        # been given one -- a locally driven route is not a mission, and
        # inventing a task id for it would make the two indistinguishable.
        "mission": None if snapshot.mission is None else snapshot.mission.to_dict(),
        "perception": snapshot.perception.summary(),
        "motion_intent": snapshot.motion_intent.to_dict(),
        # Why that intent is what it is. A stopped rover looks identical in
        # `motion_intent` whether navigation had nowhere to go or the safety
        # gate refused to let it move; this is the field that tells them apart.
        "safety": snapshot.safety.to_dict(),
        "battery": snapshot.power.to_dict(),
        "health": snapshot.health.to_dict(),
        "communication": snapshot.communication.to_dict(),
        # What the ESP32 reports, as reported; null with no ESP32 link. DIAG is
        # deliberately not here -- it is diagnostics, served from the snapshot.
        "controller": None if snapshot.controller is None else snapshot.controller.to_dict(),
        "last_error": snapshot.last_error,
    }
