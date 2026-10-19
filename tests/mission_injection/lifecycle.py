"""Record what the agent reports about a test mission, and nothing it does not.

Input is the agent's own state as a dict -- `build_telemetry(snapshot)` in
process, `GET /state` from the running service; the keys used here are common
to both. Each stage is recorded when, and only when, the agent's state shows
it:

- PICKUP_REACHED  `mission.pickup_reached_at` set by the MissionManager
- CUSTODY_*       `mission.custody_*_at` set by a real custody record
- DROP_REACHED    `mission.drop_reached_at`
- COMPLETED       `mission.status == "COMPLETE"`

The fixture's pickup and drop are never compared with the position here:
supplying a route proves nothing about having driven it. Backend telemetry is
summarised from the Pi's own link status -- STREAMING means the Pi says its
authenticated link is sending, not that RobotX stored anything.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from tests.mission_injection.ledger import LIFECYCLE, Ledger


_MISSION_STAGES = (
    ("pickup_reached_at", "PICKUP_REACHED"),
    ("custody_acquired_at", "CUSTODY_ACQUIRED"),
    ("drop_reached_at", "DROP_REACHED"),
    ("custody_released_at", "CUSTODY_RELEASED"),
)


def _get(data: Optional[Dict[str, Any]], *path: str) -> Any:
    for key in path:
        if not isinstance(data, dict):
            return None
        data = data.get(key)
    return data


class LifecycleRecorder:
    """Turns successive state dicts into the transitions they show."""

    def __init__(self, task_id: str, ledger: Optional[Ledger] = None, *, test_run_id: Optional[str] = None) -> None:
        self.task_id = task_id
        self.test_run_id = test_run_id
        self.ledger = ledger
        self.events: List[Dict[str, Any]] = []
        self._last: Dict[str, Any] = {}
        self._stages: set = set()
        self.backend_seen: set = set()

    def _emit(self, stage: str, **fields: Any) -> None:
        event = {"stage": stage, **fields}
        self.events.append(event)
        if self.ledger is not None:
            self.ledger.append(LIFECYCLE, test_run_id=self.test_run_id, task_id=self.task_id, **event)

    def _changed(self, key: str, value: Any) -> bool:
        if key in self._last and self._last[key] == value:
            return False
        self._last[key] = value
        return True

    def observe(self, state: Dict[str, Any]) -> None:
        held = state.get("mission")
        ours = isinstance(held, dict) and held.get("task_id") == self.task_id
        mission: Dict[str, Any] = held if isinstance(held, dict) and ours else {}
        safety = state.get("safety") or {}
        backend = _get(state, "communication", "backend")
        if backend is not None:
            self.backend_seen.add(backend)

        if self._changed("held", ours):
            self._emit("MISSION_HELD" if ours else "MISSION_NOT_HELD",
                       mission_task_id=_get(held, "task_id"))
        if self._changed("mode", state.get("mode")):
            self._emit("MODE", mode=state.get("mode"), last_error=state.get("last_error"))
        if ours and self._changed("mission_status", mission.get("status")):
            self._emit("MISSION_STATUS", status=mission.get("status"), segment=mission.get("segment"))
        if self._changed("navigation", _get(state, "navigation", "status")):
            self._emit("NAVIGATION", status=_get(state, "navigation", "status"),
                       position_issue=_get(state, "navigation", "position_issue"))
        motion_permitted = safety.get("verdict") in ("ALLOWED", "CLAMPED") and safety.get("rule") != "stop"
        if self._changed("safety", (safety.get("verdict"), safety.get("rule"))):
            self._emit("NAVIGATION_PERMITTED" if motion_permitted else "NAVIGATION_BLOCKED",
                       verdict=safety.get("verdict"), rule=safety.get("rule"), reason=safety.get("reason"))
        if self._changed("esp32", _get(state, "communication", "esp32")):
            self._emit("ESP32_LINK", status=_get(state, "communication", "esp32"))
        if self._changed("backend", backend):
            self._emit("BACKEND_LINK", status=backend,
                       detail=_get(state, "communication", "backend_detail"))

        if not ours:
            return
        for key, stage in _MISSION_STAGES:
            if mission.get(key) is not None and stage not in self._stages:
                self._stages.add(stage)
                self._emit(stage, at=mission.get(key),
                           measured=mission.get("pickup_measured" if "pickup" in key else "drop_measured"))
        if mission.get("status") == "COMPLETE" and "COMPLETED" not in self._stages:
            self._stages.add("COMPLETED")
            self._emit("COMPLETED", at=mission.get("completed_at"),
                       arrivals_measured=bool(mission.get("pickup_measured") and mission.get("drop_measured")))

    @property
    def completed(self) -> bool:
        return "COMPLETED" in self._stages

    def stages(self) -> List[str]:
        return [e["stage"] for e in self.events]

    def backend_telemetry(self) -> str:
        """STREAMING_THROUGHOUT, INTERRUPTED, FAILED, or NOT_OBSERVED -- from the Pi's link status."""

        if not self.backend_seen:
            return "NOT_OBSERVED"
        if self.backend_seen == {"STREAMING"}:
            return "STREAMING_THROUGHOUT"
        if "STREAMING" in self.backend_seen:
            return "INTERRUPTED"
        return "FAILED"
