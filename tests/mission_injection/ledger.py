"""The local test log: what was injected, when, and what became of it.

Append-only JSON Lines. Every record carries the same provenance block, so no
line read on its own can be taken for a RobotX assignment::

    "provenance": {"source": "test_injected", "backend_assigned": false,
                   "reason": "... cannot assign tasks to physical robots ..."}

One injection attempt per task id, ever: `claim_attempt` checks and appends
under an exclusive lock, so two harness processes cannot both inject the same
task. Fails closed -- an unreadable or corrupt ledger refuses the claim rather
than forgetting what it held. The Rover's `MissionManager` independently
refuses a task id it has already run (`DUPLICATE_TASK`); this is the harness's
own guard, which also covers a restarted agent that has forgotten.
"""

from __future__ import annotations

import fcntl
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

from tests.mission_injection.fixture import PROVENANCE, PROVENANCE_REASON, MissionFixture


DEFAULT_LEDGER_PATH = Path.home() / ".local" / "state" / "robotx-test-injection" / "ledger.jsonl"

FIXTURE_VALIDATED = "FIXTURE_VALIDATED"
FIXTURE_REJECTED = "FIXTURE_REJECTED"
PREFLIGHT = "PREFLIGHT"
INJECTION_ATTEMPTED = "INJECTION_ATTEMPTED"
INJECTION_BLOCKED = "INJECTION_BLOCKED"
ACCEPTED = "ACCEPTED"
REJECTED = "REJECTED"
LIFECYCLE = "LIFECYCLE"
OUTCOME_UNKNOWN = "OUTCOME_UNKNOWN"


class LedgerError(Exception):
    pass


class DuplicateInjection(LedgerError):
    def __init__(self, task_id: str, previous: Dict[str, Any]) -> None:
        super().__init__(
            f"task {task_id!r} was already injected at {previous.get('at')} "
            f"(test run {previous.get('test_run_id')!r}); a task id is injected once"
        )
        self.task_id = task_id
        self.previous = previous


def provenance() -> Dict[str, Any]:
    return {"source": PROVENANCE, "backend_assigned": False, "reason": PROVENANCE_REASON}


class Ledger:
    def __init__(self, path: Union[str, Path] = DEFAULT_LEDGER_PATH) -> None:
        self.path = Path(path)

    def _open(self) -> Any:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_APPEND, 0o600)
        return os.fdopen(fd, "r+", encoding="utf-8")

    @staticmethod
    def _parse(handle: Any) -> List[Dict[str, Any]]:
        handle.seek(0)
        records = []
        for number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except ValueError:
                raise LedgerError(f"ledger line {number} is not JSON; refusing to guess what it held") from None
            if not isinstance(record, dict):
                raise LedgerError(f"ledger line {number} is not an object")
            records.append(record)
        return records

    def records(self) -> List[Dict[str, Any]]:
        if not self.path.exists():
            return []
        with self._open() as handle:
            fcntl.flock(handle, fcntl.LOCK_SH)
            return self._parse(handle)

    @staticmethod
    def _record(event: str, *, test_run_id: Optional[str], task_id: Optional[str],
                fields: Dict[str, Any]) -> Dict[str, Any]:
        now = time.time()
        return {
            "at": datetime.fromtimestamp(now, timezone.utc).isoformat(timespec="milliseconds"),
            "at_unix": round(now, 3),
            "event": event,
            "test_run_id": test_run_id,
            "task_id": task_id,
            "provenance": provenance(),
            **fields,
        }

    @staticmethod
    def _write(handle: Any, record: Dict[str, Any]) -> None:
        handle.write(json.dumps(record, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())

    def append(self, event: str, *, fixture: Optional[MissionFixture] = None,
               test_run_id: Optional[str] = None, task_id: Optional[str] = None,
               **fields: Any) -> Dict[str, Any]:
        record = self._record(
            event,
            test_run_id=fixture.test_run_id if fixture else test_run_id,
            task_id=fixture.task_id if fixture else task_id,
            fields=fields,
        )
        with self._open() as handle:
            fcntl.flock(handle, fcntl.LOCK_EX)
            self._write(handle, record)
        return record

    def previous_attempt(self, task_id: str) -> Optional[Dict[str, Any]]:
        for record in self.records():
            if record.get("event") == INJECTION_ATTEMPTED and record.get("task_id") == task_id:
                return record
        return None

    def claim_attempt(self, fixture: MissionFixture, *, ingress: str) -> Dict[str, Any]:
        """Record INJECTION_ATTEMPTED, or raise `DuplicateInjection`. Atomic."""

        with self._open() as handle:
            fcntl.flock(handle, fcntl.LOCK_EX)
            for record in self._parse(handle):
                if record.get("event") == INJECTION_ATTEMPTED and record.get("task_id") == fixture.task_id:
                    raise DuplicateInjection(fixture.task_id, record)
            record = self._record(INJECTION_ATTEMPTED, test_run_id=fixture.test_run_id,
                                  task_id=fixture.task_id,
                                  fields={"ingress": ingress, "fixture": fixture.summary()})
            self._write(handle, record)
            return record
