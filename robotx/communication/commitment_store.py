"""Per-commitment high-water marks, persisted across restarts.

The backend delivers engine envelopes at least once (handoff §15), so the Pi
must remember, **across a reboot**, for every commitment it has seen:

- the highest fence and sequence it applied (admission steps 4-5),
- whether the commitment is tombstoned (WITHDRAW / RECALL / ABORT_MISSION),
- which outboxIds it has acknowledged,
- whether it has already answered the OFFER -- the backend must get exactly
  one response per commitment, and a second ACCEPT would reset a Leg,
- which custody events and whether TASK_COMPLETE have left the Pi.

Handoff step 6 says the marks are persisted **before** acting. `update`
therefore writes to disk first and only then changes the in-memory copy: if the
write fails, nothing changed, and the caller must not act.

Stored the same way as the session token: one JSON file, written to a temp
file and renamed, mode 0600. With `path=None` it is memory-only (tests).
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
import time
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any, Dict, List, Optional

from robotx.config.logging_setup import log_event


logger = logging.getLogger(__name__)

DEFAULT_COMMITMENT_PATH = "~/.robotx/commitments.json"
# Enough for a long run of offers; oldest records are dropped first.
MAX_RECORDS = 256
MAX_OUTBOX_IDS = 32


@dataclass(frozen=True)
class CommitmentRecord:
    commitment_id: str
    highest_fence: Optional[int] = None
    # The highest applied fence exactly as it arrived on the wire, for echoing.
    fence_wire: Any = None
    highest_sequence: Optional[int] = None
    task_id: Optional[str] = None
    # "ACCEPT" | "REJECT" | "DEFER" once the OFFER has been answered.
    response: Optional[str] = None
    acked_outbox_ids: List[str] = field(default_factory=list)
    tombstoned: bool = False
    custody_sent: List[str] = field(default_factory=list)
    # "SENT" | "WITHHELD" once completion has been decided.
    completion: Optional[str] = None
    updated_at: float = 0.0

    @property
    def is_held(self) -> bool:
        """An accepted commitment this Rover still owes work on."""

        return self.response == "ACCEPT" and not self.tombstoned and self.completion is None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CommitmentRecord":
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in data.items() if k in known})


class CommitmentStore:
    def __init__(self, path: Optional[str] = DEFAULT_COMMITMENT_PATH) -> None:
        self.path = Path(os.path.expanduser(path)) if path else None
        self._records: Dict[str, CommitmentRecord] = self._load()

    # --- read -----------------------------------------------------------------

    def _load(self) -> Dict[str, CommitmentRecord]:
        if self.path is None:
            return {}
        try:
            data = json.loads(self.path.read_text())
        except FileNotFoundError:
            return {}
        except (OSError, ValueError) as e:
            # Unreadable marks mean redelivered envelopes may look new. Loud,
            # because the fence check is what stops a replayed OFFER.
            log_event(
                logger,
                "commitments.unreadable",
                f"could not read commitment high-water marks: {e}",
                level=logging.ERROR,
                path=str(self.path),
            )
            return {}
        records = {}
        for item in data.get("commitments", []) if isinstance(data, dict) else []:
            try:
                record = CommitmentRecord.from_dict(item)
            except TypeError:
                continue
            records[record.commitment_id] = record
        return records

    def get(self, commitment_id: str) -> Optional[CommitmentRecord]:
        return self._records.get(commitment_id)

    def held(self) -> Optional[CommitmentRecord]:
        """The accepted commitment in progress, if any (the Rover holds at most one)."""

        held = [r for r in self._records.values() if r.is_held]
        return max(held, key=lambda r: r.updated_at) if held else None

    def all(self) -> List[CommitmentRecord]:
        return list(self._records.values())

    # --- write ----------------------------------------------------------------

    def update(self, commitment_id: str, **changes: Any) -> Optional[CommitmentRecord]:
        """Apply changes, persisting first. None when the write failed."""

        current = self._records.get(commitment_id) or CommitmentRecord(commitment_id)
        updated = replace(current, updated_at=time.time(), **changes)
        if len(updated.acked_outbox_ids) > MAX_OUTBOX_IDS:
            updated = replace(updated, acked_outbox_ids=updated.acked_outbox_ids[-MAX_OUTBOX_IDS:])

        candidate = dict(self._records)
        candidate[commitment_id] = updated
        if len(candidate) > MAX_RECORDS:
            for old in sorted(candidate.values(), key=lambda r: r.updated_at)[: len(candidate) - MAX_RECORDS]:
                candidate.pop(old.commitment_id, None)

        if not self._write(candidate):
            return None
        self._records = candidate
        return updated

    def _write(self, records: Dict[str, CommitmentRecord]) -> bool:
        if self.path is None:
            return True
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            os.chmod(self.path.parent, 0o700)
            fd, tmp_name = tempfile.mkstemp(dir=str(self.path.parent), prefix=".commitments-")
            try:
                os.fchmod(fd, 0o600)
                with os.fdopen(fd, "w") as handle:
                    json.dump({"commitments": [asdict(r) for r in records.values()]}, handle)
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(tmp_name, self.path)
            except BaseException:
                try:
                    os.unlink(tmp_name)
                except OSError:
                    pass
                raise
        except OSError as e:
            log_event(
                logger,
                "commitments.unwritable",
                f"could not persist commitment high-water marks: {e}",
                level=logging.ERROR,
                path=str(self.path),
            )
            return False
        return True
