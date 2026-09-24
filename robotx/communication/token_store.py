"""Where the robot's FalconAut session token lives between runs.

Why this exists
---------------
Commissioning produces a **6-digit pairing code with a 300 second TTL**. It is
a one-time bootstrap, not a credential: if the robot needed one on every
connect, a person would have to stand at a dashboard and issue a fresh code
every time the Wi-Fi blinked or the agent restarted. `AUTH_SUCCESS` returns a
session token precisely so that does not happen, and the token is only useful
if it outlives the process.

How secure is this, honestly
----------------------------
It is a JSON file with mode 0600, owned by the user running the agent. That
protects it from other accounts on the Pi and from a world-readable backup. It
does **not** protect it from anyone with root, physical access to the SD card,
or the ability to read this process's memory. For an MVP on a trusted network
that is the right trade; a deployment that needs more wants the Pi's TPM or an
encrypted partition, neither of which is in scope here.

The file is written atomically (temp file + rename) because the alternative --
a half-written token after a power cut mid-write -- would send the robot back
to manual commissioning for no reason.
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

from robotx.config.logging_setup import log_event


logger = logging.getLogger(__name__)

DEFAULT_TOKEN_PATH = "~/.robotx/backend_session.json"


@dataclass(frozen=True)
class StoredSession:
    """A persisted FalconAut session, as read back from disk."""

    robot_id: str
    token: str
    obtained_at: float

    @property
    def age_s(self) -> float:
        return max(0.0, time.time() - self.obtained_at)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robotId": self.robot_id,
            "token": self.token,
            "obtainedAt": self.obtained_at,
        }


class TokenStore:
    """Reads and writes one robot's session token.

    Every method is failure-tolerant: a token that cannot be read or written is
    a reason to fall back to the pairing code, never a reason to stop the
    agent. A robot that refuses to boot because it could not write a cache file
    is worse than one that asks to be commissioned again.
    """

    def __init__(self, path: Optional[str] = None) -> None:
        self.path = Path(os.path.expanduser(path or DEFAULT_TOKEN_PATH))

    # --- read -----------------------------------------------------------------

    def load(self, *, robot_id: str) -> Optional[StoredSession]:
        """The stored session for this robot, or None.

        A token stored under a *different* `robotId` is ignored rather than
        used. Presenting robot A's token while claiming to be robot B is a
        request the backend should refuse, and getting a silent disconnect for
        it would be very hard to diagnose from the Pi.
        """

        try:
            raw = self.path.read_text()
        except FileNotFoundError:
            return None
        except OSError as e:
            log_event(
                logger,
                "token_store.unreadable",
                f"could not read the session token: {e}",
                level=logging.WARNING,
                path=str(self.path),
            )
            return None

        try:
            data = json.loads(raw)
        except ValueError:
            log_event(
                logger,
                "token_store.corrupt",
                "session token file is not valid JSON; ignoring it",
                level=logging.WARNING,
                path=str(self.path),
            )
            return None

        if not isinstance(data, dict):
            return None

        token = data.get("token")
        stored_id = data.get("robotId")
        if not isinstance(token, str) or not token.strip():
            return None
        if stored_id != robot_id:
            log_event(
                logger,
                "token_store.wrong_robot",
                "stored token belongs to a different robot; ignoring it",
                level=logging.WARNING,
                stored_robot_id=str(stored_id)[:40],
                robot_id=robot_id,
            )
            return None

        obtained = data.get("obtainedAt")
        return StoredSession(
            robot_id=robot_id,
            token=token.strip(),
            obtained_at=float(obtained) if isinstance(obtained, (int, float)) else 0.0,
        )

    # --- write ----------------------------------------------------------------

    def save(self, *, robot_id: str, token: str) -> bool:
        """Persist a session token. Returns whether it reached disk.

        Written to a temp file in the same directory and renamed, so a reader
        either sees the old token or the new one and never a truncated file.
        """

        session = StoredSession(robot_id=robot_id, token=token, obtained_at=time.time())
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            # 0700 on the directory too: the file mode alone does not stop
            # another account from watching the directory for new names.
            os.chmod(self.path.parent, 0o700)

            fd, tmp_name = tempfile.mkstemp(dir=str(self.path.parent), prefix=".session-")
            try:
                os.fchmod(fd, 0o600)
                with os.fdopen(fd, "w") as handle:
                    json.dump(session.to_dict(), handle)
                os.replace(tmp_name, self.path)
            except BaseException:
                # Never leave a temp file holding a live credential behind.
                try:
                    os.unlink(tmp_name)
                except OSError:
                    pass
                raise
        except OSError as e:
            log_event(
                logger,
                "token_store.unwritable",
                f"could not persist the session token: {e}. The robot will need a "
                f"fresh pairing code after the next restart.",
                level=logging.WARNING,
                path=str(self.path),
            )
            return False

        # The token itself is never logged, here or anywhere else.
        log_event(logger, "token_store.saved", path=str(self.path), robot_id=robot_id)
        return True

    def clear(self) -> None:
        """Forget the stored token, e.g. after the backend rejects it."""

        try:
            self.path.unlink()
        except FileNotFoundError:
            return
        except OSError as e:
            log_event(
                logger,
                "token_store.unremovable",
                f"could not remove the stale session token: {e}",
                level=logging.WARNING,
                path=str(self.path),
            )
            return
        log_event(logger, "token_store.cleared", path=str(self.path))

    def describe(self, *, robot_id: str) -> Dict[str, Any]:
        """Whether a token exists, never what it is."""

        session = self.load(robot_id=robot_id)
        return {
            "path": str(self.path),
            "token": "SET" if session else "UNSET",
            "age_s": None if session is None else round(session.age_s, 1),
        }
