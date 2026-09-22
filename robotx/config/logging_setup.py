"""One place that configures logging for the whole Pi agent.

Every module uses `logging.getLogger(__name__)`; only the application entry
point calls `setup_logging()`. `log_event()` exists so lifecycle-significant
transitions (startup, hardware connect/disconnect, health degradation) are
greppable by a stable event name instead of prose.
"""

from __future__ import annotations

import logging
from typing import Any, Optional


LOG_FORMAT = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"

_configured = False


def setup_logging(level: str = "INFO", *, force: bool = False) -> None:
    """Configure root logging once. Safe to call more than once."""

    global _configured
    if _configured and not force:
        return

    numeric = getattr(logging, str(level).upper(), None)
    if not isinstance(numeric, int):
        numeric = logging.INFO

    logging.basicConfig(level=numeric, format=LOG_FORMAT, force=True)

    # These libraries are chatty at DEBUG and drown out the agent's own events.
    for noisy in ("engineio", "socketio", "urllib3", "httpx", "httpcore", "picamera2"):
        logging.getLogger(noisy).setLevel(max(numeric, logging.WARNING))

    _configured = True


def log_event(
    logger: logging.Logger,
    event: str,
    message: str = "",
    *,
    level: int = logging.INFO,
    exc_info: Any = None,
    **fields: Any,
) -> None:
    """Emit a log line tagged with a stable, greppable event name.

    Example: `log_event(log, "camera.connected", width=640, height=480)`
    renders as `[camera.connected] width=640 height=480`.
    """

    parts = [f"[{event}]"]
    if message:
        parts.append(message)
    if fields:
        parts.append(" ".join(f"{k}={v}" for k, v in fields.items()))
    logger.log(level, " ".join(parts), exc_info=exc_info)


def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name)


def describe_level(level: Optional[str]) -> str:
    return str(level or "INFO").upper()
