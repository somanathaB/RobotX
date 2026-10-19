"""The mission fixture: the body of `POST /test/mission/inject`, read from a file.

The schema and every rule live in production, in
`robotx.application.injection.validate_request`, so the harness and the agent
can never disagree about what a valid test task is. This module only loads a
file and keeps the harness's names for the same things.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Union

from robotx.application.injection import (  # noqa: F401 - re-exported
    MAX_REGION_RADIUS_M,
    PROVENANCE_REASON,
    PROVENANCE_SOURCE as PROVENANCE,
    REQUIRED_FIELDS,
    SCHEMA_VERSION,
    TASK_ID_PREFIX,
    InjectedAssignment,
    InjectionRejected as FixtureError,
    InjectionRequest as MissionFixture,
    distance_m,
    validate_request as validate_fixture,
)


def load_fixture(source: Union[str, Path, Dict[str, Any]], **kwargs: Any) -> MissionFixture:
    if isinstance(source, dict):
        return validate_fixture(source, **kwargs)
    try:
        raw = json.loads(Path(source).read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        raise FixtureError("UNREADABLE", f"{source}: {e}") from None
    return validate_fixture(raw, **kwargs)
