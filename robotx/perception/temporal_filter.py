from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
from typing import Deque, Dict, Iterable, List, Sequence, Set, Tuple

from robotx.perception.object_tracker import Track


@dataclass(frozen=True)
class TemporalFilterConfig:
    window_size: int = 6
    min_presence: int = 3  # required occurrences within window


class TemporalFilter:
    """Sliding-window temporal filter over *confirmed* tracks.

    This does two jobs:
    - Stabilize which labels/tracks are considered "present"
    - Provide stable label set for downstream decisions
    """

    def __init__(self, cfg: TemporalFilterConfig) -> None:
        self.cfg = cfg
        self._label_window: Deque[Set[str]] = deque(maxlen=int(cfg.window_size))
        self._track_window: Deque[Set[int]] = deque(maxlen=int(cfg.window_size))

    def update(self, confirmed_tracks: Sequence[Track]) -> Dict[str, object]:
        labels = {t.label for t in confirmed_tracks}
        track_ids = {int(t.track_id) for t in confirmed_tracks}

        self._label_window.append(labels)
        self._track_window.append(track_ids)

        stable_labels = self._stable_from_window(self._label_window)
        stable_track_ids = self._stable_from_window(self._track_window)

        stable_tracks = [t for t in confirmed_tracks if int(t.track_id) in stable_track_ids]

        return {
            "stable_labels": sorted(stable_labels),
            "stable_tracks": stable_tracks,
        }

    def _stable_from_window(self, window: Iterable[Set[object]]) -> Set[object]:
        counts: Counter = Counter()
        for s in window:
            for item in s:
                counts[item] += 1

        out: Set[object] = set()
        for item, c in counts.items():
            if int(c) >= int(self.cfg.min_presence):
                out.add(item)
        return out


class ActionSmoother:
    """Stabilize actions (STOP/SLOW/MOVE_FORWARD) to avoid flicker.

    Safety-priority behavior:
    - If STOP is present anywhere in the window, return STOP
    - Else if SLOW is present anywhere in the window, return SLOW
    - Else return the most recent raw action
    """

    def __init__(self, window_size: int = 5) -> None:
        self._w: Deque[str] = deque(maxlen=int(window_size))

    def update(self, raw_action: str) -> str:
        raw_action = str(raw_action)
        self._w.append(raw_action)
        if not self._w:
            return raw_action

        # 🚨 PRIORITY SAFETY
        if "STOP" in self._w:
            return "STOP"

        if "SLOW" in self._w:
            return "SLOW"

        return raw_action