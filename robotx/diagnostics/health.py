"""Health monitoring for the Pi agent.

Reports only what this machine can actually measure. System metrics come from
the Linux interfaces the Pi already exposes -- `/proc/stat`, `/proc/meminfo`,
`/proc/loadavg`, `/sys/class/thermal` -- so there is no extra dependency and no
value is invented. Anything unreadable is reported as `None`, never as zero and
never as a plausible-looking default.

There is no battery sensing on this robot: no fuel gauge IC, no ADC, no voltage
divider referenced anywhere in the wiring. Battery therefore reports
`UNKNOWN`/`None` rather than a number. Hardware that does not exist is not
given a health status.
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

from robotx.config.logging_setup import log_event


logger = logging.getLogger(__name__)


class HealthStatus(str, Enum):
    """Three states, ordered by severity."""

    HEALTHY = "HEALTHY"    # working as expected
    DEGRADED = "DEGRADED"  # working, but something is wrong or unavailable
    FAILED = "FAILED"      # not working
    UNKNOWN = "UNKNOWN"    # not measurable on this hardware

    @property
    def severity(self) -> int:
        return {"HEALTHY": 0, "UNKNOWN": 1, "DEGRADED": 2, "FAILED": 3}[self.value]


@dataclass(frozen=True)
class ComponentHealth:
    name: str
    status: HealthStatus
    detail: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {"status": self.status.value, "detail": self.detail}


@dataclass(frozen=True)
class SystemMetrics:
    """Host metrics. Every field is Optional: unreadable means None."""

    cpu_percent: Optional[float] = None
    memory_used_percent: Optional[float] = None
    memory_available_mb: Optional[float] = None
    cpu_temp_c: Optional[float] = None
    load_avg_1m: Optional[float] = None
    disk_used_percent: Optional[float] = None
    uptime_s: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        def r(v: Optional[float], digits: int = 1) -> Optional[float]:
            return None if v is None else round(v, digits)

        return {
            "cpu_percent": r(self.cpu_percent),
            "memory_used_percent": r(self.memory_used_percent),
            "memory_available_mb": r(self.memory_available_mb),
            "cpu_temp_c": r(self.cpu_temp_c),
            "load_avg_1m": r(self.load_avg_1m, 2),
            "disk_used_percent": r(self.disk_used_percent),
            "uptime_s": r(self.uptime_s, 0),
        }


@dataclass(frozen=True)
class HealthReport:
    status: HealthStatus
    components: Dict[str, ComponentHealth] = field(default_factory=dict)
    system: SystemMetrics = field(default_factory=SystemMetrics)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status.value,
            "components": {k: v.to_dict() for k, v in self.components.items()},
            "system": self.system.to_dict(),
            "timestamp": self.timestamp,
        }


# --- /proc and /sys readers ---------------------------------------------------


def read_cpu_temp_c() -> Optional[float]:
    """CPU temperature in degrees Celsius, or None if unavailable."""

    for path in (
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/devices/virtual/thermal/thermal_zone0/temp",
    ):
        try:
            raw = Path(path).read_text().strip()
        except OSError:
            continue
        try:
            value = float(raw)
        except ValueError:
            continue
        # Kernel reports millidegrees on the Pi.
        return value / 1000.0 if value > 200 else value
    return None


def read_memory() -> Tuple[Optional[float], Optional[float]]:
    """(used percent, available MB) from /proc/meminfo."""

    try:
        lines = Path("/proc/meminfo").read_text().splitlines()
    except OSError:
        return None, None

    values: Dict[str, float] = {}
    for line in lines:
        key, _, rest = line.partition(":")
        if key in ("MemTotal", "MemAvailable"):
            try:
                values[key] = float(rest.strip().split()[0])  # kB
            except (IndexError, ValueError):
                pass

    total = values.get("MemTotal")
    available = values.get("MemAvailable")
    if not total or available is None:
        return None, None
    return (total - available) / total * 100.0, available / 1024.0


def read_load_avg() -> Optional[float]:
    try:
        return os.getloadavg()[0]
    except OSError:
        return None


def read_uptime_s() -> Optional[float]:
    try:
        return float(Path("/proc/uptime").read_text().split()[0])
    except (OSError, IndexError, ValueError):
        return None


def read_disk_used_percent(path: str = "/") -> Optional[float]:
    try:
        stat = os.statvfs(path)
    except OSError:
        return None
    total = stat.f_blocks * stat.f_frsize
    if total <= 0:
        return None
    free = stat.f_bavail * stat.f_frsize
    return (total - free) / total * 100.0


class _CpuSampler:
    """CPU utilisation between successive /proc/stat reads."""

    def __init__(self) -> None:
        self._prev: Optional[Tuple[float, float]] = None

    def sample(self) -> Optional[float]:
        try:
            first_line = Path("/proc/stat").read_text().split("\n", 1)[0]
        except OSError:
            return None

        parts = first_line.split()
        if len(parts) < 5 or parts[0] != "cpu":
            return None
        try:
            fields = [float(v) for v in parts[1:]]
        except ValueError:
            return None

        idle = fields[3] + (fields[4] if len(fields) > 4 else 0.0)
        total = sum(fields)

        previous = self._prev
        self._prev = (idle, total)
        if previous is None:
            return None  # first sample establishes the baseline

        d_idle = idle - previous[0]
        d_total = total - previous[1]
        if d_total <= 0:
            return None
        return max(0.0, min(100.0, (1.0 - d_idle / d_total) * 100.0))


# --- monitor ------------------------------------------------------------------


@dataclass(frozen=True)
class HealthConfig:
    cpu_warn_percent: float = 90.0
    memory_warn_percent: float = 90.0
    temp_warn_c: float = 80.0

    @classmethod
    def from_settings(cls, settings: Any) -> "HealthConfig":
        return cls(
            cpu_warn_percent=settings.health_cpu_warn_percent,
            memory_warn_percent=settings.health_memory_warn_percent,
            temp_warn_c=settings.health_temp_warn_c,
        )


class HealthMonitor:
    """Aggregates subsystem statuses and host metrics into one report."""

    def __init__(self, cfg: Optional[HealthConfig] = None) -> None:
        self.cfg = cfg or HealthConfig()
        self._cpu = _CpuSampler()
        self._last_status: Optional[HealthStatus] = None
        self._report = HealthReport(status=HealthStatus.UNKNOWN)

    @property
    def last_report(self) -> HealthReport:
        return self._report

    def read_system_metrics(self) -> SystemMetrics:
        memory_percent, memory_available = read_memory()
        return SystemMetrics(
            cpu_percent=self._cpu.sample(),
            memory_used_percent=memory_percent,
            memory_available_mb=memory_available,
            cpu_temp_c=read_cpu_temp_c(),
            load_avg_1m=read_load_avg(),
            disk_used_percent=read_disk_used_percent(),
            uptime_s=read_uptime_s(),
        )

    def evaluate(self, components: Dict[str, ComponentHealth]) -> HealthReport:
        """Combine subsystem health with host metrics into an overall status."""

        metrics = self.read_system_metrics()
        all_components = dict(components)
        all_components["system"] = self._system_health(metrics)

        overall = HealthStatus.HEALTHY
        for component in all_components.values():
            if component.status.severity > overall.severity:
                overall = component.status
        # UNKNOWN subsystems do not, alone, make the agent unhealthy; they make
        # it degraded, because something expected is not reporting.
        if overall is HealthStatus.UNKNOWN:
            overall = HealthStatus.DEGRADED

        report = HealthReport(
            status=overall, components=all_components, system=metrics
        )
        self._report = report
        self._log_transition(report)
        return report

    def _system_health(self, metrics: SystemMetrics) -> ComponentHealth:
        problems = []
        if metrics.cpu_percent is not None and metrics.cpu_percent >= self.cfg.cpu_warn_percent:
            problems.append(f"cpu {metrics.cpu_percent:.0f}%")
        if (
            metrics.memory_used_percent is not None
            and metrics.memory_used_percent >= self.cfg.memory_warn_percent
        ):
            problems.append(f"memory {metrics.memory_used_percent:.0f}%")
        if metrics.cpu_temp_c is not None and metrics.cpu_temp_c >= self.cfg.temp_warn_c:
            problems.append(f"temp {metrics.cpu_temp_c:.0f}C")

        if problems:
            return ComponentHealth("system", HealthStatus.DEGRADED, ", ".join(problems))
        if metrics.cpu_percent is None and metrics.memory_used_percent is None:
            return ComponentHealth("system", HealthStatus.UNKNOWN, "no host metrics readable")
        return ComponentHealth("system", HealthStatus.HEALTHY, "")

    def _log_transition(self, report: HealthReport) -> None:
        if report.status is self._last_status:
            return

        detail = ", ".join(
            f"{name}={c.status.value}"
            for name, c in report.components.items()
            if c.status is not HealthStatus.HEALTHY
        )
        level = (
            logging.ERROR
            if report.status is HealthStatus.FAILED
            else logging.WARNING
            if report.status is HealthStatus.DEGRADED
            else logging.INFO
        )
        log_event(
            logger,
            "health.changed",
            level=level,
            status=report.status.value,
            issues=detail or "none",
        )
        self._last_status = report.status
