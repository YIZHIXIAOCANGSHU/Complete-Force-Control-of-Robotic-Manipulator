"""Mode-aware scalar logging helpers for Rerun."""

from __future__ import annotations

try:
    import rerun as rr

    RERUN_AVAILABLE = True
except ImportError:
    rr = None
    RERUN_AVAILABLE = False

from robot_control.shared.rerun.time import set_time_seconds


def log_scalar(path: str, value: float, *, t: float | None = None) -> None:
    if not RERUN_AVAILABLE:
        return
    if t is not None:
        set_time_seconds(rr, "time", float(t))
    rr.log(path, rr.Scalars(float(value)))
