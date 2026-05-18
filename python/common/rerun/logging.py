"""Mode-aware scalar logging helpers for Rerun."""

from __future__ import annotations

try:
    import rerun as rr

    RERUN_AVAILABLE = True
except ImportError:
    rr = None
    RERUN_AVAILABLE = False


def log_scalar(path: str, value: float, *, t: float | None = None) -> None:
    if not RERUN_AVAILABLE:
        return
    if t is not None:
        rr.set_time_seconds("time", float(t))
    rr.log(path, rr.Scalars(float(value)))

