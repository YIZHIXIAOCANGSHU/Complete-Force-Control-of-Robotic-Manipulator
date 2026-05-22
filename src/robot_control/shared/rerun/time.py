"""Compatibility helpers for Rerun timeline APIs."""

from __future__ import annotations


def set_time_seconds(rr_module, timeline: str, seconds: float) -> None:
    """Set a Rerun timeline in seconds across old and new SDK APIs."""
    value = float(seconds)
    if hasattr(rr_module, "set_time_seconds"):
        rr_module.set_time_seconds(timeline, value)
        return
    if hasattr(rr_module, "set_time"):
        rr_module.set_time(timeline, duration=value)
        return
    raise AttributeError("rerun SDK does not provide set_time_seconds or set_time")
