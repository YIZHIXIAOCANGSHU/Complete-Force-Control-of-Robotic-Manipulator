"""Reusable Rerun layout fragments."""

from __future__ import annotations

try:
    import rerun.blueprint as rrb

    RERUN_BLUEPRINT_AVAILABLE = True
except ImportError:
    rrb = None
    RERUN_BLUEPRINT_AVAILABLE = False


def mode_timeseries_view(name: str, origin: str):
    if not RERUN_BLUEPRINT_AVAILABLE:
        return None
    return rrb.TimeSeriesView(name=name, origin=origin)


def mode_spatial_view(name: str, origin: str):
    if not RERUN_BLUEPRINT_AVAILABLE:
        return None
    return rrb.Spatial3DView(name=name, origin=origin)

