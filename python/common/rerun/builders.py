"""Small Rerun style builders used by mode-specific layouts."""

from __future__ import annotations

try:
    import rerun as rr

    RERUN_AVAILABLE = True
except ImportError:
    rr = None
    RERUN_AVAILABLE = False


def log_series_style(path: str, *, names, colors=None, widths=None, static: bool = True) -> None:
    if not RERUN_AVAILABLE:
        return
    kwargs = {"names": list(names)}
    if colors is not None:
        kwargs["colors"] = colors
    if widths is not None:
        kwargs["widths"] = widths
    rr.log(path, rr.SeriesLines(**kwargs), static=static)

