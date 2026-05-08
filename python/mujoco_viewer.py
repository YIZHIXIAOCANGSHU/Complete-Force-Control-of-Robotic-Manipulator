"""Helpers for launching a minimal MuJoCo passive viewer."""

from __future__ import annotations

try:
    import mujoco.viewer as _viewer_module
except ImportError:
    _viewer_module = None


VIEWER_AVAILABLE = _viewer_module is not None


def launch_passive_viewer(model, data):
    if _viewer_module is None:
        raise RuntimeError("MuJoCo viewer is not available")

    return _viewer_module.launch_passive(
        model,
        data,
        show_left_ui=False,
        show_right_ui=False,
    )
