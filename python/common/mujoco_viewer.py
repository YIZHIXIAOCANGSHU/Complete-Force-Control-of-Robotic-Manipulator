"""Helpers for launching a minimal MuJoCo passive viewer."""

from __future__ import annotations

import warnings


def _probe_glfw_context(glfw_module=None) -> str | None:
    """Return an explanatory string when GLFW cannot create a hidden window."""
    try:
        if glfw_module is None:
            import glfw as glfw_module
    except ImportError as exc:
        return f"GLFW Python binding is not available: {exc}"

    initialized = False
    window = None
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            initialized = bool(glfw_module.init())
            if not initialized:
                return "GLFW init failed"

            glfw_module.window_hint(glfw_module.VISIBLE, glfw_module.FALSE)
            window = glfw_module.create_window(64, 64, "am-d02-viewer-probe", None, None)
            if not window:
                return "GLFW could not create an OpenGL window/context"

            glfw_module.make_context_current(window)
            glfw_module.swap_buffers(window)
    except Exception as exc:
        return f"GLFW OpenGL context probe failed: {exc}"
    finally:
        if window:
            try:
                glfw_module.destroy_window(window)
            except Exception:
                pass
        if initialized:
            try:
                glfw_module.terminate()
            except Exception:
                pass

    return None


try:
    import mujoco.viewer as _viewer_module
except ImportError as exc:
    _viewer_module = None
    _VIEWER_UNAVAILABLE_REASON = f"MuJoCo viewer is not available: {exc}"
else:
    _VIEWER_UNAVAILABLE_REASON = _probe_glfw_context()


VIEWER_AVAILABLE = _viewer_module is not None and _VIEWER_UNAVAILABLE_REASON is None


def viewer_unavailable_reason() -> str | None:
    return _VIEWER_UNAVAILABLE_REASON


def launch_passive_viewer(model, data):
    if _viewer_module is None:
        raise RuntimeError("MuJoCo viewer is not available")
    if _VIEWER_UNAVAILABLE_REASON is not None:
        raise RuntimeError(_VIEWER_UNAVAILABLE_REASON)

    return _viewer_module.launch_passive(
        model,
        data,
        show_left_ui=False,
        show_right_ui=False,
    )
