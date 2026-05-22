"""Helpers for launching a minimal MuJoCo passive viewer."""

from __future__ import annotations

try:
    import mujoco.viewer as _viewer_module
except ImportError:
    _viewer_module = None


VIEWER_AVAILABLE = _viewer_module is not None


def _format_glfw_error(error) -> str:
    code, description = error
    if isinstance(description, bytes):
        description = description.decode("utf-8", errors="replace")
    if description:
        return f"{description} (code {code})"
    return f"GLFW error code {code}"


def _verify_glfw_context_available() -> None:
    try:
        import glfw
    except ImportError as exc:
        raise RuntimeError("GLFW is not available") from exc

    previous_error_callback = glfw.set_error_callback(lambda _code, _description: None)
    if not glfw.init():
        init_error = glfw.get_error()
        glfw.set_error_callback(previous_error_callback)
        raise RuntimeError(f"GLFW initialization failed: {_format_glfw_error(init_error)}")

    window = None
    try:
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        glfw.get_error()
        window = glfw.create_window(1, 1, "mujoco-viewer-check", None, None)
        context_error = glfw.get_error()
        if window is None or context_error[0] != glfw.NO_ERROR:
            raise RuntimeError(
                f"GLFW OpenGL context is not available: {_format_glfw_error(context_error)}"
            )
    finally:
        if window is not None:
            glfw.destroy_window(window)
        glfw.terminate()
        glfw.set_error_callback(previous_error_callback)


def launch_passive_viewer(model, data):
    if _viewer_module is None:
        raise RuntimeError("MuJoCo viewer is not available")

    _verify_glfw_context_available()

    return _viewer_module.launch_passive(
        model,
        data,
        show_left_ui=False,
        show_right_ui=False,
    )
