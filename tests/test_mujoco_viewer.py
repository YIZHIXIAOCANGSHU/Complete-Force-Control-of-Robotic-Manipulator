from __future__ import annotations

import sys

import numpy as np
import pytest

from robot_control.shared.mujoco import viewer as mujoco_viewer


class DummyViewerModule:
    def __init__(self) -> None:
        self.calls: list[tuple[object, object, dict[str, bool]]] = []

    def launch_passive(self, model, data, **kwargs):
        self.calls.append((model, data, kwargs))
        return "viewer-handle"


def test_launch_passive_viewer_hides_both_side_panels(monkeypatch):
    dummy_viewer_module = DummyViewerModule()
    monkeypatch.setattr(mujoco_viewer, "_viewer_module", dummy_viewer_module)
    monkeypatch.setattr(mujoco_viewer, "_verify_glfw_context_available", lambda: None)

    viewer = mujoco_viewer.launch_passive_viewer("model", "data")

    assert viewer == "viewer-handle"
    assert dummy_viewer_module.calls == [
        (
            "model",
            "data",
            {
                "show_left_ui": False,
                "show_right_ui": False,
            },
        )
    ]


def test_launch_passive_viewer_reports_unavailable_glfw_context(monkeypatch):
    dummy_viewer_module = DummyViewerModule()
    monkeypatch.setattr(mujoco_viewer, "_viewer_module", dummy_viewer_module)

    def fail_preflight():
        raise RuntimeError("GLFW OpenGL context is not available: no context")

    monkeypatch.setattr(mujoco_viewer, "_verify_glfw_context_available", fail_preflight)

    with pytest.raises(RuntimeError, match="no context"):
        mujoco_viewer.launch_passive_viewer("model", "data")

    assert dummy_viewer_module.calls == []


def test_glfw_preflight_rejects_context_error_even_when_window_is_returned(monkeypatch):
    class FakeGlfw:
        FALSE = 0
        NO_ERROR = 0
        VISIBLE = 1

        def __init__(self) -> None:
            self.destroyed = False
            self.terminated = False
            self.previous_callback = object()
            self.current_callback = self.previous_callback
            self.errors = [
                (self.NO_ERROR, None),
                (65543, b"GLX: Failed to create context"),
            ]

        def set_error_callback(self, callback):
            previous = self.current_callback
            self.current_callback = callback
            return previous

        def init(self):
            return True

        def window_hint(self, _hint, _value):
            pass

        def get_error(self):
            return self.errors.pop(0) if self.errors else (self.NO_ERROR, None)

        def create_window(self, *_args):
            return object()

        def destroy_window(self, _window):
            self.destroyed = True

        def terminate(self):
            self.terminated = True

    fake_glfw = FakeGlfw()
    monkeypatch.setitem(sys.modules, "glfw", fake_glfw)

    with pytest.raises(RuntimeError, match="GLX: Failed to create context"):
        mujoco_viewer._verify_glfw_context_available()

    assert fake_glfw.destroyed
    assert fake_glfw.terminated
    assert fake_glfw.current_callback is fake_glfw.previous_callback


def test_mujoco_sim_env_uses_model_default_joint_dynamics():
    pytest.importorskip("mujoco")

    from robot_control.shared.mujoco.env import MujocoSimEnv

    env = MujocoSimEnv()

    assert env.model.dof_damping[env.dof_ids].shape == (env.dof_ids.size,)
