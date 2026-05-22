from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from common import mujoco_viewer


class DummyViewerModule:
    def __init__(self) -> None:
        self.calls: list[tuple[object, object, dict[str, bool]]] = []

    def launch_passive(self, model, data, **kwargs):
        self.calls.append((model, data, kwargs))
        return "viewer-handle"


class DummyGlfwModule:
    VISIBLE = "visible"
    FALSE = False

    def __init__(self, *, window) -> None:
        self.window = window
        self.terminated = False

    def init(self):
        return True

    def window_hint(self, hint, value):
        _ = (hint, value)

    def create_window(self, width, height, title, monitor, share):
        _ = (width, height, title, monitor, share)
        return self.window

    def make_context_current(self, window):
        _ = window

    def swap_buffers(self, window):
        _ = window

    def destroy_window(self, window):
        _ = window

    def terminate(self):
        self.terminated = True


def test_launch_passive_viewer_hides_both_side_panels(monkeypatch):
    dummy_viewer_module = DummyViewerModule()
    monkeypatch.setattr(mujoco_viewer, "_viewer_module", dummy_viewer_module)
    monkeypatch.setattr(mujoco_viewer, "_VIEWER_UNAVAILABLE_REASON", None)

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


def test_glfw_probe_reports_window_creation_failure():
    glfw_module = DummyGlfwModule(window=None)

    reason = mujoco_viewer._probe_glfw_context(glfw_module)

    assert reason == "GLFW could not create an OpenGL window/context"
    assert glfw_module.terminated is True


def test_launch_passive_viewer_fails_before_mujoco_when_probe_failed(monkeypatch):
    dummy_viewer_module = DummyViewerModule()
    monkeypatch.setattr(mujoco_viewer, "_viewer_module", dummy_viewer_module)
    monkeypatch.setattr(
        mujoco_viewer,
        "_VIEWER_UNAVAILABLE_REASON",
        "GLFW could not create an OpenGL window/context",
    )

    with pytest.raises(RuntimeError, match="GLFW could not create"):
        mujoco_viewer.launch_passive_viewer("model", "data")

    assert dummy_viewer_module.calls == []


def test_mujoco_sim_env_uses_model_default_joint_dynamics():
    pytest.importorskip("mujoco")

    from sim.env import MujocoSimEnv

    env = MujocoSimEnv()

    assert env.model.dof_damping[env.dof_ids].shape == (env.dof_ids.size,)
