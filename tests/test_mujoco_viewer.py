from __future__ import annotations

import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import mujoco_viewer
import udp_server


class DummyViewerModule:
    def __init__(self) -> None:
        self.calls: list[tuple[object, object, dict[str, bool]]] = []

    def launch_passive(self, model, data, **kwargs):
        self.calls.append((model, data, kwargs))
        return "viewer-handle"


def test_launch_passive_viewer_hides_both_side_panels(monkeypatch):
    dummy_viewer_module = DummyViewerModule()
    monkeypatch.setattr(mujoco_viewer, "_viewer_module", dummy_viewer_module)

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


class DummyEnv:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def step(self) -> None:
        self.calls.append("step")


class DummyViewer:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def sync(self) -> None:
        self.calls.append("sync")


def test_step_env_with_viewer_sync_pulls_perturbations_before_step():
    env = DummyEnv()
    viewer = DummyViewer()
    order: list[str] = []

    env.calls = order
    viewer.calls = order

    udp_server._step_env_with_viewer_sync(env, viewer)

    assert order == ["sync", "step", "sync"]
