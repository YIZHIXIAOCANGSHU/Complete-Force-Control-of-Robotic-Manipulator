from __future__ import annotations

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


def test_mujoco_sim_env_uses_model_default_joint_dynamics():
    pytest.importorskip("mujoco")

    from robot_control.shared.mujoco.env import MujocoSimEnv

    env = MujocoSimEnv()

    assert env.model.dof_damping[env.dof_ids].shape == (env.dof_ids.size,)
