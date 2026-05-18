from __future__ import annotations

import numpy as np
import pytest

from robot_control.shared.mujoco.ghost import MujocoGhostRobot, create_mujoco_ghost_if_enabled


class DummyModel:
    def __init__(self) -> None:
        self.body_mocapid = np.array([0], dtype=np.int32)
        self.geom_rgba = np.array([[0.2, 0.6, 1.0, 0.25]], dtype=np.float64)


class DummyData:
    def __init__(self) -> None:
        self.mocap_pos = np.zeros((1, 3), dtype=np.float64)
        self.mocap_quat = np.zeros((1, 4), dtype=np.float64)


def test_ghost_updates_pose_without_touching_dynamics():
    model = DummyModel()
    data = DummyData()
    ghost = MujocoGhostRobot(model, data)

    ghost.update_from_qpos(np.zeros(7))
    ghost.update_from_pose(np.array([1.0, 2.0, 3.0]), np.array([1.0, 0.0, 0.0, 0.0]))

    assert np.allclose(data.mocap_pos[0], [1.0, 2.0, 3.0])
    assert np.allclose(data.mocap_quat[0], [1.0, 0.0, 0.0, 0.0])
    assert ghost._last_qpos.shape == (7,)


def test_ghost_rejects_wrong_qpos_shape_and_can_disable_visibility():
    model = DummyModel()
    data = DummyData()
    ghost = MujocoGhostRobot(model, data)

    with pytest.raises(ValueError, match="q_target"):
        ghost.update_from_qpos(np.zeros(6))

    ghost.set_visible(False)

    assert not ghost.visible
    assert model.geom_rgba[0, 3] == 0.0


def test_ghost_factory_returns_none_when_disabled():
    assert create_mujoco_ghost_if_enabled(DummyModel(), DummyData(), enabled=False) is None


def test_real_mujoco_ghost_updates_target_pose_from_qpos():
    pytest.importorskip("mujoco")

    from robot_control.shared.mujoco.env import MujocoSimEnv

    env = MujocoSimEnv()
    ghost = MujocoGhostRobot(env.model, env.data, dof_ids=env.dof_ids)
    q_target = np.asarray(env.clip_qpos(np.zeros(7)), dtype=np.float64)

    ghost.update_from_qpos(q_target)

    assert ghost._last_qpos.shape == (7,)
    assert np.linalg.norm(env.data.mocap_pos[env.target_mocap_id]) > 0.0
