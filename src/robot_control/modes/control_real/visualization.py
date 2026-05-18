"""Viewer loop for real SocketCAN control mode."""

from __future__ import annotations

import time
import threading

from robot_control.config import Config
from robot_control.shared.mujoco.env import MujocoSimEnv
from robot_control.shared.mujoco.ghost import create_mujoco_ghost_if_enabled
from robot_control.shared.mujoco.viewer import launch_passive_viewer
from robot_control.shared.state import SharedRobotState
from robot_control.shared.transforms import RobotMujocoTransformer


def run_viewer_loop(
    env: MujocoSimEnv,
    shared_state: SharedRobotState,
    transformer: RobotMujocoTransformer,
    shutdown_event: threading.Event,
) -> None:
    import mujoco

    model = env.model
    data = env.data
    mocap_idx_reported = env.reported_mocap_id

    print("\n[Running] 双回路运行中: CAN 线程 (MIT torque) | 主线程 (MuJoCo 渲染)")
    with launch_passive_viewer(model, data) as viewer:
        ghost = create_mujoco_ghost_if_enabled(
            model,
            data,
            enabled=Config.ENABLE_MUJOCO_GHOST,
            alpha=Config.MUJOCO_GHOST_ALPHA,
        )
        while viewer.is_running() and not shutdown_event.is_set():
            current_q, reported_pos, reported_quat = shared_state.snapshot_viewer_state()
            active_joints = min(len(current_q), model.nq)
            data.qpos[:active_joints] = current_q[:active_joints]

            if mocap_idx_reported >= 0:
                mj_pos, mj_quat = transformer.robot_to_mujoco(reported_pos, reported_quat)
                data.mocap_pos[mocap_idx_reported] = mj_pos
                data.mocap_quat[mocap_idx_reported] = mj_quat

            mujoco.mj_forward(model, data)
            viewer.sync()

            dragged_target_pos_mj, dragged_target_quat_mj = env.get_target_pose()
            dragged_target_pos, dragged_target_quat = transformer.mujoco_to_robot(
                dragged_target_pos_mj,
                dragged_target_quat_mj,
            )
            shared_state.set_target_pose(dragged_target_pos, dragged_target_quat)
            if ghost is not None:
                ghost.update_from_pose(dragged_target_pos_mj, dragged_target_quat_mj)

            time.sleep(1.0 / Config.REAL_VIEWER_FPS)
