#!/usr/bin/env python3
"""Real-hardware SocketCAN USB2FDCAN control application."""

from __future__ import annotations

import threading
import time

from robot_control.config import Config
from robot_control.dynamics.gravity import GravityCompTool
from robot_control.modes.control_real.can_loop import can_thread_func, shutdown_event
from robot_control.modes.control_real.runtime_config import (
    CAN_DATA_BITRATE,
    CAN_FORCE_FD,
    CAN_INTERFACE,
    CAN_NOMINAL_BITRATE,
    open_can_transport,
)
from robot_control.modes.control_real.visualization import run_viewer_loop
from robot_control.shared.mujoco.env import MujocoSimEnv
from robot_control.shared.mujoco.viewer import VIEWER_AVAILABLE
from robot_control.shared.rerun import viz as rerun_viz
from robot_control.shared.rerun.async_logger import RerunLogger
from robot_control.shared.state import SharedRobotState
from robot_control.shared.transforms import RobotMujocoTransformer


def main() -> None:
    print("=" * 60)
    print("      AM-D02 SocketCAN USB2FDCAN Real Control Program    ")
    print("=" * 60)

    shutdown_event.clear()
    shared_state = SharedRobotState()
    rerun_logger = None

    if Config.ENABLE_RERUN:
        rerun_viz.init_rerun("AM-D02 SocketCAN Real Control")
        rerun_viz.setup_realtime_styles()
        rerun_logger = RerunLogger()
        rerun_logger.start()

    try:
        comp_tool = GravityCompTool()
        print("[System] Pinocchio 计算后端已就绪。")
    except Exception as exc:
        print(f"[Error] 启动 Pinocchio 计算后端失败: {exc}")
        if rerun_logger is not None:
            rerun_logger.close()
        return

    try:
        transport = open_can_transport()
        print(
            "[System] SocketCAN 已就绪: "
            f"{CAN_INTERFACE}, nominal={CAN_NOMINAL_BITRATE}, data={CAN_DATA_BITRATE}, force_fd={int(CAN_FORCE_FD)}"
        )
    except Exception as exc:
        print(f"[Error] 无法打开 SocketCAN USB2FDCAN: {exc}")
        comp_tool.close()
        if rerun_logger is not None:
            rerun_logger.close()
        return

    initial_target_pos, initial_target_quat = comp_tool.compute_fk(Config.TARGET_Q.tolist())
    shared_state.set_target_pose(initial_target_pos, initial_target_quat)

    print("[System] 正在加载 MuJoCo 场景模型 (MujocoSimEnv)...")
    env = None
    mujoco_ready = False
    transformer = None
    try:
        import mujoco  # noqa: F401

        if not VIEWER_AVAILABLE:
            raise RuntimeError("MuJoCo viewer is not available")

        env = MujocoSimEnv()
        env.reset(Config.HOME_QPOS)
        env.forward()

        transformer = RobotMujocoTransformer()
        initial_mj_pos, initial_mj_quat = transformer.robot_to_mujoco(initial_target_pos, initial_target_quat)
        env.set_target_pose(initial_mj_pos, initial_mj_quat)

        print(f"[System] MuJoCo 模型加载成功, nmocap={env.model.nmocap}")
        print("[System] 初始目标保持当前配置，启动后可直接拖动绿色方块修改目标。")
        mujoco_ready = True
    except Exception as exc:
        print(f"[Warning] MuJoCo 初始化失败 (仅使用 Rerun 可视化): {exc}")

    can_thread = threading.Thread(
        target=can_thread_func,
        args=(transport, comp_tool, shared_state, rerun_logger),
        daemon=True,
    )
    can_thread.start()

    try:
        if mujoco_ready and env is not None and transformer is not None:
            run_viewer_loop(env, shared_state, transformer, shutdown_event)
        else:
            print("\n[Running] SocketCAN MIT torque 控制回路运行中。按下 Ctrl+C 停止。")
            while not shutdown_event.is_set():
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_event.set()
        can_thread.join(timeout=2.0)
        comp_tool.close()
        if rerun_logger is not None:
            rerun_logger.close()
        print("[System] 已安全退出。")


if __name__ == "__main__":
    main()
