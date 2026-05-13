"""UDP server that exposes the MuJoCo simulation to the C controller."""

from __future__ import annotations

import socket
import time

import numpy as np

from config import Config
from common.mujoco_viewer import VIEWER_AVAILABLE, launch_passive_viewer
from common import rerun_viz
from sim.state_packets import STATE_PACKET_SIZE


def _write_ready_file(ready_file: str | None) -> None:
    if ready_file is None:
        return
    with open(ready_file, "w", encoding="utf-8") as file_obj:
        file_obj.write("ready\n")


def _create_sim_env():
    try:
        from sim.env import MujocoSimEnv

        return MujocoSimEnv()
    except ModuleNotFoundError as exc:
        if exc.name == "mujoco":
            raise RuntimeError(
                "缺少 Python 依赖 mujoco，请先执行 "
                "`python3 -m pip install -r python/requirements.txt`。"
            ) from exc
        raise


_VIEWER_SYNC_INTERVAL = 1.0 / 30.0


def _sleep_until_next_step(next_step_time: float, dt: float, realtime: bool) -> float:
    if not realtime:
        return next_step_time

    now = time.perf_counter()
    if next_step_time <= 0.0:
        return now + dt

    sleep_s = next_step_time - now
    if sleep_s > 0.0:
        time.sleep(sleep_s)
        return next_step_time + dt

    return now + dt


def run_udp_server(ready_file: str | None = None) -> None:
    print("=" * 60)
    print("      AM-D02 Python UDP 仿真服务                ")
    print("   允许外部 Pinocchio 控制器通过 Socket 接入    ")
    print("=" * 60)

    if Config.ENABLE_RERUN:
        rerun_viz.init_rerun()
        rerun_viz.setup_sim_realtime_styles()
        time.sleep(0.5)

    env = _create_sim_env()

    env.reset(Config.INIT_QPOS)
    env.forward()
    box_init_pos = env.get_ee_pos().copy()
    box_init_quat = env.get_ee_quat().copy()
    print(f"[Server] INIT_QPOS 正向运动学 => 初始目标位置: {box_init_pos}")

    env.reset(Config.HOME_QPOS)
    env.forward()
    env.set_target_pose(box_init_pos, box_init_quat)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_addr = ("0.0.0.0", 9876)
    sock.bind(server_addr)
    sock.settimeout(0.01)
    print(f"[UDP Server] 监听端口 {server_addr[1]}...")

    viewer = None
    if VIEWER_AVAILABLE and Config.ENABLE_VIEWER:
        viewer = launch_passive_viewer(env.model, env.data)
        viewer.sync()
        print("[UDP Server] 可视化窗口已打开。此时等待 C 端客户端发送请求...")

    _write_ready_file(ready_file)

    step_count = 0
    next_step_time = 0.0
    last_viewer_sync = 0.0
    state_packet = np.empty(STATE_PACKET_SIZE, dtype=np.float64)
    state_packet_view = memoryview(state_packet).cast("B")

    def _throttled_viewer_sync(viewer, last_sync: float) -> float:
        now = time.perf_counter()
        if now - last_sync >= _VIEWER_SYNC_INTERVAL:
            viewer.sync()
            return now
        return last_sync

    try:
        while True:
            if viewer and not viewer.is_running():
                print("[UDP Server] 可视化窗口已关闭，退出。")
                break

            try:
                data, addr = sock.recvfrom(1024)

                if data == b"INIT":
                    print(f"[UDP Server] 客户端 {addr} 已连接（收到 INIT）。")
                    env.write_state_packet(state_packet)
                    sock.sendto(state_packet_view, addr)
                    continue

                if len(data) == 35 * 8:
                    mit_command = np.frombuffer(data, dtype="<f8", count=35)
                    q_ref = mit_command[0:7]
                    qd_ref = mit_command[7:14]
                    kp = mit_command[14:21]
                    kd = mit_command[21:28]
                    tau_ff = mit_command[28:35]
                    q = env.get_qpos()
                    qd = env.get_qvel()
                    tau = kp * (q_ref - q) + kd * (qd_ref - qd) + tau_ff
                elif len(data) == 56:
                    tau = np.frombuffer(data, dtype="<f8", count=Config.NUM_JOINTS)
                else:
                    print(f"[UDP Server] 收到未知长度的数据: {len(data)} bytes")
                    continue

                clipped_tau = env.clip_torque(tau)
                next_step_time = _sleep_until_next_step(
                    next_step_time,
                    Config.DT,
                    Config.SIM_REALTIME,
                )
                t_start = time.perf_counter()
                env.apply_torque(clipped_tau)
                env.step()
                clipped = env.enforce_joint_limits()
                cycle_time_ms = (time.perf_counter() - t_start) * 1000.0

                if clipped:
                    env.forward()
                if viewer:
                    last_viewer_sync = _throttled_viewer_sync(viewer, last_viewer_sync)

                if Config.ENABLE_RERUN:
                    q, qd, pos_current, quat_current, pos_desired, quat_desired = env.get_state_snapshot()
                    rerun_viz.log_sim_realtime_step(
                        t=step_count * Config.DT,
                        pos_actual=pos_current,
                        pos_desired=pos_desired,
                        quat_actual=quat_current,
                        quat_desired=quat_desired,
                        tau_received=tau,
                        tau_applied=clipped_tau,
                        cycle_time=cycle_time_ms,
                        q=q,
                        qd=qd,
                        step_count=step_count,
                    )

                env.write_state_packet(state_packet)
                sock.sendto(state_packet_view, addr)
                step_count += 1

            except socket.timeout:
                if viewer:
                    last_viewer_sync = _throttled_viewer_sync(viewer, last_viewer_sync)
                continue

    except KeyboardInterrupt:
        print("\n[UDP Server] 用户中断，正在退出...")
    finally:
        if viewer:
            viewer.close()
        sock.close()
