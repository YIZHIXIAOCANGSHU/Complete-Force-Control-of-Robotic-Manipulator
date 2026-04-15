"""UDP server that exposes the MuJoCo simulation to the C controller."""

from __future__ import annotations

import socket
import time

import numpy as np

from config import Config
import rerun_viz
from sim_env import MujocoSimEnv
from state_packets import STATE_PACKET_SIZE

try:
    import mujoco.viewer

    VIEWER_AVAILABLE = True
except ImportError:
    VIEWER_AVAILABLE = False


def _write_ready_file(ready_file: str | None) -> None:
    if ready_file is None:
        return
    with open(ready_file, "w", encoding="utf-8") as file_obj:
        file_obj.write("ready\n")


def run_udp_server(ready_file: str | None = None) -> None:
    print("=" * 60)
    print("      AM-D02 Python UDP Simulation Server       ")
    print("   允许独立的外部 C 语言控制器通过 Socket 接入  ")
    print("=" * 60)

    if Config.ENABLE_RERUN:
        rerun_viz.init_rerun()
        rerun_viz.setup_realtime_styles()
        time.sleep(0.5)

    env = MujocoSimEnv()

    env.reset(Config.INIT_QPOS)
    env.forward()
    box_init_pos = env.get_ee_pos().copy()
    box_init_quat = env.get_ee_quat().copy()
    print(f"[Server] INIT_QPOS FK => box init pos: {box_init_pos}")

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
        viewer = mujoco.viewer.launch_passive(env.model, env.data)
        viewer.sync()
        print("[UDP Server] 可视化窗口已打开。此时等待 C 端客户端发送请求...")

    _write_ready_file(ready_file)

    step_count = 0
    state_packet = np.empty(STATE_PACKET_SIZE, dtype=np.float64)
    state_packet_view = memoryview(state_packet).cast("B")

    try:
        while True:
            if viewer and not viewer.is_running():
                print("[UDP Server] Viewer is closed. Exiting.")
                break

            try:
                data, addr = sock.recvfrom(1024)

                if data == b"INIT":
                    print(f"[UDP Server] Client {addr} connected! (INIT received)")
                    env.write_state_packet(state_packet)
                    sock.sendto(state_packet_view, addr)
                    continue

                if len(data) != 56:
                    print(f"[UDP Server] 收到未知长度的数据: {len(data)} bytes")
                    continue

                tau = np.frombuffer(data, dtype="<f8", count=Config.NUM_JOINTS)
                clipped_tau = env.clip_torque(tau)
                t_start = time.perf_counter()
                env.apply_torque(clipped_tau)
                env.step()
                clipped = env.enforce_joint_limits()
                cycle_time_ms = (time.perf_counter() - t_start) * 1000.0

                if clipped:
                    env.forward()
                if viewer:
                    viewer.sync()

                if Config.ENABLE_RERUN:
                    q, qd, pos_current, quat_current, pos_desired, quat_desired = env.get_state_snapshot()
                    rerun_viz.log_realtime_step(
                        t=step_count * Config.DT,
                        pos_actual=pos_current,
                        pos_desired=pos_desired,
                        quat_actual=quat_current,
                        quat_desired=quat_desired,
                        tau_total=clipped_tau,
                        cycle_time=cycle_time_ms,
                        q=q,
                        qd=qd,
                    )

                env.write_state_packet(state_packet)
                sock.sendto(state_packet_view, addr)
                step_count += 1

            except socket.timeout:
                if viewer:
                    viewer.sync()
                continue

    except KeyboardInterrupt:
        print("\n[UDP Server] Interrupted by user. Exiting...")
    finally:
        if viewer:
            viewer.close()
        sock.close()
