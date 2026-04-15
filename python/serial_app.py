"""Real-hardware serial target forwarding application."""

from __future__ import annotations

import threading
import time

import serial

from config import Config
from coord_transforms import RobotMujocoTransformer
from gravity_backend import GravityCompTool
from rerun_async import RerunLogger
from serial_protocol import (
    RECV_FRAME_SIZE,
    SEND_FRAME_SIZE,
    TargetPoseFramePacker,
    SerialFrameReader,
)
from shared_state import SharedRobotState
from sim_env import MujocoSimEnv
import rerun_viz


SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
shutdown_event = threading.Event()


def serial_thread_func(ser, comp_tool: GravityCompTool, shared_state: SharedRobotState, rerun_logger=None) -> None:
    print("[Serial] 开始接收数据进程...")

    frame_reader = SerialFrameReader()
    frame_packer = TargetPoseFramePacker()
    step_count = 0
    last_cycle_end = None

    set_reported_pose = shared_state.set_reported_pose
    snapshot_control_inputs = shared_state.snapshot_control_inputs
    update_joint_feedback = shared_state.update_joint_feedback
    write_serial = ser.write
    tx_cycle_bytes = SEND_FRAME_SIZE if Config.SERIAL_FORWARD_TARGET else 0
    uart_cycle_bytes = RECV_FRAME_SIZE * Config.NUM_JOINTS + tx_cycle_bytes

    try:
        while not shutdown_event.is_set():
            bytes_waiting = frame_reader.read_available(ser)

            while True:
                parsed = frame_reader.pop_frame()
                if parsed is None:
                    break

                _, motor_id, _state, pos, vel, tor, _t_mos, _t_coil = parsed
                if not 1 <= motor_id <= Config.NUM_JOINTS:
                    continue

                joint_idx = motor_id - 1
                update_joint_feedback(joint_idx, pos, vel, tor)
                if motor_id != Config.NUM_JOINTS:
                    continue

                current_q, current_qd, tau_actual, target_pos, target_quat = snapshot_control_inputs()

                python_t0 = time.perf_counter()
                tau_total, ee_pos, ee_quat, stm_status, stm32_calc_ms = comp_tool.compute(
                    current_q,
                    current_qd,
                    target_pos,
                    target_quat,
                )
                python_cycle_ms = (time.perf_counter() - python_t0) * 1000.0

                set_reported_pose(ee_pos, ee_quat)

                if Config.SERIAL_FORWARD_TARGET:
                    write_serial(frame_packer.pack(target_pos, target_quat))

                cycle_end = time.perf_counter()
                uart_latency_ms = 0.0
                uart_cycle_hz = 0.0
                uart_transfer_kbps = 0.0
                if last_cycle_end is not None:
                    cycle_dt = cycle_end - last_cycle_end
                    if cycle_dt > 0.0:
                        uart_latency_ms = cycle_dt * 1000.0
                        uart_cycle_hz = 1.0 / cycle_dt
                        uart_transfer_kbps = (uart_cycle_bytes * 8.0) / cycle_dt / 1000.0
                last_cycle_end = cycle_end

                stm32_calc_hz = 1000.0 / stm32_calc_ms if stm32_calc_ms > 1e-9 else 0.0
                should_log_rerun = (
                    rerun_logger is not None
                    and Config.ENABLE_RERUN
                    and (
                        Config.RERUN_LOG_STRIDE <= 1
                        or step_count % Config.RERUN_LOG_STRIDE == 0
                    )
                )
                if should_log_rerun:
                    rx_str = None
                    tx_str = None
                    if step_count % Config.UART_TEXT_LOG_INTERVAL == 0:
                        rx_str = ", ".join(f"{x:.3f}" for x in current_q)
                        tx_str = ", ".join(
                            f"{x:.3f}" for x in (
                                target_pos[0],
                                target_pos[1],
                                target_pos[2],
                                target_quat[0],
                                target_quat[1],
                                target_quat[2],
                                target_quat[3],
                            )
                        )
                    tx_label = "Target Pose" if Config.SERIAL_FORWARD_TARGET else "Target Pose (local only)"
                    rerun_logger.log_step(
                        t=step_count * Config.DT,
                        pos_actual=ee_pos,
                        pos_desired=target_pos,
                        quat_actual=ee_quat,
                        quat_desired=target_quat,
                        tau_total=tau_total,
                        cycle_time=python_cycle_ms,
                        q=current_q,
                        qd=current_qd,
                        tau_actual=tau_actual,
                        rx_str=rx_str,
                        tx_str=tx_str,
                        tx_label=tx_label,
                        step_count=step_count,
                        uart_latency_ms=uart_latency_ms,
                        uart_cycle_hz=uart_cycle_hz,
                        uart_transfer_kbps=uart_transfer_kbps,
                        stm32_calc_time_ms=stm32_calc_ms,
                        stm32_calc_hz=stm32_calc_hz,
                    )
                if stm_status < 0:
                    print(f"[Serial Warning] 本地 STM32 控制计算返回异常状态: {stm_status}")
                step_count += 1

            if shutdown_event.is_set():
                break

            if bytes_waiting == 0 and not frame_reader.has_complete_frame():
                time.sleep(Config.SERIAL_IDLE_SLEEP_S)

    except Exception as exc:
        print(f"[Serial Error] {exc}")
        shutdown_event.set()
    finally:
        try:
            ser.close()
        except Exception:
            pass


def main() -> None:
    print("=" * 60)
    print("      AM-D02 Serial Target Forwarding Program           ")
    print("=" * 60)

    shutdown_event.clear()
    shared_state = SharedRobotState()
    rerun_logger = None

    if Config.ENABLE_RERUN:
        rerun_viz.init_rerun("AM-D02 Serial Control")
        rerun_viz.setup_realtime_styles()
        rerun_logger = RerunLogger()
        rerun_logger.start()

    try:
        comp_tool = GravityCompTool()
        print("[System] C 语言计算后端已就绪。")
    except Exception as exc:
        print(f"[Error] 启动 C 计算后端失败: {exc}")
        if rerun_logger is not None:
            rerun_logger.close()
        return

    initial_target_pos, initial_target_quat = comp_tool.compute_fk(Config.TARGET_Q.tolist())
    shared_state.set_target_pose(initial_target_pos, initial_target_quat)

    print(f"[System] 正在尝试连接串口: {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0)
        ser.reset_input_buffer()
        print(f"[System] 串口 {SERIAL_PORT} 连接成功。")
    except Exception as exc:
        print(f"[Error] 无法打开串口: {exc}")
        comp_tool.close()
        if rerun_logger is not None:
            rerun_logger.close()
        return

    print("[System] 正在加载 MuJoCo 场景模型 (MujocoSimEnv)...")
    env = None
    mujoco_ready = False
    transformer = None
    try:
        import mujoco
        import mujoco.viewer

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

    serial_thread = threading.Thread(
        target=serial_thread_func,
        args=(ser, comp_tool, shared_state, rerun_logger),
        daemon=True,
    )
    serial_thread.start()

    try:
        if mujoco_ready and env is not None and transformer is not None:
            import mujoco
            import mujoco.viewer

            model = env.model
            data = env.data
            mocap_idx_reported = env.reported_mocap_id

            serial_mode_desc = "接收反馈 + 发送目标位姿" if Config.SERIAL_FORWARD_TARGET else "接收反馈 + 本地力矩计算(不下发)"
            print(f"\n[Running] 双回路运行中: 串口线程 ({serial_mode_desc}) | 主线程 (MuJoCo 渲染)")
            with mujoco.viewer.launch_passive(model, data) as viewer:
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
                        dragged_target_pos_mj, dragged_target_quat_mj
                    )
                    shared_state.set_target_pose(dragged_target_pos, dragged_target_quat)

                    time.sleep(1.0 / Config.REAL_VIEWER_FPS)
        else:
            if Config.SERIAL_FORWARD_TARGET:
                print("\n[Running] 串口目标位姿转发回路运行中。按下 Ctrl+C 停止。")
            else:
                print("\n[Running] 串口反馈监视 + 本地力矩计算回路运行中（不向下位机发送目标）。按下 Ctrl+C 停止。")
            while not shutdown_event.is_set():
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_event.set()
        serial_thread.join(timeout=1.0)
        comp_tool.close()
        if rerun_logger is not None:
            rerun_logger.close()
        print("[System] 已安全退出。")
