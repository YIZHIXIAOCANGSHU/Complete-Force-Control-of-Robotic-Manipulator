#!/usr/bin/env python3
"""Real-hardware SocketCAN USB2FDCAN control application."""

from __future__ import annotations

import os
import sys
import threading
import time
from dataclasses import dataclass

from config import Config, _env_bool, _env_float, _env_int
from coord_transforms import RobotMujocoTransformer
from gravity_backend import GravityCompTool
from mujoco_viewer import VIEWER_AVAILABLE, launch_passive_viewer
from rerun_async import RerunLogger
from shared_state import SharedRobotState
import rerun_viz


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USBFDCAN_ROOT = os.path.join(PROJECT_ROOT, "usbfdcan")
if USBFDCAN_ROOT not in sys.path:
    sys.path.insert(0, USBFDCAN_ROOT)

from send.damiao import DamiaoSocketCanTransport  # noqa: E402


CAN_INTERFACE = os.getenv("AM_D02_CAN_INTERFACE", "can0")
CAN_NOMINAL_BITRATE = _env_int("AM_D02_CAN_NOMINAL_BITRATE", 1_000_000)
CAN_DATA_BITRATE = _env_int("AM_D02_CAN_DATA_BITRATE", 5_000_000)
CAN_FORCE_FD = _env_bool("AM_D02_CAN_FORCE_FD", True)
CAN_CONFIGURE_INTERFACE = _env_bool("AM_D02_CAN_CONFIGURE_INTERFACE", False)
CAN_FEEDBACK_TIMEOUT_S = max(0.001, _env_float("AM_D02_CAN_FEEDBACK_TIMEOUT_S", 0.10))
CAN_STARTUP_ENABLE = _env_bool("AM_D02_CAN_STARTUP_ENABLE", True)
CAN_READ_TIMEOUT_S = max(0.0, _env_float("AM_D02_CAN_READ_TIMEOUT_S", 0.002))
CAN_READ_CHUNK_SIZE = max(19, _env_int("AM_D02_CAN_READ_CHUNK_SIZE", 256))

shutdown_event = threading.Event()


@dataclass(frozen=True)
class CanTransportConfig:
    read_timeout: float = CAN_READ_TIMEOUT_S
    read_chunk_size: int = CAN_READ_CHUNK_SIZE
    flush_input_before_round: bool = True
    sync_timeout: float = 2.0
    interface: str = CAN_INTERFACE
    nominal_bitrate: int = CAN_NOMINAL_BITRATE
    data_bitrate: int = CAN_DATA_BITRATE
    configure_interface: bool = CAN_CONFIGURE_INTERFACE
    force_fd: bool = CAN_FORCE_FD
    motor_can_ids: tuple[int, ...] = (0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07)
    motor_mst_ids: tuple[int, ...] = (0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17)
    motor_types: tuple[str, ...] = ("DM8009", "DM8009", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310")


@dataclass(frozen=True)
class CanRuntimeConfig:
    transport: CanTransportConfig
    motor_ids: tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7)


def build_can_runtime_config() -> CanRuntimeConfig:
    return CanRuntimeConfig(transport=CanTransportConfig())


def open_can_transport():
    return DamiaoSocketCanTransport(build_can_runtime_config())


def _safe_zero_and_disable(transport, motor_ids) -> None:
    for motor_id in motor_ids:
        try:
            transport.send_mit_torque(int(motor_id), 0.0)
        except Exception as exc:
            print(f"[CAN Warning] motor {motor_id} 零力矩下发失败: {exc}")
    for motor_id in motor_ids:
        try:
            transport.disable_motor(int(motor_id))
        except Exception as exc:
            print(f"[CAN Warning] motor {motor_id} disable 失败: {exc}")


def _startup_enable(transport, motor_ids) -> None:
    try:
        transport.reset_input_buffer()
    except Exception as exc:
        print(f"[CAN Warning] 清空 CAN 输入缓冲失败: {exc}")
    for motor_id in motor_ids:
        transport.clear_error(int(motor_id))
        transport.enable_motor(int(motor_id))
        transport.send_mit_torque(int(motor_id), 0.0)


def _feedback_state(frame) -> int:
    return int(getattr(frame, "state", getattr(frame, "state_code", 0)))


def _feedback_rotor_temperature(frame) -> float:
    return float(getattr(frame, "rotor_temperature", getattr(frame, "mos_temperature", 0.0)))


def can_thread_func(
    transport,
    comp_tool: GravityCompTool,
    shared_state: SharedRobotState,
    rerun_logger=None,
    *,
    startup_enable: bool = CAN_STARTUP_ENABLE,
    feedback_timeout_s: float = CAN_FEEDBACK_TIMEOUT_S,
    control_period_s: float = Config.DT,
) -> None:
    print("[CAN] SocketCAN USB2FDCAN 控制线程启动...")

    motor_ids = tuple(range(1, Config.NUM_JOINTS + 1))
    complete_feedback_mask = (1 << Config.NUM_JOINTS) - 1
    feedback_mask = 0
    step_count = 0
    last_cycle_end = None
    feedback_round_start = time.perf_counter()
    last_stm_status = 0

    update_joint_feedback = shared_state.update_joint_feedback
    snapshot_control_inputs = shared_state.snapshot_control_inputs
    set_reported_pose = shared_state.set_reported_pose

    try:
        if startup_enable:
            _startup_enable(transport, motor_ids)
            print("[CAN] 已完成 clear_error、enable 和 MIT 零力矩预置。")

        next_tick = time.perf_counter()
        while not shutdown_event.is_set():
            now = time.perf_counter()
            if control_period_s > 0.0 and now < next_tick:
                time.sleep(min(next_tick - now, 0.001))
                continue
            next_tick = max(next_tick + control_period_s, time.perf_counter())

            try:
                transport.read(CAN_READ_CHUNK_SIZE)
            except Exception as exc:
                print(f"[CAN Error] 读取 CAN 反馈失败: {exc}")
                shutdown_event.set()
                break

            while True:
                frame = transport.pop_feedback_frame()
                if frame is None:
                    break
                motor_id = int(frame.motor_id)
                if not 1 <= motor_id <= Config.NUM_JOINTS:
                    continue

                joint_idx = motor_id - 1
                update_joint_feedback(joint_idx, frame.position, frame.velocity, frame.torque)
                feedback_mask |= 1 << joint_idx

            if feedback_mask != complete_feedback_mask:
                if time.perf_counter() - feedback_round_start > feedback_timeout_s:
                    print(f"[CAN Error] {feedback_timeout_s:.3f}s 内未凑齐 7 轴反馈，进入安全停机。")
                    shutdown_event.set()
                    break
                continue
            feedback_mask = 0
            feedback_round_start = time.perf_counter()

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

            if stm_status < 0:
                print(f"[CAN Safety] 本地 STM32 控制计算返回异常状态: {stm_status}，停止下发非零力矩。")
                shutdown_event.set()
                break

            for index, motor_id in enumerate(motor_ids):
                transport.send_mit_torque(int(motor_id), float(tau_total[index]))

            cycle_end = time.perf_counter()
            can_latency_ms = 0.0
            can_cycle_hz = 0.0
            if last_cycle_end is not None:
                cycle_dt = cycle_end - last_cycle_end
                if cycle_dt > 0.0:
                    can_latency_ms = cycle_dt * 1000.0
                    can_cycle_hz = 1.0 / cycle_dt
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
                    tx_str = ", ".join(f"{x:.3f}" for x in tau_total)
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
                    tx_label="MIT torque via SocketCAN",
                    step_count=step_count,
                    uart_latency_ms=can_latency_ms,
                    uart_cycle_hz=can_cycle_hz,
                    uart_transfer_kbps=0.0,
                    stm32_calc_time_ms=stm32_calc_ms,
                    stm32_calc_hz=stm32_calc_hz,
                )
            if stm_status == 0 and last_stm_status < 0:
                print("[CAN] 本地 STM32 控制计算已恢复正常。")
            last_stm_status = stm_status
            step_count += 1

    except Exception as exc:
        print(f"[CAN Error] {exc}")
        shutdown_event.set()
    finally:
        _safe_zero_and_disable(transport, motor_ids)
        try:
            transport.close()
        except Exception:
            pass
        print("[CAN] 控制线程已退出，已执行零力矩和 disable。")


def _run_viewer_loop(env: MujocoSimEnv, shared_state: SharedRobotState, transformer: RobotMujocoTransformer) -> None:
    import mujoco

    model = env.model
    data = env.data
    mocap_idx_reported = env.reported_mocap_id

    print("\n[Running] 双回路运行中: CAN 线程 (MIT torque) | 主线程 (MuJoCo 渲染)")
    with launch_passive_viewer(model, data) as viewer:
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

            time.sleep(1.0 / Config.REAL_VIEWER_FPS)


def main() -> None:
    print("=" * 60)
    print("      AM-D02 SocketCAN USB2FDCAN Real Control Program    ")
    print("=" * 60)

    shutdown_event.clear()
    shared_state = SharedRobotState()
    rerun_logger = None
    transport = None

    if Config.ENABLE_RERUN:
        rerun_viz.init_rerun("AM-D02 SocketCAN Real Control")
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
        from sim_env import MujocoSimEnv

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
            _run_viewer_loop(env, shared_state, transformer)
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
