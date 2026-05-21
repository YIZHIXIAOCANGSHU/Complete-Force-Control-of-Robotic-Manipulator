"""SocketCAN control loop and safety shutdown helpers."""

from __future__ import annotations

import threading
import time

from robot_control.config import Config
from robot_control.dynamics.gravity import GravityCompTool
from robot_control.modes.control_real import can_feedback
from robot_control.modes.control_real.runtime_config import (
    CAN_FEEDBACK_TIMEOUT_S,
    CAN_READ_CHUNK_SIZE,
    CAN_STARTUP_ENABLE,
)
from robot_control.shared.state import SharedRobotState

shutdown_event = threading.Event()


def _safe_zero_and_disable(transport, motor_ids) -> None:
    can_feedback.safe_zero_and_disable(transport, motor_ids)


def _send_zero_keepalive(transport, motor_ids) -> None:
    can_feedback.send_zero_keepalive(transport, motor_ids)


def _startup_enable(transport, motor_ids) -> None:
    can_feedback.startup_enable(transport, motor_ids)


def _send_mit_round(transport, motor_ids, control_output) -> None:
    for index, motor_id in enumerate(motor_ids):
        transport.send_mit_command(
            int(motor_id),
            position=float(control_output.q_ref[index]),
            velocity=float(control_output.qd_ref[index]),
            kp=float(control_output.kp[index]),
            kd=float(control_output.kd[index]),
            torque=float(control_output.tau_ff[index]),
        )


def _feedback_state(frame) -> int:
    return int(getattr(frame, "state", getattr(frame, "state_code", 0)))


def _feedback_rotor_temperature(frame) -> float:
    return float(getattr(frame, "rotor_temperature", getattr(frame, "mos_temperature", 0.0)))


def _missing_feedback_ids(feedback_mask: int) -> tuple[int, ...]:
    return can_feedback.missing_feedback_ids(feedback_mask)


def can_thread_func(
    transport,
    comp_tool: GravityCompTool,
    shared_state: SharedRobotState,
    rerun_logger=None,
    *,
    startup_enable: bool = CAN_STARTUP_ENABLE,
    feedback_timeout_s: float = CAN_FEEDBACK_TIMEOUT_S,
    control_period_s: float = 0.0,
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

        while not shutdown_event.is_set():
            _ = control_period_s

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
                _send_zero_keepalive(transport, motor_ids)
                if time.perf_counter() - feedback_round_start > feedback_timeout_s:
                    missing_ids = _missing_feedback_ids(feedback_mask)
                    print(
                        f"[CAN Error] {feedback_timeout_s:.3f}s 内未凑齐 7 轴反馈，"
                        f"缺失电机={missing_ids}，进入安全停机。"
                    )
                    shutdown_event.set()
                    break
                continue
            feedback_mask = 0
            feedback_round_start = time.perf_counter()

            current_q, current_qd, tau_actual, target_pos, target_quat = snapshot_control_inputs()

            python_t0 = time.perf_counter()
            control_output = comp_tool.compute(
                current_q,
                current_qd,
                target_pos,
                target_quat,
            )
            tau_total = control_output.tau_total
            ee_pos = control_output.ee_pos
            ee_quat = control_output.ee_quat
            stm_status = control_output.status
            calc_time_ms = control_output.calc_time_ms
            python_cycle_ms = (time.perf_counter() - python_t0) * 1000.0

            set_reported_pose(ee_pos, ee_quat)

            if stm_status < 0:
                print(f"[CAN Safety] 控制计算返回异常状态: {stm_status}，停止下发非零力矩。")
                shutdown_event.set()
                break

            _send_mit_round(transport, motor_ids, control_output)

            cycle_end = time.perf_counter()
            can_latency_ms = 0.0
            can_cycle_hz = 0.0
            if last_cycle_end is not None:
                cycle_dt = cycle_end - last_cycle_end
                if cycle_dt > 0.0:
                    can_latency_ms = cycle_dt * 1000.0
                    can_cycle_hz = 1.0 / cycle_dt
            last_cycle_end = cycle_end

            calc_hz = 1000.0 / calc_time_ms if calc_time_ms > 1e-9 else 0.0
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
                if step_count % 100 == 0:
                    rx_str = ", ".join(f"{x:.3f}" for x in current_q)
                    tx_str = ", ".join(
                        f"p={control_output.q_ref[i]:.3f}/v={control_output.qd_ref[i]:.3f}/"
                        f"kp={control_output.kp[i]:.1f}/kd={control_output.kd[i]:.1f}/"
                        f"t={control_output.tau_ff[i]:.3f}"
                        for i in range(Config.NUM_JOINTS)
                    )
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
                    q_target=control_output.q_ref,
                    tau_actual=tau_actual,
                    rx_str=rx_str,
                    tx_str=tx_str,
                    tx_label="MIT torque via SocketCAN",
                    step_count=step_count,
                    uart_latency_ms=can_latency_ms,
                    uart_cycle_hz=can_cycle_hz,
                    uart_transfer_kbps=0.0,
                    calc_time_ms=calc_time_ms,
                    calc_hz=calc_hz,
                )
            if stm_status == 0 and last_stm_status < 0:
                print("[CAN] 控制计算已恢复正常。")
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
