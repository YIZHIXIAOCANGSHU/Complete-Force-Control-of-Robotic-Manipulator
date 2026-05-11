from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass

import numpy as np


PYTHON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(PYTHON_ROOT)
if PYTHON_ROOT not in sys.path:
    sys.path.insert(0, PYTHON_ROOT)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from config import Config, _env_bool, _env_float, _env_int
from common.mujoco_viewer import VIEWER_AVAILABLE, launch_passive_viewer
from common.shared_state import SharedRobotState
from usb2fdcan_send.damiao import Usb2FdcanConfig, Usb2FdcanZeroTransport
from .rerun_feedback import MotorQuality, STATE_CODE_LABELS, UsbfdcanSimRerunRecorder


CAN_INTERFACE = os.getenv("AM_D02_CAN_INTERFACE", "can0")
CAN_NOMINAL_BITRATE = _env_int("AM_D02_CAN_NOMINAL_BITRATE", 1_000_000)
CAN_DATA_BITRATE = _env_int("AM_D02_CAN_DATA_BITRATE", 5_000_000)
CAN_FORCE_FD = _env_bool("AM_D02_CAN_FORCE_FD", True)
CAN_CONFIGURE_INTERFACE = _env_bool("AM_D02_CAN_CONFIGURE_INTERFACE", False)
CAN_READ_TIMEOUT_S = max(0.0, _env_float("AM_D02_CAN_READ_TIMEOUT_S", 0.002))
CAN_READ_CHUNK_SIZE = max(1, _env_int("AM_D02_CAN_READ_CHUNK_SIZE", 256))
CAN_FEEDBACK_TIMEOUT_S = max(0.001, _env_float("AM_D02_CAN_FEEDBACK_TIMEOUT_S", 0.10))
CAN_STARTUP_ENABLE = _env_bool("AM_D02_CAN_STARTUP_ENABLE", True)
USBFDCAN_SIM_VELOCITY_LIMIT = max(0.001, _env_float("AM_D02_USBFDCAN_SIM_VELOCITY_LIMIT", 10.0))
JOINT_POSITION_TOLERANCE = 0.01

shutdown_event = threading.Event()


JOINT_POSITION_MIN = np.deg2rad(
    np.array([-89.971835, -89.954374, -68.754935, -119.748454, -45.836624, -61.306275, -61.306275])
)
JOINT_POSITION_MAX = np.deg2rad(
    np.array([89.971835, 20.587610, 45.836624, 119.954374, 45.836624, 45.263666, 61.306275])
)


@dataclass(frozen=True)
class FeedbackSafetyResult:
    ok: bool
    reason: str = ""
    position_ok: bool = True
    velocity_ok: bool = True
    state_ok: bool = True

    @property
    def safety_ok(self) -> bool:
        return self.ok


def build_zero_transport_config() -> Usb2FdcanConfig:
    return Usb2FdcanConfig(
        interface=CAN_INTERFACE,
        nominal_bitrate=CAN_NOMINAL_BITRATE,
        data_bitrate=CAN_DATA_BITRATE,
        configure_interface=CAN_CONFIGURE_INTERFACE,
        force_fd=CAN_FORCE_FD,
        read_timeout=CAN_READ_TIMEOUT_S,
    )


def open_zero_transport() -> Usb2FdcanZeroTransport:
    return Usb2FdcanZeroTransport(build_zero_transport_config())


def _startup_enable_zero(transport, motor_ids: tuple[int, ...]) -> None:
    try:
        transport.reset_input_buffer()
    except Exception as exc:
        print(f"[USB2FDCAN Warning] 清空 CAN 输入缓冲失败: {exc}")
    for motor_id in motor_ids:
        transport.clear_error(int(motor_id))
        transport.enable_motor(int(motor_id))
        transport.send_zero_mit(int(motor_id))


def _safe_zero_and_disable(transport, motor_ids: tuple[int, ...]) -> None:
    for motor_id in motor_ids:
        try:
            transport.send_zero_mit(int(motor_id))
        except Exception as exc:
            print(f"[USB2FDCAN Warning] motor {motor_id} 全零下发失败: {exc}")
    for motor_id in motor_ids:
        try:
            transport.disable_motor(int(motor_id))
        except Exception as exc:
            print(f"[USB2FDCAN Warning] motor {motor_id} disable 失败: {exc}")


def check_feedback_frame_safety(frame) -> FeedbackSafetyResult:
    motor_id = int(frame.motor_id)
    if not 1 <= motor_id <= Config.NUM_JOINTS:
        return FeedbackSafetyResult(False, f"motor_id out of range: {motor_id}")
    values = (float(frame.position), float(frame.velocity), float(frame.torque))
    if not all(math.isfinite(value) for value in values):
        return FeedbackSafetyResult(False, f"non-finite feedback motor={motor_id}", position_ok=False, velocity_ok=False)
    state_ok = int(frame.state) in STATE_CODE_LABELS
    position_ok = bool(
        JOINT_POSITION_MIN[motor_id - 1] - JOINT_POSITION_TOLERANCE
        <= float(frame.position)
        <= JOINT_POSITION_MAX[motor_id - 1] + JOINT_POSITION_TOLERANCE
    )
    velocity_ok = abs(float(frame.velocity)) <= USBFDCAN_SIM_VELOCITY_LIMIT
    if not state_ok:
        reason = f"state error motor={motor_id} state=0x{int(frame.state):X}"
    elif not position_ok:
        reason = f"position limit violated motor={motor_id} position={float(frame.position):.6f}"
    elif not velocity_ok:
        reason = f"velocity limit violated motor={motor_id} velocity={float(frame.velocity):.6f}"
    else:
        reason = ""
    return FeedbackSafetyResult(
        ok=state_ok and position_ok and velocity_ok,
        reason=reason,
        position_ok=position_ok,
        velocity_ok=velocity_ok,
        state_ok=state_ok,
    )


def _quality_from_safety(safety: FeedbackSafetyResult) -> MotorQuality:
    return MotorQuality(
        state_ok=safety.state_ok,
        position_ok=safety.position_ok,
        velocity_ok=safety.velocity_ok,
        feedback_recent=True,
        safety_ok=safety.safety_ok,
    )


def tx_zero_loop(
    transport,
    *,
    motor_ids: tuple[int, ...],
    max_rounds: int | None = None,
) -> None:
    round_count = 0
    while not shutdown_event.is_set():
        for motor_id in motor_ids:
            if shutdown_event.is_set():
                break
            transport.send_zero_mit(int(motor_id))
        round_count += 1
        if max_rounds is not None and round_count >= int(max_rounds):
            break


def rx_feedback_loop(
    transport,
    shared_state: SharedRobotState,
    rerun_recorder=None,
    *,
    startup_enable: bool = CAN_STARTUP_ENABLE,
    feedback_timeout_s: float = CAN_FEEDBACK_TIMEOUT_S,
    max_complete_rounds: int | None = None,
) -> None:
    motor_ids = tuple(range(1, Config.NUM_JOINTS + 1))
    complete_feedback_mask = (1 << Config.NUM_JOINTS) - 1
    feedback_mask = 0
    complete_rounds = 0
    feedback_round_start = time.perf_counter()
    last_feedback_count = 0
    last_send_count = 0
    last_round_time = None
    perf_window_start = time.perf_counter()
    missing_feedback_mask = complete_feedback_mask

    if startup_enable:
        _startup_enable_zero(transport, motor_ids)
        print("[USB2FDCAN] 已完成 clear_error、enable 和全零 MIT 预置。")

    while not shutdown_event.is_set():
        try:
            transport.read(CAN_READ_CHUNK_SIZE)
        except Exception as exc:
            if rerun_recorder is not None:
                rerun_recorder.log_abort(reason=f"read failed: {exc}", missing_feedback_mask=missing_feedback_mask)
            print(f"[USB2FDCAN Error] 读取反馈失败: {exc}")
            shutdown_event.set()
            break

        saw_frame = False
        while True:
            frame = transport.pop_feedback_frame()
            if frame is None:
                break
            saw_frame = True
            motor_id = int(frame.motor_id)
            if not 1 <= motor_id <= Config.NUM_JOINTS:
                continue
            safety = check_feedback_frame_safety(frame)
            joint_idx = motor_id - 1
            shared_state.update_joint_feedback(joint_idx, frame.position, frame.velocity, frame.torque)
            feedback_mask |= 1 << joint_idx
            missing_feedback_mask = complete_feedback_mask ^ feedback_mask
            if rerun_recorder is not None:
                rerun_recorder.log_feedback_frame(frame=frame, quality=_quality_from_safety(safety))
            if not safety.ok:
                if rerun_recorder is not None:
                    rerun_recorder.log_abort(reason=safety.reason, missing_feedback_mask=missing_feedback_mask)
                print(f"[USB2FDCAN Safety] {safety.reason}")
                shutdown_event.set()
                break

        if shutdown_event.is_set():
            break

        now = time.perf_counter()
        if feedback_mask == complete_feedback_mask:
            complete_rounds += 1
            feedback_mask = 0
            missing_feedback_mask = 0
            feedback_round_start = now
            complete_round_rate_hz = 0.0
            if last_round_time is not None and now > last_round_time:
                complete_round_rate_hz = 1.0 / (now - last_round_time)
            last_round_time = now

            stats = getattr(transport, "stats", None)
            send_count = int(getattr(stats, "send_count", 0))
            feedback_count = int(getattr(stats, "feedback_count", 0))
            window_dt = max(now - perf_window_start, 1e-9)
            tx_send_rate_hz = (send_count - last_send_count) / window_dt
            rx_feedback_rate_hz = (feedback_count - last_feedback_count) / window_dt
            last_send_count = send_count
            last_feedback_count = feedback_count
            perf_window_start = now
            if rerun_recorder is not None:
                rerun_recorder.log_performance(
                    tx_send_rate_hz=tx_send_rate_hz,
                    rx_feedback_rate_hz=rx_feedback_rate_hz,
                    complete_round_rate_hz=complete_round_rate_hz,
                    missing_feedback_mask=missing_feedback_mask,
                    backpressure_count=int(getattr(stats, "backpressure_count", 0)),
                    raw_zero_packet=bytes(getattr(stats, "last_zero_packet", b"")),
                )
            if max_complete_rounds is not None and complete_rounds >= int(max_complete_rounds):
                break
        elif not saw_frame and now - feedback_round_start > feedback_timeout_s:
            missing_feedback_mask = complete_feedback_mask ^ feedback_mask
            reason = f"{feedback_timeout_s:.3f}s 内未凑齐 7 轴反馈"
            if rerun_recorder is not None:
                rerun_recorder.log_abort(reason=reason, missing_feedback_mask=missing_feedback_mask)
            print(f"[USB2FDCAN Error] {reason}，进入安全停机。")
            shutdown_event.set()
            break


def run_viewer_loop(shared_state: SharedRobotState) -> None:
    if not VIEWER_AVAILABLE:
        print("[USB2FDCAN Warning] MuJoCo viewer 不可用，仅运行 CAN 反馈镜像线程。")
        while not shutdown_event.is_set():
            time.sleep(0.1)
        return

    import mujoco
    from sim.env import MujocoSimEnv

    env = MujocoSimEnv()
    env.reset(Config.HOME_QPOS)
    env.forward()
    model = env.model
    data = env.data
    print("\n[Running] USB2FDCAN 反馈驱动 MuJoCo 仿真，全零 MIT 命令高速发送中。按 Ctrl+C 停止。")
    with launch_passive_viewer(model, data) as viewer:
        while viewer.is_running() and not shutdown_event.is_set():
            q, _, _ = shared_state.snapshot_viewer_state()
            env.set_qpos(np.asarray(q, dtype=np.float64))
            env.forward()
            viewer.sync()
            time.sleep(1.0 / Config.REAL_VIEWER_FPS)


def run_mirror_session(
    *,
    start_viewer: bool = True,
    feedback_timeout_s: float = CAN_FEEDBACK_TIMEOUT_S,
    startup_enable: bool = CAN_STARTUP_ENABLE,
) -> None:
    shutdown_event.clear()
    motor_ids = tuple(range(1, Config.NUM_JOINTS + 1))
    shared_state = SharedRobotState()
    rerun_recorder = None
    transport = open_zero_transport()
    if Config.ENABLE_RERUN:
        rerun_recorder = UsbfdcanSimRerunRecorder(motor_ids=motor_ids)

    rx_thread = threading.Thread(
        target=rx_feedback_loop,
        args=(transport, shared_state, rerun_recorder),
        kwargs={"startup_enable": startup_enable, "feedback_timeout_s": feedback_timeout_s},
        daemon=True,
    )
    tx_thread = threading.Thread(
        target=tx_zero_loop,
        args=(transport,),
        kwargs={"motor_ids": motor_ids},
        daemon=True,
    )

    try:
        rx_thread.start()
        tx_thread.start()
        if start_viewer:
            run_viewer_loop(shared_state)
        else:
            while rx_thread.is_alive() and not shutdown_event.is_set():
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_event.set()
        tx_thread.join(timeout=1.0)
        rx_thread.join(timeout=1.0)
        _safe_zero_and_disable(transport, motor_ids)
        try:
            transport.close()
        finally:
            if rerun_recorder is not None:
                rerun_recorder.close()
        print("[USB2FDCAN] 已安全退出，完成全零下发、disable 和 CAN 关闭。")


def main() -> None:
    print("=" * 60)
    print("      AM-D02 USB2FDCAN Feedback Mirror Simulation       ")
    print("=" * 60)
    print(
        "[System] SocketCAN: "
        f"{CAN_INTERFACE}, nominal={CAN_NOMINAL_BITRATE}, data={CAN_DATA_BITRATE}, force_fd={int(CAN_FORCE_FD)}"
    )
    run_mirror_session()


if __name__ == "__main__":
    main()
