"""Real hardware SocketCAN runtime for split left/right arm deployment."""

from __future__ import annotations

import os
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

import numpy as np

from config import Config, _env_bool, _env_float, _env_int
from mujoco_viewer import VIEWER_AVAILABLE, launch_passive_viewer
from real.can_transport import Usb2FdcanConfig, Usb2FdcanTransport
from real.controller_bridge import RealControllerBridge
from sim.env import MujocoSimEnv
import rerun_viz


CAN_NOMINAL_BITRATE = _env_int("AM_D02_CAN_NOMINAL_BITRATE", 1_000_000)
CAN_DATA_BITRATE = _env_int("AM_D02_CAN_DATA_BITRATE", 5_000_000)
CAN_FORCE_FD = _env_bool("AM_D02_CAN_FORCE_FD", True)
CAN_CONFIGURE_INTERFACE = _env_bool("AM_D02_CAN_CONFIGURE_INTERFACE", False)
CAN_FEEDBACK_TIMEOUT_S = max(0.001, _env_float("AM_D02_CAN_FEEDBACK_TIMEOUT_S", 0.10))
CAN_READ_TIMEOUT_S = max(0.0, _env_float("AM_D02_CAN_READ_TIMEOUT_S", 0.002))
CAN_READ_CHUNK_SIZE = max(19, _env_int("AM_D02_CAN_READ_CHUNK_SIZE", 256))
REAL_FEEDBACK_REQUEST_INTERVAL_S = max(0.0, _env_float("AM_D02_REAL_FEEDBACK_REQUEST_INTERVAL_S", 0.001))
REAL_TORQUE_SLEW_NM_PER_S = max(0.0, _env_float("AM_D02_REAL_TORQUE_SLEW_NM_PER_S", 30.0))
REAL_DIAG_WINDOW_STEPS = max(1, _env_int("AM_D02_REAL_DIAG_WINDOW_STEPS", 40))
REAL_FEEDBACK_ONLY = _env_bool("AM_D02_REAL_FEEDBACK_ONLY", False)
LEFT_CAN_INTERFACE = os.getenv("AM_D02_LEFT_CAN_INTERFACE", "can0")
RIGHT_CAN_INTERFACE = os.getenv("AM_D02_RIGHT_CAN_INTERFACE", "can1")

MOTOR_IDS = tuple(range(1, Config.ARM_JOINTS + 1))
ACTIVE_ARM_MASK = {
    "left": 1 << Config.LEFT_ARM,
    "right": 1 << Config.RIGHT_ARM,
    "both": (1 << Config.LEFT_ARM) | (1 << Config.RIGHT_ARM),
}
ARM_INTERFACE = {
    Config.LEFT_ARM: LEFT_CAN_INTERFACE,
    Config.RIGHT_ARM: RIGHT_CAN_INTERFACE,
}

shutdown_event = threading.Event()


@dataclass(frozen=True)
class ArmCanRuntime:
    arm: int
    interface: str
    transport: object
    motor_ids: tuple[int, ...] = MOTOR_IDS


class RealSharedState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.q = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.qd = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.tau_actual = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.tau_total = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.ee_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        self.ee_quat = np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1))
        self.target_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        self.target_quat = np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1))
        self.active_arm_mask = 0
        self.feedback_valid = False
        self.targets_initialized = False

    def update_feedback(self, q: np.ndarray, qd: np.ndarray, tau_actual: np.ndarray, active_arm_mask: int) -> None:
        with self._lock:
            self.q[:] = np.asarray(q, dtype=np.float64).reshape(Config.NUM_JOINTS)
            self.qd[:] = np.asarray(qd, dtype=np.float64).reshape(Config.NUM_JOINTS)
            self.tau_actual[:] = np.asarray(tau_actual, dtype=np.float64).reshape(Config.NUM_JOINTS)
            self.active_arm_mask = int(active_arm_mask)
            self.feedback_valid = True

    def update_control_output(self, tau_total: np.ndarray, ee_pos: np.ndarray, ee_quat: np.ndarray) -> None:
        with self._lock:
            self.tau_total[:] = np.asarray(tau_total, dtype=np.float64).reshape(Config.NUM_JOINTS)
            self.ee_pos[:] = np.asarray(ee_pos, dtype=np.float64).reshape(Config.NUM_ARMS, 3)
            self.ee_quat[:] = np.asarray(ee_quat, dtype=np.float64).reshape(Config.NUM_ARMS, 4)

    def set_targets(self, target_pos: np.ndarray, target_quat: np.ndarray) -> None:
        with self._lock:
            self.target_pos[:] = np.asarray(target_pos, dtype=np.float64).reshape(Config.NUM_ARMS, 3)
            self.target_quat[:] = np.asarray(target_quat, dtype=np.float64).reshape(Config.NUM_ARMS, 4)
            self.targets_initialized = True

    def snapshot_control_inputs(self):
        with self._lock:
            return (
                self.q.copy(),
                self.qd.copy(),
                self.tau_actual.copy(),
                self.target_pos.copy(),
                self.target_quat.copy(),
            )

    def snapshot_targets(self):
        with self._lock:
            return self.target_pos.copy(), self.target_quat.copy()

    def targets_ready(self) -> bool:
        with self._lock:
            return bool(self.targets_initialized)

    def snapshot_mirror(self):
        with self._lock:
            return self.q.copy(), self.qd.copy(), bool(self.feedback_valid), bool(self.targets_initialized)


class RealRerunLogger:
    def __init__(self) -> None:
        self._queue = queue.Queue(maxsize=max(1, _env_int("AM_D02_RERUN_QUEUE_SIZE", 512)))
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._worker, name="real-rerun-logger", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _worker(self) -> None:
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                payload = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            rerun_viz.log_realtime_step(**payload)

    def log_step(self, **payload) -> None:
        if self._stop_event.is_set():
            return
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(payload)
            except queue.Full:
                pass

    def close(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)


def active_arms_for_mode(mode: str) -> tuple[int, ...]:
    if mode == "left":
        return (Config.LEFT_ARM,)
    if mode == "right":
        return (Config.RIGHT_ARM,)
    if mode == "both":
        return (Config.LEFT_ARM, Config.RIGHT_ARM)
    raise ValueError(f"unsupported real arm mode: {mode}")


def make_transport_config(interface: str) -> Usb2FdcanConfig:
    return Usb2FdcanConfig(
        interface=interface,
        nominal_bitrate=CAN_NOMINAL_BITRATE,
        data_bitrate=CAN_DATA_BITRATE,
        configure_interface=CAN_CONFIGURE_INTERFACE,
        force_fd=CAN_FORCE_FD,
        read_timeout=CAN_READ_TIMEOUT_S,
        motor_ids=MOTOR_IDS,
    )


def open_arm_runtimes(
    mode: str,
    *,
    transport_factory: Callable[[Usb2FdcanConfig], object] = Usb2FdcanTransport,
) -> tuple[ArmCanRuntime, ...]:
    runtimes = []
    for arm in active_arms_for_mode(mode):
        interface = ARM_INTERFACE[arm]
        runtimes.append(
            ArmCanRuntime(
                arm=arm,
                interface=interface,
                transport=transport_factory(make_transport_config(interface)),
            )
        )
    return tuple(runtimes)


def _safe_zero_and_disable(runtimes: tuple[ArmCanRuntime, ...]) -> None:
    for runtime in runtimes:
        for motor_id in runtime.motor_ids:
            try:
                runtime.transport.send_mit_torque(int(motor_id), 0.0)
            except Exception as exc:
                print(f"[CAN Warning] {runtime.interface} motor {motor_id} zero failed: {exc}")
        for motor_id in runtime.motor_ids:
            try:
                runtime.transport.disable_motor(int(motor_id))
            except Exception as exc:
                print(f"[CAN Warning] {runtime.interface} motor {motor_id} disable failed: {exc}")


def _startup_enable(runtimes: tuple[ArmCanRuntime, ...]) -> None:
    for runtime in runtimes:
        try:
            runtime.transport.reset_input_buffer()
        except Exception as exc:
            print(f"[CAN Warning] {runtime.interface} reset input failed: {exc}")
        for motor_id in runtime.motor_ids:
            runtime.transport.clear_error(int(motor_id))
            runtime.transport.enable_motor(int(motor_id))
            runtime.transport.send_mit_torque(int(motor_id), 0.0)


def _send_zero_keepalive(runtimes: tuple[ArmCanRuntime, ...]) -> None:
    for runtime in runtimes:
        for motor_id in runtime.motor_ids:
            runtime.transport.send_mit_torque(int(motor_id), 0.0)


def _send_torque_keepalive(runtimes: tuple[ArmCanRuntime, ...], tau: np.ndarray) -> None:
    tau_values = np.asarray(tau, dtype=np.float64).reshape(Config.NUM_JOINTS)
    for runtime in runtimes:
        offset = runtime.arm * Config.ARM_JOINTS
        for motor_id in runtime.motor_ids:
            runtime.transport.send_mit_torque(
                int(motor_id),
                float(tau_values[offset + int(motor_id) - 1]),
            )


def _collect_feedback(
    runtimes: tuple[ArmCanRuntime, ...],
    q: np.ndarray,
    qd: np.ndarray,
    tau_actual: np.ndarray,
    received_joint_masks: dict[int, int],
) -> int:
    feedback_mask = 0
    for runtime in runtimes:
        if hasattr(runtime.transport, "read_available"):
            runtime.transport.read_available(CAN_READ_CHUNK_SIZE)
        else:
            runtime.transport.read(CAN_READ_CHUNK_SIZE)
        arm_bit = 1 << runtime.arm
        arm_mask = int(received_joint_masks.get(runtime.arm, 0))
        while True:
            frame = runtime.transport.pop_feedback_frame()
            if frame is None:
                break
            motor_id = int(frame.motor_id)
            if motor_id not in runtime.motor_ids:
                continue
            joint_local = motor_id - 1
            joint_global = runtime.arm * Config.ARM_JOINTS + joint_local
            q[joint_global] = float(frame.position)
            qd[joint_global] = float(frame.velocity)
            tau_actual[joint_global] = float(frame.torque)
            arm_mask |= 1 << joint_local
        received_joint_masks[runtime.arm] = arm_mask
        if arm_mask == (1 << Config.ARM_JOINTS) - 1:
            feedback_mask |= arm_bit
    return feedback_mask


def _reset_feedback_masks(runtimes: tuple[ArmCanRuntime, ...], received_joint_masks: dict[int, int]) -> None:
    for runtime in runtimes:
        received_joint_masks[runtime.arm] = 0


def _format_missing_feedback(runtimes: tuple[ArmCanRuntime, ...], received_joint_masks: dict[int, int]) -> str:
    missing_parts = []
    for runtime in runtimes:
        arm_mask = int(received_joint_masks.get(runtime.arm, 0))
        missing_ids = [
            str(int(motor_id))
            for motor_id in runtime.motor_ids
            if (arm_mask & (1 << (int(motor_id) - 1))) == 0
        ]
        if missing_ids:
            missing_parts.append(f"{runtime.interface}: motor id {','.join(missing_ids)}")
    return "; ".join(missing_parts) if missing_parts else "none"


def _copy_active_arm_values(dst: np.ndarray, src: np.ndarray, active_arm_mask: int) -> np.ndarray:
    result = np.asarray(dst, dtype=np.float64).copy()
    source = np.asarray(src, dtype=np.float64)
    for arm in range(Config.NUM_ARMS):
        if (active_arm_mask & (1 << arm)) == 0:
            continue
        arm_slice = slice(arm * Config.ARM_JOINTS, (arm + 1) * Config.ARM_JOINTS)
        result[arm_slice] = source[arm_slice]
    return result


def _copy_active_arm_pose_values(dst: np.ndarray, src: np.ndarray, active_arm_mask: int) -> np.ndarray:
    result = np.asarray(dst, dtype=np.float64).copy()
    source = np.asarray(src, dtype=np.float64).reshape(result.shape)
    for arm in range(Config.NUM_ARMS):
        if (active_arm_mask & (1 << arm)) != 0:
            result[arm] = source[arm]
    return result


def mirror_real_state_to_env(
    env,
    shared_state: RealSharedState,
    active_arm_mask: int,
    initial_target_pos_base: np.ndarray | None = None,
) -> None:
    q, qd, feedback_valid, targets_initialized = shared_state.snapshot_mirror()
    if not feedback_valid:
        return
    env.set_qpos(_copy_active_arm_values(env.get_qpos(), q, active_arm_mask))
    env.set_qvel(_copy_active_arm_values(env.get_qvel(), qd, active_arm_mask))
    env.forward()
    if not targets_initialized:
        target_pos = env.get_all_ee_pos()
        target_quat = env.get_all_ee_quat()
        if initial_target_pos_base is not None:
            target_pos = _copy_active_arm_pose_values(target_pos, initial_target_pos_base, active_arm_mask)
        env.set_all_target_poses_base(target_pos, target_quat)
        for arm, arm_name in enumerate(Config.ARM_NAMES):
            if (active_arm_mask & (1 << arm)) == 0:
                continue
            current_tcp = env.get_all_ee_pos()[arm]
            target_tcp = target_pos[arm]
            distance = float(np.linalg.norm(target_tcp - current_tcp))
            print(
                f"[Real Target] {arm_name}: current_tcp={np.array2string(current_tcp, precision=4)} "
                f"target_tcp={np.array2string(target_tcp, precision=4)} "
                f"distance={distance:.4f}m target_pos=INIT_QPOS_TCP "
                "target_quat=current_real_tcp"
            )
    target_pos, target_quat = env.get_all_target_poses()
    shared_state.set_targets(target_pos, target_quat)


def _format_real_rx(q: np.ndarray, active_arm_mask: int) -> str:
    values = []
    for arm in range(Config.NUM_ARMS):
        if (active_arm_mask & (1 << arm)) == 0:
            continue
        arm_slice = slice(arm * Config.ARM_JOINTS, (arm + 1) * Config.ARM_JOINTS)
        values.extend(float(v) for v in q[arm_slice])
    return ", ".join(f"{value:.3f}" for value in values)


def _format_real_tx(tau: np.ndarray, active_arm_mask: int) -> str:
    values = []
    for arm in range(Config.NUM_ARMS):
        if (active_arm_mask & (1 << arm)) == 0:
            continue
        arm_slice = slice(arm * Config.ARM_JOINTS, (arm + 1) * Config.ARM_JOINTS)
        values.extend(float(v) for v in tau[arm_slice])
    return ", ".join(f"{value:.3f}" for value in values)


def _joint_safe_limits_rad() -> tuple[np.ndarray, np.ndarray]:
    limits = np.asarray(Config.JOINT_LIMITS_RAD, dtype=np.float64)
    span = limits[:, 1] - limits[:, 0]
    inset = float(Config.CONTROL_JOINT_LIMIT_INSET_RATIO) * span
    return limits[:, 0] + inset, limits[:, 1] - inset


def _format_real_safety_margins(q: np.ndarray, qd: np.ndarray, active_arm_mask: int) -> str:
    safe_min, safe_max = _joint_safe_limits_rad()
    q_values = np.asarray(q, dtype=np.float64).reshape(Config.NUM_JOINTS)
    qd_values = np.asarray(qd, dtype=np.float64).reshape(Config.NUM_JOINTS)
    parts = []
    for arm, arm_name in enumerate(Config.ARM_NAMES):
        if (active_arm_mask & (1 << arm)) == 0:
            continue
        offset = arm * Config.ARM_JOINTS
        for i in range(Config.ARM_JOINTS):
            index = offset + i
            low_margin = q_values[index] - safe_min[index]
            high_margin = safe_max[index] - q_values[index]
            vel_margin = float(Config.JOINT_VEL_LIMIT) - abs(qd_values[index])
            if low_margin < 0.0:
                parts.append(
                    f"{arm_name}/J{i + 1} q={q_values[index]:.4f} "
                    f"below_safe_min_by={-low_margin:.4f}rad "
                    f"(safe_min={safe_min[index]:.4f})"
                )
            if high_margin < 0.0:
                parts.append(
                    f"{arm_name}/J{i + 1} q={q_values[index]:.4f} "
                    f"above_safe_max_by={-high_margin:.4f}rad "
                    f"(safe_max={safe_max[index]:.4f})"
                )
            if vel_margin < 0.0:
                parts.append(
                    f"{arm_name}/J{i + 1} qd={qd_values[index]:.4f} "
                    f"over_vel_by={-vel_margin:.4f}rad/s "
                    f"(limit={Config.JOINT_VEL_LIMIT:.4f})"
                )
    return "; ".join(parts) if parts else "no Python-side q/qd limit violation detected"


def _right_j7_index() -> int:
    return Config.RIGHT_ARM * Config.ARM_JOINTS + (Config.ARM_JOINTS - 1)


def _make_right_j7_diag(
    q: np.ndarray,
    qd: np.ndarray,
    tau_actual: np.ndarray,
    tau_raw: np.ndarray,
    tau_sent: np.ndarray,
    elapsed_s: float,
    cycle_hz: float,
    step_count: int,
) -> dict:
    index = _right_j7_index()
    return {
        "joint": "right/J7",
        "step": int(step_count),
        "elapsed_s": float(elapsed_s),
        "cycle_hz": float(cycle_hz),
        "q": float(np.asarray(q, dtype=np.float64).reshape(Config.NUM_JOINTS)[index]),
        "qd": float(np.asarray(qd, dtype=np.float64).reshape(Config.NUM_JOINTS)[index]),
        "tau_actual": float(np.asarray(tau_actual, dtype=np.float64).reshape(Config.NUM_JOINTS)[index]),
        "tau_cmd_raw": float(np.asarray(tau_raw, dtype=np.float64).reshape(Config.NUM_JOINTS)[index]),
        "tau_cmd_sent": float(np.asarray(tau_sent, dtype=np.float64).reshape(Config.NUM_JOINTS)[index]),
    }


def _format_right_j7_diag_window(diag_window) -> str:
    if not diag_window:
        return "Recent right/J7 diagnostic window: empty"
    lines = ["Recent right/J7 diagnostic window:"]
    for item in list(diag_window):
        lines.append(
            "  "
            f"step={item['step']} "
            f"elapsed={item['elapsed_s']:.6f}s "
            f"hz={item['cycle_hz']:.1f} "
            f"q={item['q']:.4f} "
            f"qd={item['qd']:.4f} "
            f"tau_raw={item['tau_cmd_raw']:.4f} "
            f"tau_sent={item['tau_cmd_sent']:.4f} "
            f"tau_actual={item['tau_actual']:.4f}"
        )
    return "\n".join(lines)


def _apply_torque_slew_limit(
    desired_tau: np.ndarray,
    previous_tau: np.ndarray,
    active_arm_mask: int,
    elapsed_s: float,
) -> np.ndarray:
    desired = np.asarray(desired_tau, dtype=np.float64).reshape(Config.NUM_JOINTS)
    previous = np.asarray(previous_tau, dtype=np.float64).reshape(Config.NUM_JOINTS)
    limited = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    max_delta = float(REAL_TORQUE_SLEW_NM_PER_S) * max(0.0, float(elapsed_s))
    for arm in range(Config.NUM_ARMS):
        if (active_arm_mask & (1 << arm)) == 0:
            continue
        arm_slice = slice(arm * Config.ARM_JOINTS, (arm + 1) * Config.ARM_JOINTS)
        delta = np.clip(desired[arm_slice] - previous[arm_slice], -max_delta, max_delta)
        limited[arm_slice] = previous[arm_slice] + delta
    return limited


def run_real_can_control_loop(
    *,
    mode: str,
    bridge,
    runtimes: tuple[ArmCanRuntime, ...],
    shared_state: RealSharedState,
    rerun_logger=None,
    startup_enable: bool = True,
    require_targets: bool = False,
    feedback_only: bool = False,
) -> None:
    active_mask = ACTIVE_ARM_MASK[mode]
    complete_feedback_mask = active_mask
    last_cycle_end = None
    step_count = 0
    q = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    qd = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    tau_actual = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    last_sent_tau = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    body_q = np.zeros(Config.NUM_BODY_JOINTS, dtype=np.float64)
    received_joint_masks = {runtime.arm: 0 for runtime in runtimes}
    diag_window = deque(maxlen=REAL_DIAG_WINDOW_STEPS)
    zero_tau = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    zero_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
    identity_quat = np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1))

    if startup_enable:
        _startup_enable(runtimes)
    feedback_wait_start = time.perf_counter()
    last_control_time = feedback_wait_start
    command_in_effect = False
    last_feedback_request_time = 0.0

    try:
        while not shutdown_event.is_set():
            try:
                feedback_mask = _collect_feedback(runtimes, q, qd, tau_actual, received_joint_masks)
            except Exception as exc:
                print(f"[CAN Error] feedback read failed: {exc}")
                shutdown_event.set()
                break

            if feedback_mask != complete_feedback_mask:
                now = time.perf_counter()
                if (not command_in_effect) or (
                    now - last_feedback_request_time >= REAL_FEEDBACK_REQUEST_INTERVAL_S
                ):
                    _send_torque_keepalive(runtimes, last_sent_tau)
                    command_in_effect = True
                    last_feedback_request_time = now
                if now - feedback_wait_start > CAN_FEEDBACK_TIMEOUT_S:
                    missing = _format_missing_feedback(runtimes, received_joint_masks)
                    print(
                        f"[CAN Error] feedback timeout after {CAN_FEEDBACK_TIMEOUT_S:.3f}s; "
                        f"missing feedback: {missing}. Disabling active motors."
                    )
                    shutdown_event.set()
                    break
                continue
            feedback_wait_start = time.perf_counter()
            shared_state.update_feedback(q, qd, tau_actual, active_mask)
            if require_targets and not shared_state.targets_ready():
                _send_zero_keepalive(runtimes)
                command_in_effect = True
                last_feedback_request_time = time.perf_counter()
                _reset_feedback_masks(runtimes, received_joint_masks)
                time.sleep(0.001)
                continue

            current_q, current_qd, current_tau_actual, target_pos, target_quat = shared_state.snapshot_control_inputs()
            now = time.perf_counter()
            elapsed_s = max(0.0, now - last_control_time)
            last_control_time = now

            if feedback_only:
                _send_zero_keepalive(runtimes)
                command_in_effect = True
                last_feedback_request_time = time.perf_counter()
                shared_state.update_control_output(zero_tau, zero_pos, identity_quat)
                cycle_end = time.perf_counter()
                cycle_ms = 0.0
                cycle_hz = 0.0
                if last_cycle_end is not None:
                    cycle_dt = cycle_end - last_cycle_end
                    if cycle_dt > 0.0:
                        cycle_ms = cycle_dt * 1000.0
                        cycle_hz = 1.0 / cycle_dt
                last_cycle_end = cycle_end
                right_j7_diag = _make_right_j7_diag(
                    current_q,
                    current_qd,
                    current_tau_actual,
                    zero_tau,
                    zero_tau,
                    elapsed_s,
                    cycle_hz,
                    step_count,
                )
                diag_window.append(right_j7_diag)
                if rerun_logger is not None and Config.ENABLE_RERUN:
                    rerun_logger.log_step(
                        t=step_count * max(elapsed_s, 0.0),
                        pos_actual=zero_pos,
                        pos_desired=target_pos,
                        quat_actual=identity_quat,
                        quat_desired=target_quat,
                        tau_raw=zero_tau,
                        tau_total=zero_tau,
                        cycle_time=elapsed_s * 1000.0,
                        elapsed_s=elapsed_s,
                        q=current_q,
                        qd=current_qd,
                        tau_actual=current_tau_actual,
                        right_j7_diag=right_j7_diag,
                        rx_str=_format_real_rx(current_q, active_mask),
                        tx_str=_format_real_tx(zero_tau, active_mask),
                        tx_label="Zero keepalive (feedback only)",
                        step_count=step_count,
                        uart_latency_ms=cycle_ms,
                        uart_cycle_hz=cycle_hz,
                        uart_transfer_kbps=0.0,
                    )
                step_count += 1
                _reset_feedback_masks(runtimes, received_joint_masks)
                continue

            result = bridge.compute(
                active_mask,
                elapsed_s,
                current_q,
                current_qd,
                body_q,
                target_pos,
                target_quat,
            )
            tau_raw = np.asarray(result.tau, dtype=np.float64).reshape(Config.NUM_JOINTS).copy()
            if result.status < 0:
                details = _format_real_safety_margins(current_q, current_qd, active_mask)
                diag_window.append(
                    _make_right_j7_diag(
                        current_q,
                        current_qd,
                        current_tau_actual,
                        tau_raw,
                        last_sent_tau,
                        elapsed_s,
                        0.0,
                        step_count,
                    )
                )
                print(
                    f"[CAN Safety] controller status={result.status}, disabling active motors. "
                    f"q/qd details: {details}\n"
                    f"{_format_right_j7_diag_window(diag_window)}"
                )
                shutdown_event.set()
                break

            tau_to_send = _apply_torque_slew_limit(tau_raw, last_sent_tau, active_mask, elapsed_s)
            shared_state.update_control_output(tau_to_send, result.ee_pos, result.ee_quat)
            for runtime in runtimes:
                offset = runtime.arm * Config.ARM_JOINTS
                for motor_id in runtime.motor_ids:
                    runtime.transport.send_mit_torque(
                        int(motor_id),
                        float(tau_to_send[offset + motor_id - 1]),
                    )
            last_sent_tau[:] = tau_to_send
            command_in_effect = True
            last_feedback_request_time = time.perf_counter()

            cycle_end = time.perf_counter()
            cycle_ms = 0.0
            cycle_hz = 0.0
            if last_cycle_end is not None:
                cycle_dt = cycle_end - last_cycle_end
                if cycle_dt > 0.0:
                    cycle_ms = cycle_dt * 1000.0
                    cycle_hz = 1.0 / cycle_dt
            last_cycle_end = cycle_end
            right_j7_diag = _make_right_j7_diag(
                current_q,
                current_qd,
                current_tau_actual,
                tau_raw,
                tau_to_send,
                elapsed_s,
                cycle_hz,
                step_count,
            )
            diag_window.append(right_j7_diag)

            if rerun_logger is not None and Config.ENABLE_RERUN:
                rerun_logger.log_step(
                    t=result.traj_t,
                    pos_actual=result.ee_pos,
                    pos_desired=target_pos,
                    quat_actual=result.ee_quat,
                    quat_desired=target_quat,
                    tau_raw=tau_raw,
                    tau_total=tau_to_send,
                    cycle_time=elapsed_s * 1000.0,
                    elapsed_s=elapsed_s,
                    q=current_q,
                    qd=current_qd,
                    tau_actual=current_tau_actual,
                    right_j7_diag=right_j7_diag,
                    rx_str=_format_real_rx(current_q, active_mask),
                    tx_str=_format_real_tx(tau_to_send, active_mask),
                    tx_label="MIT torque via SocketCAN",
                    step_count=step_count,
                    uart_latency_ms=cycle_ms,
                    uart_cycle_hz=cycle_hz,
                    uart_transfer_kbps=0.0,
                )
            step_count += 1
            _reset_feedback_masks(runtimes, received_joint_masks)
    finally:
        _safe_zero_and_disable(runtimes)
        for runtime in runtimes:
            try:
                runtime.transport.close()
            except Exception:
                pass


def run_real_control_loop(
    *,
    mode: str,
    bridge,
    runtimes: tuple[ArmCanRuntime, ...],
    env: MujocoSimEnv | None = None,
    viewer=None,
    startup_enable: bool = True,
) -> None:
    shared_state = RealSharedState()
    run_real_can_control_loop(
        mode=mode,
        bridge=bridge,
        runtimes=runtimes,
        shared_state=shared_state,
        startup_enable=startup_enable,
    )


def run_real_control_with_bridge(
    arm: str,
    bridge=None,
    *,
    bridge_factory=None,
    control_title: str = "AM-DPBSURDF0422 Real SocketCAN Control",
    rerun_app_name: str = "AM-DPBSURDF0422 Real CAN",
) -> None:
    if bridge is None and bridge_factory is None:
        raise ValueError("run_real_control_with_bridge requires bridge or bridge_factory")
    print("=" * 60)
    print(f"[Real Config] torque slew limit: {REAL_TORQUE_SLEW_NM_PER_S:.3f} N*m/s")
    print(f"[Real Config] feedback-only diagnostics: {REAL_FEEDBACK_ONLY}")
    print("[Real Config] CAN loop trigger: complete active-arm feedback (nonblocking read)")
    print(f"      {control_title} ({arm})")
    print("=" * 60)
    shutdown_event.clear()
    runtimes = ()
    shared_state = RealSharedState()
    rerun_logger = None
    env = MujocoSimEnv()
    env.reset(Config.HOME_QPOS)
    env.forward()
    init_env = MujocoSimEnv()
    init_env.reset(Config.INIT_QPOS)
    init_env.forward()
    initial_target_pos_base = init_env.get_all_ee_pos()
    viewer = None
    try:
        if Config.ENABLE_RERUN:
            rerun_viz.init_rerun(rerun_app_name)
            rerun_viz.setup_realtime_styles()
            rerun_logger = RealRerunLogger()
            rerun_logger.start()
        if VIEWER_AVAILABLE and Config.ENABLE_VIEWER:
            viewer = launch_passive_viewer(env.model, env.data)
            viewer.sync()
        if bridge is None:
            bridge = bridge_factory()
        runtimes = open_arm_runtimes(arm)
        can_thread = threading.Thread(
            target=run_real_can_control_loop,
            kwargs={
                "mode": arm,
                "bridge": bridge,
                "runtimes": runtimes,
                "shared_state": shared_state,
                "rerun_logger": rerun_logger,
                "require_targets": True,
                "feedback_only": REAL_FEEDBACK_ONLY,
            },
            name="real-can-control",
            daemon=True,
        )
        can_thread.start()
        try:
            while not shutdown_event.is_set():
                if viewer is not None and not viewer.is_running():
                    shutdown_event.set()
                    break
                mirror_real_state_to_env(
                    env,
                    shared_state,
                    ACTIVE_ARM_MASK[arm],
                    initial_target_pos_base,
                )
                if viewer is not None:
                    viewer.sync()
                time.sleep(1.0 / max(1.0, float(_env_int("AM_D02_REAL_VIEWER_FPS", 30))))
        except KeyboardInterrupt:
            shutdown_event.set()
        can_thread.join(timeout=2.0)
    finally:
        if viewer is not None:
            viewer.close()
        if rerun_logger is not None:
            rerun_logger.close()
        if bridge is not None:
            bridge.close()


def run_real_control(arm: str) -> None:
    bridge = RealControllerBridge()
    run_real_control_with_bridge(arm, bridge)
