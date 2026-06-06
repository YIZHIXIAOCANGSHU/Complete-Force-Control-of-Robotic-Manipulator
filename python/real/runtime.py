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
CAN_RX_BUFFER_BYTES = max(0, _env_int("AM_D02_CAN_RX_BUFFER_BYTES", 1_048_576))
CAN_FILTER_FEEDBACK = _env_bool("AM_D02_CAN_FILTER_FEEDBACK", True)
REAL_FEEDBACK_REQUEST_INTERVAL_S = max(0.0, _env_float("AM_D02_REAL_FEEDBACK_REQUEST_INTERVAL_S", 0.001))
REAL_IDLE_SLEEP_S = max(0.0, _env_float("AM_D02_REAL_IDLE_SLEEP_S", 0.0005))
REAL_DIAG_WINDOW_STEPS = max(1, _env_int("AM_D02_REAL_DIAG_WINDOW_STEPS", 40))
REAL_FEEDBACK_ONLY = _env_bool("AM_D02_REAL_FEEDBACK_ONLY", False)
REAL_FEEDBACK_BATCH_RX = _env_bool("AM_D02_REAL_FEEDBACK_BATCH_RX", True)
REAL_FEEDBACK_WAIT_FOR_CONSUME = _env_bool("AM_D02_REAL_FEEDBACK_WAIT_FOR_CONSUME", False)
REAL_THREAD_JOIN_TIMEOUT_S = max(0.1, _env_float("AM_D02_REAL_THREAD_JOIN_TIMEOUT_S", 2.0))
REAL_CONTROL_TARGET_HZ = max(1.0, _env_float("AM_D02_REAL_CONTROL_TARGET_HZ", 1000.0))
REAL_STATS_INTERVAL_S = max(0.0, _env_float("AM_D02_REAL_STATS_INTERVAL_S", 1.0))
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
REAL_C_BODY_Q_ZERO = np.zeros(Config.NUM_BODY_JOINTS, dtype=np.float64)
SEND_MODE_CONTROL = "control"
SEND_MODE_ZERO = "zero"
SEND_MODE_GRAVITY = "gravity"
SEND_MODE_GC = "gc"
SEND_MODES = frozenset({SEND_MODE_CONTROL, SEND_MODE_ZERO, SEND_MODE_GRAVITY, SEND_MODE_GC})

shutdown_event = threading.Event()
_THREAD_WAIT_CLOCK = time.monotonic


@dataclass(frozen=True)
class ArmCanRuntime:
    arm: int
    interface: str
    transport: object
    motor_ids: tuple[int, ...] = MOTOR_IDS


@dataclass(frozen=True)
class RealFeedbackSnapshot:
    seq: int
    timestamp_s: float
    q: np.ndarray
    qd: np.ndarray
    tau_actual: np.ndarray
    complete_mask: int
    received_joint_masks: dict[int, int]
    arm_seq: dict[int, int]


class RealSharedState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.q = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.qd = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.tau_actual = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.tau_total = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.ee_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        self.ee_quat = np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1))
        self.ref_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        self.ref_quat = np.zeros((Config.NUM_ARMS, 4), dtype=np.float64)
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

    def update_control_output(
        self,
        tau_total: np.ndarray,
        ee_pos: np.ndarray,
        ee_quat: np.ndarray,
        ref_pos: np.ndarray | None = None,
        ref_quat: np.ndarray | None = None,
    ) -> None:
        with self._lock:
            self.tau_total[:] = np.asarray(tau_total, dtype=np.float64).reshape(Config.NUM_JOINTS)
            self.ee_pos[:] = np.asarray(ee_pos, dtype=np.float64).reshape(Config.NUM_ARMS, 3)
            self.ee_quat[:] = np.asarray(ee_quat, dtype=np.float64).reshape(Config.NUM_ARMS, 4)
            if ref_pos is not None:
                self.ref_pos[:] = np.asarray(ref_pos, dtype=np.float64).reshape(Config.NUM_ARMS, 3)
            if ref_quat is not None:
                self.ref_quat[:] = np.asarray(ref_quat, dtype=np.float64).reshape(Config.NUM_ARMS, 4)

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
            return (
                self.q.copy(),
                self.qd.copy(),
                self.ref_pos.copy(),
                self.ref_quat.copy(),
                bool(self.feedback_valid),
                bool(self.targets_initialized),
            )


class RealRerunLogger:
    def __init__(
        self,
        queue_size: int | None = None,
        *,
        max_hz: float | None = None,
        perf_counter=time.perf_counter,
        sleep_fn=time.sleep,
    ) -> None:
        self._queue = queue.Queue(maxsize=max(1, int(queue_size or _env_int("AM_D02_RERUN_QUEUE_SIZE", 512))))
        effective_max_hz = Config.RERUN_MAX_HZ if max_hz is None else float(max_hz)
        self._period_s = 1.0 / effective_max_hz if effective_max_hz > 0.0 else 0.0
        self._last_log_time: float | None = None
        self._perf_counter = perf_counter
        self._sleep_fn = sleep_fn
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
            while True:
                try:
                    payload = self._queue.get_nowait()
                except queue.Empty:
                    break
            if self._last_log_time is not None and self._period_s > 0.0:
                elapsed_s = self._perf_counter() - self._last_log_time
                if elapsed_s < self._period_s:
                    self._sleep_fn(self._period_s - elapsed_s)
            rerun_viz.log_realtime_step(**payload)
            self._last_log_time = self._perf_counter()

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


class RealFeedbackHub:
    def __init__(self, perf_counter=_THREAD_WAIT_CLOCK) -> None:
        self._condition = threading.Condition()
        self._q = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self._qd = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self._tau_actual = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self._received_joint_masks = {arm: 0 for arm in range(Config.NUM_ARMS)}
        self._motor_seq = np.zeros((Config.NUM_ARMS, Config.ARM_JOINTS), dtype=np.int64)
        self._last_feedback_time = np.zeros((Config.NUM_ARMS, Config.ARM_JOINTS), dtype=np.float64)
        self._feedback_count = np.zeros((Config.NUM_ARMS, Config.ARM_JOINTS), dtype=np.int64)
        self._last_state = np.full((Config.NUM_ARMS, Config.ARM_JOINTS), -1, dtype=np.int64)
        self._last_can_id = np.zeros((Config.NUM_ARMS, Config.ARM_JOINTS), dtype=np.int64)
        self._arm_seq = {arm: 0 for arm in range(Config.NUM_ARMS)}
        self._arm_timestamp_s = {arm: 0.0 for arm in range(Config.NUM_ARMS)}
        self._consumed_arm_seq = {arm: 0 for arm in range(Config.NUM_ARMS)}
        self._complete_mask = 0
        self._seq = 0
        self._timestamp_s = 0.0
        self._error: Exception | None = None
        self._perf_counter = perf_counter

    def record_feedback_frame(self, arm: int, motor_ids: tuple[int, ...], frame) -> int:
        sample = (
            int(frame.motor_id),
            int(getattr(frame, "can_id", int(frame.motor_id))),
            int(getattr(frame, "state", 0)),
            float(frame.position),
            float(frame.velocity),
            float(frame.torque),
        )
        return self.record_feedback_batch(arm, motor_ids, (sample,))

    def record_feedback_batch(self, arm: int, motor_ids: tuple[int, ...], samples) -> int:
        arm_index = int(arm)
        valid_motor_mask = 0
        for motor_id in motor_ids:
            local = int(motor_id) - 1
            if 0 <= local < Config.ARM_JOINTS:
                valid_motor_mask |= 1 << local
        full_arm_mask = (1 << Config.ARM_JOINTS) - 1
        now_s = self._perf_counter()
        published_seq = 0
        with self._condition:
            arm_mask = int(self._received_joint_masks.get(arm_index, 0))
            for sample in samples:
                motor_id = int(sample[0])
                joint_local = motor_id - 1
                joint_bit = 1 << joint_local if 0 <= joint_local < Config.ARM_JOINTS else 0
                if (valid_motor_mask & joint_bit) == 0:
                    continue
                joint_global = arm_index * Config.ARM_JOINTS + joint_local
                self._q[joint_global] = float(sample[3])
                self._qd[joint_global] = float(sample[4])
                self._tau_actual[joint_global] = float(sample[5])
                self._motor_seq[arm_index, joint_local] += 1
                self._last_feedback_time[arm_index, joint_local] = now_s
                self._feedback_count[arm_index, joint_local] += 1
                self._last_can_id[arm_index, joint_local] = int(sample[1])
                self._last_state[arm_index, joint_local] = int(sample[2])
                arm_mask |= joint_bit
                if arm_mask == full_arm_mask:
                    self._complete_mask |= 1 << arm_index
                    self._received_joint_masks[arm_index] = 0
                    arm_mask = 0
                    self._seq += 1
                    published_seq = int(self._seq)
                    self._arm_seq[arm_index] = published_seq
                    self._arm_timestamp_s[arm_index] = now_s
                    self._timestamp_s = now_s
            self._received_joint_masks[arm_index] = arm_mask
            if published_seq:
                self._condition.notify_all()
        return published_seq

    def wait_until_consumed(self, arm: int, seq: int, timeout_s: float) -> None:
        deadline = _THREAD_WAIT_CLOCK() + max(0.0, float(timeout_s))
        arm_index = int(arm)
        seq_value = int(seq)
        with self._condition:
            while (
                not shutdown_event.is_set()
                and self._error is None
                and int(self._consumed_arm_seq.get(arm_index, 0)) < seq_value
            ):
                remaining = deadline - _THREAD_WAIT_CLOCK()
                if remaining <= 0.0:
                    return
                self._condition.wait(timeout=remaining)

    def set_error(self, exc: Exception) -> None:
        with self._condition:
            self._error = exc
            self._condition.notify_all()

    def latest_received_joint_masks(
        self,
        active_arm_mask: int | None = None,
        last_arm_seq: dict[int, int] | None = None,
    ) -> dict[int, int]:
        with self._condition:
            masks = dict(self._received_joint_masks)
            if active_arm_mask is not None and last_arm_seq is not None:
                full_arm_mask = (1 << Config.ARM_JOINTS) - 1
                for arm in range(Config.NUM_ARMS):
                    if (int(active_arm_mask) & (1 << arm)) != 0 and self._arm_seq[arm] > int(last_arm_seq.get(arm, 0)):
                        masks[arm] = full_arm_mask
            return masks

    def feedback_diagnostics(self, runtimes: tuple[ArmCanRuntime, ...], active_arm_mask: int) -> str:
        now_s = self._perf_counter()
        parts = []
        full_arm_mask = (1 << Config.ARM_JOINTS) - 1
        with self._condition:
            for runtime in runtimes:
                arm = int(runtime.arm)
                if (int(active_arm_mask) & (1 << arm)) == 0:
                    continue
                partial_mask = int(self._received_joint_masks.get(arm, 0))
                received_ids = [
                    str(int(motor_id))
                    for motor_id in runtime.motor_ids
                    if partial_mask & (1 << (int(motor_id) - 1))
                ]
                complete_age_ms = None
                if int(self._arm_seq.get(arm, 0)) > 0:
                    complete_age_ms = max(0.0, (now_s - float(self._arm_timestamp_s.get(arm, 0.0))) * 1000.0)
                motor_parts = []
                for motor_id in runtime.motor_ids:
                    local = int(motor_id) - 1
                    last_time = float(self._last_feedback_time[arm, local])
                    if last_time <= 0.0:
                        age = "never"
                    else:
                        age = f"{max(0.0, (now_s - last_time) * 1000.0):.1f}ms"
                    count = int(self._feedback_count[arm, local])
                    can_id = int(self._last_can_id[arm, local])
                    state = int(self._last_state[arm, local])
                    if can_id:
                        motor_parts.append(
                            f"id{int(motor_id)} age={age} count={count} can=0x{can_id:03X} state=0x{state:X}"
                        )
                    else:
                        motor_parts.append(f"id{int(motor_id)} age={age} count={count}")
                arm_complete = (self._complete_mask & (1 << arm)) != 0
                complete_age = "n/a" if complete_age_ms is None else f"{complete_age_ms:.1f}ms"
                parts.append(
                    f"{runtime.interface}: partial_ids={','.join(received_ids) if received_ids else 'none'} "
                    f"partial_mask=0x{partial_mask:02X}/0x{full_arm_mask:02X} "
                    f"complete={arm_complete} complete_age={complete_age} "
                    f"motors=[{'; '.join(motor_parts)}]"
                )
        return " | ".join(parts) if parts else "no active feedback diagnostics"

    def wait_for_next(
        self,
        active_arm_mask: int,
        *,
        last_seq: int,
        timeout_s: float,
        last_arm_seq: dict[int, int] | None = None,
    ) -> RealFeedbackSnapshot | None:
        deadline = _THREAD_WAIT_CLOCK() + max(0.0, float(timeout_s))
        with self._condition:
            while True:
                if self._error is not None:
                    raise self._error
                active_ready = (self._complete_mask & int(active_arm_mask)) == int(active_arm_mask)
                if active_ready and last_arm_seq is not None:
                    active_ready = all(
                        self._arm_seq[arm] > int(last_arm_seq.get(arm, 0))
                        for arm in range(Config.NUM_ARMS)
                        if (int(active_arm_mask) & (1 << arm)) != 0
                    )
                if self._seq > int(last_seq) and active_ready:
                    snapshot = RealFeedbackSnapshot(
                        seq=int(self._seq),
                        timestamp_s=float(self._timestamp_s),
                        q=self._q.copy(),
                        qd=self._qd.copy(),
                        tau_actual=self._tau_actual.copy(),
                        complete_mask=int(self._complete_mask),
                        received_joint_masks=dict(self._received_joint_masks),
                        arm_seq=dict(self._arm_seq),
                    )
                    for arm in range(Config.NUM_ARMS):
                        if (int(active_arm_mask) & (1 << arm)) != 0:
                            self._consumed_arm_seq[arm] = max(
                                int(self._consumed_arm_seq.get(arm, 0)),
                                int(snapshot.arm_seq.get(arm, 0)),
                            )
                    self._condition.notify_all()
                    return snapshot
                if shutdown_event.is_set():
                    return None
                remaining = deadline - _THREAD_WAIT_CLOCK()
                if remaining <= 0.0:
                    return None
                self._condition.wait(timeout=remaining)
        return None


class RealFeedbackWorker:
    def __init__(
        self,
        runtime: ArmCanRuntime,
        hub: RealFeedbackHub,
        *,
        poll_sleep_s: float | None = None,
        join_timeout_s: float | None = None,
        sleep_fn=time.sleep,
    ) -> None:
        self.runtime = runtime
        self.hub = hub
        self._poll_sleep_s = REAL_IDLE_SLEEP_S if poll_sleep_s is None else max(0.0, float(poll_sleep_s))
        self._join_timeout_s = REAL_THREAD_JOIN_TIMEOUT_S if join_timeout_s is None else max(0.0, float(join_timeout_s))
        self._sleep_fn = sleep_fn
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._worker,
            name=f"real-feedback-{runtime.interface}",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def request_stop(self) -> None:
        self._stop_event.set()

    def join(self) -> None:
        self._thread.join(timeout=self._join_timeout_s)

    def stop(self) -> None:
        self.request_stop()
        self.join()

    def _worker(self) -> None:
        while not self._stop_event.is_set() and not shutdown_event.is_set():
            processed_frames = 0
            published_seq = 0
            try:
                if REAL_FEEDBACK_BATCH_RX and hasattr(self.runtime.transport, "read_feedback_batch"):
                    samples = self.runtime.transport.read_feedback_batch(
                        CAN_READ_CHUNK_SIZE,
                        first_timeout_s=self._poll_sleep_s,
                        drain_timeout_s=0.0,
                    )
                    processed_frames = len(samples)
                    if samples:
                        published_seq = self.hub.record_feedback_batch(
                            self.runtime.arm,
                            self.runtime.motor_ids,
                            samples,
                        )
                elif hasattr(self.runtime.transport, "read_available"):
                    self.runtime.transport.read_available(CAN_READ_CHUNK_SIZE)
                    pop_feedback_frame = self.runtime.transport.pop_feedback_frame
                    while True:
                        frame = pop_feedback_frame()
                        if frame is None:
                            break
                        published_seq = (
                            self.hub.record_feedback_frame(self.runtime.arm, self.runtime.motor_ids, frame)
                            or published_seq
                        )
                        processed_frames += 1
                else:
                    self.runtime.transport.read(CAN_READ_CHUNK_SIZE)
                    pop_feedback_frame = self.runtime.transport.pop_feedback_frame
                    while True:
                        frame = pop_feedback_frame()
                        if frame is None:
                            break
                        published_seq = (
                            self.hub.record_feedback_frame(self.runtime.arm, self.runtime.motor_ids, frame)
                            or published_seq
                        )
                        processed_frames += 1
            except Exception as exc:
                self.hub.set_error(exc)
                shutdown_event.set()
                break
            if processed_frames == 0 and self._poll_sleep_s > 0.0:
                self._sleep_fn(self._poll_sleep_s)
            elif published_seq and REAL_FEEDBACK_WAIT_FOR_CONSUME:
                self.hub.wait_until_consumed(
                    self.runtime.arm,
                    published_seq,
                    timeout_s=max(0.0001, 1.0 / REAL_CONTROL_TARGET_HZ),
                )


class RealTxWorker:
    def __init__(
        self,
        runtime: ArmCanRuntime,
        *,
        join_timeout_s: float | None = None,
    ) -> None:
        self.runtime = runtime
        self._join_timeout_s = REAL_THREAD_JOIN_TIMEOUT_S if join_timeout_s is None else max(0.0, float(join_timeout_s))
        self._condition = threading.Condition()
        self._pending_tau: np.ndarray | None = None
        self._overwritten_pending_count = 0
        self._busy = False
        self._finalize_on_stop = False
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._worker,
            name=f"real-tx-{runtime.interface}",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def submit_torque(self, tau: np.ndarray) -> None:
        tau_values = np.asarray(tau, dtype=np.float64).reshape(Config.NUM_JOINTS).copy()
        with self._condition:
            if self._stop_event.is_set():
                return
            if self._pending_tau is not None:
                self._overwritten_pending_count += 1
            self._pending_tau = tau_values
            self._condition.notify_all()

    @property
    def overwritten_pending_count(self) -> int:
        with self._condition:
            return int(self._overwritten_pending_count)

    def wait_idle(self, timeout: float | None = None) -> bool:
        deadline = None if timeout is None else _THREAD_WAIT_CLOCK() + max(0.0, float(timeout))
        with self._condition:
            while self._busy or self._pending_tau is not None:
                if deadline is None:
                    remaining = None
                else:
                    remaining = deadline - _THREAD_WAIT_CLOCK()
                    if remaining <= 0.0:
                        return False
                self._condition.wait(timeout=remaining)
            return True

    def request_stop(self, *, finalize: bool = False) -> None:
        with self._condition:
            self._stop_event.set()
            self._finalize_on_stop = bool(finalize)
            self._pending_tau = None
            self._condition.notify_all()

    def join(self) -> None:
        self._thread.join(timeout=self._join_timeout_s)

    def stop(self, *, finalize: bool = False) -> None:
        self.request_stop(finalize=finalize)
        self.join()

    def send_zero_and_disable(self) -> None:
        for motor_id in self.runtime.motor_ids:
            try:
                self.runtime.transport.send_mit_torque(int(motor_id), 0.0)
            except Exception as exc:
                print(f"[CAN Warning] {self.runtime.interface} motor {motor_id} zero failed: {exc}")
        for motor_id in self.runtime.motor_ids:
            try:
                self.runtime.transport.disable_motor(int(motor_id))
            except Exception as exc:
                print(f"[CAN Warning] {self.runtime.interface} motor {motor_id} disable failed: {exc}")

    def _stop_requested(self) -> bool:
        with self._condition:
            return self._stop_event.is_set()

    def _worker(self) -> None:
        while True:
            with self._condition:
                while self._pending_tau is None and not self._stop_event.is_set():
                    self._condition.wait(timeout=0.1)
                if self._pending_tau is None and self._stop_event.is_set():
                    if not self._finalize_on_stop:
                        self._condition.notify_all()
                        return
                    self._busy = True
                    self._condition.notify_all()
                    tau = None
                else:
                    tau = self._pending_tau
                    self._pending_tau = None
                    self._busy = True
            if tau is None:
                try:
                    self.send_zero_and_disable()
                finally:
                    with self._condition:
                        self._busy = False
                        self._condition.notify_all()
                return
            try:
                offset = self.runtime.arm * Config.ARM_JOINTS
                for motor_id in self.runtime.motor_ids:
                    if self._stop_requested():
                        break
                    self.runtime.transport.send_mit_torque(
                        int(motor_id),
                        float(tau[offset + int(motor_id) - 1]),
                    )
            except Exception as exc:
                print(f"[CAN Error] {self.runtime.interface} torque send failed: {exc}")
                shutdown_event.set()
            finally:
                with self._condition:
                    self._busy = False
                    self._condition.notify_all()


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
        rx_buffer_bytes=CAN_RX_BUFFER_BYTES,
        filter_feedback=CAN_FILTER_FEEDBACK,
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


def _start_feedback_workers(
    runtimes: tuple[ArmCanRuntime, ...],
    hub: RealFeedbackHub,
) -> tuple[RealFeedbackWorker, ...]:
    workers = tuple(RealFeedbackWorker(runtime, hub) for runtime in runtimes)
    for worker in workers:
        worker.start()
    return workers


def _start_tx_workers(runtimes: tuple[ArmCanRuntime, ...]) -> dict[int, RealTxWorker]:
    workers = {runtime.arm: RealTxWorker(runtime) for runtime in runtimes}
    for worker in workers.values():
        worker.start()
    return workers


def _submit_torque_to_tx_workers(tx_workers: dict[int, RealTxWorker], tau: np.ndarray) -> None:
    tau_values = np.asarray(tau, dtype=np.float64).reshape(Config.NUM_JOINTS)
    for worker in tx_workers.values():
        worker.submit_torque(tau_values)


def _stop_feedback_workers(workers: tuple[RealFeedbackWorker, ...]) -> None:
    for worker in workers:
        worker.request_stop()
    for worker in workers:
        worker.join()


def _stop_tx_workers(workers: dict[int, RealTxWorker], *, finalize: bool) -> None:
    for worker in workers.values():
        worker.request_stop(finalize=finalize)
    for worker in workers.values():
        worker.join()


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


def _rerun_tcp_twist_with_position_fallback(
    controller_twist: np.ndarray,
    ee_pos: np.ndarray,
    previous_ee_pos: np.ndarray | None,
    elapsed_s: float,
    active_arm_mask: int,
) -> np.ndarray:
    """Prefer controller J(q)*qd twist, but keep Rerun speed live if velocity feedback is zero."""
    twist = np.asarray(controller_twist, dtype=np.float64).reshape(Config.NUM_ARMS, 6).copy()
    current_pos = np.asarray(ee_pos, dtype=np.float64).reshape(Config.NUM_ARMS, 3)
    if previous_ee_pos is None or elapsed_s <= 1e-9:
        return twist
    previous_pos = np.asarray(previous_ee_pos, dtype=np.float64).reshape(Config.NUM_ARMS, 3)
    for arm in range(Config.NUM_ARMS):
        if (active_arm_mask & (1 << arm)) == 0:
            continue
        linear = twist[arm, :3]
        delta = current_pos[arm] - previous_pos[arm]
        if not np.all(np.isfinite(linear)) or (
            np.linalg.norm(linear) <= 1e-12 and np.linalg.norm(delta) > 1e-12
        ):
            twist[arm, :3] = delta / float(elapsed_s)
    return twist


def _should_log_real_rerun_step(step_count: int) -> bool:
    stride = max(1, int(Config.RERUN_LOG_STRIDE))
    return stride <= 1 or int(step_count) % stride == 0


def _normalize_send_mode(send_mode: str | None, send_control: bool | None) -> str:
    if send_mode is None:
        if send_control is None:
            return SEND_MODE_CONTROL
        return SEND_MODE_CONTROL if bool(send_control) else SEND_MODE_ZERO
    normalized = str(send_mode).strip().lower()
    if normalized == "no-send":
        normalized = SEND_MODE_ZERO
    if normalized == "gc-only":
        normalized = SEND_MODE_GC
    if normalized not in SEND_MODES:
        raise ValueError(f"unsupported real send mode: {send_mode}")
    return normalized


def _send_mode_label(send_mode: str) -> str:
    if send_mode == SEND_MODE_CONTROL:
        return "MIT torque via SocketCAN"
    if send_mode == SEND_MODE_ZERO:
        return "Zero MIT torque (control no-send)"
    if send_mode == SEND_MODE_GRAVITY:
        return "Gravity compensation MIT torque"
    if send_mode == SEND_MODE_GC:
        return "G+C compensation MIT torque"
    raise ValueError(f"unsupported real send mode: {send_mode}")


def _copy_active_arm_torque(source: np.ndarray, active_arm_mask: int) -> np.ndarray:
    tau_to_send = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    source_values = np.asarray(source, dtype=np.float64).reshape(Config.NUM_JOINTS)
    for arm_index in range(Config.NUM_ARMS):
        if (int(active_arm_mask) & (1 << arm_index)) == 0:
            continue
        arm_slice = slice(
            arm_index * Config.ARM_JOINTS,
            (arm_index + 1) * Config.ARM_JOINTS,
        )
        tau_to_send[arm_slice] = source_values[arm_slice]
    return tau_to_send


def _select_tau_to_send(
    send_mode: str,
    *,
    tau_for_display: np.ndarray,
    tau_gravity: np.ndarray,
    tau_gc: np.ndarray,
    zero_tau: np.ndarray,
    active_arm_mask: int,
) -> np.ndarray:
    if send_mode == SEND_MODE_CONTROL:
        return np.asarray(tau_for_display, dtype=np.float64).reshape(Config.NUM_JOINTS).copy()
    if send_mode == SEND_MODE_ZERO:
        return np.asarray(zero_tau, dtype=np.float64).reshape(Config.NUM_JOINTS).copy()
    if send_mode == SEND_MODE_GRAVITY:
        return _copy_active_arm_torque(tau_gravity, active_arm_mask)
    if send_mode == SEND_MODE_GC:
        return _copy_active_arm_torque(tau_gc, active_arm_mask)
    raise ValueError(f"unsupported real send mode: {send_mode}")


def _tx_overwrite_count(tx_workers: dict[int, RealTxWorker]) -> int:
    return sum(worker.overwritten_pending_count for worker in tx_workers.values())


def _can_backpressure_count(runtimes: tuple[ArmCanRuntime, ...]) -> int:
    total = 0
    for runtime in runtimes:
        stats = getattr(runtime.transport, "stats", None)
        total += int(getattr(stats, "backpressure_count", 0))
    return total


def _format_can_rx_stats(runtimes: tuple[ArmCanRuntime, ...]) -> str:
    parts = []
    for runtime in runtimes:
        stats = getattr(runtime.transport, "stats", None)
        if stats is None:
            continue
        feedback_count = int(getattr(stats, "feedback_count", 0))
        frames_per_s = 0.0
        started_at = float(getattr(stats, "started_at_s", 0.0))
        if started_at > 0.0:
            elapsed_s = max(0.0, time.monotonic() - started_at)
            if elapsed_s > 0.0:
                frames_per_s = feedback_count / elapsed_s
        parts.append(
            f"{runtime.interface}: "
            f"reads={int(getattr(stats, 'read_count', 0))} "
            f"feedback={feedback_count} "
            f"frames_s={frames_per_s:.1f} "
            f"batches={int(getattr(stats, 'batch_read_count', 0))} "
            f"batch_feedback={int(getattr(stats, 'batch_feedback_count', 0))} "
            f"short={int(getattr(stats, 'ignored_short_count', 0))} "
            f"echo={int(getattr(stats, 'ignored_control_echo_count', 0))} "
            f"unknown={int(getattr(stats, 'unknown_feedback_id_count', 0))} "
            f"state_err={int(getattr(stats, 'state_error_count', 0))} "
            f"drain_limit={int(getattr(stats, 'drain_limit_hit_count', 0))}"
        )
    return "; ".join(parts) if parts else "none"


def _maybe_print_real_stats(
    *,
    now_s: float,
    last_print_s: float | None,
    cycle_hz: float,
    cycle_ms: float,
    c_bridge_ms: float,
    feedback_wait_ms: float,
    tx_overwrites: int,
    can_backpressure: int,
    can_rx_stats: str = "",
    send_mode: str,
) -> float | None:
    if REAL_STATS_INTERVAL_S <= 0.0:
        return last_print_s
    if last_print_s is not None and now_s - last_print_s < REAL_STATS_INTERVAL_S:
        return last_print_s
    print(
        "[Real Stats] "
        f"mode={send_mode} "
        f"target={REAL_CONTROL_TARGET_HZ:.0f}Hz "
        f"cycle={cycle_hz:.1f}Hz/{cycle_ms:.3f}ms "
        f"c_bridge={c_bridge_ms:.3f}ms "
        f"feedback_wait={feedback_wait_ms:.3f}ms "
        f"tx_overwrites={tx_overwrites} "
        f"can_backpressure={can_backpressure} "
        f"rx=[{can_rx_stats}]"
    )
    return now_s


def _real_c_body_q_zero() -> np.ndarray:
    # The current SocketCAN real C path has no live torso feedback.
    return REAL_C_BODY_Q_ZERO.copy()


def mirror_real_state_to_env(
    env,
    shared_state: RealSharedState,
    active_arm_mask: int,
    initial_target_pos_base: np.ndarray | None = None,
    initial_target_quat_base: np.ndarray | None = None,
) -> None:
    q, qd, ref_pos, ref_quat, feedback_valid, targets_initialized = shared_state.snapshot_mirror()
    if not feedback_valid:
        return
    env.set_qpos(_copy_active_arm_values(env.get_qpos(), q, active_arm_mask))
    env.set_qvel(_copy_active_arm_values(env.get_qvel(), qd, active_arm_mask))
    env.forward()
    if hasattr(env, "set_all_reference_poses"):
        env.set_all_reference_poses(ref_pos, ref_quat)
    if not targets_initialized:
        target_pos = env.get_all_ee_pos()
        target_quat = env.get_all_ee_quat()
        if initial_target_pos_base is not None:
            target_pos = _copy_active_arm_pose_values(target_pos, initial_target_pos_base, active_arm_mask)
        if initial_target_quat_base is not None:
            target_quat = _copy_active_arm_pose_values(target_quat, initial_target_quat_base, active_arm_mask)
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
                "target_quat=INIT_QPOS_TCP"
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
    send_mode: str | None = None,
    send_control: bool | None = None,
) -> None:
    active_mask = ACTIVE_ARM_MASK[mode]
    effective_send_mode = _normalize_send_mode(send_mode, send_control)
    last_cycle_end = None
    last_stats_print_s = None
    step_count = 0
    last_sent_tau = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    diag_window = deque(maxlen=REAL_DIAG_WINDOW_STEPS)
    zero_tau = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    zero_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
    zero_twist = np.zeros((Config.NUM_ARMS, 6), dtype=np.float64)
    zero_ref_quat = np.zeros((Config.NUM_ARMS, 4), dtype=np.float64)
    identity_quat = np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1))
    previous_rerun_ee_pos = None
    hub = RealFeedbackHub()
    feedback_workers: tuple[RealFeedbackWorker, ...] = ()
    tx_workers: dict[int, RealTxWorker] = {}

    def submit_keepalive(tau: np.ndarray) -> None:
        _submit_torque_to_tx_workers(tx_workers, tau)

    try:
        if startup_enable:
            _startup_enable(runtimes)
        feedback_wait_start = _THREAD_WAIT_CLOCK()
        last_control_time = time.perf_counter()
        command_in_effect = False
        last_feedback_request_time = 0.0
        last_feedback_seq = 0
        last_arm_seq = {arm: 0 for arm in range(Config.NUM_ARMS)}
        tx_workers = _start_tx_workers(runtimes)
        feedback_workers = _start_feedback_workers(runtimes, hub)

        def consume_ready_snapshot() -> RealFeedbackSnapshot | None:
            return hub.wait_for_next(
                active_mask,
                last_seq=last_feedback_seq,
                timeout_s=0.0,
                last_arm_seq=last_arm_seq,
            )

        while not shutdown_event.is_set():
            scheduler_now = _THREAD_WAIT_CLOCK()
            try:
                snapshot = consume_ready_snapshot()
            except Exception as exc:
                print(f"[CAN Error] feedback read failed: {exc}")
                shutdown_event.set()
                break

            wait_elapsed_s = max(0.0, scheduler_now - feedback_wait_start)
            if snapshot is None and wait_elapsed_s > CAN_FEEDBACK_TIMEOUT_S:
                try:
                    snapshot = consume_ready_snapshot()
                except Exception as exc:
                    print(f"[CAN Error] feedback read failed: {exc}")
                    shutdown_event.set()
                    break
                if snapshot is None:
                    missing_masks = hub.latest_received_joint_masks(active_mask, last_arm_seq)
                    missing = _format_missing_feedback(runtimes, missing_masks)
                    print(
                        f"[CAN Error] feedback timeout after {CAN_FEEDBACK_TIMEOUT_S:.3f}s; "
                        f"missing feedback: {missing}. Disabling active motors.\n"
                        f"[CAN Diagnostics] {hub.feedback_diagnostics(runtimes, active_mask)}\n"
                        f"[CAN RX Stats] {_format_can_rx_stats(runtimes)} "
                        f"tx_overwrites={_tx_overwrite_count(tx_workers)} "
                        f"can_backpressure={_can_backpressure_count(runtimes)}"
                    )
                    shutdown_event.set()
                    break

            remaining_timeout_s = max(0.0, CAN_FEEDBACK_TIMEOUT_S - wait_elapsed_s)
            if REAL_FEEDBACK_REQUEST_INTERVAL_S > 0.0:
                next_keepalive_s = max(
                    0.0,
                    REAL_FEEDBACK_REQUEST_INTERVAL_S - (scheduler_now - last_feedback_request_time),
                )
            else:
                next_keepalive_s = max(REAL_IDLE_SLEEP_S, 0.0001)
            wait_timeout_s = min(
                remaining_timeout_s,
                max(REAL_IDLE_SLEEP_S, 0.0001),
                max(next_keepalive_s, 0.0001),
            )
            if snapshot is None:
                try:
                    snapshot = hub.wait_for_next(
                        active_mask,
                        last_seq=last_feedback_seq,
                        timeout_s=wait_timeout_s,
                        last_arm_seq=last_arm_seq,
                    )
                except Exception as exc:
                    print(f"[CAN Error] feedback read failed: {exc}")
                    shutdown_event.set()
                    break

            if snapshot is None:
                scheduler_now = _THREAD_WAIT_CLOCK()
                if (not command_in_effect) or (
                    scheduler_now - last_feedback_request_time >= REAL_FEEDBACK_REQUEST_INTERVAL_S
                ):
                    submit_keepalive(last_sent_tau)
                    command_in_effect = True
                    last_feedback_request_time = scheduler_now
                continue

            last_feedback_seq = snapshot.seq
            last_arm_seq = dict(snapshot.arm_seq)
            feedback_ready_time = _THREAD_WAIT_CLOCK()
            feedback_wait_ms = max(0.0, (feedback_ready_time - feedback_wait_start) * 1000.0)
            feedback_wait_start = feedback_ready_time
            shared_state.update_feedback(snapshot.q, snapshot.qd, snapshot.tau_actual, active_mask)
            if require_targets and not shared_state.targets_ready():
                submit_keepalive(zero_tau)
                last_sent_tau[:] = zero_tau
                command_in_effect = True
                last_feedback_request_time = _THREAD_WAIT_CLOCK()
                continue

            current_q, current_qd, current_tau_actual, target_pos, target_quat = shared_state.snapshot_control_inputs()
            now = time.perf_counter()
            elapsed_s = max(0.0, now - last_control_time)
            last_control_time = now

            if feedback_only:
                submit_keepalive(zero_tau)
                last_sent_tau[:] = zero_tau
                command_in_effect = True
                last_feedback_request_time = _THREAD_WAIT_CLOCK()
                shared_state.update_control_output(
                    zero_tau,
                    zero_pos,
                    identity_quat,
                    ref_pos=zero_pos,
                    ref_quat=zero_ref_quat,
                )
                cycle_end = time.perf_counter()
                cycle_ms = 0.0
                cycle_hz = 0.0
                if last_cycle_end is not None:
                    cycle_dt = cycle_end - last_cycle_end
                    if cycle_dt > 0.0:
                        cycle_ms = cycle_dt * 1000.0
                        cycle_hz = 1.0 / cycle_dt
                last_cycle_end = cycle_end
                tx_overwrites = _tx_overwrite_count(tx_workers)
                can_backpressure = _can_backpressure_count(runtimes)
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
                if (
                    rerun_logger is not None
                    and Config.ENABLE_RERUN
                    and _should_log_real_rerun_step(step_count)
                ):
                    rerun_logger.log_step(
                        t=step_count * max(elapsed_s, 0.0),
                        pos_actual=zero_pos,
                        pos_desired=target_pos,
                        quat_actual=identity_quat,
                        quat_desired=target_quat,
                        tau_raw=zero_tau,
                        tau_total=zero_tau,
                        ee_twist=zero_twist,
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
                        c_bridge_ms=0.0,
                        feedback_wait_ms=feedback_wait_ms,
                        tx_overwrite_count=tx_overwrites,
                        can_backpressure_count=can_backpressure,
                        control_target_hz=REAL_CONTROL_TARGET_HZ,
                    )
                last_stats_print_s = _maybe_print_real_stats(
                    now_s=cycle_end,
                    last_print_s=last_stats_print_s,
                    cycle_hz=cycle_hz,
                    cycle_ms=cycle_ms,
                    c_bridge_ms=0.0,
                    feedback_wait_ms=feedback_wait_ms,
                    tx_overwrites=tx_overwrites,
                    can_backpressure=can_backpressure,
                    can_rx_stats=_format_can_rx_stats(runtimes),
                    send_mode="feedback-only",
                )
                step_count += 1
                continue

            bridge_start = time.perf_counter()
            try:
                result = bridge.compute(
                    active_mask,
                    elapsed_s,
                    current_q,
                    current_qd,
                    _real_c_body_q_zero(),
                    target_pos,
                    target_quat,
                )
            except Exception as exc:
                print(f"[CAN Error] real controller bridge failed: {exc}. Disabling active motors.")
                shutdown_event.set()
                break
            c_bridge_ms = (time.perf_counter() - bridge_start) * 1000.0
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
            tau_gravity = np.asarray(
                getattr(result, "tau_gravity", zero_tau),
                dtype=np.float64,
            ).reshape(Config.NUM_JOINTS)
            tau_gc = np.asarray(
                getattr(result, "tau_gc", zero_tau),
                dtype=np.float64,
            ).reshape(Config.NUM_JOINTS)
            ref_pos = np.asarray(
                getattr(result, "ref_pos", zero_pos),
                dtype=np.float64,
            ).reshape(Config.NUM_ARMS, 3)
            ref_quat = np.asarray(
                getattr(result, "ref_quat", zero_ref_quat),
                dtype=np.float64,
            ).reshape(Config.NUM_ARMS, 4)

            rerun_ee_twist = _rerun_tcp_twist_with_position_fallback(
                result.ee_twist,
                result.ee_pos,
                previous_rerun_ee_pos,
                elapsed_s,
                active_mask,
            )
            previous_rerun_ee_pos = np.asarray(result.ee_pos, dtype=np.float64).reshape(
                Config.NUM_ARMS, 3
            ).copy()

            tau_for_display = tau_raw.copy()
            for arm_index in range(Config.NUM_ARMS):
                if (active_mask & (1 << arm_index)) == 0:
                    arm_slice = slice(
                        arm_index * Config.ARM_JOINTS,
                        (arm_index + 1) * Config.ARM_JOINTS,
                    )
                    tau_for_display[arm_slice] = 0.0
            tau_to_send = _select_tau_to_send(
                effective_send_mode,
                tau_for_display=tau_for_display,
                tau_gravity=tau_gravity,
                tau_gc=tau_gc,
                zero_tau=zero_tau,
                active_arm_mask=active_mask,
            )
            tx_label = _send_mode_label(effective_send_mode)
            shared_state.update_control_output(
                tau_for_display,
                result.ee_pos,
                result.ee_quat,
                ref_pos=ref_pos,
                ref_quat=ref_quat,
            )
            _submit_torque_to_tx_workers(tx_workers, tau_to_send)
            last_sent_tau[:] = tau_to_send
            command_in_effect = True
            last_feedback_request_time = _THREAD_WAIT_CLOCK()
            if shutdown_event.is_set():
                for worker in tx_workers.values():
                    worker.wait_idle(timeout=REAL_THREAD_JOIN_TIMEOUT_S)

            cycle_end = time.perf_counter()
            cycle_ms = 0.0
            cycle_hz = 0.0
            if last_cycle_end is not None:
                cycle_dt = cycle_end - last_cycle_end
                if cycle_dt > 0.0:
                    cycle_ms = cycle_dt * 1000.0
                    cycle_hz = 1.0 / cycle_dt
            last_cycle_end = cycle_end
            tx_overwrites = _tx_overwrite_count(tx_workers)
            can_backpressure = _can_backpressure_count(runtimes)
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

            if (
                rerun_logger is not None
                and Config.ENABLE_RERUN
                and _should_log_real_rerun_step(step_count)
            ):
                rerun_logger.log_step(
                    t=result.traj_t,
                    pos_actual=result.ee_pos,
                    pos_desired=target_pos,
                    quat_actual=result.ee_quat,
                    quat_desired=target_quat,
                    pos_reference=ref_pos,
                    quat_reference=ref_quat,
                    tau_raw=tau_raw,
                    tau_total=tau_for_display,
                    ee_twist=rerun_ee_twist,
                    cycle_time=elapsed_s * 1000.0,
                    elapsed_s=elapsed_s,
                    q=current_q,
                    qd=current_qd,
                    tau_actual=current_tau_actual,
                    right_j7_diag=right_j7_diag,
                    rx_str=_format_real_rx(current_q, active_mask),
                    tx_str=_format_real_tx(tau_to_send, active_mask),
                    tx_label=tx_label,
                    step_count=step_count,
                    uart_latency_ms=cycle_ms,
                    uart_cycle_hz=cycle_hz,
                    uart_transfer_kbps=0.0,
                    c_bridge_ms=c_bridge_ms,
                    feedback_wait_ms=feedback_wait_ms,
                    tx_overwrite_count=tx_overwrites,
                    can_backpressure_count=can_backpressure,
                    control_target_hz=REAL_CONTROL_TARGET_HZ,
                )
            last_stats_print_s = _maybe_print_real_stats(
                now_s=cycle_end,
                last_print_s=last_stats_print_s,
                cycle_hz=cycle_hz,
                cycle_ms=cycle_ms,
                c_bridge_ms=c_bridge_ms,
                feedback_wait_ms=feedback_wait_ms,
                tx_overwrites=tx_overwrites,
                can_backpressure=can_backpressure,
                can_rx_stats=_format_can_rx_stats(runtimes),
                send_mode=effective_send_mode,
            )
            step_count += 1
    finally:
        _stop_feedback_workers(feedback_workers)
        if tx_workers:
            _stop_tx_workers(tx_workers, finalize=True)
        else:
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
    send_mode: str | None = None,
    send_control: bool | None = None,
) -> None:
    shared_state = RealSharedState()
    run_real_can_control_loop(
        mode=mode,
        bridge=bridge,
        runtimes=runtimes,
        shared_state=shared_state,
        startup_enable=startup_enable,
        send_mode=send_mode,
        send_control=send_control,
    )


def run_real_control_with_bridge(
    arm: str,
    bridge=None,
    *,
    bridge_factory=None,
    control_title: str = "AM-DPBSURDF0422 Real SocketCAN Control",
    rerun_app_name: str = "AM-DPBSURDF0422 Real CAN",
    send_mode: str | None = None,
    send_control: bool | None = None,
) -> None:
    if bridge is None and bridge_factory is None:
        raise ValueError("run_real_control_with_bridge requires bridge or bridge_factory")
    effective_send_mode = _normalize_send_mode(send_mode, send_control)
    print("=" * 60)
    print(f"[Real Config] idle sleep while waiting feedback: {REAL_IDLE_SLEEP_S * 1000.0:.3f} ms")
    print(f"[Real Config] control target: {REAL_CONTROL_TARGET_HZ:.0f} Hz")
    print(f"[Real Config] rerun visualization: {Config.ENABLE_RERUN}")
    print(f"[Real Config] feedback-only diagnostics: {REAL_FEEDBACK_ONLY}")
    print(f"[Real Config] torque send mode: {effective_send_mode}")
    print(
        "[Real Config] CAN RX: "
        f"batch={REAL_FEEDBACK_BATCH_RX} "
        f"wait_for_consume={REAL_FEEDBACK_WAIT_FOR_CONSUME} "
        f"filter_feedback={CAN_FILTER_FEEDBACK} "
        f"rx_buffer={CAN_RX_BUFFER_BYTES} bytes"
    )
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
    initial_target_quat_base = init_env.get_all_ee_quat()
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
                "send_mode": effective_send_mode,
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
                    initial_target_quat_base,
                )
                if viewer is not None:
                    viewer.sync()
                time.sleep(1.0 / max(1.0, float(_env_int("AM_D02_REAL_VIEWER_FPS", 30))))
        except KeyboardInterrupt:
            shutdown_event.set()
        can_thread.join(timeout=REAL_THREAD_JOIN_TIMEOUT_S)
    finally:
        if viewer is not None:
            viewer.close()
        if rerun_logger is not None:
            rerun_logger.close()
        if bridge is not None:
            bridge.close()


def run_real_control(
    arm: str,
    *,
    send_mode: str | None = None,
    send_control: bool | None = None,
) -> None:
    bridge = RealControllerBridge()
    run_real_control_with_bridge(
        arm,
        bridge,
        send_mode=send_mode,
        send_control=send_control,
    )
