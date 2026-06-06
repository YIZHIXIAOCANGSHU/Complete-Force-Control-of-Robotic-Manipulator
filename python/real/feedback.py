"""Feedback aggregation for real SocketCAN control."""

from __future__ import annotations

import threading
import time

import numpy as np

from config import Config
from real.types import ArmCanRuntime, RealFeedbackSnapshot


class RealFeedbackHub:
    def __init__(self, *, shutdown_event=None, wait_clock=time.monotonic, perf_counter=time.monotonic) -> None:
        self._condition = threading.Condition()
        self._shutdown_event = threading.Event() if shutdown_event is None else shutdown_event
        self._wait_clock = wait_clock
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
        deadline = self._wait_clock() + max(0.0, float(timeout_s))
        arm_index = int(arm)
        seq_value = int(seq)
        with self._condition:
            while (
                not self._shutdown_event.is_set()
                and self._error is None
                and int(self._consumed_arm_seq.get(arm_index, 0)) < seq_value
            ):
                remaining = deadline - self._wait_clock()
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
        deadline = self._wait_clock() + max(0.0, float(timeout_s))
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
                if self._shutdown_event.is_set():
                    return None
                remaining = deadline - self._wait_clock()
                if remaining <= 0.0:
                    return None
                self._condition.wait(timeout=remaining)
        return None
