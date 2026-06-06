"""Feedback and TX worker threads for real SocketCAN control."""

from __future__ import annotations

import threading
import time

import numpy as np

from config import Config
from real.feedback import RealFeedbackHub
from real.types import ArmCanRuntime


class RealFeedbackWorker:
    def __init__(
        self,
        runtime: ArmCanRuntime,
        hub: RealFeedbackHub,
        *,
        poll_sleep_s: float | None = None,
        join_timeout_s: float | None = None,
        sleep_fn=time.sleep,
        default_idle_sleep_s: float = 0.0005,
        default_join_timeout_s: float = 2.0,
        read_chunk_size: int = 256,
        feedback_batch_rx: bool = True,
        feedback_wait_for_consume: bool = False,
        control_target_hz: float = 1000.0,
        shutdown_event=None,
    ) -> None:
        self.runtime = runtime
        self.hub = hub
        self._poll_sleep_s = default_idle_sleep_s if poll_sleep_s is None else max(0.0, float(poll_sleep_s))
        self._join_timeout_s = default_join_timeout_s if join_timeout_s is None else max(0.0, float(join_timeout_s))
        self._sleep_fn = sleep_fn
        self._read_chunk_size = int(read_chunk_size)
        self._feedback_batch_rx = bool(feedback_batch_rx)
        self._feedback_wait_for_consume = bool(feedback_wait_for_consume)
        self._control_target_hz = max(1.0, float(control_target_hz))
        self._shutdown_event = threading.Event() if shutdown_event is None else shutdown_event
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
        while not self._stop_event.is_set() and not self._shutdown_event.is_set():
            processed_frames = 0
            published_seq = 0
            try:
                if self._feedback_batch_rx and hasattr(self.runtime.transport, "read_feedback_batch"):
                    samples = self.runtime.transport.read_feedback_batch(
                        self._read_chunk_size,
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
                    self.runtime.transport.read_available(self._read_chunk_size)
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
                    self.runtime.transport.read(self._read_chunk_size)
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
                self._shutdown_event.set()
                break
            if processed_frames == 0 and self._poll_sleep_s > 0.0:
                self._sleep_fn(self._poll_sleep_s)
            elif published_seq and self._feedback_wait_for_consume:
                self.hub.wait_until_consumed(
                    self.runtime.arm,
                    published_seq,
                    timeout_s=max(0.0001, 1.0 / self._control_target_hz),
                )


class RealTxWorker:
    def __init__(
        self,
        runtime: ArmCanRuntime,
        *,
        join_timeout_s: float | None = None,
        default_join_timeout_s: float = 2.0,
        shutdown_event=None,
        wait_clock=time.monotonic,
    ) -> None:
        self.runtime = runtime
        self._join_timeout_s = default_join_timeout_s if join_timeout_s is None else max(0.0, float(join_timeout_s))
        self._shutdown_event = threading.Event() if shutdown_event is None else shutdown_event
        self._wait_clock = wait_clock
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
        deadline = None if timeout is None else self._wait_clock() + max(0.0, float(timeout))
        with self._condition:
            while self._busy or self._pending_tau is not None:
                if deadline is None:
                    remaining = None
                else:
                    remaining = deadline - self._wait_clock()
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
                self._shutdown_event.set()
            finally:
                with self._condition:
                    self._busy = False
                    self._condition.notify_all()
