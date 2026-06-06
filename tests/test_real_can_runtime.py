from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "python"))

from config import Config
from real.can_transport import Usb2FdcanConfig, Usb2FdcanTransport
from real import runtime


class FakeTransport:
    def __init__(self, config=None, frames=None) -> None:
        self.config = config
        self.frames = list(frames or [])
        self.commands: list[tuple[str, int, float | None]] = []
        self.closed = False
        self.read_available_calls: list[int] = []

    def read(self, size: int) -> bytes:
        _ = size
        return b""

    def read_available(self, max_frames: int) -> bytes:
        self.read_available_calls.append(int(max_frames))
        return b""

    def pop_feedback_frame(self):
        if not self.frames:
            return None
        return self.frames.pop(0)

    def reset_input_buffer(self) -> None:
        self.commands.append(("reset", 0, None))

    def clear_error(self, motor_id: int) -> bytes:
        self.commands.append(("clear", int(motor_id), None))
        return b"clear"

    def enable_motor(self, motor_id: int) -> bytes:
        self.commands.append(("enable", int(motor_id), None))
        return b"enable"

    def send_mit_torque(self, motor_id: int, torque: float) -> bytes:
        self.commands.append(("torque", int(motor_id), float(torque)))
        return b"torque"

    def disable_motor(self, motor_id: int) -> bytes:
        self.commands.append(("disable", int(motor_id), None))
        return b"disable"

    def close(self) -> None:
        self.closed = True


def _feedback_sample(motor_id: int, *, offset: float = 0.0, can_id: int | None = None, state: int = 1):
    return (
        int(motor_id),
        int(motor_id if can_id is None else can_id),
        int(state),
        0.1 * motor_id + float(offset),
        0.01 * motor_id + float(offset),
        0.001 * motor_id + float(offset),
    )


class BlockingTransport(FakeTransport):
    def __init__(self, config=None, frames=None) -> None:
        super().__init__(config=config, frames=frames)
        self.block_event = runtime.threading.Event()
        self.entered_event = runtime.threading.Event()

    def send_mit_torque(self, motor_id: int, torque: float) -> bytes:
        self.entered_event.set()
        self.block_event.wait(timeout=1.0)
        return super().send_mit_torque(motor_id, torque)


class StopOnZeroTransport(FakeTransport):
    def send_mit_torque(self, motor_id: int, torque: float) -> bytes:
        result = super().send_mit_torque(motor_id, torque)
        if float(torque) == 0.0:
            runtime.shutdown_event.set()
        return result


class ChunkedFeedbackTransport(FakeTransport):
    def __init__(self, chunks) -> None:
        super().__init__(frames=[])
        self.chunks = [list(chunk) for chunk in chunks]
        self.current_chunk: list = []

    def read_available(self, max_frames: int) -> bytes:
        self.read_available_calls.append(int(max_frames))
        self.current_chunk = self.chunks.pop(0) if self.chunks else []
        return b""

    def pop_feedback_frame(self):
        if not self.current_chunk:
            return None
        return self.current_chunk.pop(0)


class BatchFeedbackTransport(FakeTransport):
    def __init__(self, chunks) -> None:
        super().__init__(frames=[])
        self.chunks = [list(chunk) for chunk in chunks]
        self.batch_calls: list[tuple[int, float, float]] = []
        self.second_batch_event = runtime.threading.Event()

    def read_feedback_batch(self, max_frames: int, first_timeout_s: float, drain_timeout_s: float = 0.0):
        self.batch_calls.append((int(max_frames), float(first_timeout_s), float(drain_timeout_s)))
        chunk = self.chunks.pop(0) if self.chunks else []
        if len(self.batch_calls) >= 2:
            self.second_batch_event.set()
        return chunk


class SlowStartupTransport(FakeTransport):
    def __init__(self, frames=None) -> None:
        super().__init__(frames=frames)
        self.enable_calls = 0

    def enable_motor(self, motor_id: int) -> bytes:
        self.enable_calls += 1
        return super().enable_motor(motor_id)


class StopAfterOneBridge:
    def __init__(self) -> None:
        self.calls = []

    def compute(self, active_arm_mask, elapsed_s, q, qd, body_q, target_pos, target_quat):
        self.calls.append(
            {
                "active_arm_mask": int(active_arm_mask),
                "q": np.asarray(q, dtype=np.float64).copy(),
                "qd": np.asarray(qd, dtype=np.float64).copy(),
                "body_q": np.asarray(body_q, dtype=np.float64).copy(),
                "target_pos": np.asarray(target_pos, dtype=np.float64).copy(),
                "target_quat": np.asarray(target_quat, dtype=np.float64).copy(),
            }
        )
        runtime.shutdown_event.set()
        return SimpleNamespace(
            status=0,
            tau=np.arange(1.0, Config.NUM_JOINTS + 1.0),
            tau_gravity=np.arange(101.0, 101.0 + Config.NUM_JOINTS),
            tau_gc=np.arange(201.0, 201.0 + Config.NUM_JOINTS),
            ee_pos=np.array([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]], dtype=np.float64),
            ee_quat=np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1)),
            ref_pos=np.array([[0.11, 0.21, 0.31], [0.0, 0.0, 0.0]], dtype=np.float64),
            ref_quat=np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]], dtype=np.float64),
            ee_twist=np.array([[0.01, 0.02, 0.03, 0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float64),
            traj_t=0.123,
        )


class StopAfterTwoBridge:
    def __init__(self) -> None:
        self.calls = []

    def compute(self, active_arm_mask, elapsed_s, q, qd, body_q, target_pos, target_quat):
        self.calls.append(
            {
                "active_arm_mask": int(active_arm_mask),
                "q": np.asarray(q, dtype=np.float64).copy(),
                "qd": np.asarray(qd, dtype=np.float64).copy(),
                "body_q": np.asarray(body_q, dtype=np.float64).copy(),
                "target_pos": np.asarray(target_pos, dtype=np.float64).copy(),
                "target_quat": np.asarray(target_quat, dtype=np.float64).copy(),
            }
        )
        if len(self.calls) >= 2:
            runtime.shutdown_event.set()
        return SimpleNamespace(
            status=0,
            tau=np.ones(Config.NUM_JOINTS, dtype=np.float64),
            tau_gravity=np.full(Config.NUM_JOINTS, 0.5, dtype=np.float64),
            tau_gc=np.full(Config.NUM_JOINTS, 0.75, dtype=np.float64),
            ee_pos=np.zeros((Config.NUM_ARMS, 3), dtype=np.float64),
            ee_quat=np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1)),
            ee_twist=np.zeros((Config.NUM_ARMS, 6), dtype=np.float64),
            traj_t=float(len(self.calls)),
        )


class MovingTcpZeroTwistBridge:
    def __init__(self) -> None:
        self.calls = []

    def compute(self, active_arm_mask, elapsed_s, q, qd, body_q, target_pos, target_quat):
        self.calls.append(
            {
                "active_arm_mask": int(active_arm_mask),
                "elapsed_s": float(elapsed_s),
                "q": np.asarray(q, dtype=np.float64).copy(),
                "qd": np.asarray(qd, dtype=np.float64).copy(),
            }
        )
        if len(self.calls) >= 2:
            runtime.shutdown_event.set()
        ee_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        ee_pos[Config.LEFT_ARM, 0] = 0.01 * (len(self.calls) - 1)
        return SimpleNamespace(
            status=0,
            tau=np.ones(Config.NUM_JOINTS, dtype=np.float64),
            tau_gravity=np.full(Config.NUM_JOINTS, 0.5, dtype=np.float64),
            tau_gc=np.full(Config.NUM_JOINTS, 0.75, dtype=np.float64),
            ee_pos=ee_pos,
            ee_quat=np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1)),
            ee_twist=np.zeros((Config.NUM_ARMS, 6), dtype=np.float64),
            traj_t=float(len(self.calls)),
        )


class TimeoutBridge:
    def __init__(self) -> None:
        self.calls = 0

    def compute(self, active_arm_mask, elapsed_s, q, qd, body_q, target_pos, target_quat):
        _ = active_arm_mask, elapsed_s, q, qd, body_q, target_pos, target_quat
        self.calls += 1
        raise TimeoutError("real controller bridge timed out")


class ShutdownAfterFirstControlThenPartialFeedbackTransport(ChunkedFeedbackTransport):
    def __init__(self, chunks) -> None:
        super().__init__(chunks)
        self.zero_before_shutdown = 0
        self.command_count_when_partial_feedback_read = None

    def read_available(self, max_frames: int) -> bytes:
        result = super().read_available(max_frames)
        if self.read_available_calls and len(self.read_available_calls) >= 2:
            self.command_count_when_partial_feedback_read = len(self.commands)
            runtime.shutdown_event.set()
        return result

    def send_mit_torque(self, motor_id: int, torque: float) -> bytes:
        if float(torque) == 0.0 and not runtime.shutdown_event.is_set():
            self.zero_before_shutdown += 1
        return super().send_mit_torque(motor_id, torque)


class StopAfterRepeatedFeedbackRequestsTransport(ChunkedFeedbackTransport):
    def __init__(self, chunks, stop_after_commands: int) -> None:
        super().__init__(chunks)
        self.stop_after_commands = int(stop_after_commands)

    def send_mit_torque(self, motor_id: int, torque: float) -> bytes:
        result = super().send_mit_torque(motor_id, torque)
        if len(self.commands) >= self.stop_after_commands:
            runtime.shutdown_event.set()
        return result


class FakeRerunLogger:
    def __init__(self) -> None:
        self.payloads = []
        self.closed = False

    def start(self) -> None:
        pass

    def log_step(self, **payload) -> None:
        self.payloads.append(payload)

    def close(self) -> None:
        self.closed = True


def test_real_rerun_logger_coalesces_backlog_to_latest_payload(monkeypatch):
    logged = []
    monkeypatch.setattr(runtime.rerun_viz, "log_realtime_step", lambda **payload: logged.append(payload))

    logger = runtime.RealRerunLogger(queue_size=4, max_hz=1000.0)
    logger.log_step(step_count=1)
    logger.log_step(step_count=2)
    logger.log_step(step_count=3)
    logger._stop_event.set()
    logger._worker()

    assert [payload["step_count"] for payload in logged] == [3]


def test_real_tx_worker_coalesces_pending_torque_to_latest():
    transport = BlockingTransport()
    arm_runtime = runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", transport)
    worker = runtime.RealTxWorker(arm_runtime, join_timeout_s=0.5)
    first = np.ones(Config.NUM_JOINTS, dtype=np.float64)
    second = np.full(Config.NUM_JOINTS, 2.0, dtype=np.float64)
    third = np.full(Config.NUM_JOINTS, 3.0, dtype=np.float64)

    try:
        worker.start()
        worker.submit_torque(first)
        assert transport.entered_event.wait(timeout=1.0)
        worker.submit_torque(second)
        worker.submit_torque(third)
        transport.block_event.set()
        worker.wait_idle(timeout=1.0)
    finally:
        worker.stop()

    nonzero = [cmd for cmd in transport.commands if cmd[0] == "torque" and cmd[2] not in (0.0, None)]
    assert [cmd[1] for cmd in nonzero[: Config.ARM_JOINTS]] == list(range(1, 8))
    assert [cmd[2] for cmd in nonzero[: Config.ARM_JOINTS]] == [1.0] * Config.ARM_JOINTS
    assert [cmd[2] for cmd in nonzero[-Config.ARM_JOINTS :]] == [3.0] * Config.ARM_JOINTS
    assert all(cmd[2] != 2.0 for cmd in nonzero)
    assert worker.overwritten_pending_count == 1


def test_real_feedback_worker_publishes_complete_arm_snapshot_across_reads():
    chunks = [
        [_feedback_frame(i) for i in range(1, 4)],
        [_feedback_frame(i) for i in range(4, 8)],
    ]
    transport = ChunkedFeedbackTransport(chunks)
    arm_runtime = runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport)
    hub = runtime.RealFeedbackHub()
    worker = runtime.RealFeedbackWorker(arm_runtime, hub, poll_sleep_s=0.0, join_timeout_s=0.5)

    try:
        worker.start()
        snapshot = hub.wait_for_next(runtime.ACTIVE_ARM_MASK["right"], last_seq=0, timeout_s=1.0)
    finally:
        worker.stop()

    assert snapshot is not None
    assert snapshot.seq == 1
    np.testing.assert_allclose(snapshot.q[Config.ARM_JOINTS :], [0.1 * i for i in range(1, 8)])
    np.testing.assert_allclose(snapshot.qd[Config.ARM_JOINTS :], [0.01 * i for i in range(1, 8)])


def test_real_feedback_hub_batch_publishes_latest_complete_snapshot():
    hub = runtime.RealFeedbackHub()
    first_cycle = [_feedback_sample(i) for i in range(1, 8)]
    second_cycle = [_feedback_sample(i, offset=1.0) for i in range(1, 8)]

    seq = hub.record_feedback_batch(Config.LEFT_ARM, tuple(range(1, 8)), first_cycle + second_cycle)
    snapshot = hub.wait_for_next(runtime.ACTIVE_ARM_MASK["left"], last_seq=0, timeout_s=0.0)

    assert seq == 2
    assert snapshot is not None
    assert snapshot.seq == 2
    np.testing.assert_allclose(snapshot.q[: Config.ARM_JOINTS], [1.0 + 0.1 * i for i in range(1, 8)])
    np.testing.assert_allclose(snapshot.qd[: Config.ARM_JOINTS], [1.0 + 0.01 * i for i in range(1, 8)])
    assert hub.wait_for_next(
        runtime.ACTIVE_ARM_MASK["left"],
        last_seq=snapshot.seq,
        timeout_s=0.0,
        last_arm_seq=snapshot.arm_seq,
    ) is None


def test_real_feedback_worker_uses_batch_rx_and_does_not_wait_for_control_consumption(monkeypatch):
    monkeypatch.setattr(runtime, "REAL_FEEDBACK_BATCH_RX", True)
    monkeypatch.setattr(runtime, "REAL_FEEDBACK_WAIT_FOR_CONSUME", False)
    transport = BatchFeedbackTransport(
        [
            [_feedback_sample(i) for i in range(1, 8)],
            [_feedback_sample(i, offset=1.0) for i in range(1, 8)],
        ]
    )
    arm_runtime = runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", transport)
    hub = runtime.RealFeedbackHub()
    worker = runtime.RealFeedbackWorker(arm_runtime, hub, poll_sleep_s=0.001, join_timeout_s=0.5)

    runtime.shutdown_event.clear()
    try:
        worker.start()
        assert transport.second_batch_event.wait(timeout=1.0)
        snapshot = hub.wait_for_next(runtime.ACTIVE_ARM_MASK["left"], last_seq=0, timeout_s=1.0)
    finally:
        worker.stop()
        runtime.shutdown_event.clear()

    assert snapshot is not None
    assert snapshot.seq == 2
    np.testing.assert_allclose(snapshot.q[: Config.ARM_JOINTS], [1.0 + 0.1 * i for i in range(1, 8)])
    assert len(transport.batch_calls) >= 2


def test_real_feedback_worker_drains_available_frames_before_sleeping():
    sleeps = []

    class StopAfterSecondReadTransport(ChunkedFeedbackTransport):
        def read_available(self, max_frames: int) -> bytes:
            result = super().read_available(max_frames)
            if len(self.read_available_calls) >= 2:
                runtime.shutdown_event.set()
            return result

    transport = StopAfterSecondReadTransport(
        [
            [_feedback_frame(i) for i in range(1, 4)],
            [_feedback_frame(i) for i in range(4, 8)],
        ]
    )
    arm_runtime = runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", transport)
    hub = runtime.RealFeedbackHub()
    worker = runtime.RealFeedbackWorker(
        arm_runtime,
        hub,
        poll_sleep_s=0.001,
        join_timeout_s=0.5,
        sleep_fn=lambda seconds: sleeps.append(float(seconds)),
    )

    runtime.shutdown_event.clear()
    try:
        worker.start()
        snapshot = hub.wait_for_next(runtime.ACTIVE_ARM_MASK["left"], last_seq=0, timeout_s=1.0)
    finally:
        worker.stop()
        runtime.shutdown_event.clear()

    assert snapshot is not None
    assert len(transport.read_available_calls) >= 2
    assert sleeps == []


class FakeEnv:
    def __init__(self) -> None:
        self.model = object()
        self.data = object()
        self.q = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.qd = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.forward_calls = 0
        self.target_init_calls = 0
        self.target_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        self.target_quat = np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1))
        self.reference_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        self.reference_quat = np.zeros((Config.NUM_ARMS, 4), dtype=np.float64)
        self.ee_pos = np.array([[0.11, 0.22, 0.33], [0.44, 0.55, 0.66]], dtype=np.float64)
        self.ee_quat = np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1))

    def get_qpos(self):
        return self.q.copy()

    def get_qvel(self):
        return self.qd.copy()

    def set_qpos(self, q):
        self.q = np.asarray(q, dtype=np.float64).copy()

    def set_qvel(self, qd):
        self.qd = np.asarray(qd, dtype=np.float64).copy()

    def forward(self):
        self.forward_calls += 1

    def get_all_ee_pos(self):
        return self.ee_pos.copy()

    def get_all_ee_quat(self):
        return self.ee_quat.copy()

    def set_all_target_poses_base(self, pos, quat):
        self.target_init_calls += 1
        self.target_pos = np.asarray(pos, dtype=np.float64).copy()
        self.target_quat = np.asarray(quat, dtype=np.float64).copy()

    def set_all_reference_poses(self, pos, quat):
        self.reference_pos = np.asarray(pos, dtype=np.float64).copy()
        self.reference_quat = np.asarray(quat, dtype=np.float64).copy()

    def get_all_target_poses(self):
        return self.target_pos.copy(), self.target_quat.copy()

    def reset(self, qpos=None):
        if qpos is not None:
            self.set_qpos(qpos)
        self.forward()


class ClosingViewer:
    def __init__(self) -> None:
        self.sync_calls = 0
        self.closed = False

    def is_running(self) -> bool:
        return False

    def sync(self) -> None:
        self.sync_calls += 1

    def close(self) -> None:
        self.closed = True


def _feedback_frame(motor_id: int):
    return SimpleNamespace(
        motor_id=motor_id,
        position=0.1 * motor_id,
        velocity=0.01 * motor_id,
        torque=0.001 * motor_id,
    )


class FakeSocketTransport:
    def __init__(self, packets) -> None:
        self.packets = list(packets)
        self.timeouts = []
        self.closed = False

    def recv(self, timeout: float = 0.1):
        self.timeouts.append(float(timeout))
        if not self.packets:
            return None
        return self.packets.pop(0)

    def send(self, can_id: int, payload: bytes) -> None:
        _ = can_id, payload

    def close(self) -> None:
        self.closed = True


def test_usb2fdcan_read_available_uses_nonblocking_recv_timeout():
    socket_transport = FakeSocketTransport(
        [
            (0x01, bytes([0x10, 0x00, 0x00, 0x00, 0x80, 0x00, 0x80, 0x00])),
            None,
        ]
    )
    transport = Usb2FdcanTransport(
        Usb2FdcanConfig(read_timeout=0.123),
        socket_transport=socket_transport,
    )

    transport.read_available(8)

    assert socket_transport.timeouts == [0.0, 0.0]
    assert transport.stats.read_count == 1


def test_usb2fdcan_read_feedback_batch_drains_and_tracks_filter_stats():
    feedback_payload = bytes([0x10, 0x00, 0x00, 0x00, 0x80, 0x00, 0x80, 0x00])
    control_echo_payload = bytes([0x10, 0x00, 0x33, 0x00, 0x80, 0x00, 0x80, 0x00])
    socket_transport = FakeSocketTransport(
        [
            (0x01, feedback_payload),
            (0x7FE, feedback_payload),
            (0x02, control_echo_payload),
            (0x02, b"\x10\x00"),
            (0x12, feedback_payload),
            None,
        ]
    )
    transport = Usb2FdcanTransport(
        Usb2FdcanConfig(read_timeout=0.123),
        socket_transport=socket_transport,
    )

    samples = transport.read_feedback_batch(16, first_timeout_s=0.123, drain_timeout_s=0.0)

    assert [sample[0] for sample in samples] == [1, 2]
    assert [sample[1] for sample in samples] == [0x01, 0x12]
    assert socket_transport.timeouts == [0.123, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert transport.stats.read_count == 5
    assert transport.stats.batch_read_count == 1
    assert transport.stats.batch_feedback_count == 2
    assert transport.stats.feedback_count == 2
    assert transport.stats.unknown_feedback_id_count == 1
    assert transport.stats.ignored_control_echo_count == 1
    assert transport.stats.ignored_short_count == 1


def test_open_arm_runtimes_uses_can0_for_left_and_can1_for_right(monkeypatch):
    monkeypatch.setattr(runtime, "LEFT_CAN_INTERFACE", "can0")
    monkeypatch.setattr(runtime, "RIGHT_CAN_INTERFACE", "can1")

    created = []

    def factory(config):
        created.append(config)
        return FakeTransport(config)

    left = runtime.open_arm_runtimes("left", transport_factory=factory)
    right = runtime.open_arm_runtimes("right", transport_factory=factory)
    both = runtime.open_arm_runtimes("both", transport_factory=factory)

    assert [rt.interface for rt in left] == ["can0"]
    assert [rt.interface for rt in right] == ["can1"]
    assert [rt.interface for rt in both] == ["can0", "can1"]
    assert all(tuple(config.motor_ids) == tuple(range(1, 8)) for config in created)


def test_real_right_maps_can1_ids_1_to_7_to_global_right_arm_and_sends_ids_1_to_7():
    transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)
    bridge = StopAfterOneBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_control_loop(
            mode="right",
            bridge=bridge,
            runtimes=runtimes,
            env=None,
            viewer=None,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert len(bridge.calls) == 1
    call = bridge.calls[0]
    assert call["active_arm_mask"] == (1 << Config.RIGHT_ARM)
    np.testing.assert_allclose(call["q"][: Config.ARM_JOINTS], 0.0)
    np.testing.assert_allclose(call["q"][Config.ARM_JOINTS :], [0.1 * i for i in range(1, 8)])
    sent_torque_ids = [cmd[1] for cmd in transport.commands if cmd[0] == "torque" and cmd[2] not in (0.0, None)]
    assert sent_torque_ids == list(range(1, 8))
    assert 8 not in sent_torque_ids


def test_real_left_maps_can0_ids_1_to_7_to_global_left_arm_and_sends_ids_1_to_7():
    transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", transport),)
    bridge = StopAfterOneBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_control_loop(
            mode="left",
            bridge=bridge,
            runtimes=runtimes,
            env=None,
            viewer=None,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert len(bridge.calls) == 1
    call = bridge.calls[0]
    assert call["active_arm_mask"] == (1 << Config.LEFT_ARM)
    np.testing.assert_allclose(call["q"][: Config.ARM_JOINTS], [0.1 * i for i in range(1, 8)])
    np.testing.assert_allclose(call["q"][Config.ARM_JOINTS :], 0.0)
    sent_torque_ids = [cmd[1] for cmd in transport.commands if cmd[0] == "torque" and cmd[2] not in (0.0, None)]
    assert sent_torque_ids == list(range(1, 8))


def test_real_both_uses_independent_tx_workers_for_left_and_right_can():
    left_transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    right_transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (
        runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", left_transport),
        runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", right_transport),
    )
    bridge = StopAfterOneBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_control_loop(
            mode="both",
            bridge=bridge,
            runtimes=runtimes,
            env=None,
            viewer=None,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert len(bridge.calls) == 1
    assert bridge.calls[0]["active_arm_mask"] == runtime.ACTIVE_ARM_MASK["both"]
    left_torques = [cmd for cmd in left_transport.commands if cmd[0] == "torque" and cmd[2] not in (0.0, None)]
    right_torques = [cmd for cmd in right_transport.commands if cmd[0] == "torque" and cmd[2] not in (0.0, None)]
    assert [cmd[1] for cmd in left_torques] == list(range(1, 8))
    assert [cmd[2] for cmd in left_torques] == [float(i) for i in range(1, 8)]
    assert [cmd[1] for cmd in right_torques] == list(range(1, 8))
    assert [cmd[2] for cmd in right_torques] == [float(i) for i in range(8, 15)]


def test_real_feedback_can_accumulate_across_multiple_reads_before_control():
    transport = ChunkedFeedbackTransport(
        [
            [_feedback_frame(i) for i in range(1, 4)],
            [_feedback_frame(i) for i in range(4, 8)],
        ]
    )
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)
    bridge = StopAfterOneBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_control_loop(
            mode="right",
            bridge=bridge,
            runtimes=runtimes,
            env=None,
            viewer=None,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert len(bridge.calls) == 1
    np.testing.assert_allclose(bridge.calls[0]["q"][Config.ARM_JOINTS :], [0.1 * i for i in range(1, 8)])
    assert transport.read_available_calls


def test_real_feedback_complete_trigger_resets_mask_between_control_steps(monkeypatch):
    monkeypatch.setattr(runtime, "REAL_FEEDBACK_WAIT_FOR_CONSUME", True)
    first_cycle = [_feedback_frame(i) for i in range(1, 8)]
    second_cycle_partial = [_feedback_frame(i) for i in range(1, 4)]
    second_cycle_rest = [_feedback_frame(i) for i in range(4, 8)]
    for frame in second_cycle_partial + second_cycle_rest:
        frame.position += 1.0
        frame.velocity += 1.0
    transport = ChunkedFeedbackTransport([first_cycle, second_cycle_partial, second_cycle_rest])
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)
    bridge = StopAfterTwoBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_control_loop(
            mode="right",
            bridge=bridge,
            runtimes=runtimes,
            env=None,
            viewer=None,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert len(bridge.calls) == 2
    np.testing.assert_allclose(bridge.calls[0]["q"][Config.ARM_JOINTS :], [0.1 * i for i in range(1, 8)])
    np.testing.assert_allclose(bridge.calls[1]["q"][Config.ARM_JOINTS :], [1.0 + 0.1 * i for i in range(1, 8)])


def test_real_control_repeats_zero_requests_until_startup_feedback_is_complete(monkeypatch):
    times = iter([0.0, 0.0, 0.002, 0.002])
    monkeypatch.setattr(runtime.time, "perf_counter", lambda: next(times, 0.002))
    monkeypatch.setattr(runtime, "REAL_FEEDBACK_REQUEST_INTERVAL_S", 0.001)
    transport = StopAfterRepeatedFeedbackRequestsTransport(
        [
            [_feedback_frame(i) for i in range(2, 8)],
            [],
        ],
        stop_after_commands=Config.ARM_JOINTS * 2,
    )
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)
    bridge = StopAfterOneBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_control_loop(
            mode="right",
            bridge=bridge,
            runtimes=runtimes,
            env=None,
            viewer=None,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert bridge.calls == []
    zero_requests = [cmd for cmd in transport.commands if cmd[0] == "torque" and cmd[2] == 0.0]
    assert len(zero_requests) >= Config.ARM_JOINTS * 2


def test_real_control_does_not_zero_last_command_while_waiting_for_next_feedback_set(monkeypatch):
    monkeypatch.setattr(runtime, "REAL_FEEDBACK_WAIT_FOR_CONSUME", True)
    transport = ShutdownAfterFirstControlThenPartialFeedbackTransport(
        [
            [_feedback_frame(i) for i in range(1, 8)],
            [_feedback_frame(1)],
        ]
    )
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)
    bridge = StopAfterTwoBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_control_loop(
            mode="right",
            bridge=bridge,
            runtimes=runtimes,
            env=None,
            viewer=None,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert len(bridge.calls) == 1
    nonzero_torques = [cmd for cmd in transport.commands if cmd[0] == "torque" and cmd[2] not in (0.0, None)]
    assert len(nonzero_torques) == Config.ARM_JOINTS
    assert transport.zero_before_shutdown == 0
    assert transport.command_count_when_partial_feedback_read is not None


def test_feedback_timeout_budget_starts_after_startup_enable(monkeypatch):
    times = iter([0.0, 1.0, 1.01, 1.02, 1.03])
    monkeypatch.setattr(runtime.time, "perf_counter", lambda: next(times, 1.03))

    transport = SlowStartupTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)
    bridge = StopAfterOneBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="right",
            bridge=bridge,
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            startup_enable=True,
        )
    finally:
        runtime.shutdown_event.clear()

    assert transport.enable_calls == Config.ARM_JOINTS
    assert len(bridge.calls) == 1


def test_real_feedback_timeout_reports_missing_motors_from_hub_masks(monkeypatch, capsys):
    monkeypatch.setattr(runtime, "CAN_FEEDBACK_TIMEOUT_S", 0.003)
    monkeypatch.setattr(runtime, "REAL_FEEDBACK_REQUEST_INTERVAL_S", 0.0005)
    left_transport = ChunkedFeedbackTransport([[_feedback_frame(i) for i in range(1, 8)]])
    right_transport = ChunkedFeedbackTransport([[_feedback_frame(i) for i in range(1, 5)]])
    runtimes = (
        runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", left_transport),
        runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", right_transport),
    )
    bridge = StopAfterOneBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_control_loop(
            mode="both",
            bridge=bridge,
            runtimes=runtimes,
            env=None,
            viewer=None,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    captured = capsys.readouterr().out
    assert bridge.calls == []
    assert "feedback timeout" in captured
    assert "can1: motor id 5,6,7" in captured
    assert "can0: motor id" not in captured
    assert "[CAN Diagnostics]" in captured
    assert "partial_ids=1,2,3,4" in captured
    assert "id5 age=never" in captured
    assert "[CAN RX Stats]" in captured


def test_real_both_consumes_ready_snapshot_before_timeout_check(monkeypatch, capsys):
    monkeypatch.setattr(runtime, "CAN_FEEDBACK_TIMEOUT_S", 0.1)
    clock_values = iter([0.0, 0.2])
    monkeypatch.setattr(runtime, "_THREAD_WAIT_CLOCK", lambda: next(clock_values, 0.2))

    def publish_complete_both(runtimes_arg, hub):
        _ = runtimes_arg
        hub.record_feedback_batch(
            Config.LEFT_ARM,
            tuple(range(1, 8)),
            [_feedback_sample(i) for i in range(1, 8)],
        )
        hub.record_feedback_batch(
            Config.RIGHT_ARM,
            tuple(range(1, 8)),
            [_feedback_sample(i, offset=1.0) for i in range(1, 8)],
        )
        return ()

    monkeypatch.setattr(runtime, "_start_feedback_workers", publish_complete_both)
    left_transport = FakeTransport()
    right_transport = FakeTransport()
    runtimes = (
        runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", left_transport),
        runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", right_transport),
    )
    bridge = StopAfterOneBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="both",
            bridge=bridge,
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    captured = capsys.readouterr().out
    assert "feedback timeout" not in captured
    assert len(bridge.calls) == 1
    np.testing.assert_allclose(bridge.calls[0]["q"][: Config.ARM_JOINTS], [0.1 * i for i in range(1, 8)])
    np.testing.assert_allclose(
        bridge.calls[0]["q"][Config.ARM_JOINTS :],
        [1.0 + 0.1 * i for i in range(1, 8)],
    )


def test_real_external_shutdown_zeroes_and_disables_active_arm():
    transport = FakeTransport(frames=[])
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)

    runtime.shutdown_event.set()
    try:
        runtime.run_real_can_control_loop(
            mode="right",
            bridge=StopAfterOneBridge(),
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert ("torque", 1, 0.0) in transport.commands
    assert ("disable", 7, None) in transport.commands
    assert transport.closed is True


def test_real_safety_status_zeroes_and_disables_active_arm():
    class SafetyBridge(StopAfterOneBridge):
        def compute(self, *args, **kwargs):
            self.calls.append({})
            return SimpleNamespace(status=-1, tau=np.ones(Config.NUM_JOINTS))

    transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", transport),)

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_control_loop(
            mode="left",
            bridge=SafetyBridge(),
            runtimes=runtimes,
            env=None,
            viewer=None,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    nonzero_torques = [cmd for cmd in transport.commands if cmd[0] == "torque" and cmd[2] not in (0.0, None)]
    assert nonzero_torques == []
    assert ("torque", 1, 0.0) in transport.commands
    assert ("disable", 7, None) in transport.commands
    assert transport.closed is True


def test_real_safety_status_prints_active_arm_limit_details(capsys):
    class SafetyBridge(StopAfterOneBridge):
        def compute(self, *args, **kwargs):
            self.calls.append({})
            return SimpleNamespace(status=-1, tau=np.zeros(Config.NUM_JOINTS))

    frame_values = [_feedback_frame(i) for i in range(1, 8)]
    frame_values[3] = SimpleNamespace(
        motor_id=4,
        position=0.0,
        velocity=0.0,
        torque=0.0,
    )
    transport = FakeTransport(frames=frame_values)
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_control_loop(
            mode="right",
            bridge=SafetyBridge(),
            runtimes=runtimes,
            env=None,
            viewer=None,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    captured = capsys.readouterr().out
    assert "controller status=-1" in captured
    assert "right/J4" in captured
    assert "below_safe_min_by" in captured
    assert "Recent right/J7 diagnostic window" in captured
    assert "tau_raw" in captured
    assert "tau_sent" in captured


def test_mirror_real_feedback_updates_only_active_arm_and_initializes_targets():
    env = FakeEnv()
    shared = runtime.RealSharedState()
    q = np.arange(1.0, Config.NUM_JOINTS + 1.0)
    qd = q * 0.01
    shared.update_feedback(q, qd, np.zeros(Config.NUM_JOINTS), runtime.ACTIVE_ARM_MASK["left"])

    runtime.mirror_real_state_to_env(env, shared, runtime.ACTIVE_ARM_MASK["left"])

    np.testing.assert_allclose(env.q[: Config.ARM_JOINTS], q[: Config.ARM_JOINTS])
    np.testing.assert_allclose(env.q[Config.ARM_JOINTS :], 0.0)
    assert env.forward_calls == 1
    assert env.target_init_calls == 1
    target_pos, target_quat = shared.snapshot_targets()
    np.testing.assert_allclose(target_pos, env.ee_pos)
    np.testing.assert_allclose(target_quat, env.ee_quat)


def test_mirror_real_feedback_initializes_active_arm_target_to_sim_initial_tcp_pose():
    env = FakeEnv()
    shared = runtime.RealSharedState()
    q = np.arange(1.0, Config.NUM_JOINTS + 1.0)
    qd = q * 0.01
    initial_target_pos = np.array([[1.1, 1.2, 1.3], [2.1, 2.2, 2.3]], dtype=np.float64)
    initial_target_quat = np.array(
        [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]],
        dtype=np.float64,
    )
    shared.update_feedback(q, qd, np.zeros(Config.NUM_JOINTS), runtime.ACTIVE_ARM_MASK["right"])

    runtime.mirror_real_state_to_env(
        env,
        shared,
        runtime.ACTIVE_ARM_MASK["right"],
        initial_target_pos,
        initial_target_quat,
    )

    target_pos, target_quat = shared.snapshot_targets()
    np.testing.assert_allclose(target_pos[Config.LEFT_ARM], env.ee_pos[Config.LEFT_ARM])
    np.testing.assert_allclose(target_pos[Config.RIGHT_ARM], initial_target_pos[Config.RIGHT_ARM])
    np.testing.assert_allclose(target_quat[Config.LEFT_ARM], env.ee_quat[Config.LEFT_ARM])
    np.testing.assert_allclose(target_quat[Config.RIGHT_ARM], initial_target_quat[Config.RIGHT_ARM])


def test_mirror_real_feedback_logs_real_start_target_summary(capsys):
    env = FakeEnv()
    shared = runtime.RealSharedState()
    initial_target_pos = np.array([[1.1, 1.2, 1.3], [2.1, 2.2, 2.3]], dtype=np.float64)
    shared.update_feedback(
        np.zeros(Config.NUM_JOINTS, dtype=np.float64),
        np.zeros(Config.NUM_JOINTS, dtype=np.float64),
        np.zeros(Config.NUM_JOINTS, dtype=np.float64),
        runtime.ACTIVE_ARM_MASK["right"],
    )

    runtime.mirror_real_state_to_env(
        env,
        shared,
        runtime.ACTIVE_ARM_MASK["right"],
        initial_target_pos,
        np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1)),
    )

    captured = capsys.readouterr().out
    assert "[Real Target] right" in captured
    assert "target_pos=INIT_QPOS_TCP" in captured
    assert "target_quat=INIT_QPOS_TCP" in captured
    assert "distance=" in captured


def test_mirror_real_feedback_initializes_both_targets_to_sim_initial_tcp_pose():
    env = FakeEnv()
    shared = runtime.RealSharedState()
    initial_target_pos = np.array([[1.1, 1.2, 1.3], [2.1, 2.2, 2.3]], dtype=np.float64)
    initial_target_quat = np.array(
        [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]],
        dtype=np.float64,
    )
    q = np.linspace(0.1, 1.4, Config.NUM_JOINTS)
    qd = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    shared.update_feedback(q, qd, np.zeros(Config.NUM_JOINTS), runtime.ACTIVE_ARM_MASK["both"])

    runtime.mirror_real_state_to_env(
        env,
        shared,
        runtime.ACTIVE_ARM_MASK["both"],
        initial_target_pos,
        initial_target_quat,
    )

    target_pos, target_quat = shared.snapshot_targets()
    np.testing.assert_allclose(target_pos, initial_target_pos)
    np.testing.assert_allclose(target_quat, initial_target_quat)


def test_mirror_real_feedback_updates_reference_marker_from_shared_output():
    env = FakeEnv()
    shared = runtime.RealSharedState()
    ref_pos = np.array([[0.11, 0.22, 0.33], [0.0, 0.0, 0.0]], dtype=np.float64)
    ref_quat = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]], dtype=np.float64)
    shared.update_feedback(
        np.zeros(Config.NUM_JOINTS, dtype=np.float64),
        np.zeros(Config.NUM_JOINTS, dtype=np.float64),
        np.zeros(Config.NUM_JOINTS, dtype=np.float64),
        runtime.ACTIVE_ARM_MASK["left"],
    )
    shared.update_control_output(
        np.zeros(Config.NUM_JOINTS, dtype=np.float64),
        np.zeros((Config.NUM_ARMS, 3), dtype=np.float64),
        np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1)),
        ref_pos=ref_pos,
        ref_quat=ref_quat,
    )

    runtime.mirror_real_state_to_env(env, shared, runtime.ACTIVE_ARM_MASK["left"])

    np.testing.assert_allclose(env.reference_pos, ref_pos)
    np.testing.assert_allclose(env.reference_quat, ref_quat)


def test_real_control_step_logs_full_rerun_payload(monkeypatch):
    times = iter([0.0, 0.1, 0.1, 0.1])
    monkeypatch.setattr(runtime.time, "perf_counter", lambda: next(times, 0.1))

    transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", transport),)
    bridge = StopAfterOneBridge()
    logger = FakeRerunLogger()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="left",
            bridge=bridge,
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            rerun_logger=logger,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert len(logger.payloads) == 1
    payload = logger.payloads[0]
    assert set(
        [
            "q",
            "qd",
            "tau_raw",
            "tau_total",
            "tau_actual",
            "ee_twist",
            "pos_actual",
            "pos_desired",
            "pos_reference",
            "quat_actual",
            "quat_desired",
            "quat_reference",
            "elapsed_s",
            "right_j7_diag",
            "c_bridge_ms",
            "feedback_wait_ms",
            "tx_overwrite_count",
            "can_backpressure_count",
            "control_target_hz",
        ]
    ).issubset(payload)
    np.testing.assert_allclose(payload["tau_raw"], np.arange(1.0, Config.NUM_JOINTS + 1.0))
    np.testing.assert_allclose(payload["tau_total"][: Config.ARM_JOINTS], np.arange(1.0, Config.ARM_JOINTS + 1.0))
    np.testing.assert_allclose(payload["tau_total"][Config.ARM_JOINTS :], 0.0)
    np.testing.assert_allclose(payload["pos_reference"][0], [0.11, 0.21, 0.31])
    np.testing.assert_allclose(payload["quat_reference"][0], [1.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(payload["pos_reference"][1], 0.0)
    np.testing.assert_allclose(payload["quat_reference"][1], 0.0)
    np.testing.assert_allclose(payload["ee_twist"][0], [0.01, 0.02, 0.03, 0.1, 0.2, 0.3])
    np.testing.assert_allclose(payload["ee_twist"][1], 0.0)
    np.testing.assert_allclose(payload["tau_actual"][: Config.ARM_JOINTS], [0.001 * i for i in range(1, 8)])
    assert payload["elapsed_s"] == pytest.approx(0.1)
    assert payload["right_j7_diag"]["joint"] == "right/J7"
    assert payload["control_target_hz"] == pytest.approx(1000.0)
    assert payload["c_bridge_ms"] >= 0.0
    assert payload["feedback_wait_ms"] >= 0.0


def test_real_no_send_computes_and_logs_but_sends_zero_torque(monkeypatch):
    times = iter([0.0, 0.1, 0.1, 0.1])
    monkeypatch.setattr(runtime.time, "perf_counter", lambda: next(times, 0.1))

    transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)
    bridge = StopAfterOneBridge()
    logger = FakeRerunLogger()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="right",
            bridge=bridge,
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            rerun_logger=logger,
            startup_enable=False,
            send_control=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert len(bridge.calls) == 1
    torque_values = [cmd[2] for cmd in transport.commands if cmd[0] == "torque"]
    assert torque_values
    assert all(value == 0.0 for value in torque_values)
    assert len(logger.payloads) == 1
    payload = logger.payloads[0]
    expected_raw = np.arange(1.0, Config.NUM_JOINTS + 1.0)
    expected_display = expected_raw.copy()
    expected_display[: Config.ARM_JOINTS] = 0.0
    np.testing.assert_allclose(payload["tau_raw"], expected_raw)
    np.testing.assert_allclose(payload["tau_total"], expected_display)
    assert payload["tx_label"] == "Zero MIT torque (control no-send)"
    assert payload["right_j7_diag"]["tau_cmd_raw"] == pytest.approx(14.0)
    assert payload["right_j7_diag"]["tau_cmd_sent"] == pytest.approx(0.0)


def test_real_gravity_only_sends_gravity_but_logs_full_control(monkeypatch):
    times = iter([0.0, 0.1, 0.1, 0.1])
    monkeypatch.setattr(runtime.time, "perf_counter", lambda: next(times, 0.1))

    transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)
    bridge = StopAfterOneBridge()
    logger = FakeRerunLogger()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="right",
            bridge=bridge,
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            rerun_logger=logger,
            startup_enable=False,
            send_mode="gravity",
        )
    finally:
        runtime.shutdown_event.clear()

    sent = [cmd[2] for cmd in transport.commands if cmd[0] == "torque" and cmd[2] not in (0.0, None)]
    assert sent == [float(value) for value in range(108, 115)]
    assert len(logger.payloads) == 1
    payload = logger.payloads[0]
    expected_raw = np.arange(1.0, Config.NUM_JOINTS + 1.0)
    expected_display = expected_raw.copy()
    expected_display[: Config.ARM_JOINTS] = 0.0
    np.testing.assert_allclose(payload["tau_raw"], expected_raw)
    np.testing.assert_allclose(payload["tau_total"], expected_display)
    assert payload["tx_label"] == "Gravity compensation MIT torque"
    assert payload["right_j7_diag"]["tau_cmd_raw"] == pytest.approx(14.0)
    assert payload["right_j7_diag"]["tau_cmd_sent"] == pytest.approx(114.0)


def test_real_gc_only_sends_gc_but_logs_full_control(monkeypatch):
    times = iter([0.0, 0.1, 0.1, 0.1])
    monkeypatch.setattr(runtime.time, "perf_counter", lambda: next(times, 0.1))

    transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)
    bridge = StopAfterOneBridge()
    logger = FakeRerunLogger()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="right",
            bridge=bridge,
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            rerun_logger=logger,
            startup_enable=False,
            send_mode="gc",
        )
    finally:
        runtime.shutdown_event.clear()

    sent = [cmd[2] for cmd in transport.commands if cmd[0] == "torque" and cmd[2] not in (0.0, None)]
    assert sent == [float(value) for value in range(208, 215)]
    assert len(logger.payloads) == 1
    payload = logger.payloads[0]
    expected_raw = np.arange(1.0, Config.NUM_JOINTS + 1.0)
    expected_display = expected_raw.copy()
    expected_display[: Config.ARM_JOINTS] = 0.0
    np.testing.assert_allclose(payload["tau_raw"], expected_raw)
    np.testing.assert_allclose(payload["tau_total"], expected_display)
    assert payload["tx_label"] == "G+C compensation MIT torque"
    assert payload["right_j7_diag"]["tau_cmd_raw"] == pytest.approx(14.0)
    assert payload["right_j7_diag"]["tau_cmd_sent"] == pytest.approx(214.0)


def test_real_bridge_timeout_zeroes_and_disables_active_arm(capsys):
    transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", transport),)
    bridge = TimeoutBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="left",
            bridge=bridge,
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    captured = capsys.readouterr().out
    nonzero_torques = [cmd for cmd in transport.commands if cmd[0] == "torque" and cmd[2] not in (0.0, None)]
    assert bridge.calls == 1
    assert nonzero_torques == []
    assert ("torque", 1, 0.0) in transport.commands
    assert ("disable", 7, None) in transport.commands
    assert "real controller bridge failed" in captured


def test_real_feedback_only_mirrors_feedback_without_control_bridge(monkeypatch):
    times = iter([0.0, 0.1, 0.1, 0.1])
    monkeypatch.setattr(runtime.time, "perf_counter", lambda: next(times, 0.1))

    transport = StopOnZeroTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)
    bridge = StopAfterOneBridge()
    logger = FakeRerunLogger()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="right",
            bridge=bridge,
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            rerun_logger=logger,
            startup_enable=False,
            feedback_only=True,
        )
    finally:
        runtime.shutdown_event.clear()

    assert bridge.calls == []
    assert ("torque", 1, 0.0) in transport.commands
    assert len(logger.payloads) == 1
    np.testing.assert_allclose(logger.payloads[0]["tau_raw"], 0.0)
    np.testing.assert_allclose(logger.payloads[0]["tau_total"], 0.0)
    np.testing.assert_allclose(logger.payloads[0]["ee_twist"], 0.0)
    assert logger.payloads[0]["right_j7_diag"]["qd"] == pytest.approx(0.07)


def test_real_rerun_logging_is_throttled_before_enqueue(monkeypatch):
    times = iter([0.0, 0.1, 0.1, 0.2, 0.2])
    monkeypatch.setattr(runtime.time, "perf_counter", lambda: next(times, 0.2))
    monkeypatch.setattr(runtime.Config, "RERUN_LOG_STRIDE", 2)

    first_cycle = [_feedback_frame(i) for i in range(1, 8)]
    second_cycle = [_feedback_frame(i) for i in range(1, 8)]
    for frame in second_cycle:
        frame.position += 1.0
    transport = ChunkedFeedbackTransport([first_cycle, second_cycle])
    runtimes = (runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", transport),)
    logger = FakeRerunLogger()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="left",
            bridge=StopAfterTwoBridge(),
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            rerun_logger=logger,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert [payload["step_count"] for payload in logger.payloads] == [0]


def test_real_rerun_tcp_speed_falls_back_to_position_delta_when_controller_twist_is_zero(monkeypatch):
    monkeypatch.setattr(runtime, "REAL_FEEDBACK_WAIT_FOR_CONSUME", True)
    times = iter([0.0, 0.1, 0.1, 0.2, 0.2])
    monkeypatch.setattr(runtime.time, "perf_counter", lambda: next(times, 0.2))
    monkeypatch.setattr(runtime.Config, "RERUN_LOG_STRIDE", 1)

    first_cycle = [_feedback_frame(i) for i in range(1, 8)]
    second_cycle = [_feedback_frame(i) for i in range(1, 8)]
    transport = ChunkedFeedbackTransport([first_cycle, second_cycle])
    runtimes = (runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", transport),)
    logger = FakeRerunLogger()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="left",
            bridge=MovingTcpZeroTwistBridge(),
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            rerun_logger=logger,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    assert len(logger.payloads) == 2
    np.testing.assert_allclose(logger.payloads[0]["ee_twist"], 0.0)
    np.testing.assert_allclose(logger.payloads[1]["ee_twist"][Config.LEFT_ARM, :3], [0.1, 0.0, 0.0])
    np.testing.assert_allclose(logger.payloads[1]["ee_twist"][Config.RIGHT_ARM], 0.0)


def test_real_control_waits_for_mirror_target_initialization_before_compute():
    transport = StopOnZeroTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", transport),)
    bridge = StopAfterOneBridge()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="left",
            bridge=bridge,
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            startup_enable=False,
            require_targets=True,
        )
    finally:
        runtime.shutdown_event.clear()

    assert bridge.calls == []
    assert ("torque", 1, 0.0) in transport.commands


def test_real_control_passes_fixed_zero_body_q_and_shared_targets_to_c_bridge():
    transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.LEFT_ARM, "can0", transport),)
    bridge = StopAfterOneBridge()
    shared = runtime.RealSharedState()
    target_pos = np.array(
        [[0.12, -0.04, 0.30], [0.21, 0.08, 0.35]],
        dtype=np.float64,
    )
    target_quat = np.array(
        [[0.5, -0.5, 0.5, -0.5], [0.5, 0.5, -0.5, 0.5]],
        dtype=np.float64,
    )
    shared.set_targets(target_pos, target_quat)

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="left",
            bridge=bridge,
            runtimes=runtimes,
            shared_state=shared,
            startup_enable=False,
            require_targets=True,
        )
    finally:
        runtime.shutdown_event.clear()

    assert len(bridge.calls) == 1
    call = bridge.calls[0]
    np.testing.assert_allclose(call["body_q"], runtime.REAL_C_BODY_Q_ZERO)
    np.testing.assert_allclose(call["target_pos"], target_pos)
    np.testing.assert_allclose(call["target_quat"], target_quat)


def test_viewer_closed_sets_shutdown_before_mirror_loop_runs():
    viewer = ClosingViewer()
    env = FakeEnv()
    shared = runtime.RealSharedState()

    runtime.shutdown_event.clear()
    try:
        # This mirrors run_real_control's viewer branch: closed viewer means stop
        # before another target update/control interaction.
        if viewer is not None and not viewer.is_running():
            runtime.shutdown_event.set()
        else:
            runtime.mirror_real_state_to_env(env, shared, runtime.ACTIVE_ARM_MASK["both"])
    finally:
        runtime.shutdown_event.clear()

    assert env.forward_calls == 0


def test_run_real_control_with_bridge_factory_starts_mujoco_before_bridge(monkeypatch):
    events = []

    class StopBeforeCanBridge:
        def close(self):
            events.append("bridge_close")

    class OneShotViewer(ClosingViewer):
        def is_running(self) -> bool:
            return False

    def fake_env_factory():
        events.append("mujoco_env")
        return FakeEnv()

    def fake_bridge_factory():
        events.append("bridge_factory")
        return StopBeforeCanBridge()

    monkeypatch.setattr(runtime, "MujocoSimEnv", fake_env_factory)
    monkeypatch.setattr(runtime, "VIEWER_AVAILABLE", True)
    monkeypatch.setattr(runtime.Config, "ENABLE_VIEWER", True)
    monkeypatch.setattr(runtime.Config, "ENABLE_RERUN", False)
    monkeypatch.setattr(runtime, "launch_passive_viewer", lambda model, data: OneShotViewer())
    monkeypatch.setattr(runtime, "open_arm_runtimes", lambda mode: ())

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_control_with_bridge(
            "right",
            bridge_factory=fake_bridge_factory,
            control_title="test",
        )
    finally:
        runtime.shutdown_event.clear()

    assert events[:3] == ["mujoco_env", "mujoco_env", "bridge_factory"]
    assert "bridge_close" in events
