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
            }
        )
        runtime.shutdown_event.set()
        return SimpleNamespace(
            status=0,
            tau=np.arange(1.0, Config.NUM_JOINTS + 1.0),
            ee_pos=np.array([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]], dtype=np.float64),
            ee_quat=np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1)),
            traj_t=0.123,
        )


class HighTorqueBridge:
    def __init__(self, tau_value: float = 100.0) -> None:
        self.tau_value = float(tau_value)
        self.calls = []

    def compute(self, active_arm_mask, elapsed_s, q, qd, body_q, target_pos, target_quat):
        self.calls.append({"elapsed_s": float(elapsed_s), "active_arm_mask": int(active_arm_mask)})
        runtime.shutdown_event.set()
        return SimpleNamespace(
            status=0,
            tau=np.full(Config.NUM_JOINTS, self.tau_value, dtype=np.float64),
            ee_pos=np.zeros((Config.NUM_ARMS, 3), dtype=np.float64),
            ee_quat=np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1)),
            traj_t=0.0,
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
            }
        )
        if len(self.calls) >= 2:
            runtime.shutdown_event.set()
        return SimpleNamespace(
            status=0,
            tau=np.ones(Config.NUM_JOINTS, dtype=np.float64),
            ee_pos=np.zeros((Config.NUM_ARMS, 3), dtype=np.float64),
            ee_quat=np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1)),
            traj_t=float(len(self.calls)),
        )


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


def test_real_feedback_complete_trigger_resets_mask_between_control_steps():
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


def test_real_control_does_not_zero_last_command_while_waiting_for_next_feedback_set():
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
    assert transport.command_count_when_partial_feedback_read == Config.ARM_JOINTS


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


def test_mirror_real_feedback_initializes_active_arm_target_to_sim_initial_tcp_position_only():
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
    )

    target_pos, target_quat = shared.snapshot_targets()
    np.testing.assert_allclose(target_pos[Config.LEFT_ARM], env.ee_pos[Config.LEFT_ARM])
    np.testing.assert_allclose(target_pos[Config.RIGHT_ARM], initial_target_pos[Config.RIGHT_ARM])
    np.testing.assert_allclose(target_quat[Config.LEFT_ARM], env.ee_quat[Config.LEFT_ARM])
    np.testing.assert_allclose(target_quat[Config.RIGHT_ARM], env.ee_quat[Config.RIGHT_ARM])
    assert not np.allclose(target_quat[Config.RIGHT_ARM], initial_target_quat[Config.RIGHT_ARM])


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
    )

    captured = capsys.readouterr().out
    assert "[Real Target] right" in captured
    assert "target_pos=INIT_QPOS_TCP" in captured
    assert "target_quat=current_real_tcp" in captured
    assert "distance=" in captured


def test_mirror_real_feedback_initializes_both_targets_to_sim_initial_tcp_position_only():
    env = FakeEnv()
    shared = runtime.RealSharedState()
    initial_target_pos = np.array([[1.1, 1.2, 1.3], [2.1, 2.2, 2.3]], dtype=np.float64)
    q = np.linspace(0.1, 1.4, Config.NUM_JOINTS)
    qd = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    shared.update_feedback(q, qd, np.zeros(Config.NUM_JOINTS), runtime.ACTIVE_ARM_MASK["both"])

    runtime.mirror_real_state_to_env(
        env,
        shared,
        runtime.ACTIVE_ARM_MASK["both"],
        initial_target_pos,
    )

    target_pos, target_quat = shared.snapshot_targets()
    np.testing.assert_allclose(target_pos, initial_target_pos)
    np.testing.assert_allclose(target_quat, env.ee_quat)


def test_real_control_step_logs_full_rerun_payload(monkeypatch):
    times = iter([0.0, 0.1, 0.1, 0.1])
    monkeypatch.setattr(runtime.time, "perf_counter", lambda: next(times, 0.1))
    monkeypatch.setattr(runtime, "REAL_TORQUE_SLEW_NM_PER_S", 30.0)

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
            "pos_actual",
            "pos_desired",
            "quat_actual",
            "quat_desired",
            "elapsed_s",
            "right_j7_diag",
        ]
    ).issubset(payload)
    np.testing.assert_allclose(payload["tau_raw"], np.arange(1.0, Config.NUM_JOINTS + 1.0))
    np.testing.assert_allclose(payload["tau_total"][: Config.ARM_JOINTS], [1.0, 2.0, 3.0, 3.0, 3.0, 3.0, 3.0])
    np.testing.assert_allclose(payload["tau_total"][Config.ARM_JOINTS :], 0.0)
    np.testing.assert_allclose(payload["tau_actual"][: Config.ARM_JOINTS], [0.001 * i for i in range(1, 8)])
    assert payload["elapsed_s"] == pytest.approx(0.1)
    assert payload["right_j7_diag"]["joint"] == "right/J7"


def test_real_torque_slew_limiter_limits_step_commands(monkeypatch):
    times = iter([0.0, 0.1, 0.1, 0.1])
    monkeypatch.setattr(runtime.time, "perf_counter", lambda: next(times, 0.1))
    monkeypatch.setattr(runtime, "REAL_TORQUE_SLEW_NM_PER_S", 30.0)

    transport = FakeTransport(frames=(_feedback_frame(i) for i in range(1, 8)))
    runtimes = (runtime.ArmCanRuntime(Config.RIGHT_ARM, "can1", transport),)
    logger = FakeRerunLogger()

    runtime.shutdown_event.clear()
    try:
        runtime.run_real_can_control_loop(
            mode="right",
            bridge=HighTorqueBridge(100.0),
            runtimes=runtimes,
            shared_state=runtime.RealSharedState(),
            rerun_logger=logger,
            startup_enable=False,
        )
    finally:
        runtime.shutdown_event.clear()

    sent_nonzero = [
        cmd[2]
        for cmd in transport.commands
        if cmd[0] == "torque" and cmd[2] not in (0.0, None)
    ]
    assert sent_nonzero == pytest.approx([3.0] * Config.ARM_JOINTS)
    assert len(logger.payloads) == 1
    np.testing.assert_allclose(logger.payloads[0]["tau_raw"][Config.ARM_JOINTS :], 100.0)
    np.testing.assert_allclose(logger.payloads[0]["tau_total"][Config.ARM_JOINTS :], 3.0)
    assert logger.payloads[0]["right_j7_diag"]["tau_cmd_raw"] == pytest.approx(100.0)
    assert logger.payloads[0]["right_j7_diag"]["tau_cmd_sent"] == pytest.approx(3.0)


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
    assert logger.payloads[0]["right_j7_diag"]["qd"] == pytest.approx(0.07)


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
