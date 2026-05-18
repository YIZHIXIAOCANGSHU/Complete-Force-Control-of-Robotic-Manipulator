from __future__ import annotations

from types import SimpleNamespace

from robot_control.shared.state import SharedRobotState
from robot_control.modes.control_real import can_loop as real_can_control


def _fake_control_output(status: int = 0):
    return SimpleNamespace(
        tau_total=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
        q_ref=[0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77],
        qd_ref=[0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07],
        kp=[10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0],
        kd=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        tau_ff=[1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
        ee_pos=[0.4, 0.5, 0.6],
        ee_quat=[1.0, 0.0, 0.0, 0.0],
        status=status,
        calc_time_ms=0.75,
    )


class FakeCanTransport:
    def __init__(self, frames=None) -> None:
        self.frames = list(frames or [])
        self.commands: list[tuple] = []
        self.closed = False

    def read(self, size: int) -> bytes:
        _ = size
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

    def send_mit_command(
        self,
        motor_id: int,
        *,
        position: float,
        velocity: float,
        kp: float,
        kd: float,
        torque: float,
    ) -> bytes:
        self.commands.append(
            ("mit", int(motor_id), float(position), float(velocity), float(kp), float(kd), float(torque))
        )
        return b"mit"

    def disable_motor(self, motor_id: int) -> bytes:
        self.commands.append(("disable", int(motor_id), None))
        return b"disable"

    def close(self) -> None:
        self.closed = True


class DelayedFeedbackTransport(FakeCanTransport):
    def __init__(self, delayed_frames) -> None:
        super().__init__([])
        self.delayed_frames = list(delayed_frames)
        self.read_count = 0

    def read(self, size: int) -> bytes:
        _ = size
        self.read_count += 1
        if self.read_count == 2:
            self.frames.extend(self.delayed_frames)
        return b""


class FakeCompTool:
    def __init__(self, status: int = 0) -> None:
        self.status = status
        self.compute_calls: list[tuple[list[float], list[float], list[float], list[float]]] = []

    def compute(self, q, qd, target_pos, target_quat):
        self.compute_calls.append((list(q), list(qd), list(target_pos), list(target_quat)))
        return _fake_control_output(status=self.status)


class StopAfterLogRerunLogger:
    def __init__(self) -> None:
        self.payloads: list[dict[str, object]] = []

    def log_step(self, **payload) -> None:
        self.payloads.append(payload)
        real_can_control.shutdown_event.set()


def _feedback_frame(motor_id: int):
    return SimpleNamespace(
        motor_id=motor_id,
        state=1,
        position=0.1 * motor_id,
        velocity=0.01 * motor_id,
        torque=0.001 * motor_id,
        mos_temperature=40.0 + motor_id,
        rotor_temperature=50.0 + motor_id,
    )


def test_can_thread_computes_and_sends_mit_torque_after_complete_feedback(monkeypatch):
    monkeypatch.setattr(real_can_control.Config, "ENABLE_RERUN", True)
    monkeypatch.setattr(real_can_control.Config, "RERUN_LOG_STRIDE", 1)
    transport = FakeCanTransport(_feedback_frame(i + 1) for i in range(real_can_control.Config.NUM_JOINTS))
    comp_tool = FakeCompTool(status=0)
    shared_state = SharedRobotState()
    shared_state.set_target_pose([0.11, 0.22, 0.33], [1.0, 0.0, 0.0, 0.0])
    rerun_logger = StopAfterLogRerunLogger()

    real_can_control.shutdown_event.clear()
    try:
        real_can_control.can_thread_func(
            transport,
            comp_tool,
            shared_state,
            rerun_logger,
            startup_enable=False,
            feedback_timeout_s=1.0,
            control_period_s=0.0,
        )
    finally:
        real_can_control.shutdown_event.clear()

    assert len(comp_tool.compute_calls) == 1
    q, qd, target_pos, target_quat = comp_tool.compute_calls[0]
    assert q == [0.1, 0.2, 0.30000000000000004, 0.4, 0.5, 0.6000000000000001, 0.7000000000000001]
    assert qd == [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]
    assert target_pos == [0.11, 0.22, 0.33]
    assert target_quat == [1.0, 0.0, 0.0, 0.0]
    assert ("mit", 1, 0.11, 0.01, 10.0, 0.1, 1.1) in transport.commands
    assert ("mit", 7, 0.77, 0.07, 70.0, 0.7, 1.7) in transport.commands
    assert len(rerun_logger.payloads) == 1
    assert rerun_logger.payloads[0]["tau_total"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    assert rerun_logger.payloads[0]["q_target"] == [0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77]
    assert transport.closed is True


def test_can_thread_sends_zero_keepalive_while_waiting_for_first_feedback(monkeypatch):
    monkeypatch.setattr(real_can_control.Config, "ENABLE_RERUN", True)
    monkeypatch.setattr(real_can_control.Config, "RERUN_LOG_STRIDE", 1)
    transport = DelayedFeedbackTransport(
        _feedback_frame(i + 1) for i in range(real_can_control.Config.NUM_JOINTS)
    )
    comp_tool = FakeCompTool(status=0)
    rerun_logger = StopAfterLogRerunLogger()

    real_can_control.shutdown_event.clear()
    try:
        real_can_control.can_thread_func(
            transport,
            comp_tool,
            SharedRobotState(),
            rerun_logger,
            startup_enable=False,
            feedback_timeout_s=1.0,
            control_period_s=0.0,
        )
    finally:
        real_can_control.shutdown_event.clear()

    assert len(comp_tool.compute_calls) == 1
    assert transport.read_count >= 2
    assert transport.commands[: real_can_control.Config.NUM_JOINTS] == [
        ("mit", motor_id, 0.0, 0.0, 0.0, 0.0, 0.0)
        for motor_id in range(1, real_can_control.Config.NUM_JOINTS + 1)
    ]


def test_missing_feedback_ids_reports_unseen_motors():
    feedback_mask = (1 << 0) | (1 << 3) | (1 << 6)

    assert real_can_control._missing_feedback_ids(feedback_mask) == (2, 3, 5, 6)


def test_can_thread_zeroes_and_disables_when_stm_reports_safety_error():
    transport = FakeCanTransport(_feedback_frame(i + 1) for i in range(real_can_control.Config.NUM_JOINTS))
    comp_tool = FakeCompTool(status=-1)

    real_can_control.shutdown_event.clear()
    try:
        real_can_control.can_thread_func(
            transport,
            comp_tool,
            SharedRobotState(),
            None,
            startup_enable=False,
            feedback_timeout_s=1.0,
            control_period_s=0.0,
        )
    finally:
        real_can_control.shutdown_event.clear()

    nonzero_torques = [cmd for cmd in transport.commands if cmd[0] == "mit" and cmd[-1] not in (0.0, None)]
    assert nonzero_torques == []
    assert ("mit", 1, 0.0, 0.0, 0.0, 0.0, 0.0) in transport.commands
    assert ("disable", 7, None) in transport.commands
    assert transport.closed is True


def test_can_thread_stops_on_feedback_timeout_without_computing():
    transport = FakeCanTransport([])
    comp_tool = FakeCompTool(status=0)

    real_can_control.shutdown_event.clear()
    try:
        real_can_control.can_thread_func(
            transport,
            comp_tool,
            SharedRobotState(),
            None,
            startup_enable=False,
            feedback_timeout_s=0.001,
            control_period_s=0.0,
        )
    finally:
        real_can_control.shutdown_event.clear()

    assert comp_tool.compute_calls == []
    assert ("mit", 1, 0.0, 0.0, 0.0, 0.0, 0.0) in transport.commands
    assert ("disable", 7, None) in transport.commands
    assert transport.closed is True


def test_can_thread_stops_when_feedback_round_never_completes():
    transport = FakeCanTransport(_feedback_frame(i + 1) for i in range(real_can_control.Config.NUM_JOINTS - 1))
    comp_tool = FakeCompTool(status=0)

    real_can_control.shutdown_event.clear()
    try:
        real_can_control.can_thread_func(
            transport,
            comp_tool,
            SharedRobotState(),
            None,
            startup_enable=False,
            feedback_timeout_s=0.001,
            control_period_s=0.0,
        )
    finally:
        real_can_control.shutdown_event.clear()

    assert comp_tool.compute_calls == []
    nonzero_torques = [cmd for cmd in transport.commands if cmd[0] == "mit" and cmd[-1] not in (0.0, None)]
    assert nonzero_torques == []
    assert ("mit", 1, 0.0, 0.0, 0.0, 0.0, 0.0) in transport.commands
    assert ("disable", 7, None) in transport.commands
