from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from robot_control.config import Config
from robot_control.modes.control_real import can_feedback
from robot_control.modes.param_id_real import main as param_id_real


class FakeCanTransport:
    def __init__(self, frames=None) -> None:
        self.frames = list(frames or [])
        self.commands: list[tuple] = []
        self.closed = False
        self.read_count = 0

    def read(self, size: int) -> bytes:
        _ = size
        self.read_count += 1
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

    def read(self, size: int) -> bytes:
        super().read(size)
        if self.read_count == 2:
            self.frames.extend(self.delayed_frames)
        return b""


class FakeBackend:
    def __init__(
        self,
        torque=None,
        kp=None,
        kd=None,
        safety_status: int = 0,
        q_ref_safe: bool = True,
        joint_safety_status: int = 0,
    ) -> None:
        self._joint_kp = np.ones(Config.NUM_JOINTS, dtype=np.float64) if kp is None else np.asarray(kp, dtype=np.float64)
        self._joint_kd = np.zeros(Config.NUM_JOINTS, dtype=np.float64) if kd is None else np.asarray(kd, dtype=np.float64)
        self.torque = np.zeros(Config.NUM_JOINTS, dtype=np.float64) if torque is None else np.asarray(torque, dtype=np.float64)
        self.safety_status = int(safety_status)
        self.q_ref_safe = bool(q_ref_safe)
        self.joint_safety_status = int(joint_safety_status)

    def compute_nonlinear_effects(self, q, qd):
        _ = q, qd
        return self.torque.copy()

    def _check_safety(self, q, qd):
        _ = q, qd
        return self.safety_status

    def _q_ref_is_safe(self, q_ref):
        _ = q_ref
        return self.q_ref_safe

    def _check_joint_safety(self, q, qd, tau):
        _ = q, qd, tau
        return self.joint_safety_status


def _feedback_frame(motor_id: int, *, q_offset: float = 0.0, torque_scale: float = 0.01):
    return SimpleNamespace(
        motor_id=motor_id,
        state=1,
        position=q_offset + 0.001 * motor_id,
        velocity=0.01 * motor_id,
        torque=torque_scale * motor_id,
        mos_temperature=40.0 + motor_id,
        rotor_temperature=50.0 + motor_id,
    )


def _feedback_round(*, q_offset: float = 0.0, torque_scale: float = 0.01):
    return [
        _feedback_frame(i + 1, q_offset=q_offset, torque_scale=torque_scale)
        for i in range(Config.NUM_JOINTS)
    ]


def test_read_complete_feedback_snapshot_sends_zero_keepalive_while_waiting():
    transport = DelayedFeedbackTransport(_feedback_round())

    snapshot = can_feedback.read_complete_feedback_snapshot(
        transport,
        tuple(range(1, Config.NUM_JOINTS + 1)),
        feedback_timeout_s=1.0,
    )

    assert transport.read_count == 2
    assert transport.commands[: Config.NUM_JOINTS] == [
        ("mit", motor_id, 0.0, 0.0, 0.0, 0.0, 0.0)
        for motor_id in range(1, Config.NUM_JOINTS + 1)
    ]
    np.testing.assert_allclose(snapshot.q, [0.001 * (i + 1) for i in range(Config.NUM_JOINTS)])
    np.testing.assert_allclose(snapshot.tau, [0.01 * (i + 1) for i in range(Config.NUM_JOINTS)])


def test_feedback_timeout_reports_missing_motors_and_safe_stop_closes():
    transport = FakeCanTransport([])

    with pytest.raises(can_feedback.CanFeedbackTimeout) as exc:
        can_feedback.read_complete_feedback_snapshot(transport, feedback_timeout_s=0.001)

    assert exc.value.missing_ids == (1, 2, 3, 4, 5, 6, 7)
    can_feedback.safe_zero_and_disable(transport)
    transport.close()

    nonzero = [cmd for cmd in transport.commands if cmd[0] == "mit" and cmd[-1] not in (0.0, None)]
    assert nonzero == []
    assert ("disable", 7, None) in transport.commands
    assert transport.closed is True


def test_collect_real_param_id_data_records_feedback_and_sends_mit_commands(monkeypatch):
    monkeypatch.setattr(param_id_real.Config, "PARAM_ID_REAL_START_TOL_RAD", 0.1)
    frames = _feedback_round() + _feedback_round() + _feedback_round()
    transport = FakeCanTransport(frames)
    backend = FakeBackend(torque=np.full(Config.NUM_JOINTS, 0.02))
    t_arr = np.array([0.0, 0.01, 0.02], dtype=np.float64)
    q_traj = np.array([[0.001 * (i + 1) for i in range(Config.NUM_JOINTS)]] * 3, dtype=np.float64)
    qd_traj = np.zeros_like(q_traj)

    data = param_id_real._collect_real_param_id_data(
        transport,
        backend,
        t_arr,
        q_traj,
        qd_traj,
        rerun_ok=False,
        feedback_timeout_s=1.0,
    )

    assert data.samples == 3
    np.testing.assert_allclose(data.q_meas[0], q_traj[0])
    np.testing.assert_allclose(data.tau_meas[0], [0.01 * (i + 1) for i in range(Config.NUM_JOINTS)])
    nonzero_pd = [
        cmd for cmd in transport.commands
        if cmd[0] == "mit" and cmd[4] == 1.0 and cmd[-1] == 0.02
    ]
    assert len(nonzero_pd) == Config.NUM_JOINTS * 3


def test_collect_real_param_id_data_rejects_initial_pose_mismatch(monkeypatch):
    monkeypatch.setattr(param_id_real.Config, "PARAM_ID_REAL_START_TOL_RAD", 0.01)
    transport = FakeCanTransport(_feedback_round(q_offset=1.0))
    backend = FakeBackend()
    t_arr = np.array([0.0, 0.01], dtype=np.float64)
    q_traj = np.zeros((2, Config.NUM_JOINTS), dtype=np.float64)
    qd_traj = np.zeros_like(q_traj)

    with pytest.raises(RuntimeError, match="initial joint pose"):
        param_id_real._collect_real_param_id_data(
            transport,
            backend,
            t_arr,
            q_traj,
            qd_traj,
            feedback_timeout_s=1.0,
        )


def test_collect_real_param_id_data_rejects_predicted_torque_over_limit(monkeypatch):
    monkeypatch.setattr(param_id_real.Config, "PARAM_ID_REAL_START_TOL_RAD", 0.1)
    monkeypatch.setattr(param_id_real.Config, "TORQUE_LIMITS", np.ones(Config.NUM_JOINTS) * 0.05)
    transport = FakeCanTransport(_feedback_round())
    backend = FakeBackend(torque=np.ones(Config.NUM_JOINTS))
    t_arr = np.array([0.0, 0.01], dtype=np.float64)
    q_traj = np.array([[0.001 * (i + 1) for i in range(Config.NUM_JOINTS)]] * 2, dtype=np.float64)
    qd_traj = np.zeros_like(q_traj)

    with pytest.raises(RuntimeError, match="exceeds limits"):
        param_id_real._collect_real_param_id_data(
            transport,
            backend,
            t_arr,
            q_traj,
            qd_traj,
            feedback_timeout_s=1.0,
        )


def test_collect_real_param_id_data_preserves_original_backend_safety(monkeypatch):
    monkeypatch.setattr(param_id_real.Config, "PARAM_ID_REAL_START_TOL_RAD", 0.1)
    t_arr = np.array([0.0, 0.01], dtype=np.float64)
    q_traj = np.array([[0.001 * (i + 1) for i in range(Config.NUM_JOINTS)]] * 2, dtype=np.float64)
    qd_traj = np.zeros_like(q_traj)

    unsafe_cases = [
        (FakeBackend(safety_status=-1), "backend safety check"),
        (FakeBackend(q_ref_safe=False), "q_ref safety"),
        (FakeBackend(joint_safety_status=-1), "joint safety"),
    ]
    for backend, message in unsafe_cases:
        transport = FakeCanTransport(_feedback_round())
        with pytest.raises(RuntimeError, match=message):
            param_id_real._collect_real_param_id_data(
                transport,
                backend,
                t_arr,
                q_traj,
                qd_traj,
                feedback_timeout_s=1.0,
            )
        nonzero_pd = [
            cmd for cmd in transport.commands
            if cmd[0] == "mit" and cmd[4] != 0.0
        ]
        assert nonzero_pd == []


def test_validate_identification_data_rejects_too_few_or_low_torque(monkeypatch):
    monkeypatch.setattr(param_id_real.Config, "PARAM_ID_REAL_MIN_SAMPLES", 3)
    monkeypatch.setattr(param_id_real.Config, "PARAM_ID_REAL_MIN_TAU_RMS", 1e-4)
    low_count = param_id_real.RealParamIdData(
        q_meas=np.zeros((2, Config.NUM_JOINTS)),
        qd_meas=np.zeros((2, Config.NUM_JOINTS)),
        qdd_meas=np.zeros((2, Config.NUM_JOINTS)),
        tau_meas=np.ones((2, Config.NUM_JOINTS)),
        samples=2,
    )
    low_torque = param_id_real.RealParamIdData(
        q_meas=np.zeros((3, Config.NUM_JOINTS)),
        qd_meas=np.zeros((3, Config.NUM_JOINTS)),
        qdd_meas=np.zeros((3, Config.NUM_JOINTS)),
        tau_meas=np.zeros((3, Config.NUM_JOINTS)),
        samples=3,
    )

    with pytest.raises(RuntimeError, match="有效样本不足"):
        param_id_real._validate_identification_data(low_count)
    with pytest.raises(RuntimeError, match="反馈力矩 RMS"):
        param_id_real._validate_identification_data(low_torque)
