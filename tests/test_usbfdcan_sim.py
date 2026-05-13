from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "python"))

from common.shared_state import SharedRobotState  # noqa: E402
from usb2fdcan_send.damiao import DecodedFeedbackFrame  # noqa: E402
from mirror import app  # noqa: E402
from mirror import rerun_feedback  # noqa: E402


class DummyRerun:
    def __init__(self) -> None:
        self.logs: list[tuple[str, object, bool]] = []
        self.blueprint = None

    def init(self, *_args, **_kwargs) -> None:
        return None

    def log(self, path: str, payload, static: bool = False) -> None:
        self.logs.append((path, payload, static))

    def send_blueprint(self, blueprint) -> None:
        self.blueprint = blueprint

    def SeriesLines(self, **kwargs):
        return {"kind": "SeriesLines", **kwargs}


class DummyBlueprint:
    @staticmethod
    def TimeSeriesView(name: str, origin: str, **kwargs):
        return {"kind": "TimeSeriesView", "name": name, "origin": origin, **kwargs}

    @staticmethod
    def TextLogView(name: str, origin: str, **kwargs):
        return {"kind": "TextLogView", "name": name, "origin": origin, **kwargs}

    @staticmethod
    def TextDocumentView(name: str, origin: str, **kwargs):
        return {"kind": "TextDocumentView", "name": name, "origin": origin, **kwargs}

    @staticmethod
    def Horizontal(*children, name: str | None = None):
        return {"kind": "Horizontal", "name": name, "children": list(children)}

    @staticmethod
    def Vertical(*children, name: str | None = None):
        return {"kind": "Vertical", "name": name, "children": list(children)}

    @staticmethod
    def Tabs(*children, name: str | None = None):
        return {"kind": "Tabs", "name": name, "children": list(children)}

    @staticmethod
    def Blueprint(root, collapse_panels: bool = False):
        return {"kind": "Blueprint", "root": root, "collapse_panels": collapse_panels}


def _iter_blueprint_nodes(node):
    if isinstance(node, dict):
        yield node
        for value in node.values():
            yield from _iter_blueprint_nodes(value)
    elif isinstance(node, list):
        for item in node:
            yield from _iter_blueprint_nodes(item)


class FakeZeroTransport:
    def __init__(self, frames=None) -> None:
        self.frames = list(frames or [])
        self.commands: list[tuple[str, int, float | None]] = []
        self.stats = SimpleNamespace(backpressure_count=0, send_count=0)
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

    def clear_error(self, motor_id: int):
        self.commands.append(("clear", int(motor_id), None))

    def enable_motor(self, motor_id: int):
        self.commands.append(("enable", int(motor_id), None))

    def send_zero_mit(self, motor_id: int) -> bytes:
        self.commands.append(("zero", int(motor_id), 0.0))
        self.stats.send_count += 1
        if self.stats.send_count == 1:
            return b"zero-packet"
        return b""

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
        self.stats.send_count += 1
        return b"mit-packet"

    def disable_motor(self, motor_id: int):
        self.commands.append(("disable", int(motor_id), None))

    def close(self) -> None:
        self.closed = True


class StopAfterRoundRerun:
    def __init__(self) -> None:
        self.feedback_payloads: list[dict[str, object]] = []
        self.perf_payloads: list[dict[str, object]] = []
        self.abort_payloads: list[dict[str, object]] = []

    def log_feedback_frame(self, **payload) -> None:
        self.feedback_payloads.append(payload)

    def log_performance(self, **payload) -> None:
        self.perf_payloads.append(payload)

    def log_abort(self, **payload) -> None:
        self.abort_payloads.append(payload)

    def close(self) -> None:
        pass


class FakeCompTool:
    def __init__(self, status: int = 0) -> None:
        self.status = status
        self.compute_calls: list[tuple[list[float], list[float], list[float], list[float]]] = []
        self.closed = False

    def compute_fk(self, q):
        return [0.1, 0.2, 0.3], [1.0, 0.0, 0.0, 0.0]

    def compute(self, q, qd, target_pos, target_quat):
        self.compute_calls.append((list(q), list(qd), list(target_pos), list(target_quat)))
        return SimpleNamespace(
            tau_total=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
            q_ref=[0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77],
            qd_ref=[0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07],
            kp=[10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0],
            kd=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            tau_ff=[1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
            ee_pos=[0.4, 0.5, 0.6],
            ee_quat=[1.0, 0.0, 0.0, 0.0],
            status=self.status,
        )

    def close(self):
        self.closed = True


def _feedback_frame(motor_id: int, *, position: float | None = None, velocity: float | None = None):
    return DecodedFeedbackFrame(
        motor_id=motor_id,
        can_id=motor_id,
        mst_id=0x10 + motor_id,
        state=1,
        controller_id=motor_id,
        position=0.1 * motor_id if position is None else position,
        velocity=0.01 * motor_id if velocity is None else velocity,
        torque=0.001 * motor_id,
        mos_temperature=40.0 + motor_id,
        rotor_temperature=50.0 + motor_id,
    )


def test_tx_loop_sends_all_seven_zero_commands_without_normal_sleep(monkeypatch):
    sleeps: list[float] = []
    monkeypatch.setattr(app.time, "sleep", sleeps.append)

    transport = FakeZeroTransport()
    app.shutdown_event.clear()
    try:
        app.tx_zero_loop(transport, motor_ids=(1, 2, 3, 4, 5, 6, 7), max_rounds=1)
    finally:
        app.shutdown_event.clear()

    assert transport.commands == [("mit", i, 0.0, 0.0, 0.0, 0.0, 0.0) for i in range(1, 8)]
    assert sleeps == []


def test_startup_enable_zero_primes_zero_before_and_after_enable():
    transport = FakeZeroTransport()

    app._startup_enable_zero(transport, (1, 2))

    assert transport.commands[:11] == [
        ("reset", 0, None),
        ("clear", 1, None),
        ("mit", 1, 0.0, 0.0, 0.0, 0.0, 0.0),
        ("enable", 1, None),
        ("mit", 1, 0.0, 0.0, 0.0, 0.0, 0.0),
        ("clear", 2, None),
        ("mit", 2, 0.0, 0.0, 0.0, 0.0, 0.0),
        ("enable", 2, None),
        ("mit", 2, 0.0, 0.0, 0.0, 0.0, 0.0),
        ("mit", 1, 0.0, 0.0, 0.0, 0.0, 0.0),
        ("mit", 2, 0.0, 0.0, 0.0, 0.0, 0.0),
    ]


def test_rerun_recorder_sends_motor_grouped_blueprint(monkeypatch):
    dummy_rr = DummyRerun()
    monkeypatch.setattr(rerun_feedback, "rr", dummy_rr)
    monkeypatch.setattr(rerun_feedback, "rrb", DummyBlueprint)

    rerun_feedback.UsbfdcanSimRerunRecorder(motor_ids=(1, 2, 3), spawn=False)

    tab_names = {
        node["name"]
        for node in _iter_blueprint_nodes(dummy_rr.blueprint)
        if node.get("kind") == "Vertical" and node.get("name")
    }
    assert {"Performance", "J1", "J2", "J3"}.issubset(tab_names)

    origins_by_name = {
        node["name"]: node["origin"]
        for node in _iter_blueprint_nodes(dummy_rr.blueprint)
        if node.get("kind") in {"TimeSeriesView", "TextLogView", "TextDocumentView"}
    }

    assert origins_by_name["J1 Position (rad)"] == "/mirror/motors/motor_01/position"
    assert origins_by_name["J1 Velocity (rad/s)"] == "/mirror/motors/motor_01/velocity"
    assert origins_by_name["J1 Feedback Torque (N*m)"] == "/mirror/motors/motor_01/torque"
    assert origins_by_name["J1 MOS Temperature (C)"] == "/mirror/motors/motor_01/mos_temperature"
    assert origins_by_name["J1 Rotor Temperature (C)"] == "/mirror/motors/motor_01/rotor_temperature"
    assert origins_by_name["J1 State Code"] == "/mirror/motors/motor_01/state_code"
    assert origins_by_name["J1 Quality Flags"] == "/mirror/motors/motor_01/quality"
    assert origins_by_name["J1 Events"] == "/mirror/motors/motor_01/events"
    assert origins_by_name["CAN Rates"] == "/mirror/performance/rates"
    assert origins_by_name["Safety / Backpressure"] == "/mirror/performance/safety"
    assert origins_by_name["Raw Zero MIT Packet"] == "/mirror/raw_zero_packet"
    assert "/usbfdcan_sim" not in origins_by_name.values()

    can_rates_view = next(
        node
        for node in _iter_blueprint_nodes(dummy_rr.blueprint)
        if node.get("kind") == "TimeSeriesView" and node.get("name") == "CAN Rates"
    )
    assert can_rates_view["contents"] == [
        "/mirror/performance/tx_send_rate_hz",
        "/mirror/performance/rx_feedback_rate_hz",
        "/mirror/performance/complete_round_rate_hz",
    ]

    static_line_names = {
        path: payload["names"]
        for path, payload, static in dummy_rr.logs
        if static and payload.get("kind") == "SeriesLines"
    }
    assert static_line_names["mirror/motors/motor_01/quality/state_ok"] == ["J1 state_ok"]
    assert static_line_names["mirror/motors/motor_01/quality/safety_ok"] == ["J1 safety_ok"]


def test_rx_loop_updates_shared_state_and_logs_feedback_round():
    transport = FakeZeroTransport(_feedback_frame(i + 1) for i in range(app.Config.NUM_JOINTS))
    shared_state = SharedRobotState()
    rerun = StopAfterRoundRerun()

    app.shutdown_event.clear()
    try:
        app.rx_feedback_loop(
            transport,
            shared_state,
            None,
            rerun,
            startup_enable=False,
            feedback_timeout_s=1.0,
            max_complete_rounds=1,
        )
    finally:
        app.shutdown_event.clear()

    q, qd, tau, _, _ = shared_state.snapshot_control_inputs()
    assert q == [0.1, 0.2, 0.30000000000000004, 0.4, 0.5, 0.6000000000000001, 0.7000000000000001]
    assert qd == [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]
    assert tau == [0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007]
    assert len(rerun.feedback_payloads) == 7
    assert rerun.perf_payloads[-1]["complete_round_rate_hz"] >= 0.0


def test_rx_loop_sends_mit_command_after_complete_feedback_round():
    transport = FakeZeroTransport(_feedback_frame(i + 1) for i in range(app.Config.NUM_JOINTS))
    shared_state = SharedRobotState()
    shared_state.set_target_pose([0.11, 0.22, 0.33], [1.0, 0.0, 0.0, 0.0])
    comp_tool = FakeCompTool()

    app.shutdown_event.clear()
    try:
        app.rx_feedback_loop(
            transport,
            shared_state,
            comp_tool,
            None,
            startup_enable=False,
            feedback_timeout_s=1.0,
            max_complete_rounds=1,
        )
    finally:
        app.shutdown_event.clear()

    assert len(comp_tool.compute_calls) == 1
    assert ("mit", 1, 0.11, 0.01, 10.0, 0.1, 1.1) in transport.commands
    assert ("mit", 7, 0.77, 0.07, 70.0, 0.7, 1.7) in transport.commands


def test_rx_loop_reports_missing_feedback_ids_on_timeout(capsys):
    transport = FakeZeroTransport([_feedback_frame(1), _feedback_frame(4)])

    app.shutdown_event.clear()
    try:
        app.rx_feedback_loop(
            transport,
            SharedRobotState(),
            None,
            None,
            startup_enable=False,
            feedback_timeout_s=0.0,
        )
    finally:
        app.shutdown_event.clear()

    captured = capsys.readouterr()
    assert "缺失电机=(2, 3, 5, 6, 7)" in captured.out


def test_rx_loop_triggers_safe_stop_on_velocity_limit_violation():
    frames = [_feedback_frame(i + 1) for i in range(app.Config.NUM_JOINTS)]
    frames[-1] = _feedback_frame(7, velocity=11.0)
    transport = FakeZeroTransport(frames)
    rerun = StopAfterRoundRerun()

    app.shutdown_event.clear()
    try:
        app.rx_feedback_loop(
            transport,
            SharedRobotState(),
            None,
            rerun,
            startup_enable=False,
            feedback_timeout_s=1.0,
            max_complete_rounds=1,
        )
    finally:
        app.shutdown_event.clear()

    assert app.shutdown_event.is_set() is False
    assert rerun.abort_payloads
    assert "velocity" in str(rerun.abort_payloads[-1]["reason"])


def test_run_mirror_session_safe_stop_zeroes_disables_and_closes_on_timeout(monkeypatch):
    transport = FakeZeroTransport([])
    monkeypatch.setattr(app, "open_zero_transport", lambda: transport)
    monkeypatch.setattr(app, "UsbfdcanSimRerunRecorder", lambda *args, **kwargs: StopAfterRoundRerun())
    monkeypatch.setattr(app, "run_viewer_loop", lambda *args, **kwargs: None)
    monkeypatch.setattr(app, "GravityCompTool", lambda: FakeCompTool())
    monkeypatch.setattr(app.Config, "ENABLE_RERUN", False)

    app.shutdown_event.clear()
    try:
        app.run_mirror_session(start_viewer=False, feedback_timeout_s=0.001, startup_enable=False)
    finally:
        app.shutdown_event.clear()

    assert ("mit", 1, 0.0, 0.0, 0.0, 0.0, 0.0) in transport.commands
    assert ("disable", 7, None) in transport.commands
    assert transport.closed is True
