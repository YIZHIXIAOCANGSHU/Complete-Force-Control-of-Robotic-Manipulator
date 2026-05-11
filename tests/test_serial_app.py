from __future__ import annotations

import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from common.shared_state import SharedRobotState
from real import serial_app
from real.serial_protocol import RECV_FRAME_STRUCT


def _feedback_frame(motor_id: int, pos: float, vel: float, tor: float) -> bytes:
    return RECV_FRAME_STRUCT.pack(0xA5, motor_id, 0, pos, vel, tor, 0.0, 0.0)


class FakeSerial:
    def __init__(self, payload: bytes) -> None:
        self._buffer = bytearray(payload)
        self.writes: list[bytes] = []
        self.closed = False

    @property
    def in_waiting(self) -> int:
        return len(self._buffer)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._buffer[:size])
        del self._buffer[:size]
        return chunk

    def write(self, data: bytes) -> int:
        self.writes.append(bytes(data))
        return len(data)

    def close(self) -> None:
        self.closed = True


class FakeCompTool:
    def __init__(self) -> None:
        self.compute_calls: list[tuple[list[float], list[float], list[float], list[float]]] = []
        self.compute_fk_calls: list[list[float]] = []

    def compute_fk(self, q):
        self.compute_fk_calls.append(list(q))
        return [0.1, 0.2, 0.3], [1.0, 0.0, 0.0, 0.0]

    def compute(self, q, qd, target_pos, target_quat):
        self.compute_calls.append((list(q), list(qd), list(target_pos), list(target_quat)))
        return [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0], [0.4, 0.5, 0.6], [1.0, 0.0, 0.0, 0.0], 0, 0.75


class FakeRerunLogger:
    def __init__(self) -> None:
        self.payloads: list[dict[str, object]] = []

    def log_step(self, **payload) -> None:
        self.payloads.append(payload)
        serial_app.shutdown_event.set()


def test_serial_thread_keeps_rbdl_prediction_updating_when_forwarding_targets(monkeypatch):
    monkeypatch.setattr(serial_app.Config, "SERIAL_FORWARD_TARGET", True)
    monkeypatch.setattr(serial_app.Config, "ENABLE_RERUN", True)
    monkeypatch.setattr(serial_app.Config, "RERUN_LOG_STRIDE", 1)
    monkeypatch.setattr(serial_app.Config, "UART_TEXT_LOG_INTERVAL", 1)
    monkeypatch.setattr(serial_app.Config, "SERIAL_IDLE_SLEEP_S", 0.0)

    shared_state = SharedRobotState()
    shared_state.set_target_pose([0.11, 0.22, 0.33], [1.0, 0.0, 0.0, 0.0])

    payload = b"".join(
        _feedback_frame(i + 1, 0.1 * (i + 1), 0.01 * (i + 1), 0.001 * (i + 1))
        for i in range(serial_app.Config.NUM_JOINTS)
    )
    ser = FakeSerial(payload)
    comp_tool = FakeCompTool()
    rerun_logger = FakeRerunLogger()

    serial_app.shutdown_event.clear()
    try:
        serial_app.serial_thread_func(ser, comp_tool, shared_state, rerun_logger)
    finally:
        serial_app.shutdown_event.clear()

    assert len(comp_tool.compute_calls) == 1
    assert comp_tool.compute_fk_calls == []
    assert len(rerun_logger.payloads) == 1
    assert rerun_logger.payloads[0]["tau_total"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    assert rerun_logger.payloads[0]["pos_actual"] == [0.4, 0.5, 0.6]
    assert len(ser.writes) == 1
    assert ser.closed is True
