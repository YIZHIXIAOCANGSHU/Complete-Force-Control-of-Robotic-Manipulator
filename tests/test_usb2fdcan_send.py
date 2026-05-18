from __future__ import annotations

import errno
import pytest

from robot_control.hardware.usb2fdcan import (
    CANFD_BRS,
    CANFD_MTU,
    DEFAULT_MOTOR_CAN_IDS,
    DEFAULT_MOTOR_MST_IDS,
    DM_Motor_Type,
    MotorFeedback,
    Usb2FdcanConfig,
    Usb2FdcanTransport,
    Usb2FdcanZeroTransport,
    build_mit_frame,
    decode_feedback,
    pack_canfd_frame,
    unpack_can_packet,
)


class FakeSocketTransport:
    def __init__(self, *, fail_once_with_enobufs: bool = False) -> None:
        self.fail_once_with_enobufs = fail_once_with_enobufs
        self.sent: list[tuple[int, bytes]] = []
        self.recv_packets: list[tuple[int, bytes]] = []
        self.closed = False

    def send(self, can_id: int, payload: bytes) -> None:
        if self.fail_once_with_enobufs:
            self.fail_once_with_enobufs = False
            raise OSError(errno.ENOBUFS, "buffer full")
        self.sent.append((int(can_id), bytes(payload)))

    def recv(self, timeout: float = 0.0):
        _ = timeout
        if not self.recv_packets:
            return None
        return self.recv_packets.pop(0)

    def close(self) -> None:
        self.closed = True


def _feedback_payload_from_mit_payload(state_controller: int, payload: bytes, mos_temp: int = 40, rotor_temp: int = 50) -> bytes:
    return bytes(
        [
            int(state_controller),
            payload[0],
            payload[1],
            payload[2],
            (payload[3] & 0xF0) | (payload[6] & 0x0F),
            payload[7],
            int(mos_temp),
            int(rotor_temp),
        ]
    )


def test_zero_mit_frame_uses_physical_zero_command_encoding():
    can_id, payload = build_mit_frame(
        0x01,
        DM_Motor_Type.DM8009,
        kp=0.0,
        kd=0.0,
        position=0.0,
        velocity=0.0,
        torque=0.0,
    )

    assert can_id == 0x01
    assert len(payload) == 8
    assert payload != b"\x00" * 8
    feedback = decode_feedback(_feedback_payload_from_mit_payload(0x01, payload), DM_Motor_Type.DM8009)
    assert abs(feedback.position) < 5e-4
    assert abs(feedback.velocity) < 2e-2
    assert abs(feedback.torque) < 2e-2


def test_canfd_packet_roundtrip_preserves_can_id_payload_and_flags():
    packet = pack_canfd_frame(0x123, b"\x01\x02\x03", flags=CANFD_BRS)

    assert len(packet) == CANFD_MTU
    assert unpack_can_packet(packet) == (0x123, b"\x01\x02\x03")


def test_transport_decodes_feedback_from_can_and_mst_ids():
    fake = FakeSocketTransport()
    config = Usb2FdcanConfig(read_timeout=0.0)
    transport = Usb2FdcanZeroTransport(config, socket_transport=fake)
    _, payload = build_mit_frame(
        DEFAULT_MOTOR_CAN_IDS[2],
        DM_Motor_Type.DM4340,
        kp=0.0,
        kd=0.0,
        position=1.25,
        velocity=-2.0,
        torque=3.5,
    )
    fake.recv_packets.extend(
        [
            (0x7FE, b"\x00" * 8),
            (DEFAULT_MOTOR_MST_IDS[2], _feedback_payload_from_mit_payload(0x13, payload, 41, 52)),
        ]
    )

    transport.read(2)
    frame = transport.pop_feedback_frame()

    assert frame is not None
    assert frame.motor_id == 3
    assert frame.can_id == DEFAULT_MOTOR_CAN_IDS[2]
    assert frame.mst_id == DEFAULT_MOTOR_MST_IDS[2]
    assert frame.state == 1
    assert frame.controller_id == 3
    assert frame.mos_temperature == 41.0
    assert frame.rotor_temperature == 52.0
    assert frame.position == pytest.approx(1.25, abs=5e-4)
    assert frame.velocity == pytest.approx(-2.0, abs=2e-2)
    assert frame.torque == pytest.approx(3.5, abs=2e-2)


def test_send_zero_mit_retries_enobufs_and_counts_backpressure(monkeypatch):
    fake = FakeSocketTransport(fail_once_with_enobufs=True)
    sleeps: list[float] = []
    monkeypatch.setattr("robot_control.hardware.usb2fdcan.transport.time.sleep", sleeps.append)

    transport = Usb2FdcanZeroTransport(Usb2FdcanConfig(), socket_transport=fake)
    packet = transport.send_zero_mit(1)

    assert len(fake.sent) == 1
    assert fake.sent[0][0] == DEFAULT_MOTOR_CAN_IDS[0]
    assert len(fake.sent[0][1]) == 8
    assert packet
    assert transport.stats.backpressure_count == 1
    assert sleeps and sleeps[0] > 0.0


def test_enable_motor_primes_and_refreshes_zero_mit(monkeypatch):
    fake = FakeSocketTransport()
    monkeypatch.setattr("robot_control.hardware.usb2fdcan.transport.time.sleep", lambda _seconds: None)
    transport = Usb2FdcanTransport(Usb2FdcanConfig(), socket_transport=fake)

    transport.enable_motor(1)

    sent_ids = [can_id for can_id, _payload in fake.sent]
    assert sent_ids[0] == 0x7FF
    assert sent_ids[1] == DEFAULT_MOTOR_CAN_IDS[0]
    assert sent_ids[2:7] == [DEFAULT_MOTOR_CAN_IDS[0]] * 5
    assert sent_ids[7] == DEFAULT_MOTOR_CAN_IDS[0]
    assert fake.sent[2][1][-1] == 0xFC


def test_transport_send_mit_torque_uses_root_usb2fdcan_package():
    fake = FakeSocketTransport()
    transport = Usb2FdcanTransport(Usb2FdcanConfig(), socket_transport=fake)

    packet = transport.send_mit_torque(3, 2.5)

    assert fake.sent
    can_id, payload = fake.sent[-1]
    assert can_id == DEFAULT_MOTOR_CAN_IDS[2]
    assert len(payload) == 8
    assert packet
    feedback = decode_feedback(_feedback_payload_from_mit_payload(0x13, payload), DM_Motor_Type.DM4340)
    assert feedback.torque == pytest.approx(2.5, abs=2e-2)


def test_transport_send_mit_command_forwards_full_mit_fields():
    fake = FakeSocketTransport()
    transport = Usb2FdcanTransport(Usb2FdcanConfig(), socket_transport=fake)

    packet = transport.send_mit_command(
        3,
        position=1.25,
        velocity=-2.0,
        kp=10.0,
        kd=0.5,
        torque=3.5,
    )

    assert fake.sent
    can_id, payload = fake.sent[-1]
    assert can_id == DEFAULT_MOTOR_CAN_IDS[2]
    assert len(payload) == 8
    assert packet
    feedback = decode_feedback(_feedback_payload_from_mit_payload(0x13, payload), DM_Motor_Type.DM4340)
    assert feedback.position == pytest.approx(1.25, abs=5e-4)
    assert feedback.velocity == pytest.approx(-2.0, abs=2e-2)
    assert feedback.torque == pytest.approx(3.5, abs=2e-2)


def test_zero_transport_remains_compatible_alias():
    transport = Usb2FdcanZeroTransport(Usb2FdcanConfig(), socket_transport=FakeSocketTransport())

    assert isinstance(transport, Usb2FdcanTransport)


def test_send_zero_mit_rejects_unknown_motor_id():
    transport = Usb2FdcanZeroTransport(Usb2FdcanConfig(), socket_transport=FakeSocketTransport())

    with pytest.raises(ValueError, match="motor_id"):
        transport.send_zero_mit(99)
