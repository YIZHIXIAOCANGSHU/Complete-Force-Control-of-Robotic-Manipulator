"""High-level seven-motor USB2FDCAN transport."""

from __future__ import annotations

import errno
import time
from collections import deque
from typing import Any

from robot_control.hardware.usb2fdcan.constants import (
    CANFD_BRS,
    CLEAR_ERROR_CMD,
    DEFAULT_BACKPRESSURE_SLEEP,
    DEFAULT_CONTROL_COMMAND_INTERVAL,
    DEFAULT_CONTROL_COMMAND_REPEAT,
    DEFAULT_PARAM_WRITE_SETTLE,
    DISABLE_CMD,
    ENABLE_CMD,
    MAX_BACKPRESSURE_SLEEP,
    VALID_FEEDBACK_STATE_CODES,
)
from robot_control.hardware.usb2fdcan.config import Usb2FdcanConfig
from robot_control.hardware.usb2fdcan.types import (
    Control_Mode,
    DecodedFeedbackFrame,
    MotorMapping,
    Usb2FdcanStats,
    mode_to_code,
    parse_motor_type,
)
from robot_control.hardware.usb2fdcan.feedback import decode_feedback
from robot_control.hardware.usb2fdcan.protocol import (
    build_control_cmd_frame,
    build_mit_frame,
    build_param_write_frame,
    pack_can_frame,
    pack_canfd_frame,
)
from robot_control.hardware.usb2fdcan.socket_transport import (
    SocketCanTransport,
    configure_can_interface,
    ensure_interface_ready,
)


class Usb2FdcanTransport:
    def __init__(self, config: Usb2FdcanConfig, *, socket_transport: Any | None = None) -> None:
        self.config = config
        self.stats = Usb2FdcanStats()
        self._decoded_frames: deque[DecodedFeedbackFrame] = deque()
        self._mappings = self._build_mappings(config)
        self._feedback_mapping = self._build_feedback_mapping(self._mappings)
        self._closed = False
        self._socket_transport = socket_transport
        if self._socket_transport is None:
            if config.configure_interface:
                configure_can_interface(config.interface, config.nominal_bitrate, config.data_bitrate)
            ensure_interface_ready(config.interface, config.nominal_bitrate, config.data_bitrate)
            self._socket_transport = SocketCanTransport(config.interface, force_fd=config.force_fd)

    @staticmethod
    def _build_mappings(config: Usb2FdcanConfig) -> tuple[MotorMapping, ...]:
        if not (
            len(config.motor_ids)
            == len(config.motor_can_ids)
            == len(config.motor_mst_ids)
            == len(config.motor_types)
        ):
            raise ValueError("motor_ids, motor_can_ids, motor_mst_ids, and motor_types must have equal length")
        return tuple(
            MotorMapping(
                motor_id=int(motor_id),
                can_id=int(config.motor_can_ids[index]),
                mst_id=int(config.motor_mst_ids[index]),
                motor_type=parse_motor_type(config.motor_types[index]),
            )
            for index, motor_id in enumerate(config.motor_ids)
        )

    @staticmethod
    def _build_feedback_mapping(mappings: tuple[MotorMapping, ...]) -> dict[int, MotorMapping]:
        feedback_mapping: dict[int, MotorMapping] = {}
        for mapping in mappings:
            feedback_mapping[int(mapping.can_id)] = mapping
            feedback_mapping[int(mapping.mst_id)] = mapping
        return feedback_mapping

    def _mapping_for_motor_id(self, motor_id: int) -> MotorMapping:
        target_motor_id = int(motor_id)
        for mapping in self._mappings:
            if int(mapping.motor_id) == target_motor_id:
                return mapping
        valid_ids = tuple(mapping.motor_id for mapping in self._mappings)
        raise ValueError(f"motor_id must be within {valid_ids}")

    def _send_with_backpressure(self, can_id: int, payload: bytes) -> bytes:
        backpressure_sleep = DEFAULT_BACKPRESSURE_SLEEP
        while True:
            try:
                self._socket_transport.send(int(can_id), bytes(payload))
                self.stats.send_count += 1
                return self._trace_packet(int(can_id), bytes(payload))
            except OSError as exc:
                if exc.errno != errno.ENOBUFS:
                    raise
                self.stats.backpressure_count += 1
                time.sleep(backpressure_sleep)
                backpressure_sleep = min(backpressure_sleep * 2.0, MAX_BACKPRESSURE_SLEEP)

    def _trace_packet(self, can_id: int, payload: bytes) -> bytes:
        if self.config.force_fd:
            return pack_canfd_frame(int(can_id), payload, flags=CANFD_BRS)
        if len(payload) <= 8:
            return pack_can_frame(int(can_id), payload)
        return pack_canfd_frame(int(can_id), payload)

    def _ensure_mit_mode(self, mapping: MotorMapping) -> bytes:
        can_id, payload = build_param_write_frame(
            int(mapping.can_id),
            10,
            bytes([int(mode_to_code(Control_Mode.MIT_MODE)), 0x00, 0x00, 0x00]),
        )
        packet = self._send_with_backpressure(can_id, payload)
        time.sleep(DEFAULT_PARAM_WRITE_SETTLE)
        return packet

    def _send_control(self, mapping: MotorMapping, cmd: int) -> bytes:
        can_id, payload = build_control_cmd_frame(int(mapping.can_id) + int(Control_Mode.MIT_MODE), int(cmd))
        packets: list[bytes] = []
        for _ in range(DEFAULT_CONTROL_COMMAND_REPEAT):
            try:
                packets.append(self._send_with_backpressure(can_id, payload))
            except OSError:
                break
            time.sleep(DEFAULT_CONTROL_COMMAND_INTERVAL)
        return b"".join(packets)

    def clear_error(self, motor_id: int) -> bytes:
        mapping = self._mapping_for_motor_id(motor_id)
        return self._send_control(mapping, CLEAR_ERROR_CMD)

    def enable_motor(self, motor_id: int) -> bytes:
        mapping = self._mapping_for_motor_id(motor_id)
        return (
            self._ensure_mit_mode(mapping)
            + self.send_zero_mit(int(motor_id))
            + self._send_control(mapping, ENABLE_CMD)
            + self.send_zero_mit(int(motor_id))
        )

    def send_zero_mit(self, motor_id: int) -> bytes:
        return self.send_mit_torque(int(motor_id), 0.0)

    def send_mit_torque(
        self,
        motor_id: int,
        torque: float,
        *,
        kp: float = 0.0,
        kd: float = 0.0,
        position: float = 0.0,
        velocity: float = 0.0,
    ) -> bytes:
        return self.send_mit_command(
            motor_id,
            position=position,
            velocity=velocity,
            kp=kp,
            kd=kd,
            torque=torque,
        )

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
        mapping = self._mapping_for_motor_id(motor_id)
        can_id, payload = build_mit_frame(
            int(mapping.can_id),
            mapping.motor_type,
            kp=float(kp),
            kd=float(kd),
            position=float(position),
            velocity=float(velocity),
            torque=float(torque),
        )
        packet = self._send_with_backpressure(can_id, payload)
        if (
            float(torque) == 0.0
            and float(kp) == 0.0
            and float(kd) == 0.0
            and float(position) == 0.0
            and float(velocity) == 0.0
        ):
            self.stats.last_zero_packet = packet
        return packet

    def disable_motor(self, motor_id: int) -> bytes:
        mapping = self._mapping_for_motor_id(motor_id)
        return self._send_control(mapping, DISABLE_CMD)

    def reset_input_buffer(self) -> None:
        self._decoded_frames.clear()
        while True:
            packet = self._socket_transport.recv(timeout=0.0)
            if packet is None:
                break

    def _append_feedback_frame(self, can_id: int, payload: bytes) -> None:
        if len(payload) < 8:
            return
        if len(payload) >= 3 and payload[2] in (0x33, 0x55, 0xAA):
            return
        mapping = self._feedback_mapping.get(int(can_id))
        if mapping is None:
            return
        decoded = decode_feedback(payload, mapping.motor_type)
        if int(decoded.state_code) not in VALID_FEEDBACK_STATE_CODES:
            raise ValueError(
                f"feedback_state_error motor_id={int(mapping.motor_id)} can_id=0x{int(can_id):03X} "
                f"state=0x{int(decoded.state_code):X}"
            )
        self._decoded_frames.append(
            DecodedFeedbackFrame(
                motor_id=int(mapping.motor_id),
                can_id=int(mapping.can_id),
                mst_id=int(mapping.mst_id),
                state=int(decoded.state_code),
                controller_id=int(decoded.controller_id),
                position=float(decoded.position),
                velocity=float(decoded.velocity),
                torque=float(decoded.torque),
                mos_temperature=float(decoded.mos_temp),
                rotor_temperature=float(decoded.rotor_temp),
            )
        )
        self.stats.feedback_count += 1

    def read(self, size: int) -> bytes:
        read_budget = max(1, int(size))
        reads = 0
        while reads < read_budget:
            packet = self._socket_transport.recv(timeout=float(self.config.read_timeout))
            if packet is None:
                break
            can_id, payload = packet
            self._append_feedback_frame(int(can_id), bytes(payload))
            reads += 1
        self.stats.read_count += reads
        return b""

    def pop_feedback_frame(self) -> DecodedFeedbackFrame | None:
        if not self._decoded_frames:
            return None
        return self._decoded_frames.popleft()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._socket_transport.close()


class Usb2FdcanZeroTransport(Usb2FdcanTransport):
    """Backward-compatible name for the zero-command USB2FDCAN transport."""
