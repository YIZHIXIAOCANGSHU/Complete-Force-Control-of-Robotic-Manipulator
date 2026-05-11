"""Minimal Damiao USB2FDCAN transport for high-rate zero MIT commands.

This module is intentionally independent from the historical ``send/`` helper.
It keeps only the protocol pieces needed by ``usbfdcan-sim``: SocketCAN I/O,
MIT command packing, feedback decoding, and seven-motor zero/safety commands.
"""

from __future__ import annotations

import errno
import os
import select
import socket
import struct
import subprocess
import time
from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any

import numpy as np


CAN_MTU = 16
CANFD_MTU = 72
CAN_RAW_FD_FRAMES = getattr(socket, "CAN_RAW_FD_FRAMES", 5)
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", socket.SOL_CAN_BASE + socket.CAN_RAW)
CANFD_BRS = getattr(socket, "CANFD_BRS", 0x01)

CLEAR_ERROR_CMD = 0xFB
ENABLE_CMD = 0xFC
DISABLE_CMD = 0xFD
DEFAULT_BACKPRESSURE_SLEEP = 0.0005
MAX_BACKPRESSURE_SLEEP = 0.01
DEFAULT_CONTROL_COMMAND_REPEAT = 5
DEFAULT_CONTROL_COMMAND_INTERVAL = 0.002
DEFAULT_PARAM_WRITE_SETTLE = 0.002
VALID_FEEDBACK_STATE_CODES = frozenset({0x0, 0x1, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE})

DEFAULT_MOTOR_IDS = (1, 2, 3, 4, 5, 6, 7)
DEFAULT_MOTOR_CAN_IDS = (0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07)
DEFAULT_MOTOR_MST_IDS = (0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17)
DEFAULT_MOTOR_TYPES = ("DM8009", "DM8009", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310")


class DM_Motor_Type(IntEnum):
    DM3507 = 0
    DM4310 = 1
    DM4310_48V = 2
    DM4340 = 3
    DM4340_48V = 4
    DM6006 = 5
    DM6248 = 6
    DM8006 = 7
    DM8009 = 8
    DM10010L = 9
    DM10010 = 10
    DMH3510 = 11
    DMH6215 = 12
    DMS3519 = 13
    DMG6220 = 14


class Control_Mode(IntEnum):
    MIT_MODE = 0x000
    POS_VEL_MODE = 0x100
    VEL_MODE = 0x200
    POS_FORCE_MODE = 0x300


class Control_Mode_Code(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    POS_FORCE = 4


LIMIT_PARAM = [
    [12.566, 50, 5],
    [12.5, 30, 10],
    [12.5, 50, 10],
    [12.5, 10, 28],
    [12.5, 20, 28],
    [12.5, 45, 12],
    [12.566, 20, 120],
    [12.5, 45, 20],
    [12.5, 45, 54],
    [12.5, 25, 200],
    [12.5, 20, 200],
    [12.5, 280, 1],
    [12.5, 45, 10],
    [12.5, 2000, 2],
    [12.5, 45, 10],
]


@dataclass(frozen=True)
class MotorLimits:
    pmax: float
    vmax: float
    tmax: float


MOTOR_LIMITS = {
    motor_type: MotorLimits(*LIMIT_PARAM[motor_type.value])
    for motor_type in DM_Motor_Type
}


@dataclass
class MotorFeedback:
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    controller_id: int = 0
    state_code: int = 0
    mos_temp: float = 0.0
    rotor_temp: float = 0.0


@dataclass(frozen=True)
class DecodedFeedbackFrame:
    motor_id: int
    can_id: int
    mst_id: int
    state: int
    controller_id: int
    position: float
    velocity: float
    torque: float
    mos_temperature: float
    rotor_temperature: float


@dataclass(frozen=True)
class _MotorMapping:
    motor_id: int
    can_id: int
    mst_id: int
    motor_type: DM_Motor_Type


@dataclass(frozen=True)
class Usb2FdcanConfig:
    interface: str = "can0"
    nominal_bitrate: int = 1_000_000
    data_bitrate: int = 5_000_000
    configure_interface: bool = False
    force_fd: bool = True
    read_timeout: float = 0.002
    motor_ids: tuple[int, ...] = DEFAULT_MOTOR_IDS
    motor_can_ids: tuple[int, ...] = DEFAULT_MOTOR_CAN_IDS
    motor_mst_ids: tuple[int, ...] = DEFAULT_MOTOR_MST_IDS
    motor_types: tuple[str, ...] = DEFAULT_MOTOR_TYPES


@dataclass
class Usb2FdcanStats:
    send_count: int = 0
    read_count: int = 0
    feedback_count: int = 0
    backpressure_count: int = 0
    last_zero_packet: bytes = b""


def float_to_uint(value: float, xmin: float, xmax: float, bits: int) -> int:
    if xmax <= xmin:
        raise ValueError("xmax must be larger than xmin")
    clamped = min(max(float(value), xmin), xmax)
    scale = (1 << bits) - 1
    return int((clamped - xmin) / (xmax - xmin) * scale)


def uint_to_float(value: int, xmin: float, xmax: float, bits: int) -> float:
    scale = (1 << bits) - 1
    return ((float(value) / scale) * (xmax - xmin)) + xmin


def parse_motor_type(value: DM_Motor_Type | str) -> DM_Motor_Type:
    if isinstance(value, DM_Motor_Type):
        return value
    return DM_Motor_Type[str(value)]


def get_motor_limits(motor_type: DM_Motor_Type | str) -> MotorLimits:
    return MOTOR_LIMITS[parse_motor_type(motor_type)]


def mode_to_code(mode: Control_Mode) -> Control_Mode_Code:
    mapping = {
        Control_Mode.MIT_MODE: Control_Mode_Code.MIT,
        Control_Mode.POS_VEL_MODE: Control_Mode_Code.POS_VEL,
        Control_Mode.VEL_MODE: Control_Mode_Code.VEL,
        Control_Mode.POS_FORCE_MODE: Control_Mode_Code.POS_FORCE,
    }
    return mapping[Control_Mode(mode)]


def pack_can_frame(can_id: int, payload: bytes) -> bytes:
    if len(payload) > 8:
        raise ValueError("Classic CAN payload must be 8 bytes or fewer")
    return struct.pack("=IB3x8s", int(can_id), len(payload), payload.ljust(8, b"\x00"))


def pack_canfd_frame(can_id: int, payload: bytes, flags: int = 0) -> bytes:
    if len(payload) > 64:
        raise ValueError("CAN FD payload must be 64 bytes or fewer")
    return struct.pack("=IBB2x64s", int(can_id), len(payload), int(flags), payload.ljust(64, b"\x00"))


def unpack_can_packet(packet: bytes) -> tuple[int, bytes]:
    if len(packet) == CAN_MTU:
        can_id, can_dlc, data = struct.unpack("=IB3x8s", packet)
        return can_id & socket.CAN_SFF_MASK, data[:can_dlc]
    if len(packet) == CANFD_MTU:
        can_id, length, _, data = struct.unpack("=IBB2x64s", packet)
        return can_id & socket.CAN_SFF_MASK, data[:length]
    raise ValueError(f"Unsupported CAN packet size: {len(packet)}")


def build_control_cmd_frame(can_id: int, cmd: int) -> tuple[int, bytes]:
    return int(can_id), bytes([0xFF] * 7 + [int(cmd)])


def build_param_write_frame(can_id: int, rid: int, data: bytes) -> tuple[int, bytes]:
    if len(data) != 4:
        raise ValueError("Motor parameter writes require exactly 4 data bytes")
    return 0x7FF, bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, 0x55, rid, *data])


def build_mit_frame(
    can_id: int,
    motor_type: DM_Motor_Type | str,
    kp: float,
    kd: float,
    position: float,
    velocity: float,
    torque: float,
) -> tuple[int, bytes]:
    limits = get_motor_limits(motor_type)
    kp_uint = float_to_uint(kp, 0.0, 500.0, 12)
    kd_uint = float_to_uint(kd, 0.0, 5.0, 12)
    q_uint = float_to_uint(position, -limits.pmax, limits.pmax, 16)
    dq_uint = float_to_uint(velocity, -limits.vmax, limits.vmax, 12)
    tau_uint = float_to_uint(torque, -limits.tmax, limits.tmax, 12)
    data = bytes(
        [
            (q_uint >> 8) & 0xFF,
            q_uint & 0xFF,
            (dq_uint >> 4) & 0xFF,
            ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F),
            kp_uint & 0xFF,
            (kd_uint >> 4) & 0xFF,
            ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F),
            tau_uint & 0xFF,
        ]
    )
    return int(can_id) + Control_Mode.MIT_MODE, data


def decode_feedback(data: bytes, motor_type: DM_Motor_Type | str) -> MotorFeedback:
    if len(data) < 8:
        raise ValueError("Motor feedback requires 8 bytes")
    limits = get_motor_limits(motor_type)
    controller_id = data[0] & 0x0F
    state_code = (data[0] >> 4) & 0x0F
    q_uint = (data[1] << 8) | data[2]
    dq_uint = (data[3] << 4) | (data[4] >> 4)
    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
    return MotorFeedback(
        position=uint_to_float(q_uint, -limits.pmax, limits.pmax, 16),
        velocity=uint_to_float(dq_uint, -limits.vmax, limits.vmax, 12),
        torque=uint_to_float(tau_uint, -limits.tmax, limits.tmax, 12),
        controller_id=controller_id,
        state_code=state_code,
        mos_temp=float(data[6]),
        rotor_temp=float(data[7]),
    )


def configure_can_interface(interface: str, nominal_bitrate: int, data_bitrate: int) -> None:
    commands = [
        ["ip", "link", "set", interface, "down"],
        [
            "ip",
            "link",
            "set",
            interface,
            "type",
            "can",
            "bitrate",
            str(nominal_bitrate),
            "dbitrate",
            str(data_bitrate),
            "fd",
            "on",
        ],
        ["ip", "link", "set", interface, "up"],
    ]
    for cmd in commands:
        subprocess.run(cmd, check=True)


def ensure_interface_ready(interface: str, nominal_bitrate: int, data_bitrate: int) -> None:
    path = f"/sys/class/net/{interface}/operstate"
    if not os.path.exists(path):
        raise RuntimeError(f"CAN interface {interface} does not exist")
    with open(path, "r", encoding="utf-8") as file_obj:
        state = file_obj.read().strip()
    if state != "up":
        raise RuntimeError(
            f"{interface} 当前不是 UP 状态。先执行:\n"
            f"  sudo ip link set {interface} down\n"
            f"  sudo ip link set {interface} type can bitrate {nominal_bitrate} dbitrate {data_bitrate} fd on\n"
            f"  sudo ip link set {interface} up"
        )


class SocketCanTransport:
    def __init__(self, interface: str, *, force_fd: bool = True, fd_flags: int = CANFD_BRS) -> None:
        self.interface = str(interface)
        self.force_fd = bool(force_fd)
        self.fd_flags = int(fd_flags)
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
        self.socket.settimeout(0.1)
        self.socket.bind((self.interface,))

    def send(self, can_id: int, payload: bytes) -> None:
        if self.force_fd:
            packet = pack_canfd_frame(int(can_id), payload, flags=self.fd_flags)
        elif len(payload) <= 8:
            packet = pack_can_frame(int(can_id), payload)
        else:
            packet = pack_canfd_frame(int(can_id), payload)
        self.socket.send(packet)

    def recv(self, timeout: float = 0.1) -> tuple[int, bytes] | None:
        try:
            ready, _, _ = select.select([self.socket], [], [], float(timeout))
            if not ready:
                return None
            packet = self.socket.recv(CANFD_MTU)
        except socket.timeout:
            return None
        return unpack_can_packet(packet)

    def close(self) -> None:
        self.socket.close()


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
    def _build_mappings(config: Usb2FdcanConfig) -> tuple[_MotorMapping, ...]:
        if not (
            len(config.motor_ids)
            == len(config.motor_can_ids)
            == len(config.motor_mst_ids)
            == len(config.motor_types)
        ):
            raise ValueError("motor_ids, motor_can_ids, motor_mst_ids, and motor_types must have equal length")
        return tuple(
            _MotorMapping(
                motor_id=int(motor_id),
                can_id=int(config.motor_can_ids[index]),
                mst_id=int(config.motor_mst_ids[index]),
                motor_type=parse_motor_type(config.motor_types[index]),
            )
            for index, motor_id in enumerate(config.motor_ids)
        )

    @staticmethod
    def _build_feedback_mapping(mappings: tuple[_MotorMapping, ...]) -> dict[int, _MotorMapping]:
        feedback_mapping: dict[int, _MotorMapping] = {}
        for mapping in mappings:
            feedback_mapping[int(mapping.can_id)] = mapping
            feedback_mapping[int(mapping.mst_id)] = mapping
        return feedback_mapping

    def _mapping_for_motor_id(self, motor_id: int) -> _MotorMapping:
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

    def _ensure_mit_mode(self, mapping: _MotorMapping) -> bytes:
        can_id, payload = build_param_write_frame(
            int(mapping.can_id),
            10,
            bytes([int(mode_to_code(Control_Mode.MIT_MODE)), 0x00, 0x00, 0x00]),
        )
        packet = self._send_with_backpressure(can_id, payload)
        time.sleep(DEFAULT_PARAM_WRITE_SETTLE)
        return packet

    def _send_control(self, mapping: _MotorMapping, cmd: int) -> bytes:
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
        return self._ensure_mit_mode(mapping) + self._send_control(mapping, ENABLE_CMD)

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
        if float(torque) == 0.0 and float(kp) == 0.0 and float(kd) == 0.0 and float(position) == 0.0 and float(velocity) == 0.0:
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
    """Backward-compatible name for the zero-command mirror transport."""


__all__ = [
    "CANFD_BRS",
    "CANFD_MTU",
    "CAN_MTU",
    "DEFAULT_MOTOR_CAN_IDS",
    "DEFAULT_MOTOR_IDS",
    "DEFAULT_MOTOR_MST_IDS",
    "DEFAULT_MOTOR_TYPES",
    "DM_Motor_Type",
    "DecodedFeedbackFrame",
    "MotorFeedback",
    "Usb2FdcanConfig",
    "Usb2FdcanStats",
    "Usb2FdcanTransport",
    "Usb2FdcanZeroTransport",
    "build_mit_frame",
    "decode_feedback",
    "pack_canfd_frame",
    "pack_can_frame",
    "unpack_can_packet",
]
