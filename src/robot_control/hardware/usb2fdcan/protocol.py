"""CAN frame packing helpers for Damiao USB2FDCAN."""

from __future__ import annotations

import socket
import struct

from robot_control.hardware.usb2fdcan.constants import (
    CAN_MTU,
    CANFD_MTU,
)
from robot_control.hardware.usb2fdcan.types import (
    Control_Mode,
    Control_Mode_Code,
    DM_Motor_Type,
    get_motor_limits,
)


def float_to_uint(value: float, xmin: float, xmax: float, bits: int) -> int:
    if xmax <= xmin:
        raise ValueError("xmax must be larger than xmin")
    clamped = min(max(float(value), xmin), xmax)
    scale = (1 << bits) - 1
    return int((clamped - xmin) / (xmax - xmin) * scale)


def uint_to_float(value: int, xmin: float, xmax: float, bits: int) -> float:
    scale = (1 << bits) - 1
    return ((float(value) / scale) * (xmax - xmin)) + xmin


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
