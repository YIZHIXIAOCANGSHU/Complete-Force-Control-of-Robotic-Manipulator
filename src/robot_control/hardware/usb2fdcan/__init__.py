"""Damiao USB2FDCAN protocol and transport package."""

from __future__ import annotations

from robot_control.hardware.usb2fdcan.constants import (
    CANFD_BRS,
    CANFD_MTU,
    CAN_MTU,
)
from robot_control.hardware.usb2fdcan.config import (
    DEFAULT_MOTOR_CAN_IDS,
    DEFAULT_MOTOR_IDS,
    DEFAULT_MOTOR_MST_IDS,
    DEFAULT_MOTOR_TYPES,
    Usb2FdcanConfig,
)
from robot_control.hardware.usb2fdcan.types import (
    DM_Motor_Type,
    DecodedFeedbackFrame,
    MotorFeedback,
    MotorLimits,
    Usb2FdcanStats,
    get_motor_limits,
    parse_motor_type,
)
from robot_control.hardware.usb2fdcan.feedback import decode_feedback
from robot_control.hardware.usb2fdcan.protocol import (
    build_control_cmd_frame,
    build_mit_frame,
    build_param_write_frame,
    pack_can_frame,
    pack_canfd_frame,
    unpack_can_packet,
)
from robot_control.hardware.usb2fdcan.socket_transport import (
    SocketCanTransport,
    configure_can_interface,
    ensure_interface_ready,
)
from robot_control.hardware.usb2fdcan.transport import Usb2FdcanTransport, Usb2FdcanZeroTransport

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
    "MotorLimits",
    "SocketCanTransport",
    "Usb2FdcanConfig",
    "Usb2FdcanStats",
    "Usb2FdcanTransport",
    "Usb2FdcanZeroTransport",
    "build_control_cmd_frame",
    "build_mit_frame",
    "build_param_write_frame",
    "configure_can_interface",
    "decode_feedback",
    "ensure_interface_ready",
    "get_motor_limits",
    "pack_canfd_frame",
    "pack_can_frame",
    "parse_motor_type",
    "unpack_can_packet",
]
