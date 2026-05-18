"""USB2FDCAN motor mapping and runtime configuration exports."""

from __future__ import annotations

from usb2fdcan_send.damiao import (
    DEFAULT_MOTOR_CAN_IDS,
    DEFAULT_MOTOR_IDS,
    DEFAULT_MOTOR_MST_IDS,
    DEFAULT_MOTOR_TYPES,
    DM_Motor_Type,
    MotorLimits,
    Usb2FdcanConfig,
    get_motor_limits,
    parse_motor_type,
)

__all__ = [
    "DEFAULT_MOTOR_CAN_IDS",
    "DEFAULT_MOTOR_IDS",
    "DEFAULT_MOTOR_MST_IDS",
    "DEFAULT_MOTOR_TYPES",
    "DM_Motor_Type",
    "MotorLimits",
    "Usb2FdcanConfig",
    "get_motor_limits",
    "parse_motor_type",
]

