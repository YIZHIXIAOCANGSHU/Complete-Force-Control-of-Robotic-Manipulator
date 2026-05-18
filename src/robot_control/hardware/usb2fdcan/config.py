"""USB2FDCAN runtime configuration."""

from __future__ import annotations

from dataclasses import dataclass

DEFAULT_MOTOR_IDS = (1, 2, 3, 4, 5, 6, 7)
DEFAULT_MOTOR_CAN_IDS = (0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07)
DEFAULT_MOTOR_MST_IDS = (0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17)
DEFAULT_MOTOR_TYPES = ("DM8009", "DM8009", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310")


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
