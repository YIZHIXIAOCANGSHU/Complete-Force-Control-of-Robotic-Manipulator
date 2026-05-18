"""USB2FDCAN runtime config helpers."""

from __future__ import annotations

from config import _env_bool, _env_float, _env_int
from usb2fdcan_send.damiao import Usb2FdcanConfig


def config_from_env(interface: str = "can0") -> Usb2FdcanConfig:
    return Usb2FdcanConfig(
        interface=interface,
        nominal_bitrate=_env_int("AM_D02_CAN_NOMINAL_BITRATE", 1_000_000),
        data_bitrate=_env_int("AM_D02_CAN_DATA_BITRATE", 5_000_000),
        configure_interface=_env_bool("AM_D02_CAN_CONFIGURE_INTERFACE", False),
        force_fd=_env_bool("AM_D02_CAN_FORCE_FD", True),
        read_timeout=max(0.0, _env_float("AM_D02_CAN_READ_TIMEOUT_S", 0.002)),
    )

