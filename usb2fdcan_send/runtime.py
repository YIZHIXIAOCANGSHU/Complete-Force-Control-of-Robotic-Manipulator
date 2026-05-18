"""Environment-driven USB2FDCAN runtime configuration."""

from __future__ import annotations

import os

from usb2fdcan_send.damiao import Usb2FdcanConfig


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def config_from_env(interface: str | None = None) -> Usb2FdcanConfig:
    return Usb2FdcanConfig(
        interface=os.getenv("AM_D02_CAN_INTERFACE", "can0") if interface is None else str(interface),
        nominal_bitrate=_env_int("AM_D02_CAN_NOMINAL_BITRATE", 1_000_000),
        data_bitrate=_env_int("AM_D02_CAN_DATA_BITRATE", 5_000_000),
        configure_interface=_env_bool("AM_D02_CAN_CONFIGURE_INTERFACE", False),
        force_fd=_env_bool("AM_D02_CAN_FORCE_FD", True),
        read_timeout=max(0.0, _env_float("AM_D02_CAN_READ_TIMEOUT_S", 0.002)),
    )

__all__ = ["config_from_env"]

