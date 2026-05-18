"""Runtime configuration for real SocketCAN control mode."""

from __future__ import annotations

import os
from dataclasses import dataclass

from robot_control.config import _env_bool, _env_float, _env_int
from robot_control.hardware.usb2fdcan import Usb2FdcanConfig, Usb2FdcanTransport

CAN_INTERFACE = os.getenv("AM_D02_CAN_INTERFACE", "can0")
CAN_NOMINAL_BITRATE = _env_int("AM_D02_CAN_NOMINAL_BITRATE", 1_000_000)
CAN_DATA_BITRATE = _env_int("AM_D02_CAN_DATA_BITRATE", 5_000_000)
CAN_FORCE_FD = _env_bool("AM_D02_CAN_FORCE_FD", True)
CAN_CONFIGURE_INTERFACE = _env_bool("AM_D02_CAN_CONFIGURE_INTERFACE", False)
CAN_FEEDBACK_TIMEOUT_S = max(0.001, _env_float("AM_D02_CAN_FEEDBACK_TIMEOUT_S", 0.10))
CAN_STARTUP_ENABLE = _env_bool("AM_D02_CAN_STARTUP_ENABLE", True)
CAN_READ_TIMEOUT_S = max(0.0, _env_float("AM_D02_CAN_READ_TIMEOUT_S", 0.002))
CAN_READ_CHUNK_SIZE = max(19, _env_int("AM_D02_CAN_READ_CHUNK_SIZE", 256))


@dataclass(frozen=True)
class CanRuntimeConfig:
    transport: Usb2FdcanConfig
    motor_ids: tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7)


def build_can_runtime_config() -> CanRuntimeConfig:
    return CanRuntimeConfig(
        transport=Usb2FdcanConfig(
            interface=CAN_INTERFACE,
            nominal_bitrate=CAN_NOMINAL_BITRATE,
            data_bitrate=CAN_DATA_BITRATE,
            configure_interface=CAN_CONFIGURE_INTERFACE,
            force_fd=CAN_FORCE_FD,
            read_timeout=CAN_READ_TIMEOUT_S,
        )
    )


def open_can_transport():
    return Usb2FdcanTransport(build_can_runtime_config().transport)
