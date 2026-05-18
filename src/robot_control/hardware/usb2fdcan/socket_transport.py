"""SocketCAN transport primitives for USB2FDCAN."""

from __future__ import annotations

import os
import select
import socket
import subprocess

from robot_control.hardware.usb2fdcan.constants import (
    CAN_RAW_FD_FRAMES,
    CANFD_BRS,
    CANFD_MTU,
    SOL_CAN_RAW,
)
from robot_control.hardware.usb2fdcan.protocol import pack_can_frame, pack_canfd_frame, unpack_can_packet


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
