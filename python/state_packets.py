"""UDP state packet helpers."""

from __future__ import annotations

import numpy as np


STATE_PACKET_SIZE = 28


def fill_state_packet(
    state_packet: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    pos_current: np.ndarray,
    quat_current: np.ndarray,
    pos_desired: np.ndarray,
    quat_desired: np.ndarray,
) -> None:
    state_packet[0:7] = q
    state_packet[7:14] = qd
    state_packet[14:17] = pos_current
    state_packet[17:21] = quat_current
    state_packet[21:24] = pos_desired
    state_packet[24:28] = quat_desired
