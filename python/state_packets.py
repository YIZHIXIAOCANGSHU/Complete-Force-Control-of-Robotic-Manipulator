"""UDP packet helpers for the Python simulation and C controller."""

from __future__ import annotations

import numpy as np


CONTROL_INPUT_PACKET_SIZE = 22
TORQUE_OUTPUT_PACKET_SIZE = 7

# Backward-compatible alias for tests/utilities that still import the old name.
STATE_PACKET_SIZE = CONTROL_INPUT_PACKET_SIZE


def fill_control_input_packet(
    control_packet: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    target_pos_base: np.ndarray,
    target_quat_base: np.ndarray,
    dt_s: float,
) -> None:
    control_packet[0:7] = q
    control_packet[7:14] = qd
    control_packet[14:17] = target_pos_base
    control_packet[17:21] = target_quat_base
    control_packet[21] = float(dt_s)


def fill_state_packet(
    state_packet: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    _pos_current: np.ndarray,
    _quat_current: np.ndarray,
    pos_desired: np.ndarray,
    quat_desired: np.ndarray,
) -> None:
    fill_control_input_packet(
        state_packet,
        q,
        qd,
        pos_desired,
        quat_desired,
        0.0,
    )
