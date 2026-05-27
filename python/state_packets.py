"""UDP packet helpers for the Python simulation and C controller."""

from __future__ import annotations

import numpy as np


ARM_JOINTS = 7
NUM_ARMS = 2
NUM_JOINTS = ARM_JOINTS * NUM_ARMS

Q_OFFSET = 0
QD_OFFSET = Q_OFFSET + NUM_JOINTS
LEFT_TARGET_POS_OFFSET = QD_OFFSET + NUM_JOINTS
LEFT_TARGET_QUAT_OFFSET = LEFT_TARGET_POS_OFFSET + 3
RIGHT_TARGET_POS_OFFSET = LEFT_TARGET_QUAT_OFFSET + 4
RIGHT_TARGET_QUAT_OFFSET = RIGHT_TARGET_POS_OFFSET + 3
DT_OFFSET = RIGHT_TARGET_QUAT_OFFSET + 4

CONTROL_INPUT_PACKET_SIZE = DT_OFFSET + 1
TORQUE_OUTPUT_PACKET_SIZE = NUM_JOINTS

# Backward-compatible alias for tests/utilities that still import the old name.
STATE_PACKET_SIZE = CONTROL_INPUT_PACKET_SIZE


def fill_control_input_packet(
    control_packet: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    left_target_pos_base: np.ndarray,
    left_target_quat_base: np.ndarray,
    right_target_pos_base: np.ndarray,
    right_target_quat_base: np.ndarray,
    dt_s: float,
) -> None:
    control_packet[Q_OFFSET:QD_OFFSET] = q
    control_packet[QD_OFFSET:LEFT_TARGET_POS_OFFSET] = qd
    control_packet[LEFT_TARGET_POS_OFFSET:LEFT_TARGET_QUAT_OFFSET] = left_target_pos_base
    control_packet[LEFT_TARGET_QUAT_OFFSET:RIGHT_TARGET_POS_OFFSET] = left_target_quat_base
    control_packet[RIGHT_TARGET_POS_OFFSET:RIGHT_TARGET_QUAT_OFFSET] = right_target_pos_base
    control_packet[RIGHT_TARGET_QUAT_OFFSET:DT_OFFSET] = right_target_quat_base
    control_packet[DT_OFFSET] = float(dt_s)


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
        pos_desired,
        quat_desired,
        0.0,
    )
