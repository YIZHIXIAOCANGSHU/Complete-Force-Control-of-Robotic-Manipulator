"""UDP packet helpers for the Python simulation and C controller."""

from __future__ import annotations

import numpy as np


ARM_JOINTS = 7
NUM_ARMS = 2
NUM_JOINTS = ARM_JOINTS * NUM_ARMS
NUM_BODY_JOINTS = 3

Q_OFFSET = 0
QD_OFFSET = Q_OFFSET + NUM_JOINTS
BODY_Q_OFFSET = QD_OFFSET + NUM_JOINTS
LEFT_TARGET_POS_OFFSET = BODY_Q_OFFSET + NUM_BODY_JOINTS
LEFT_TARGET_QUAT_OFFSET = LEFT_TARGET_POS_OFFSET + 3
RIGHT_TARGET_POS_OFFSET = LEFT_TARGET_QUAT_OFFSET + 4
RIGHT_TARGET_QUAT_OFFSET = RIGHT_TARGET_POS_OFFSET + 3

CONTROL_INPUT_PACKET_SIZE = RIGHT_TARGET_QUAT_OFFSET + 4
TORQUE_OUTPUT_PACKET_SIZE = NUM_JOINTS

# Backward-compatible alias for tests/utilities that still import the old name.
STATE_PACKET_SIZE = CONTROL_INPUT_PACKET_SIZE


def fill_control_input_packet(
    control_packet: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    body_q: np.ndarray,
    left_target_pos_body: np.ndarray,
    left_target_quat: np.ndarray,
    right_target_pos_body: np.ndarray,
    right_target_quat: np.ndarray,
) -> None:
    control_packet[Q_OFFSET:QD_OFFSET] = q
    control_packet[QD_OFFSET:BODY_Q_OFFSET] = qd
    control_packet[BODY_Q_OFFSET:LEFT_TARGET_POS_OFFSET] = body_q
    control_packet[LEFT_TARGET_POS_OFFSET:LEFT_TARGET_QUAT_OFFSET] = left_target_pos_body
    control_packet[LEFT_TARGET_QUAT_OFFSET:RIGHT_TARGET_POS_OFFSET] = left_target_quat
    control_packet[RIGHT_TARGET_POS_OFFSET:RIGHT_TARGET_QUAT_OFFSET] = right_target_pos_body
    control_packet[RIGHT_TARGET_QUAT_OFFSET:CONTROL_INPUT_PACKET_SIZE] = right_target_quat


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
        np.zeros(NUM_BODY_JOINTS, dtype=np.float64),
        pos_desired,
        quat_desired,
        pos_desired,
        quat_desired,
    )
