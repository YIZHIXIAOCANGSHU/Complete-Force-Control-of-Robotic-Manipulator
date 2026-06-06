"""Shared runtime data types for real SocketCAN control."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from config import Config


MOTOR_IDS = tuple(range(1, Config.ARM_JOINTS + 1))


@dataclass(frozen=True)
class ArmCanRuntime:
    arm: int
    interface: str
    transport: object
    motor_ids: tuple[int, ...] = MOTOR_IDS


@dataclass(frozen=True)
class RealFeedbackSnapshot:
    seq: int
    timestamp_s: float
    q: np.ndarray
    qd: np.ndarray
    tau_actual: np.ndarray
    complete_mask: int
    received_joint_masks: dict[int, int]
    arm_seq: dict[int, int]
