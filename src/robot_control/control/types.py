"""Shared control-mode data contracts."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from robot_control.config import Config


def _joint_vector(values, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (Config.NUM_JOINTS,):
        raise ValueError(f"{name} must have shape ({Config.NUM_JOINTS},), got {arr.shape}")
    return arr.copy()


@dataclass(frozen=True)
class ControlTarget:
    q: np.ndarray
    qd: np.ndarray | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "q", _joint_vector(self.q, "q"))
        qd = np.zeros(Config.NUM_JOINTS, dtype=np.float64) if self.qd is None else _joint_vector(self.qd, "qd")
        object.__setattr__(self, "qd", qd)


@dataclass(frozen=True)
class ControlCommand:
    q_ref: np.ndarray
    qd_ref: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    tau_ff: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "q_ref", _joint_vector(self.q_ref, "q_ref"))
        object.__setattr__(self, "qd_ref", _joint_vector(self.qd_ref, "qd_ref"))
        object.__setattr__(self, "kp", _joint_vector(self.kp, "kp"))
        object.__setattr__(self, "kd", _joint_vector(self.kd, "kd"))
        object.__setattr__(self, "tau_ff", _joint_vector(self.tau_ff, "tau_ff"))

