#!/usr/bin/env python3
"""PD controller primitives for parameter-identification simulation mode."""

from __future__ import annotations

from typing import Iterable

import numpy as np

from robot_control.config import Config
from robot_control.dynamics.pinocchio import PinocchioGravityBackend


_DEFAULT_KP = np.array([10.0, 10.0, 5.0, 5.0, 0.2, 0.2, 0.2], dtype=np.float64)
_DEFAULT_KD = np.array([0.316, 0.316, 0.158, 0.158, 0.032, 0.032, 0.032], dtype=np.float64)


def _as_joint_vector(values, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (Config.NUM_JOINTS,):
        raise ValueError(f"{name} must have shape ({Config.NUM_JOINTS},), got {arr.shape}")
    return arr


def _validate_trajectory_pair(q_traj, qd_traj) -> tuple[np.ndarray, np.ndarray]:
    q = np.asarray(q_traj, dtype=np.float64)
    qd = np.asarray(qd_traj, dtype=np.float64)
    expected_rank = 2
    expected_width = Config.NUM_JOINTS
    if q.ndim != expected_rank or q.shape[1] != expected_width:
        raise ValueError(f"q_traj must have shape (n, {expected_width}), got {q.shape}")
    if qd.shape != q.shape:
        raise ValueError(f"qd_traj must match q_traj shape {q.shape}, got {qd.shape}")
    return q, qd


class PDController:
    """Lightweight PD + nonlinear-effects feedforward torque controller."""

    def __init__(
        self,
        backend: PinocchioGravityBackend,
        kp: Iterable[float] | None = None,
        kd: Iterable[float] | None = None,
        torque_limits: Iterable[float] | None = None,
    ) -> None:
        self.backend = backend
        backend_kp = getattr(backend, "_joint_kp", _DEFAULT_KP)
        backend_kd = getattr(backend, "_joint_kd", _DEFAULT_KD)
        backend_limits = getattr(backend, "_torque_limits", Config.TORQUE_LIMITS)
        self.kp = _as_joint_vector(backend_kp if kp is None else kp, "kp")
        self.kd = _as_joint_vector(backend_kd if kd is None else kd, "kd")
        limits = backend_limits if torque_limits is None else torque_limits
        self.torque_limits = _as_joint_vector(limits, "torque_limits")

    def compute_torque(self, q, qd, q_ref, qd_ref) -> np.ndarray:
        q_arr = _as_joint_vector(q, "q")
        qd_arr = _as_joint_vector(qd, "qd")
        q_ref_arr = _as_joint_vector(q_ref, "q_ref")
        qd_ref_arr = _as_joint_vector(qd_ref, "qd_ref")
        tau_ff = _as_joint_vector(
            self.backend.compute_nonlinear_effects(q_arr, qd_arr),
            "tau_ff",
        )
        tau = self.kp * (q_ref_arr - q_arr) + self.kd * (qd_ref_arr - qd_arr) + tau_ff
        return np.clip(tau, -self.torque_limits, self.torque_limits)
