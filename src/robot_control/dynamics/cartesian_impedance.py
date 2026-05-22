"""Shared Cartesian impedance + gravity/Coriolis + nullspace torque control."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np

from robot_control.config import Config
from robot_control.dynamics.pinocchio import normalize_angle


def _as_vector(values, name: str, size: int) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {arr.shape}")
    return arr


def _normalize_quat_wxyz(quat, fallback=None) -> np.ndarray:
    arr = _as_vector(quat, "quat", 4)
    norm = float(np.linalg.norm(arr))
    if norm > 1e-12 and math.isfinite(norm):
        return arr / norm
    if fallback is not None:
        return _normalize_quat_wxyz(fallback)
    raise ValueError("quat must be finite and non-zero")


def pose_error_6d(pos_ref, quat_ref, pos, quat) -> np.ndarray:
    """Return [position error, orientation error] using wxyz quaternions."""
    ref_pos = _as_vector(pos_ref, "pos_ref", 3)
    cur_pos = _as_vector(pos, "pos", 3)
    ref_quat = _normalize_quat_wxyz(quat_ref)
    cur_quat = _normalize_quat_wxyz(quat)

    err = np.zeros(6, dtype=np.float64)
    err[:3] = ref_pos - cur_pos

    qt = np.array([ref_quat[1], ref_quat[2], ref_quat[3], ref_quat[0]], dtype=np.float64)
    qc = np.array([cur_quat[1], cur_quat[2], cur_quat[3], cur_quat[0]], dtype=np.float64)
    q_inv = np.array([-qc[0], -qc[1], -qc[2], qc[3]], dtype=np.float64)
    q_err = _quat_mul_xyzw(qt, q_inv)
    if q_err[3] < 0.0:
        q_err = -q_err
    s = math.sqrt(float(q_err[0] ** 2 + q_err[1] ** 2 + q_err[2] ** 2))
    if s < 1e-6:
        err[3:] = 2.0 * q_err[:3]
    else:
        angle = 2.0 * math.atan2(s, float(q_err[3]))
        err[3:] = (angle / s) * q_err[:3]
    return err


@dataclass(frozen=True)
class CartesianImpedanceOutput:
    tau_total: np.ndarray
    tau_task: np.ndarray
    tau_null: np.ndarray
    tau_gc: np.ndarray
    ee_pos: np.ndarray
    ee_quat: np.ndarray
    ee_twist: np.ndarray
    pose_error: np.ndarray
    twist_ref: np.ndarray
    wrench_task: np.ndarray
    q_null_ref: np.ndarray
    clipped: bool


class CartesianImpedanceController:
    """Compute tau = J.T F + N.T tau_null + g+c for a 7-DOF arm."""

    def __init__(
        self,
        backend,
        *,
        cartesian_kp: Iterable[float] | None = None,
        cartesian_kd: Iterable[float] | None = None,
        nullspace_kp: Iterable[float] | None = None,
        nullspace_kd: Iterable[float] | None = None,
        nullspace_damping: float | None = None,
        q_null_ref: Iterable[float] | None = None,
        torque_limits: Iterable[float] | None = None,
    ) -> None:
        self.backend = backend
        self.cartesian_kp = _as_vector(
            Config.CARTESIAN_KP if cartesian_kp is None else cartesian_kp,
            "cartesian_kp",
            6,
        )
        self.cartesian_kd = _as_vector(
            Config.CARTESIAN_KD if cartesian_kd is None else cartesian_kd,
            "cartesian_kd",
            6,
        )
        self.nullspace_kp = _as_vector(
            Config.NULLSPACE_KP if nullspace_kp is None else nullspace_kp,
            "nullspace_kp",
            Config.NUM_JOINTS,
        )
        self.nullspace_kd = _as_vector(
            Config.NULLSPACE_KD if nullspace_kd is None else nullspace_kd,
            "nullspace_kd",
            Config.NUM_JOINTS,
        )
        self.nullspace_damping = float(Config.NULLSPACE_DAMPING if nullspace_damping is None else nullspace_damping)
        self.q_null_ref = _as_vector(
            Config.NULLSPACE_Q_REF if q_null_ref is None else q_null_ref,
            "q_null_ref",
            Config.NUM_JOINTS,
        )
        backend_limits = getattr(backend, "_torque_limits", Config.TORQUE_LIMITS)
        self.torque_limits = _as_vector(backend_limits if torque_limits is None else torque_limits, "torque_limits", Config.NUM_JOINTS)

    def compute(self, q, qd, pos_ref, quat_ref, twist_ref=None, q_null_ref=None) -> CartesianImpedanceOutput:
        q_arr = _as_vector(q, "q", Config.NUM_JOINTS)
        qd_arr = _as_vector(qd, "qd", Config.NUM_JOINTS)
        ref_pos = _as_vector(pos_ref, "pos_ref", 3)
        ref_quat = _normalize_quat_wxyz(quat_ref)
        ref_twist = np.zeros(6, dtype=np.float64) if twist_ref is None else _as_vector(twist_ref, "twist_ref", 6)
        q_null = self.q_null_ref if q_null_ref is None else _as_vector(q_null_ref, "q_null_ref", Config.NUM_JOINTS)

        ee_pos_raw, ee_quat_raw = self.backend.compute_fk(q_arr)
        ee_pos = _as_vector(ee_pos_raw, "ee_pos", 3)
        ee_quat = _normalize_quat_wxyz(ee_quat_raw, fallback=ref_quat)
        jacobian = np.asarray(self.backend.compute_jacobian(q_arr), dtype=np.float64)
        if jacobian.shape != (6, Config.NUM_JOINTS):
            raise ValueError(f"jacobian must have shape (6, {Config.NUM_JOINTS}), got {jacobian.shape}")

        ee_twist = jacobian @ qd_arr
        err6 = pose_error_6d(ref_pos, ref_quat, ee_pos, ee_quat)
        wrench_task = self.cartesian_kp * err6 + self.cartesian_kd * (ref_twist - ee_twist)
        tau_task = jacobian.T @ wrench_task

        q_err = np.array([normalize_angle(float(q_null[i] - q_arr[i])) for i in range(Config.NUM_JOINTS)], dtype=np.float64)
        tau_posture = self.nullspace_kp * q_err - self.nullspace_kd * qd_arr
        nullspace = self._nullspace_projector(jacobian)
        tau_null = nullspace.T @ tau_posture

        tau_gc = _as_vector(self.backend.compute_nonlinear_effects(q_arr, qd_arr), "tau_gc", Config.NUM_JOINTS)
        tau_raw = tau_task + tau_null + tau_gc
        tau_total = np.clip(tau_raw, -self.torque_limits, self.torque_limits)
        clipped = bool(np.any(np.abs(tau_total - tau_raw) > 1e-12))

        return CartesianImpedanceOutput(
            tau_total=tau_total,
            tau_task=tau_task,
            tau_null=tau_null,
            tau_gc=tau_gc,
            ee_pos=ee_pos,
            ee_quat=ee_quat,
            ee_twist=ee_twist,
            pose_error=err6,
            twist_ref=ref_twist,
            wrench_task=wrench_task,
            q_null_ref=q_null.copy(),
            clipped=clipped,
        )

    def compute_from_joint_reference(self, q, qd, q_ref, qd_ref, q_null_ref=None) -> CartesianImpedanceOutput:
        q_ref_arr = _as_vector(q_ref, "q_ref", Config.NUM_JOINTS)
        qd_ref_arr = _as_vector(qd_ref, "qd_ref", Config.NUM_JOINTS)
        pos_ref, quat_ref = self.backend.compute_fk(q_ref_arr)
        jac_ref = np.asarray(self.backend.compute_jacobian(q_ref_arr), dtype=np.float64)
        if jac_ref.shape != (6, Config.NUM_JOINTS):
            raise ValueError(f"reference jacobian must have shape (6, {Config.NUM_JOINTS}), got {jac_ref.shape}")
        twist_ref = jac_ref @ qd_ref_arr
        return self.compute(q, qd, pos_ref, quat_ref, twist_ref, q_null_ref=q_null_ref)

    def _nullspace_projector(self, jacobian: np.ndarray) -> np.ndarray:
        damping2 = max(self.nullspace_damping, 1e-8) ** 2
        jj_t = jacobian @ jacobian.T
        try:
            j_pinv = jacobian.T @ np.linalg.solve(jj_t + damping2 * np.eye(6), np.eye(6))
        except np.linalg.LinAlgError:
            j_pinv = np.linalg.pinv(jacobian)
        return np.eye(Config.NUM_JOINTS) - j_pinv @ jacobian


def _quat_mul_xyzw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.array(
        [
            a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
            a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
            a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
        ],
        dtype=np.float64,
    )
