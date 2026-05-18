"""Pinocchio-based kinematics/dynamics/control computation.

Key frame conventions:
- URDF has a world_to_base transform (1m Z offset + ~120 deg rotation).
- The C RBDL model starts at identity base → C computations are in "body frame".
- Pinocchio FK/Jacobian are in world frame; we convert to body frame by
  applying the inverse of the world_to_base placement.
- Joint 3 (ArmLfourth_Joint) has nq=2 (cos/sin for continuous joint) while
  all other joints have nq=1, so q7→q8 conversion is needed.
"""

from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

_CONDA_SITE = "/home/luwen/miniconda3/lib/python3.10/site-packages"

if _CONDA_SITE not in sys.path:
    sys.path.insert(0, _CONDA_SITE)

_EXCLUDE_MARKERS = ("cmeel",)
sys.path = [p for p in sys.path if not any(m in p for m in _EXCLUDE_MARKERS)]

import pinocchio as pin  # noqa: E402


# =========================================================================
# Quaternion / rotation utilities
# =========================================================================


def rotmat_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix → quaternion [w, x, y, z]."""
    quat = pin.Quaternion(R)
    return np.array([quat.w, quat.x, quat.y, quat.z], dtype=np.float64)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# =========================================================================
# q7 ↔ q8 conversion (handles joint 3 cos/sin)
# =========================================================================

def q7_to_q8(q7: np.ndarray) -> np.ndarray:
    q7 = np.asarray(q7, dtype=np.float64)
    q8 = np.zeros(8, dtype=np.float64)
    q8[0] = q7[0]
    q8[1] = q7[1]
    q8[2] = math.cos(q7[2])
    q8[3] = math.sin(q7[2])
    q8[4] = q7[3]
    q8[5] = q7[4]
    q8[6] = q7[5]
    q8[7] = q7[6]
    return q8


# =========================================================================
# PinocchioGravityBackend
# =========================================================================


class PinocchioGravityBackend:
    """Pinocchio-based kinematics/dynamics/control computation backend."""

    def __init__(
        self,
        urdf_path: str,
        ee_frame_name: str = "ArmLseventh_Link",
        tcp_offset: np.ndarray = np.array([0.0, 0.07, -0.03]),
        joint_kp: Optional[List[float]] = None,
        joint_kd: Optional[List[float]] = None,
        torque_limits: Optional[List[float]] = None,
        joint_pos_min: Optional[List[float]] = None,
        joint_pos_max: Optional[List[float]] = None,
        joint_vel_limit: float = 5.0,
        ik_max_iterations: int = 80,
        ik_tol_pos: float = 0.002,
        ik_tol_ori: float = 0.01,
        ik_max_step: float = 0.2,
        ik_damping: float = 0.1,
        traj_plan_step_m: float = 0.0002,
        traj_plan_speed: float = 0.1,
        traj_plan_accel: float = 1.25,
        kalman_q_vel: float = 0.001,
        kalman_r_vel: float = 0.1,
    ) -> None:
        # --- Load URDF model ---
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")
        self._model = pin.buildModelFromUrdf(urdf_path)
        self._data = self._model.createData()

        if not self._model.existFrame(ee_frame_name):
            available = [self._model.frames[i].name for i in range(self._model.nframes)]
            raise ValueError(f"Frame '{ee_frame_name}' not found. Available: {available}")
        self._ee_frame_id = self._model.getFrameId(ee_frame_name)

        if not self._model.existFrame("world_to_base"):
            raise ValueError("world_to_base frame not found in URDF")
        self._wtb_frame_id = self._model.getFrameId("world_to_base")

        # Validate DOF
        if self._model.nv != 7:
            raise ValueError(f"Expected nv=7, got nv={self._model.nv}")

        self._tcp_offset = np.asarray(tcp_offset, dtype=np.float64)

        # Compute cached R_wtb^T for body-frame conversions
        q_neutral = pin.neutral(self._model)
        pin.forwardKinematics(self._model, self._data, q_neutral)
        pin.updateFramePlacements(self._model, self._data)
        self._R_wtb_T = self._data.oMf[self._wtb_frame_id].rotation.T.copy()

        # --- Control gains ---
        self._joint_kp = np.array(
            joint_kp or [10.0, 10.0, 5.0, 5.0, 0.2, 0.2, 0.2], dtype=np.float64,
        )
        self._joint_kd = np.array(
            joint_kd or [0.316, 0.316, 0.158, 0.158, 0.032, 0.032, 0.032],
            dtype=np.float64,
        )
        self._torque_limits = np.array(
            torque_limits or [40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 9.0], dtype=np.float64,
        )

        # --- Joint limits ---
        self._joint_pos_min = np.array(
            joint_pos_min
            or [
                math.radians(-89.971835),
                math.radians(-89.954374),
                math.radians(-68.754935),
                math.radians(-119.748454),
                math.radians(-45.836624),
                math.radians(-61.306275),
                math.radians(-61.306275),
            ],
            dtype=np.float64,
        )
        self._joint_pos_max = np.array(
            joint_pos_max
            or [
                math.radians(89.971835),
                math.radians(20.587610),
                math.radians(45.836624),
                math.radians(119.954374),
                math.radians(45.836624),
                math.radians(45.263666),
                math.radians(61.306275),
            ],
            dtype=np.float64,
        )
        self._joint_vel_limit = joint_vel_limit

        # --- IK parameters ---
        self._ik_max_iterations = ik_max_iterations
        self._ik_tol_pos = ik_tol_pos
        self._ik_tol_ori = ik_tol_ori
        self._ik_max_step = ik_max_step
        self._ik_damping = ik_damping

        # --- Trajectory parameters ---
        self._traj_plan_step_m = traj_plan_step_m
        self._traj_plan_speed = traj_plan_speed
        self._traj_plan_accel = traj_plan_accel

        # --- Velocity filters ---
        self._vel_filters = [
            _KalmanFilter1D(kalman_q_vel, kalman_r_vel) for _ in range(7)
        ]

        # --- Controller state ---
        self._reset_controller_state()

    def _reset_controller_state(self) -> None:
        self._path_s: float = 0.0
        self._step_count: int = 0
        self._path_valid: bool = False
        self._have_last_q_ref: bool = False
        self._latched_target_pos = np.zeros(3)
        self._latched_target_quat = np.zeros(4)
        self._last_q_ref = np.zeros(7)
        self._path_start_pos = np.zeros(3)
        self._path_start_quat = np.zeros(4)
        self._path_end_pos = np.zeros(3)
        self._path_end_quat = np.zeros(4)
        self._path_L: float = 0.0
        self._path_dir = np.zeros(3)
        self._path_v_max: float = 0.0
        self._path_a: float = 0.0
        self._path_t_a: float = 0.0
        self._path_t_c: float = 0.0
        self._path_d_a: float = 0.0

    # ------------------------------------------------------------------
    # FK (body frame, with TCP offset, quaternion [w,x,y,z])
    # ------------------------------------------------------------------

    def compute_fk(self, q: List[float]) -> Tuple[List[float], List[float]]:
        q7 = np.asarray(q, dtype=np.float64)
        q8 = q7_to_q8(q7)

        pin.forwardKinematics(self._model, self._data, q8)
        pin.updateFramePlacements(self._model, self._data)

        pl_ee_world = self._data.oMf[self._ee_frame_id]
        pl_wtb = self._data.oMf[self._wtb_frame_id]
        pl_ee_body = pl_wtb.actInv(pl_ee_world)

        pos = pl_ee_body.translation.copy()
        R = pl_ee_body.rotation

        tcp_delta = R @ self._tcp_offset
        pos += tcp_delta

        quat_wxyz = rotmat_to_quat_wxyz(R)
        return pos.tolist(), quat_wxyz.tolist()

    # ------------------------------------------------------------------
    # Jacobian (body frame, 6×7 column-major, matches C rbdl_calc_jacobian)
    # ------------------------------------------------------------------

    def compute_jacobian(self, q: List[float]) -> np.ndarray:
        q7 = np.asarray(q, dtype=np.float64)
        q8 = q7_to_q8(q7)

        pin.computeJointJacobians(self._model, self._data, q8)
        pin.updateFramePlacements(self._model, self._data)

        # LOCAL_WORLD_ALIGNED gives velocity at EE in world-aligned axes
        J_lwa = pin.getFrameJacobian(
            self._model, self._data, self._ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        J_body = np.zeros((6, 7), dtype=np.float64)
        for j in range(7):
            J_body[:3, j] = self._R_wtb_T @ J_lwa[:3, j]
            J_body[3:, j] = self._R_wtb_T @ J_lwa[3:, j]
        return J_body

    # ------------------------------------------------------------------
    # Dynamics (world frame → Pinocchio handles the gravity direction
    #           automatically via model.gravity set from URDF default)
    # ------------------------------------------------------------------

    def compute_gravity(self, q: List[float]) -> np.ndarray:
        q7 = np.asarray(q, dtype=np.float64)
        q8 = q7_to_q8(q7)
        G = pin.computeGeneralizedGravity(self._model, self._data, q8)
        return G.copy()

    def compute_nonlinear_effects(self, q: List[float], qd: List[float]) -> np.ndarray:
        q7 = np.asarray(q, dtype=np.float64)
        qd7 = np.asarray(qd, dtype=np.float64)
        q8 = q7_to_q8(q7)
        nle = pin.nonLinearEffects(self._model, self._data, q8, qd7)
        return nle.copy()

    # ------------------------------------------------------------------
    # Full control step (mirrors stm_controller_step)
    # ------------------------------------------------------------------

    def compute(self, q, qd, target_pos, target_quat):
        """Full control step → MitControlOutput."""
        from robot_control.dynamics.gravity import MitControlOutput

        q7 = np.asarray(q, dtype=np.float64)
        qd7 = np.asarray(qd, dtype=np.float64)
        tgt_pos = np.asarray(target_pos, dtype=np.float64)
        tgt_quat = np.asarray(target_quat, dtype=np.float64)

        t_start = time.perf_counter()
        status = 0

        # Filter velocities
        qd_filt = np.array([self._vel_filters[i].update(qd7[i]) for i in range(7)])

        # FK
        ee_pos_arr, ee_quat_arr = self.compute_fk(q7)
        ee_pos = np.asarray(ee_pos_arr, dtype=np.float64)
        ee_quat = np.asarray(ee_quat_arr, dtype=np.float64)

        # Sanitize target
        tgt_pos_s, tgt_quat_s = self._sanitize_target(tgt_pos, tgt_quat, ee_pos, ee_quat)

        # Safety check
        if self._check_safety(q7, qd_filt) < 0:
            t_end = time.perf_counter()
            return MitControlOutput(
                tau_total=[0.0] * 7, q_ref=q7.tolist(), qd_ref=[0.0] * 7,
                kp=self._joint_kp.tolist(), kd=self._joint_kd.tolist(),
                tau_ff=[0.0] * 7,
                ee_pos=ee_pos.tolist(), ee_quat=ee_quat.tolist(),
                status=-1, path_progress=self._path_s,
                step_count=self._step_count,
                calc_time_ms=(t_end - t_start) * 1000.0,
            )

        # Target changed → restart path
        if self._target_changed(tgt_pos_s, tgt_quat_s):
            self._start_path(ee_pos, ee_quat, tgt_pos_s, tgt_quat_s)

        # Evaluate path
        if self._path_valid:
            ref_pos, ref_quat = self._evaluate_round_path()
        else:
            ref_pos, ref_quat = tgt_pos_s.copy(), tgt_quat_s.copy()

        # DLS IK
        ik_ok, q_ref = self._compute_q_ref(q7, ref_pos, ref_quat)
        if ik_ok:
            qd_ref = self._compute_qd_ref(q_ref)
        else:
            qd_ref = np.zeros(7)
            if not self._q_ref_is_safe(q_ref):
                q_ref = q7.copy()
            status = 1

        # Nonlinear effects → tau_ff
        tau_ff = self.compute_nonlinear_effects(q7, qd_filt)

        # Equivalent torque
        tau_total = self._compute_equivalent_tau(q7, qd_filt, q_ref, qd_ref, tau_ff)

        # Joint safety
        if self._check_joint_safety(q7, qd_filt, tau_total) == 0:
            self._last_q_ref = q_ref.copy()
            self._have_last_q_ref = True
        else:
            q_ref, qd_ref, tau_ff, tau_total = q7.copy(), np.zeros(7), np.zeros(7), np.zeros(7)
            status = -1

        self._step_count += 1
        t_end = time.perf_counter()

        return MitControlOutput(
            tau_total=tau_total.tolist(), q_ref=q_ref.tolist(),
            qd_ref=qd_ref.tolist(),
            kp=self._joint_kp.tolist(), kd=self._joint_kd.tolist(),
            tau_ff=tau_ff.tolist(),
            ee_pos=ee_pos.tolist(), ee_quat=ee_quat.tolist(),
            status=status, path_progress=self._path_s,
            step_count=self._step_count,
            calc_time_ms=(t_end - t_start) * 1000.0,
        )

    # ------------------------------------------------------------------
    # Safety
    # ------------------------------------------------------------------

    def _check_safety(self, q: np.ndarray, qd: np.ndarray) -> int:
        if not (np.isfinite(q).all() and np.isfinite(qd).all()):
            return -1
        for i in range(7):
            if q[i] < self._joint_pos_min[i] - 0.01:
                return -1
            if q[i] > self._joint_pos_max[i] + 0.01:
                return -1
            if abs(qd[i]) > self._joint_vel_limit:
                return -2
        return 0

    def _check_joint_safety(self, q: np.ndarray, qd: np.ndarray, tau: np.ndarray) -> int:
        if self._check_safety(q, qd) < 0 or not np.isfinite(tau).all():
            return -1
        for i in range(7):
            if abs(tau[i]) > self._torque_limits[i]:
                return -1
        return 0

    def _q_ref_is_safe(self, q_ref: np.ndarray) -> bool:
        if not np.isfinite(q_ref).all():
            return False
        for i in range(7):
            if q_ref[i] < self._joint_pos_min[i] - 0.01:
                return False
            if q_ref[i] > self._joint_pos_max[i] + 0.01:
                return False
        return True

    # ------------------------------------------------------------------
    # Target sanitization
    # ------------------------------------------------------------------

    def _sanitize_target(
        self, tgt_pos: np.ndarray, tgt_quat: np.ndarray,
        fb_pos: np.ndarray, fb_quat: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        out_pos = fb_pos.copy()
        out_quat = fb_quat.copy()
        if np.isfinite(tgt_pos).all():
            out_pos = tgt_pos.copy()
        elif self._path_valid:
            out_pos = self._latched_target_pos.copy()
        n2 = float(np.sum(tgt_quat**2))
        if n2 > 1e-12 and math.isfinite(n2):
            out_quat = tgt_quat / math.sqrt(n2)
        elif self._path_valid:
            out_quat = self._latched_target_quat.copy()
        return out_pos, out_quat

    def _target_changed(self, tgt_pos: np.ndarray, tgt_quat: np.ndarray) -> bool:
        if not self._path_valid:
            return True
        if float(np.sum((tgt_pos - self._latched_target_pos) ** 2)) > 1e-8:
            return True
        if abs(float(np.dot(tgt_quat, self._latched_target_quat))) < 0.99999:
            return True
        return False

    # ------------------------------------------------------------------
    # Linear path planner
    # ------------------------------------------------------------------

    def _start_path(self, start_pos, start_quat, end_pos, end_quat):
        self._path_start_pos = start_pos.copy()
        self._path_start_quat = start_quat.copy()
        self._path_end_pos = end_pos.copy()
        self._path_end_quat = end_quat.copy()
        self._latched_target_pos = end_pos.copy()
        self._latched_target_quat = end_quat.copy()

        delta = end_pos - start_pos
        L = float(np.linalg.norm(delta))
        self._path_L = L

        if L < 1e-6:
            self._path_dir = np.zeros(3)
            self._path_v_max = 0.0
            self._path_a = self._traj_plan_accel
            self._path_t_a = 0.0
            self._path_t_c = 0.02
            self._path_d_a = 0.0
        else:
            self._path_dir = delta / L
            s = self._traj_plan_speed
            a = self._traj_plan_accel
            d_a_ideal = 0.5 * s * s / a
            if 2.0 * d_a_ideal > L:
                v = math.sqrt(a * L)
                self._path_v_max = v
                self._path_t_a = v / a
                self._path_d_a = L / 2.0
                self._path_t_c = 0.0
            else:
                self._path_v_max = s
                self._path_t_a = s / a
                self._path_d_a = d_a_ideal
                self._path_t_c = (L - 2.0 * d_a_ideal) / s

        self._path_s = 0.0
        self._have_last_q_ref = False
        self._path_valid = True

    def _sample_path_at_s(self, s: float) -> Tuple[np.ndarray, np.ndarray]:
        if not self._path_valid or self._path_L < 1e-9:
            return self._latched_target_pos.copy(), self._latched_target_quat.copy()
        s = max(0.0, min(s, self._path_L))
        pos = self._path_start_pos + s * self._path_dir
        r = min(s / self._path_L if self._path_L > 1e-9 else 1.0, 1.0)
        quat = _quat_slerp(self._path_start_quat, self._path_end_quat, r)
        return pos, quat

    def _evaluate_round_path(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self._path_valid or self._path_L < 1e-9:
            self._path_s = self._path_L
            return self._latched_target_pos.copy(), self._latched_target_quat.copy()
        if self._path_s < self._path_L:
            self._path_s += self._traj_plan_step_m
            if self._path_s > self._path_L:
                self._path_s = self._path_L
        return self._sample_path_at_s(self._path_s)

    # ------------------------------------------------------------------
    # DLS IK
    # ------------------------------------------------------------------

    def _pose_error_6d(
        self, tgt_pos, tgt_quat, cur_pos, cur_quat,
    ) -> np.ndarray:
        e6 = np.zeros(6)
        e6[:3] = tgt_pos - cur_pos
        # quaternion to xyzw for internal computation
        qt = np.array([tgt_quat[1], tgt_quat[2], tgt_quat[3], tgt_quat[0]])
        qc = np.array([cur_quat[1], cur_quat[2], cur_quat[3], cur_quat[0]])
        q_inv = np.array([-qc[0], -qc[1], -qc[2], qc[3]])
        q_err = _quat_mul(qt, q_inv)
        if q_err[3] < 0.0:
            q_err = -q_err
        s = math.sqrt(q_err[0]**2 + q_err[1]**2 + q_err[2]**2)
        if s < 1e-6:
            e6[3:6] = 2.0 * q_err[:3]
        else:
            angle = 2.0 * math.atan2(s, q_err[3])
            e6[3:6] = (angle / s) * q_err[:3]
        return e6

    def _compute_q_ref(self, cur_q, ref_pos, ref_quat) -> Tuple[int, np.ndarray]:
        q = self._last_q_ref.copy() if self._have_last_q_ref else cur_q.copy()
        for _ in range(self._ik_max_iterations):
            fk_pos, fk_quat = self.compute_fk(q)
            e6 = self._pose_error_6d(ref_pos, ref_quat,
                                     np.asarray(fk_pos), np.asarray(fk_quat))
            if np.linalg.norm(e6[:3]) < self._ik_tol_pos and np.linalg.norm(e6[3:6]) < self._ik_tol_ori:
                return 1, q.copy()

            J = self.compute_jacobian(q)  # 6×7
            A = J @ J.T + (self._ik_damping**2) * np.eye(6)
            try:
                dq = 0.5 * np.clip(J.T @ np.linalg.solve(A, e6),
                                   -self._ik_max_step, self._ik_max_step)
            except np.linalg.LinAlgError:
                return 0, q.copy()
            q += dq
            for i in range(7):
                q[i] = normalize_angle(q[i])
            self._clamp_joints(q)
        return 0, q.copy()

    def _clamp_joints(self, q):
        q[:] = np.clip(q, self._joint_pos_min, self._joint_pos_max)

    def _compute_qd_ref(self, q_ref: np.ndarray) -> np.ndarray:
        if not self._path_valid or self._path_L < 1e-9 or self._traj_plan_speed <= 0.0:
            return np.zeros(7)
        next_s = min(self._path_s + self._traj_plan_step_m, self._path_L)
        ds = next_s - self._path_s
        if ds <= 1e-12:
            return np.zeros(7)
        dt = ds / self._traj_plan_speed
        if dt <= 1e-12 or not math.isfinite(dt):
            return np.zeros(7)
        npos, nquat = self._sample_path_at_s(next_s)
        ik_ok, q_next = self._compute_q_ref(q_ref, npos, nquat)
        if not ik_ok:
            return np.zeros(7)
        qd = np.zeros(7)
        for i in range(7):
            qd[i] = normalize_angle(q_next[i] - q_ref[i]) / dt
        return qd

    def _compute_equivalent_tau(self, q, qd, q_ref, qd_ref, tau_ff):
        tau = np.zeros(7)
        for i in range(7):
            e = normalize_angle(q_ref[i] - q[i])
            tau[i] = self._joint_kp[i] * e + self._joint_kd[i] * (qd_ref[i] - qd[i]) + tau_ff[i]
            tau[i] = max(-self._torque_limits[i], min(self._torque_limits[i], tau[i]))
        return tau

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def close(self) -> None:
        pass


# =========================================================================
# Internal utilities
# =========================================================================


class _KalmanFilter1D:
    def __init__(self, q: float, r: float) -> None:
        self._x: float = 0.0
        self._p: float = 1.0
        self._q = q
        self._r = r

    def update(self, z: float) -> float:
        self._p += self._q
        k = self._p / (self._p + self._r)
        self._x += k * (z - self._x)
        self._p *= 1.0 - k
        return self._x


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """xyzw quaternion multiplication."""
    return np.array([
        a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
        a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
        a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
        a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2],
    ])


def _quat_slerp(qa: np.ndarray, qb: np.ndarray, t: float) -> np.ndarray:
    qa, qb = np.asarray(qa, dtype=np.float64), np.asarray(qb, dtype=np.float64)
    cos_half = float(np.dot(qa, qb))
    if cos_half < 0.0:
        qb, cos_half = -qb, -cos_half
    if cos_half >= 1.0:
        return qa.copy()
    half = math.acos(cos_half)
    sin_half = math.sqrt(1.0 - cos_half**2)
    if abs(sin_half) < 1e-12:
        return 0.5 * (qa + qb)
    ra = math.sin((1.0 - t) * half) / sin_half
    rb = math.sin(t * half) / sin_half
    r = ra * qa + rb * qb
    n = float(np.linalg.norm(r))
    return r / n if n > 1e-12 else r
