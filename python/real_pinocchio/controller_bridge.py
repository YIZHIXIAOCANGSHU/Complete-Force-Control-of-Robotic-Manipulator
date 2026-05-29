"""Pinocchio real-mode controller bridge with the C bridge output contract."""

from __future__ import annotations

import math
import os
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

import numpy as np

from config import Config


ARM_JOINTS = Config.ARM_JOINTS
NUM_ARMS = Config.NUM_ARMS
NUM_JOINTS = Config.NUM_JOINTS
PINOCCHIO_EE_FRAMES = ("ArmL07Output_Link", "ArmR07Output_Link")


@dataclass(frozen=True)
class PinocchioRealControllerOutput:
    status: int
    tau: np.ndarray
    ee_pos: np.ndarray
    ee_quat: np.ndarray
    traj_t: float
    step_count: int


def _normalize_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if not math.isfinite(n) or n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def _quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.array(
        [
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
        ],
        dtype=np.float64,
    )


def _quat_conj_wxyz(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_error_axis_angle(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    qt = _normalize_quat_wxyz(target)
    qc = _normalize_quat_wxyz(current)
    qe = _quat_mul_wxyz(qt, _quat_conj_wxyz(qc))
    if qe[0] < 0.0:
        qe = -qe
    s = float(np.linalg.norm(qe[1:]))
    if s < 1e-9:
        return 2.0 * qe[1:]
    angle = 2.0 * math.atan2(s, float(qe[0]))
    return (angle / s) * qe[1:]


def _quat_slerp_wxyz(start: np.ndarray, end: np.ndarray, ratio: float) -> np.ndarray:
    q1 = _normalize_quat_wxyz(start)
    q2 = _normalize_quat_wxyz(end)
    t = min(1.0, max(0.0, float(ratio)))
    dot = float(np.dot(q1, q2))
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    if dot > 0.9995:
        return _normalize_quat_wxyz(q1 + t * (q2 - q1))
    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return _normalize_quat_wxyz((s0 * q1) + (s1 * q2))


def _rotmat_to_quat_wxyz(pin_module: Any, rotation: np.ndarray) -> np.ndarray:
    quat = pin_module.Quaternion(rotation)
    return _normalize_quat_wxyz(np.array([quat.w, quat.x, quat.y, quat.z], dtype=np.float64))


def _quat_wxyz_to_rotmat(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quat_wxyz(quat)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _rotz(angle: float) -> np.ndarray:
    c, s = math.cos(float(angle)), math.sin(float(angle))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _fallback_target_frame_rotation(body_q: np.ndarray | None) -> np.ndarray:
    """Match the Body0422 dynamic target frame convention used by C control."""
    if body_q is None:
        return np.eye(3, dtype=np.float64)
    q = np.asarray(body_q, dtype=np.float64).reshape(Config.NUM_BODY_JOINTS)
    cx, sx = math.cos(-math.pi / 2.0), math.sin(-math.pi / 2.0)
    r_zero = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
    r_current = r_zero @ _rotz(q[0]) @ _rotz(q[1]) @ _rotz(q[2])
    return r_current @ r_zero.T


def _normalize_angle(angle: np.ndarray) -> np.ndarray:
    return (np.asarray(angle, dtype=np.float64) + math.pi) % (2.0 * math.pi) - math.pi


def _ramp_from_deadband(value: float, deadband: float, full_scale: float) -> float:
    v = float(value)
    if v <= deadband:
        return 0.0
    if v >= full_scale:
        return 1.0
    span = max(1e-12, full_scale - deadband)
    return (v - deadband) / span


class LinearPathReference:
    def __init__(self) -> None:
        self.valid = False
        self.start_pos = np.zeros(3, dtype=np.float64)
        self.end_pos = np.zeros(3, dtype=np.float64)
        self.start_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.end_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.path_start_t = 0.0
        self.distance = 0.0
        self.direction = np.zeros(3, dtype=np.float64)
        self.v_max = 0.0
        self.accel = float(Config.TRAJ_PLAN_ACCEL)
        self.t_accel = 0.0
        self.t_cruise = 0.0
        self.d_accel = 0.0
        self.total_time = 0.02

    def target_changed(self, target_pos: np.ndarray, target_quat: np.ndarray) -> bool:
        if not self.valid:
            return True
        if np.any(np.abs(np.asarray(target_pos, dtype=np.float64).reshape(3) - self.end_pos) > 1e-9):
            return True
        dot = float(np.dot(_normalize_quat_wxyz(target_quat), self.end_quat))
        return abs(abs(dot) - 1.0) > 1e-9

    def initialize(
        self,
        current_pos: np.ndarray,
        current_quat: np.ndarray,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        traj_t: float,
    ) -> None:
        self.start_pos[:] = np.asarray(current_pos, dtype=np.float64).reshape(3)
        self.end_pos[:] = np.asarray(target_pos, dtype=np.float64).reshape(3)
        self.start_quat[:] = _normalize_quat_wxyz(current_quat)
        self.end_quat[:] = _normalize_quat_wxyz(target_quat)
        self.path_start_t = float(traj_t)
        delta = self.end_pos - self.start_pos
        self.distance = float(np.linalg.norm(delta))
        self.accel = max(1e-9, float(Config.TRAJ_PLAN_ACCEL))
        speed = max(0.0, float(Config.TRAJ_PLAN_SPEED))
        if self.distance < 1e-6:
            self.direction[:] = 0.0
            self.v_max = 0.0
            self.t_accel = 0.0
            self.t_cruise = 0.0
            self.d_accel = 0.0
            self.total_time = 0.02
        else:
            self.direction[:] = delta / self.distance
            d_accel_ideal = 0.5 * speed * speed / self.accel
            if 2.0 * d_accel_ideal > self.distance:
                self.v_max = math.sqrt(self.accel * self.distance)
                self.t_accel = self.v_max / self.accel
                self.d_accel = self.distance / 2.0
                self.t_cruise = 0.0
            else:
                self.v_max = speed
                self.t_accel = speed / self.accel
                self.d_accel = d_accel_ideal
                self.t_cruise = (self.distance - 2.0 * d_accel_ideal) / max(speed, 1e-9)
            self.total_time = 2.0 * self.t_accel + self.t_cruise
        self.valid = True

    def evaluate(self, traj_t: float) -> tuple[np.ndarray, np.ndarray]:
        t = max(0.0, float(traj_t) - self.path_start_t)
        if self.distance < 1e-6:
            ratio = min(1.0, t / max(self.total_time, 1e-9))
            return self.end_pos.copy(), _quat_slerp_wxyz(self.start_quat, self.end_quat, ratio)
        if t >= self.total_time:
            return self.end_pos.copy(), self.end_quat.copy()
        if t < self.t_accel:
            s = 0.5 * self.accel * t * t
        elif t < self.t_accel + self.t_cruise:
            dt = t - self.t_accel
            s = self.d_accel + self.v_max * dt
        else:
            dt = t - self.t_accel - self.t_cruise
            s = self.d_accel + self.v_max * self.t_cruise + self.v_max * dt - 0.5 * self.accel * dt * dt
        s = min(self.distance, max(0.0, s))
        ratio = min(1.0, s / max(self.distance, 1e-9))
        return self.start_pos + s * self.direction, _quat_slerp_wxyz(self.start_quat, self.end_quat, ratio)


class ZeroArmTauBackend:
    def compute(self, q, qd, target_pos, target_quat, body_q=None) -> dict[str, np.ndarray | int]:
        _ = q, qd, target_pos, target_quat, body_q
        return {
            "status": 0,
            "tau": np.zeros(ARM_JOINTS, dtype=np.float64),
            "ee_pos": np.zeros(3, dtype=np.float64),
            "ee_quat": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        }

    def compute_fk(self, q, body_q=None) -> tuple[np.ndarray, np.ndarray]:
        _ = q, body_q
        return (
            np.zeros(3, dtype=np.float64),
            np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        )

    def close(self) -> None:
        pass


class PinocchioArmTauBackend:
    """Single-arm pure-tau Cartesian PD backend built on the full dual-arm URDF."""

    def __init__(self, arm: int) -> None:
        self.arm = int(arm)
        self._pin = self._import_pinocchio()
        self._urdf_path = self._prepare_urdf_for_pinocchio()
        self._model = self._pin.buildModelFromUrdf(self._urdf_path)
        self._data = self._model.createData()
        self._q_neutral = self._pin.neutral(self._model)
        self._joint_names = list(Config.LEFT_JOINT_NAMES if arm == Config.LEFT_ARM else Config.RIGHT_JOINT_NAMES)
        self._body_joint_names = list(Config.BODY_JOINT_NAMES)
        self._ee_frame_name = PINOCCHIO_EE_FRAMES[arm]
        self._tcp_offset = np.asarray(Config.TCP_OFFSETS[arm], dtype=np.float64)
        self._tcp_frame_quat = np.asarray(Config.TCP_FRAME_QUATS[arm], dtype=np.float64)
        self._target_frame_origin = np.asarray(Config.TARGET_FRAME_ORIGIN_BASE_ZERO, dtype=np.float64)
        self._torque_limits = np.asarray(
            Config.TORQUE_LIMITS[arm * ARM_JOINTS : (arm + 1) * ARM_JOINTS],
            dtype=np.float64,
        )
        self._joint_limits = np.asarray(
            Config.JOINT_LIMITS_RAD[arm * ARM_JOINTS : (arm + 1) * ARM_JOINTS],
            dtype=np.float64,
        )
        if not self._model.existFrame(self._ee_frame_name):
            raise ValueError(f"Pinocchio frame not found: {self._ee_frame_name}")
        self._ee_frame_id = self._model.getFrameId(self._ee_frame_name, self._pin.FrameType.BODY)
        self._target_frame_id = (
            self._model.getFrameId(Config.TARGET_FRAME_BODY_NAME, self._pin.FrameType.BODY)
            if self._model.existFrame(Config.TARGET_FRAME_BODY_NAME, self._pin.FrameType.BODY)
            else -1
        )
        self._target_frame_rot_zero = self._compute_target_frame_rot_zero()
        self._joint_ids = [self._require_joint(name) for name in self._joint_names]
        self._body_joint_ids = [self._require_joint(name) for name in self._body_joint_names]
        self._step_count = 0

    @staticmethod
    def _import_pinocchio():
        try:
            import pinocchio as pin  # type: ignore
        except Exception as exc:
            raise RuntimeError(
                "Pinocchio backend selected, but importing pinocchio failed. "
                "Use a Python environment with a Pinocchio build compatible with the installed NumPy."
            ) from exc
        return pin

    @staticmethod
    def _prepare_urdf_for_pinocchio() -> str:
        """Return a URDF path whose duplicate joint names match the MuJoCo config."""
        source_path = Path(Config.URDF_PATH)
        root = ET.parse(source_path).getroot()
        seen: dict[str, int] = {}
        changed = False
        for joint in root.findall(".//joint"):
            name = joint.attrib.get("name")
            if not name:
                continue
            seen[name] = seen.get(name, 0) + 1
            if seen[name] > 1:
                joint.attrib["name"] = f"{name}_duplicate_{seen[name]}"
                changed = True
        if not changed:
            return str(source_path)
        with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8") as handle:
            ET.ElementTree(root).write(handle, encoding="unicode", xml_declaration=True)
            return handle.name

    def _require_joint(self, name: str) -> int:
        if not self._model.existJointName(name):
            raise ValueError(f"Pinocchio joint not found in URDF: {name}")
        return int(self._model.getJointId(name))

    def _make_full_state(
        self,
        q_arm: np.ndarray,
        qd_arm: np.ndarray | None = None,
        body_q: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        q_full = np.asarray(self._q_neutral, dtype=np.float64).copy()
        v_full = np.zeros(self._model.nv, dtype=np.float64)
        if body_q is not None:
            body_values = np.asarray(body_q, dtype=np.float64).reshape(Config.NUM_BODY_JOINTS)
            for value, joint_id in zip(body_values, self._body_joint_ids):
                joint = self._model.joints[joint_id]
                q_full[joint.idx_q] = float(value)
        q_values = np.asarray(q_arm, dtype=np.float64).reshape(ARM_JOINTS)
        qd_values = np.zeros(ARM_JOINTS, dtype=np.float64) if qd_arm is None else np.asarray(qd_arm, dtype=np.float64).reshape(ARM_JOINTS)
        for index, joint_id in enumerate(self._joint_ids):
            joint = self._model.joints[joint_id]
            q_full[joint.idx_q] = float(q_values[index])
            v_full[joint.idx_v] = float(qd_values[index])
        return q_full, v_full

    def _compute_target_frame_rot_zero(self) -> np.ndarray:
        if self._target_frame_id < 0:
            return np.eye(3, dtype=np.float64)
        q_full = np.asarray(self._q_neutral, dtype=np.float64).copy()
        self._pin.forwardKinematics(self._model, self._data, q_full)
        self._pin.updateFramePlacements(self._model, self._data)
        return np.asarray(self._data.oMf[self._target_frame_id].rotation, dtype=np.float64).copy()

    def _target_frame_transform(self, body_q: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        if self._target_frame_id >= 0:
            placement = self._data.oMf[self._target_frame_id]
            origin = np.asarray(placement.translation, dtype=np.float64)
            rot = np.asarray(placement.rotation @ self._target_frame_rot_zero.T, dtype=np.float64)
            return origin, rot
        return self._target_frame_origin, _fallback_target_frame_rotation(body_q)

    def compute_fk(self, q: np.ndarray, body_q: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        q_full, _ = self._make_full_state(q, body_q=body_q)
        self._pin.forwardKinematics(self._model, self._data, q_full)
        self._pin.updateFramePlacements(self._model, self._data)
        placement = self._data.oMf[self._ee_frame_id]
        target_origin, target_rot = self._target_frame_transform(body_q)
        pos_base = placement.translation + placement.rotation @ self._tcp_offset
        pos = (pos_base - target_origin) @ target_rot
        tcp_rot = placement.rotation @ _quat_wxyz_to_rotmat(self._tcp_frame_quat)
        quat = _rotmat_to_quat_wxyz(self._pin, target_rot.T @ tcp_rot)
        return np.asarray(pos, dtype=np.float64), quat

    def _compute_jacobian(self, q: np.ndarray, body_q: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        q_full, v_full = self._make_full_state(q, np.zeros(ARM_JOINTS), body_q)
        self._pin.computeJointJacobians(self._model, self._data, q_full)
        self._pin.updateFramePlacements(self._model, self._data)
        full_j = self._pin.getFrameJacobian(
            self._model,
            self._data,
            self._ee_frame_id,
            self._pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        jac = np.zeros((6, ARM_JOINTS), dtype=np.float64)
        placement = self._data.oMf[self._ee_frame_id]
        delta = placement.rotation @ self._tcp_offset
        _, target_rot = self._target_frame_transform(body_q)
        for index, joint_id in enumerate(self._joint_ids):
            column = full_j[:, self._model.joints[joint_id].idx_v].copy()
            column[:3] += np.cross(column[3:], delta)
            column[:3] = target_rot.T @ column[:3]
            column[3:] = target_rot.T @ column[3:]
            jac[:, index] = column
        return q_full, v_full, jac

    def _safety_status(self, q: np.ndarray, qd: np.ndarray) -> int:
        if not (np.isfinite(q).all() and np.isfinite(qd).all()):
            return -1
        span = self._joint_limits[:, 1] - self._joint_limits[:, 0]
        inset = float(Config.CONTROL_JOINT_LIMIT_INSET_RATIO) * span
        safe_min = self._joint_limits[:, 0] + inset
        safe_max = self._joint_limits[:, 1] - inset
        if np.any(q < safe_min) or np.any(q > safe_max):
            return -1
        if np.any(np.abs(qd) > float(Config.JOINT_VEL_LIMIT)):
            return -1
        return 0

    def compute(self, q, qd, target_pos, target_quat, body_q=None) -> dict[str, np.ndarray | int]:
        q_arm = np.asarray(q, dtype=np.float64).reshape(ARM_JOINTS)
        qd_arm = np.asarray(qd, dtype=np.float64).reshape(ARM_JOINTS)
        start = time.perf_counter()
        status = self._safety_status(q_arm, qd_arm)
        ee_pos, ee_quat = self.compute_fk(q_arm, body_q)
        if status < 0:
            return {"status": status, "tau": np.zeros(ARM_JOINTS), "ee_pos": ee_pos, "ee_quat": ee_quat}

        q_full, v_full, jac = self._compute_jacobian(q_arm, body_q)
        tau_bias_full = self._pin.nonLinearEffects(self._model, self._data, q_full, v_full)
        tau_bias = np.array(
            [tau_bias_full[self._model.joints[joint_id].idx_v] for joint_id in self._joint_ids],
            dtype=np.float64,
        )

        target_pos = np.asarray(target_pos, dtype=np.float64).reshape(3)
        target_quat = _normalize_quat_wxyz(np.asarray(target_quat, dtype=np.float64).reshape(4))
        pos_err = target_pos - ee_pos
        ori_err = _quat_error_axis_angle(target_quat, ee_quat)
        ee_vel = jac @ qd_arm
        wrench = np.zeros(6, dtype=np.float64)
        wrench[:3] = np.array([Config.KP_CART_X, Config.KP_CART_Y, Config.KP_CART_Z]) * pos_err
        wrench[:3] -= np.array([Config.KD_CART_X, Config.KD_CART_Y, Config.KD_CART_Z]) * ee_vel[:3]
        wrench[3:] = np.array([Config.KP_CART_ROLL, Config.KP_CART_PITCH, Config.KP_CART_YAW]) * ori_err
        wrench[3:] -= np.array([Config.KD_CART_ROLL, Config.KD_CART_PITCH, Config.KD_CART_YAW]) * ee_vel[3:]
        tau_task = jac.T @ wrench
        jj_t = jac @ jac.T
        inv = np.linalg.pinv(jj_t + (0.05 * 0.05) * np.eye(6, dtype=np.float64))
        j_pinv = jac.T @ inv
        dq_ik = j_pinv @ np.concatenate([pos_err, ori_err])
        q_ref = (1.0 - float(Config.POSTURE_ALPHA)) * (q_arm + dq_ik)
        q_ref += float(Config.POSTURE_ALPHA) * np.asarray(Config.Q_PREFERRED, dtype=np.float64)
        tau_null = np.asarray(Config.KP_JOINT, dtype=np.float64) * _normalize_angle(q_ref - q_arm)
        tau_null -= np.asarray(Config.KD_JOINT, dtype=np.float64) * qd_arm
        null_projector = np.eye(ARM_JOINTS, dtype=np.float64) - j_pinv @ jac
        tau_null_projected = null_projector.T @ tau_null
        null_scale = _ramp_from_deadband(
            float(np.linalg.norm(pos_err)),
            float(Config.NULLSPACE_POS_DEADBAND),
            float(Config.NULLSPACE_POS_FULL_SCALE),
        )
        ori_null_scale = _ramp_from_deadband(
            float(np.linalg.norm(ori_err)),
            float(Config.NULLSPACE_ORI_DEADBAND),
            float(Config.NULLSPACE_ORI_FULL_SCALE),
        )
        null_scale = max(null_scale, ori_null_scale)
        tau_null_projected = np.clip(
            null_scale * tau_null_projected,
            -float(Config.NULLSPACE_TORQUE_LIMIT),
            float(Config.NULLSPACE_TORQUE_LIMIT),
        )
        tau = (
            float(Config.W_CARTESIAN) * tau_task
            + float(Config.W_JOINT) * tau_null_projected
            + tau_bias
        )
        tau = np.clip(tau, -self._torque_limits, self._torque_limits)
        if not np.isfinite(tau).all():
            status = -1
            tau = np.zeros(ARM_JOINTS, dtype=np.float64)
        self._step_count += 1
        _ = time.perf_counter() - start
        return {"status": status, "tau": tau, "ee_pos": ee_pos, "ee_quat": ee_quat}

    def close(self) -> None:
        urdf_path = getattr(self, "_urdf_path", None)
        if urdf_path and Path(urdf_path) != Path(Config.URDF_PATH):
            try:
                os.unlink(urdf_path)
            except OSError:
                pass


class PinocchioRealControllerBridge:
    """Drop-in bridge for real.runtime using Pinocchio and pure tau output."""

    def __init__(
        self,
        *,
        active_arm_mask: int = (1 << Config.LEFT_ARM) | (1 << Config.RIGHT_ARM),
        left_backend=None,
        right_backend=None,
    ) -> None:
        normalized_mask = int(active_arm_mask)
        def make_backend(arm: int, supplied):
            if supplied is not None:
                return supplied
            if (normalized_mask & (1 << arm)) == 0:
                return ZeroArmTauBackend()
            return PinocchioArmTauBackend(arm)

        self._backends = {
            Config.LEFT_ARM: make_backend(Config.LEFT_ARM, left_backend),
            Config.RIGHT_ARM: make_backend(Config.RIGHT_ARM, right_backend),
        }
        self._step_count = 0
        self._traj_t = 0.0
        self._paths = {arm: LinearPathReference() for arm in range(NUM_ARMS)}

    def compute(
        self,
        active_arm_mask: int,
        elapsed_s: float,
        q: np.ndarray,
        qd: np.ndarray,
        body_q: np.ndarray,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
    ) -> PinocchioRealControllerOutput:
        step_s = max(0.0, float(elapsed_s)) if math.isfinite(float(elapsed_s)) else 0.0
        q_arr = np.asarray(q, dtype=np.float64).reshape(NUM_JOINTS)
        qd_arr = np.asarray(qd, dtype=np.float64).reshape(NUM_JOINTS)
        body_arr = np.asarray(body_q, dtype=np.float64).reshape(Config.NUM_BODY_JOINTS)
        target_pos_arr = np.asarray(target_pos, dtype=np.float64).reshape(NUM_ARMS, 3)
        target_quat_arr = np.asarray(target_quat, dtype=np.float64).reshape(NUM_ARMS, 4)
        tau = np.zeros(NUM_JOINTS, dtype=np.float64)
        ee_pos = np.zeros((NUM_ARMS, 3), dtype=np.float64)
        ee_quat = np.tile([1.0, 0.0, 0.0, 0.0], (NUM_ARMS, 1))
        status = 0

        for arm in range(NUM_ARMS):
            offset = arm * ARM_JOINTS
            if (int(active_arm_mask) & (1 << arm)) == 0:
                continue
            q_arm = q_arr[offset : offset + ARM_JOINTS]
            qd_arm = qd_arr[offset : offset + ARM_JOINTS]
            if not (np.isfinite(q_arm).all() and np.isfinite(qd_arm).all()):
                status = -1
                tau[:] = 0.0
                break
            fk_pos, fk_quat = self._backends[arm].compute_fk(q_arm, body_arr)
            if self._paths[arm].target_changed(target_pos_arr[arm], target_quat_arr[arm]):
                self._paths[arm].initialize(
                    fk_pos,
                    fk_quat,
                    target_pos_arr[arm],
                    target_quat_arr[arm],
                    self._traj_t,
                )
            ref_pos, ref_quat = self._paths[arm].evaluate(self._traj_t + step_s)
            result = self._backends[arm].compute(
                q_arm,
                qd_arm,
                ref_pos,
                ref_quat,
                body_arr,
            )
            ee_pos[arm] = np.asarray(result["ee_pos"], dtype=np.float64).reshape(3)
            ee_quat[arm] = _normalize_quat_wxyz(np.asarray(result["ee_quat"], dtype=np.float64).reshape(4))
            if int(result["status"]) < 0:
                status = int(result["status"])
                tau[:] = 0.0
                break
            tau[offset : offset + ARM_JOINTS] = np.asarray(result["tau"], dtype=np.float64).reshape(ARM_JOINTS)

        self._step_count += 1
        self._traj_t += step_s
        return PinocchioRealControllerOutput(
            status=int(status),
            tau=tau,
            ee_pos=ee_pos,
            ee_quat=ee_quat,
            traj_t=float(self._traj_t),
            step_count=int(self._step_count),
        )

    def compute_fk(
        self,
        active_arm_mask: int,
        q: np.ndarray,
        body_q: np.ndarray | None = None,
    ) -> PinocchioRealControllerOutput:
        q_arr = np.asarray(q, dtype=np.float64).reshape(NUM_JOINTS)
        body_arr = np.zeros(Config.NUM_BODY_JOINTS, dtype=np.float64) if body_q is None else np.asarray(body_q, dtype=np.float64).reshape(Config.NUM_BODY_JOINTS)
        ee_pos = np.zeros((NUM_ARMS, 3), dtype=np.float64)
        ee_quat = np.tile([1.0, 0.0, 0.0, 0.0], (NUM_ARMS, 1))
        for arm in range(NUM_ARMS):
            if (int(active_arm_mask) & (1 << arm)) == 0:
                continue
            offset = arm * ARM_JOINTS
            pos, quat = self._backends[arm].compute_fk(q_arr[offset : offset + ARM_JOINTS], body_arr)
            ee_pos[arm] = np.asarray(pos, dtype=np.float64).reshape(3)
            ee_quat[arm] = _normalize_quat_wxyz(np.asarray(quat, dtype=np.float64).reshape(4))
        return PinocchioRealControllerOutput(
            status=0,
            tau=np.zeros(NUM_JOINTS, dtype=np.float64),
            ee_pos=ee_pos,
            ee_quat=ee_quat,
            traj_t=float(self._traj_t),
            step_count=int(self._step_count),
        )

    def close(self) -> None:
        for backend in self._backends.values():
            close = getattr(backend, "close", None)
            if callable(close):
                close()
