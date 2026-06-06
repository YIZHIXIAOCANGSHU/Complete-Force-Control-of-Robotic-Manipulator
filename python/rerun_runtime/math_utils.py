"""Math helpers for Rerun visualization."""

from __future__ import annotations

import math

import numpy as np

from config import Config
from rerun_runtime.constants import POSE_NAME_MAP, POSITION_DISPLAY_SCALE


def _as_arm_array(values: np.ndarray, width: int) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.shape == (width,):
        return array.reshape(1, width)
    if array.shape == (Config.NUM_ARMS * width,):
        return array.reshape(Config.NUM_ARMS, width)
    if array.shape == (Config.NUM_ARMS, width):
        return array
    raise ValueError(f"expected shape ({width},) or ({Config.NUM_ARMS}, {width}), got {array.shape}")


def _torque_utilization(tau_values: np.ndarray, torque_limits: np.ndarray) -> float:
    limits = np.asarray(torque_limits, dtype=np.float64)
    safe_limits = np.where(np.abs(limits) > 1e-12, np.abs(limits), 1.0)
    ratios = np.abs(np.asarray(tau_values, dtype=np.float64)) / safe_limits
    return float(np.max(ratios)) if ratios.size else 0.0


def _safe_pose_name(name: str) -> str:
    return POSE_NAME_MAP.get(name, f"pose_{hash(name) % 10000}")


def _position_to_display_units(position: np.ndarray) -> np.ndarray:
    return np.asarray(position, dtype=np.float64) * POSITION_DISPLAY_SCALE


def quaternion_to_euler(w, x, y, z):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def quat_to_euler(quat):
    return quaternion_to_euler(quat[0], quat[1], quat[2], quat[3])


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def compute_rotation_error(quat_actual: np.ndarray, quat_desired: np.ndarray) -> np.ndarray:
    errors = np.zeros((len(quat_actual), 3))
    for i in range(len(quat_actual)):
        q_act = quat_actual[i]
        q_des = quat_desired[i]
        q_des_inv = np.array([q_des[0], -q_des[1], -q_des[2], -q_des[3]])
        q_err = quaternion_multiply(q_act, q_des_inv)
        if q_err[0] < 0:
            q_err = -q_err
        errors[i] = quat_to_euler(q_err)
    return np.degrees(errors)


def compute_rotation_error_single(quat_actual: np.ndarray, quat_desired: np.ndarray) -> np.ndarray:
    q_des_inv = np.array([quat_desired[0], -quat_desired[1], -quat_desired[2], -quat_desired[3]])
    q_err = quaternion_multiply(quat_actual, q_des_inv)
    if q_err[0] < 0:
        q_err = -q_err
    return np.rad2deg(quat_to_euler(q_err))
