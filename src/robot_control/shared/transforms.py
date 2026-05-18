"""Coordinate transforms between robot space and the MuJoCo scene."""

from __future__ import annotations

import numpy as np


def _quat_multiply(lhs, rhs) -> np.ndarray:
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float64,
    )


class RobotMujocoTransformer:
    def __init__(self) -> None:
        self._q_b2w_wxyz = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float64)
        self._q_w2b_wxyz = np.array([0.5, -0.5, -0.5, -0.5], dtype=np.float64)

    def robot_to_mujoco(self, robot_pos, robot_quat):
        mj_pos = np.array([robot_pos[2], robot_pos[0], robot_pos[1] + 1.0], dtype=np.float64)
        mj_quat = _quat_multiply(self._q_b2w_wxyz, robot_quat)
        return mj_pos, mj_quat

    def mujoco_to_robot(self, mj_pos, mj_quat):
        robot_pos = np.array([mj_pos[1], mj_pos[2] - 1.0, mj_pos[0]], dtype=np.float64)
        robot_quat = _quat_multiply(self._q_w2b_wxyz, mj_quat)
        return robot_pos, robot_quat
