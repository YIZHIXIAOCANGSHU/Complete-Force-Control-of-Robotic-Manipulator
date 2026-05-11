"""Thread-safe shared state for serial control and visualization."""

from __future__ import annotations

import threading

from config import Config


class SharedRobotState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._positions = [0.0] * Config.NUM_JOINTS
        self._velocities = [0.0] * Config.NUM_JOINTS
        self._torques = [0.0] * Config.NUM_JOINTS
        self._reported_ee_pos = [0.0] * 3
        self._reported_ee_quat = [1.0, 0.0, 0.0, 0.0]
        self._target_pos = [0.0] * 3
        self._target_quat = [1.0, 0.0, 0.0, 0.0]

    def update_joint_feedback(self, joint_idx: int, pos: float, vel: float, tor: float) -> None:
        with self._lock:
            self._positions[joint_idx] = pos
            self._velocities[joint_idx] = vel
            self._torques[joint_idx] = tor

    def set_target_pose(self, pos, quat) -> None:
        with self._lock:
            self._target_pos[0:3] = pos[0:3]
            self._target_quat[0:4] = quat[0:4]

    def get_target_pose(self):
        with self._lock:
            return list(self._target_pos), list(self._target_quat)

    def set_reported_pose(self, pos, quat) -> None:
        with self._lock:
            self._reported_ee_pos[0:3] = pos[0:3]
            self._reported_ee_quat[0:4] = quat[0:4]

    def snapshot_control_inputs(self):
        with self._lock:
            return (
                self._positions[:],
                self._velocities[:],
                self._torques[:],
                self._target_pos[:],
                self._target_quat[:],
            )

    def snapshot_viewer_state(self):
        with self._lock:
            return (
                self._positions[:],
                self._reported_ee_pos[:],
                self._reported_ee_quat[:],
            )
