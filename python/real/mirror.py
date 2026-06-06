"""Shared real-control state and MuJoCo mirror helpers."""

from __future__ import annotations

import threading

import numpy as np

from config import Config


class RealSharedState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.q = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.qd = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.tau_actual = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.tau_total = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self.ee_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        self.ee_quat = np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1))
        self.ref_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        self.ref_quat = np.zeros((Config.NUM_ARMS, 4), dtype=np.float64)
        self.target_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        self.target_quat = np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1))
        self.active_arm_mask = 0
        self.feedback_valid = False
        self.targets_initialized = False

    def update_feedback(self, q: np.ndarray, qd: np.ndarray, tau_actual: np.ndarray, active_arm_mask: int) -> None:
        with self._lock:
            self.q[:] = np.asarray(q, dtype=np.float64).reshape(Config.NUM_JOINTS)
            self.qd[:] = np.asarray(qd, dtype=np.float64).reshape(Config.NUM_JOINTS)
            self.tau_actual[:] = np.asarray(tau_actual, dtype=np.float64).reshape(Config.NUM_JOINTS)
            self.active_arm_mask = int(active_arm_mask)
            self.feedback_valid = True

    def update_control_output(
        self,
        tau_total: np.ndarray,
        ee_pos: np.ndarray,
        ee_quat: np.ndarray,
        ref_pos: np.ndarray | None = None,
        ref_quat: np.ndarray | None = None,
    ) -> None:
        with self._lock:
            self.tau_total[:] = np.asarray(tau_total, dtype=np.float64).reshape(Config.NUM_JOINTS)
            self.ee_pos[:] = np.asarray(ee_pos, dtype=np.float64).reshape(Config.NUM_ARMS, 3)
            self.ee_quat[:] = np.asarray(ee_quat, dtype=np.float64).reshape(Config.NUM_ARMS, 4)
            if ref_pos is not None:
                self.ref_pos[:] = np.asarray(ref_pos, dtype=np.float64).reshape(Config.NUM_ARMS, 3)
            if ref_quat is not None:
                self.ref_quat[:] = np.asarray(ref_quat, dtype=np.float64).reshape(Config.NUM_ARMS, 4)

    def set_targets(self, target_pos: np.ndarray, target_quat: np.ndarray) -> None:
        with self._lock:
            self.target_pos[:] = np.asarray(target_pos, dtype=np.float64).reshape(Config.NUM_ARMS, 3)
            self.target_quat[:] = np.asarray(target_quat, dtype=np.float64).reshape(Config.NUM_ARMS, 4)
            self.targets_initialized = True

    def snapshot_control_inputs(self):
        with self._lock:
            return (
                self.q.copy(),
                self.qd.copy(),
                self.tau_actual.copy(),
                self.target_pos.copy(),
                self.target_quat.copy(),
            )

    def snapshot_targets(self):
        with self._lock:
            return self.target_pos.copy(), self.target_quat.copy()

    def targets_ready(self) -> bool:
        with self._lock:
            return bool(self.targets_initialized)

    def snapshot_mirror(self):
        with self._lock:
            return (
                self.q.copy(),
                self.qd.copy(),
                self.ref_pos.copy(),
                self.ref_quat.copy(),
                bool(self.feedback_valid),
                bool(self.targets_initialized),
            )


def _copy_active_arm_values(dst: np.ndarray, src: np.ndarray, active_arm_mask: int) -> np.ndarray:
    result = np.asarray(dst, dtype=np.float64).copy()
    source = np.asarray(src, dtype=np.float64)
    for arm in range(Config.NUM_ARMS):
        if (active_arm_mask & (1 << arm)) == 0:
            continue
        arm_slice = slice(arm * Config.ARM_JOINTS, (arm + 1) * Config.ARM_JOINTS)
        result[arm_slice] = source[arm_slice]
    return result


def _copy_active_arm_pose_values(dst: np.ndarray, src: np.ndarray, active_arm_mask: int) -> np.ndarray:
    result = np.asarray(dst, dtype=np.float64).copy()
    source = np.asarray(src, dtype=np.float64).reshape(result.shape)
    for arm in range(Config.NUM_ARMS):
        if (active_arm_mask & (1 << arm)) != 0:
            result[arm] = source[arm]
    return result


def mirror_real_state_to_env(
    env,
    shared_state: RealSharedState,
    active_arm_mask: int,
    initial_target_pos_base: np.ndarray | None = None,
    initial_target_quat_base: np.ndarray | None = None,
) -> None:
    q, qd, ref_pos, ref_quat, feedback_valid, targets_initialized = shared_state.snapshot_mirror()
    if not feedback_valid:
        return
    env.set_qpos(_copy_active_arm_values(env.get_qpos(), q, active_arm_mask))
    env.set_qvel(_copy_active_arm_values(env.get_qvel(), qd, active_arm_mask))
    env.forward()
    if hasattr(env, "set_all_reference_poses"):
        env.set_all_reference_poses(ref_pos, ref_quat)
    if not targets_initialized:
        target_pos = env.get_all_ee_pos()
        target_quat = env.get_all_ee_quat()
        if initial_target_pos_base is not None:
            target_pos = _copy_active_arm_pose_values(target_pos, initial_target_pos_base, active_arm_mask)
        if initial_target_quat_base is not None:
            target_quat = _copy_active_arm_pose_values(target_quat, initial_target_quat_base, active_arm_mask)
        env.set_all_target_poses_base(target_pos, target_quat)
        for arm, arm_name in enumerate(Config.ARM_NAMES):
            if (active_arm_mask & (1 << arm)) == 0:
                continue
            current_tcp = env.get_all_ee_pos()[arm]
            target_tcp = target_pos[arm]
            distance = float(np.linalg.norm(target_tcp - current_tcp))
            print(
                f"[Real Target] {arm_name}: current_tcp={np.array2string(current_tcp, precision=4)} "
                f"target_tcp={np.array2string(target_tcp, precision=4)} "
                f"distance={distance:.4f}m target_pos=INIT_QPOS_TCP "
                "target_quat=INIT_QPOS_TCP"
            )
    target_pos, target_quat = env.get_all_target_poses()
    shared_state.set_targets(target_pos, target_quat)
