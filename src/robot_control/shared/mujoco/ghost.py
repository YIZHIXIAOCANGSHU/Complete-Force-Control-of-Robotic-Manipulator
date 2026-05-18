"""Viewer-only target-pose ghost for MuJoCo scenes."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class MujocoGhostRobot:
    """Minimal, non-physical ghost marker driven by target qpos or pose.

    The current scene already contains a non-colliding ``target_pose`` mocap
    marker.  This class wraps that marker as the first migration step toward a
    fuller robot ghost, keeping it out of dynamics and safe to disable.
    """

    model: object
    data: object
    alpha: float = 0.25
    color: tuple[float, float, float, float] = (0.2, 0.6, 1.0, 0.25)
    body_name: str = "target_pose"
    ee_body_name: str = "tcp"
    dof_ids: np.ndarray | None = None
    visible: bool = True

    def __post_init__(self) -> None:
        self.alpha = float(np.clip(self.alpha, 0.0, 1.0))
        self._mocap_id = self._resolve_mocap_id(self.body_name)
        self._ee_body_id = self._resolve_body_id(self.ee_body_name)
        self._scratch_data = self._create_scratch_data()
        self._last_qpos: np.ndarray | None = None
        self._last_position: np.ndarray | None = None
        self._last_quat: np.ndarray | None = None
        if self._mocap_id is None:
            self.visible = False
        self._apply_visibility()

    def _resolve_mocap_id(self, body_name: str) -> int | None:
        body_mocapid = getattr(self.model, "body_mocapid", None)
        if body_mocapid is None:
            return None

        try:
            import mujoco

            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id >= 0:
                mocap_id = int(body_mocapid[body_id])
                return mocap_id if mocap_id >= 0 else None
        except Exception:
            pass

        if len(body_mocapid) == 1:
            return int(body_mocapid[0])
        return None

    def _resolve_body_id(self, body_name: str) -> int | None:
        try:
            import mujoco

            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            return int(body_id) if body_id >= 0 else None
        except Exception:
            return None

    def _create_scratch_data(self):
        try:
            import mujoco

            return mujoco.MjData(self.model)
        except Exception:
            return None

    def _target_dof_ids(self, width: int) -> np.ndarray:
        if self.dof_ids is not None:
            ids = np.asarray(self.dof_ids, dtype=np.int64)
            if ids.shape != (width,):
                raise ValueError(f"dof_ids must have shape ({width},), got {ids.shape}")
            return ids
        return np.arange(width, dtype=np.int64)

    def _apply_visibility(self) -> None:
        geom_rgba = getattr(self.model, "geom_rgba", None)
        if geom_rgba is None:
            return
        alpha = self.alpha if self.visible else 0.0
        for geom_id in range(len(geom_rgba)):
            rgba = geom_rgba[geom_id]
            if np.isclose(float(rgba[3]), self.alpha) or (not self.visible and np.isclose(float(rgba[3]), 0.0)):
                rgba[3] = alpha

    def update_from_qpos(self, q_target: np.ndarray) -> None:
        arr = np.asarray(q_target, dtype=np.float64)
        if arr.shape != (7,):
            raise ValueError(f"q_target must have shape (7,), got {arr.shape}")
        self._last_qpos = arr.copy()
        if self._scratch_data is None or self._ee_body_id is None:
            return
        try:
            import mujoco

            dof_ids = self._target_dof_ids(len(arr))
            if int(np.max(dof_ids)) < len(self._scratch_data.qpos):
                self._scratch_data.qpos[dof_ids] = arr
            else:
                self._scratch_data.qpos[: len(arr)] = arr
            mujoco.mj_forward(self.model, self._scratch_data)
            self.update_from_pose(
                self._scratch_data.xpos[self._ee_body_id],
                self._scratch_data.xquat[self._ee_body_id],
            )
        except Exception as exc:
            print(f"[MuJoCo Ghost] warning: qpos update skipped ({exc})")

    def update_from_pose(self, position: np.ndarray, quat: np.ndarray) -> None:
        pos = np.asarray(position, dtype=np.float64)
        quat_arr = np.asarray(quat, dtype=np.float64)
        if pos.shape != (3,):
            raise ValueError(f"position must have shape (3,), got {pos.shape}")
        if quat_arr.shape != (4,):
            raise ValueError(f"quat must have shape (4,), got {quat_arr.shape}")
        self._last_position = pos.copy()
        self._last_quat = quat_arr.copy()
        if self.visible and self._mocap_id is not None:
            self.data.mocap_pos[self._mocap_id] = pos
            self.data.mocap_quat[self._mocap_id] = quat_arr

    def set_visible(self, visible: bool) -> None:
        self.visible = bool(visible) and self._mocap_id is not None
        self._apply_visibility()


def create_mujoco_ghost_if_enabled(model, data, *, enabled: bool, alpha: float = 0.25) -> MujocoGhostRobot | None:
    if not enabled:
        return None
    try:
        return MujocoGhostRobot(model, data, alpha=alpha)
    except Exception as exc:
        print(f"[MuJoCo Ghost] warning: disabled target ghost ({exc})")
        return None
