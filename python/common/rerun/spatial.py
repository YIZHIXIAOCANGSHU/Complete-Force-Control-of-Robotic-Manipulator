"""Rerun 3D logging helpers for mode-specific spatial diagnostics."""

from __future__ import annotations

import numpy as np

try:
    import rerun as rr

    RERUN_AVAILABLE = True
except ImportError:
    rr = None
    RERUN_AVAILABLE = False


def _clean_prefix(prefix: str) -> str:
    return str(prefix).strip("/")


def _as_point3(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {arr.shape}")
    return arr


def _as_points(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3), got {arr.shape}")
    return arr


def log_ee_pose(prefix: str, t: float, position, quat=None) -> None:
    if not RERUN_AVAILABLE:
        return
    rr.set_time_seconds("time", float(t))
    path = f"{_clean_prefix(prefix)}/ee_pose"
    rr.log(path, rr.Points3D([_as_point3(position, "position")], radii=[0.01]))
    if quat is not None:
        rr.log(f"{path}/quat_wxyz", rr.Scalars(float(np.asarray(quat, dtype=np.float64)[0])))


def log_target_pose(prefix: str, position, quat=None) -> None:
    if not RERUN_AVAILABLE:
        return
    path = f"{_clean_prefix(prefix)}/target_pose"
    rr.log(path, rr.Points3D([_as_point3(position, "position")], radii=[0.012]))
    if quat is not None:
        rr.log(f"{path}/quat_wxyz", rr.Scalars(float(np.asarray(quat, dtype=np.float64)[0])))


def log_ee_trajectory(prefix: str, points) -> None:
    if not RERUN_AVAILABLE:
        return
    arr = _as_points(points, "points")
    if len(arr) == 0:
        return
    rr.log(f"{_clean_prefix(prefix)}/ee_trajectory", rr.LineStrips3D([arr]))


def log_error_vector(prefix: str, actual, target) -> None:
    if not RERUN_AVAILABLE:
        return
    actual_point = _as_point3(actual, "actual")
    target_point = _as_point3(target, "target")
    rr.log(
        f"{_clean_prefix(prefix)}/ee_error_vector",
        rr.Arrows3D(origins=[actual_point], vectors=[target_point - actual_point]),
    )


def log_workspace_points(prefix: str, points, *, max_points: int = 2000) -> None:
    if not RERUN_AVAILABLE:
        return
    arr = _as_points(points, "points")
    if len(arr) == 0:
        return
    if max_points > 0 and len(arr) > max_points:
        indices = np.linspace(0, len(arr) - 1, int(max_points), dtype=np.int64)
        arr = arr[indices]
    rr.log(f"{_clean_prefix(prefix)}/workspace_points", rr.Points3D(arr, radii=[0.003]))

