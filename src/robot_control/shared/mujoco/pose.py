"""Pose helpers for MuJoCo-backed visualization."""

from __future__ import annotations

import numpy as np


def normalize_quat_wxyz(quat) -> np.ndarray:
    arr = np.asarray(quat, dtype=np.float64)
    if arr.shape != (4,):
        raise ValueError(f"quat must have shape (4,), got {arr.shape}")
    norm = float(np.linalg.norm(arr))
    if norm <= 0.0:
        raise ValueError("quat must be non-zero")
    return arr / norm

