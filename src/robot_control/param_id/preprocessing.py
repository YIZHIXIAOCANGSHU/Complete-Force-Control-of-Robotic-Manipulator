"""Shared preprocessing helpers for parameter-identification data."""

from __future__ import annotations

import numpy as np


def estimate_qd_qdd(q, dt: float) -> tuple[np.ndarray, np.ndarray]:
    samples = np.asarray(q, dtype=np.float64)
    if samples.ndim != 2:
        raise ValueError(f"q must be a 2D trajectory, got {samples.shape}")
    if samples.shape[0] < 2:
        raise ValueError("q must contain at least two samples")
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    qd = np.zeros_like(samples)
    qdd = np.zeros_like(samples)
    qd[1:-1] = (samples[2:] - samples[:-2]) / (2.0 * dt)
    qd[0] = (samples[1] - samples[0]) / dt
    qd[-1] = (samples[-1] - samples[-2]) / dt
    if samples.shape[0] > 2:
        qdd[1:-1] = (samples[2:] - 2.0 * samples[1:-1] + samples[:-2]) / (dt**2)
        qdd[0] = qdd[1]
        qdd[-1] = qdd[-2]
    return qd, qdd


def estimate_qdd_from_qd(qd, dt: float) -> np.ndarray:
    velocities = np.asarray(qd, dtype=np.float64)
    if velocities.ndim != 2:
        raise ValueError(f"qd must be a 2D trajectory, got {velocities.shape}")
    if velocities.shape[0] < 2:
        raise ValueError("qd must contain at least two samples")
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    edge_order = 2 if velocities.shape[0] >= 3 else 1
    return np.gradient(velocities, float(dt), axis=0, edge_order=edge_order)
