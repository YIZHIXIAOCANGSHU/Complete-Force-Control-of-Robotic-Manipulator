"""Shared parameter-identification data contracts."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class IdentificationDataset:
    q: np.ndarray
    qd: np.ndarray
    qdd: np.ndarray
    tau: np.ndarray

    def __post_init__(self) -> None:
        q = np.asarray(self.q, dtype=np.float64)
        qd = np.asarray(self.qd, dtype=np.float64)
        qdd = np.asarray(self.qdd, dtype=np.float64)
        tau = np.asarray(self.tau, dtype=np.float64)
        if q.ndim != 2:
            raise ValueError(f"q must be 2D, got {q.shape}")
        if qd.shape != q.shape or qdd.shape != q.shape or tau.shape != q.shape:
            raise ValueError("qd, qdd, and tau must match q shape")
        object.__setattr__(self, "q", q.copy())
        object.__setattr__(self, "qd", qd.copy())
        object.__setattr__(self, "qdd", qdd.copy())
        object.__setattr__(self, "tau", tau.copy())

