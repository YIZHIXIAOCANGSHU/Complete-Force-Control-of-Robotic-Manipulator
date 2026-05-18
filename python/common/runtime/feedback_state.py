"""Unified joint feedback snapshot used by real and simulated backends."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FeedbackSnapshot:
    q: np.ndarray
    qd: np.ndarray
    tau: np.ndarray
    timestamp: float
    state_codes: np.ndarray | None = None
    source: str = "unknown"

    def __post_init__(self) -> None:
        q = np.asarray(self.q, dtype=np.float64)
        qd = np.asarray(self.qd, dtype=np.float64)
        tau = np.asarray(self.tau, dtype=np.float64)
        state_codes = (
            np.zeros(q.shape, dtype=np.int32)
            if self.state_codes is None
            else np.asarray(self.state_codes, dtype=np.int32)
        )
        if q.ndim != 1:
            raise ValueError(f"q must be one-dimensional, got {q.shape}")
        if qd.shape != q.shape:
            raise ValueError(f"qd must match q shape {q.shape}, got {qd.shape}")
        if tau.shape != q.shape:
            raise ValueError(f"tau must match q shape {q.shape}, got {tau.shape}")
        if state_codes.shape != q.shape:
            raise ValueError(f"state_codes must match q shape {q.shape}, got {state_codes.shape}")
        object.__setattr__(self, "q", q.copy())
        object.__setattr__(self, "qd", qd.copy())
        object.__setattr__(self, "tau", tau.copy())
        object.__setattr__(self, "state_codes", state_codes.copy())
        object.__setattr__(self, "timestamp", float(self.timestamp))
        object.__setattr__(self, "source", str(self.source))

    @property
    def joint_count(self) -> int:
        return int(self.q.shape[0])


def snapshot_from_frames(frames, *, timestamp: float, joint_count: int = 7, source: str = "usb2fdcan") -> FeedbackSnapshot:
    q = np.zeros(joint_count, dtype=np.float64)
    qd = np.zeros(joint_count, dtype=np.float64)
    tau = np.zeros(joint_count, dtype=np.float64)
    state_codes = np.zeros(joint_count, dtype=np.int32)
    for frame in frames:
        motor_id = int(getattr(frame, "motor_id"))
        if not 1 <= motor_id <= joint_count:
            continue
        idx = motor_id - 1
        q[idx] = float(getattr(frame, "position", 0.0))
        qd[idx] = float(getattr(frame, "velocity", 0.0))
        tau[idx] = float(getattr(frame, "torque", 0.0))
        state_codes[idx] = int(getattr(frame, "state", getattr(frame, "state_code", 0)))
    return FeedbackSnapshot(q=q, qd=qd, tau=tau, timestamp=timestamp, state_codes=state_codes, source=source)
