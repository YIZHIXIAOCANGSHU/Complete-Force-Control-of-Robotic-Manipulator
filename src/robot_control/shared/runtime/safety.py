"""Small shared safety predicates for runtime feedback checks."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from robot_control.shared.runtime.feedback_state import FeedbackSnapshot


@dataclass(frozen=True)
class SafetyStatus:
    ok: bool
    reason: str = ""


def check_feedback_age(snapshot: FeedbackSnapshot, *, now: float | None = None, timeout_s: float) -> SafetyStatus:
    current = time.perf_counter() if now is None else float(now)
    age = current - float(snapshot.timestamp)
    if age > float(timeout_s):
        return SafetyStatus(False, f"feedback timeout: {age:.3f}s > {float(timeout_s):.3f}s")
    return SafetyStatus(True, "")


def check_joint_limits(snapshot: FeedbackSnapshot, lower, upper) -> SafetyStatus:
    lower_arr = np.asarray(lower, dtype=np.float64)
    upper_arr = np.asarray(upper, dtype=np.float64)
    if lower_arr.shape != snapshot.q.shape or upper_arr.shape != snapshot.q.shape:
        raise ValueError("joint limit arrays must match feedback q shape")
    low_bad = snapshot.q < lower_arr
    high_bad = snapshot.q > upper_arr
    if np.any(low_bad | high_bad):
        ids = (np.flatnonzero(low_bad | high_bad) + 1).tolist()
        return SafetyStatus(False, f"joint limit violation: {ids}")
    return SafetyStatus(True, "")

