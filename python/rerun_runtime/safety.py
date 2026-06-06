"""Safety margin helpers for Rerun logging."""

from __future__ import annotations

import numpy as np

from config import Config
from rerun_runtime.constants import ARM_LABELS, SAFETY_LOG_MARGIN_RAD, SAFETY_LOG_MARGIN_RAD_S


def _joint_safe_limits_rad() -> tuple[np.ndarray, np.ndarray]:
    limits = np.asarray(Config.JOINT_LIMITS_RAD, dtype=np.float64)
    span = limits[:, 1] - limits[:, 0]
    inset = float(Config.CONTROL_JOINT_LIMIT_INSET_RATIO) * span
    return limits[:, 0] + inset, limits[:, 1] - inset


def _joint_safety_margins(q: np.ndarray | None, qd: np.ndarray | None):
    q_margin_low = q_margin_high = vel_margin = None
    if q is not None:
        q_values = np.asarray(q, dtype=np.float64).reshape(Config.NUM_JOINTS)
        safe_min, safe_max = _joint_safe_limits_rad()
        q_margin_low = q_values - safe_min
        q_margin_high = safe_max - q_values
    if qd is not None:
        qd_values = np.asarray(qd, dtype=np.float64).reshape(Config.NUM_JOINTS)
        vel_margin = float(Config.JOINT_VEL_LIMIT) - np.abs(qd_values)
    return q_margin_low, q_margin_high, vel_margin


def _format_safety_warnings(
    q_margin_low: np.ndarray | None,
    q_margin_high: np.ndarray | None,
    vel_margin: np.ndarray | None,
) -> str | None:
    warnings = []
    for arm, arm_label in enumerate(ARM_LABELS):
        offset = arm * Config.ARM_JOINTS
        for i in range(Config.ARM_JOINTS):
            joint = f"{arm_label}/J{i + 1}"
            index = offset + i
            if q_margin_low is not None and q_margin_low[index] <= SAFETY_LOG_MARGIN_RAD:
                warnings.append(f"{joint} low_limit_margin={q_margin_low[index]:.4f}rad")
            if q_margin_high is not None and q_margin_high[index] <= SAFETY_LOG_MARGIN_RAD:
                warnings.append(f"{joint} high_limit_margin={q_margin_high[index]:.4f}rad")
            if vel_margin is not None and vel_margin[index] <= SAFETY_LOG_MARGIN_RAD_S:
                warnings.append(f"{joint} vel_margin={vel_margin[index]:.4f}rad/s")
    return "; ".join(warnings) if warnings else None
