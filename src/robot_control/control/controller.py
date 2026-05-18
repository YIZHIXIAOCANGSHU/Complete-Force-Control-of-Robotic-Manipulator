"""Small shared control-command builders."""

from __future__ import annotations

import numpy as np

from robot_control.config import Config
from robot_control.control.types import ControlCommand, ControlTarget
from robot_control.shared.runtime.feedback_state import FeedbackSnapshot


def zero_command(target: ControlTarget | None = None) -> ControlCommand:
    q_ref = Config.HOME_QPOS if target is None else target.q
    qd_ref = np.zeros(Config.NUM_JOINTS, dtype=np.float64) if target is None else target.qd
    return ControlCommand(
        q_ref=q_ref,
        qd_ref=qd_ref,
        kp=np.zeros(Config.NUM_JOINTS, dtype=np.float64),
        kd=np.zeros(Config.NUM_JOINTS, dtype=np.float64),
        tau_ff=np.zeros(Config.NUM_JOINTS, dtype=np.float64),
    )


def pd_command(
    feedback: FeedbackSnapshot,
    target: ControlTarget,
    *,
    kp,
    kd,
    tau_ff=None,
) -> ControlCommand:
    del feedback
    return ControlCommand(
        q_ref=target.q,
        qd_ref=target.qd,
        kp=kp,
        kd=kd,
        tau_ff=np.zeros(Config.NUM_JOINTS, dtype=np.float64) if tau_ff is None else tau_ff,
    )

