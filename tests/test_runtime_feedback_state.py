from __future__ import annotations

from dataclasses import dataclass
import numpy as np
import pytest

from robot_control.shared.runtime.feedback_state import FeedbackSnapshot, snapshot_from_frames
from robot_control.shared.runtime.safety import check_feedback_age, check_joint_limits


@dataclass
class Frame:
    motor_id: int
    position: float
    velocity: float
    torque: float
    state: int


def test_feedback_snapshot_validates_vector_shapes():
    with pytest.raises(ValueError, match="qd must match"):
        FeedbackSnapshot(q=np.zeros(7), qd=np.zeros(6), tau=np.zeros(7), timestamp=1.0)


def test_snapshot_from_frames_maps_motor_ids_to_joint_arrays():
    snapshot = snapshot_from_frames(
        [
            Frame(1, 0.1, 0.2, 0.3, 8),
            Frame(7, 0.7, 0.8, 0.9, 9),
        ],
        timestamp=12.0,
        source="test",
    )

    assert snapshot.source == "test"
    assert snapshot.q[0] == 0.1
    assert snapshot.q[6] == 0.7
    assert snapshot.state_codes.tolist() == [8, 0, 0, 0, 0, 0, 9]


def test_safety_checks_report_feedback_timeout_and_joint_limit_violation():
    snapshot = FeedbackSnapshot(q=np.array([0.0, 2.0]), qd=np.zeros(2), tau=np.zeros(2), timestamp=1.0)

    assert not check_feedback_age(snapshot, now=1.2, timeout_s=0.1).ok
    status = check_joint_limits(snapshot, lower=np.array([-1.0, -1.0]), upper=np.array([1.0, 1.0]))

    assert not status.ok
    assert "2" in status.reason

