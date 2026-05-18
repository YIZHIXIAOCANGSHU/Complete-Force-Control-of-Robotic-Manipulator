from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))


def test_control_contracts_validate_joint_vectors_and_build_zero_command():
    from control.controller import zero_command
    from control.types import ControlTarget

    target = ControlTarget(q=np.arange(7, dtype=np.float64))
    command = zero_command(target)

    np.testing.assert_allclose(command.q_ref, np.arange(7, dtype=np.float64))
    np.testing.assert_allclose(command.tau_ff, np.zeros(7))

    with pytest.raises(ValueError, match="q must have shape"):
        ControlTarget(q=np.zeros(6))


def test_param_id_preprocessing_estimates_derivatives():
    from param_id.preprocessing import estimate_qd_qdd

    q = np.array([[0.0, 0.0], [1.0, 2.0], [4.0, 8.0]], dtype=np.float64)
    qd, qdd = estimate_qd_qdd(q, 1.0)

    np.testing.assert_allclose(qd[1], [2.0, 4.0])
    np.testing.assert_allclose(qdd[1], [2.0, 4.0])


def test_runtime_feedback_snapshot_validates_shapes():
    from common.runtime.feedback_state import FeedbackSnapshot

    snapshot = FeedbackSnapshot(q=np.zeros(7), qd=np.ones(7), tau=np.arange(7), timestamp=1.5)

    assert snapshot.joint_count == 7
    np.testing.assert_allclose(snapshot.state_codes, np.zeros(7))

    with pytest.raises(ValueError, match="qd must match q shape"):
        FeedbackSnapshot(q=np.zeros(7), qd=np.ones(6), tau=np.zeros(7), timestamp=0.0)
