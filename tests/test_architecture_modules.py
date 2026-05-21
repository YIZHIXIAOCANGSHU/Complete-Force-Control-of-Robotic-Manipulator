from __future__ import annotations

import numpy as np
import pytest

def test_param_id_preprocessing_estimates_derivatives():
    from robot_control.param_id.preprocessing import estimate_qd_qdd, estimate_qdd_from_qd

    q = np.array([[0.0, 0.0], [1.0, 2.0], [4.0, 8.0]], dtype=np.float64)
    qd, qdd = estimate_qd_qdd(q, 1.0)

    np.testing.assert_allclose(qd[1], [2.0, 4.0])
    np.testing.assert_allclose(qdd[1], [2.0, 4.0])
    np.testing.assert_allclose(estimate_qdd_from_qd(qd, 1.0)[1], [1.0, 2.0])


def test_runtime_feedback_snapshot_validates_shapes():
    from robot_control.shared.runtime.feedback_state import FeedbackSnapshot

    snapshot = FeedbackSnapshot(q=np.zeros(7), qd=np.ones(7), tau=np.arange(7), timestamp=1.5)

    assert snapshot.joint_count == 7
    np.testing.assert_allclose(snapshot.state_codes, np.zeros(7))

    with pytest.raises(ValueError, match="qd must match q shape"):
        FeedbackSnapshot(q=np.zeros(7), qd=np.ones(6), tau=np.zeros(7), timestamp=0.0)


def test_control_mode_packages_are_removed():
    import importlib.util

    assert importlib.util.find_spec("robot_control.control") is None
    assert importlib.util.find_spec("robot_control.modes.control_real") is None
    assert importlib.util.find_spec("robot_control.modes.control_sim") is None
