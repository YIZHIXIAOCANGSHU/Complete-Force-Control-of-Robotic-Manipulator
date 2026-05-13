import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from param_id.excitation import fourier_trajectory, limit_ee_speed
from param_id.sim_main import _distal_column_groups, _projection_residual_metrics


def test_fourier_distal_amplitude_weights_increase_distal_motion():
    q0 = np.zeros(7)
    limits = (np.full(7, -1.0), np.full(7, 1.0))

    _, q_default, _, _ = fourier_trajectory(
        q0=q0,
        duration=1.0,
        dt=0.01,
        joint_limits=limits,
        random_seed=7,
    )
    _, q_weighted, _, _ = fourier_trajectory(
        q0=q0,
        duration=1.0,
        dt=0.01,
        joint_limits=limits,
        random_seed=7,
        joint_amplitude_weights=np.array([1.0, 1.0, 1.0, 1.0, 1.5, 1.5, 1.5]),
        joint_frequency_weights=np.array([1.0, 1.0, 1.0, 1.0, 1.2, 1.2, 1.2]),
    )

    default_amp = np.ptp(q_default[:, 4:], axis=0)
    weighted_amp = np.ptp(q_weighted[:, 4:], axis=0)

    assert np.all(q_weighted <= limits[1] + 1e-12)
    assert np.all(q_weighted >= limits[0] - 1e-12)
    assert float(np.mean(weighted_amp)) > float(np.mean(default_amp))


def test_fourier_phase_offsets_change_distal_motion_without_breaking_limits():
    q0 = np.zeros(7)
    limits = (np.full(7, -1.0), np.full(7, 1.0))
    phase_offsets = np.array([0.0, 0.0, 0.0, 0.0, 0.2, 0.7, 1.3])

    _, q_default, qd_default, qdd_default = fourier_trajectory(
        q0=q0,
        duration=1.0,
        dt=0.01,
        joint_limits=limits,
        random_seed=11,
    )
    _, q_phased, qd_phased, qdd_phased = fourier_trajectory(
        q0=q0,
        duration=1.0,
        dt=0.01,
        joint_limits=limits,
        random_seed=11,
        phase_offsets=phase_offsets,
    )

    assert np.all(q_phased <= limits[1] + 1e-12)
    assert np.all(q_phased >= limits[0] - 1e-12)
    assert not np.allclose(q_default[:, 4:], q_phased[:, 4:])
    assert not np.allclose(qd_default[:, 4:], qd_phased[:, 4:])
    assert not np.allclose(qdd_default[:, 4:], qdd_phased[:, 4:])


class _LinearJacobianEnv:
    def __init__(self):
        self.q = None
        self.qd = None

    def set_qpos(self, q):
        self.q = q

    def set_qvel(self, qd):
        self.qd = qd

    def forward(self):
        pass

    def get_jacobian_7dof(self):
        jac = np.zeros((6, 7))
        jac[0] = 1.0
        return jac


def test_limit_ee_speed_scales_trajectory_to_speed_cap():
    env = _LinearJacobianEnv()
    q = np.array([[0.0] * 7, [1.0] * 7])
    qd = np.array([[2.0] * 7, [2.0] * 7])
    qdd = np.array([[4.0] * 7, [4.0] * 7])

    q_limited, qd_limited, qdd_limited, max_speed, scale = limit_ee_speed(env, q, qd, qdd, max_speed=7.0)

    assert np.isclose(scale, 0.5)
    assert np.isclose(max_speed, 7.0)
    assert np.allclose(q_limited[1], 0.5)
    assert np.allclose(qd_limited, qd * 0.5)
    assert np.allclose(qdd_limited, qdd * 0.5)


def test_distal_column_groups_split_inertial_columns_before_joint_terms():
    distal_cols, other_cols = _distal_column_groups(77, include_joint_terms=True)

    assert distal_cols[0] == 28
    assert distal_cols[-1] == 48
    assert 49 in other_cols
    assert not np.intersect1d(distal_cols, other_cols).size


def test_projection_residual_metrics_separate_dependent_and_independent_distal_columns():
    basis = np.eye(5)
    dependent = basis[:, :2]
    independent = np.array([
        [1.0, 0.0],
        [0.0, 1.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
    ])
    richer_independent = np.array([
        [0.0, 0.0],
        [0.0, 0.0],
        [1.0, 0.0],
        [0.0, 1.0],
        [0.5, 0.5],
    ])
    y_dependent = np.hstack([dependent, basis])
    y_independent = np.hstack([richer_independent, basis[:, :2]])

    dependent_metrics = _projection_residual_metrics(y_dependent, np.array([0, 1]), np.arange(2, 7))
    independent_metrics = _projection_residual_metrics(y_independent, np.array([0, 1]), np.array([2, 3]))

    assert dependent_metrics["ratio"] < 1e-12
    assert independent_metrics["ratio"] > 0.99
    assert independent_metrics["rank"] == 2
