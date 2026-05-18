import numpy as np
from robot_control.param_id.diagnostics import (
    _distal_column_groups,
    _j7_column_diagnostics,
    _parameter_column_groups,
    _parameter_group_observability,
    _projection_residual_metrics,
    _regularization_grid,
    _stratified_validation_rows,
)
from robot_control.param_id.excitation import fourier_trajectory, limit_ee_speed
from robot_control.param_id.trajectory import (
    _apply_specialized_profile,
    _build_planned_trajectory,
    _trajectory_profiles,
)


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


def test_specialized_j7_profile_adds_terminal_motion_with_smooth_edges():
    t = np.linspace(0.0, 2.0, 201)
    q0 = np.zeros(7)
    q = np.zeros((len(t), 7))
    limits = (np.full(7, -1.0), np.full(7, 1.0))

    q_profile, qd_profile, qdd_profile = _apply_specialized_profile("j7-heavy", t, q, q0, limits)

    assert qd_profile is not None
    assert qdd_profile is not None
    assert np.ptp(q_profile[:, 6]) > 0.4
    assert np.allclose(q_profile[0], q0, atol=1e-12)
    assert np.allclose(q_profile[-1], q0, atol=1e-12)
    assert np.all(q_profile <= limits[1] + 1e-12)
    assert np.all(q_profile >= limits[0] - 1e-12)


def test_regularization_grid_uses_planned_joint_path_candidates():
    grid = _regularization_grid()

    assert grid == [
        (32.0, 1.20, 2.40, 0.035),
        (64.0, 1.20, 2.40, 0.035),
        (16.0, 1.20, 2.40, 0.035),
        (48.0, 1.20, 2.40, 0.035),
        (32.0, 0.80, 2.40, 0.035),
        (32.0, 1.60, 2.40, 0.035),
        (64.0, 1.60, 3.20, 0.035),
        (32.0, 1.20, 3.20, 0.050),
    ]


def test_trajectory_profiles_match_planned_t0_to_t7_matrix():
    profiles = _trajectory_profiles()

    assert [profile["name"] for profile in profiles] == ["T0", "T1", "T2", "T3", "T4", "T5", "T6", "T7"]
    assert profiles[0]["modifiers"] == ()
    assert profiles[1]["modifiers"] == ("j7_high_frequency",)
    assert profiles[2]["modifiers"] == ("j6_j7_phase_sweep",)
    assert profiles[3]["with_gravity"]
    assert profiles[4]["modifiers"] == ("j7_high_frequency", "j6_j7_phase_sweep")
    assert profiles[4]["with_gravity"]
    assert profiles[5]["with_com_gravity"]
    assert profiles[6]["with_inertia_burst"]
    assert profiles[7]["with_com_gravity"]
    assert profiles[7]["with_inertia_burst"]


def test_planned_t4_t5_t6_trajectories_add_specialized_segments():
    q0 = np.zeros(7)
    limits = (np.full(7, -1.0), np.full(7, 1.0))
    profiles = {profile["name"]: profile for profile in _trajectory_profiles()}

    t0, q_t0, _, _, labels_t0 = _build_planned_trajectory(profiles["T0"], 43, q0, limits)
    t4, q_t4, qd_t4, qdd_t4, labels_t4 = _build_planned_trajectory(profiles["T4"], 43, q0, limits)
    t5, q_t5, qd_t5, qdd_t5, labels_t5 = _build_planned_trajectory(profiles["T5"], 43, q0, limits)
    t6, q_t6, qd_t6, qdd_t6, labels_t6 = _build_planned_trajectory(profiles["T6"], 43, q0, limits)
    t7, q_t7, qd_t7, qdd_t7, labels_t7 = _build_planned_trajectory(profiles["T7"], 43, q0, limits)

    assert len(t4) > len(t0)
    assert q_t4.shape == qd_t4.shape == qdd_t4.shape
    assert q_t4.shape[0] == labels_t4.shape[0]
    assert "gravity" in set(labels_t4.tolist())
    assert "j6j7" in set(labels_t4.tolist())
    assert len(t5) > len(t0)
    assert q_t5.shape == qd_t5.shape == qdd_t5.shape
    assert "com_gravity" in set(labels_t5.tolist())
    assert len(t6) > len(t0)
    assert q_t6.shape == qd_t6.shape == qdd_t6.shape
    assert "inertia" in set(labels_t6.tolist())
    assert len(t7) > len(t5)
    assert q_t7.shape == qd_t7.shape == qdd_t7.shape
    assert "com_gravity" in set(labels_t7.tolist())
    assert "inertia" in set(labels_t7.tolist())
    assert np.ptp(q_t6[:, 6]) > np.ptp(q_t0[:, 6])
    assert set(labels_t0.tolist()) == {"dynamic"}
    for q in (q_t4, q_t5, q_t6):
        assert np.all(q <= limits[1] + 1e-12)
        assert np.all(q >= limits[0] - 1e-12)


def test_j7_column_diagnostics_reports_terminal_link_norms():
    y = np.zeros((3, 49))
    y[:, 42] = [3.0, 4.0, 0.0]
    y[:, 43] = [0.0, 0.0, 12.0]

    diagnostics = _j7_column_diagnostics(y)

    assert np.isclose(diagnostics["mass_norm"], 5.0)
    assert np.isclose(diagnostics["max_norm"], 12.0)
    assert diagnostics["mean_norm"] > 0.0
    assert diagnostics["min_norm"] == 0.0


def test_parameter_column_groups_include_com_and_inertia_distal_sets():
    groups = _parameter_column_groups(77)

    assert np.array_equal(groups["mass"][:3], [0, 7, 14])
    assert np.array_equal(groups["com"][:6], [1, 2, 3, 8, 9, 10])
    assert np.array_equal(groups["inertia"][-3:], [46, 47, 48])
    assert np.array_equal(groups["distal_com"][:3], [29, 30, 31])
    assert np.array_equal(groups["distal_inertia"][-3:], [46, 47, 48])
    assert groups["joint"][0] == 49


def test_parameter_group_observability_reports_group_projection_metrics():
    y = np.eye(77)

    diagnostics = _parameter_group_observability(y)

    assert diagnostics["mass"]["rank"] == 7
    assert diagnostics["com"]["rank"] == 21
    assert diagnostics["inertia"]["rank"] == 21
    assert diagnostics["distal_com"]["rank"] == 9
    assert diagnostics["distal_inertia"]["projection"]["ratio"] > 0.99


def test_stratified_validation_rows_samples_each_trajectory_label():
    labels = np.array(["dynamic"] * 10 + ["gravity"] * 10 + ["inertia"] * 10, dtype=object)

    rows = _stratified_validation_rows(labels, len(labels), fraction=0.2)

    assert rows.tolist() == [8, 9, 18, 19, 28, 29]
