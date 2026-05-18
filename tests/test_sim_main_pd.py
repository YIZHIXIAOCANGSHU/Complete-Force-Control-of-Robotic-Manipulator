import contextlib
import numpy as np
import robot_control.param_id.diagnostics as param_diag
import robot_control.param_id.trajectory as param_traj
import robot_control.modes.param_id_sim.main as sim_pd
import robot_control.modes.param_id_sim.validation as sim_pd_validation


class _FakeBackend:
    def __init__(self, feedforward=None):
        self._joint_kp = np.ones(7)
        self._joint_kd = np.zeros(7)
        self._torque_limits = np.ones(7) * 100.0
        self.feedforward = np.zeros(7) if feedforward is None else np.asarray(feedforward, dtype=np.float64)

    def compute_nonlinear_effects(self, q, qd):
        return self.feedforward.copy()


class _PointMassEnv:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.q = np.zeros(7, dtype=np.float64)
        self.qd = np.zeros(7, dtype=np.float64)
        self.tau = np.zeros(7, dtype=np.float64)
        self.step_count = 0
        self.targets = []

    def reset(self, qpos=None):
        self.q = np.zeros(7, dtype=np.float64) if qpos is None else np.asarray(qpos, dtype=np.float64).copy()
        self.qd = np.zeros(7, dtype=np.float64)
        self.tau = np.zeros(7, dtype=np.float64)

    def forward(self):
        pass

    def get_qpos(self):
        return self.q.copy()

    def get_qvel(self):
        return self.qd.copy()

    def apply_torque(self, tau):
        self.tau = np.asarray(tau, dtype=np.float64).copy()

    def step(self):
        self.qd = self.qd + self.tau * self.dt
        self.q = self.q + self.qd * self.dt
        self.step_count += 1

    def enforce_joint_limits(self):
        return False

    def set_target_pose(self, pos, quat):
        self.targets.append((np.asarray(pos, dtype=np.float64), np.asarray(quat, dtype=np.float64)))


def test_pd_controller_torque_within_limits():
    controller = sim_pd.PDController(
        _FakeBackend(feedforward=np.ones(7) * 0.25),
        kp=np.ones(7) * 100.0,
        kd=np.zeros(7),
        torque_limits=np.ones(7) * 3.0,
    )

    positive = controller.compute_torque(np.zeros(7), np.zeros(7), np.ones(7), np.zeros(7))
    negative = controller.compute_torque(np.zeros(7), np.zeros(7), -np.ones(7), np.zeros(7))

    assert np.allclose(positive, 3.0)
    assert np.allclose(negative, -3.0)


def test_collect_pd_data_produces_reasonable_tracking():
    env = _PointMassEnv(dt=0.01)
    controller = sim_pd.PDController(
        _FakeBackend(),
        kp=np.ones(7) * 10.0,
        kd=np.ones(7) * 6.0,
        torque_limits=np.ones(7) * 100.0,
    )
    q_traj = np.full((300, 7), 0.1, dtype=np.float64)
    qd_traj = np.zeros_like(q_traj)

    q_meas, qd_meas, tau_meas = sim_pd._collect_pd_data(env, controller, q_traj, qd_traj)

    assert q_meas.shape == qd_meas.shape == tau_meas.shape == (300, 7)
    assert env.step_count == 300
    assert np.allclose(q_meas[0], 0.0)
    assert float(np.max(np.abs(env.q - q_traj[-1]))) < 0.01


def test_pd_identification_torque_subtracts_joint_effect_model():
    q = np.full((2, 7), 0.2, dtype=np.float64)
    qd = np.full((2, 7), 0.1, dtype=np.float64)
    tau_cmd = np.full((2, 7), 10.0, dtype=np.float64)
    priors = [{"fc": 1.0, "k": 2.0, "fv": 3.0, "fo": 4.0} for _ in range(7)]

    tau_id = sim_pd._pd_identification_torque(tau_cmd, q, qd, np.zeros(7), priors=priors, scale=1.0)

    expected_joint = np.tanh(0.1 / sim_pd.Config.PARAM_ID_COULOMB_EPS) + 2.0 * 0.2 + 3.0 * 0.1 + 4.0
    assert np.allclose(tau_id, 10.0 - expected_joint)


def test_pd_joint_prior_scale_defaults_to_no_subtraction():
    q = np.full((2, 7), 0.2, dtype=np.float64)
    qd = np.full((2, 7), 0.1, dtype=np.float64)
    tau_cmd = np.full((2, 7), 10.0, dtype=np.float64)
    priors = [{"fc": 1.0, "k": 2.0, "fv": 3.0, "fo": 4.0} for _ in range(7)]

    tau_id = sim_pd._pd_identification_torque(tau_cmd, q, qd, np.zeros(7), priors=priors)

    assert sim_pd.Config.PARAM_ID_PD_JOINT_PRIOR_SCALE == 0.0
    assert np.allclose(tau_id, tau_cmd)


def test_estimate_qdd_from_measured_velocity():
    t = np.arange(6, dtype=np.float64) * 0.01
    qd = np.outer(t, np.arange(1, 8, dtype=np.float64))

    qdd = sim_pd._estimate_qdd_from_qd(qd, dt=0.01)

    assert np.allclose(qdd, np.arange(1, 8, dtype=np.float64))


def test_estimate_qdd_uses_filtered_derivative_for_noisy_velocity():
    dt = 0.002
    t = np.arange(180, dtype=np.float64) * dt
    clean_qd = 0.5 * t * t
    noise = 0.0008 * np.sin(2.0 * np.pi * 180.0 * t)
    qd = np.tile((clean_qd + noise)[:, None], (1, 7))
    expected = np.tile(t[:, None], (1, 7))
    raw = np.gradient(qd, dt, axis=0, edge_order=2)

    filtered = sim_pd._estimate_qdd_from_qd(qd, dt=dt)

    raw_rms = float(np.sqrt(np.mean((raw - expected) ** 2)))
    filtered_rms = float(np.sqrt(np.mean((filtered - expected) ** 2)))
    assert filtered_rms < raw_rms * 0.65


def test_solve_hierarchical_pd_case_marks_distal_refinement(monkeypatch):
    param_names = [
        f"L{link}_{suffix}"
        for link in range(7)
        for suffix in ("mass", "mcx", "mcy", "mcz", "Ixx", "Iyy", "Izz")
    ]
    rows = 84
    y = np.zeros((rows, 49), dtype=np.float64)
    for col in range(49):
        y[col % rows, col] = 1.0 + col * 0.01
        y[(col + 7) % rows, col] = 0.25
    theta = np.ones(49, dtype=np.float64)
    theta[28:] = 1.5
    tau = y @ theta

    def fake_build_stacked_regressor(*_args, **_kwargs):
        return y, param_names

    monkeypatch.setattr(sim_pd_validation, "build_stacked_regressor", fake_build_stacked_regressor)
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_DISTAL_LINK_START", 5)
    labels = np.asarray(["dynamic"] * (rows // sim_pd.Config.NUM_JOINTS), dtype=object)

    case = sim_pd._solve_hierarchical_pd_case(
        "hierarchical smoke",
        backend=None,
        q_meas=np.zeros((len(labels), 7)),
        qd_meas=np.zeros((len(labels), 7)),
        qdd_traj=np.zeros((len(labels), 7)),
        tau_inertial=tau.reshape(-1, 7),
        trajectory_labels=labels,
        stride=1,
        q_ref=np.zeros(7),
        true_masses=[1.0] * 7,
        true_coms=[[0.0, 0.0, 0.0]] * 7,
        true_inertias=[[1.0, 1.0, 1.0]] * 7,
        mass_prior_lambda=16.0,
        com_prior_lambda=1.0,
        inertia_prior_lambda=2.0,
    )

    assert case["selection"]["mode"] == "hierarchical"
    assert case["selection"]["distal_mass_prior_lambda"] == 32.0
    assert case["selection"]["distal_inertia_prior_lambda"] == 20.0
    assert case["diagnostics"]["hierarchical"] == 1.0


def test_pd_inertia_summary_uses_stricter_target(monkeypatch):
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_PD_INERTIA_ERROR_TARGET_PCT", 5.0)
    inertias = np.ones((7, 3), dtype=np.float64)
    true_inertias = np.ones((7, 3), dtype=np.float64)
    inertias[6, 2] = 1.06

    summary = sim_pd._pd_inertia_error_summary(inertias, true_inertias)

    assert summary["target_pct"] == 5.0
    assert not summary["passes_target"]


def test_pd_regularization_grid_adds_strict_inertia_candidate(monkeypatch):
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_PD_STRICT_MASS_PRIOR_LAMBDA", 64.0)
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_PD_STRICT_COM_PRIOR_LAMBDA", 1.6)
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_PD_STRICT_INERTIA_PRIOR_LAMBDA", 6.4)

    grid = sim_pd._pd_regularization_grid()

    assert grid[0] == (64.0, 1.6, 6.4, sim_pd.Config.PARAM_ID_PRIOR_LAMBDA_JOINT)
    assert len(grid) == len(set(grid))


def test_viewer_simulation_returns_same_shape_as_collect(monkeypatch):
    monkeypatch.setattr(sim_pd._base, "_viewer_context", lambda env: contextlib.nullcontext(None))
    monkeypatch.setattr(sim_pd._base, "_sync_realtime", lambda start_wall, t_target: None)
    monkeypatch.setattr(sim_pd._base, "_log_rerun_step", lambda *args, **kwargs: None)
    monkeypatch.setattr(sim_pd._base, "_log_sim_realtime_step_from_env", lambda *args, **kwargs: None)
    env = _PointMassEnv(dt=0.01)
    controller = sim_pd.PDController(
        _FakeBackend(),
        kp=np.ones(7) * 10.0,
        kd=np.ones(7),
        torque_limits=np.ones(7) * 100.0,
    )
    t_arr = np.arange(5, dtype=np.float64) * 0.01
    q_traj = np.full((5, 7), 0.05, dtype=np.float64)
    qd_traj = np.zeros_like(q_traj)
    ee_pos = np.zeros((5, 3), dtype=np.float64)
    ee_quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (5, 1))

    q_meas, qd_meas, tau_meas = sim_pd._run_pd_simulation_with_viewer(
        env,
        controller,
        t_arr,
        q_traj,
        qd_traj,
        rerun_ok=False,
        ee_pos_desired_all=ee_pos,
        ee_quat_desired_all=ee_quat,
    )

    assert q_meas.shape == qd_meas.shape == tau_meas.shape == (5, 7)
    assert len(env.targets) == 5


def test_select_excitation_trajectory_pd_validates_top_svd_candidate(monkeypatch):
    profiles = [
        {"name": "low", "description": "low score"},
        {"name": "rich", "description": "high score"},
    ]
    validated_scores = []

    def fake_build_planned(profile, seed, q0, limits):
        score = 0.2 if profile["name"] == "low" else 1.0
        t_arr = np.arange(4, dtype=np.float64) * 0.01
        q = np.full((4, 7), score, dtype=np.float64)
        qd = np.zeros_like(q)
        qdd = np.zeros_like(q)
        labels = np.array(["dynamic"] * 4, dtype=object)
        return t_arr, q, qd, qdd, labels

    def fake_build_stacked(backend, q, qd, qdd, stride=1, include_joint_terms=True, **kwargs):
        cols = 77 if include_joint_terms else 49
        return np.full((7, cols), float(np.mean(q))), [f"p{i}" for i in range(cols)]

    def fake_metrics(Y, *args, **kwargs):
        return {"rank": 7, "condition": 1.0, "sigma_min": 1.0, "score": float(Y[0, 0])}

    def fake_distal(Y, include_joint_terms=False):
        return {"rank": 7, "condition": 1.0, "correlation": 0.0, "projection": {"ratio": 1.0, "rank": 7}}

    def fake_solve(name, backend, q_meas, qd_meas, qdd_traj, tau_meas, *args, **kwargs):
        validated_scores.append(float(np.mean(q_meas)))
        return {
            "mass_summary": {"max_abs": 1.0, "max_abs_joint": 1, "j7_abs": 1.0},
            "com_summary": {"max_distance": 0.0, "max_distance_joint": 1},
            "inertia_summary": {"max_component_abs": 0.0, "max_component_joint": 1},
            "prediction_error": 0.1,
            "validation_rms": 0.1,
            "diagnostics": {"rank": 77},
            "selection": {},
        }

    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_TRAJECTORY_CANDIDATES", 1)
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_VALIDATION_TOP_N", 1)
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_TRAJECTORY_PROFILE_DIAGNOSTICS", False)
    monkeypatch.setattr(param_diag, "_extract_ground_truth", lambda backend: ([1.0] * 7, [[0.0] * 3] * 7, [[1.0] * 3] * 7))
    monkeypatch.setattr(param_traj, "_trajectory_profiles", lambda: profiles)
    monkeypatch.setattr(param_traj, "_trajectory_seeds", lambda: [43])
    monkeypatch.setattr(param_traj, "_build_planned_trajectory", fake_build_planned)
    monkeypatch.setattr(sim_pd_validation, "limit_ee_speed", lambda env, q, qd, qdd, max_speed: (q, qd, qdd, max_speed, 1.0))
    monkeypatch.setattr(sim_pd_validation, "build_stacked_regressor", fake_build_stacked)
    monkeypatch.setattr(param_diag, "_scaled_svd_metrics", fake_metrics)
    monkeypatch.setattr(param_diag, "_distal_observability", fake_distal)
    monkeypatch.setattr(param_diag, "_parameter_group_observability", lambda Y: {})
    monkeypatch.setattr(param_traj, "_joint_coverage", lambda q: {"mean": 1.0})
    monkeypatch.setattr(param_traj, "_candidate_score", lambda overall, *args: overall["score"])
    monkeypatch.setattr(sim_pd_validation, "_pd_validation_grid", lambda: [(1.0, 1.0, 1.0, 1.0)])
    monkeypatch.setattr(sim_pd_validation, "_solve_hierarchical_pd_case", fake_solve)
    monkeypatch.setattr(param_diag, "_case_selection_key", lambda case: case["validation_rms"])

    env = _PointMassEnv(dt=0.01)
    controller = sim_pd.PDController(_FakeBackend(), kp=np.ones(7), kd=np.zeros(7))

    result = sim_pd._select_excitation_trajectory_pd(
        env,
        _FakeBackend(),
        controller,
        np.zeros(7),
        (np.full(7, -1.0), np.full(7, 1.0)),
    )

    assert result[9]["profile"] == "rich"
    assert validated_scores == [1.0]


# ---- new tests for unified preprocessing and diagnostics ----


def test_prepare_pd_identification_data_filters_and_returns_arrays():
    dt = 0.002
    t = np.arange(200, dtype=np.float64) * dt
    q = 0.5 * t[:, None] + np.random.default_rng(42).normal(0, 1e-4, (200, 7))
    qd = 0.5 * np.ones((200, 7), dtype=np.float64)
    tau_cmd = np.full((200, 7), 5.0, dtype=np.float64)

    prep = sim_pd._prepare_pd_identification_data(q, qd, tau_cmd, np.zeros(7), dt=dt)

    assert prep["q_meas"].shape == (200, 7)
    assert prep["qd_meas"].shape == (200, 7)
    assert prep["qdd_meas"].shape == (200, 7)
    assert prep["tau_id"].shape == (200, 7)
    diag = prep["diag"]
    assert diag["qdd_rms_mean"] >= 0.0
    assert "qdd_rms_J1" in diag
    assert "tau_cmd_rms" in diag
    assert "tau_joint_prior_rms" in diag
    assert "tau_joint_prior_applied_rms" in diag
    assert "tau_id_rms" in diag
    assert diag["tau_joint_prior_to_cmd_ratio"] >= 0.0
    assert diag["tau_joint_prior_applied_to_cmd_ratio"] >= 0.0
    assert "tau_cmd_rms_J1" in diag
    assert "tau_pri_rms_J1" in diag
    assert "tau_pri_applied_rms_J1" in diag
    assert "tau_id_rms_J1" in diag
    assert "tau_pri_ratio_J1" in diag
    assert "tau_pri_applied_ratio_J1" in diag


def test_pd_validation_grid_uses_full_grid_by_default(monkeypatch):
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_REG_SWEEP", True)
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_PD_VALIDATION_REG_GRID_LIMIT", 0)
    grid = sim_pd._pd_validation_grid()
    base_grid = list(param_diag._regularization_grid())
    assert len(grid) >= len(base_grid)
    for item in base_grid:
        assert item in grid
    assert len(grid) >= 8


def test_pd_validation_grid_respects_limit(monkeypatch):
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_PD_VALIDATION_REG_GRID_LIMIT", 2)
    grid = sim_pd._pd_validation_grid()
    assert len(grid) == 2


def test_pd_identification_torque_respects_joint_prior_scale():
    q = np.full((2, 7), 0.2, dtype=np.float64)
    qd = np.full((2, 7), 0.1, dtype=np.float64)
    tau_cmd = np.full((2, 7), 10.0, dtype=np.float64)
    priors = [{"fc": 1.0, "k": 2.0, "fv": 3.0, "fo": 4.0} for _ in range(7)]

    tau_default = sim_pd._pd_identification_torque(tau_cmd, q, qd, np.zeros(7), priors=priors, scale=1.0)
    tau_zero = sim_pd._pd_identification_torque(tau_cmd, q, qd, np.zeros(7), priors=priors, scale=0.0)
    tau_half = sim_pd._pd_identification_torque(tau_cmd, q, qd, np.zeros(7), priors=priors, scale=0.5)

    assert np.allclose(tau_zero, tau_cmd)
    assert not np.allclose(tau_default, tau_cmd)
    assert not np.allclose(tau_default, tau_zero)
    mid = (tau_cmd + tau_default) / 2.0
    assert np.allclose(tau_half, mid, rtol=1e-12)


def test_pd_clipping_summary_reports_percentages():
    tau = np.zeros((10, 7), dtype=np.float64)
    tau[:, 0] = 1.0
    limits = np.ones(7, dtype=np.float64) * 1.0
    summary = sim_pd._pd_clipping_summary(tau, limits)
    assert abs(summary["clipped_pct_J1"] - 100.0) < 0.1
    assert abs(summary["clipped_pct_J2"] - 0.0) < 0.1
    assert abs(summary["clipped_any_pct"] - 100.0) < 0.1


def test_pd_tracking_summary_includes_per_joint_and_velocity():
    q = np.full((100, 7), 0.1, dtype=np.float64)
    q_ref = np.zeros((100, 7), dtype=np.float64)
    qd = np.full((100, 7), 0.5, dtype=np.float64)
    qd_ref = np.zeros_like(qd)

    summary = sim_pd._pd_tracking_summary(q, q_ref, qd, qd_ref)
    assert abs(summary["joint_rms_rad"] - 0.1) < 1e-10
    assert abs(summary["velocity_rms_rad_s"] - 0.5) < 1e-10
    for j in range(7):
        assert f"joint_rms_J{j + 1}_rad" in summary
        assert f"velocity_rms_J{j + 1}_rad_s" in summary


def test_candidate_validation_metadata_includes_new_diagnostics(monkeypatch):
    profiles = [{"name": "test", "description": "test profile"}]

    def fake_build_planned(profile, seed, q0, limits):
        t_arr = np.arange(4, dtype=np.float64) * 0.01
        q = np.full((4, 7), 0.1, dtype=np.float64)
        qd = np.full((4, 7), 0.01, dtype=np.float64)
        qdd = np.zeros_like(q)
        labels = np.array(["dynamic"] * 4, dtype=object)
        return t_arr, q, qd, qdd, labels

    def fake_build_stacked(backend, q, qd, qdd, stride=1, include_joint_terms=True, **kwargs):
        cols = 77 if include_joint_terms else 49
        return np.full((7, cols), float(np.mean(q))), [f"p{i}" for i in range(cols)]

    def fake_metrics(Y, *args, **kwargs):
        return {"rank": 7, "condition": 1.0, "sigma_min": 1.0, "score": float(Y[0, 0])}

    def fake_distal(Y, include_joint_terms=False):
        return {"rank": 7, "condition": 1.0, "correlation": 0.0, "projection": {"ratio": 1.0, "rank": 7}}

    def fake_solve(name, backend, q_meas, qd_meas, qdd_traj, tau_meas, *args, **kwargs):
        return {
            "mass_summary": {"max_abs": 1.0, "max_abs_joint": 1, "j7_abs": 1.0},
            "com_summary": {"max_distance": 0.0, "max_distance_joint": 1},
            "inertia_summary": {"max_component_abs": 0.0, "max_component_joint": 1},
            "prediction_error": 0.1,
            "validation_rms": 0.1,
            "diagnostics": {"rank": 77},
            "selection": {"mass_prior_lambda": 32.0, "com_prior_lambda": 1.2, "inertia_prior_lambda": 2.4},
        }

    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_TRAJECTORY_CANDIDATES", 1)
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_VALIDATION_TOP_N", 1)
    monkeypatch.setattr(sim_pd.Config, "PARAM_ID_TRAJECTORY_PROFILE_DIAGNOSTICS", False)
    monkeypatch.setattr(param_diag, "_extract_ground_truth", lambda backend: ([1.0] * 7, [[0.0] * 3] * 7, [[1.0] * 3] * 7))
    monkeypatch.setattr(param_traj, "_trajectory_profiles", lambda: profiles)
    monkeypatch.setattr(param_traj, "_trajectory_seeds", lambda: [43])
    monkeypatch.setattr(param_traj, "_build_planned_trajectory", fake_build_planned)
    monkeypatch.setattr(sim_pd_validation, "limit_ee_speed", lambda env, q, qd, qdd, max_speed: (q, qd, qdd, max_speed, 1.0))
    monkeypatch.setattr(sim_pd_validation, "build_stacked_regressor", fake_build_stacked)
    monkeypatch.setattr(param_diag, "_scaled_svd_metrics", fake_metrics)
    monkeypatch.setattr(param_diag, "_distal_observability", fake_distal)
    monkeypatch.setattr(param_diag, "_parameter_group_observability", lambda Y: {})
    monkeypatch.setattr(param_traj, "_joint_coverage", lambda q: {"mean": 1.0})
    monkeypatch.setattr(param_traj, "_candidate_score", lambda overall, *args: overall["score"])
    monkeypatch.setattr(sim_pd_validation, "_pd_validation_grid", lambda: [(1.0, 1.0, 1.0, 1.0)])
    monkeypatch.setattr(sim_pd_validation, "_solve_hierarchical_pd_case", fake_solve)
    monkeypatch.setattr(param_diag, "_case_selection_key", lambda case: case["validation_rms"])

    env = _PointMassEnv(dt=0.01)
    controller = sim_pd.PDController(_FakeBackend(), kp=np.ones(7), kd=np.zeros(7))

    result = sim_pd._select_excitation_trajectory_pd(
        env,
        _FakeBackend(),
        controller,
        np.zeros(7),
        (np.full(7, -1.0), np.full(7, 1.0)),
    )

    metadata = result[9]
    assert metadata["profile"] == "test"
    assert metadata.get("pd_validation_rms") is not None
    assert metadata.get("pd_tracking_rms_rad") is not None
    assert metadata.get("svd_score") is not None
    assert metadata.get("selected_mass_lambda") is not None
    assert metadata.get("selected_com_lambda") is not None
    assert metadata.get("selected_inertia_lambda") is not None
    assert metadata.get("pd_validation_reg_grid_size") == 1
    assert metadata.get("pd_qdd_rms_mean") is not None
    assert metadata.get("pd_tau_prior_ratio") is not None
    assert metadata.get("pd_tau_prior_applied_ratio") is not None


def test_end_to_end_pd_pipeline_composes(monkeypatch):
    """Verify the full sim-pd pipeline composes: prep → solve → best case."""
    n_steps = 120
    q = np.full((n_steps, 7), 0.1, dtype=np.float64)
    qd = np.full((n_steps, 7), 0.05, dtype=np.float64)
    tau_cmd = np.full((n_steps, 7), 5.0, dtype=np.float64)

    prep = sim_pd._prepare_pd_identification_data(q, qd, tau_cmd, np.zeros(7))

    param_names = [
        f"L{link}_{suffix}"
        for link in range(7)
        for suffix in ("mass", "mcx", "mcy", "mcz", "Ixx", "Iyy", "Izz")
    ]
    Y = np.zeros((n_steps * 7, 49), dtype=np.float64)
    for col in range(49):
        Y[col % (n_steps * 7), col] = 1.0 + col * 0.01
    theta = np.ones(49, dtype=np.float64)
    theta[28:] = 1.5
    tau = Y @ theta + np.random.default_rng(42).normal(0, 1e-6, Y.shape[0])

    monkeypatch.setattr(sim_pd_validation, "build_stacked_regressor", lambda *args, **kwargs: (Y, param_names))
    monkeypatch.setattr(param_diag, "solve_least_squares", lambda Ys, tau_s, pn, prior, **kw: {n: float(v) for n, v in zip(pn, theta)})
    monkeypatch.setattr(param_diag, "to_link_params", lambda result, prior: (
        [1.0] * 7, [[0.0, 0.0, 0.0]] * 7, [[1.0, 1.0, 1.0]] * 7,
    ))
    monkeypatch.setattr(param_diag, "compute_prediction_error", lambda *args: 0.01)
    monkeypatch.setattr(param_diag, "compute_condition_number", lambda *args: 100.0)
    monkeypatch.setattr(param_diag, "_scaled_svd_metrics", lambda *args: {"rank": 49, "condition": 100.0, "sigma_min": 0.01})
    monkeypatch.setattr(param_diag, "_distal_observability", lambda *args, **kw: {"rank": 21, "condition": 10.0, "correlation": 0.1, "projection": {"ratio": 0.8, "rank": 21}})
    monkeypatch.setattr(param_diag, "_parameter_group_observability", lambda *args: {})
    monkeypatch.setattr(param_diag, "_j7_column_diagnostics", lambda *args: {"mass_norm": 1.0, "mean_norm": 1.0, "max_norm": 1.0, "min_norm": 1.0})
    monkeypatch.setattr(param_diag, "_validation_rms", lambda *args, **kw: 0.02)
    monkeypatch.setattr(param_diag, "_segment_prediction_rms", lambda *args, **kw: 0.01)
    monkeypatch.setattr(param_diag, "_segment_indices", lambda labels, tag: np.arange(len(labels) // 2, dtype=np.int64))
    monkeypatch.setattr(param_diag, "_mass_error_summary", lambda masses, true_masses: {
        "errors": [0.0] * 7, "abs_errors": [0.0] * 7, "max_abs": 0.0, "max_abs_joint": 1,
        "passes_5pct": True, "target_pct": 5.0, "j7_abs": 0.0, "distal_abs_mean": 0.0,
    })
    monkeypatch.setattr(param_diag, "_com_error_summary", lambda coms, true_coms: {
        "error_vectors": [[0.0, 0.0, 0.0]] * 7, "distance_errors": [0.0] * 7,
        "max_distance": 0.0, "max_distance_joint": 1, "distal_distance_mean": 0.0,
        "target_m": 0.01, "passes_target": True,
    })
    monkeypatch.setattr(param_diag, "_inertia_error_summary", lambda inertias, true_inertias: {
        "relative_errors": [[0.0] * 3] * 7, "absolute_relative_errors": [[0.0] * 3] * 7,
        "link_l2_errors": [0.0] * 7, "max_component_abs": 0.0, "max_component_joint": 1,
        "max_component_axis": "Ixx", "max_link_l2": 0.0, "max_link_l2_joint": 1,
        "distal_l2_mean": 0.0, "target_pct": 15.0, "passes_target": True,
    })
    monkeypatch.setattr(param_diag, "get_last_diagnostics", lambda: {"rank": 49.0, "num_params": 49.0, "data_rank": 49.0})

    labels = np.array(["dynamic"] * n_steps, dtype=object)
    case = sim_pd._best_pd_inertial_case(
        "e2e-test",
        None,
        prep["q_meas"],
        prep["qd_meas"],
        prep["qdd_meas"],
        prep["tau_id"],
        labels,
        stride=1,
        q_ref=np.zeros(7),
        true_masses=[1.0] * 7,
        true_coms=[[0.0, 0.0, 0.0]] * 7,
        true_inertias=[[1.0, 1.0, 1.0]] * 7,
    )

    assert case["name"] == "e2e-test"
    assert case["diagnostics"]["hierarchical"] == 1.0
    assert "prediction_error" in case
    assert "validation_rms" in case
    assert "mass_summary" in case
    assert "com_summary" in case
    assert "inertia_summary" in case
    assert case["selection"]["mode"] == "hierarchical"
