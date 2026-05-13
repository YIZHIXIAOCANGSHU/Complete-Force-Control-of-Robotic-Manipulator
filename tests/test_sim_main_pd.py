import contextlib
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import param_id.sim_main_pd as sim_pd


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

    tau_id = sim_pd._pd_identification_torque(tau_cmd, q, qd, np.zeros(7), priors=priors)

    expected_joint = np.tanh(0.1 / sim_pd.Config.PARAM_ID_COULOMB_EPS) + 2.0 * 0.2 + 3.0 * 0.1 + 4.0
    assert np.allclose(tau_id, 10.0 - expected_joint)


def test_estimate_qdd_from_measured_velocity():
    t = np.arange(6, dtype=np.float64) * 0.01
    qd = np.outer(t, np.arange(1, 8, dtype=np.float64))

    qdd = sim_pd._estimate_qdd_from_qd(qd, dt=0.01)

    assert np.allclose(qdd, np.arange(1, 8, dtype=np.float64))


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
    monkeypatch.setattr(sim_pd._base, "_extract_ground_truth", lambda backend: ([1.0] * 7, [[0.0] * 3] * 7, [[1.0] * 3] * 7))
    monkeypatch.setattr(sim_pd._base, "_trajectory_profiles", lambda: profiles)
    monkeypatch.setattr(sim_pd._base, "_trajectory_seeds", lambda: [43])
    monkeypatch.setattr(sim_pd._base, "_build_planned_trajectory", fake_build_planned)
    monkeypatch.setattr(sim_pd, "limit_ee_speed", lambda env, q, qd, qdd, max_speed: (q, qd, qdd, max_speed, 1.0))
    monkeypatch.setattr(sim_pd, "build_stacked_regressor", fake_build_stacked)
    monkeypatch.setattr(sim_pd._base, "_scaled_svd_metrics", fake_metrics)
    monkeypatch.setattr(sim_pd._base, "_distal_observability", fake_distal)
    monkeypatch.setattr(sim_pd._base, "_parameter_group_observability", lambda Y: {})
    monkeypatch.setattr(sim_pd._base, "_joint_coverage", lambda q: {"mean": 1.0})
    monkeypatch.setattr(sim_pd._base, "_candidate_score", lambda overall, *args: overall["score"])
    monkeypatch.setattr(sim_pd, "_pd_validation_grid", lambda: [(1.0, 1.0, 1.0, 1.0)])
    monkeypatch.setattr(sim_pd, "_solve_pd_inertial_case", fake_solve)
    monkeypatch.setattr(sim_pd._base, "_case_selection_key", lambda case: case["validation_rms"])

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
