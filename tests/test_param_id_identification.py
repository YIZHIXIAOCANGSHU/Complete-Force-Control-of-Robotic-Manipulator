import inspect
import sys
import types

import numpy as np
import robot_control.param_id.sim_main as sim_main
import robot_control.param_id.reporting as reporting
import robot_control.param_id.trajectory as trajectory
import robot_control.param_id.diagnostics as diagnostics
from robot_control.param_id.identification import (
    _link_excitation_quality,
    _prior_weights,
    _relative_scale,
    get_last_diagnostics,
    solve_least_squares,
)
from robot_control.param_id.regressor import build_joint_term_regressor, build_stacked_regressor, joint_term_param_names
from robot_control.param_id.diagnostics import (
    _case_selection_key,
    _com_error_summary,
    _inertia_error_summary,
    _mass_error_summary,
)


def test_scaled_svd_solver_keeps_nullspace_near_prior():
    x = np.linspace(-1.0, 1.0, 50)
    y = np.column_stack([x, x * (1.0 + 1e-12)])
    true_theta = np.array([2.0, 0.0])
    prior = {"a": 0.5, "b": 0.5}

    result = solve_least_squares(
        y,
        y @ true_theta,
        ["a", "b"],
        prior=prior,
        rcond=1e-6,
        inertial_prior_lambda=0.0,
        joint_prior_lambda=0.0,
    )
    pred = y @ np.array([result["a"], result["b"]])

    assert np.sqrt(np.mean((pred - y @ true_theta) ** 2)) < 1e-4
    assert abs(result["a"] - result["b"]) < 1e-6
    diagnostics = get_last_diagnostics()
    assert diagnostics["rank"] == 1
    assert diagnostics["nullity"] == 1
    assert diagnostics["prior_delta_rms"] > 0.0


def test_grouped_prior_regularization_holds_joint_terms_near_prior():
    y = np.array(
        [
            [1.0, 0.0, 1e-6],
            [0.0, 1.0, 1e-6],
            [1.0, 1.0, 2e-6],
        ]
    )
    tau = np.array([1.0, 2.0, 3.0])
    prior = {"L0_mass": 0.8, "L1_mass": 2.2, "J1_fc": 5.0}

    result = solve_least_squares(
        y,
        tau,
        ["L0_mass", "L1_mass", "J1_fc"],
        prior=prior,
        inertial_prior_lambda=1e-8,
        joint_prior_lambda=10.0,
    )

    assert abs(result["J1_fc"] - prior["J1_fc"]) < 1e-3
    pred = y @ np.array([result["L0_mass"], result["L1_mass"], result["J1_fc"]])
    assert np.sqrt(np.mean((pred - tau) ** 2)) < 1e-3


def test_prior_weights_split_inertial_parameter_groups():
    names = ["L0_mass", "L0_mcx", "L0_mcy", "L0_mcz", "L0_Ixx", "L0_Iyy", "L0_Izz", "J1_fc"]

    weights = _prior_weights(
        names,
        inertial_lambda=0.2,
        joint_lambda=0.05,
        mass_lambda=0.04,
        com_lambda=0.3,
        inertia_lambda=0.6,
    )

    assert np.allclose(weights, [0.04, 0.3, 0.3, 0.3, 0.6, 0.6, 0.6, 0.05])


def test_link_excitation_quality_scales_weak_link_priors():
    names = [
        *(f"L0_{suffix}" for suffix in ("mass", "mcx", "mcy", "mcz", "Ixx", "Iyy", "Izz")),
        *(f"L1_{suffix}" for suffix in ("mass", "mcx", "mcy", "mcz", "Ixx", "Iyy", "Izz")),
    ]
    strong = np.eye(7, dtype=np.float64)
    weak_base = np.linspace(0.2, 1.0, 7, dtype=np.float64)[:, None]
    weak = np.repeat(weak_base, 7, axis=1)
    y = np.vstack([np.hstack([strong, weak]), np.hstack([strong * 0.5, weak * 0.5])])

    qualities = _link_excitation_quality(y, names)
    weights = _prior_weights(
        names,
        inertial_lambda=1.0,
        joint_lambda=0.0,
        mass_lambda=2.0,
        com_lambda=3.0,
        inertia_lambda=4.0,
        link_excitation=qualities,
    )

    assert qualities[0] > qualities[1]
    assert weights[7] > weights[0] * 10.0


def test_relative_scale_uses_natural_floors_for_small_inertial_terms():
    names = ["L0_mass", "L0_mcx", "L0_Ixx", "J1_fo"]
    prior = np.array([0.2, 0.0002, 0.00004, 0.005])

    scale = _relative_scale(prior, names)

    assert np.allclose(scale, [1.0, 0.01, 0.001, 1.0])


def test_grouped_regularization_can_hold_com_and_inertia_without_freezing_mass():
    y = np.eye(3)
    tau = np.array([2.0, 0.1, 0.01])
    names = ["L0_mass", "L0_mcx", "L0_Ixx"]
    prior = {"L0_mass": 1.0, "L0_mcx": 0.0, "L0_Ixx": 0.0}

    result = solve_least_squares(
        y,
        tau,
        names,
        prior=prior,
        mass_prior_lambda=0.0,
        com_prior_lambda=100.0,
        inertia_prior_lambda=100.0,
        joint_prior_lambda=0.0,
    )

    assert result["L0_mass"] > 1.5
    assert abs(result["L0_mcx"]) < 0.02
    assert abs(result["L0_Ixx"]) < 0.002
    diagnostics = get_last_diagnostics()
    assert "com_prior_delta_rms" in diagnostics
    assert "inertia_prior_delta_rms" in diagnostics


def test_joint_term_regressor_columns_are_per_joint():
    q = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7])
    qd = np.array([0.01, -0.02, 0.03, -0.04, 0.05, -0.06, 0.07])
    q_ref = np.zeros(7)

    y = build_joint_term_regressor(q, qd, q_ref=q_ref, coulomb_eps=0.02)

    assert y.shape == (7, 28)
    assert joint_term_param_names()[:4] == ["J1_fc", "J1_k", "J1_fv", "J1_fo"]
    assert y[0, 0] == np.tanh(qd[0] / 0.02)
    assert y[0, 1] == q[0]
    assert y[0, 2] == qd[0]
    assert y[0, 3] == 1.0
    assert np.count_nonzero(y[0, 4:]) == 0
    assert np.count_nonzero(y[1, :4]) == 0


class _ZeroInertialBackend:
    def __init__(self):
        self._model = None
        self._data = None


def test_stacked_regressor_path_shapes_with_and_without_joint_terms(monkeypatch):
    def fake_build_regressor(backend, q, qd, qdd, include_joint_terms=True, q_ref=None, coulomb_eps=0.02):
        inertial = np.zeros((7, 49))
        if not include_joint_terms:
            return inertial
        return np.hstack([inertial, build_joint_term_regressor(q, qd, q_ref=q_ref, coulomb_eps=coulomb_eps)])

    monkeypatch.setattr("robot_control.param_id.regressor.build_regressor", fake_build_regressor)
    q = np.zeros((3, 7))
    qd = np.ones((3, 7)) * 0.1
    qdd = np.zeros((3, 7))

    combined, combined_names = build_stacked_regressor(
        _ZeroInertialBackend(), q, qd, qdd, include_joint_terms=True, q_ref=np.zeros(7)
    )
    inertial, inertial_names = build_stacked_regressor(
        _ZeroInertialBackend(), q, qd, qdd, include_joint_terms=False, q_ref=np.zeros(7)
    )

    assert combined.shape == (21, 77)
    assert inertial.shape == (21, 49)
    assert any(name.startswith("J") for name in combined_names)
    assert not any(name.startswith("J") for name in inertial_names)


def _case_with_physical_summaries(
    mass_errors,
    com_offsets=None,
    inertia_errors=None,
    prediction_error=0.0,
):
    true_masses = np.ones(len(mass_errors))
    masses = true_masses * (1.0 + np.asarray(mass_errors, dtype=np.float64) / 100.0)
    true_coms = np.zeros((len(mass_errors), 3), dtype=np.float64)
    if com_offsets is None:
        coms = true_coms.copy()
    else:
        coms = true_coms + np.asarray(com_offsets, dtype=np.float64)
    true_inertias = np.ones((len(mass_errors), 3), dtype=np.float64)
    if inertia_errors is None:
        inertias = true_inertias.copy()
    else:
        inertias = true_inertias * (1.0 + np.asarray(inertia_errors, dtype=np.float64) / 100.0)
    return {
        "mass_summary": _mass_error_summary(masses, true_masses),
        "com_summary": _com_error_summary(coms, true_coms),
        "inertia_summary": _inertia_error_summary(inertias, true_inertias),
        "prediction_error": prediction_error,
        "diagnostics": {"data_rank": 7},
        "inertial_distal": {"projection": {"ratio": 0.5}},
    }


def test_mass_error_summary_reports_max_joint_and_target_pass():
    summary = _mass_error_summary(
        np.array([1.01, 0.98, 1.03, 1.0, 0.96, 1.02, 0.955]),
        np.ones(7),
    )

    assert np.allclose(summary["errors"], [1.0, -2.0, 3.0, 0.0, -4.0, 2.0, -4.5])
    assert np.isclose(summary["max_abs"], 4.5)
    assert summary["max_abs_joint"] == 7
    assert summary["passes_5pct"]


def test_com_error_summary_reports_vectors_distances_and_target_pass():
    true_coms = np.zeros((7, 3), dtype=np.float64)
    coms = true_coms.copy()
    coms[4] = [0.001, 0.002, 0.0]
    coms[6] = [0.0, -0.003, 0.004]

    summary = _com_error_summary(coms, true_coms)

    assert np.allclose(summary["error_vectors"][6], [0.0, -0.003, 0.004])
    assert np.isclose(summary["distance_errors"][6], 0.005)
    assert summary["max_distance_joint"] == 7
    assert np.isclose(summary["distal_distance_mean"], (np.sqrt(5e-6) + 0.0 + 0.005) / 3.0)
    assert summary["passes_target"]


def test_inertia_error_summary_reports_component_and_link_errors():
    true_inertias = np.ones((7, 3), dtype=np.float64)
    inertias = true_inertias.copy()
    inertias[1] = [1.05, 0.95, 1.0]
    inertias[6] = [1.10, 0.80, 1.05]

    summary = _inertia_error_summary(inertias, true_inertias)

    assert np.allclose(summary["relative_errors"][6], [10.0, -20.0, 5.0])
    assert np.isclose(summary["link_l2_errors"][6], np.sqrt(10.0**2 + 20.0**2 + 5.0**2))
    assert np.isclose(summary["max_component_abs"], 20.0)
    assert summary["max_component_joint"] == 7
    assert summary["max_component_axis"] == "Iyy"


def test_case_selection_prefers_passing_max_error_over_lower_distal_mean():
    passing = _case_with_physical_summaries([4.9, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0], prediction_error=1.0)
    failing = _case_with_physical_summaries([5.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], prediction_error=0.1)

    assert _case_selection_key(passing) < _case_selection_key(failing)


def test_case_selection_considers_com_and_inertia_within_passing_candidates():
    mass_errors = [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
    good_physics = _case_with_physical_summaries(
        mass_errors,
        com_offsets=np.full((7, 3), 0.001),
        inertia_errors=np.full((7, 3), 2.0),
        prediction_error=1.0,
    )
    lower_rms = _case_with_physical_summaries(
        mass_errors,
        com_offsets=np.full((7, 3), 0.006),
        inertia_errors=np.full((7, 3), 8.0),
        prediction_error=0.1,
    )

    assert _case_selection_key(good_physics) < _case_selection_key(lower_rms)


def test_sim_main_reports_single_joint_inclusive_result():
    source = inspect.getsource(sim_main.main)

    assert "仅惯性辨识结果" not in source
    assert "_print_case_summary" not in source
    assert "inertial_case" not in source


def test_viewer_context_degrades_when_viewer_preflight_fails(monkeypatch, capsys):
    class DummyEnv:
        model = object()
        data = object()

    def fail_launch(_model, _data):
        raise RuntimeError("GLFW OpenGL context is not available")

    monkeypatch.setattr(reporting.Config, "ENABLE_VIEWER", True)
    monkeypatch.setattr(reporting, "launch_passive_viewer", fail_launch)

    with reporting._viewer_context(DummyEnv()) as viewer:
        assert viewer is None

    output = capsys.readouterr().out
    assert "MuJoCo 窗口不可用" in output
    assert "改为无窗口运行" in output


class _DummyRerun(types.ModuleType):
    def __init__(self):
        super().__init__("rerun")
        self.__path__ = []
        self.logs = []
        self.blueprint = None
        self.time_calls = []

    def init(self, *_args, **_kwargs):
        return None

    def set_time_seconds(self, timeline, value):
        self.time_calls.append((timeline, value))

    def log(self, path, payload, static=False):
        self.logs.append((path, payload, static))

    def send_blueprint(self, blueprint):
        self.blueprint = blueprint

    def Scalars(self, value):
        return value

    def BarChart(self, values, **kwargs):
        return {"kind": "BarChart", "values": np.asarray(values, dtype=np.float64), **kwargs}

    def SeriesLines(self, **kwargs):
        return {"kind": "SeriesLines", **kwargs}

    def Arrows3D(self, **kwargs):
        return {"kind": "Arrows3D", **kwargs}

    def Points3D(self, points, **kwargs):
        return {"kind": "Points3D", "points": points, **kwargs}

    def LineStrips3D(self, strips, **kwargs):
        return {"kind": "LineStrips3D", "strips": strips, **kwargs}


class _DummyBlueprint:
    @staticmethod
    def TimeSeriesView(name, origin):
        return {"kind": "TimeSeriesView", "name": name, "origin": origin}

    @staticmethod
    def BarChartView(name, origin):
        return {"kind": "BarChartView", "name": name, "origin": origin}

    @staticmethod
    def Spatial3DView(name, origin):
        return {"kind": "Spatial3DView", "name": name, "origin": origin}

    @staticmethod
    def Horizontal(*children, name=None):
        return {"kind": "Horizontal", "name": name, "children": list(children)}

    @staticmethod
    def Vertical(*children, name=None):
        return {"kind": "Vertical", "name": name, "children": list(children)}

    @staticmethod
    def Tabs(*children, name=None):
        return {"kind": "Tabs", "name": name, "children": list(children)}

    @staticmethod
    def Blueprint(root, collapse_panels=False):
        return {"kind": "Blueprint", "root": root, "collapse_panels": collapse_panels}


def _iter_blueprint_nodes(node):
    if isinstance(node, dict):
        yield node
        for value in node.values():
            yield from _iter_blueprint_nodes(value)
    elif isinstance(node, list):
        for item in node:
            yield from _iter_blueprint_nodes(item)


def _install_dummy_rerun(monkeypatch):
    dummy_rr = _DummyRerun()
    blueprint_module = types.ModuleType("rerun.blueprint")
    for name in ("TimeSeriesView", "BarChartView", "Spatial3DView", "Horizontal", "Vertical", "Tabs", "Blueprint"):
        setattr(blueprint_module, name, getattr(_DummyBlueprint, name))
    dummy_rr.blueprint = blueprint_module
    monkeypatch.setitem(sys.modules, "rerun", dummy_rr)
    monkeypatch.setitem(sys.modules, "rerun.blueprint", blueprint_module)
    monkeypatch.setattr(reporting.rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(reporting.rerun_viz, "rr", dummy_rr, raising=False)
    monkeypatch.setattr(reporting.rerun_viz, "rrb", blueprint_module, raising=False)
    return dummy_rr


def test_param_id_rerun_step_logs_q_qd_tau_for_all_joints(monkeypatch):
    dummy_rr = _install_dummy_rerun(monkeypatch)
    q = np.arange(7, dtype=np.float64) + 0.1
    qd = np.arange(7, dtype=np.float64) + 0.2
    tau = np.arange(7, dtype=np.float64) + 0.3

    reporting._log_rerun_step(True, 1.25, q, qd, tau)

    logged = {path: payload for path, payload, _static in dummy_rr.logs}
    assert dummy_rr.time_calls == [("time", 1.25)]
    for joint in range(1, 8):
        assert logged[f"param_id/excitation_q_rad/J{joint}"] == q[joint - 1]
        assert logged[f"param_id/excitation_qd_rad_s/J{joint}"] == qd[joint - 1]
        assert logged[f"param_id/tau_nm/J{joint}"] == tau[joint - 1]


def test_param_id_rerun_blueprint_contains_joint_detail_panels(monkeypatch):
    dummy_rr = _install_dummy_rerun(monkeypatch)
    monkeypatch.setattr(sim_main.Config, "ENABLE_RERUN", True)

    assert reporting._setup_rerun()

    names = {
        node["name"]
        for node in _iter_blueprint_nodes(dummy_rr.blueprint)
        if node.get("name")
    }
    origins = {
        node["origin"]
        for node in _iter_blueprint_nodes(dummy_rr.blueprint)
        if node.get("kind") == "TimeSeriesView"
    }
    static_line_names = {
        path: payload["names"]
        for path, payload, static in dummy_rr.logs
        if static and payload.get("kind") == "SeriesLines"
    }

    assert "Joint Details" in names
    assert "J1 Position (rad)" in names
    assert "J7 Velocity (rad/s)" in names
    assert "/param_id/excitation_q_rad/J1" in origins
    assert "/param_id/excitation_qd_rad_s/J7" in origins
    assert static_line_names["param_id/excitation_q_rad/J1"] == ["J1 position"]
    assert static_line_names["param_id/excitation_qd_rad_s/J7"] == ["J7 velocity"]
    assert static_line_names["param_id/tau_nm/J7"] == ["J7 torque"]


def test_param_id_rerun_setup_uses_common_init_and_keeps_sim_views(monkeypatch):
    dummy_rr = _install_dummy_rerun(monkeypatch)
    monkeypatch.setattr(sim_main.Config, "ENABLE_RERUN", True)
    init_calls = []

    def fake_init(app_name):
        init_calls.append(app_name)
        return True

    monkeypatch.setattr(reporting.rerun_viz, "init_rerun", fake_init)

    assert reporting._setup_rerun()

    origins = {
        node["origin"]
        for node in _iter_blueprint_nodes(dummy_rr.blueprint)
        if node.get("kind") in {"TimeSeriesView", "BarChartView", "Spatial3DView"}
    }
    assert init_calls == ["AM-D02 参数辨识 (Sim)"]
    assert "/param_id/excitation_q_rad/J1" in origins
    assert "/tracking/pos/X" in origins
    assert "/trajectory_3d" in origins


def test_param_id_rerun_setup_can_use_pd_sim_prefix(monkeypatch):
    dummy_rr = _install_dummy_rerun(monkeypatch)
    monkeypatch.setattr(sim_main.Config, "ENABLE_RERUN", True)
    init_calls = []

    def fake_init(app_name):
        init_calls.append(app_name)
        return True

    monkeypatch.setattr(reporting.rerun_viz, "init_rerun", fake_init)

    assert reporting._setup_rerun(app_name="AM-D02 参数辨识 (PD Sim)", param_prefix="param_id_sim_pd")

    origins = {
        node["origin"]
        for node in _iter_blueprint_nodes(dummy_rr.blueprint)
        if node.get("kind") in {"TimeSeriesView", "BarChartView", "Spatial3DView"}
    }
    logged = {path for path, _payload, _static in dummy_rr.logs}
    assert init_calls == ["AM-D02 参数辨识 (PD Sim)"]
    assert "/param_id_sim_pd/excitation_q_rad/J1" in origins
    assert "/param_id_sim_pd/result_bar/mass" in origins
    assert "param_id_sim_pd/excitation_q_rad/J1" in logged


def test_param_id_rerun_setup_degrades_when_common_init_fails(monkeypatch, capsys):
    _install_dummy_rerun(monkeypatch)
    monkeypatch.setattr(sim_main.Config, "ENABLE_RERUN", True)
    monkeypatch.setattr(reporting.rerun_viz, "init_rerun", lambda _app_name: False)

    assert not reporting._setup_rerun()

    assert "Rerun" in capsys.readouterr().out


def test_param_id_main_initializes_rerun_before_mujoco_env():
    source = inspect.getsource(sim_main.main)

    assert source.index("rerun_ok = _setup_rerun()") < source.index("env = MujocoSimEnv()")


def test_param_id_logs_common_sim_realtime_step_from_env(monkeypatch):
    class FakeEnv:
        def __init__(self):
            self.data = types.SimpleNamespace(
                qpos=np.full(7, 9.0, dtype=np.float64),
                qvel=np.full(7, -9.0, dtype=np.float64),
                qacc=np.full(7, 0.5, dtype=np.float64),
            )
            self.forward_calls = 0

        def set_qpos(self, q):
            self.data.qpos[:7] = q

        def set_qvel(self, qd):
            self.data.qvel[:7] = qd

        def forward(self):
            self.forward_calls += 1

        def get_ee_pos(self):
            return np.array(
                [
                    float(np.sum(self.data.qpos[:7])),
                    float(self.data.qpos[0]),
                    float(self.data.qpos[1]),
                ],
                dtype=np.float64,
            )

        def get_ee_quat(self):
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    captured = {}

    def fake_log_sim_realtime_step(**kwargs):
        captured.update(kwargs)

    monkeypatch.setattr(reporting.rerun_viz, "log_sim_realtime_step", fake_log_sim_realtime_step)
    env = FakeEnv()
    saved_qpos = env.data.qpos.copy()
    saved_qvel = env.data.qvel.copy()
    saved_qacc = env.data.qacc.copy()
    q_actual = np.ones(7, dtype=np.float64)
    q_desired = np.ones(7, dtype=np.float64) * 2.0
    qd_actual = np.ones(7, dtype=np.float64) * 0.1
    tau = np.arange(7, dtype=np.float64)

    reporting._log_sim_realtime_step_from_env(
        rerun_ok=True,
        env=env,
        t=0.5,
        step=3,
        q_actual=q_actual,
        qd_actual=qd_actual,
        q_desired=q_desired,
        tau_received=tau,
        tau_applied=tau + 1.0,
        cycle_time_ms=2.5,
    )

    assert captured["t"] == 0.5
    assert captured["step_count"] == 3
    assert np.allclose(captured["q"], q_actual)
    assert np.allclose(captured["qd"], qd_actual)
    assert np.allclose(captured["tau_received"], tau)
    assert np.allclose(captured["tau_applied"], tau + 1.0)
    assert np.allclose(captured["pos_actual"], [7.0, 1.0, 1.0])
    assert np.allclose(captured["pos_desired"], [14.0, 2.0, 2.0])
    assert np.allclose(env.data.qpos, saved_qpos)
    assert np.allclose(env.data.qvel, saved_qvel)
    assert np.allclose(env.data.qacc, saved_qacc)


def _minimal_identified_case():
    masses = np.arange(1, 8, dtype=np.float64).tolist()
    coms = (np.arange(21, dtype=np.float64).reshape(7, 3) / 1000.0).tolist()
    inertias = (np.ones((7, 3), dtype=np.float64) * 0.01).tolist()
    return {
        "name": "联合辨识结果（惯性 + 关节项）",
        "masses": masses,
        "coms": coms,
        "inertias": inertias,
        "result": {
            f"J{joint}_{term}": float(joint)
            for joint in range(1, 8)
            for term in ("fc", "k", "fv", "fo")
        },
        "diagnostics": {
            "rank": 69,
            "data_rank": 70,
            "retained_condition": 12.5,
            "prior_delta_rms": 0.01,
        },
        "condition": 14.0,
        "prediction_error": 0.123,
        "validation_rms": 0.234,
        "validation_ratio": 1.902,
        "segment_rms": {"dynamic": 0.1, "j7": 0.2},
        "selection": {
            "mass_prior_lambda": 32.0,
            "com_prior_lambda": 1.2,
            "inertia_prior_lambda": 2.4,
            "joint_prior_lambda": 0.035,
            "rcond": 1e-8,
        },
        "mass_summary": diagnostics._mass_error_summary(masses, np.ones(7)),
        "com_summary": diagnostics._com_error_summary(coms, np.zeros((7, 3))),
        "inertia_summary": diagnostics._inertia_error_summary(inertias, np.ones((7, 3)) * 0.01),
        "joint_term_error_summary": diagnostics._joint_term_error_summary(
            {
                f"J{joint}_{term}": float(joint)
                for joint in range(1, 8)
                for term in ("fc", "k", "fv", "fo")
            },
            np.zeros((2, 7)),
            np.zeros((2, 7)),
            np.zeros(7),
        ),
        "param_names": [f"L{link}_{suffix}" for link in range(7) for suffix in ("mass", "mcx", "mcy", "mcz", "Ixx", "Iyy", "Izz")],
    }


def test_joint_term_error_summary_reports_parameter_and_torque_errors():
    result = {
        f"J{joint}_{term}": float(values[term])
        for joint, values in enumerate(sim_main.Config.PARAM_ID_JOINT_PRIORS, start=1)
        for term in ("fc", "k", "fv", "fo")
    }
    result["J1_fc"] += 0.1
    result["J2_fv"] += 0.02
    q = np.zeros((3, 7), dtype=np.float64)
    qd = np.zeros((3, 7), dtype=np.float64)
    qd[:, 0] = 0.5
    qd[:, 1] = 0.25

    summary = diagnostics._joint_term_error_summary(result, q, qd, np.zeros(7))

    assert summary["max_abs_param_error"] >= 0.1
    assert summary["max_abs_param"] == "J1_fc"
    assert summary["torque_rms"] > 0.0
    assert summary["torque_max_abs"] > 0.0
    assert summary["torque_max_abs_joint"] == 1
    assert summary["reference"] == "PARAM_ID_JOINT_PRIORS"
    assert summary["error_definition"] == "identified - prior"
    assert summary["per_joint"][0]["torque_rms"] > 0.0


def test_terminal_report_uses_summary_tables_and_folds_diagnostics(monkeypatch, capsys):
    monkeypatch.delenv("AM_D02_PARAM_ID_DIAGNOSTICS", raising=False)

    reporting._print_identification_case(
        _minimal_identified_case(),
        true_masses=np.ones(7),
        true_inertias=np.ones((7, 3)) * 0.01,
        true_coms=np.zeros((7, 3)),
    )
    concise = capsys.readouterr().out

    assert "参数辨识结果总览" in concise
    assert "质量+COM" in concise
    assert "Izz(辨识)" in concise
    assert "训练/验证 RMS" in concise
    assert "摩擦/弹性关节项:" in concise
    assert "先验偏离 RMS" not in concise
    assert "J7专项" not in concise

    monkeypatch.setenv("AM_D02_PARAM_ID_DIAGNOSTICS", "1")
    reporting._print_identification_case(
        _minimal_identified_case(),
        true_masses=np.ones(7),
        true_inertias=np.ones((7, 3)) * 0.01,
        true_coms=np.zeros((7, 3)),
    )
    detailed = capsys.readouterr().out

    assert "诊断详情" in detailed
    assert "J7专项" in detailed


def test_error_pct_formatter_caps_uninformative_large_errors():
    assert reporting._fmt_error_pct(3.2, target=5.0) == "+3.2% ✓"
    assert reporting._fmt_error_pct(-7.25, target=5.0) == "-7.2% ✗"
    assert reporting._fmt_error_pct(5000.0, target=5.0) == ">1000% ✗"


def test_trajectory_profiles_include_combined_com_and_inertia_t7(monkeypatch):
    monkeypatch.setattr(sim_main.Config, "PARAM_ID_TRAJECTORY_PROFILES", 8)

    profiles = trajectory._trajectory_profiles()
    t7 = next(profile for profile in profiles if profile["name"] == "T7")

    assert t7["with_com_gravity"]
    assert t7["with_inertia_burst"]
