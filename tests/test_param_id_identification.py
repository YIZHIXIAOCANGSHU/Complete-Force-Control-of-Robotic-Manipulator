import inspect
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import param_id.sim_main as sim_main
from param_id.identification import _prior_weights, _relative_scale, get_last_diagnostics, solve_least_squares
from param_id.regressor import build_joint_term_regressor, build_stacked_regressor, joint_term_param_names
from param_id.sim_main import (
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

    monkeypatch.setattr("param_id.regressor.build_regressor", fake_build_regressor)
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
