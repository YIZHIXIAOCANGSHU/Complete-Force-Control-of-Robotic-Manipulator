import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from param_id.identification import get_last_diagnostics, solve_least_squares
from param_id.regressor import build_joint_term_regressor, joint_term_param_names


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
