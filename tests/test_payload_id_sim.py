from __future__ import annotations

import numpy as np
import pytest

from robot_control.param_id import payload


def test_payload_param_names_and_column_selection():
    names = [
        f"L{link}_{suffix}"
        for link in range(7)
        for suffix in ("mass", "mcx", "mcy", "mcz", "Ixx", "Iyy", "Izz")
    ]
    y = np.arange(3 * len(names), dtype=np.float64).reshape(3, len(names))

    y_payload, payload_names = payload.select_payload_regressor(y, names)

    assert payload_names == [
        "L6_mass",
        "L6_mcx",
        "L6_mcy",
        "L6_mcz",
        "L6_Ixx",
        "L6_Iyy",
        "L6_Izz",
    ]
    np.testing.assert_allclose(y_payload, y[:, -7:])


def test_solve_payload_only_recovers_synthetic_payload():
    rng = np.random.default_rng(7)
    y_payload = rng.normal(size=(80, 7))
    theta = np.array([2.5, 0.25, -0.125, 0.05, 0.04, 0.05, 0.06], dtype=np.float64)
    tau = y_payload @ theta
    names = payload.payload_param_names()

    result = payload.solve_payload_only(
        y_payload,
        tau,
        names,
        prior=dict(zip(names, theta)),
        inertial_prior_lambda=0.0,
    )

    assert result.rank == 7
    assert result.mass == pytest.approx(2.5, rel=1e-8)
    np.testing.assert_allclose(result.com, [0.1, -0.05, 0.02], rtol=1e-8, atol=1e-10)
    np.testing.assert_allclose(result.inertia_diag, [0.04, 0.05, 0.06], rtol=1e-8, atol=1e-10)
    assert result.prediction_error_rms < 1e-7


def test_solve_payload_only_rejects_low_excitation():
    names = payload.payload_param_names()
    y_payload = np.zeros((20, 7), dtype=np.float64)
    y_payload[:, 0] = 1.0

    with pytest.raises(RuntimeError, match="payload excitation"):
        payload.solve_payload_only(y_payload, np.ones(20), names)


def test_subtract_known_body_torque_preserves_payload_residual():
    rng = np.random.default_rng(11)
    full_names = [
        f"L{link}_{suffix}"
        for link in range(7)
        for suffix in ("mass", "mcx", "mcy", "mcz", "Ixx", "Iyy", "Izz")
    ]
    y_full = rng.normal(size=(100, len(full_names)))
    payload_names = payload.payload_param_names()
    payload_cols = payload.payload_column_indices(full_names)
    other_cols = np.setdiff1d(np.arange(len(full_names)), payload_cols)
    theta_payload = np.array([2.0, 0.2, -0.1, 0.06, 0.03, 0.04, 0.05], dtype=np.float64)
    theta_other = rng.normal(size=other_cols.size)
    prior = {full_names[index]: float(value) for index, value in zip(other_cols, theta_other)}
    tau = y_full[:, payload_cols] @ theta_payload + y_full[:, other_cols] @ theta_other

    tau_payload = payload.subtract_known_body_torque(y_full, tau, full_names, prior)
    result = payload.solve_payload_only(
        y_full[:, payload_cols],
        tau_payload,
        payload_names,
        prior=dict(zip(payload_names, theta_payload)),
        inertial_prior_lambda=0.0,
    )

    np.testing.assert_allclose(tau_payload, y_full[:, payload_cols] @ theta_payload, rtol=1e-10, atol=1e-10)
    assert result.mass == pytest.approx(2.0, rel=1e-8)
    np.testing.assert_allclose(result.inertia_diag, [0.03, 0.04, 0.05], rtol=1e-8, atol=1e-10)
