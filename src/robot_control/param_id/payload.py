"""Payload-only identification helpers inspired by robot_payload_id.

This module keeps the first integration lightweight: it reuses the local
Pinocchio regressor and solves only the terminal-link inertial columns.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable

import numpy as np

from robot_control.config import Config
from robot_control.param_id.identification import (
    compute_condition_number,
    compute_prediction_error,
    solve_least_squares,
)
from robot_control.param_id.regressor import IDENTIFIED_NAMES, build_stacked_regressor


PAYLOAD_LINK_INDEX = Config.NUM_JOINTS - 1


@dataclass(frozen=True)
class PayloadIdentificationResult:
    values: Dict[str, float]
    mass: float
    com: np.ndarray
    inertia_diag: np.ndarray
    condition_number: float
    prediction_error_rms: float
    rank: int


def payload_param_names(link_index: int = PAYLOAD_LINK_INDEX) -> list[str]:
    return [f"L{int(link_index)}_{name}" for name in IDENTIFIED_NAMES]


def payload_column_indices(
    param_names: Iterable[str],
    link_index: int = PAYLOAD_LINK_INDEX,
) -> list[int]:
    wanted = set(payload_param_names(link_index))
    return [index for index, name in enumerate(param_names) if name in wanted]


def select_payload_regressor(
    y_stack: np.ndarray,
    param_names: list[str],
    *,
    link_index: int = PAYLOAD_LINK_INDEX,
) -> tuple[np.ndarray, list[str]]:
    cols = payload_column_indices(param_names, link_index=link_index)
    names = [param_names[index] for index in cols]
    expected = payload_param_names(link_index)
    if names != expected:
        raise ValueError(
            "payload columns are incomplete or out of order: "
            f"expected={expected}, got={names}"
        )
    return np.asarray(y_stack, dtype=np.float64)[:, cols], names


def subtract_known_body_torque(
    y_stack: np.ndarray,
    tau_stack: np.ndarray,
    param_names: list[str],
    baseline: dict[str, float],
    *,
    link_index: int = PAYLOAD_LINK_INDEX,
) -> np.ndarray:
    """Remove known non-payload inertial torque so only payload residual remains."""
    y = np.asarray(y_stack, dtype=np.float64)
    tau = np.asarray(tau_stack, dtype=np.float64).reshape(-1)
    if y.ndim != 2:
        raise ValueError(f"regressor must be 2D, got {y.shape}")
    if y.shape[0] != tau.shape[0]:
        raise ValueError(f"tau length {tau.shape[0]} must match regressor rows {y.shape[0]}")
    if y.shape[1] != len(param_names):
        raise ValueError(f"param_names length {len(param_names)} must match regressor columns {y.shape[1]}")

    payload_cols = np.asarray(payload_column_indices(param_names, link_index=link_index), dtype=np.int64)
    other_cols = np.setdiff1d(np.arange(y.shape[1], dtype=np.int64), payload_cols)
    if other_cols.size == 0:
        return tau.copy()

    theta_other = np.array([float(baseline.get(param_names[index], 0.0)) for index in other_cols], dtype=np.float64)
    return tau - y[:, other_cols] @ theta_other


def validate_payload_regressor(
    y_payload: np.ndarray,
    *,
    min_rank: int | None = None,
    min_column_norm: float = 1e-10,
) -> int:
    y = np.asarray(y_payload, dtype=np.float64)
    if y.ndim != 2:
        raise ValueError(f"payload regressor must be 2D, got {y.shape}")
    if y.shape[1] != len(IDENTIFIED_NAMES):
        raise ValueError(f"payload regressor must have {len(IDENTIFIED_NAMES)} columns, got {y.shape[1]}")
    norms = np.linalg.norm(y, axis=0)
    if np.any(norms < float(min_column_norm)):
        weak = np.flatnonzero(norms < float(min_column_norm)).tolist()
        raise RuntimeError(f"payload excitation is too weak for columns {weak}")

    col_scale = np.maximum(norms, 1e-12)
    singular_values = np.linalg.svd(y / col_scale, compute_uv=False)
    rank = int(np.linalg.matrix_rank(y / col_scale, tol=Config.PARAM_ID_RCOND))
    required_rank = y.shape[1] if min_rank is None else int(min_rank)
    if rank < required_rank:
        sigma_min = float(singular_values[-1]) if singular_values.size else 0.0
        raise RuntimeError(
            "payload excitation rank is too low: "
            f"rank={rank} < {required_rank}, sigma_min={sigma_min:.3e}"
        )
    return rank


def payload_prior_from_link_params(
    mass: float,
    com: Iterable[float],
    inertia_diag: Iterable[float],
    *,
    link_index: int = PAYLOAD_LINK_INDEX,
) -> dict[str, float]:
    m = float(mass)
    com_arr = np.asarray(com, dtype=np.float64)
    inertia = np.asarray(inertia_diag, dtype=np.float64)
    if com_arr.shape != (3,):
        raise ValueError(f"com must have shape (3,), got {com_arr.shape}")
    if inertia.shape != (3,):
        raise ValueError(f"inertia_diag must have shape (3,), got {inertia.shape}")
    prefix = f"L{int(link_index)}_"
    return {
        f"{prefix}mass": m,
        f"{prefix}mcx": m * float(com_arr[0]),
        f"{prefix}mcy": m * float(com_arr[1]),
        f"{prefix}mcz": m * float(com_arr[2]),
        f"{prefix}Ixx": float(inertia[0]),
        f"{prefix}Iyy": float(inertia[1]),
        f"{prefix}Izz": float(inertia[2]),
    }


def solve_payload_only(
    y_payload: np.ndarray,
    tau_stack: np.ndarray,
    payload_names: list[str],
    *,
    prior: dict[str, float] | None = None,
    min_rank: int | None = None,
    inertial_prior_lambda: float | None = None,
) -> PayloadIdentificationResult:
    rank = validate_payload_regressor(y_payload, min_rank=min_rank)
    regularization = Config.PARAM_ID_PRIOR_LAMBDA_INERTIAL if inertial_prior_lambda is None else inertial_prior_lambda
    result = solve_least_squares(
        y_payload,
        tau_stack,
        payload_names,
        prior=prior,
        inertial_prior_lambda=float(regularization),
        mass_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_MASS,
        com_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_COM,
        inertia_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIA,
        joint_prior_lambda=0.0,
        rcond=Config.PARAM_ID_RCOND,
        ridge=Config.PARAM_ID_RIDGE,
    )
    mass = float(result[payload_names[0]])
    if mass <= 1e-9 or not np.isfinite(mass):
        raise RuntimeError(f"identified payload mass is not physically valid: {mass:.6g}")
    com = np.array(
        [
            result[payload_names[1]] / mass,
            result[payload_names[2]] / mass,
            result[payload_names[3]] / mass,
        ],
        dtype=np.float64,
    )
    inertia = np.array(
        [
            result[payload_names[4]],
            result[payload_names[5]],
            result[payload_names[6]],
        ],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(com)) or np.max(np.abs(com)) > 1.0:
        raise RuntimeError(f"identified payload COM is not physically plausible: {com.tolist()}")
    if not np.all(np.isfinite(inertia)) or np.any(inertia <= 0.0):
        raise RuntimeError(f"identified payload inertia is not physically valid: {inertia.tolist()}")

    return PayloadIdentificationResult(
        values=result,
        mass=mass,
        com=com,
        inertia_diag=inertia,
        condition_number=compute_condition_number(y_payload),
        prediction_error_rms=compute_prediction_error(y_payload, tau_stack, result, payload_names),
        rank=rank,
    )


def build_payload_regressor(
    backend,
    q_seq: np.ndarray,
    qd_seq: np.ndarray,
    qdd_seq: np.ndarray,
    *,
    stride: int = 1,
    link_index: int = PAYLOAD_LINK_INDEX,
) -> tuple[np.ndarray, list[str]]:
    y_stack, param_names = build_stacked_regressor(
        backend,
        q_seq,
        qd_seq,
        qdd_seq,
        stride=stride,
        include_joint_terms=False,
    )
    return select_payload_regressor(y_stack, param_names, link_index=link_index)
