"""Compatibility exports for parameter-identification solver cases."""

from __future__ import annotations

from param_id.identification import (
    compute_condition_number,
    compute_prediction_error,
    get_last_diagnostics,
    make_prior_from_link_params,
    solve_least_squares,
    to_link_params,
)

__all__ = [
    "compute_condition_number",
    "compute_prediction_error",
    "get_last_diagnostics",
    "make_prior_from_link_params",
    "solve_least_squares",
    "to_link_params",
]

