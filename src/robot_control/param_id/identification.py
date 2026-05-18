"""Parameter identification via numerically stable least-squares."""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np


_LAST_DIAGNOSTICS: Dict[str, float] = {}


def _as_prior_vector(param_names: List[str], prior: Dict[str, float] | None) -> np.ndarray:
    if prior is None:
        return np.zeros(len(param_names), dtype=np.float64)
    return np.array([float(prior.get(name, 0.0)) for name in param_names], dtype=np.float64)


def _link_index_from_name(name: str) -> int | None:
    if not name.startswith("L"):
        return None
    head = name.split("_", 1)[0]
    try:
        return int(head[1:])
    except ValueError:
        return None


def _link_excitation_quality(Y_stack: np.ndarray, param_names: List[str]) -> np.ndarray:
    """Estimate per-link excitation quality in ``(0, 1]`` from available columns."""
    Y = np.asarray(Y_stack, dtype=np.float64)
    link_to_cols: Dict[int, List[int]] = {}
    for col, name in enumerate(param_names):
        link = _link_index_from_name(name)
        if link is not None:
            link_to_cols.setdefault(link, []).append(col)
    if not link_to_cols:
        return np.ones(0, dtype=np.float64)

    qualities = np.ones(max(link_to_cols) + 1, dtype=np.float64)
    for link, cols in link_to_cols.items():
        block = Y[:, cols]
        norms = np.linalg.norm(block, axis=0)
        active = norms > 1e-12
        if not np.any(active):
            qualities[link] = 1e-6
            continue
        block = block[:, active] / norms[active]
        singular_values = np.linalg.svd(block, compute_uv=False)
        if singular_values.size == 0 or singular_values[0] <= 1e-12:
            quality = 1e-6
        else:
            quality = singular_values[-1] / singular_values[0]
        qualities[link] = float(np.clip(quality, 1e-6, 1.0))
    return qualities


def _prior_weights(
    param_names: List[str],
    inertial_lambda: float,
    joint_lambda: float,
    mass_lambda: float | None = None,
    com_lambda: float | None = None,
    inertia_lambda: float | None = None,
    link_excitation: np.ndarray | None = None,
) -> np.ndarray:
    mass_lambda = inertial_lambda if mass_lambda is None else mass_lambda
    com_lambda = inertial_lambda if com_lambda is None else com_lambda
    inertia_lambda = inertial_lambda if inertia_lambda is None else inertia_lambda
    quality = None if link_excitation is None else np.asarray(link_excitation, dtype=np.float64)
    weights = np.empty(len(param_names), dtype=np.float64)
    for i, name in enumerate(param_names):
        if name.startswith("J"):
            weights[i] = joint_lambda
        elif name.endswith("_mass"):
            weights[i] = mass_lambda
        elif name.endswith(("_mcx", "_mcy", "_mcz")):
            weights[i] = com_lambda
        elif name.endswith(("_Ixx", "_Iyy", "_Izz")):
            weights[i] = inertia_lambda
        else:
            weights[i] = inertial_lambda
        link = _link_index_from_name(name)
        if link is not None and quality is not None and link < quality.size:
            weights[i] *= 1.0 / np.sqrt(max(float(quality[link]), 1e-6))
    return np.maximum(weights, 0.0)


def _natural_scale_floor(name: str) -> float:
    if name.endswith("_mass"):
        return 1.0
    if name.endswith(("_mcx", "_mcy", "_mcz")):
        return 1e-2
    if name.endswith(("_Ixx", "_Iyy", "_Izz")):
        return 1e-3
    return 1.0


def _relative_scale(theta_prior: np.ndarray, param_names: List[str]) -> np.ndarray:
    floors = np.array([_natural_scale_floor(name) for name in param_names], dtype=np.float64)
    return np.maximum(np.abs(theta_prior), floors)


def solve_least_squares(
    Y_stack: np.ndarray,
    tau_stack: np.ndarray,
    param_names: List[str],
    prior: Dict[str, float] | None = None,
    rcond: float = 1e-8,
    ridge: float = 1e-8,
    inertial_prior_lambda: float = 1e-3,
    mass_prior_lambda: float | None = None,
    com_prior_lambda: float | None = None,
    inertia_prior_lambda: float | None = None,
    joint_prior_lambda: float = 1e-1,
) -> Dict[str, float]:
    """Solve with column scaling, SVD truncation, and grouped prior regularization."""
    global _LAST_DIAGNOSTICS

    Y = np.asarray(Y_stack, dtype=np.float64)
    tau = np.asarray(tau_stack, dtype=np.float64)
    theta_prior = _as_prior_vector(param_names, prior)

    col_scale = np.linalg.norm(Y, axis=0)
    col_scale = np.where(col_scale > 1e-12, col_scale, 1.0)
    Ys = Y / col_scale
    prior_scaled = theta_prior * col_scale

    data_singular_values = np.linalg.svd(Ys, compute_uv=False)
    if data_singular_values.size == 0:
        data_rank = 0
    else:
        data_rank = int(np.count_nonzero(data_singular_values > (float(rcond) * data_singular_values[0])))

    link_excitation = _link_excitation_quality(Y, param_names)
    prior_weights = _prior_weights(
        param_names,
        inertial_prior_lambda,
        joint_prior_lambda,
        mass_lambda=mass_prior_lambda,
        com_lambda=com_prior_lambda,
        inertia_lambda=inertia_prior_lambda,
        link_excitation=link_excitation,
    )
    if np.any(prior_weights > 0.0):
        rel = _relative_scale(theta_prior, param_names)
        reg_diag = np.sqrt(prior_weights) / (rel * col_scale)
        active = reg_diag > 0.0
        active_cols = np.flatnonzero(active)
        reg_rows = np.zeros((active_cols.size, len(param_names)), dtype=np.float64)
        reg_rows[np.arange(active_cols.size), active_cols] = reg_diag[active]
        A_aug = np.vstack([Ys, reg_rows])
        b_aug = np.concatenate([tau, reg_rows @ prior_scaled])
    else:
        A_aug = Ys
        b_aug = tau

    U, singular_values, Vt = np.linalg.svd(A_aug, full_matrices=False)
    if singular_values.size == 0:
        retained = np.zeros(0, dtype=bool)
    else:
        retained = singular_values > (float(rcond) * singular_values[0])
    rank = int(np.count_nonzero(retained))

    if rank > 0:
        Sr = singular_values[retained]
        Ur = U[:, retained]
        Vtr = Vt[retained]
        filtered = Sr / (Sr * Sr + float(ridge))
        theta_scaled = Vtr.T @ (filtered * (Ur.T @ b_aug))
    else:
        theta_scaled = prior_scaled.copy()
    theta = theta_scaled / col_scale

    if singular_values.size > 0:
        full_condition = singular_values[0] / max(singular_values[-1], 1e-15)
        retained_condition = singular_values[0] / max(singular_values[rank - 1], 1e-15) if rank else float("inf")
    else:
        full_condition = float("inf")
        retained_condition = float("inf")

    residual = tau - Y @ theta
    prior_delta = theta - theta_prior
    mass_mask = np.array([name.endswith("_mass") for name in param_names], dtype=bool)
    com_mask = np.array([name.endswith(("_mcx", "_mcy", "_mcz")) for name in param_names], dtype=bool)
    inertia_mask = np.array([name.endswith(("_Ixx", "_Iyy", "_Izz")) for name in param_names], dtype=bool)
    inertial_mask = mass_mask | com_mask | inertia_mask
    joint_mask = ~inertial_mask
    nullspace_residual = 0.0
    if Vt.shape[0] < len(param_names):
        nullspace_residual = float("nan")
    elif rank < len(param_names):
        null_basis = Vt[rank:].T / col_scale[:, None]
        nullspace_residual = float(np.linalg.norm(null_basis.T @ prior_delta))

    _LAST_DIAGNOSTICS = {
        "num_params": float(len(param_names)),
        "rank": float(rank),
        "data_rank": float(data_rank),
        "nullity": float(len(param_names) - rank),
        "scaled_condition": float(full_condition),
        "retained_condition": float(retained_condition),
        "residual_rms": float(np.sqrt(np.mean(residual**2))) if residual.size else 0.0,
        "prior_delta_rms": float(np.sqrt(np.mean(prior_delta**2))) if prior_delta.size else 0.0,
        "inertial_prior_delta_rms": float(np.sqrt(np.mean(prior_delta[inertial_mask] ** 2))) if np.any(inertial_mask) else 0.0,
        "mass_prior_delta_rms": float(np.sqrt(np.mean(prior_delta[mass_mask] ** 2))) if np.any(mass_mask) else 0.0,
        "com_prior_delta_rms": float(np.sqrt(np.mean(prior_delta[com_mask] ** 2))) if np.any(com_mask) else 0.0,
        "inertia_prior_delta_rms": float(np.sqrt(np.mean(prior_delta[inertia_mask] ** 2))) if np.any(inertia_mask) else 0.0,
        "joint_prior_delta_rms": float(np.sqrt(np.mean(prior_delta[joint_mask] ** 2))) if np.any(joint_mask) else 0.0,
        "nullspace_prior_delta_norm": nullspace_residual,
        "link_excitation_min": float(np.min(link_excitation)) if link_excitation.size else 1.0,
        "link_excitation_mean": float(np.mean(link_excitation)) if link_excitation.size else 1.0,
    }

    return {name: float(value) for name, value in zip(param_names, theta)}


def solve_weighted_least_squares(
    Y_stack: np.ndarray,
    tau_stack: np.ndarray,
    param_names: List[str],
    weights: np.ndarray = None,
    prior: Dict[str, float] | None = None,
    inertial_prior_lambda: float = 1e-3,
    mass_prior_lambda: float | None = None,
    com_prior_lambda: float | None = None,
    inertia_prior_lambda: float | None = None,
    joint_prior_lambda: float = 1e-1,
) -> Dict[str, float]:
    Y = np.asarray(Y_stack, dtype=np.float64)
    tau = np.asarray(tau_stack, dtype=np.float64)
    if weights is None:
        return solve_least_squares(
            Y,
            tau,
            param_names,
            prior=prior,
            inertial_prior_lambda=inertial_prior_lambda,
            mass_prior_lambda=mass_prior_lambda,
            com_prior_lambda=com_prior_lambda,
            inertia_prior_lambda=inertia_prior_lambda,
            joint_prior_lambda=joint_prior_lambda,
        )

    w = np.sqrt(np.asarray(weights, dtype=np.float64))
    return solve_least_squares(
        Y * w[:, None],
        tau * w,
        param_names,
        prior=prior,
        inertial_prior_lambda=inertial_prior_lambda,
        mass_prior_lambda=mass_prior_lambda,
        com_prior_lambda=com_prior_lambda,
        inertia_prior_lambda=inertia_prior_lambda,
        joint_prior_lambda=joint_prior_lambda,
    )


def get_last_diagnostics() -> Dict[str, float]:
    return dict(_LAST_DIAGNOSTICS)


def make_prior_from_link_params(
    param_names: List[str],
    masses: List[float],
    coms: List[List[float]],
    inertias: List[List[float]],
    joint_priors: List[Dict[str, float]] | None = None,
) -> Dict[str, float]:
    prior: Dict[str, float] = {}
    for link in range(7):
        prefix = f"L{link}_"
        mass = float(masses[link])
        com = np.asarray(coms[link], dtype=np.float64)
        inertia = np.asarray(inertias[link], dtype=np.float64)
        prior[f"{prefix}mass"] = mass
        prior[f"{prefix}mcx"] = mass * float(com[0])
        prior[f"{prefix}mcy"] = mass * float(com[1])
        prior[f"{prefix}mcz"] = mass * float(com[2])
        prior[f"{prefix}Ixx"] = float(inertia[0])
        prior[f"{prefix}Iyy"] = float(inertia[1])
        prior[f"{prefix}Izz"] = float(inertia[2])

    if joint_priors is not None:
        for joint, values in enumerate(joint_priors):
            for name in ("fc", "k", "fv", "fo"):
                prior[f"J{joint + 1}_{name}"] = float(values[name])

    return {name: prior.get(name, 0.0) for name in param_names}


def to_link_params(
    identified: Dict[str, float],
    prior: Dict[str, float] | None = None,
) -> Tuple[List[float], List[List[float]], List[List[float]]]:
    """Convert flat identified dict to per-link lists.

    Returns (masses[7], coms[7][3], inertias[7][3]).
    """
    masses = []
    coms = []
    inertias = []

    for link in range(7):
        prefix = f"L{link}_"
        m = float(identified.get(f"{prefix}mass", 0.0))
        mcx = float(identified.get(f"{prefix}mcx", 0.0))
        mcy = float(identified.get(f"{prefix}mcy", 0.0))
        mcz = float(identified.get(f"{prefix}mcz", 0.0))
        Ixx = float(identified.get(f"{prefix}Ixx", 0.0))
        Iyy = float(identified.get(f"{prefix}Iyy", 0.0))
        Izz = float(identified.get(f"{prefix}Izz", 0.0))

        if prior is not None and (m <= 1e-5 or not np.isfinite(m)):
            m = float(prior.get(f"{prefix}mass", max(m, 1e-5)))
            mcx = float(prior.get(f"{prefix}mcx", mcx))
            mcy = float(prior.get(f"{prefix}mcy", mcy))
            mcz = float(prior.get(f"{prefix}mcz", mcz))

        m = max(m, 1e-5)
        com = np.array([mcx / m, mcy / m, mcz / m], dtype=np.float64)
        if prior is not None and (not np.all(np.isfinite(com)) or np.max(np.abs(com)) > 0.5):
            pm = max(float(prior.get(f"{prefix}mass", m)), 1e-5)
            com = np.array([
                float(prior.get(f"{prefix}mcx", 0.0)) / pm,
                float(prior.get(f"{prefix}mcy", 0.0)) / pm,
                float(prior.get(f"{prefix}mcz", 0.0)) / pm,
            ])

        inertia = np.array([Ixx, Iyy, Izz], dtype=np.float64)
        if prior is not None and (not np.all(np.isfinite(inertia)) or np.any(inertia <= 1e-8)):
            inertia = np.array([
                float(prior.get(f"{prefix}Ixx", max(Ixx, 1e-8))),
                float(prior.get(f"{prefix}Iyy", max(Iyy, 1e-8))),
                float(prior.get(f"{prefix}Izz", max(Izz, 1e-8))),
            ])
        inertia = np.maximum(inertia, 1e-8)

        masses.append(float(m))
        coms.append([float(com[0]), float(com[1]), float(com[2])])
        inertias.append([float(inertia[0]), float(inertia[1]), float(inertia[2])])

    return masses, coms, inertias


def compute_condition_number(Y_stack: np.ndarray) -> float:
    """Scaled condition number of the stacked regressor matrix."""
    Y = np.asarray(Y_stack, dtype=np.float64)
    col_scale = np.linalg.norm(Y, axis=0)
    col_scale = np.where(col_scale > 1e-12, col_scale, 1.0)
    singular_values = np.linalg.svd(Y / col_scale, compute_uv=False)
    if singular_values.size == 0:
        return float("inf")
    return float(singular_values[0] / max(singular_values[-1], 1e-15))


def compute_prediction_error(
    Y_stack: np.ndarray,
    tau_stack: np.ndarray,
    identified: Dict[str, float],
    param_names: List[str],
) -> float:
    """RMS prediction error: ||tau - Y * pi|| / sqrt(N)."""
    Y = np.asarray(Y_stack, dtype=np.float64)
    tau = np.asarray(tau_stack, dtype=np.float64)
    pi = np.array([identified[n] for n in param_names])
    residuals = tau - Y @ pi
    return float(np.sqrt(np.mean(residuals**2)))
