#!/usr/bin/env python3
"""Diagnostics, observability, and solver-case helpers for parameter identification."""

from __future__ import annotations

import numpy as np

from robot_control.config import Config
from robot_control.dynamics.pinocchio import PinocchioGravityBackend
from robot_control.param_id.regressor import build_stacked_regressor
from robot_control.param_id.identification import (
    compute_condition_number,
    compute_prediction_error,
    get_last_diagnostics,
    make_prior_from_link_params,
    solve_least_squares,
    to_link_params,
)


def _extract_ground_truth(backend: PinocchioGravityBackend):
    model = backend._model
    masses, coms, inertias = [], [], []
    for i in range(1, 8):
        inert = model.inertias[i]
        masses.append(float(inert.mass))
        com = inert.lever
        coms.append([float(com[0]), float(com[1]), float(com[2])])
        I = inert.inertia
        inertias.append([float(I[0, 0]), float(I[1, 1]), float(I[2, 2])])
    return masses, coms, inertias

def _joint_effect_torque(q, qd, priors, q_ref):
    tau = np.zeros(7, dtype=np.float64)
    eps = Config.PARAM_ID_COULOMB_EPS
    for i, params in enumerate(priors):
        tau[i] = (
            params["fc"] * np.tanh(qd[i] / eps)
            + params["k"] * (q[i] - q_ref[i])
            + params["fv"] * qd[i]
            + params["fo"]
        )
    return tau

def _finite_float(value, default=float("nan")):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default

def _as_joint_matrix(values):
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 7:
        return np.zeros((0, 7), dtype=np.float64)
    return arr[:, :7]

def _joint_term_values_from_result(result, priors=None):
    values = []
    priors = Config.PARAM_ID_JOINT_PRIORS if priors is None else priors
    for joint, prior in enumerate(priors, start=1):
        row = {}
        for term in ("fc", "k", "fv", "fo"):
            row[term] = _finite_float(
                result.get(f"J{joint}_{term}", prior.get(term, 0.0)),
                prior.get(term, 0.0),
            )
        values.append(row)
    return values

def _joint_term_error_summary(result, q_meas=None, qd_meas=None, q_ref=None, priors=None):
    priors = Config.PARAM_ID_JOINT_PRIORS if priors is None else priors
    identified = _joint_term_values_from_result(result or {}, priors=priors)
    param_rows = []
    abs_errors = []
    rel_errors = []
    for joint, (identified_terms, prior_terms) in enumerate(zip(identified, priors), start=1):
        for term in ("fc", "k", "fv", "fo"):
            prior_value = float(prior_terms.get(term, 0.0))
            identified_value = float(identified_terms.get(term, prior_value))
            error = identified_value - prior_value
            abs_error = abs(error)
            rel_error = abs_error / abs(prior_value) * 100.0 if abs(prior_value) > 1e-12 else 0.0
            param_rows.append(
                {
                    "parameter": f"J{joint}_{term}",
                    "joint": joint,
                    "term": term,
                    "identified": identified_value,
                    "prior": prior_value,
                    "error": error,
                    "abs_error": abs_error,
                    "relative_error_pct": rel_error,
                }
            )
            abs_errors.append(abs_error)
            rel_errors.append(rel_error)

    n_joints = Config.NUM_JOINTS
    q = _as_joint_matrix(q_meas) if q_meas is not None else np.zeros((0, n_joints), dtype=np.float64)
    qd = _as_joint_matrix(qd_meas) if qd_meas is not None else np.zeros((0, n_joints), dtype=np.float64)
    count = min(len(q), len(qd))
    torque_error = np.zeros((0, n_joints), dtype=np.float64)
    if count:
        q = q[:count]
        qd = qd[:count]
        q_ref_arr = np.asarray(Config.HOME_QPOS if q_ref is None else q_ref, dtype=np.float64)
        torque_identified = np.zeros((count, n_joints), dtype=np.float64)
        torque_prior = np.zeros((count, n_joints), dtype=np.float64)
        for step in range(count):
            torque_identified[step] = _joint_effect_torque(q[step], qd[step], identified, q_ref_arr)
            torque_prior[step] = _joint_effect_torque(q[step], qd[step], priors, q_ref_arr)
        torque_error = torque_identified - torque_prior

    if torque_error.size:
        per_joint_rms = np.sqrt(np.mean(torque_error ** 2, axis=0))
        per_joint_max = np.max(np.abs(torque_error), axis=0)
        torque_rms = float(np.sqrt(np.mean(torque_error ** 2)))
        torque_max = float(np.max(np.abs(torque_error)))
        torque_max_joint = int(np.argmax(per_joint_max)) + 1
    else:
        per_joint_rms = np.zeros(n_joints, dtype=np.float64)
        per_joint_max = np.zeros(n_joints, dtype=np.float64)
        torque_rms = 0.0
        torque_max = 0.0
        torque_max_joint = 1

    max_abs_idx = int(np.argmax(abs_errors)) if abs_errors else 0
    max_rel_idx = int(np.argmax(rel_errors)) if rel_errors else 0
    return {
        "max_abs_param_error": float(abs_errors[max_abs_idx]) if abs_errors else 0.0,
        "max_abs_param": param_rows[max_abs_idx]["parameter"] if param_rows else "",
        "max_relative_param_error_pct": float(rel_errors[max_rel_idx]) if rel_errors else 0.0,
        "max_relative_param": param_rows[max_rel_idx]["parameter"] if param_rows else "",
        "param_error_rms": float(np.sqrt(np.mean(np.asarray(abs_errors, dtype=np.float64) ** 2))) if abs_errors else 0.0,
        "torque_rms": torque_rms,
        "torque_max_abs": torque_max,
        "torque_max_abs_joint": torque_max_joint,
        "sample_count": int(count),
        "reference": "PARAM_ID_JOINT_PRIORS",
        "error_definition": "identified - prior",
        "torque_error_model": "fc*tanh(qd/eps) + k*(q-q_ref) + fv*qd + fo",
        "per_param": param_rows,
        "per_joint": [
            {
                "joint": joint + 1,
                "torque_rms": float(per_joint_rms[joint]),
                "torque_max_abs": float(per_joint_max[joint]),
            }
            for joint in range(n_joints)
        ],
    }

def _scaled_svd_metrics(Y, rcond=None):
    Y = np.asarray(Y, dtype=np.float64)
    if Y.size == 0:
        return {"rank": 0, "condition": float("inf"), "sigma_min": 0.0}
    col_norm = np.maximum(np.linalg.norm(Y, axis=0), 1e-12)
    s = np.linalg.svd(Y / col_norm, compute_uv=False)
    if s.size == 0:
        return {"rank": 0, "condition": float("inf"), "sigma_min": 0.0}
    cutoff = (Config.PARAM_ID_RCOND if rcond is None else rcond) * s[0]
    rank = int(np.count_nonzero(s > cutoff))
    return {
        "rank": rank,
        "condition": float(s[0] / max(s[-1], 1e-15)),
        "sigma_min": float(s[rank - 1]) if rank else 0.0,
    }

def _distal_column_groups(n_cols, include_joint_terms=False):
    per_link = 7
    distal_start = (Config.PARAM_ID_DISTAL_LINK_START - 1) * per_link
    inertial_cols = 7 * per_link
    distal_cols = np.arange(distal_start, min(inertial_cols, n_cols))
    other_cols = np.setdiff1d(np.arange(n_cols), distal_cols)
    return distal_cols, other_cols

def _parameter_column_groups(n_cols):
    per_link = 7
    inertial_cols = min(7 * per_link, int(n_cols))
    groups = {
        "mass": [],
        "com": [],
        "inertia": [],
        "distal_com": [],
        "distal_inertia": [],
        "joint": list(range(inertial_cols, int(n_cols))),
    }
    distal_start = Config.PARAM_ID_DISTAL_LINK_START - 1
    for link in range(7):
        base = link * per_link
        if base >= inertial_cols:
            continue
        mass_cols = [base]
        com_cols = [base + offset for offset in (1, 2, 3) if base + offset < inertial_cols]
        inertia_cols = [base + offset for offset in (4, 5, 6) if base + offset < inertial_cols]
        groups["mass"].extend(mass_cols)
        groups["com"].extend(com_cols)
        groups["inertia"].extend(inertia_cols)
        if link >= distal_start:
            groups["distal_com"].extend(com_cols)
            groups["distal_inertia"].extend(inertia_cols)
    return {name: np.asarray(cols, dtype=np.int64) for name, cols in groups.items()}

def _column_group_observability(Y, target_cols, basis_cols=None):
    Y = np.asarray(Y, dtype=np.float64)
    target_cols = np.asarray(target_cols, dtype=np.int64)
    if basis_cols is None:
        basis_cols = np.setdiff1d(np.arange(Y.shape[1]), target_cols)
    else:
        basis_cols = np.asarray(basis_cols, dtype=np.int64)

    if Y.size == 0 or target_cols.size == 0:
        return {
            "rank": 0,
            "condition": float("inf"),
            "sigma_min": 0.0,
            "correlation": 0.0,
            "projection": {"ratio": 0.0, "rank": 0, "condition": float("inf"), "sigma_min": 0.0},
        }

    metrics = _scaled_svd_metrics(Y[:, target_cols])
    Yn = Y / np.maximum(np.linalg.norm(Y, axis=0), 1e-12)
    if basis_cols.size:
        corr = float(np.max(np.abs(Yn[:, target_cols].T @ Yn[:, basis_cols])))
    else:
        corr = 0.0
    projection = _projection_residual_metrics(Y, target_cols, basis_cols)
    return {**metrics, "correlation": corr, "projection": projection}

def _parameter_group_observability(Y):
    groups = _parameter_column_groups(Y.shape[1])
    return {
        name: _column_group_observability(Y, cols)
        for name, cols in groups.items()
        if name != "joint"
    }

def _projection_residual_metrics(Y, target_cols, basis_cols):
    Y = np.asarray(Y, dtype=np.float64)
    target_cols = np.asarray(target_cols, dtype=np.int64)
    basis_cols = np.asarray(basis_cols, dtype=np.int64)
    if Y.size == 0 or target_cols.size == 0:
        return {"ratio": 0.0, "rank": 0, "condition": float("inf"), "sigma_min": 0.0}

    Yn = Y / np.maximum(np.linalg.norm(Y, axis=0), 1e-12)
    target = Yn[:, target_cols]
    if basis_cols.size:
        basis = Yn[:, basis_cols]
        coeff, *_ = np.linalg.lstsq(basis, target, rcond=Config.PARAM_ID_RCOND)
        residual = target - basis @ coeff
    else:
        residual = target

    denom = max(float(np.linalg.norm(target)), 1e-12)
    metrics = _scaled_svd_metrics(residual)
    return {**metrics, "ratio": float(np.linalg.norm(residual) / denom)}

def _distal_observability(Y, include_joint_terms=False):
    distal_cols, other_cols = _distal_column_groups(Y.shape[1], include_joint_terms=include_joint_terms)
    distal_metrics = _scaled_svd_metrics(Y[:, distal_cols])

    Yn = Y / np.maximum(np.linalg.norm(Y, axis=0), 1e-12)
    if other_cols.size and distal_cols.size:
        corr = float(np.max(np.abs(Yn[:, distal_cols].T @ Yn[:, other_cols])))
    else:
        corr = 0.0
    projection = _projection_residual_metrics(Y, distal_cols, other_cols)
    condition_term = np.log10(max(distal_metrics["condition"], 1.0))
    sigma_term = np.log10(max(distal_metrics["sigma_min"], 1e-15) / 1e-15)
    residual_sigma_term = np.log10(max(projection["sigma_min"], 1e-15) / 1e-15)
    score = (
        distal_metrics["rank"] * Config.PARAM_ID_DISTAL_WEIGHT
        + projection["rank"] * Config.PARAM_ID_DISTAL_WEIGHT
        + sigma_term
        + residual_sigma_term
        + projection["ratio"] * 8.0
        - 2.0 * corr
        - 0.15 * condition_term
    )
    return {**distal_metrics, "correlation": corr, "projection": projection, "score": float(score)}

def _mass_error_summary(masses, true_masses):
    errors = []
    for mass, true_mass in zip(masses, true_masses):
        if true_mass > 1e-9:
            errors.append((float(mass) - float(true_mass)) / float(true_mass) * 100.0)
        else:
            errors.append(0.0)
    abs_errors = [abs(err) for err in errors]
    distal_start = Config.PARAM_ID_DISTAL_LINK_START - 1
    distal_abs = abs_errors[distal_start:]
    max_abs = float(np.max(abs_errors)) if abs_errors else 0.0
    max_idx = int(np.argmax(abs_errors)) if abs_errors else 0
    target = float(Config.PARAM_ID_MASS_ERROR_TARGET_PCT)
    return {
        "errors": errors,
        "abs_errors": abs_errors,
        "max_abs": max_abs,
        "max_abs_joint": max_idx + 1,
        "passes_5pct": max_abs <= target,
        "target_pct": target,
        "j7_abs": abs(errors[6]) if len(errors) >= 7 else 0.0,
        "distal_abs_mean": float(np.mean(distal_abs)) if distal_abs else 0.0,
    }

def _com_error_summary(coms, true_coms):
    com_arr = np.asarray(coms, dtype=np.float64)
    true_arr = np.asarray(true_coms, dtype=np.float64)
    if com_arr.shape != true_arr.shape:
        raise ValueError("coms and true_coms must have the same shape")

    error_vectors = com_arr - true_arr
    distance_errors = np.linalg.norm(error_vectors, axis=1)
    distal_start = Config.PARAM_ID_DISTAL_LINK_START - 1
    distal_distances = distance_errors[distal_start:]
    max_idx = int(np.argmax(distance_errors)) if distance_errors.size else 0
    target_m = float(getattr(Config, "PARAM_ID_COM_ERROR_TARGET_M", 0.01))
    return {
        "error_vectors": error_vectors.tolist(),
        "distance_errors": distance_errors.tolist(),
        "max_distance": float(np.max(distance_errors)) if distance_errors.size else 0.0,
        "max_distance_joint": max_idx + 1,
        "distal_distance_mean": float(np.mean(distal_distances)) if distal_distances.size else 0.0,
        "target_m": target_m,
        "passes_target": bool(np.max(distance_errors) <= target_m) if distance_errors.size else True,
    }

def _inertia_error_summary(inertias, true_inertias):
    inertia_arr = np.asarray(inertias, dtype=np.float64)
    true_arr = np.asarray(true_inertias, dtype=np.float64)
    if inertia_arr.shape != true_arr.shape:
        raise ValueError("inertias and true_inertias must have the same shape")

    rel = np.zeros_like(inertia_arr, dtype=np.float64)
    valid = np.abs(true_arr) > 1e-9
    rel[valid] = (inertia_arr[valid] - true_arr[valid]) / true_arr[valid] * 100.0
    abs_rel = np.abs(rel)
    link_l2 = np.linalg.norm(abs_rel, axis=1)
    distal_start = Config.PARAM_ID_DISTAL_LINK_START - 1
    distal_l2 = link_l2[distal_start:]
    max_flat = int(np.argmax(abs_rel)) if abs_rel.size else 0
    max_joint, max_axis = divmod(max_flat, 3) if abs_rel.size else (0, 0)
    axis_names = ("Ixx", "Iyy", "Izz")
    target_pct = float(getattr(Config, "PARAM_ID_INERTIA_ERROR_TARGET_PCT", 15.0))
    return {
        "relative_errors": rel.tolist(),
        "absolute_relative_errors": abs_rel.tolist(),
        "link_l2_errors": link_l2.tolist(),
        "max_component_abs": float(np.max(abs_rel)) if abs_rel.size else 0.0,
        "max_component_joint": int(max_joint) + 1,
        "max_component_axis": axis_names[int(max_axis)],
        "max_link_l2": float(np.max(link_l2)) if link_l2.size else 0.0,
        "max_link_l2_joint": int(np.argmax(link_l2)) + 1 if link_l2.size else 1,
        "distal_l2_mean": float(np.mean(distal_l2)) if distal_l2.size else 0.0,
        "target_pct": target_pct,
        "passes_target": bool(np.max(abs_rel) <= target_pct) if abs_rel.size else True,
    }

def _case_selection_key(case):
    summary = case["mass_summary"]
    com_summary = case["com_summary"]
    inertia_summary = case["inertia_summary"]
    diagnostics = case.get("diagnostics", {})
    num_params = diagnostics.get("num_params", 0.0)
    rank = diagnostics.get("rank", 0.0)
    rank_failure = num_params >= 69 and rank < 69
    mass_norm = summary["max_abs"] / max(summary["target_pct"], 1e-12)
    com_norm = com_summary["max_distance"] / max(com_summary["target_m"], 1e-12)
    inertia_norm = inertia_summary["max_component_abs"] / max(inertia_summary["target_pct"], 1e-12)
    distal_com_norm = com_summary["distal_distance_mean"] / max(com_summary["target_m"], 1e-12)
    distal_inertia_norm = inertia_summary["distal_l2_mean"] / max(inertia_summary["target_pct"], 1e-12)
    return (
        summary["max_abs"] > summary["target_pct"],
        not com_summary["passes_target"],
        not inertia_summary["passes_target"],
        mass_norm,
        com_norm,
        inertia_norm,
        distal_com_norm,
        distal_inertia_norm,
        inertia_summary["max_link_l2"] / max(inertia_summary["target_pct"], 1e-12),
        summary["j7_abs"] / max(summary["target_pct"], 1e-12),
        summary["distal_abs_mean"] / max(summary["target_pct"], 1e-12),
        case["prediction_error"],
        case.get("validation_rms", float("inf")),
        case.get("validation_ratio", float("inf")),
        rank_failure,
        -rank,
        -diagnostics.get("data_rank", 0),
        -case["inertial_distal"]["projection"]["ratio"],
    )

def _j7_column_diagnostics(Y):
    Y = np.asarray(Y, dtype=np.float64)
    if Y.size == 0 or Y.shape[1] < 49:
        return {"mass_norm": 0.0, "mean_norm": 0.0, "max_norm": 0.0, "min_norm": 0.0}
    cols = np.arange(42, 49)
    norms = np.linalg.norm(Y[:, cols], axis=0)
    return {
        "mass_norm": float(norms[0]),
        "mean_norm": float(np.mean(norms)),
        "max_norm": float(np.max(norms)),
        "min_norm": float(np.min(norms)),
    }

def _segment_indices(labels, tag):
    labels = np.asarray(labels, dtype=object)
    if labels.size == 0:
        return np.array([], dtype=np.int64)
    return np.flatnonzero(labels == tag)

def _segment_prediction_rms(Y_stack, tau_stack, result, param_names, row_indices):
    if row_indices.size == 0:
        return float("nan")
    Y_sel = Y_stack[row_indices, :]
    tau_sel = tau_stack[row_indices]
    return compute_prediction_error(Y_sel, tau_sel, result, param_names)

def _stratified_validation_rows(row_labels, rows, fraction=0.2):
    row_labels = np.asarray(row_labels, dtype=object)
    if row_labels.size != rows or rows < 14:
        cut = int(rows * (1.0 - fraction))
        if cut <= 0 or cut >= rows:
            return np.array([], dtype=np.int64)
        return np.arange(cut, rows, dtype=np.int64)

    selected = []
    for label in sorted(set(row_labels.tolist()), key=str):
        label_rows = np.flatnonzero(row_labels == label)
        if label_rows.size == 0:
            continue
        count = max(1, int(np.ceil(label_rows.size * fraction)))
        selected.extend(label_rows[-count:].tolist())
    return np.asarray(sorted(set(selected)), dtype=np.int64)

def _validation_rms(Y_stack, tau_stack, result, param_names, row_labels=None):
    rows = Y_stack.shape[0]
    val_rows = _stratified_validation_rows(row_labels, rows) if row_labels is not None else _stratified_validation_rows([], rows)
    if val_rows.size == 0:
        return float("nan")
    Y_val = Y_stack[val_rows, :]
    tau_val = tau_stack[val_rows]
    return compute_prediction_error(Y_val, tau_val, result, param_names)

def _solve_identification_case(
    name,
    backend,
    q_meas,
    qd_meas,
    qdd_traj,
    tau_meas,
    trajectory_labels,
    stride,
    q_ref,
    true_masses,
    true_coms,
    true_inertias,
    inertial_prior_lambda=None,
    mass_prior_lambda=None,
    com_prior_lambda=None,
    inertia_prior_lambda=None,
    joint_prior_lambda=None,
    rcond=None,
):
    Y_stack, param_names = build_stacked_regressor(
        backend,
        q_meas,
        qd_meas,
        qdd_traj,
        stride=stride,
        include_joint_terms=True,
        q_ref=q_ref,
        coulomb_eps=Config.PARAM_ID_COULOMB_EPS,
    )
    tau_stack = tau_meas[::stride, :].ravel()

    labels = np.asarray(trajectory_labels[::stride], dtype=object)
    if labels.size:
        row_labels = np.repeat(labels, 7)
    else:
        row_labels = np.array([], dtype=object)

    prior = make_prior_from_link_params(
        param_names,
        true_masses,
        true_coms,
        true_inertias,
        Config.PARAM_ID_JOINT_PRIORS,
    )
    result = solve_least_squares(
        Y_stack,
        tau_stack,
        param_names,
        prior=prior,
        inertial_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIAL if inertial_prior_lambda is None else inertial_prior_lambda,
        mass_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_MASS if mass_prior_lambda is None else mass_prior_lambda,
        com_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_COM if com_prior_lambda is None else com_prior_lambda,
        inertia_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIA if inertia_prior_lambda is None else inertia_prior_lambda,
        joint_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_JOINT if joint_prior_lambda is None else joint_prior_lambda,
        rcond=Config.PARAM_ID_RCOND if rcond is None else rcond,
        ridge=Config.PARAM_ID_RIDGE,
    )
    masses, coms, inertias = to_link_params(result, prior=prior)
    inertial_Y, _ = build_stacked_regressor(
        backend,
        q_meas,
        qd_meas,
        qdd_traj,
        stride=stride,
        include_joint_terms=False,
    )
    diagnostics = dict(get_last_diagnostics())
    mass_summary = _mass_error_summary(masses, true_masses)
    com_summary = _com_error_summary(coms, true_coms)
    inertia_summary = _inertia_error_summary(inertias, true_inertias)
    joint_term_summary = _joint_term_error_summary(result, q_meas, qd_meas, q_ref)
    dynamic_rows = _segment_indices(row_labels, "dynamic")
    j67_rows = _segment_indices(row_labels, "j6j7")
    j7_rows = _segment_indices(row_labels, "j7")
    gravity_rows = _segment_indices(row_labels, "gravity")
    com_gravity_rows = _segment_indices(row_labels, "com_gravity")
    inertia_rows = _segment_indices(row_labels, "inertia")
    segment_rms = {
        "dynamic": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, dynamic_rows),
        "j6j7": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, j67_rows),
        "j7": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, j7_rows),
        "gravity": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, gravity_rows),
        "com_gravity": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, com_gravity_rows),
        "inertia": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, inertia_rows),
    }
    train_rms = compute_prediction_error(Y_stack, tau_stack, result, param_names)
    validation_rms = _validation_rms(Y_stack, tau_stack, result, param_names, row_labels=row_labels)
    validation_ratio = validation_rms / max(train_rms, 1e-12) if np.isfinite(validation_rms) else float("nan")
    return {
        "name": name,
        "include_joint_terms": True,
        "Y_stack": Y_stack,
        "param_names": param_names,
        "tau_stack": tau_stack,
        "result": result,
        "masses": masses,
        "coms": coms,
        "inertias": inertias,
        "condition": compute_condition_number(Y_stack),
        "prediction_error": train_rms,
        "validation_rms": validation_rms,
        "validation_ratio": validation_ratio,
        "segment_rms": segment_rms,
        "diagnostics": diagnostics,
        "inertial_metrics": _scaled_svd_metrics(inertial_Y),
        "distal": _distal_observability(Y_stack, include_joint_terms=True),
        "inertial_distal": _distal_observability(inertial_Y, include_joint_terms=False),
        "group_observability": _parameter_group_observability(Y_stack),
        "j7_columns": _j7_column_diagnostics(inertial_Y),
        "mass_summary": mass_summary,
        "com_summary": com_summary,
        "inertia_summary": inertia_summary,
        "joint_term_error_summary": joint_term_summary,
        "selection": {
            "inertial_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_INERTIAL if inertial_prior_lambda is None else inertial_prior_lambda,
            "mass_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_MASS if mass_prior_lambda is None else mass_prior_lambda,
            "com_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_COM if com_prior_lambda is None else com_prior_lambda,
            "inertia_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_INERTIA if inertia_prior_lambda is None else inertia_prior_lambda,
            "joint_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_JOINT if joint_prior_lambda is None else joint_prior_lambda,
            "rcond": Config.PARAM_ID_RCOND if rcond is None else rcond,
        },
    }

def _regularization_grid():
    if not Config.PARAM_ID_REG_SWEEP:
        return [(
            Config.PARAM_ID_PRIOR_LAMBDA_MASS,
            Config.PARAM_ID_PRIOR_LAMBDA_COM,
            Config.PARAM_ID_PRIOR_LAMBDA_INERTIA,
            Config.PARAM_ID_PRIOR_LAMBDA_JOINT,
        )]
    return [
        (32.0, 1.20, 2.40, 0.035),
        (64.0, 1.20, 2.40, 0.035),
        (16.0, 1.20, 2.40, 0.035),
        (48.0, 1.20, 2.40, 0.035),
        (32.0, 0.80, 2.40, 0.035),
        (32.0, 1.60, 2.40, 0.035),
        (64.0, 1.60, 3.20, 0.035),
        (32.0, 1.20, 3.20, 0.050),
    ]

def _best_regularized_case(
    name,
    backend,
    q_meas,
    qd_meas,
    qdd_traj,
    tau_meas,
    trajectory_labels,
    stride,
    q_ref,
    true_masses,
    true_coms,
    true_inertias,
):
    best_case = None
    for mass_lambda, com_lambda, inertia_lambda, joint_lambda in _regularization_grid():
        case = _solve_identification_case(
            name,
            backend,
            q_meas,
            qd_meas,
            qdd_traj,
            tau_meas,
            trajectory_labels,
            stride,
            q_ref,
            true_masses,
            true_coms,
            true_inertias,
            mass_prior_lambda=mass_lambda,
            com_prior_lambda=com_lambda,
            inertia_prior_lambda=inertia_lambda,
            joint_prior_lambda=joint_lambda,
        )
        if best_case is None or _case_selection_key(case) < _case_selection_key(best_case):
            best_case = case
    return best_case

