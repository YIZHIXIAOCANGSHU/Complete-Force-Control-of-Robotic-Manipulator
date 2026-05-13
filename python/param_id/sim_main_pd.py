#!/usr/bin/env python3
"""参数辨识 — PD + 前馈闭环仿真模式.

This entry point keeps the trajectory generation, identification solver, Rerun
logging, and HTML report pipeline from ``sim_main.py``.  The sampling path is
different: MuJoCo is stepped with a lightweight PD + nonlinear-effects
feedforward controller, and the measured states plus commanded torque are used
for identification.
"""

from __future__ import annotations

import os
import sys
import time
from typing import Iterable

import numpy as np

PYTHON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(PYTHON_ROOT)
for path in (PYTHON_ROOT, PROJECT_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import Config
from core.pinocchio_backend import PinocchioGravityBackend
from param_id import sim_main as _base
from param_id.excitation import limit_ee_speed
from param_id.regressor import build_stacked_regressor


_DEFAULT_KP = np.array([10.0, 10.0, 5.0, 5.0, 0.2, 0.2, 0.2], dtype=np.float64)
_DEFAULT_KD = np.array([0.316, 0.316, 0.158, 0.158, 0.032, 0.032, 0.032], dtype=np.float64)


def _as_joint_vector(values, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (Config.NUM_JOINTS,):
        raise ValueError(f"{name} must have shape ({Config.NUM_JOINTS},), got {arr.shape}")
    return arr


def _validate_trajectory_pair(q_traj, qd_traj) -> tuple[np.ndarray, np.ndarray]:
    q = np.asarray(q_traj, dtype=np.float64)
    qd = np.asarray(qd_traj, dtype=np.float64)
    expected_rank = 2
    expected_width = Config.NUM_JOINTS
    if q.ndim != expected_rank or q.shape[1] != expected_width:
        raise ValueError(f"q_traj must have shape (n, {expected_width}), got {q.shape}")
    if qd.shape != q.shape:
        raise ValueError(f"qd_traj must match q_traj shape {q.shape}, got {qd.shape}")
    return q, qd


class PDController:
    """Lightweight PD + nonlinear-effects feedforward torque controller."""

    def __init__(
        self,
        backend: PinocchioGravityBackend,
        kp: Iterable[float] | None = None,
        kd: Iterable[float] | None = None,
        torque_limits: Iterable[float] | None = None,
    ) -> None:
        self.backend = backend
        backend_kp = getattr(backend, "_joint_kp", _DEFAULT_KP)
        backend_kd = getattr(backend, "_joint_kd", _DEFAULT_KD)
        backend_limits = getattr(backend, "_torque_limits", Config.TORQUE_LIMITS)
        self.kp = _as_joint_vector(backend_kp if kp is None else kp, "kp")
        self.kd = _as_joint_vector(backend_kd if kd is None else kd, "kd")
        limits = backend_limits if torque_limits is None else torque_limits
        self.torque_limits = _as_joint_vector(limits, "torque_limits")

    def compute_torque(self, q, qd, q_ref, qd_ref) -> np.ndarray:
        q_arr = _as_joint_vector(q, "q")
        qd_arr = _as_joint_vector(qd, "qd")
        q_ref_arr = _as_joint_vector(q_ref, "q_ref")
        qd_ref_arr = _as_joint_vector(qd_ref, "qd_ref")
        tau_ff = _as_joint_vector(
            self.backend.compute_nonlinear_effects(q_arr, qd_arr),
            "tau_ff",
        )
        tau = self.kp * (q_ref_arr - q_arr) + self.kd * (qd_ref_arr - qd_arr) + tau_ff
        return np.clip(tau, -self.torque_limits, self.torque_limits)


def _simulate_pd_step(env, controller: PDController, q_ref, qd_ref) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Run one closed-loop MuJoCo step and return pre-step state plus torque."""
    q = env.get_qpos()
    qd = env.get_qvel()
    tau = controller.compute_torque(q, qd, q_ref, qd_ref)
    env.apply_torque(tau)
    env.step()
    if hasattr(env, "enforce_joint_limits"):
        env.enforce_joint_limits()
    return q, qd, tau


def _collect_pd_data(env, controller: PDController, q_traj, qd_traj) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Run a headless PD simulation and return ``(q_meas, qd_meas, tau_cmd)``."""
    q_ref_traj, qd_ref_traj = _validate_trajectory_pair(q_traj, qd_traj)
    n_steps = len(q_ref_traj)
    q_meas = np.zeros((n_steps, Config.NUM_JOINTS), dtype=np.float64)
    qd_meas = np.zeros_like(q_meas)
    tau_meas = np.zeros_like(q_meas)

    for step in range(n_steps):
        q, qd, tau = _simulate_pd_step(env, controller, q_ref_traj[step], qd_ref_traj[step])
        q_meas[step] = q
        qd_meas[step] = qd
        tau_meas[step] = tau
    return q_meas, qd_meas, tau_meas


def _viewer_sync_stride() -> int:
    configured = getattr(Config, "VIEWER_SYNC_STRIDE", None)
    if configured is not None:
        return max(1, int(configured))
    fps = max(1.0, float(getattr(Config, "REAL_VIEWER_FPS", 60.0)))
    return max(1, int(round(1.0 / max(float(Config.DT) * fps, 1e-9))))


def _set_desired_pose(env, pos, quat) -> None:
    if hasattr(env, "set_target_pose"):
        env.set_target_pose(np.asarray(pos, dtype=np.float64), np.asarray(quat, dtype=np.float64))


def _run_pd_simulation_with_viewer(
    env,
    controller: PDController,
    t_arr,
    q_traj,
    qd_traj,
    qdd_traj=None,
    rerun_ok: bool = False,
    ee_pos_desired_all=None,
    ee_quat_desired_all=None,
    q_ref_friction=None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Run the final closed-loop simulation, keeping viewer/Rerun in sync."""
    del qdd_traj
    t_arr = np.asarray(t_arr, dtype=np.float64)
    q_ref_traj, qd_ref_traj = _validate_trajectory_pair(q_traj, qd_traj)
    if len(t_arr) != len(q_ref_traj):
        raise ValueError(f"t_arr length {len(t_arr)} must match trajectory length {len(q_ref_traj)}")
    if ee_pos_desired_all is None or ee_quat_desired_all is None:
        ee_pos_desired_all, ee_quat_desired_all = _base._compute_ee_poses_for_q_traj(env, q_ref_traj)

    q_meas = np.zeros_like(q_ref_traj)
    qd_meas = np.zeros_like(q_ref_traj)
    tau_meas = np.zeros_like(q_ref_traj)
    sync_stride = _viewer_sync_stride()
    del q_ref_friction

    with _base._viewer_context(env) as viewer:
        start_wall = time.perf_counter()
        for step in range(len(q_ref_traj)):
            if step % 250 == 0:
                sys.stdout.write(f"\r  PD 进度: {step}/{len(q_ref_traj)} ({100 * step // max(len(q_ref_traj), 1)}%)")
                sys.stdout.flush()

            _set_desired_pose(env, ee_pos_desired_all[step], ee_quat_desired_all[step])
            q, qd, tau = _simulate_pd_step(env, controller, q_ref_traj[step], qd_ref_traj[step])
            q_meas[step] = q
            qd_meas[step] = qd
            tau_meas[step] = tau

            if viewer is not None and step % sync_stride == 0:
                viewer.sync()

            if step % Config.RERUN_LOG_STRIDE == 0:
                _base._log_rerun_step(rerun_ok, t_arr[step], q, qd, tau)
                _base._log_sim_realtime_step_from_env(
                    rerun_ok=rerun_ok,
                    env=env,
                    t=t_arr[step],
                    step=step,
                    q_actual=q,
                    qd_actual=qd,
                    q_desired=q_ref_traj[step],
                    tau_received=tau,
                    tau_applied=tau,
                    cycle_time_ms=Config.DT * 1000.0,
                    pos_desired=ee_pos_desired_all[step],
                    quat_desired=ee_quat_desired_all[step],
                )
            _base._sync_realtime(start_wall, t_arr[step])

        if viewer is not None:
            viewer.sync()

    print()
    return q_meas, qd_meas, tau_meas


def _joint_effect_torque_sequence(q_meas, qd_meas, q_ref, priors=None) -> np.ndarray:
    q = np.asarray(q_meas, dtype=np.float64)
    qd = np.asarray(qd_meas, dtype=np.float64)
    q_ref_arr = _as_joint_vector(q_ref, "q_ref")
    if q.shape != qd.shape or q.ndim != 2 or q.shape[1] != Config.NUM_JOINTS:
        raise ValueError(f"q_meas and qd_meas must both have shape (n, {Config.NUM_JOINTS})")
    joint_priors = Config.PARAM_ID_JOINT_PRIORS if priors is None else priors
    tau_joint = np.zeros_like(q)
    for step in range(len(q)):
        tau_joint[step] = _base._joint_effect_torque(q[step], qd[step], joint_priors, q_ref_arr)
    return tau_joint


def _pd_identification_torque(tau_cmd, q_meas, qd_meas, q_ref, priors=None) -> np.ndarray:
    """Remove the configured joint effect model from commanded PD torque."""
    tau = np.asarray(tau_cmd, dtype=np.float64)
    tau_joint = _joint_effect_torque_sequence(q_meas, qd_meas, q_ref, priors=priors)
    if tau.shape != tau_joint.shape:
        raise ValueError(f"tau_cmd must have shape {tau_joint.shape}, got {tau.shape}")
    return tau - tau_joint


def _pd_tracking_summary(q_meas, q_ref) -> dict[str, float]:
    q = np.asarray(q_meas, dtype=np.float64)
    ref = np.asarray(q_ref, dtype=np.float64)
    count = min(len(q), len(ref))
    if count == 0:
        return {"joint_rms_rad": 0.0, "joint_max_abs_rad": 0.0}
    err = q[:count] - ref[:count]
    return {
        "joint_rms_rad": float(np.sqrt(np.mean(err**2))),
        "joint_max_abs_rad": float(np.max(np.abs(err))),
    }


def _estimate_qdd_from_qd(qd_meas, dt: float | None = None) -> np.ndarray:
    qd = np.asarray(qd_meas, dtype=np.float64)
    if qd.ndim != 2 or qd.shape[1] != Config.NUM_JOINTS:
        raise ValueError(f"qd_meas must have shape (n, {Config.NUM_JOINTS}), got {qd.shape}")
    if len(qd) == 0:
        return np.zeros_like(qd)
    if len(qd) == 1:
        return np.zeros_like(qd)
    step = float(Config.DT if dt is None else dt)
    edge_order = 2 if len(qd) >= 3 else 1
    return np.gradient(qd, step, axis=0, edge_order=edge_order)


def _pd_validation_grid() -> list[tuple[float, float, float, float]]:
    grid = list(_base._regularization_grid())
    if not grid:
        return []
    return grid[: min(2, len(grid))]


def _with_joint_prior_terms(result: dict[str, float]) -> dict[str, float]:
    combined = dict(result)
    for joint, prior in enumerate(Config.PARAM_ID_JOINT_PRIORS, start=1):
        for term in ("fc", "k", "fv", "fo"):
            combined[f"J{joint}_{term}"] = float(prior[term])
    return combined


def _solve_pd_inertial_case(
    name,
    backend,
    q_meas,
    qd_meas,
    qdd_traj,
    tau_inertial,
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
    del q_ref, joint_prior_lambda
    Y_stack, param_names = build_stacked_regressor(
        backend,
        q_meas,
        qd_meas,
        qdd_traj,
        stride=stride,
        include_joint_terms=False,
        coulomb_eps=Config.PARAM_ID_COULOMB_EPS,
    )
    tau_stack = np.asarray(tau_inertial, dtype=np.float64)[::stride, :].ravel()
    labels = np.asarray(trajectory_labels[::stride], dtype=object)
    row_labels = np.repeat(labels, Config.NUM_JOINTS) if labels.size else np.array([], dtype=object)

    prior = _base.make_prior_from_link_params(param_names, true_masses, true_coms, true_inertias, None)
    inertial_result = _base.solve_least_squares(
        Y_stack,
        tau_stack,
        param_names,
        prior=prior,
        inertial_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIAL
        if inertial_prior_lambda is None
        else inertial_prior_lambda,
        mass_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_MASS if mass_prior_lambda is None else mass_prior_lambda,
        com_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_COM if com_prior_lambda is None else com_prior_lambda,
        inertia_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIA
        if inertia_prior_lambda is None
        else inertia_prior_lambda,
        joint_prior_lambda=0.0,
        rcond=Config.PARAM_ID_RCOND if rcond is None else rcond,
        ridge=Config.PARAM_ID_RIDGE,
    )
    masses, coms, inertias = _base.to_link_params(inertial_result, prior=prior)
    diagnostics = dict(_base.get_last_diagnostics())
    mass_summary = _base._mass_error_summary(masses, true_masses)
    com_summary = _base._com_error_summary(coms, true_coms)
    inertia_summary = _base._inertia_error_summary(inertias, true_inertias)
    train_rms = _base.compute_prediction_error(Y_stack, tau_stack, inertial_result, param_names)
    validation_rms = _base._validation_rms(Y_stack, tau_stack, inertial_result, param_names, row_labels=row_labels)
    validation_ratio = validation_rms / max(train_rms, 1e-12) if np.isfinite(validation_rms) else float("nan")
    segment_rms = {
        "dynamic": _base._segment_prediction_rms(
            Y_stack, tau_stack, inertial_result, param_names, _base._segment_indices(row_labels, "dynamic")
        ),
        "j6j7": _base._segment_prediction_rms(
            Y_stack, tau_stack, inertial_result, param_names, _base._segment_indices(row_labels, "j6j7")
        ),
        "j7": _base._segment_prediction_rms(
            Y_stack, tau_stack, inertial_result, param_names, _base._segment_indices(row_labels, "j7")
        ),
        "gravity": _base._segment_prediction_rms(
            Y_stack, tau_stack, inertial_result, param_names, _base._segment_indices(row_labels, "gravity")
        ),
        "com_gravity": _base._segment_prediction_rms(
            Y_stack, tau_stack, inertial_result, param_names, _base._segment_indices(row_labels, "com_gravity")
        ),
        "inertia": _base._segment_prediction_rms(
            Y_stack, tau_stack, inertial_result, param_names, _base._segment_indices(row_labels, "inertia")
        ),
    }
    selection = {
        "inertial_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_INERTIAL
        if inertial_prior_lambda is None
        else inertial_prior_lambda,
        "mass_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_MASS if mass_prior_lambda is None else mass_prior_lambda,
        "com_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_COM if com_prior_lambda is None else com_prior_lambda,
        "inertia_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_INERTIA
        if inertia_prior_lambda is None
        else inertia_prior_lambda,
        "joint_prior_lambda": 0.0,
        "rcond": Config.PARAM_ID_RCOND if rcond is None else rcond,
    }
    return {
        "name": name,
        "include_joint_terms": False,
        "Y_stack": Y_stack,
        "param_names": param_names,
        "tau_stack": tau_stack,
        "result": _with_joint_prior_terms(inertial_result),
        "masses": masses,
        "coms": coms,
        "inertias": inertias,
        "condition": _base.compute_condition_number(Y_stack),
        "prediction_error": train_rms,
        "validation_rms": validation_rms,
        "validation_ratio": validation_ratio,
        "segment_rms": segment_rms,
        "diagnostics": diagnostics,
        "inertial_metrics": _base._scaled_svd_metrics(Y_stack),
        "distal": _base._distal_observability(Y_stack, include_joint_terms=False),
        "inertial_distal": _base._distal_observability(Y_stack, include_joint_terms=False),
        "group_observability": _base._parameter_group_observability(Y_stack),
        "j7_columns": _base._j7_column_diagnostics(Y_stack),
        "mass_summary": mass_summary,
        "com_summary": com_summary,
        "inertia_summary": inertia_summary,
        "selection": selection,
    }


def _best_pd_inertial_case(
    name,
    backend,
    q_meas,
    qd_meas,
    qdd_traj,
    tau_inertial,
    trajectory_labels,
    stride,
    q_ref,
    true_masses,
    true_coms,
    true_inertias,
):
    best_case = None
    for mass_lambda, com_lambda, inertia_lambda, joint_lambda in _base._regularization_grid():
        case = _solve_pd_inertial_case(
            name,
            backend,
            q_meas,
            qd_meas,
            qdd_traj,
            tau_inertial,
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
        if best_case is None or _base._case_selection_key(case) < _base._case_selection_key(best_case):
            best_case = case
    return best_case


def _candidate_return_tuple(candidate: dict, metadata_extra: dict | None = None):
    metadata = {
        "profile": candidate["profile"],
        "description": candidate["description"],
        "seed": candidate["seed"],
        "score": candidate["score"],
        "selection_mode": "pd_closed_loop",
    }
    if metadata_extra:
        metadata.update(metadata_extra)
    return (
        candidate["t"],
        candidate["q"],
        candidate["qd"],
        candidate["qdd"],
        candidate["max_ee_speed"],
        candidate["speed_scale"],
        candidate["overall"],
        candidate["distal"],
        candidate["labels"],
        metadata,
    )


def _select_excitation_trajectory_pd(
    env,
    backend: PinocchioGravityBackend,
    controller: PDController,
    q0,
    limits,
) -> tuple[np.ndarray, ...]:
    """Select an excitation trajectory with SVD pre-screening and PD validation."""
    true_masses, true_coms, true_inertias = _base._extract_ground_truth(backend)
    candidates = []
    for profile in _base._trajectory_profiles():
        for seed in _base._trajectory_seeds()[: Config.PARAM_ID_TRAJECTORY_CANDIDATES]:
            t_arr, q_traj, qd_traj, qdd_traj, labels = _base._build_planned_trajectory(profile, seed, q0, limits)
            q_limited, qd_limited, qdd_limited, max_ee_speed, speed_scale = limit_ee_speed(
                env,
                q_traj,
                qd_traj,
                qdd_traj,
                Config.PARAM_ID_MAX_EE_SPEED,
            )
            stride = max(1, len(t_arr) // Config.PARAM_ID_MAX_SAMPLES)
            Y_probe, _ = build_stacked_regressor(
                backend,
                q_limited,
                qd_limited,
                qdd_limited,
                stride=stride,
                include_joint_terms=True,
                q_ref=Config.HOME_QPOS,
                coulomb_eps=Config.PARAM_ID_COULOMB_EPS,
            )
            inertial_Y_probe, _ = build_stacked_regressor(
                backend,
                q_limited,
                qd_limited,
                qdd_limited,
                stride=stride,
                include_joint_terms=False,
                q_ref=Config.HOME_QPOS,
                coulomb_eps=Config.PARAM_ID_COULOMB_EPS,
            )
            overall = _base._scaled_svd_metrics(Y_probe)
            distal = _base._distal_observability(Y_probe, include_joint_terms=True)
            inertial_overall = _base._scaled_svd_metrics(inertial_Y_probe)
            inertial_distal = _base._distal_observability(inertial_Y_probe, include_joint_terms=False)
            group_observability = _base._parameter_group_observability(Y_probe)
            coverage = _base._joint_coverage(q_limited)
            score = _base._candidate_score(
                overall,
                distal,
                inertial_overall,
                inertial_distal,
                group_observability,
                coverage,
                speed_scale,
            )
            candidates.append(
                {
                    "score": score,
                    "profile": profile["name"],
                    "description": profile["description"],
                    "seed": seed,
                    "t": t_arr,
                    "q": q_limited,
                    "qd": qd_limited,
                    "qdd": qdd_limited,
                    "labels": labels,
                    "max_ee_speed": max_ee_speed,
                    "speed_scale": speed_scale,
                    "overall": overall,
                    "distal": distal,
                    "inertial_overall": inertial_overall,
                    "inertial_distal": inertial_distal,
                    "group_observability": group_observability,
                    "coverage": coverage,
                }
            )

    ranked = sorted(candidates, key=lambda item: item["score"], reverse=True)
    if not ranked:
        raise RuntimeError("No excitation trajectory candidates were generated")

    top_n = min(Config.PARAM_ID_VALIDATION_TOP_N, len(ranked))
    validation_grid = _pd_validation_grid()
    validation_cases = []
    validation_errors = []
    print(f"[辨识-PD] SVD 初筛完成，使用 PD 闭环验证 top-{top_n} 候选...")
    for cand in ranked[:top_n]:
        try:
            env.reset(cand["q"][0])
            env.forward()
            q_meas, qd_meas, tau_cmd = _collect_pd_data(env, controller, cand["q"], cand["qd"])
            tau_id = _pd_identification_torque(tau_cmd, q_meas, qd_meas, Config.HOME_QPOS)
            qdd_meas = _estimate_qdd_from_qd(qd_meas)
            tracking = _pd_tracking_summary(q_meas, cand["q"])
            for mass_lambda, com_lambda, inertia_lambda, joint_lambda in validation_grid:
                case = _solve_pd_inertial_case(
                    (
                        f"PD候选验证 {cand['profile']} seed={cand['seed']} "
                        f"λm={mass_lambda:.2g} λc={com_lambda:.2g} "
                        f"λI={inertia_lambda:.2g} λj={joint_lambda:.2g}"
                    ),
                    backend,
                    q_meas,
                    qd_meas,
                    qdd_meas,
                    tau_id,
                    cand["labels"],
                    max(1, len(cand["t"]) // Config.PARAM_ID_MAX_SAMPLES),
                    Config.HOME_QPOS,
                    true_masses,
                    true_coms,
                    true_inertias,
                    mass_prior_lambda=mass_lambda,
                    com_prior_lambda=com_lambda,
                    inertia_prior_lambda=inertia_lambda,
                    joint_prior_lambda=joint_lambda,
                )
                case["candidate"] = cand
                case["pd_tracking"] = tracking
                validation_cases.append(case)
        except Exception as exc:
            validation_errors.append(f"{cand['profile']} seed={cand['seed']}: {exc}")
            print(f"[辨识-PD] 候选 {cand['profile']} seed={cand['seed']} 验证失败，跳过: {exc}")

    if validation_cases:
        validated = sorted(validation_cases, key=_base._case_selection_key)
        best_case = validated[0]
        best_candidate = best_case["candidate"]
        if Config.PARAM_ID_TRAJECTORY_PROFILE_DIAGNOSTICS:
            print("[辨识-PD] PD 验证矩阵Top:")
            for case in validated[:top_n]:
                cand = case["candidate"]
                summary = case["mass_summary"]
                tracking = case.get("pd_tracking", {})
                print(
                    f"  {cand['profile']:<2} seed={cand['seed']:<4} "
                    f"max={summary['max_abs']:.2f}%@J{summary['max_abs_joint']} "
                    f"J7={summary['j7_abs']:.2f}% "
                    f"trackRMS={tracking.get('joint_rms_rad', float('nan')):.4f}rad "
                    f"valRMS={case['validation_rms']:.4f}"
                )
        best_tracking = best_case.get("pd_tracking", {})
        print(
            f"[辨识-PD] 选择激励 {best_candidate['profile']} ({best_candidate['description']}) "
            f"seed={best_candidate['seed']}, 验证最大误差={best_case['mass_summary']['max_abs']:.2f}%, "
            f"跟踪RMS={best_tracking.get('joint_rms_rad', float('nan')):.4f} rad"
        )
        return _candidate_return_tuple(
            best_candidate,
            {
                "pd_validation_rms": best_case["validation_rms"],
                "pd_tracking_rms_rad": best_tracking.get("joint_rms_rad"),
                "pd_tracking_max_abs_rad": best_tracking.get("joint_max_abs_rad"),
            },
        )

    if validation_errors:
        raise RuntimeError("PD validation failed for all top candidates: " + "; ".join(validation_errors))

    fallback = ranked[0]
    print(
        f"[辨识-PD] PD 验证没有可用结果，回退到 SVD 最优激励 {fallback['profile']} "
        f"seed={fallback['seed']}."
    )
    return _candidate_return_tuple(fallback, {"pd_validation_rms": None})


def _log_final_rerun_result(rerun_ok: bool, identified_case: dict) -> None:
    if not rerun_ok:
        return
    import rerun as rr

    for j in range(7):
        rr.log(f"param_id/result/mass/J{j + 1}", rr.Scalars(float(identified_case["masses"][j])))
        rr.log(f"param_id/result/com_x/J{j + 1}", rr.Scalars(float(identified_case["coms"][j][0])))
        rr.log(f"param_id/result/com_y/J{j + 1}", rr.Scalars(float(identified_case["coms"][j][1])))
        rr.log(f"param_id/result/com_z/J{j + 1}", rr.Scalars(float(identified_case["coms"][j][2])))
        rr.log(f"param_id/result/Ixx/J{j + 1}", rr.Scalars(float(identified_case["inertias"][j][0])))
        rr.log(f"param_id/result/Iyy/J{j + 1}", rr.Scalars(float(identified_case["inertias"][j][1])))
        rr.log(f"param_id/result/Izz/J{j + 1}", rr.Scalars(float(identified_case["inertias"][j][2])))


def main() -> None:
    rerun_ok = _base._setup_rerun()
    backend = PinocchioGravityBackend(
        urdf_path=Config.URDF_PATH,
        ee_frame_name="ArmLseventh_Link",
        tcp_offset=Config.TCP_OFFSET,
        torque_limits=Config.TORQUE_LIMITS.tolist(),
    )
    try:
        true_masses, true_coms, true_inertias = _base._extract_ground_truth(backend)

        from sim.env import MujocoSimEnv

        env = MujocoSimEnv()
        env.reset(Config.HOME_QPOS)
        env.forward()
        controller = PDController(backend)

        q0 = Config.HOME_QPOS.copy()
        limits = (
            np.array([np.deg2rad(d) for d in [-80, -80, -60, -110, -40, -50, -50]]),
            np.array([np.deg2rad(d) for d in [80, 20, 40, 110, 40, 40, 50]]),
        )
        print("[辨识-PD] 生成 Fourier 激励轨迹并进行闭环验证...")
        (
            t_arr,
            q_traj,
            qd_traj,
            qdd_traj,
            max_ee_speed,
            speed_scale,
            excitation_overall,
            excitation_distal,
            trajectory_labels,
            trajectory_metadata,
        ) = _select_excitation_trajectory_pd(env, backend, controller, q0, limits)

        n_steps = len(t_arr)
        print(f"[辨识-PD] 轨迹: {n_steps} 步 @ {Config.DT * 1000:.0f}ms, 共 {t_arr[-1]:.1f}s")
        print(f"[辨识-PD] TCP 最大速度: {max_ee_speed:.3f} m/s (缩放系数 {speed_scale:.3f})")

        ee_pos_desired_all, ee_quat_desired_all = _base._compute_ee_poses_for_q_traj(env, q_traj)
        env.reset(q_traj[0])
        env.forward()
        print("[辨识-PD] 启动 MuJoCo 窗口，执行 PD + 前馈闭环仿真...")
        t0 = time.perf_counter()
        q_meas, qd_meas, tau_cmd = _run_pd_simulation_with_viewer(
            env,
            controller,
            t_arr,
            q_traj,
            qd_traj,
            qdd_traj,
            rerun_ok=rerun_ok,
            ee_pos_desired_all=ee_pos_desired_all,
            ee_quat_desired_all=ee_quat_desired_all,
            q_ref_friction=Config.HOME_QPOS,
        )
        elapsed = time.perf_counter() - t0
        print(f"[辨识-PD] 闭环轨迹执行完毕，耗时 {elapsed:.1f}s")

        tau_id = _pd_identification_torque(tau_cmd, q_meas, qd_meas, Config.HOME_QPOS)
        qdd_meas = _estimate_qdd_from_qd(qd_meas)
        tracking = _pd_tracking_summary(q_meas, q_traj)
        print(
            "[辨识-PD] 已扣除关节摩擦/弹性先验项用于动力学辨识，"
            f"关节跟踪 RMS={tracking['joint_rms_rad']:.4f} rad, "
            f"max={tracking['joint_max_abs_rad']:.4f} rad"
        )

        print("[辨识-PD] 构建力矩回归器，执行正则化辨识...")
        stride = max(1, n_steps // Config.PARAM_ID_MAX_SAMPLES)
        identified_case = _best_pd_inertial_case(
            "PD闭环惯性辨识结果（指令力矩扣除关节项）",
            backend,
            q_meas,
            qd_meas,
            qdd_meas,
            tau_id,
            trajectory_labels,
            stride,
            Config.HOME_QPOS,
            true_masses,
            true_coms,
            true_inertias,
        )

        _base._print_chinese_header()
        _base._print_identification_case(identified_case, true_masses, true_inertias)
        report_metadata = {
            **trajectory_metadata,
            "stride": stride,
            "rerun_log_stride": Config.RERUN_LOG_STRIDE,
            "max_ee_speed": max_ee_speed,
            "speed_scale": speed_scale,
            "pd_tracking_rms_rad": tracking["joint_rms_rad"],
            "pd_tracking_max_abs_rad": tracking["joint_max_abs_rad"],
            "excitation_rank": excitation_overall.get("rank"),
            "excitation_distal_rank": excitation_distal.get("rank"),
            "qdd_source": "gradient(qd_meas)",
            "torque_target": "tau_cmd_minus_joint_effect_prior",
        }
        trajectory_records = _base._build_trajectory_records_from_env(
            env,
            t_arr,
            q_meas,
            q_traj,
            cycle_time_ms=Config.DT * 1000.0,
        )
        report_path = _base._write_html_report(
            identified_case,
            true_masses,
            true_coms,
            true_inertias,
            t_arr,
            q_meas,
            qd_meas,
            tau_cmd,
            rerun_ok,
            report_metadata,
            trajectory_records=trajectory_records,
        )
        if report_path:
            print(f"HTML 报告已保存: {report_path}")
        print("\nPD 闭环辨识参数已计算，可用于后续导出/验证。")
        print("=" * 78)
        _log_final_rerun_result(rerun_ok, identified_case)
    finally:
        backend.close()
    print("\n[辨识-PD] 完成。")


if __name__ == "__main__":
    main()
