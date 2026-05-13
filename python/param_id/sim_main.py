#!/usr/bin/env python3
"""参数辨识 — 仿真模式 (MuJoCo + Pinocchio + Rerun)

在 MuJoCo 中执行 Fourier 激励轨迹，同时弹出 MuJoCo 窗口实时显示机械臂运动，
通过 Rerun 记录轨迹/力矩/辨识结果，终端打印中文辨识报告。
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np

PYTHON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(PYTHON_ROOT)
for path in (PYTHON_ROOT, PROJECT_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import Config
from core.pinocchio_backend import PinocchioGravityBackend
from common.mujoco_viewer import launch_passive_viewer
from common import rerun_viz
from param_id.excitation import fourier_trajectory, limit_ee_speed
from param_id.regressor import build_stacked_regressor
from param_id.identification import (
    solve_least_squares,
    to_link_params,
    compute_condition_number,
    compute_prediction_error,
    get_last_diagnostics,
    make_prior_from_link_params,
)

import mujoco


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


def _print_chinese_header():
    print()
    print("=" * 78)
    print("                    参数辨识结果（仿真模式）")
    print("=" * 78)


def _print_identified_params(masses, coms, inertias):
    print(f"\n{'关节':<6} {'质量(kg)':>10}  {'质心 COM (m)':<36} {'惯量对角 (kg·m²)':<42}")
    print("-" * 78)
    for j in range(7):
        print(
            f" J{j + 1:<5} {masses[j]:>10.4f}  "
            f"[{coms[j][0]: .4f} {coms[j][1]: .4f} {coms[j][2]: .4f}]"
            f"{'':>6}"
            f"[{inertias[j][0]:.6f}  {inertias[j][1]:.6f}  {inertias[j][2]:.6f}]"
        )


def _print_comparison(masses, true_masses, inertias, true_inertias):
    print(f"\n===== 与 URDF 真值对比 =====")
    print(f"{'关节':<6} {'辨识质量':>10} {'真值质量':>10} {'误差%':>8}  "
          f"{'辨识Ixx':>12} {'真值Ixx':>12}  {'辨识Iyy':>12} {'真值Iyy':>12}")
    print("-" * 90)
    distal_abs_err = []
    for j in range(7):
        tm = true_masses[j]
        err = (masses[j] - tm) / tm * 100 if tm > 1e-9 else 0.0
        if j >= Config.PARAM_ID_DISTAL_LINK_START - 1:
            distal_abs_err.append(abs(err))
        print(
            f" J{j + 1:<5} {masses[j]:>10.4f} {tm:>10.4f} {err:>7.1f}%  "
            f"{inertias[j][0]:>12.6f} {true_inertias[j][0]:>12.6f}  "
            f"{inertias[j][1]:>12.6f} {true_inertias[j][1]:>12.6f}"
        )
    if distal_abs_err:
        print(f"末端质量平均绝对误差: {np.mean(distal_abs_err):.1f}%")


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


def _print_joint_term_comparison(result):
    print(f"\n===== 关节摩擦/弹性/偏置辨识对比 =====")
    print(f"{'关节':<6} {'fc辨识':>9} {'fc先验':>9} {'k辨识':>9} {'k先验':>9} {'fv辨识':>9} {'fv先验':>9} {'fo辨识':>9} {'fo先验':>9}")
    print("-" * 96)
    for j, prior in enumerate(Config.PARAM_ID_JOINT_PRIORS):
        print(
            f" J{j + 1:<5} "
            f"{result.get(f'J{j + 1}_fc', 0.0):>9.3f} {prior['fc']:>9.3f} "
            f"{result.get(f'J{j + 1}_k', 0.0):>9.3f} {prior['k']:>9.3f} "
            f"{result.get(f'J{j + 1}_fv', 0.0):>9.3f} {prior['fv']:>9.3f} "
            f"{result.get(f'J{j + 1}_fo', 0.0):>9.3f} {prior['fo']:>9.3f}"
        )


def _sync_realtime(start_wall, t_target):
    if not Config.PARAM_ID_REALTIME or not Config.ENABLE_VIEWER:
        return
    delay = start_wall + float(t_target) - time.perf_counter()
    if delay > 0.0:
        time.sleep(delay)


def _viewer_context(env):
    if not Config.ENABLE_VIEWER:
        from contextlib import nullcontext
        return nullcontext(None)
    try:
        return launch_passive_viewer(env.model, env.data)
    except Exception as exc:
        print(f"[辨识] MuJoCo 窗口不可用，改为无窗口运行: {exc}")
        from contextlib import nullcontext
        return nullcontext(None)


def _setup_rerun() -> bool:
    if not Config.ENABLE_RERUN:
        return False
    try:
        import rerun as rr
        import rerun.blueprint as rrb

        rr.init("AM-D02 参数辨识 (Sim)", spawn=True)
        for name in ["excitation_q_rad", "excitation_qd_rad_s", "tau_nm"]:
            rr.log(f"param_id/{name}", rr.SeriesLines(
                colors=[[230, 100, 50], [80, 200, 220]],
                names=["J1", "J2"], widths=[2, 2],
            ), static=True)
        blueprint = rrb.Blueprint(
            rrb.Vertical(
                rrb.TimeSeriesView(name="关节位置 q", origin="/param_id/excitation_q_rad"),
                rrb.TimeSeriesView(name="关节速度 qd", origin="/param_id/excitation_qd_rad_s"),
                rrb.TimeSeriesView(name="力矩对比", origin="/param_id/tau_nm"),
            ),
            collapse_panels=True,
        )
        rr.send_blueprint(blueprint)
        return True
    except Exception:
        return False


def _log_rerun_step(rerun_ok: bool, t: float, q, qd, tau):
    if not rerun_ok:
        return
    import rerun as rr

    rr.set_time_seconds("time", t)
    for i in range(7):
        rr.log("param_id/excitation_q_rad/J%d" % (i + 1), rr.Scalars(float(q[i])))
        rr.log("param_id/excitation_qd_rad_s/J%d" % (i + 1), rr.Scalars(float(qd[i])))
        rr.log("param_id/tau_nm/J%d" % (i + 1), rr.Scalars(float(tau[i])))


def _trajectory_seeds() -> list[int]:
    seeds = []
    for item in str(Config.PARAM_ID_TRAJECTORY_SEEDS).split(","):
        item = item.strip()
        if item:
            seeds.append(int(item))
    if not seeds:
        seeds.append(42)
    while len(seeds) < Config.PARAM_ID_TRAJECTORY_CANDIDATES:
        seeds.append(seeds[-1] + 1)
    return seeds[:Config.PARAM_ID_TRAJECTORY_CANDIDATES]


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


def _distal_excitation_weights(amp_scale=1.45, freq_scale=1.25, proximal_scale=1.0):
    amp = np.full(7, float(proximal_scale), dtype=np.float64)
    freq = np.ones(7, dtype=np.float64)
    start = Config.PARAM_ID_DISTAL_LINK_START - 1
    amp[start:] = float(amp_scale)
    freq[start:] = float(freq_scale)
    return amp, freq


def _phase_offsets(phase_span):
    phases = np.zeros(7, dtype=np.float64)
    start = Config.PARAM_ID_DISTAL_LINK_START - 1
    if start < 7:
        phases[start:] = np.linspace(0.0, float(phase_span), 7 - start)
    return phases


def _trajectory_profiles():
    profiles = [
        {"name": "balanced", "amp": 1.45, "freq": 1.25, "prox": 1.0, "phase": 0.0},
        {"name": "distal-wide", "amp": 1.75, "freq": 1.15, "prox": 0.9, "phase": np.pi / 3.0},
        {"name": "distal-fast", "amp": 1.35, "freq": 1.55, "prox": 0.85, "phase": 2.0 * np.pi / 3.0},
        {"name": "decorrelated", "amp": 1.6, "freq": 1.4, "prox": 0.95, "phase": np.pi},
    ]
    return profiles[:Config.PARAM_ID_TRAJECTORY_PROFILES]


def _joint_coverage(q):
    ptp = np.ptp(q, axis=0)
    distal_start = Config.PARAM_ID_DISTAL_LINK_START - 1
    distal = ptp[distal_start:]
    return {
        "min": float(np.min(distal)) if distal.size else 0.0,
        "mean": float(np.mean(distal)) if distal.size else 0.0,
    }


def _candidate_score(overall, distal, inertial_overall, inertial_distal, coverage, speed_scale):
    speed_penalty = max(0.0, 1.0 - float(speed_scale))
    condition_penalty = np.log10(max(inertial_overall["condition"], 1.0))
    inertial_projection = inertial_distal["projection"]
    joint_projection = distal["projection"]
    return (
        overall["rank"] * 90.0
        + inertial_overall["rank"] * 5.0
        + inertial_distal["rank"] * 30.0
        + inertial_projection["rank"] * 20.0
        + inertial_projection["ratio"] * 220.0
        + joint_projection["ratio"] * 60.0
        + np.log10(max(inertial_projection["sigma_min"], 1e-15) / 1e-15) * 2.0
        + np.log10(max(inertial_distal["sigma_min"], 1e-15) / 1e-15)
        + min(coverage["mean"], 1.0) * 4.0
        + min(coverage["min"], 0.5) * 6.0
        - inertial_distal["correlation"] * 12.0
        - distal["correlation"] * 4.0
        - np.log10(max(inertial_distal["condition"], 1.0)) * 2.0
        - condition_penalty * 0.2
        - speed_penalty * 12.0
    )


def _select_excitation_trajectory(backend, env, q0, limits):
    best = None
    candidates = []
    for profile in _trajectory_profiles():
        amp_weights, freq_weights = _distal_excitation_weights(profile["amp"], profile["freq"], profile["prox"])
        phases = _phase_offsets(profile["phase"])
        for seed in _trajectory_seeds():
            t_arr, q_traj, qd_traj, qdd_traj = fourier_trajectory(
                q0=q0,
                n_harmonics=5,
                base_freq=0.2,
                duration=8.0,
                dt=Config.DT,
                joint_limits=limits,
                random_seed=seed,
                joint_amplitude_weights=amp_weights,
                joint_frequency_weights=freq_weights,
                phase_offsets=phases,
            )
            q_limited, qd_limited, qdd_limited, max_ee_speed, speed_scale = limit_ee_speed(
                env, q_traj, qd_traj, qdd_traj, Config.PARAM_ID_MAX_EE_SPEED,
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
            overall = _scaled_svd_metrics(Y_probe)
            distal = _distal_observability(Y_probe, include_joint_terms=True)
            inertial_overall = _scaled_svd_metrics(inertial_Y_probe)
            inertial_distal = _distal_observability(inertial_Y_probe, include_joint_terms=False)
            coverage = _joint_coverage(q_limited)
            score = _candidate_score(overall, distal, inertial_overall, inertial_distal, coverage, speed_scale)
            candidate = {
                "score": score,
                "profile": profile["name"],
                "seed": seed,
                "t": t_arr,
                "q": q_limited,
                "qd": qd_limited,
                "qdd": qdd_limited,
                "max_ee_speed": max_ee_speed,
                "speed_scale": speed_scale,
                "overall": overall,
                "distal": distal,
                "inertial_overall": inertial_overall,
                "inertial_distal": inertial_distal,
                "coverage": coverage,
            }
            candidates.append(candidate)
            if best is None or score > best["score"]:
                best = candidate

    if Config.PARAM_ID_TRAJECTORY_PROFILE_DIAGNOSTICS:
        print("[辨识] 激励候选Top:")
        for cand in sorted(candidates, key=lambda item: item["score"], reverse=True)[:min(5, len(candidates))]:
            iproj = cand["inertial_distal"]["projection"]
            jproj = cand["distal"]["projection"]
            print(
                f"  profile={cand['profile']:<12} seed={cand['seed']:<4} score={cand['score']:.2f} "
                f"惯性rank={cand['inertial_overall']['rank']} 惯性末端cond={cand['inertial_distal']['condition']:.1f} "
                f"残差={iproj['ratio']:.3f}/{iproj['rank']} 联合残差={jproj['ratio']:.3f}/{jproj['rank']} "
                f"相关={cand['inertial_distal']['correlation']:.3f} TCP={cand['max_ee_speed']:.3f} "
                f"缩放={cand['speed_scale']:.3f} 覆盖={cand['coverage']['mean']:.3f}"
            )

    print(
        f"[辨识] 选择激励 profile={best['profile']} seed={best['seed']}, "
        f"惯性回归条件数={best['inertial_overall']['condition']:.1f}, "
        f"rank={best['inertial_overall']['rank']}, 末端rank={best['inertial_distal']['rank']}, "
        f"末端条件数={best['inertial_distal']['condition']:.1f}, "
        f"残差={best['inertial_distal']['projection']['ratio']:.3f}, "
        f"末端相关={best['inertial_distal']['correlation']:.3f}"
    )
    return (
        best["t"], best["q"], best["qd"], best["qdd"], best["max_ee_speed"],
        best["speed_scale"], best["overall"], best["distal"],
    )


def main() -> None:
    # ---- 初始化 ----
    backend = PinocchioGravityBackend(
        urdf_path=Config.URDF_PATH,
        ee_frame_name="ArmLseventh_Link",
        tcp_offset=Config.TCP_OFFSET,
        torque_limits=Config.TORQUE_LIMITS.tolist(),
    )
    true_masses, true_coms, true_inertias = _extract_ground_truth(backend)

    from sim.env import MujocoSimEnv

    env = MujocoSimEnv()
    env.reset(Config.HOME_QPOS)
    env.forward()

    # ---- 激励轨迹 ----
    q0 = Config.HOME_QPOS.copy()
    limits = (
        np.array([np.deg2rad(d) for d in [-80, -80, -60, -110, -40, -50, -50]]),
        np.array([np.deg2rad(d) for d in [80, 20, 40, 110, 40, 40, 50]]),
    )
    print("[辨识] 生成 Fourier 激励轨迹...")
    t_arr, q_traj, qd_traj, qdd_traj, max_ee_speed, speed_scale, excitation_overall, excitation_distal = _select_excitation_trajectory(
        backend, env, q0, limits,
    )
    env.reset(Config.HOME_QPOS)
    env.forward()
    n_steps = len(t_arr)
    print(f"[辨识] 轨迹: {n_steps} 步 @ {Config.DT*1000:.0f}ms, 共 {t_arr[-1]:.1f}s")
    print(f"[辨识] TCP 最大速度: {max_ee_speed:.3f} m/s (缩放系数 {speed_scale:.3f})")

    # ---- Rerun ----
    rerun_ok = _setup_rerun()

    # ---- MuJoCo 窗口 + 轨迹执行 ----
    print("[辨识] 启动 MuJoCo 窗口，执行激励轨迹...")
    tau_meas = np.zeros((n_steps, 7))
    tau_joint = np.zeros((n_steps, 7))
    q_meas = np.zeros((n_steps, 7))
    qd_meas = np.zeros((n_steps, 7))
    q_ref = Config.HOME_QPOS.copy()
    prior_joint = Config.PARAM_ID_JOINT_PRIORS

    with _viewer_context(env) as viewer:
        t0 = time.perf_counter()
        for step in range(n_steps):
            if step % 250 == 0:
                sys.stdout.write(f"\r  进度: {step}/{n_steps} ({100*step//n_steps}%)")
                sys.stdout.flush()

            data = env.data
            data.qpos[:7] = q_traj[step]
            data.qvel[:7] = qd_traj[step]
            data.qacc[:7] = qdd_traj[step]

            mujoco.mj_inverse(env.model, data)
            q_meas[step] = data.qpos[:7].copy()
            qd_meas[step] = data.qvel[:7].copy()
            tau_joint[step] = _joint_effect_torque(q_meas[step], qd_meas[step], prior_joint, q_ref)
            tau_meas[step] = data.qfrc_inverse[:7].copy() + tau_joint[step]

            if viewer is not None and step % 5 == 0:
                viewer.sync()
            _sync_realtime(t0, t_arr[step])

        elapsed = time.perf_counter() - t0
    print(f"\n[辨识] 轨迹执行完毕，耗时 {elapsed:.1f}s")

    # ---- 回归器 + 辨识 ----
    print("[辨识] 构建力矩回归器...")
    stride = max(1, n_steps // Config.PARAM_ID_MAX_SAMPLES)
    include_joint_terms = not Config.PARAM_ID_FREEZE_JOINT_TERMS_SIM
    Y_stack, param_names = build_stacked_regressor(
        backend, q_meas, qd_meas, qdd_traj, stride=stride,
        include_joint_terms=include_joint_terms, q_ref=q_ref, coulomb_eps=Config.PARAM_ID_COULOMB_EPS,
    )
    if include_joint_terms:
        tau_stack = tau_meas[::stride, :].ravel()
    else:
        tau_stack = (tau_meas[::stride, :] - tau_joint[::stride, :]).ravel()

    prior = make_prior_from_link_params(
        param_names,
        true_masses,
        true_coms,
        true_inertias,
        Config.PARAM_ID_JOINT_PRIORS if include_joint_terms else None,
    )
    result = solve_least_squares(
        Y_stack,
        tau_stack,
        param_names,
        prior=prior,
        inertial_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIAL,
        joint_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_JOINT,
        rcond=Config.PARAM_ID_RCOND,
        ridge=Config.PARAM_ID_RIDGE,
    )
    masses, coms, inertias = to_link_params(result, prior=prior)
    cond = compute_condition_number(Y_stack)
    pred_err = compute_prediction_error(Y_stack, tau_stack, result, param_names)
    diagnostics = get_last_diagnostics()
    inertial_Y, _ = build_stacked_regressor(
        backend, q_meas, qd_meas, qdd_traj, stride=stride, include_joint_terms=False,
    )
    inertial_metrics = _scaled_svd_metrics(inertial_Y)
    final_distal = _distal_observability(Y_stack, include_joint_terms=include_joint_terms)
    inertial_distal = _distal_observability(inertial_Y, include_joint_terms=False)

    # ---- 中文终端输出 ----
    _print_chinese_header()
    _print_identified_params(masses, coms, inertias)
    _print_comparison(masses, true_masses, inertias, true_inertias)
    if include_joint_terms:
        _print_joint_term_comparison(result)
    else:
        print("\n===== 关节摩擦/弹性/偏置辨识对比 =====")
        print("仿真模式已冻结关节项：从测量力矩中扣除已知先验关节项，仅辨识惯性参数。")
    print(f"\n惯性子回归条件数: {inertial_metrics['condition']:.1f}, rank={inertial_metrics['rank']}")
    print(f"联合回归矩阵缩放后条件数: {cond:.1f}")
    print(
        f"SVD rank: {diagnostics.get('rank', 0):.0f}/{diagnostics.get('num_params', len(param_names)):.0f}, "
        f"data-rank: {diagnostics.get('data_rank', 0):.0f}, "
        f"nullity: {diagnostics.get('nullity', 0):.0f}, "
        f"保留子空间条件数: {diagnostics.get('retained_condition', float('inf')):.1f}"
    )
    print(
        f"末端可观测性: rank={final_distal['rank']}, "
        f"condition={final_distal['condition']:.1f}, 相关={final_distal['correlation']:.3f}, "
        f"残差={final_distal['projection']['ratio']:.3f}/{final_distal['projection']['rank']}"
    )
    print(
        f"惯性末端独立性: rank={inertial_distal['rank']}, "
        f"condition={inertial_distal['condition']:.1f}, 相关={inertial_distal['correlation']:.3f}, "
        f"残差={inertial_distal['projection']['ratio']:.3f}/{inertial_distal['projection']['rank']}"
    )
    print(f"力矩预测 RMS 误差: {pred_err:.4f} N·m")
    print(f"先验偏离 RMS: {diagnostics.get('prior_delta_rms', 0.0):.6f}")
    print(f"惯性先验偏离 RMS: {diagnostics.get('inertial_prior_delta_rms', 0.0):.6f}")
    print(f"关节项先验偏离 RMS: {diagnostics.get('joint_prior_delta_rms', 0.0):.6f}")
    print(f"\n辨识参数已计算，可用于后续导出/验证。")
    print("=" * 78)

    # ---- Rerun 最终结果 ----
    if rerun_ok:
        import rerun as rr

        for j in range(7):
            rr.log(f"param_id/result/mass/J{j+1}", rr.Scalars(float(masses[j])))
            rr.log(f"param_id/result/Ixx/J{j+1}", rr.Scalars(float(inertias[j][0])))
            rr.log(f"param_id/result/Iyy/J{j+1}", rr.Scalars(float(inertias[j][1])))

    backend.close()
    print("\n[辨识] 完成。")


if __name__ == "__main__":
    main()
