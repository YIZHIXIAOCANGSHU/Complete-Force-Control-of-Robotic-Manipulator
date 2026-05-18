#!/usr/bin/env python3
"""参数辨识 — 仿真模式入口."""

from __future__ import annotations

import sys
import time

import numpy as np

from robot_control.config import Config
from robot_control.dynamics.pinocchio import PinocchioGravityBackend
from robot_control.shared.mujoco.env import MujocoSimEnv
from robot_control.shared.mujoco.ghost import create_mujoco_ghost_if_enabled
from robot_control.param_id.identification import (
    compute_condition_number,
    compute_prediction_error,
    get_last_diagnostics,
    make_prior_from_link_params,
    solve_least_squares,
    to_link_params,
)
from robot_control.param_id.diagnostics import (
    _best_regularized_case,
    _case_selection_key,
    _com_error_summary,
    _distal_observability,
    _extract_ground_truth,
    _inertia_error_summary,
    _j7_column_diagnostics,
    _joint_effect_torque,
    _joint_term_error_summary,
    _mass_error_summary,
    _parameter_group_observability,
    _regularization_grid,
    _scaled_svd_metrics,
    _segment_indices,
    _segment_prediction_rms,
    _solve_identification_case,
    _stratified_validation_rows,
    _validation_rms,
)
from robot_control.param_id.reporting import (
    _fmt_error_pct,
    _log_rerun_step,
    _log_sim_realtime_step_from_env,
    _print_chinese_header,
    _print_identification_case,
    _setup_rerun,
    _sync_realtime,
    _viewer_context,
    _write_html_report,
)
from robot_control.param_id.trajectory import (
    _apply_specialized_profile,
    _build_planned_trajectory,
    _build_trajectory_records_from_env,
    _candidate_score,
    _compute_ee_poses_for_q_traj,
    _joint_coverage,
    _select_excitation_trajectory,
    _simulate_identification_samples,
    _trajectory_profiles,
    _trajectory_seeds,
)


def main() -> None:
    # ---- 初始化 ----
    rerun_ok = _setup_rerun()

    backend = PinocchioGravityBackend(
        urdf_path=Config.URDF_PATH,
        ee_frame_name="ArmLseventh_Link",
        tcp_offset=Config.TCP_OFFSET,
        torque_limits=Config.TORQUE_LIMITS.tolist(),
    )
    true_masses, true_coms, true_inertias = _extract_ground_truth(backend)

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
    ) = _select_excitation_trajectory(
        backend,
        env,
        q0,
        limits,
    )
    env.reset(Config.HOME_QPOS)
    env.forward()
    n_steps = len(t_arr)
    print(f"[辨识] 轨迹: {n_steps} 步 @ {Config.DT*1000:.0f}ms, 共 {t_arr[-1]:.1f}s")
    print(f"[辨识] TCP 最大速度: {max_ee_speed:.3f} m/s (缩放系数 {speed_scale:.3f})")

    # ---- MuJoCo 窗口 + 轨迹执行 ----
    print("[辨识] 启动 MuJoCo 窗口，执行激励轨迹...")
    q_ref = Config.HOME_QPOS.copy()

    q_meas, qd_meas, tau_meas, _tau_joint = _simulate_identification_samples(
        env, q_traj, qd_traj, qdd_traj, q_ref,
    )
    ee_pos_desired_all, ee_quat_desired_all = _compute_ee_poses_for_q_traj(env, q_traj)

    with _viewer_context(env) as viewer:
        ghost = create_mujoco_ghost_if_enabled(
            env.model,
            env.data,
            enabled=viewer is not None and Config.ENABLE_MUJOCO_GHOST,
            alpha=Config.MUJOCO_GHOST_ALPHA,
        )
        if ghost is not None:
            ghost.dof_ids = env.dof_ids
        t0 = time.perf_counter()
        for step in range(n_steps):
            if step % 250 == 0:
                sys.stdout.write(f"\r  进度: {step}/{n_steps} ({100*step//n_steps}%)")
                sys.stdout.flush()

            data = env.data
            data.qpos[:7] = q_traj[step]
            data.qvel[:7] = qd_traj[step]
            data.qacc[:7] = qdd_traj[step]
            env.forward()
            if ghost is not None:
                ghost.update_from_qpos(q_traj[step])
                ghost.update_from_pose(ee_pos_desired_all[step], ee_quat_desired_all[step])

            if viewer is not None and step % 5 == 0:
                viewer.sync()
            if step % Config.RERUN_LOG_STRIDE == 0:
                _log_rerun_step(
                    rerun_ok,
                    t_arr[step],
                    q_meas[step],
                    qd_meas[step],
                    tau_meas[step],
                )
                _log_sim_realtime_step_from_env(
                    rerun_ok=rerun_ok,
                    env=env,
                    t=t_arr[step],
                    step=step,
                    q_actual=q_meas[step],
                    qd_actual=qd_meas[step],
                    q_desired=q_traj[step],
                    tau_received=tau_meas[step],
                    tau_applied=tau_meas[step],
                    cycle_time_ms=Config.DT * 1000.0,
                    pos_desired=ee_pos_desired_all[step],
                    quat_desired=ee_quat_desired_all[step],
                )
            _sync_realtime(t0, t_arr[step])

        elapsed = time.perf_counter() - t0
    print(f"\n[辨识] 轨迹执行完毕，耗时 {elapsed:.1f}s")

    # ---- 回归器 + 联合辨识 ----
    print("[辨识] 构建力矩回归器，执行联合辨识（惯性 + 关节项）...")
    stride = max(1, n_steps // Config.PARAM_ID_MAX_SAMPLES)
    identified_case = _best_regularized_case(
        "联合辨识结果（惯性 + 关节项）",
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
    )

    # ---- 中文终端输出 ----
    _print_chinese_header()
    _print_identification_case(identified_case, true_masses, true_inertias, true_coms=true_coms)
    report_metadata = {
        **trajectory_metadata,
        "stride": stride,
        "rerun_log_stride": Config.RERUN_LOG_STRIDE,
        "max_ee_speed": max_ee_speed,
        "speed_scale": speed_scale,
    }
    trajectory_records = _build_trajectory_records_from_env(
        env,
        t_arr,
        q_meas,
        q_traj,
        cycle_time_ms=Config.DT * 1000.0,
    )
    report_path = _write_html_report(
        identified_case,
        true_masses,
        true_coms,
        true_inertias,
        t_arr,
        q_meas,
        qd_meas,
        tau_meas,
        rerun_ok,
        report_metadata,
        trajectory_records=trajectory_records,
    )
    if report_path:
        print(f"HTML 报告已保存: {report_path}")
    print(f"\n辨识参数已计算，可用于后续导出/验证。")
    print("=" * 78)

    # ---- Rerun 最终结果 ----
    if rerun_ok:
        import rerun as rr

        for j in range(7):
            rr.log(f"param_id/result/mass/J{j+1}", rr.Scalars(float(identified_case["masses"][j])))
            rr.log(f"param_id/result/com_x/J{j+1}", rr.Scalars(float(identified_case["coms"][j][0])))
            rr.log(f"param_id/result/com_y/J{j+1}", rr.Scalars(float(identified_case["coms"][j][1])))
            rr.log(f"param_id/result/com_z/J{j+1}", rr.Scalars(float(identified_case["coms"][j][2])))
            rr.log(f"param_id/result/Ixx/J{j+1}", rr.Scalars(float(identified_case["inertias"][j][0])))
            rr.log(f"param_id/result/Iyy/J{j+1}", rr.Scalars(float(identified_case["inertias"][j][1])))
            rr.log(f"param_id/result/Izz/J{j+1}", rr.Scalars(float(identified_case["inertias"][j][2])))

    backend.close()
    print("\n[辨识] 完成。")


if __name__ == "__main__":
    main()
