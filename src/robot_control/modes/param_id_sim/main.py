#!/usr/bin/env python3
"""参数辨识 — PD + 前馈闭环仿真模式."""

from __future__ import annotations

import time

import numpy as np

from robot_control.config import Config
from robot_control.dynamics.pinocchio import PinocchioGravityBackend
from robot_control.param_id import sim_main as _base
from robot_control.shared.mujoco.env import MujocoSimEnv
from robot_control.modes.param_id_sim import acquisition as _acquisition
from robot_control.modes.param_id_sim import validation as _validation
from robot_control.modes.param_id_sim.pd_controller import (
    PDController,
    _as_joint_vector,
    _validate_trajectory_pair,
)


def _sync_module_base() -> None:
    _acquisition._base = _base
    _validation._base = _base


def _simulate_pd_step(*args, **kwargs):
    _sync_module_base()
    return _acquisition._simulate_pd_step(*args, **kwargs)


def _collect_pd_data(*args, **kwargs):
    _sync_module_base()
    return _acquisition._collect_pd_data(*args, **kwargs)


def _viewer_sync_stride(*args, **kwargs):
    return _acquisition._viewer_sync_stride(*args, **kwargs)


def _set_desired_pose(*args, **kwargs):
    return _acquisition._set_desired_pose(*args, **kwargs)


def _run_pd_simulation_with_viewer(*args, **kwargs):
    _sync_module_base()
    return _acquisition._run_pd_simulation_with_viewer(*args, **kwargs)


def _joint_effect_torque_sequence(*args, **kwargs):
    _sync_module_base()
    return _validation._joint_effect_torque_sequence(*args, **kwargs)


def _pd_identification_torque(*args, **kwargs):
    _sync_module_base()
    return _validation._pd_identification_torque(*args, **kwargs)


def _pd_tracking_summary(*args, **kwargs):
    return _validation._pd_tracking_summary(*args, **kwargs)


def _pd_clipping_summary(*args, **kwargs):
    return _validation._pd_clipping_summary(*args, **kwargs)


def _pd_inertia_error_summary(*args, **kwargs):
    _sync_module_base()
    return _validation._pd_inertia_error_summary(*args, **kwargs)


def _estimate_qdd_from_qd(*args, **kwargs):
    return _validation._estimate_qdd_from_qd(*args, **kwargs)


def _lowpass_filter(*args, **kwargs):
    return _validation._lowpass_filter(*args, **kwargs)


def _prepare_pd_identification_data(*args, **kwargs):
    _sync_module_base()
    return _validation._prepare_pd_identification_data(*args, **kwargs)


def _pd_validation_grid(*args, **kwargs):
    _sync_module_base()
    return _validation._pd_validation_grid(*args, **kwargs)


def _pd_regularization_grid(*args, **kwargs):
    _sync_module_base()
    return _validation._pd_regularization_grid(*args, **kwargs)


def _with_joint_prior_terms(*args, **kwargs):
    return _validation._with_joint_prior_terms(*args, **kwargs)


def _solve_pd_inertial_case(*args, **kwargs):
    _sync_module_base()
    return _validation._solve_pd_inertial_case(*args, **kwargs)


def _solve_hierarchical_pd_case(*args, **kwargs):
    _sync_module_base()
    return _validation._solve_hierarchical_pd_case(*args, **kwargs)


def _best_pd_inertial_case(*args, **kwargs):
    _sync_module_base()
    return _validation._best_pd_inertial_case(*args, **kwargs)


def _candidate_return_tuple(*args, **kwargs):
    return _validation._candidate_return_tuple(*args, **kwargs)


def _select_excitation_trajectory_pd(*args, **kwargs):
    _sync_module_base()
    return _validation._select_excitation_trajectory_pd(*args, **kwargs)


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

        prep = _prepare_pd_identification_data(q_meas, qd_meas, tau_cmd, Config.HOME_QPOS)
        q_meas = prep["q_meas"]
        qd_meas = prep["qd_meas"]
        qdd_meas = prep["qdd_meas"]
        tau_id = prep["tau_id"]
        tracking = _pd_tracking_summary(q_meas, q_traj, qd_meas, qd_traj)
        clipping = _pd_clipping_summary(tau_cmd, controller.torque_limits)
        print(
            f"[辨识-PD] 已按 scale={Config.PARAM_ID_PD_JOINT_PRIOR_SCALE:.3g} "
            "扣除关节摩擦/弹性先验项用于动力学辨识，"
            f"关节跟踪 RMS={tracking['joint_rms_rad']:.4f} rad, "
            f"max={tracking['joint_max_abs_rad']:.4f} rad"
        )
        if clipping.get("clipped_any_pct", 0.0) > 1.0:
            print(f"[辨识-PD] 力矩饱和: {clipping['clipped_any_pct']:.1f}% 的时间步至少一个关节饱和")

        print("[辨识-PD] 构建力矩回归器，执行正则化辨识...")
        stride = max(1, n_steps // Config.PARAM_ID_MAX_SAMPLES)
        identified_case = _best_pd_inertial_case(
            "PD闭环惯性辨识结果（指令力矩按比例扣除关节项）",
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
        _base._print_identification_case(identified_case, true_masses, true_inertias, true_coms=true_coms)
        report_metadata = {
            **trajectory_metadata,
            "stride": stride,
            "rerun_log_stride": Config.RERUN_LOG_STRIDE,
            "max_ee_speed": max_ee_speed,
            "speed_scale": speed_scale,
            "pd_tracking_rms_rad": tracking["joint_rms_rad"],
            "pd_tracking_max_abs_rad": tracking["joint_max_abs_rad"],
            "pd_velocity_rms_rad_s": tracking.get("velocity_rms_rad_s"),
            "pd_clipped_any_pct": clipping.get("clipped_any_pct"),
            "excitation_rank": excitation_overall.get("rank"),
            "excitation_distal_rank": excitation_distal.get("rank"),
            "qdd_source": "lowpass + savgol_gradient(qd_meas)",
            "torque_target": "tau_cmd_minus_scaled_joint_effect_prior",
            "qdd_rms_mean": prep["diag"]["qdd_rms_mean"],
            "qdd_max_abs": prep["diag"]["qdd_max_abs"],
            "tau_cmd_rms": prep["diag"]["tau_cmd_rms"],
            "tau_joint_prior_to_cmd_ratio": prep["diag"]["tau_joint_prior_to_cmd_ratio"],
            "tau_joint_prior_applied_to_cmd_ratio": prep["diag"]["tau_joint_prior_applied_to_cmd_ratio"],
            "pd_joint_prior_scale": Config.PARAM_ID_PD_JOINT_PRIOR_SCALE,
            "pd_inertia_target_pct": Config.PARAM_ID_PD_INERTIA_ERROR_TARGET_PCT,
            "pd_distal_inertia_prior_multiplier": Config.PARAM_ID_PD_DISTAL_INERTIA_PRIOR_MULTIPLIER,
            "pd_kp": controller.kp.tolist(),
            "pd_kd": controller.kd.tolist(),
            "pd_validation_reg_grid_size": len(_pd_validation_grid()),
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
