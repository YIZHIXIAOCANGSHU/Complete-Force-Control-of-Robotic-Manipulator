#!/usr/bin/env python3
"""参数辨识 — 末端笛卡尔阻抗闭环仿真模式."""

from __future__ import annotations

import time

import numpy as np

from robot_control.config import Config
from robot_control.dynamics.pinocchio import PinocchioGravityBackend
from robot_control.param_id import sim_main as _base
from robot_control.shared.mujoco.env import MujocoSimEnv
from robot_control.shared.rerun.mode_param_id import param_id_prefix, result_path
from robot_control.modes.param_id_sim import acquisition as _acquisition
from robot_control.modes.param_id_sim import validation as _validation
from robot_control.modes.param_id_sim.pd_controller import PDController


_PD_RERUN_PREFIX = param_id_prefix(simulated_pd=True)


def _log_final_rerun_result(rerun_ok: bool, identified_case: dict) -> None:
    if not rerun_ok:
        return
    import rerun as rr

    for j in range(7):
        rr.log(result_path(_PD_RERUN_PREFIX, "mass", j + 1), rr.Scalars(float(identified_case["masses"][j])))
        rr.log(result_path(_PD_RERUN_PREFIX, "com_x", j + 1), rr.Scalars(float(identified_case["coms"][j][0])))
        rr.log(result_path(_PD_RERUN_PREFIX, "com_y", j + 1), rr.Scalars(float(identified_case["coms"][j][1])))
        rr.log(result_path(_PD_RERUN_PREFIX, "com_z", j + 1), rr.Scalars(float(identified_case["coms"][j][2])))
        rr.log(result_path(_PD_RERUN_PREFIX, "Ixx", j + 1), rr.Scalars(float(identified_case["inertias"][j][0])))
        rr.log(result_path(_PD_RERUN_PREFIX, "Iyy", j + 1), rr.Scalars(float(identified_case["inertias"][j][1])))
        rr.log(result_path(_PD_RERUN_PREFIX, "Izz", j + 1), rr.Scalars(float(identified_case["inertias"][j][2])))
    _base._log_final_result_bars(
        rerun_ok,
        identified_case["masses"],
        identified_case["coms"],
        identified_case["inertias"],
        param_prefix=_PD_RERUN_PREFIX,
    )


def main() -> None:
    rerun_ok = _base._setup_rerun(app_name="AM-D02 参数辨识 (Cartesian Impedance Sim)", param_prefix=_PD_RERUN_PREFIX)
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
        print("[辨识-笛卡尔阻抗] 生成 Fourier 激励轨迹并进行闭环验证...")
        (
            t_arr,
            q_traj,
            qd_traj,
            qdd_traj,
            max_ee_speed,
            speed_scale,
            _excitation_overall,
            _excitation_distal,
            trajectory_labels,
            _trajectory_metadata,
        ) = _validation._select_excitation_trajectory_pd(env, backend, controller, q0, limits)

        n_steps = len(t_arr)
        print(f"[辨识-笛卡尔阻抗] 轨迹: {n_steps} 步 @ {Config.DT * 1000:.0f}ms, 共 {t_arr[-1]:.1f}s")
        print(f"[辨识-笛卡尔阻抗] TCP 最大速度: {max_ee_speed:.3f} m/s (缩放系数 {speed_scale:.3f})")

        ee_pos_desired_all, ee_quat_desired_all = _base._compute_ee_poses_for_q_traj(env, q_traj)
        env.reset(q_traj[0])
        env.forward()
        print("[辨识-笛卡尔阻抗] 启动 MuJoCo 窗口，执行末端笛卡尔阻抗 + g+c + 零空间闭环仿真...")
        t0 = time.perf_counter()
        q_meas, qd_meas, tau_cmd = _acquisition._run_pd_simulation_with_viewer(
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
        print(f"[辨识-笛卡尔阻抗] 闭环轨迹执行完毕，耗时 {elapsed:.1f}s")

        prep = _validation._prepare_pd_identification_data(q_meas, qd_meas, tau_cmd, Config.HOME_QPOS)
        q_meas = prep["q_meas"]
        qd_meas = prep["qd_meas"]
        qdd_meas = prep["qdd_meas"]
        tau_id = prep["tau_id"]
        tracking = _validation._pd_tracking_summary(q_meas, q_traj, qd_meas, qd_traj)
        clipping = _validation._pd_clipping_summary(tau_cmd, controller.torque_limits)
        print(
            f"[辨识-PD] 已按 scale={Config.PARAM_ID_PD_JOINT_PRIOR_SCALE:.3g} "
            "扣除关节摩擦/弹性先验项用于动力学辨识，"
            f"关节跟踪 RMS={tracking['joint_rms_rad']:.4f} rad, "
            f"max={tracking['joint_max_abs_rad']:.4f} rad"
        )
        if clipping.get("clipped_any_pct", 0.0) > 1.0:
            print(f"[辨识-笛卡尔阻抗] 力矩饱和: {clipping['clipped_any_pct']:.1f}% 的时间步至少一个关节饱和")

        print("[辨识-笛卡尔阻抗] 构建力矩回归器，执行正则化辨识...")
        stride = max(1, n_steps // Config.PARAM_ID_MAX_SAMPLES)
        identified_case = _validation._best_pd_inertial_case(
            "笛卡尔阻抗闭环惯性辨识结果（指令力矩按比例扣除关节项）",
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
        print("\n笛卡尔阻抗闭环辨识参数已计算，可用于后续导出/验证。")
        print("=" * 78)
        _log_final_rerun_result(rerun_ok, identified_case)
    finally:
        backend.close()
    print("\n[辨识-笛卡尔阻抗] 完成。")


if __name__ == "__main__":
    main()
