#!/usr/bin/env python3
"""Simulation-only payload identification mode."""

from __future__ import annotations

import numpy as np

from robot_control.config import Config
from robot_control.dynamics.pinocchio import PinocchioGravityBackend
from robot_control.modes.param_id_sim.acquisition import _collect_pd_data
from robot_control.modes.param_id_sim.pd_controller import PDController
from robot_control.param_id.excitation import fourier_trajectory
from robot_control.param_id.payload import (
    build_payload_regressor,
    payload_prior_from_link_params,
    solve_payload_only,
)
from robot_control.param_id.preprocessing import estimate_qdd_from_qd
from robot_control.shared.mujoco.env import MujocoSimEnv


def _extract_payload_truth(backend: PinocchioGravityBackend):
    inert = backend._model.inertias[Config.NUM_JOINTS]
    mass = float(inert.mass)
    com = np.asarray(inert.lever, dtype=np.float64)
    inertia = np.asarray(inert.inertia, dtype=np.float64)
    return mass, com, np.array([inertia[0, 0], inertia[1, 1], inertia[2, 2]], dtype=np.float64)


def _generate_payload_trajectory():
    limits = (
        np.array([np.deg2rad(d) for d in [-70, -70, -50, -100, -35, -45, -45]]),
        np.array([np.deg2rad(d) for d in [70, 20, 35, 100, 35, 35, 45]]),
    )
    return fourier_trajectory(
        q0=Config.HOME_QPOS.copy(),
        n_harmonics=4,
        base_freq=0.16,
        duration=8.0,
        dt=Config.DT,
        joint_limits=limits,
        random_seed=47,
        joint_amplitude_weights=np.array([0.6, 0.6, 0.6, 0.7, 1.0, 1.0, 1.1]),
        joint_frequency_weights=np.array([0.8, 0.9, 1.0, 1.0, 1.2, 1.4, 1.6]),
    )


def _print_payload_result(result, true_mass, true_com, true_inertia) -> None:
    mass_error_pct = (result.mass - true_mass) / true_mass * 100.0 if true_mass > 1e-12 else float("nan")
    com_error = result.com - true_com
    inertia_error_pct = np.divide(
        result.inertia_diag - true_inertia,
        np.maximum(np.abs(true_inertia), 1e-12),
    ) * 100.0

    print()
    print("=" * 78)
    print("                    末端载荷辨识结果（仿真模式）")
    print("=" * 78)
    print(f"质量: {result.mass:.5f} kg  |  真值: {true_mass:.5f} kg  |  误差: {mass_error_pct:.2f}%")
    print(
        "质心: "
        f"[{result.com[0]: .5f}, {result.com[1]: .5f}, {result.com[2]: .5f}] m"
        "  |  误差: "
        f"[{com_error[0]: .5f}, {com_error[1]: .5f}, {com_error[2]: .5f}] m"
    )
    print(
        "惯量对角: "
        f"[{result.inertia_diag[0]:.6f}, {result.inertia_diag[1]:.6f}, {result.inertia_diag[2]:.6f}] kg*m^2"
    )
    print(
        "惯量误差: "
        f"[{inertia_error_pct[0]:.2f}%, {inertia_error_pct[1]:.2f}%, {inertia_error_pct[2]:.2f}%]"
    )
    print(f"payload 回归矩阵 rank: {result.rank}")
    print(f"payload 回归矩阵条件数: {result.condition_number:.1f}")
    print(f"力矩预测 RMS 误差: {result.prediction_error_rms:.6f} N*m")
    print("=" * 78)


def main() -> None:
    backend = PinocchioGravityBackend(
        urdf_path=Config.URDF_PATH,
        ee_frame_name="ArmLseventh_Link",
        tcp_offset=Config.TCP_OFFSET,
        torque_limits=Config.TORQUE_LIMITS.tolist(),
    )
    try:
        true_mass, true_com, true_inertia = _extract_payload_truth(backend)
        env = MujocoSimEnv()
        env.reset(Config.HOME_QPOS)
        env.forward()
        controller = PDController(backend)

        print("[payload-id] 生成 Fourier 激励轨迹...")
        t_arr, q_traj, qd_traj, _qdd_traj = _generate_payload_trajectory()
        print(f"[payload-id] 轨迹: {len(t_arr)} 步 @ {Config.DT * 1000:.0f}ms, 共 {t_arr[-1]:.1f}s")

        print("[payload-id] 执行仿真采集...")
        q_meas, qd_meas, tau_cmd = _collect_pd_data(env, controller, q_traj, qd_traj)
        qdd_meas = estimate_qdd_from_qd(qd_meas, Config.DT)
        stride = max(1, len(q_meas) // Config.PARAM_ID_MAX_SAMPLES)
        y_payload, payload_names = build_payload_regressor(
            backend,
            q_meas,
            qd_meas,
            qdd_meas,
            stride=stride,
        )
        tau_stack = tau_cmd[::stride, :].ravel()
        prior = payload_prior_from_link_params(true_mass, true_com, true_inertia)
        result = solve_payload_only(
            y_payload,
            tau_stack,
            payload_names,
            prior=prior,
            inertial_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIAL,
        )
        _print_payload_result(result, true_mass, true_com, true_inertia)
    finally:
        backend.close()


if __name__ == "__main__":
    main()
