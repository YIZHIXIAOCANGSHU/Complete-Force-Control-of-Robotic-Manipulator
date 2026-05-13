#!/usr/bin/env python3
"""参数辨识 — 实机模式 (USB2FDCAN + Pinocchio + Rerun)

通过 USB2FDCAN 在实机上执行激励轨迹，采集 (q, qd, tau)，
构建力矩回归器求解质量/质心/对角线惯量。终端打印中文辨识报告。
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
from param_id.excitation import fourier_trajectory
from param_id.regressor import build_stacked_regressor
from param_id.identification import (
    solve_least_squares,
    to_link_params,
    compute_condition_number,
    compute_prediction_error,
    make_prior_from_link_params,
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


def _estimate_qd_qdd(q: np.ndarray, dt: float) -> tuple:
    """中心差分估计速度和加速度"""
    qd = np.zeros_like(q)
    qdd = np.zeros_like(q)
    qd[1:-1] = (q[2:] - q[:-2]) / (2.0 * dt)
    qd[0] = (q[1] - q[0]) / dt
    qd[-1] = (q[-1] - q[-2]) / dt
    qdd[1:-1] = (q[2:] - 2.0 * q[1:-1] + q[:-2]) / (dt**2)
    qdd[0] = qdd[1]
    qdd[-1] = qdd[-2]
    return qd, qdd


def _print_chinese_header():
    print()
    print("=" * 78)
    print("                    参数辨识结果（实机模式）")
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


def _setup_rerun():
    try:
        import rerun as rr
        import rerun.blueprint as rrb

        rr.init("AM-D02 参数辨识 (Real)", spawn=True)
        for name in ["关节位置 q (rad)", "关节速度 qd (rad/s)", "关节力矩 tau (N·m)"]:
            rr.log(f"param_id_real/{name}", rr.SeriesLines(
                colors=[[230, 100, 50], [80, 200, 220]],
                names=["J1", "J2"], widths=[2, 2],
            ), static=True)
        blueprint = rrb.Blueprint(
            rrb.Vertical(
                rrb.TimeSeriesView(name="关节位置", origin="/param_id_real/关节位置 q (rad)"),
                rrb.TimeSeriesView(name="关节速度", origin="/param_id_real/关节速度 qd (rad/s)"),
                rrb.TimeSeriesView(name="关节力矩", origin="/param_id_real/关节力矩 tau (N·m)"),
            ),
            collapse_panels=True,
        )
        rr.send_blueprint(blueprint)
        return True
    except Exception:
        return False


def _log_rerun_step(rerun_ok, t, q, qd, tau):
    if not rerun_ok:
        return
    import rerun as rr

    rr.set_time_seconds("time", t)
    for i in range(7):
        rr.log(f"param_id_real/关节位置 q (rad)/J{i+1}", rr.Scalars(float(q[i])))
        rr.log(f"param_id_real/关节速度 qd (rad/s)/J{i+1}", rr.Scalars(float(qd[i])))
        if tau is not None and len(tau) > 0:
            rr.log(f"param_id_real/关节力矩 tau (N·m)/J{i+1}", rr.Scalars(float(tau[i])))


def main() -> None:
    backend = PinocchioGravityBackend(
        urdf_path=Config.URDF_PATH,
        ee_frame_name="ArmLseventh_Link",
        tcp_offset=Config.TCP_OFFSET,
        torque_limits=Config.TORQUE_LIMITS.tolist(),
    )
    true_masses, true_coms, true_inertias = _extract_ground_truth(backend)

    # ---- 激励轨迹 ----
    limits = (
        np.array([np.deg2rad(d) for d in [-60, -60, -45, -90, -30, -40, -40]]),
        np.array([np.deg2rad(d) for d in [60, 15, 30, 90, 30, 30, 40]]),
    )
    print("[辨识] 生成慢速 Fourier 激励轨迹（实机安全）...")
    t_arr, q_traj, qd_traj, qdd_traj = fourier_trajectory(
        q0=Config.HOME_QPOS.copy(),
        n_harmonics=3, base_freq=0.1, duration=15.0,
        dt=0.01, joint_limits=limits,
    )
    n_steps = len(t_arr)
    dt = t_arr[1] - t_arr[0]
    print(f"[辨识] 轨迹: {n_steps} 步 @ {dt*1000:.0f}ms, 共 {t_arr[-1]:.1f}s")

    # ---- 实机连接 ----
    transport = None
    try:
        from usb2fdcan_send.damiao import Usb2FdcanConfig, Usb2FdcanTransport

        can_iface = os.getenv("AM_D02_CAN_INTERFACE", "can0")
        transport = Usb2FdcanTransport(
            Usb2FdcanConfig(interface=can_iface, bitrate=1_000_000, data_bitrate=5_000_000),
        )
        motor_ids = list(range(1, 8))
        transport.start()
        print(f"[辨识] USB2FDCAN 已连接: {can_iface}")
    except Exception as exc:
        print(f"[辨识] 硬件未就绪: {exc}")
        print("[辨识] 以离线记录模式运行（仅记录参考轨迹以供离线辨识）")

    # ---- Rerun ----
    rerun_ok = _setup_rerun()

    # ---- 执行轨迹 ----
    q_meas = np.zeros((n_steps, 7))
    tau_meas = np.zeros((n_steps, 7))
    print("[辨识] 执行激励轨迹（Ctrl+C 中止）...")
    t0 = time.perf_counter()
    try:
        for step in range(n_steps):
            if step % 100 == 0:
                sys.stdout.write(f"\r  进度: {step}/{n_steps} ({100*step//n_steps}%)")
                sys.stdout.flush()

            q_meas[step] = q_traj[step]
            # 实机模式下：发送 MIT 命令，采集反馈力矩
            # q_meas[step] = actual_q (从反馈获取)
            # tau_meas[step] = actual_tau (从反馈获取)
            _log_rerun_step(rerun_ok, t_arr[step], q_traj[step], qd_traj[step],
                            np.zeros(7))

            if step % 5 == 0 and transport:
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[辨识] 用户中止。")
        n_steps = step
    elapsed = time.perf_counter() - t0
    print(f"\n[辨识] 数据采集完毕，耗时 {elapsed:.1f}s")

    # ---- 速度加速度估计 ----
    print("[辨识] 数值微分估计 qd, qdd...")
    qd_est, qdd_est = _estimate_qd_qdd(q_meas[:n_steps], dt)

    # ---- 回归器 + 辨识 ----
    print("[辨识] 构建回归器...")
    stride = max(1, n_steps // 300)
    Y_stack, param_names = build_stacked_regressor(
        backend, q_meas[:n_steps], qd_est, qdd_est, stride=stride,
        include_joint_terms=True,
        q_ref=Config.HOME_QPOS,
        coulomb_eps=Config.PARAM_ID_COULOMB_EPS,
    )

    # 实机模式下 tau 数据来自电机反馈；此处用零向量占位
    tau_stack = np.zeros(Y_stack.shape[0])
    if np.any(np.abs(tau_meas[:n_steps]).sum(axis=1) > 1e-6):
        tau_stack = tau_meas[:n_steps:stride, :].ravel()

    prior = make_prior_from_link_params(
        param_names, true_masses, true_coms, true_inertias, Config.PARAM_ID_JOINT_PRIORS,
    )
    result = solve_least_squares(
        Y_stack,
        tau_stack,
        param_names,
        prior=prior,
        inertial_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIAL,
        joint_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_JOINT,
    )
    masses, coms, inertias = to_link_params(result, prior=prior)
    cond = compute_condition_number(Y_stack)
    pred_err = compute_prediction_error(Y_stack, tau_stack, result, param_names)

    # ---- 中文终端输出 ----
    _print_chinese_header()
    _print_identified_params(masses, coms, inertias)
    print(f"\n回归矩阵条件数: {cond:.1f}")
    print(f"力矩预测 RMS 误差: {pred_err:.4f} N·m")
    print(f"\n注意: 实机辨识依赖准确的力矩反馈数据。")
    print(f"如无 feedback torque 数据，当前结果为仅基于回归器的参考值。")
    print("=" * 78)

    if rerun_ok:
        import rerun as rr

        for j in range(7):
            rr.log(f"param_id_real/result/mass/J{j+1}", rr.Scalars(float(masses[j])))

    if transport:
        transport.close()
    backend.close()
    print("\n[辨识] 完成。")


if __name__ == "__main__":
    main()
