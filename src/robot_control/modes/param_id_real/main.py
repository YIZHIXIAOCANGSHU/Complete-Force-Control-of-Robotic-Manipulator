#!/usr/bin/env python3
"""Real-hardware parameter identification via USB2FDCAN feedback."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass

import numpy as np

from robot_control.config import Config
from robot_control.dynamics.pinocchio import PinocchioGravityBackend
from robot_control.modes.control_real import can_feedback
from robot_control.modes.control_real.runtime_config import (
    CAN_FEEDBACK_TIMEOUT_S,
    open_can_transport,
)
from robot_control.param_id.excitation import fourier_trajectory
from robot_control.param_id.identification import (
    compute_condition_number,
    compute_prediction_error,
    make_prior_from_link_params,
    solve_least_squares,
    to_link_params,
)
from robot_control.param_id.preprocessing import estimate_qdd_from_qd
from robot_control.param_id.regressor import build_stacked_regressor


@dataclass(frozen=True)
class RealParamIdData:
    q_meas: np.ndarray
    qd_meas: np.ndarray
    qdd_meas: np.ndarray
    tau_meas: np.ndarray
    samples: int


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
            rr.log(
                f"param_id_real/{name}",
                rr.SeriesLines(
                    colors=[[230, 100, 50], [80, 200, 220]],
                    names=["J1", "J2"],
                    widths=[2, 2],
                ),
                static=True,
            )
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
        rr.log(f"param_id_real/关节力矩 tau (N·m)/J{i+1}", rr.Scalars(float(tau[i])))


def _real_joint_gains(backend: PinocchioGravityBackend) -> tuple[np.ndarray, np.ndarray]:
    kp = np.asarray(getattr(backend, "_joint_kp"), dtype=np.float64)
    kd = np.asarray(getattr(backend, "_joint_kd"), dtype=np.float64)
    return kp.copy(), kd.copy()


def _send_param_id_command(
    transport,
    motor_ids,
    *,
    q_ref,
    qd_ref,
    kp,
    kd,
    tau_ff,
) -> None:
    for index, motor_id in enumerate(motor_ids):
        transport.send_mit_command(
            int(motor_id),
            position=float(q_ref[index]),
            velocity=float(qd_ref[index]),
            kp=float(kp[index]),
            kd=float(kd[index]),
            torque=float(tau_ff[index]),
        )


def _assert_start_pose_close(q_actual, q_start) -> None:
    error = np.asarray(q_actual, dtype=np.float64) - np.asarray(q_start, dtype=np.float64)
    max_error = float(np.max(np.abs(error)))
    if max_error > float(Config.PARAM_ID_REAL_START_TOL_RAD):
        raise RuntimeError(
            "initial joint pose is too far from trajectory start: "
            f"max_error={max_error:.4f} rad > {Config.PARAM_ID_REAL_START_TOL_RAD:.4f} rad"
        )


def _assert_torque_within_limits(tau_total) -> None:
    tau = np.asarray(tau_total, dtype=np.float64)
    limits = np.asarray(Config.TORQUE_LIMITS, dtype=np.float64)
    over = np.flatnonzero(np.abs(tau) > limits)
    if over.size:
        detail = ", ".join(
            f"J{idx + 1}={tau[idx]:.3f}/{limits[idx]:.3f}Nm"
            for idx in over
        )
        raise RuntimeError(f"predicted MIT torque exceeds limits: {detail}")


def _assert_original_safety_checks(
    backend: PinocchioGravityBackend,
    q,
    qd,
    q_ref,
    tau_total=None,
) -> None:
    q_arr = np.asarray(q, dtype=np.float64)
    qd_arr = np.asarray(qd, dtype=np.float64)
    q_ref_arr = np.asarray(q_ref, dtype=np.float64)

    check_safety = getattr(backend, "_check_safety", None)
    if callable(check_safety):
        status = int(check_safety(q_arr, qd_arr))
        if status < 0:
            raise RuntimeError(f"backend safety check failed: status={status}")

    q_ref_is_safe = getattr(backend, "_q_ref_is_safe", None)
    if callable(q_ref_is_safe) and not bool(q_ref_is_safe(q_ref_arr)):
        raise RuntimeError("backend q_ref safety check failed")

    check_joint_safety = getattr(backend, "_check_joint_safety", None)
    if callable(check_joint_safety) and tau_total is not None:
        status = int(check_joint_safety(q_arr, qd_arr, np.asarray(tau_total, dtype=np.float64)))
        if status != 0:
            raise RuntimeError(f"backend joint safety check failed: status={status}")


def _validate_identification_data(data: RealParamIdData) -> None:
    if int(data.samples) < int(Config.PARAM_ID_REAL_MIN_SAMPLES):
        raise RuntimeError(
            f"有效样本不足: {data.samples} < {Config.PARAM_ID_REAL_MIN_SAMPLES}"
        )
    tau_rms = float(np.sqrt(np.mean(np.asarray(data.tau_meas[: data.samples], dtype=np.float64) ** 2)))
    if tau_rms < float(Config.PARAM_ID_REAL_MIN_TAU_RMS):
        raise RuntimeError(
            f"反馈力矩 RMS 过低: {tau_rms:.6g} < {Config.PARAM_ID_REAL_MIN_TAU_RMS:.6g}"
        )


def _collect_real_param_id_data(
    transport,
    backend: PinocchioGravityBackend,
    t_arr,
    q_traj,
    qd_traj,
    *,
    rerun_ok: bool = False,
    feedback_timeout_s: float = CAN_FEEDBACK_TIMEOUT_S,
) -> RealParamIdData:
    t_arr = np.asarray(t_arr, dtype=np.float64)
    q_ref_traj = np.asarray(q_traj, dtype=np.float64)
    qd_ref_traj = np.asarray(qd_traj, dtype=np.float64)
    if q_ref_traj.shape != qd_ref_traj.shape or q_ref_traj.ndim != 2 or q_ref_traj.shape[1] != Config.NUM_JOINTS:
        raise ValueError(f"q_traj and qd_traj must have shape (n, {Config.NUM_JOINTS})")
    if len(t_arr) != len(q_ref_traj):
        raise ValueError("t_arr length must match trajectory length")

    motor_ids = tuple(range(1, Config.NUM_JOINTS + 1))
    dt = float(t_arr[1] - t_arr[0]) if len(t_arr) > 1 else float(Config.DT)
    kp, kd = _real_joint_gains(backend)
    q_meas = np.zeros_like(q_ref_traj)
    qd_meas = np.zeros_like(q_ref_traj)
    tau_meas = np.zeros_like(q_ref_traj)
    samples = 0

    can_feedback.startup_enable(transport, motor_ids)
    print("[辨识] 已完成 clear_error、enable 和 MIT 零力矩预置。")

    first = can_feedback.read_complete_feedback_snapshot(
        transport,
        motor_ids,
        feedback_timeout_s=feedback_timeout_s,
    )
    _assert_start_pose_close(first.q, q_ref_traj[0])

    start_wall = time.perf_counter()
    for step in range(len(q_ref_traj)):
        if step % 100 == 0:
            sys.stdout.write(f"\r  进度: {step}/{len(q_ref_traj)} ({100 * step // max(len(q_ref_traj), 1)}%)")
            sys.stdout.flush()

        snapshot = first if step == 0 else can_feedback.read_complete_feedback_snapshot(
            transport,
            motor_ids,
            feedback_timeout_s=feedback_timeout_s,
        )
        q = snapshot.q
        qd = snapshot.qd
        tau_actual = snapshot.tau
        _assert_original_safety_checks(backend, q, qd, q_ref_traj[step])
        tau_ff = np.asarray(backend.compute_nonlinear_effects(q, qd), dtype=np.float64)
        _assert_torque_within_limits(tau_ff)
        tau_total = kp * (q_ref_traj[step] - q) + kd * (qd_ref_traj[step] - qd) + tau_ff
        _assert_original_safety_checks(backend, q, qd, q_ref_traj[step], tau_total=tau_total)
        _assert_torque_within_limits(tau_total)

        q_meas[step] = q
        qd_meas[step] = qd
        tau_meas[step] = tau_actual
        samples = step + 1
        _log_rerun_step(rerun_ok, float(t_arr[step]), q, qd, tau_actual)
        _send_param_id_command(
            transport,
            motor_ids,
            q_ref=q_ref_traj[step],
            qd_ref=qd_ref_traj[step],
            kp=kp,
            kd=kd,
            tau_ff=tau_ff,
        )

        target_time = start_wall + float(t_arr[step]) + dt
        sleep_s = target_time - time.perf_counter()
        if sleep_s > 0.0:
            time.sleep(sleep_s)

    print()
    qdd_meas = estimate_qdd_from_qd(qd_meas[:samples], dt)
    return RealParamIdData(
        q_meas=q_meas[:samples],
        qd_meas=qd_meas[:samples],
        qdd_meas=qdd_meas,
        tau_meas=tau_meas[:samples],
        samples=samples,
    )


def _solve_and_report(
    backend: PinocchioGravityBackend,
    data: RealParamIdData,
    true_masses,
    true_coms,
    true_inertias,
    *,
    rerun_ok: bool,
) -> None:
    _validate_identification_data(data)
    print("[辨识] 构建回归器...")
    stride = max(1, data.samples // 300)
    Y_stack, param_names = build_stacked_regressor(
        backend,
        data.q_meas,
        data.qd_meas,
        data.qdd_meas,
        stride=stride,
        include_joint_terms=True,
        q_ref=Config.HOME_QPOS,
        coulomb_eps=Config.PARAM_ID_COULOMB_EPS,
    )
    tau_stack = data.tau_meas[::stride, :].ravel()
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
        inertial_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIAL,
        mass_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_MASS,
        com_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_COM,
        inertia_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIA,
        joint_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_JOINT,
        rcond=Config.PARAM_ID_RCOND,
        ridge=Config.PARAM_ID_RIDGE,
    )
    masses, coms, inertias = to_link_params(result, prior=prior)
    cond = compute_condition_number(Y_stack)
    pred_err = compute_prediction_error(Y_stack, tau_stack, result, param_names)

    _print_chinese_header()
    _print_identified_params(masses, coms, inertias)
    print(f"\n有效样本数: {data.samples}")
    print(f"反馈力矩 RMS: {np.sqrt(np.mean(data.tau_meas ** 2)):.6f} N·m")
    print(f"回归矩阵条件数: {cond:.1f}")
    print(f"力矩预测 RMS 误差: {pred_err:.4f} N·m")
    print("=" * 78)

    if rerun_ok:
        import rerun as rr

        for j in range(7):
            rr.log(f"param_id_real/result/mass/J{j+1}", rr.Scalars(float(masses[j])))


def _generate_real_trajectory():
    limits = (
        np.array([np.deg2rad(d) for d in [-60, -60, -45, -90, -30, -40, -40]]),
        np.array([np.deg2rad(d) for d in [60, 15, 30, 90, 30, 30, 40]]),
    )
    print("[辨识] 生成慢速 Fourier 激励轨迹（实机安全）...")
    return fourier_trajectory(
        q0=Config.HOME_QPOS.copy(),
        n_harmonics=3,
        base_freq=0.1,
        duration=15.0,
        dt=0.01,
        joint_limits=limits,
    )


def main() -> None:
    backend = PinocchioGravityBackend(
        urdf_path=Config.URDF_PATH,
        ee_frame_name="ArmLseventh_Link",
        tcp_offset=Config.TCP_OFFSET,
        torque_limits=Config.TORQUE_LIMITS.tolist(),
    )
    transport = None
    try:
        true_masses, true_coms, true_inertias = _extract_ground_truth(backend)
        t_arr, q_traj, qd_traj, _qdd_traj = _generate_real_trajectory()
        print(f"[辨识] 轨迹: {len(t_arr)} 步 @ {(t_arr[1] - t_arr[0]) * 1000:.0f}ms, 共 {t_arr[-1]:.1f}s")

        try:
            transport = open_can_transport()
        except Exception as exc:
            print(f"[辨识] 硬件未就绪，退出: {exc}")
            return
        print("[辨识] USB2FDCAN 已连接。")

        rerun_ok = _setup_rerun()
        print("[辨识] 执行激励轨迹（Ctrl+C 中止）...")
        t0 = time.perf_counter()
        try:
            data = _collect_real_param_id_data(
                transport,
                backend,
                t_arr,
                q_traj,
                qd_traj,
                rerun_ok=rerun_ok,
            )
        finally:
            can_feedback.safe_zero_and_disable(transport)
            try:
                transport.close()
            except Exception:
                pass
            transport = None
            print("[辨识] 已执行零力矩、disable 并关闭 CAN。")

        elapsed = time.perf_counter() - t0
        print(f"[辨识] 数据采集完毕，耗时 {elapsed:.1f}s")
        _solve_and_report(
            backend,
            data,
            true_masses,
            true_coms,
            true_inertias,
            rerun_ok=rerun_ok,
        )
    except KeyboardInterrupt:
        print("\n[辨识] 用户中止。")
    except can_feedback.CanFeedbackTimeout as exc:
        print(f"\n[辨识] 反馈超时，进入安全停机: 缺失电机={exc.missing_ids}")
    except RuntimeError as exc:
        print(f"\n[辨识] 已停止: {exc}")
    finally:
        if transport is not None:
            can_feedback.safe_zero_and_disable(transport)
            try:
                transport.close()
            except Exception:
                pass
        backend.close()
    print("\n[辨识] 完成。")


if __name__ == "__main__":
    main()
