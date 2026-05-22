#!/usr/bin/env python3
"""Rerun and terminal reporting for parameter identification."""

from __future__ import annotations

import os
import time

import numpy as np

from robot_control.config import Config
from robot_control.shared.mujoco.viewer import launch_passive_viewer
from robot_control.shared.rerun.time import set_time_seconds
from robot_control.shared.rerun import viz as rerun_viz
from robot_control.param_id.diagnostics import (
    _com_error_summary,
    _inertia_error_summary,
    _mass_error_summary,
)


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

def _bar_chart_view(rrb, name: str, origin: str):
    if hasattr(rrb, "BarChartView"):
        return rrb.BarChartView(name=name, origin=origin)
    return rrb.TimeSeriesView(name=name, origin=origin)


def _setup_rerun(app_name: str = "AM-D02 参数辨识 (Sim)", param_prefix: str = "param_id") -> bool:
    if not Config.ENABLE_RERUN:
        return False
    prefix = str(param_prefix).strip("/") or "param_id"
    if not rerun_viz.init_rerun(app_name):
        print("[辨识] Rerun 初始化失败，跳过可视化。")
        return False
    try:
        import rerun as rr
        import rerun.blueprint as rrb

        rerun_viz.setup_sim_realtime_styles()
        colors = [
            [230, 90, 70],
            [80, 170, 240],
            [70, 190, 120],
            [245, 180, 65],
            [170, 110, 230],
            [70, 200, 190],
            [220, 95, 150],
        ]
        for i in range(7):
            joint = f"J{i + 1}"
            rr.log(
                f"{prefix}/excitation_q_rad/{joint}",
                rr.SeriesLines(colors=[colors[i]], names=[f"{joint} position"], widths=[2]),
                static=True,
            )
            rr.log(
                f"{prefix}/excitation_qd_rad_s/{joint}",
                rr.SeriesLines(colors=[colors[i]], names=[f"{joint} velocity"], widths=[2]),
                static=True,
            )
            rr.log(
                f"{prefix}/tau_nm/{joint}",
                rr.SeriesLines(colors=[colors[i]], names=[f"{joint} torque"], widths=[2]),
                static=True,
            )
        overview = rrb.Vertical(
            rrb.TimeSeriesView(name="All Joint Positions q (rad)", origin=f"/{prefix}/excitation_q_rad"),
            rrb.TimeSeriesView(name="All Joint Velocities qd (rad/s)", origin=f"/{prefix}/excitation_qd_rad_s"),
            rrb.TimeSeriesView(name="All Joint Torques tau (N*m)", origin=f"/{prefix}/tau_nm"),
            name="Joint Overview",
        )
        detail_rows = []
        for start in range(1, 8, 2):
            children = []
            for joint in range(start, min(start + 2, 8)):
                children.append(
                    rrb.Vertical(
                        rrb.TimeSeriesView(
                            name=f"J{joint} Position (rad)",
                            origin=f"/{prefix}/excitation_q_rad/J{joint}",
                        ),
                        rrb.TimeSeriesView(
                            name=f"J{joint} Velocity (rad/s)",
                            origin=f"/{prefix}/excitation_qd_rad_s/J{joint}",
                        ),
                        name=f"J{joint}",
                    )
                )
            detail_rows.append(rrb.Horizontal(*children, name=f"J{start}-J{min(start + 1, 7)}"))
        details = rrb.Vertical(*detail_rows, name="Joint Details")
        results = rrb.Vertical(
            _bar_chart_view(rrb, "Identified Mass by Joint", f"/{prefix}/result_bar/mass"),
            _bar_chart_view(rrb, "Identified COM X by Joint", f"/{prefix}/result_bar/com_x"),
            _bar_chart_view(rrb, "Identified COM Y by Joint", f"/{prefix}/result_bar/com_y"),
            _bar_chart_view(rrb, "Identified COM Z by Joint", f"/{prefix}/result_bar/com_z"),
            _bar_chart_view(rrb, "Identified Ixx by Joint", f"/{prefix}/result_bar/Ixx"),
            _bar_chart_view(rrb, "Identified Iyy by Joint", f"/{prefix}/result_bar/Iyy"),
            _bar_chart_view(rrb, "Identified Izz by Joint", f"/{prefix}/result_bar/Izz"),
            name="Identification Results",
        )

        sim_pos_views = [
            rrb.TimeSeriesView(name=f"EE Position {axis} (mm)", origin=f"/tracking/pos/{axis}")
            for axis in ("X", "Y", "Z")
        ]
        sim_rot_views = [
            rrb.TimeSeriesView(name=f"EE Rotation {axis} (deg)", origin=f"/tracking/rot/{axis}")
            for axis in ("Roll", "Pitch", "Yaw")
        ]
        sim_pos_err_views = [
            rrb.TimeSeriesView(name=f"Position Error {axis} (mm)", origin=f"/error/{axis}")
            for axis in ("X", "Y", "Z")
        ]
        sim_rot_err_views = [
            rrb.TimeSeriesView(name=f"Rotation Error {axis} (deg)", origin=f"/error/{axis}")
            for axis in ("Roll", "Pitch", "Yaw")
        ]
        sim_torque_views = [
            rrb.TimeSeriesView(
                name=f"J{i + 1} Received/Applied Torque (N*m)",
                origin=f"/sim/control/torque/J{i + 1}",
            )
            for i in range(7)
        ]
        sim_tabs = [
            rrb.Spatial3DView(name="3D Interactive", origin="/trajectory_3d"),
            rrb.Vertical(
                rrb.Horizontal(*sim_pos_views),
                rrb.Horizontal(*sim_rot_views),
                name="EE Tracking",
            ),
            rrb.Vertical(
                rrb.Horizontal(*sim_pos_err_views),
                rrb.Horizontal(*sim_rot_err_views),
                name="EE Tracking Error",
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(name="Joint Positions (rad)", origin="/joint_state/q"),
                rrb.TimeSeriesView(name="Joint Velocities (rad/s)", origin="/joint_state/qd"),
                name="Joint States",
            ),
            rrb.Vertical(
                rrb.Horizontal(*sim_torque_views[:4], name="J1-J4 Torque"),
                rrb.Horizontal(*sim_torque_views[4:], name="J5-J7 Torque"),
                name="Sim Joint Torque Input",
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(name="MuJoCo Step Time (ms)", origin="/sim/performance/step_time_ms"),
                name="Sim Performance",
            ),
        ]
        combined_blueprint = rrb.Blueprint(
            rrb.Tabs(*sim_tabs, overview, details, results, name="Param ID + Sim"),
            collapse_panels=True,
        )
        rr.send_blueprint(combined_blueprint)
        return True
    except Exception as exc:
        print(f"[辨识] Rerun 初始化失败，跳过可视化: {exc}")
        return False

def _log_rerun_step(rerun_ok: bool, t: float, q, qd, tau, param_prefix: str = "param_id"):
    if not rerun_ok:
        return
    import rerun as rr

    prefix = str(param_prefix).strip("/") or "param_id"
    set_time_seconds(rr, "time", t)
    for i in range(7):
        rr.log(f"{prefix}/excitation_q_rad/J{i + 1}", rr.Scalars(float(q[i])))
        rr.log(f"{prefix}/excitation_qd_rad_s/J{i + 1}", rr.Scalars(float(qd[i])))
        rr.log(f"{prefix}/tau_nm/J{i + 1}", rr.Scalars(float(tau[i])))


def _log_result_bar(rr, prefix: str, group: str, values) -> None:
    path = f"{prefix}/result_bar/{group}"
    arr = np.asarray(values, dtype=np.float64)
    if hasattr(rr, "BarChart"):
        rr.log(path, rr.BarChart(arr, abscissa=np.arange(1, len(arr) + 1, dtype=np.float64)))
    else:
        for index, value in enumerate(arr, start=1):
            rr.log(f"{path}/J{index}", rr.Scalars(float(value)))


def _log_final_result_bars(rerun_ok: bool, masses, coms, inertias, param_prefix: str = "param_id") -> None:
    if not rerun_ok:
        return
    import rerun as rr

    prefix = str(param_prefix).strip("/") or "param_id"
    masses = np.asarray(masses, dtype=np.float64)
    coms = np.asarray(coms, dtype=np.float64)
    inertias = np.asarray(inertias, dtype=np.float64)
    _log_result_bar(rr, prefix, "mass", masses)
    for axis, axis_index in (("com_x", 0), ("com_y", 1), ("com_z", 2)):
        _log_result_bar(rr, prefix, axis, coms[:, axis_index])
    for axis, axis_index in (("Ixx", 0), ("Iyy", 1), ("Izz", 2)):
        _log_result_bar(rr, prefix, axis, inertias[:, axis_index])

def _log_sim_realtime_step_from_env(
    rerun_ok: bool,
    env,
    t: float,
    step: int,
    q_actual,
    qd_actual,
    q_desired,
    tau_received,
    tau_applied,
    cycle_time_ms: float,
    pos_desired=None,
    quat_desired=None,
) -> None:
    if not rerun_ok:
        return

    saved_qpos = env.data.qpos.copy()
    saved_qvel = env.data.qvel.copy()
    saved_qacc = env.data.qacc.copy()

    try:
        env.set_qpos(np.asarray(q_actual, dtype=np.float64))
        env.set_qvel(np.asarray(qd_actual, dtype=np.float64))
        env.forward()
        pos_actual = env.get_ee_pos()
        quat_actual = env.get_ee_quat()

        if pos_desired is None or quat_desired is None:
            env.set_qpos(np.asarray(q_desired, dtype=np.float64))
            env.set_qvel(np.zeros(Config.NUM_JOINTS, dtype=np.float64))
            env.forward()
            pos_desired = env.get_ee_pos()
            quat_desired = env.get_ee_quat()
    finally:
        env.data.qpos[:] = saved_qpos
        env.data.qvel[:] = saved_qvel
        env.data.qacc[:] = saved_qacc
        env.forward()

    rerun_viz.log_sim_realtime_step(
        t=t,
        pos_actual=pos_actual,
        pos_desired=np.asarray(pos_desired, dtype=np.float64),
        quat_actual=quat_actual,
        quat_desired=np.asarray(quat_desired, dtype=np.float64),
        tau_received=tau_received,
        tau_applied=tau_applied,
        cycle_time=cycle_time_ms,
        q=q_actual,
        qd=qd_actual,
        q_target=np.asarray(q_desired, dtype=np.float64),
        step_count=step,
    )

def _fmt_error_pct(value, target=5.0):
    value = float(value)
    if not np.isfinite(value):
        return "nan ✗"
    if abs(value) > 1000.0:
        return ">1000% ✗"
    return f"{value:+.1f}% {'✓' if abs(value) <= float(target) else '✗'}"

def _print_box_line(text="", left="║", right="║", width=74):
    clipped = str(text)[:width]
    print(f"{left}{clipped:<{width}}{right}")

def _pass_label(passes):
    return "✓ 通过" if passes else "✗ 未通过"

def _print_executive_summary(case):
    mass = case.get("mass_summary", {})
    com = case.get("com_summary", {})
    inertia = case.get("inertia_summary", {})
    print()
    print("╔" + "═" * 74 + "╗")
    _print_box_line("参数辨识结果总览".center(60))
    print("╠" + "═" * 74 + "╣")
    _print_box_line(
        f"  质量: {_pass_label(mass.get('passes_5pct', False)):<8} "
        f"最大误差 J{mass.get('max_abs_joint', 0)}: {mass.get('max_abs', float('nan')):.2f}% "
        f"(目标 ≤ {mass.get('target_pct', Config.PARAM_ID_MASS_ERROR_TARGET_PCT):.1f}%)"
    )
    _print_box_line(
        f"  COM:  {_pass_label(com.get('passes_target', False)):<8} "
        f"最大误差 J{com.get('max_distance_joint', 0)}: {com.get('max_distance', float('nan')):.4f} m "
        f"(目标 ≤ {com.get('target_m', Config.PARAM_ID_COM_ERROR_TARGET_M):.4f} m)"
    )
    _print_box_line(
        f"  惯量: {_pass_label(inertia.get('passes_target', False)):<8} "
        f"最大误差 J{inertia.get('max_component_joint', 0)}-{inertia.get('max_component_axis', '')}: "
        f"{inertia.get('max_component_abs', float('nan')):.2f}% "
        f"(目标 ≤ {inertia.get('target_pct', Config.PARAM_ID_INERTIA_ERROR_TARGET_PCT):.1f}%)"
    )
    joint_terms = case.get("joint_term_error_summary", {})
    _print_box_line(
        f"  摩擦/弹性关节项: 力矩RMS {joint_terms.get('torque_rms', 0.0):.4f} N·m, "
        f"最大 J{joint_terms.get('torque_max_abs_joint', 0)}: {joint_terms.get('torque_max_abs', 0.0):.4f} N·m"
    )
    _print_box_line(
        f"  训练/验证 RMS: {case.get('prediction_error', float('nan')):.4f} / "
        f"{case.get('validation_rms', float('nan')):.4f} N·m "
        f"(比值 {case.get('validation_ratio', float('nan')):.3f})"
    )
    print("╚" + "═" * 74 + "╝")

def _print_inertial_results(case, true_masses, true_coms=None, true_inertias=None):
    masses = np.asarray(case.get("masses", np.zeros(7)), dtype=np.float64)
    coms = np.asarray(case.get("coms", np.zeros((7, 3))), dtype=np.float64)
    inertias = np.asarray(case.get("inertias", np.zeros((7, 3))), dtype=np.float64)
    true_masses = np.asarray(true_masses, dtype=np.float64)
    true_inertias = np.asarray(true_inertias if true_inertias is not None else np.zeros((7, 3)), dtype=np.float64)
    mass_summary = case.get("mass_summary") or _mass_error_summary(masses, true_masses)
    com_summary = case.get("com_summary") or _com_error_summary(
        coms,
        np.zeros_like(coms) if true_coms is None else true_coms,
    )

    print("\n┌─ 惯性参数辨识结果 ──────────────────────────────────────────────┐")
    print("│ 质量+COM")
    print("│ 关节  质量(kg)  真值(kg)  误差        COMx      COMy      COMz     COM误差m")
    for j in range(7):
        err = mass_summary.get("errors", [0.0] * 7)[j]
        tm = true_masses[j] if j < true_masses.size else 0.0
        com_err = com_summary.get("distance_errors", [0.0] * 7)[j]
        print(
            f"│ J{j + 1:<2} {masses[j]:>10.4f} {tm:>9.4f} "
            f"{_fmt_error_pct(err, mass_summary.get('target_pct', 5.0)):>12} "
            f"{coms[j][0]:>9.4f} {coms[j][1]:>9.4f} {coms[j][2]:>9.4f} {com_err:>10.5f}"
        )
    print("│")
    print("│ 惯量")
    print("│ 关节  Ixx(辨识)  Ixx(真值)  Iyy(辨识)  Iyy(真值)  Izz(辨识)  Izz(真值)")
    for j in range(7):
        truth = true_inertias[j] if j < len(true_inertias) else np.zeros(3, dtype=np.float64)
        print(
            f"│ J{j + 1:<2} {inertias[j][0]:>10.6f} {truth[0]:>10.6f} "
            f"{inertias[j][1]:>10.6f} {truth[1]:>10.6f} "
            f"{inertias[j][2]:>10.6f} {truth[2]:>10.6f}"
        )
    print("└────────────────────────────────────────────────────────────────┘")

def _print_joint_results(case):
    print("\n┌─ 关节摩擦/弹性辨识 ─────────────────────────────────────────────┐")
    _print_joint_term_comparison(case.get("result", {}))
    summary = case.get("joint_term_error_summary", {})
    print(
        f"关节项力矩误差: RMS={summary.get('torque_rms', 0.0):.5f} N·m, "
        f"Max={summary.get('torque_max_abs', 0.0):.5f} N·m @J{summary.get('torque_max_abs_joint', 0)}, "
        f"最大参数误差={summary.get('max_abs_param', '')} {summary.get('max_abs_param_error', 0.0):.5f}"
    )
    print("└────────────────────────────────────────────────────────────────┘")

def _diagnostics_requested():
    return os.getenv("AM_D02_PARAM_ID_DIAGNOSTICS", "").strip().lower() in ("1", "true", "yes", "on")

def _print_diagnostics(case):
    diagnostics = case.get("diagnostics", {})
    inertial_metrics = case.get("inertial_metrics", {})
    distal = case.get("distal", {})
    inertial_distal = case.get("inertial_distal", {})
    seg = case.get("segment_rms", {})
    sel = case.get("selection", {})

    print("\n┌─ 诊断摘要 ─────────────────────────────────────────────────────┐")
    print(
        f"│ 回归矩阵条件数: scaled={case.get('condition', float('nan')):.3g} "
        f"inertial={inertial_metrics.get('condition', float('nan')):.3g}"
    )
    print(
        f"│ SVD: rank={diagnostics.get('rank', 0):.0f}/"
        f"{diagnostics.get('num_params', len(case.get('param_names', []))):.0f} "
        f"data-rank={diagnostics.get('data_rank', 0):.0f} "
        f"retained-cond={diagnostics.get('retained_condition', float('nan')):.3g}"
    )
    print(
        f"│ 末端/惯性末端: distal-rank={distal.get('rank', 0)} "
        f"inertial-rank={inertial_distal.get('rank', 0)} "
        f"residual={inertial_distal.get('projection', {}).get('ratio', float('nan')):.3f}"
    )
    print(
        f"│ 分段RMS: 动态={seg.get('dynamic', float('nan')):.4f}, "
        f"远端={seg.get('j6j7', float('nan')):.4f}/{seg.get('j7', float('nan')):.4f}, "
        f"静态={seg.get('gravity', float('nan')):.4f}/{seg.get('com_gravity', float('nan')):.4f}, "
        f"惯量={seg.get('inertia', float('nan')):.4f}"
    )
    if sel:
        print(
            f"│ 正则化: λ_m={sel.get('mass_prior_lambda', 0.0):.3g} "
            f"λ_c={sel.get('com_prior_lambda', 0.0):.3g} "
            f"λ_i={sel.get('inertia_prior_lambda', 0.0):.3g} "
            f"λ_j={sel.get('joint_prior_lambda', 0.0):.3g} "
            f"rcond={sel.get('rcond', 0.0):.1e}"
        )
    print("└────────────────────────────────────────────────────────────────┘")

    if not _diagnostics_requested():
        return

    print("\n┌─ 诊断详情 (AM_D02_PARAM_ID_DIAGNOSTICS=1) ─────────────────────┐")
    group_observability = case.get("group_observability", {})
    for label, key in (
        ("质量列", "mass"),
        ("COM列", "com"),
        ("惯量列", "inertia"),
        ("末端COM列", "distal_com"),
        ("末端惯量列", "distal_inertia"),
    ):
        obs = group_observability.get(key)
        if not obs:
            continue
        proj = obs.get("projection", {})
        print(
            f"│ {label}: rank={obs.get('rank', 0)}, condition={obs.get('condition', float('nan')):.3g}, "
            f"相关={obs.get('correlation', float('nan')):.3f}, 残差={proj.get('ratio', float('nan')):.3f}/{proj.get('rank', 0)}"
        )
    mass_summary = case.get("mass_summary", {})
    j7_columns = case.get("j7_columns", {})
    print(
        f"│ J7专项: 质量误差={mass_summary.get('j7_abs', float('nan')):.2f}%, "
        f"列范数 mass={j7_columns.get('mass_norm', 0.0):.3e}, "
        f"mean={j7_columns.get('mean_norm', 0.0):.3e}, min={j7_columns.get('min_norm', 0.0):.3e}"
    )
    print(
        f"│ 先验偏离 RMS: all={diagnostics.get('prior_delta_rms', 0.0):.6f}, "
        f"mass={diagnostics.get('mass_prior_delta_rms', 0.0):.6f}, "
        f"COM={diagnostics.get('com_prior_delta_rms', 0.0):.6f}, "
        f"inertia={diagnostics.get('inertia_prior_delta_rms', 0.0):.6f}"
    )
    print(
        f"│ 激励质量: min={diagnostics.get('link_excitation_min', float('nan')):.3g}, "
        f"mean={diagnostics.get('link_excitation_mean', float('nan')):.3g}"
    )
    print("└────────────────────────────────────────────────────────────────┘")

def _print_identification_case(case, true_masses, true_inertias, true_coms=None):
    print()
    print("=" * 78)
    print(f"                    {case['name']}")
    print("=" * 78)
    _print_executive_summary(case)
    _print_inertial_results(case, true_masses, true_coms=true_coms, true_inertias=true_inertias)
    _print_joint_results(case)
    _print_diagnostics(case)
