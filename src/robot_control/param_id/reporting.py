#!/usr/bin/env python3
"""Rerun, terminal, CSV, JSON, and HTML reporting for parameter identification."""

from __future__ import annotations

import csv
import json
import os
import time
from contextlib import nullcontext
from datetime import datetime
from html import escape
from pathlib import Path

import numpy as np

from robot_control.config import Config
from robot_control.shared.mujoco.viewer import launch_passive_viewer
from robot_control.shared.rerun import viz as rerun_viz
from robot_control.param_id.diagnostics import (
    _com_error_summary,
    _inertia_error_summary,
    _mass_error_summary,
)


_TRAJECTORY_CSV_COLUMNS = [
    "time",
    "step",
    "actual_x",
    "actual_y",
    "actual_z",
    "expected_x",
    "expected_y",
    "expected_z",
    "actual_roll",
    "actual_pitch",
    "actual_yaw",
    "expected_roll",
    "expected_pitch",
    "expected_yaw",
    "error_x_mm",
    "error_y_mm",
    "error_z_mm",
    "error_roll_deg",
    "error_pitch_deg",
    "error_yaw_deg",
    "cycle_time_ms",
]

_DOF_SPECS = [
    ("X", "error_x_mm", "mm", "position"),
    ("Y", "error_y_mm", "mm", "position"),
    ("Z", "error_z_mm", "mm", "position"),
    ("Roll", "error_roll_deg", "deg", "rotation"),
    ("Pitch", "error_pitch_deg", "deg", "rotation"),
    ("Yaw", "error_yaw_deg", "deg", "rotation"),
]

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

def _setup_rerun() -> bool:
    if not Config.ENABLE_RERUN:
        return False
    if not rerun_viz.init_rerun("AM-D02 参数辨识 (Sim)"):
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
                f"param_id/excitation_q_rad/{joint}",
                rr.SeriesLines(colors=[colors[i]], names=[f"{joint} position"], widths=[2]),
                static=True,
            )
            rr.log(
                f"param_id/excitation_qd_rad_s/{joint}",
                rr.SeriesLines(colors=[colors[i]], names=[f"{joint} velocity"], widths=[2]),
                static=True,
            )
            rr.log(
                f"param_id/tau_nm/{joint}",
                rr.SeriesLines(colors=[colors[i]], names=[f"{joint} torque"], widths=[2]),
                static=True,
            )
        overview = rrb.Vertical(
            rrb.TimeSeriesView(name="All Joint Positions q (rad)", origin="/param_id/excitation_q_rad"),
            rrb.TimeSeriesView(name="All Joint Velocities qd (rad/s)", origin="/param_id/excitation_qd_rad_s"),
            rrb.TimeSeriesView(name="All Joint Torques tau (N*m)", origin="/param_id/tau_nm"),
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
                            origin=f"/param_id/excitation_q_rad/J{joint}",
                        ),
                        rrb.TimeSeriesView(
                            name=f"J{joint} Velocity (rad/s)",
                            origin=f"/param_id/excitation_qd_rad_s/J{joint}",
                        ),
                        name=f"J{joint}",
                    )
                )
            detail_rows.append(rrb.Horizontal(*children, name=f"J{start}-J{min(start + 1, 7)}"))
        details = rrb.Vertical(*detail_rows, name="Joint Details")
        results = rrb.Vertical(
            rrb.TimeSeriesView(name="Identified Mass", origin="/param_id/result/mass"),
            rrb.TimeSeriesView(name="Identified COM X", origin="/param_id/result/com_x"),
            rrb.TimeSeriesView(name="Identified COM Y", origin="/param_id/result/com_y"),
            rrb.TimeSeriesView(name="Identified COM Z", origin="/param_id/result/com_z"),
            rrb.TimeSeriesView(name="Identified Inertia Diagonal", origin="/param_id/result"),
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

def _log_rerun_step(rerun_ok: bool, t: float, q, qd, tau):
    if not rerun_ok:
        return
    import rerun as rr

    rr.set_time_seconds("time", t)
    for i in range(7):
        rr.log("param_id/excitation_q_rad/J%d" % (i + 1), rr.Scalars(float(q[i])))
        rr.log("param_id/excitation_qd_rad_s/J%d" % (i + 1), rr.Scalars(float(qd[i])))
        rr.log("param_id/tau_nm/J%d" % (i + 1), rr.Scalars(float(tau[i])))

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
        step_count=step,
    )

def _fmt(value, digits=4):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return ""
    if np.isnan(number):
        return "nan"
    if np.isposinf(number):
        return "inf"
    if np.isneginf(number):
        return "-inf"
    return f"{number:.{digits}f}"

def _json_safe(value):
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if np.isfinite(number) else None
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    return value

def _finite_float(value, default=float("nan")):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default

def _case_final_loss(case):
    validation_rms = _finite_float(case.get("validation_rms"))
    if np.isfinite(validation_rms):
        return validation_rms
    return _finite_float(case.get("prediction_error"))

def _format_csv_value(value, integer=False):
    if value is None:
        return ""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not np.isfinite(number):
        return ""
    if integer and abs(number - round(number)) < 1e-12 and abs(number) < 1e12:
        return str(int(round(number)))
    return f"{number:.6f}"

def _trajectory_records_to_arrays(trajectory_records):
    records = [dict(record) for record in (trajectory_records or [])]
    arrays = {}
    for column in _TRAJECTORY_CSV_COLUMNS:
        values = [_finite_float(record.get(column)) for record in records]
        arrays[column] = np.asarray(values, dtype=np.float64)
    return records, arrays

def _dof_error_stats(trajectory_records):
    records, arrays = _trajectory_records_to_arrays(trajectory_records)
    stats = {}
    for name, column, unit, group in _DOF_SPECS:
        values = arrays.get(column, np.array([], dtype=np.float64))
        values = values[np.isfinite(values)]
        if values.size:
            stats[name] = {
                "unit": unit,
                "group": group,
                "mean": float(np.mean(values)),
                "rms": float(np.sqrt(np.mean(values**2))),
                "max_abs": float(np.max(np.abs(values))),
                "final": float(values[-1]),
                "p95_abs": float(np.percentile(np.abs(values), 95)),
            }
        else:
            stats[name] = {
                "unit": unit,
                "group": group,
                "mean": None,
                "rms": None,
                "max_abs": None,
                "final": None,
                "p95_abs": None,
            }

    pos = np.column_stack([arrays[column] for _name, column, _unit, group in _DOF_SPECS if group == "position"]) if records else np.zeros((0, 3))
    rot = np.column_stack([arrays[column] for _name, column, _unit, group in _DOF_SPECS if group == "rotation"]) if records else np.zeros((0, 3))
    pos_norm = np.linalg.norm(pos, axis=1) if pos.size else np.array([], dtype=np.float64)
    rot_norm = np.linalg.norm(rot, axis=1) if rot.size else np.array([], dtype=np.float64)

    return {
        "by_dof": stats,
        "position_norm_rms": float(np.sqrt(np.mean(pos_norm**2))) if pos_norm.size else None,
        "position_norm_max": float(np.max(pos_norm)) if pos_norm.size else None,
        "rotation_norm_rms": float(np.sqrt(np.mean(rot_norm**2))) if rot_norm.size else None,
        "rotation_norm_max": float(np.max(rot_norm)) if rot_norm.size else None,
    }

def _worst_dof_and_time(trajectory_records):
    records, arrays = _trajectory_records_to_arrays(trajectory_records)
    if not records:
        return None, None
    worst_name = None
    worst_abs = -1.0
    worst_index = None
    for name, column, _unit, _group in _DOF_SPECS:
        values = arrays.get(column, np.array([], dtype=np.float64))
        if values.size == 0:
            continue
        finite_mask = np.isfinite(values)
        if not np.any(finite_mask):
            continue
        abs_values = np.abs(values)
        index = int(np.nanargmax(abs_values))
        value = float(abs_values[index])
        if value > worst_abs:
            worst_abs = value
            worst_name = name
            worst_index = index
    if worst_index is None:
        return None, None
    times = arrays.get("time", np.array([], dtype=np.float64))
    max_time = float(times[worst_index]) if worst_index < times.size and np.isfinite(times[worst_index]) else None
    return worst_name, max_time

def _joint_param_records(case):
    result = case.get("result", {})
    records = []
    for joint in range(1, 8):
        prior = Config.PARAM_ID_JOINT_PRIORS[joint - 1]
        for term in ("fc", "k", "fv", "fo"):
            initial = float(prior.get(term, 0.0))
            final = _finite_float(result.get(f"J{joint}_{term}", initial), initial)
            delta = final - initial
            pct = delta / abs(initial) * 100.0 if abs(initial) > 1e-12 else float("nan")
            records.append(
                {
                    "parameter": f"J{joint}_{term}",
                    "joint": f"J{joint}",
                    "term": term,
                    "initial": initial,
                    "final": final,
                    "delta": delta,
                    "pct_change": pct,
                    "bound_status": "n/a",
                }
            )
    return records

def _joint_param_table_records(case):
    records = []
    for row in _joint_param_records(case):
        records.append(
            {
                "parameter": row["parameter"],
                "initial": _fmt(row["initial"], 4),
                "final": _fmt(row["final"], 4),
                "delta": _fmt(row["delta"], 4),
                "pct_change": _fmt(row["pct_change"], 2),
                "bound_status": row["bound_status"],
            }
        )
    return records

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

def _joint_term_error_table_records(case):
    summary = case.get("joint_term_error_summary", {})
    per_param_by_joint = {}
    for param in summary.get("per_param", []):
        joint = int(param.get("joint", 0))
        current = per_param_by_joint.get(joint)
        if current is None or float(param.get("abs_error", 0.0)) > float(current.get("abs_error", 0.0)):
            per_param_by_joint[joint] = param

    rows = []
    for row in summary.get("per_joint", []):
        joint = int(row.get("joint", 0))
        worst_param = per_param_by_joint.get(joint, {})
        rows.append(
            {
                "joint": f"J{joint}",
                "torque_rms": _fmt(row.get("torque_rms", 0.0), 5),
                "torque_max_abs": _fmt(row.get("torque_max_abs", 0.0), 5),
                "worst_param": worst_param.get("parameter", ""),
                "param_abs_error": _fmt(worst_param.get("abs_error", 0.0), 5),
                "param_relative_error_pct": _fmt(worst_param.get("relative_error_pct", 0.0), 2),
            }
        )
    return rows

def _error_summary_records(trajectory_records):
    stats = _dof_error_stats(trajectory_records)["by_dof"]
    records = []
    for name, _column, unit, group in _DOF_SPECS:
        item = stats[name]
        status = _error_level(group, item["max_abs"])
        records.append(
            {
                "group": "Position" if group == "position" else "Rotation",
                "dof": name,
                "unit": unit,
                "status": status,
                "rms": _fmt(item["rms"], 3) if item["rms"] is not None else "",
                "max_abs": _fmt(item["max_abs"], 3) if item["max_abs"] is not None else "",
                "final": _fmt(item["final"], 3) if item["final"] is not None else "",
                "p95_abs": _fmt(item["p95_abs"], 3) if item["p95_abs"] is not None else "",
            }
        )
    return records

def _error_level(group, max_abs):
    if max_abs is None:
        return "unavailable"
    if group == "position":
        warning = getattr(Config, "PARAM_ID_REPORT_POSITION_WARNING_MM", 5.0)
        danger = getattr(Config, "PARAM_ID_REPORT_POSITION_DANGER_MM", 10.0)
    else:
        warning = getattr(Config, "PARAM_ID_REPORT_ROTATION_WARNING_DEG", 2.0)
        danger = getattr(Config, "PARAM_ID_REPORT_ROTATION_DANGER_DEG", 5.0)
    if max_abs >= danger:
        return "danger"
    if max_abs >= warning:
        return "warning"
    return "ok"

def _threshold_warnings(trajectory_records):
    warnings = []
    stats = _dof_error_stats(trajectory_records)["by_dof"]
    for name, _column, unit, group in _DOF_SPECS:
        max_abs = stats[name]["max_abs"]
        if _error_level(group, max_abs) == "danger":
            warnings.append(f"{name} error exceeds danger threshold: max={max_abs:.3f} {unit}.")
    return warnings

def _summary_cards(summary):
    cards = []
    for item in summary.get("identification_quality", []):
        cards.append(
            {
                "label": item["label"],
                "value": item["value"],
                "level": item["level"],
            }
        )
    cards.extend(
        [
            {"label": "Status", "value": summary.get("status", "warning"), "level": summary.get("status", "warning")},
            {"label": "Final Loss", "value": _fmt(summary.get("final_loss"), 4), "level": "neutral"},
            {"label": "RMS Pos Err", "value": f"{_fmt(summary.get('rms_position_error_mm'), 3)} mm", "level": "neutral"},
            {"label": "RMS Rot Err", "value": f"{_fmt(summary.get('rms_rotation_error_deg'), 3)} deg", "level": "neutral"},
            {"label": "Worst DOF", "value": summary.get("worst_dof") or "n/a", "level": "neutral"},
            {
                "label": "Max Error Time",
                "value": f"{_fmt(summary.get('max_error_time'), 3)} s" if summary.get("max_error_time") is not None else "n/a",
                "level": "neutral",
            },
        ]
    )
    return cards

def _quality_level(value, target):
    value = float(value)
    target = max(float(target), 1e-12)
    if value <= target:
        return "ok"
    if value <= target * 2.0:
        return "warning"
    return "danger"

def _identification_quality_summary(case):
    mass = case.get("mass_summary", {})
    com = case.get("com_summary", {})
    inertia = case.get("inertia_summary", {})
    joint_terms = case.get("joint_term_error_summary", {})
    return [
        {
            "label": "质量",
            "value": (
                f"{'通过' if mass.get('passes_5pct') else '未通过'} · "
                f"{mass.get('max_abs', float('nan')):.2f}% / {mass.get('target_pct', Config.PARAM_ID_MASS_ERROR_TARGET_PCT):.1f}%"
            ),
            "level": _quality_level(mass.get("max_abs", float("inf")), mass.get("target_pct", Config.PARAM_ID_MASS_ERROR_TARGET_PCT)),
        },
        {
            "label": "COM",
            "value": (
                f"{'通过' if com.get('passes_target') else '未通过'} · "
                f"{com.get('max_distance', float('nan')):.4f} / {com.get('target_m', Config.PARAM_ID_COM_ERROR_TARGET_M):.4f} m"
            ),
            "level": _quality_level(com.get("max_distance", float("inf")), com.get("target_m", Config.PARAM_ID_COM_ERROR_TARGET_M)),
        },
        {
            "label": "惯量",
            "value": (
                f"{'通过' if inertia.get('passes_target') else '未通过'} · "
                f"{inertia.get('max_component_abs', float('nan')):.2f}% / "
                f"{inertia.get('target_pct', Config.PARAM_ID_INERTIA_ERROR_TARGET_PCT):.1f}%"
            ),
            "level": _quality_level(
                inertia.get("max_component_abs", float("inf")),
                inertia.get("target_pct", Config.PARAM_ID_INERTIA_ERROR_TARGET_PCT),
            ),
        },
        {
            "label": "摩擦/弹性关节项",
            "value": (
                f"力矩RMS {joint_terms.get('torque_rms', 0.0):.4f} N·m · "
                f"最大 {joint_terms.get('torque_max_abs', 0.0):.4f} N·m @J{joint_terms.get('torque_max_abs_joint', 0)}"
            ),
            "level": "ok" if joint_terms.get("torque_rms", 0.0) <= 0.05 else "warning",
        },
    ]

def _build_identification_summary(
    case,
    trajectory_records,
    trajectory_metadata,
    warnings,
    generated_at,
    run_id,
):
    error_stats = _dof_error_stats(trajectory_records)
    worst_dof, max_error_time = _worst_dof_and_time(trajectory_records)
    joint_records = _joint_param_records(case)
    final_params = {row["parameter"]: row["final"] for row in joint_records}
    initial_params = {row["parameter"]: row["initial"] for row in joint_records}
    rms_by_dof = {name: stats["rms"] for name, stats in error_stats["by_dof"].items()}
    max_by_dof = {name: stats["max_abs"] for name, stats in error_stats["by_dof"].items()}
    final_by_dof = {name: stats["final"] for name, stats in error_stats["by_dof"].items()}
    final_loss = _case_final_loss(case)
    initial_loss = case.get("initial_loss")
    if initial_loss is None:
        initial_loss = case.get("baseline_loss")
    initial_loss = _finite_float(initial_loss, float("nan"))
    loss_drop_ratio = (
        (initial_loss - final_loss) / initial_loss
        if np.isfinite(initial_loss) and abs(initial_loss) > 1e-12 and np.isfinite(final_loss)
        else None
    )
    status = "success"
    if warnings:
        status = "warning"
    if trajectory_records and any(value is not None and value > 0.0 for value in max_by_dof.values()):
        status = "warning" if warnings else "success"
    elif not trajectory_records:
        status = "warning"
    start_time = _finite_float(trajectory_records[0].get("time")) if trajectory_records else float("nan")
    end_time = _finite_float(trajectory_records[-1].get("time")) if trajectory_records else float("nan")
    duration = end_time - start_time if np.isfinite(start_time) and np.isfinite(end_time) else 0.0
    return _json_safe(
        {
            "run_id": run_id,
            "generated_at": generated_at,
            "status": status,
            "start_time": start_time if np.isfinite(start_time) else None,
            "end_time": end_time if np.isfinite(end_time) else None,
            "duration": float(max(duration, 0.0)),
            "sample_count": len(trajectory_records),
            "initial_params": initial_params,
            "final_params": final_params,
            "param_bounds": {},
            "final_loss": final_loss,
            "initial_loss": initial_loss if np.isfinite(initial_loss) else None,
            "loss_drop_ratio": loss_drop_ratio,
            "rms_error_by_dof": rms_by_dof,
            "max_error_by_dof": max_by_dof,
            "final_error_by_dof": final_by_dof,
            "rms_position_error_mm": error_stats["position_norm_rms"],
            "max_position_error_mm": error_stats["position_norm_max"],
            "rms_rotation_error_deg": error_stats["rotation_norm_rms"],
            "max_rotation_error_deg": error_stats["rotation_norm_max"],
            "worst_dof": worst_dof,
            "max_error_time": max_error_time,
            "before_after_metrics": case.get("before_after_metrics"),
            "identification_quality": _identification_quality_summary(case),
            "joint_term_error_summary": case.get("joint_term_error_summary", {}),
            "warnings": list(warnings),
            "trajectory_metadata": dict(trajectory_metadata or {}),
            "config": {
                "dt": Config.DT,
                "rerun_log_stride": Config.RERUN_LOG_STRIDE,
                "param_id_max_samples": Config.PARAM_ID_MAX_SAMPLES,
                "position_warning_mm": getattr(Config, "PARAM_ID_REPORT_POSITION_WARNING_MM", 5.0),
                "position_danger_mm": getattr(Config, "PARAM_ID_REPORT_POSITION_DANGER_MM", 10.0),
                "rotation_warning_deg": getattr(Config, "PARAM_ID_REPORT_ROTATION_WARNING_DEG", 2.0),
                "rotation_danger_deg": getattr(Config, "PARAM_ID_REPORT_ROTATION_DANGER_DEG", 5.0),
            },
        }
    )

def _write_trajectory_csv(report_dir, trajectory_records):
    path = Path(report_dir) / "trajectory_log.csv"
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=_TRAJECTORY_CSV_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for record in trajectory_records:
            writer.writerow(
                {
                    column: _format_csv_value(record.get(column), integer=(column == "step"))
                    for column in _TRAJECTORY_CSV_COLUMNS
                }
            )
    return path

def _write_summary_json(report_dir, summary):
    path = Path(report_dir) / "identification_summary.json"
    path.write_text(json.dumps(_json_safe(summary), ensure_ascii=False, indent=2), encoding="utf-8")
    return path

def _as_joint_matrix(values):
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 7:
        return np.zeros((0, 7), dtype=np.float64)
    return arr[:, :7]

def _records_to_html_table(records, columns):
    if not records:
        return '<p class="empty">无数据</p>'
    rows = [
        {label: str(record.get(key, "")) for key, label in columns}
        for record in records
    ]
    try:
        import pandas as pd

        return pd.DataFrame(rows).to_html(
            index=False,
            border=0,
            escape=True,
            classes=["data-table"],
        )
    except Exception:
        head = "".join(f"<th>{escape(label)}</th>" for _key, label in columns)
        body_rows = []
        for row in rows:
            cells = "".join(f"<td>{escape(row[label])}</td>" for _key, label in columns)
            body_rows.append(f"<tr>{cells}</tr>")
        return f'<table class="data-table"><thead><tr>{head}</tr></thead><tbody>{"".join(body_rows)}</tbody></table>'

def _series_stats_records(q_meas, qd_meas, tau_meas):
    records = []
    for series, unit, values in (
        ("q", "rad", _as_joint_matrix(q_meas)),
        ("qd", "rad/s", _as_joint_matrix(qd_meas)),
        ("tau", "N*m", _as_joint_matrix(tau_meas)),
    ):
        for joint in range(7):
            col = values[:, joint] if values.size else np.array([], dtype=np.float64)
            if col.size:
                minimum = float(np.min(col))
                maximum = float(np.max(col))
                mean = float(np.mean(col))
                std = float(np.std(col))
            else:
                minimum = maximum = mean = std = float("nan")
            records.append(
                {
                    "series": series,
                    "unit": unit,
                    "joint": f"J{joint + 1}",
                    "min": _fmt(minimum),
                    "max": _fmt(maximum),
                    "span": _fmt(maximum - minimum),
                    "mean": _fmt(mean),
                    "std": _fmt(std),
                }
            )
    return records

def _parameter_records(case):
    result = case.get("result", {})
    masses = np.asarray(case.get("masses", np.zeros(7)), dtype=np.float64)
    coms = np.asarray(case.get("coms", np.zeros((7, 3))), dtype=np.float64)
    inertias = np.asarray(case.get("inertias", np.zeros((7, 3))), dtype=np.float64)
    records = []
    for joint in range(7):
        records.append(
            {
                "joint": f"J{joint + 1}",
                "mass": _fmt(masses[joint]),
                "com_x": _fmt(coms[joint][0]),
                "com_y": _fmt(coms[joint][1]),
                "com_z": _fmt(coms[joint][2]),
                "ixx": _fmt(inertias[joint][0], 6),
                "iyy": _fmt(inertias[joint][1], 6),
                "izz": _fmt(inertias[joint][2], 6),
                "fc": _fmt(result.get(f"J{joint + 1}_fc", 0.0), 3),
                "k": _fmt(result.get(f"J{joint + 1}_k", 0.0), 3),
                "fv": _fmt(result.get(f"J{joint + 1}_fv", 0.0), 3),
                "fo": _fmt(result.get(f"J{joint + 1}_fo", 0.0), 3),
            }
        )
    return records

def _comparison_records(case, true_masses, true_coms, true_inertias):
    masses = np.asarray(case.get("masses", np.zeros(7)), dtype=np.float64)
    true_masses = np.asarray(true_masses, dtype=np.float64)
    com_summary = case.get("com_summary") or _com_error_summary(case.get("coms", np.zeros((7, 3))), true_coms)
    inertia_summary = case.get("inertia_summary") or _inertia_error_summary(case.get("inertias", np.zeros((7, 3))), true_inertias)
    mass_summary = case.get("mass_summary") or _mass_error_summary(masses, true_masses)
    inertia_errors = inertia_summary.get("relative_errors", [[0.0, 0.0, 0.0] for _ in range(7)])
    records = []
    for joint in range(7):
        records.append(
            {
                "joint": f"J{joint + 1}",
                "mass": _fmt(masses[joint]),
                "true_mass": _fmt(true_masses[joint] if joint < len(true_masses) else 0.0),
                "mass_error": _fmt(mass_summary["errors"][joint], 2),
                "com_error": _fmt(com_summary["distance_errors"][joint], 5),
                "ixx_error": _fmt(inertia_errors[joint][0], 2),
                "iyy_error": _fmt(inertia_errors[joint][1], 2),
                "izz_error": _fmt(inertia_errors[joint][2], 2),
            }
        )
    return records

def _diagnostic_records(case, t_arr, rerun_ok, trajectory_metadata, generated_at):
    t_arr = np.asarray(t_arr, dtype=np.float64)
    diagnostics = case.get("diagnostics", {})
    selection = case.get("selection", {})
    segment_rms = case.get("segment_rms", {})
    rows = [
        ("生成时间", generated_at),
        ("步数", len(t_arr)),
        ("dt (s)", _fmt(Config.DT, 6)),
        ("总时长 (s)", _fmt(t_arr[-1] if t_arr.size else 0.0, 3)),
        ("Rerun 启用", "是" if rerun_ok else "否"),
        ("轨迹 profile", trajectory_metadata.get("profile", "")),
        ("轨迹 seed", trajectory_metadata.get("seed", "")),
        ("回归 stride", trajectory_metadata.get("stride", "")),
        ("SVD rank", f"{diagnostics.get('rank', 0)}/{diagnostics.get('num_params', len(case.get('param_names', [])))}"),
        ("data rank", diagnostics.get("data_rank", "")),
        ("retained condition", _fmt(diagnostics.get("retained_condition", float("nan")), 3)),
        ("scaled condition", _fmt(case.get("condition", float("nan")), 3)),
        ("训练/验证 RMS", f"{_fmt(case.get('prediction_error', float('nan')))} / {_fmt(case.get('validation_rms', float('nan')))} N*m"),
        ("验证/训练比值", _fmt(case.get("validation_ratio", float("nan")), 3)),
        ("λ_mass", selection.get("mass_prior_lambda", "")),
        ("λ_com", selection.get("com_prior_lambda", "")),
        ("λ_inertia", selection.get("inertia_prior_lambda", "")),
        ("λ_joint", selection.get("joint_prior_lambda", "")),
        ("rcond", selection.get("rcond", "")),
        ("先验偏离 RMS", _fmt(diagnostics.get("prior_delta_rms", 0.0), 6)),
        ("惯性先验偏离 RMS", _fmt(diagnostics.get("inertial_prior_delta_rms", 0.0), 6)),
        ("质量先验偏离 RMS", _fmt(diagnostics.get("mass_prior_delta_rms", 0.0), 6)),
        ("COM先验偏离 RMS", _fmt(diagnostics.get("com_prior_delta_rms", 0.0), 6)),
        ("惯量先验偏离 RMS", _fmt(diagnostics.get("inertia_prior_delta_rms", 0.0), 6)),
        ("关节项先验偏离 RMS", _fmt(diagnostics.get("joint_prior_delta_rms", 0.0), 6)),
    ]
    for label in ("dynamic", "j6j7", "j7", "gravity", "com_gravity", "inertia"):
        rows.append((f"分段 RMS {label}", _fmt(segment_rms.get(label, float("nan")))))
    return [{"metric": metric, "value": value} for metric, value in rows]

def _make_plotly_charts(t_arr, q_meas, qd_meas, tau_meas):
    try:
        import plotly.graph_objects as go
    except Exception:
        return '<div class="notice">plotly 未安装，已生成表格型 HTML 报告。</div>', ["plotly 未安装，图表已降级为表格。"]

    t = np.asarray(t_arr, dtype=np.float64)
    if t.size == 0:
        t = np.arange(_as_joint_matrix(q_meas).shape[0], dtype=np.float64) * Config.DT
    chart_parts = []
    for idx, (title, unit, values) in enumerate(
        (
            ("关节位置 q", "rad", _as_joint_matrix(q_meas)),
            ("关节速度 qd", "rad/s", _as_joint_matrix(qd_meas)),
            ("测得力矩 tau", "N*m", _as_joint_matrix(tau_meas)),
        )
    ):
        fig = go.Figure()
        x = t[: values.shape[0]] if values.size else t
        for joint in range(7):
            if values.size:
                fig.add_trace(go.Scatter(x=x, y=values[:, joint], mode="lines", name=f"J{joint + 1}"))
        fig.update_layout(
            title=title,
            height=320,
            margin={"l": 52, "r": 20, "t": 50, "b": 42},
            template="plotly_white",
            xaxis_title="time (s)",
            yaxis_title=unit,
            legend={"orientation": "h", "y": -0.25},
        )
        chart_parts.append(fig.to_html(full_html=False, include_plotlyjs=True if idx == 0 else False))
    return "\n".join(chart_parts), []

def _notice_html(message):
    return f'<div class="notice">{escape(str(message))}</div>'

def _figure_block(title, html):
    return f'<div class="figure-block"><h3>{escape(title)}</h3>{html}</div>'

def _chart_placeholders(message):
    position_titles = [
        "X Error over Time (mm)",
        "Y Error over Time (mm)",
        "Z Error over Time (mm)",
    ]
    rotation_titles = [
        "Roll Error over Time (deg)",
        "Pitch Error over Time (deg)",
        "Yaw Error over Time (deg)",
    ]
    detail_titles = [
        "X Actual vs Expected (m)",
        "Y Actual vs Expected (m)",
        "Z Actual vs Expected (m)",
        "Roll Actual vs Expected (deg)",
        "Pitch Actual vs Expected (deg)",
        "Yaw Actual vs Expected (deg)",
    ]
    return {
        "parameter_charts_html": _notice_html(message),
        "before_after_html": _notice_html(
            "Before data not available. This report shows post-identification absolute tracking quality only."
        ),
        "trajectory_overview_html": _notice_html(message),
        "position_error_charts_html": "\n".join(_figure_block(title, _notice_html(message)) for title in position_titles),
        "rotation_error_charts_html": "\n".join(_figure_block(title, _notice_html(message)) for title in rotation_titles),
        "actual_expected_html": "\n".join(_figure_block(title, _notice_html(message)) for title in detail_titles),
        "diagnostic_charts_html": _notice_html(message),
    }

def _make_report_charts(case, t_arr, q_meas, qd_meas, tau_meas, trajectory_records, true_masses=None, true_inertias=None):
    try:
        import plotly.graph_objects as go
    except Exception:
        return _chart_placeholders("plotly 未安装，已生成表格型 HTML 报告。"), ["plotly 未安装，图表已降级为表格。"]

    include_plotlyjs = {"value": True}

    def to_html(fig):
        include = "inline" if include_plotlyjs["value"] else False
        include_plotlyjs["value"] = False
        return fig.to_html(
            full_html=False,
            include_plotlyjs=include,
            config={"responsive": True, "displaylogo": False},
        )

    def style(fig, title, y_title, height=320):
        fig.update_layout(
            title=title,
            template="plotly_white",
            height=height,
            margin={"l": 58, "r": 24, "t": 54, "b": 44},
            xaxis_title="sim time (s)",
            yaxis_title=y_title,
            legend={"orientation": "h", "y": -0.25},
        )
        return fig

    def use_log_axis(values):
        arr = np.asarray(values, dtype=np.float64)
        positive = arr[np.isfinite(arr) & (arr > 0.0)]
        if positive.size < 2:
            return False
        return float(np.max(positive) / max(np.min(positive), 1e-15)) >= 100.0

    records, arrays = _trajectory_records_to_arrays(trajectory_records)
    times = arrays["time"] if records else np.asarray(t_arr, dtype=np.float64)

    joint_rows = _joint_param_records(case)
    names = [row["parameter"] for row in joint_rows]
    initial = [row["initial"] for row in joint_rows]
    final = [row["final"] for row in joint_rows]
    pct_change = [row["pct_change"] for row in joint_rows]
    param_parts = []
    if joint_rows:
        fig = go.Figure()
        fig.add_trace(go.Bar(name="Initial", x=names, y=initial, marker_color="#6b7280"))
        fig.add_trace(go.Bar(name="Identified", x=names, y=final, marker_color="#2563eb"))
        fig.update_layout(
            title="Initial vs Identified Friction Parameters",
            template="plotly_white",
            barmode="group",
            height=420,
            margin={"l": 58, "r": 24, "t": 54, "b": 120},
            xaxis_tickangle=-60,
            yaxis_title="parameter value",
            legend={"orientation": "h", "y": -0.25},
        )
        param_parts.append(_figure_block("Initial vs Identified Friction Parameters", to_html(fig)))

        fig = go.Figure()
        fig.add_trace(go.Bar(x=names, y=pct_change, marker_color="#0f766e"))
        fig.update_layout(
            title="Parameter Change (%)",
            template="plotly_white",
            height=380,
            margin={"l": 58, "r": 24, "t": 54, "b": 120},
            xaxis_tickangle=-60,
            yaxis_title="change (%)",
        )
        fig.add_hline(y=0, line_color="#9ca3af", line_width=1)
        param_parts.append(_figure_block("Parameter Change (%)", to_html(fig)))
    else:
        param_parts.append(_notice_html("Friction parameter data unavailable."))

    loss_values = case.get("loss_history") or case.get("diagnostics", {}).get("loss_history")
    if loss_values:
        y = np.asarray(loss_values, dtype=np.float64)
        fig = go.Figure(go.Scatter(x=np.arange(y.size), y=y, mode="lines+markers", name="loss"))
        fig.update_layout(
            title="Loss Convergence",
            template="plotly_white",
            height=300,
            margin={"l": 58, "r": 24, "t": 54, "b": 44},
            xaxis_title="iteration",
            yaxis_title="loss",
        )
        param_parts.append(_figure_block("Loss Convergence", to_html(fig)))
    else:
        param_parts.append(_notice_html("Loss convergence history unavailable."))

    mass_values = np.asarray(case.get("masses", []), dtype=np.float64)
    true_mass_values = None if true_masses is None else np.asarray(true_masses, dtype=np.float64)
    if mass_values.size >= 7 and true_mass_values is not None and true_mass_values.size >= 7:
        joints = [f"J{joint + 1}" for joint in range(7)]
        fig = go.Figure()
        fig.add_trace(go.Bar(name="URDF", x=joints, y=true_mass_values[:7], marker_color="#64748b"))
        fig.add_trace(go.Bar(name="Identified", x=joints, y=mass_values[:7], marker_color="#2563eb"))
        fig.update_layout(
            title="Mass: Identified vs URDF",
            template="plotly_white",
            barmode="group",
            height=340,
            margin={"l": 58, "r": 24, "t": 54, "b": 52},
            yaxis_title="kg",
            legend={"orientation": "h", "y": -0.22},
        )
        if use_log_axis([*true_mass_values[:7], *mass_values[:7]]):
            fig.update_yaxes(type="log")
        param_parts.append(_figure_block("Mass: Identified vs URDF", to_html(fig)))

    inertia_values = np.asarray(case.get("inertias", []), dtype=np.float64)
    true_inertia_values = None if true_inertias is None else np.asarray(true_inertias, dtype=np.float64)
    if (
        inertia_values.ndim == 2
        and inertia_values.shape[0] >= 7
        and inertia_values.shape[1] >= 3
        and true_inertia_values is not None
        and true_inertia_values.ndim == 2
        and true_inertia_values.shape[0] >= 7
        and true_inertia_values.shape[1] >= 3
    ):
        joints = [f"J{joint + 1}" for joint in range(7)]
        for axis, axis_name in enumerate(("Ixx", "Iyy", "Izz")):
            fig = go.Figure()
            fig.add_trace(go.Bar(name="URDF", x=joints, y=true_inertia_values[:7, axis], marker_color="#64748b"))
            fig.add_trace(go.Bar(name="Identified", x=joints, y=inertia_values[:7, axis], marker_color="#0f766e"))
            fig.update_layout(
                title=f"{axis_name}: Identified vs URDF",
                template="plotly_white",
                barmode="group",
                height=320,
                margin={"l": 58, "r": 24, "t": 54, "b": 52},
                yaxis_title="kg*m^2",
                legend={"orientation": "h", "y": -0.24},
            )
            if use_log_axis([*true_inertia_values[:7, axis], *inertia_values[:7, axis]]):
                fig.update_yaxes(type="log")
            param_parts.append(_figure_block(f"{axis_name}: Identified vs URDF", to_html(fig)))
    parameter_charts_html = "\n".join(param_parts)

    before_after = case.get("before_after_metrics") or {}
    if before_after:
        metrics = list(before_after.keys())
        values = [before_after[key] for key in metrics]
        fig = go.Figure(go.Bar(x=metrics, y=values, marker_color="#059669"))
        fig.update_layout(
            title="Before / After Improvement Metrics",
            template="plotly_white",
            height=320,
            margin={"l": 58, "r": 24, "t": 54, "b": 100},
            xaxis_tickangle=-35,
            yaxis_title="improvement (%)",
        )
        before_after_html = _figure_block("Before / After Improvement Metrics", to_html(fig))
    else:
        before_after_html = _notice_html(
            "Before data not available. This report shows post-identification absolute tracking quality only."
        )

    if records:
        fig = go.Figure()
        fig.add_trace(
            go.Scatter3d(
                x=arrays["actual_x"],
                y=arrays["actual_y"],
                z=arrays["actual_z"],
                mode="lines",
                name="actual",
                line={"color": "#2563eb", "width": 5},
            )
        )
        fig.add_trace(
            go.Scatter3d(
                x=arrays["expected_x"],
                y=arrays["expected_y"],
                z=arrays["expected_z"],
                mode="lines",
                name="expected",
                line={"color": "#16a34a", "width": 4, "dash": "dash"},
            )
        )
        pos_errors = np.column_stack([arrays["error_x_mm"], arrays["error_y_mm"], arrays["error_z_mm"]])
        pos_norm = np.linalg.norm(pos_errors, axis=1)
        max_idx = int(np.nanargmax(pos_norm)) if pos_norm.size else 0
        marker_indices = [0, len(records) - 1, max_idx]
        marker_names = ["start", "end", "max position error"]
        marker_colors = ["#111827", "#7c3aed", "#dc2626"]
        for idx, label, color in zip(marker_indices, marker_names, marker_colors):
            fig.add_trace(
                go.Scatter3d(
                    x=[arrays["actual_x"][idx]],
                    y=[arrays["actual_y"][idx]],
                    z=[arrays["actual_z"][idx]],
                    mode="markers",
                    name=label,
                    marker={"size": 5, "color": color},
                )
            )
        fig.update_layout(
            title="3D Actual vs Expected End-Effector Trajectory",
            template="plotly_white",
            height=520,
            margin={"l": 0, "r": 0, "t": 54, "b": 0},
            scene={
                "xaxis_title": "x (m)",
                "yaxis_title": "y (m)",
                "zaxis_title": "z (m)",
                "aspectmode": "data",
            },
            legend={"orientation": "h", "y": -0.05},
        )
        trajectory_overview_html = _figure_block("3D Actual vs Expected End-Effector Trajectory", to_html(fig))
    else:
        trajectory_overview_html = _notice_html("Trajectory data unavailable.")

    stats = _dof_error_stats(trajectory_records)["by_dof"]

    def error_curve(title, column, unit, dof_name):
        if not records:
            return _figure_block(title, _notice_html("Trajectory data unavailable."))
        fig = go.Figure()
        fig.add_trace(
            go.Scatter(
                x=times,
                y=arrays[column],
                mode="lines",
                name=dof_name,
                hovertemplate="time=%{x:.3f}s<br>error=%{y:.4f} " + unit + "<extra></extra>",
            )
        )
        fig.add_hline(y=0, line_color="#9ca3af", line_width=1)
        item = stats[dof_name]
        if item["rms"] is not None:
            fig.add_annotation(
                xref="paper",
                yref="paper",
                x=0.99,
                y=0.96,
                xanchor="right",
                showarrow=False,
                text=f"RMS={item['rms']:.3f} {unit}<br>Max={item['max_abs']:.3f} {unit}",
                bgcolor="rgba(255,255,255,0.82)",
                bordercolor="#d1d5db",
            )
        style(fig, title, unit)
        return _figure_block(title, to_html(fig))

    position_error_charts_html = "\n".join(
        error_curve(f"{name} Error over Time (mm)", column, "mm", name)
        for name, column, _unit, group in _DOF_SPECS
        if group == "position"
    )
    rotation_error_charts_html = "\n".join(
        error_curve(f"{name} Error over Time (deg)", column, "deg", name)
        for name, column, _unit, group in _DOF_SPECS
        if group == "rotation"
    )

    detail_specs = [
        ("X Actual vs Expected (m)", "actual_x", "expected_x", "m"),
        ("Y Actual vs Expected (m)", "actual_y", "expected_y", "m"),
        ("Z Actual vs Expected (m)", "actual_z", "expected_z", "m"),
        ("Roll Actual vs Expected (deg)", "actual_roll", "expected_roll", "deg"),
        ("Pitch Actual vs Expected (deg)", "actual_pitch", "expected_pitch", "deg"),
        ("Yaw Actual vs Expected (deg)", "actual_yaw", "expected_yaw", "deg"),
    ]
    detail_parts = []
    for title, actual_col, expected_col, unit in detail_specs:
        if not records:
            detail_parts.append(_figure_block(title, _notice_html("Trajectory data unavailable.")))
            continue
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=times, y=arrays[actual_col], mode="lines", name="actual"))
        fig.add_trace(go.Scatter(x=times, y=arrays[expected_col], mode="lines", name="expected"))
        style(fig, title, unit)
        detail_parts.append(_figure_block(title, to_html(fig)))
    actual_expected_html = "\n".join(detail_parts)

    diagnostic_parts = []
    final_loss = _case_final_loss(case)
    if np.isfinite(final_loss):
        fig = go.Figure(go.Scatter(x=[0], y=[final_loss], mode="markers", name="final loss"))
        fig.update_layout(
            title="Final Loss Snapshot",
            template="plotly_white",
            height=260,
            margin={"l": 58, "r": 24, "t": 54, "b": 44},
            xaxis_title="sample",
            yaxis_title="loss",
        )
        diagnostic_parts.append(_figure_block("Final Loss Snapshot", to_html(fig)))
    t = np.asarray(t_arr, dtype=np.float64)
    if t.size == 0:
        t = np.arange(_as_joint_matrix(q_meas).shape[0], dtype=np.float64) * Config.DT
    for title, unit, values in (
        ("Joint Position Overview", "rad", _as_joint_matrix(q_meas)),
        ("Joint Velocity Overview", "rad/s", _as_joint_matrix(qd_meas)),
        ("Torque Overview", "N*m", _as_joint_matrix(tau_meas)),
    ):
        if not values.size:
            continue
        fig = go.Figure()
        x = t[: values.shape[0]]
        for joint in range(7):
            fig.add_trace(go.Scatter(x=x, y=values[:, joint], mode="lines", name=f"J{joint + 1}"))
        style(fig, title, unit)
        diagnostic_parts.append(_figure_block(title, to_html(fig)))
    diagnostic_charts_html = "\n".join(diagnostic_parts) if diagnostic_parts else _notice_html("Diagnostic chart data unavailable.")

    return (
        {
            "parameter_charts_html": parameter_charts_html,
            "before_after_html": before_after_html,
            "trajectory_overview_html": trajectory_overview_html,
            "position_error_charts_html": position_error_charts_html,
            "rotation_error_charts_html": rotation_error_charts_html,
            "actual_expected_html": actual_expected_html,
            "diagnostic_charts_html": diagnostic_charts_html,
        },
        [],
    )

def _report_styles():
    return """
body { margin: 0; font-family: Arial, "Noto Sans CJK SC", sans-serif; color: #1f2937; background: #f6f7f9; }
main { max-width: 1240px; margin: 0 auto; padding: 28px 22px 48px; }
header { margin-bottom: 20px; }
h1 { margin: 0 0 8px; font-size: 28px; font-weight: 700; letter-spacing: 0; }
h2 { margin: 0 0 14px; font-size: 19px; letter-spacing: 0; }
h3 { margin: 18px 0 10px; font-size: 15px; letter-spacing: 0; }
.subtitle { margin: 0; color: #667085; }
.section { background: #fff; border: 1px solid #dfe3ea; border-radius: 8px; padding: 18px; margin: 14px 0; }
.summary-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 10px; }
.metric { border: 1px solid #e5e7eb; border-radius: 8px; padding: 12px; background: #fbfcfe; min-height: 70px; }
.metric.ok { border-color: #86efac; background: #f0fdf4; }
.metric.warning { border-color: #fde68a; background: #fffbeb; }
.metric.danger { border-color: #fca5a5; background: #fef2f2; }
.metric.success { border-color: #86efac; background: #f0fdf4; }
.metric.neutral { background: #fbfcfe; }
.metric-label { color: #667085; font-size: 12px; margin-bottom: 8px; }
.metric-value { color: #111827; font-size: 20px; font-weight: 700; overflow-wrap: anywhere; }
.grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 14px; }
.figure-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(360px, 1fr)); gap: 12px; }
.figure-block { margin: 8px 0 16px; }
.data-table { width: 100%; border-collapse: collapse; font-size: 13px; }
.data-table th, .data-table td { padding: 8px 9px; border-bottom: 1px solid #e8eaed; text-align: right; white-space: nowrap; }
.data-table th:first-child, .data-table td:first-child { text-align: left; }
.data-table th { background: #eef2f6; color: #344054; font-weight: 700; }
.notice { padding: 12px 14px; border: 1px solid #d7b46a; background: #fff7df; border-radius: 6px; color: #634500; }
.warnings { color: #7a3500; }
.table-wrap { overflow-x: auto; margin: 8px 0 16px; }
.empty { color: #6b7280; }
.notes { color: #4b5563; line-height: 1.55; }
details summary { cursor: pointer; font-weight: 700; margin-bottom: 10px; }
@media (max-width: 760px) {
  main { padding: 20px 12px 36px; }
  h1 { font-size: 23px; }
  .section { padding: 14px; }
  .figure-grid { grid-template-columns: 1fr; }
}
"""

def _render_html_report(context):
    try:
        from jinja2 import Environment, BaseLoader

        env = Environment(loader=BaseLoader(), autoescape=True)
        return env.from_string(_PARAM_ID_HTML_TEMPLATE).render(**context)
    except Exception:
        warnings_html = "".join(f"<li>{escape(str(warning))}</li>" for warning in context.get("warnings", []))
        warning_section = ""
        if warnings_html:
            warning_section = f'<section class="section warnings"><h2>Warnings</h2><ul>{warnings_html}</ul></section>'
        cards = "".join(
            f'<div class="metric {escape(str(card.get("level", "neutral")))}"><div class="metric-label">{escape(str(card["label"]))}</div>'
            f'<div class="metric-value">{escape(str(card["value"]))}</div></div>'
            for card in context.get("summary_cards", [])
        )
        return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{escape(context["title"])}</title>
  <style>{context["styles"]}</style>
</head>
<body>
  <main>
    <header>
      <h1>{escape(context["title"])}</h1>
      <p class="subtitle">{escape(context["subtitle"])}</p>
    </header>
    <section class="section"><h2>Executive Summary</h2><div class="summary-grid">{cards}</div></section>
    {warning_section}
    <section class="section"><h2>Identification Result</h2><div class="table-wrap">{context["joint_parameter_table"]}</div><h3>摩擦/弹性关节项误差（相对先验）</h3><div class="table-wrap">{context["joint_term_error_table"]}</div>{context["parameter_charts_html"]}</section>
    <section class="section"><h2>Before / After Comparison</h2>{context["before_after_html"]}</section>
    <section class="section"><h2>Trajectory Overview</h2>{context["trajectory_overview_html"]}</section>
    <section class="section"><h2>6DoF Error Summary</h2><div class="table-wrap">{context["error_summary_table"]}</div></section>
    <section class="section"><h2>Position Error Curves</h2>{context["position_error_charts_html"]}</section>
    <section class="section"><h2>Rotation Error Curves</h2>{context["rotation_error_charts_html"]}</section>
    <section class="section"><h2>Actual vs Expected Detail</h2>{context["actual_expected_html"]}</section>
    <section class="section"><h2>Identification Diagnostics</h2>{context["diagnostic_charts_html"]}<h3>诊断摘要</h3><div class="table-wrap">{context["diagnostics_table"]}</div><h3>参数表</h3><div class="table-wrap">{context["parameter_table"]}</div><h3>真值对比</h3><div class="table-wrap">{context["comparison_table"]}</div><h3>激励统计</h3><div class="table-wrap">{context["excitation_table"]}</div></section>
    <section class="section notes"><h2>Data Notes</h2><p>误差定义：actual - expected。位置误差显示单位为 mm，姿态误差显示单位为 degree。</p><p>姿态误差基于实际四元数和目标四元数的相对旋转，再转换为 Roll / Pitch / Yaw。</p><p>摩擦/弹性关节项误差以 PARAM_ID_JOINT_PRIORS 为参考，力矩误差按 fc*tanh(qd/eps) + k*(q-q_ref) + fv*qd + fo 计算。</p><p>HTML 是摩擦辨识结束后的离线总结报告；Rerun 仍用于运行时实时观察。</p></section>
  </main>
</body>
</html>
"""

def _write_html_report(
    case,
    true_masses,
    true_coms,
    true_inertias,
    t_arr,
    q_meas,
    qd_meas,
    tau_meas,
    rerun_ok,
    trajectory_metadata=None,
    warnings=None,
    trajectory_records=None,
):
    if not getattr(Config, "PARAM_ID_ENABLE_HTML_REPORT", True):
        return None
    trajectory_metadata = dict(trajectory_metadata or {})
    warnings = list(warnings or [])
    generated_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    trajectory_records = [dict(record) for record in (trajectory_records or [])]
    warnings.extend(_threshold_warnings(trajectory_records))
    if not trajectory_records:
        warnings.append("Trajectory data unavailable; trajectory plots and 6DoF error statistics are limited.")
    if not case.get("before_after_metrics"):
        warnings.append("Before data not available. This report shows post-identification absolute tracking quality only.")
    try:
        report_dir = Path(Config.RESULTS_DIR) / "friction_id" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        report_dir.mkdir(parents=True, exist_ok=True)
        run_id = report_dir.name
        summary = _build_identification_summary(
            case,
            trajectory_records,
            trajectory_metadata,
            warnings,
            generated_at,
            run_id,
        )
        chart_parts, chart_warnings = _make_report_charts(
            case,
            t_arr,
            q_meas,
            qd_meas,
            tau_meas,
            trajectory_records,
            true_masses=true_masses,
            true_inertias=true_inertias,
        )
        warnings.extend(chart_warnings)
        summary["warnings"] = list(warnings)
        summary["status"] = "warning" if warnings else summary.get("status", "success")
        context = {
            "title": "参数辨识报告（仿真模式）",
            "subtitle": f"{case.get('name', '联合辨识结果')} · {generated_at}",
            "styles": _report_styles(),
            "warnings": warnings,
            "summary_cards": _summary_cards(summary),
            "joint_parameter_table": _records_to_html_table(
                _joint_param_table_records(case),
                [
                    ("parameter", "摩擦参数"),
                    ("initial", "初始值"),
                    ("final", "辨识值"),
                    ("delta", "变化量"),
                    ("pct_change", "变化 %"),
                    ("bound_status", "边界状态"),
                ],
            ),
            "joint_term_error_table": _records_to_html_table(
                _joint_term_error_table_records(case),
                [
                    ("joint", "关节"),
                    ("torque_rms", "力矩RMS N*m"),
                    ("torque_max_abs", "最大力矩误差 N*m"),
                    ("worst_param", "最大参数误差项"),
                    ("param_abs_error", "参数绝对误差"),
                    ("param_relative_error_pct", "参数相对误差 %"),
                ],
            ),
            "error_summary_table": _records_to_html_table(
                _error_summary_records(trajectory_records),
                [
                    ("group", "类型"),
                    ("dof", "自由度"),
                    ("unit", "单位"),
                    ("status", "状态"),
                    ("rms", "RMS"),
                    ("max_abs", "Max Abs"),
                    ("final", "Final"),
                    ("p95_abs", "P95 Abs"),
                ],
            ),
            **chart_parts,
            "diagnostics_table": _records_to_html_table(
                _diagnostic_records(case, t_arr, rerun_ok, trajectory_metadata, generated_at),
                [("metric", "指标"), ("value", "值")],
            ),
            "parameter_table": _records_to_html_table(
                _parameter_records(case),
                [
                    ("joint", "关节"),
                    ("mass", "质量 kg"),
                    ("com_x", "COM x m"),
                    ("com_y", "COM y m"),
                    ("com_z", "COM z m"),
                    ("ixx", "Ixx kg*m^2"),
                    ("iyy", "Iyy kg*m^2"),
                    ("izz", "Izz kg*m^2"),
                    ("fc", "fc"),
                    ("k", "k"),
                    ("fv", "fv"),
                    ("fo", "fo"),
                ],
            ),
            "comparison_table": _records_to_html_table(
                _comparison_records(case, true_masses, true_coms, true_inertias),
                [
                    ("joint", "关节"),
                    ("mass", "辨识质量"),
                    ("true_mass", "真值质量"),
                    ("mass_error", "质量误差 %"),
                    ("com_error", "COM距离误差 m"),
                    ("ixx_error", "Ixx误差 %"),
                    ("iyy_error", "Iyy误差 %"),
                    ("izz_error", "Izz误差 %"),
                ],
            ),
            "excitation_table": _records_to_html_table(
                _series_stats_records(q_meas, qd_meas, tau_meas),
                [
                    ("series", "序列"),
                    ("joint", "关节"),
                    ("unit", "单位"),
                    ("min", "min"),
                    ("max", "max"),
                    ("span", "span"),
                    ("mean", "mean"),
                    ("std", "std"),
                ],
            ),
        }
        _write_trajectory_csv(report_dir, trajectory_records)
        _write_summary_json(report_dir, summary)
        report_path = report_dir / "report.html"
        report_path.write_text(_render_html_report(context), encoding="utf-8")
        if getattr(Config, "PARAM_ID_HTML_OPEN_BROWSER", False):
            try:
                import webbrowser

                webbrowser.open(report_path.resolve().as_uri())
            except Exception as exc:
                print(f"[辨识] HTML 报告已生成，但无法自动打开浏览器: {exc}")
        return str(report_path)
    except Exception as exc:
        print(f"[辨识] HTML 报告生成失败: {exc}")
        return None

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

