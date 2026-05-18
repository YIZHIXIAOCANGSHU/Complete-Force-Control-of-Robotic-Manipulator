#!/usr/bin/env python3
"""参数辨识 — 仿真模式 (MuJoCo + Pinocchio + Rerun)

在 MuJoCo 中执行 Fourier 激励轨迹，同时弹出 MuJoCo 窗口实时显示机械臂运动，
通过 Rerun 记录轨迹/力矩/辨识结果，终端打印中文辨识报告。
"""

from __future__ import annotations

import os
import csv
import json
import sys
import time
from datetime import datetime
from html import escape
from pathlib import Path

import numpy as np

PYTHON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(PYTHON_ROOT)
for path in (PYTHON_ROOT, PROJECT_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import Config
from common.mujoco.ghost import create_mujoco_ghost_if_enabled
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


def _joint_term_error_summary(result, q_meas=None, qd_meas=None, q_ref=None, priors=None):
    priors = Config.PARAM_ID_JOINT_PRIORS if priors is None else priors
    identified = _joint_term_values_from_result(result or {}, priors=priors)
    param_rows = []
    abs_errors = []
    rel_errors = []
    for joint, (identified_terms, prior_terms) in enumerate(zip(identified, priors), start=1):
        for term in ("fc", "k", "fv", "fo"):
            prior_value = float(prior_terms.get(term, 0.0))
            identified_value = float(identified_terms.get(term, prior_value))
            error = identified_value - prior_value
            abs_error = abs(error)
            rel_error = abs_error / abs(prior_value) * 100.0 if abs(prior_value) > 1e-12 else 0.0
            param_rows.append(
                {
                    "parameter": f"J{joint}_{term}",
                    "joint": joint,
                    "term": term,
                    "identified": identified_value,
                    "prior": prior_value,
                    "error": error,
                    "abs_error": abs_error,
                    "relative_error_pct": rel_error,
                }
            )
            abs_errors.append(abs_error)
            rel_errors.append(rel_error)

    n_joints = Config.NUM_JOINTS
    q = _as_joint_matrix(q_meas) if q_meas is not None else np.zeros((0, n_joints), dtype=np.float64)
    qd = _as_joint_matrix(qd_meas) if qd_meas is not None else np.zeros((0, n_joints), dtype=np.float64)
    count = min(len(q), len(qd))
    torque_error = np.zeros((0, n_joints), dtype=np.float64)
    if count:
        q = q[:count]
        qd = qd[:count]
        q_ref_arr = np.asarray(Config.HOME_QPOS if q_ref is None else q_ref, dtype=np.float64)
        torque_identified = np.zeros((count, n_joints), dtype=np.float64)
        torque_prior = np.zeros((count, n_joints), dtype=np.float64)
        for step in range(count):
            torque_identified[step] = _joint_effect_torque(q[step], qd[step], identified, q_ref_arr)
            torque_prior[step] = _joint_effect_torque(q[step], qd[step], priors, q_ref_arr)
        torque_error = torque_identified - torque_prior

    if torque_error.size:
        per_joint_rms = np.sqrt(np.mean(torque_error ** 2, axis=0))
        per_joint_max = np.max(np.abs(torque_error), axis=0)
        torque_rms = float(np.sqrt(np.mean(torque_error ** 2)))
        torque_max = float(np.max(np.abs(torque_error)))
        torque_max_joint = int(np.argmax(per_joint_max)) + 1
    else:
        per_joint_rms = np.zeros(n_joints, dtype=np.float64)
        per_joint_max = np.zeros(n_joints, dtype=np.float64)
        torque_rms = 0.0
        torque_max = 0.0
        torque_max_joint = 1

    max_abs_idx = int(np.argmax(abs_errors)) if abs_errors else 0
    max_rel_idx = int(np.argmax(rel_errors)) if rel_errors else 0
    return {
        "max_abs_param_error": float(abs_errors[max_abs_idx]) if abs_errors else 0.0,
        "max_abs_param": param_rows[max_abs_idx]["parameter"] if param_rows else "",
        "max_relative_param_error_pct": float(rel_errors[max_rel_idx]) if rel_errors else 0.0,
        "max_relative_param": param_rows[max_rel_idx]["parameter"] if param_rows else "",
        "param_error_rms": float(np.sqrt(np.mean(np.asarray(abs_errors, dtype=np.float64) ** 2))) if abs_errors else 0.0,
        "torque_rms": torque_rms,
        "torque_max_abs": torque_max,
        "torque_max_abs_joint": torque_max_joint,
        "sample_count": int(count),
        "reference": "PARAM_ID_JOINT_PRIORS",
        "error_definition": "identified - prior",
        "torque_error_model": "fc*tanh(qd/eps) + k*(q-q_ref) + fv*qd + fo",
        "per_param": param_rows,
        "per_joint": [
            {
                "joint": joint + 1,
                "torque_rms": float(per_joint_rms[joint]),
                "torque_max_abs": float(per_joint_max[joint]),
            }
            for joint in range(n_joints)
        ],
    }


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


_PARAM_ID_HTML_TEMPLATE = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{{ title }}</title>
  <style>{{ styles }}</style>
</head>
<body>
  <main>
    <header>
      <h1>{{ title }}</h1>
      <p class="subtitle">{{ subtitle }}</p>
    </header>
    <section class="section">
      <h2>Executive Summary</h2>
      <div class="summary-grid">
      {% for card in summary_cards %}
        <div class="metric {{ card.level | default('neutral') }}">
          <div class="metric-label">{{ card.label }}</div>
          <div class="metric-value">{{ card.value }}</div>
        </div>
      {% endfor %}
      </div>
    </section>
    {% if warnings %}
    <section class="section warnings">
      <h2>Warnings</h2>
      <ul>
      {% for warning in warnings %}
        <li>{{ warning }}</li>
      {% endfor %}
      </ul>
    </section>
    {% endif %}
    <section class="section">
      <h2>Identification Result</h2>
      <div class="table-wrap">{{ joint_parameter_table | safe }}</div>
      <h3>摩擦/弹性关节项误差（相对先验）</h3>
      <div class="table-wrap">{{ joint_term_error_table | safe }}</div>
      {{ parameter_charts_html | safe }}
    </section>
    <section class="section">
      <h2>Before / After Comparison</h2>
      {{ before_after_html | safe }}
    </section>
    <section class="section">
      <h2>Trajectory Overview</h2>
      {{ trajectory_overview_html | safe }}
    </section>
    <section class="section">
      <h2>6DoF Error Summary</h2>
      <div class="table-wrap">{{ error_summary_table | safe }}</div>
    </section>
    <section class="section">
      <h2>Position Error Curves</h2>
      <div class="figure-grid">{{ position_error_charts_html | safe }}</div>
    </section>
    <section class="section">
      <h2>Rotation Error Curves</h2>
      <div class="figure-grid">{{ rotation_error_charts_html | safe }}</div>
    </section>
    <section class="section">
      <h2>Actual vs Expected Detail</h2>
      <details open>
        <summary>Actual and expected component traces</summary>
        <div class="figure-grid">{{ actual_expected_html | safe }}</div>
      </details>
    </section>
    <section class="section">
      <h2>Identification Diagnostics</h2>
      {{ diagnostic_charts_html | safe }}
      <h3>诊断摘要</h3>
      <div class="table-wrap">{{ diagnostics_table | safe }}</div>
      <h3>参数表</h3>
      <div class="table-wrap">{{ parameter_table | safe }}</div>
      <h3>真值对比</h3>
      <div class="table-wrap">{{ comparison_table | safe }}</div>
      <h3>激励统计</h3>
      <div class="table-wrap">{{ excitation_table | safe }}</div>
    </section>
    <section class="section notes">
      <h2>Data Notes</h2>
      <p>误差定义：actual - expected。位置误差显示单位为 mm，姿态误差显示单位为 degree。</p>
      <p>姿态误差基于实际四元数和目标四元数的相对旋转，再转换为 Roll / Pitch / Yaw。</p>
      <p>摩擦/弹性关节项误差以 PARAM_ID_JOINT_PRIORS 为参考，力矩误差按 fc*tanh(qd/eps) + k*(q-q_ref) + fv*qd + fo 计算。</p>
      <p>HTML 是摩擦辨识结束后的离线总结报告；Rerun 仍用于运行时实时观察。</p>
    </section>
  </main>
</body>
</html>
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


def _parameter_column_groups(n_cols):
    per_link = 7
    inertial_cols = min(7 * per_link, int(n_cols))
    groups = {
        "mass": [],
        "com": [],
        "inertia": [],
        "distal_com": [],
        "distal_inertia": [],
        "joint": list(range(inertial_cols, int(n_cols))),
    }
    distal_start = Config.PARAM_ID_DISTAL_LINK_START - 1
    for link in range(7):
        base = link * per_link
        if base >= inertial_cols:
            continue
        mass_cols = [base]
        com_cols = [base + offset for offset in (1, 2, 3) if base + offset < inertial_cols]
        inertia_cols = [base + offset for offset in (4, 5, 6) if base + offset < inertial_cols]
        groups["mass"].extend(mass_cols)
        groups["com"].extend(com_cols)
        groups["inertia"].extend(inertia_cols)
        if link >= distal_start:
            groups["distal_com"].extend(com_cols)
            groups["distal_inertia"].extend(inertia_cols)
    return {name: np.asarray(cols, dtype=np.int64) for name, cols in groups.items()}


def _column_group_observability(Y, target_cols, basis_cols=None):
    Y = np.asarray(Y, dtype=np.float64)
    target_cols = np.asarray(target_cols, dtype=np.int64)
    if basis_cols is None:
        basis_cols = np.setdiff1d(np.arange(Y.shape[1]), target_cols)
    else:
        basis_cols = np.asarray(basis_cols, dtype=np.int64)

    if Y.size == 0 or target_cols.size == 0:
        return {
            "rank": 0,
            "condition": float("inf"),
            "sigma_min": 0.0,
            "correlation": 0.0,
            "projection": {"ratio": 0.0, "rank": 0, "condition": float("inf"), "sigma_min": 0.0},
        }

    metrics = _scaled_svd_metrics(Y[:, target_cols])
    Yn = Y / np.maximum(np.linalg.norm(Y, axis=0), 1e-12)
    if basis_cols.size:
        corr = float(np.max(np.abs(Yn[:, target_cols].T @ Yn[:, basis_cols])))
    else:
        corr = 0.0
    projection = _projection_residual_metrics(Y, target_cols, basis_cols)
    return {**metrics, "correlation": corr, "projection": projection}


def _parameter_group_observability(Y):
    groups = _parameter_column_groups(Y.shape[1])
    return {
        name: _column_group_observability(Y, cols)
        for name, cols in groups.items()
        if name != "joint"
    }


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
    """Planned experiment trajectories from the J7 accuracy improvement plan."""
    profiles = [
        {
            "name": "T0",
            "description": "distal-wide seed baseline",
            "modifiers": (),
            "with_gravity": False,
            "with_com_gravity": False,
            "with_inertia_burst": False,
            "dynamic_label": "dynamic",
        },
        {
            "name": "T1",
            "description": "T0 + J7 mid/high-frequency excitation",
            "modifiers": ("j7_high_frequency",),
            "with_gravity": False,
            "with_com_gravity": False,
            "with_inertia_burst": False,
            "dynamic_label": "j7",
        },
        {
            "name": "T2",
            "description": "T0 + J6/J7 90/180 deg phase sweep",
            "modifiers": ("j6_j7_phase_sweep",),
            "with_gravity": False,
            "with_com_gravity": False,
            "with_inertia_burst": False,
            "dynamic_label": "j6j7",
        },
        {
            "name": "T3",
            "description": "T0 + quasi-static gravity posture layers",
            "modifiers": (),
            "with_gravity": True,
            "with_com_gravity": False,
            "with_inertia_burst": False,
            "dynamic_label": "dynamic",
        },
        {
            "name": "T4",
            "description": "T1 + T2 + T3 combined long trajectory",
            "modifiers": ("j7_high_frequency", "j6_j7_phase_sweep"),
            "with_gravity": True,
            "with_com_gravity": False,
            "with_inertia_burst": False,
            "dynamic_label": "j6j7",
        },
        {
            "name": "T5",
            "description": "COM gravity multi-posture holds and distal scans",
            "modifiers": (),
            "with_gravity": False,
            "with_com_gravity": True,
            "with_inertia_burst": False,
            "dynamic_label": "dynamic",
        },
        {
            "name": "T6",
            "description": "J5/J6/J7 smooth inertia burst chirps",
            "modifiers": ("j7_high_frequency",),
            "with_gravity": False,
            "with_com_gravity": False,
            "with_inertia_burst": True,
            "dynamic_label": "inertia",
        },
        {
            "name": "T7",
            "description": "COM gravity holds + distal inertia burst chirps",
            "modifiers": ("j7_high_frequency",),
            "with_gravity": False,
            "with_com_gravity": True,
            "with_inertia_burst": True,
            "dynamic_label": "inertia",
        },
    ]
    return profiles[:Config.PARAM_ID_TRAJECTORY_PROFILES]


def _limit_joint_ranges(q, q0, limits):
    q_limited = np.asarray(q, dtype=np.float64).copy()
    q_min, q_max = limits
    for joint in range(q_limited.shape[1]):
        amp = np.max(np.abs(q_limited[:, joint] - q0[joint]))
        if amp <= 0.0:
            continue
        available = min(abs(q_max[joint] - q0[joint]), abs(q0[joint] - q_min[joint]))
        if amp > 0.8 * available:
            scale = 0.8 * available / amp
            q_limited[:, joint] = q0[joint] + (q_limited[:, joint] - q0[joint]) * scale
    return q_limited


def _differentiate_trajectory(q, dt):
    edge_order = 2 if q.shape[0] > 2 else 1
    qd = np.gradient(q, dt, axis=0, edge_order=edge_order)
    qdd = np.gradient(qd, dt, axis=0, edge_order=edge_order)
    return qd, qdd


def _safe_joint_amplitude(q0, limits, joint, fraction):
    q_min, q_max = limits
    available = min(abs(q_max[joint] - q0[joint]), abs(q0[joint] - q_min[joint]))
    return float(fraction) * float(max(available, 0.0))


def _apply_j7_high_frequency(t_arr, q_traj, q0, limits):
    q = np.asarray(q_traj, dtype=np.float64).copy()
    duration = max(float(t_arr[-1] - t_arr[0]), Config.DT)
    tau = (t_arr - t_arr[0]) / duration
    window = np.sin(np.pi * tau) ** 2
    amp6 = _safe_joint_amplitude(q0, limits, 5, 0.50)
    amp7 = _safe_joint_amplitude(q0, limits, 6, 0.68)
    q[:, 5] += window * amp6 * np.sin(2.0 * np.pi * 0.55 * t_arr + np.pi / 2.0)
    q[:, 6] += window * amp7 * np.sin(2.0 * np.pi * 0.85 * t_arr + np.pi)
    return _limit_joint_ranges(q, q0, limits)


def _apply_j6_j7_phase_sweep(t_arr, q_traj, q0, limits):
    q = np.asarray(q_traj, dtype=np.float64).copy()
    duration = max(float(t_arr[-1] - t_arr[0]), Config.DT)
    tau = (t_arr - t_arr[0]) / duration
    amp6 = _safe_joint_amplitude(q0, limits, 5, 0.48)
    amp7 = _safe_joint_amplitude(q0, limits, 6, 0.66)
    for start, end, phase in ((0.0, 0.5, np.pi / 2.0), (0.5, 1.0, np.pi)):
        mask = (tau >= start) & (tau <= end)
        if not np.any(mask):
            continue
        local = (tau[mask] - start) / max(end - start, 1e-12)
        window = np.sin(np.pi * local) ** 2
        q[mask, 3] += 0.18 * window * np.sin(2.0 * np.pi * 0.20 * t_arr[mask])
        q[mask, 4] += 0.16 * window * np.sin(2.0 * np.pi * 0.34 * t_arr[mask] + np.pi / 3.0)
        q[mask, 5] += amp6 * window * np.sin(2.0 * np.pi * 0.46 * t_arr[mask])
        q[mask, 6] += amp7 * window * np.sin(2.0 * np.pi * 0.62 * t_arr[mask] + phase)
    return _limit_joint_ranges(q, q0, limits)


def _cosine_segment(q_start, q_end, duration, dt):
    n = max(2, int(round(float(duration) / float(dt))) + 1)
    alpha = np.linspace(0.0, 1.0, n)
    blend = 0.5 - 0.5 * np.cos(np.pi * alpha)
    return q_start[None, :] + (q_end - q_start)[None, :] * blend[:, None]


def _hold_segment(q, duration, dt):
    n = max(2, int(round(float(duration) / float(dt))) + 1)
    return np.repeat(np.asarray(q, dtype=np.float64)[None, :], n, axis=0)


def _clip_to_limits(q, limits):
    q_min, q_max = limits
    return np.minimum(np.maximum(q, q_min), q_max)


def _concat_labeled_segments(segments):
    q_parts = []
    labels = []
    cursor = 0
    for label, q_segment in segments:
        q_segment = np.asarray(q_segment, dtype=np.float64)
        if q_segment.size == 0:
            continue
        if q_parts:
            q_segment = q_segment[1:]
        if q_segment.size == 0:
            continue
        q_parts.append(q_segment)
        labels.extend([label] * len(q_segment))
        cursor += len(q_segment)
    if not q_parts:
        return np.zeros((0, 7), dtype=np.float64), np.array([], dtype=object)
    return np.vstack(q_parts), np.asarray(labels, dtype=object)


def _quasi_static_gravity_segment(q_start, limits, dt):
    postures = [
        [0.00, -0.55, 0.30, 0.95, 0.25, 0.25, 0.50],
        [0.45, -0.65, -0.10, 0.55, -0.30, -0.25, -0.50],
        [-0.45, -0.35, 0.25, 0.85, 0.35, -0.45, 0.00],
        [0.20, -0.20, 0.45, 0.25, -0.35, 0.35, 0.55],
    ]
    scan_delta = np.array([0.0, 0.0, 0.0, -0.35, -0.25, 0.35, -0.55], dtype=np.float64)

    current = np.asarray(q_start, dtype=np.float64)
    segments = []
    for posture in postures:
        target = _clip_to_limits(np.asarray(posture, dtype=np.float64), limits)
        scan_target = _clip_to_limits(target + scan_delta, limits)
        segments.append(("gravity", _cosine_segment(current, target, 0.9, dt)))
        segments.append(("gravity", _hold_segment(target, 1.0, dt)))
        segments.append(("gravity", _cosine_segment(target, scan_target, 1.1, dt)))
        segments.append(("gravity", _hold_segment(scan_target, 0.8, dt)))
        current = scan_target
    return _concat_labeled_segments(segments)


def _append_quasi_static_gravity(q, labels, limits, dt):
    gravity_q, gravity_labels = _quasi_static_gravity_segment(q[-1], limits, dt)
    if gravity_q.size == 0:
        return q, labels
    return (
        np.vstack([q, gravity_q[1:]]),
        np.concatenate([labels, gravity_labels[1:]]),
    )


def _com_gravity_segment(q_start, limits, dt):
    postures = [
        [0.00, -0.70, 0.55, 0.95, 0.35, -0.35, 0.55],
        [0.35, -0.55, -0.35, 0.75, -0.40, 0.35, -0.45],
        [-0.35, -0.30, 0.50, 0.45, 0.25, 0.45, 0.10],
        [0.15, -0.75, 0.10, 1.05, -0.20, -0.20, 0.45],
        [-0.20, -0.45, -0.45, 0.70, 0.40, -0.45, -0.55],
    ]
    scan_vectors = [
        [0.0, 0.0, -0.18, 0.20, -0.25, 0.25, -0.35],
        [0.0, 0.0, 0.16, -0.25, 0.25, -0.25, 0.35],
    ]

    current = np.asarray(q_start, dtype=np.float64)
    segments = []
    for idx, posture in enumerate(postures):
        target = _clip_to_limits(np.asarray(posture, dtype=np.float64), limits)
        scan_delta = np.asarray(scan_vectors[idx % len(scan_vectors)], dtype=np.float64)
        scan_target = _clip_to_limits(target + scan_delta, limits)
        segments.append(("com_gravity", _cosine_segment(current, target, 1.0, dt)))
        segments.append(("com_gravity", _hold_segment(target, 1.2, dt)))
        segments.append(("com_gravity", _cosine_segment(target, scan_target, 1.2, dt)))
        segments.append(("com_gravity", _hold_segment(scan_target, 0.9, dt)))
        current = scan_target
    return _concat_labeled_segments(segments)


def _append_com_gravity(q, labels, limits, dt):
    com_q, com_labels = _com_gravity_segment(q[-1], limits, dt)
    if com_q.size == 0:
        return q, labels
    return (
        np.vstack([q, com_q[1:]]),
        np.concatenate([labels, com_labels[1:]]),
    )


def _inertia_burst_segment(q_start, limits, dt):
    duration = 6.0
    n = max(2, int(round(duration / float(dt))) + 1)
    t = np.linspace(0.0, duration, n)
    tau = t / max(duration, 1e-12)
    window = np.sin(np.pi * tau) ** 2
    q_start = np.asarray(q_start, dtype=np.float64)
    q = np.repeat(q_start[None, :], n, axis=0)

    amp4 = _safe_joint_amplitude(q_start, limits, 3, 0.20)
    amp5 = _safe_joint_amplitude(q_start, limits, 4, 0.47)
    amp6 = _safe_joint_amplitude(q_start, limits, 5, 0.55)
    amp7 = _safe_joint_amplitude(q_start, limits, 6, 0.72)
    chirp_a = 0.35 * t + 0.045 * t * t
    chirp_b = 0.50 * t + 0.065 * t * t
    chirp_c = 0.70 * t + 0.085 * t * t
    chirp_d = 0.95 * t + 0.105 * t * t
    q[:, 3] += amp4 * window * np.sin(2.0 * np.pi * chirp_a)
    q[:, 4] += amp5 * window * np.sin(2.0 * np.pi * chirp_b + np.pi / 5.0)
    q[:, 5] += amp6 * window * np.sin(2.0 * np.pi * chirp_c + np.pi / 2.0)
    q[:, 6] += amp7 * window * np.sin(2.0 * np.pi * chirp_d + np.pi)
    return _clip_to_limits(q, limits), np.asarray(["inertia"] * n, dtype=object)


def _append_inertia_burst(q, labels, limits, dt):
    burst_q, burst_labels = _inertia_burst_segment(q[-1], limits, dt)
    if burst_q.size == 0:
        return q, labels
    return (
        np.vstack([q, burst_q[1:]]),
        np.concatenate([labels, burst_labels[1:]]),
    )


def _build_planned_trajectory(profile, seed, q0, limits):
    amp_weights, freq_weights = _distal_excitation_weights(1.75, 1.15, 0.9)
    phases = _phase_offsets(np.pi / 3.0)
    t_arr, q_traj, _, _ = fourier_trajectory(
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

    for modifier in profile["modifiers"]:
        if modifier == "j7_high_frequency":
            q_traj = _apply_j7_high_frequency(t_arr, q_traj, q0, limits)
        elif modifier == "j6_j7_phase_sweep":
            q_traj = _apply_j6_j7_phase_sweep(t_arr, q_traj, q0, limits)

    labels = np.asarray([profile["dynamic_label"]] * len(q_traj), dtype=object)
    if profile.get("with_gravity", False):
        q_traj, labels = _append_quasi_static_gravity(q_traj, labels, limits, Config.DT)
    if profile.get("with_com_gravity", False):
        q_traj, labels = _append_com_gravity(q_traj, labels, limits, Config.DT)
    if profile.get("with_inertia_burst", False):
        q_traj, labels = _append_inertia_burst(q_traj, labels, limits, Config.DT)

    q_traj = _limit_joint_ranges(q_traj, q0, limits)
    qd_traj, qdd_traj = _differentiate_trajectory(q_traj, Config.DT)
    t_arr = np.arange(len(q_traj), dtype=np.float64) * Config.DT
    return t_arr, q_traj, qd_traj, qdd_traj, labels


def _apply_specialized_profile(profile_name, t_arr, q_traj, q0, limits):
    if profile_name in ("j7-heavy", "T1", "j7_high_frequency"):
        q = _apply_j7_high_frequency(t_arr, q_traj, q0, limits)
    elif profile_name in ("gravity-scan", "T3", "gravity"):
        q, _ = _append_quasi_static_gravity(np.asarray(q_traj, dtype=np.float64), np.asarray(["dynamic"] * len(q_traj), dtype=object), limits, Config.DT)
    elif profile_name in ("T5", "com_gravity"):
        q, _ = _append_com_gravity(np.asarray(q_traj, dtype=np.float64), np.asarray(["dynamic"] * len(q_traj), dtype=object), limits, Config.DT)
    elif profile_name in ("T6", "inertia", "inertia_burst"):
        q, _ = _append_inertia_burst(np.asarray(q_traj, dtype=np.float64), np.asarray(["dynamic"] * len(q_traj), dtype=object), limits, Config.DT)
    elif profile_name in ("T2", "j6_j7_phase_sweep"):
        q = _apply_j6_j7_phase_sweep(t_arr, q_traj, q0, limits)
    else:
        return q_traj, None, None
    qd, qdd = _differentiate_trajectory(q, Config.DT)
    return q, qd, qdd


def _joint_coverage(q):
    ptp = np.ptp(q, axis=0)
    distal_start = Config.PARAM_ID_DISTAL_LINK_START - 1
    distal = ptp[distal_start:]
    return {
        "min": float(np.min(distal)) if distal.size else 0.0,
        "mean": float(np.mean(distal)) if distal.size else 0.0,
    }


def _candidate_score(overall, distal, inertial_overall, inertial_distal, group_observability, coverage, speed_scale):
    speed_penalty = max(0.0, 1.0 - float(speed_scale))
    condition_penalty = np.log10(max(inertial_overall["condition"], 1.0))
    inertial_projection = inertial_distal["projection"]
    joint_projection = distal["projection"]
    com_obs = group_observability.get("com", {})
    inertia_obs = group_observability.get("inertia", {})
    distal_com_obs = group_observability.get("distal_com", {})
    distal_inertia_obs = group_observability.get("distal_inertia", {})
    com_projection = com_obs.get("projection", {})
    inertia_projection = inertia_obs.get("projection", {})
    distal_com_projection = distal_com_obs.get("projection", {})
    distal_inertia_projection = distal_inertia_obs.get("projection", {})
    return (
        overall["rank"] * 90.0
        + inertial_overall["rank"] * 5.0
        + inertial_distal["rank"] * 30.0
        + inertial_projection["rank"] * 20.0
        + inertial_projection["ratio"] * 260.0
        + joint_projection["ratio"] * 60.0
        + com_obs.get("rank", 0) * 8.0
        + inertia_obs.get("rank", 0) * 7.0
        + distal_com_obs.get("rank", 0) * 18.0
        + distal_inertia_obs.get("rank", 0) * 18.0
        + com_projection.get("rank", 0) * 8.0
        + inertia_projection.get("rank", 0) * 7.0
        + distal_com_projection.get("ratio", 0.0) * 120.0
        + distal_inertia_projection.get("ratio", 0.0) * 180.0
        + np.log10(max(inertial_projection["sigma_min"], 1e-15) / 1e-15) * 2.0
        + np.log10(max(inertial_distal["sigma_min"], 1e-15) / 1e-15)
        + np.log10(max(com_projection.get("sigma_min", 1e-15), 1e-15) / 1e-15)
        + np.log10(max(inertia_projection.get("sigma_min", 1e-15), 1e-15) / 1e-15)
        + min(coverage["mean"], 1.0) * 4.0
        + min(coverage["min"], 0.5) * 6.0
        - inertial_distal["correlation"] * 12.0
        - com_obs.get("correlation", 0.0) * 10.0
        - inertia_obs.get("correlation", 0.0) * 10.0
        - distal["correlation"] * 4.0
        - np.log10(max(inertial_distal["condition"], 1.0)) * 2.0
        - condition_penalty * 0.2
        - speed_penalty * 12.0
    )


def _build_trajectory_records_from_env(env, t_arr, q_actual, q_expected, cycle_time_ms=None):
    t_arr = np.asarray(t_arr, dtype=np.float64)
    q_actual = np.asarray(q_actual, dtype=np.float64)
    q_expected = np.asarray(q_expected if q_expected is not None else q_actual, dtype=np.float64)
    count = min(len(t_arr), len(q_actual), len(q_expected))
    if count <= 0:
        return []

    if cycle_time_ms is None:
        cycle = np.full(count, Config.DT * 1000.0, dtype=np.float64)
    else:
        cycle = np.asarray(cycle_time_ms, dtype=np.float64)
        if cycle.ndim == 0:
            cycle = np.full(count, float(cycle), dtype=np.float64)
        else:
            cycle = cycle[:count]

    saved_qpos = env.data.qpos.copy()
    saved_qvel = env.data.qvel.copy()
    saved_qacc = env.data.qacc.copy()

    def pose_for(q):
        env.set_qpos(q)
        env.set_qvel(np.zeros(Config.NUM_JOINTS, dtype=np.float64))
        env.forward()
        return env.get_ee_pos(), env.get_ee_quat()

    records = []
    try:
        for step in range(count):
            actual_pos, actual_quat = pose_for(q_actual[step])
            expected_pos, expected_quat = pose_for(q_expected[step])
            actual_rpy = np.rad2deg(rerun_viz.quat_to_euler(actual_quat))
            expected_rpy = np.rad2deg(rerun_viz.quat_to_euler(expected_quat))
            pos_err_mm = rerun_viz._position_to_display_units(actual_pos - expected_pos)
            rot_err_deg = rerun_viz.compute_rotation_error_single(actual_quat, expected_quat)
            records.append(
                {
                    "time": float(t_arr[step]),
                    "step": int(step),
                    "actual_x": float(actual_pos[0]),
                    "actual_y": float(actual_pos[1]),
                    "actual_z": float(actual_pos[2]),
                    "expected_x": float(expected_pos[0]),
                    "expected_y": float(expected_pos[1]),
                    "expected_z": float(expected_pos[2]),
                    "actual_roll": float(actual_rpy[0]),
                    "actual_pitch": float(actual_rpy[1]),
                    "actual_yaw": float(actual_rpy[2]),
                    "expected_roll": float(expected_rpy[0]),
                    "expected_pitch": float(expected_rpy[1]),
                    "expected_yaw": float(expected_rpy[2]),
                    "error_x_mm": float(pos_err_mm[0]),
                    "error_y_mm": float(pos_err_mm[1]),
                    "error_z_mm": float(pos_err_mm[2]),
                    "error_roll_deg": float(rot_err_deg[0]),
                    "error_pitch_deg": float(rot_err_deg[1]),
                    "error_yaw_deg": float(rot_err_deg[2]),
                    "cycle_time_ms": float(cycle[min(step, len(cycle) - 1)]),
                }
            )
    finally:
        env.data.qpos[:] = saved_qpos
        env.data.qvel[:] = saved_qvel
        env.data.qacc[:] = saved_qacc
        env.forward()
    return records


def _compute_ee_poses_for_q_traj(env, q_traj):
    q_traj = np.asarray(q_traj, dtype=np.float64)
    count = len(q_traj)
    positions = np.zeros((count, 3), dtype=np.float64)
    quats = np.zeros((count, 4), dtype=np.float64)

    saved_qpos = env.data.qpos.copy()
    saved_qvel = env.data.qvel.copy()
    saved_qacc = env.data.qacc.copy()
    try:
        for step, q in enumerate(q_traj):
            env.set_qpos(q)
            env.set_qvel(np.zeros(Config.NUM_JOINTS, dtype=np.float64))
            env.forward()
            positions[step] = env.get_ee_pos()
            quats[step] = env.get_ee_quat()
    finally:
        env.data.qpos[:] = saved_qpos
        env.data.qvel[:] = saved_qvel
        env.data.qacc[:] = saved_qacc
        env.forward()

    return positions, quats


def _simulate_identification_samples(env, q_traj, qd_traj, qdd_traj, q_ref):
    n_steps = len(q_traj)
    tau_meas = np.zeros((n_steps, 7))
    tau_joint = np.zeros((n_steps, 7))
    q_meas = np.zeros((n_steps, 7))
    qd_meas = np.zeros((n_steps, 7))
    for step in range(n_steps):
        data = env.data
        data.qpos[:7] = q_traj[step]
        data.qvel[:7] = qd_traj[step]
        data.qacc[:7] = qdd_traj[step]
        mujoco.mj_inverse(env.model, data)
        q_meas[step] = data.qpos[:7].copy()
        qd_meas[step] = data.qvel[:7].copy()
        tau_joint[step] = _joint_effect_torque(q_meas[step], qd_meas[step], Config.PARAM_ID_JOINT_PRIORS, q_ref)
        tau_meas[step] = data.qfrc_inverse[:7].copy() + tau_joint[step]
    return q_meas, qd_meas, tau_meas, tau_joint


def _select_excitation_trajectory(backend, env, q0, limits):
    best = None
    candidates = []
    for profile in _trajectory_profiles():
        for seed in _trajectory_seeds()[: Config.PARAM_ID_TRAJECTORY_CANDIDATES]:
            t_arr, q_traj, qd_traj, qdd_traj, labels = _build_planned_trajectory(profile, seed, q0, limits)
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
            group_observability = _parameter_group_observability(Y_probe)
            coverage = _joint_coverage(q_limited)
            score = _candidate_score(
                overall,
                distal,
                inertial_overall,
                inertial_distal,
                group_observability,
                coverage,
                speed_scale,
            )
            candidate = {
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
            candidates.append(candidate)

    ranked_candidates = sorted(candidates, key=lambda item: item["score"], reverse=True)
    validation_cases = []
    if ranked_candidates:
        print(f"[辨识] 验证 {len(ranked_candidates)} 个候选 × {len(_regularization_grid())} 组正则的真实联合辨识误差...")
    for cand in ranked_candidates:
        env.reset(q0)
        env.forward()
        q_meas, qd_meas, tau_meas, _tau_joint = _simulate_identification_samples(
            env, cand["q"], cand["qd"], cand["qdd"], Config.HOME_QPOS,
        )
        for mass_lambda, com_lambda, inertia_lambda, joint_lambda in _regularization_grid():
            case = _solve_identification_case(
                (
                    f"候选验证 {cand['profile']} seed={cand['seed']} "
                    f"λm={mass_lambda:.2g} λc={com_lambda:.2g} "
                    f"λI={inertia_lambda:.2g} λj={joint_lambda:.2g}"
                ),
                backend,
                q_meas,
                qd_meas,
                cand["qdd"],
                tau_meas,
                cand["labels"],
                max(1, len(cand["t"]) // Config.PARAM_ID_MAX_SAMPLES),
                Config.HOME_QPOS,
                *(_extract_ground_truth(backend)),
                mass_prior_lambda=mass_lambda,
                com_prior_lambda=com_lambda,
                inertia_prior_lambda=inertia_lambda,
                joint_prior_lambda=joint_lambda,
            )
            case["candidate"] = cand
            validation_cases.append(case)

    if validation_cases:
        validated = sorted(validation_cases, key=_case_selection_key)
        best_case = validated[0]
        best_candidate = best_case["candidate"]
        if Config.PARAM_ID_TRAJECTORY_PROFILE_DIAGNOSTICS:
            print("[辨识] 真实误差验证矩阵Top:")
            for case in validated[:min(Config.PARAM_ID_VALIDATION_TOP_N, len(validated))]:
                cand = case["candidate"]
                summary = case["mass_summary"]
                com_summary = case["com_summary"]
                inertia_summary = case["inertia_summary"]
                sel = case["selection"]
                status = "达标" if summary["passes_5pct"] else "未达标"
                print(
                    f"  {cand['profile']:<2} seed={cand['seed']:<4} {status} "
                    f"max={summary['max_abs']:.2f}%@J{summary['max_abs_joint']} "
                    f"J7={summary['j7_abs']:.2f}% "
                    f"COM={com_summary['max_distance']:.4f}m@J{com_summary['max_distance_joint']} "
                    f"I={inertia_summary['max_component_abs']:.1f}%@J{inertia_summary['max_component_joint']} "
                    f"末端均值={summary['distal_abs_mean']:.2f}% "
                    f"trainRMS={case['prediction_error']:.4f} "
                    f"valRMS={case['validation_rms']:.4f} "
                    f"rank={case['diagnostics'].get('rank', 0):.0f} "
                    f"cond={case['diagnostics'].get('retained_condition', float('inf')):.1f} "
                    f"λm={sel['mass_prior_lambda']:.2g} "
                    f"λc={sel['com_prior_lambda']:.2g} "
                    f"λI={sel['inertia_prior_lambda']:.2g} "
                    f"λj={sel['joint_prior_lambda']:.2g}"
                )
        best = {
            **best_candidate,
            "validated_case": best_case,
        }
        print(
            f"[辨识] 选择激励 {best['profile']} ({best['description']}) seed={best['seed']}, "
            f"验证最大误差={best_case['mass_summary']['max_abs']:.2f}% "
            f"(关节 J{best_case['mass_summary']['max_abs_joint']}), "
            f"COM={best_case['com_summary']['max_distance']:.4f} m, "
            f"惯量={best_case['inertia_summary']['max_component_abs']:.1f}%, "
            f"J7={best_case['mass_summary']['j7_abs']:.2f}%, "
            f"验证RMS={best_case['validation_rms']:.4f}"
        )
        return (
            best["t"], best["q"], best["qd"], best["qdd"], best["max_ee_speed"],
            best["speed_scale"], best["overall"], best["distal"], best["labels"],
            {
                "profile": best["profile"],
                "description": best["description"],
                "seed": best["seed"],
                "score": best["score"],
            },
        )

    best = ranked_candidates[0] if ranked_candidates else None
    if Config.PARAM_ID_TRAJECTORY_PROFILE_DIAGNOSTICS:
        print("[辨识] 激励候选Top:")
        for cand in ranked_candidates[:min(5, len(ranked_candidates))]:
            iproj = cand["inertial_distal"]["projection"]
            jproj = cand["distal"]["projection"]
            cproj = cand["group_observability"]["com"]["projection"]
            iparam_proj = cand["group_observability"]["inertia"]["projection"]
            print(
                f"  {cand['profile']:<3} seed={cand['seed']:<4} score={cand['score']:.2f} "
                f"惯性rank={cand['inertial_overall']['rank']} 惯性末端cond={cand['inertial_distal']['condition']:.1f} "
                f"残差={iproj['ratio']:.3f}/{iproj['rank']} 联合残差={jproj['ratio']:.3f}/{jproj['rank']} "
                f"COM残差={cproj['ratio']:.3f}/{cproj['rank']} 惯量残差={iparam_proj['ratio']:.3f}/{iparam_proj['rank']} "
                f"相关={cand['inertial_distal']['correlation']:.3f} TCP={cand['max_ee_speed']:.3f} "
                f"缩放={cand['speed_scale']:.3f} 覆盖={cand['coverage']['mean']:.3f}"
            )

    print(
        f"[辨识] 选择激励 {best['profile']} ({best['description']}) seed={best['seed']}, "
        f"惯性回归条件数={best['inertial_overall']['condition']:.1f}, "
        f"rank={best['inertial_overall']['rank']}, 末端rank={best['inertial_distal']['rank']}, "
        f"末端条件数={best['inertial_distal']['condition']:.1f}, "
        f"残差={best['inertial_distal']['projection']['ratio']:.3f}, "
        f"末端相关={best['inertial_distal']['correlation']:.3f}"
    )
    return (
        best["t"], best["q"], best["qd"], best["qdd"], best["max_ee_speed"],
        best["speed_scale"], best["overall"], best["distal"], best["labels"],
        {
            "profile": best["profile"],
            "description": best["description"],
            "seed": best["seed"],
            "score": best["score"],
        },
    )


def _mass_error_summary(masses, true_masses):
    errors = []
    for mass, true_mass in zip(masses, true_masses):
        if true_mass > 1e-9:
            errors.append((float(mass) - float(true_mass)) / float(true_mass) * 100.0)
        else:
            errors.append(0.0)
    abs_errors = [abs(err) for err in errors]
    distal_start = Config.PARAM_ID_DISTAL_LINK_START - 1
    distal_abs = abs_errors[distal_start:]
    max_abs = float(np.max(abs_errors)) if abs_errors else 0.0
    max_idx = int(np.argmax(abs_errors)) if abs_errors else 0
    target = float(Config.PARAM_ID_MASS_ERROR_TARGET_PCT)
    return {
        "errors": errors,
        "abs_errors": abs_errors,
        "max_abs": max_abs,
        "max_abs_joint": max_idx + 1,
        "passes_5pct": max_abs <= target,
        "target_pct": target,
        "j7_abs": abs(errors[6]) if len(errors) >= 7 else 0.0,
        "distal_abs_mean": float(np.mean(distal_abs)) if distal_abs else 0.0,
    }


def _com_error_summary(coms, true_coms):
    com_arr = np.asarray(coms, dtype=np.float64)
    true_arr = np.asarray(true_coms, dtype=np.float64)
    if com_arr.shape != true_arr.shape:
        raise ValueError("coms and true_coms must have the same shape")

    error_vectors = com_arr - true_arr
    distance_errors = np.linalg.norm(error_vectors, axis=1)
    distal_start = Config.PARAM_ID_DISTAL_LINK_START - 1
    distal_distances = distance_errors[distal_start:]
    max_idx = int(np.argmax(distance_errors)) if distance_errors.size else 0
    target_m = float(getattr(Config, "PARAM_ID_COM_ERROR_TARGET_M", 0.01))
    return {
        "error_vectors": error_vectors.tolist(),
        "distance_errors": distance_errors.tolist(),
        "max_distance": float(np.max(distance_errors)) if distance_errors.size else 0.0,
        "max_distance_joint": max_idx + 1,
        "distal_distance_mean": float(np.mean(distal_distances)) if distal_distances.size else 0.0,
        "target_m": target_m,
        "passes_target": bool(np.max(distance_errors) <= target_m) if distance_errors.size else True,
    }


def _inertia_error_summary(inertias, true_inertias):
    inertia_arr = np.asarray(inertias, dtype=np.float64)
    true_arr = np.asarray(true_inertias, dtype=np.float64)
    if inertia_arr.shape != true_arr.shape:
        raise ValueError("inertias and true_inertias must have the same shape")

    rel = np.zeros_like(inertia_arr, dtype=np.float64)
    valid = np.abs(true_arr) > 1e-9
    rel[valid] = (inertia_arr[valid] - true_arr[valid]) / true_arr[valid] * 100.0
    abs_rel = np.abs(rel)
    link_l2 = np.linalg.norm(abs_rel, axis=1)
    distal_start = Config.PARAM_ID_DISTAL_LINK_START - 1
    distal_l2 = link_l2[distal_start:]
    max_flat = int(np.argmax(abs_rel)) if abs_rel.size else 0
    max_joint, max_axis = divmod(max_flat, 3) if abs_rel.size else (0, 0)
    axis_names = ("Ixx", "Iyy", "Izz")
    target_pct = float(getattr(Config, "PARAM_ID_INERTIA_ERROR_TARGET_PCT", 15.0))
    return {
        "relative_errors": rel.tolist(),
        "absolute_relative_errors": abs_rel.tolist(),
        "link_l2_errors": link_l2.tolist(),
        "max_component_abs": float(np.max(abs_rel)) if abs_rel.size else 0.0,
        "max_component_joint": int(max_joint) + 1,
        "max_component_axis": axis_names[int(max_axis)],
        "max_link_l2": float(np.max(link_l2)) if link_l2.size else 0.0,
        "max_link_l2_joint": int(np.argmax(link_l2)) + 1 if link_l2.size else 1,
        "distal_l2_mean": float(np.mean(distal_l2)) if distal_l2.size else 0.0,
        "target_pct": target_pct,
        "passes_target": bool(np.max(abs_rel) <= target_pct) if abs_rel.size else True,
    }


def _case_selection_key(case):
    summary = case["mass_summary"]
    com_summary = case["com_summary"]
    inertia_summary = case["inertia_summary"]
    diagnostics = case.get("diagnostics", {})
    num_params = diagnostics.get("num_params", 0.0)
    rank = diagnostics.get("rank", 0.0)
    rank_failure = num_params >= 69 and rank < 69
    mass_norm = summary["max_abs"] / max(summary["target_pct"], 1e-12)
    com_norm = com_summary["max_distance"] / max(com_summary["target_m"], 1e-12)
    inertia_norm = inertia_summary["max_component_abs"] / max(inertia_summary["target_pct"], 1e-12)
    distal_com_norm = com_summary["distal_distance_mean"] / max(com_summary["target_m"], 1e-12)
    distal_inertia_norm = inertia_summary["distal_l2_mean"] / max(inertia_summary["target_pct"], 1e-12)
    return (
        summary["max_abs"] > summary["target_pct"],
        not com_summary["passes_target"],
        not inertia_summary["passes_target"],
        mass_norm,
        com_norm,
        inertia_norm,
        distal_com_norm,
        distal_inertia_norm,
        inertia_summary["max_link_l2"] / max(inertia_summary["target_pct"], 1e-12),
        summary["j7_abs"] / max(summary["target_pct"], 1e-12),
        summary["distal_abs_mean"] / max(summary["target_pct"], 1e-12),
        case["prediction_error"],
        case.get("validation_rms", float("inf")),
        case.get("validation_ratio", float("inf")),
        rank_failure,
        -rank,
        -diagnostics.get("data_rank", 0),
        -case["inertial_distal"]["projection"]["ratio"],
    )


def _j7_column_diagnostics(Y):
    Y = np.asarray(Y, dtype=np.float64)
    if Y.size == 0 or Y.shape[1] < 49:
        return {"mass_norm": 0.0, "mean_norm": 0.0, "max_norm": 0.0, "min_norm": 0.0}
    cols = np.arange(42, 49)
    norms = np.linalg.norm(Y[:, cols], axis=0)
    return {
        "mass_norm": float(norms[0]),
        "mean_norm": float(np.mean(norms)),
        "max_norm": float(np.max(norms)),
        "min_norm": float(np.min(norms)),
    }


def _segment_indices(labels, tag):
    labels = np.asarray(labels, dtype=object)
    if labels.size == 0:
        return np.array([], dtype=np.int64)
    return np.flatnonzero(labels == tag)


def _segment_prediction_rms(Y_stack, tau_stack, result, param_names, row_indices):
    if row_indices.size == 0:
        return float("nan")
    Y_sel = Y_stack[row_indices, :]
    tau_sel = tau_stack[row_indices]
    return compute_prediction_error(Y_sel, tau_sel, result, param_names)


def _stratified_validation_rows(row_labels, rows, fraction=0.2):
    row_labels = np.asarray(row_labels, dtype=object)
    if row_labels.size != rows or rows < 14:
        cut = int(rows * (1.0 - fraction))
        if cut <= 0 or cut >= rows:
            return np.array([], dtype=np.int64)
        return np.arange(cut, rows, dtype=np.int64)

    selected = []
    for label in sorted(set(row_labels.tolist()), key=str):
        label_rows = np.flatnonzero(row_labels == label)
        if label_rows.size == 0:
            continue
        count = max(1, int(np.ceil(label_rows.size * fraction)))
        selected.extend(label_rows[-count:].tolist())
    return np.asarray(sorted(set(selected)), dtype=np.int64)


def _validation_rms(Y_stack, tau_stack, result, param_names, row_labels=None):
    rows = Y_stack.shape[0]
    val_rows = _stratified_validation_rows(row_labels, rows) if row_labels is not None else _stratified_validation_rows([], rows)
    if val_rows.size == 0:
        return float("nan")
    Y_val = Y_stack[val_rows, :]
    tau_val = tau_stack[val_rows]
    return compute_prediction_error(Y_val, tau_val, result, param_names)


def _solve_identification_case(
    name,
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
    inertial_prior_lambda=None,
    mass_prior_lambda=None,
    com_prior_lambda=None,
    inertia_prior_lambda=None,
    joint_prior_lambda=None,
    rcond=None,
):
    Y_stack, param_names = build_stacked_regressor(
        backend,
        q_meas,
        qd_meas,
        qdd_traj,
        stride=stride,
        include_joint_terms=True,
        q_ref=q_ref,
        coulomb_eps=Config.PARAM_ID_COULOMB_EPS,
    )
    tau_stack = tau_meas[::stride, :].ravel()

    labels = np.asarray(trajectory_labels[::stride], dtype=object)
    if labels.size:
        row_labels = np.repeat(labels, 7)
    else:
        row_labels = np.array([], dtype=object)

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
        inertial_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIAL if inertial_prior_lambda is None else inertial_prior_lambda,
        mass_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_MASS if mass_prior_lambda is None else mass_prior_lambda,
        com_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_COM if com_prior_lambda is None else com_prior_lambda,
        inertia_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_INERTIA if inertia_prior_lambda is None else inertia_prior_lambda,
        joint_prior_lambda=Config.PARAM_ID_PRIOR_LAMBDA_JOINT if joint_prior_lambda is None else joint_prior_lambda,
        rcond=Config.PARAM_ID_RCOND if rcond is None else rcond,
        ridge=Config.PARAM_ID_RIDGE,
    )
    masses, coms, inertias = to_link_params(result, prior=prior)
    inertial_Y, _ = build_stacked_regressor(
        backend,
        q_meas,
        qd_meas,
        qdd_traj,
        stride=stride,
        include_joint_terms=False,
    )
    diagnostics = dict(get_last_diagnostics())
    mass_summary = _mass_error_summary(masses, true_masses)
    com_summary = _com_error_summary(coms, true_coms)
    inertia_summary = _inertia_error_summary(inertias, true_inertias)
    joint_term_summary = _joint_term_error_summary(result, q_meas, qd_meas, q_ref)
    dynamic_rows = _segment_indices(row_labels, "dynamic")
    j67_rows = _segment_indices(row_labels, "j6j7")
    j7_rows = _segment_indices(row_labels, "j7")
    gravity_rows = _segment_indices(row_labels, "gravity")
    com_gravity_rows = _segment_indices(row_labels, "com_gravity")
    inertia_rows = _segment_indices(row_labels, "inertia")
    segment_rms = {
        "dynamic": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, dynamic_rows),
        "j6j7": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, j67_rows),
        "j7": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, j7_rows),
        "gravity": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, gravity_rows),
        "com_gravity": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, com_gravity_rows),
        "inertia": _segment_prediction_rms(Y_stack, tau_stack, result, param_names, inertia_rows),
    }
    train_rms = compute_prediction_error(Y_stack, tau_stack, result, param_names)
    validation_rms = _validation_rms(Y_stack, tau_stack, result, param_names, row_labels=row_labels)
    validation_ratio = validation_rms / max(train_rms, 1e-12) if np.isfinite(validation_rms) else float("nan")
    return {
        "name": name,
        "include_joint_terms": True,
        "Y_stack": Y_stack,
        "param_names": param_names,
        "tau_stack": tau_stack,
        "result": result,
        "masses": masses,
        "coms": coms,
        "inertias": inertias,
        "condition": compute_condition_number(Y_stack),
        "prediction_error": train_rms,
        "validation_rms": validation_rms,
        "validation_ratio": validation_ratio,
        "segment_rms": segment_rms,
        "diagnostics": diagnostics,
        "inertial_metrics": _scaled_svd_metrics(inertial_Y),
        "distal": _distal_observability(Y_stack, include_joint_terms=True),
        "inertial_distal": _distal_observability(inertial_Y, include_joint_terms=False),
        "group_observability": _parameter_group_observability(Y_stack),
        "j7_columns": _j7_column_diagnostics(inertial_Y),
        "mass_summary": mass_summary,
        "com_summary": com_summary,
        "inertia_summary": inertia_summary,
        "joint_term_error_summary": joint_term_summary,
        "selection": {
            "inertial_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_INERTIAL if inertial_prior_lambda is None else inertial_prior_lambda,
            "mass_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_MASS if mass_prior_lambda is None else mass_prior_lambda,
            "com_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_COM if com_prior_lambda is None else com_prior_lambda,
            "inertia_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_INERTIA if inertia_prior_lambda is None else inertia_prior_lambda,
            "joint_prior_lambda": Config.PARAM_ID_PRIOR_LAMBDA_JOINT if joint_prior_lambda is None else joint_prior_lambda,
            "rcond": Config.PARAM_ID_RCOND if rcond is None else rcond,
        },
    }


def _regularization_grid():
    if not Config.PARAM_ID_REG_SWEEP:
        return [(
            Config.PARAM_ID_PRIOR_LAMBDA_MASS,
            Config.PARAM_ID_PRIOR_LAMBDA_COM,
            Config.PARAM_ID_PRIOR_LAMBDA_INERTIA,
            Config.PARAM_ID_PRIOR_LAMBDA_JOINT,
        )]
    return [
        (32.0, 1.20, 2.40, 0.035),
        (64.0, 1.20, 2.40, 0.035),
        (16.0, 1.20, 2.40, 0.035),
        (48.0, 1.20, 2.40, 0.035),
        (32.0, 0.80, 2.40, 0.035),
        (32.0, 1.60, 2.40, 0.035),
        (64.0, 1.60, 3.20, 0.035),
        (32.0, 1.20, 3.20, 0.050),
    ]


def _best_regularized_case(
    name,
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
):
    best_case = None
    for mass_lambda, com_lambda, inertia_lambda, joint_lambda in _regularization_grid():
        case = _solve_identification_case(
            name,
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
            mass_prior_lambda=mass_lambda,
            com_prior_lambda=com_lambda,
            inertia_prior_lambda=inertia_lambda,
            joint_prior_lambda=joint_lambda,
        )
        if best_case is None or _case_selection_key(case) < _case_selection_key(best_case):
            best_case = case
    return best_case


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
