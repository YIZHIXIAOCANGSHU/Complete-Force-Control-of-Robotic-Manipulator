#!/usr/bin/env python3
"""参数辨识 — 仿真模式 (MuJoCo + Pinocchio + Rerun)

在 MuJoCo 中执行 Fourier 激励轨迹，同时弹出 MuJoCo 窗口实时显示机械臂运动，
通过 Rerun 记录轨迹/力矩/辨识结果，终端打印中文辨识报告。
"""

from __future__ import annotations

import os
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
        blueprint = rrb.Blueprint(
            rrb.Tabs(overview, details, results, name="Param ID"),
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


def _report_styles():
    return """
body { margin: 0; font-family: Arial, "Noto Sans CJK SC", sans-serif; color: #202124; background: #f5f7fa; }
main { max-width: 1180px; margin: 0 auto; padding: 32px 24px 48px; }
header { margin-bottom: 24px; }
h1 { margin: 0 0 8px; font-size: 30px; font-weight: 700; }
h2 { margin: 28px 0 12px; font-size: 20px; }
.subtitle { margin: 0; color: #5f6368; }
.section { background: #fff; border: 1px solid #dfe3ea; border-radius: 8px; padding: 18px; margin: 16px 0; }
.grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 16px; }
.data-table { width: 100%; border-collapse: collapse; font-size: 13px; }
.data-table th, .data-table td { padding: 8px 9px; border-bottom: 1px solid #e8eaed; text-align: right; white-space: nowrap; }
.data-table th:first-child, .data-table td:first-child { text-align: left; }
.data-table th { background: #eef2f6; color: #344054; font-weight: 700; }
.notice { padding: 12px 14px; border: 1px solid #d7b46a; background: #fff6db; border-radius: 6px; color: #634500; }
.warnings { color: #7a3500; }
.table-wrap { overflow-x: auto; }
.empty { color: #6b7280; }
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
      <h2>轨迹图</h2>
      {{ chart_html | safe }}
    </section>
    <section class="section">
      <h2>诊断摘要</h2>
      <div class="table-wrap">{{ diagnostics_table | safe }}</div>
    </section>
    <section class="section">
      <h2>参数表</h2>
      <div class="table-wrap">{{ parameter_table | safe }}</div>
    </section>
    <section class="section">
      <h2>真值对比</h2>
      <div class="table-wrap">{{ comparison_table | safe }}</div>
    </section>
    <section class="section">
      <h2>激励统计</h2>
      <div class="table-wrap">{{ excitation_table | safe }}</div>
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
        warnings_html = "".join(f"<li>{escape(str(warning))}</li>" for warning in context["warnings"])
        warning_section = ""
        if warnings_html:
            warning_section = f'<section class="section warnings"><h2>Warnings</h2><ul>{warnings_html}</ul></section>'
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
    {warning_section}
    <section class="section"><h2>轨迹图</h2>{context["chart_html"]}</section>
    <section class="section"><h2>诊断摘要</h2><div class="table-wrap">{context["diagnostics_table"]}</div></section>
    <section class="section"><h2>参数表</h2><div class="table-wrap">{context["parameter_table"]}</div></section>
    <section class="section"><h2>真值对比</h2><div class="table-wrap">{context["comparison_table"]}</div></section>
    <section class="section"><h2>激励统计</h2><div class="table-wrap">{context["excitation_table"]}</div></section>
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
):
    if not getattr(Config, "PARAM_ID_ENABLE_HTML_REPORT", True):
        return None
    trajectory_metadata = dict(trajectory_metadata or {})
    warnings = list(warnings or [])
    generated_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        chart_html, chart_warnings = _make_plotly_charts(t_arr, q_meas, qd_meas, tau_meas)
        warnings.extend(chart_warnings)
        context = {
            "title": "参数辨识报告（仿真模式）",
            "subtitle": f"{case.get('name', '联合辨识结果')} · {generated_at}",
            "styles": _report_styles(),
            "warnings": warnings,
            "chart_html": chart_html,
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
        report_dir = Path(Config.RESULTS_DIR) / "param_id" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        report_dir.mkdir(parents=True, exist_ok=True)
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
    amp5 = _safe_joint_amplitude(q_start, limits, 4, 0.36)
    amp6 = _safe_joint_amplitude(q_start, limits, 5, 0.42)
    amp7 = _safe_joint_amplitude(q_start, limits, 6, 0.55)
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
        + inertial_projection["ratio"] * 220.0
        + joint_projection["ratio"] * 60.0
        + com_obs.get("rank", 0) * 8.0
        + inertia_obs.get("rank", 0) * 7.0
        + distal_com_obs.get("rank", 0) * 18.0
        + distal_inertia_obs.get("rank", 0) * 18.0
        + com_projection.get("rank", 0) * 8.0
        + inertia_projection.get("rank", 0) * 7.0
        + distal_com_projection.get("ratio", 0.0) * 120.0
        + distal_inertia_projection.get("ratio", 0.0) * 150.0
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


def _print_identification_case(case, true_masses, true_inertias):
    print()
    print("=" * 78)
    print(f"                    {case['name']}")
    print("=" * 78)
    _print_identified_params(case["masses"], case["coms"], case["inertias"])
    _print_comparison(case["masses"], true_masses, case["inertias"], true_inertias)
    _print_joint_term_comparison(case["result"])

    diagnostics = case["diagnostics"]
    final_distal = case["distal"]
    inertial_distal = case["inertial_distal"]
    inertial_metrics = case["inertial_metrics"]
    print(f"\n惯性子回归条件数: {inertial_metrics['condition']:.1f}, rank={inertial_metrics['rank']}")
    print(f"当前路径回归矩阵缩放后条件数: {case['condition']:.1f}")
    print(
        f"SVD rank: {diagnostics.get('rank', 0):.0f}/{diagnostics.get('num_params', len(case['param_names'])):.0f}, "
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
    group_observability = case.get("group_observability", {})
    for label, key in (("质量列", "mass"), ("COM列", "com"), ("惯量列", "inertia"), ("末端COM列", "distal_com"), ("末端惯量列", "distal_inertia")):
        obs = group_observability.get(key)
        if not obs:
            continue
        proj = obs["projection"]
        print(
            f"{label}可观测性: rank={obs['rank']}, condition={obs['condition']:.1f}, "
            f"相关={obs['correlation']:.3f}, 残差={proj['ratio']:.3f}/{proj['rank']}"
        )
    mass_summary = case["mass_summary"]
    com_summary = case["com_summary"]
    inertia_summary = case["inertia_summary"]
    err_text = " ".join(f"J{i + 1}:{err:+.1f}%" for i, err in enumerate(mass_summary["errors"]))
    mass_status = "达标" if mass_summary["passes_5pct"] else "未达标"
    com_status = "达标" if com_summary["passes_target"] else "未达标"
    inertia_status = "达标" if inertia_summary["passes_target"] else "未达标"
    print(f"单关节质量误差: {err_text}")
    print(
        f"最大单关节质量误差: {mass_summary['max_abs']:.2f}% "
        f"(J{mass_summary['max_abs_joint']}, 目标≤{mass_summary['target_pct']:.1f}%, {mass_status})"
    )
    com_err_text = " ".join(
        f"J{i + 1}:[{vec[0]:+.4f},{vec[1]:+.4f},{vec[2]:+.4f}]"
        for i, vec in enumerate(com_summary["error_vectors"])
    )
    print(f"单关节COM误差向量(m): {com_err_text}")
    print(
        f"最大COM距离误差: {com_summary['max_distance']:.4f} m "
        f"(J{com_summary['max_distance_joint']}, 末端均值={com_summary['distal_distance_mean']:.4f} m, "
        f"目标≤{com_summary['target_m']:.4f} m, {com_status})"
    )
    inertia_err_text = " ".join(
        f"J{i + 1}:[{vec[0]:+.1f}%,{vec[1]:+.1f}%,{vec[2]:+.1f}%]"
        for i, vec in enumerate(inertia_summary["relative_errors"])
    )
    print(f"单关节惯量相对误差(Ixx/Iyy/Izz): {inertia_err_text}")
    print(
        f"最大惯量分量误差: {inertia_summary['max_component_abs']:.2f}% "
        f"(J{inertia_summary['max_component_joint']}-{inertia_summary['max_component_axis']}), "
        f"最大链节L2={inertia_summary['max_link_l2']:.2f}%@J{inertia_summary['max_link_l2_joint']}, "
        f"末端L2均值={inertia_summary['distal_l2_mean']:.2f}%, "
        f"目标≤{inertia_summary['target_pct']:.1f}%, {inertia_status}"
    )
    j7_columns = case.get("j7_columns", {})
    print(
        f"J7专项: 质量误差={mass_summary['j7_abs']:.2f}%, "
        f"列范数 mass={j7_columns.get('mass_norm', 0.0):.3e}, "
        f"mean={j7_columns.get('mean_norm', 0.0):.3e}, "
        f"min={j7_columns.get('min_norm', 0.0):.3e}, "
        f"目标≤4.0%={'是' if mass_summary['j7_abs'] <= 4.0 else '否'}"
    )
    sel = case.get("selection", {})
    if sel:
        print(
            f"正则化参数: λ_mass={sel.get('mass_prior_lambda', 0.0):.3g}, "
            f"λ_com={sel.get('com_prior_lambda', 0.0):.3g}, "
            f"λ_inertia={sel.get('inertia_prior_lambda', 0.0):.3g}, "
            f"λ_joint={sel.get('joint_prior_lambda', 0.0):.3g}, rcond={sel.get('rcond', 0.0):.1e}"
        )
    print(
        f"训练/验证 RMS: {case['prediction_error']:.4f} / {case.get('validation_rms', float('nan')):.4f} N·m "
        f"(比值={case.get('validation_ratio', float('nan')):.3f})"
    )
    seg = case.get("segment_rms", {})
    print(
        f"分段RMS: dynamic={seg.get('dynamic', float('nan')):.4f}, "
        f"j6j7={seg.get('j6j7', float('nan')):.4f}, "
        f"j7={seg.get('j7', float('nan')):.4f}, "
        f"gravity={seg.get('gravity', float('nan')):.4f}, "
        f"com_gravity={seg.get('com_gravity', float('nan')):.4f}, "
        f"inertia={seg.get('inertia', float('nan')):.4f}"
    )
    print(f"先验偏离 RMS: {diagnostics.get('prior_delta_rms', 0.0):.6f}")
    print(f"惯性先验偏离 RMS: {diagnostics.get('inertial_prior_delta_rms', 0.0):.6f}")
    print(f"质量先验偏离 RMS: {diagnostics.get('mass_prior_delta_rms', 0.0):.6f}")
    print(f"COM先验偏离 RMS: {diagnostics.get('com_prior_delta_rms', 0.0):.6f}")
    print(f"惯量先验偏离 RMS: {diagnostics.get('inertia_prior_delta_rms', 0.0):.6f}")
    print(f"关节项先验偏离 RMS: {diagnostics.get('joint_prior_delta_rms', 0.0):.6f}")
    print(
        f"综合结论: mass={'通过' if mass_summary['passes_5pct'] else '未通过'}, "
        f"COM={'通过' if com_summary['passes_target'] else '未通过'}, "
        f"inertia={'通过' if inertia_summary['passes_target'] else '未通过'}"
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

    # ---- Rerun ----
    rerun_ok = _setup_rerun()

    # ---- MuJoCo 窗口 + 轨迹执行 ----
    print("[辨识] 启动 MuJoCo 窗口，执行激励轨迹...")
    q_ref = Config.HOME_QPOS.copy()

    q_meas, qd_meas, tau_meas, _tau_joint = _simulate_identification_samples(
        env, q_traj, qd_traj, qdd_traj, q_ref,
    )

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
            env.forward()

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
    _print_identification_case(identified_case, true_masses, true_inertias)
    report_metadata = {
        **trajectory_metadata,
        "stride": stride,
        "rerun_log_stride": Config.RERUN_LOG_STRIDE,
        "max_ee_speed": max_ee_speed,
        "speed_scale": speed_scale,
    }
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
