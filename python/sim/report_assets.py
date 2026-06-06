"""Report formatting and asset writers for sim-only analysis."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path

import numpy as np

from config import Config
from sim.workspace import (
    DEFAULT_MC_MAX_HULL_POINTS,
    InternalWorkspaceBox,
    MonteCarloWorkspaceResult,
    RangeSnapshot,
    compute_internal_workspace_box_intersection,
)


DEFAULT_REPORT_SAMPLES = 500000
DEFAULT_REPORT_PROGRESS_INTERVAL = 10000
DEFAULT_REPORT_CONTROL_DURATION_S = 10.0
DEFAULT_REPORT_CONTROL_LOG_STRIDE = 10
DEFAULT_REPORT_MAX_SVG_POINTS = 1800


@dataclass(frozen=True)
class ControlTargetSegment:
    index: int
    label: str
    start_s: float
    end_s: float
    target_pos_base: np.ndarray
    target_quat_base: np.ndarray


@dataclass(frozen=True)
class ControlLoopResult:
    duration_s: float
    dt_s: float
    steps: int
    log_stride: int
    schedule: list[ControlTargetSegment]
    telemetry_rows: list[dict[str, object]]
    summary: dict[str, object]
    error_history: np.ndarray
    tau_history: np.ndarray
    time_history: np.ndarray


def _format_vector(values: np.ndarray, precision: int = 6) -> str:
    return "[" + ", ".join(f"{value:.{precision}f}" for value in values) + "]"


def _format_range_block(labels: list[str], snapshot: RangeSnapshot, unit: str = "") -> str:
    suffix = f" {unit}" if unit else ""
    lines = ["名称        最小值        最大值        跨度          均值"]
    for label, lo, hi, span, mean in zip(
        labels,
        snapshot.minimum,
        snapshot.maximum,
        snapshot.span,
        snapshot.mean,
    ):
        lines.append(f"{label:<5} {lo:>12.6f} {hi:>12.6f} {span:>12.6f} {mean:>12.6f}{suffix}")
    return "\n".join(lines)


def _format_internal_workspace_box(box: InternalWorkspaceBox, hull_point_count: int) -> str:
    if box.is_empty:
        return "\n".join(
            [
                "最大内部轴对齐长方体：",
                "不可用（工作空间凸包为空或退化）",
            ]
        )

    lines = [
        "最大内部轴对齐长方体（安全 pos 输入范围）：",
        f"参与凸包计算的采样点数：{hull_point_count}",
        f"中心位置(m)：{_format_vector(box.center)}",
        f"半边长(m)：{_format_vector(box.half_size)}",
        f"体积(m^3)：{box.volume:.9f}",
        "轴          最小值        最大值        跨度          中心值",
    ]
    for label, lo, hi, span, center in zip(
        ["x", "y", "z"],
        box.lower,
        box.upper,
        2.0 * box.half_size,
        box.center,
    ):
        lines.append(f"{label:<5} {lo:>12.6f} {hi:>12.6f} {span:>12.6f} {center:>12.6f} m")
    return "\n".join(lines)


def _format_safe_box_summary_block(
    label: str,
    box: InternalWorkspaceBox,
) -> list[str]:
    lines = [f"{label}："]
    if box.is_empty:
        lines.append("  不可用（内部安全盒为空或左右交集为空）")
        return lines
    lines.append("  轴          最小值        最大值        跨度          中心值")
    for axis, lo, hi, span, center in zip(
        ("x", "y", "z"),
        box.lower,
        box.upper,
        2.0 * box.half_size,
        box.center,
    ):
        lines.append(f"  {axis:<5} {lo:>12.6f} {hi:>12.6f} {span:>12.6f} {center:>12.6f}")
    lines.append(f"  体积(m^3)：{box.volume:.9f}")
    return lines


def _format_safe_box_summary(internal_boxes: list[InternalWorkspaceBox]) -> str:
    common_box = compute_internal_workspace_box_intersection(internal_boxes)
    lines = ["安全盒范围汇总(m)："]
    for label, box in zip(("左臂", "右臂"), internal_boxes):
        lines.extend(_format_safe_box_summary_block(label, box))
    lines.extend(_format_safe_box_summary_block("公共交集", common_box))
    return "\n".join(lines)


def _format_monte_carlo_report(
    *,
    samples: int,
    seed: int | None,
    joint_lower: np.ndarray,
    joint_upper: np.ndarray,
    pos_stats: RangeSnapshot,
    quat_stats: RangeSnapshot,
    quat_norm_stats: RangeSnapshot,
    internal_box: InternalWorkspaceBox,
    hull_point_count: int,
    last_qpos: np.ndarray,
) -> str:
    seed_text = "随机" if seed is None else str(seed)
    return "\n".join(
        [
            "",
            "=" * 72,
            "AM-DPBSURDF0422 蒙特卡洛末端位姿范围检查",
            "=" * 72,
            f"采样数量：{samples}",
            f"随机种子：{seed_text}",
            f"关节下限(rad)：{_format_vector(joint_lower)}",
            f"关节上限(rad)：{_format_vector(joint_upper)}",
            "",
            "末端位置范围：",
            _format_range_block(["x", "y", "z"], pos_stats, "m"),
            "",
            _format_internal_workspace_box(internal_box, hull_point_count),
            "",
            "末端四元数范围 [w, x, y, z]：",
            _format_range_block(["w", "x", "y", "z"], quat_stats),
            "",
            "四元数范数范围：",
            _format_range_block(["|q|"], quat_norm_stats),
            "",
            "最后刷新样本：",
            f"关节角 qpos(rad)：{_format_vector(last_qpos)}",
            f"末端位置 ee_pos(m)：{_format_vector(pos_stats.last)}",
            f"末端四元数 ee_quat(wxyz)：{_format_vector(quat_stats.last)}",
            "=" * 72,
        ]
    )


def _format_dual_monte_carlo_report(
    *,
    samples: int,
    seed: int | None,
    joint_lower: np.ndarray,
    joint_upper: np.ndarray,
    pos_stats: list[RangeSnapshot],
    quat_stats: list[RangeSnapshot],
    quat_norm_stats: list[RangeSnapshot],
    internal_boxes: list[InternalWorkspaceBox],
    hull_point_count: int,
    last_qpos: np.ndarray,
) -> str:
    seed_text = "随机" if seed is None else str(seed)
    lines = [
        "",
        "=" * 72,
        "AM-DPBSURDF0422 双臂蒙特卡洛末端位姿范围检查",
        "=" * 72,
        f"采样数量：{samples}",
        f"随机种子：{seed_text}",
        f"关节下限(rad)：{_format_vector(joint_lower)}",
        f"关节上限(rad)：{_format_vector(joint_upper)}",
        "",
        _format_safe_box_summary(internal_boxes),
    ]
    for arm, label in enumerate(("左臂", "右臂")):
        lines.extend(
            [
                "",
                f"{label}末端位置范围：",
                _format_range_block(["x", "y", "z"], pos_stats[arm], "m"),
                "",
                f"{label}" + _format_internal_workspace_box(internal_boxes[arm], hull_point_count),
                "",
                f"{label}末端四元数范围 [w, x, y, z]：",
                _format_range_block(["w", "x", "y", "z"], quat_stats[arm]),
                "",
                f"{label}四元数范数范围：",
                _format_range_block(["|q|"], quat_norm_stats[arm]),
            ]
        )
    lines.extend(
        [
            "",
            "最后刷新样本：",
            f"关节角 qpos(rad)：{_format_vector(last_qpos)}",
            f"左臂末端位置 ee_pos(m)：{_format_vector(pos_stats[Config.LEFT_ARM].last)}",
            f"右臂末端位置 ee_pos(m)：{_format_vector(pos_stats[Config.RIGHT_ARM].last)}",
            f"左臂末端四元数 ee_quat(wxyz)：{_format_vector(quat_stats[Config.LEFT_ARM].last)}",
            f"右臂末端四元数 ee_quat(wxyz)：{_format_vector(quat_stats[Config.RIGHT_ARM].last)}",
            "=" * 72,
        ]
    )
    return "\n".join(lines)


def _array_to_list(values: np.ndarray) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=np.float64).tolist()]


def _snapshot_to_dict(snapshot: RangeSnapshot) -> dict[str, object]:
    return {
        "count": int(snapshot.count),
        "minimum": _array_to_list(snapshot.minimum),
        "maximum": _array_to_list(snapshot.maximum),
        "span": _array_to_list(snapshot.span),
        "mean": _array_to_list(snapshot.mean),
        "last": _array_to_list(snapshot.last),
    }


def _internal_box_to_dict(box: InternalWorkspaceBox) -> dict[str, object]:
    return {
        "available": not box.is_empty,
        "center": _array_to_list(box.center),
        "half_size": _array_to_list(box.half_size),
        "lower": _array_to_list(box.lower),
        "upper": _array_to_list(box.upper),
        "volume": float(box.volume),
    }


def _write_monte_carlo_report_assets(
    output_dir: str | Path,
    *,
    samples: int,
    seed: int | None,
    joint_lower: np.ndarray,
    joint_upper: np.ndarray,
    points: list[np.ndarray],
    quats: list[np.ndarray],
    pos_stats: list[RangeSnapshot],
    quat_stats: list[RangeSnapshot],
    quat_norm_stats: list[RangeSnapshot],
    internal_boxes: list[InternalWorkspaceBox],
    hull_point_count: int,
    report_text: str,
    metadata: dict[str, object] | None = None,
    extra_files: dict[str, str] | None = None,
) -> None:
    """Write report-facing Monte Carlo artifacts without changing sampling."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    points_path = out / "workspace_points.csv"
    with points_path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = ["sample", "arm", "x", "y", "z", "qx", "qy", "qz", "qw"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        arm_labels = ("left", "right")
        row_count = min(len(points[Config.LEFT_ARM]), len(points[Config.RIGHT_ARM]))
        for sample_index in range(row_count):
            for arm_index, arm_label in enumerate(arm_labels):
                pos = np.asarray(points[arm_index][sample_index], dtype=np.float64)
                quat = np.asarray(quats[arm_index][sample_index], dtype=np.float64)
                writer.writerow(
                    {
                        "sample": sample_index,
                        "arm": arm_label,
                        "x": f"{pos[0]:.9f}",
                        "y": f"{pos[1]:.9f}",
                        "z": f"{pos[2]:.9f}",
                        "qx": f"{quat[1]:.9f}",
                        "qy": f"{quat[2]:.9f}",
                        "qz": f"{quat[3]:.9f}",
                        "qw": f"{quat[0]:.9f}",
                    }
                )

    summary = {
        "samples": int(samples),
        "seed": None if seed is None else int(seed),
        "joint_lower_rad": _array_to_list(joint_lower),
        "joint_upper_rad": _array_to_list(joint_upper),
        "hull_point_count": int(hull_point_count),
        "arms": {
            "left": {
                "position": _snapshot_to_dict(pos_stats[Config.LEFT_ARM]),
                "quaternion_wxyz": _snapshot_to_dict(quat_stats[Config.LEFT_ARM]),
                "quaternion_norm": _snapshot_to_dict(quat_norm_stats[Config.LEFT_ARM]),
                "internal_workspace_box": _internal_box_to_dict(internal_boxes[Config.LEFT_ARM]),
            },
            "right": {
                "position": _snapshot_to_dict(pos_stats[Config.RIGHT_ARM]),
                "quaternion_wxyz": _snapshot_to_dict(quat_stats[Config.RIGHT_ARM]),
                "quaternion_norm": _snapshot_to_dict(quat_norm_stats[Config.RIGHT_ARM]),
                "internal_workspace_box": _internal_box_to_dict(internal_boxes[Config.RIGHT_ARM]),
            },
        },
        "files": {
            "workspace_points_csv": str(points_path),
            "terminal_summary_txt": str(out / "mc_terminal_summary.txt"),
        },
    }
    if metadata is not None:
        summary["metadata"] = metadata
    if extra_files is not None:
        summary["files"].update(extra_files)
    (out / "workspace_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    (out / "mc_terminal_summary.txt").write_text(report_text + "\n", encoding="utf-8")
    print(f"[MC] 报告数据已保存: {out}")


def _default_report_output_dir() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(Config.RESULTS_DIR) / f"sim_report_{timestamp}"


def _model_metadata() -> dict[str, object]:
    return {
        "label": "sim-only MuJoCo/C-controller data",
        "urdf_path": Config.URDF_PATH,
        "dt_s": float(Config.DT),
        "num_arms": int(Config.NUM_ARMS),
        "num_joints": int(Config.NUM_JOINTS),
        "arm_names": list(Config.ARM_NAMES),
        "joint_names": list(Config.JOINT_NAMES),
        "body_joint_names": list(Config.BODY_JOINT_NAMES),
        "torque_limits_nm": _array_to_list(Config.TORQUE_LIMITS),
        "joint_velocity_limit_rad_s": float(Config.JOINT_VEL_LIMIT),
        "enable_follower_friction": bool(Config.ENABLE_FOLLOWER_FRICTION),
        "end_effector_linear_speed_mps": float(Config.END_EFFECTOR_LINEAR_SPEED_MPS),
        "end_effector_real_speed_limit_mps": float(Config.END_EFFECTOR_REAL_SPEED_LIMIT_MPS),
    }

def _build_control_target_schedule(
    home_pos_base: np.ndarray,
    home_quat_base: np.ndarray,
    *,
    duration_s: float,
) -> list[ControlTargetSegment]:
    if duration_s <= 0.0:
        raise ValueError("control_duration_s 必须为正数")
    home_pos = np.asarray(home_pos_base, dtype=np.float64).reshape(Config.NUM_ARMS, 3)
    home_quat = np.asarray(home_quat_base, dtype=np.float64).reshape(Config.NUM_ARMS, 4)
    segment_duration = float(duration_s) / 5.0
    offsets = [
        np.zeros((Config.NUM_ARMS, 3), dtype=np.float64),
        np.array([[0.020, 0.000, 0.015], [0.018, -0.012, 0.012]], dtype=np.float64),
        np.array([[-0.015, 0.018, 0.010], [-0.015, -0.018, 0.010]], dtype=np.float64),
        np.array([[0.000, -0.020, 0.018], [0.000, 0.020, 0.018]], dtype=np.float64),
        np.zeros((Config.NUM_ARMS, 3), dtype=np.float64),
    ]
    labels = ["home_hold", "step_1", "step_2", "step_3", "return_home"]
    segments: list[ControlTargetSegment] = []
    for index, (label, offset) in enumerate(zip(labels, offsets)):
        segments.append(
            ControlTargetSegment(
                index=index,
                label=label,
                start_s=index * segment_duration,
                end_s=(index + 1) * segment_duration if index < 4 else float(duration_s),
                target_pos_base=home_pos + offset,
                target_quat_base=home_quat.copy(),
            )
        )
    return segments


def _segment_for_time(schedule: list[ControlTargetSegment], t_s: float) -> ControlTargetSegment:
    for segment in schedule:
        if t_s < segment.end_s or segment.index == len(schedule) - 1:
            return segment
    return schedule[-1]


def _status_counts(status_values: np.ndarray) -> dict[str, int]:
    counts: dict[str, int] = {}
    for status in np.asarray(status_values, dtype=np.int32):
        key = str(int(status))
        counts[key] = counts.get(key, 0) + 1
    return counts


def _summarize_control_loop_arrays(
    *,
    duration_s: float,
    dt_s: float,
    log_stride: int,
    error_history: np.ndarray,
    tau_history: np.ndarray,
    status_history: np.ndarray,
) -> dict[str, object]:
    errors = np.asarray(error_history, dtype=np.float64)
    tau = np.asarray(tau_history, dtype=np.float64)
    statuses = np.asarray(status_history, dtype=np.int32)
    if errors.ndim != 2 or errors.shape[1] != Config.NUM_ARMS:
        raise ValueError(f"error_history must have shape (N, {Config.NUM_ARMS})")
    if tau.ndim != 2 or tau.shape[1] != Config.NUM_JOINTS:
        raise ValueError(f"tau_history must have shape (N, {Config.NUM_JOINTS})")
    steps = int(errors.shape[0])
    steady_steps = max(1, min(steps, int(round(min(1.0, max(duration_s, dt_s)) / dt_s))))
    arms: dict[str, object] = {}
    for arm_index, arm_label in enumerate(Config.ARM_NAMES):
        arm_slice = slice(arm_index * Config.ARM_JOINTS, (arm_index + 1) * Config.ARM_JOINTS)
        arm_errors = errors[:, arm_index]
        arm_tau = tau[:, arm_slice]
        arms[arm_label] = {
            "max_error_m": float(np.max(arm_errors)),
            "mean_error_m": float(np.mean(arm_errors)),
            "final_error_m": float(arm_errors[-1]),
            "steady_state_error_m": float(np.mean(arm_errors[-steady_steps:])),
            "peak_abs_tau_nm": float(np.max(np.abs(arm_tau))),
            "rms_tau_nm": float(np.sqrt(np.mean(arm_tau * arm_tau))),
        }
    return {
        "sim_only": True,
        "duration_s": float(duration_s),
        "dt_s": float(dt_s),
        "steps": steps,
        "log_stride": int(log_stride),
        "status_counts": _status_counts(statuses),
        "arms": arms,
    }


def _control_csv_fieldnames() -> list[str]:
    fields = [
        "step",
        "t_s",
        "segment",
        "status",
        "traj_t",
        "left_error_m",
        "right_error_m",
        "left_speed_mps",
        "right_speed_mps",
    ]
    for arm_label in Config.ARM_NAMES:
        for prefix in ("target", "actual"):
            for axis in ("x", "y", "z"):
                fields.append(f"{arm_label}_{prefix}_{axis}_m")
    fields.extend([f"tau_{index:02d}_nm" for index in range(Config.NUM_JOINTS)])
    fields.extend([f"q_{index:02d}_rad" for index in range(Config.NUM_JOINTS)])
    fields.extend([f"qd_{index:02d}_rad_s" for index in range(Config.NUM_JOINTS)])
    return fields


def _telemetry_row(
    *,
    step: int,
    t_s: float,
    segment: ControlTargetSegment,
    status: int,
    traj_t: float,
    target_pos: np.ndarray,
    actual_pos: np.ndarray,
    errors: np.ndarray,
    speeds: np.ndarray,
    tau: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
) -> dict[str, object]:
    row: dict[str, object] = {
        "step": int(step),
        "t_s": f"{t_s:.6f}",
        "segment": segment.label,
        "status": int(status),
        "traj_t": f"{traj_t:.6f}",
        "left_error_m": f"{errors[Config.LEFT_ARM]:.9f}",
        "right_error_m": f"{errors[Config.RIGHT_ARM]:.9f}",
        "left_speed_mps": f"{speeds[Config.LEFT_ARM]:.9f}",
        "right_speed_mps": f"{speeds[Config.RIGHT_ARM]:.9f}",
    }
    for arm_index, arm_label in enumerate(Config.ARM_NAMES):
        for prefix, values in (("target", target_pos[arm_index]), ("actual", actual_pos[arm_index])):
            for axis, value in zip(("x", "y", "z"), values):
                row[f"{arm_label}_{prefix}_{axis}_m"] = f"{float(value):.9f}"
    for index, value in enumerate(np.asarray(tau, dtype=np.float64)):
        row[f"tau_{index:02d}_nm"] = f"{float(value):.9f}"
    for index, value in enumerate(np.asarray(q, dtype=np.float64)):
        row[f"q_{index:02d}_rad"] = f"{float(value):.9f}"
    for index, value in enumerate(np.asarray(qd, dtype=np.float64)):
        row[f"qd_{index:02d}_rad_s"] = f"{float(value):.9f}"
    return row


def _run_closed_loop_report_experiment(
    *,
    env,
    duration_s: float,
    log_stride: int,
) -> ControlLoopResult:
    if duration_s <= 0.0:
        raise ValueError("control_duration_s 必须为正数")
    if log_stride <= 0:
        raise ValueError("control_log_stride 必须为正数")

    from real.controller_bridge import RealControllerBridge

    dt_s = float(Config.DT)
    steps = max(1, int(round(duration_s / dt_s)))
    env.reset(Config.HOME_QPOS)
    env.forward()
    home_pos = env.get_all_ee_pos()
    home_quat = env.get_all_ee_quat()
    schedule = _build_control_target_schedule(home_pos, home_quat, duration_s=duration_s)

    error_history = np.zeros((steps, Config.NUM_ARMS), dtype=np.float64)
    tau_history = np.zeros((steps, Config.NUM_JOINTS), dtype=np.float64)
    status_history = np.zeros(steps, dtype=np.int32)
    time_history = np.zeros(steps, dtype=np.float64)
    telemetry_rows: list[dict[str, object]] = []

    bridge = RealControllerBridge(exchange_timeout_s=1.0)
    try:
        for step in range(steps):
            t_s = step * dt_s
            segment = _segment_for_time(schedule, t_s)
            env.set_all_target_poses_base(segment.target_pos_base, segment.target_quat_base)
            target_pos_body, target_quat_body = env.get_all_target_poses()

            q = env.get_qpos()
            qd = env.get_qvel()
            output = bridge.compute(
                active_arm_mask=(1 << Config.LEFT_ARM) | (1 << Config.RIGHT_ARM),
                elapsed_s=dt_s,
                q=q,
                qd=qd,
                body_q=env.get_body_qpos(),
                target_pos=target_pos_body,
                target_quat=target_quat_body,
            )
            clipped_tau = env.clip_torque(output.tau)
            env.apply_torque(clipped_tau)
            env.step()
            if env.enforce_joint_limits():
                env.forward()

            actual_pos = env.get_all_ee_pos()
            actual_twist = env.get_all_ee_twist()
            errors = np.linalg.norm(segment.target_pos_base - actual_pos, axis=1)
            speeds = np.linalg.norm(actual_twist[:, :3], axis=1)
            error_history[step] = errors
            tau_history[step] = clipped_tau
            status_history[step] = int(output.status)
            time_history[step] = t_s

            if step % log_stride == 0 or step == steps - 1:
                telemetry_rows.append(
                    _telemetry_row(
                        step=step,
                        t_s=t_s,
                        segment=segment,
                        status=int(output.status),
                        traj_t=float(output.traj_t),
                        target_pos=segment.target_pos_base,
                        actual_pos=actual_pos,
                        errors=errors,
                        speeds=speeds,
                        tau=clipped_tau,
                        q=env.get_qpos(),
                        qd=env.get_qvel(),
                    )
                )
    finally:
        bridge.close()

    summary = _summarize_control_loop_arrays(
        duration_s=duration_s,
        dt_s=dt_s,
        log_stride=log_stride,
        error_history=error_history,
        tau_history=tau_history,
        status_history=status_history,
    )
    summary["schedule"] = [
        {
            "index": segment.index,
            "label": segment.label,
            "start_s": float(segment.start_s),
            "end_s": float(segment.end_s),
            "target_pos_base_m": [
                _array_to_list(segment.target_pos_base[Config.LEFT_ARM]),
                _array_to_list(segment.target_pos_base[Config.RIGHT_ARM]),
            ],
        }
        for segment in schedule
    ]
    return ControlLoopResult(
        duration_s=duration_s,
        dt_s=dt_s,
        steps=steps,
        log_stride=log_stride,
        schedule=schedule,
        telemetry_rows=telemetry_rows,
        summary=summary,
        error_history=error_history,
        tau_history=tau_history,
        time_history=time_history,
    )


def _write_control_loop_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=_control_csv_fieldnames())
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _downsample_indices(length: int, max_points: int) -> np.ndarray:
    if length <= 0:
        return np.empty(0, dtype=np.int64)
    if max_points <= 0 or length <= max_points:
        return np.arange(length, dtype=np.int64)
    return np.linspace(0, length - 1, max_points, dtype=np.int64)


def _svg_scale(
    values: np.ndarray,
    lower: float,
    upper: float,
    *,
    start: float,
    end: float,
    invert: bool = False,
) -> np.ndarray:
    span = max(float(upper - lower), 1e-12)
    scaled = start + (np.asarray(values, dtype=np.float64) - lower) / span * (end - start)
    return start + end - scaled if invert else scaled


def _write_workspace_projection_svg(
    path: Path,
    *,
    points: list[np.ndarray],
    axes: tuple[int, int],
    title: str,
    max_points: int = DEFAULT_REPORT_MAX_SVG_POINTS,
) -> None:
    width, height, margin = 760, 520, 58
    axis_names = ("x", "y", "z")
    selected = []
    for arm_points in points:
        indices = _downsample_indices(len(arm_points), max_points)
        selected.append(np.asarray(arm_points, dtype=np.float64)[indices][:, axes])
    combined = np.vstack(selected)
    lower = np.min(combined, axis=0)
    upper = np.max(combined, axis=0)
    padding = np.maximum((upper - lower) * 0.08, 1e-3)
    lower -= padding
    upper += padding
    colors = ("#1f77b4", "#d9480f")
    circles = []
    for arm_index, arm_points in enumerate(selected):
        xs = _svg_scale(arm_points[:, 0], lower[0], upper[0], start=margin, end=width - margin)
        ys = _svg_scale(arm_points[:, 1], lower[1], upper[1], start=margin, end=height - margin, invert=True)
        color = colors[arm_index % len(colors)]
        for x, y in zip(xs, ys):
            circles.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="1.6" fill="{color}" opacity="0.45"/>')
    path.write_text(
        "\n".join(
            [
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
                '<rect width="100%" height="100%" fill="#ffffff"/>',
                f'<text x="{width / 2:.1f}" y="28" text-anchor="middle" font-size="20" font-family="Arial">{title}</text>',
                f'<rect x="{margin}" y="{margin}" width="{width - 2 * margin}" height="{height - 2 * margin}" fill="#f8fafc" stroke="#334155"/>',
                *circles,
                f'<text x="{width / 2:.1f}" y="{height - 16}" text-anchor="middle" font-size="14" font-family="Arial">{axis_names[axes[0]]} / m</text>',
                f'<text x="18" y="{height / 2:.1f}" transform="rotate(-90 18 {height / 2:.1f})" text-anchor="middle" font-size="14" font-family="Arial">{axis_names[axes[1]]} / m</text>',
                f'<text x="{margin}" y="48" font-size="13" font-family="Arial" fill="#1f77b4">left</text>',
                f'<text x="{margin + 62}" y="48" font-size="13" font-family="Arial" fill="#d9480f">right</text>',
                "</svg>",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def _write_position_span_svg(path: Path, *, pos_stats: list[RangeSnapshot]) -> None:
    width, height, margin = 760, 420, 58
    labels = ["x", "y", "z"]
    values = np.vstack([snapshot.span for snapshot in pos_stats])
    max_value = max(float(np.max(values)), 1e-9)
    colors = ("#1f77b4", "#d9480f")
    bar_width = 54
    group_gap = 150
    bars = []
    for axis_index, axis_label in enumerate(labels):
        group_x = margin + 75 + axis_index * group_gap
        for arm_index, arm_label in enumerate(Config.ARM_NAMES):
            value = float(values[arm_index, axis_index])
            bar_h = value / max_value * (height - 2 * margin - 30)
            x = group_x + arm_index * (bar_width + 14)
            y = height - margin - bar_h
            bars.extend(
                [
                    f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_width}" height="{bar_h:.1f}" fill="{colors[arm_index]}"/>',
                    f'<text x="{x + bar_width / 2:.1f}" y="{y - 6:.1f}" text-anchor="middle" font-size="12" font-family="Arial">{value:.3f}</text>',
                    f'<text x="{x + bar_width / 2:.1f}" y="{height - margin + 18}" text-anchor="middle" font-size="12" font-family="Arial">{axis_label}-{arm_label}</text>',
                ]
            )
    path.write_text(
        "\n".join(
            [
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
                '<rect width="100%" height="100%" fill="#ffffff"/>',
                f'<text x="{width / 2:.1f}" y="30" text-anchor="middle" font-size="20" font-family="Arial">Monte Carlo TCP position span</text>',
                f'<line x1="{margin}" y1="{height - margin}" x2="{width - margin}" y2="{height - margin}" stroke="#334155"/>',
                *bars,
                f'<text x="18" y="{height / 2:.1f}" transform="rotate(-90 18 {height / 2:.1f})" text-anchor="middle" font-size="14" font-family="Arial">span / m</text>',
                "</svg>",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def _polyline_points(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    width: int,
    height: int,
    margin: int,
) -> str:
    xs = _svg_scale(x_values, x_min, x_max, start=margin, end=width - margin)
    ys = _svg_scale(y_values, y_min, y_max, start=margin, end=height - margin, invert=True)
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in zip(xs, ys))


def _write_control_error_svg(path: Path, *, control: ControlLoopResult) -> None:
    width, height, margin = 760, 420, 58
    indices = _downsample_indices(len(control.time_history), 1400)
    times = control.time_history[indices]
    errors = control.error_history[indices]
    y_max = max(float(np.max(errors)), 1e-6)
    colors = ("#1f77b4", "#d9480f")
    lines = []
    for arm_index, arm_label in enumerate(Config.ARM_NAMES):
        points = _polyline_points(
            times,
            errors[:, arm_index],
            x_min=0.0,
            x_max=max(float(control.duration_s), float(times[-1]) if len(times) else 1.0),
            y_min=0.0,
            y_max=y_max * 1.08,
            width=width,
            height=height,
            margin=margin,
        )
        lines.append(f'<polyline points="{points}" fill="none" stroke="{colors[arm_index]}" stroke-width="2.0"/>')
        lines.append(f'<text x="{margin + arm_index * 70}" y="48" font-size="13" font-family="Arial" fill="{colors[arm_index]}">{arm_label}</text>')
    path.write_text(
        "\n".join(
            [
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
                '<rect width="100%" height="100%" fill="#ffffff"/>',
                f'<text x="{width / 2:.1f}" y="30" text-anchor="middle" font-size="20" font-family="Arial">Closed-loop TCP position error</text>',
                f'<rect x="{margin}" y="{margin}" width="{width - 2 * margin}" height="{height - 2 * margin}" fill="#f8fafc" stroke="#334155"/>',
                *lines,
                f'<text x="{width / 2:.1f}" y="{height - 16}" text-anchor="middle" font-size="14" font-family="Arial">time / s</text>',
                f'<text x="18" y="{height / 2:.1f}" transform="rotate(-90 18 {height / 2:.1f})" text-anchor="middle" font-size="14" font-family="Arial">error / m</text>',
                "</svg>",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def _write_torque_summary_svg(path: Path, *, control_summary: dict[str, object]) -> None:
    width, height, margin = 760, 420, 58
    arms = control_summary["arms"]
    values = []
    labels = []
    for arm_label in Config.ARM_NAMES:
        arm_summary = arms[arm_label]
        values.extend([float(arm_summary["peak_abs_tau_nm"]), float(arm_summary["rms_tau_nm"])])
        labels.extend([f"{arm_label} peak", f"{arm_label} rms"])
    max_value = max(max(values), 1e-9)
    colors = ("#1f77b4", "#60a5fa", "#d9480f", "#fb923c")
    bars = []
    for index, (label, value) in enumerate(zip(labels, values)):
        bar_h = value / max_value * (height - 2 * margin - 30)
        x = margin + 52 + index * 145
        y = height - margin - bar_h
        bars.extend(
            [
                f'<rect x="{x:.1f}" y="{y:.1f}" width="78" height="{bar_h:.1f}" fill="{colors[index]}"/>',
                f'<text x="{x + 39:.1f}" y="{y - 6:.1f}" text-anchor="middle" font-size="12" font-family="Arial">{value:.2f}</text>',
                f'<text x="{x + 39:.1f}" y="{height - margin + 18}" text-anchor="middle" font-size="12" font-family="Arial">{label}</text>',
            ]
        )
    path.write_text(
        "\n".join(
            [
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
                '<rect width="100%" height="100%" fill="#ffffff"/>',
                f'<text x="{width / 2:.1f}" y="30" text-anchor="middle" font-size="20" font-family="Arial">Closed-loop torque summary</text>',
                f'<line x1="{margin}" y1="{height - margin}" x2="{width - margin}" y2="{height - margin}" stroke="#334155"/>',
                *bars,
                f'<text x="18" y="{height / 2:.1f}" transform="rotate(-90 18 {height / 2:.1f})" text-anchor="middle" font-size="14" font-family="Arial">N m</text>',
                "</svg>",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def _markdown_workspace_table(mc: MonteCarloWorkspaceResult) -> str:
    lines = ["| Arm | X span m | Y span m | Z span m | Safe box volume m^3 |", "|---|---:|---:|---:|---:|"]
    for arm_index, arm_label in enumerate(Config.ARM_NAMES):
        span = mc.pos_stats[arm_index].span
        box = mc.internal_boxes[arm_index]
        lines.append(
            f"| {arm_label} | {span[0]:.6f} | {span[1]:.6f} | {span[2]:.6f} | {box.volume:.9f} |"
        )
    return "\n".join(lines)


def _markdown_control_table(control: ControlLoopResult) -> str:
    arms = control.summary["arms"]
    lines = ["| Arm | Max error m | Mean error m | Final error m | Steady error m | Peak tau Nm | RMS tau Nm |", "|---|---:|---:|---:|---:|---:|---:|"]
    for arm_label in Config.ARM_NAMES:
        arm_summary = arms[arm_label]
        lines.append(
            "| "
            f"{arm_label} | "
            f"{float(arm_summary['max_error_m']):.6f} | "
            f"{float(arm_summary['mean_error_m']):.6f} | "
            f"{float(arm_summary['final_error_m']):.6f} | "
            f"{float(arm_summary['steady_state_error_m']):.6f} | "
            f"{float(arm_summary['peak_abs_tau_nm']):.3f} | "
            f"{float(arm_summary['rms_tau_nm']):.3f} |"
        )
    return "\n".join(lines)


def _write_report_markdown(
    path: Path,
    *,
    mc: MonteCarloWorkspaceResult,
    control: ControlLoopResult,
    files: dict[str, str],
) -> None:
    seed_text = "随机" if mc.seed is None else str(mc.seed)
    file_lines = "\n".join(f"- `{name}`: `{value}`" for name, value in sorted(files.items()))
    path.write_text(
        "\n".join(
            [
                "# AM-DPBSURDF0422 Sim-only 数据报告",
                "",
                "本报告包只使用 MuJoCo 仿真和 C/STM32 控制核心生成，不包含真机实验结论。",
                "",
                "## 实验配置",
                f"- 模型: `{Path(Config.URDF_PATH).name}`",
                f"- Monte Carlo 样本数: `{mc.samples}`",
                f"- Monte Carlo 随机种子: `{seed_text}`",
                f"- 闭环实验时长: `{control.duration_s:.3f} s`",
                f"- 固定步长: `{control.dt_s:.6f} s`",
                f"- 闭环目标段数: `{len(control.schedule)}`",
                "",
                "## Monte Carlo 工作空间结果",
                _markdown_workspace_table(mc),
                "",
                "说明：`workspace_points.csv` 仅保留末端位置和四元数，不保存采样 qpos；关节采样边界和统计摘要保存在 `workspace_summary.json`。",
                "",
                "## 闭环多目标阶跃结果",
                _markdown_control_table(control),
                "",
                f"状态计数: `{json.dumps(control.summary['status_counts'], ensure_ascii=False)}`",
                "",
                "## 可复现性",
                "- 数据包中的 CSV/JSON/SVG 均由同一次 `sim-report` 运行生成。",
                "- 闭环实验使用固定 `Config.DT` 推进控制器时间，不依赖墙钟周期。",
                "- 所有结论应表述为 sim-only 证据，不能替代真机验证。",
                "",
                "## 文件索引",
                file_lines,
                "",
            ]
        ),
        encoding="utf-8",
    )


def _write_sim_report_assets(
    output_dir: Path,
    *,
    mc: MonteCarloWorkspaceResult,
    control: ControlLoopResult,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    control_csv = output_dir / "control_loop.csv"
    control_summary_path = output_dir / "control_summary.json"
    report_md = output_dir / "report.md"
    chart_paths = {
        "workspace_xy_svg": output_dir / "workspace_xy.svg",
        "workspace_xz_svg": output_dir / "workspace_xz.svg",
        "workspace_yz_svg": output_dir / "workspace_yz.svg",
        "workspace_span_svg": output_dir / "workspace_position_spans.svg",
        "control_error_svg": output_dir / "control_error.svg",
        "control_torque_svg": output_dir / "control_torque.svg",
    }
    file_index = {
        "workspace_points_csv": str(output_dir / "workspace_points.csv"),
        "workspace_summary_json": str(output_dir / "workspace_summary.json"),
        "mc_terminal_summary_txt": str(output_dir / "mc_terminal_summary.txt"),
        "control_loop_csv": str(control_csv),
        "control_summary_json": str(control_summary_path),
        "report_md": str(report_md),
        **{name: str(path) for name, path in chart_paths.items()},
    }

    _write_control_loop_csv(control_csv, control.telemetry_rows)
    control_summary = {
        **control.summary,
        "metadata": _model_metadata(),
        "files": file_index,
    }
    control_summary_path.write_text(
        json.dumps(control_summary, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    _write_workspace_projection_svg(chart_paths["workspace_xy_svg"], points=mc.points, axes=(0, 1), title="Workspace projection XY")
    _write_workspace_projection_svg(chart_paths["workspace_xz_svg"], points=mc.points, axes=(0, 2), title="Workspace projection XZ")
    _write_workspace_projection_svg(chart_paths["workspace_yz_svg"], points=mc.points, axes=(1, 2), title="Workspace projection YZ")
    _write_position_span_svg(chart_paths["workspace_span_svg"], pos_stats=mc.pos_stats)
    _write_control_error_svg(chart_paths["control_error_svg"], control=control)
    _write_torque_summary_svg(chart_paths["control_torque_svg"], control_summary=control.summary)
    _write_report_markdown(report_md, mc=mc, control=control, files=file_index)
    _write_monte_carlo_report_assets(
        output_dir,
        samples=mc.samples,
        seed=mc.seed,
        joint_lower=mc.joint_lower,
        joint_upper=mc.joint_upper,
        points=mc.points,
        quats=mc.quats,
        pos_stats=mc.pos_stats,
        quat_stats=mc.quat_stats,
        quat_norm_stats=mc.quat_norm_stats,
        internal_boxes=mc.internal_boxes,
        hull_point_count=mc.hull_point_count,
        report_text=mc.report_text,
        metadata=_model_metadata(),
        extra_files={key: value for key, value in file_index.items() if key != "workspace_points_csv"},
    )

