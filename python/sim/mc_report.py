"""Monte Carlo and sim-only report entrypoints."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

from config import Config
from sim.env_factory import _create_sim_env
from sim.mc_viewer import _show_monte_carlo_workspace_viewer
from sim.report_assets import (
    DEFAULT_REPORT_CONTROL_DURATION_S,
    DEFAULT_REPORT_CONTROL_LOG_STRIDE,
    DEFAULT_REPORT_PROGRESS_INTERVAL,
    DEFAULT_REPORT_SAMPLES,
    _default_report_output_dir,
    _run_closed_loop_report_experiment,
    _write_monte_carlo_report_assets,
    _write_sim_report_assets,
)
from sim.workspace import (
    DEFAULT_MC_FALLBACK_LIMIT,
    DEFAULT_MC_MAX_HULL_POINTS,
    DEFAULT_MC_MAX_VIS_POINTS,
    DEFAULT_MC_PROGRESS_INTERVAL,
    DEFAULT_MC_SAMPLES,
    MonteCarloWorkspaceResult,
    RangeAccumulator,
    compute_largest_internal_workspace_box,
    compute_workspace_hull,
    resolve_sampling_bounds,
    select_visualization_points,
)
from sim.report_assets import _format_dual_monte_carlo_report, _format_vector


def _collect_monte_carlo_workspace(
    *,
    env,
    samples: int,
    seed: int | None,
    progress_interval: int,
    fallback_joint_limit: float = DEFAULT_MC_FALLBACK_LIMIT,
    max_hull_points: int = DEFAULT_MC_MAX_HULL_POINTS,
    allow_partial_on_interrupt: bool = False,
    progress_label: str = "[Report MC]",
) -> MonteCarloWorkspaceResult:
    if samples <= 0:
        raise ValueError("samples 必须为正数")
    if progress_interval < 0:
        raise ValueError("progress_interval 不能为负数")
    if max_hull_points <= 0:
        raise ValueError("max_hull_points 必须为正数")

    env.reset(Config.HOME_QPOS)
    env.forward()
    joint_lower, joint_upper = resolve_sampling_bounds(
        env.joint_lower,
        env.joint_upper,
        fallback_limit=fallback_joint_limit,
    )
    rng = np.random.default_rng(seed)
    pos_stats = [RangeAccumulator(3) for _ in range(Config.NUM_ARMS)]
    quat_stats = [RangeAccumulator(4) for _ in range(Config.NUM_ARMS)]
    quat_norm_stats = [RangeAccumulator(1) for _ in range(Config.NUM_ARMS)]
    points = [
        np.empty((samples, 3), dtype=np.float64)
        for _ in range(Config.NUM_ARMS)
    ]
    quats = [
        np.empty((samples, 4), dtype=np.float64)
        for _ in range(Config.NUM_ARMS)
    ]
    last_qpos = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    sampled_count = 0

    try:
        for index in range(samples):
            last_qpos = rng.uniform(joint_lower, joint_upper)
            env.set_qpos(last_qpos)
            env.set_qvel(np.zeros_like(last_qpos))
            env.forward()

            pos = env.get_all_ee_pos()
            quat = env.get_all_ee_quat()
            for arm in range(Config.NUM_ARMS):
                points[arm][index] = pos[arm]
                quats[arm][index] = quat[arm]
                pos_stats[arm].update(pos[arm])
                quat_stats[arm].update(quat[arm])
                quat_norm_stats[arm].update(np.array([np.linalg.norm(quat[arm])], dtype=np.float64))

            sampled_count = index + 1
            if progress_interval and (sampled_count % progress_interval == 0 or sampled_count == samples):
                sys.stdout.write(
                    "\r"
                    f"{progress_label} {sampled_count:>{len(str(samples))}}/{samples} "
                    f"左臂={_format_vector(pos[Config.LEFT_ARM], precision=4)} "
                    f"右臂={_format_vector(pos[Config.RIGHT_ARM], precision=4)}"
                )
                sys.stdout.flush()
    except KeyboardInterrupt:
        if not allow_partial_on_interrupt:
            raise
        if progress_interval:
            sys.stdout.write("\n")
        print("[MC] 用户中断，输出已经采集到的范围。")
    if progress_interval:
        sys.stdout.write("\n")
    if sampled_count <= 0:
        raise ValueError("没有采到样本")

    points = [arm_points[:sampled_count].copy() for arm_points in points]
    quats = [arm_quats[:sampled_count].copy() for arm_quats in quats]

    hull_points = [select_visualization_points(arm_points, max_hull_points) for arm_points in points]
    workspace_hulls = [compute_workspace_hull(arm_points) for arm_points in hull_points]
    internal_boxes = [compute_largest_internal_workspace_box(hull) for hull in workspace_hulls]
    pos_snapshots = [stats.snapshot() for stats in pos_stats]
    quat_snapshots = [stats.snapshot() for stats in quat_stats]
    quat_norm_snapshots = [stats.snapshot() for stats in quat_norm_stats]
    hull_point_count = len(hull_points[Config.LEFT_ARM])
    report_text = _format_dual_monte_carlo_report(
        samples=sampled_count,
        seed=seed,
        joint_lower=joint_lower,
        joint_upper=joint_upper,
        pos_stats=pos_snapshots,
        quat_stats=quat_snapshots,
        quat_norm_stats=quat_norm_snapshots,
        internal_boxes=internal_boxes,
        hull_point_count=hull_point_count,
        last_qpos=last_qpos,
    )
    return MonteCarloWorkspaceResult(
        samples=sampled_count,
        seed=seed,
        joint_lower=joint_lower,
        joint_upper=joint_upper,
        points=points,
        quats=quats,
        pos_stats=pos_snapshots,
        quat_stats=quat_snapshots,
        quat_norm_stats=quat_norm_snapshots,
        workspace_hulls=workspace_hulls,
        internal_boxes=internal_boxes,
        hull_point_count=hull_point_count,
        last_qpos=last_qpos,
        report_text=report_text,
    )

def run_sim_report(
    *,
    samples: int = DEFAULT_REPORT_SAMPLES,
    seed: int | None = 42,
    progress_interval: int = DEFAULT_REPORT_PROGRESS_INTERVAL,
    max_hull_points: int = DEFAULT_MC_MAX_HULL_POINTS,
    output_dir: str | Path | None = None,
    control_duration_s: float = DEFAULT_REPORT_CONTROL_DURATION_S,
    control_log_stride: int = DEFAULT_REPORT_CONTROL_LOG_STRIDE,
) -> Path:
    """Generate a sim-only report package with MC workspace and closed-loop data."""
    out = Path(output_dir) if output_dir is not None else _default_report_output_dir()
    print("=" * 60)
    print("      AM-DPBSURDF0422 Sim-only 报告数据包生成      ")
    print("=" * 60)
    print(f"[Report] 输出目录: {out}")
    print(f"[Report] Monte Carlo samples={samples}, seed={'随机' if seed is None else seed}")
    sys.stdout.flush()

    env = _create_sim_env()
    mc_result = _collect_monte_carlo_workspace(
        env=env,
        samples=samples,
        seed=seed,
        progress_interval=progress_interval,
        max_hull_points=max_hull_points,
    )
    print(mc_result.report_text)
    print(f"[Report] 运行闭环多目标阶跃实验: duration={control_duration_s:.3f}s")
    control_result = _run_closed_loop_report_experiment(
        env=env,
        duration_s=control_duration_s,
        log_stride=control_log_stride,
    )
    _write_sim_report_assets(out, mc=mc_result, control=control_result)
    print(f"[Report] 报告数据包已生成: {out}")
    print(f"[Report] 中文摘要: {out / 'report.md'}")
    return out


def run_monte_carlo_range_check(
    *,
    samples: int = DEFAULT_MC_SAMPLES,
    seed: int | None = None,
    progress_interval: int = DEFAULT_MC_PROGRESS_INTERVAL,
    fallback_joint_limit: float = DEFAULT_MC_FALLBACK_LIMIT,
    show_viewer: bool = True,
    max_visual_points: int = DEFAULT_MC_MAX_VIS_POINTS,
    max_hull_points: int = DEFAULT_MC_MAX_HULL_POINTS,
    output_dir: str | Path | None = None,
) -> None:
    """Use the normal sim environment to sample FK ranges without starting UDP."""
    if samples <= 0:
        raise ValueError("samples 必须为正数")
    if progress_interval < 0:
        raise ValueError("progress_interval 不能为负数")
    if max_hull_points <= 0:
        raise ValueError("max_hull_points 必须为正数")

    print("=" * 60)
    print("      AM-DPBSURDF0422 Python MuJoCo 仿真服务      ")
    print("      蒙特卡洛末端位置/四元数范围检查              ")
    print("=" * 60)
    sys.stdout.flush()

    print(f"[MC] 采样数量={samples}, 随机种子={'随机' if seed is None else seed}")
    print("[MC] 使用原 sim 的 MujocoSimEnv 做双臂 FK 采样，按 Ctrl+C 可提前输出已采样范围。")

    env = _create_sim_env()
    try:
        mc_result = _collect_monte_carlo_workspace(
            env=env,
            samples=samples,
            seed=seed,
            progress_interval=progress_interval,
            fallback_joint_limit=fallback_joint_limit,
            max_hull_points=max_hull_points,
            allow_partial_on_interrupt=True,
            progress_label="[MC]",
        )
    except ValueError as exc:
        if "没有采到样本" not in str(exc):
            raise
        print("[MC] 没有采到样本，结束。")
        return

    print(mc_result.report_text)

    if output_dir is not None:
        _write_monte_carlo_report_assets(
            output_dir,
            samples=mc_result.samples,
            seed=seed,
            joint_lower=mc_result.joint_lower,
            joint_upper=mc_result.joint_upper,
            points=mc_result.points,
            quats=mc_result.quats,
            pos_stats=mc_result.pos_stats,
            quat_stats=mc_result.quat_stats,
            quat_norm_stats=mc_result.quat_norm_stats,
            internal_boxes=mc_result.internal_boxes,
            hull_point_count=mc_result.hull_point_count,
            report_text=mc_result.report_text,
        )

    if show_viewer:
        _show_monte_carlo_workspace_viewer(
            env,
            points=mc_result.points,
            pos_stats=mc_result.pos_stats,
            last_qpos=mc_result.last_qpos,
            hull=mc_result.workspace_hulls,
            internal_box=mc_result.internal_boxes,
            max_visual_points=max_visual_points,
            max_hull_points=max_hull_points,
        )

