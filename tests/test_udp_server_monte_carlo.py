from __future__ import annotations

import sys
import json
from pathlib import Path

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from udp_server import (
    InternalWorkspaceBox,
    RangeAccumulator,
    RangeSnapshot,
    WorkspaceHull,
    _build_control_target_schedule,
    _format_internal_workspace_box,
    _format_monte_carlo_report,
    _summarize_control_loop_arrays,
    _write_monte_carlo_report_assets,
    _write_sim_report_assets,
    _workspace_box_edges,
    compute_largest_internal_workspace_box,
    compute_workspace_hull,
    compute_workspace_bounds,
    ControlLoopResult,
    MonteCarloWorkspaceResult,
    resolve_sampling_bounds,
    select_visualization_points,
)


def test_range_accumulator_tracks_min_max_mean_and_last():
    accumulator = RangeAccumulator(3)

    accumulator.update([1.0, 3.0, -2.0])
    accumulator.update([-1.0, 5.0, 4.0])

    snapshot = accumulator.snapshot()

    assert snapshot.count == 2
    np.testing.assert_allclose(snapshot.minimum, [-1.0, 3.0, -2.0])
    np.testing.assert_allclose(snapshot.maximum, [1.0, 5.0, 4.0])
    np.testing.assert_allclose(snapshot.span, [2.0, 2.0, 6.0])
    np.testing.assert_allclose(snapshot.mean, [0.0, 4.0, 1.0])
    np.testing.assert_allclose(snapshot.last, [-1.0, 5.0, 4.0])


def test_range_accumulator_rejects_wrong_width():
    accumulator = RangeAccumulator(3)

    with pytest.raises(ValueError, match="expected vector shape"):
        accumulator.update([1.0, 2.0])


def test_resolve_sampling_bounds_replaces_infinite_limits():
    lower, upper = resolve_sampling_bounds(
        np.array([-np.inf, -1.0, 0.0]),
        np.array([np.inf, 1.0, 2.0]),
        fallback_limit=3.14,
    )

    np.testing.assert_allclose(lower, [-3.14, -1.0, 0.0])
    np.testing.assert_allclose(upper, [3.14, 1.0, 2.0])


def test_resolve_sampling_bounds_rejects_invalid_limits():
    with pytest.raises(ValueError, match="invalid joint sampling bounds"):
        resolve_sampling_bounds(np.array([1.0]), np.array([1.0]))


def test_compute_workspace_bounds_adds_padding_and_minimum_size():
    center, half_size = compute_workspace_bounds(
        np.array([-1.0, 2.0, 0.5]),
        np.array([3.0, 2.0, 1.5]),
        padding_ratio=0.1,
        min_half_size=0.25,
    )

    np.testing.assert_allclose(center, [1.0, 2.0, 1.0])
    np.testing.assert_allclose(half_size, [2.2, 0.25, 0.55])


def test_select_visualization_points_keeps_endpoints_when_decimating():
    points = np.arange(30, dtype=np.float64).reshape(10, 3)

    selected = select_visualization_points(points, max_points=4)

    assert selected.shape == (4, 3)
    np.testing.assert_allclose(selected[0], points[0])
    np.testing.assert_allclose(selected[-1], points[-1])


def test_compute_workspace_hull_builds_convex_polytope_for_cube_points():
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [0.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.5, 0.5, 0.5],
        ],
        dtype=np.float64,
    )

    hull = compute_workspace_hull(points)

    assert isinstance(hull, WorkspaceHull)
    assert hull.vertices.shape == (8, 3)
    assert hull.triangles.shape == (12, 3)
    assert len(hull.edges) == 18
    np.testing.assert_allclose(hull.center, [0.5, 0.5, 0.5])


def test_compute_workspace_hull_returns_empty_for_degenerate_points():
    points = np.zeros((8, 3), dtype=np.float64)

    hull = compute_workspace_hull(points)

    assert hull.vertices.shape == (0, 3)
    assert hull.triangles.shape == (0, 3)
    assert hull.edges == []


def test_compute_largest_internal_workspace_box_matches_axis_aligned_prism():
    points = np.array(
        [
            [-1.0, -2.0, 0.5],
            [3.0, -2.0, 0.5],
            [-1.0, 2.0, 0.5],
            [3.0, 2.0, 0.5],
            [-1.0, -2.0, 2.5],
            [3.0, -2.0, 2.5],
            [-1.0, 2.0, 2.5],
            [3.0, 2.0, 2.5],
        ],
        dtype=np.float64,
    )
    hull = compute_workspace_hull(points)

    box = compute_largest_internal_workspace_box(hull)

    assert isinstance(box, InternalWorkspaceBox)
    assert not box.is_empty
    np.testing.assert_allclose(box.center, [1.0, 0.0, 1.5], atol=1e-5)
    np.testing.assert_allclose(box.half_size, [2.0, 2.0, 1.0], atol=1e-5)
    np.testing.assert_allclose(box.lower, [-1.0, -2.0, 0.5], atol=1e-5)
    np.testing.assert_allclose(box.upper, [3.0, 2.0, 2.5], atol=1e-5)
    assert box.volume == pytest.approx(32.0, rel=1e-5)


def test_compute_largest_internal_workspace_box_stays_inside_slanted_hull():
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            [0.0, 0.0, 2.0],
        ],
        dtype=np.float64,
    )
    hull = compute_workspace_hull(points)

    box = compute_largest_internal_workspace_box(hull)

    assert not box.is_empty
    corners = np.array(
        [
            [x, y, z]
            for x in (box.lower[0], box.upper[0])
            for y in (box.lower[1], box.upper[1])
            for z in (box.lower[2], box.upper[2])
        ],
        dtype=np.float64,
    )
    equations = np.column_stack((hull.halfspace_normals, hull.halfspace_offsets))
    assert np.max(corners @ equations[:, :3].T + equations[:, 3]) <= 1e-7


def test_compute_largest_internal_workspace_box_returns_empty_for_empty_hull():
    box = compute_largest_internal_workspace_box(
        WorkspaceHull(
            vertices=np.empty((0, 3), dtype=np.float64),
            triangles=np.empty((0, 3), dtype=np.int32),
            edges=[],
            center=np.zeros(3, dtype=np.float64),
        )
    )

    assert box.is_empty


def test_format_internal_workspace_box_reports_safe_pos_ranges():
    box = InternalWorkspaceBox(
        center=np.array([1.0, 2.0, 3.0]),
        half_size=np.array([0.5, 0.25, 0.75]),
        lower=np.array([0.5, 1.75, 2.25]),
        upper=np.array([1.5, 2.25, 3.75]),
        volume=1.5,
    )

    report = _format_internal_workspace_box(box, hull_point_count=42)

    assert "安全 pos 输入范围" in report
    assert "参与凸包计算的采样点数：42" in report
    assert "x" in report
    assert "0.500000" in report
    assert "3.750000" in report


def test_format_monte_carlo_report_uses_chinese_terminal_labels():
    pos_stats = RangeSnapshot(
        count=2,
        minimum=np.array([0.0, 1.0, 2.0]),
        maximum=np.array([1.0, 2.0, 3.0]),
        mean=np.array([0.5, 1.5, 2.5]),
        last=np.array([1.0, 2.0, 3.0]),
    )
    quat_stats = RangeSnapshot(
        count=2,
        minimum=np.array([1.0, 0.0, 0.0, 0.0]),
        maximum=np.array([1.0, 0.1, 0.2, 0.3]),
        mean=np.array([1.0, 0.05, 0.1, 0.15]),
        last=np.array([1.0, 0.1, 0.2, 0.3]),
    )
    quat_norm_stats = RangeSnapshot(
        count=2,
        minimum=np.array([1.0]),
        maximum=np.array([1.1]),
        mean=np.array([1.05]),
        last=np.array([1.1]),
    )
    internal_box = InternalWorkspaceBox(
        center=np.array([0.5, 1.5, 2.5]),
        half_size=np.array([0.25, 0.25, 0.25]),
        lower=np.array([0.25, 1.25, 2.25]),
        upper=np.array([0.75, 1.75, 2.75]),
        volume=0.125,
    )

    report = _format_monte_carlo_report(
        samples=2,
        seed=None,
        joint_lower=np.array([-1.0, -2.0]),
        joint_upper=np.array([1.0, 2.0]),
        pos_stats=pos_stats,
        quat_stats=quat_stats,
        quat_norm_stats=quat_norm_stats,
        internal_box=internal_box,
        hull_point_count=2,
        last_qpos=np.array([0.1, 0.2]),
    )

    assert "AM-DPBSURDF0422 蒙特卡洛末端位姿范围检查" in report
    assert "采样数量：2" in report
    assert "随机种子：随机" in report
    assert "末端位置范围：" in report
    assert "最大内部轴对齐长方体" in report
    assert "末端四元数范围" in report
    assert "最后刷新样本：" in report


def test_workspace_box_edges_returns_twelve_edges():
    edges = _workspace_box_edges(np.array([1.0, 2.0, 3.0]), np.array([0.5, 1.0, 1.5]))

    assert len(edges) == 12
    for start, end in edges:
        assert start.shape == (3,)
        assert end.shape == (3,)


def test_write_monte_carlo_report_assets_exports_csv_json_and_terminal_text(tmp_path):
    pos_stats = [
        RangeSnapshot(
            count=2,
            minimum=np.array([0.0, 1.0, 2.0]),
            maximum=np.array([1.0, 2.0, 3.0]),
            mean=np.array([0.5, 1.5, 2.5]),
            last=np.array([1.0, 2.0, 3.0]),
        ),
        RangeSnapshot(
            count=2,
            minimum=np.array([-1.0, -2.0, -3.0]),
            maximum=np.array([0.0, -1.0, -2.0]),
            mean=np.array([-0.5, -1.5, -2.5]),
            last=np.array([0.0, -1.0, -2.0]),
        ),
    ]
    quat_stats = [
        RangeSnapshot(2, np.array([1.0, 0.0, 0.0, 0.0]), np.array([1.0, 0.1, 0.2, 0.3]), np.array([1.0, 0.05, 0.1, 0.15]), np.array([1.0, 0.1, 0.2, 0.3])),
        RangeSnapshot(2, np.array([1.0, -0.3, -0.2, -0.1]), np.array([1.0, 0.0, 0.0, 0.0]), np.array([1.0, -0.15, -0.1, -0.05]), np.array([1.0, 0.0, 0.0, 0.0])),
    ]
    quat_norm_stats = [
        RangeSnapshot(2, np.array([1.0]), np.array([1.1]), np.array([1.05]), np.array([1.1])),
        RangeSnapshot(2, np.array([1.0]), np.array([1.2]), np.array([1.1]), np.array([1.2])),
    ]
    boxes = [
        InternalWorkspaceBox(np.array([0.5, 1.5, 2.5]), np.array([0.5, 0.5, 0.5]), np.array([0.0, 1.0, 2.0]), np.array([1.0, 2.0, 3.0]), 1.0),
        InternalWorkspaceBox(np.array([-0.5, -1.5, -2.5]), np.array([0.5, 0.5, 0.5]), np.array([-1.0, -2.0, -3.0]), np.array([0.0, -1.0, -2.0]), 1.0),
    ]
    points = [
        np.array([[0.0, 1.0, 2.0], [1.0, 2.0, 3.0]], dtype=np.float64),
        np.array([[-1.0, -2.0, -3.0], [0.0, -1.0, -2.0]], dtype=np.float64),
    ]
    quats = [
        np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.1, 0.2, 0.3]], dtype=np.float64),
        np.array([[1.0, -0.3, -0.2, -0.1], [1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
    ]

    _write_monte_carlo_report_assets(
        tmp_path,
        samples=2,
        seed=42,
        joint_lower=np.array([-1.0, -2.0]),
        joint_upper=np.array([1.0, 2.0]),
        points=points,
        quats=quats,
        pos_stats=pos_stats,
        quat_stats=quat_stats,
        quat_norm_stats=quat_norm_stats,
        internal_boxes=boxes,
        hull_point_count=2,
        report_text="terminal report",
    )

    csv_text = (tmp_path / "workspace_points.csv").read_text(encoding="utf-8")
    summary = json.loads((tmp_path / "workspace_summary.json").read_text(encoding="utf-8"))
    terminal = (tmp_path / "mc_terminal_summary.txt").read_text(encoding="utf-8")

    assert "sample,arm,x,y,z,qx,qy,qz,qw" in csv_text
    assert "left" in csv_text
    assert summary["samples"] == 2
    assert summary["seed"] == 42
    assert summary["arms"]["left"]["position"]["span"] == [1.0, 1.0, 1.0]
    assert "terminal report" in terminal


def test_control_target_schedule_uses_five_small_step_segments():
    home_pos = np.array([[0.4, 0.2, 0.3], [0.4, -0.2, 0.3]], dtype=np.float64)
    home_quat = np.tile([1.0, 0.0, 0.0, 0.0], (2, 1))

    schedule = _build_control_target_schedule(home_pos, home_quat, duration_s=10.0)

    assert len(schedule) == 5
    assert [segment.label for segment in schedule] == [
        "home_hold",
        "step_1",
        "step_2",
        "step_3",
        "return_home",
    ]
    np.testing.assert_allclose(schedule[0].target_pos_base, home_pos)
    np.testing.assert_allclose(schedule[-1].target_pos_base, home_pos)
    for segment in schedule[1:4]:
        assert np.max(np.abs(segment.target_pos_base - home_pos)) <= 0.03
    assert schedule[0].start_s == pytest.approx(0.0)
    assert schedule[-1].end_s == pytest.approx(10.0)


def test_summarize_control_loop_arrays_reports_error_torque_and_status_counts():
    error_history = np.array(
        [
            [0.03, 0.04],
            [0.02, 0.03],
            [0.01, 0.02],
            [0.005, 0.010],
        ],
        dtype=np.float64,
    )
    tau_history = np.zeros((4, 14), dtype=np.float64)
    tau_history[:, 0] = [1.0, -2.0, 3.0, -4.0]
    tau_history[:, 7] = [0.5, -1.0, 1.5, -2.0]
    status_history = np.array([0, 0, -1, 0], dtype=np.int32)

    summary = _summarize_control_loop_arrays(
        duration_s=0.004,
        dt_s=0.001,
        log_stride=2,
        error_history=error_history,
        tau_history=tau_history,
        status_history=status_history,
    )

    assert summary["steps"] == 4
    assert summary["status_counts"] == {"0": 3, "-1": 1}
    assert summary["arms"]["left"]["max_error_m"] == pytest.approx(0.03)
    assert summary["arms"]["left"]["final_error_m"] == pytest.approx(0.005)
    assert summary["arms"]["left"]["peak_abs_tau_nm"] == pytest.approx(4.0)
    assert summary["arms"]["right"]["peak_abs_tau_nm"] == pytest.approx(2.0)


def test_write_sim_report_assets_exports_report_package(tmp_path):
    pos_stats = [
        RangeSnapshot(2, np.array([0.0, 1.0, 2.0]), np.array([1.0, 2.0, 3.0]), np.array([0.5, 1.5, 2.5]), np.array([1.0, 2.0, 3.0])),
        RangeSnapshot(2, np.array([-1.0, -2.0, -3.0]), np.array([0.0, -1.0, -2.0]), np.array([-0.5, -1.5, -2.5]), np.array([0.0, -1.0, -2.0])),
    ]
    quat_stats = [
        RangeSnapshot(2, np.array([1.0, 0.0, 0.0, 0.0]), np.array([1.0, 0.1, 0.2, 0.3]), np.array([1.0, 0.05, 0.1, 0.15]), np.array([1.0, 0.1, 0.2, 0.3])),
        RangeSnapshot(2, np.array([1.0, -0.3, -0.2, -0.1]), np.array([1.0, 0.0, 0.0, 0.0]), np.array([1.0, -0.15, -0.1, -0.05]), np.array([1.0, 0.0, 0.0, 0.0])),
    ]
    quat_norm_stats = [
        RangeSnapshot(2, np.array([1.0]), np.array([1.0]), np.array([1.0]), np.array([1.0])),
        RangeSnapshot(2, np.array([1.0]), np.array([1.0]), np.array([1.0]), np.array([1.0])),
    ]
    boxes = [
        InternalWorkspaceBox(np.array([0.5, 1.5, 2.5]), np.array([0.5, 0.5, 0.5]), np.array([0.0, 1.0, 2.0]), np.array([1.0, 2.0, 3.0]), 1.0),
        InternalWorkspaceBox(np.array([-0.5, -1.5, -2.5]), np.array([0.5, 0.5, 0.5]), np.array([-1.0, -2.0, -3.0]), np.array([0.0, -1.0, -2.0]), 1.0),
    ]
    points = [
        np.array([[0.0, 1.0, 2.0], [1.0, 2.0, 3.0]], dtype=np.float64),
        np.array([[-1.0, -2.0, -3.0], [0.0, -1.0, -2.0]], dtype=np.float64),
    ]
    quats = [
        np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.1, 0.2, 0.3]], dtype=np.float64),
        np.array([[1.0, -0.3, -0.2, -0.1], [1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
    ]
    mc = MonteCarloWorkspaceResult(
        samples=2,
        seed=42,
        joint_lower=np.array([-1.0, -2.0]),
        joint_upper=np.array([1.0, 2.0]),
        points=points,
        quats=quats,
        pos_stats=pos_stats,
        quat_stats=quat_stats,
        quat_norm_stats=quat_norm_stats,
        workspace_hulls=[
            WorkspaceHull(np.empty((0, 3)), np.empty((0, 3), dtype=np.int32), [], np.zeros(3)),
            WorkspaceHull(np.empty((0, 3)), np.empty((0, 3), dtype=np.int32), [], np.zeros(3)),
        ],
        internal_boxes=boxes,
        hull_point_count=2,
        last_qpos=np.zeros(14),
        report_text="terminal report",
    )
    home_pos = np.array([[0.4, 0.2, 0.3], [0.4, -0.2, 0.3]], dtype=np.float64)
    schedule = _build_control_target_schedule(
        home_pos,
        np.tile([1.0, 0.0, 0.0, 0.0], (2, 1)),
        duration_s=0.004,
    )
    control_summary = _summarize_control_loop_arrays(
        duration_s=0.004,
        dt_s=0.001,
        log_stride=1,
        error_history=np.array([[0.02, 0.03], [0.01, 0.02]], dtype=np.float64),
        tau_history=np.zeros((2, 14), dtype=np.float64),
        status_history=np.array([0, 0], dtype=np.int32),
    )
    control = ControlLoopResult(
        duration_s=0.004,
        dt_s=0.001,
        steps=2,
        log_stride=1,
        schedule=schedule,
        telemetry_rows=[
            {
                field: 0
                for field in [
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
            }
        ],
        summary=control_summary,
        error_history=np.array([[0.02, 0.03], [0.01, 0.02]], dtype=np.float64),
        tau_history=np.zeros((2, 14), dtype=np.float64),
        time_history=np.array([0.0, 0.001], dtype=np.float64),
    )

    _write_sim_report_assets(tmp_path, mc=mc, control=control)

    assert (tmp_path / "workspace_points.csv").is_file()
    assert (tmp_path / "control_loop.csv").is_file()
    assert (tmp_path / "control_summary.json").is_file()
    assert (tmp_path / "report.md").is_file()
    assert (tmp_path / "workspace_xy.svg").read_text(encoding="utf-8").startswith("<svg")
    workspace_csv = (tmp_path / "workspace_points.csv").read_text(encoding="utf-8")
    assert "sample,arm,x,y,z,qx,qy,qz,qw" in workspace_csv
    assert "q_00_rad" not in workspace_csv
    control_summary_json = json.loads((tmp_path / "control_summary.json").read_text(encoding="utf-8"))
    assert control_summary_json["sim_only"] is True
    assert "Sim-only" in (tmp_path / "report.md").read_text(encoding="utf-8")
