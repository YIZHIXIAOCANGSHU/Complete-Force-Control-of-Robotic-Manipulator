from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from udp_server import (
    InternalWorkspaceBox,
    RangeAccumulator,
    RangeSnapshot,
    WorkspaceHull,
    _format_internal_workspace_box,
    _format_monte_carlo_report,
    _workspace_box_edges,
    compute_largest_internal_workspace_box,
    compute_workspace_hull,
    compute_workspace_bounds,
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
