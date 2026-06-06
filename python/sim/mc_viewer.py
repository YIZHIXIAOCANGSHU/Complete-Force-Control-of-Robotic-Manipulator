"""MuJoCo viewer drawing helpers for Monte Carlo workspace results."""

from __future__ import annotations

from contextlib import nullcontext
import sys
import time

import numpy as np

from config import Config
from mujoco_viewer import VIEWER_AVAILABLE, launch_passive_viewer
from sim.workspace import (
    DEFAULT_MC_MAX_HULL_POINTS,
    DEFAULT_MC_MAX_VIS_POINTS,
    InternalWorkspaceBox,
    RangeSnapshot,
    WorkspaceHull,
    compute_largest_internal_workspace_box,
    compute_workspace_bounds,
    compute_workspace_hull,
    select_visualization_points,
)


def _add_viewer_geom(
    scene,
    geom_type,
    *,
    size: np.ndarray,
    pos: np.ndarray,
    rgba: np.ndarray,
    mat: np.ndarray | None = None,
) -> bool:
    if scene.ngeom >= scene.maxgeom:
        return False
    import mujoco

    matrix = np.eye(3, dtype=np.float64).reshape(-1) if mat is None else np.asarray(mat, dtype=np.float64)
    mujoco.mjv_initGeom(
        scene.geoms[scene.ngeom],
        int(geom_type),
        np.asarray(size, dtype=np.float64),
        np.asarray(pos, dtype=np.float64),
        matrix,
        np.asarray(rgba, dtype=np.float32),
    )
    scene.ngeom += 1
    return True


def _add_viewer_connector(scene, geom_type, *, width: float, start: np.ndarray, end: np.ndarray, rgba: np.ndarray) -> bool:
    if scene.ngeom >= scene.maxgeom:
        return False
    import mujoco

    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        int(geom_type),
        np.zeros(3, dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(-1),
        np.asarray(rgba, dtype=np.float32),
    )
    mujoco.mjv_connector(
        geom,
        int(geom_type),
        width,
        np.asarray(start, dtype=np.float64),
        np.asarray(end, dtype=np.float64),
    )
    scene.ngeom += 1
    return True


def _add_viewer_triangle(scene, v0: np.ndarray, v1: np.ndarray, v2: np.ndarray, rgba: np.ndarray) -> bool:
    edge1 = np.asarray(v1, dtype=np.float64) - np.asarray(v0, dtype=np.float64)
    edge2 = np.asarray(v2, dtype=np.float64) - np.asarray(v0, dtype=np.float64)
    normal = np.cross(edge1, edge2)
    lengths = np.array(
        [np.linalg.norm(edge1), np.linalg.norm(edge2), np.linalg.norm(normal)],
        dtype=np.float64,
    )
    if np.any(lengths <= 1e-12):
        return True

    matrix = np.column_stack((edge1 / lengths[0], edge2 / lengths[1], normal / lengths[2])).reshape(-1)
    return _add_viewer_geom(
        scene,
        _mujoco_geom_triangle(),
        size=lengths,
        pos=np.asarray(v0, dtype=np.float64),
        mat=matrix,
        rgba=rgba,
    )


def _mujoco_geom_triangle():
    import mujoco

    return mujoco.mjtGeom.mjGEOM_TRIANGLE


def _workspace_box_edges(center: np.ndarray, half_size: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    signs = np.array(
        [
            [-1.0, -1.0, -1.0],
            [-1.0, -1.0, 1.0],
            [-1.0, 1.0, -1.0],
            [-1.0, 1.0, 1.0],
            [1.0, -1.0, -1.0],
            [1.0, -1.0, 1.0],
            [1.0, 1.0, -1.0],
            [1.0, 1.0, 1.0],
        ],
        dtype=np.float64,
    )
    corners = center + signs * half_size
    edge_indices = [
        (0, 1),
        (0, 2),
        (0, 4),
        (3, 1),
        (3, 2),
        (3, 7),
        (5, 1),
        (5, 4),
        (5, 7),
        (6, 2),
        (6, 4),
        (6, 7),
    ]
    return [(corners[start], corners[end]) for start, end in edge_indices]


def _draw_workspace_shape(scene, center: np.ndarray, half_size: np.ndarray, points: np.ndarray) -> None:
    import mujoco

    scene.ngeom = 0
    _add_viewer_geom(
        scene,
        mujoco.mjtGeom.mjGEOM_BOX,
        size=half_size,
        pos=center,
        rgba=np.array([0.1, 0.55, 1.0, 0.16], dtype=np.float32),
    )
    for start, end in _workspace_box_edges(center, half_size):
        if not _add_viewer_connector(
            scene,
            mujoco.mjtGeom.mjGEOM_LINE,
            width=4.0,
            start=start,
            end=end,
            rgba=np.array([0.05, 0.35, 1.0, 0.95], dtype=np.float32),
        ):
            break

    point_radius = max(float(np.min(half_size)) * 0.025, 0.003)
    for point in points:
        if not _add_viewer_geom(
            scene,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([point_radius, 0.0, 0.0], dtype=np.float64),
            pos=point,
            rgba=np.array([1.0, 0.72, 0.1, 0.85], dtype=np.float32),
        ):
            break


def _draw_internal_workspace_box(scene, box: InternalWorkspaceBox) -> None:
    if box.is_empty:
        return
    import mujoco

    _add_viewer_geom(
        scene,
        mujoco.mjtGeom.mjGEOM_BOX,
        size=box.half_size,
        pos=box.center,
        rgba=np.array([0.1, 0.85, 0.35, 0.24], dtype=np.float32),
    )
    for start, end in _workspace_box_edges(box.center, box.half_size):
        if not _add_viewer_connector(
            scene,
            mujoco.mjtGeom.mjGEOM_LINE,
            width=5.0,
            start=start,
            end=end,
            rgba=np.array([0.0, 0.72, 0.28, 0.98], dtype=np.float32),
        ):
            break


def _draw_workspace_hull(
    scene,
    hull: WorkspaceHull,
    points: np.ndarray,
    internal_box: InternalWorkspaceBox | None = None,
) -> None:
    import mujoco

    scene.ngeom = 0
    if hull.is_empty:
        center, half_size = compute_workspace_bounds(np.min(points, axis=0), np.max(points, axis=0))
        _draw_workspace_shape(scene, center, half_size, points)
        return

    face_rgba = np.array([0.0, 0.52, 1.0, 0.22], dtype=np.float32)
    edge_rgba = np.array([0.0, 0.24, 0.95, 0.95], dtype=np.float32)
    vertex_rgba = np.array([0.0, 0.68, 1.0, 0.85], dtype=np.float32)
    point_rgba = np.array([1.0, 0.72, 0.1, 0.78], dtype=np.float32)

    if internal_box is not None:
        _draw_internal_workspace_box(scene, internal_box)

    for a, b, c in hull.triangles:
        if not _add_viewer_triangle(scene, hull.vertices[a], hull.vertices[b], hull.vertices[c], face_rgba):
            break

    for start_index, end_index in hull.edges:
        if not _add_viewer_connector(
            scene,
            mujoco.mjtGeom.mjGEOM_LINE,
            width=3.0,
            start=hull.vertices[start_index],
            end=hull.vertices[end_index],
            rgba=edge_rgba,
        ):
            break

    hull_span = np.ptp(hull.vertices, axis=0)
    point_radius = max(float(np.max(hull_span)) * 0.006, 0.002)
    vertex_radius = point_radius * 1.4
    for vertex in hull.vertices:
        if not _add_viewer_geom(
            scene,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([vertex_radius, 0.0, 0.0], dtype=np.float64),
            pos=vertex,
            rgba=vertex_rgba,
        ):
            break

    for point in points:
        if not _add_viewer_geom(
            scene,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([point_radius, 0.0, 0.0], dtype=np.float64),
            pos=point,
            rgba=point_rgba,
        ):
            break


def _draw_workspace_hull_layer(
    scene,
    hull: WorkspaceHull,
    points: np.ndarray,
    internal_box: InternalWorkspaceBox | None,
    *,
    face_rgba: np.ndarray,
    edge_rgba: np.ndarray,
    vertex_rgba: np.ndarray,
    point_rgba: np.ndarray,
    box_rgba: np.ndarray,
) -> None:
    import mujoco

    if internal_box is not None and not internal_box.is_empty:
        _add_viewer_geom(
            scene,
            mujoco.mjtGeom.mjGEOM_BOX,
            size=internal_box.half_size,
            pos=internal_box.center,
            rgba=box_rgba,
        )
        for start, end in _workspace_box_edges(internal_box.center, internal_box.half_size):
            if not _add_viewer_connector(
                scene,
                mujoco.mjtGeom.mjGEOM_LINE,
                width=4.0,
                start=start,
                end=end,
                rgba=edge_rgba,
            ):
                break

    if hull.is_empty:
        point_radius = 0.003
        for point in points:
            if not _add_viewer_geom(
                scene,
                mujoco.mjtGeom.mjGEOM_SPHERE,
                size=np.array([point_radius, 0.0, 0.0], dtype=np.float64),
                pos=point,
                rgba=point_rgba,
            ):
                break
        return

    for a, b, c in hull.triangles:
        if not _add_viewer_triangle(scene, hull.vertices[a], hull.vertices[b], hull.vertices[c], face_rgba):
            break

    for start_index, end_index in hull.edges:
        if not _add_viewer_connector(
            scene,
            mujoco.mjtGeom.mjGEOM_LINE,
            width=3.0,
            start=hull.vertices[start_index],
            end=hull.vertices[end_index],
            rgba=edge_rgba,
        ):
            break

    hull_span = np.ptp(hull.vertices, axis=0)
    point_radius = max(float(np.max(hull_span)) * 0.006, 0.002)
    vertex_radius = point_radius * 1.4
    for vertex in hull.vertices:
        if not _add_viewer_geom(
            scene,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([vertex_radius, 0.0, 0.0], dtype=np.float64),
            pos=vertex,
            rgba=vertex_rgba,
        ):
            break

    for point in points:
        if not _add_viewer_geom(
            scene,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([point_radius, 0.0, 0.0], dtype=np.float64),
            pos=point,
            rgba=point_rgba,
        ):
            break


def _draw_dual_workspace_hulls(
    scene,
    hulls: list[WorkspaceHull],
    points_by_arm: list[np.ndarray],
    internal_boxes: list[InternalWorkspaceBox],
) -> None:
    scene.ngeom = 0
    palettes = [
        {
            "face_rgba": np.array([0.0, 0.52, 1.0, 0.20], dtype=np.float32),
            "edge_rgba": np.array([0.0, 0.24, 0.95, 0.95], dtype=np.float32),
            "vertex_rgba": np.array([0.0, 0.68, 1.0, 0.85], dtype=np.float32),
            "point_rgba": np.array([1.0, 0.72, 0.1, 0.78], dtype=np.float32),
            "box_rgba": np.array([0.1, 0.85, 0.35, 0.22], dtype=np.float32),
        },
        {
            "face_rgba": np.array([1.0, 0.28, 0.20, 0.18], dtype=np.float32),
            "edge_rgba": np.array([0.95, 0.12, 0.08, 0.92], dtype=np.float32),
            "vertex_rgba": np.array([1.0, 0.35, 0.18, 0.82], dtype=np.float32),
            "point_rgba": np.array([0.2, 1.0, 0.72, 0.72], dtype=np.float32),
            "box_rgba": np.array([0.95, 0.45, 0.1, 0.20], dtype=np.float32),
        },
    ]
    for arm, hull in enumerate(hulls):
        _draw_workspace_hull_layer(
            scene,
            hull,
            points_by_arm[arm],
            internal_boxes[arm],
            **palettes[arm % len(palettes)],
        )


def _show_monte_carlo_workspace_viewer(
    env,
    *,
    points: np.ndarray | list[np.ndarray],
    pos_stats: RangeSnapshot | list[RangeSnapshot],
    last_qpos: np.ndarray,
    hull: WorkspaceHull | list[WorkspaceHull] | None = None,
    internal_box: InternalWorkspaceBox | list[InternalWorkspaceBox] | None = None,
    max_visual_points: int = DEFAULT_MC_MAX_VIS_POINTS,
    max_hull_points: int = DEFAULT_MC_MAX_HULL_POINTS,
) -> None:
    if not VIEWER_AVAILABLE:
        print("[MC Viewer] MuJoCo viewer 不可用，跳过窗口显示。")
        return
    points_by_arm = points if isinstance(points, list) else [points]
    pos_stats_by_arm = pos_stats if isinstance(pos_stats, list) else [pos_stats]
    if len(points_by_arm) == 0 or any(len(arm_points) == 0 for arm_points in points_by_arm):
        print("[MC Viewer] 没有采样点，跳过窗口显示。")
        return

    visual_points_by_arm = [
        select_visualization_points(arm_points, max_visual_points)
        for arm_points in points_by_arm
    ]
    if hull is None:
        hulls = [
            compute_workspace_hull(select_visualization_points(arm_points, max_hull_points))
            for arm_points in points_by_arm
        ]
    else:
        hulls = hull if isinstance(hull, list) else [hull]
    if internal_box is None:
        internal_boxes = [compute_largest_internal_workspace_box(arm_hull) for arm_hull in hulls]
    else:
        internal_boxes = internal_box if isinstance(internal_box, list) else [internal_box]

    env.set_qpos(last_qpos)
    env.set_qvel(np.zeros_like(last_qpos))
    env.forward()
    target_pos = []
    for arm, box in enumerate(internal_boxes):
        arm_hull = hulls[arm]
        arm_stats = pos_stats_by_arm[arm]
        target_pos.append(
            box.center if not box.is_empty else arm_hull.center if not arm_hull.is_empty else arm_stats.mean
        )
    env.set_all_target_poses_base(np.asarray(target_pos, dtype=np.float64))

    if len(hulls) == 1 and hulls[0].is_empty:
        print("[MC Viewer] 凸包点云退化，回退显示范围盒子。关闭窗口后程序退出。")
    else:
        print(
            "[MC Viewer] 打开 MuJoCo 窗口：蓝色为左臂工作空间，橙色为右臂工作空间，"
            "透明长方体为各自内部最大可输入 pos 范围。关闭窗口后程序退出。"
        )
    sys.stdout.flush()
    with launch_passive_viewer(env.model, env.data) as viewer:
        while viewer.is_running():
            lock = viewer.lock() if hasattr(viewer, "lock") else nullcontext()
            with lock:
                if hasattr(viewer, "user_scn"):
                    if len(hulls) == 1:
                        _draw_workspace_hull(
                            viewer.user_scn,
                            hulls[0],
                            visual_points_by_arm[0],
                            internal_boxes[0],
                        )
                    else:
                        _draw_dual_workspace_hulls(
                            viewer.user_scn,
                            hulls,
                            visual_points_by_arm,
                            internal_boxes,
                        )
            viewer.sync()
            time.sleep(1.0 / 30.0)

