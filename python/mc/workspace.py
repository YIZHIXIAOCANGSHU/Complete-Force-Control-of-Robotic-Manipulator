"""Monte Carlo workspace range checks for the AM-D02 MuJoCo model."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from contextlib import nullcontext

import numpy as np

from config import Config
from common.gravity_backend import GravityCompTool
from common.mujoco_viewer import VIEWER_AVAILABLE, launch_passive_viewer


DEFAULT_MC_SAMPLES = 20000
DEFAULT_MC_PROGRESS_INTERVAL = 100
DEFAULT_MC_FALLBACK_LIMIT = np.pi
DEFAULT_MC_MAX_VIS_POINTS = 2000
DEFAULT_MC_MAX_HULL_POINTS = 6000
WORKSPACE_PADDING_RATIO = 0.08
WORKSPACE_MIN_HALF_SIZE = 0.01


@dataclass(frozen=True)
class RangeSnapshot:
    count: int
    minimum: np.ndarray
    maximum: np.ndarray
    mean: np.ndarray
    last: np.ndarray

    @property
    def span(self) -> np.ndarray:
        return self.maximum - self.minimum


class RangeAccumulator:
    """Streaming min/max/mean accumulator for fixed-width numeric vectors."""

    def __init__(self, width: int) -> None:
        self._width = width
        self._count = 0
        self._minimum = np.full(width, np.inf, dtype=np.float64)
        self._maximum = np.full(width, -np.inf, dtype=np.float64)
        self._mean = np.zeros(width, dtype=np.float64)
        self._last = np.zeros(width, dtype=np.float64)

    @property
    def count(self) -> int:
        return self._count

    def update(self, values: np.ndarray | list[float] | tuple[float, ...]) -> None:
        vector = np.asarray(values, dtype=np.float64)
        if vector.shape != (self._width,):
            raise ValueError(f"expected vector shape ({self._width},), got {vector.shape}")
        if not np.all(np.isfinite(vector)):
            raise ValueError(f"range sample contains non-finite values: {vector}")

        self._count += 1
        self._last = vector.copy()
        self._minimum = np.minimum(self._minimum, vector)
        self._maximum = np.maximum(self._maximum, vector)
        self._mean += (vector - self._mean) / self._count

    def snapshot(self) -> RangeSnapshot:
        if self._count == 0:
            raise ValueError("cannot snapshot an empty accumulator")
        return RangeSnapshot(
            count=self._count,
            minimum=self._minimum.copy(),
            maximum=self._maximum.copy(),
            mean=self._mean.copy(),
            last=self._last.copy(),
        )


@dataclass(frozen=True)
class WorkspaceHull:
    vertices: np.ndarray
    triangles: np.ndarray
    edges: list[tuple[int, int]]
    center: np.ndarray
    halfspace_normals: np.ndarray = field(
        default_factory=lambda: np.empty((0, 3), dtype=np.float64)
    )
    halfspace_offsets: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )

    @property
    def is_empty(self) -> bool:
        return len(self.vertices) == 0 or len(self.triangles) == 0


@dataclass(frozen=True)
class InternalWorkspaceBox:
    center: np.ndarray
    half_size: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    volume: float

    @property
    def is_empty(self) -> bool:
        return self.volume <= 0.0 or not np.all(self.half_size > 0.0)


def _write_ready_file(ready_file: str | None) -> None:
    if ready_file is None:
        return
    with open(ready_file, "w", encoding="utf-8") as file_obj:
        file_obj.write("ready\n")


def _create_sim_env():
    try:
        from sim.env import MujocoSimEnv
    except ModuleNotFoundError as exc:
        if exc.name == "mujoco":
            raise RuntimeError(
                "缺少 Python 依赖 mujoco，请先执行 "
                "`python3 -m pip install -r python/requirements.txt`。"
            ) from exc
        raise

    return MujocoSimEnv()


def resolve_sampling_bounds(
    joint_lower: np.ndarray,
    joint_upper: np.ndarray,
    fallback_limit: float = DEFAULT_MC_FALLBACK_LIMIT,
) -> tuple[np.ndarray, np.ndarray]:
    lower = np.asarray(joint_lower, dtype=np.float64).copy()
    upper = np.asarray(joint_upper, dtype=np.float64).copy()
    if lower.shape != upper.shape:
        raise ValueError(f"joint limit shape mismatch: {lower.shape} vs {upper.shape}")
    if lower.ndim != 1:
        raise ValueError("joint limits must be one-dimensional")
    if fallback_limit <= 0.0 or not np.isfinite(fallback_limit):
        raise ValueError("fallback_limit must be a positive finite value")

    lower[~np.isfinite(lower)] = -fallback_limit
    upper[~np.isfinite(upper)] = fallback_limit
    invalid = lower >= upper
    if np.any(invalid):
        indices = np.flatnonzero(invalid).tolist()
        raise ValueError(f"invalid joint sampling bounds at indices {indices}")
    return lower, upper


def compute_workspace_bounds(
    minimum: np.ndarray,
    maximum: np.ndarray,
    *,
    padding_ratio: float = WORKSPACE_PADDING_RATIO,
    min_half_size: float = WORKSPACE_MIN_HALF_SIZE,
) -> tuple[np.ndarray, np.ndarray]:
    lower = np.asarray(minimum, dtype=np.float64)
    upper = np.asarray(maximum, dtype=np.float64)
    if lower.shape != (3,) or upper.shape != (3,):
        raise ValueError(f"workspace bounds must be 3D vectors, got {lower.shape} and {upper.shape}")
    if padding_ratio < 0.0:
        raise ValueError("padding_ratio 不能为负数")
    if min_half_size <= 0.0:
        raise ValueError("min_half_size 必须为正数")

    center = 0.5 * (lower + upper)
    half_size = 0.5 * (upper - lower)
    half_size = half_size * (1.0 + padding_ratio)
    half_size = np.maximum(half_size, min_half_size)
    return center, half_size


def select_visualization_points(points: np.ndarray, max_points: int = DEFAULT_MC_MAX_VIS_POINTS) -> np.ndarray:
    samples = np.asarray(points, dtype=np.float64)
    if samples.ndim != 2 or samples.shape[1] != 3:
        raise ValueError(f"可视化点必须是 (N, 3) 形状，当前为 {samples.shape}")
    if max_points <= 0 or len(samples) <= max_points:
        return samples.copy()

    indices = np.linspace(0, len(samples) - 1, max_points, dtype=np.int64)
    return samples[indices].copy()


def compute_workspace_hull(points: np.ndarray) -> WorkspaceHull:
    samples = np.asarray(points, dtype=np.float64)
    empty = WorkspaceHull(
        vertices=np.empty((0, 3), dtype=np.float64),
        triangles=np.empty((0, 3), dtype=np.int32),
        edges=[],
        center=np.zeros(3, dtype=np.float64),
        halfspace_normals=np.empty((0, 3), dtype=np.float64),
        halfspace_offsets=np.empty(0, dtype=np.float64),
    )
    if samples.ndim != 2 or samples.shape[1] != 3:
        raise ValueError(f"工作空间凸包点必须是 (N, 3) 形状，当前为 {samples.shape}")
    if len(samples) < 4:
        return empty
    if not np.all(np.isfinite(samples)):
        raise ValueError("工作空间凸包点包含非有限数值")
    if np.linalg.matrix_rank(samples - np.mean(samples, axis=0), tol=1e-9) < 3:
        return empty

    try:
        from scipy.spatial import ConvexHull, QhullError
    except ImportError:
        return empty

    try:
        hull = ConvexHull(samples, qhull_options="QJ")
    except QhullError:
        return empty

    hull_point_indices = np.asarray(hull.vertices, dtype=np.int64)
    vertex_lookup = {int(point_index): vertex_index for vertex_index, point_index in enumerate(hull_point_indices)}
    triangles = []
    for simplex in np.asarray(hull.simplices, dtype=np.int64):
        try:
            triangles.append([vertex_lookup[int(point_index)] for point_index in simplex])
        except KeyError:
            continue

    if not triangles:
        return empty

    edge_set: set[tuple[int, int]] = set()
    for a, b, c in triangles:
        edge_set.add(tuple(sorted((a, b))))
        edge_set.add(tuple(sorted((b, c))))
        edge_set.add(tuple(sorted((c, a))))

    vertices = samples[hull_point_indices].copy()
    return WorkspaceHull(
        vertices=vertices,
        triangles=np.asarray(triangles, dtype=np.int32),
        edges=sorted(edge_set),
        center=np.mean(vertices, axis=0),
        halfspace_normals=np.asarray(hull.equations[:, :3], dtype=np.float64),
        halfspace_offsets=np.asarray(hull.equations[:, 3], dtype=np.float64),
    )


def _empty_internal_workspace_box() -> InternalWorkspaceBox:
    zeros = np.zeros(3, dtype=np.float64)
    return InternalWorkspaceBox(
        center=zeros.copy(),
        half_size=zeros.copy(),
        lower=zeros.copy(),
        upper=zeros.copy(),
        volume=0.0,
    )


def compute_largest_internal_workspace_box(hull: WorkspaceHull) -> InternalWorkspaceBox:
    """Find the largest axis-aligned box fully contained in a convex workspace hull."""
    if hull.is_empty or len(hull.halfspace_normals) == 0:
        return _empty_internal_workspace_box()

    normals = np.asarray(hull.halfspace_normals, dtype=np.float64)
    offsets = np.asarray(hull.halfspace_offsets, dtype=np.float64)
    vertices = np.asarray(hull.vertices, dtype=np.float64)
    if normals.ndim != 2 or normals.shape[1] != 3:
        raise ValueError(f"hull halfspace normals must have shape (N, 3), got {normals.shape}")
    if offsets.shape != (normals.shape[0],):
        raise ValueError(f"hull halfspace offsets must have shape ({normals.shape[0]},), got {offsets.shape}")
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) == 0:
        return _empty_internal_workspace_box()
    if not (np.all(np.isfinite(normals)) and np.all(np.isfinite(offsets)) and np.all(np.isfinite(vertices))):
        raise ValueError("工作空间凸包包含非有限数值")

    abs_normals = np.abs(normals)
    vertex_min = np.min(vertices, axis=0)
    vertex_max = np.max(vertices, axis=0)
    span = np.maximum(vertex_max - vertex_min, 0.0)
    center0 = np.asarray(hull.center, dtype=np.float64)
    if center0.shape != (3,) or not np.all(np.isfinite(center0)):
        center0 = 0.5 * (vertex_min + vertex_max)
    center0 = np.clip(center0, vertex_min, vertex_max)

    slack0 = -offsets - normals @ center0
    denominator = np.sum(abs_normals, axis=1)
    feasible_scale = np.min(slack0 / np.maximum(denominator, 1e-12))
    half0 = np.minimum(span * 0.25, max(float(feasible_scale) * 0.5, 0.0))
    if np.any(half0 <= 1e-10):
        half0 = np.maximum(span * 1e-3, 1e-8)

    # Variables are [cx, cy, cz, hx, hy, hz]. A box is inside a halfspace when
    # n.c + |n|.h + offset <= 0 for every hull plane.
    constraint_matrix = np.hstack((normals, abs_normals))
    result = _maximize_log_box_half_size(
        constraint_matrix,
        -offsets,
        np.concatenate((center0, half0)),
    )
    if result is None:
        return _empty_internal_workspace_box()

    center = result[:3].astype(np.float64, copy=True)
    half_size = result[3:6].astype(np.float64, copy=True)
    half_size = _expand_box_half_size_at_center(normals, offsets, center, half_size)
    residual = normals @ center + abs_normals @ half_size + offsets
    if np.max(residual) > 1e-6:
        half_size *= max(0.0, 1.0 - np.max(residual) / max(float(np.max(abs_normals @ half_size)), 1e-12))
    if np.any(half_size <= 1e-9):
        return _empty_internal_workspace_box()

    lower = center - half_size
    upper = center + half_size
    volume = float(np.prod(2.0 * half_size))
    return InternalWorkspaceBox(
        center=center,
        half_size=half_size,
        lower=lower,
        upper=upper,
        volume=volume,
    )


def _expand_box_half_size_at_center(
    normals: np.ndarray,
    offsets: np.ndarray,
    center: np.ndarray,
    half_size: np.ndarray,
) -> np.ndarray:
    expanded = np.asarray(half_size, dtype=np.float64).copy()
    abs_normals = np.abs(np.asarray(normals, dtype=np.float64))
    remaining = -np.asarray(offsets, dtype=np.float64) - np.asarray(normals, dtype=np.float64) @ center
    if np.any(remaining <= 0.0):
        return expanded

    for axis in range(3):
        other_axes = [idx for idx in range(3) if idx != axis]
        base = abs_normals[:, other_axes] @ expanded[other_axes]
        candidates = remaining - base
        relevant = abs_normals[:, axis] > 1e-12
        if np.any(relevant):
            expanded[axis] = max(
                expanded[axis],
                float(np.min(candidates[relevant] / abs_normals[relevant, axis])),
            )
            expanded[axis] = max(expanded[axis], 0.0)
    return expanded


def _maximize_log_box_half_size(
    constraint_matrix: np.ndarray,
    constraint_upper: np.ndarray,
    initial: np.ndarray,
) -> np.ndarray | None:
    x = np.asarray(initial, dtype=np.float64).copy()
    a = np.asarray(constraint_matrix, dtype=np.float64)
    upper = np.asarray(constraint_upper, dtype=np.float64)
    if x.shape != (6,) or a.ndim != 2 or a.shape[1] != 6 or upper.shape != (a.shape[0],):
        raise ValueError("内部长方体优化输入形状无效")

    def is_strictly_feasible(values: np.ndarray) -> bool:
        return bool(np.all(values[3:6] > 0.0) and np.all(upper - a @ values > 0.0))

    if not is_strictly_feasible(x):
        return None

    identity_floor = np.eye(6, dtype=np.float64) * 1e-12
    for barrier_weight in (1.0, 10.0, 100.0, 1_000.0, 10_000.0, 100_000.0, 1_000_000.0):
        for _ in range(80):
            half_size = x[3:6]
            slack = upper - a @ x
            if np.any(half_size <= 0.0) or np.any(slack <= 0.0):
                return None

            gradient = a.T @ (1.0 / slack)
            gradient[3:6] -= barrier_weight / half_size
            weighted_a = a / slack[:, np.newaxis]
            hessian = weighted_a.T @ weighted_a
            hessian[3:6, 3:6] += np.diag(barrier_weight / (half_size * half_size))
            hessian += identity_floor

            try:
                step = np.linalg.solve(hessian, -gradient)
            except np.linalg.LinAlgError:
                step = np.linalg.lstsq(hessian, -gradient, rcond=None)[0]

            decrement_sq = -float(gradient @ step)
            if decrement_sq <= 1e-14:
                break

            current_value = _box_barrier_objective(x, a, upper, barrier_weight)
            step_scale = 1.0
            directional_derivative = float(gradient @ step)
            while step_scale > 1e-12:
                candidate = x + step_scale * step
                if is_strictly_feasible(candidate):
                    candidate_value = _box_barrier_objective(candidate, a, upper, barrier_weight)
                    if candidate_value <= current_value + 1e-4 * step_scale * directional_derivative:
                        x = candidate
                        break
                step_scale *= 0.5
            else:
                break
    return x


def _box_barrier_objective(
    values: np.ndarray,
    constraint_matrix: np.ndarray,
    constraint_upper: np.ndarray,
    barrier_weight: float,
) -> float:
    half_size = values[3:6]
    slack = constraint_upper - constraint_matrix @ values
    if np.any(half_size <= 0.0) or np.any(slack <= 0.0):
        return float("inf")
    return float(-barrier_weight * np.sum(np.log(half_size)) - np.sum(np.log(slack)))


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


def _show_monte_carlo_workspace_viewer(
    env,
    *,
    points: np.ndarray,
    pos_stats: RangeSnapshot,
    last_qpos: np.ndarray,
    hull: WorkspaceHull | None = None,
    internal_box: InternalWorkspaceBox | None = None,
    max_visual_points: int = DEFAULT_MC_MAX_VIS_POINTS,
    max_hull_points: int = DEFAULT_MC_MAX_HULL_POINTS,
) -> None:
    if not VIEWER_AVAILABLE:
        print("[MC Viewer] MuJoCo viewer 不可用，跳过窗口显示。")
        return
    if len(points) == 0:
        print("[MC Viewer] 没有采样点，跳过窗口显示。")
        return

    visual_points = select_visualization_points(points, max_visual_points)
    if hull is None:
        hull_points = select_visualization_points(points, max_hull_points)
        hull = compute_workspace_hull(hull_points)
    if internal_box is None:
        internal_box = compute_largest_internal_workspace_box(hull)

    env.set_qpos(last_qpos)
    env.set_qvel(np.zeros_like(last_qpos))
    env.forward()
    target_pos = internal_box.center if not internal_box.is_empty else hull.center if not hull.is_empty else pos_stats.mean
    env.set_target_pose(target_pos)

    if hull.is_empty:
        print("[MC Viewer] 凸包点云退化，回退显示范围盒子。关闭窗口后程序退出。")
    else:
        print(
            "[MC Viewer] 打开 MuJoCo 窗口：蓝色透明多面体为末端可达空间凸包，"
            f"绿色透明长方体为内部最大可输入 pos 范围，面数={len(hull.triangles)}, "
            f"顶点={len(hull.vertices)}, 黄色点为采样末端位置。关闭窗口后程序退出。"
        )
    sys.stdout.flush()
    with launch_passive_viewer(env.model, env.data) as viewer:
        while viewer.is_running():
            lock = viewer.lock() if hasattr(viewer, "lock") else nullcontext()
            with lock:
                if hasattr(viewer, "user_scn"):
                    _draw_workspace_hull(viewer.user_scn, hull, visual_points, internal_box)
            viewer.sync()
            time.sleep(1.0 / 30.0)


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
            "AM-D02 蒙特卡洛末端位姿范围检查",
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


def run_monte_carlo_range_check(
    *,
    samples: int = DEFAULT_MC_SAMPLES,
    seed: int | None = None,
    progress_interval: int = DEFAULT_MC_PROGRESS_INTERVAL,
    fallback_joint_limit: float = DEFAULT_MC_FALLBACK_LIMIT,
    show_viewer: bool = True,
    max_visual_points: int = DEFAULT_MC_MAX_VIS_POINTS,
    max_hull_points: int = DEFAULT_MC_MAX_HULL_POINTS,
) -> None:
    """Sample FK ranges through the C backend without starting UDP."""
    if samples <= 0:
        raise ValueError("samples 必须为正数")
    if progress_interval < 0:
        raise ValueError("progress_interval 不能为负数")
    if max_hull_points <= 0:
        raise ValueError("max_hull_points 必须为正数")

    print("=" * 60)
    print("      AM-D02 C 后端蒙特卡洛末端位置/四元数范围检查  ")
    print("=" * 60)
    sys.stdout.flush()

    env = _create_sim_env()
    env.reset(Config.HOME_QPOS)
    env.forward()
    joint_lower, joint_upper = resolve_sampling_bounds(
        env.joint_lower,
        env.joint_upper,
        fallback_limit=fallback_joint_limit,
    )

    rng = np.random.default_rng(seed)
    pos_stats = RangeAccumulator(3)
    quat_stats = RangeAccumulator(4)
    quat_norm_stats = RangeAccumulator(1)
    last_qpos = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    ee_points: list[np.ndarray] = []

    comp_tool = GravityCompTool()

    print(f"[MC] 采样数量={samples}, 随机种子={'随机' if seed is None else seed}")
    print("[MC] 使用 C 后端 serial_gravity_comp 做 FK 采样；MuJoCo 仅用于关节限位和结果可视化。")

    try:
        for index in range(samples):
            last_qpos = rng.uniform(joint_lower, joint_upper)

            pos, quat = comp_tool.compute_fk(last_qpos.tolist())
            pos = np.asarray(pos, dtype=np.float64)
            quat = np.asarray(quat, dtype=np.float64)
            pos_stats.update(pos)
            quat_stats.update(quat)
            quat_norm_stats.update(np.array([np.linalg.norm(quat)], dtype=np.float64))
            ee_points.append(pos)

            current = index + 1
            if progress_interval and (current % progress_interval == 0 or current == samples):
                sys.stdout.write(
                    "\r"
                    f"[MC] {current:>{len(str(samples))}}/{samples} "
                    f"末端位置={_format_vector(pos, precision=4)} "
                    f"末端四元数={_format_vector(quat, precision=4)}"
                )
                sys.stdout.flush()
        if progress_interval:
            sys.stdout.write("\n")
    except KeyboardInterrupt:
        if progress_interval:
            sys.stdout.write("\n")
        print("[MC] 用户中断，输出已经采集到的范围。")
    finally:
        comp_tool.close()

    if pos_stats.count == 0:
        print("[MC] 没有采到样本，结束。")
        return

    points_array = np.asarray(ee_points, dtype=np.float64)
    hull_points = select_visualization_points(points_array, max_hull_points)
    workspace_hull = compute_workspace_hull(hull_points)
    internal_box = compute_largest_internal_workspace_box(workspace_hull)

    print(
        _format_monte_carlo_report(
            samples=pos_stats.snapshot().count,
            seed=seed,
            joint_lower=joint_lower,
            joint_upper=joint_upper,
            pos_stats=pos_stats.snapshot(),
            quat_stats=quat_stats.snapshot(),
            quat_norm_stats=quat_norm_stats.snapshot(),
            internal_box=internal_box,
            hull_point_count=len(hull_points),
            last_qpos=last_qpos,
        )
    )

    if show_viewer:
        _show_monte_carlo_workspace_viewer(
            env,
            points=points_array,
            pos_stats=pos_stats.snapshot(),
            last_qpos=last_qpos,
            hull=workspace_hull,
            internal_box=internal_box,
            max_visual_points=max_visual_points,
            max_hull_points=max_hull_points,
        )
