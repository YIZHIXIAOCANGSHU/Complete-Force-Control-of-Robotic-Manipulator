"""UDP server that exposes the MuJoCo simulation to the C controller."""

from __future__ import annotations

import socket
import sys
import time
from dataclasses import dataclass, field
from contextlib import nullcontext

import numpy as np

from config import Config
from mujoco_viewer import VIEWER_AVAILABLE, launch_passive_viewer
import rerun_viz
from state_packets import STATE_PACKET_SIZE


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
        from sim_env import MujocoSimEnv
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
        raise ValueError("padding_ratio cannot be negative")
    if min_half_size <= 0.0:
        raise ValueError("min_half_size must be positive")

    center = 0.5 * (lower + upper)
    half_size = 0.5 * (upper - lower)
    half_size = half_size * (1.0 + padding_ratio)
    half_size = np.maximum(half_size, min_half_size)
    return center, half_size


def select_visualization_points(points: np.ndarray, max_points: int = DEFAULT_MC_MAX_VIS_POINTS) -> np.ndarray:
    samples = np.asarray(points, dtype=np.float64)
    if samples.ndim != 2 or samples.shape[1] != 3:
        raise ValueError(f"visualization points must have shape (N, 3), got {samples.shape}")
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
        raise ValueError(f"workspace hull points must have shape (N, 3), got {samples.shape}")
    if len(samples) < 4:
        return empty
    if not np.all(np.isfinite(samples)):
        raise ValueError("workspace hull points contain non-finite values")
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
        raise ValueError("workspace hull contains non-finite values")

    try:
        from scipy.optimize import Bounds, LinearConstraint, minimize
    except ImportError:
        return _empty_internal_workspace_box()

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
    linear_constraint = LinearConstraint(
        constraint_matrix,
        lb=np.full(len(offsets), -np.inf, dtype=np.float64),
        ub=-offsets,
    )
    lower_bounds = np.concatenate((vertex_min, np.full(3, 1e-10, dtype=np.float64)))
    upper_bounds = np.concatenate((vertex_max, np.maximum(span * 0.5, 1e-10)))
    bounds = Bounds(lower_bounds, upper_bounds)

    def objective(values: np.ndarray) -> float:
        half_size = np.maximum(values[3:6], 1e-12)
        return -float(np.sum(np.log(half_size)))

    result = minimize(
        objective,
        np.concatenate((center0, half0)),
        method="SLSQP",
        bounds=bounds,
        constraints=[linear_constraint],
        options={"ftol": 1e-10, "maxiter": 400, "disp": False},
    )
    if not result.success or not np.all(np.isfinite(result.x)):
        return _empty_internal_workspace_box()

    center = result.x[:3].astype(np.float64, copy=True)
    half_size = result.x[3:6].astype(np.float64, copy=True)
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


def _draw_workspace_hull(scene, hull: WorkspaceHull, points: np.ndarray) -> None:
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
    hull_points = select_visualization_points(points, max_hull_points)
    hull = compute_workspace_hull(hull_points)

    env.set_qpos(last_qpos)
    env.set_qvel(np.zeros_like(last_qpos))
    env.forward()
    env.set_target_pose(hull.center if not hull.is_empty else pos_stats.mean)

    if hull.is_empty:
        print("[MC Viewer] 凸包点云退化，回退显示范围盒子。关闭窗口后程序退出。")
    else:
        print(
            "[MC Viewer] 打开 MuJoCo 窗口：蓝色透明多面体为末端可达空间凸包，"
            f"面数={len(hull.triangles)}, 顶点={len(hull.vertices)}, 黄色点为采样末端位置。关闭窗口后程序退出。"
        )
    sys.stdout.flush()
    with launch_passive_viewer(env.model, env.data) as viewer:
        while viewer.is_running():
            lock = viewer.lock() if hasattr(viewer, "lock") else nullcontext()
            with lock:
                if hasattr(viewer, "user_scn"):
                    _draw_workspace_hull(viewer.user_scn, hull, visual_points)
            viewer.sync()
            time.sleep(1.0 / 30.0)


def _format_vector(values: np.ndarray, precision: int = 6) -> str:
    return "[" + ", ".join(f"{value:.{precision}f}" for value in values) + "]"


def _format_range_block(labels: list[str], snapshot: RangeSnapshot, unit: str = "") -> str:
    suffix = f" {unit}" if unit else ""
    lines = ["name        min           max           span          mean"]
    for label, lo, hi, span, mean in zip(
        labels,
        snapshot.minimum,
        snapshot.maximum,
        snapshot.span,
        snapshot.mean,
    ):
        lines.append(f"{label:<5} {lo:>12.6f} {hi:>12.6f} {span:>12.6f} {mean:>12.6f}{suffix}")
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
    last_qpos: np.ndarray,
) -> str:
    seed_text = "random" if seed is None else str(seed)
    return "\n".join(
        [
            "",
            "=" * 72,
            "AM-D02 Monte Carlo end-effector range check",
            "=" * 72,
            f"samples: {samples}",
            f"seed: {seed_text}",
            f"joint lower(rad): {_format_vector(joint_lower)}",
            f"joint upper(rad): {_format_vector(joint_upper)}",
            "",
            "End-effector position range:",
            _format_range_block(["x", "y", "z"], pos_stats, "m"),
            "",
            "End-effector quaternion range [w, x, y, z]:",
            _format_range_block(["w", "x", "y", "z"], quat_stats),
            "",
            "Quaternion norm range:",
            _format_range_block(["|q|"], quat_norm_stats),
            "",
            "Latest refreshed sample:",
            f"qpos(rad): {_format_vector(last_qpos)}",
            f"ee_pos(m): {_format_vector(pos_stats.last)}",
            f"ee_quat(wxyz): {_format_vector(quat_stats.last)}",
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
    """Use the normal sim environment to sample FK ranges without starting UDP."""
    if samples <= 0:
        raise ValueError("samples must be positive")
    if progress_interval < 0:
        raise ValueError("progress_interval cannot be negative")
    if max_hull_points <= 0:
        raise ValueError("max_hull_points must be positive")

    print("=" * 60)
    print("      AM-D02 Python MuJoCo Simulation Server       ")
    print("      Monte Carlo 末端位置/四元数范围检查          ")
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

    print(f"[MC] samples={samples}, seed={'random' if seed is None else seed}")
    print("[MC] 使用原 sim 的 MujocoSimEnv 做 FK 采样，按 Ctrl+C 可提前输出已采样范围。")

    try:
        for index in range(samples):
            last_qpos = rng.uniform(joint_lower, joint_upper)
            env.set_qpos(last_qpos)
            env.set_qvel(np.zeros_like(last_qpos))
            env.forward()

            pos = env.get_ee_pos()
            quat = env.get_ee_quat()
            pos_stats.update(pos)
            quat_stats.update(quat)
            quat_norm_stats.update(np.array([np.linalg.norm(quat)], dtype=np.float64))
            ee_points.append(pos)

            current = index + 1
            if progress_interval and (current % progress_interval == 0 or current == samples):
                sys.stdout.write(
                    "\r"
                    f"[MC] {current:>{len(str(samples))}}/{samples} "
                    f"ee_pos={_format_vector(pos, precision=4)} "
                    f"ee_quat={_format_vector(quat, precision=4)}"
                )
                sys.stdout.flush()
        if progress_interval:
            sys.stdout.write("\n")
    except KeyboardInterrupt:
        if progress_interval:
            sys.stdout.write("\n")
        print("[MC] 用户中断，输出已经采集到的范围。")

    if pos_stats.count == 0:
        print("[MC] 没有采到样本，结束。")
        return

    print(
        _format_monte_carlo_report(
            samples=pos_stats.snapshot().count,
            seed=seed,
            joint_lower=joint_lower,
            joint_upper=joint_upper,
            pos_stats=pos_stats.snapshot(),
            quat_stats=quat_stats.snapshot(),
            quat_norm_stats=quat_norm_stats.snapshot(),
            last_qpos=last_qpos,
        )
    )

    if show_viewer:
        _show_monte_carlo_workspace_viewer(
            env,
            points=np.asarray(ee_points, dtype=np.float64),
            pos_stats=pos_stats.snapshot(),
            last_qpos=last_qpos,
            max_visual_points=max_visual_points,
            max_hull_points=max_hull_points,
        )


def run_udp_server(ready_file: str | None = None) -> None:
    print("=" * 60)
    print("      AM-D02 Python UDP Simulation Server       ")
    print("   允许独立的外部 C 语言控制器通过 Socket 接入  ")
    print("=" * 60)

    if Config.ENABLE_RERUN:
        rerun_viz.init_rerun()
        rerun_viz.setup_realtime_styles()
        time.sleep(0.5)

    env = _create_sim_env()

    env.reset(Config.INIT_QPOS)
    env.forward()
    box_init_pos = env.get_ee_pos().copy()
    box_init_quat = env.get_ee_quat().copy()
    print(f"[Server] INIT_QPOS FK => box init pos: {box_init_pos}")

    env.reset(Config.HOME_QPOS)
    env.forward()
    env.set_target_pose(box_init_pos, box_init_quat)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_addr = ("0.0.0.0", 9876)
    sock.bind(server_addr)
    sock.settimeout(0.01)
    print(f"[UDP Server] 监听端口 {server_addr[1]}...")

    viewer = None
    if VIEWER_AVAILABLE and Config.ENABLE_VIEWER:
        viewer = launch_passive_viewer(env.model, env.data)
        viewer.sync()
        print("[UDP Server] 可视化窗口已打开。此时等待 C 端客户端发送请求...")

    _write_ready_file(ready_file)

    step_count = 0
    state_packet = np.empty(STATE_PACKET_SIZE, dtype=np.float64)
    state_packet_view = memoryview(state_packet).cast("B")

    try:
        while True:
            if viewer and not viewer.is_running():
                print("[UDP Server] Viewer is closed. Exiting.")
                break

            try:
                data, addr = sock.recvfrom(1024)

                if data == b"INIT":
                    print(f"[UDP Server] Client {addr} connected! (INIT received)")
                    env.write_state_packet(state_packet)
                    sock.sendto(state_packet_view, addr)
                    continue

                if len(data) != 56:
                    print(f"[UDP Server] 收到未知长度的数据: {len(data)} bytes")
                    continue

                tau = np.frombuffer(data, dtype="<f8", count=Config.NUM_JOINTS)
                clipped_tau = env.clip_torque(tau)
                t_start = time.perf_counter()
                env.apply_torque(clipped_tau)
                env.step()
                clipped = env.enforce_joint_limits()
                cycle_time_ms = (time.perf_counter() - t_start) * 1000.0

                if clipped:
                    env.forward()
                if viewer:
                    viewer.sync()

                if Config.ENABLE_RERUN:
                    q, qd, pos_current, quat_current, pos_desired, quat_desired = env.get_state_snapshot()
                    rerun_viz.log_realtime_step(
                        t=step_count * Config.DT,
                        pos_actual=pos_current,
                        pos_desired=pos_desired,
                        quat_actual=quat_current,
                        quat_desired=quat_desired,
                        tau_total=clipped_tau,
                        cycle_time=cycle_time_ms,
                        q=q,
                        qd=qd,
                    )

                env.write_state_packet(state_packet)
                sock.sendto(state_packet_view, addr)
                step_count += 1

            except socket.timeout:
                if viewer:
                    viewer.sync()
                continue

    except KeyboardInterrupt:
        print("\n[UDP Server] Interrupted by user. Exiting...")
    finally:
        if viewer:
            viewer.close()
        sock.close()
