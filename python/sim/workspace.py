"""Monte Carlo workspace geometry and range helpers."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


DEFAULT_MC_SAMPLES = 500000
DEFAULT_MC_PROGRESS_INTERVAL = 10000
DEFAULT_MC_FALLBACK_LIMIT = np.pi
DEFAULT_MC_MAX_VIS_POINTS = 2000
DEFAULT_MC_MAX_HULL_POINTS = 50000
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


@dataclass(frozen=True)
class MonteCarloWorkspaceResult:
    samples: int
    seed: int | None
    joint_lower: np.ndarray
    joint_upper: np.ndarray
    points: list[np.ndarray]
    quats: list[np.ndarray]
    pos_stats: list[RangeSnapshot]
    quat_stats: list[RangeSnapshot]
    quat_norm_stats: list[RangeSnapshot]
    workspace_hulls: list[WorkspaceHull]
    internal_boxes: list[InternalWorkspaceBox]
    hull_point_count: int
    last_qpos: np.ndarray
    report_text: str


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


def compute_internal_workspace_box_intersection(boxes: list[InternalWorkspaceBox]) -> InternalWorkspaceBox:
    """Compute the common axis-aligned safe range shared by internal workspace boxes."""
    if not boxes or any(box.is_empty for box in boxes):
        return _empty_internal_workspace_box()
    lower = np.max(np.asarray([box.lower for box in boxes], dtype=np.float64), axis=0)
    upper = np.min(np.asarray([box.upper for box in boxes], dtype=np.float64), axis=0)
    span = upper - lower
    if lower.shape != (3,) or upper.shape != (3,) or np.any(span <= 0.0):
        return _empty_internal_workspace_box()
    center = 0.5 * (lower + upper)
    half_size = 0.5 * span
    volume = float(np.prod(span))
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
