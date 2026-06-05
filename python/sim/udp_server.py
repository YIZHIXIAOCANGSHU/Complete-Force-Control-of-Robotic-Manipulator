"""UDP server that exposes the MuJoCo simulation to the C controller."""

from __future__ import annotations

import csv
from datetime import datetime
import json
from pathlib import Path
import socket
import sys
import time
import threading
import queue
from dataclasses import dataclass, field
from contextlib import nullcontext

import numpy as np

from config import Config
from mujoco_viewer import VIEWER_AVAILABLE, launch_passive_viewer
import rerun_viz
from sim.state_packets import CONTROL_INPUT_PACKET_SIZE, TORQUE_OUTPUT_PACKET_SIZE


DEFAULT_MC_SAMPLES = 20000
DEFAULT_MC_PROGRESS_INTERVAL = 100
DEFAULT_MC_FALLBACK_LIMIT = np.pi
DEFAULT_MC_MAX_VIS_POINTS = 2000
DEFAULT_MC_MAX_HULL_POINTS = 6000
DEFAULT_REPORT_SAMPLES = 500000
DEFAULT_REPORT_PROGRESS_INTERVAL = 10000
DEFAULT_REPORT_CONTROL_DURATION_S = 10.0
DEFAULT_REPORT_CONTROL_LOG_STRIDE = 10
DEFAULT_REPORT_MAX_SVG_POINTS = 1800
WORKSPACE_PADDING_RATIO = 0.08
WORKSPACE_MIN_HALF_SIZE = 0.01


def _should_log_sim_rerun_step(step_count: int) -> bool:
    stride = max(1, int(Config.RERUN_LOG_STRIDE))
    return stride <= 1 or int(step_count) % stride == 0


def _sim_rerun_stats(rerun_logger) -> tuple[int, int]:
    if rerun_logger is None:
        return 0, 0
    return (
        int(getattr(rerun_logger, "overwritten_payload_count", 0)),
        int(getattr(rerun_logger, "dropped_payload_count", 0)),
    )


def _sim_viewer_stats(viewer_worker) -> dict[str, float | int]:
    if viewer_worker is None or not hasattr(viewer_worker, "stats_snapshot"):
        return {
            "viewer_sync_count": 0,
            "viewer_skip_count": 0,
            "viewer_sync_ms": 0.0,
            "viewer_lock_wait_ms": 0.0,
        }
    return viewer_worker.stats_snapshot()


def _maybe_print_sim_stats(
    *,
    now_s: float,
    last_print_s: float | None,
    loop_hz: float,
    loop_ms: float,
    mujoco_step_ms: float,
    state_packet_ms: float,
    rerun_overwrite_count: int,
    rerun_drop_count: int,
    viewer_stats: dict[str, float | int],
    socket_timeout_count: int,
) -> float | None:
    if Config.SIM_STATS_INTERVAL_S <= 0.0:
        return last_print_s
    if last_print_s is not None and now_s - last_print_s < Config.SIM_STATS_INTERVAL_S:
        return last_print_s
    print(
        "[Sim Stats] "
        f"target={Config.SIM_TARGET_HZ:.0f}Hz "
        f"loop={loop_hz:.1f}Hz/{loop_ms:.3f}ms "
        f"mujoco_step={mujoco_step_ms:.3f}ms "
        f"state_packet={state_packet_ms:.3f}ms "
        f"rerun_overwrites={rerun_overwrite_count} "
        f"rerun_drops={rerun_drop_count} "
        f"viewer_sync={int(viewer_stats['viewer_sync_count'])} "
        f"viewer_skip={int(viewer_stats['viewer_skip_count'])} "
        f"viewer_sync_ms={float(viewer_stats['viewer_sync_ms']):.3f} "
        f"viewer_lock_wait_ms={float(viewer_stats['viewer_lock_wait_ms']):.3f} "
        f"socket_timeouts={socket_timeout_count}"
    )
    return now_s


class SimRerunLogger:
    """Async Rerun logger for keeping UDP/MuJoCo stepping off the logging path."""

    def __init__(
        self,
        queue_size: int | None = None,
        log_fn=None,
        *,
        max_hz: float | None = None,
        perf_counter=time.perf_counter,
        sleep_fn=time.sleep,
    ) -> None:
        self._queue = queue.Queue(maxsize=max(1, int(queue_size or Config.SIM_RERUN_QUEUE_SIZE)))
        self._log_fn = rerun_viz.log_realtime_step if log_fn is None else log_fn
        self._period_s = 0.0
        effective_max_hz = Config.RERUN_MAX_HZ if max_hz is None else float(max_hz)
        if effective_max_hz > 0.0:
            self._period_s = 1.0 / effective_max_hz
        self._last_log_time: float | None = None
        self._perf_counter = perf_counter
        self._sleep_fn = sleep_fn
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._stats_lock = threading.Lock()
        self._overwritten_payload_count = 0
        self._dropped_payload_count = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._worker, name="sim-rerun-logger", daemon=True)
        self._thread.start()

    def log_step(self, **payload) -> None:
        if self._stop_event.is_set():
            return
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            try:
                self._queue.get_nowait()
                with self._stats_lock:
                    self._overwritten_payload_count += 1
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(payload)
            except queue.Full:
                with self._stats_lock:
                    self._dropped_payload_count += 1

    @property
    def overwritten_payload_count(self) -> int:
        with self._stats_lock:
            return int(self._overwritten_payload_count)

    @property
    def dropped_payload_count(self) -> int:
        with self._stats_lock:
            return int(self._dropped_payload_count)

    def _worker(self) -> None:
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                payload = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            while True:
                try:
                    payload = self._queue.get_nowait()
                except queue.Empty:
                    break
            if self._last_log_time is not None and self._period_s > 0.0:
                elapsed_s = self._perf_counter() - self._last_log_time
                if elapsed_s < self._period_s:
                    self._sleep_fn(self._period_s - elapsed_s)
            self._log_fn(**payload)
            self._last_log_time = self._perf_counter()

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


class SimViewerSyncWorker:
    """Refresh the passive MuJoCo viewer outside the UDP/control hot path."""

    def __init__(
        self,
        viewer,
        env_lock: threading.RLock,
        shutdown_event: threading.Event,
        *,
        fps: float | None = None,
        lock_timeout_s: float | None = None,
        sync_budget_ms: float | None = None,
        backoff_frames: int | None = None,
        perf_counter=time.perf_counter,
        sleep_fn=time.sleep,
    ) -> None:
        self.viewer = viewer
        self.env_lock = env_lock
        self.shutdown_event = shutdown_event
        self.fps = max(1.0, float(Config.SIM_VIEWER_FPS if fps is None else fps))
        self.lock_timeout_s = max(
            0.0,
            float(Config.SIM_VIEWER_LOCK_TIMEOUT_S if lock_timeout_s is None else lock_timeout_s),
        )
        self.sync_budget_ms = max(
            0.0,
            float(Config.SIM_VIEWER_SYNC_BUDGET_MS if sync_budget_ms is None else sync_budget_ms),
        )
        self.backoff_frames = max(
            0,
            int(Config.SIM_VIEWER_BACKOFF_FRAMES if backoff_frames is None else backoff_frames),
        )
        self._perf_counter = perf_counter
        self._sleep_fn = sleep_fn
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._stats_lock = threading.Lock()
        self._sync_count = 0
        self._skip_count = 0
        self._last_sync_ms = 0.0
        self._last_lock_wait_ms = 0.0
        self._backoff_remaining = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._worker, name="sim-viewer-sync", daemon=True)
        self._thread.start()

    def sync_once(self) -> bool:
        if hasattr(self.viewer, "is_running") and not self.viewer.is_running():
            self.shutdown_event.set()
            return False
        if self._backoff_remaining > 0:
            self._backoff_remaining -= 1
            with self._stats_lock:
                self._skip_count += 1
                self._last_lock_wait_ms = 0.0
            return True
        lock_wait_start = self._perf_counter()
        acquired = self.env_lock.acquire(timeout=self.lock_timeout_s)
        lock_wait_ms = max(0.0, (self._perf_counter() - lock_wait_start) * 1000.0)
        if not acquired:
            with self._stats_lock:
                self._skip_count += 1
                self._last_lock_wait_ms = lock_wait_ms
            return True
        sync_start = self._perf_counter()
        try:
            self.viewer.sync()
        finally:
            sync_ms = max(0.0, (self._perf_counter() - sync_start) * 1000.0)
            self.env_lock.release()
        with self._stats_lock:
            self._sync_count += 1
            self._last_sync_ms = sync_ms
            self._last_lock_wait_ms = lock_wait_ms
        if (
            self.sync_budget_ms > 0.0
            and self.backoff_frames > 0
            and sync_ms > self.sync_budget_ms
        ):
            self._backoff_remaining = self.backoff_frames
        return True

    def _worker(self) -> None:
        period_s = 1.0 / self.fps
        while not self._stop_event.is_set() and not self.shutdown_event.is_set():
            if not self.sync_once():
                break
            self._sleep_fn(period_s)

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def stats_snapshot(self) -> dict[str, float | int]:
        with self._stats_lock:
            return {
                "viewer_sync_count": int(self._sync_count),
                "viewer_skip_count": int(self._skip_count),
                "viewer_sync_ms": float(self._last_sync_ms),
                "viewer_lock_wait_ms": float(self._last_lock_wait_ms),
            }


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


class BodyJointGui:
    """Small Tk slider window for commanding the three Body0422 joints."""

    def __init__(self, env, env_lock=None) -> None:
        self._env = env
        self._env_lock = env_lock
        self._root = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        with self._env_context():
            self._pending_body_qpos = env.get_body_qpos()
        self._dirty = False

    def _env_context(self):
        return self._env_lock if self._env_lock is not None else nullcontext()

    def start(self) -> None:
        if not Config.ENABLE_BODY_GUI:
            return
        if not self._env.has_body_joints():
            print("[Body GUI] 当前模型未保留躯干关节，跳过 GUI。")
            return
        self._thread = threading.Thread(target=self._run, name="body-joint-gui", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._running = False
        if self._root is not None:
            try:
                self._root.after(0, self._root.destroy)
            except Exception:
                pass

    def apply_pending(self) -> bool:
        with self._lock:
            if not self._dirty:
                return False
            body_qpos = self._pending_body_qpos.copy()
            self._dirty = False
        with self._env_context():
            self._env.set_body_qpos(body_qpos)
            self._env.forward()
        return True

    def _run(self) -> None:
        try:
            import tkinter as tk
        except Exception as exc:
            print(f"[Body GUI] tkinter 不可用，跳过躯干滑条: {exc}")
            return

        self._running = True
        root = tk.Tk()
        self._root = root
        root.title("Body0422 躯干角度")
        root.geometry("360x210")
        root.protocol("WM_DELETE_WINDOW", self.close)

        names = ("Waist01", "Waist02", "Body0422")
        with self._env_context():
            initial = self._env.get_body_qpos()
        limits_deg = np.rad2deg(Config.BODY_JOINT_LIMITS_RAD)

        for i, name in enumerate(names):
            frame = tk.Frame(root)
            frame.pack(fill="x", padx=12, pady=8)
            label = tk.Label(frame, text=name, width=10, anchor="w")
            label.pack(side="left")
            value_label = tk.Label(frame, text=f"{np.rad2deg(initial[i]):.1f} deg", width=10)
            value_label.pack(side="right")

            def on_change(value: str, joint_index: int = i, output_label=value_label) -> None:
                angle_rad = np.deg2rad(float(value))
                with self._lock:
                    self._pending_body_qpos[joint_index] = angle_rad
                    self._dirty = True
                output_label.config(text=f"{float(value):.1f} deg")

            slider = tk.Scale(
                frame,
                from_=float(limits_deg[i, 0]),
                to=float(limits_deg[i, 1]),
                orient="horizontal",
                resolution=0.1,
                showvalue=False,
                command=on_change,
            )
            slider.set(float(np.rad2deg(initial[i])))
            slider.pack(side="left", fill="x", expand=True)

        try:
            root.mainloop()
        finally:
            self._running = False
            self._root = None


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


def _collect_monte_carlo_workspace(
    *,
    env,
    samples: int,
    seed: int | None,
    progress_interval: int,
    fallback_joint_limit: float = DEFAULT_MC_FALLBACK_LIMIT,
    max_hull_points: int = DEFAULT_MC_MAX_HULL_POINTS,
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

        current = index + 1
        if progress_interval and (current % progress_interval == 0 or current == samples):
            sys.stdout.write(
                "\r"
                f"[Report MC] {current:>{len(str(samples))}}/{samples} "
                f"左臂={_format_vector(pos[Config.LEFT_ARM], precision=4)} "
                f"右臂={_format_vector(pos[Config.RIGHT_ARM], precision=4)}"
            )
            sys.stdout.flush()
    if progress_interval:
        sys.stdout.write("\n")

    hull_points = [select_visualization_points(arm_points, max_hull_points) for arm_points in points]
    workspace_hulls = [compute_workspace_hull(arm_points) for arm_points in hull_points]
    internal_boxes = [compute_largest_internal_workspace_box(hull) for hull in workspace_hulls]
    pos_snapshots = [stats.snapshot() for stats in pos_stats]
    quat_snapshots = [stats.snapshot() for stats in quat_stats]
    quat_norm_snapshots = [stats.snapshot() for stats in quat_norm_stats]
    hull_point_count = len(hull_points[Config.LEFT_ARM])
    report_text = _format_dual_monte_carlo_report(
        samples=samples,
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
        samples=samples,
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

    env = _create_sim_env()
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
    last_qpos = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    ee_points: list[list[np.ndarray]] = [[] for _ in range(Config.NUM_ARMS)]
    ee_quats: list[list[np.ndarray]] = [[] for _ in range(Config.NUM_ARMS)]

    print(f"[MC] 采样数量={samples}, 随机种子={'随机' if seed is None else seed}")
    print("[MC] 使用原 sim 的 MujocoSimEnv 做双臂 FK 采样，按 Ctrl+C 可提前输出已采样范围。")

    try:
        for index in range(samples):
            last_qpos = rng.uniform(joint_lower, joint_upper)
            env.set_qpos(last_qpos)
            env.set_qvel(np.zeros_like(last_qpos))
            env.forward()

            pos = env.get_all_ee_pos()
            quat = env.get_all_ee_quat()
            for arm in range(Config.NUM_ARMS):
                pos_stats[arm].update(pos[arm])
                quat_stats[arm].update(quat[arm])
                quat_norm_stats[arm].update(np.array([np.linalg.norm(quat[arm])], dtype=np.float64))
                ee_points[arm].append(pos[arm])
                ee_quats[arm].append(quat[arm])

            current = index + 1
            if progress_interval and (current % progress_interval == 0 or current == samples):
                sys.stdout.write(
                    "\r"
                    f"[MC] {current:>{len(str(samples))}}/{samples} "
                    f"左臂={_format_vector(pos[Config.LEFT_ARM], precision=4)} "
                    f"右臂={_format_vector(pos[Config.RIGHT_ARM], precision=4)}"
                )
                sys.stdout.flush()
        if progress_interval:
            sys.stdout.write("\n")
    except KeyboardInterrupt:
        if progress_interval:
            sys.stdout.write("\n")
        print("[MC] 用户中断，输出已经采集到的范围。")

    if pos_stats[Config.LEFT_ARM].count == 0:
        print("[MC] 没有采到样本，结束。")
        return

    points_array = [np.asarray(points, dtype=np.float64) for points in ee_points]
    hull_points = [select_visualization_points(points, max_hull_points) for points in points_array]
    workspace_hulls = [compute_workspace_hull(points) for points in hull_points]
    internal_boxes = [compute_largest_internal_workspace_box(hull) for hull in workspace_hulls]
    pos_snapshots = [stats.snapshot() for stats in pos_stats]
    quat_snapshots = [stats.snapshot() for stats in quat_stats]
    quat_norm_snapshots = [stats.snapshot() for stats in quat_norm_stats]

    report_text = _format_dual_monte_carlo_report(
        samples=pos_snapshots[Config.LEFT_ARM].count,
        seed=seed,
        joint_lower=joint_lower,
        joint_upper=joint_upper,
        pos_stats=pos_snapshots,
        quat_stats=quat_snapshots,
        quat_norm_stats=quat_norm_snapshots,
        internal_boxes=internal_boxes,
        hull_point_count=len(hull_points[Config.LEFT_ARM]),
        last_qpos=last_qpos,
    )
    print(report_text)

    if output_dir is not None:
        _write_monte_carlo_report_assets(
            output_dir,
            samples=pos_snapshots[Config.LEFT_ARM].count,
            seed=seed,
            joint_lower=joint_lower,
            joint_upper=joint_upper,
            points=points_array,
            quats=[np.asarray(quats, dtype=np.float64) for quats in ee_quats],
            pos_stats=pos_snapshots,
            quat_stats=quat_snapshots,
            quat_norm_stats=quat_norm_snapshots,
            internal_boxes=internal_boxes,
            hull_point_count=len(hull_points[Config.LEFT_ARM]),
            report_text=report_text,
        )

    if show_viewer:
        _show_monte_carlo_workspace_viewer(
            env,
            points=points_array,
            pos_stats=pos_snapshots,
            last_qpos=last_qpos,
            hull=workspace_hulls,
            internal_box=internal_boxes,
            max_visual_points=max_visual_points,
            max_hull_points=max_hull_points,
        )


def run_udp_server(ready_file: str | None = None) -> None:
    print("=" * 60)
    print("      AM-DPBSURDF0422 Python UDP 仿真服务        ")
    print("   允许独立的外部 C 语言控制器通过 Socket 接入  ")
    print("=" * 60)

    rerun_logger = None
    if Config.ENABLE_RERUN:
        rerun_viz.init_rerun()
        rerun_viz.setup_realtime_styles()
        time.sleep(0.5)
        rerun_logger = SimRerunLogger()
        rerun_logger.start()

    env = _create_sim_env()
    env_lock = threading.RLock()
    shutdown_event = threading.Event()

    with env_lock:
        env.reset(Config.INIT_QPOS)
        env.forward()
        box_init_pos = env.get_all_ee_pos().copy()
        box_init_quat = env.get_all_ee_quat().copy()
    print(
        "[Server] INIT_QPOS 正向运动学 => "
        f"左臂初始目标位置: {box_init_pos[Config.LEFT_ARM]}, "
        f"右臂初始目标位置: {box_init_pos[Config.RIGHT_ARM]}"
    )

    with env_lock:
        env.reset(Config.HOME_QPOS)
        env.forward()
        env.set_all_target_poses_base(box_init_pos, box_init_quat)
    body_gui = BodyJointGui(env, env_lock=env_lock)
    body_gui.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_addr = ("0.0.0.0", 9876)
    sock.bind(server_addr)
    sock.settimeout(Config.SIM_UDP_TIMEOUT_S)
    print(f"[UDP Server] 监听端口 {server_addr[1]}...")

    viewer = None
    viewer_worker = None
    if VIEWER_AVAILABLE and Config.ENABLE_VIEWER:
        viewer = launch_passive_viewer(env.model, env.data)
        viewer_worker = SimViewerSyncWorker(viewer, env_lock, shutdown_event)
        viewer_worker.start()
        print("[UDP Server] 可视化窗口已打开。此时等待 C 端客户端发送请求...")

    _write_ready_file(ready_file)

    step_count = 0
    state_packet = np.empty(CONTROL_INPUT_PACKET_SIZE, dtype=np.float64)
    state_packet_view = memoryview(state_packet).cast("B")
    expected_torque_bytes = TORQUE_OUTPUT_PACKET_SIZE * np.dtype("<f8").itemsize
    last_loop_end_s = None
    last_stats_print_s = None
    socket_timeout_count = 0

    try:
        while not shutdown_event.is_set():
            try:
                data, addr = sock.recvfrom(1024)

                if data == b"INIT":
                    print(f"[UDP Server] 客户端 {addr} 已连接（收到 INIT）。")
                    with env_lock:
                        body_gui.apply_pending()
                        env.write_state_packet(state_packet)
                    sock.sendto(state_packet_view, addr)
                    continue

                if len(data) != expected_torque_bytes:
                    print(f"[UDP Server] 收到未知长度的数据: {len(data)} bytes")
                    continue

                service_start_s = time.perf_counter()
                tau = np.frombuffer(data, dtype="<f8", count=Config.NUM_JOINTS)
                rerun_payload = None
                with env_lock:
                    body_gui.apply_pending()
                    clipped_tau = env.clip_torque(tau)
                    step_start_s = time.perf_counter()
                    env.apply_torque(clipped_tau)
                    env.step()
                    clipped = env.enforce_joint_limits()
                    if clipped:
                        env.forward()
                    mujoco_step_ms = (time.perf_counter() - step_start_s) * 1000.0

                    if (
                        Config.ENABLE_RERUN
                        and rerun_logger is not None
                        and _should_log_sim_rerun_step(step_count)
                    ):
                        include_twist = bool(Config.SIM_RERUN_INCLUDE_TWIST)
                        (
                            q,
                            qd,
                            pos_current,
                            quat_current,
                            pos_desired,
                            quat_desired,
                            *twist_values,
                        ) = env.get_state_snapshot(include_twist=include_twist)
                        rerun_payload = {
                            "t": step_count * Config.DT,
                            "pos_actual": pos_current,
                            "pos_desired": pos_desired,
                            "quat_actual": quat_current,
                            "quat_desired": quat_desired,
                            "tau_total": clipped_tau.copy(),
                            "q": q,
                            "qd": qd,
                            "step_count": step_count,
                        }
                        if include_twist:
                            rerun_payload["ee_twist"] = twist_values[0]

                    packet_start_s = time.perf_counter()
                    env.write_state_packet(state_packet)
                    state_packet_ms = (time.perf_counter() - packet_start_s) * 1000.0

                sock.sendto(state_packet_view, addr)
                loop_end_s = time.perf_counter()
                service_ms = max(0.0, (loop_end_s - service_start_s) * 1000.0)
                loop_ms = 0.0
                loop_hz = 0.0
                if last_loop_end_s is not None:
                    loop_dt_s = loop_end_s - last_loop_end_s
                    if loop_dt_s > 0.0:
                        loop_ms = loop_dt_s * 1000.0
                        loop_hz = 1.0 / loop_dt_s
                last_loop_end_s = loop_end_s

                rerun_overwrites, rerun_drops = _sim_rerun_stats(rerun_logger)
                viewer_stats = _sim_viewer_stats(viewer_worker)
                if rerun_payload is not None:
                    rerun_logger.log_step(
                        **rerun_payload,
                        cycle_time=service_ms,
                        uart_latency_ms=loop_ms,
                        uart_cycle_hz=loop_hz,
                        sim_target_hz=Config.SIM_TARGET_HZ,
                        sim_service_ms=service_ms,
                        sim_mujoco_step_ms=mujoco_step_ms,
                        sim_state_packet_ms=state_packet_ms,
                        sim_rerun_overwrite_count=rerun_overwrites,
                        sim_rerun_drop_count=rerun_drops,
                        sim_socket_timeout_count=socket_timeout_count,
                        **viewer_stats,
                    )
                last_stats_print_s = _maybe_print_sim_stats(
                    now_s=loop_end_s,
                    last_print_s=last_stats_print_s,
                    loop_hz=loop_hz,
                    loop_ms=loop_ms,
                    mujoco_step_ms=mujoco_step_ms,
                    state_packet_ms=state_packet_ms,
                    rerun_overwrite_count=rerun_overwrites,
                    rerun_drop_count=rerun_drops,
                    viewer_stats=viewer_stats,
                    socket_timeout_count=socket_timeout_count,
                )
                step_count += 1

            except socket.timeout:
                socket_timeout_count += 1
                continue

    except KeyboardInterrupt:
        shutdown_event.set()
        print("\n[UDP Server] 用户中断，正在退出...")
    finally:
        body_gui.close()
        if viewer_worker:
            viewer_worker.close()
        if viewer:
            viewer.close()
        if rerun_logger:
            rerun_logger.close()
        sock.close()
