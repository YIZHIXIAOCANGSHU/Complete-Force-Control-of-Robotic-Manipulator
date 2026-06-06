"""UDP server that exposes the MuJoCo simulation to the C controller."""

from __future__ import annotations

import socket
import time
import threading
import queue
from contextlib import nullcontext

import numpy as np

from config import Config
from mujoco_viewer import VIEWER_AVAILABLE, launch_passive_viewer
import rerun_viz
from sim.state_packets import (
    CONTROL_INPUT_PACKET_SIZE,
    OUTPUT_REF_POS_OFFSET,
    OUTPUT_REF_QUAT_OFFSET,
    OUTPUT_TAU_OFFSET,
    TORQUE_OUTPUT_PACKET_SIZE,
)

from sim.env_factory import _create_sim_env
from sim.mc_report import _collect_monte_carlo_workspace, run_monte_carlo_range_check, run_sim_report
from sim.mc_viewer import _show_monte_carlo_workspace_viewer, _workspace_box_edges
from sim.report_assets import (
    DEFAULT_REPORT_CONTROL_DURATION_S,
    DEFAULT_REPORT_CONTROL_LOG_STRIDE,
    DEFAULT_REPORT_MAX_SVG_POINTS,
    DEFAULT_REPORT_PROGRESS_INTERVAL,
    DEFAULT_REPORT_SAMPLES,
    ControlLoopResult,
    ControlTargetSegment,
    _array_to_list,
    _build_control_target_schedule,
    _control_csv_fieldnames,
    _default_report_output_dir,
    _downsample_indices,
    _format_dual_monte_carlo_report,
    _format_internal_workspace_box,
    _format_monte_carlo_report,
    _format_range_block,
    _format_safe_box_summary,
    _format_safe_box_summary_block,
    _format_vector,
    _internal_box_to_dict,
    _markdown_control_table,
    _markdown_workspace_table,
    _model_metadata,
    _polyline_points,
    _run_closed_loop_report_experiment,
    _segment_for_time,
    _snapshot_to_dict,
    _status_counts,
    _summarize_control_loop_arrays,
    _svg_scale,
    _telemetry_row,
    _write_control_error_svg,
    _write_control_loop_csv,
    _write_monte_carlo_report_assets,
    _write_position_span_svg,
    _write_report_markdown,
    _write_sim_report_assets,
    _write_torque_summary_svg,
    _write_workspace_projection_svg,
)
from sim.workspace import (
    DEFAULT_MC_FALLBACK_LIMIT,
    DEFAULT_MC_MAX_HULL_POINTS,
    DEFAULT_MC_MAX_VIS_POINTS,
    DEFAULT_MC_PROGRESS_INTERVAL,
    DEFAULT_MC_SAMPLES,
    WORKSPACE_MIN_HALF_SIZE,
    WORKSPACE_PADDING_RATIO,
    InternalWorkspaceBox,
    MonteCarloWorkspaceResult,
    RangeAccumulator,
    RangeSnapshot,
    WorkspaceHull,
    _box_barrier_objective,
    _empty_internal_workspace_box,
    _expand_box_half_size_at_center,
    _maximize_log_box_half_size,
    compute_internal_workspace_box_intersection,
    compute_largest_internal_workspace_box,
    compute_workspace_bounds,
    compute_workspace_hull,
    resolve_sampling_bounds,
    select_visualization_points,
)


_CONTROL_OUTPUT_DTYPE = np.dtype("<f8")
_EXPECTED_TORQUE_BYTES = TORQUE_OUTPUT_PACKET_SIZE * _CONTROL_OUTPUT_DTYPE.itemsize
_TAU_OUTPUT_SLICE = slice(OUTPUT_TAU_OFFSET, OUTPUT_TAU_OFFSET + Config.NUM_JOINTS)
_REF_POS_OUTPUT_SLICE = slice(
    OUTPUT_REF_POS_OFFSET,
    OUTPUT_REF_POS_OFFSET + Config.NUM_ARMS * 3,
)
_REF_QUAT_OUTPUT_SLICE = slice(
    OUTPUT_REF_QUAT_OFFSET,
    OUTPUT_REF_QUAT_OFFSET + Config.NUM_ARMS * 4,
)
_REF_POS_OUTPUT_SHAPE = (Config.NUM_ARMS, 3)
_REF_QUAT_OUTPUT_SHAPE = (Config.NUM_ARMS, 4)


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


def _should_print_sim_stats(now_s: float, last_print_s: float | None) -> bool:
    if Config.SIM_STATS_INTERVAL_S <= 0.0:
        return False
    return last_print_s is None or now_s - last_print_s >= Config.SIM_STATS_INTERVAL_S


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


def _write_ready_file(ready_file: str | None) -> None:
    if ready_file is None:
        return
    with open(ready_file, "w", encoding="utf-8") as file_obj:
        file_obj.write("ready\n")



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
    expected_torque_bytes = _EXPECTED_TORQUE_BYTES
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
                control_output = np.frombuffer(data, dtype=_CONTROL_OUTPUT_DTYPE, count=TORQUE_OUTPUT_PACKET_SIZE)
                tau = control_output[_TAU_OUTPUT_SLICE]
                ref_pos = control_output[_REF_POS_OUTPUT_SLICE].reshape(_REF_POS_OUTPUT_SHAPE)
                ref_quat = control_output[_REF_QUAT_OUTPUT_SLICE].reshape(_REF_QUAT_OUTPUT_SHAPE)
                rerun_payload = None
                with env_lock:
                    body_gui.apply_pending()
                    env.set_all_reference_poses(ref_pos, ref_quat)
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
                        ref_pos_base, ref_quat_base = env.get_all_reference_poses_base()
                        rerun_payload = {
                            "t": step_count * Config.DT,
                            "pos_actual": pos_current,
                            "pos_desired": pos_desired,
                            "quat_actual": quat_current,
                            "quat_desired": quat_desired,
                            "pos_reference": ref_pos_base,
                            "quat_reference": ref_quat_base,
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

                should_log_rerun = rerun_payload is not None
                should_print_stats = _should_print_sim_stats(loop_end_s, last_stats_print_s)
                if should_log_rerun or should_print_stats:
                    rerun_overwrites, rerun_drops = _sim_rerun_stats(rerun_logger)
                    viewer_stats = _sim_viewer_stats(viewer_worker)
                else:
                    rerun_overwrites = 0
                    rerun_drops = 0
                    viewer_stats = None
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
                if should_print_stats:
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
