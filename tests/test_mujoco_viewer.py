from __future__ import annotations

import threading
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import mujoco_viewer
import udp_server


class DummyViewerModule:
    def __init__(self) -> None:
        self.calls: list[tuple[object, object, dict[str, bool]]] = []

    def launch_passive(self, model, data, **kwargs):
        self.calls.append((model, data, kwargs))
        return "viewer-handle"


def test_launch_passive_viewer_hides_both_side_panels(monkeypatch):
    dummy_viewer_module = DummyViewerModule()
    monkeypatch.setattr(mujoco_viewer, "_viewer_module", dummy_viewer_module)

    viewer = mujoco_viewer.launch_passive_viewer("model", "data")

    assert viewer == "viewer-handle"
    assert dummy_viewer_module.calls == [
        (
            "model",
            "data",
            {
                "show_left_ui": False,
                "show_right_ui": False,
            },
        )
    ]


class DummyViewer:
    def __init__(self, running: bool = True) -> None:
        self.calls: list[str] = []
        self.running = running

    def is_running(self) -> bool:
        return self.running

    def sync(self) -> None:
        self.calls.append("sync")


def test_sim_viewer_worker_syncs_viewer_under_env_lock():
    viewer = DummyViewer()
    env_lock = threading.RLock()
    shutdown_event = threading.Event()

    worker = udp_server.SimViewerSyncWorker(viewer, env_lock, shutdown_event, fps=60)

    assert worker.sync_once() is True
    assert viewer.calls == ["sync"]
    assert not shutdown_event.is_set()
    stats = worker.stats_snapshot()
    assert stats["viewer_sync_count"] == 1
    assert stats["viewer_skip_count"] == 0
    assert stats["viewer_sync_ms"] >= 0.0
    assert stats["viewer_lock_wait_ms"] >= 0.0


def test_sim_viewer_worker_skips_when_env_lock_is_busy():
    viewer = DummyViewer()
    env_lock = threading.Lock()
    shutdown_event = threading.Event()
    env_lock.acquire()
    worker = udp_server.SimViewerSyncWorker(
        viewer,
        env_lock,
        shutdown_event,
        fps=60,
        lock_timeout_s=0.0,
    )

    try:
        assert worker.sync_once() is True
    finally:
        env_lock.release()

    assert viewer.calls == []
    assert not shutdown_event.is_set()
    stats = worker.stats_snapshot()
    assert stats["viewer_sync_count"] == 0
    assert stats["viewer_skip_count"] == 1
    assert stats["viewer_lock_wait_ms"] >= 0.0


def test_sim_viewer_worker_backs_off_after_slow_sync_then_resumes():
    viewer = DummyViewer()
    env_lock = threading.RLock()
    shutdown_event = threading.Event()
    now = [0.0]

    def fake_perf_counter() -> float:
        value = now[0]
        now[0] += 0.002
        return value

    worker = udp_server.SimViewerSyncWorker(
        viewer,
        env_lock,
        shutdown_event,
        fps=60,
        sync_budget_ms=1.0,
        backoff_frames=2,
        perf_counter=fake_perf_counter,
    )

    assert worker.sync_once() is True
    assert worker.sync_once() is True
    assert worker.sync_once() is True
    assert worker.sync_once() is True

    assert viewer.calls == ["sync", "sync"]
    stats = worker.stats_snapshot()
    assert stats["viewer_sync_count"] == 2
    assert stats["viewer_skip_count"] == 2
    assert stats["viewer_sync_ms"] > 1.0


def test_sim_viewer_worker_sets_shutdown_when_viewer_closes():
    viewer = DummyViewer(running=False)
    shutdown_event = threading.Event()

    worker = udp_server.SimViewerSyncWorker(viewer, threading.RLock(), shutdown_event, fps=60)

    assert worker.sync_once() is False
    assert viewer.calls == []
    assert shutdown_event.is_set()


def test_sim_rerun_logger_coalesces_to_latest_payload():
    logged = []
    logger = udp_server.SimRerunLogger(
        queue_size=4,
        max_hz=1000.0,
        log_fn=lambda **payload: logged.append(payload),
    )

    logger.log_step(step_count=1)
    logger.log_step(step_count=2)
    logger.log_step(step_count=3)
    logger._stop_event.set()
    logger._worker()

    assert [payload["step_count"] for payload in logged] == [3]


def test_sim_rerun_logger_counts_overwritten_pending_payload():
    logged = []
    logger = udp_server.SimRerunLogger(
        queue_size=1,
        max_hz=1000.0,
        log_fn=lambda **payload: logged.append(payload),
    )

    logger.log_step(step_count=1)
    logger.log_step(step_count=2)
    logger._stop_event.set()
    logger._worker()

    assert [payload["step_count"] for payload in logged] == [2]
    assert logger.overwritten_payload_count == 1
    assert logger.dropped_payload_count == 0


def test_sim_rerun_logger_respects_max_hz_between_sends():
    logged = []
    sleep_calls = []
    now = [0.0]

    def fake_sleep(seconds: float) -> None:
        sleep_calls.append(seconds)
        now[0] += seconds

    logger = udp_server.SimRerunLogger(
        queue_size=4,
        max_hz=20.0,
        log_fn=lambda **payload: logged.append(payload),
        perf_counter=lambda: now[0],
        sleep_fn=fake_sleep,
    )

    logger.log_step(step_count=1)
    logger._stop_event.set()
    logger._worker()
    now[0] += 0.01
    logger._stop_event.clear()
    logger.log_step(step_count=2)
    logger._stop_event.set()
    logger._worker()

    assert [payload["step_count"] for payload in logged] == [1, 2]
    assert sleep_calls == [0.04]


def test_sim_udp_hot_path_no_longer_syncs_viewer_or_logs_rerun_directly():
    source = (Path(__file__).resolve().parents[1] / "python" / "sim" / "udp_server.py").read_text(
        encoding="utf-8"
    )
    run_start = source.index("def run_udp_server(")
    run_end = source.find("if __name__ == \"__main__\":", run_start)
    if run_end == -1:
        run_end = len(source)
    run_source = source[run_start:run_end]

    assert "_step_env_with_viewer_sync" not in run_source
    assert ".sync()" not in run_source
    assert "rerun_viz.log_realtime_step(" not in run_source
    assert "SimViewerSyncWorker" in run_source
    assert "SimRerunLogger" in run_source


def test_sim_udp_hot_path_replies_before_enqueuing_rerun_payload():
    source = (Path(__file__).resolve().parents[1] / "python" / "sim" / "udp_server.py").read_text(
        encoding="utf-8"
    )
    run_start = source.index("def run_udp_server(")
    run_end = source.find("if __name__ == \"__main__\":", run_start)
    if run_end == -1:
        run_end = len(source)
    run_source = source[run_start:run_end]
    torque_loop_start = run_source.index("service_start_s = time.perf_counter()")

    send_index = run_source.index("sock.sendto(state_packet_view, addr)", torque_loop_start)
    rerun_index = run_source.index("rerun_logger.log_step(", send_index)

    assert send_index < rerun_index


def test_sim_udp_rerun_snapshot_twist_is_profile_gated():
    source = (Path(__file__).resolve().parents[1] / "python" / "sim" / "udp_server.py").read_text(
        encoding="utf-8"
    )
    run_start = source.index("def run_udp_server(")
    run_end = source.find("if __name__ == \"__main__\":", run_start)
    if run_end == -1:
        run_end = len(source)
    run_source = source[run_start:run_end]
    payload_start = run_source.index("rerun_payload = {")
    payload_end = run_source.index("env.write_state_packet(state_packet)", payload_start)
    payload_block = run_source[payload_start:payload_end]

    assert "include_twist = bool(Config.SIM_RERUN_INCLUDE_TWIST)" in run_source
    assert "env.get_state_snapshot(include_twist=include_twist)" in run_source
    assert 'rerun_payload["ee_twist"] = twist_values[0]' in run_source
    assert '"ee_twist":' not in payload_block


def test_maybe_print_sim_stats_reports_worker_metrics(monkeypatch, capsys):
    monkeypatch.setattr(udp_server.Config, "SIM_STATS_INTERVAL_S", 1.0)
    monkeypatch.setattr(udp_server.Config, "SIM_TARGET_HZ", 1000.0)

    last_print = udp_server._maybe_print_sim_stats(
        now_s=0.5,
        last_print_s=None,
        loop_hz=990.0,
        loop_ms=1.01,
        mujoco_step_ms=0.62,
        state_packet_ms=0.04,
        rerun_overwrite_count=3,
        rerun_drop_count=1,
        viewer_stats={
            "viewer_sync_count": 9,
            "viewer_skip_count": 4,
            "viewer_sync_ms": 0.35,
            "viewer_lock_wait_ms": 0.02,
        },
        socket_timeout_count=2,
    )

    assert last_print == 0.5
    output = capsys.readouterr().out
    assert "[Sim Stats]" in output
    assert "target=1000Hz" in output
    assert "loop=990.0Hz/1.010ms" in output
    assert "mujoco_step=0.620ms" in output
    assert "state_packet=0.040ms" in output
    assert "rerun_overwrites=3" in output
    assert "rerun_drops=1" in output
    assert "viewer_sync=9" in output
    assert "viewer_skip=4" in output
    assert "socket_timeouts=2" in output

    unchanged = udp_server._maybe_print_sim_stats(
        now_s=0.6,
        last_print_s=last_print,
        loop_hz=1.0,
        loop_ms=1.0,
        mujoco_step_ms=1.0,
        state_packet_ms=1.0,
        rerun_overwrite_count=0,
        rerun_drop_count=0,
        viewer_stats={
            "viewer_sync_count": 0,
            "viewer_skip_count": 0,
            "viewer_sync_ms": 0.0,
            "viewer_lock_wait_ms": 0.0,
        },
        socket_timeout_count=0,
    )

    assert unchanged == last_print
    assert capsys.readouterr().out == ""
