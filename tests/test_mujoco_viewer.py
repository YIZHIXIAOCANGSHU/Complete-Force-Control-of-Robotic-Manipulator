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
