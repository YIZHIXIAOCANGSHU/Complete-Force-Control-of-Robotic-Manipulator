"""Async Rerun logger for the real control loop."""

from __future__ import annotations

import queue
import threading
import time

from config import Config, _env_int
import rerun_viz


class RealRerunLogger:
    def __init__(
        self,
        queue_size: int | None = None,
        *,
        max_hz: float | None = None,
        perf_counter=time.perf_counter,
        sleep_fn=time.sleep,
    ) -> None:
        self._queue = queue.Queue(maxsize=max(1, int(queue_size or _env_int("AM_D02_RERUN_QUEUE_SIZE", 512))))
        effective_max_hz = Config.RERUN_MAX_HZ if max_hz is None else float(max_hz)
        self._period_s = 1.0 / effective_max_hz if effective_max_hz > 0.0 else 0.0
        self._last_log_time: float | None = None
        self._perf_counter = perf_counter
        self._sleep_fn = sleep_fn
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._worker, name="real-rerun-logger", daemon=True)

    def start(self) -> None:
        self._thread.start()

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
            rerun_viz.log_realtime_step(**payload)
            self._last_log_time = self._perf_counter()

    def log_step(self, **payload) -> None:
        if self._stop_event.is_set():
            return
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(payload)
            except queue.Full:
                pass

    def close(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)
