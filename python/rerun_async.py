"""Background Rerun logging helpers."""

from __future__ import annotations

import queue
import threading

import rerun_viz
from config import Config


class RerunLogger:
    """将 Rerun 记录搬到后台线程，减少控制线程抖动。"""

    def __init__(self) -> None:
        self._queue = queue.Queue(maxsize=Config.RERUN_QUEUE_SIZE)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _worker(self) -> None:
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                payload = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            rerun_viz.log_realtime_step(**payload)

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
