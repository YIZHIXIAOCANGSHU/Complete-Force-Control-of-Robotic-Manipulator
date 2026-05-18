"""Timing helpers shared by mode pipelines."""

from __future__ import annotations

import time
from dataclasses import dataclass


@dataclass
class LoopTimer:
    last_time: float | None = None

    def mark(self, now: float | None = None) -> float:
        current = time.perf_counter() if now is None else float(now)
        if self.last_time is None:
            self.last_time = current
            return 0.0
        dt = current - self.last_time
        self.last_time = current
        return dt

    def mark_ms(self, now: float | None = None) -> float:
        return self.mark(now) * 1000.0

