"""Shared control-loop contract for real and simulated backends."""

from __future__ import annotations

from typing import Protocol

from robot_control.shared.runtime.feedback_state import FeedbackSnapshot
from robot_control.control.types import ControlCommand, ControlTarget


class ControlBackend(Protocol):
    def start(self) -> None:
        ...

    def read_feedback(self) -> FeedbackSnapshot:
        ...

    def send_command(self, command: ControlCommand) -> None:
        ...

    def update_viewer(self, feedback: FeedbackSnapshot, target: ControlTarget) -> None:
        ...

    def stop_safely(self) -> None:
        ...


def run_single_cycle(backend: ControlBackend, command: ControlCommand, target: ControlTarget) -> FeedbackSnapshot:
    feedback = backend.read_feedback()
    backend.send_command(command)
    backend.update_viewer(feedback, target)
    return feedback

