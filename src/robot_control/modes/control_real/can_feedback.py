"""Shared SocketCAN feedback and safe-stop helpers."""

from __future__ import annotations

import time
from typing import Callable, Iterable

from robot_control.config import Config
from robot_control.modes.control_real.runtime_config import (
    CAN_FEEDBACK_TIMEOUT_S,
    CAN_READ_CHUNK_SIZE,
)
from robot_control.shared.runtime.feedback_state import FeedbackSnapshot, snapshot_from_frames


class CanFeedbackTimeout(TimeoutError):
    def __init__(self, timeout_s: float, missing_ids: Iterable[int]) -> None:
        self.timeout_s = float(timeout_s)
        self.missing_ids = tuple(int(motor_id) for motor_id in missing_ids)
        super().__init__(
            f"{self.timeout_s:.3f}s feedback timeout, missing motors={self.missing_ids}"
        )


def _default_motor_ids() -> tuple[int, ...]:
    return tuple(range(1, Config.NUM_JOINTS + 1))


def _normalize_motor_ids(motor_ids: Iterable[int] | None) -> tuple[int, ...]:
    source = _default_motor_ids() if motor_ids is None else motor_ids
    return tuple(int(motor_id) for motor_id in source)


def missing_feedback_ids(feedback_mask: int, joint_count: int | None = None) -> tuple[int, ...]:
    count = Config.NUM_JOINTS if joint_count is None else int(joint_count)
    return tuple(
        joint_idx + 1
        for joint_idx in range(count)
        if not (int(feedback_mask) & (1 << joint_idx))
    )


def safe_zero_and_disable(transport, motor_ids: Iterable[int] | None = None) -> None:
    ids = _normalize_motor_ids(motor_ids)
    for motor_id in ids:
        try:
            transport.send_mit_command(
                int(motor_id),
                position=0.0,
                velocity=0.0,
                kp=0.0,
                kd=0.0,
                torque=0.0,
            )
        except Exception as exc:
            print(f"[CAN Warning] motor {motor_id} zero-torque send failed: {exc}")
    for motor_id in ids:
        try:
            transport.disable_motor(int(motor_id))
        except Exception as exc:
            print(f"[CAN Warning] motor {motor_id} disable failed: {exc}")


def send_zero_keepalive(transport, motor_ids: Iterable[int] | None = None) -> None:
    for motor_id in _normalize_motor_ids(motor_ids):
        transport.send_mit_command(
            int(motor_id),
            position=0.0,
            velocity=0.0,
            kp=0.0,
            kd=0.0,
            torque=0.0,
        )


def startup_enable(transport, motor_ids: Iterable[int] | None = None) -> None:
    ids = _normalize_motor_ids(motor_ids)
    try:
        transport.reset_input_buffer()
    except Exception as exc:
        print(f"[CAN Warning] reset CAN input buffer failed: {exc}")
    for motor_id in ids:
        transport.clear_error(int(motor_id))
        transport.send_mit_command(
            int(motor_id),
            position=0.0,
            velocity=0.0,
            kp=0.0,
            kd=0.0,
            torque=0.0,
        )
        transport.enable_motor(int(motor_id))
        transport.send_mit_command(
            int(motor_id),
            position=0.0,
            velocity=0.0,
            kp=0.0,
            kd=0.0,
            torque=0.0,
        )
    send_zero_keepalive(transport, ids)


def _complete_feedback_mask(motor_ids: tuple[int, ...]) -> int:
    mask = 0
    for motor_id in motor_ids:
        if 1 <= int(motor_id) <= Config.NUM_JOINTS:
            mask |= 1 << (int(motor_id) - 1)
    return mask


def read_complete_feedback_snapshot(
    transport,
    motor_ids: Iterable[int] | None = None,
    *,
    feedback_timeout_s: float = CAN_FEEDBACK_TIMEOUT_S,
    read_chunk_size: int = CAN_READ_CHUNK_SIZE,
    keepalive: bool = True,
    source: str = "usb2fdcan",
    now: Callable[[], float] | None = None,
) -> FeedbackSnapshot:
    ids = _normalize_motor_ids(motor_ids)
    expected_mask = _complete_feedback_mask(ids)
    feedback_mask = 0
    frames_by_motor = {}
    clock = time.perf_counter if now is None else now
    start = clock()

    while feedback_mask != expected_mask:
        transport.read(read_chunk_size)
        while True:
            frame = transport.pop_feedback_frame()
            if frame is None:
                break
            motor_id = int(getattr(frame, "motor_id"))
            if motor_id not in ids or not 1 <= motor_id <= Config.NUM_JOINTS:
                continue
            frames_by_motor[motor_id] = frame
            feedback_mask |= 1 << (motor_id - 1)
            if feedback_mask == expected_mask:
                ordered_frames = [frames_by_motor[motor_id] for motor_id in ids]
                return snapshot_from_frames(
                    ordered_frames,
                    timestamp=clock(),
                    joint_count=Config.NUM_JOINTS,
                    source=source,
                )

        if feedback_mask == expected_mask:
            ordered_frames = [frames_by_motor[motor_id] for motor_id in ids]
            return snapshot_from_frames(
                ordered_frames,
                timestamp=clock(),
                joint_count=Config.NUM_JOINTS,
                source=source,
            )

        if keepalive:
            send_zero_keepalive(transport, ids)
        if clock() - start > float(feedback_timeout_s):
            raise CanFeedbackTimeout(
                feedback_timeout_s,
                missing_feedback_ids(feedback_mask, Config.NUM_JOINTS),
            )

    raise CanFeedbackTimeout(feedback_timeout_s, ids)
