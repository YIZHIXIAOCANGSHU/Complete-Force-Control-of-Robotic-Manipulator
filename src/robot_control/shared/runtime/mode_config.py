"""Mode-level environment configuration helpers."""

from __future__ import annotations

from dataclasses import dataclass

from robot_control.config import Config


@dataclass(frozen=True)
class ModeRuntimeConfig:
    mode: str
    enable_viewer: bool
    enable_rerun: bool
    rerun_log_stride: int


def current_mode_config(mode: str) -> ModeRuntimeConfig:
    return ModeRuntimeConfig(
        mode=str(mode),
        enable_viewer=bool(Config.ENABLE_VIEWER),
        enable_rerun=bool(Config.ENABLE_RERUN),
        rerun_log_stride=int(Config.RERUN_LOG_STRIDE),
    )

