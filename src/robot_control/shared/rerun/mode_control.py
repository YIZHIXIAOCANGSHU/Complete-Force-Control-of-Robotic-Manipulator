"""Rerun path helpers for control modes."""

from __future__ import annotations


def control_prefix(simulated: bool) -> str:
    return "control_sim" if simulated else "control_real"


def joint_path(prefix: str, group: str, joint_index: int) -> str:
    return f"{prefix}/joint/{group}/J{int(joint_index)}"

