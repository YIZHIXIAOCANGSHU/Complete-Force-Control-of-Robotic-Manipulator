"""Rerun path helpers for parameter-identification modes."""

from __future__ import annotations


def param_id_prefix(simulated_pd: bool) -> str:
    return "param_id_sim_pd" if simulated_pd else "param_id_real"


def result_path(prefix: str, group: str, joint_index: int) -> str:
    return f"{prefix}/result/{group}/J{int(joint_index)}"

