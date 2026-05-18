#!/usr/bin/env python3
"""PD closed-loop data acquisition for parameter-identification simulation mode."""

from __future__ import annotations

import sys
import time

import numpy as np

from robot_control.config import Config
from robot_control.param_id import sim_main as _base
from robot_control.shared.mujoco.ghost import create_mujoco_ghost_if_enabled
from robot_control.modes.param_id_sim.pd_controller import PDController, _validate_trajectory_pair


def _simulate_pd_step(env, controller: PDController, q_ref, qd_ref) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Run one closed-loop MuJoCo step and return pre-step state plus torque."""
    q = env.get_qpos()
    qd = env.get_qvel()
    tau = controller.compute_torque(q, qd, q_ref, qd_ref)
    env.apply_torque(tau)
    env.step()
    if hasattr(env, "enforce_joint_limits"):
        env.enforce_joint_limits()
    return q, qd, tau


def _collect_pd_data(env, controller: PDController, q_traj, qd_traj) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Run a headless PD simulation and return ``(q_meas, qd_meas, tau_cmd)``."""
    q_ref_traj, qd_ref_traj = _validate_trajectory_pair(q_traj, qd_traj)
    n_steps = len(q_ref_traj)
    q_meas = np.zeros((n_steps, Config.NUM_JOINTS), dtype=np.float64)
    qd_meas = np.zeros_like(q_meas)
    tau_meas = np.zeros_like(q_meas)

    for step in range(n_steps):
        q, qd, tau = _simulate_pd_step(env, controller, q_ref_traj[step], qd_ref_traj[step])
        q_meas[step] = q
        qd_meas[step] = qd
        tau_meas[step] = tau
    return q_meas, qd_meas, tau_meas


def _viewer_sync_stride() -> int:
    configured = getattr(Config, "VIEWER_SYNC_STRIDE", None)
    if configured is not None:
        return max(1, int(configured))
    fps = max(1.0, float(getattr(Config, "REAL_VIEWER_FPS", 60.0)))
    return max(1, int(round(1.0 / max(float(Config.DT) * fps, 1e-9))))


def _set_desired_pose(env, pos, quat) -> None:
    if hasattr(env, "set_target_pose"):
        env.set_target_pose(np.asarray(pos, dtype=np.float64), np.asarray(quat, dtype=np.float64))


def _run_pd_simulation_with_viewer(
    env,
    controller: PDController,
    t_arr,
    q_traj,
    qd_traj,
    qdd_traj=None,
    rerun_ok: bool = False,
    ee_pos_desired_all=None,
    ee_quat_desired_all=None,
    q_ref_friction=None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Run the final closed-loop simulation, keeping viewer/Rerun in sync."""
    del qdd_traj
    t_arr = np.asarray(t_arr, dtype=np.float64)
    q_ref_traj, qd_ref_traj = _validate_trajectory_pair(q_traj, qd_traj)
    if len(t_arr) != len(q_ref_traj):
        raise ValueError(f"t_arr length {len(t_arr)} must match trajectory length {len(q_ref_traj)}")
    if ee_pos_desired_all is None or ee_quat_desired_all is None:
        ee_pos_desired_all, ee_quat_desired_all = _base._compute_ee_poses_for_q_traj(env, q_ref_traj)

    q_meas = np.zeros_like(q_ref_traj)
    qd_meas = np.zeros_like(q_ref_traj)
    tau_meas = np.zeros_like(q_ref_traj)
    sync_stride = _viewer_sync_stride()
    del q_ref_friction

    with _base._viewer_context(env) as viewer:
        ghost = None
        if viewer is not None and Config.ENABLE_MUJOCO_GHOST and hasattr(env, "model") and hasattr(env, "data"):
            ghost = create_mujoco_ghost_if_enabled(
                env.model,
                env.data,
                enabled=True,
                alpha=Config.MUJOCO_GHOST_ALPHA,
            )
            if ghost is not None and hasattr(env, "dof_ids"):
                ghost.dof_ids = env.dof_ids
        start_wall = time.perf_counter()
        for step in range(len(q_ref_traj)):
            if step % 250 == 0:
                sys.stdout.write(f"\r  PD 进度: {step}/{len(q_ref_traj)} ({100 * step // max(len(q_ref_traj), 1)}%)")
                sys.stdout.flush()

            _set_desired_pose(env, ee_pos_desired_all[step], ee_quat_desired_all[step])
            if ghost is not None:
                ghost.update_from_qpos(q_ref_traj[step])
                ghost.update_from_pose(ee_pos_desired_all[step], ee_quat_desired_all[step])
            q, qd, tau = _simulate_pd_step(env, controller, q_ref_traj[step], qd_ref_traj[step])
            q_meas[step] = q
            qd_meas[step] = qd
            tau_meas[step] = tau

            if viewer is not None and step % sync_stride == 0:
                viewer.sync()

            if step % Config.RERUN_LOG_STRIDE == 0:
                _base._log_rerun_step(rerun_ok, t_arr[step], q, qd, tau)
                _base._log_sim_realtime_step_from_env(
                    rerun_ok=rerun_ok,
                    env=env,
                    t=t_arr[step],
                    step=step,
                    q_actual=q,
                    qd_actual=qd,
                    q_desired=q_ref_traj[step],
                    tau_received=tau,
                    tau_applied=tau,
                    cycle_time_ms=Config.DT * 1000.0,
                    pos_desired=ee_pos_desired_all[step],
                    quat_desired=ee_quat_desired_all[step],
                )
            _base._sync_realtime(start_wall, t_arr[step])

        if viewer is not None:
            viewer.sync()

    print()
    return q_meas, qd_meas, tau_meas
