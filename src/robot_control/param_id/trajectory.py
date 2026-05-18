#!/usr/bin/env python3
"""Trajectory generation and excitation selection for parameter identification."""

from __future__ import annotations

import numpy as np
import mujoco

from robot_control.config import Config
from robot_control.param_id.excitation import fourier_trajectory, limit_ee_speed
from robot_control.param_id.regressor import build_stacked_regressor
from robot_control.shared.rerun import viz as rerun_viz
from robot_control.param_id.diagnostics import (
    _case_selection_key,
    _distal_observability,
    _extract_ground_truth,
    _joint_effect_torque,
    _parameter_group_observability,
    _regularization_grid,
    _scaled_svd_metrics,
    _solve_identification_case,
)


def _trajectory_seeds() -> list[int]:
    seeds = []
    for item in str(Config.PARAM_ID_TRAJECTORY_SEEDS).split(","):
        item = item.strip()
        if item:
            seeds.append(int(item))
    if not seeds:
        seeds.append(42)
    while len(seeds) < Config.PARAM_ID_TRAJECTORY_CANDIDATES:
        seeds.append(seeds[-1] + 1)
    return seeds[:Config.PARAM_ID_TRAJECTORY_CANDIDATES]

def _distal_excitation_weights(amp_scale=1.45, freq_scale=1.25, proximal_scale=1.0):
    amp = np.full(7, float(proximal_scale), dtype=np.float64)
    freq = np.ones(7, dtype=np.float64)
    start = Config.PARAM_ID_DISTAL_LINK_START - 1
    amp[start:] = float(amp_scale)
    freq[start:] = float(freq_scale)
    return amp, freq

def _phase_offsets(phase_span):
    phases = np.zeros(7, dtype=np.float64)
    start = Config.PARAM_ID_DISTAL_LINK_START - 1
    if start < 7:
        phases[start:] = np.linspace(0.0, float(phase_span), 7 - start)
    return phases

def _trajectory_profiles():
    """Planned experiment trajectories from the J7 accuracy improvement plan."""
    profiles = [
        {
            "name": "T0",
            "description": "distal-wide seed baseline",
            "modifiers": (),
            "with_gravity": False,
            "with_com_gravity": False,
            "with_inertia_burst": False,
            "dynamic_label": "dynamic",
        },
        {
            "name": "T1",
            "description": "T0 + J7 mid/high-frequency excitation",
            "modifiers": ("j7_high_frequency",),
            "with_gravity": False,
            "with_com_gravity": False,
            "with_inertia_burst": False,
            "dynamic_label": "j7",
        },
        {
            "name": "T2",
            "description": "T0 + J6/J7 90/180 deg phase sweep",
            "modifiers": ("j6_j7_phase_sweep",),
            "with_gravity": False,
            "with_com_gravity": False,
            "with_inertia_burst": False,
            "dynamic_label": "j6j7",
        },
        {
            "name": "T3",
            "description": "T0 + quasi-static gravity posture layers",
            "modifiers": (),
            "with_gravity": True,
            "with_com_gravity": False,
            "with_inertia_burst": False,
            "dynamic_label": "dynamic",
        },
        {
            "name": "T4",
            "description": "T1 + T2 + T3 combined long trajectory",
            "modifiers": ("j7_high_frequency", "j6_j7_phase_sweep"),
            "with_gravity": True,
            "with_com_gravity": False,
            "with_inertia_burst": False,
            "dynamic_label": "j6j7",
        },
        {
            "name": "T5",
            "description": "COM gravity multi-posture holds and distal scans",
            "modifiers": (),
            "with_gravity": False,
            "with_com_gravity": True,
            "with_inertia_burst": False,
            "dynamic_label": "dynamic",
        },
        {
            "name": "T6",
            "description": "J5/J6/J7 smooth inertia burst chirps",
            "modifiers": ("j7_high_frequency",),
            "with_gravity": False,
            "with_com_gravity": False,
            "with_inertia_burst": True,
            "dynamic_label": "inertia",
        },
        {
            "name": "T7",
            "description": "COM gravity holds + distal inertia burst chirps",
            "modifiers": ("j7_high_frequency",),
            "with_gravity": False,
            "with_com_gravity": True,
            "with_inertia_burst": True,
            "dynamic_label": "inertia",
        },
    ]
    return profiles[:Config.PARAM_ID_TRAJECTORY_PROFILES]

def _limit_joint_ranges(q, q0, limits):
    q_limited = np.asarray(q, dtype=np.float64).copy()
    q_min, q_max = limits
    for joint in range(q_limited.shape[1]):
        amp = np.max(np.abs(q_limited[:, joint] - q0[joint]))
        if amp <= 0.0:
            continue
        available = min(abs(q_max[joint] - q0[joint]), abs(q0[joint] - q_min[joint]))
        if amp > 0.8 * available:
            scale = 0.8 * available / amp
            q_limited[:, joint] = q0[joint] + (q_limited[:, joint] - q0[joint]) * scale
    return q_limited

def _differentiate_trajectory(q, dt):
    edge_order = 2 if q.shape[0] > 2 else 1
    qd = np.gradient(q, dt, axis=0, edge_order=edge_order)
    qdd = np.gradient(qd, dt, axis=0, edge_order=edge_order)
    return qd, qdd

def _safe_joint_amplitude(q0, limits, joint, fraction):
    q_min, q_max = limits
    available = min(abs(q_max[joint] - q0[joint]), abs(q0[joint] - q_min[joint]))
    return float(fraction) * float(max(available, 0.0))

def _apply_j7_high_frequency(t_arr, q_traj, q0, limits):
    q = np.asarray(q_traj, dtype=np.float64).copy()
    duration = max(float(t_arr[-1] - t_arr[0]), Config.DT)
    tau = (t_arr - t_arr[0]) / duration
    window = np.sin(np.pi * tau) ** 2
    amp6 = _safe_joint_amplitude(q0, limits, 5, 0.50)
    amp7 = _safe_joint_amplitude(q0, limits, 6, 0.68)
    q[:, 5] += window * amp6 * np.sin(2.0 * np.pi * 0.55 * t_arr + np.pi / 2.0)
    q[:, 6] += window * amp7 * np.sin(2.0 * np.pi * 0.85 * t_arr + np.pi)
    return _limit_joint_ranges(q, q0, limits)

def _apply_j6_j7_phase_sweep(t_arr, q_traj, q0, limits):
    q = np.asarray(q_traj, dtype=np.float64).copy()
    duration = max(float(t_arr[-1] - t_arr[0]), Config.DT)
    tau = (t_arr - t_arr[0]) / duration
    amp6 = _safe_joint_amplitude(q0, limits, 5, 0.48)
    amp7 = _safe_joint_amplitude(q0, limits, 6, 0.66)
    for start, end, phase in ((0.0, 0.5, np.pi / 2.0), (0.5, 1.0, np.pi)):
        mask = (tau >= start) & (tau <= end)
        if not np.any(mask):
            continue
        local = (tau[mask] - start) / max(end - start, 1e-12)
        window = np.sin(np.pi * local) ** 2
        q[mask, 3] += 0.18 * window * np.sin(2.0 * np.pi * 0.20 * t_arr[mask])
        q[mask, 4] += 0.16 * window * np.sin(2.0 * np.pi * 0.34 * t_arr[mask] + np.pi / 3.0)
        q[mask, 5] += amp6 * window * np.sin(2.0 * np.pi * 0.46 * t_arr[mask])
        q[mask, 6] += amp7 * window * np.sin(2.0 * np.pi * 0.62 * t_arr[mask] + phase)
    return _limit_joint_ranges(q, q0, limits)

def _cosine_segment(q_start, q_end, duration, dt):
    n = max(2, int(round(float(duration) / float(dt))) + 1)
    alpha = np.linspace(0.0, 1.0, n)
    blend = 0.5 - 0.5 * np.cos(np.pi * alpha)
    return q_start[None, :] + (q_end - q_start)[None, :] * blend[:, None]

def _hold_segment(q, duration, dt):
    n = max(2, int(round(float(duration) / float(dt))) + 1)
    return np.repeat(np.asarray(q, dtype=np.float64)[None, :], n, axis=0)

def _clip_to_limits(q, limits):
    q_min, q_max = limits
    return np.minimum(np.maximum(q, q_min), q_max)

def _concat_labeled_segments(segments):
    q_parts = []
    labels = []
    cursor = 0
    for label, q_segment in segments:
        q_segment = np.asarray(q_segment, dtype=np.float64)
        if q_segment.size == 0:
            continue
        if q_parts:
            q_segment = q_segment[1:]
        if q_segment.size == 0:
            continue
        q_parts.append(q_segment)
        labels.extend([label] * len(q_segment))
        cursor += len(q_segment)
    if not q_parts:
        return np.zeros((0, 7), dtype=np.float64), np.array([], dtype=object)
    return np.vstack(q_parts), np.asarray(labels, dtype=object)

def _quasi_static_gravity_segment(q_start, limits, dt):
    postures = [
        [0.00, -0.55, 0.30, 0.95, 0.25, 0.25, 0.50],
        [0.45, -0.65, -0.10, 0.55, -0.30, -0.25, -0.50],
        [-0.45, -0.35, 0.25, 0.85, 0.35, -0.45, 0.00],
        [0.20, -0.20, 0.45, 0.25, -0.35, 0.35, 0.55],
    ]
    scan_delta = np.array([0.0, 0.0, 0.0, -0.35, -0.25, 0.35, -0.55], dtype=np.float64)

    current = np.asarray(q_start, dtype=np.float64)
    segments = []
    for posture in postures:
        target = _clip_to_limits(np.asarray(posture, dtype=np.float64), limits)
        scan_target = _clip_to_limits(target + scan_delta, limits)
        segments.append(("gravity", _cosine_segment(current, target, 0.9, dt)))
        segments.append(("gravity", _hold_segment(target, 1.0, dt)))
        segments.append(("gravity", _cosine_segment(target, scan_target, 1.1, dt)))
        segments.append(("gravity", _hold_segment(scan_target, 0.8, dt)))
        current = scan_target
    return _concat_labeled_segments(segments)

def _append_quasi_static_gravity(q, labels, limits, dt):
    gravity_q, gravity_labels = _quasi_static_gravity_segment(q[-1], limits, dt)
    if gravity_q.size == 0:
        return q, labels
    return (
        np.vstack([q, gravity_q[1:]]),
        np.concatenate([labels, gravity_labels[1:]]),
    )

def _com_gravity_segment(q_start, limits, dt):
    postures = [
        [0.00, -0.70, 0.55, 0.95, 0.35, -0.35, 0.55],
        [0.35, -0.55, -0.35, 0.75, -0.40, 0.35, -0.45],
        [-0.35, -0.30, 0.50, 0.45, 0.25, 0.45, 0.10],
        [0.15, -0.75, 0.10, 1.05, -0.20, -0.20, 0.45],
        [-0.20, -0.45, -0.45, 0.70, 0.40, -0.45, -0.55],
    ]
    scan_vectors = [
        [0.0, 0.0, -0.18, 0.20, -0.25, 0.25, -0.35],
        [0.0, 0.0, 0.16, -0.25, 0.25, -0.25, 0.35],
    ]

    current = np.asarray(q_start, dtype=np.float64)
    segments = []
    for idx, posture in enumerate(postures):
        target = _clip_to_limits(np.asarray(posture, dtype=np.float64), limits)
        scan_delta = np.asarray(scan_vectors[idx % len(scan_vectors)], dtype=np.float64)
        scan_target = _clip_to_limits(target + scan_delta, limits)
        segments.append(("com_gravity", _cosine_segment(current, target, 1.0, dt)))
        segments.append(("com_gravity", _hold_segment(target, 1.2, dt)))
        segments.append(("com_gravity", _cosine_segment(target, scan_target, 1.2, dt)))
        segments.append(("com_gravity", _hold_segment(scan_target, 0.9, dt)))
        current = scan_target
    return _concat_labeled_segments(segments)

def _append_com_gravity(q, labels, limits, dt):
    com_q, com_labels = _com_gravity_segment(q[-1], limits, dt)
    if com_q.size == 0:
        return q, labels
    return (
        np.vstack([q, com_q[1:]]),
        np.concatenate([labels, com_labels[1:]]),
    )

def _inertia_burst_segment(q_start, limits, dt):
    duration = 6.0
    n = max(2, int(round(duration / float(dt))) + 1)
    t = np.linspace(0.0, duration, n)
    tau = t / max(duration, 1e-12)
    window = np.sin(np.pi * tau) ** 2
    q_start = np.asarray(q_start, dtype=np.float64)
    q = np.repeat(q_start[None, :], n, axis=0)

    amp4 = _safe_joint_amplitude(q_start, limits, 3, 0.20)
    amp5 = _safe_joint_amplitude(q_start, limits, 4, 0.47)
    amp6 = _safe_joint_amplitude(q_start, limits, 5, 0.55)
    amp7 = _safe_joint_amplitude(q_start, limits, 6, 0.72)
    chirp_a = 0.35 * t + 0.045 * t * t
    chirp_b = 0.50 * t + 0.065 * t * t
    chirp_c = 0.70 * t + 0.085 * t * t
    chirp_d = 0.95 * t + 0.105 * t * t
    q[:, 3] += amp4 * window * np.sin(2.0 * np.pi * chirp_a)
    q[:, 4] += amp5 * window * np.sin(2.0 * np.pi * chirp_b + np.pi / 5.0)
    q[:, 5] += amp6 * window * np.sin(2.0 * np.pi * chirp_c + np.pi / 2.0)
    q[:, 6] += amp7 * window * np.sin(2.0 * np.pi * chirp_d + np.pi)
    return _clip_to_limits(q, limits), np.asarray(["inertia"] * n, dtype=object)

def _append_inertia_burst(q, labels, limits, dt):
    burst_q, burst_labels = _inertia_burst_segment(q[-1], limits, dt)
    if burst_q.size == 0:
        return q, labels
    return (
        np.vstack([q, burst_q[1:]]),
        np.concatenate([labels, burst_labels[1:]]),
    )

def _build_planned_trajectory(profile, seed, q0, limits):
    amp_weights, freq_weights = _distal_excitation_weights(1.75, 1.15, 0.9)
    phases = _phase_offsets(np.pi / 3.0)
    t_arr, q_traj, _, _ = fourier_trajectory(
        q0=q0,
        n_harmonics=5,
        base_freq=0.2,
        duration=8.0,
        dt=Config.DT,
        joint_limits=limits,
        random_seed=seed,
        joint_amplitude_weights=amp_weights,
        joint_frequency_weights=freq_weights,
        phase_offsets=phases,
    )

    for modifier in profile["modifiers"]:
        if modifier == "j7_high_frequency":
            q_traj = _apply_j7_high_frequency(t_arr, q_traj, q0, limits)
        elif modifier == "j6_j7_phase_sweep":
            q_traj = _apply_j6_j7_phase_sweep(t_arr, q_traj, q0, limits)

    labels = np.asarray([profile["dynamic_label"]] * len(q_traj), dtype=object)
    if profile.get("with_gravity", False):
        q_traj, labels = _append_quasi_static_gravity(q_traj, labels, limits, Config.DT)
    if profile.get("with_com_gravity", False):
        q_traj, labels = _append_com_gravity(q_traj, labels, limits, Config.DT)
    if profile.get("with_inertia_burst", False):
        q_traj, labels = _append_inertia_burst(q_traj, labels, limits, Config.DT)

    q_traj = _limit_joint_ranges(q_traj, q0, limits)
    qd_traj, qdd_traj = _differentiate_trajectory(q_traj, Config.DT)
    t_arr = np.arange(len(q_traj), dtype=np.float64) * Config.DT
    return t_arr, q_traj, qd_traj, qdd_traj, labels

def _apply_specialized_profile(profile_name, t_arr, q_traj, q0, limits):
    if profile_name in ("j7-heavy", "T1", "j7_high_frequency"):
        q = _apply_j7_high_frequency(t_arr, q_traj, q0, limits)
    elif profile_name in ("gravity-scan", "T3", "gravity"):
        q, _ = _append_quasi_static_gravity(np.asarray(q_traj, dtype=np.float64), np.asarray(["dynamic"] * len(q_traj), dtype=object), limits, Config.DT)
    elif profile_name in ("T5", "com_gravity"):
        q, _ = _append_com_gravity(np.asarray(q_traj, dtype=np.float64), np.asarray(["dynamic"] * len(q_traj), dtype=object), limits, Config.DT)
    elif profile_name in ("T6", "inertia", "inertia_burst"):
        q, _ = _append_inertia_burst(np.asarray(q_traj, dtype=np.float64), np.asarray(["dynamic"] * len(q_traj), dtype=object), limits, Config.DT)
    elif profile_name in ("T2", "j6_j7_phase_sweep"):
        q = _apply_j6_j7_phase_sweep(t_arr, q_traj, q0, limits)
    else:
        return q_traj, None, None
    qd, qdd = _differentiate_trajectory(q, Config.DT)
    return q, qd, qdd

def _joint_coverage(q):
    ptp = np.ptp(q, axis=0)
    distal_start = Config.PARAM_ID_DISTAL_LINK_START - 1
    distal = ptp[distal_start:]
    return {
        "min": float(np.min(distal)) if distal.size else 0.0,
        "mean": float(np.mean(distal)) if distal.size else 0.0,
    }

def _candidate_score(overall, distal, inertial_overall, inertial_distal, group_observability, coverage, speed_scale):
    speed_penalty = max(0.0, 1.0 - float(speed_scale))
    condition_penalty = np.log10(max(inertial_overall["condition"], 1.0))
    inertial_projection = inertial_distal["projection"]
    joint_projection = distal["projection"]
    com_obs = group_observability.get("com", {})
    inertia_obs = group_observability.get("inertia", {})
    distal_com_obs = group_observability.get("distal_com", {})
    distal_inertia_obs = group_observability.get("distal_inertia", {})
    com_projection = com_obs.get("projection", {})
    inertia_projection = inertia_obs.get("projection", {})
    distal_com_projection = distal_com_obs.get("projection", {})
    distal_inertia_projection = distal_inertia_obs.get("projection", {})
    return (
        overall["rank"] * 90.0
        + inertial_overall["rank"] * 5.0
        + inertial_distal["rank"] * 30.0
        + inertial_projection["rank"] * 20.0
        + inertial_projection["ratio"] * 260.0
        + joint_projection["ratio"] * 60.0
        + com_obs.get("rank", 0) * 8.0
        + inertia_obs.get("rank", 0) * 7.0
        + distal_com_obs.get("rank", 0) * 18.0
        + distal_inertia_obs.get("rank", 0) * 18.0
        + com_projection.get("rank", 0) * 8.0
        + inertia_projection.get("rank", 0) * 7.0
        + distal_com_projection.get("ratio", 0.0) * 120.0
        + distal_inertia_projection.get("ratio", 0.0) * 180.0
        + np.log10(max(inertial_projection["sigma_min"], 1e-15) / 1e-15) * 2.0
        + np.log10(max(inertial_distal["sigma_min"], 1e-15) / 1e-15)
        + np.log10(max(com_projection.get("sigma_min", 1e-15), 1e-15) / 1e-15)
        + np.log10(max(inertia_projection.get("sigma_min", 1e-15), 1e-15) / 1e-15)
        + min(coverage["mean"], 1.0) * 4.0
        + min(coverage["min"], 0.5) * 6.0
        - inertial_distal["correlation"] * 12.0
        - com_obs.get("correlation", 0.0) * 10.0
        - inertia_obs.get("correlation", 0.0) * 10.0
        - distal["correlation"] * 4.0
        - np.log10(max(inertial_distal["condition"], 1.0)) * 2.0
        - condition_penalty * 0.2
        - speed_penalty * 12.0
    )

def _build_trajectory_records_from_env(env, t_arr, q_actual, q_expected, cycle_time_ms=None):
    t_arr = np.asarray(t_arr, dtype=np.float64)
    q_actual = np.asarray(q_actual, dtype=np.float64)
    q_expected = np.asarray(q_expected if q_expected is not None else q_actual, dtype=np.float64)
    count = min(len(t_arr), len(q_actual), len(q_expected))
    if count <= 0:
        return []

    if cycle_time_ms is None:
        cycle = np.full(count, Config.DT * 1000.0, dtype=np.float64)
    else:
        cycle = np.asarray(cycle_time_ms, dtype=np.float64)
        if cycle.ndim == 0:
            cycle = np.full(count, float(cycle), dtype=np.float64)
        else:
            cycle = cycle[:count]

    saved_qpos = env.data.qpos.copy()
    saved_qvel = env.data.qvel.copy()
    saved_qacc = env.data.qacc.copy()

    def pose_for(q):
        env.set_qpos(q)
        env.set_qvel(np.zeros(Config.NUM_JOINTS, dtype=np.float64))
        env.forward()
        return env.get_ee_pos(), env.get_ee_quat()

    records = []
    try:
        for step in range(count):
            actual_pos, actual_quat = pose_for(q_actual[step])
            expected_pos, expected_quat = pose_for(q_expected[step])
            actual_rpy = np.rad2deg(rerun_viz.quat_to_euler(actual_quat))
            expected_rpy = np.rad2deg(rerun_viz.quat_to_euler(expected_quat))
            pos_err_mm = rerun_viz._position_to_display_units(actual_pos - expected_pos)
            rot_err_deg = rerun_viz.compute_rotation_error_single(actual_quat, expected_quat)
            records.append(
                {
                    "time": float(t_arr[step]),
                    "step": int(step),
                    "actual_x": float(actual_pos[0]),
                    "actual_y": float(actual_pos[1]),
                    "actual_z": float(actual_pos[2]),
                    "expected_x": float(expected_pos[0]),
                    "expected_y": float(expected_pos[1]),
                    "expected_z": float(expected_pos[2]),
                    "actual_roll": float(actual_rpy[0]),
                    "actual_pitch": float(actual_rpy[1]),
                    "actual_yaw": float(actual_rpy[2]),
                    "expected_roll": float(expected_rpy[0]),
                    "expected_pitch": float(expected_rpy[1]),
                    "expected_yaw": float(expected_rpy[2]),
                    "error_x_mm": float(pos_err_mm[0]),
                    "error_y_mm": float(pos_err_mm[1]),
                    "error_z_mm": float(pos_err_mm[2]),
                    "error_roll_deg": float(rot_err_deg[0]),
                    "error_pitch_deg": float(rot_err_deg[1]),
                    "error_yaw_deg": float(rot_err_deg[2]),
                    "cycle_time_ms": float(cycle[min(step, len(cycle) - 1)]),
                }
            )
    finally:
        env.data.qpos[:] = saved_qpos
        env.data.qvel[:] = saved_qvel
        env.data.qacc[:] = saved_qacc
        env.forward()
    return records

def _compute_ee_poses_for_q_traj(env, q_traj):
    q_traj = np.asarray(q_traj, dtype=np.float64)
    count = len(q_traj)
    positions = np.zeros((count, 3), dtype=np.float64)
    quats = np.zeros((count, 4), dtype=np.float64)

    saved_qpos = env.data.qpos.copy()
    saved_qvel = env.data.qvel.copy()
    saved_qacc = env.data.qacc.copy()
    try:
        for step, q in enumerate(q_traj):
            env.set_qpos(q)
            env.set_qvel(np.zeros(Config.NUM_JOINTS, dtype=np.float64))
            env.forward()
            positions[step] = env.get_ee_pos()
            quats[step] = env.get_ee_quat()
    finally:
        env.data.qpos[:] = saved_qpos
        env.data.qvel[:] = saved_qvel
        env.data.qacc[:] = saved_qacc
        env.forward()

    return positions, quats

def _simulate_identification_samples(env, q_traj, qd_traj, qdd_traj, q_ref):
    n_steps = len(q_traj)
    tau_meas = np.zeros((n_steps, 7))
    tau_joint = np.zeros((n_steps, 7))
    q_meas = np.zeros((n_steps, 7))
    qd_meas = np.zeros((n_steps, 7))
    for step in range(n_steps):
        data = env.data
        data.qpos[:7] = q_traj[step]
        data.qvel[:7] = qd_traj[step]
        data.qacc[:7] = qdd_traj[step]
        mujoco.mj_inverse(env.model, data)
        q_meas[step] = data.qpos[:7].copy()
        qd_meas[step] = data.qvel[:7].copy()
        tau_joint[step] = _joint_effect_torque(q_meas[step], qd_meas[step], Config.PARAM_ID_JOINT_PRIORS, q_ref)
        tau_meas[step] = data.qfrc_inverse[:7].copy() + tau_joint[step]
    return q_meas, qd_meas, tau_meas, tau_joint

def _select_excitation_trajectory(backend, env, q0, limits):
    best = None
    candidates = []
    for profile in _trajectory_profiles():
        for seed in _trajectory_seeds()[: Config.PARAM_ID_TRAJECTORY_CANDIDATES]:
            t_arr, q_traj, qd_traj, qdd_traj, labels = _build_planned_trajectory(profile, seed, q0, limits)
            q_limited, qd_limited, qdd_limited, max_ee_speed, speed_scale = limit_ee_speed(
                env, q_traj, qd_traj, qdd_traj, Config.PARAM_ID_MAX_EE_SPEED,
            )
            stride = max(1, len(t_arr) // Config.PARAM_ID_MAX_SAMPLES)
            Y_probe, _ = build_stacked_regressor(
                backend,
                q_limited,
                qd_limited,
                qdd_limited,
                stride=stride,
                include_joint_terms=True,
                q_ref=Config.HOME_QPOS,
                coulomb_eps=Config.PARAM_ID_COULOMB_EPS,
            )
            inertial_Y_probe, _ = build_stacked_regressor(
                backend,
                q_limited,
                qd_limited,
                qdd_limited,
                stride=stride,
                include_joint_terms=False,
                q_ref=Config.HOME_QPOS,
                coulomb_eps=Config.PARAM_ID_COULOMB_EPS,
            )
            overall = _scaled_svd_metrics(Y_probe)
            distal = _distal_observability(Y_probe, include_joint_terms=True)
            inertial_overall = _scaled_svd_metrics(inertial_Y_probe)
            inertial_distal = _distal_observability(inertial_Y_probe, include_joint_terms=False)
            group_observability = _parameter_group_observability(Y_probe)
            coverage = _joint_coverage(q_limited)
            score = _candidate_score(
                overall,
                distal,
                inertial_overall,
                inertial_distal,
                group_observability,
                coverage,
                speed_scale,
            )
            candidate = {
                "score": score,
                "profile": profile["name"],
                "description": profile["description"],
                "seed": seed,
                "t": t_arr,
                "q": q_limited,
                "qd": qd_limited,
                "qdd": qdd_limited,
                "labels": labels,
                "max_ee_speed": max_ee_speed,
                "speed_scale": speed_scale,
                "overall": overall,
                "distal": distal,
                "inertial_overall": inertial_overall,
                "inertial_distal": inertial_distal,
                "group_observability": group_observability,
                "coverage": coverage,
            }
            candidates.append(candidate)

    ranked_candidates = sorted(candidates, key=lambda item: item["score"], reverse=True)
    validation_cases = []
    if ranked_candidates:
        print(f"[辨识] 验证 {len(ranked_candidates)} 个候选 × {len(_regularization_grid())} 组正则的真实联合辨识误差...")
    for cand in ranked_candidates:
        env.reset(q0)
        env.forward()
        q_meas, qd_meas, tau_meas, _tau_joint = _simulate_identification_samples(
            env, cand["q"], cand["qd"], cand["qdd"], Config.HOME_QPOS,
        )
        for mass_lambda, com_lambda, inertia_lambda, joint_lambda in _regularization_grid():
            case = _solve_identification_case(
                (
                    f"候选验证 {cand['profile']} seed={cand['seed']} "
                    f"λm={mass_lambda:.2g} λc={com_lambda:.2g} "
                    f"λI={inertia_lambda:.2g} λj={joint_lambda:.2g}"
                ),
                backend,
                q_meas,
                qd_meas,
                cand["qdd"],
                tau_meas,
                cand["labels"],
                max(1, len(cand["t"]) // Config.PARAM_ID_MAX_SAMPLES),
                Config.HOME_QPOS,
                *(_extract_ground_truth(backend)),
                mass_prior_lambda=mass_lambda,
                com_prior_lambda=com_lambda,
                inertia_prior_lambda=inertia_lambda,
                joint_prior_lambda=joint_lambda,
            )
            case["candidate"] = cand
            validation_cases.append(case)

    if validation_cases:
        validated = sorted(validation_cases, key=_case_selection_key)
        best_case = validated[0]
        best_candidate = best_case["candidate"]
        if Config.PARAM_ID_TRAJECTORY_PROFILE_DIAGNOSTICS:
            print("[辨识] 真实误差验证矩阵Top:")
            for case in validated[:min(Config.PARAM_ID_VALIDATION_TOP_N, len(validated))]:
                cand = case["candidate"]
                summary = case["mass_summary"]
                com_summary = case["com_summary"]
                inertia_summary = case["inertia_summary"]
                sel = case["selection"]
                status = "达标" if summary["passes_5pct"] else "未达标"
                print(
                    f"  {cand['profile']:<2} seed={cand['seed']:<4} {status} "
                    f"max={summary['max_abs']:.2f}%@J{summary['max_abs_joint']} "
                    f"J7={summary['j7_abs']:.2f}% "
                    f"COM={com_summary['max_distance']:.4f}m@J{com_summary['max_distance_joint']} "
                    f"I={inertia_summary['max_component_abs']:.1f}%@J{inertia_summary['max_component_joint']} "
                    f"末端均值={summary['distal_abs_mean']:.2f}% "
                    f"trainRMS={case['prediction_error']:.4f} "
                    f"valRMS={case['validation_rms']:.4f} "
                    f"rank={case['diagnostics'].get('rank', 0):.0f} "
                    f"cond={case['diagnostics'].get('retained_condition', float('inf')):.1f} "
                    f"λm={sel['mass_prior_lambda']:.2g} "
                    f"λc={sel['com_prior_lambda']:.2g} "
                    f"λI={sel['inertia_prior_lambda']:.2g} "
                    f"λj={sel['joint_prior_lambda']:.2g}"
                )
        best = {
            **best_candidate,
            "validated_case": best_case,
        }
        print(
            f"[辨识] 选择激励 {best['profile']} ({best['description']}) seed={best['seed']}, "
            f"验证最大误差={best_case['mass_summary']['max_abs']:.2f}% "
            f"(关节 J{best_case['mass_summary']['max_abs_joint']}), "
            f"COM={best_case['com_summary']['max_distance']:.4f} m, "
            f"惯量={best_case['inertia_summary']['max_component_abs']:.1f}%, "
            f"J7={best_case['mass_summary']['j7_abs']:.2f}%, "
            f"验证RMS={best_case['validation_rms']:.4f}"
        )
        return (
            best["t"], best["q"], best["qd"], best["qdd"], best["max_ee_speed"],
            best["speed_scale"], best["overall"], best["distal"], best["labels"],
            {
                "profile": best["profile"],
                "description": best["description"],
                "seed": best["seed"],
                "score": best["score"],
            },
        )

    best = ranked_candidates[0] if ranked_candidates else None
    if Config.PARAM_ID_TRAJECTORY_PROFILE_DIAGNOSTICS:
        print("[辨识] 激励候选Top:")
        for cand in ranked_candidates[:min(5, len(ranked_candidates))]:
            iproj = cand["inertial_distal"]["projection"]
            jproj = cand["distal"]["projection"]
            cproj = cand["group_observability"]["com"]["projection"]
            iparam_proj = cand["group_observability"]["inertia"]["projection"]
            print(
                f"  {cand['profile']:<3} seed={cand['seed']:<4} score={cand['score']:.2f} "
                f"惯性rank={cand['inertial_overall']['rank']} 惯性末端cond={cand['inertial_distal']['condition']:.1f} "
                f"残差={iproj['ratio']:.3f}/{iproj['rank']} 联合残差={jproj['ratio']:.3f}/{jproj['rank']} "
                f"COM残差={cproj['ratio']:.3f}/{cproj['rank']} 惯量残差={iparam_proj['ratio']:.3f}/{iparam_proj['rank']} "
                f"相关={cand['inertial_distal']['correlation']:.3f} TCP={cand['max_ee_speed']:.3f} "
                f"缩放={cand['speed_scale']:.3f} 覆盖={cand['coverage']['mean']:.3f}"
            )

    print(
        f"[辨识] 选择激励 {best['profile']} ({best['description']}) seed={best['seed']}, "
        f"惯性回归条件数={best['inertial_overall']['condition']:.1f}, "
        f"rank={best['inertial_overall']['rank']}, 末端rank={best['inertial_distal']['rank']}, "
        f"末端条件数={best['inertial_distal']['condition']:.1f}, "
        f"残差={best['inertial_distal']['projection']['ratio']:.3f}, "
        f"末端相关={best['inertial_distal']['correlation']:.3f}"
    )
    return (
        best["t"], best["q"], best["qd"], best["qdd"], best["max_ee_speed"],
        best["speed_scale"], best["overall"], best["distal"], best["labels"],
        {
            "profile": best["profile"],
            "description": best["description"],
            "seed": best["seed"],
            "score": best["score"],
        },
    )

