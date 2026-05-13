"""Fourier-series excitation trajectory generation.

Produces smooth, band-limited joint-space trajectories that excite all
identifiable inertial parameters so the regressor matrix is well-conditioned.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def fourier_trajectory(
    q0: np.ndarray,
    n_harmonics: int = 5,
    base_freq: float = 0.2,  # Hz
    duration: float = 10.0,   # seconds
    dt: float = 0.002,
    joint_limits: Tuple[np.ndarray, np.ndarray] = None,
    random_seed: int = 42,
    joint_amplitude_weights: np.ndarray | None = None,
    joint_frequency_weights: np.ndarray | None = None,
    phase_offsets: np.ndarray | None = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Generate a Fourier-series excitation trajectory for all joints.

    q_i(t) = q0_i + Σ_k a_{i,k}/(k ω) sin(k ω t + φ_i) - b_{i,k}/(k ω) cos(k ω t + φ_i)

    Returns (t, q, qd, qdd) arrays of shape (n_steps,), (n_steps, n_joints), etc.
    """
    n_joints = len(q0)
    q0 = np.asarray(q0, dtype=np.float64)
    omega_f = 2.0 * math.pi * base_freq
    n_steps = int(duration / dt) + 1
    t = np.linspace(0.0, duration, n_steps)

    rng = np.random.default_rng(random_seed)
    amp_weights = np.ones(n_joints, dtype=np.float64)
    freq_weights = np.ones(n_joints, dtype=np.float64)
    phases = np.zeros(n_joints, dtype=np.float64)
    if joint_amplitude_weights is not None:
        amp_weights = np.asarray(joint_amplitude_weights, dtype=np.float64)
    if joint_frequency_weights is not None:
        freq_weights = np.asarray(joint_frequency_weights, dtype=np.float64)
    if phase_offsets is not None:
        phases = np.asarray(phase_offsets, dtype=np.float64)
    amp_weights = np.resize(amp_weights, n_joints)
    freq_weights = np.maximum(np.resize(freq_weights, n_joints), 0.2)
    phases = np.resize(phases, n_joints)
    a = np.zeros((n_joints, n_harmonics))
    b = np.zeros((n_joints, n_harmonics))

    for j in range(n_joints):
        a[j] = rng.uniform(-0.3, 0.3, n_harmonics) * amp_weights[j]
        b[j] = rng.uniform(-0.3, 0.3, n_harmonics) * amp_weights[j]

    q = np.zeros((n_steps, n_joints))
    qd = np.zeros((n_steps, n_joints))
    qdd = np.zeros((n_steps, n_joints))

    for j in range(n_joints):
        q[:, j] = q0[j]
        joint_omega_f = omega_f * freq_weights[j]
        phase = phases[j]
        for k in range(1, n_harmonics + 1):
            kw = k * joint_omega_f
            sin_kwt = np.sin(kw * t + phase)
            cos_kwt = np.cos(kw * t + phase)
            q[:, j] += (a[j, k - 1] / kw) * sin_kwt - (b[j, k - 1] / kw) * cos_kwt
            qd[:, j] += a[j, k - 1] * cos_kwt + b[j, k - 1] * sin_kwt
            qdd[:, j] += (-a[j, k - 1] * kw) * sin_kwt + (b[j, k - 1] * kw) * cos_kwt

    if joint_limits is not None:
        q_min, q_max = joint_limits
        for j in range(n_joints):
            amp = np.max(np.abs(q[:, j] - q0[j]))
            if amp > 0.0:
                available = min(
                    abs(q_max[j] - q0[j]),
                    abs(q0[j] - q_min[j]),
                )
                if amp > 0.8 * available:
                    scale = 0.8 * available / amp
                    q[:, j] = q0[j] + (q[:, j] - q0[j]) * scale
                    qd[:, j] *= scale
                    qdd[:, j] *= scale

    return t, q, qd, qdd


def compute_ee_speeds(env, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
    speeds = np.zeros(q.shape[0], dtype=np.float64)
    for i in range(q.shape[0]):
        env.set_qpos(q[i])
        env.set_qvel(qd[i])
        env.forward()
        linear_jac = env.get_jacobian_7dof()[:3]
        speeds[i] = float(np.linalg.norm(linear_jac @ qd[i]))
    return speeds


def limit_ee_speed(
    env,
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    max_speed: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
    speeds = compute_ee_speeds(env, q, qd)
    peak = float(np.max(speeds)) if speeds.size else 0.0
    if peak <= max_speed or peak <= 1e-12:
        return q, qd, qdd, peak, 1.0

    scale = float(max_speed / peak)
    q_limited = q[0] + (q - q[0]) * scale
    qd_limited = qd * scale
    qdd_limited = qdd * scale
    limited_peak = float(np.max(compute_ee_speeds(env, q_limited, qd_limited)))
    return q_limited, qd_limited, qdd_limited, limited_peak, scale
