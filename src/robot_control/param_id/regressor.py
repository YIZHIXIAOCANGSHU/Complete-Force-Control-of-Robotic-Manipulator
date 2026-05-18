"""Pinocchio-based regressor computation for parameter identification.

Extracts rigid-body inertial columns and appends joint friction, stiffness,
viscous, and offset terms.
"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np

from robot_control.config import Config
from robot_control.dynamics.pinocchio import PinocchioGravityBackend, q7_to_q8


# Inertial parameter index layout (Pinocchio convention):
#  0: mass
#  1: mcx (mass * com_x)
#  2: mcy (mass * com_y)
#  3: mcz (mass * com_z)
#  4: Ixx
#  5: Ixy
#  6: Iyy
#  7: Ixz
#  8: Iyz
#  9: Izz
_PARAM_NAMES_PER_LINK = [
    "mass", "mcx", "mcy", "mcz",
    "Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz",
]

# We only identify mass, COM, and diagonal inertia (7 params / link).
IDENTIFIED_INDICES = [0, 1, 2, 3, 4, 6, 9]  # mass, mcx, mcy, mcz, Ixx, Iyy, Izz
IDENTIFIED_NAMES = [_PARAM_NAMES_PER_LINK[i] for i in IDENTIFIED_INDICES]
JOINT_TERM_NAMES = ["fc", "k", "fv", "fo"]


def build_joint_term_regressor(
    q: np.ndarray,
    qd: np.ndarray,
    q_ref: np.ndarray | None = None,
    coulomb_eps: float = 0.02,
) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    qd = np.asarray(qd, dtype=np.float64)
    if q_ref is None:
        q_ref = Config.HOME_QPOS
    q_ref = np.asarray(q_ref, dtype=np.float64)

    Y_joint = np.zeros((Config.NUM_JOINTS, Config.NUM_JOINTS * len(JOINT_TERM_NAMES)))
    coulomb = np.tanh(qd / max(float(coulomb_eps), 1e-9))
    stiffness = q - q_ref
    for joint in range(Config.NUM_JOINTS):
        col = joint * len(JOINT_TERM_NAMES)
        Y_joint[joint, col + 0] = coulomb[joint]
        Y_joint[joint, col + 1] = stiffness[joint]
        Y_joint[joint, col + 2] = qd[joint]
        Y_joint[joint, col + 3] = 1.0
    return Y_joint


def joint_term_param_names() -> List[str]:
    names = []
    for joint in range(Config.NUM_JOINTS):
        for name in JOINT_TERM_NAMES:
            names.append(f"J{joint + 1}_{name}")
    return names


def build_regressor(
    backend: PinocchioGravityBackend,
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    include_joint_terms: bool = True,
    q_ref: np.ndarray | None = None,
    coulomb_eps: float = 0.02,
) -> np.ndarray:
    """Build 7 × n_params regressor matrix for a single time step.

    Parameters
    ----------
    q, qd, qdd : 1-D arrays of length 7
    """
    import pinocchio as pin

    q8 = q7_to_q8(q)
    model = backend._model
    data = backend._data

    Y_full = pin.computeJointTorqueRegressor(model, data, q8, qd, qdd)

    n_links = 7
    params_per_link = 10
    identified_per_link = len(IDENTIFIED_INDICES)

    Y_inertial = np.zeros((7, n_links * identified_per_link))
    for link in range(n_links):
        src_start = link * params_per_link
        dst_start = link * identified_per_link
        for k, src_idx in enumerate(IDENTIFIED_INDICES):
            Y_inertial[:, dst_start + k] = Y_full[:, src_start + src_idx]

    if not include_joint_terms:
        return Y_inertial

    Y_joint = build_joint_term_regressor(q, qd, q_ref=q_ref, coulomb_eps=coulomb_eps)
    return np.hstack([Y_inertial, Y_joint])


def build_stacked_regressor(
    backend: PinocchioGravityBackend,
    q_seq: np.ndarray,
    qd_seq: np.ndarray,
    qdd_seq: np.ndarray,
    stride: int = 1,
    include_joint_terms: bool = True,
    q_ref: np.ndarray | None = None,
    coulomb_eps: float = 0.02,
) -> Tuple[np.ndarray, List[str]]:
    """Stack regressor from a trajectory.

    Returns (Y_stack, param_names).
    """
    n_samples = q_seq.shape[0]
    n_inertial_params = 7 * len(IDENTIFIED_INDICES)
    n_joint_params = Config.NUM_JOINTS * len(JOINT_TERM_NAMES) if include_joint_terms else 0
    n_params = n_inertial_params + n_joint_params
    samples_used = (n_samples + stride - 1) // stride

    Y_stack = np.zeros((samples_used * 7, n_params))

    row = 0
    for i in range(0, n_samples, stride):
        Y = build_regressor(
            backend, q_seq[i], qd_seq[i], qdd_seq[i],
            include_joint_terms=include_joint_terms,
            q_ref=q_ref,
            coulomb_eps=coulomb_eps,
        )
        Y_stack[row : row + 7, :] = Y
        row += 7

    param_names = []
    for link in range(7):
        for name in IDENTIFIED_NAMES:
            param_names.append(f"L{link}_{name}")
    if include_joint_terms:
        param_names.extend(joint_term_param_names())

    return Y_stack, param_names
