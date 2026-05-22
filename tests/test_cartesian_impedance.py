from __future__ import annotations

import numpy as np

from robot_control.config import Config
from robot_control.dynamics.cartesian_impedance import CartesianImpedanceController
from robot_control.dynamics.pinocchio import PinocchioGravityBackend
from robot_control.shared.mujoco.env import MujocoSimEnv


class FakeCartesianBackend:
    def __init__(self, *, jacobian=None, tau_gc=None, torque_limits=None):
        self.jacobian = np.zeros((6, Config.NUM_JOINTS), dtype=np.float64) if jacobian is None else np.asarray(
            jacobian, dtype=np.float64
        )
        self.tau_gc = np.zeros(Config.NUM_JOINTS, dtype=np.float64) if tau_gc is None else np.asarray(
            tau_gc, dtype=np.float64
        )
        self._torque_limits = (
            np.ones(Config.NUM_JOINTS, dtype=np.float64) * 100.0
            if torque_limits is None
            else np.asarray(torque_limits, dtype=np.float64)
        )

    def compute_fk(self, q):
        q = np.asarray(q, dtype=np.float64)
        return q[:3].tolist(), [1.0, 0.0, 0.0, 0.0]

    def compute_jacobian(self, q):
        _ = q
        return self.jacobian.copy()

    def compute_nonlinear_effects(self, q, qd):
        _ = q, qd
        return self.tau_gc.copy()


def test_nullspace_torque_is_zero_at_default_target():
    backend = FakeCartesianBackend(tau_gc=np.ones(Config.NUM_JOINTS) * 0.25)
    controller = CartesianImpedanceController(
        backend,
        cartesian_kp=np.zeros(6),
        cartesian_kd=np.zeros(6),
        nullspace_kp=np.ones(Config.NUM_JOINTS),
        nullspace_kd=np.ones(Config.NUM_JOINTS),
    )
    q = Config.NULLSPACE_Q_REF.copy()

    out = controller.compute(q, np.zeros(Config.NUM_JOINTS), np.zeros(3), [1.0, 0.0, 0.0, 0.0], np.zeros(6))

    np.testing.assert_allclose(out.tau_null, 0.0)
    np.testing.assert_allclose(out.tau_total, backend.tau_gc)


def test_task_torque_is_jacobian_transpose_times_wrench():
    jacobian = np.zeros((6, Config.NUM_JOINTS), dtype=np.float64)
    jacobian[0, 0] = 2.0
    backend = FakeCartesianBackend(jacobian=jacobian)
    controller = CartesianImpedanceController(
        backend,
        cartesian_kp=np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        cartesian_kd=np.zeros(6),
        nullspace_kp=np.zeros(Config.NUM_JOINTS),
        nullspace_kd=np.zeros(Config.NUM_JOINTS),
    )

    out = controller.compute(
        np.zeros(Config.NUM_JOINTS),
        np.zeros(Config.NUM_JOINTS),
        np.array([0.1, 0.0, 0.0]),
        [1.0, 0.0, 0.0, 0.0],
        np.zeros(6),
    )

    np.testing.assert_allclose(out.wrench_task, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(out.tau_task, [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_cartesian_impedance_clips_total_torque():
    jacobian = np.zeros((6, Config.NUM_JOINTS), dtype=np.float64)
    jacobian[0, 0] = 1.0
    backend = FakeCartesianBackend(jacobian=jacobian, torque_limits=np.ones(Config.NUM_JOINTS))
    controller = CartesianImpedanceController(
        backend,
        cartesian_kp=np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        cartesian_kd=np.zeros(6),
        nullspace_kp=np.zeros(Config.NUM_JOINTS),
        nullspace_kd=np.zeros(Config.NUM_JOINTS),
    )

    out = controller.compute(
        np.zeros(Config.NUM_JOINTS),
        np.zeros(Config.NUM_JOINTS),
        np.array([1.0, 0.0, 0.0]),
        [1.0, 0.0, 0.0, 0.0],
        np.zeros(6),
    )

    assert out.tau_total[0] == 1.0
    assert out.clipped


def test_joint_reference_generates_cartesian_target_and_twist():
    jacobian = np.zeros((6, Config.NUM_JOINTS), dtype=np.float64)
    jacobian[0, 0] = 1.0
    backend = FakeCartesianBackend(jacobian=jacobian)
    controller = CartesianImpedanceController(
        backend,
        cartesian_kp=np.zeros(6),
        cartesian_kd=np.array([3.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        nullspace_kp=np.zeros(Config.NUM_JOINTS),
        nullspace_kd=np.zeros(Config.NUM_JOINTS),
    )
    qd_ref = np.zeros(Config.NUM_JOINTS)
    qd_ref[0] = 0.5

    out = controller.compute_from_joint_reference(
        np.zeros(Config.NUM_JOINTS),
        np.zeros(Config.NUM_JOINTS),
        np.zeros(Config.NUM_JOINTS),
        qd_ref,
    )

    np.testing.assert_allclose(out.twist_ref, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(out.wrench_task, [1.5, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_pinocchio_backend_exposes_finite_tcp_fk_and_jacobian():
    backend = PinocchioGravityBackend(
        urdf_path=Config.URDF_PATH,
        ee_frame_name="ArmLseventh_Link",
        tcp_offset=Config.TCP_OFFSET,
        torque_limits=Config.TORQUE_LIMITS.tolist(),
    )
    try:
        pos, quat = backend.compute_fk(Config.HOME_QPOS)
        jacobian = backend.compute_jacobian(Config.HOME_QPOS)
    finally:
        backend.close()

    assert np.asarray(pos).shape == (3,)
    assert np.asarray(quat).shape == (4,)
    assert jacobian.shape == (6, Config.NUM_JOINTS)
    assert np.isfinite(pos).all()
    assert np.isfinite(quat).all()
    assert np.isfinite(jacobian).all()


def test_default_cartesian_impedance_holds_home_without_saturation():
    backend = PinocchioGravityBackend(
        urdf_path=Config.URDF_PATH,
        ee_frame_name="ArmLseventh_Link",
        tcp_offset=Config.TCP_OFFSET,
        torque_limits=Config.TORQUE_LIMITS.tolist(),
    )
    env = MujocoSimEnv(Config.URDF_PATH)
    controller = CartesianImpedanceController(backend)

    try:
        env.reset(Config.HOME_QPOS)
        env.forward()
        pos_ref, quat_ref = backend.compute_fk(Config.HOME_QPOS)
        max_qd = 0.0
        max_pos_error = 0.0
        clipped_steps = 0
        max_tau = 0.0
        for _ in range(250):
            q = env.get_qpos()
            qd = env.get_qvel()
            out = controller.compute(q, qd, pos_ref, quat_ref, np.zeros(6))
            clipped_steps += int(out.clipped)
            max_tau = max(max_tau, float(np.max(np.abs(out.tau_total))))
            max_qd = max(max_qd, float(np.max(np.abs(qd))))
            max_pos_error = max(max_pos_error, float(np.linalg.norm(out.pose_error[:3])))
            env.apply_torque(out.tau_total)
            env.step()
            env.enforce_joint_limits()
    finally:
        backend.close()

    assert clipped_steps == 0
    assert max_tau < float(np.max(Config.TORQUE_LIMITS))
    assert max_qd < 0.05
    assert max_pos_error < 1e-3
