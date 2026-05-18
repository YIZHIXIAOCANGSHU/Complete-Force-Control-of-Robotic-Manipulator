from __future__ import annotations

import numpy as np
import pytest

from robot_control.config import Config
from robot_control.dynamics.gravity import GravityCompTool
import robot_control.modes.control_sim.mujoco_can_backend as udp_server


def test_simulation_default_timestep_is_python_owned():
    assert not hasattr(Config, "CONTROL_DT")
    assert Config.DT == 0.002
    assert Config.SIM_REALTIME is True
    assert not hasattr(Config, "MUJOCO_DOF_DAMPING")


def test_sleep_until_next_step_paces_realtime_loop(monkeypatch):
    perf_values = iter([10.000, 10.001])
    sleeps: list[float] = []

    monkeypatch.setattr(udp_server.time, "perf_counter", lambda: next(perf_values))
    monkeypatch.setattr(udp_server.time, "sleep", sleeps.append)

    next_step_time = udp_server._sleep_until_next_step(10.002, 0.002, True)

    assert sleeps == [pytest.approx(0.002)]
    assert next_step_time == pytest.approx(10.004)


def test_sleep_until_next_step_can_run_unpaced_for_batch_tests(monkeypatch):
    sleeps: list[float] = []
    monkeypatch.setattr(udp_server.time, "sleep", sleeps.append)

    next_step_time = udp_server._sleep_until_next_step(10.002, 0.002, False)

    assert sleeps == []
    assert next_step_time == 10.002


def test_backend_qd_ref_uses_path_speed_without_control_dt():
    tool = GravityCompTool()
    try:
        target_pos, target_quat = tool.compute_fk(Config.INIT_QPOS.tolist())
        q = Config.HOME_QPOS.tolist()
        qd = [0.0] * Config.NUM_JOINTS
        output = None
        for _ in range(80):
            output = tool.compute(q, qd, target_pos, target_quat)
            if max(abs(value) for value in output.qd_ref) > 0.05:
                break
    finally:
        tool.close()

    assert output is not None
    assert output.status == 0
    assert max(abs(value) for value in output.qd_ref) > 0.05
    assert max(abs(value) for value in output.qd_ref) < 10.0


def test_backend_qd_ref_goes_zero_at_path_end():
    tool = GravityCompTool()
    try:
        home_pos, _home_quat = tool.compute_fk(Config.HOME_QPOS.tolist())
        target_pos, target_quat = tool.compute_fk(Config.INIT_QPOS.tolist())
        path_length = float(np.linalg.norm(np.array(target_pos) - np.array(home_pos)))
        q = Config.HOME_QPOS.tolist()
        qd = [0.0] * Config.NUM_JOINTS
        output = None
        for _ in range(2500):
            output = tool.compute(q, qd, target_pos, target_quat)
            q = list(output.q_ref)
            qd = [0.0] * Config.NUM_JOINTS
        assert output is not None
    finally:
        tool.close()

    assert output.status in (0, 1)
    assert output.path_progress == pytest.approx(path_length, abs=1e-4)
    assert np.linalg.norm(np.array(output.ee_pos) - np.array(target_pos)) < 0.003
    assert max(abs(value) for value in output.qd_ref) == pytest.approx(0.0)


def test_backend_uses_best_effort_ik_result_when_orientation_does_not_converge():
    tool = GravityCompTool()
    try:
        q = Config.HOME_QPOS.tolist()
        target_pos, _target_quat = tool.compute_fk(q)
        output = tool.compute(q, [0.0] * Config.NUM_JOINTS, target_pos, [0.0, 1.0, 0.0, 0.0])
    finally:
        tool.close()

    assert output.status == 1
    assert np.all(np.isfinite(output.q_ref))
    assert np.linalg.norm(np.array(output.q_ref) - np.array(q)) > 0.01
    assert max(abs(value) for value in output.qd_ref) == pytest.approx(0.0)
