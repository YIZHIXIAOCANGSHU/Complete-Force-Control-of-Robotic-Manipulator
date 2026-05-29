from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "python"))

from config import Config
from real_pinocchio.controller_bridge import PinocchioRealControllerBridge


class FakeArmBackend:
    def __init__(self, tau_value: float) -> None:
        self.tau_value = float(tau_value)
        self.calls = []

    def compute(self, q, qd, target_pos, target_quat, body_q=None):
        self.calls.append(
            {
                "q": np.asarray(q, dtype=np.float64).copy(),
                "qd": np.asarray(qd, dtype=np.float64).copy(),
                "target_pos": np.asarray(target_pos, dtype=np.float64).copy(),
                "target_quat": np.asarray(target_quat, dtype=np.float64).copy(),
                "body_q": None
                if body_q is None
                else np.asarray(body_q, dtype=np.float64).copy(),
            }
        )
        return {
            "status": 0,
            "tau": np.full(Config.ARM_JOINTS, self.tau_value, dtype=np.float64),
            "ee_pos": np.array([self.tau_value, 0.2, 0.3], dtype=np.float64),
            "ee_quat": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        }

    def compute_fk(self, q, body_q=None):
        _ = body_q
        return (
            np.array([self.tau_value, 0.0, 0.0], dtype=np.float64),
            np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        )

    def close(self):
        pass


def _inputs():
    q = np.arange(Config.NUM_JOINTS, dtype=np.float64) * 0.01
    qd = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
    body_q = np.zeros(Config.NUM_BODY_JOINTS, dtype=np.float64)
    target_pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
    target_quat = np.tile([1.0, 0.0, 0.0, 0.0], (Config.NUM_ARMS, 1))
    return q, qd, body_q, target_pos, target_quat


def test_pinocchio_bridge_left_mask_zeroes_right_arm():
    left = FakeArmBackend(1.0)
    right = FakeArmBackend(2.0)
    bridge = PinocchioRealControllerBridge(left_backend=left, right_backend=right)
    q, qd, body_q, target_pos, target_quat = _inputs()

    out = bridge.compute(1 << Config.LEFT_ARM, 0.001, q, qd, body_q, target_pos, target_quat)

    assert out.status == 0
    np.testing.assert_allclose(out.tau[: Config.ARM_JOINTS], 1.0)
    np.testing.assert_allclose(out.tau[Config.ARM_JOINTS :], 0.0)
    assert len(left.calls) == 1
    assert right.calls == []
    assert out.ee_pos.shape == (Config.NUM_ARMS, 3)
    assert out.ee_quat.shape == (Config.NUM_ARMS, 4)


def test_pinocchio_bridge_right_mask_zeroes_left_arm():
    left = FakeArmBackend(1.0)
    right = FakeArmBackend(2.0)
    bridge = PinocchioRealControllerBridge(left_backend=left, right_backend=right)
    q, qd, body_q, target_pos, target_quat = _inputs()

    out = bridge.compute(1 << Config.RIGHT_ARM, 0.001, q, qd, body_q, target_pos, target_quat)

    assert out.status == 0
    np.testing.assert_allclose(out.tau[: Config.ARM_JOINTS], 0.0)
    np.testing.assert_allclose(out.tau[Config.ARM_JOINTS :], 2.0)
    assert left.calls == []
    assert len(right.calls) == 1


def test_pinocchio_bridge_nonfinite_input_returns_safety_status_and_zero_tau():
    bridge = PinocchioRealControllerBridge(
        left_backend=FakeArmBackend(1.0),
        right_backend=FakeArmBackend(2.0),
    )
    q, qd, body_q, target_pos, target_quat = _inputs()
    q[Config.ARM_JOINTS] = np.nan

    out = bridge.compute(1 << Config.RIGHT_ARM, 0.001, q, qd, body_q, target_pos, target_quat)

    assert out.status < 0
    np.testing.assert_allclose(out.tau, 0.0)


def test_pinocchio_bridge_fk_only_returns_pose_and_zero_torque():
    bridge = PinocchioRealControllerBridge(
        left_backend=FakeArmBackend(1.0),
        right_backend=FakeArmBackend(2.0),
    )
    q, _, body_q, _, _ = _inputs()

    out = bridge.compute_fk((1 << Config.LEFT_ARM) | (1 << Config.RIGHT_ARM), q, body_q)

    assert out.status == 0
    np.testing.assert_allclose(out.tau, 0.0)
    np.testing.assert_allclose(out.ee_pos[:, 0], [1.0, 2.0])


def test_pinocchio_runtime_delegates_to_shared_real_runtime(monkeypatch):
    from real_pinocchio import runtime as pin_runtime

    calls = []

    class FakeBridge:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    def fake_run_real_control_with_bridge(arm, **kwargs):
        calls.append({"arm": arm, "kwargs": kwargs})

    monkeypatch.setattr(pin_runtime, "PinocchioRealControllerBridge", FakeBridge)
    monkeypatch.setattr(pin_runtime, "verify_pinocchio_runtime_available", lambda: None)
    monkeypatch.setattr(
        pin_runtime,
        "run_real_control_with_bridge",
        fake_run_real_control_with_bridge,
    )

    pin_runtime.run_real_pinocchio_control("right")

    assert calls
    assert calls[0]["arm"] == "right"
    bridge = calls[0]["kwargs"]["bridge_factory"]()
    assert isinstance(bridge, FakeBridge)
    assert bridge.kwargs["active_arm_mask"] == (1 << Config.RIGHT_ARM)
    assert "Pinocchio" in calls[0]["kwargs"]["control_title"]


def test_pinocchio_runtime_exits_before_bridge_when_import_probe_fails(monkeypatch, capsys):
    from real_pinocchio import runtime as pin_runtime

    def fail_probe():
        raise pin_runtime.PinocchioUnavailableError("pinocchio import failed")

    class ExplodingBridge:
        def __init__(self, **kwargs):
            raise AssertionError("bridge should not be constructed")

    monkeypatch.setattr(pin_runtime, "verify_pinocchio_runtime_available", fail_probe)
    monkeypatch.setattr(pin_runtime, "PinocchioRealControllerBridge", ExplodingBridge)

    try:
        pin_runtime.run_real_pinocchio_control("right")
    except SystemExit as exc:
        assert exc.code == 2
    else:
        raise AssertionError("expected SystemExit")

    captured = capsys.readouterr()
    assert "pinocchio import failed" in captured.err
