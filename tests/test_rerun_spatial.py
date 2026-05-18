from __future__ import annotations

import numpy as np
import pytest

from robot_control.shared.rerun import spatial


class DummyRR:
    def __init__(self) -> None:
        self.logs = []
        self.times = []

    def set_time_seconds(self, timeline: str, value: float) -> None:
        self.times.append((timeline, value))

    def log(self, path: str, payload) -> None:
        self.logs.append((path, payload))

    def Points3D(self, points, **kwargs):
        return {"kind": "Points3D", "points": points, **kwargs}

    def LineStrips3D(self, strips, **kwargs):
        return {"kind": "LineStrips3D", "strips": strips, **kwargs}

    def Arrows3D(self, **kwargs):
        return {"kind": "Arrows3D", **kwargs}

    def Scalars(self, value: float) -> float:
        return value


def test_spatial_logs_pose_trajectory_and_error_vector(monkeypatch):
    dummy = DummyRR()
    monkeypatch.setattr(spatial, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(spatial, "rr", dummy)

    spatial.log_ee_pose("/control_sim/spatial", 0.5, np.array([1.0, 2.0, 3.0]))
    spatial.log_ee_trajectory("control_sim/spatial", np.zeros((2, 3)))
    spatial.log_error_vector("control_sim/spatial", np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))

    paths = [path for path, _payload in dummy.logs]
    assert "control_sim/spatial/ee_pose" in paths
    assert "control_sim/spatial/ee_trajectory" in paths
    assert "control_sim/spatial/ee_error_vector" in paths
    assert dummy.times == [("time", 0.5)]


def test_spatial_validates_point_shapes(monkeypatch):
    monkeypatch.setattr(spatial, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(spatial, "rr", DummyRR())

    with pytest.raises(ValueError, match="position"):
        spatial.log_ee_pose("x", 0.0, np.zeros(2))

