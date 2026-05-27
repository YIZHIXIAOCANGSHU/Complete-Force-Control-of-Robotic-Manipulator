from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import rerun_viz


class DummyRR:
    def __init__(self) -> None:
        self.logs: list[tuple[str, object, bool]] = []
        self.blueprint = None
        self.time_calls: list[tuple[str, float]] = []

    def init(self, *_args, **_kwargs):
        return None

    def set_time_seconds(self, timeline: str, value: float) -> None:
        self.time_calls.append((timeline, value))

    def log(self, path: str, payload, static: bool = False) -> None:
        self.logs.append((path, payload, static))

    def send_blueprint(self, blueprint) -> None:
        self.blueprint = blueprint

    def Scalars(self, value: float) -> float:
        return value

    def SeriesLines(self, **kwargs):
        return {"kind": "SeriesLines", **kwargs}

    def TextLog(self, text: str):
        return {"kind": "TextLog", "text": text}

    def Points3D(self, points, **kwargs):
        return {"kind": "Points3D", "points": points, **kwargs}

    def LineStrips3D(self, strips, **kwargs):
        return {"kind": "LineStrips3D", "strips": strips, **kwargs}

    def Arrows3D(self, **kwargs):
        return {"kind": "Arrows3D", **kwargs}


class DummyRRB:
    @staticmethod
    def TimeSeriesView(name: str, origin: str):
        return {"kind": "TimeSeriesView", "name": name, "origin": origin}

    @staticmethod
    def TextLogView(name: str, origin: str):
        return {"kind": "TextLogView", "name": name, "origin": origin}

    @staticmethod
    def Spatial3DView(name: str, origin: str):
        return {"kind": "Spatial3DView", "name": name, "origin": origin}

    @staticmethod
    def Horizontal(*children, name: str | None = None):
        return {"kind": "Horizontal", "name": name, "children": list(children)}

    @staticmethod
    def Vertical(*children, name: str | None = None):
        return {"kind": "Vertical", "name": name, "children": list(children)}

    @staticmethod
    def Tabs(*children, name: str | None = None):
        return {"kind": "Tabs", "name": name, "children": list(children)}

    @staticmethod
    def Blueprint(root, collapse_panels: bool = False):
        return {
            "kind": "Blueprint",
            "root": root,
            "collapse_panels": collapse_panels,
        }


def _iter_nodes(node):
    if isinstance(node, dict):
        yield node
        for value in node.values():
            yield from _iter_nodes(value)
    elif isinstance(node, list):
        for item in node:
            yield from _iter_nodes(item)


def _logged_scalar(dummy_rr: DummyRR, path: str) -> float:
    for logged_path, payload, _static in dummy_rr.logs:
        if logged_path == path:
            return payload
    raise AssertionError(f"Missing log for {path}")


def test_setup_realtime_styles_labels_position_views_in_mm(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)
    monkeypatch.setattr(rerun_viz, "rrb", DummyRRB)

    rerun_viz.setup_realtime_styles()

    names_by_origin = {
        node["origin"]: node["name"]
        for node in _iter_nodes(dummy_rr.blueprint)
        if node.get("kind") == "TimeSeriesView"
    }

    assert names_by_origin["/arms/left/position/X"] == "Left Position X (mm)"
    assert names_by_origin["/arms/left/position/Y"] == "Left Position Y (mm)"
    assert names_by_origin["/arms/left/position/Z"] == "Left Position Z (mm)"
    assert names_by_origin["/arms/right/position/X"] == "Right Position X (mm)"
    assert names_by_origin["/arms/left/rotation/Roll"] == "Left Rotation Roll (deg)"
    assert names_by_origin["/arms/right/rotation/Yaw"] == "Right Rotation Yaw (deg)"
    assert names_by_origin["/arms/left/position_error/X"] == "Left Position Error X (mm)"
    assert names_by_origin["/arms/right/position_error/Z"] == "Right Position Error Z (mm)"
    assert names_by_origin["/arms/left/rotation_error/Roll"] == "Left Rotation Error Roll (deg)"
    assert names_by_origin["/arms/right/rotation_error/Yaw"] == "Right Rotation Error Yaw (deg)"
    assert names_by_origin["/arms/left/joint_q/J1"] == "Left Joint Q J1 (rad)"
    assert names_by_origin["/arms/right/joint_q/J7"] == "Right Joint Q J7 (rad)"
    assert names_by_origin["/arms/left/joint_qd/J1"] == "Left Joint QD J1 (rad/s)"
    assert names_by_origin["/arms/right/joint_qd/J7"] == "Right Joint QD J7 (rad/s)"
    assert names_by_origin["/arms/left/torque/J1"] == "Left Torque J1 (N*m)"
    assert names_by_origin["/arms/right/torque/J7"] == "Right Torque J7 (N*m)"
    assert names_by_origin["/arms/left/torque_gap/J1"] == "Left Torque Gap J1 (N*m)"
    assert names_by_origin["/arms/right/torque_gap/J7"] == "Right Torque Gap J7 (N*m)"
    assert "/arms/left/position" not in names_by_origin
    assert "/arms/left/rotation_error" not in names_by_origin
    assert "/arms/left/joint_q" not in names_by_origin
    assert "/arms/left/torque" not in names_by_origin
    assert "/tracking/pos/X" not in names_by_origin
    assert "/tracking/rot/Roll" not in names_by_origin
    assert "/error/X" not in names_by_origin

    tab_names = {
        node["name"]
        for node in _iter_nodes(dummy_rr.blueprint)
        if node.get("kind") in ("Vertical", "Spatial3DView", "TimeSeriesView", "TextLogView") and node.get("name")
    }
    expected_tabs = {
        "3D",
        "Left Position",
        "Right Position",
        "Left Rotation",
        "Right Rotation",
        "Left Position Error",
        "Right Position Error",
        "Left Rotation Error",
        "Right Rotation Error",
        "Left Joint Q",
        "Right Joint Q",
        "Left Joint QD",
        "Right Joint QD",
        "Left Torque",
        "Right Torque",
        "Left Torque Gap",
        "Right Torque Gap",
        "Performance",
        "Control Link",
    }
    assert expected_tabs <= tab_names
    assert "Left Arm" not in tab_names
    assert "Right Arm" not in tab_names


def test_log_realtime_step_logs_position_tracking_in_mm(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)

    rerun_viz.log_realtime_step(
        t=0.1,
        pos_actual=np.array([[0.123, 0.0, -0.001], [0.010, 0.020, 0.030]]),
        pos_desired=np.array([[0.100, -0.002, -0.003], [0.001, 0.002, 0.003]]),
        quat_actual=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        quat_desired=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        tau_total=np.zeros(rerun_viz.Config.NUM_JOINTS),
        cycle_time=1.25,
        step_count=0,
    )

    assert _logged_scalar(dummy_rr, "arms/left/position/X/actual") == 123.0
    assert _logged_scalar(dummy_rr, "arms/left/position/X/target") == 100.0
    assert _logged_scalar(dummy_rr, "arms/left/position_error/X/value") == 23.0
    assert _logged_scalar(dummy_rr, "arms/right/position/X/actual") == 10.0
    assert _logged_scalar(dummy_rr, "arms/right/position/X/target") == 1.0


def test_log_realtime_step_logs_torque_gap_by_arm(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)

    tau_total = np.arange(14, dtype=np.float64)
    tau_actual = tau_total - 0.5
    rerun_viz.log_realtime_step(
        t=0.1,
        pos_actual=np.zeros((2, 3), dtype=np.float64),
        pos_desired=np.zeros((2, 3), dtype=np.float64),
        quat_actual=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        quat_desired=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        tau_total=tau_total,
        tau_actual=tau_actual,
        cycle_time=1.25,
        step_count=0,
    )

    assert _logged_scalar(dummy_rr, "arms/left/torque_gap/J1/value") == 0.5
    assert _logged_scalar(dummy_rr, "arms/right/torque_gap/J1/value") == 0.5
    assert _logged_scalar(dummy_rr, "arms/left/torque_actual/J1/value") == -0.5
    assert _logged_scalar(dummy_rr, "arms/right/torque_actual/J1/value") == 6.5
