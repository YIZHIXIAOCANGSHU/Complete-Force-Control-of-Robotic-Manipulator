from __future__ import annotations

import numpy as np

from robot_control.shared.rerun import viz as rerun_viz


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

    assert names_by_origin["/tracking/pos/X"] == "EE Position X (mm)"
    assert names_by_origin["/tracking/pos/Y"] == "EE Position Y (mm)"
    assert names_by_origin["/tracking/pos/Z"] == "EE Position Z (mm)"
    assert names_by_origin["/joint_target/q"] == "Target Joint Positions (q_ref, rad)"

    static_line_names = {
        path: payload["names"]
        for path, payload, static in dummy_rr.logs
        if static and payload.get("kind") == "SeriesLines"
    }
    assert static_line_names["joint_target/q/J1"] == ["J1 target"]


def test_setup_sim_realtime_styles_groups_joint_torques_as_small_views(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)
    monkeypatch.setattr(rerun_viz, "rrb", DummyRRB)

    rerun_viz.setup_sim_realtime_styles()

    names_by_origin = {
        node["origin"]: node["name"]
        for node in _iter_nodes(dummy_rr.blueprint)
        if node.get("kind") == "TimeSeriesView"
    }
    layout_names = {
        node["name"]
        for node in _iter_nodes(dummy_rr.blueprint)
        if node.get("kind") in {"Horizontal", "Vertical"} and node.get("name")
    }

    assert "Sim Joint Torque Input" in layout_names
    assert "J1-J4 Torque" in layout_names
    assert "J5-J7 Torque" in layout_names
    assert "UART Protocol" not in layout_names
    assert "Performance Rates" not in layout_names

    for joint_idx in range(rerun_viz.Config.NUM_JOINTS):
        origin = f"/sim/control/torque/J{joint_idx + 1}"
        assert names_by_origin[origin].startswith(f"J{joint_idx + 1}")
        assert "Received/Applied Torque" in names_by_origin[origin]

    static_line_names = {
        path: payload["names"]
        for path, payload, static in dummy_rr.logs
        if static and payload.get("kind") == "SeriesLines"
    }
    assert static_line_names["sim/control/torque/J1/received"] == ["Received torque"]
    assert static_line_names["sim/control/torque/J1/applied"] == ["Applied after limit"]


def test_log_realtime_step_logs_position_tracking_in_mm(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)

    rerun_viz.log_realtime_step(
        t=0.1,
        pos_actual=np.array([0.123, 0.0, -0.001]),
        pos_desired=np.array([0.100, -0.002, -0.003]),
        quat_actual=np.array([1.0, 0.0, 0.0, 0.0]),
        quat_desired=np.array([1.0, 0.0, 0.0, 0.0]),
        tau_total=np.zeros(rerun_viz.Config.NUM_JOINTS),
        cycle_time=1.25,
        step_count=0,
    )

    assert _logged_scalar(dummy_rr, "tracking/pos/X/actual") == 123.0
    assert _logged_scalar(dummy_rr, "tracking/pos/X/desired") == 100.0
    assert _logged_scalar(dummy_rr, "error/X") == 23.0


def test_log_realtime_step_logs_target_joint_positions(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)

    q_target = np.array([0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77])

    rerun_viz.log_realtime_step(
        t=0.1,
        pos_actual=np.array([0.123, 0.0, -0.001]),
        pos_desired=np.array([0.100, -0.002, -0.003]),
        quat_actual=np.array([1.0, 0.0, 0.0, 0.0]),
        quat_desired=np.array([1.0, 0.0, 0.0, 0.0]),
        tau_total=np.zeros(rerun_viz.Config.NUM_JOINTS),
        cycle_time=1.25,
        q_target=q_target,
        step_count=0,
    )

    assert _logged_scalar(dummy_rr, "joint_target/q/J1") == 0.11
    assert _logged_scalar(dummy_rr, "joint_target/q/J7") == 0.77


def test_log_sim_realtime_step_logs_received_and_applied_torque_per_joint(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)

    tau_received = np.array([10.0, -20.0, 30.0, -40.0, 8.0, -9.0, 11.0])
    tau_applied = np.array([10.0, -20.0, 27.0, -27.0, 7.0, -7.0, 9.0])

    rerun_viz.log_sim_realtime_step(
        t=0.1,
        pos_actual=np.array([0.123, 0.0, -0.001]),
        pos_desired=np.array([0.100, -0.002, -0.003]),
        quat_actual=np.array([1.0, 0.0, 0.0, 0.0]),
        quat_desired=np.array([1.0, 0.0, 0.0, 0.0]),
        tau_received=tau_received,
        tau_applied=tau_applied,
        cycle_time=1.25,
        q=np.zeros(rerun_viz.Config.NUM_JOINTS),
        qd=np.zeros(rerun_viz.Config.NUM_JOINTS),
        step_count=0,
    )

    assert _logged_scalar(dummy_rr, "sim/control/torque/J1/received") == 10.0
    assert _logged_scalar(dummy_rr, "sim/control/torque/J3/received") == 30.0
    assert _logged_scalar(dummy_rr, "sim/control/torque/J3/applied") == 27.0
    assert _logged_scalar(dummy_rr, "sim/performance/step_time_ms") == 1.25
