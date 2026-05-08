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

    assert names_by_origin["/tracking/pos/X"] == "EE Position X (mm)"
    assert names_by_origin["/tracking/pos/Y"] == "EE Position Y (mm)"
    assert names_by_origin["/tracking/pos/Z"] == "EE Position Z (mm)"


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
