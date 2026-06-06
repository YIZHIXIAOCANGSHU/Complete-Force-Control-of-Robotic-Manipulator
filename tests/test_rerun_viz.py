from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest


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


def _has_log(dummy_rr: DummyRR, path: str) -> bool:
    return any(logged_path == path for logged_path, _payload, _static in dummy_rr.logs)


def _text_logs(dummy_rr: DummyRR) -> list[str]:
    return [
        payload["text"]
        for path, payload, _static in dummy_rr.logs
        if path == "control_link_log" and isinstance(payload, dict)
    ]


def _first_level_tab_names(blueprint) -> list[str]:
    root = blueprint["root"]
    assert root["kind"] == "Tabs"
    return [child.get("name") for child in root["children"]]


def _first_level_tab(blueprint, name: str):
    root = blueprint["root"]
    assert root["kind"] == "Tabs"
    for child in root["children"]:
        if child.get("name") == name:
            return child
    raise AssertionError(f"Missing first-level tab {name}")


def _child_by_name(node, name: str):
    for child in node.get("children", []):
        if child.get("name") == name:
            return child
    raise AssertionError(f"Missing child {name}")


def _child_names(node) -> list[str]:
    return [child.get("name") for child in node.get("children", [])]


def _assert_tabs_children(node, expected_names: list[str]) -> None:
    assert node["kind"] == "Tabs"
    assert _child_names(node) == expected_names


def _time_series_origins(node) -> set[str]:
    return {
        child["origin"]
        for child in _iter_nodes(node)
        if child.get("kind") == "TimeSeriesView"
    }


def test_sim_udp_server_passes_step_count_to_rerun_logger():
    source = (Path(__file__).resolve().parents[1] / "python" / "sim" / "udp_server.py").read_text(
        encoding="utf-8"
    )
    payload_start = source.index("rerun_payload = {")
    payload_end = source.index("env.write_state_packet(state_packet)", payload_start)
    payload_block = source[payload_start:payload_end]

    assert "SimRerunLogger()" in source
    assert "rerun_logger.log_step(" in source
    assert "**rerun_payload" in source
    assert '"step_count": step_count' in payload_block
    assert "include_twist = bool(Config.SIM_RERUN_INCLUDE_TWIST)" in source
    assert "env.get_state_snapshot(include_twist=include_twist)" in source
    assert 'rerun_payload["ee_twist"] = twist_values[0]' in source
    assert '"ee_twist":' not in payload_block


def test_setup_realtime_styles_blueprint_does_not_reuse_tile_objects(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)
    monkeypatch.setattr(rerun_viz, "rrb", DummyRRB)

    rerun_viz.setup_realtime_styles()

    tile_kinds = {"TimeSeriesView", "TextLogView", "Spatial3DView", "Vertical", "Horizontal", "Tabs"}
    seen: dict[int, tuple[str, str | None]] = {}
    duplicates = []
    for node in _iter_nodes(dummy_rr.blueprint):
        if node.get("kind") not in tile_kinds:
            continue
        node_id = id(node)
        identity = (node["kind"], node.get("name"))
        if node_id in seen:
            duplicates.append((seen[node_id], identity))
        seen[node_id] = identity

    assert duplicates == []


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
    assert names_by_origin["/arms/left/tcp_speed/linear"] == "Left TCP Linear Speed (m/s)"
    assert names_by_origin["/arms/right/tcp_speed/linear"] == "Right TCP Linear Speed (m/s)"
    assert names_by_origin["/arms/left/tcp_velocity/X"] == "Left TCP Velocity X (m/s)"
    assert names_by_origin["/arms/right/tcp_velocity/Z"] == "Right TCP Velocity Z (m/s)"
    assert names_by_origin["/limits/tcp_speed"] == "TCP Speed Limit (m/s)"
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

    tile_names = {
        node["name"]
        for node in _iter_nodes(dummy_rr.blueprint)
        if node.get("kind")
        in ("Tabs", "Vertical", "Horizontal", "Spatial3DView", "TimeSeriesView", "TextLogView")
        and node.get("name")
    }
    expected_tiles = {
        "Status",
        "3D",
        "Details",
        "Tracking Detail",
        "Joint Detail",
        "Torque Detail",
        "Safety",
        "Fast Status",
        "Position",
        "Rotation",
        "Position Error",
        "Rotation Error",
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
        "TCP Speed",
        "Left Torque",
        "Right Torque",
        "Left Torque Gap",
        "Right Torque Gap",
        "Command",
        "Raw",
        "Actual",
        "Gap",
        "Utilization",
        "Summary",
        "Warnings",
        "Performance",
        "Overview",
        "Link",
    }
    assert expected_tiles <= tile_names
    assert "Left Arm" not in tile_names
    assert "Right Arm" not in tile_names


def test_setup_realtime_styles_adds_status_dashboard_without_removing_details(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)
    monkeypatch.setattr(rerun_viz, "rrb", DummyRRB)

    rerun_viz.setup_realtime_styles()

    assert _first_level_tab_names(dummy_rr.blueprint) == [
        "Status",
        "3D",
        "Details",
        "Performance",
        "Link",
    ]
    status_tab = _first_level_tab(dummy_rr.blueprint, "Status")
    assert not any(node.get("kind") == "Spatial3DView" for node in _iter_nodes(status_tab))
    _assert_tabs_children(
        status_tab,
        [
            "Tracking",
            "Torque",
            "Safety",
            "Link",
        ],
    )

    tracking_status = _child_by_name(status_tab, "Tracking")
    assert _time_series_origins(tracking_status) == {
        "/dashboard/position_error_norm_mm",
        "/dashboard/reference_error_norm_mm",
        "/dashboard/rotation_error_norm_deg",
        "/dashboard/tcp_speed_mps",
    }
    assert not any(node.get("kind") == "Spatial3DView" for node in _iter_nodes(tracking_status))
    assert _time_series_origins(_child_by_name(status_tab, "Torque")) == {
        "/dashboard/torque_utilization",
    }
    assert _time_series_origins(_child_by_name(status_tab, "Safety")) == {
        "/dashboard/min_joint_limit_margin_rad",
        "/dashboard/min_velocity_margin_rad_s",
    }
    link_origins = _time_series_origins(_child_by_name(status_tab, "Link"))
    assert link_origins == {
        "/performance/c_engine_time",
        "/performance/link_latency",
        "/performance/link_cycle_hz",
        "/performance/tx_overwrite_count",
        "/performance/can_backpressure_count",
    }
    performance_tab = _first_level_tab(dummy_rr.blueprint, "Performance")
    _assert_tabs_children(performance_tab, ["Overview"])

    details_tab = _first_level_tab(dummy_rr.blueprint, "Details")
    _assert_tabs_children(
        details_tab,
        ["Tracking Detail", "Joint Detail", "Torque Detail", "Safety"],
    )
    _assert_tabs_children(
        _child_by_name(details_tab, "Tracking Detail"),
        ["Fast Status", "Position", "Rotation", "Position Error", "Rotation Error", "TCP Speed"],
    )
    _assert_tabs_children(_child_by_name(details_tab, "Joint Detail"), ["Joint Position", "Joint Velocity"])
    _assert_tabs_children(
        _child_by_name(details_tab, "Torque Detail"),
        ["Command", "Raw", "Actual", "Gap", "Utilization"],
    )
    _assert_tabs_children(
        _child_by_name(details_tab, "Safety"),
        ["Velocity Margin", "Low Limit Margin", "High Limit Margin", "Summary", "Warnings"],
    )

    names_by_origin = {
        node["origin"]: node["name"]
        for node in _iter_nodes(dummy_rr.blueprint)
        if node.get("kind") == "TimeSeriesView"
    }
    assert names_by_origin["/dashboard/position_error_norm_mm"] == "Position Error Norm (mm)"
    assert names_by_origin["/dashboard/reference_error_norm_mm"] == "Reference Error Norm (mm)"
    assert names_by_origin["/dashboard/rotation_error_norm_deg"] == "Rotation Error Norm (deg)"
    assert names_by_origin["/dashboard/tcp_speed_mps"] == "TCP Speed (m/s)"
    assert names_by_origin["/dashboard/torque_utilization"] == "Command Torque Utilization"
    assert names_by_origin["/dashboard/min_joint_limit_margin_rad"] == "Min Joint Limit Margin (rad)"
    assert names_by_origin["/dashboard/min_velocity_margin_rad_s"] == "Min Velocity Margin (rad/s)"
    assert names_by_origin["/performance/link_latency"] == "Control Link Period (ms)"
    assert names_by_origin["/performance/link_cycle_hz"] == "Control Link Rate (Hz)"
    assert names_by_origin["/performance/tx_overwrite_count"] == "TX Pending Overwrites"
    assert names_by_origin["/performance/can_backpressure_count"] == "CAN Backpressure Count"

    assert "/arms/left/fast_status/position_error_norm_mm" in names_by_origin
    assert "/arms/right/fast_status/reference_error_norm_mm" in names_by_origin
    assert "/arms/left/position/X" in names_by_origin
    assert "/arms/right/rotation/Yaw" in names_by_origin
    assert "/arms/left/joint_q/J1" in names_by_origin
    assert "/arms/right/torque_gap/J7" in names_by_origin
    assert "/arms/left/velocity_margin/J1" in names_by_origin


def test_setup_realtime_styles_uses_balanced_performance_views_by_default(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)
    monkeypatch.setattr(rerun_viz, "rrb", DummyRRB)
    monkeypatch.setattr(rerun_viz.Config, "RERUN_DETAILED_PERF", False)

    rerun_viz.setup_realtime_styles()

    performance_tab = _first_level_tab(dummy_rr.blueprint, "Performance")
    _assert_tabs_children(performance_tab, ["Overview"])

    origins = {
        node["origin"]
        for node in _iter_nodes(dummy_rr.blueprint)
        if node.get("kind") == "TimeSeriesView"
    }
    assert "/performance/sim_service_ms" in origins
    assert "/performance/viewer_sync_ms" in origins
    assert "/performance/tx_overwrite_count" in origins
    assert "/performance/can_backpressure_count" in origins
    assert "/performance/sim_socket_timeout_count" not in origins
    assert "/performance/viewer_skip_count" not in origins
    assert not _has_log(dummy_rr, "performance/sim_socket_timeout_count")
    assert not _has_log(dummy_rr, "performance/viewer_skip_count")


def test_setup_realtime_styles_full_profile_includes_detailed_performance_views(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)
    monkeypatch.setattr(rerun_viz, "rrb", DummyRRB)
    monkeypatch.setattr(rerun_viz.Config, "RERUN_DETAILED_PERF", True)

    rerun_viz.setup_realtime_styles()

    performance_tab = _first_level_tab(dummy_rr.blueprint, "Performance")
    _assert_tabs_children(performance_tab, ["Overview", "Detailed"])

    origins = {
        node["origin"]
        for node in _iter_nodes(dummy_rr.blueprint)
        if node.get("kind") == "TimeSeriesView"
    }
    assert "/performance/sim_socket_timeout_count" in origins
    assert "/performance/viewer_skip_count" in origins
    assert _has_log(dummy_rr, "performance/sim_socket_timeout_count")
    assert _has_log(dummy_rr, "performance/viewer_skip_count")


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
    tau_raw = tau_total + 1.0
    rerun_viz.log_realtime_step(
        t=0.1,
        pos_actual=np.zeros((2, 3), dtype=np.float64),
        pos_desired=np.zeros((2, 3), dtype=np.float64),
        quat_actual=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        quat_desired=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        tau_raw=tau_raw,
        tau_total=tau_total,
        tau_actual=tau_actual,
        cycle_time=1.25,
        elapsed_s=0.001,
        right_j7_diag={
            "q": 0.7,
            "qd": -3.2,
            "tau_cmd_raw": 5.0,
            "tau_cmd_sent": 4.0,
            "tau_actual": 3.0,
        },
        step_count=0,
    )

    assert _logged_scalar(dummy_rr, "arms/left/torque_gap/J1/value") == 0.5
    assert _logged_scalar(dummy_rr, "arms/right/torque_gap/J1/value") == 0.5
    assert _logged_scalar(dummy_rr, "arms/left/torque_actual/J1/value") == -0.5
    assert _logged_scalar(dummy_rr, "arms/right/torque_actual/J1/value") == 6.5
    assert _logged_scalar(dummy_rr, "arms/left/torque_raw/J1/value") == 1.0
    assert _logged_scalar(dummy_rr, "arms/right/torque_raw/J7/value") == 14.0
    assert _logged_scalar(dummy_rr, "diagnostics/right_j7/qd") == -3.2
    assert _logged_scalar(dummy_rr, "diagnostics/right_j7/tau_cmd_raw") == 5.0
    assert _logged_scalar(dummy_rr, "diagnostics/right_j7/tau_cmd_sent") == 4.0
    assert _logged_scalar(dummy_rr, "diagnostics/right_j7/tau_actual") == 3.0
    assert _logged_scalar(dummy_rr, "performance/elapsed_s") == 0.001


def test_log_realtime_step_logs_tcp_speed_and_velocity(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)

    rerun_viz.log_realtime_step(
        t=0.1,
        pos_actual=np.zeros((2, 3), dtype=np.float64),
        pos_desired=np.zeros((2, 3), dtype=np.float64),
        quat_actual=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        quat_desired=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        tau_total=np.zeros(rerun_viz.Config.NUM_JOINTS),
        ee_twist=np.array(
            [[0.003, 0.004, 0.0, 0.1, 0.2, 0.3], [0.0, 0.0, -0.002, 0.0, 0.0, 0.0]],
            dtype=np.float64,
        ),
        cycle_time=1.25,
        step_count=0,
    )

    assert _logged_scalar(dummy_rr, "arms/left/tcp_velocity/X/value") == pytest.approx(0.003)
    assert _logged_scalar(dummy_rr, "arms/left/tcp_velocity/Y/value") == pytest.approx(0.004)
    assert _logged_scalar(dummy_rr, "arms/left/tcp_speed/linear/value") == pytest.approx(0.005)
    assert _logged_scalar(dummy_rr, "arms/right/tcp_velocity/Z/value") == pytest.approx(-0.002)
    assert _logged_scalar(dummy_rr, "arms/right/tcp_speed/linear/value") == pytest.approx(0.002)
    assert _logged_scalar(dummy_rr, "limits/tcp_speed/value") == pytest.approx(
        rerun_viz.Config.END_EFFECTOR_REAL_SPEED_LIMIT_MPS
    )


def test_log_realtime_step_logs_reference_pose(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)

    rerun_viz.log_realtime_step(
        t=0.1,
        pos_actual=np.array([[0.10, 0.20, 0.30], [0.40, 0.50, 0.60]], dtype=np.float64),
        pos_desired=np.zeros((2, 3), dtype=np.float64),
        pos_reference=np.array([[0.11, 0.22, 0.33], [0.0, 0.0, 0.0]], dtype=np.float64),
        quat_actual=np.tile([1.0, 0.0, 0.0, 0.0], (2, 1)),
        quat_desired=np.tile([1.0, 0.0, 0.0, 0.0], (2, 1)),
        quat_reference=np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]], dtype=np.float64),
        tau_total=np.zeros(rerun_viz.Config.NUM_JOINTS),
        cycle_time=1.25,
        step_count=0,
    )

    assert _logged_scalar(dummy_rr, "arms/left/position/X/reference") == pytest.approx(110.0)
    assert _logged_scalar(dummy_rr, "arms/left/position/Y/reference") == pytest.approx(220.0)
    assert _logged_scalar(dummy_rr, "arms/left/rotation/Roll/reference") == pytest.approx(0.0)
    assert not _has_log(dummy_rr, "arms/right/position/X/reference")
    assert _has_log(dummy_rr, "trajectory_3d/reference_point")
    assert _has_log(dummy_rr, "trajectory_3d/reference_error_line")


def test_log_realtime_step_logs_fast_status_aggregates(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)

    tau_total = np.zeros(rerun_viz.Config.NUM_JOINTS, dtype=np.float64)
    tau_actual = np.zeros(rerun_viz.Config.NUM_JOINTS, dtype=np.float64)
    tau_raw = np.zeros(rerun_viz.Config.NUM_JOINTS, dtype=np.float64)
    tau_total[0] = 20.0
    tau_actual[0] = -10.0
    tau_raw[0] = 40.0
    tau_total[rerun_viz.Config.ARM_JOINTS] = 20.0

    safe_min, safe_max = rerun_viz._joint_safe_limits_rad()
    q = (safe_min + safe_max) * 0.5
    q[0] = safe_min[0] + 0.03
    q[rerun_viz.Config.ARM_JOINTS] = safe_max[rerun_viz.Config.ARM_JOINTS] - 0.04
    qd = np.zeros(rerun_viz.Config.NUM_JOINTS, dtype=np.float64)
    qd[1] = rerun_viz.Config.JOINT_VEL_LIMIT - 0.20
    qd[rerun_viz.Config.ARM_JOINTS + 1] = -(rerun_viz.Config.JOINT_VEL_LIMIT - 0.30)

    rerun_viz.log_realtime_step(
        t=0.1,
        pos_actual=np.array([[0.10, 0.20, 0.30], [0.40, 0.50, 0.60]], dtype=np.float64),
        pos_desired=np.array([[0.07, 0.16, 0.30], [0.40, 0.45, 0.60]], dtype=np.float64),
        pos_reference=np.array([[0.10, 0.22, 0.30], [0.0, 0.0, 0.0]], dtype=np.float64),
        quat_actual=np.array(
            [[0.9238795325, 0.3826834324, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            dtype=np.float64,
        ),
        quat_desired=np.tile([1.0, 0.0, 0.0, 0.0], (2, 1)),
        quat_reference=np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]], dtype=np.float64),
        tau_total=tau_total,
        tau_actual=tau_actual,
        tau_raw=tau_raw,
        q=q,
        qd=qd,
        ee_twist=np.array(
            [[0.003, 0.004, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, -0.002, 0.0, 0.0, 0.0]],
            dtype=np.float64,
        ),
        cycle_time=1.25,
        step_count=0,
    )

    assert _logged_scalar(dummy_rr, "arms/left/fast_status/position_error_norm_mm/value") == pytest.approx(50.0)
    assert _logged_scalar(dummy_rr, "arms/right/fast_status/position_error_norm_mm/value") == pytest.approx(50.0)
    assert _logged_scalar(dummy_rr, "dashboard/position_error_norm_mm/left") == pytest.approx(50.0)
    assert _logged_scalar(dummy_rr, "dashboard/position_error_norm_mm/right") == pytest.approx(50.0)
    assert _logged_scalar(dummy_rr, "arms/left/fast_status/reference_error_norm_mm/value") == pytest.approx(20.0)
    assert _logged_scalar(dummy_rr, "dashboard/reference_error_norm_mm/left") == pytest.approx(20.0)
    assert not _has_log(dummy_rr, "arms/right/fast_status/reference_error_norm_mm/value")
    assert not _has_log(dummy_rr, "dashboard/reference_error_norm_mm/right")
    assert _logged_scalar(dummy_rr, "arms/left/fast_status/rotation_error_norm_deg/value") == pytest.approx(45.0)
    assert _logged_scalar(dummy_rr, "dashboard/rotation_error_norm_deg/left") == pytest.approx(45.0)
    assert _logged_scalar(dummy_rr, "arms/left/fast_status/tcp_speed_mps/value") == pytest.approx(0.005)
    assert _logged_scalar(dummy_rr, "arms/right/fast_status/tcp_speed_mps/value") == pytest.approx(0.002)
    assert _logged_scalar(dummy_rr, "dashboard/tcp_speed_mps/left") == pytest.approx(0.005)
    assert _logged_scalar(dummy_rr, "dashboard/tcp_speed_mps/right") == pytest.approx(0.002)
    assert _logged_scalar(dummy_rr, "dashboard/tcp_speed_mps/limit") == pytest.approx(
        rerun_viz.Config.END_EFFECTOR_REAL_SPEED_LIMIT_MPS
    )
    assert _logged_scalar(dummy_rr, "arms/left/fast_status/torque_utilization/command") == pytest.approx(0.5)
    assert _logged_scalar(dummy_rr, "arms/left/fast_status/torque_utilization/actual") == pytest.approx(0.25)
    assert _logged_scalar(dummy_rr, "arms/left/fast_status/torque_utilization/raw") == pytest.approx(1.0)
    assert _logged_scalar(dummy_rr, "arms/right/fast_status/torque_utilization/command") == pytest.approx(0.5)
    assert _logged_scalar(dummy_rr, "dashboard/torque_utilization/left") == pytest.approx(0.5)
    assert _logged_scalar(dummy_rr, "dashboard/torque_utilization/right") == pytest.approx(0.5)
    assert _logged_scalar(dummy_rr, "arms/left/fast_status/min_joint_limit_margin_rad/value") == pytest.approx(0.03)
    assert _logged_scalar(dummy_rr, "arms/right/fast_status/min_joint_limit_margin_rad/value") == pytest.approx(0.04)
    assert _logged_scalar(dummy_rr, "dashboard/min_joint_limit_margin_rad/left") == pytest.approx(0.03)
    assert _logged_scalar(dummy_rr, "dashboard/min_joint_limit_margin_rad/right") == pytest.approx(0.04)
    assert _logged_scalar(dummy_rr, "dashboard/min_joint_limit_margin_rad/warning_threshold") == pytest.approx(0.02)
    assert _logged_scalar(dummy_rr, "arms/left/fast_status/min_velocity_margin_rad_s/value") == pytest.approx(0.20)
    assert _logged_scalar(dummy_rr, "arms/right/fast_status/min_velocity_margin_rad_s/value") == pytest.approx(0.30)
    assert _logged_scalar(dummy_rr, "dashboard/min_velocity_margin_rad_s/left") == pytest.approx(0.20)
    assert _logged_scalar(dummy_rr, "dashboard/min_velocity_margin_rad_s/right") == pytest.approx(0.30)
    assert _logged_scalar(dummy_rr, "dashboard/min_velocity_margin_rad_s/warning_threshold") == pytest.approx(0.2)


def test_log_realtime_step_logs_balanced_sim_performance_metrics(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)
    monkeypatch.setattr(rerun_viz.Config, "RERUN_DETAILED_PERF", False)

    rerun_viz.log_realtime_step(
        t=0.1,
        pos_actual=np.zeros((2, 3), dtype=np.float64),
        pos_desired=np.zeros((2, 3), dtype=np.float64),
        quat_actual=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        quat_desired=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        tau_total=np.zeros(rerun_viz.Config.NUM_JOINTS),
        cycle_time=1.25,
        step_count=0,
        sim_target_hz=1000.0,
        sim_service_ms=0.91,
        sim_mujoco_step_ms=0.62,
        sim_state_packet_ms=0.04,
        sim_rerun_overwrite_count=3,
        sim_rerun_drop_count=1,
        sim_socket_timeout_count=2,
        viewer_sync_count=9,
        viewer_skip_count=4,
        viewer_sync_ms=0.35,
        viewer_lock_wait_ms=0.02,
    )

    assert _logged_scalar(dummy_rr, "performance/sim_target_hz") == pytest.approx(1000.0)
    assert _logged_scalar(dummy_rr, "performance/sim_service_ms") == pytest.approx(0.91)
    assert _logged_scalar(dummy_rr, "performance/sim_mujoco_step_ms") == pytest.approx(0.62)
    assert _logged_scalar(dummy_rr, "performance/sim_state_packet_ms") == pytest.approx(0.04)
    assert _logged_scalar(dummy_rr, "performance/sim_rerun_overwrite_count") == pytest.approx(3.0)
    assert _logged_scalar(dummy_rr, "performance/sim_rerun_drop_count") == pytest.approx(1.0)
    assert _logged_scalar(dummy_rr, "performance/viewer_sync_ms") == pytest.approx(0.35)
    assert not _has_log(dummy_rr, "performance/sim_socket_timeout_count")
    assert not _has_log(dummy_rr, "performance/viewer_sync_count")
    assert not _has_log(dummy_rr, "performance/viewer_skip_count")
    assert not _has_log(dummy_rr, "performance/viewer_lock_wait_ms")


def test_log_realtime_step_logs_full_sim_performance_metrics(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)
    monkeypatch.setattr(rerun_viz.Config, "RERUN_DETAILED_PERF", True)

    rerun_viz.log_realtime_step(
        t=0.1,
        pos_actual=np.zeros((2, 3), dtype=np.float64),
        pos_desired=np.zeros((2, 3), dtype=np.float64),
        quat_actual=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        quat_desired=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        tau_total=np.zeros(rerun_viz.Config.NUM_JOINTS),
        cycle_time=1.25,
        step_count=0,
        sim_socket_timeout_count=2,
        viewer_sync_count=9,
        viewer_skip_count=4,
        viewer_lock_wait_ms=0.02,
    )

    assert _logged_scalar(dummy_rr, "performance/sim_socket_timeout_count") == pytest.approx(2.0)
    assert _logged_scalar(dummy_rr, "performance/viewer_sync_count") == pytest.approx(9.0)
    assert _logged_scalar(dummy_rr, "performance/viewer_skip_count") == pytest.approx(4.0)
    assert _logged_scalar(dummy_rr, "performance/viewer_lock_wait_ms") == pytest.approx(0.02)


def test_log_realtime_step_logs_joint_safety_margins_and_warning(monkeypatch):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)

    q = np.zeros(rerun_viz.Config.NUM_JOINTS, dtype=np.float64)
    qd = np.zeros(rerun_viz.Config.NUM_JOINTS, dtype=np.float64)
    qd[0] = rerun_viz.Config.JOINT_VEL_LIMIT + 0.05
    safe_min, safe_max = rerun_viz._joint_safe_limits_rad()
    q[1] = safe_min[1] - 0.001
    q[rerun_viz.Config.ARM_JOINTS] = safe_max[rerun_viz.Config.ARM_JOINTS] + 0.001

    rerun_viz.log_realtime_step(
        t=0.1,
        pos_actual=np.zeros((2, 3), dtype=np.float64),
        pos_desired=np.zeros((2, 3), dtype=np.float64),
        quat_actual=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        quat_desired=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        tau_total=np.zeros(rerun_viz.Config.NUM_JOINTS),
        q=q,
        qd=qd,
        cycle_time=1.25,
        step_count=0,
    )

    assert _logged_scalar(dummy_rr, "arms/left/velocity_margin/J1/value") == pytest.approx(-0.05)
    assert _logged_scalar(dummy_rr, "arms/left/limit_margin_low/J2/value") == pytest.approx(-0.001)
    assert _logged_scalar(dummy_rr, "arms/right/limit_margin_high/J1/value") == pytest.approx(-0.001)
    text_logs = _text_logs(dummy_rr)
    assert any("SAFETY margin warning" in text for text in text_logs)
    assert any("left/J1 vel_margin" in text for text in text_logs)


def test_log_realtime_step_throttles_repeated_safety_text(monkeypatch, capsys):
    dummy_rr = DummyRR()
    monkeypatch.setattr(rerun_viz, "RERUN_AVAILABLE", True)
    monkeypatch.setattr(rerun_viz, "rr", dummy_rr)
    monkeypatch.setattr(rerun_viz, "_last_safety_warning_signature", None)
    monkeypatch.setattr(rerun_viz, "_last_safety_warning_step", None)

    safe_min, _safe_max = rerun_viz._joint_safe_limits_rad()
    q = np.zeros(rerun_viz.Config.NUM_JOINTS, dtype=np.float64)
    qd = np.zeros(rerun_viz.Config.NUM_JOINTS, dtype=np.float64)
    right_j4 = rerun_viz.Config.ARM_JOINTS + 3

    for step, margin in ((0, 0.001), (10, 0.002), (20, 0.003), (500, 0.004)):
        q[right_j4] = safe_min[right_j4] + margin
        rerun_viz.log_realtime_step(
            t=step * 0.001,
            pos_actual=np.zeros((2, 3), dtype=np.float64),
            pos_desired=np.zeros((2, 3), dtype=np.float64),
            quat_actual=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
            quat_desired=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
            tau_total=np.zeros(rerun_viz.Config.NUM_JOINTS),
            q=q,
            qd=qd,
            cycle_time=1.25,
            step_count=step,
        )

    text_logs = [text for text in _text_logs(dummy_rr) if "SAFETY margin warning" in text]
    assert len(text_logs) == 2
    assert text_logs[0].startswith("[0] SAFETY")
    assert text_logs[1].startswith("[500] SAFETY")

    printed = capsys.readouterr().out
    assert printed.count("[Rerun Safety]") == 2
