from __future__ import annotations

import time
from dataclasses import dataclass


try:
    import rerun as rr
    import rerun.blueprint as rrb
except ImportError:  # pragma: no cover
    rr = None
    rrb = None


STATE_CODE_LABELS = {
    0x0: "disabled",
    0x1: "enabled",
    0x8: "overvoltage",
    0x9: "undervoltage",
    0xA: "overcurrent",
    0xB: "mos_overtemp",
    0xC: "rotor_overtemp",
    0xD: "comm_lost",
    0xE: "overload",
}


@dataclass(frozen=True)
class MotorQuality:
    state_ok: bool
    position_ok: bool
    velocity_ok: bool
    feedback_recent: bool
    safety_ok: bool


class UsbfdcanSimRerunRecorder:
    def __init__(self, *, motor_ids: tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7), spawn: bool = True) -> None:
        self._motor_ids = tuple(int(motor_id) for motor_id in motor_ids)
        self._start_time = time.perf_counter()
        self._sequence = 0
        self._enabled = rr is not None
        if not self._enabled:
            return
        rr.init("AM-D02 USB2FDCAN Feedback Mirror", spawn=spawn)
        self._setup_styles()
        self._setup_blueprint()

    def _setup_styles(self) -> None:
        if not self._enabled:
            return
        colors = [
            [230, 50, 50],
            [230, 140, 30],
            [210, 200, 30],
            [50, 200, 50],
            [50, 200, 200],
            [50, 80, 230],
            [150, 50, 230],
        ]
        for index, motor_id in enumerate(self._motor_ids):
            base = self._motor_path(motor_id)
            color = colors[index % len(colors)]
            rr.log(f"{base}/position", rr.SeriesLines(colors=[color], names=[f"J{motor_id} position"], widths=[2]), static=True)
            rr.log(f"{base}/velocity", rr.SeriesLines(colors=[color], names=[f"J{motor_id} velocity"], widths=[2]), static=True)
            rr.log(f"{base}/torque", rr.SeriesLines(colors=[color], names=[f"J{motor_id} feedback torque"], widths=[2]), static=True)
            rr.log(f"{base}/state_code", rr.SeriesLines(colors=[[255, 215, 0]], names=[f"J{motor_id} state"], widths=[2]), static=True)
            rr.log(f"{base}/mos_temperature", rr.SeriesLines(colors=[[255, 140, 0]], names=[f"J{motor_id} mos temp"], widths=[2]), static=True)
            rr.log(f"{base}/rotor_temperature", rr.SeriesLines(colors=[[220, 20, 60]], names=[f"J{motor_id} rotor temp"], widths=[2]), static=True)
            rr.log(f"{base}/quality/state_ok", rr.SeriesLines(names=[f"J{motor_id} state_ok"], widths=[2]), static=True)
            rr.log(f"{base}/quality/position_ok", rr.SeriesLines(names=[f"J{motor_id} position_ok"], widths=[2]), static=True)
            rr.log(f"{base}/quality/velocity_ok", rr.SeriesLines(names=[f"J{motor_id} velocity_ok"], widths=[2]), static=True)
            rr.log(f"{base}/quality/feedback_recent", rr.SeriesLines(names=[f"J{motor_id} feedback_recent"], widths=[2]), static=True)
            rr.log(f"{base}/quality/safety_ok", rr.SeriesLines(names=[f"J{motor_id} safety_ok"], widths=[2]), static=True)
        for name in ("tx_send_rate_hz", "rx_feedback_rate_hz", "complete_round_rate_hz", "missing_feedback_mask", "backpressure_count"):
            rr.log(f"usbfdcan_sim/performance/{name}", rr.SeriesLines(names=[name], widths=[2]), static=True)

    def _setup_blueprint(self) -> None:
        if not self._enabled or rrb is None:
            return
        performance_base = "/usbfdcan_sim/performance"
        motor_pages = [self._motor_blueprint_page(motor_id) for motor_id in self._motor_ids]
        blueprint = rrb.Blueprint(
            rrb.Tabs(
                rrb.Vertical(
                    rrb.Horizontal(
                        rrb.TimeSeriesView(
                            name="CAN Rates",
                            origin=f"{performance_base}/rates",
                            contents=[
                                f"{performance_base}/tx_send_rate_hz",
                                f"{performance_base}/rx_feedback_rate_hz",
                                f"{performance_base}/complete_round_rate_hz",
                            ],
                        ),
                        name="CAN Rates",
                    ),
                    rrb.Horizontal(
                        rrb.TimeSeriesView(
                            name="Safety / Backpressure",
                            origin=f"{performance_base}/safety",
                            contents=[
                                f"{performance_base}/missing_feedback_mask",
                                f"{performance_base}/backpressure_count",
                            ],
                        ),
                        rrb.TextDocumentView(
                            name="Raw Zero MIT Packet",
                            origin="/usbfdcan_sim/raw_zero_packet",
                        ),
                        name="Safety / Backpressure",
                    ),
                    name="Performance",
                ),
                *motor_pages,
            ),
            collapse_panels=True,
        )
        rr.send_blueprint(blueprint)

    @staticmethod
    def _motor_blueprint_page(motor_id: int):
        base = f"/{UsbfdcanSimRerunRecorder._motor_path(motor_id)}"
        return rrb.Vertical(
            rrb.Horizontal(
                rrb.TimeSeriesView(name=f"J{motor_id} Position (rad)", origin=f"{base}/position"),
                rrb.TimeSeriesView(name=f"J{motor_id} Velocity (rad/s)", origin=f"{base}/velocity"),
                rrb.TimeSeriesView(name=f"J{motor_id} Feedback Torque (N*m)", origin=f"{base}/torque"),
            ),
            rrb.Horizontal(
                rrb.TimeSeriesView(name=f"J{motor_id} MOS Temperature (C)", origin=f"{base}/mos_temperature"),
                rrb.TimeSeriesView(name=f"J{motor_id} Rotor Temperature (C)", origin=f"{base}/rotor_temperature"),
                rrb.TimeSeriesView(name=f"J{motor_id} State Code", origin=f"{base}/state_code"),
            ),
            rrb.Horizontal(
                rrb.TimeSeriesView(name=f"J{motor_id} Quality Flags", origin=f"{base}/quality"),
                rrb.TextLogView(name=f"J{motor_id} Events", origin=f"{base}/events"),
            ),
            name=f"J{motor_id}",
        )

    @staticmethod
    def _motor_path(motor_id: int) -> str:
        return f"usbfdcan_sim/motors/motor_{int(motor_id):02d}"

    def _set_time(self) -> None:
        if not self._enabled:
            return
        elapsed = time.perf_counter() - self._start_time
        rr.set_time("usbfdcan_time", duration=elapsed)
        rr.set_time("usbfdcan_seq", sequence=self._sequence)
        self._sequence += 1

    def log_feedback_frame(self, *, frame, quality: MotorQuality) -> None:
        if not self._enabled:
            return
        self._set_time()
        base = self._motor_path(int(frame.motor_id))
        rr.log(f"{base}/position", rr.Scalars(float(frame.position)))
        rr.log(f"{base}/velocity", rr.Scalars(float(frame.velocity)))
        rr.log(f"{base}/torque", rr.Scalars(float(frame.torque)))
        rr.log(f"{base}/state_code", rr.Scalars(float(frame.state)))
        rr.log(f"{base}/mos_temperature", rr.Scalars(float(frame.mos_temperature)))
        rr.log(f"{base}/rotor_temperature", rr.Scalars(float(frame.rotor_temperature)))
        rr.log(f"{base}/quality/state_ok", rr.Scalars(1.0 if quality.state_ok else 0.0))
        rr.log(f"{base}/quality/position_ok", rr.Scalars(1.0 if quality.position_ok else 0.0))
        rr.log(f"{base}/quality/velocity_ok", rr.Scalars(1.0 if quality.velocity_ok else 0.0))
        rr.log(f"{base}/quality/feedback_recent", rr.Scalars(1.0 if quality.feedback_recent else 0.0))
        rr.log(f"{base}/quality/safety_ok", rr.Scalars(1.0 if quality.safety_ok else 0.0))
        state_label = STATE_CODE_LABELS.get(int(frame.state), f"unknown_{int(frame.state):X}")
        rr.log(
            f"{base}/events",
            rr.TextLog(
                (
                    f"state={state_label} controller_id=0x{int(frame.controller_id):02X} "
                    f"pos={float(frame.position):+.5f} vel={float(frame.velocity):+.5f} "
                    f"tau={float(frame.torque):+.5f} mos={float(frame.mos_temperature):.1f} "
                    f"rotor={float(frame.rotor_temperature):.1f}"
                )
            ),
        )

    def log_performance(
        self,
        *,
        tx_send_rate_hz: float,
        rx_feedback_rate_hz: float,
        complete_round_rate_hz: float,
        missing_feedback_mask: int,
        backpressure_count: int,
        raw_zero_packet: bytes = b"",
    ) -> None:
        if not self._enabled:
            return
        self._set_time()
        base = "usbfdcan_sim/performance"
        rr.log(f"{base}/tx_send_rate_hz", rr.Scalars(float(tx_send_rate_hz)))
        rr.log(f"{base}/rx_feedback_rate_hz", rr.Scalars(float(rx_feedback_rate_hz)))
        rr.log(f"{base}/complete_round_rate_hz", rr.Scalars(float(complete_round_rate_hz)))
        rr.log(f"{base}/missing_feedback_mask", rr.Scalars(float(missing_feedback_mask)))
        rr.log(f"{base}/backpressure_count", rr.Scalars(float(backpressure_count)))
        if raw_zero_packet:
            rr.log("usbfdcan_sim/raw_zero_packet", rr.TextDocument(raw_zero_packet.hex(" ").upper()))

    def log_abort(self, *, reason: str, missing_feedback_mask: int = 0) -> None:
        if not self._enabled:
            return
        self._set_time()
        rr.log(
            "usbfdcan_sim/abort",
            rr.TextDocument(f"reason={reason}\nmissing_feedback_mask=0x{int(missing_feedback_mask):02X}"),
        )

    def close(self) -> None:
        if not self._enabled:
            return
        disconnect = getattr(rr, "disconnect", None)
        if callable(disconnect):
            disconnect()


__all__ = ["MotorQuality", "STATE_CODE_LABELS", "UsbfdcanSimRerunRecorder"]
