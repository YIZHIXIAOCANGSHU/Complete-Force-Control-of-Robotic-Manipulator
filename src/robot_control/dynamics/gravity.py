"""Gravity compensation backend — Pinocchio-based computation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from robot_control.dynamics.pinocchio import PinocchioGravityBackend
from robot_control.config import Config


@dataclass(frozen=True)
class MitControlOutput:
    tau_total: list[float]
    q_ref: list[float]
    qd_ref: list[float]
    kp: list[float]
    kd: list[float]
    tau_ff: list[float]
    ee_pos: list[float]
    ee_quat: list[float]
    status: int
    path_progress: float
    step_count: int
    calc_time_ms: float

    def legacy_tuple(self):
        return (
            self.tau_total,
            self.ee_pos,
            self.ee_quat,
            self.status,
            self.calc_time_ms,
        )


class GravityCompTool:
    """Pinocchio-based gravity compensation and control computation.

    Loads the AM-D02 URDF, runs FK / Jacobian / RNEA dynamics / DLS IK
    entirely through Pinocchio, and exposes the compute_fk / compute
    interface used by the retained real-control and identification callers.
    """

    def __init__(self) -> None:
        self._backend = PinocchioGravityBackend(
            urdf_path=Config.URDF_PATH,
            ee_frame_name="ArmLseventh_Link",
            tcp_offset=Config.TCP_OFFSET,
            torque_limits=Config.TORQUE_LIMITS.tolist(),
        )

    def compute_fk(self, q: List[float]) -> Tuple[List[float], List[float]]:
        return self._backend.compute_fk(q)

    def compute(
        self,
        q: List[float],
        qd: List[float],
        target_pos: List[float],
        target_quat: List[float],
    ) -> MitControlOutput:
        return self._backend.compute(q, qd, target_pos, target_quat)

    def close(self) -> None:
        self._backend.close()
