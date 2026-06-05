"""Binary bridge to the real-mode C controller."""

from __future__ import annotations

import os
import select
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTROLLER = PROJECT_ROOT / "c_interface" / "real" / "real_controller"
MAGIC = 0xAA55
MODE_CONTROL = 1
MODE_FK_ONLY = 2
ARM_JOINTS = 7
NUM_ARMS = 2
NUM_JOINTS = 14
NUM_BODY_JOINTS = 3


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


DEFAULT_EXCHANGE_TIMEOUT_S = max(0.0, _env_float("AM_D02_REAL_C_BRIDGE_TIMEOUT_S", 0.005))


@dataclass(frozen=True)
class RealControllerOutput:
    status: int
    tau: np.ndarray
    tau_gravity: np.ndarray
    ee_pos: np.ndarray
    ee_quat: np.ndarray
    ee_twist: np.ndarray
    traj_t: float
    step_count: int


class RealControllerBridge:
    """Low-overhead stdin/stdout bridge for the C real controller."""

    _input_struct = struct.Struct("<HBBd14d14d3d6d8d")
    _output_struct = struct.Struct("<Hi14d14d6d8d12ddi")

    def __init__(
        self,
        executable: str | os.PathLike[str] = DEFAULT_CONTROLLER,
        *,
        exchange_timeout_s: float | None = None,
    ) -> None:
        self.executable = Path(executable)
        if not self.executable.exists():
            raise FileNotFoundError(f"real controller not found: {self.executable}")
        self.exchange_timeout_s = (
            DEFAULT_EXCHANGE_TIMEOUT_S
            if exchange_timeout_s is None
            else max(0.0, float(exchange_timeout_s))
        )
        self.process = subprocess.Popen(
            [str(self.executable)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=sys.stderr,
            bufsize=0,
        )
        self._in_buffer = bytearray(self._input_struct.size)
        self._out_buffer = bytearray(self._output_struct.size)

    def _read_exact(self, timeout_s: float | None = None) -> bool:
        if self.process.stdout is None:
            return False
        view = memoryview(self._out_buffer)
        received = 0
        timeout = self.exchange_timeout_s if timeout_s is None else max(0.0, float(timeout_s))
        deadline = time.monotonic() + timeout if timeout > 0.0 else None
        while received < len(self._out_buffer):
            if deadline is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(f"real controller output timeout after {timeout:.6f}s")
                ready, _, _ = select.select([self.process.stdout], [], [], remaining)
                if not ready:
                    raise TimeoutError(f"real controller output timeout after {timeout:.6f}s")
            chunk = self.process.stdout.readinto(view[received:])
            if not chunk:
                return False
            received += chunk
        return True

    def exchange(
        self,
        *,
        mode: int,
        active_arm_mask: int,
        elapsed_s: float,
        q: np.ndarray,
        qd: np.ndarray,
        body_q: np.ndarray,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
    ) -> RealControllerOutput:
        if self.process.poll() is not None or self.process.stdin is None:
            raise RuntimeError("real controller process is not running")

        q_arr = np.asarray(q, dtype=np.float64).reshape(NUM_JOINTS)
        qd_arr = np.asarray(qd, dtype=np.float64).reshape(NUM_JOINTS)
        body_q_arr = np.asarray(body_q, dtype=np.float64).reshape(NUM_BODY_JOINTS)
        target_pos_arr = np.asarray(target_pos, dtype=np.float64).reshape(NUM_ARMS, 3)
        target_quat_arr = np.asarray(target_quat, dtype=np.float64).reshape(NUM_ARMS, 4)

        self._input_struct.pack_into(
            self._in_buffer,
            0,
            MAGIC,
            int(mode),
            int(active_arm_mask),
            float(elapsed_s),
            *q_arr,
            *qd_arr,
            *body_q_arr,
            *target_pos_arr.reshape(-1),
            *target_quat_arr.reshape(-1),
        )
        self.process.stdin.write(self._in_buffer)
        if not self._read_exact():
            raise RuntimeError("failed to read real controller output")

        parsed = self._output_struct.unpack_from(self._out_buffer)
        if int(parsed[0]) != MAGIC:
            raise RuntimeError(f"invalid real controller magic: 0x{int(parsed[0]):04X}")
        return RealControllerOutput(
            status=int(parsed[1]),
            tau=np.asarray(parsed[2:16], dtype=np.float64),
            tau_gravity=np.asarray(parsed[16:30], dtype=np.float64),
            ee_pos=np.asarray(parsed[30:36], dtype=np.float64).reshape(NUM_ARMS, 3),
            ee_quat=np.asarray(parsed[36:44], dtype=np.float64).reshape(NUM_ARMS, 4),
            ee_twist=np.asarray(parsed[44:56], dtype=np.float64).reshape(NUM_ARMS, 6),
            traj_t=float(parsed[56]),
            step_count=int(parsed[57]),
        )

    def compute(
        self,
        active_arm_mask: int,
        elapsed_s: float,
        q: np.ndarray,
        qd: np.ndarray,
        body_q: np.ndarray,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
    ) -> RealControllerOutput:
        return self.exchange(
            mode=MODE_CONTROL,
            active_arm_mask=active_arm_mask,
            elapsed_s=elapsed_s,
            q=q,
            qd=qd,
            body_q=body_q,
            target_pos=target_pos,
            target_quat=target_quat,
        )

    def compute_fk(
        self,
        active_arm_mask: int,
        q: np.ndarray,
        body_q: np.ndarray | None = None,
    ) -> RealControllerOutput:
        return self.exchange(
            mode=MODE_FK_ONLY,
            active_arm_mask=active_arm_mask,
            elapsed_s=0.0,
            q=q,
            qd=np.zeros(NUM_JOINTS, dtype=np.float64),
            body_q=np.zeros(NUM_BODY_JOINTS, dtype=np.float64)
            if body_q is None
            else body_q,
            target_pos=np.zeros((NUM_ARMS, 3), dtype=np.float64),
            target_quat=np.tile([1.0, 0.0, 0.0, 0.0], (NUM_ARMS, 1)),
        )

    def close(self) -> None:
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
