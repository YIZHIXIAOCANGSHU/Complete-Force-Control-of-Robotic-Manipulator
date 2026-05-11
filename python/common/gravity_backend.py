"""Thin wrapper around the C gravity-compensation backend."""

from __future__ import annotations

import os
import struct
import subprocess
import sys
from dataclasses import dataclass


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
        return self.tau_total, self.ee_pos, self.ee_quat, self.status, self.calc_time_ms


class GravityCompTool:
    """封装 C 语言重力补偿工具的低开销二进制调用。"""

    def __init__(self) -> None:
        cmd = ["./c_interface/serial_gravity_comp"]
        if not os.path.exists(cmd[0]):
            raise FileNotFoundError(f"未找到 C 语言工具: {cmd[0]}，请先运行 make")

        self.process = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=sys.stderr,
            bufsize=0,
        )
        self.bin_struct_in = struct.Struct("<HB7d7d3d4d")
        self.bin_struct_out = struct.Struct("<Hi7d7d7d7d7d7d3d4ddid")
        self.in_size = self.bin_struct_in.size
        self.out_size = self.bin_struct_out.size
        self._in_buffer = bytearray(self.in_size)
        self._out_buffer = bytearray(self.out_size)
        self._zero_qd = (0.0,) * 7
        self._zero_pos = (0.0,) * 3
        self._unit_quat = (1.0, 0.0, 0.0, 0.0)

    def _read_exact(self) -> bool:
        if self.process.stdout is None:
            return False
        view = memoryview(self._out_buffer)
        received = 0
        while received < self.out_size:
            chunk_size = self.process.stdout.readinto(view[received:])
            if not chunk_size:
                return False
            received += chunk_size
        return True

    def _exchange(self, mode, q, qd, target_pos, target_quat):
        if self.process.poll() is not None or self.process.stdin is None:
            return None

        self.bin_struct_in.pack_into(
            self._in_buffer,
            0,
            0xAA55,
            mode,
            *q,
            *qd,
            *target_pos,
            *target_quat,
        )

        try:
            # Popen 使用了无缓冲管道，直接写入即可，避免在 1kHz 回路中重复 flush。
            self.process.stdin.write(self._in_buffer)
        except BrokenPipeError:
            return None

        if not self._read_exact():
            return None
        return self.bin_struct_out.unpack_from(self._out_buffer)

    def compute_fk(self, q):
        parsed = self._exchange(2, q, self._zero_qd, self._zero_pos, self._unit_quat)
        if parsed is None:
            return [0.0] * 3, [1.0, 0.0, 0.0, 0.0]
        return list(parsed[44:47]), list(parsed[47:51])

    def compute(self, q, qd, target_pos, target_quat):
        parsed = self._exchange(1, q, qd, target_pos, target_quat)
        if parsed is None:
            return MitControlOutput(
                tau_total=[0.0] * 7,
                q_ref=[0.0] * 7,
                qd_ref=[0.0] * 7,
                kp=[0.0] * 7,
                kd=[0.0] * 7,
                tau_ff=[0.0] * 7,
                ee_pos=[0.0] * 3,
                ee_quat=[1.0, 0.0, 0.0, 0.0],
                status=-1,
                path_progress=0.0,
                step_count=0,
                calc_time_ms=0.0,
            )
        return MitControlOutput(
            tau_total=list(parsed[2:9]),
            q_ref=list(parsed[9:16]),
            qd_ref=list(parsed[16:23]),
            kp=list(parsed[23:30]),
            kd=list(parsed[30:37]),
            tau_ff=list(parsed[37:44]),
            ee_pos=list(parsed[44:47]),
            ee_quat=list(parsed[47:51]),
            status=int(parsed[1]),
            path_progress=float(parsed[51]),
            step_count=int(parsed[52]),
            calc_time_ms=float(parsed[53]),
        )

    def close(self) -> None:
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
