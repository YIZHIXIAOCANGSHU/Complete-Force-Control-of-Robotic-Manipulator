"""Thin wrapper around the C gravity-compensation backend."""

from __future__ import annotations

import os
import struct
import subprocess
import sys


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
        self.bin_struct_out = struct.Struct("<Hi7d3d4dd")
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
        return list(parsed[9:12]), list(parsed[12:16])

    def compute(self, q, qd, target_pos, target_quat):
        parsed = self._exchange(1, q, qd, target_pos, target_quat)
        if parsed is None:
            return [0.0] * 7, [0.0] * 3, [1.0, 0.0, 0.0, 0.0], -1, 0.0
        return (
            parsed[2:9],
            parsed[9:12],
            parsed[12:16],
            parsed[1],
            parsed[16],
        )

    def close(self) -> None:
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
