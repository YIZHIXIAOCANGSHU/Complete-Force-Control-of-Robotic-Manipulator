"""Serial protocol helpers for the AM-D02 controller."""

from __future__ import annotations

import struct

from config import Config


RECV_FRAME_HEAD = 0xA5
RECV_FRAME_FORMAT = "<BBBfffff"
RECV_FRAME_STRUCT = struct.Struct(RECV_FRAME_FORMAT)
RECV_FRAME_SIZE = RECV_FRAME_STRUCT.size

SEND_FRAME_HEAD = 0xAA55
SEND_FRAME_TAIL = 0x55AA
SEND_FRAME_CMD = 0x01
SEND_BODY_STRUCT = struct.Struct("<HB21d")
SEND_FRAME_STRUCT = struct.Struct("<HB21dHH")
SEND_FRAME_SIZE = SEND_FRAME_STRUCT.size
UART_CYCLE_BYTES = RECV_FRAME_SIZE * Config.NUM_JOINTS + SEND_FRAME_SIZE


def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


class TargetPoseFramePacker:
    """Pack frames to match stm32_code/uart_protocol.h."""

    def __init__(self) -> None:
        self._frame = bytearray(SEND_FRAME_SIZE)
        self._body_size = SEND_BODY_STRUCT.size

    def pack(self, target_pos, target_quat, current_q, current_qd):
        SEND_BODY_STRUCT.pack_into(
            self._frame,
            0,
            SEND_FRAME_HEAD,
            SEND_FRAME_CMD,
            float(target_pos[0]),
            float(target_pos[1]),
            float(target_pos[2]),
            float(target_quat[0]),
            float(target_quat[1]),
            float(target_quat[2]),
            float(target_quat[3]),
            float(current_q[0]),
            float(current_q[1]),
            float(current_q[2]),
            float(current_q[3]),
            float(current_q[4]),
            float(current_q[5]),
            float(current_q[6]),
            float(current_qd[0]),
            float(current_qd[1]),
            float(current_qd[2]),
            float(current_qd[3]),
            float(current_qd[4]),
            float(current_qd[5]),
            float(current_qd[6]),
        )
        crc16 = calculate_crc16(self._frame[:self._body_size])
        struct.pack_into("<HH", self._frame, self._body_size, crc16, SEND_FRAME_TAIL)
        return self._frame


class SerialFrameReader:
    """Incrementally decodes fixed-size feedback frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def read_available(self, ser) -> int:
        bytes_waiting = ser.in_waiting
        if bytes_waiting <= 0:
            return 0
        chunk = ser.read(bytes_waiting)
        if chunk:
            self._buffer.extend(chunk)
        return len(chunk)

    def pop_frame(self):
        while True:
            if len(self._buffer) < RECV_FRAME_SIZE:
                return None

            head_index = self._buffer.find(RECV_FRAME_HEAD)
            if head_index < 0:
                self._buffer.clear()
                return None
            if head_index > 0:
                del self._buffer[:head_index]
                if len(self._buffer) < RECV_FRAME_SIZE:
                    return None

            try:
                parsed = RECV_FRAME_STRUCT.unpack_from(self._buffer)
            except struct.error:
                return None
            del self._buffer[:RECV_FRAME_SIZE]
            return parsed

    def has_complete_frame(self) -> bool:
        return len(self._buffer) >= RECV_FRAME_SIZE
