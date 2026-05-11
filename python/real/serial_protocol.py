"""Serial protocol helpers for the AM-D02 controller."""

from __future__ import annotations

import struct

from config import Config


RECV_FRAME_HEAD = 0xA5
RECV_FRAME_FORMAT = "<BBBfffff"
RECV_FRAME_STRUCT = struct.Struct(RECV_FRAME_FORMAT)
RECV_FRAME_SIZE = RECV_FRAME_STRUCT.size

SEND_FRAME_HEAD = b"\xAA\x55"
SEND_FRAME_TAIL = b"\x55\xAA"
SEND_TARGET_POSE_FLOATS = 7
SEND_PAYLOAD_STRUCT = struct.Struct("<7f")
SEND_FRAME_STRUCT = struct.Struct("<2s7fB2s")
SEND_FRAME_SIZE = SEND_FRAME_STRUCT.size
UART_CYCLE_BYTES = RECV_FRAME_SIZE * Config.NUM_JOINTS + SEND_FRAME_SIZE


def calculate_xor_checksum(data: bytes) -> int:
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum & 0xFF


class TargetPoseFramePacker:
    """Pack frames for dm_motor_uart_rx_frame_t."""

    def __init__(self) -> None:
        self._frame = bytearray(SEND_FRAME_SIZE)
        self._payload_size = SEND_PAYLOAD_STRUCT.size
        self._checksum_offset = 2 + self._payload_size

    def pack(self, target_pos, target_quat, current_q=None, current_qd=None):
        SEND_PAYLOAD_STRUCT.pack_into(
            self._frame,
            2,
            float(target_pos[0]),
            float(target_pos[1]),
            float(target_pos[2]),
            float(target_quat[0]),
            float(target_quat[1]),
            float(target_quat[2]),
            float(target_quat[3]),
        )
        self._frame[0:2] = SEND_FRAME_HEAD
        self._frame[self._checksum_offset] = calculate_xor_checksum(self._frame[:self._checksum_offset])
        self._frame[self._checksum_offset + 1 : self._checksum_offset + 3] = SEND_FRAME_TAIL
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
