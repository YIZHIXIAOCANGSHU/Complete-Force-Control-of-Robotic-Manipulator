"""USB2FDCAN protocol data types."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class DM_Motor_Type(IntEnum):
    DM3507 = 0
    DM4310 = 1
    DM4310_48V = 2
    DM4340 = 3
    DM4340_48V = 4
    DM6006 = 5
    DM6248 = 6
    DM8006 = 7
    DM8009 = 8
    DM10010L = 9
    DM10010 = 10
    DMH3510 = 11
    DMH6215 = 12
    DMS3519 = 13
    DMG6220 = 14


class Control_Mode(IntEnum):
    MIT_MODE = 0x000
    POS_VEL_MODE = 0x100
    VEL_MODE = 0x200
    POS_FORCE_MODE = 0x300


class Control_Mode_Code(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    POS_FORCE = 4


LIMIT_PARAM = [
    [12.566, 50, 5],
    [12.5, 30, 10],
    [12.5, 50, 10],
    [12.5, 10, 28],
    [12.5, 20, 28],
    [12.5, 45, 12],
    [12.566, 20, 120],
    [12.5, 45, 20],
    [12.5, 45, 54],
    [12.5, 25, 200],
    [12.5, 20, 200],
    [12.5, 280, 1],
    [12.5, 45, 10],
    [12.5, 2000, 2],
    [12.5, 45, 10],
]


@dataclass(frozen=True)
class MotorLimits:
    pmax: float
    vmax: float
    tmax: float


MOTOR_LIMITS = {
    motor_type: MotorLimits(*LIMIT_PARAM[motor_type.value])
    for motor_type in DM_Motor_Type
}


@dataclass
class MotorFeedback:
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    controller_id: int = 0
    state_code: int = 0
    mos_temp: float = 0.0
    rotor_temp: float = 0.0


@dataclass(frozen=True)
class DecodedFeedbackFrame:
    motor_id: int
    can_id: int
    mst_id: int
    state: int
    controller_id: int
    position: float
    velocity: float
    torque: float
    mos_temperature: float
    rotor_temperature: float


@dataclass(frozen=True)
class MotorMapping:
    motor_id: int
    can_id: int
    mst_id: int
    motor_type: DM_Motor_Type


@dataclass
class Usb2FdcanStats:
    send_count: int = 0
    read_count: int = 0
    feedback_count: int = 0
    backpressure_count: int = 0
    last_zero_packet: bytes = b""


def parse_motor_type(value: DM_Motor_Type | str) -> DM_Motor_Type:
    if isinstance(value, DM_Motor_Type):
        return value
    return DM_Motor_Type[str(value)]


def get_motor_limits(motor_type: DM_Motor_Type | str) -> MotorLimits:
    return MOTOR_LIMITS[parse_motor_type(motor_type)]


def mode_to_code(mode: Control_Mode) -> Control_Mode_Code:
    mapping = {
        Control_Mode.MIT_MODE: Control_Mode_Code.MIT,
        Control_Mode.POS_VEL_MODE: Control_Mode_Code.POS_VEL,
        Control_Mode.VEL_MODE: Control_Mode_Code.VEL,
        Control_Mode.POS_FORCE_MODE: Control_Mode_Code.POS_FORCE,
    }
    return mapping[Control_Mode(mode)]
