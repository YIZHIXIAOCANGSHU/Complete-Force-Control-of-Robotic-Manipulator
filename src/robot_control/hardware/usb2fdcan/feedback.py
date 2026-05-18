"""Feedback decoding for Damiao motor frames."""

from __future__ import annotations

from robot_control.hardware.usb2fdcan.protocol import uint_to_float
from robot_control.hardware.usb2fdcan.types import DM_Motor_Type, MotorFeedback, get_motor_limits


def decode_feedback(data: bytes, motor_type: DM_Motor_Type | str) -> MotorFeedback:
    if len(data) < 8:
        raise ValueError("Motor feedback requires 8 bytes")
    limits = get_motor_limits(motor_type)
    controller_id = data[0] & 0x0F
    state_code = (data[0] >> 4) & 0x0F
    q_uint = (data[1] << 8) | data[2]
    dq_uint = (data[3] << 4) | (data[4] >> 4)
    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
    return MotorFeedback(
        position=uint_to_float(q_uint, -limits.pmax, limits.pmax, 16),
        velocity=uint_to_float(dq_uint, -limits.vmax, limits.vmax, 12),
        torque=uint_to_float(tau_uint, -limits.tmax, limits.tmax, 12),
        controller_id=controller_id,
        state_code=state_code,
        mos_temp=float(data[6]),
        rotor_temp=float(data[7]),
    )
