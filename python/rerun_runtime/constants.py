"""Shared constants for Rerun visualization helpers."""

AXIS_COLORS = {
    "X": [230, 80, 80],
    "Y": [80, 190, 80],
    "Z": [80, 120, 230],
    "Roll": [230, 80, 80],
    "Pitch": [80, 190, 80],
    "Yaw": [80, 120, 230],
}

MODE_COLORS = {
    "mujoco": [50, 150, 230],
    "c_engine": [230, 100, 50],
}
ACTUAL_COLOR = [50, 150, 230]
TARGET_COLOR = [80, 200, 90]
REFERENCE_COLOR = [240, 150, 40]
ERROR_COLOR = [220, 70, 70]
LIMIT_COLOR = [130, 130, 130]
LEFT_ARM_COLOR = [50, 150, 230]
RIGHT_ARM_COLOR = [230, 120, 50]

JOINT_COLORS = [
    [230, 50, 50],
    [230, 140, 30],
    [210, 200, 30],
    [50, 200, 50],
    [50, 200, 200],
    [50, 80, 230],
    [150, 50, 230],
] * 2

POSITION_DISPLAY_UNIT = "mm"
POSITION_DISPLAY_SCALE = 1000.0
SAFETY_LOG_MARGIN_RAD = 0.02
SAFETY_LOG_MARGIN_RAD_S = 0.2
SAFETY_TEXT_LOG_INTERVAL_STEPS = 500
POSE_NAME_MAP = {
    "零位": "zero",
    "伸展位": "extend",
    "随机位": "random",
}
ARM_LABELS = ("left", "right")
ARM_DISPLAY_NAMES = ("Left", "Right")
