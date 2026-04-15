"""
AM-D02 七轴机械臂仿真配置
"""
import os
import numpy as np

def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


class Config:
    # === 路径配置 (Paths) ===
    PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    URDF_PATH = os.path.join(
        PROJECT_ROOT,
        "AM-D02-AemLURDF0413",
        "AM-D02-AemLURDF0413",
        "urdf",
        "AM-D02-AemLURDF0413.urdf",
    )
    RESULTS_DIR = os.path.join(PROJECT_ROOT, "results")
    
    # === 可视化配置 (Visualization) ===
    ENABLE_VIEWER = _env_bool("AM_D02_ENABLE_VIEWER", True)
    ENABLE_RERUN = _env_bool("AM_D02_ENABLE_RERUN", True)
    RERUN_LOG_STRIDE = max(1, _env_int("AM_D02_RERUN_LOG_STRIDE", 10))
    REAL_VIEWER_FPS = max(1.0, _env_float("AM_D02_REAL_VIEWER_FPS", 60.0))
    SERIAL_IDLE_SLEEP_S = max(0.0, _env_float("AM_D02_SERIAL_IDLE_SLEEP_S", 0.00005))
    RERUN_QUEUE_SIZE = max(8, _env_int("AM_D02_RERUN_QUEUE_SIZE", 256))
    UART_TEXT_LOG_INTERVAL = max(1, _env_int("AM_D02_UART_TEXT_LOG_INTERVAL", 100))
    SERIAL_FORWARD_TARGET = _env_bool("AM_D02_SERIAL_FORWARD_TARGET", False)
    
    # === 关节配置 (Joints) ===
    NUM_JOINTS = 7
    JOINT_NAMES = [
        "ArmLsecond_Joint",
        "ArmLthird_Joint",
        "ArmLfourth_Joint",
        "ArmLfifth_Joint",
        "ArmLsixth_Joint",
        "ArmLsixthoutput_Joint",
        "ArmLseventh_Joint",
    ]
    
    # 末端连杆名称（用于获取末端位姿）
    END_EFFECTOR_BODY = "tcp"
    
    # TCP 偏移量 (相对于 ArmLseventh_Joint 本体坐标系, 单位 m)
    TCP_OFFSET = np.array([0.0, 0.07, -0.03])  # 向前70mm, 向右40mm
    
    # 力矩限制 (N·m)
    TORQUE_LIMITS = np.array([40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 9.0])
    
    # === 仿真参数 ===
    DT = 0.001  # 仿真步长 (秒)，对应 1000 Hz
    
    # === 初始位置 ===
    # 机械臂仿真实际起始关节角（全零，C端从这里出发）
    HOME_QPOS = np.array([0.0, 0.0, 0.0, np.pi/3, 0.0, 0.0, 0.0])

    # 用户希望机械臂最终到达的构型（用于FK计算方块的初始摆放位置）
    # 修改此参数来改变方块的初始位置
    INIT_QPOS = np.array([np.pi/9, -np.pi/9, np.pi/9, np.pi/9, np.pi/9, np.pi/9, np.pi/9])

    # === 目标位置 (Target Posture) ===
    # 用于重力补偿与 PD 控制的目标位置 (rad)
    TARGET_Q = np.array([-np.pi/6, 0, 0.0, np.pi/3, 0.0, 0.0, 0.0])

