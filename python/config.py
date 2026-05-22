"""AM-DPBSURDF0422 左臂七轴仿真配置。"""
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
        "AM-DPBSURDF0422",
        "urdf",
        "AM-DPBSURDF0422.urdf",
    )
    RESULTS_DIR = os.path.join(PROJECT_ROOT, "results")
    
    # === 可视化配置 (Visualization) ===
    ENABLE_VIEWER = _env_bool("AM_D02_ENABLE_VIEWER", True)
    ENABLE_RERUN = _env_bool("AM_D02_ENABLE_RERUN", True)
    RERUN_LOG_STRIDE = max(1, _env_int("AM_D02_RERUN_LOG_STRIDE", 10))
    
    # === 关节配置 (Joints) ===
    NUM_JOINTS = 7
    JOINT_NAMES = [
        "ArmL02_Joint",
        "AM-D02-J14_Joint",
        "ArmL04_Joint",
        "ArmL05_Joint",
        "ArmL06_Joint",
        "ArmL07_Joint",
        "ArmL07Output_Joint",
    ]
    
    # 末端连杆名称（用于获取末端位姿）
    END_EFFECTOR_BODY = "tcp"
    
    # TCP 偏移量 (相对于 ArmL07Output_Link 本体坐标系, 单位 m)
    TCP_OFFSET = np.array([0.0, 0.07, -0.03])
    
    # 力矩限制 (N·m)
    TORQUE_LIMITS = np.array([40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 9.0])

    # 单臂参考关节安全限位 (deg/rad)。MuJoCo 双臂 URDF 的部分限位更宽，
    # 仿真侧也使用这里的单臂限位，避免 C 控制端下一帧才发现越界急停。
    JOINT_LIMITS_DEG = np.array(
        [
            [-89.971835, 89.971835],
            [-89.954374, 20.587610],
            [-68.754935, 45.836624],
            [-119.748454, 119.954374],
            [-45.836624, 45.836624],
            [-61.306275, 45.263666],
            [-61.306275, 61.306275],
        ],
        dtype=np.float64,
    )
    JOINT_LIMITS_RAD = np.deg2rad(JOINT_LIMITS_DEG)

    # 新模型腕部惯量较轻，给纯力矩仿真补一点被动阻尼/转子惯量，避免默认目标启动时冲过速度保护。
    JOINT_DAMPING = np.array([2.0, 2.0, 1.5, 1.5, 0.8, 0.8, 0.8])
    JOINT_ARMATURE = np.array([0.02, 0.02, 0.015, 0.015, 0.01, 0.01, 0.01])

    # === 仿真参数 ===
    DT = 0.01  # 仿真步长 (秒)，对应 100 Hz
    
    # === 初始位置 ===
    # 机械臂仿真实际起始关节角（全零，C端从这里出发）
    HOME_QPOS = np.array([0.0, 0.0, 0.0, np.pi/3, 0.0, 0.0, 0.0])

    # 用户希望机械臂最终到达的构型（用于FK计算方块的初始摆放位置）
    # 肘部关节抬起 90 度。
    INIT_QPOS = np.array([0.0, 0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0])

    # === 目标位置 (Target Posture) ===
    # 用于重力补偿与 PD 控制的目标位置 (rad)
    TARGET_Q = np.array([-np.pi/6, 0, 0.0, np.pi/3, 0.0, 0.0, 0.0])
