"""AM-DPBSURDF0422 左右双臂十四轴仿真配置。"""
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
    FIX_UNCONTROLLED_JOINTS = _env_bool("AM_D02_FIX_UNCONTROLLED_JOINTS", True)
    ENABLE_BODY_GUI = _env_bool("AM_D02_ENABLE_BODY_GUI", True)
    
    # === 关节配置 (Joints) ===
    ARM_JOINTS = 7
    NUM_ARMS = 2
    NUM_JOINTS = ARM_JOINTS * NUM_ARMS
    NUM_BODY_JOINTS = 3
    ARM_NAMES = ("left", "right")
    LEFT_ARM = 0
    RIGHT_ARM = 1
    BODY_JOINT_NAMES = [
        "Waist01_Joint",
        "Waist02_Joint",
        "Body0422_Joint",
    ]
    BODY_JOINT_LIMITS_RAD = np.array(
        [
            [0.0, 2.09],
            [-2.09, 0.0],
            [-1.57, 1.57],
        ],
        dtype=np.float64,
    )
    BODY_INIT_QPOS = np.zeros(NUM_BODY_JOINTS, dtype=np.float64)
    LEFT_JOINT_NAMES = [
        "ArmL02_Joint",
        "AM-D02-J14_Joint",
        "ArmL04_Joint",
        "ArmL05_Joint",
        "ArmL06_Joint",
        "ArmL07_Joint",
        "ArmL07Output_Joint",
    ]
    RIGHT_JOINT_NAMES = [
        "ArmR01_Joint_duplicate_2",
        "AM-D02R-J03_Joint",
        "ArmR04_Joint",
        "ArmR05_Link",
        "ArmR06_Link",
        "ArmR07_Link",
        "ArmR07Output_Link",
    ]
    JOINT_NAMES = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
    
    # 末端连杆名称（用于获取末端位姿）
    END_EFFECTOR_BODIES = ("tcp_left", "tcp_right")
    END_EFFECTOR_BODY = END_EFFECTOR_BODIES[LEFT_ARM]
    
    # TCP 偏移量 (相对于各自 Arm*07Output_Link 本体坐标系, 单位 m)
    LEFT_TCP_OFFSET = np.array([0.0, 0.07, -0.03])
    RIGHT_TCP_OFFSET = np.array([0.0, -0.07, 0.03])
    TCP_OFFSETS = np.vstack([LEFT_TCP_OFFSET, RIGHT_TCP_OFFSET])
    TCP_OFFSET = LEFT_TCP_OFFSET
    # MuJoCo body quat 使用 [w, x, y, z]。TCP +Z 指向机器人前方
    # (当前 URDF 零位 base/world +X)，同时保持左右末端坐标轴方向统一。
    LEFT_TCP_FRAME_QUAT = np.array([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
    RIGHT_TCP_FRAME_QUAT = np.array([np.sqrt(0.5), np.sqrt(0.5), 0.0, 0.0])
    TCP_FRAME_QUATS = np.vstack([LEFT_TCP_FRAME_QUAT, RIGHT_TCP_FRAME_QUAT])

    # 目标方块坐标系：原点跟随 Body0422_Link，坐标轴跟随其相对零位旋转。
    TARGET_FRAME_BODY_NAME = "Body0422_Link"
    TARGET_FRAME_MARKER_BODY = "target_frame_body0422"
    TARGET_FRAME_ORIGIN_BASE_ZERO = np.array([0.0, 0.0715607946769668, 0.213])
    TARGET_FRAME_QUAT_BASE = np.array([1.0, 0.0, 0.0, 0.0])
    
    # 力矩限制 (N·m)
    LEFT_TORQUE_LIMITS = np.array([40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 9.0])
    RIGHT_TORQUE_LIMITS = np.array([40.0, 40.0, 27.0, 27.0, 9.0, 9.0, 9.0])
    TORQUE_LIMITS = np.concatenate([LEFT_TORQUE_LIMITS, RIGHT_TORQUE_LIMITS])

    # 关节安全限位 (deg/rad)。左臂沿用真机安全限位，右臂使用当前双臂 URDF 限位。
    LEFT_JOINT_LIMITS_DEG = np.array(
        [
            [-89.971835, 89.971835],
            [-20.587610, 89.954374],
            [-45.836624, 68.754935],
            [-119.748454, 119.954374],
            [-45.836624, 45.836624],
            [-45.263666, 61.306275],
            [-61.306275, 61.306275],
        ],
        dtype=np.float64,
    )
    RIGHT_JOINT_LIMITS_RAD = np.array(
        [
            [-2.405, 2.2175],
            [-0.6605, 2.203],
            [-1.763, 1.594],
            [-0.0165, 2.3235],
            [-1.5935, 1.574],
            [-0.6015, 0.6755],
            [-1.1075, 1.068],
        ],
        dtype=np.float64,
    )
    RIGHT_JOINT_LIMITS_DEG = np.rad2deg(RIGHT_JOINT_LIMITS_RAD)
    JOINT_LIMITS_DEG = np.vstack([LEFT_JOINT_LIMITS_DEG, RIGHT_JOINT_LIMITS_DEG])
    JOINT_LIMITS_RAD = np.deg2rad(JOINT_LIMITS_DEG)

    # 新模型腕部惯量较轻，给纯力矩仿真补一点被动阻尼/转子惯量，避免默认目标启动时冲过速度保护。
    ARM_JOINT_DAMPING = np.array([2.0, 2.0, 1.5, 1.5, 0.8, 0.8, 0.8])
    ARM_JOINT_ARMATURE = np.array([0.02, 0.02, 0.015, 0.015, 0.01, 0.01, 0.01])
    JOINT_DAMPING = np.tile(ARM_JOINT_DAMPING, NUM_ARMS)
    JOINT_ARMATURE = np.tile(ARM_JOINT_ARMATURE, NUM_ARMS)

    # === 仿真参数 ===
    DT = 0.01  # 仿真步长 (秒)，对应 100 Hz
    
    # === 初始位置 ===
    # 机械臂初始关节定义：左右臂均为全 0，第四关节为 pi/2。初始目标由该构型 FK 得到。
    ARM_INIT_QPOS = np.array([0.0, 0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0])
    INIT_QPOS = np.tile(ARM_INIT_QPOS, NUM_ARMS)
    HOME_QPOS = INIT_QPOS.copy()

    # === 目标位置 (Target Posture) ===
    # 用于重力补偿与 PD 控制的目标位置 (rad)
    ARM_TARGET_Q = np.array([-np.pi/6, 0, 0.0, np.pi/3, 0.0, 0.0, 0.0])
    TARGET_Q = np.tile(ARM_TARGET_Q, NUM_ARMS)
