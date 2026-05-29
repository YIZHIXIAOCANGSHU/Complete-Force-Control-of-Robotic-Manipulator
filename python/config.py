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
    CONTROL_JOINT_LIMIT_INSET_RATIO = 0.01
    JOINT_VEL_LIMIT = 4.0

    # MuJoCo 不额外添加 dof damping/armature；仿真被动项只使用下面的 follower friction 模型。
    ARM_JOINT_DAMPING = np.zeros(ARM_JOINTS, dtype=np.float64)
    ARM_JOINT_ARMATURE = np.zeros(ARM_JOINTS, dtype=np.float64)
    JOINT_DAMPING = np.tile(ARM_JOINT_DAMPING, NUM_ARMS)
    JOINT_ARMATURE = np.tile(ARM_JOINT_ARMATURE, NUM_ARMS)

    # OpenArm Follower 七轴 tanh 摩擦模型，作为独立仿真物理摩擦注入，不写入 URDF。
    # tau_f = Fo + Fv * dq + Fc * tanh(0.1 * k * dq)
    ENABLE_FOLLOWER_FRICTION = _env_bool("AM_D02_ENABLE_FOLLOWER_FRICTION", True)
    FOLLOWER_FRICTION_FC = np.array([0.306, 0.306, 0.400, 0.166, 0.050, 0.093, 0.172], dtype=np.float64)
    FOLLOWER_FRICTION_K = np.array([28.417, 28.417, 29.065, 130.038, 151.771, 242.287, 7.888], dtype=np.float64)
    FOLLOWER_FRICTION_FV = np.array([0.063, 0.063, 0.604, 0.813, 0.029, 0.072, 0.084], dtype=np.float64)
    FOLLOWER_FRICTION_FO = np.array([0.088, 0.088, 0.008, -0.058, 0.005, 0.009, -0.059], dtype=np.float64)
    FOLLOWER_FRICTION_FC_14 = np.tile(FOLLOWER_FRICTION_FC, NUM_ARMS)
    FOLLOWER_FRICTION_K_14 = np.tile(FOLLOWER_FRICTION_K, NUM_ARMS)
    FOLLOWER_FRICTION_FV_14 = np.tile(FOLLOWER_FRICTION_FV, NUM_ARMS)
    FOLLOWER_FRICTION_FO_14 = np.tile(FOLLOWER_FRICTION_FO, NUM_ARMS)

    # === 控制器镜像参数 (kept in sync with stm32_code/config.h) ===
    KP_CART_X = 260.0
    KP_CART_Y = 260.0
    KP_CART_Z = 260.0
    KD_CART_X = 70.0
    KD_CART_Y = 70.0
    KD_CART_Z = 70.0
    KP_CART_ROLL = 12.0
    KP_CART_PITCH = 12.0
    KP_CART_YAW = 12.0
    KD_CART_ROLL = 4.0
    KD_CART_PITCH = 4.0
    KD_CART_YAW = 4.0
    KP_JOINT = np.array([115.0, 100.0, 30.0, 40.0, 20.0, 20.0, 20.0], dtype=np.float64)
    KD_JOINT = np.array([5.0, 5.0, 2.0, 2.0, 1.0, 1.0, 1.0], dtype=np.float64)
    Q_PREFERRED = np.array([0.0, 0.0, 0.0, np.pi / 3.0, 0.0, 0.0, 0.0], dtype=np.float64)
    POSTURE_ALPHA = 0.35
    W_CARTESIAN = 0.75
    W_JOINT = 0.25
    NULLSPACE_POS_DEADBAND = 0.001
    NULLSPACE_ORI_DEADBAND = 0.002
    NULLSPACE_POS_FULL_SCALE = 0.02
    NULLSPACE_ORI_FULL_SCALE = 0.05
    NULLSPACE_TORQUE_LIMIT = 0.08
    TRAJ_PLAN_SPEED = 0.5
    TRAJ_PLAN_ACCEL = 0.2
    CONTROL_PATH_GATE_FULL_ERROR_M = 0.005
    CONTROL_PATH_GATE_STOP_ERROR_M = 0.020
    CONTROL_PATH_GATE_RISE_TIME_S = 0.080
    CONTROL_PATH_GATE_FALL_TIME_S = 0.030
    CONTROL_PATH_LOOKAHEAD_M = 0.008
    CONTROL_PATH_LOOKAHEAD_RAMP_S = 0.30
    CONTROL_TARGET_REPLAN_POS_EPS_M = 0.001
    CONTROL_TARGET_REPLAN_ORI_EPS_RAD = 0.002

    # === 仿真参数 ===
    DT = 0.001  # 仿真步长 (秒)，对应 100 Hz
    
    # === 初始位置 ===
    # INIT_QPOS 只用于计算默认目标 TCP；仿真机器人本体启动姿态使用 HOME_QPOS。
    # 保持目标点起始仍来自旧的第四轴 pi/2 构型。
    ARM_INIT_QPOS = np.array([0.0, 0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0])
    INIT_QPOS = np.tile(ARM_INIT_QPOS, NUM_ARMS)
    # 仿真启动姿态：双臂 14 轴全 0。
    ARM_HOME_QPOS = np.zeros(ARM_JOINTS, dtype=np.float64)
    HOME_QPOS = np.tile(ARM_HOME_QPOS, NUM_ARMS)

    # === 目标位置 (Target Posture) ===
    # 用于重力补偿与 PD 控制的目标位置 (rad)
    ARM_TARGET_Q = np.array([-np.pi/6, 0, 0.0, np.pi/3, 0.0, 0.0, 0.0])
    TARGET_Q = np.tile(ARM_TARGET_Q, NUM_ARMS)
