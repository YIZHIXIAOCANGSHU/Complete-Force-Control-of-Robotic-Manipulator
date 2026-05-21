"""
AM-D02 七轴机械臂仿真配置
"""
import os
from pathlib import Path

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
    PROJECT_ROOT = Path(__file__).resolve().parents[2]
    URDF_PATH = str(
        PROJECT_ROOT
        / "assets"
        / "robot_model"
        / "AM-D02-AemLURDF0413"
        / "urdf"
        / "AM-D02-AemLURDF0413.urdf"
    )
    RESULTS_DIR = str(PROJECT_ROOT / "outputs")
    
    # === 可视化配置 (Visualization) ===
    ENABLE_VIEWER = _env_bool("AM_D02_ENABLE_VIEWER", True)
    ENABLE_RERUN = _env_bool("AM_D02_ENABLE_RERUN", True)
    ENABLE_MUJOCO_GHOST = _env_bool("AM_D02_ENABLE_MUJOCO_GHOST", True)
    MUJOCO_GHOST_ALPHA = max(0.0, min(1.0, _env_float("AM_D02_MUJOCO_GHOST_ALPHA", 0.25)))
    RERUN_LOG_STRIDE = max(1, _env_int("AM_D02_RERUN_LOG_STRIDE", 10))
    REAL_VIEWER_FPS = max(1.0, _env_float("AM_D02_REAL_VIEWER_FPS", 60.0))
    RERUN_QUEUE_SIZE = max(8, _env_int("AM_D02_RERUN_QUEUE_SIZE", 256))
    
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
    DT = 0.002  # MuJoCo 仿真步长 (秒)
    SIM_REALTIME = _env_bool("AM_D02_SIM_REALTIME", True)
    
    # === 初始位置 ===
    # 机械臂仿真实际起始关节角
    HOME_QPOS = np.array([0.0, 0.0, 0.0, np.pi/3, 0.0, 0.0, 0.0])

    # 用户希望机械臂最终到达的构型（用于FK计算方块的初始摆放位置）
    # 修改此参数来改变方块的初始位置
    INIT_QPOS = np.array([np.pi/9, -np.pi/9, np.pi/9, np.pi/9, np.pi/9, np.pi/9, np.pi/9])

    # === 参数辨识配置 ===
    PARAM_ID_JOINT_PRIORS = [
        {"fc": 0.306, "k": 28.417, "fv": 0.063, "fo": 0.088},
        {"fc": 0.306, "k": 28.417, "fv": 0.063, "fo": 0.088},
        {"fc": 0.400, "k": 29.065, "fv": 0.604, "fo": 0.008},
        {"fc": 0.166, "k": 130.038, "fv": 0.813, "fo": -0.058},
        {"fc": 0.050, "k": 151.771, "fv": 0.029, "fo": 0.005},
        {"fc": 0.083, "k": 242.287, "fv": 0.072, "fo": 0.009},
        {"fc": 0.172, "k": 7.888, "fv": 0.084, "fo": -0.059},
    ]
    PARAM_ID_MAX_EE_SPEED = 3.0
    PARAM_ID_REALTIME = _env_bool("AM_D02_PARAM_ID_REALTIME", True)
    PARAM_ID_COULOMB_EPS = 0.02
    PARAM_ID_TRAJECTORY_CANDIDATES = max(1, _env_int("AM_D02_PARAM_ID_TRAJECTORY_CANDIDATES", 1))
    PARAM_ID_TRAJECTORY_SEEDS = os.getenv("AM_D02_PARAM_ID_TRAJECTORY_SEEDS", "43")
    PARAM_ID_TRAJECTORY_PROFILES = max(1, _env_int("AM_D02_PARAM_ID_TRAJECTORY_PROFILES", 8))
    PARAM_ID_TRAJECTORY_PROFILE_DIAGNOSTICS = _env_bool("AM_D02_PARAM_ID_TRAJECTORY_PROFILE_DIAGNOSTICS", True)
    PARAM_ID_PRIOR_LAMBDA_INERTIAL = max(0.0, _env_float("AM_D02_PARAM_ID_PRIOR_LAMBDA_INERTIAL", 2e-1))
    PARAM_ID_PRIOR_LAMBDA_MASS = max(0.0, _env_float("AM_D02_PARAM_ID_PRIOR_LAMBDA_MASS", 64.0))
    PARAM_ID_PRIOR_LAMBDA_COM = max(0.0, _env_float("AM_D02_PARAM_ID_PRIOR_LAMBDA_COM", 1.2))
    PARAM_ID_PRIOR_LAMBDA_INERTIA = max(0.0, _env_float("AM_D02_PARAM_ID_PRIOR_LAMBDA_INERTIA", 2.4))
    PARAM_ID_PRIOR_LAMBDA_JOINT = max(0.0, _env_float("AM_D02_PARAM_ID_PRIOR_LAMBDA_JOINT", 3.5e-2))
    PARAM_ID_MASS_ERROR_TARGET_PCT = max(0.0, _env_float("AM_D02_PARAM_ID_MASS_ERROR_TARGET_PCT", 5.0))
    PARAM_ID_COM_ERROR_TARGET_M = max(0.0, _env_float("AM_D02_PARAM_ID_COM_ERROR_TARGET_M", 0.01))
    PARAM_ID_INERTIA_ERROR_TARGET_PCT = max(0.0, _env_float("AM_D02_PARAM_ID_INERTIA_ERROR_TARGET_PCT", 15.0))
    PARAM_ID_VALIDATION_TOP_N = max(1, _env_int("AM_D02_PARAM_ID_VALIDATION_TOP_N", 5))
    PARAM_ID_REG_SWEEP = _env_bool("AM_D02_PARAM_ID_REG_SWEEP", True)
    PARAM_ID_REAL_START_TOL_RAD = max(0.0, _env_float("AM_D02_PARAM_ID_REAL_START_TOL_RAD", 0.15))
    PARAM_ID_REAL_MIN_SAMPLES = max(2, _env_int("AM_D02_PARAM_ID_REAL_MIN_SAMPLES", 50))
    PARAM_ID_REAL_MIN_TAU_RMS = max(0.0, _env_float("AM_D02_PARAM_ID_REAL_MIN_TAU_RMS", 1e-4))
    PARAM_ID_PD_VALIDATION_REG_GRID_LIMIT = max(0, _env_int("AM_D02_PARAM_ID_PD_VALIDATION_REG_GRID_LIMIT", 0))
    PARAM_ID_PD_JOINT_PRIOR_SCALE = max(0.0, _env_float("AM_D02_PARAM_ID_PD_JOINT_PRIOR_SCALE", 0.0))
    PARAM_ID_PD_INERTIA_ERROR_TARGET_PCT = max(0.0, _env_float("AM_D02_PARAM_ID_PD_INERTIA_ERROR_TARGET_PCT", 5.0))
    PARAM_ID_PD_STRICT_MASS_PRIOR_LAMBDA = max(0.0, _env_float("AM_D02_PARAM_ID_PD_STRICT_MASS_PRIOR_LAMBDA", 64.0))
    PARAM_ID_PD_STRICT_COM_PRIOR_LAMBDA = max(0.0, _env_float("AM_D02_PARAM_ID_PD_STRICT_COM_PRIOR_LAMBDA", 1.6))
    PARAM_ID_PD_STRICT_INERTIA_PRIOR_LAMBDA = max(
        0.0,
        _env_float("AM_D02_PARAM_ID_PD_STRICT_INERTIA_PRIOR_LAMBDA", 6.4),
    )
    PARAM_ID_PD_DISTAL_INERTIA_PRIOR_MULTIPLIER = max(
        0.0,
        _env_float("AM_D02_PARAM_ID_PD_DISTAL_INERTIA_PRIOR_MULTIPLIER", 10.0),
    )
    PARAM_ID_RCOND = max(0.0, _env_float("AM_D02_PARAM_ID_RCOND", 1e-8))
    PARAM_ID_RIDGE = max(0.0, _env_float("AM_D02_PARAM_ID_RIDGE", 1e-8))
    PARAM_ID_DISTAL_WEIGHT = max(0.0, _env_float("AM_D02_PARAM_ID_DISTAL_WEIGHT", 2.0))
    PARAM_ID_DISTAL_LINK_START = min(7, max(1, _env_int("AM_D02_PARAM_ID_DISTAL_LINK_START", 5)))
    PARAM_ID_MAX_SAMPLES = max(50, _env_int("AM_D02_PARAM_ID_MAX_SAMPLES", 700))
    PARAM_ID_ENABLE_HTML_REPORT = _env_bool("AM_D02_PARAM_ID_HTML", True)
    PARAM_ID_HTML_OPEN_BROWSER = _env_bool("AM_D02_PARAM_ID_HTML_OPEN_BROWSER", False)

    # === 目标位置 (Target Posture) ===
    # 用于重力补偿与 PD 控制的目标位置 (rad)
    TARGET_Q = np.array([0, -np.pi/4, 0.0, np.pi/4, 0.0, 0.0, 0.0])
