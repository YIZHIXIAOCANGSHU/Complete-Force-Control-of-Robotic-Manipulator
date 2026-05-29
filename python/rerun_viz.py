"""
Rerun 可视化模块
输出图表:
- 末端位置误差: X/Y/Z 轴各一个图表 (MuJoCo vs RBDL-Lite)
- 末端姿态误差: Roll/Pitch/Yaw 各一个图表 (MuJoCo vs RBDL-Lite)
- 关节力矩误差: 每个关节一个图表 (Δτ = RBDL-Lite - MuJoCo)
- 静态自稳: 各姿态的角速度和漂移
"""
import numpy as np

try:
    import rerun as rr
    import rerun.blueprint as rrb
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False
    print("警告: rerun-sdk 未安装，将跳过 Rerun 可视化")

from config import Config

_AXIS_COLORS = {
    'X':     [230, 80, 80],
    'Y':     [80, 190, 80],
    'Z':     [80, 120, 230],
    'Roll':  [230, 80, 80],
    'Pitch': [80, 190, 80],
    'Yaw':   [80, 120, 230],
}

_MODE_COLORS = {
    'mujoco': [50, 150, 230],
    'c_engine': [230, 100, 50],
}

_JOINT_COLORS = [
    [230, 50, 50],
    [230, 140, 30],
    [210, 200, 30],
    [50, 200, 50],
    [50, 200, 200],
    [50, 80, 230],
    [150, 50, 230],
] * 2

_POSITION_DISPLAY_UNIT = "mm"
_POSITION_DISPLAY_SCALE = 1000.0
_SAFETY_LOG_MARGIN_RAD = 0.02
_SAFETY_LOG_MARGIN_RAD_S = 0.2
_SAFETY_TEXT_LOG_INTERVAL_STEPS = 500
_last_safety_warning_signature: str | None = None
_last_safety_warning_step: int | None = None


_POSE_NAME_MAP = {
    "零位": "zero",
    "伸展位": "extend",
    "随机位": "random",
}

_ARM_LABELS = ("left", "right")
_ARM_DISPLAY_NAMES = ("Left", "Right")


def _as_arm_array(values: np.ndarray, width: int) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.shape == (width,):
        return array.reshape(1, width)
    if array.shape == (Config.NUM_ARMS * width,):
        return array.reshape(Config.NUM_ARMS, width)
    if array.shape == (Config.NUM_ARMS, width):
        return array
    raise ValueError(f"expected shape ({width},) or ({Config.NUM_ARMS}, {width}), got {array.shape}")

def _safe_pose_name(name: str) -> str:
    """将中文姿态名转为 ASCII 安全名称"""
    return _POSE_NAME_MAP.get(name, f"pose_{hash(name) % 10000}")


def _position_to_display_units(position: np.ndarray) -> np.ndarray:
    """将内部米制位置转换为 Rerun 图表展示使用的毫米。"""
    return np.asarray(position, dtype=np.float64) * _POSITION_DISPLAY_SCALE


def quaternion_to_euler(w, x, y, z):
    """
    四元数转欧拉角 (Roll, Pitch, Yaw) - ZYX 顺序
    """
    import math
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])

def quat_to_euler(quat):
    """适配 [w, x, y, z] 或 [x, y, z, w] 的包装"""
    # 假设输入是 [w, x, y, z]
    return quaternion_to_euler(quat[0], quat[1], quat[2], quat[3])

def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """四元数乘法 [w, x, y, z]"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def compute_rotation_error(quat_actual: np.ndarray, quat_desired: np.ndarray) -> np.ndarray:
    """
    计算旋转误差 (欧拉角表示, 单位 deg)
    quat_actual: (N, 4), quat_desired: (N, 4), 格式 [w,x,y,z]
    返回: (N, 3) [roll_err, pitch_err, yaw_err] in degrees
    """
    N = len(quat_actual)
    errors = np.zeros((N, 3))
    for i in range(N):
        q_act = quat_actual[i]
        q_des = quat_desired[i]
        q_des_inv = np.array([q_des[0], -q_des[1], -q_des[2], -q_des[3]])
        q_err = quaternion_multiply(q_act, q_des_inv)
        if q_err[0] < 0:
            q_err = -q_err
        errors[i] = quat_to_euler(q_err)
    return np.degrees(errors)


def compute_rotation_error_single(quat_actual: np.ndarray, quat_desired: np.ndarray) -> np.ndarray:
    """单步姿态误差，避免为单个样本构造额外批量数组。"""
    q_des_inv = np.array([quat_desired[0], -quat_desired[1], -quat_desired[2], -quat_desired[3]])
    q_err = quaternion_multiply(quat_actual, q_des_inv)
    if q_err[0] < 0:
        q_err = -q_err
    return np.rad2deg(quat_to_euler(q_err))


def _joint_safe_limits_rad() -> tuple[np.ndarray, np.ndarray]:
    limits = np.asarray(Config.JOINT_LIMITS_RAD, dtype=np.float64)
    span = limits[:, 1] - limits[:, 0]
    inset = float(Config.CONTROL_JOINT_LIMIT_INSET_RATIO) * span
    return limits[:, 0] + inset, limits[:, 1] - inset


def _joint_safety_margins(q: np.ndarray | None, qd: np.ndarray | None):
    q_margin_low = q_margin_high = vel_margin = None
    if q is not None:
        q_values = np.asarray(q, dtype=np.float64).reshape(Config.NUM_JOINTS)
        safe_min, safe_max = _joint_safe_limits_rad()
        q_margin_low = q_values - safe_min
        q_margin_high = safe_max - q_values
    if qd is not None:
        qd_values = np.asarray(qd, dtype=np.float64).reshape(Config.NUM_JOINTS)
        vel_margin = float(Config.JOINT_VEL_LIMIT) - np.abs(qd_values)
    return q_margin_low, q_margin_high, vel_margin


def _format_safety_warnings(
    q_margin_low: np.ndarray | None,
    q_margin_high: np.ndarray | None,
    vel_margin: np.ndarray | None,
) -> str | None:
    warnings = []
    for arm, arm_label in enumerate(_ARM_LABELS):
        offset = arm * Config.ARM_JOINTS
        for i in range(Config.ARM_JOINTS):
            joint = f"{arm_label}/J{i + 1}"
            index = offset + i
            if q_margin_low is not None and q_margin_low[index] <= _SAFETY_LOG_MARGIN_RAD:
                warnings.append(f"{joint} low_limit_margin={q_margin_low[index]:.4f}rad")
            if q_margin_high is not None and q_margin_high[index] <= _SAFETY_LOG_MARGIN_RAD:
                warnings.append(f"{joint} high_limit_margin={q_margin_high[index]:.4f}rad")
            if vel_margin is not None and vel_margin[index] <= _SAFETY_LOG_MARGIN_RAD_S:
                warnings.append(f"{joint} vel_margin={vel_margin[index]:.4f}rad/s")
    return "; ".join(warnings) if warnings else None


def _safety_warning_signature(safety_warning: str) -> str:
    parts = []
    for item in safety_warning.split("; "):
        label = item.split("=", 1)[0]
        parts.append(label)
    return "; ".join(parts)


def _should_log_safety_warning(safety_warning: str, step_count: int) -> bool:
    global _last_safety_warning_signature, _last_safety_warning_step

    signature = _safety_warning_signature(safety_warning)
    if signature != _last_safety_warning_signature:
        _last_safety_warning_signature = signature
        _last_safety_warning_step = int(step_count)
        return True

    last_step = _last_safety_warning_step
    if last_step is None or int(step_count) - last_step >= _SAFETY_TEXT_LOG_INTERVAL_STEPS:
        _last_safety_warning_step = int(step_count)
        return True

    return False


def _reset_safety_warning_throttle() -> None:
    global _last_safety_warning_signature, _last_safety_warning_step
    _last_safety_warning_signature = None
    _last_safety_warning_step = None

def init_rerun(app_name: str = "AM-D02 Simulation"):
    """初始化 Rerun (不发送 Blueprint，等数据写入后再发)"""
    if not RERUN_AVAILABLE:
        return False
    rr.init(app_name, spawn=True)
    # 强制在某些环境下也尝试打开浏览器界面
    # rr.spawn() 
    return True


def _setup_trajectory_styles():
    """设置 3D 和性能曲线的基础样式。"""
    rr.log(
        "trajectory_3d/origin",
        rr.Arrows3D(
            vectors=[[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
            colors=[[220, 50, 50], [50, 220, 50], [50, 50, 220]],
        ),
        static=True,
    )

    rr.log("performance/c_engine_time",
           rr.SeriesLines(colors=[_MODE_COLORS['c_engine']],
                          names=["Python Control Step Time (ms)"],
                          widths=[2]),
           static=True)

    rr.log("performance/link_latency",
           rr.SeriesLines(colors=[[230, 150, 50]],
                          names=["Control Link Period (ms)"],
                          widths=[2]),
           static=True)

    rr.log("performance/link_cycle_hz",
           rr.SeriesLines(colors=[[230, 120, 40]],
                          names=["Control Link Rate (Hz)"],
                          widths=[2]),
           static=True)

    rr.log("performance/link_transfer_kbps",
           rr.SeriesLines(colors=[[230, 180, 80]],
                          names=["Control Link Throughput (kbps)"],
                          widths=[2]),
           static=True)
    
    # STM32 计算耗时样式
    rr.log("performance/stm32_calc_time",
           rr.SeriesLines(colors=[[100, 200, 100]],
                          names=["STM32 Algorithm Calc Time (ms)"],
                          widths=[2]),
           static=True)

    rr.log("performance/stm32_calc_hz",
           rr.SeriesLines(colors=[[80, 180, 220]],
                          names=["STM32 Calc Rate (Hz)"],
                          widths=[2]),
           static=True)


def _setup_arm_realtime_styles() -> None:
    """为左右臂合并曲线写入 Rerun SeriesLines 样式。"""
    for arm_label, arm_name in zip(_ARM_LABELS, _ARM_DISPLAY_NAMES):
        for axis in ("X", "Y", "Z"):
            rr.log(
                f"arms/{arm_label}/position/{axis}",
                rr.SeriesLines(
                    colors=[_AXIS_COLORS[axis], _AXIS_COLORS[axis]],
                    names=[f"{arm_name} {axis} Actual", f"{arm_name} {axis} Target"],
                    widths=[2.0, 1.2],
                ),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/position_error/{axis}",
                rr.SeriesLines(
                    colors=[_AXIS_COLORS[axis]],
                    names=[f"{arm_name} {axis} Error"],
                    widths=[2.0],
                ),
                static=True,
            )
        for axis in ("Roll", "Pitch", "Yaw"):
            rr.log(
                f"arms/{arm_label}/rotation/{axis}",
                rr.SeriesLines(
                    colors=[_AXIS_COLORS[axis], _AXIS_COLORS[axis]],
                    names=[f"{arm_name} {axis} Actual", f"{arm_name} {axis} Target"],
                    widths=[2.0, 1.2],
                ),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/rotation_error/{axis}",
                rr.SeriesLines(
                    colors=[_AXIS_COLORS[axis]],
                    names=[f"{arm_name} {axis} Error"],
                    widths=[2.0],
                ),
                static=True,
            )
        for i in range(Config.ARM_JOINTS):
            joint = f"J{i + 1}"
            color = _JOINT_COLORS[i]
            rr.log(
                f"arms/{arm_label}/joint_q/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} q"], widths=[2.0]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/joint_qd/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} qd"], widths=[2.0]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/torque/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} Tau"], widths=[2.0]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/torque_actual/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} Actual Tau"], widths=[1.5]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/torque_gap/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} Tau Gap"], widths=[2.0]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/velocity_margin/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} Velocity Margin"], widths=[2.0]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/limit_margin_low/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} Low Limit Margin"], widths=[1.5]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/limit_margin_high/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} High Limit Margin"], widths=[1.5]),
                static=True,
            )

def setup_realtime_styles():
    """设置交互式 Rerun 的曲线样式和试图蓝图，在仿真启动前调用"""
    if not RERUN_AVAILABLE: return
    _setup_trajectory_styles()
    _setup_arm_realtime_styles()

    rr.set_time_seconds("time", 0.0)
    for arm_label in _ARM_LABELS:
        for axis in ("X", "Y", "Z"):
            rr.log(f"arms/{arm_label}/position/{axis}/actual", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/position/{axis}/target", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/position_error/{axis}/value", rr.Scalars(0.0))
        for axis in ("Roll", "Pitch", "Yaw"):
            rr.log(f"arms/{arm_label}/rotation/{axis}/actual", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/rotation/{axis}/target", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/rotation_error/{axis}/value", rr.Scalars(0.0))
        for i in range(Config.ARM_JOINTS):
            joint = f"J{i + 1}"
            rr.log(f"arms/{arm_label}/joint_q/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/joint_qd/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/torque/{joint}/command", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/torque_actual/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/torque_gap/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/velocity_margin/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/limit_margin_low/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/limit_margin_high/{joint}/value", rr.Scalars(0.0))
    rr.log("performance/c_engine_time", rr.Scalars(0.0))
    rr.log("performance/link_latency", rr.Scalars(0.0))
    rr.log("performance/link_cycle_hz", rr.Scalars(0.0))
    rr.log("performance/link_transfer_kbps", rr.Scalars(0.0))
    rr.log("performance/stm32_calc_time", rr.Scalars(0.0))
    rr.log("performance/stm32_calc_hz", rr.Scalars(0.0))

    def link_latency_view():
        return rrb.TimeSeriesView(
            name="Control Link Period (ms)", origin="/performance/link_latency",
        )

    link_log_view = rrb.TextLogView(name="Control Link Log", origin="/control_link_log")

    latency_view = link_latency_view()

    link_cycle_rate_view = rrb.TimeSeriesView(
        name="Control Link Rate (Hz)", origin="/performance/link_cycle_hz",
    )

    link_transfer_rate_view = rrb.TimeSeriesView(
        name="Control Link Throughput (kbps)", origin="/performance/link_transfer_kbps",
    )
    
    stm32_time_view = rrb.TimeSeriesView(
        name="STM32 Calculation Time (ms)", origin="/performance/stm32_calc_time",
    )

    stm32_rate_view = rrb.TimeSeriesView(
        name="STM32 Calculation Rate (Hz)", origin="/performance/stm32_calc_hz",
    )

    python_time_view = rrb.TimeSeriesView(
        name="Python Control Step Time (ms)", origin="/performance/c_engine_time",
    )

    def control_link_view():
        return rrb.Vertical(
            link_log_view,
            link_latency_view(),
            name="Control Link",
        )

    def arm_view(arm_label: str, arm_name: str, key: str, title: str, unit: str):
        return rrb.Vertical(
            rrb.TimeSeriesView(
                name=f"{arm_name} {title} ({unit})",
                origin=f"/arms/{arm_label}/{key}",
            ),
            name=f"{arm_name} {title}",
        )

    def arm_axis_view(
        arm_label: str,
        arm_name: str,
        key: str,
        title: str,
        axes: tuple[str, ...],
        unit: str,
    ):
        return rrb.Vertical(
            *[
                rrb.TimeSeriesView(
                    name=f"{arm_name} {title} {axis} ({unit})",
                    origin=f"/arms/{arm_label}/{key}/{axis}",
                )
                for axis in axes
            ],
            name=f"{arm_name} {title}",
        )

    def arm_joint_axis_view(arm_label: str, arm_name: str, key: str, title: str, unit: str):
        return arm_axis_view(
            arm_label,
            arm_name,
            key,
            title,
            tuple(f"J{i + 1}" for i in range(Config.ARM_JOINTS)),
            unit,
        )

    blueprint = rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(name="3D", origin="/trajectory_3d"),
            arm_axis_view("left", "Left", "position", "Position", ("X", "Y", "Z"), _POSITION_DISPLAY_UNIT),
            arm_axis_view("right", "Right", "position", "Position", ("X", "Y", "Z"), _POSITION_DISPLAY_UNIT),
            arm_axis_view("left", "Left", "rotation", "Rotation", ("Roll", "Pitch", "Yaw"), "deg"),
            arm_axis_view("right", "Right", "rotation", "Rotation", ("Roll", "Pitch", "Yaw"), "deg"),
            arm_axis_view("left", "Left", "position_error", "Position Error", ("X", "Y", "Z"), "mm"),
            arm_axis_view("right", "Right", "position_error", "Position Error", ("X", "Y", "Z"), "mm"),
            arm_axis_view("left", "Left", "rotation_error", "Rotation Error", ("Roll", "Pitch", "Yaw"), "deg"),
            arm_axis_view("right", "Right", "rotation_error", "Rotation Error", ("Roll", "Pitch", "Yaw"), "deg"),
            arm_joint_axis_view("left", "Left", "joint_q", "Joint Q", "rad"),
            arm_joint_axis_view("right", "Right", "joint_q", "Joint Q", "rad"),
            arm_joint_axis_view("left", "Left", "joint_qd", "Joint QD", "rad/s"),
            arm_joint_axis_view("right", "Right", "joint_qd", "Joint QD", "rad/s"),
            arm_joint_axis_view("left", "Left", "torque", "Torque", "N*m"),
            arm_joint_axis_view("right", "Right", "torque", "Torque", "N*m"),
            arm_joint_axis_view("left", "Left", "torque_gap", "Torque Gap", "N*m"),
            arm_joint_axis_view("right", "Right", "torque_gap", "Torque Gap", "N*m"),
            arm_joint_axis_view("left", "Left", "velocity_margin", "Velocity Safety Margin", "rad/s"),
            arm_joint_axis_view("right", "Right", "velocity_margin", "Velocity Safety Margin", "rad/s"),
            arm_joint_axis_view("left", "Left", "limit_margin_low", "Low Limit Margin", "rad"),
            arm_joint_axis_view("right", "Right", "limit_margin_low", "Low Limit Margin", "rad"),
            arm_joint_axis_view("left", "Left", "limit_margin_high", "High Limit Margin", "rad"),
            arm_joint_axis_view("right", "Right", "limit_margin_high", "High Limit Margin", "rad"),
            rrb.Vertical(
                python_time_view,
                link_cycle_rate_view,
                link_transfer_rate_view,
                latency_view,
                stm32_time_view,
                stm32_rate_view,
                name="Performance",
            ),
            control_link_view(),
        ),
        collapse_panels=True,
    )
    rr.send_blueprint(blueprint)

def log_realtime_step(
    t: float,
    pos_actual: np.ndarray,
    pos_desired: np.ndarray,
    quat_actual: np.ndarray,
    quat_desired: np.ndarray,
    tau_total: np.ndarray,
    cycle_time: float,
    q: np.ndarray = None,
    qd: np.ndarray = None,
    tau_raw: np.ndarray = None,
    tau_actual: np.ndarray = None,
    elapsed_s: float = None,
    right_j7_diag: dict = None,
    rx_str: str = None,
    tx_str: str = None,
    tx_label: str = "Torques",
    step_count: int = 0,
    uart_latency_ms: float = None,
    uart_cycle_hz: float = None,
    uart_transfer_kbps: float = None,
    stm32_calc_time_ms: float = None,
    stm32_calc_hz: float = None,
):
    """单步记录交互式仿真数据"""
    if not RERUN_AVAILABLE: return
    if Config.RERUN_LOG_STRIDE > 1 and step_count % Config.RERUN_LOG_STRIDE != 0:
        return
    rr.set_time_seconds("time", t)
    pos_actual_by_arm = _as_arm_array(pos_actual, 3)
    pos_desired_by_arm = _as_arm_array(pos_desired, 3)
    quat_actual_by_arm = _as_arm_array(quat_actual, 4)
    quat_desired_by_arm = _as_arm_array(quat_desired, 4)
    
    for arm, arm_label in enumerate(_ARM_LABELS[: len(pos_actual_by_arm)]):
        pos_actual_display = _position_to_display_units(pos_actual_by_arm[arm])
        pos_desired_display = _position_to_display_units(pos_desired_by_arm[arm])
        for i, axis in enumerate(('X', 'Y', 'Z')):
            rr.log(f"arms/{arm_label}/position/{axis}/actual", rr.Scalars(float(pos_actual_display[i])))
            rr.log(f"arms/{arm_label}/position/{axis}/target", rr.Scalars(float(pos_desired_display[i])))
            rr.log(f"arms/{arm_label}/position_error/{axis}/value", rr.Scalars(float(pos_actual_display[i] - pos_desired_display[i])))

        rot_actual = quat_to_euler(quat_actual_by_arm[arm])
        rot_desired = quat_to_euler(quat_desired_by_arm[arm])
        rot_err = compute_rotation_error_single(quat_actual_by_arm[arm], quat_desired_by_arm[arm])
        rot_actual_deg = np.rad2deg(rot_actual)
        rot_desired_deg = np.rad2deg(rot_desired)

        for i, axis in enumerate(('Roll', 'Pitch', 'Yaw')):
            rr.log(f"arms/{arm_label}/rotation/{axis}/actual", rr.Scalars(float(rot_actual_deg[i])))
            rr.log(f"arms/{arm_label}/rotation/{axis}/target", rr.Scalars(float(rot_desired_deg[i])))
            rr.log(f"arms/{arm_label}/rotation_error/{axis}/value", rr.Scalars(float(rot_err[i])))

    if q is not None:
        q_by_arm = _as_arm_array(q, Config.ARM_JOINTS)
        for arm, arm_label in enumerate(_ARM_LABELS[: len(q_by_arm)]):
            for i, value in enumerate(q_by_arm[arm]):
                rr.log(f"arms/{arm_label}/joint_q/J{i+1}/value", rr.Scalars(float(value)))
    if qd is not None:
        qd_by_arm = _as_arm_array(qd, Config.ARM_JOINTS)
        for arm, arm_label in enumerate(_ARM_LABELS[: len(qd_by_arm)]):
            for i, value in enumerate(qd_by_arm[arm]):
                rr.log(f"arms/{arm_label}/joint_qd/J{i+1}/value", rr.Scalars(float(value)))
        
    tau_by_arm = _as_arm_array(tau_total, Config.ARM_JOINTS)
    tau_raw_by_arm = None if tau_raw is None else _as_arm_array(tau_raw, Config.ARM_JOINTS)
    tau_actual_by_arm = None if tau_actual is None else _as_arm_array(tau_actual, Config.ARM_JOINTS)
    for arm, arm_label in enumerate(_ARM_LABELS[: len(tau_by_arm)]):
        for i, value in enumerate(tau_by_arm[arm]):
            rr.log(f"arms/{arm_label}/torque/J{i+1}/command", rr.Scalars(float(value)))
            if tau_raw_by_arm is not None:
                rr.log(f"arms/{arm_label}/torque_raw/J{i+1}/value", rr.Scalars(float(tau_raw_by_arm[arm, i])))
            if tau_actual_by_arm is not None:
                rr.log(f"arms/{arm_label}/torque_actual/J{i+1}/value", rr.Scalars(float(tau_actual_by_arm[arm, i])))
                rr.log(
                    f"arms/{arm_label}/torque_gap/J{i+1}/value",
                    rr.Scalars(float(value - tau_actual_by_arm[arm, i])),
                )

    q_margin_low, q_margin_high, vel_margin = _joint_safety_margins(q, qd)
    if vel_margin is not None:
        vel_margin_by_arm = _as_arm_array(vel_margin, Config.ARM_JOINTS)
        for arm, arm_label in enumerate(_ARM_LABELS[: len(vel_margin_by_arm)]):
            for i, value in enumerate(vel_margin_by_arm[arm]):
                rr.log(f"arms/{arm_label}/velocity_margin/J{i+1}/value", rr.Scalars(float(value)))
    if q_margin_low is not None and q_margin_high is not None:
        low_by_arm = _as_arm_array(q_margin_low, Config.ARM_JOINTS)
        high_by_arm = _as_arm_array(q_margin_high, Config.ARM_JOINTS)
        for arm, arm_label in enumerate(_ARM_LABELS[: len(low_by_arm)]):
            for i, value in enumerate(low_by_arm[arm]):
                rr.log(f"arms/{arm_label}/limit_margin_low/J{i+1}/value", rr.Scalars(float(value)))
                rr.log(f"arms/{arm_label}/limit_margin_high/J{i+1}/value", rr.Scalars(float(high_by_arm[arm, i])))

    safety_warning = _format_safety_warnings(q_margin_low, q_margin_high, vel_margin)
    if safety_warning and _should_log_safety_warning(safety_warning, step_count):
        text = f"[{step_count}] SAFETY margin warning: {safety_warning}"
        rr.log("control_link_log", rr.TextLog(text))
        print(f"[Rerun Safety] {text}")
    elif not safety_warning:
        _reset_safety_warning_throttle()
            
    # Text Log for Sent/Received Data (Throttled to 10Hz to prevent lag at 1kHz loop)
    if rx_str and tx_str:
        rr.log("control_link_log", rr.TextLog(f"[{step_count}] RX (Positions): {rx_str}\n[{step_count}] TX ({tx_label}): {tx_str}"))

    # Performance
    rr.log("performance/c_engine_time", rr.Scalars(float(cycle_time)))
    if elapsed_s is not None:
        rr.log("performance/elapsed_s", rr.Scalars(float(elapsed_s)))
    if uart_latency_ms is not None:
        rr.log("performance/link_latency", rr.Scalars(float(uart_latency_ms)))
    if uart_cycle_hz is not None:
        rr.log("performance/link_cycle_hz", rr.Scalars(float(uart_cycle_hz)))
    if uart_transfer_kbps is not None:
        rr.log("performance/link_transfer_kbps", rr.Scalars(float(uart_transfer_kbps)))
    if stm32_calc_time_ms is not None:
        rr.log("performance/stm32_calc_time", rr.Scalars(float(stm32_calc_time_ms)))
    if stm32_calc_hz is not None:
        rr.log("performance/stm32_calc_hz", rr.Scalars(float(stm32_calc_hz)))

    if right_j7_diag:
        for key in ("q", "qd", "tau_cmd_raw", "tau_cmd_sent", "tau_actual"):
            if key in right_j7_diag:
                rr.log(f"diagnostics/right_j7/{key}", rr.Scalars(float(right_j7_diag[key])))
    
    # 3D
    actual_colors = [[230, 100, 50], [50, 150, 230]][: len(pos_actual_by_arm)]
    target_colors = [[80, 220, 80], [230, 220, 80]][: len(pos_desired_by_arm)]
    labels_actual = [f"{name} Actual" for name in _ARM_DISPLAY_NAMES[: len(pos_actual_by_arm)]]
    labels_target = [f"{name} Target" for name in _ARM_DISPLAY_NAMES[: len(pos_desired_by_arm)]]
    rr.log("trajectory_3d/actual_ee",
           rr.Points3D(pos_actual_by_arm, colors=actual_colors, radii=0.015, labels=labels_actual))
    rr.log("trajectory_3d/target_goal",
           rr.Points3D(pos_desired_by_arm, colors=target_colors, radii=0.015, labels=labels_target))

    error_lines = [
        [pos_actual_by_arm[arm], pos_desired_by_arm[arm]]
        for arm in range(len(pos_actual_by_arm))
    ]
    rr.log("trajectory_3d/error_line",
           rr.LineStrips3D(error_lines, colors=[[255, 0, 0]], radii=0.002))



